/*
 * Copyright (C) 2026 DroidVM
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sub license, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice (including the
 * next paragraph) shall be included in all copies or substantial portions
 * of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
 * IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
 * ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include "session.h"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <system_error>
#include <thread>

extern "C" {
#include <linux/videodev2.h>
}

namespace stateful {

namespace {

    using Clock = std::chrono::steady_clock;

    constexpr int kDefaultSyncTimeoutMs = 500;
    constexpr int kDefaultSyncIdleMs = 50;
    constexpr int kDefaultProvisionRetryMs = 2000;
    constexpr int kProvisionRetryStepMs = 50;
    constexpr unsigned kMaxCaptureBuffers = 32;
    constexpr unsigned kPoolShareCap = 8;
    constexpr int kWaitSliceMs = 10;
    /* VA3-fakeau SPIKE: cap padding-AU injections per single stalled sync. One
     * flushes a 1-frame pipeline delay; the small headroom tolerates a deeper
     * hold without letting a genuinely-lost frame spin injecting forever. */
    constexpr unsigned kMaxFakeInjectionsPerSync = 4;

    int resolve_sync_timeout(int configured)
    {
        if (configured >= 0) {
            return configured;
        }
        if (const char* env = getenv("LIBVA_V4L2_SYNC_TIMEOUT_MS"); env != nullptr) {
            char* end = nullptr;
            long value = strtol(env, &end, 10);
            if (end != env && *end == '\0' && value >= 0 && value <= 60000) {
                return static_cast<int>(value);
            }
        }
        return kDefaultSyncTimeoutMs;
    }

    int resolve_sync_idle(int configured)
    {
        if (configured >= 0) {
            return configured;
        }
        if (const char* env = getenv("LIBVA_V4L2_SYNC_IDLE_MS"); env != nullptr) {
            char* end = nullptr;
            long value = strtol(env, &end, 10);
            if (end != env && *end == '\0' && value >= 0 && value <= 60000) {
                return static_cast<int>(value);
            }
        }
        return kDefaultSyncIdleMs;
    }

    int remaining_ms(Clock::time_point deadline)
    {
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        return left > 0 ? static_cast<int>(left) : 0;
    }

} // namespace

/*
 * Locking (D83): mutex_ guards every field and every device call EXCEPT
 * StatefulDevice::wait, which can sleep. wait_for_progress() elects exactly
 * one harvester: it drops the lock, waits on the device, retakes the lock,
 * pumps, and wakes the condvar; every other thread sleeps on the condvar in
 * bounded slices. So no thread ever sleeps for the device while holding the
 * lock, and no two threads DQBUF concurrently.
 */

StatefulSession::StatefulSession(StatefulDevice& device, uint32_t coded_pixelformat, unsigned coded_width,
    unsigned coded_height, const Options& options, std::function<void(const char*)> log)
    : device_(device)
    , log_(std::move(log))
    , num_surfaces_(options.num_surfaces)
    , surface_count_(options.surface_count)
    , allocator_(options.allocator)
    , output_ring_size_(std::max(options.output_ring_size, 2u))
    , sync_timeout_ms_(resolve_sync_timeout(options.sync_timeout_ms))
    , sync_idle_ms_(resolve_sync_idle(options.sync_idle_ms))
    , allow_midstream_drain_(options.allow_midstream_drain)
    , fake_au_injection_(options.fake_au_injection)
    , provision_retry_ms_(options.provision_retry_ms >= 0 ? options.provision_retry_ms : kDefaultProvisionRetryMs)
    , output_pixelformat_(coded_pixelformat)
    , coded_width_(coded_width)
    , coded_height_(coded_height)
    , trace_(getenv("LIBVA_V4L2_TRACE") != nullptr)
{
    /* Construction is single-threaded: the context does not exist yet. */

    /* Subscribe before streaming so the first SOURCE_CHANGE cannot be lost. */
    device_.subscribe_events();

    const uint32_t initial_size = std::max(1024u * 1024u, coded_width * coded_height);
    device_.set_output_format(output_pixelformat_, coded_width_, coded_height_, initial_size);
    output_size_ = device_.output_buffer_size();

    /* Post-crash: a fresh client's REQBUFS/STREAMON can be refused EBUSY while
     * the device reaps the dead client's session. Retry with a bounded backoff
     * (D88); on exhaustion this throws and vaCreateContext fails cleanly with
     * VA_STATUS_ERROR_OPERATION_FAILED rather than "surface is in use". */
    unsigned granted = 0;
    retry_provision("REQBUFS(OUTPUT)", [&] { granted = device_.request_output_buffers(output_ring_size_); });
    if (granted == 0) {
        throw std::runtime_error("no OUTPUT buffers granted");
    }
    output_ring_size_ = granted;
    for (unsigned i = 0; i < granted; i++) {
        free_outputs_.push_back(i);
        usable_outputs_.insert(i);
    }

    retry_provision("STREAMON(OUTPUT)", [&] { device_.stream_output(true); });
}

void StatefulSession::retry_provision(const char* what, const std::function<void()>& op)
{
    const auto deadline = Clock::now() + std::chrono::milliseconds(provision_retry_ms_);
    bool logged = false;
    for (;;) {
        try {
            op();
            return;
        } catch (const DeviceLost&) {
            throw;
        } catch (const std::system_error& e) {
            if (e.code().value() != EBUSY) {
                throw;
            }
            if (Clock::now() >= deadline) {
                char line[176];
                snprintf(line, sizeof(line),
                    "provisioning %s still EBUSY after %d ms: the device has not released the previous client's "
                    "session -- failing context creation",
                    what, provision_retry_ms_);
                log(line);
                throw std::runtime_error(std::string(what) + ": device busy after retry budget");
            }
            if (!logged) {
                logged = true;
                char line[176];
                snprintf(line, sizeof(line),
                    "provisioning %s hit EBUSY (device still reaping a crashed client); retrying up to %d ms", what,
                    provision_retry_ms_);
                log(line);
            }
            std::this_thread::sleep_for(
                std::chrono::milliseconds(std::min(kProvisionRetryStepMs, remaining_ms(deadline) + 1)));
        }
    }
}

void StatefulSession::log(const char* message)
{
    if (log_) {
        log_(message);
    } else {
        fprintf(stderr, "libva-v4l2 stateful: %s\n", message);
    }
}

void StatefulSession::trace_dump_locked(const char* tag, uint64_t await)
{
    if (!trace_) {
        return;
    }
    /* free CAPTURE = pool minus what the client holds minus what is stashed
     * (produced, not yet claimed): if this is small the codec is starved of
     * CAPTURE buffers; if it is large the codec has room but is not producing
     * the awaited frame. */
    const long free_cap = static_cast<long>(capture_count_) - static_cast<long>(client_owned_.size())
        - static_cast<long>(stash_.size());
    char head[256];
    snprintf(head, sizeof(head),
        "TRACE %s await=%llu provisioned=%d cap_count=%u client_owned=%zu stash=%zu free_cap=%ld free_out=%zu "
        "qout=%u submits=%llu delivered=%llu harvesting=%d",
        tag, static_cast<unsigned long long>(await), provisioned_.load() ? 1 : 0, capture_count_, client_owned_.size(),
        stash_.size(), free_cap, free_outputs_.size(), queued_outputs_, static_cast<unsigned long long>(submit_count_),
        static_cast<unsigned long long>(delivered_), harvesting_ ? 1 : 0);
    log(head);

    /* The stash keys tell whether the awaited sequence is already produced. */
    std::string keys = "TRACE   stash_keys=[";
    unsigned n = 0;
    for (const auto& [seq, idx] : stash_) {
        if (n++ >= 24) {
            keys += " ...";
            break;
        }
        char b[24];
        snprintf(b, sizeof(b), "%s%llu", n == 1 ? "" : " ", static_cast<unsigned long long>(seq));
        keys += b;
    }
    keys += "]";
    log(keys.c_str());
}

void StatefulSession::pump_locked()
{
    while (auto event = device_.dequeue_event()) {
        if (*event == DeviceEvent::source_change) {
            handle_source_change_locked();
        }
    }

    while (auto index = device_.dequeue_output()) {
        if (queued_outputs_ > 0) {
            queued_outputs_ -= 1;
        }
        if (usable_outputs_.contains(*index)) {
            free_outputs_.push_back(*index);
        }
    }

    if (capture_streaming_) {
        while (auto frame = device_.dequeue_capture()) {
            handle_capture_locked(*frame);
            if (frame->last) {
                break;
            }
        }
    }
}

void StatefulSession::wait_for_progress(std::unique_lock<std::mutex>& lock, int timeout_ms, bool include_output)
{
    if (harvesting_) {
        /* Someone else is at the device; it will pump and notify. */
        cv_.wait_for(lock, std::chrono::milliseconds(std::max(timeout_ms, 1)));
        return;
    }

    harvesting_ = true;
    lock.unlock();
    try {
        device_.wait(timeout_ms, include_output);
    } catch (...) {
        lock.lock();
        harvesting_ = false;
        cv_.notify_all();
        throw;
    }
    lock.lock();
    harvesting_ = false;
    pump_locked();
    cv_.notify_all();
}

StatefulSession::CaptureMode StatefulSession::decide_capture_mode_locked()
{
    /* 7.7 point 5: gbm-dmabuf needs (a) a usable allocator, (b) the device's
     * REQBUFS(CAPTURE) capabilities to carry V4L2_BUF_CAP_SUPPORTS_DMABUF (an
     * r22 driver masks it, so the SAME .so keeps working on the r22 guest via
     * MMAP), (c) a negotiable stride (checked in provision_capture_gbm_locked,
     * which falls back on refusal). The rig can force either with
     * LIBVA_V4L2_SURFACES=mmap|gbm. */
    const char* env = getenv("LIBVA_V4L2_SURFACES");
    const bool force_mmap = env != nullptr && std::strcmp(env, "mmap") == 0;

    if (force_mmap) {
        log("stateful surfaces: mmap (reason: LIBVA_V4L2_SURFACES=mmap)");
        return CaptureMode::mmap;
    }
    if (allocator_ == nullptr || !allocator_->usable()) {
        log("stateful surfaces: mmap (reason: no usable GBM allocator -- render node is not virtio_gpu, "
            "or gbm_create_device failed)");
        return CaptureMode::mmap;
    }
    if ((device_.capture_buffer_capabilities() & V4L2_BUF_CAP_SUPPORTS_DMABUF) == 0) {
        log("stateful surfaces: mmap (reason: the device does not advertise SUPPORTS_DMABUF -- r22 driver)");
        return CaptureMode::mmap;
    }
    return CaptureMode::gbm_dmabuf;
}

void StatefulSession::release_capture_pool_locked()
{
    if (capture_streaming_) {
        device_.stream_capture(false);
        capture_streaming_ = false;
    }
    if (capture_mode_ == CaptureMode::gbm_dmabuf) {
        device_.request_capture_buffers_dmabuf(0);
        capture_bos_.clear();
    } else {
        device_.request_capture_buffers(0);
    }
}

void StatefulSession::handle_source_change_locked()
{
    if (provisioned_) {
        /* Mid-stream resolution change: VA1 re-provisions and drops what was
         * decoded against the old pool. The mode chosen for the context does
         * not change. */
        log("mid-stream SOURCE_CHANGE: re-provisioning the CAPTURE pool, dropping stashed frames");
        stash_.clear();
        client_owned_.clear();
        release_capture_pool_locked();
        provisioned_ = false;
    }

    /* D84: num_surfaces from vaCreateContext is 0 with modern clients, so
     * re-read the live surface count here -- provisioning happens at the first
     * SOURCE_CHANGE (the first vaEndPicture submit, by which time every
     * vaCreateSurfaces has run: 7.6 point 5b's deferral). In gbm-dmabuf mode
     * the surfaces ARE the buffers, so the pool-share race disappears. */
    if (surface_count_) {
        num_surfaces_ = std::max(num_surfaces_, surface_count_());
    }
    capture_format_ = device_.capture_format();

    if (!capture_mode_decided_) {
        capture_mode_ = decide_capture_mode_locked();
        capture_mode_decided_ = true;
    }

    const unsigned min_buffers = std::max(device_.min_buffers_for_capture(), 1);

    if (capture_mode_ == CaptureMode::gbm_dmabuf) {
        /* 7.7 point 2: with DMABUF the client owns the buffers, so a surface is
         * a buffer -- every surface the client holds for its display/reorder
         * queue is subtracted from the codec's own working set, unlike MMAP
         * where a held surface is a CPU copy and the buffer is recycled at once
         * (7.6 point 4). So the pool must be the codec minimum PLUS the client's
         * concurrent hold-set. VA2d gave gbm the MMAP headroom (min + share),
         * but share was min(num_surfaces, 8) and num_surfaces here is the count
         * at the first SOURCE_CHANGE (D84): a lazily-allocating client reports
         * it as 1 (Firefox makes one surface at a time), collapsing the share to
         * +1. A high-minimum codec then got only min+1: AV1's
         * MIN_BUFFERS_FOR_CAPTURE is 21, so the browser pool was 22 and the
         * moment Firefox held a second surface the codec had < 21 buffers
         * queued, stalled, and the awaited frame never arrived -- Stateful sync
         * failed on sequence ~4, softing YouTube AV1 off zero-copy (VA2h). VP9
         * (min 4) and H.264 (min 10) had enough slack under the same share to
         * absorb the browser's ~6-surface hold-set (VP9-854 sustained 825
         * imports), so only AV1 at its 21-buffer floor starved (D85/D86). Fix
         * (VA2i): give gbm the full kPoolShareCap headroom over the codec
         * minimum regardless of the early surface count, capped at 32 -- AV1
         * 854/1080p -> 29 (8 holdable, matching the 1080p ffmpeg pool that never
         * truncated), VP9 -> 12, H.264 -> 18 -- and still at least num_surfaces
         * so a client that made MORE gets a bo per surface. */
        const unsigned gbm_share = kPoolShareCap;
        unsigned count = std::min(std::max(num_surfaces_, min_buffers + gbm_share), kMaxCaptureBuffers);
        if (provision_capture_gbm_locked(count)) {
            provisioned_ = true;
            cv_.notify_all();
            return;
        }
        /* Fell back: the allocator, stride or REQBUFS(DMABUF) refused. Redo as
         * MMAP with the same .so; the reason was already logged. */
        capture_mode_ = CaptureMode::mmap;
    }

    /* 7.6 point 4: pool size = MIN_BUFFERS_FOR_CAPTURE + min(num_surfaces, 8),
     * capped at 32. Without the share the client's held surfaces come out of
     * the codec's own slots and the session deadlocks the way mpv did in B18. */
    const unsigned share = std::min(num_surfaces_, kPoolShareCap);
    unsigned count = std::min(min_buffers + share, kMaxCaptureBuffers);
    provision_capture_mmap_locked(count);
    provisioned_ = true;
    cv_.notify_all();
}

void StatefulSession::provision_capture_mmap_locked(unsigned count)
{
    const unsigned min_buffers = std::max(device_.min_buffers_for_capture(), 1);
    const unsigned share = count >= min_buffers ? count - min_buffers : 0;

    /* Post-crash EBUSY retry (D88): the same reaping window can refuse
     * REQBUFS(CAPTURE)/STREAMON(CAPTURE) for the first client after a crash.
     * Retrying here (not per-picture) turns it into one clean failure of the
     * first sync instead of a cascade of "surface is in use" per picture. */
    retry_provision("REQBUFS(CAPTURE)", [&] { capture_count_ = device_.request_capture_buffers(count); });
    if (capture_count_ == 0) {
        throw std::runtime_error("no CAPTURE buffers granted");
    }
    generation_ += 1; /* D83: every binding handed out before this is stale */

    /* The mode line was logged by decide_capture_mode_locked (or the gbm
     * fallback). This is the pool line the rig's va.sh reads (D84). */
    char line[128];
    snprintf(line, sizeof(line), "CAPTURE pool: min %u + share %u = %u (surfaces %u, granted %u)", min_buffers, share,
        count, num_surfaces_, capture_count_);
    log(line);
    for (unsigned i = 0; i < capture_count_; i++) {
        requeue_capture_locked(i);
    }
    retry_provision("STREAMON(CAPTURE)", [&] { device_.stream_capture(true); });
    capture_streaming_ = true;
}

bool StatefulSession::provision_capture_gbm_locked(unsigned count)
{
    /* VA2d: size the R8 container's WIDTH from the device's luma stride
     * (bytesperline), so GBM's own R8 pitch cannot come out NARROWER than the
     * device needs -- align_up(bytesperline, GBM row alignment) >= bytesperline.
     * The height is the coded height, so the container's chroma plane and its
     * size are the device's. */
    const uint32_t alloc_width
        = capture_format_.bytesperline != 0 ? capture_format_.bytesperline : capture_format_.width;
    const uint32_t height = capture_format_.height;

    /* Allocate the first bo to learn its GBM stride and reconcile it with the
     * device's G_FMT bytesperline (7.7 point 1). */
    uint32_t stride = 0;
    auto first = allocator_->allocate(alloc_width, height, stride);
    if (!first) {
        log("stateful surfaces: mmap (reason: GBM allocation failed)");
        return false;
    }

    /* Record the exact geometry the fallback ladder turns on, so a phone run at
     * a non-aligned resolution shows the stride/bytesperline divergence without
     * a debug build (VA2d). */
    {
        char probe[240];
        snprintf(probe, sizeof(probe),
            "stateful surfaces: gbm probe -- G_FMT %ux%u bytesperline %u sizeimage %u planes %u; R8 request width %u "
            "-> bo stride %u, container %zu",
            capture_format_.width, capture_format_.height, capture_format_.bytesperline, capture_format_.sizeimage,
            capture_format_.num_planes, alloc_width, stride, first->size());
        log(probe);
    }

    if (stride != capture_format_.bytesperline) {
        /* The bo's GBM stride differs from the device's luma stride. First ask
         * the device to adopt the bo stride via S_FMT(CAPTURE): if it agrees the
         * codec writes bo-stride-packed NV12 and the whole bo width is used. */
        uint32_t granted = device_.set_capture_stride(stride);
        if (granted == stride) {
            capture_format_ = device_.capture_format(); /* re-read the negotiated geometry */
        } else if (stride > capture_format_.bytesperline) {
            /* VA2d, THE fix: at 854x480 the qti decoder emits a tightly-packed
             * 854-byte luma stride and will NOT grow it (S_FMT granted 854),
             * while GBM rounds an R8 width up to its 64-byte row alignment and
             * cannot produce a 854 stride (it returns 896). That is still fine
             * for zero-copy: the codec writes its own bytesperline-packed NV12
             * (854) into the roomier bo, and both the DMABUF QBUF plane offsets
             * and the exported VADRMPRIMESurfaceDescriptor already describe the
             * planes at the device bytesperline -- the bo's extra bytes per row
             * are unused slack. Keep the device geometry (bytesperline stays
             * 854) and proceed; the sizeimage guard below confirms the bo holds
             * a bytesperline-strided frame (it does: 896*720 > 854*480*3/2).
             * Before this, every such resolution fell back to MMAP and lost the
             * export, so Firefox on YouTube dropped to software dav1d at
             * decode-order frame 1 (7.7 point 1). */
            char line[208];
            snprintf(line, sizeof(line),
                "stateful surfaces: gbm bo stride %u > device bytesperline %u (S_FMT kept %u) -- using the device "
                "stride, %u B/row slack",
                stride, capture_format_.bytesperline, granted, stride - capture_format_.bytesperline);
            log(line);
        } else {
            /* stride < bytesperline: the bo is too NARROW to hold a
             * bytesperline-strided frame row-for-row (should not happen now the
             * bo is requested at bytesperline, but guard it). Fall back to MMAP. */
            char line[192];
            snprintf(line, sizeof(line),
                "stateful surfaces: mmap (reason: GBM stride %u < device bytesperline %u and S_FMT granted %u)", stride,
                capture_format_.bytesperline, granted);
            log(line);
            return false;
        }
    }

    /* 7.7 (2): the stride was matched, negotiated, or accepted as slack above;
     * the remaining check is that each bo actually holds a bytesperline-strided
     * frame -- the decoder writes sizeimage bytes into the buffer, so a bo
     * smaller than that (a stride shortfall, or a height that grew on re-read)
     * would be an out-of-bounds write. At 1080p the R8 container (3112960) is a
     * touch larger than sizeimage (3110400); at 854x480 the 896-strided
     * container (645120) clears the 854-strided sizeimage (614880) with room to
     * spare. A shortfall frees the bos and falls back to MMAP rather than hand
     * the codec a short buffer. */
    if (capture_format_.sizeimage != 0 && first->size() < capture_format_.sizeimage) {
        char line[176];
        snprintf(line, sizeof(line),
            "stateful surfaces: mmap (reason: GBM container %zu < device sizeimage %u -- too small for the decoder)",
            first->size(), capture_format_.sizeimage);
        log(line);
        return false;
    }

    capture_bos_.clear();
    capture_bos_.reserve(count);
    capture_bos_.push_back(std::move(first));
    for (unsigned i = 1; i < count; i++) {
        uint32_t s = 0;
        auto bo = allocator_->allocate(alloc_width, height, s);
        if (!bo || s != stride) {
            log("stateful surfaces: mmap (reason: a later GBM allocation failed or changed stride)");
            capture_bos_.clear();
            return false;
        }
        capture_bos_.push_back(std::move(bo));
    }

    try {
        retry_provision(
            "REQBUFS(CAPTURE,DMABUF)", [&] { capture_count_ = device_.request_capture_buffers_dmabuf(count); });
    } catch (const DeviceLost&) {
        throw;
    } catch (const std::exception& e) {
        char line[176];
        snprintf(line, sizeof(line), "stateful surfaces: mmap (reason: REQBUFS(CAPTURE,DMABUF) refused: %s)", e.what());
        log(line);
        capture_bos_.clear();
        return false;
    }
    if (capture_count_ == 0 || capture_count_ > capture_bos_.size()) {
        log("stateful surfaces: mmap (reason: REQBUFS(CAPTURE,DMABUF) granted an unusable count)");
        capture_bos_.clear();
        capture_count_ = 0;
        return false;
    }
    generation_ += 1; /* D83 */

    const unsigned spares = capture_count_ > num_surfaces_ ? capture_count_ - num_surfaces_ : 0;
    char line[208];
    snprintf(line, sizeof(line),
        "stateful surfaces: gbm-dmabuf (%s, stride %u, planes %u); CAPTURE pool: %u (surfaces %u, spares %u, min %u)",
        allocator_->supports_nv12_linear() ? "NV12" : "R8 container", capture_format_.bytesperline,
        capture_format_.num_planes, capture_count_, num_surfaces_, spares,
        std::max(device_.min_buffers_for_capture(), 1));
    log(line);

    /* RUNTIME fallback (D90): the r24 driver grants REQBUFS(CAPTURE,DMABUF)
     * and advertises SUPPORTS_DMABUF, yet on a protected DroidVM guest the
     * first QBUF of a virtio-gpu GBM bo fails -EIO -- every virtio device is
     * bound to a restricted DMA pool, so the vram exporter's map_dma_buf
     * (dma_map_resource) is refused (B21-accept §3). REQBUFS refusal was
     * already handled above; a QBUF/STREAMON that fails here must NOT abort the
     * decode (B21: Epiphany 300 -> 0). Tear the DMABUF provisioning down, free
     * the bos, and return false so the caller re-provisions MMAP -- a broken
     * zero-copy path costs zero frames. */
    try {
        for (unsigned i = 0; i < capture_count_; i++) {
            requeue_capture_locked(i);
        }
        retry_provision("STREAMON(CAPTURE)", [&] { device_.stream_capture(true); });
    } catch (const DeviceLost&) {
        throw;
    } catch (const std::exception& e) {
        int errnum = 0;
        if (const auto* se = dynamic_cast<const std::system_error*>(&e)) {
            errnum = se->code().value();
        }
        char reason[224];
        snprintf(reason, sizeof(reason),
            "stateful surfaces: mmap (reason: DMABUF QBUF failed errno %d: %s) -- the zero-copy path is refused at "
            "runtime, re-provisioning MMAP",
            errnum, e.what());
        log(reason);
        if (capture_streaming_) {
            device_.stream_capture(false);
            capture_streaming_ = false;
        }
        try {
            device_.request_capture_buffers_dmabuf(0);
        } catch (const std::exception&) {
            /* Best-effort release; the MMAP re-provision issues its own
             * REQBUFS which supersedes whatever survived here. */
        }
        capture_bos_.clear();
        capture_count_ = 0;
        return false;
    }
    capture_streaming_ = true;
    return true;
}

void StatefulSession::requeue_capture_locked(unsigned index)
{
    if (capture_mode_ != CaptureMode::gbm_dmabuf) {
        device_.queue_capture(index);
        return;
    }

    /* 7.7 point 2/5b: QBUF the bo as DMABUF. Plane 0 is the bo fd at offset 0;
     * a two-plane NV12 puts the chroma plane at the SAME fd, data_offset
     * stride*height (the r23 driver accepts one fd per plane). */
    SurfaceBuffer* bo = capture_buffer(index);
    if (bo == nullptr) {
        throw std::runtime_error("gbm CAPTURE index has no bo");
    }
    const uint32_t stride = capture_format_.bytesperline != 0 ? capture_format_.bytesperline : capture_format_.width;
    const uint32_t luma = stride * capture_format_.height;

    DmabufPlane planes[2] = {};
    unsigned n;
    if (capture_format_.num_planes >= 2) {
        planes[0] = { bo->fd(), luma, 0 };
        planes[1] = { bo->fd(), luma / 2, luma };
        n = 2;
    } else {
        planes[0] = { bo->fd(), static_cast<uint32_t>(bo->size()), 0 };
        n = 1;
    }
    device_.queue_capture_dmabuf(index, std::span<const DmabufPlane>(planes, n));
}

SurfaceBuffer* StatefulSession::capture_buffer(unsigned index)
{
    if (capture_mode_ != CaptureMode::gbm_dmabuf || index >= capture_bos_.size()) {
        return nullptr;
    }
    return capture_bos_[index].get();
}

void StatefulSession::handle_capture_locked(const DequeuedCapture& frame)
{
    if (frame.index == DequeuedCapture::no_buffer) {
        return; /* synthetic end-of-drain marker */
    }
    if (frame.bytesused == 0 || frame.error) {
        /* An empty LAST (or erroneous) buffer carries no frame; recycle it. */
        if (trace_) {
            char line[128];
            snprintf(line, sizeof(line), "TRACE CAP-DROP seq=%llu idx=%u bytesused=%u error=%d last=%d",
                static_cast<unsigned long long>(frame.sequence), frame.index, frame.bytesused, frame.error ? 1 : 0,
                frame.last ? 1 : 0);
            log(line);
        }
        requeue_capture_locked(frame.index);
        return;
    }
    if (fake_au_injection_ && frame.sequence >= kFakeAuSequenceBase) {
        /* VA3-fakeau SPIKE: the decoded output of an injected padding AU. The
         * client never submitted this sequence, so drop it (recycle the buffer)
         * and never stash it. It advanced the codec's pipeline; its pixels are
         * discarded. */
        fake_au_dropped_ += 1;
        if (trace_) {
            char line[128];
            snprintf(line, sizeof(line), "TRACE FAKE-AU-DROP seq=%llu idx=%u dropped=%u",
                static_cast<unsigned long long>(frame.sequence), frame.index, fake_au_dropped_);
            log(line);
        }
        requeue_capture_locked(frame.index);
        return;
    }
    if (unwanted_sequences_.erase(frame.sequence) > 0) {
        requeue_capture_locked(frame.index);
        return;
    }
    stash_[frame.sequence] = frame.index;
    if (trace_) {
        char line[128];
        snprintf(line, sizeof(line), "TRACE CAP seq=%llu idx=%u stash=%zu client_owned=%zu",
            static_cast<unsigned long long>(frame.sequence), frame.index, stash_.size(), client_owned_.size());
        log(line);
    }
    cv_.notify_all();
}

bool StatefulSession::claim_locked(uint64_t sequence, Frame* frame)
{
    auto it = stash_.find(sequence);
    if (it == stash_.end()) {
        return false;
    }
    frame->index = it->second;
    frame->generation = generation_;
    client_owned_.insert(it->second);
    stash_.erase(it);
    delivered_ += 1;
    return true;
}

int StatefulSession::acquire_output_buffer_locked(std::unique_lock<std::mutex>& lock, size_t needed)
{
    const auto deadline = Clock::now() + std::chrono::milliseconds(sync_timeout_ms_);

    if (needed > output_size_) {
        grow_output_buffers_locked(lock, needed);
    }

    while (true) {
        pump_locked();
        if (!free_outputs_.empty()) {
            int index = static_cast<int>(free_outputs_.back());
            free_outputs_.pop_back();
            return index;
        }
        if (Clock::now() >= deadline) {
            return -1;
        }
        wait_for_progress(lock, std::min(kWaitSliceMs, remaining_ms(deadline) + 1), true);
    }
}

void StatefulSession::grow_output_buffers_locked(std::unique_lock<std::mutex>& lock, size_t needed)
{
    uint32_t new_size = std::max<size_t>(needed, static_cast<size_t>(output_size_) * 2);
    new_size = (new_size + 0xffffu) & ~0xffffu; /* round up to 64 KiB */

    /* Preferred: CREATE_BUFS keeps the queue streaming (7.6 point 4); the
     * virtio-media device accepts it while streaming (video_decoder.rs:1758).
     * The retired small buffers stay allocated but unused. */
    try {
        auto [first, count] = device_.create_output_buffers(output_ring_size_, new_size);
        if (count > 0) {
            usable_outputs_.clear();
            free_outputs_.clear();
            for (unsigned i = 0; i < count; i++) {
                usable_outputs_.insert(first + i);
                free_outputs_.push_back(first + i);
            }
            output_size_ = new_size;
            return;
        }
    } catch (const DeviceLost&) {
        throw;
    } catch (const std::exception&) {
        /* Fall through to idle reallocation. */
    }

    /* Fallback for devices without CREATE_BUFS: wait until every queued
     * OUTPUT buffer came back (the decoder is idle), then STREAMOFF(OUTPUT) +
     * REQBUFS(0) + S_FMT + REQBUFS + STREAMON. With an empty queue the
     * implicit seek of STREAMOFF(OUTPUT) has nothing to drop. */
    const auto deadline = Clock::now() + std::chrono::milliseconds(sync_timeout_ms_);
    while (queued_outputs_ > 0) {
        if (Clock::now() >= deadline) {
            throw std::runtime_error("cannot grow OUTPUT buffers: queue never went idle");
        }
        pump_locked();
        if (queued_outputs_ > 0) {
            wait_for_progress(lock, std::min(kWaitSliceMs, remaining_ms(deadline) + 1), true);
        }
    }
    device_.stream_output(false);
    device_.request_output_buffers(0);
    device_.set_output_format(output_pixelformat_, coded_width_, coded_height_, new_size);
    unsigned granted = device_.request_output_buffers(output_ring_size_);
    if (granted == 0) {
        throw std::runtime_error("no OUTPUT buffers granted after reallocation");
    }
    usable_outputs_.clear();
    free_outputs_.clear();
    for (unsigned i = 0; i < granted; i++) {
        usable_outputs_.insert(i);
        free_outputs_.push_back(i);
    }
    output_size_ = device_.output_buffer_size();
    device_.stream_output(true);
}

bool StatefulSession::inject_fake_au_locked(std::unique_lock<std::mutex>& lock, unsigned k)
{
    /* VA3-fakeau SPIKE. The caller has established the wedge: input-starved
     * (queued_outputs_ == 0), provisioned, the awaited frame not produced. Feed
     * the codec one more access unit under a reserved sentinel sequence so its
     * pipeline advances by one and the held real frame is emitted (tagged with
     * its real sequence -> stashed and delivered), while this injected AU's own
     * output arrives tagged with the sentinel and is dropped by
     * handle_capture_locked.
     *
     * VA3-fakeau (byte-copy, factory unset): the AU is a byte copy of the last
     * real AU. It carries the SAME order_hint, so the QTI AV1 codec recognised
     * it as a same-frame replay -- consumed but never emitted, no flush (the
     * measured outcome (i)).
     *
     * VA3-fakeau2 (bumped order_hint, factory set): the AU is a synthesised
     * NON-reference frame with order_hint = last real +k, which the codec should
     * accept as a genuinely NEW frame and so advance the reorder to flush the
     * held real frame (VA3-reorder-probe: a larger order_hint is the emit
     * trigger; reorder depth bounded at 4). refresh_frame_flags=0 leaves the
     * real reference state untouched, so 关卡二 (bit-exactness / perceptual
     * quality of the following real frames) is the verdict, settled on the
     * phone. */
    std::vector<uint8_t> synthesized;
    std::span<const uint8_t> au_bytes;
    if (fake_au_factory_) {
        std::optional<std::vector<uint8_t>> built = fake_au_factory_(k);
        if (!built || built->empty()) {
            return false;
        }
        synthesized = std::move(*built);
        au_bytes = synthesized;
    } else {
        if (last_au_bytes_.empty()) {
            return false;
        }
        au_bytes = last_au_bytes_;
    }
    int index = acquire_output_buffer_locked(lock, au_bytes.size());
    if (index < 0) {
        return false;
    }
    auto plane = device_.output_plane(static_cast<unsigned>(index));
    if (plane.size() < au_bytes.size()) {
        return false;
    }
    std::copy(au_bytes.begin(), au_bytes.end(), plane.begin());
    const uint64_t seq = fake_au_next_seq_++;
    /* Deliberately NOT counted in submit_count_: a fake AU is not client input
     * flow, so a coupled sync must not read it as "the client is still feeding". */
    device_.queue_output(static_cast<unsigned>(index), seq, au_bytes.size());
    queued_outputs_ += 1;
    fake_au_injected_ += 1;
    if (trace_) {
        char line[176];
        snprintf(line, sizeof(line), "TRACE FAKE-AU-INJECT seq=%llu k=%u bytes=%zu synth=%d qout=%u injected=%u",
            static_cast<unsigned long long>(seq), k, au_bytes.size(), fake_au_factory_ ? 1 : 0, queued_outputs_,
            fake_au_injected_);
        log(line);
    }
    cv_.notify_all();
    return true;
}

void StatefulSession::submit(uint64_t sequence, std::span<const uint8_t> access_unit)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (dead_) {
        throw std::runtime_error("session is dead");
    }
    if (access_unit.empty()) {
        throw std::invalid_argument("empty access unit");
    }

    try {
        int index = acquire_output_buffer_locked(lock, access_unit.size());
        if (index < 0) {
            throw std::runtime_error("timed out waiting for a free OUTPUT buffer");
        }
        auto plane = device_.output_plane(static_cast<unsigned>(index));
        if (plane.size() < access_unit.size()) {
            throw std::runtime_error("OUTPUT buffer too small after growth");
        }
        std::copy(access_unit.begin(), access_unit.end(), plane.begin());
        device_.queue_output(static_cast<unsigned>(index), sequence, access_unit.size());
        queued_outputs_ += 1;
        submit_count_ += 1; /* D85: waiting syncs watch this for input flow */
        if (fake_au_injection_) {
            /* VA3-fakeau SPIKE: remember this real AU's bytes; a later stalled
             * sync re-submits a copy of it to flush the codec's held frame. */
            last_au_bytes_.assign(access_unit.begin(), access_unit.end());
        }
        if (trace_) {
            char line[128];
            snprintf(line, sizeof(line), "TRACE SUBMIT seq=%llu qout=%u free_out=%zu submits=%llu",
                static_cast<unsigned long long>(sequence), queued_outputs_, free_outputs_.size(),
                static_cast<unsigned long long>(submit_count_));
            log(line);
        }
        cv_.notify_all();
    } catch (const DeviceLost&) {
        dead_ = true;
        cv_.notify_all();
        throw;
    }
}

bool StatefulSession::drain_capture_locked(std::unique_lock<std::mutex>& lock, int timeout_ms)
{
    /* The caller owns harvesting_: no other thread is at the device. */
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);

    while (true) {
        while (auto event = device_.dequeue_event()) {
            if (*event == DeviceEvent::source_change) {
                handle_source_change_locked();
            }
        }
        while (auto frame = device_.dequeue_capture()) {
            handle_capture_locked(*frame);
            if (frame->last) {
                return true;
            }
        }
        if (Clock::now() >= deadline) {
            return false;
        }
        lock.unlock();
        try {
            device_.wait(std::min(kWaitSliceMs, remaining_ms(deadline) + 1), false);
        } catch (...) {
            lock.lock();
            throw;
        }
        lock.lock();
    }
}

StatefulSession::SyncStatus StatefulSession::recover_locked(
    std::unique_lock<std::mutex>& lock, uint64_t sequence, Frame* frame, bool idle)
{
    /* One recovery at a time: wait out any harvester (its pump may deliver
     * our frame), then own the role for the whole stop/drain/start. */
    while (harvesting_) {
        cv_.wait_for(lock, std::chrono::milliseconds(kWaitSliceMs));
        if (claim_locked(sequence, frame)) {
            return SyncStatus::ok;
        }
    }
    harvesting_ = true;

    SyncStatus status = SyncStatus::decode_error;
    try {
        /* 7.6 point 5b: the decoder holds frames the client is waiting
         * for. Drain and restart; count and log the two triggers apart
         * (D85): 'idle drain' -- no new input for sync_idle_ms_, the
         * stream tail or a genuine stall, drained fast; 'sync timeout' --
         * input still flowing, the hard cap expired (the B18 bar wants 0
         * of these on the 1080p reference clip). */
        char line[192];
        if (idle) {
            idle_drains_ += 1;
            snprintf(line, sizeof(line),
                "idle drain after %d ms without new input on sequence %llu: DEC_CMD_STOP drain + restart "
                "(occurrence %u, sync timeouts %u)",
                sync_idle_ms_, static_cast<unsigned long long>(sequence), idle_drains_, timeout_recoveries_);
        } else {
            timeout_recoveries_ += 1;
            snprintf(line, sizeof(line),
                "sync timeout after %d ms on sequence %llu: DEC_CMD_STOP drain + restart "
                "(occurrence %u, idle drains %u)",
                sync_timeout_ms_, static_cast<unsigned long long>(sequence), timeout_recoveries_, idle_drains_);
        }
        log(line);

        device_.decoder_stop();
        const bool saw_last = drain_capture_locked(lock, sync_timeout_ms_);
        device_.decoder_start();
        if (trace_) {
            char line[160];
            snprintf(line, sizeof(line), "TRACE DRAIN-DONE await=%llu saw_last=%d stash=%zu submits=%llu",
                static_cast<unsigned long long>(sequence), saw_last ? 1 : 0, stash_.size(),
                static_cast<unsigned long long>(submit_count_));
            log(line);
        }

        /* D86: make the recovery re-enterable. DEC_CMD_STOP's seek can pause
         * the OUTPUT queue, so re-assert STREAMON(OUTPUT) before feeding
         * again (idempotent on a queue already streaming); then pump so the
         * OUTPUT buffers the drain finished are returned to free_outputs_
         * and queued_outputs_ is right for the next submit. Without this the
         * second drain in a session starves and the old code let the whole
         * session die. */
        if (!capture_streaming_ && provisioned_) {
            device_.stream_capture(true);
            capture_streaming_ = true;
        }
        device_.stream_output(true);
        pump_locked();

        if (claim_locked(sequence, frame)) {
            status = SyncStatus::ok;
        } else if (saw_last) {
            /* The drain reached LAST without our frame: the codec dropped
             * this sequence. Report a decode error for THIS surface only and
             * leave the session streaming (never dead_ except on
             * DeviceLost). */
            status = SyncStatus::decode_error;
        }
    } catch (...) {
        harvesting_ = false;
        cv_.notify_all();
        throw;
    }
    harvesting_ = false;
    cv_.notify_all();
    return status;
}

StatefulSession::SyncStatus StatefulSession::sync(uint64_t sequence, Frame* frame)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (dead_) {
        return SyncStatus::dead;
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(sync_timeout_ms_);
    /* D85: wait while input is flowing (another thread submitted since the
     * wait began), up to the hard cap; but once no new submission arrives
     * for sync_idle_ms_, drain now -- the frame is either held back by the
     * codec (the B-frame stream tail) or lost, and only a drain settles
     * which. */
    uint64_t submits_seen = submit_count_;
    auto idle_since = Clock::now();
    bool logged_wait = false;
    unsigned fakes_this_sync = 0; /* VA3-fakeau SPIKE injection budget */

    try {
        while (true) {
            pump_locked();

            if (claim_locked(sequence, frame)) {
                return SyncStatus::ok;
            }
            if (trace_ && !logged_wait) {
                logged_wait = true;
                trace_dump_locked("SYNC-WAIT", sequence);
            }
            if (dead_) {
                return SyncStatus::dead;
            }

            const auto now = Clock::now();
            if (submit_count_ != submits_seen) {
                submits_seen = submit_count_;
                idle_since = now;
            }
            const auto idle_deadline = idle_since + std::chrono::milliseconds(sync_idle_ms_);
            const bool hard_expired = now >= deadline;
            if (!provisioned_) {
                /* D86: before the first SOURCE_CHANGE a drain has nowhere to
                 * deliver -- DEC_CMD_STOP at sequence 1 was exactly what
                 * killed the B18 session. Wait for the announce up to the
                 * hard cap, then give up on this surface without a drain;
                 * the session stays alive for the frames that follow. */
                if (hard_expired) {
                    trace_dump_locked("WEDGE-preannounce", sequence);
                    return SyncStatus::decode_error;
                }
                wait_for_progress(lock, std::min(kWaitSliceMs, remaining_ms(deadline) + 1), false);
                continue;
            }

            const bool idle_expired = now >= idle_deadline;
            if (hard_expired || idle_expired) {
                if (!allow_midstream_drain_) {
                    /* VA3-fakeau SPIKE: try to flush the codec's held
                     * output-pipeline frame before giving up. Only when
                     * input-starved (queued_outputs_ == 0: every real AU
                     * consumed, the awaited frame held for one more input the
                     * coupled client will not send), bounded per sync. Injecting
                     * feeds a padding AU; the loop then waits for the flushed
                     * real frame and claims it. The padding AU's own output is
                     * dropped by its sentinel tag. Disabled by default; only the
                     * AV1 context with LIBVA_V4L2_FAKE_AU reaches here. */
                    if (fake_au_injection_ && provisioned_ && queued_outputs_ == 0
                        && fakes_this_sync < kMaxFakeInjectionsPerSync
                        && inject_fake_au_locked(lock, fakes_this_sync + 1)) {
                        fakes_this_sync += 1;
                        idle_since = Clock::now(); /* let the codec emit the flushed frame */
                        submits_seen = submit_count_;
                        continue;
                    }
                    /* VP9 and AV1 (VA3-sync-reorder). VP9's display order is
                     * in-band (show_existing_frame) so the decoder emits
                     * displayable frames with no held tail -- solid 300/300 incl.
                     * 854 (VA2j). AV1 deep-B DOES hold a decode-order pipeline
                     * tail, but a mid-stream DEC_CMD_STOP drain cannot extract it:
                     * on this device the drain DROPS the held frame and emits only
                     * an empty LAST (VA3-sync-reorder measurement), so draining
                     * would lose the frame AND reset the reference chain. Only
                     * H.264 (allow_midstream_drain default true) takes the
                     * recover_locked branch below, and only at its EOS tail where
                     * the drain flushes cleanly. So here keep waiting up to the
                     * hard cap and, on the cap, fail THIS surface only -- no
                     * DEC_CMD_STOP, DPB and session intact for the frames that
                     * follow (deep-B AV1 then falls back to software; VP9 is the
                     * zero-copy browser path). finish() still drains the genuine
                     * tail at EOS. */
                    if (hard_expired) {
                        trace_dump_locked("WEDGE-nodrain", sequence);
                        return SyncStatus::decode_error;
                    }
                    wait_for_progress(lock, std::min(kWaitSliceMs, remaining_ms(deadline) + 1), false);
                    continue;
                }
                return recover_locked(lock, sequence, frame, idle_expired && !hard_expired);
            }

            int slice = std::min({ kWaitSliceMs, remaining_ms(deadline) + 1, remaining_ms(idle_deadline) + 1 });
            wait_for_progress(lock, slice, false);
        }
    } catch (const DeviceLost&) {
        dead_ = true;
        cv_.notify_all();
        return SyncStatus::dead;
    } catch (const std::exception&) {
        return SyncStatus::decode_error;
    }
}

void StatefulSession::release_frame(const Frame& frame)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (frame.generation != generation_) {
        /* D83: the pool was re-provisioned since this binding was handed
         * out; its index means nothing now. Log once, never QBUF. */
        if (!stale_release_logged_) {
            stale_release_logged_ = true;
            char line[128];
            snprintf(line, sizeof(line), "ignoring release of stale CAPTURE index %u (generation %llu, pool at %llu)",
                frame.index, static_cast<unsigned long long>(frame.generation),
                static_cast<unsigned long long>(generation_));
            log(line);
        }
        return;
    }

    client_owned_.erase(frame.index);
    if (dead_ || !capture_streaming_) {
        return;
    }
    try {
        requeue_capture_locked(frame.index);
        cv_.notify_all(); /* a free CAPTURE buffer lets the decoder progress */
    } catch (const DeviceLost&) {
        dead_ = true;
        cv_.notify_all();
    } catch (const std::exception& e) {
        /* A refused QBUF loses one buffer; losing the session (or the whole
         * process, as the D83 abort did) would be worse. */
        char line[160];
        snprintf(line, sizeof(line), "release of CAPTURE index %u failed: %s", frame.index, e.what());
        log(line);
    }
}

void StatefulSession::drop_sequence(uint64_t sequence)
{
    std::unique_lock<std::mutex> lock(mutex_);

    if (auto it = stash_.find(sequence); it != stash_.end()) {
        unsigned index = it->second;
        stash_.erase(it);
        if (dead_ || !capture_streaming_) {
            return;
        }
        try {
            requeue_capture_locked(index);
            cv_.notify_all();
        } catch (const DeviceLost&) {
            dead_ = true;
            cv_.notify_all();
        }
        return;
    }
    unwanted_sequences_.insert(sequence);
}

void StatefulSession::finish()
{
    std::unique_lock<std::mutex> lock(mutex_);

    /* Exclusive teardown: wait out any thread still at the device. */
    while (harvesting_) {
        cv_.wait_for(lock, std::chrono::milliseconds(kWaitSliceMs));
    }
    harvesting_ = true;

    if (!dead_) {
        try {
            /* 7.6 point 6: DEC_CMD_STOP, drain to LAST, then tear down. */
            if (capture_streaming_ && queued_outputs_ > 0) {
                device_.decoder_stop();
                drain_capture_locked(lock, sync_timeout_ms_);
            }
        } catch (const std::exception&) {
            dead_ = true;
        }
    }

    try {
        device_.stream_output(false);
        device_.stream_capture(false);
        device_.request_output_buffers(0);
        if (capture_mode_ == CaptureMode::gbm_dmabuf) {
            device_.request_capture_buffers_dmabuf(0);
        } else {
            device_.request_capture_buffers(0);
        }
    } catch (const std::exception&) {
        /* The device is gone; nothing left to release. */
    }

    capture_streaming_ = false;
    provisioned_ = false;
    stash_.clear();
    client_owned_.clear();
    /* The GBM bos (and their dma-buf fds) are freed here at context teardown
     * (7.7 point 2: owned by the surfaces until vaDestroySurfaces). */
    capture_bos_.clear();
    free_outputs_.clear();
    usable_outputs_.clear();
    queued_outputs_ = 0;

    harvesting_ = false;
    cv_.notify_all();
}

} // namespace stateful
