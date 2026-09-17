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

#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <span>
#include <vector>

#include "allocator.h"
#include "device.h"

namespace stateful {

/*
 * The stateful M2M decode session (VPU_DESIGN.md 7.6 points 4-6): access
 * units go into a small OUTPUT ring tagged with a 64-bit sequence; the device
 * copies the tag to the decoded CAPTURE buffer; sync() collects CAPTURE
 * buffers into a sequence-keyed stash until the requested one arrives, with
 * the bounded-wait DEC_CMD_STOP/drain/START recovery of point 5b.
 *
 * Thread safety (D83): ffmpeg's vaapi hwaccel is HWACCEL_CAP_ASYNC_SAFE, so
 * frame threads call vaBeginPicture/vaEndPicture/vaSyncSurface concurrently
 * on one context. Every public entry point serialises on an internal mutex;
 * a thread that must sleep for the device does so through a single elected
 * harvester with the mutex released (7.6 point 5: a sync waiting for frame S
 * while holding the lock would starve the thread submitting more input into
 * a deadlock).
 */
class StatefulSession {
public:
    struct Options {
        unsigned num_surfaces = 0;
        /* D84: the live surface count, re-read at every CAPTURE
         * provisioning; the larger of it and num_surfaces sets the pool
         * share. vaCreateContext's render-target list is empty in modern
         * clients (ffmpeg passes none), so the count must come from the
         * surfaces vaCreateSurfaces actually made. Called with the session
         * mutex held: it must not take the driver-wide mutex (read an
         * atomic counter instead). */
        std::function<unsigned()> surface_count;
        /* VA3-fakeau SPIKE (env LIBVA_V4L2_FAKE_AU, default off): only the AV1
         * context sets this, and only when the env flag is present. When true,
         * a mid-stream sync that stalls input-starved (the deep-B AV1 reorder
         * wedge: the QTI decoder holds decoded frame K until fed AU K+1, and a
         * single-threaded display-order VA-API client cannot feed ahead --
         * VA3-sync-reorder) INJECTS a copy of the most-recently-submitted real
         * access unit, tagged with a reserved sentinel sequence, to advance the
         * codec's output pipeline by one and flush the held frame; the injected
         * AU's own CAPTURE output is recognised by its sentinel tag and dropped.
         * This is a corruption-risk spike: the duplicate frame decodes into the
         * DPB, so 关卡二 (does the injected AU corrupt the subsequent real
         * frames' references) is the make-or-break question, settled by a
         * bit-exactness check against software dav1d. Left false for H.264/VP9,
         * whose sessions never construct with it and so are bit-identical. */
        bool fake_au_injection = false;
        unsigned output_ring_size = 8;
        /* < 0: LIBVA_V4L2_SYNC_TIMEOUT_MS or the 500 ms default (point 5b). */
        int sync_timeout_ms = -1;
        /* D85: how long a sync waits with NO new submission arriving before
         * draining (a B-frame stream's tail is held by the codec until a
         * drain -- VA-API has no EOS call, so the tail can only come out
         * this way). While input keeps flowing the sync waits up to
         * sync_timeout_ms instead. < 0: LIBVA_V4L2_SYNC_IDLE_MS or 50. */
        int sync_idle_ms = -1;
        /* D85/D86: the mid-stream idle/timeout DEC_CMD_STOP drain
         * (recover_locked) exists for H.264, whose reorder tail the codec holds
         * back until a drain shakes it loose -- VA-API has no flush/EOS call, so
         * a mid-stream sync must drain to extract it (B18/B19). VP9 clears it:
         * its display order is in-band (show_existing_frame), so there is no held
         * tail mid-stream. AV1 ALSO clears it, but for a subtler reason than
         * VA2e's original note claimed (VA3-sync-reorder corrects it): the QTI
         * decoder DOES hold a small decode-order output-pipeline tail on deep-B
         * content, but a mid-stream DEC_CMD_STOP cannot extract it -- on this
         * device the drain DROPS the held frame and emits only an empty LAST
         * (measured: CAP-DROP seq=0 bytesused=0 last=1, stash unchanged), so
         * draining would lose the frame and reset the reference chain. A
         * single-threaded display-order VA-API client blocks on that held frame
         * and cannot feed ahead, so deep-B AV1 wedges with no in-stack remedy
         * (crosvm KEY_LOW_LATENCY VA2k and the .low_latency variant VA2l are both
         * refuted); it falls back to software while VP9 carries browser zero-copy.
         * When false, a mid-stream sync waits out the hard cap without a reset and
         * fails only THIS surface as a last resort. finish() (EOS) still drains
         * the true tail in every codec. */
        bool allow_midstream_drain = true;
        /* D88/post-crash: a fresh client's REQBUFS/STREAMON can be refused
         * EBUSY while the device is still reaping a crashed client's session
         * (B18/B19: the next process hits it within milliseconds). Provisioning
         * retries EBUSY with a bounded backoff for up to this long before
         * failing the context/first begin cleanly. < 0: 2000 ms. */
        int provision_retry_ms = -1;
        /* VA3 zero-copy (7.7): when set and the device advertises
         * V4L2_BUF_CAP_SUPPORTS_DMABUF, CAPTURE is provisioned from these GBM
         * dma-bufs instead of device MMAP buffers, so vaExportSurfaceHandle
         * can hand the browser a GPU-importable fd. Null (or an unusable
         * allocator, or an r22 device that masks the cap) -> VA1 MMAP. Not
         * owned by the session. */
        SurfaceAllocator* allocator = nullptr;
    };

    enum class SyncStatus {
        ok,
        decode_error,
        dead,
    };

    /* Decided once per context at the first CAPTURE provisioning and logged on
     * one line (7.7 point 5): gbm-dmabuf (a surface is a GBM bo) or mmap. */
    enum class CaptureMode {
        mmap,
        gbm_dmabuf,
    };

    /* A claimed decoded frame: the CAPTURE index plus the provisioning
     * generation it belongs to (D83). A re-provision (mid-stream
     * SOURCE_CHANGE) increments the generation; releasing a stale binding is
     * a logged no-op instead of a QBUF of an index the pool no longer has. */
    struct Frame {
        unsigned index = 0;
        uint64_t generation = 0;
        friend bool operator==(const Frame&, const Frame&) = default;
    };

    StatefulSession(StatefulDevice& device, uint32_t coded_pixelformat, unsigned coded_width, unsigned coded_height,
        const Options& options, std::function<void(const char*)> log = {});

    /* Copy one access unit into the ring and queue it; throws DeviceLost or
     * std::runtime_error. */
    void submit(uint64_t sequence, std::span<const uint8_t> access_unit);

    /* Wait (bounded) for the frame with this sequence; other frames dequeued
     * on the way are stashed. */
    SyncStatus sync(uint64_t sequence, Frame* frame);

    /* Return a claimed CAPTURE buffer to the queue (surface reuse/destroy,
     * point 4); a binding from a previous provisioning generation is a
     * logged no-op. */
    void release_frame(const Frame& frame);

    /* Forget a submitted-but-never-synced sequence; its frame is re-queued
     * on arrival. */
    void drop_sequence(uint64_t sequence);

    /* vaDestroyContext path (point 6): DEC_CMD_STOP + drain + STREAMOFF both
     * + REQBUFS(0). */
    void finish();

    /* Serialise an external reader of decoded frame memory (the vaGetImage /
     * vaDeriveImage copies) against re-provisioning (D83). Never take the
     * driver-wide mutex while holding this: the established order is
     * driver mutex, then session mutex (destroySurfaces -> release_frame). */
    std::unique_lock<std::mutex> hold() { return std::unique_lock<std::mutex>(mutex_); }

    /* The CAPTURE mode chosen at provisioning (7.7 point 5); mmap until the
     * first SOURCE_CHANGE decides. Stable once provisioned; under churn read it
     * holding hold(). */
    CaptureMode capture_mode() const { return capture_mode_; }

    /* The GBM buffer backing a claimed CAPTURE index (gbm-dmabuf mode), for
     * vaExportSurfaceHandle and the vaDeriveImage/vaGetImage CPU views. Null in
     * MMAP mode or for an out-of-range index. Call holding hold(), and while
     * the surface's binding is still the current generation. */
    SurfaceBuffer* capture_buffer(unsigned index);

    bool dead() const { return dead_.load(std::memory_order_relaxed); }
    bool provisioned() const { return provisioned_.load(std::memory_order_relaxed); }
    /* Stable once provisioned; under churn read it holding hold(). */
    const CaptureFormat& capture_format() const { return capture_format_; }
    /* Bumped by every CAPTURE provisioning; under churn read it holding
     * hold(). */
    uint64_t generation() const { return generation_; }
    unsigned timeout_recoveries() const { return timeout_recoveries_; }
    unsigned idle_drains() const { return idle_drains_; }
    /* VA3-fakeau SPIKE: how many sentinel padding AUs were injected, and how
     * many of their CAPTURE outputs were recognised and dropped. Both stay 0
     * unless LIBVA_V4L2_FAKE_AU is set AND a mid-stream AV1 sync stalled. */
    unsigned fake_au_injected() const { return fake_au_injected_; }
    unsigned fake_au_dropped() const { return fake_au_dropped_; }
    int sync_timeout_ms() const { return sync_timeout_ms_; }
    int sync_idle_ms() const { return sync_idle_ms_; }
    bool allow_midstream_drain() const { return allow_midstream_drain_; }
    StatefulDevice& device() { return device_; }

private:
    /* _locked members run with mutex_ held. */
    void pump_locked();
    void handle_source_change_locked();
    /* 7.7 point 5: choose gbm-dmabuf vs mmap once, honouring the
     * LIBVA_V4L2_SURFACES override and the SUPPORTS_DMABUF capability, and log
     * the one-line reason. */
    CaptureMode decide_capture_mode_locked();
    /* Provision the CAPTURE pool in the decided mode. The gbm variant returns
     * false when it must fall back (allocation, stride or REQBUFS refusal),
     * having left nothing allocated, so the caller retries as mmap. */
    void provision_capture_mmap_locked(unsigned count);
    bool provision_capture_gbm_locked(unsigned count);
    void release_capture_pool_locked();
    /* QBUF one CAPTURE buffer in whichever mode is live (7.6/7.7). */
    void requeue_capture_locked(unsigned index);
    void handle_capture_locked(const DequeuedCapture& frame);
    bool claim_locked(uint64_t sequence, Frame* frame);
    /* Drop the lock around one device wait; exactly one thread is the
     * harvester at a time (the others sleep on the condvar). */
    void wait_for_progress(std::unique_lock<std::mutex>& lock, int timeout_ms, bool include_output);
    /* Caller owns harvesting_; true when LAST was seen. */
    bool drain_capture_locked(std::unique_lock<std::mutex>& lock, int timeout_ms);
    SyncStatus recover_locked(std::unique_lock<std::mutex>& lock, uint64_t sequence, Frame* frame, bool idle);
    /* Run a provisioning device call, retrying EBUSY with a bounded backoff
     * (post-crash: the device is still reaping a dead client). Throws
     * std::runtime_error if the budget is exhausted; other errors propagate. */
    void retry_provision(const char* what, const std::function<void()>& op);
    void grow_output_buffers_locked(std::unique_lock<std::mutex>& lock, size_t needed);
    int acquire_output_buffer_locked(std::unique_lock<std::mutex>& lock, size_t needed); /* -1 on timeout */
    /* VA3-fakeau SPIKE: copy the buffered last real AU into a free OUTPUT
     * buffer and queue it under a fresh sentinel sequence; true if one was
     * injected. No-op (returns false) when disabled or nothing is buffered. */
    bool inject_fake_au_locked(std::unique_lock<std::mutex>& lock);
    void log(const char* message);

    StatefulDevice& device_;
    std::function<void(const char*)> log_;
    unsigned num_surfaces_;
    std::function<unsigned()> surface_count_;
    SurfaceAllocator* allocator_; /* not owned; null -> MMAP only */
    unsigned output_ring_size_;
    int sync_timeout_ms_;
    int sync_idle_ms_;
    bool allow_midstream_drain_;
    bool fake_au_injection_; /* VA3-fakeau SPIKE (AV1 + LIBVA_V4L2_FAKE_AU only) */
    int provision_retry_ms_;
    uint32_t output_pixelformat_;
    uint32_t coded_width_;
    uint32_t coded_height_;
    uint32_t output_size_ = 0;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool harvesting_ = false; /* one device waiter/drainer at a time */

    std::vector<unsigned> free_outputs_;
    std::set<unsigned> usable_outputs_; /* current ring; retired buffers are dropped on dequeue */
    unsigned queued_outputs_ = 0;

    bool capture_streaming_ = false;
    std::atomic<bool> provisioned_ { false };
    std::atomic<bool> dead_ { false };
    CaptureFormat capture_format_ {};
    unsigned capture_count_ = 0;
    uint64_t generation_ = 0; /* CAPTURE provisioning generation (D83) */
    bool stale_release_logged_ = false;

    CaptureMode capture_mode_ = CaptureMode::mmap;
    bool capture_mode_decided_ = false;
    /* gbm-dmabuf mode: one GBM bo per CAPTURE index (the surfaces plus any
     * internal spares up to the codec minimum), owned for the context's life
     * (7.7 point 2). Empty in MMAP mode. */
    std::vector<std::unique_ptr<SurfaceBuffer>> capture_bos_;

    std::map<uint64_t, unsigned> stash_; /* decoded, not yet claimed */
    std::set<uint64_t> unwanted_sequences_;
    std::set<unsigned> client_owned_; /* claimed CAPTURE indices */
    uint64_t submit_count_ = 0; /* total submits; syncs watch it for input flow (D85) */
    unsigned timeout_recoveries_ = 0;
    unsigned idle_drains_ = 0;

    /* VA3-sync-reorder investigation: behaviour-neutral tracing gated on
     * LIBVA_V4L2_TRACE. Logs submit/capture/claim and, at a stalled sync, a
     * full accounting (awaited sequence, stash contents, CAPTURE ownership,
     * OUTPUT flow) so the reorder wedge mechanism can be read off directly. */
    bool trace_ = false;
    uint64_t delivered_ = 0; /* successful claims */
    void trace_dump_locked(const char* tag, uint64_t await);

    /* VA3-fakeau SPIKE. Sentinel sequences are reserved at 1e9 and up -- far
     * above any real per-context sequence (one per decoded picture from 1), so
     * a CAPTURE frame tagged >= kFakeAuSequenceBase is an injected padding AU's
     * output and is dropped, never delivered. */
    static constexpr uint64_t kFakeAuSequenceBase = 1000000000ULL;
    std::vector<uint8_t> last_au_bytes_; /* copy of the last real AU, for injection */
    uint64_t fake_au_next_seq_ = kFakeAuSequenceBase;
    unsigned fake_au_injected_ = 0;
    unsigned fake_au_dropped_ = 0;
};

} // namespace stateful
