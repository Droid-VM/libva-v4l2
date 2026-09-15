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
    , output_ring_size_(std::max(options.output_ring_size, 2u))
    , sync_timeout_ms_(resolve_sync_timeout(options.sync_timeout_ms))
    , sync_idle_ms_(resolve_sync_idle(options.sync_idle_ms))
    , provision_retry_ms_(options.provision_retry_ms >= 0 ? options.provision_retry_ms : kDefaultProvisionRetryMs)
    , output_pixelformat_(coded_pixelformat)
    , coded_width_(coded_width)
    , coded_height_(coded_height)
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

void StatefulSession::handle_source_change_locked()
{
    if (provisioned_) {
        /* Mid-stream resolution change: VA1 re-provisions and drops what was
         * decoded against the old pool. */
        log("mid-stream SOURCE_CHANGE: re-provisioning the CAPTURE pool, dropping stashed frames");
        if (capture_streaming_) {
            device_.stream_capture(false);
            capture_streaming_ = false;
        }
        stash_.clear();
        client_owned_.clear();
        device_.request_capture_buffers(0);
        provisioned_ = false;
    }

    /* 7.6 point 4: pool size = MIN_BUFFERS_FOR_CAPTURE + min(num_surfaces, 8),
     * capped at 32; geometry from G_FMT(CAPTURE). D84: num_surfaces from
     * vaCreateContext is 0 with modern clients, so re-read the live surface
     * count here -- provisioning happens at the first SOURCE_CHANGE, by
     * which time vaCreateSurfaces has made the pool. Without the share the
     * client's held surfaces come out of the codec's own slots and the
     * session deadlocks the way mpv did in B18. */
    if (surface_count_) {
        num_surfaces_ = std::max(num_surfaces_, surface_count_());
    }
    capture_format_ = device_.capture_format();
    const unsigned min_buffers = std::max(device_.min_buffers_for_capture(), 1);
    const unsigned share = std::min(num_surfaces_, kPoolShareCap);
    unsigned count = std::min(min_buffers + share, kMaxCaptureBuffers);

    /* Post-crash EBUSY retry (D88): the same reaping window can refuse
     * REQBUFS(CAPTURE)/STREAMON(CAPTURE) for the first client after a crash.
     * Retrying here (not per-picture) turns it into one clean failure of the
     * first sync instead of a cascade of "surface is in use" per picture. */
    retry_provision("REQBUFS(CAPTURE)", [&] { capture_count_ = device_.request_capture_buffers(count); });
    if (capture_count_ == 0) {
        throw std::runtime_error("no CAPTURE buffers granted");
    }
    generation_ += 1; /* D83: every binding handed out before this is stale */

    char line[128];
    snprintf(line, sizeof(line), "CAPTURE pool: min %u + share %u = %u (surfaces %u, granted %u)", min_buffers, share,
        count, num_surfaces_, capture_count_);
    log(line);
    for (unsigned i = 0; i < capture_count_; i++) {
        device_.queue_capture(i);
    }
    retry_provision("STREAMON(CAPTURE)", [&] { device_.stream_capture(true); });
    capture_streaming_ = true;
    provisioned_ = true;
    cv_.notify_all();
}

void StatefulSession::handle_capture_locked(const DequeuedCapture& frame)
{
    if (frame.index == DequeuedCapture::no_buffer) {
        return; /* synthetic end-of-drain marker */
    }
    if (frame.bytesused == 0 || frame.error) {
        /* An empty LAST (or erroneous) buffer carries no frame; recycle it. */
        device_.queue_capture(frame.index);
        return;
    }
    if (unwanted_sequences_.erase(frame.sequence) > 0) {
        device_.queue_capture(frame.index);
        return;
    }
    stash_[frame.sequence] = frame.index;
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

    try {
        while (true) {
            pump_locked();

            if (claim_locked(sequence, frame)) {
                return SyncStatus::ok;
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
                    return SyncStatus::decode_error;
                }
                wait_for_progress(lock, std::min(kWaitSliceMs, remaining_ms(deadline) + 1), false);
                continue;
            }

            const bool idle_expired = now >= idle_deadline;
            if (hard_expired || idle_expired) {
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
        device_.queue_capture(frame.index);
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
            device_.queue_capture(index);
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
        device_.request_capture_buffers(0);
    } catch (const std::exception&) {
        /* The device is gone; nothing left to release. */
    }

    capture_streaming_ = false;
    provisioned_ = false;
    stash_.clear();
    client_owned_.clear();
    free_outputs_.clear();
    usable_outputs_.clear();
    queued_outputs_ = 0;

    harvesting_ = false;
    cv_.notify_all();
}

} // namespace stateful
