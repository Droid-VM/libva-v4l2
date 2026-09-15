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
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace stateful {

namespace {

    using Clock = std::chrono::steady_clock;

    constexpr int kDefaultSyncTimeoutMs = 500;
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

    int remaining_ms(Clock::time_point deadline)
    {
        auto left = std::chrono::duration_cast<std::chrono::milliseconds>(deadline - Clock::now()).count();
        return left > 0 ? static_cast<int>(left) : 0;
    }

} // namespace

StatefulSession::StatefulSession(StatefulDevice& device, uint32_t coded_pixelformat, unsigned coded_width,
    unsigned coded_height, const Options& options, std::function<void(const char*)> log)
    : device_(device)
    , log_(std::move(log))
    , num_surfaces_(options.num_surfaces)
    , output_ring_size_(std::max(options.output_ring_size, 2u))
    , sync_timeout_ms_(resolve_sync_timeout(options.sync_timeout_ms))
    , output_pixelformat_(coded_pixelformat)
    , coded_width_(coded_width)
    , coded_height_(coded_height)
{
    /* Subscribe before streaming so the first SOURCE_CHANGE cannot be lost. */
    device_.subscribe_events();

    const uint32_t initial_size = std::max(1024u * 1024u, coded_width * coded_height);
    device_.set_output_format(output_pixelformat_, coded_width_, coded_height_, initial_size);
    output_size_ = device_.output_buffer_size();

    unsigned granted = device_.request_output_buffers(output_ring_size_);
    if (granted == 0) {
        throw std::runtime_error("no OUTPUT buffers granted");
    }
    output_ring_size_ = granted;
    for (unsigned i = 0; i < granted; i++) {
        free_outputs_.push_back(i);
        usable_outputs_.insert(i);
    }

    device_.stream_output(true);
}

void StatefulSession::log(const char* message)
{
    if (log_) {
        log_(message);
    } else {
        fprintf(stderr, "libva-v4l2 stateful: %s\n", message);
    }
}

void StatefulSession::pump()
{
    while (auto event = device_.dequeue_event()) {
        if (*event == DeviceEvent::source_change) {
            handle_source_change();
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
            handle_capture(*frame);
            if (frame->last) {
                break;
            }
        }
    }
}

void StatefulSession::handle_source_change()
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
     * capped at 32; geometry from G_FMT(CAPTURE). */
    capture_format_ = device_.capture_format();
    const unsigned min_buffers = std::max(device_.min_buffers_for_capture(), 1);
    unsigned count = min_buffers + std::min(num_surfaces_, kPoolShareCap);
    count = std::min(count, kMaxCaptureBuffers);

    capture_count_ = device_.request_capture_buffers(count);
    if (capture_count_ == 0) {
        throw std::runtime_error("no CAPTURE buffers granted");
    }
    for (unsigned i = 0; i < capture_count_; i++) {
        device_.queue_capture(i);
    }
    device_.stream_capture(true);
    capture_streaming_ = true;
    provisioned_ = true;
}

void StatefulSession::handle_capture(const DequeuedCapture& frame)
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
}

int StatefulSession::acquire_output_buffer(size_t needed)
{
    const auto deadline = Clock::now() + std::chrono::milliseconds(sync_timeout_ms_);

    if (needed > output_size_) {
        grow_output_buffers(needed);
    }

    while (true) {
        pump();
        if (!free_outputs_.empty()) {
            int index = static_cast<int>(free_outputs_.back());
            free_outputs_.pop_back();
            return index;
        }
        if (Clock::now() >= deadline) {
            return -1;
        }
        device_.wait(std::min(kWaitSliceMs, remaining_ms(deadline) + 1), true);
    }
}

void StatefulSession::grow_output_buffers(size_t needed)
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
        pump();
        if (queued_outputs_ > 0) {
            device_.wait(std::min(kWaitSliceMs, remaining_ms(deadline) + 1), true);
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
    if (dead_) {
        throw std::runtime_error("session is dead");
    }
    if (access_unit.empty()) {
        throw std::invalid_argument("empty access unit");
    }

    try {
        int index = acquire_output_buffer(access_unit.size());
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
    } catch (const DeviceLost&) {
        dead_ = true;
        throw;
    }
}

bool StatefulSession::drain_capture(int timeout_ms)
{
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);

    while (true) {
        while (auto event = device_.dequeue_event()) {
            if (*event == DeviceEvent::source_change) {
                handle_source_change();
            }
        }
        while (auto frame = device_.dequeue_capture()) {
            handle_capture(*frame);
            if (frame->last) {
                return true;
            }
        }
        if (Clock::now() >= deadline) {
            return false;
        }
        device_.wait(std::min(kWaitSliceMs, remaining_ms(deadline) + 1), false);
    }
}

StatefulSession::SyncStatus StatefulSession::sync(uint64_t sequence, unsigned* capture_index)
{
    if (dead_) {
        return SyncStatus::dead;
    }

    const auto deadline = Clock::now() + std::chrono::milliseconds(sync_timeout_ms_);

    try {
        while (true) {
            pump();

            if (auto it = stash_.find(sequence); it != stash_.end()) {
                *capture_index = it->second;
                client_owned_.insert(it->second);
                stash_.erase(it);
                return SyncStatus::ok;
            }

            if (Clock::now() >= deadline) {
                /* 7.6 point 5b: assume a reorder deadlock -- the decoder holds
                 * frames the client is waiting for. Drain and restart; count
                 * it (the B18 bar wants 0 on the 1080p reference clip). */
                timeout_recoveries_ += 1;
                char line[160];
                snprintf(line, sizeof(line),
                    "sync timeout after %d ms on sequence %llu: DEC_CMD_STOP drain + restart (occurrence %u)",
                    sync_timeout_ms_, static_cast<unsigned long long>(sequence), timeout_recoveries_);
                log(line);

                device_.decoder_stop();
                drain_capture(sync_timeout_ms_);
                device_.decoder_start();

                if (auto it = stash_.find(sequence); it != stash_.end()) {
                    *capture_index = it->second;
                    client_owned_.insert(it->second);
                    stash_.erase(it);
                    return SyncStatus::ok;
                }
                return SyncStatus::decode_error;
            }

            device_.wait(std::min(kWaitSliceMs, remaining_ms(deadline) + 1), false);
        }
    } catch (const DeviceLost&) {
        dead_ = true;
        return SyncStatus::dead;
    } catch (const std::exception&) {
        return SyncStatus::decode_error;
    }
}

void StatefulSession::release_frame(unsigned capture_index)
{
    client_owned_.erase(capture_index);
    if (dead_ || !capture_streaming_) {
        return;
    }
    try {
        device_.queue_capture(capture_index);
    } catch (const DeviceLost&) {
        dead_ = true;
    }
}

void StatefulSession::drop_sequence(uint64_t sequence)
{
    if (auto it = stash_.find(sequence); it != stash_.end()) {
        unsigned index = it->second;
        stash_.erase(it);
        release_frame(index);
        return;
    }
    unwanted_sequences_.insert(sequence);
}

void StatefulSession::finish()
{
    if (!dead_) {
        try {
            /* 7.6 point 6: DEC_CMD_STOP, drain to LAST, then tear down. */
            if (capture_streaming_ && queued_outputs_ > 0) {
                device_.decoder_stop();
                drain_capture(sync_timeout_ms_);
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
}

} // namespace stateful
