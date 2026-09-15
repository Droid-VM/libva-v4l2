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

#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <span>
#include <vector>

#include "device.h"

namespace stateful {

/*
 * The stateful M2M decode session (VPU_DESIGN.md 7.6 points 4-6): access
 * units go into a small OUTPUT ring tagged with a 64-bit sequence; the device
 * copies the tag to the decoded CAPTURE buffer; sync() collects CAPTURE
 * buffers into a sequence-keyed stash until the requested one arrives, with
 * the bounded-wait DEC_CMD_STOP/drain/START recovery of point 5b.
 */
class StatefulSession {
public:
    struct Options {
        unsigned num_surfaces = 0;
        unsigned output_ring_size = 8;
        /* < 0: LIBVA_V4L2_SYNC_TIMEOUT_MS or the 500 ms default (point 5b). */
        int sync_timeout_ms = -1;
    };

    enum class SyncStatus {
        ok,
        decode_error,
        dead,
    };

    StatefulSession(StatefulDevice& device, uint32_t coded_pixelformat, unsigned coded_width, unsigned coded_height,
        const Options& options, std::function<void(const char*)> log = {});

    /* Copy one access unit into the ring and queue it; throws DeviceLost or
     * std::runtime_error. */
    void submit(uint64_t sequence, std::span<const uint8_t> access_unit);

    /* Wait (bounded) for the frame with this sequence; other frames dequeued
     * on the way are stashed. */
    SyncStatus sync(uint64_t sequence, unsigned* capture_index);

    /* Return a claimed CAPTURE buffer to the queue (surface reuse/destroy,
     * point 4). */
    void release_frame(unsigned capture_index);

    /* Forget a submitted-but-never-synced sequence; its frame is re-queued
     * on arrival. */
    void drop_sequence(uint64_t sequence);

    /* vaDestroyContext path (point 6): DEC_CMD_STOP + drain + STREAMOFF both
     * + REQBUFS(0). */
    void finish();

    bool dead() const { return dead_; }
    bool provisioned() const { return provisioned_; }
    const CaptureFormat& capture_format() const { return capture_format_; }
    unsigned timeout_recoveries() const { return timeout_recoveries_; }
    int sync_timeout_ms() const { return sync_timeout_ms_; }
    StatefulDevice& device() { return device_; }

private:
    void pump();
    void handle_source_change();
    void handle_capture(const DequeuedCapture& frame);
    bool drain_capture(int timeout_ms); /* true when LAST was seen */
    void grow_output_buffers(size_t needed);
    int acquire_output_buffer(size_t needed); /* -1 on timeout */
    void log(const char* message);

    StatefulDevice& device_;
    std::function<void(const char*)> log_;
    unsigned num_surfaces_;
    unsigned output_ring_size_;
    int sync_timeout_ms_;
    uint32_t output_pixelformat_;
    uint32_t coded_width_;
    uint32_t coded_height_;
    uint32_t output_size_ = 0;

    std::vector<unsigned> free_outputs_;
    std::set<unsigned> usable_outputs_; /* current ring; retired buffers are dropped on dequeue */
    unsigned queued_outputs_ = 0;

    bool capture_streaming_ = false;
    bool provisioned_ = false;
    bool dead_ = false;
    CaptureFormat capture_format_ {};
    unsigned capture_count_ = 0;

    std::map<uint64_t, unsigned> stash_; /* decoded, not yet claimed */
    std::set<uint64_t> unwanted_sequences_;
    std::set<unsigned> client_owned_; /* claimed CAPTURE indices */
    unsigned timeout_recoveries_ = 0;
};

} // namespace stateful
