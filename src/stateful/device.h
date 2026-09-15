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
#include <optional>
#include <span>
#include <stdexcept>
#include <utility>

extern "C" {
#include <sys/time.h>
}

namespace stateful {

/*
 * The narrow seam between the stateful session state machine and the V4L2
 * ioctls it uses (VPU_DESIGN.md 7.6; host-side verification fakes this
 * interface to script SOURCE_CHANGE, MIN_BUFFERS, timestamp-copied DQBUFs,
 * out-of-order delivery, stalls and ENODEV).
 */

/* The device vanished (DQBUF ENODEV and friends): the session is dead and
 * later syncs return VA_STATUS_ERROR_DECODING_ERROR (7.6 point 6). */
class DeviceLost : public std::runtime_error {
public:
    using std::runtime_error::runtime_error;
};

struct CaptureFormat {
    uint32_t pixelformat = 0; /* V4L2 fourcc, NV12 expected */
    uint32_t width = 0; /* coded size */
    uint32_t height = 0;
    uint32_t bytesperline = 0; /* luma stride */
    uint32_t sizeimage = 0; /* total bytes of plane 0 */
    uint32_t num_planes = 0; /* memory planes (1 for NV12) */
};

struct DequeuedCapture {
    static constexpr uint32_t no_buffer = UINT32_MAX;

    uint32_t index = no_buffer;
    uint64_t sequence = 0; /* decoded from the copied timestamp */
    uint32_t bytesused = 0;
    bool last = false; /* V4L2_BUF_FLAG_LAST */
    bool error = false; /* V4L2_BUF_FLAG_ERROR */
};

enum class DeviceEvent {
    source_change,
    eos,
};

class StatefulDevice {
public:
    virtual ~StatefulDevice() = default;

    /* Formats. */
    virtual void set_output_format(uint32_t pixelformat, uint32_t width, uint32_t height, uint32_t sizeimage) = 0;
    virtual uint32_t output_buffer_size() = 0; /* granted OUTPUT sizeimage */
    virtual CaptureFormat capture_format() = 0; /* G_FMT(CAPTURE) */

    /* Buffers. Requesting maps the pool; a request of 0 releases it. */
    virtual unsigned request_output_buffers(unsigned count) = 0;
    /* CREATE_BUFS: adds buffers of the given size to the streaming OUTPUT
     * queue; returns {first index, count}. */
    virtual std::pair<unsigned, unsigned> create_output_buffers(unsigned count, uint32_t sizeimage) = 0;
    virtual std::span<uint8_t> output_plane(unsigned index) = 0;
    virtual unsigned request_capture_buffers(unsigned count) = 0;
    virtual std::span<uint8_t> capture_plane(unsigned index, unsigned plane) = 0;

    /* Queueing; dequeues are non-blocking. The OUTPUT timestamp carries the
     * 64-bit sequence (7.6 point 4). */
    virtual void queue_output(unsigned index, uint64_t sequence, unsigned bytes_used) = 0;
    virtual std::optional<uint32_t> dequeue_output() = 0;
    virtual void queue_capture(unsigned index) = 0;
    virtual std::optional<DequeuedCapture> dequeue_capture() = 0;

    /* Controls, events, commands, streaming. */
    virtual int min_buffers_for_capture() = 0; /* V4L2_CID_MIN_BUFFERS_FOR_CAPTURE */
    virtual void subscribe_events() = 0; /* SOURCE_CHANGE + EOS */
    virtual std::optional<DeviceEvent> dequeue_event() = 0;
    virtual void decoder_stop() = 0; /* VIDIOC_DECODER_CMD(V4L2_DEC_CMD_STOP) */
    virtual void decoder_start() = 0;
    virtual void stream_output(bool enable) = 0;
    virtual void stream_capture(bool enable) = 0;

    /* Wait for capture data or an event (and optionally OUTPUT space);
     * false on timeout. */
    virtual bool wait(int timeout_ms, bool include_output) = 0;
};

/* The OUTPUT timestamp <-> sequence mapping; the device copies the timeval
 * verbatim to the CAPTURE buffer (TIMESTAMP_COPY, video_decoder.rs:244). */
inline timeval sequence_to_timeval(uint64_t sequence)
{
    return timeval { static_cast<time_t>(sequence / 1000000u), static_cast<suseconds_t>(sequence % 1000000u) };
}

inline uint64_t timeval_to_sequence(const timeval& tv)
{
    return static_cast<uint64_t>(tv.tv_sec) * 1000000u + static_cast<uint64_t>(tv.tv_usec);
}

} // namespace stateful
