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

#include <cerrno>
#include <cstring>
#include <deque>
#include <functional>
#include <optional>
#include <stdexcept>
#include <system_error>
#include <vector>

#include "../../src/stateful/device.h"

/*
 * A scripted fake of the StatefulDevice seam (VPU_DESIGN.md 7.6 host
 * verification (c)). It models a decoder that produces a frame as soon as it
 * holds both an OUTPUT buffer (queued) and a CAPTURE buffer (provisioned
 * after the SOURCE_CHANGE the fake raises on the first submit). Tests choose:
 *
 *  - MIN_BUFFERS_FOR_CAPTURE and the coded format,
 *  - delivery order: automatic FIFO, or manual via deliver()/a delivery
 *    queue (for out-of-order and stall scenarios),
 *  - stalls (manual mode + no scheduled delivery until decoder_stop),
 *  - ENODEV (lose_device) and CREATE_BUFS being unsupported.
 */
class FakeDevice final : public stateful::StatefulDevice {
public:
    /* --- scripting knobs --- */
    stateful::CaptureFormat scripted_format = {
        .pixelformat = 0x3231564e, /* NV12 */
        .width = 1920,
        .height = 1088,
        .bytesperline = 1920,
        .sizeimage = 1920 * 1088 * 3 / 2,
        .num_planes = 1,
    };
    int scripted_min_buffers = 4;
    bool manual_delivery = false; /* when true, only deliver() produces frames */
    bool lose_device = false;
    bool fail_create_bufs = false;
    /* Post-crash provisioning (D88): the device refuses REQBUFS this many
     * times with EBUSY while it reaps a dead client's session, then succeeds.
     * A huge value models a device that never releases it. */
    int ebusy_output_provisions = 0;
    int ebusy_capture_provisions = 0;
    /* Model a stateful decoder whose DEC_CMD_STOP seek pauses the OUTPUT
     * queue: feeding it again needs STREAMON(OUTPUT) after DEC_CMD_START
     * (D86). When set, queue_output on a paused queue fails, so a recovery
     * that does not resume OUTPUT starves the next decode -- the B18
     * one-shot. */
    bool stop_pauses_output = false;
    std::function<void()> on_decoder_stop;

    /* --- observable state --- */
    struct QueuedOutput {
        unsigned index;
        uint64_t sequence;
        unsigned bytes;
    };
    std::deque<QueuedOutput> pending_decodes;
    std::deque<uint64_t> delivery_queue; /* explicit delivery order */
    std::deque<unsigned> completed_outputs;
    std::deque<unsigned> free_captures;
    std::deque<stateful::DequeuedCapture> ready_captures;
    std::deque<stateful::DeviceEvent> pending_events;
    unsigned output_count = 0;
    unsigned capture_count = 0;
    uint32_t output_size = 0;
    bool output_streaming = false;
    bool capture_streaming = false;
    bool source_change_pending_on_submit = true;
    unsigned decoder_stops = 0;
    unsigned decoder_starts = 0;
    unsigned create_bufs_calls = 0;
    unsigned stream_off_output_calls = 0;
    bool subscribed = false;
    std::vector<std::vector<uint8_t>> output_memory;

    /* Schedule a specific sequence to be delivered next (manual order or an
     * override of FIFO); flags LAST on the buffer. */
    void deliver(uint64_t sequence, bool last = false)
    {
        delivery_queue.push_back(sequence);
        if (last) {
            last_sequences_.push_back(sequence);
        }
    }

    /* The decoder emitting a final empty LAST buffer with no frame. */
    void deliver_empty_last() { deliver_empty_last_ = true; }

    /* --- StatefulDevice --- */
    void set_output_format(uint32_t, uint32_t, uint32_t, uint32_t sizeimage) override
    {
        check_alive();
        output_size = sizeimage;
    }

    uint32_t output_buffer_size() override { return output_size; }

    stateful::CaptureFormat capture_format() override
    {
        check_alive();
        return scripted_format;
    }

    unsigned request_output_buffers(unsigned count) override
    {
        check_alive();
        if (ebusy_output_provisions > 0) {
            ebusy_output_provisions -= 1;
            throw std::system_error(EBUSY, std::generic_category(), "VIDIOC_REQBUFS(OUTPUT)");
        }
        output_count = count;
        output_memory.assign(count, std::vector<uint8_t>(output_size));
        return count;
    }

    std::pair<unsigned, unsigned> create_output_buffers(unsigned count, uint32_t sizeimage) override
    {
        check_alive();
        create_bufs_calls += 1;
        if (fail_create_bufs) {
            throw std::system_error(ENOTTY, std::generic_category(), "VIDIOC_CREATE_BUFS");
        }
        unsigned first = output_count;
        output_count += count;
        output_memory.resize(output_count);
        for (unsigned i = first; i < output_count; i++) {
            output_memory[i].assign(sizeimage, 0);
        }
        output_size = sizeimage;
        return { first, count };
    }

    std::span<uint8_t> output_plane(unsigned index) override { return output_memory.at(index); }

    unsigned request_capture_buffers(unsigned count) override
    {
        check_alive();
        if (count > 0 && ebusy_capture_provisions > 0) {
            ebusy_capture_provisions -= 1;
            throw std::system_error(EBUSY, std::generic_category(), "VIDIOC_REQBUFS(CAPTURE)");
        }
        capture_count = count;
        free_captures.clear();
        return count;
    }

    std::span<uint8_t> capture_plane(unsigned, unsigned) override { return {}; }

    void queue_output(unsigned index, uint64_t sequence, unsigned bytes) override
    {
        check_alive();
        if (stop_pauses_output && !output_streaming) {
            throw std::system_error(EPIPE, std::generic_category(), "VIDIOC_QBUF(OUTPUT): queue paused by seek");
        }
        pending_decodes.push_back({ index, sequence, bytes });
        if (source_change_pending_on_submit) {
            source_change_pending_on_submit = false;
            pending_events.push_back(stateful::DeviceEvent::source_change);
        }
    }

    std::optional<uint32_t> dequeue_output() override
    {
        check_alive();
        if (completed_outputs.empty()) {
            return std::nullopt;
        }
        unsigned index = completed_outputs.front();
        completed_outputs.pop_front();
        return index;
    }

    void queue_capture(unsigned index) override
    {
        check_alive();
        free_captures.push_back(index);
    }

    std::optional<stateful::DequeuedCapture> dequeue_capture() override
    {
        check_alive();
        try_decode();
        if (ready_captures.empty()) {
            return std::nullopt;
        }
        auto frame = ready_captures.front();
        ready_captures.pop_front();
        return frame;
    }

    int min_buffers_for_capture() override
    {
        check_alive();
        return scripted_min_buffers;
    }

    void subscribe_events() override { subscribed = true; }

    std::optional<stateful::DeviceEvent> dequeue_event() override
    {
        check_alive();
        if (pending_events.empty()) {
            return std::nullopt;
        }
        auto event = pending_events.front();
        pending_events.pop_front();
        return event;
    }

    void decoder_stop() override
    {
        check_alive();
        decoder_stops += 1;
        if (stop_pauses_output) {
            output_streaming = false; /* the seek pauses OUTPUT until STREAMON */
        }
        if (on_decoder_stop) {
            on_decoder_stop();
        }
    }

    void decoder_start() override
    {
        check_alive();
        decoder_starts += 1;
    }

    void stream_output(bool enable) override
    {
        check_alive();
        if (!enable && output_streaming) {
            stream_off_output_calls += 1;
        }
        output_streaming = enable;
    }

    void stream_capture(bool enable) override
    {
        check_alive();
        capture_streaming = enable;
    }

    bool wait(int, bool) override
    {
        try_decode();
        return !ready_captures.empty() || !pending_events.empty() || !completed_outputs.empty();
    }

private:
    void check_alive()
    {
        if (lose_device) {
            throw stateful::DeviceLost("fake device lost");
        }
    }

    bool is_last(uint64_t sequence) const
    {
        for (uint64_t s : last_sequences_) {
            if (s == sequence) {
                return true;
            }
        }
        return false;
    }

    /* Model the decode: turn one queued OUTPUT plus one free CAPTURE into a
     * decoded frame, in FIFO order unless a delivery order was scripted. */
    void try_decode()
    {
        if (deliver_empty_last_ && !free_captures.empty()) {
            deliver_empty_last_ = false;
            stateful::DequeuedCapture frame;
            frame.index = free_captures.front();
            free_captures.pop_front();
            frame.bytesused = 0;
            frame.last = true;
            ready_captures.push_back(frame);
            return;
        }

        while (!pending_decodes.empty() && !free_captures.empty()) {
            uint64_t sequence;
            if (!delivery_queue.empty()) {
                sequence = delivery_queue.front();
            } else if (!manual_delivery) {
                sequence = pending_decodes.front().sequence;
            } else {
                return; /* stalled: nothing scheduled */
            }

            auto it = pending_decodes.begin();
            for (; it != pending_decodes.end(); ++it) {
                if (it->sequence == sequence) {
                    break;
                }
            }
            if (it == pending_decodes.end()) {
                return; /* scheduled sequence not queued yet */
            }
            if (!delivery_queue.empty()) {
                delivery_queue.pop_front();
            }

            completed_outputs.push_back(it->index);
            stateful::DequeuedCapture frame;
            frame.index = free_captures.front();
            free_captures.pop_front();
            frame.sequence = sequence;
            frame.bytesused = scripted_format.sizeimage;
            frame.last = is_last(sequence);
            ready_captures.push_back(frame);
            pending_decodes.erase(it);
        }
    }

    std::vector<uint64_t> last_sequences_;
    bool deliver_empty_last_ = false;
};
