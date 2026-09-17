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

extern "C" {
#include <linux/videodev2.h>
}

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
    /* VA3 (7.7): the CAPTURE queue's advertised V4L2_BUF_CAP_* bits. Default
     * carries no SUPPORTS_DMABUF, so the session picks MMAP (the r22 shape); a
     * gbm test adds V4L2_BUF_CAP_SUPPORTS_DMABUF (the r23 shape). */
    uint32_t capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP;
    /* set_capture_stride: true grants the requested stride; false refuses (it
     * returns the device's own bytesperline), so a bo-stride mismatch falls
     * back to MMAP. */
    bool capture_stride_negotiable = true;
    /* D90 runtime shape: REQBUFS(CAPTURE,DMABUF) is granted but the first N
     * DMABUF QBUFs fail -EIO (the protected guest's restricted DMA pool refuses
     * the virtio-gpu vram exporter's map_dma_buf). errno is configurable so the
     * EFAULT/EINVAL variants can be exercised too. */
    int eio_dmabuf_qbufs = 0;
    int dmabuf_qbuf_errno = EIO;

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

    /* VA3 observability: the DMABUF provisioning the gbm path takes. */
    bool capture_dmabuf = false; /* the CAPTURE queue was REQBUFS'd as DMABUF */
    unsigned capabilities_probes = 0; /* capture_buffer_capabilities() calls */
    unsigned set_capture_stride_calls = 0;
    struct QueuedDmabuf {
        unsigned index;
        std::vector<stateful::DmabufPlane> planes;
    };
    std::vector<QueuedDmabuf> dmabuf_queues; /* every DMABUF QBUF, in order */

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

    uint32_t capture_buffer_capabilities() override
    {
        check_alive();
        capabilities_probes += 1;
        return capture_caps;
    }

    uint32_t set_capture_stride(uint32_t bytesperline) override
    {
        check_alive();
        set_capture_stride_calls += 1;
        if (!capture_stride_negotiable) {
            return scripted_format.bytesperline;
        }
        /* Model a real S_FMT(CAPTURE) that adopts a padded luma stride (VA2g):
         * the granted stride becomes the geometry a subsequent G_FMT reports,
         * so the session's "re-read the negotiated geometry" sees the wider
         * bytesperline AND the matching sizeimage. Before VA2g the fake granted
         * the stride but left scripted_format untouched, so capture_format()
         * kept reporting the tight stride and the negotiated-then-gbm path was
         * only checked by its call count -- test_non_aligned_resolution_
         * negotiates_stride now asserts the adopted geometry. */
        scripted_format.bytesperline = bytesperline;
        scripted_format.sizeimage = bytesperline * (scripted_format.height + scripted_format.height / 2);
        return bytesperline;
    }

    unsigned request_capture_buffers_dmabuf(unsigned count) override
    {
        check_alive();
        if (count > 0 && ebusy_capture_provisions > 0) {
            ebusy_capture_provisions -= 1;
            throw std::system_error(EBUSY, std::generic_category(), "VIDIOC_REQBUFS(CAPTURE,DMABUF)");
        }
        capture_dmabuf = count > 0;
        capture_count = count;
        free_captures.clear();
        return count;
    }

    void queue_capture_dmabuf(unsigned index, std::span<const stateful::DmabufPlane> planes) override
    {
        check_alive();
        if (eio_dmabuf_qbufs > 0) {
            eio_dmabuf_qbufs -= 1;
            throw std::system_error(dmabuf_qbuf_errno, std::generic_category(), "VIDIOC_QBUF(CAPTURE,DMABUF)");
        }
        dmabuf_queues.push_back({ index, { planes.begin(), planes.end() } });
        free_captures.push_back(index);
    }

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
