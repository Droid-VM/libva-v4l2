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

#include <vector>

extern "C" {
#include <linux/videodev2.h>
}

#include "device.h"

class V4L2M2MDevice;

namespace stateful {

/* The real StatefulDevice: plain MMAP V4L2 ioctls on the M2M video node (no
 * media node, no requests -- 7.6 point 1). */
class V4L2StatefulDevice final : public StatefulDevice {
public:
    explicit V4L2StatefulDevice(V4L2M2MDevice& m2m_device);
    ~V4L2StatefulDevice() override;

    V4L2StatefulDevice(const V4L2StatefulDevice&) = delete;
    V4L2StatefulDevice& operator=(const V4L2StatefulDevice&) = delete;

    void set_output_format(uint32_t pixelformat, uint32_t width, uint32_t height, uint32_t sizeimage) override;
    uint32_t output_buffer_size() override;
    CaptureFormat capture_format() override;

    unsigned request_output_buffers(unsigned count) override;
    std::pair<unsigned, unsigned> create_output_buffers(unsigned count, uint32_t sizeimage) override;
    std::span<uint8_t> output_plane(unsigned index) override;
    unsigned request_capture_buffers(unsigned count) override;
    std::span<uint8_t> capture_plane(unsigned index, unsigned plane) override;

    void queue_output(unsigned index, uint64_t sequence, unsigned bytes_used) override;
    std::optional<uint32_t> dequeue_output() override;
    void queue_capture(unsigned index) override;
    std::optional<DequeuedCapture> dequeue_capture() override;

    int min_buffers_for_capture() override;
    void subscribe_events() override;
    std::optional<DeviceEvent> dequeue_event() override;
    void decoder_stop() override;
    void decoder_start() override;
    void stream_output(bool enable) override;
    void stream_capture(bool enable) override;

    bool wait(int timeout_ms, bool include_output) override;

private:
    struct MappedBuffer {
        std::vector<std::span<uint8_t>> planes;
    };

    int xioctl(unsigned long request, void* argument, const char* name);
    void check_capture_index(unsigned index) const;
    void map_buffers(v4l2_buf_type type, unsigned first, unsigned count, std::vector<MappedBuffer>& buffers);
    static void unmap_buffers(std::vector<MappedBuffer>& buffers);

    int fd_;
    v4l2_buf_type output_type_;
    v4l2_buf_type capture_type_;
    v4l2_format output_format_ {};
    std::vector<MappedBuffer> output_buffers_;
    std::vector<MappedBuffer> capture_buffers_;
};

} // namespace stateful
