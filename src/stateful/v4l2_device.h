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

#include <cstddef>
#include <cstdio>
#include <stdexcept>
#include <vector>

extern "C" {
#include <linux/videodev2.h>
}

#include "device.h"

class V4L2M2MDevice;

namespace stateful {

/*
 * D83: a stale CAPTURE binding -- an index that survived a re-provision, or a
 * plane count the client never saw -- must fail with a message that names the
 * bug, never a bare container exception (std::out_of_range "map::at" /
 * "vector::_M_range_check") that terminates the process. These are the checked
 * lookups the hot path uses instead of operator[]/.at(); they are free inline
 * functions so the bound check is unit-testable without a device.
 */
[[noreturn]] inline void throw_capture_out_of_range(const char* what, unsigned value, std::size_t bound)
{
    char message[112];
    snprintf(message, sizeof(message), "CAPTURE %s %u outside the provisioned pool of %zu", what, value, bound);
    throw std::runtime_error(message);
}

inline void ensure_capture_index(unsigned index, std::size_t pool_size)
{
    if (index >= pool_size) {
        throw_capture_out_of_range("index", index, pool_size);
    }
}

inline void ensure_capture_plane(unsigned plane, std::size_t plane_count)
{
    if (plane >= plane_count) {
        throw_capture_out_of_range("plane", plane, plane_count);
    }
}

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

    uint32_t capture_buffer_capabilities() override;
    uint32_t set_capture_stride(uint32_t bytesperline) override;
    unsigned request_capture_buffers_dmabuf(unsigned count) override;
    void queue_capture_dmabuf(unsigned index, std::span<const DmabufPlane> planes) override;

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
    /* MMAP (VA1) or DMABUF (VA3 gbm): DQBUF must name the CAPTURE queue's
     * memory model, and DMABUF buffers are client-owned so they are never
     * mmap'd here. */
    v4l2_memory capture_memory_ = V4L2_MEMORY_MMAP;
    v4l2_format output_format_ {};
    std::vector<MappedBuffer> output_buffers_;
    std::vector<MappedBuffer> capture_buffers_;
};

} // namespace stateful
