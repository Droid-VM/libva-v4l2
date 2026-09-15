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

#include "v4l2_device.h"

#include <algorithm>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <system_error>

extern "C" {
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
}

#include "../v4l2.h"

namespace stateful {

namespace {

    std::string ioctl_error(const char* name, int error)
    {
        return std::string(name) + ": " + strerror(error);
    }

} // namespace

V4L2StatefulDevice::V4L2StatefulDevice(V4L2M2MDevice& m2m_device)
    : fd_(m2m_device.video_fd)
    , output_type_(m2m_device.output_buf_type)
    , capture_type_(m2m_device.capture_buf_type)
{
}

V4L2StatefulDevice::~V4L2StatefulDevice()
{
    unmap_buffers(output_buffers_);
    unmap_buffers(capture_buffers_);
}

int V4L2StatefulDevice::xioctl(unsigned long request, void* argument, const char* name)
{
    int result;
    do {
        result = ioctl(fd_, request, argument);
    } while (result < 0 && errno == EINTR);

    if (result < 0) {
        if (errno == ENODEV || errno == ENXIO || errno == EBADF) {
            throw DeviceLost(ioctl_error(name, errno));
        }
        return -errno;
    }
    return result;
}

void V4L2StatefulDevice::unmap_buffers(std::vector<MappedBuffer>& buffers)
{
    for (auto& buffer : buffers) {
        for (auto& plane : buffer.planes) {
            if (!plane.empty()) {
                munmap(plane.data(), plane.size());
            }
        }
    }
    buffers.clear();
}

void V4L2StatefulDevice::map_buffers(
    v4l2_buf_type type, unsigned first, unsigned count, std::vector<MappedBuffer>& buffers)
{
    if (buffers.size() < first + count) {
        buffers.resize(first + count);
    }

    for (unsigned index = first; index < first + count; index++) {
        v4l2_plane planes[VIDEO_MAX_PLANES] = {};
        v4l2_buffer buffer = {
            .index = index,
            .type = type,
            .m = { .planes = planes },
            .length = VIDEO_MAX_PLANES,
        };
        if (int error = xioctl(VIDIOC_QUERYBUF, &buffer, "VIDIOC_QUERYBUF"); error < 0) {
            throw std::system_error(-error, std::generic_category(), "VIDIOC_QUERYBUF");
        }

        unsigned plane_count;
        struct {
            uint32_t length;
            uint32_t offset;
        } plane_info[VIDEO_MAX_PLANES];
        if (V4L2_TYPE_IS_MULTIPLANAR(type)) {
            plane_count = buffer.length;
            for (unsigned i = 0; i < plane_count; i++) {
                plane_info[i] = { planes[i].length, static_cast<uint32_t>(planes[i].m.mem_offset) };
            }
        } else {
            plane_count = 1;
            plane_info[0] = { buffer.length, buffer.m.offset };
        }

        MappedBuffer mapped;
        for (unsigned i = 0; i < plane_count; i++) {
            void* address
                = mmap(nullptr, plane_info[i].length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, plane_info[i].offset);
            if (address == MAP_FAILED) {
                throw std::system_error(errno, std::generic_category(), "mmap");
            }
            mapped.planes.emplace_back(static_cast<uint8_t*>(address), plane_info[i].length);
        }
        buffers[index] = std::move(mapped);
    }
}

void V4L2StatefulDevice::set_output_format(uint32_t pixelformat, uint32_t width, uint32_t height, uint32_t sizeimage)
{
    v4l2_format format = { .type = output_type_ };
    if (V4L2_TYPE_IS_MULTIPLANAR(output_type_)) {
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.width = width;
        format.fmt.pix_mp.height = height;
        format.fmt.pix_mp.num_planes = 1;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;
    } else {
        format.fmt.pix.pixelformat = pixelformat;
        format.fmt.pix.width = width;
        format.fmt.pix.height = height;
        format.fmt.pix.sizeimage = sizeimage;
    }
    if (int error = xioctl(VIDIOC_S_FMT, &format, "VIDIOC_S_FMT"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_S_FMT(OUTPUT)");
    }
    output_format_ = format;
}

uint32_t V4L2StatefulDevice::output_buffer_size()
{
    if (V4L2_TYPE_IS_MULTIPLANAR(output_type_)) {
        return output_format_.fmt.pix_mp.plane_fmt[0].sizeimage;
    }
    return output_format_.fmt.pix.sizeimage;
}

CaptureFormat V4L2StatefulDevice::capture_format()
{
    v4l2_format format = { .type = capture_type_ };
    if (int error = xioctl(VIDIOC_G_FMT, &format, "VIDIOC_G_FMT"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_G_FMT(CAPTURE)");
    }

    CaptureFormat result;
    if (V4L2_TYPE_IS_MULTIPLANAR(capture_type_)) {
        result.pixelformat = format.fmt.pix_mp.pixelformat;
        result.width = format.fmt.pix_mp.width;
        result.height = format.fmt.pix_mp.height;
        result.bytesperline = format.fmt.pix_mp.plane_fmt[0].bytesperline;
        result.sizeimage = format.fmt.pix_mp.plane_fmt[0].sizeimage;
        result.num_planes = format.fmt.pix_mp.num_planes;
    } else {
        result.pixelformat = format.fmt.pix.pixelformat;
        result.width = format.fmt.pix.width;
        result.height = format.fmt.pix.height;
        result.bytesperline = format.fmt.pix.bytesperline;
        result.sizeimage = format.fmt.pix.sizeimage;
        result.num_planes = 1;
    }
    return result;
}

unsigned V4L2StatefulDevice::request_output_buffers(unsigned count)
{
    unmap_buffers(output_buffers_);
    v4l2_requestbuffers request = { .count = count, .type = output_type_, .memory = V4L2_MEMORY_MMAP };
    if (int error = xioctl(VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_REQBUFS(OUTPUT)");
    }
    if (request.count > 0) {
        map_buffers(output_type_, 0, request.count, output_buffers_);
    }
    return request.count;
}

std::pair<unsigned, unsigned> V4L2StatefulDevice::create_output_buffers(unsigned count, uint32_t sizeimage)
{
    v4l2_create_buffers create = { .count = count, .memory = V4L2_MEMORY_MMAP, .format = output_format_ };
    if (V4L2_TYPE_IS_MULTIPLANAR(output_type_)) {
        create.format.fmt.pix_mp.plane_fmt[0].sizeimage = sizeimage;
    } else {
        create.format.fmt.pix.sizeimage = sizeimage;
    }
    if (int error = xioctl(VIDIOC_CREATE_BUFS, &create, "VIDIOC_CREATE_BUFS"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_CREATE_BUFS(OUTPUT)");
    }
    if (create.count > 0) {
        map_buffers(output_type_, create.index, create.count, output_buffers_);
    }
    return { create.index, create.count };
}

std::span<uint8_t> V4L2StatefulDevice::output_plane(unsigned index)
{
    return output_buffers_.at(index).planes.at(0);
}

unsigned V4L2StatefulDevice::request_capture_buffers(unsigned count)
{
    unmap_buffers(capture_buffers_);
    v4l2_requestbuffers request = { .count = count, .type = capture_type_, .memory = V4L2_MEMORY_MMAP };
    if (int error = xioctl(VIDIOC_REQBUFS, &request, "VIDIOC_REQBUFS"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_REQBUFS(CAPTURE)");
    }
    if (request.count > 0) {
        map_buffers(capture_type_, 0, request.count, capture_buffers_);
    }
    return request.count;
}

std::span<uint8_t> V4L2StatefulDevice::capture_plane(unsigned index, unsigned plane)
{
    check_capture_index(index);
    const auto& planes = capture_buffers_[index].planes;
    ensure_capture_plane(plane, planes.size());
    return planes[plane];
}

void V4L2StatefulDevice::queue_output(unsigned index, uint64_t sequence, unsigned bytes_used)
{
    v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    v4l2_buffer buffer = {
        .index = index,
        .type = output_type_,
        .memory = V4L2_MEMORY_MMAP,
    };
    buffer.timestamp = sequence_to_timeval(sequence);
    if (V4L2_TYPE_IS_MULTIPLANAR(output_type_)) {
        planes[0].bytesused = bytes_used;
        buffer.m.planes = planes;
        buffer.length = 1;
    } else {
        buffer.bytesused = bytes_used;
    }
    if (int error = xioctl(VIDIOC_QBUF, &buffer, "VIDIOC_QBUF"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_QBUF(OUTPUT)");
    }
}

std::optional<uint32_t> V4L2StatefulDevice::dequeue_output()
{
    v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    v4l2_buffer buffer = {
        .type = output_type_,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (V4L2_TYPE_IS_MULTIPLANAR(output_type_)) {
        buffer.m.planes = planes;
        buffer.length = VIDEO_MAX_PLANES;
    }
    if (int error = xioctl(VIDIOC_DQBUF, &buffer, "VIDIOC_DQBUF"); error < 0) {
        if (error == -EAGAIN || error == -EPIPE) {
            return std::nullopt;
        }
        throw std::system_error(-error, std::generic_category(), "VIDIOC_DQBUF(OUTPUT)");
    }
    return buffer.index;
}

void V4L2StatefulDevice::check_capture_index(unsigned index) const
{
    /* D83: a stale index (a binding surviving a CAPTURE re-provision) must
     * fail with a message that names the bug, not a bare std::out_of_range
     * from a container. It is not DeviceLost: the device is fine. */
    ensure_capture_index(index, capture_buffers_.size());
}

void V4L2StatefulDevice::queue_capture(unsigned index)
{
    check_capture_index(index);

    v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    v4l2_buffer buffer = {
        .index = index,
        .type = capture_type_,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (V4L2_TYPE_IS_MULTIPLANAR(capture_type_)) {
        buffer.m.planes = planes;
        buffer.length = static_cast<uint32_t>(capture_buffers_[index].planes.size());
    }
    if (int error = xioctl(VIDIOC_QBUF, &buffer, "VIDIOC_QBUF"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_QBUF(CAPTURE)");
    }
}

std::optional<DequeuedCapture> V4L2StatefulDevice::dequeue_capture()
{
    v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    v4l2_buffer buffer = {
        .type = capture_type_,
        .memory = V4L2_MEMORY_MMAP,
    };
    if (V4L2_TYPE_IS_MULTIPLANAR(capture_type_)) {
        buffer.m.planes = planes;
        buffer.length = VIDEO_MAX_PLANES;
    }
    if (int error = xioctl(VIDIOC_DQBUF, &buffer, "VIDIOC_DQBUF"); error < 0) {
        if (error == -EAGAIN) {
            return std::nullopt;
        }
        if (error == -EPIPE) {
            /* The LAST buffer was already delivered; report a synthetic
             * end-of-drain marker. */
            DequeuedCapture marker;
            marker.last = true;
            return marker;
        }
        throw std::system_error(-error, std::generic_category(), "VIDIOC_DQBUF(CAPTURE)");
    }

    DequeuedCapture result;
    result.index = buffer.index;
    result.sequence = timeval_to_sequence(buffer.timestamp);
    result.last = (buffer.flags & V4L2_BUF_FLAG_LAST) != 0;
    result.error = (buffer.flags & V4L2_BUF_FLAG_ERROR) != 0;
    if (V4L2_TYPE_IS_MULTIPLANAR(capture_type_)) {
        for (unsigned i = 0; i < buffer.length; i++) {
            result.bytesused += planes[i].bytesused;
        }
    } else {
        result.bytesused = buffer.bytesused;
    }
    return result;
}

int V4L2StatefulDevice::min_buffers_for_capture()
{
    v4l2_control control = { .id = V4L2_CID_MIN_BUFFERS_FOR_CAPTURE };
    if (int error = xioctl(VIDIOC_G_CTRL, &control, "VIDIOC_G_CTRL"); error < 0) {
        return 1; /* the control is optional on other stateful decoders */
    }
    return control.value;
}

void V4L2StatefulDevice::subscribe_events()
{
    for (uint32_t type : { static_cast<uint32_t>(V4L2_EVENT_SOURCE_CHANGE), static_cast<uint32_t>(V4L2_EVENT_EOS) }) {
        v4l2_event_subscription subscription = { .type = type };
        if (int error = xioctl(VIDIOC_SUBSCRIBE_EVENT, &subscription, "VIDIOC_SUBSCRIBE_EVENT"); error < 0) {
            throw std::system_error(-error, std::generic_category(), "VIDIOC_SUBSCRIBE_EVENT");
        }
    }
}

std::optional<DeviceEvent> V4L2StatefulDevice::dequeue_event()
{
    v4l2_event event = {};
    if (int error = xioctl(VIDIOC_DQEVENT, &event, "VIDIOC_DQEVENT"); error < 0) {
        return std::nullopt; /* ENOENT: no event pending */
    }
    switch (event.type) {
    case V4L2_EVENT_SOURCE_CHANGE:
        return DeviceEvent::source_change;
    case V4L2_EVENT_EOS:
        return DeviceEvent::eos;
    default:
        return std::nullopt;
    }
}

void V4L2StatefulDevice::decoder_stop()
{
    v4l2_decoder_cmd command = { .cmd = V4L2_DEC_CMD_STOP };
    if (int error = xioctl(VIDIOC_DECODER_CMD, &command, "VIDIOC_DECODER_CMD"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_DECODER_CMD(STOP)");
    }
}

void V4L2StatefulDevice::decoder_start()
{
    v4l2_decoder_cmd command = { .cmd = V4L2_DEC_CMD_START };
    if (int error = xioctl(VIDIOC_DECODER_CMD, &command, "VIDIOC_DECODER_CMD"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_DECODER_CMD(START)");
    }
}

void V4L2StatefulDevice::stream_output(bool enable)
{
    int type = output_type_;
    if (int error = xioctl(enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type, "VIDIOC_STREAM*"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_STREAMON/OFF(OUTPUT)");
    }
}

void V4L2StatefulDevice::stream_capture(bool enable)
{
    int type = capture_type_;
    if (int error = xioctl(enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &type, "VIDIOC_STREAM*"); error < 0) {
        throw std::system_error(-error, std::generic_category(), "VIDIOC_STREAMON/OFF(CAPTURE)");
    }
}

bool V4L2StatefulDevice::wait(int timeout_ms, bool include_output)
{
    pollfd descriptor = { .fd = fd_, .events = static_cast<short>(POLLIN | POLLPRI | (include_output ? POLLOUT : 0)) };
    int result;
    do {
        result = poll(&descriptor, 1, timeout_ms);
    } while (result < 0 && errno == EINTR);

    if (result < 0) {
        throw std::system_error(errno, std::generic_category(), "poll");
    }
    if ((descriptor.revents & (POLLERR | POLLNVAL)) != 0 && (descriptor.revents & (POLLIN | POLLPRI | POLLOUT)) == 0) {
        /* POLLERR alone: vb2 flags an empty CAPTURE queue this way before the
         * pool exists. poll() returned immediately, so pace the caller's
         * retry loop instead of spinning, and let the next ioctl decide
         * whether the device is really gone. */
        usleep(static_cast<useconds_t>(std::min(timeout_ms, 10)) * 1000);
        return false;
    }
    return result > 0;
}

} // namespace stateful
