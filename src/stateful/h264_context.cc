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

#include "h264_context.h"

#include <cstdio>
#include <cstring>

extern "C" {
#include <linux/videodev2.h>

#include <libdrm/drm_fourcc.h>
#include <va/va.h>
#include <va/va_drmcommon.h>
}

#include "../driver.h"
#include "../utils.h"
#include "prime_descriptor.h"

namespace {

stateful::StatefulSession::Options session_options(
    DriverData* driver_data, std::span<VASurfaceID> surface_ids, stateful::SurfaceAllocator* allocator)
{
    stateful::StatefulSession::Options options;
    options.num_surfaces = surface_ids.size();
    /* D84: modern clients pass no render targets to vaCreateContext (the
     * B18 strace showed the pool provisioned at the bare device minimum),
     * so the share is sized from the surfaces that actually exist when the
     * first SOURCE_CHANGE provisions the pool. The counter is atomic
     * because this runs under the session mutex, which must never take the
     * driver-wide mutex. */
    options.surface_count = [driver_data] { return driver_data->surface_count.load(std::memory_order_relaxed); };
    /* 7.7: provision CAPTURE from GBM dma-bufs when the allocator is usable
     * and the device advertises SUPPORTS_DMABUF; the session decides and logs
     * the mode once. */
    options.allocator = allocator;
    return options;
}

} // namespace

std::set<VAProfile> StatefulH264Context::supported_profiles(const V4L2M2MDevice& device)
{
    /* See the header: the VA1 hardcode replaced by device profile menus in
     * VA1b (VPU_DESIGN.md 7.6 point 2). */
    return (device.stateful_decoder() && device.format_supported(device.output_buf_type, V4L2_PIX_FMT_H264))
        ? std::set<VAProfile> { VAProfileH264ConstrainedBaseline, VAProfileH264Main, VAProfileH264High }
        : std::set<VAProfile>();
}

StatefulH264Context::StatefulH264Context(DriverData* driver_data, V4L2M2MDevice& device, VAProfile profile,
    int picture_width, int picture_height, std::span<VASurfaceID> surface_ids)
    : Context(driver_data, device, picture_width, picture_height)
    , profile_(profile)
    , device_io_(device)
    , allocator_(driver_data->drm_fd)
    , session_(device_io_, V4L2_PIX_FMT_H264, picture_width, picture_height,
          session_options(driver_data, surface_ids, &allocator_))
    , au_builder_(profile)
{
}

StatefulH264Context::~StatefulH264Context()
{
    for (auto&& [id, surface] : driver_data->surfaces) {
        if (surface.stateful_context == this) {
            surface.stateful_context = nullptr;
            surface.stateful_capture_index = -1;
            if (surface.status == VASurfaceRendering || surface.status == VASurfaceDisplaying) {
                surface.status = VASurfaceReady;
            }
        }
    }
    session_.finish();
}

VAStatus StatefulH264Context::store_buffer(const Buffer& buffer) const
{
    std::lock_guard<std::mutex> guard(builder_mutex_);

    switch (buffer.type) {
    case VAPictureParameterBufferType:
        au_builder_.set_picture_parameters(*reinterpret_cast<const VAPictureParameterBufferH264*>(buffer.data.get()));
        break;

    case VAIQMatrixBufferType:
        au_builder_.set_iq_matrix(*reinterpret_cast<const VAIQMatrixBufferH264*>(buffer.data.get()));
        break;

    case VASliceParameterBufferType:
        au_builder_.add_slice_parameters(
            { reinterpret_cast<const VASliceParameterBufferH264*>(buffer.data.get()), buffer.count });
        break;

    case VASliceDataBufferType:
        try {
            au_builder_.add_slice_data({ buffer.data.get(), static_cast<size_t>(buffer.size) * buffer.count });
        } catch (const std::exception&) {
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }
        break;

    default:
        return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
    }

    return VA_STATUS_SUCCESS;
}

void StatefulH264Context::stateful_begin_picture(Surface& surface)
{
    /* 7.6 point 4: a surface reused by a later vaBeginPicture re-queues its
     * CAPTURE buffer. */
    if (surface.stateful_context == this && surface.stateful_capture_index >= 0) {
        session_.release_frame(
            { static_cast<unsigned>(surface.stateful_capture_index), surface.stateful_capture_generation });
        surface.stateful_capture_index = -1;
    }
}

VAStatus StatefulH264Context::stateful_end_picture(VADriverContextP va_context, Surface& surface)
{
    std::vector<uint8_t> access_unit;
    try {
        std::lock_guard<std::mutex> guard(builder_mutex_);
        access_unit = au_builder_.finish(surface.width, surface.height);
    } catch (const std::exception& e) {
        error_log(va_context, "Failed to synthesize access unit: %s\n", e.what());
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    if (access_unit.empty()) {
        error_log(va_context, "vaEndPicture without picture parameters or slices\n");
        return VA_STATUS_ERROR_INVALID_PARAMETER;
    }

    uint64_t sequence;
    {
        std::lock_guard<std::mutex> guard(builder_mutex_);
        sequence = next_sequence_++;
    }
    try {
        session_.submit(sequence, access_unit);
    } catch (const std::exception& e) {
        error_log(va_context, "Failed to submit access unit: %s\n", e.what());
        return session_.dead() ? VA_STATUS_ERROR_DECODING_ERROR : VA_STATUS_ERROR_OPERATION_FAILED;
    }

    surface.stateful_context = this;
    surface.stateful_sequence = sequence;
    surface.stateful_capture_index = -1;

    render_surface_id = VA_INVALID_ID;
    return VA_STATUS_SUCCESS;
}

VAStatus StatefulH264Context::sync_surface(VADriverContextP va_context, Surface& surface)
{
    if (session_.dead()) {
        /* 7.6 point 6: the session died (ENODEV and friends); the client
         * must recreate the context. */
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    if (surface.status != VASurfaceRendering) {
        return VA_STATUS_SUCCESS;
    }

    stateful::StatefulSession::Frame frame;
    switch (session_.sync(surface.stateful_sequence, &frame)) {
    case stateful::StatefulSession::SyncStatus::ok:
        surface.stateful_capture_index = static_cast<int>(frame.index);
        surface.stateful_capture_generation = frame.generation;
        surface.status = VASurfaceDisplaying;
        return VA_STATUS_SUCCESS;
    case stateful::StatefulSession::SyncStatus::dead:
    case stateful::StatefulSession::SyncStatus::decode_error:
    default:
        /* D88: a failed sync must leave the surface REUSABLE. If it stayed
         * VASurfaceRendering, the client's next vaBeginPicture on this pooled
         * surface would be rejected VA_STATUS_ERROR_SURFACE_BUSY (16) -- the
         * cascade B19 §6.2 saw, where one dropped frame turned into dozens of
         * "surface is in use" and a short, wrong decode. The frame was never
         * claimed (no CAPTURE binding to release); just unbind and free the
         * status so the surface can carry the next picture. */
        surface.stateful_capture_index = -1;
        surface.status = VASurfaceReady;
        error_log(va_context, "Stateful sync failed on sequence %llu\n",
            static_cast<unsigned long long>(surface.stateful_sequence));
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
}

void StatefulH264Context::release_surface(Surface& surface)
{
    if (surface.stateful_capture_index >= 0) {
        session_.release_frame(
            { static_cast<unsigned>(surface.stateful_capture_index), surface.stateful_capture_generation });
        surface.stateful_capture_index = -1;
    } else if (surface.status == VASurfaceRendering) {
        session_.drop_sequence(surface.stateful_sequence);
    }
    surface.stateful_context = nullptr;
}

std::optional<StatefulH264Context::FrameView> StatefulH264Context::frame_view(const Surface& surface)
{
    if (surface.stateful_capture_index < 0 || !session_.provisioned()
        || surface.stateful_capture_generation != session_.generation()) {
        return std::nullopt; /* stale binding after a re-provision (D83) */
    }

    const auto& format = session_.capture_format();
    const unsigned index = static_cast<unsigned>(surface.stateful_capture_index);
    FrameView view;
    view.pitch = format.bytesperline != 0 ? format.bytesperline : format.width;
    view.coded_width = format.width;
    view.coded_height = format.height;

    const size_t luma_size = static_cast<size_t>(view.pitch) * format.height;

    if (session_.capture_mode() == stateful::StatefulSession::CaptureMode::gbm_dmabuf) {
        /* 7.7 point 4: one gbm_bo_map (here, the bo's stable CPU view) covers
         * the whole NV12 container, so derive works in both plane layouts --
         * luma at offset 0, chroma at pitch*height. */
        stateful::SurfaceBuffer* bo = session_.capture_buffer(index);
        if (bo == nullptr) {
            return std::nullopt;
        }
        auto whole = bo->map();
        if (whole.size() < luma_size + luma_size / 2) {
            return std::nullopt;
        }
        view.luma = whole.subspan(0, luma_size);
        view.chroma = whole.subspan(luma_size, luma_size / 2);
        view.contiguous = whole;
        return view;
    }

    if (format.num_planes == 1) {
        auto plane = device_io_.capture_plane(index, 0);
        if (plane.size() < luma_size + luma_size / 2) {
            return std::nullopt;
        }
        view.luma = plane.subspan(0, luma_size);
        view.chroma = plane.subspan(luma_size, luma_size / 2);
        view.contiguous = plane;
    } else {
        view.luma = device_io_.capture_plane(index, 0);
        view.chroma = device_io_.capture_plane(index, 1);
    }
    return view;
}

VAStatus StatefulH264Context::export_surface(
    VADriverContextP va_context, Surface& surface, uint32_t flags, void* descriptor)
{
    if (session_.dead()) {
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    /* Sync first if the client exports before syncing (7.7 acceptance: a
     * not-yet-decoded surface must not hand out a stale buffer). */
    if (surface.status == VASurfaceRendering) {
        VAStatus status = sync_surface(va_context, surface);
        if (status != VA_STATUS_SUCCESS) {
            return status;
        }
    }

    auto frames_guard = session_.hold();

    const bool gbm = session_.capture_mode() == stateful::StatefulSession::CaptureMode::gbm_dmabuf;
    switch (stateful::export_gate(session_.dead(), surface.stateful_capture_index >= 0, gbm,
        surface.stateful_capture_generation == session_.generation())) {
    case stateful::ExportGate::dead:
        return VA_STATUS_ERROR_DECODING_ERROR;
    case stateful::ExportGate::busy:
        return VA_STATUS_ERROR_SURFACE_BUSY;
    case stateful::ExportGate::unimplemented:
        /* VA1 MMAP mode: no GPU-importable buffer (7.6 point 7). The browser
         * falls back to software as it did before VA3. */
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    case stateful::ExportGate::ready:
        break;
    }

    stateful::SurfaceBuffer* bo = session_.capture_buffer(static_cast<unsigned>(surface.stateful_capture_index));
    if (bo == nullptr) {
        return VA_STATUS_ERROR_SURFACE_BUSY;
    }
    int fd = bo->export_fd();
    if (fd < 0) {
        error_log(va_context, "vaExportSurfaceHandle: dup of the GBM dma-buf failed\n");
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    const auto& format = session_.capture_format();
    const uint32_t stride = format.bytesperline != 0 ? format.bytesperline : format.width;
    stateful::fill_nv12_prime_descriptor(static_cast<VADRMPRIMESurfaceDescriptor*>(descriptor), fd, surface.width,
        surface.height, stride, static_cast<uint32_t>(bo->size()), flags);
    return VA_STATUS_SUCCESS;
}
