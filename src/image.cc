/*
 * Copyright (C) 2007 Intel Corporation
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2023 Max Schettler <max.schettler@posteo.de>
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

#include "image.h"

#include <algorithm>
#include <cassert>
#include <cstring>
#include <va/va.h>

extern "C" {
#include <linux/videodev2.h>
}

#include "buffer.h"
#include "driver.h"
#include "format.h"
#include "stateful/h264_context.h"
#include "surface.h"
#include "utils.h"
#include "v4l2.h"

namespace {

/* Stateful path (VPU_DESIGN.md 7.6 point 4): row-wise NV12 copy from the
 * claimed CAPTURE buffer's mmap, honouring the G_FMT stride. */
VAStatus copy_stateful_surface_to_image(DriverData* driver_data, const Surface& surface, VAImage* image)
{
    /* D83: resolve the destination buffer under the shared lock, then release
     * it before taking the session's hold() (lock order: driver -> session).
     * The image's own buffer is not destroyed while its getImage runs. */
    uint8_t* dest;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto buffer_it = driver_data->buffers.find(image->buf);
        if (buffer_it == driver_data->buffers.end()) {
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }
        dest = buffer_it->second.map();
    }

    /* Hold the session while reading the CAPTURE mmap so a concurrent
     * re-provision cannot unmap it mid-copy. */
    auto frames_guard = surface.stateful_context->hold_frames();
    auto view = surface.stateful_context->frame_view(surface);
    if (!view) {
        return VA_STATUS_ERROR_SURFACE_BUSY;
    }

    if (image->num_planes != 2) {
        return VA_STATUS_ERROR_INVALID_IMAGE_FORMAT;
    }

    const unsigned luma_rows = std::min<unsigned>(image->height, view->coded_height);
    const unsigned row_bytes = std::min<unsigned>(image->pitches[0], view->pitch);
    for (unsigned row = 0; row < luma_rows; row++) {
        std::copy_n(view->luma.data() + static_cast<size_t>(row) * view->pitch, row_bytes,
            dest + image->offsets[0] + static_cast<size_t>(row) * image->pitches[0]);
    }
    const unsigned chroma_rows = std::min<unsigned>((image->height + 1) / 2, view->coded_height / 2);
    const unsigned chroma_bytes = std::min<unsigned>(image->pitches[1], view->pitch);
    for (unsigned row = 0; row < chroma_rows; row++) {
        std::copy_n(view->chroma.data() + static_cast<size_t>(row) * view->pitch, chroma_bytes,
            dest + image->offsets[1] + static_cast<size_t>(row) * image->pitches[1]);
    }

    return VA_STATUS_SUCCESS;
}

VAStatus copy_surface_to_image(DriverData* driver_data, const Surface& surface, VAImage* image)
{
    unsigned int i;

    if (surface.stateful_context != nullptr) {
        return copy_stateful_surface_to_image(driver_data, surface, image);
    }

    uint8_t* dest_base;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto buffer_it = driver_data->buffers.find(image->buf);
        if (buffer_it == driver_data->buffers.end()) {
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }
        dest_base = buffer_it->second.data.get();
    }

    assert(image->num_planes == surface.logical_destination_layout.size());
    for (i = 0; i < surface.logical_destination_layout.size(); i++) {
        const auto& mapping = surface.destination_buffer->get().mapping();

        const auto source = mapping[surface.logical_destination_layout[i].physical_plane_index].data()
            + surface.logical_destination_layout[i].offset;
        const auto dest = dest_base + image->offsets[i];

        // Image planes may be smaller than buffer due to decoding blocks
        const auto size
            = ((i < (surface.logical_destination_layout.size() - 1)) ? image->offsets[i + 1] : image->data_size)
            - image->offsets[i];
        std::copy_n(source, size, dest);
    }

    return VA_STATUS_SUCCESS;
}

} // namespace

VAStatus createImage(VADriverContextP context, VAImageFormat* format, int width, int height, VAImage* image)
{
    auto driver_data = static_cast<DriverData*>(context->pDriverData);

    memset(image, 0, sizeof(*image));
    image->format = *format;
    image->width = width;
    image->height = height;

    BufferLayout (*derive_layout)(unsigned, unsigned) = nullptr;
    try {
        derive_layout = lookup_format(format->fourcc).v4l2.derive_layout;
    } catch (std::invalid_argument& e) { // TODO decouple planarity from format layout
        error_log(context, "Image format not specified\n");
        return VA_STATUS_ERROR_INVALID_IMAGE_FORMAT;
    }
    if (!derive_layout) {
        error_log(context, "Image format not specified\n");
        return VA_STATUS_ERROR_INVALID_IMAGE_FORMAT;
    }

    const auto layout = derive_layout(width, height);

    image->num_planes = layout.size();
    for (unsigned i = 0; i < image->num_planes; i += 1) {
        image->data_size += layout[i].size;
        image->pitches[i] = layout[i].pitch;
        image->offsets[i] = layout[i].offset;
    }

    VAStatus status = createBuffer(context, 0, VAImageBufferType, image->data_size, 1, NULL, &image->buf);
    if (status != VA_STATUS_SUCCESS) {
        return status;
    }

    std::lock_guard<std::shared_mutex> guard(driver_data->mutex);
    image->image_id = smallest_free_key(driver_data->images);
    auto [image_it, inserted] = driver_data->images.emplace(std::make_pair(image->image_id, *image));
    if (!inserted) {
        return VA_STATUS_ERROR_ALLOCATION_FAILED;
    }

    return VA_STATUS_SUCCESS;
}

VAStatus destroyImage(VADriverContextP context, VAImageID image_id)
{
    auto driver_data = static_cast<DriverData*>(context->pDriverData);

    /* D83: read the image's buffer id under the shared lock, then drop it --
     * destroyBuffer takes the lock exclusively itself, and the final erase
     * retakes it. */
    VABufferID buf;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto image_it = driver_data->images.find(image_id);
        if (image_it == driver_data->images.end()) {
            return VA_STATUS_ERROR_INVALID_IMAGE;
        }
        buf = image_it->second.buf;
    }

    VAStatus status = destroyBuffer(context, buf);
    if (status != VA_STATUS_SUCCESS) {
        return status;
    }

    std::lock_guard<std::shared_mutex> guard(driver_data->mutex);
    if (!driver_data->images.erase(image_id)) {
        return VA_STATUS_ERROR_INVALID_IMAGE;
    }

    return VA_STATUS_SUCCESS;
}

VAStatus deriveImage(VADriverContextP context, VASurfaceID surface_id, VAImage* image)
{
    auto driver_data = static_cast<DriverData*>(context->pDriverData);
    VAImageFormat format;
    VAStatus status;

    /* D83: resolve the surface under the shared lock, then drop it before
     * syncSurface (which waits on the device and takes the lock itself). The
     * surface reference stays valid across the release. */
    Surface* surface_ptr;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto surface_it = driver_data->surfaces.find(surface_id);
        if (surface_it == driver_data->surfaces.end()) {
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        surface_ptr = &surface_it->second;
    }
    auto& surface = *surface_ptr;

    if (surface.stateful_context != nullptr) {
        /* 7.6 point 4: map the claimed CAPTURE buffer's NV12 mmap directly,
         * with the G_FMT stride. This is a single-buffer view: it works only
         * when the device presents NV12 as one contiguous CAPTURE plane
         * (num_planes == 1, so frame_view fills contiguous). A device that
         * reports two memory planes (separate luma/chroma mmaps) has no single
         * buffer to derive, so this returns OPERATION_FAILED and the client
         * copies through vaCreateImage + vaGetImage instead (GStreamer's va
         * plugin logs 'use derived: false' and does exactly that -- 300/300 in
         * B19-browser); getImage's copy path handles both plane counts. */
        if (surface.status == VASurfaceRendering) {
            status = syncSurface(context, surface_id);
            if (status != VA_STATUS_SUCCESS)
                return status;
        }
        auto frames_guard = surface.stateful_context->hold_frames();
        auto view = surface.stateful_context->frame_view(surface);
        if (!view || view->contiguous.empty()) {
            /* 7.7 (4): explain the OPERATION_FAILED once. The frames guard does
             * not take the driver mutex, and the log helpers do not either, so
             * this is safe to call while holding it. */
            const bool have_view = view.has_value();
            frames_guard.unlock();
            surface.stateful_context->note_derive_unavailable(context, have_view);
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }

        memset(image, 0, sizeof(*image));
        image->format = { .fourcc = VA_FOURCC_NV12, .byte_order = VA_LSB_FIRST, .bits_per_pixel = 12 };
        image->width = surface.width;
        image->height = surface.height;
        image->data_size = view->contiguous.size();
        image->num_planes = 2;
        image->pitches[0] = view->pitch;
        image->pitches[1] = view->pitch;
        image->offsets[0] = 0;
        image->offsets[1] = view->pitch * view->coded_height;

        /* Lock order is driver mutex before session mutex (destroySurfaces
         * holds the former while releasing frames): drop the session guard
         * before taking the driver mutex. The mapped pointer escaping into
         * the image buffer is inherent to vaDeriveImage. */
        frames_guard.unlock();

        std::lock_guard<std::shared_mutex> guard(driver_data->mutex);
        VABufferID buffer_id = smallest_free_key(driver_data->buffers);
        driver_data->buffers.emplace(std::make_pair(
            buffer_id, Buffer(VAImageBufferType, 1, image->data_size, surface_id, view->contiguous.data())));
        image->buf = buffer_id;

        image->image_id = smallest_free_key(driver_data->images);
        auto [image_it, inserted] = driver_data->images.emplace(std::make_pair(image->image_id, *image));
        if (!inserted) {
            driver_data->buffers.erase(buffer_id);
            return VA_STATUS_ERROR_ALLOCATION_FAILED;
        }

        surface.status = VASurfaceReady;
        return VA_STATUS_SUCCESS;
    }

    // Attempt to derive image from uninitialized surface
    if (!surface.destination_buffer) {
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    if (surface.status == VASurfaceRendering) {
        status = syncSurface(context, surface_id);
        if (status != VA_STATUS_SUCCESS)
            return status;
    }

    format.fourcc = VA_FOURCC_NV12;

    status = createImage(context, &format, surface.width, surface.height, image);
    if (status != VA_STATUS_SUCCESS)
        return status;

    status = copy_surface_to_image(driver_data, surface, image);
    if (status != VA_STATUS_SUCCESS)
        return status;

    surface.status = VASurfaceReady;

    std::lock_guard<std::shared_mutex> guard(driver_data->mutex);
    auto buffer_it = driver_data->buffers.find(image->buf);
    if (buffer_it == driver_data->buffers.end()) {
        return VA_STATUS_ERROR_INVALID_BUFFER;
    }
    buffer_it->second.derived_surface_id = surface_id;

    return VA_STATUS_SUCCESS;
}

VAStatus queryImageFormats(VADriverContextP context, VAImageFormat* formats, int* formats_count)
{
    formats[0].fourcc = VA_FOURCC_NV12;
    *formats_count = 1;

    return VA_STATUS_SUCCESS;
}

VAStatus setImagePalette(VADriverContextP context, VAImageID image_id, unsigned char* palette)
{
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}

VAStatus getImage(VADriverContextP context, VASurfaceID surface_id, int x, int y, unsigned int width,
    unsigned int height, VAImageID image_id)
{
    auto driver_data = static_cast<DriverData*>(context->pDriverData);

    /* D83: resolve the surface reference and a copy of the image descriptor
     * under the shared lock, then release it before the copy. The copy only
     * writes into the image's own buffer memory (re-resolved under the lock in
     * copy_surface_to_image), never the maps; neither the surface nor the
     * image is destroyed by the client mid-getImage. */
    Surface* surface_ptr;
    VAImage image;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto surface_it = driver_data->surfaces.find(surface_id);
        if (surface_it == driver_data->surfaces.end()) {
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        surface_ptr = &surface_it->second;

        auto image_it = driver_data->images.find(image_id);
        if (image_it == driver_data->images.end()) {
            return VA_STATUS_ERROR_INVALID_IMAGE;
        }
        image = image_it->second;
    }

    if (x != 0 || y != 0 || width != image.width || height != image.height)
        return VA_STATUS_ERROR_UNIMPLEMENTED;

    return copy_surface_to_image(driver_data, *surface_ptr, &image);
}

VAStatus putImage(VADriverContextP context, VASurfaceID surface_id, VAImageID image, int src_x, int src_y,
    unsigned int src_width, unsigned int src_height, int dst_x, int dst_y, unsigned int dst_width,
    unsigned int dst_height)
{
    return VA_STATUS_ERROR_UNIMPLEMENTED;
}
