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

#include "picture.h"

#include <cassert>
#include <cstring>
#include <functional>
#include <system_error>

extern "C" {
#include <linux/videodev2.h>
#include <sys/ioctl.h>

#include <va/va.h>
}

#include "context.h"
#include "driver.h"
#include "media.h"
#include "surface.h"
#include "utils.h"
#include "v4l2.h"

using fourcc = uint32_t;

VAStatus beginPicture(VADriverContextP va_context, VAContextID context_id, VASurfaceID surface_id)
{
    auto driver_data = static_cast<DriverData*>(va_context->pDriverData);

    /* D83: resolve context and surface under the shared lock; a concurrent
     * vaCreateBuffer/vaDestroyBuffer on another frame thread must not race
     * the lookups. Release the lock before touching the session. */
    Context* context;
    Surface* surface;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto context_it = driver_data->contexts.find(context_id);
        if (context_it == driver_data->contexts.end()) {
            return VA_STATUS_ERROR_INVALID_CONTEXT;
        }
        context = context_it->second.get();

        auto surface_it = driver_data->surfaces.find(surface_id);
        if (surface_it == driver_data->surfaces.end()) {
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        surface = &surface_it->second;

        /* A surface the client decoded into and released without syncing is not
         * busy -- the codec never decoded into it (see
         * allows_abandoned_surface_reuse); the context below drops the abandoned
         * sequence and the client reuses the surface. */
        if (surface->status == VASurfaceRendering && !context->allows_abandoned_surface_reuse()) {
            return VA_STATUS_ERROR_SURFACE_BUSY;
        }
    }

    /* Stateful path: a reused surface re-queues its CAPTURE buffer
     * (VPU_DESIGN.md 7.6 point 4). */
    context->stateful_begin_picture(*surface);

    surface->status = VASurfaceRendering;
    context->render_surface_id = surface_id;

    return VA_STATUS_SUCCESS;
}

VAStatus renderPicture(VADriverContextP va_context, VAContextID context_id, VABufferID* buffers_ids, int buffers_count)
{
    auto driver_data = static_cast<DriverData*>(va_context->pDriverData);
    int rc;
    int i;

    /* D83: store_buffer copies the buffer's bytes immediately (no device
     * wait), so the shared lock is held for the whole call -- it is what keeps
     * the buffers map stable against the vaCreateBuffer/vaDestroyBuffer the
     * client interleaves. (Named mutation the threads test fails under: drop
     * this lock -- TSan reports the map race.) */
    std::shared_lock<std::shared_mutex> guard(driver_data->mutex);

    auto context_it = driver_data->contexts.find(context_id);
    if (context_it == driver_data->contexts.end()) {
        return VA_STATUS_ERROR_INVALID_CONTEXT;
    }
    const auto& context = *context_it->second;

    if (!driver_data->surfaces.contains(context.render_surface_id)) {
        return VA_STATUS_ERROR_INVALID_SURFACE;
    }
    for (i = 0; i < buffers_count; i++) {
        auto buffer_it = driver_data->buffers.find(buffers_ids[i]);
        if (buffer_it == driver_data->buffers.end()) {
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }

        rc = context.store_buffer(buffer_it->second);
        if (rc != VA_STATUS_SUCCESS)
            return rc;
    }

    return VA_STATUS_SUCCESS;
}

VAStatus endPicture(VADriverContextP va_context, VAContextID context_id)
{
    auto driver_data = static_cast<DriverData*>(va_context->pDriverData);
    VAStatus status;

    /* D83: resolve refs under the shared lock, then release it before the
     * session submit (which waits on the device) or the stateless queue path
     * (which does its own I/O). */
    Context* context_ptr;
    Surface* surface_ptr;
    {
        std::shared_lock<std::shared_mutex> guard(driver_data->mutex);
        auto context_it = driver_data->contexts.find(context_id);
        if (context_it == driver_data->contexts.end()) {
            return VA_STATUS_ERROR_INVALID_CONTEXT;
        }
        context_ptr = context_it->second.get();
        auto surface_it = driver_data->surfaces.find(context_ptr->render_surface_id);
        if (surface_it == driver_data->surfaces.end()) {
            return VA_STATUS_ERROR_INVALID_SURFACE;
        }
        surface_ptr = &surface_it->second;
    }
    auto& context = *context_ptr;
    auto& surface = *surface_ptr;

    if (context.is_stateful()) {
        /* One access unit per vaEndPicture into one OUTPUT buffer
         * (VPU_DESIGN.md 7.6 point 3). */
        return context.stateful_end_picture(va_context, surface);
    }

    gettimeofday(&surface.timestamp, NULL);

    if (context.device.media_fd >= 0) {
        if (surface.request_fd < 0) {
            surface.request_fd = media_request_alloc(context.device.media_fd);
        }

        status = context.set_controls();
        if (status != VA_STATUS_SUCCESS)
            return status;
    }

    try {
        surface.destination_buffer->get().queue();
        surface.source_buffer->get().queue(surface.request_fd, &surface.timestamp, surface.source_size_used);
    } catch (std::system_error& e) {
        error_log(va_context, "Unable to queue buffer: %s\n", e.what());
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    if (surface.request_fd >= 0) {
        try {
            media_request_queue(surface.request_fd);
            media_request_wait_completion(surface.request_fd);
            media_request_reinit(surface.request_fd);
        } catch (std::runtime_error& e) {
            close(surface.request_fd);
            surface.request_fd = -1;
            error_log(va_context, "Failed to process request: %s\n", e.what());
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }

    surface.source_size_used = 0;

    context.render_surface_id = VA_INVALID_ID;
    memset(&surface.params, 0, sizeof(surface.params));

    return VA_STATUS_SUCCESS;
}
