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

#pragma once

#include <cstdint>
#include <mutex>
#include <span>

extern "C" {
#include <va/va_backend.h>
}

#include "buffer.h"
#include "v4l2.h"

struct DriverData;
struct Surface;

class Context {
public:
    static Context* create(DriverData* driver_data, VAProfile profile, int picture_width, int picture_height,
        std::span<VASurfaceID> surface_ids);
    static std::set<VAProfile> supported_profiles(const std::vector<V4L2M2MDevice>& devices);

    Context(DriverData* driver_data, V4L2M2MDevice& device, fourcc pixelformat, int picture_width, int picture_height,
        std::span<VASurfaceID> surface_ids);
    virtual ~Context();

    virtual VAStatus store_buffer(const Buffer& buffer) const = 0;
    virtual int set_controls() = 0;

    /* Stateful decode path (VPU_DESIGN.md 7.6): a stateful context owns the
     * whole queue lifecycle, so vaBeginPicture/vaEndPicture divert to these
     * hooks instead of the stateless per-request flow. */
    virtual bool is_stateful() const { return false; }
    virtual void stateful_begin_picture(Surface& surface) { (void)surface; }
    virtual VAStatus stateful_end_picture(VADriverContextP va_context, Surface& surface)
    {
        (void)va_context;
        (void)surface;
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    /*
     * The codec-agnostic stateful-surface interface (VPU_DESIGN.md 7.6/7.7).
     * A Surface's stateful_context is a Context*, and vaSyncSurface /
     * vaDestroySurfaces / vaExportSurfaceHandle / the vaGetImage-vaDeriveImage
     * copies reach the owning stateful context through these. The concrete
     * stateful contexts (StatefulH264Context, StatefulVP9Context) override
     * them against their own session; the default no-op/error bodies are never
     * reached on a stateless context (whose surfaces carry no stateful_context).
     * frame_view is deliberately NOT here: it returns a codec-owned view type,
     * so image.cc resolves it per concrete context (see stateful::FrameView).
     */
    virtual VAStatus sync_surface(VADriverContextP va_context, Surface& surface)
    {
        (void)va_context;
        (void)surface;
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }
    virtual void release_surface(Surface& surface) { (void)surface; }
    virtual VAStatus export_surface(VADriverContextP va_context, Surface& surface, uint32_t flags, void* descriptor)
    {
        (void)va_context;
        (void)surface;
        (void)flags;
        (void)descriptor;
        return VA_STATUS_ERROR_UNIMPLEMENTED;
    }
    virtual std::unique_lock<std::mutex> hold_frames() { return {}; }
    virtual void note_derive_unavailable(VADriverContextP va_context, bool have_view)
    {
        (void)va_context;
        (void)have_view;
    }

    VASurfaceID render_surface_id;
    int picture_width;
    int picture_height;
    DriverData* driver_data;
    V4L2M2MDevice& device;

protected:
    /* For stateful contexts: no capture allocation, no per-surface buffer
     * binding, no streaming -- the session does that itself. */
    Context(DriverData* driver_data, V4L2M2MDevice& device, int picture_width, int picture_height);
};

VAStatus createContext(VADriverContextP va_context, VAConfigID config_id, int picture_width, int picture_height,
    int flags, VASurfaceID* surfaces_ids, int surfaces_count, VAContextID* context_id);
VAStatus destroyContext(VADriverContextP va_context, VAContextID context_id);
