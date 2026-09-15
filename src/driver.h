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

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <shared_mutex>

extern "C" {
#include <linux/videodev2.h>

#include <va/va.h>
}

#include "buffer.h"
#include "config.h"
#include "context.h"
#include "surface.h"
#include "v4l2.h"

class Context;

#define V4L2_STR_VENDOR "v4l2"
/* Shown by vainfo so it is obvious which path answered (VPU_DESIGN.md 7.6). */
#define V4L2_STR_VENDOR_STATEFUL "DroidVM libva-v4l2 (stateful virtio-media)"
#define V4L2_MAX_PROFILES 11
#define V4L2_MAX_ENTRYPOINTS 5
#define V4L2_MAX_IMAGE_FORMATS 10
#define V4L2_MAX_SUBPIC_FORMATS 4
#define V4L2_MAX_DISPLAY_ATTRIBUTES 4

struct DriverData {
    DriverData(const std::vector<std::pair<std::string, std::optional<std::string>>>& device_paths);

    std::map<VAConfigID, Config> configs;
    std::map<VAContextID, std::unique_ptr<Context>> contexts;
    std::map<VASurfaceID, Surface> surfaces;
    /* Mirrors surfaces.size(); readable without the mutex (D84: the
     * stateful session sizes its CAPTURE pool share from it while holding
     * its own lock, and the lock order forbids taking this mutex there). */
    std::atomic<unsigned> surface_count { 0 };
    std::map<VABufferID, Buffer> buffers;
    std::map<VAImageID, VAImage> images;
    std::vector<V4L2M2MDevice> devices;
    /* The VA display's DRM fd (VPU_DESIGN.md 7.7): the stateful GBM surface
     * allocator opens a gbm_device on it so vaExportSurfaceHandle can return a
     * GPU-importable dma-buf. -1 when the display carries no DRM fd (the
     * allocator then falls back to opening /dev/dri/renderD128, and to VA1
     * MMAP if that fails too). Borrowed from libva; never closed here. */
    int drm_fd = -1;
    /*
     * D83: the id->object maps above are read on the hot decode path
     * (vaBeginPicture/vaRenderPicture/vaEndPicture/vaSyncSurface/vaDeriveImage/
     * vaGetImage/vaMapBuffer) while another ffmpeg frame thread mutates them
     * (vaCreateBuffer/vaDestroyBuffer run per picture; ffmpeg's vaapi hwaccel
     * is HWACCEL_CAP_ASYNC_SAFE, so the overlap is real). B18/B19 caught the
     * lock-free 'contains() then .at()' racing a concurrent insert/erase and
     * aborting with std::out_of_range 'map::at'. This is now a shared_mutex:
     * readers take it SHARED (std::shared_lock), create/destroy take it
     * EXCLUSIVE (std::lock_guard/std::unique_lock). Rules a reader must keep:
     *   - resolve the id to a reference/pointer under the lock, then RELEASE
     *     the lock before any long operation (a StatefulSession sync/submit
     *     that waits on the device). std::map nodes stay valid across
     *     inserts/erases of OTHER keys, and ffmpeg never destroys a
     *     surface/buffer that is still in flight, so the resolved reference
     *     stays valid after the lock drops;
     *   - never hold this lock across StatefulSession::sync/submit, i.e.
     *     across StatefulDevice::wait. Lock order is DriverData -> session
     *     mutex, never the reverse (destroySurfaces holds this exclusively
     *     while release_frame takes the session mutex; vaDeriveImage drops the
     *     session's hold() before taking this one).
     * destroySurfaces refuses to erase a surface still VASurfaceRendering
     * (VA_STATUS_ERROR_SURFACE_BUSY) so the in-flight assumption holds even if
     * a client misbehaves.
     */
    std::shared_mutex mutex;
};

extern "C" VAStatus VA_DRIVER_INIT_FUNC(VADriverContextP context);
VAStatus terminate(VADriverContextP va_context);
