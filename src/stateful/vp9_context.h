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

#include <atomic>
#include <mutex>
#include <optional>
#include <set>
#include <span>

extern "C" {
#include <va/va_backend.h>
}

#include "../context.h"
#include "../surface.h"
#include "frame_view.h"
#include "gbm_allocator.h"
#include "session.h"
#include "v4l2_device.h"
#include "vp9_bitstream.h"

struct DriverData;

/*
 * The stateful VP9 decode context (VPU_DESIGN.md 7.6/7.7; VA2-survey (C),
 * "VP9 -- pure passthrough"). VA1's H.264 path re-synthesises Annex-B from
 * separate VA parameter buffers; VP9 needs none of that -- the VASliceDataBuffer
 * already holds the complete in-band VP9 frame bitstream, so the access unit
 * fed to the stateful V4L2_PIX_FMT_VP9 decoder is that frame forwarded verbatim
 * (VP9AccessUnitBuilder). Everything below the access unit -- the session, the
 * NV12 CAPTURE buffers, the sequence/timestamp mapping, the VA3 GBM zero-copy
 * export -- is codec-agnostic and mirrors StatefulH264Context.
 *
 * VA2a keeps this as a deliberate, pragmatic sibling of StatefulH264Context
 * rather than refactoring a shared base out of it: the codec-agnostic wrappers
 * (sync/release/export/frame_view) are thin and are duplicated here so the
 * H.264 path stays bit-identical and untouched (a base-class extraction is a
 * clean follow-up). The two share only the Context virtual seam and the shared
 * stateful::FrameView.
 */
class StatefulVP9Context final : public Context {
public:
    /* Advertise VAProfileVP9Profile0 when the device is a stateful decoder that
     * advertises V4L2_PIX_FMT_VP9 ('VP90'). The device/driver are already
     * codec-agnostic (VA2-survey (A)/(D)): /dev/video0 lists VP90 and the
     * c2.qti.vp9.decoder is hardware, announcing at 1 access unit. Profile2
     * (10/12-bit) is a follow-up: the device advertises only the VP90 fourcc
     * with no bit-depth caps to gate it, and CAPTURE is 8-bit NV12. */
    static std::set<VAProfile> supported_profiles(const V4L2M2MDevice& device);

    StatefulVP9Context(DriverData* driver_data, V4L2M2MDevice& device, VAProfile profile, int picture_width,
        int picture_height, std::span<VASurfaceID> surface_ids);
    ~StatefulVP9Context() override;

    VAStatus store_buffer(const Buffer& buffer) const override;
    int set_controls() override { return VA_STATUS_SUCCESS; }
    bool is_stateful() const override { return true; }
    void stateful_begin_picture(Surface& surface) override;
    VAStatus stateful_end_picture(VADriverContextP va_context, Surface& surface) override;

    /* vaSyncSurface (7.6 points 4-6). */
    VAStatus sync_surface(VADriverContextP va_context, Surface& surface) override;
    /* vaDestroySurfaces / surface teardown: recycle the CAPTURE claim. */
    void release_surface(Surface& surface) override;

    /* vaExportSurfaceHandle (7.7 point 3): gbm-dmabuf zero-copy or, in MMAP
     * mode, VA_STATUS_ERROR_UNIMPLEMENTED (the browser falls back to software).
     * Identical semantics to the H.264 path -- the CAPTURE buffers are NV12. */
    VAStatus export_surface(VADriverContextP va_context, Surface& surface, uint32_t flags, void* descriptor) override;

    /* A claimed frame's NV12 planes for the image path (copy or derive). */
    std::optional<stateful::FrameView> frame_view(const Surface& surface);

    /* vaDeriveImage OPERATION_FAILED explanation, logged once per context. */
    void note_derive_unavailable(VADriverContextP va_context, bool have_view) override;

    /* Serialise a reader of decoded frame memory against CAPTURE
     * re-provisioning (D83); do not take the driver-wide mutex while held. */
    std::unique_lock<std::mutex> hold_frames() override { return session_.hold(); }

private:
    VAProfile profile_;
    stateful::V4L2StatefulDevice device_io_;
    /* Declared before session_ so it outlives it (7.7); usable() is false on
     * r22/no-render-node and the session then falls back to MMAP. */
    stateful::GbmAllocator allocator_;
    stateful::StatefulSession session_;
    /* D83: guards au_builder_ and next_sequence_ against concurrent
     * vaRenderPicture/vaEndPicture calls; never held across session waits. */
    mutable std::mutex builder_mutex_;
    mutable stateful::VP9AccessUnitBuilder au_builder_;
    uint64_t next_sequence_ = 1;
    /* 7.7 (4): the derive-unavailable reason is logged once per context. */
    std::atomic<bool> derive_unavailable_logged_ { false };
};
