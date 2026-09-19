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
#include "gbm_allocator.h"
#include "h264_bitstream.h"
#include "session.h"
#include "v4l2_device.h"

struct DriverData;

/*
 * The stateful H.264 decode context (VPU_DESIGN.md 7.6): VA parameter buffers
 * are synthesised back into Annex-B access units and fed to a stateful V4L2
 * M2M decoder; surfaces are logical slots mapped to decoded CAPTURE buffers
 * by the 64-bit sequence carried in the buffer timestamp.
 */
class StatefulH264Context final : public Context {
public:
    /*
     * VA1b (7.6 point 2): ConstrainedBaseline/Main/High is what this bridge
     * IMPLEMENTS -- the three profiles h264_bitstream.cc can synthesise SPS/PPS
     * for -- intersected with the device's V4L2_CID_MPEG_VIDEO_H264_PROFILE
     * menu, which is MediaCodec's real capability. The VA1 hardcode was this
     * list with no second half, because the decoder device then had no profile
     * control at all (its only control was the read-only
     * MIN_BUFFERS_FOR_CAPTURE); a device that still has none is handled by
     * profile_menu::supported_profiles(), which keeps the implemented set.
     */
    static std::set<VAProfile> supported_profiles(const V4L2M2MDevice& device);

    StatefulH264Context(DriverData* driver_data, V4L2M2MDevice& device, VAProfile profile, int picture_width,
        int picture_height, std::span<VASurfaceID> surface_ids);
    ~StatefulH264Context() override;

    VAStatus store_buffer(const Buffer& buffer) const override;
    int set_controls() override { return VA_STATUS_SUCCESS; }
    bool is_stateful() const override { return true; }
    void stateful_begin_picture(Surface& surface) override;
    VAStatus stateful_end_picture(VADriverContextP va_context, Surface& surface) override;

    /* vaSyncSurface (7.6 points 4-6). */
    VAStatus sync_surface(VADriverContextP va_context, Surface& surface);
    /* vaDestroySurfaces / surface teardown: recycle the CAPTURE claim. */
    void release_surface(Surface& surface);

    /* vaExportSurfaceHandle (7.7 point 3): in gbm-dmabuf mode fill a
     * VADRMPRIMESurfaceDescriptor with the surface's GBM dma-buf (NV12/LINEAR,
     * composed or separate layers); in MMAP mode there is no zero-copy buffer
     * so return VA_STATUS_ERROR_UNIMPLEMENTED (the browser falls back to
     * software, as VA1 did). A not-yet-synced surface returns SURFACE_BUSY. */
    VAStatus export_surface(VADriverContextP va_context, Surface& surface, uint32_t flags, void* descriptor);

    /* A claimed frame's NV12 planes for the image path (copy or derive). */
    struct FrameView {
        std::span<uint8_t> luma;
        std::span<uint8_t> chroma;
        unsigned pitch;
        unsigned coded_width;
        unsigned coded_height;
        /* Non-empty when both planes live in one contiguous mapping
         * (single-plane NV12): the whole mapping, for vaDeriveImage. */
        std::span<uint8_t> contiguous;
    };
    std::optional<FrameView> frame_view(const Surface& surface);

    /* vaDeriveImage returned VA_STATUS_ERROR_OPERATION_FAILED: explain why,
     * exactly once per context (7.7 (4)). GStreamer's va plugin probes derive
     * at NEGOTIATION time, before any decode, so the surface is not yet bound
     * to a decoded frame and derive cannot map one; the plugin then negotiates
     * system memory and copies through vaGetImage (B19: 300/300). have_view is
     * false when no frame is bound (the negotiation case), true when a frame is
     * bound but is not a single contiguous NV12 plane. */
    void note_derive_unavailable(VADriverContextP va_context, bool have_view);

    /* Serialise a reader of decoded frame memory (vaGetImage/vaDeriveImage)
     * against CAPTURE re-provisioning (D83). Do not take the driver-wide
     * mutex while holding this (lock order: driver mutex, then session). */
    std::unique_lock<std::mutex> hold_frames() { return session_.hold(); }

private:
    VAProfile profile_;
    stateful::V4L2StatefulDevice device_io_;
    /* Declared before session_ so it outlives it: the GBM surface allocator
     * (7.7) the session provisions CAPTURE from in gbm-dmabuf mode. Always
     * constructed; usable() is false on r22/no-render-node, and the session
     * then falls back to MMAP. */
    stateful::GbmAllocator allocator_;
    stateful::StatefulSession session_;
    /* D83: guards au_builder_ and next_sequence_ against concurrent
     * vaRenderPicture/vaEndPicture calls; never held across session waits. */
    mutable std::mutex builder_mutex_;
    mutable stateful::H264AccessUnitBuilder au_builder_;
    uint64_t next_sequence_ = 1;
    /* 7.7 (4): the derive-unavailable reason is logged once per context. */
    std::atomic<bool> derive_unavailable_logged_ { false };
};
