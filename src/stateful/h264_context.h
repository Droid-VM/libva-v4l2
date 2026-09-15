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

#include <mutex>
#include <optional>
#include <set>
#include <span>

extern "C" {
#include <va/va_backend.h>
}

#include "../context.h"
#include "../surface.h"
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
     * The VA1 profile hardcode (7.6 point 2): the virtio-media decoder
     * exposes no profile/level controls (its only control is the read-only
     * MIN_BUFFERS_FOR_CAPTURE), so a stateful node advertising
     * V4L2_PIX_FMT_H264 gets this fixed list. VA1b replaces this table by
     * reading the V4L2_CID_MPEG_VIDEO_H264_PROFILE menu once the device
     * exposes MediaCodec's real capabilities.
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

    /* Serialise a reader of decoded frame memory (vaGetImage/vaDeriveImage)
     * against CAPTURE re-provisioning (D83). Do not take the driver-wide
     * mutex while holding this (lock order: driver mutex, then session). */
    std::unique_lock<std::mutex> hold_frames() { return session_.hold(); }

private:
    VAProfile profile_;
    stateful::V4L2StatefulDevice device_io_;
    stateful::StatefulSession session_;
    /* D83: guards au_builder_ and next_sequence_ against concurrent
     * vaRenderPicture/vaEndPicture calls; never held across session waits. */
    mutable std::mutex builder_mutex_;
    mutable stateful::H264AccessUnitBuilder au_builder_;
    uint64_t next_sequence_ = 1;
};
