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

#include "av1_context.h"

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <vector>

extern "C" {
#include <linux/videodev2.h>

#include <va/va.h>
#include <va/va_dec_av1.h>
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
    /* D84: the pool is sized from the surfaces that actually exist when the
     * first SOURCE_CHANGE provisions it (modern clients pass none to
     * vaCreateContext). Atomic read under the session mutex; must not take the
     * driver-wide mutex. */
    options.surface_count = [driver_data] { return driver_data->surface_count.load(std::memory_order_relaxed); };
    /* 7.7: provision CAPTURE from GBM dma-bufs when the allocator is usable and
     * the device advertises SUPPORTS_DMABUF; the session decides and logs once. */
    options.allocator = allocator;
    /* NO mid-stream drain for AV1. The recover_locked DEC_CMD_STOP drain exists
     * for H.264, whose reorder tail the codec holds back until a drain shakes it
     * loose; this decoder holds nothing back on a correctly reconstructed AV1
     * stream. It emits a frame per access unit and keeps up with a client that
     * syncs every frame before feeding the next, deep-B pyramids included
     * (measured on the phone: a 166 s browser AV1 session, 5113 bitstream
     * buffers in / 5113 frames out, zero sync failures). So a mid-stream drain
     * would extract nothing and cost a seek on a live reference chain: keep it
     * off, fail the one stalled surface on a hard cap and keep the session alive
     * for the frames that follow. finish() still drains the genuine tail at EOS.
     *
     * Both earlier rationales for this line are WITHDRAWN: VA2e's "AV1 emits
     * every frame as shown, so there is no held tail", and VA3-sync-reorder's
     * correction of it, "there IS a decode-order tail and the drain drops it, so
     * deep-B AV1 cannot decode in a browser". Both were measured while this
     * backend rebuilt the bitstream incorrectly (heuristic refresh_frame_flags,
     * a deferred frame carrying another frame's tile_size_bytes, and a missing
     * lr_uv_shift bit). The decoder was failing on an invalid stream, not
     * holding frames; with 86ee648 / 95f7c81 / bf6d262 / aec13f4 the same
     * content decodes zero-copy end to end. The setting itself never changed. */
    options.allow_midstream_drain = false;
    return options;
}

} // namespace

std::set<VAProfile> StatefulAV1Context::supported_profiles(const V4L2M2MDevice& device)
{
    return (device.stateful_decoder() && device.format_supported(device.output_buf_type, V4L2_PIX_FMT_AV1))
        ? std::set<VAProfile> { VAProfileAV1Profile0 }
        : std::set<VAProfile>();
}

StatefulAV1Context::StatefulAV1Context(DriverData* driver_data, V4L2M2MDevice& device, VAProfile profile,
    int picture_width, int picture_height, std::span<VASurfaceID> surface_ids)
    : Context(driver_data, device, picture_width, picture_height)
    , profile_(profile)
    , device_io_(device)
    /* D89: pass an explicit, non-empty stderr logger so the gbm mode/fallback
     * lines stay visible under the rig's default log level (mirrors H.264/VP9). */
    , allocator_(
          driver_data->drm_fd, [](const char* message) { fprintf(stderr, "libva-v4l2 stateful: %s\n", message); })
    , session_(device_io_, V4L2_PIX_FMT_AV1, picture_width, picture_height,
          session_options(driver_data, surface_ids, &allocator_))
{
}

StatefulAV1Context::~StatefulAV1Context()
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
    /* Emit any frame still held for the lookahead so the session drains it
     * cleanly (best-effort: its surface is unbound above). */
    try {
        std::vector<stateful::Av1AccessUnitBuilder::ReadyFrame> tail;
        {
            std::lock_guard<std::mutex> guard(builder_mutex_);
            tail = au_builder_.flush();
        }
        for (auto& ready : tail) {
            session_.submit(ready.tag, ready.access_unit);
        }
    } catch (const std::exception&) {
        /* Teardown: a failed tail submit is harmless. */
    }
    session_.finish();
}

VAStatus StatefulAV1Context::store_buffer(const Buffer& buffer) const
{
    std::lock_guard<std::mutex> guard(builder_mutex_);

    switch (buffer.type) {
    case VAPictureParameterBufferType:
        au_builder_.set_picture_parameters(*reinterpret_cast<const VADecPictureParameterBufferAV1*>(buffer.data.get()));
        break;

    case VASliceParameterBufferType:
        /* VASliceParameterBufferAV1 is really a per-tile parameter buffer. */
        au_builder_.add_tile_parameters(
            { reinterpret_cast<const VASliceParameterBufferAV1*>(buffer.data.get()), buffer.count });
        break;

    case VASliceDataBufferType:
        try {
            au_builder_.add_tile_data({ buffer.data.get(), static_cast<size_t>(buffer.size) * buffer.count });
        } catch (const std::exception&) {
            return VA_STATUS_ERROR_INVALID_BUFFER;
        }
        break;

    default:
        return VA_STATUS_ERROR_UNSUPPORTED_BUFFERTYPE;
    }

    return VA_STATUS_SUCCESS;
}

void StatefulAV1Context::stateful_begin_picture(Surface& surface)
{
    /* 7.6 point 4: a surface reused by a later vaBeginPicture re-queues its
     * CAPTURE buffer. */
    if (surface.stateful_context == this && surface.stateful_capture_index >= 0) {
        session_.release_frame(
            { static_cast<unsigned>(surface.stateful_capture_index), surface.stateful_capture_generation });
        surface.stateful_capture_index = -1;
    } else if (surface.stateful_context == this && surface.status == VASurfaceRendering) {
        /* The client is reusing a surface it decoded into but never synced -- the
         * not-shown references of a random-access stream, which it keeps only as
         * references and drops without ever asking for the pixels. Nobody will
         * ever claim that sequence, so drop it: the CAPTURE buffer its frame
         * holds (or will hold when the codec emits it) goes straight back to the
         * codec instead of sitting in the stash until the pool starves. The
         * access unit itself is still submitted -- later frames reference that
         * picture, so the codec must decode it. status is still the old value
         * here; beginPicture sets Rendering after this returns. */
        session_.drop_sequence(surface.stateful_sequence);
    }
}

VAStatus StatefulAV1Context::stateful_end_picture(VADriverContextP va_context, Surface& surface)
{
    stateful::Av1AccessUnitBuilder::PictureResult result;
    uint64_t sequence;
    {
        std::lock_guard<std::mutex> guard(builder_mutex_);
        if (!au_builder_.has_picture()) {
            error_log(va_context, "vaEndPicture without AV1 frame data\n");
            return VA_STATUS_ERROR_INVALID_PARAMETER;
        }
        sequence = next_sequence_++;
        try {
            /* Resolve the one-frame lookahead. This frame's ref_frame_map
             * DERIVES the previously held frame's refresh_frame_flags exactly,
             * emitting it; this frame is then held in turn (a key frame is
             * exact on its own and goes out immediately). */
            result = au_builder_.submit_picture(sequence);
        } catch (const std::exception& e) {
            error_log(va_context, "Failed to assemble AV1 access unit: %s\n", e.what());
            return VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }

    /* Submit the temporal units that became ready, in decode order, outside the
     * builder lock (a submit can wait on the device; the driver-wide/session
     * lock order forbids holding it across that). */
    for (auto& ready : result.ready) {
        try {
            session_.submit(ready.tag, ready.access_unit);
        } catch (const std::exception& e) {
            error_log(va_context, "Failed to submit AV1 access unit: %s\n", e.what());
            return session_.dead() ? VA_STATUS_ERROR_DECODING_ERROR : VA_STATUS_ERROR_OPERATION_FAILED;
        }
    }

    if (result.current_failed) {
        error_log(va_context, "Failed to assemble AV1 access unit: segmentation is unsupported\n");
        render_surface_id = VA_INVALID_ID;
        return VA_STATUS_ERROR_OPERATION_FAILED;
    }

    /* Bind this surface to its sequence whether it was submitted now or is held
     * for the lookahead -- it is then submitted by the next vaEndPicture, or by
     * sync_surface below if the client waits on it first. */
    surface.stateful_context = this;
    surface.stateful_sequence = sequence;
    surface.stateful_capture_index = -1;

    render_surface_id = VA_INVALID_ID;
    return VA_STATUS_SUCCESS;
}

VAStatus StatefulAV1Context::sync_surface(VADriverContextP va_context, Surface& surface)
{
    if (session_.dead()) {
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    if (surface.status != VASurfaceRendering) {
        return VA_STATUS_SUCCESS;
    }

    /* The client is waiting on a frame still held for the one-frame lookahead --
     * the end-of-stream tail, or a low-delay client that syncs each frame before
     * decoding the next. No successor is coming in time, so its
     * refresh_frame_flags cannot be derived: emit it with the best-effort
     * (self-consistent) mask and submit it before we wait. */
    std::vector<stateful::Av1AccessUnitBuilder::ReadyFrame> tail;
    {
        std::lock_guard<std::mutex> guard(builder_mutex_);
        if (au_builder_.has_pending() && au_builder_.pending_tag() == surface.stateful_sequence) {
            tail = au_builder_.flush();
        }
    }
    for (auto& ready : tail) {
        try {
            session_.submit(ready.tag, ready.access_unit);
        } catch (const std::exception& e) {
            error_log(va_context, "Failed to submit deferred AV1 access unit: %s\n", e.what());
            return session_.dead() ? VA_STATUS_ERROR_DECODING_ERROR : VA_STATUS_ERROR_OPERATION_FAILED;
        }
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
        /* D88: a failed sync must leave the surface REUSABLE (unbind, free the
         * status) so the next vaBeginPicture on this pooled surface is not
         * rejected SURFACE_BUSY. */
        surface.stateful_capture_index = -1;
        surface.status = VASurfaceReady;
        error_log(va_context, "Stateful sync failed on sequence %llu\n",
            static_cast<unsigned long long>(surface.stateful_sequence));
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
}

void StatefulAV1Context::release_surface(Surface& surface)
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

std::optional<stateful::FrameView> StatefulAV1Context::frame_view(const Surface& surface)
{
    if (surface.stateful_capture_index < 0 || !session_.provisioned()
        || surface.stateful_capture_generation != session_.generation()) {
        return std::nullopt; /* stale binding after a re-provision (D83) */
    }

    const auto& format = session_.capture_format();
    const unsigned index = static_cast<unsigned>(surface.stateful_capture_index);
    stateful::FrameView view;
    view.pitch = format.bytesperline != 0 ? format.bytesperline : format.width;
    view.coded_width = format.width;
    view.coded_height = format.height;

    const size_t luma_size = static_cast<size_t>(view.pitch) * format.height;

    if (session_.capture_mode() == stateful::StatefulSession::CaptureMode::gbm_dmabuf) {
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

void StatefulAV1Context::note_derive_unavailable(VADriverContextP va_context, bool have_view)
{
    if (derive_unavailable_logged_.exchange(true)) {
        return;
    }
    const bool gbm = session_.capture_mode() == stateful::StatefulSession::CaptureMode::gbm_dmabuf;
    info_log(va_context,
        "vaDeriveImage: %s (%s mode) -- returning OPERATION_FAILED; the client copies via vaGetImage\n",
        stateful::derive_unavailable_message(have_view), gbm ? "gbm-dmabuf" : "mmap");
}

VAStatus StatefulAV1Context::export_surface(
    VADriverContextP va_context, Surface& surface, uint32_t flags, void* descriptor)
{
    if (session_.dead()) {
        return VA_STATUS_ERROR_DECODING_ERROR;
    }
    /* Sync first if the client exports before syncing (7.7 acceptance). */
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
        /* VA1 MMAP mode: no GPU-importable buffer (7.6 point 7). */
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
