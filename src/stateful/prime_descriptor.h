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

#include <cstdint>
#include <cstring>

extern "C" {
#include <libdrm/drm_fourcc.h>
#include <va/va.h>
#include <va/va_drmcommon.h>
}

namespace stateful {

/*
 * The VA3 zero-copy export helpers (VPU_DESIGN.md 7.7 point 3), split out as
 * pure functions so the descriptor layout and the pre-sync gate are testable
 * on the host without a real decode context (the context binds a concrete
 * V4L2 node; only the guest B21 run exercises the real vtable call).
 */

/* What vaExportSurfaceHandle must return before it fills a descriptor. */
enum class ExportGate {
    ready, /* a synced, current-generation gbm surface: fill the descriptor */
    busy, /* not yet decoded into (VA_STATUS_ERROR_SURFACE_BUSY) */
    unimplemented, /* MMAP mode: no GPU-importable buffer (VA1 behaviour) */
    dead, /* the session died (VA_STATUS_ERROR_DECODING_ERROR) */
};

/*
 * Decide the gate from already-resolved facts (after any sync):
 *   session_dead        -- the decode session is dead;
 *   has_decoded_frame   -- the surface holds a claimed CAPTURE frame
 *                          (stateful_capture_index >= 0);
 *   gbm_mode            -- CAPTURE was provisioned as gbm-dmabuf;
 *   generation_current  -- the surface's binding is the live provisioning
 *                          generation (not stale after a re-provision, D83).
 * A surface with no decoded frame is BUSY (the 7.7 "before sync" case); a gbm
 * surface whose binding went stale is also BUSY; MMAP mode is UNIMPLEMENTED so
 * the browser falls back to software as in VA1.
 */
inline ExportGate export_gate(bool session_dead, bool has_decoded_frame, bool gbm_mode, bool generation_current)
{
    if (session_dead) {
        return ExportGate::dead;
    }
    if (!has_decoded_frame) {
        return ExportGate::busy;
    }
    if (!gbm_mode) {
        return ExportGate::unimplemented;
    }
    if (!generation_current) {
        return ExportGate::busy;
    }
    return ExportGate::ready;
}

/*
 * Fill an NV12/LINEAR descriptor for a single dma-buf object (7.7 point 3).
 * Composed layers (Firefox's GetVAAPISurfaceDescriptor, B19): one NV12 layer,
 * two planes in the one object -- luma at offset 0, chroma at stride*height.
 * Separate layers (VA_EXPORT_SURFACE_SEPARATE_LAYERS; GStreamer accepts
 * either): an R8 luma layer and a GR88 chroma layer. READ_ONLY/WRITE are
 * honoured trivially -- the fd is a full R/W dma-buf and the caller uses it
 * per its own flags. The caller owns and dups the fd.
 */
inline void fill_nv12_prime_descriptor(VADRMPRIMESurfaceDescriptor* desc, int fd, uint32_t width, uint32_t height,
    uint32_t stride, uint32_t size, uint32_t export_flags)
{
    memset(desc, 0, sizeof(*desc));
    const uint32_t luma = stride * height;

    desc->fourcc = VA_FOURCC_NV12;
    desc->width = width;
    desc->height = height;
    desc->num_objects = 1;
    desc->objects[0].fd = fd;
    desc->objects[0].size = size;
    desc->objects[0].drm_format_modifier = DRM_FORMAT_MOD_LINEAR;

    if (export_flags & VA_EXPORT_SURFACE_SEPARATE_LAYERS) {
        desc->num_layers = 2;
        desc->layers[0].drm_format = DRM_FORMAT_R8;
        desc->layers[0].num_planes = 1;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].pitch[0] = stride;
        desc->layers[1].drm_format = DRM_FORMAT_GR88;
        desc->layers[1].num_planes = 1;
        desc->layers[1].object_index[0] = 0;
        desc->layers[1].offset[0] = luma;
        desc->layers[1].pitch[0] = stride;
    } else {
        desc->num_layers = 1;
        desc->layers[0].drm_format = DRM_FORMAT_NV12;
        desc->layers[0].num_planes = 2;
        desc->layers[0].object_index[0] = 0;
        desc->layers[0].offset[0] = 0;
        desc->layers[0].pitch[0] = stride;
        desc->layers[0].object_index[1] = 0;
        desc->layers[0].offset[1] = luma;
        desc->layers[0].pitch[1] = stride;
    }
}

} // namespace stateful
