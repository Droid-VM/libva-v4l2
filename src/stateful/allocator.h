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

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <span>

namespace stateful {

/*
 * The GPU-buffer seam for the zero-copy (VA3) path (VPU_DESIGN.md 7.7).
 *
 * A stateful surface is one linear buffer the GPU can import: the VA3 spike
 * (logs/vpu_wp/VA3-spike.md) proved that on this stack the buffer must come
 * from GBM (a Mesa guest-alloc virtio-gpu blob the native context can bind);
 * a bare RESOURCE_CREATE_BLOB imports without error but samples all-zeros, so
 * that rung is dead. GBM also refuses GBM_FORMAT_NV12 here, so the primary
 * shape is a GBM_FORMAT_R8 byte-container of width x (height * 3/2), whose
 * stride came back byte-for-byte equal to the decoder's G_FMT bytesperline
 * and which EGL imports as two-plane NV12 (offsets 0 / stride*height).
 *
 * This interface hides GBM from the session so the host tests can drive the
 * gbm-dmabuf provisioning path with a plain memory-backed fake -- the build
 * host has a GBM (Mesa) but no render node, and no virtio-gpu at all.
 */

/* One allocated container buffer: the surface's dma-buf for its whole life
 * (owned until the SurfaceBuffer is destroyed at vaDestroySurfaces). */
class SurfaceBuffer {
public:
    virtual ~SurfaceBuffer() = default;

    /* The primary dma-buf fd, borrowed: it is what QBUF(memory=DMABUF) uses
     * and stays owned by this buffer. */
    virtual int fd() const = 0;
    /* A fresh dup of the primary fd for a VA client (vaExportSurfaceHandle);
     * the caller owns and closes it. -1 on failure. */
    virtual int export_fd() const = 0;

    virtual uint32_t stride() const = 0; /* luma/byte stride */
    virtual std::size_t size() const = 0; /* total container bytes */

    /* A CPU view of the whole container (NV12 laid out inside), stable for
     * this buffer's lifetime so vaDeriveImage can hand the pointer out. Empty
     * span on failure. */
    virtual std::span<uint8_t> map() = 0;
};

class SurfaceAllocator {
public:
    virtual ~SurfaceAllocator() = default;

    /* True once a device/context was opened: a gbm_create_device that failed
     * (no render node, not virtio_gpu) forces the VA1 MMAP fallback. */
    virtual bool usable() const = 0;

    /* Probed once: today GBM refuses NV12 on this stack so the R8 container is
     * used; a future Mesa that says yes lets allocate() hand back NV12
     * directly through the same code path. */
    virtual bool supports_nv12_linear() const = 0;

    /* Allocate one linear NV12 container sized for a width x height (luma)
     * frame; sets stride_out to the byte stride the allocator chose. Returns
     * nullptr on failure. */
    virtual std::unique_ptr<SurfaceBuffer> allocate(uint32_t width, uint32_t height, uint32_t& stride_out) = 0;

    /* For the one-line mode log. */
    virtual const char* name() const = 0;
};

} // namespace stateful
