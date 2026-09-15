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

#include <functional>
#include <memory>

#include "allocator.h"

struct gbm_device;

namespace stateful {

/*
 * The real SurfaceAllocator: GBM on the VA display's DRM render node
 * (VPU_DESIGN.md 7.7, corrected by the VA3 spike). DroidVM's Mesa turns a GBM
 * allocation into a MEM_GUEST virtio-gpu blob the native context can bind, so
 * these buffers -- unlike a raw RESOURCE_CREATE_BLOB -- are the only guest-
 * side allocation the GPU leg samples correctly.
 */
class GbmAllocator final : public SurfaceAllocator {
public:
    /* drm_fd is the VA display's DRM fd (borrowed, from the driver's
     * drm_state); if < 0 the allocator opens /dev/dri/renderD128 itself and
     * owns that fd. On any failure usable() stays false and the session falls
     * back to VA1 MMAP. `logger` may be empty -- the ctor then installs a
     * stderr writer so log_ is never an empty std::function (D89). */
    explicit GbmAllocator(int drm_fd, std::function<void(const char*)> logger = {});
    ~GbmAllocator() override;

    GbmAllocator(const GbmAllocator&) = delete;
    GbmAllocator& operator=(const GbmAllocator&) = delete;

    bool usable() const override { return device_ != nullptr; }
    bool supports_nv12_linear() const override { return nv12_linear_; }
    std::unique_ptr<SurfaceBuffer> allocate(uint32_t width, uint32_t height, uint32_t& stride_out) override;
    const char* name() const override { return name_; }

private:
    void log(const char* message);

    std::function<void(const char*)> log_;
    gbm_device* device_ = nullptr;
    int owned_fd_ = -1; /* closed in the dtor when we opened the node ourselves */
    bool nv12_linear_ = false;
    const char* name_ = "gbm";
};

} // namespace stateful
