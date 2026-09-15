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

#include "gbm_allocator.h"

#include <cstdio>
#include <cstring>

extern "C" {
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <gbm.h>
}

namespace stateful {

namespace {

    constexpr const char* kRenderNode = "/dev/dri/renderD128";

} // namespace

/*
 * One GBM buffer object: the R8 (or, if a future Mesa allows it, NV12)
 * container. The primary fd is exported once at construction and lives with
 * the object; export_fd() dups it for a VA client, and map() keeps a single
 * stable CPU view of the whole container so vaDeriveImage can hand the
 * pointer out (the spike confirmed a plain mmap of the dma-buf fd is a
 * coherent linear view, no DMA_BUF_IOCTL_SYNC needed).
 */
class GbmSurfaceBuffer final : public SurfaceBuffer {
public:
    GbmSurfaceBuffer(gbm_bo* bo, int fd, uint32_t stride, std::size_t size)
        : bo_(bo)
        , fd_(fd)
        , stride_(stride)
        , size_(size)
    {
    }

    ~GbmSurfaceBuffer() override
    {
        if (map_ != nullptr && map_ != MAP_FAILED) {
            munmap(map_, size_);
        }
        if (fd_ >= 0) {
            close(fd_);
        }
        if (bo_ != nullptr) {
            gbm_bo_destroy(bo_);
        }
    }

    int fd() const override { return fd_; }

    int export_fd() const override { return fd_ >= 0 ? fcntl(fd_, F_DUPFD_CLOEXEC, 0) : -1; }

    uint32_t stride() const override { return stride_; }
    std::size_t size() const override { return size_; }

    std::span<uint8_t> map() override
    {
        if (map_ == nullptr) {
            map_ = mmap(nullptr, size_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
        }
        if (map_ == MAP_FAILED) {
            return {};
        }
        return { static_cast<uint8_t*>(map_), size_ };
    }

private:
    gbm_bo* bo_;
    int fd_;
    uint32_t stride_;
    std::size_t size_;
    void* map_ = nullptr;
};

GbmAllocator::GbmAllocator(int drm_fd, std::function<void(const char*)> log)
    : log_(std::move(log))
{
    int fd = drm_fd;
    if (fd < 0) {
        owned_fd_ = open(kRenderNode, O_RDWR | O_CLOEXEC);
        if (owned_fd_ < 0) {
            log("gbm: the VA display carries no DRM fd and /dev/dri/renderD128 will not open -- MMAP fallback");
            return;
        }
        log("gbm: the VA display carried no DRM fd; opened /dev/dri/renderD128 for allocation");
        fd = owned_fd_;
    }

    device_ = gbm_create_device(fd);
    if (device_ == nullptr) {
        log("gbm: gbm_create_device failed (render node is not usable) -- MMAP fallback");
        if (owned_fd_ >= 0) {
            close(owned_fd_);
            owned_fd_ = -1;
        }
        return;
    }

    nv12_linear_ = gbm_device_is_format_supported(device_, GBM_FORMAT_NV12, GBM_BO_USE_LINEAR) != 0;
    char line[128];
    snprintf(line, sizeof(line), "gbm: NV12/LINEAR %s (%s)", nv12_linear_ ? "supported" : "unsupported",
        nv12_linear_ ? "allocating NV12 directly" : "using the R8 byte-container");
    log(line);
}

GbmAllocator::~GbmAllocator()
{
    if (device_ != nullptr) {
        gbm_device_destroy(device_);
    }
    if (owned_fd_ >= 0) {
        close(owned_fd_);
    }
}

std::unique_ptr<SurfaceBuffer> GbmAllocator::allocate(uint32_t width, uint32_t height, uint32_t& stride_out)
{
    if (device_ == nullptr) {
        return nullptr;
    }

    gbm_bo* bo = nullptr;
    if (nv12_linear_) {
        bo = gbm_bo_create(device_, width, height, GBM_FORMAT_NV12, GBM_BO_USE_LINEAR);
    }
    if (bo == nullptr) {
        /* The spike's exact primary shape: an R8 byte-container of the whole
         * NV12 payload, width x (height + height/2), linear. */
        bo = gbm_bo_create(device_, width, height + height / 2, GBM_FORMAT_R8, GBM_BO_USE_LINEAR);
        if (bo == nullptr) {
            log("gbm: gbm_bo_create(R8 container) failed -- MMAP fallback");
            return nullptr;
        }
    }

    uint32_t stride = gbm_bo_get_stride(bo);
    int fd = gbm_bo_get_fd(bo);
    if (fd < 0) {
        log("gbm: gbm_bo_get_fd failed -- MMAP fallback");
        gbm_bo_destroy(bo);
        return nullptr;
    }

    off_t size = lseek(fd, 0, SEEK_END);
    std::size_t container
        = size > 0 ? static_cast<std::size_t>(size) : static_cast<std::size_t>(stride) * (height + height / 2);

    stride_out = stride;
    return std::make_unique<GbmSurfaceBuffer>(bo, fd, stride, container);
}

void GbmAllocator::log(const char* message)
{
    if (log_) {
        log_(message);
    } else {
        fprintf(stderr, "libva-v4l2 stateful: %s\n", message);
    }
}

} // namespace stateful
