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

    /* The logger the allocator falls back to when the caller passes none: it
     * must never be an empty std::function. D89 (B21-accept §4): the shipped
     * r385 constructor called an empty callback -- std::bad_function_call out
     * of every vaCreateContext -- because it invoked the moved-from ctor
     * parameter instead of the guarded member, and no test constructed a real
     * GbmAllocator the way StatefulH264Context does. */
    void default_stderr_log(const char* message)
    {
        fprintf(stderr, "libva-v4l2 stateful: %s\n", message);
    }

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

/* The ctor parameter is named `logger`, never `log`: `log(...)` in the body
 * MUST bind to the guarded member function GbmAllocator::log, not to a
 * (moved-from, empty) std::function parameter. `log_` is also defaulted to a
 * stderr writer so it is never empty even if the member guard were dropped
 * (D89, belt and suspenders). */
GbmAllocator::GbmAllocator(int drm_fd, std::function<void(const char*)> logger)
    : log_(logger ? std::move(logger) : std::function<void(const char*)>(default_stderr_log))
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
            log_sandbox_wall();
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

/*
 * P-4 layer 3 (E2E-vpu.md 11.1). One failed gbm_bo_create inside Firefox's RDD
 * (media) process cost a whole WP to explain, so explain it here instead.
 *
 * The chain: Firefox's seccomp policy answers sysinfo(2) with EPERM in every
 * child but the content one (SandboxPolicyCommon,
 * security/sandbox/linux/SandboxFilter.cpp); glibc's sysconf(_SC_PHYS_PAGES)
 * IS that syscall; Mesa's os_get_total_physical_memory() is that sysconf; and
 * zink_screen.c refuses to create a screen when it fails. GBM then falls back
 * to kms_swrast, whose winsys allocates with DRM_IOCTL_MODE_CREATE_DUMB -- an
 * ioctl a DRM RENDER node rejects with EACCES by construction, because it is
 * not DRM_RENDER_ALLOW. Hence "KMS: DRM_IOCTL_MODE_CREATE_DUMB failed:
 * Permission denied" from a process that was handed a perfectly good render
 * node.
 *
 * The test is a hint, not a proof: glibc does not check that sysinfo failed,
 * so it returns uninitialised stack, which usually but not always reads as an
 * error. A false negative costs nothing -- the MMAP fallback line above is
 * printed either way.
 */
void GbmAllocator::log_sandbox_wall()
{
    if (sysconf(_SC_PHYS_PAGES) > 0) {
        return;
    }
    log("gbm: this process cannot read the system memory size (sysconf(_SC_PHYS_PAGES) failed), so it is "
        "sandboxed -- Firefox answers sysinfo(2) with EPERM outside the content process");
    log("gbm: Mesa's zink screen refuses to start without that number and GBM degrades to kms_swrast, which "
        "allocates with DRM_IOCTL_MODE_CREATE_DUMB -- an ioctl a DRM render node always refuses (EACCES)");
    log("gbm: so zero-copy cannot work in this process until the guest Mesa carries the zink fix. Decoding "
        "still works; vaExportSurfaceHandle will answer UNIMPLEMENTED and a browser falls back to software");
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
