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
#include <memory>
#include <vector>

extern "C" {
#include <sys/mman.h>
#include <unistd.h>
}

#include "../../src/stateful/allocator.h"

/*
 * A memory-backed fake of the GBM SurfaceAllocator seam (VPU_DESIGN.md 7.7):
 * the build host has a GBM (Mesa) but no render node and no virtio-gpu, so the
 * gbm-dmabuf provisioning path is driven with plain memfd-backed buffers here.
 * A real fd (memfd) is handed out so QBUF has a genuine number and export_fd()
 * dups an openable descriptor the test can close.
 */
class FakeSurfaceBuffer final : public stateful::SurfaceBuffer {
public:
    FakeSurfaceBuffer(uint32_t stride, std::size_t size)
        : stride_(stride)
        , storage_(size, 0)
    {
        fd_ = memfd_create("fake-gbm-bo", 0);
        if (fd_ >= 0) {
            (void)!ftruncate(fd_, static_cast<off_t>(size));
        }
    }

    ~FakeSurfaceBuffer() override
    {
        if (fd_ >= 0) {
            close(fd_);
        }
    }

    int fd() const override { return fd_; }
    int export_fd() const override { return fd_ >= 0 ? dup(fd_) : -1; }
    uint32_t stride() const override { return stride_; }
    std::size_t size() const override { return storage_.size(); }
    std::span<uint8_t> map() override { return { storage_.data(), storage_.size() }; }

private:
    int fd_ = -1;
    uint32_t stride_;
    std::vector<uint8_t> storage_;
};

class FakeAllocator final : public stateful::SurfaceAllocator {
public:
    /* --- scripting knobs --- */
    bool is_usable = true;
    bool nv12_linear = false;
    uint32_t stride = 1920; /* what allocate() reports; set != device bytesperline to force S_FMT */
    int fail_after = -1; /* >= 0: the Nth allocate() (0-based) fails, modelling a mid-pool failure */

    /* --- observable --- */
    unsigned allocations = 0;

    bool usable() const override { return is_usable; }
    bool supports_nv12_linear() const override { return nv12_linear; }
    const char* name() const override { return "fake"; }

    std::unique_ptr<stateful::SurfaceBuffer> allocate(uint32_t width, uint32_t height, uint32_t& stride_out) override
    {
        if (fail_after >= 0 && static_cast<int>(allocations) >= fail_after) {
            allocations += 1;
            return nullptr;
        }
        allocations += 1;
        stride_out = stride;
        std::size_t size = static_cast<std::size_t>(stride) * (height + height / 2);
        return std::make_unique<FakeSurfaceBuffer>(stride, size);
    }
};
