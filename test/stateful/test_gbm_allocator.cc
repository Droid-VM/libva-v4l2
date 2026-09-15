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

/*
 * D89 (B21-accept §4): the shipped r385 GbmAllocator constructor invoked an
 * empty std::function (it called the moved-from ctor PARAMETER `log`, which
 * shadowed the guarded member, instead of the member itself), so every
 * vaCreateContext threw std::bad_function_call. The seven pre-existing tests
 * drove the gbm-dmabuf provisioning through the FakeAllocator seam and the one
 * that touched a real allocator passed a logger, so none constructed a real
 * GbmAllocator the way StatefulH264Context does -- with the DEFAULT (empty)
 * logger. This test closes that gap:
 *
 *   1. it constructs a real GbmAllocator with the EXACT signature and
 *      arguments the context uses (GbmAllocator(fd), default logger) and
 *      requires it not to throw -- this is the direct D89 regression;
 *   2. it drives a StatefulSession with a real GbmAllocator against the
 *      scripted device seam ("in gbm mode"), the closest host analogue of the
 *      vaCreateConfig/vaCreateSurfaces/vaCreateContext + decode the guest runs,
 *      so a crash at the first real allocator call (below-seam tests missed)
 *      is caught here.
 *
 * Named mutation this fails under: restore the ctor to call the moved-from
 * parameter `log(...)` (or drop the `?:` default so log_ is empty and the
 * member guard is the only defence, then also drop the member guard) -->
 * std::bad_function_call terminates the process, so no "all checks passed".
 *
 * The build host has a Mesa GBM but usually no render node, so a real
 * GbmAllocator is typically unusable() -> the session falls back to MMAP; the
 * point is that CONSTRUCTION and the usable()/allocate() probe never crash, in
 * either outcome. The pixel-exact GPU import stays a guest bar.
 */

#include <string>
#include <vector>

extern "C" {
#include <va/va.h>
}

#include "../../src/stateful/gbm_allocator.h"
#include "../../src/stateful/session.h"
#include "check.h"
#include "fake_device.h"

using stateful::GbmAllocator;
using stateful::StatefulSession;

namespace {

std::vector<uint8_t> fake_au(size_t size = 512)
{
    return std::vector<uint8_t>(size, 0xab);
}

/* 1. Construct exactly as StatefulH264Context does: one argument, the default
 * (empty) logger. This is the line the shipped r385 crashed on. */
void test_default_logger_construction_does_not_throw()
{
    /* drm_fd < 0: the ctor tries /dev/dri/renderD128 and, whatever happens
     * (opened, refused, or gbm_create_device failing), logs the outcome --
     * the exact call site of D89. It must return, not throw. */
    GbmAllocator allocator(-1);
    /* usable() is host-dependent (a render node may or may not exist); the
     * test only requires construction not to crash and the accessors to be
     * callable. */
    (void)allocator.usable();
    (void)allocator.supports_nv12_linear();
    (void)allocator.name();
    CHECK(true); /* reaching here means no bad_function_call was thrown */
}

/* 2. An explicit logger is still honoured (the production path since the D89
 * fix: the context passes a stderr logger), and at least one line is emitted
 * during construction. */
void test_explicit_logger_is_called()
{
    std::vector<std::string> lines;
    GbmAllocator allocator(-1, [&](const char* message) { lines.emplace_back(message); });
    (void)allocator.usable();
    /* The ctor always logs its outcome once (render-node open/refuse, or the
     * NV12/LINEAR probe result). */
    CHECK(!lines.empty());
}

/* 3. Drive the real allocator through a StatefulSession against the fake
 * device "in gbm mode" (SUPPORTS_DMABUF advertised, a usable-or-not real
 * allocator). This is the host stand-in for the guest's real
 * vaCreateContext + decode: it constructs the real GbmAllocator and runs it
 * through the session's decide_capture_mode / provisioning, then a decode.
 * Whether gbm engages depends on the host render node; either way it must not
 * crash and the decode must complete. */
void test_real_allocator_drives_session()
{
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;

    GbmAllocator allocator(-1); /* default logger, exactly as the context */

    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.output_ring_size = 4;
    options.sync_timeout_ms = 30;
    options.allocator = &allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    session.submit(1, fake_au());
    StatefulSession::Frame frame;
    CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);

    /* A host with no render node lands in MMAP; a host with a working
     * virtio_gpu-like node could land in gbm-dmabuf. Both are valid; the
     * invariant is that a real GbmAllocator neither crashed the ctor nor the
     * provisioning probe. */
    const auto mode = session.capture_mode();
    CHECK(mode == StatefulSession::CaptureMode::mmap || mode == StatefulSession::CaptureMode::gbm_dmabuf);
    if (!allocator.usable()) {
        CHECK(mode == StatefulSession::CaptureMode::mmap);
    }
}

} // namespace

int main()
{
    test_default_logger_construction_does_not_throw();
    test_explicit_logger_is_called();
    test_real_allocator_drives_session();
    return check_result("test_gbm_allocator");
}
