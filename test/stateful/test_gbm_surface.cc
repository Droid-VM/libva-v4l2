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
 * VPU_DESIGN.md 7.7 host verification: the gbm-dmabuf CAPTURE path against the
 * scripted device seam and a memory-backed GBM allocator fake. Covers the mode
 * / fallback ladder, the DMABUF QBUF plane fields, the derive/get NV12 view,
 * the export descriptor layout for both layer modes, and the pre-sync gate.
 * The pixel-exact GPU import and the real vtable call are the guest B21 bars
 * (a real GBM/virtio-gpu is off-host); everything at or below the allocator +
 * device seam is here.
 */

#include <cstring>
#include <span>
#include <string>
#include <vector>

extern "C" {
#include <stdlib.h>

#include <va/va.h>
#include <va/va_drmcommon.h>
}

#include "../../src/stateful/prime_descriptor.h"
#include "../../src/stateful/session.h"
#include "check.h"
#include "fake_allocator.h"
#include "fake_device.h"

using stateful::StatefulSession;

namespace {

StatefulSession::Options gbm_options(FakeAllocator* allocator)
{
    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.output_ring_size = 4;
    options.sync_timeout_ms = 30;
    options.allocator = allocator;
    return options;
}

std::vector<uint8_t> fake_au(size_t size = 512)
{
    return std::vector<uint8_t>(size, 0xab);
}

/* Drive one AU through and return the claimed frame. */
StatefulSession::Frame decode_one(StatefulSession& session, uint64_t sequence)
{
    session.submit(sequence, fake_au());
    StatefulSession::Frame frame;
    CHECK(session.sync(sequence, &frame) == StatefulSession::SyncStatus::ok);
    return frame;
}

/* --- mode / fallback ladder (7.7 point 5) --- */

void test_no_dmabuf_cap_falls_back_to_mmap()
{
    /* r22 shape: the device masks SUPPORTS_DMABUF. Even with a usable
     * allocator the session must stay MMAP so the same .so keeps working.
     * Named mutation: drop the SUPPORTS_DMABUF check in
     * decide_capture_mode_locked -> this picks gbm and dmabuf_queues fills. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP; /* no DMABUF */
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK(!device.capture_dmabuf);
    CHECK(device.dmabuf_queues.empty());
    CHECK_EQ(allocator.allocations, 0u);
}

void test_unusable_allocator_falls_back_to_mmap()
{
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    FakeAllocator allocator;
    allocator.is_usable = false; /* no render node / not virtio_gpu */
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK(!device.capture_dmabuf);
}

void test_null_allocator_falls_back_to_mmap()
{
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    StatefulSession::Options options = gbm_options(nullptr);
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
}

void test_env_forces_mmap()
{
    setenv("LIBVA_V4L2_SURFACES", "mmap", 1);
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    unsetenv("LIBVA_V4L2_SURFACES");
}

void test_gbm_mode_when_capable()
{
    /* r23 shape: SUPPORTS_DMABUF advertised, usable allocator, stride matches
     * the device bytesperline -> gbm-dmabuf, no S_FMT. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    FakeAllocator allocator;
    allocator.stride = 1920; /* == scripted_format.bytesperline */
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    CHECK(device.capture_dmabuf);
    CHECK_EQ(device.set_capture_stride_calls, 0u); /* strides matched, no negotiation */
    /* count = max(surfaces 6, min 4) = 6 (7.7 point 2). */
    CHECK_EQ(device.capture_count, 6u);
    CHECK_EQ(allocator.allocations, 6u);
}

void test_stride_negotiated_then_gbm()
{
    /* bo stride differs; the device accepts S_FMT(CAPTURE) -> stays gbm. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.capture_stride_negotiable = true;
    FakeAllocator allocator;
    allocator.stride = 2048; /* != 1920 */
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    CHECK_EQ(device.set_capture_stride_calls, 1u);
}

void test_stride_refused_falls_back_to_mmap()
{
    /* bo stride differs and the device refuses S_FMT -> MMAP (7.7 point 1).
     * Named mutation: make provision_capture_gbm_locked ignore the granted !=
     * requested check -> it proceeds in gbm with a wrong stride. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.capture_stride_negotiable = false;
    FakeAllocator allocator;
    allocator.stride = 2048;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK_EQ(device.set_capture_stride_calls, 1u);
    CHECK(!device.capture_dmabuf);
    /* The device's MMAP pool was provisioned instead. */
    CHECK(device.capture_streaming);
    CHECK_EQ(device.capture_count, 10u); /* min 4 + share 6 */
}

void test_gbm_allocation_failure_falls_back()
{
    /* A mid-pool allocation failure unwinds cleanly to MMAP. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    FakeAllocator allocator;
    allocator.fail_after = 3; /* the 4th bo fails */
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK(!device.capture_dmabuf);
}

void test_dmabuf_qbuf_eio_falls_back_to_mmap_and_decodes()
{
    /* D90 runtime shape: SUPPORTS_DMABUF advertised, a usable allocator,
     * REQBUFS(CAPTURE,DMABUF) granted -- but the first DMABUF QBUF fails -EIO
     * (the protected guest's restricted DMA pool refuses the vram exporter).
     * The session must tear the DMABUF provisioning down and re-provision MMAP
     * WITHOUT losing the frame (B21: Epiphany 300 -> 0). Named mutation: let
     * the QBUF exception propagate out of provision_capture_gbm_locked (drop
     * the try/catch) -> decode_one's sync fails, no fallback, mode stays
     * gbm_dmabuf with a dead decode. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.eio_dmabuf_qbufs = 1; /* the first DMABUF QBUF is refused -EIO */
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    /* The decode still completes, in MMAP mode. */
    StatefulSession::Frame frame = decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK(!device.capture_dmabuf); /* the DMABUF pool was released */
    CHECK(device.capture_streaming); /* the MMAP pool is streaming */
    CHECK_EQ(device.capture_count, 10u); /* min 4 + share 6, the MMAP pool */
    /* A bo-backed buffer is not handed out in MMAP mode. */
    auto guard = session.hold();
    CHECK(session.capture_buffer(frame.index) == nullptr);
    guard.unlock();
    session.release_frame(frame);
}

void test_dmabuf_qbuf_einval_also_falls_back()
{
    /* The ladder covers EFAULT/EINVAL as well as EIO (7.7 (3)). */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.eio_dmabuf_qbufs = 1;
    device.dmabuf_qbuf_errno = EINVAL;
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK(!device.capture_dmabuf);
}

void test_spares_when_min_exceeds_surfaces()
{
    /* Fewer surfaces than the codec minimum: the pool is the minimum, the
     * extra bos are internal spares (7.7 point 2). */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.scripted_min_buffers = 10;
    FakeAllocator allocator;
    StatefulSession::Options options = gbm_options(&allocator);
    options.num_surfaces = 2;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    CHECK_EQ(device.capture_count, 10u); /* max(2, 10) */
    CHECK_EQ(allocator.allocations, 10u);
}

/* --- DMABUF QBUF plane fields (7.7 point 2/5b) --- */

void test_dmabuf_qbuf_single_plane()
{
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.scripted_format.num_planes = 1;
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    REQUIRE(!device.dmabuf_queues.empty());
    const auto& q = device.dmabuf_queues.front();
    CHECK_EQ(q.planes.size(), 1u);
    auto guard = session.hold();
    stateful::SurfaceBuffer* bo = session.capture_buffer(q.index);
    guard.unlock();
    REQUIRE(bo != nullptr);
    CHECK_EQ(q.planes[0].fd, bo->fd());
    CHECK_EQ(q.planes[0].data_offset, 0u);
    CHECK_EQ(q.planes[0].length, static_cast<uint32_t>(bo->size()));
}

void test_dmabuf_qbuf_two_plane_offsets()
{
    /* num_planes == 2: the same fd backs both planes, chroma at
     * data_offset stride*height (7.7 point 5b). Named mutation: use a fresh
     * fd or offset 0 for plane 1 -> these offsets are wrong. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.scripted_format.num_planes = 2;
    device.scripted_format.bytesperline = 1920;
    device.scripted_format.height = 1088;
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    REQUIRE(!device.dmabuf_queues.empty());
    const auto& q = device.dmabuf_queues.front();
    CHECK_EQ(q.planes.size(), 2u);
    const uint32_t luma = 1920u * 1088u;
    CHECK_EQ(q.planes[0].fd, q.planes[1].fd); /* one fd for both planes */
    CHECK_EQ(q.planes[0].data_offset, 0u);
    CHECK_EQ(q.planes[0].length, luma);
    CHECK_EQ(q.planes[1].data_offset, luma);
    CHECK_EQ(q.planes[1].length, luma / 2);
}

/* --- the CPU (derive/get) view backed by the bo (7.7 point 4) --- */

void test_capture_buffer_map_roundtrip()
{
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    StatefulSession::Frame frame = decode_one(session, 1);
    auto guard = session.hold();
    stateful::SurfaceBuffer* bo = session.capture_buffer(frame.index);
    REQUIRE(bo != nullptr);
    auto whole = bo->map();
    /* Container covers luma + chroma of the NV12 (7.7 point 4). */
    const size_t luma = static_cast<size_t>(1920) * 1088;
    CHECK(whole.size() >= luma + luma / 2);
    whole[0] = 0x5a;
    whole[luma] = 0xa5; /* start of chroma */
    auto again = bo->map();
    CHECK_EQ(again[0], 0x5a);
    CHECK_EQ(again[luma], 0xa5);
    /* export gives a distinct, openable fd. */
    int fd = bo->export_fd();
    CHECK(fd >= 0);
    CHECK(fd != bo->fd());
    if (fd >= 0) {
        close(fd);
    }
}

/* --- the export descriptor layout (7.7 point 3), pure --- */

void test_prime_descriptor_composed()
{
    /* Firefox's composed NV12: one layer, two planes in one object. Named
     * mutation: swap offsets[0]/offsets[1] -> chroma/luma cross. */
    VADRMPRIMESurfaceDescriptor desc;
    stateful::fill_nv12_prime_descriptor(&desc, 7, 1920, 1080, 1920, 1920u * 1620u, 0);

    CHECK_EQ(desc.fourcc, static_cast<uint32_t>(VA_FOURCC_NV12));
    CHECK_EQ(desc.width, 1920u);
    CHECK_EQ(desc.height, 1080u);
    CHECK_EQ(desc.num_objects, 1u);
    CHECK_EQ(desc.objects[0].fd, 7);
    CHECK_EQ(desc.objects[0].size, 1920u * 1620u);
    CHECK(desc.objects[0].drm_format_modifier == DRM_FORMAT_MOD_LINEAR);
    CHECK_EQ(desc.num_layers, 1u);
    CHECK_EQ(desc.layers[0].drm_format, static_cast<uint32_t>(DRM_FORMAT_NV12));
    CHECK_EQ(desc.layers[0].num_planes, 2u);
    CHECK_EQ(desc.layers[0].object_index[0], 0u);
    CHECK_EQ(desc.layers[0].object_index[1], 0u);
    CHECK_EQ(desc.layers[0].offset[0], 0u);
    CHECK_EQ(desc.layers[0].offset[1], 1920u * 1080u);
    CHECK_EQ(desc.layers[0].pitch[0], 1920u);
    CHECK_EQ(desc.layers[0].pitch[1], 1920u);
}

void test_prime_descriptor_separate()
{
    /* Two layers: R8 luma, GR88 chroma. */
    VADRMPRIMESurfaceDescriptor desc;
    stateful::fill_nv12_prime_descriptor(
        &desc, 9, 1280, 720, 1280, 1280u * 1080u, VA_EXPORT_SURFACE_SEPARATE_LAYERS | VA_EXPORT_SURFACE_READ_ONLY);

    CHECK_EQ(desc.num_objects, 1u);
    CHECK_EQ(desc.num_layers, 2u);
    CHECK_EQ(desc.layers[0].drm_format, static_cast<uint32_t>(DRM_FORMAT_R8));
    CHECK_EQ(desc.layers[0].num_planes, 1u);
    CHECK_EQ(desc.layers[0].offset[0], 0u);
    CHECK_EQ(desc.layers[0].pitch[0], 1280u);
    CHECK_EQ(desc.layers[1].drm_format, static_cast<uint32_t>(DRM_FORMAT_GR88));
    CHECK_EQ(desc.layers[1].num_planes, 1u);
    CHECK_EQ(desc.layers[1].offset[0], 1280u * 720u);
    CHECK_EQ(desc.layers[1].pitch[0], 1280u);
}

/* --- the export gate (7.7 point 3): before sync -> SURFACE_BUSY --- */

void test_export_gate()
{
    using stateful::export_gate;
    using stateful::ExportGate;

    /* Not decoded into yet (vaExportSurfaceHandle before sync): SURFACE_BUSY.
     * Named mutation: return ready when !has_decoded_frame -> a stale/empty
     * buffer is exported. */
    CHECK(export_gate(false, /*has_frame=*/false, /*gbm=*/true, /*gen_ok=*/true) == ExportGate::busy);
    /* Decoded, gbm, current generation: ready. */
    CHECK(export_gate(false, true, true, true) == ExportGate::ready);
    /* Decoded but MMAP mode: no zero-copy buffer -> UNIMPLEMENTED (VA1). */
    CHECK(export_gate(false, true, false, true) == ExportGate::unimplemented);
    /* Decoded, gbm, but the binding went stale after a re-provision: BUSY. */
    CHECK(export_gate(false, true, true, false) == ExportGate::busy);
    /* Dead session: DECODING_ERROR. */
    CHECK(export_gate(true, true, true, true) == ExportGate::dead);
}

/* --- MMAP mode still works (7.7 point 6: keep VA1 behaviour) --- */

void test_mmap_mode_unaffected()
{
    FakeDevice device; /* default caps: no DMABUF */
    FakeAllocator allocator;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    StatefulSession::Frame frame = decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    auto guard = session.hold();
    CHECK(session.capture_buffer(frame.index) == nullptr); /* no bo in MMAP mode */
    guard.unlock();
    session.release_frame(frame);
    CHECK_EQ(device.free_captures.size(), 10u);
}

} // namespace

int main()
{
    test_no_dmabuf_cap_falls_back_to_mmap();
    test_unusable_allocator_falls_back_to_mmap();
    test_null_allocator_falls_back_to_mmap();
    test_env_forces_mmap();
    test_gbm_mode_when_capable();
    test_stride_negotiated_then_gbm();
    test_stride_refused_falls_back_to_mmap();
    test_gbm_allocation_failure_falls_back();
    test_dmabuf_qbuf_eio_falls_back_to_mmap_and_decodes();
    test_dmabuf_qbuf_einval_also_falls_back();
    test_spares_when_min_exceeds_surfaces();
    test_dmabuf_qbuf_single_plane();
    test_dmabuf_qbuf_two_plane_offsets();
    test_capture_buffer_map_roundtrip();
    test_prime_descriptor_composed();
    test_prime_descriptor_separate();
    test_export_gate();
    test_mmap_mode_unaffected();
    return check_result("test_gbm_surface");
}
