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
    /* count = max(surfaces 6, min 4 + share 8 = 12) = 12 (7.7 point 2, VA2i: the
     * codec's minimum plus the full kPoolShareCap held-surface headroom, since
     * in gbm mode a surface IS a buffer). */
    CHECK_EQ(device.capture_count, 12u);
    CHECK_EQ(allocator.allocations, 12u);
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
    /* The device adopted the bo stride, so the session re-read it: the QBUF
     * offsets and the export descriptor now describe the planes at 2048, which
     * is exactly the bo's own stride -- the only geometry turnip imports. */
    CHECK_EQ(session.capture_format().bytesperline, 2048u);
}

void test_non_aligned_resolution_negotiates_stride()
{
    /* VA2g, THE 854x480 zero-copy fix. The phone measurement (VA2g §investigation)
     * settled the ambiguity VA2d left open: c2.qti.av1.decoder genuinely emits a
     * TIGHT 854-byte NV12 (KEY_STRIDE 854, MediaImage2 row_inc 854, buffer 614880),
     * so the data is really at an 854 pitch -- there is no aligned MediaCodec
     * stride to "report". GBM must round the R8 container to its 64 B row
     * alignment (854 -> 896), and turnip's dma-buf EGLImage import rejects an
     * export pitch (854) that is not the bo's own stride (896) -> EGL_BAD_ALLOC ->
     * YouTube drops to dav1d. VA2d's slack path (keep bytesperline 854, export
     * 854 into a 896 bo) is bit-exact for the CPU download but is exactly the
     * pitch turnip refuses. The fix pads on the DEVICE side: S_FMT(CAPTURE) now
     * adopts the bo's 64-aligned luma stride (the crosvm android backend already
     * copies row by row, so it writes each 854-wide row into a 896-stride slot),
     * and this libva path -- unchanged since it was written -- negotiates that
     * stride and re-reads it, so the export pitch, the bo stride and the data's
     * row pitch are all 896 and the visible 854 rides as the surface width/crop.
     * This models the padded device: negotiable stride, tight 854 geometry, a bo
     * GBM rounds to 896. Named mutation: drop the `granted == stride` re-read in
     * provision_capture_gbm_locked -> the export keeps the tight 854 pitch and
     * this assertion fails. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.capture_stride_negotiable = true; /* the padded device adopts the bo stride */
    device.scripted_format.width = 854;
    device.scripted_format.height = 480;
    device.scripted_format.bytesperline = 854; /* tight: MediaCodec's real KEY_STRIDE at 854 */
    device.scripted_format.sizeimage = 854u * 480u * 3u / 2u; /* 614880 */
    device.scripted_format.num_planes = 1;
    FakeAllocator allocator;
    allocator.align_stride_to_width = true; /* model GBM: stride = align_up(width, 64) */
    allocator.align_stride = 64; /* align_up(854, 64) = 896 */
    StatefulSession session(device, 0x31305641, 854, 480, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    /* S_FMT was tried once (bo stride 896 != device 854) and GRANTED (unlike the
     * pre-VA2g qti codec, the padded device widens). */
    CHECK_EQ(device.set_capture_stride_calls, 1u);
    /* The bo was requested at the tight device stride (854); GBM's 64 B rounding
     * gives 896. */
    CHECK_EQ(allocator.last_width, 854u);
    /* The negotiated geometry is adopted: bytesperline == the bo's 896 stride, so
     * the QBUF plane offsets and the exported descriptor use 896 -- the pitch that
     * equals the bo's real stride AND the data's row pitch, which is what the GPU
     * importer requires. */
    CHECK_EQ(session.capture_format().bytesperline, 896u);
}

void test_narrow_bo_stride_refused_falls_back_to_mmap()
{
    /* The one stride case that still cannot do zero-copy: the bo is NARROWER
     * than the device's luma stride (a bo of 1600 for a 1920 bytesperline) and
     * the device refuses S_FMT, so the codec's 1920-packed rows would not fit
     * the bo row-for-row -- fall back to MMAP (7.7 point 1). (A bo WIDER than
     * bytesperline is fine and stays gbm-dmabuf; see
     * test_non_aligned_resolution_stays_gbm.) Named mutation: drop the
     * `stride < bytesperline` MMAP branch -> it proceeds in gbm with a bo that
     * cannot hold the frame. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.capture_stride_negotiable = false;
    FakeAllocator allocator;
    allocator.stride = 1600; /* < bytesperline 1920 */
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK_EQ(device.set_capture_stride_calls, 1u);
    CHECK(!device.capture_dmabuf);
    /* The device's MMAP pool was provisioned instead. */
    CHECK(device.capture_streaming);
    CHECK_EQ(device.capture_count, 10u); /* min 4 + share 6 */
}

void test_wider_bo_stride_refused_stays_gbm()
{
    /* A bo WIDER than the device stride with S_FMT refused: the aligned-1088
     * shape from the spike, but with a bo GBM padded past the device's 1920
     * (2048) and a codec that will not grow. Zero-copy still holds -- the codec
     * writes 1920-packed into the 2048-wide bo and the descriptor uses 1920.
     * This is the general form of the 854x480 fix at an otherwise ordinary
     * resolution. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.capture_stride_negotiable = false;
    FakeAllocator allocator;
    allocator.stride = 2048; /* > bytesperline 1920 */
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    CHECK(device.capture_dmabuf);
    CHECK_EQ(device.set_capture_stride_calls, 1u);
    CHECK_EQ(session.capture_format().bytesperline, 1920u); /* device stride kept */
}

void test_gbm_container_too_small_falls_back_to_mmap()
{
    /* 7.7 (2): the stride matches (no S_FMT) but the device's G_FMT sizeimage
     * exceeds the R8 container -- the decoder would write past the bo, so the
     * session must free the bos and fall back to MMAP. Named mutation: drop the
     * first->size() < sizeimage guard -> this proceeds in gbm with a short
     * buffer. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.scripted_format.bytesperline = 1920; /* stride matches the bo */
    /* The fake bo is stride*(height+height/2) = 1920*1632 = 3133440; ask the
     * device for more than that. */
    device.scripted_format.sizeimage = 4000000;
    FakeAllocator allocator;
    allocator.stride = 1920;
    StatefulSession session(device, 0x34363248, 1920, 1088, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::mmap);
    CHECK(!device.capture_dmabuf);
    CHECK(device.capture_streaming);
}

void test_non_aligned_resolution_stays_gbm()
{
    /* VA2d, THE fix: a non-16-aligned resolution (854x480, where YouTube
     * starts). The phone's qti decoder emits a tightly-packed 854-byte luma
     * stride (bytesperline == the visible width, no padding) and REFUSES to
     * grow it (S_FMT granted 854), while GBM rounds an R8 width up to its 64 B
     * row alignment and cannot produce a 854 stride -- it returns 896. So the
     * bo is WIDER than the device needs. The old code required stride ==
     * bytesperline (or a granted S_FMT) and fell back to MMAP on the refusal,
     * losing zero-copy export. The fix accepts the wider bo: the codec writes
     * its 854-packed NV12 into the roomier container and the export descriptor
     * describes the planes at bytesperline (the extra 42 B/row is slack). Named
     * mutation: turn the `stride > bytesperline` slack branch back into
     * `return false` -> this falls back to MMAP as before. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.capture_stride_negotiable = false; /* the qti codec will not move its stride */
    device.scripted_format.width = 854;
    device.scripted_format.height = 480;
    device.scripted_format.bytesperline = 854; /* tightly packed: stride == width */
    device.scripted_format.sizeimage = 854u * 480u * 3u / 2u; /* 614880 */
    device.scripted_format.num_planes = 1;
    FakeAllocator allocator;
    allocator.align_stride_to_width = true; /* model GBM: stride = align_up(width, 64) */
    allocator.align_stride = 64; /* align_up(854, 64) = 896 */
    StatefulSession session(device, 0x34363248, 854, 480, gbm_options(&allocator));

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    CHECK(device.capture_dmabuf);
    /* S_FMT was tried once (the bo stride 896 != device 854) and refused; the
     * session accepted the wider bo as slack rather than falling back. */
    CHECK_EQ(device.set_capture_stride_calls, 1u);
    /* The bo was requested at the device bytesperline (854), so GBM's 64 B
     * rounding gives 896 -- wide enough to hold the 854-strided frame. */
    CHECK_EQ(allocator.last_width, 854u);
    /* The device geometry is KEPT (bytesperline stays 854, not the bo's 896),
     * so the QBUF offsets and the export descriptor use the codec's real
     * stride. */
    CHECK_EQ(session.capture_format().bytesperline, 854u);
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
    /* Fewer surfaces than the codec minimum: in gbm mode a surface IS a buffer,
     * so the pool is min + the full kPoolShareCap headroom (VA2i), and the bos
     * beyond the client's surfaces are internal spares (7.7 point 2). */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.scripted_min_buffers = 10;
    FakeAllocator allocator;
    StatefulSession::Options options = gbm_options(&allocator);
    options.num_surfaces = 2;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    /* max(surfaces 2, min 10 + share 8 = 18) = 18. */
    CHECK_EQ(device.capture_count, 18u);
    CHECK_EQ(allocator.allocations, 18u);
}

void test_gbm_headroom_survives_a_lazy_surface_count()
{
    /* VA2i, THE fix: AV1's MIN_BUFFERS_FOR_CAPTURE is 21. A lazily-allocating
     * browser has made only its first surface when the first SOURCE_CHANGE
     * provisions the gbm pool (D84), so surface_count reads 1. The shipped
     * share of min(num_surfaces, 8) then collapsed to +1, giving AV1 a
     * 22-buffer pool with only ONE buffer holdable above the codec's 21-buffer
     * floor -- the moment Firefox held a second surface the codec had < 21
     * queued, stalled, and the awaited frame never arrived (Stateful sync
     * failed on sequence ~4, VA2h). The pool must instead give the full
     * kPoolShareCap headroom regardless of the early count, so a client that
     * grows its hold-set later (Firefox composites ~6) does not starve the
     * codec. Named mutation this fails under: restore
     * gbm_share = min(num_surfaces_, kPoolShareCap) -> the pool is 22 again and
     * only 1 buffer is holdable. */
    FakeDevice device;
    device.capture_caps = V4L2_BUF_CAP_SUPPORTS_MMAP | V4L2_BUF_CAP_SUPPORTS_DMABUF;
    device.scripted_min_buffers = 21; /* the phone's AV1 MIN_BUFFERS_FOR_CAPTURE */
    /* The phone's 854x480 AV1 geometry: tight 854 luma stride, GBM pads the R8
     * container to 896 and the qti codec keeps its 854 stride (VA2g slack). */
    device.capture_stride_negotiable = false;
    device.scripted_format.width = 854;
    device.scripted_format.height = 480;
    device.scripted_format.bytesperline = 854;
    device.scripted_format.sizeimage = 854u * 480u * 3u / 2u; /* 614880 */
    device.scripted_format.num_planes = 1;
    FakeAllocator allocator;
    allocator.align_stride_to_width = true;
    allocator.align_stride = 64; /* align_up(854, 64) = 896 */
    StatefulSession::Options options = gbm_options(&allocator);
    options.num_surfaces = 0; /* what vaCreateContext really passes */
    options.surface_count = [] { return 1u; }; /* Firefox has made one so far */
    StatefulSession session(device, 0x30315641 /* AV01 */, 854, 480, options);

    decode_one(session, 1);
    CHECK(session.capture_mode() == StatefulSession::CaptureMode::gbm_dmabuf);
    /* max(surfaces 1, min 21 + share 8 = 29) = 29, NOT the old min+1 = 22. */
    CHECK_EQ(device.capture_count, 29u);
    CHECK_EQ(allocator.allocations, 29u);
    /* The client can hold a full compositor queue without starving the codec. */
    CHECK_EQ(device.capture_count - static_cast<unsigned>(device.scripted_min_buffers), 8u);
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

void test_prime_descriptor_non_aligned_pitch()
{
    /* VA2g: a non-16-aligned visible width (854x480, YouTube's 480p rendition)
     * exported through the padded-device fix. The stride passed to the
     * descriptor is the bo's 64-aligned luma stride (896) -- the value the
     * device now writes at and the value turnip's dma-buf importer demands --
     * while the descriptor width stays the visible 854 (turnip crops to it).
     * Both plane pitches and the chroma offset ride the 896 stride, NOT the 854
     * visible width. This is the export contract the whole VA2g chain relies on:
     * pitch == bo stride == data row pitch, width == crop. Named mutation: make
     * fill_nv12_prime_descriptor derive the pitch or the chroma offset from
     * `width` instead of `stride` -> these checks fail. */
    VADRMPRIMESurfaceDescriptor desc;
    stateful::fill_nv12_prime_descriptor(&desc, 11, 854, 480, 896, 896u * 720u, 0);

    CHECK_EQ(desc.width, 854u); /* the visible crop, not the padded stride */
    CHECK_EQ(desc.height, 480u);
    CHECK_EQ(desc.num_objects, 1u);
    CHECK_EQ(desc.objects[0].size, 896u * 720u);
    CHECK_EQ(desc.num_layers, 1u);
    CHECK_EQ(desc.layers[0].drm_format, static_cast<uint32_t>(DRM_FORMAT_NV12));
    CHECK_EQ(desc.layers[0].num_planes, 2u);
    CHECK_EQ(desc.layers[0].offset[0], 0u);
    CHECK_EQ(desc.layers[0].offset[1], 896u * 480u); /* chroma at stride*height, not width*height */
    CHECK_EQ(desc.layers[0].pitch[0], 896u); /* luma pitch == bo stride */
    CHECK_EQ(desc.layers[0].pitch[1], 896u); /* chroma pitch == bo stride */
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

/* --- vaDeriveImage unavailable reason (7.7 (4)), pure --- */

void test_derive_unavailable_message()
{
    using stateful::derive_unavailable_message;
    /* No frame bound (the negotiation-time probe, the phone's num_planes == 1
     * case): the message names the unbound surface. Named mutation: swap the
     * have_view branch -> the two messages cross. */
    CHECK(std::string(derive_unavailable_message(false)) == "no decoded frame is bound to this surface yet");
    /* A frame is bound but not a single contiguous plane. */
    CHECK(std::string(derive_unavailable_message(true)) == "the decoded frame is not a single contiguous NV12 plane");
    CHECK(std::string(derive_unavailable_message(false)) != std::string(derive_unavailable_message(true)));
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
    test_non_aligned_resolution_negotiates_stride();
    test_narrow_bo_stride_refused_falls_back_to_mmap();
    test_wider_bo_stride_refused_stays_gbm();
    test_gbm_container_too_small_falls_back_to_mmap();
    test_non_aligned_resolution_stays_gbm();
    test_gbm_allocation_failure_falls_back();
    test_dmabuf_qbuf_eio_falls_back_to_mmap_and_decodes();
    test_dmabuf_qbuf_einval_also_falls_back();
    test_spares_when_min_exceeds_surfaces();
    test_gbm_headroom_survives_a_lazy_surface_count();
    test_dmabuf_qbuf_single_plane();
    test_dmabuf_qbuf_two_plane_offsets();
    test_capture_buffer_map_roundtrip();
    test_prime_descriptor_composed();
    test_prime_descriptor_separate();
    test_prime_descriptor_non_aligned_pitch();
    test_export_gate();
    test_derive_unavailable_message();
    test_mmap_mode_unaffected();
    return check_result("test_gbm_surface");
}
