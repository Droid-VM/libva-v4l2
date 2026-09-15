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
 * VA2a (VPU_DESIGN.md 7.6; VA2-survey (C) "VP9 -- pure passthrough"). Two
 * things a VP9 stateful context must get right:
 *
 *  (A) VP9AccessUnitBuilder forwards the compressed VP9 frame VERBATIM -- the
 *      slice-data byte range(s) with the slice_data_offset/size honoured, no
 *      Annex-B-style start code, no parameter-set injection, no synthesis (the
 *      opposite of the H.264 path, which re-synthesises SPS/PPS).
 *  (B) the assembled frame reaches the device OUTPUT plane byte-for-byte
 *      through the real StatefulSession built for V4L2_PIX_FMT_VP9, driven
 *      against the scripted fake device, and comes back 1 in / 1 out.
 */

#include <algorithm>
#include <cstdint>
#include <exception>
#include <vector>

extern "C" {
#include <linux/videodev2.h>

#include <va/va.h>
#include <va/va_dec_vp9.h>
}

#include "../../src/stateful/session.h"
#include "../../src/stateful/vp9_bitstream.h"
#include "check.h"
#include "fake_device.h"

using stateful::StatefulSession;
using stateful::VP9AccessUnitBuilder;

namespace {

/* A distinctive byte pattern; its content is opaque to a passthrough. */
std::vector<uint8_t> fake_vp9_frame(size_t size, uint8_t seed)
{
    std::vector<uint8_t> frame(size);
    for (size_t i = 0; i < size; i++) {
        frame[i] = static_cast<uint8_t>(seed + i);
    }
    return frame;
}

VASliceParameterBufferVP9 slice_param(uint32_t offset, uint32_t size)
{
    VASliceParameterBufferVP9 slice = {};
    slice.slice_data_offset = offset;
    slice.slice_data_size = size;
    return slice;
}

/* (A) A single slice covering the whole buffer: the frame passes through
 * byte-for-byte, same length, nothing prepended or appended. */
void test_single_slice_verbatim()
{
    VP9AccessUnitBuilder builder;
    auto frame = fake_vp9_frame(321, 0x11);
    builder.add_slice_data(frame);
    VASliceParameterBufferVP9 slice = slice_param(0, frame.size());
    builder.add_slice_parameters({ &slice, 1 });

    auto access_unit = builder.finish();
    CHECK(access_unit == frame);
    CHECK_EQ(access_unit.size(), frame.size());
}

/* (A) slice_data_offset is honoured: with junk framing the real payload in the
 * slice-data buffer, only the declared [offset, offset+size) region comes out. */
void test_offset_and_size_honoured()
{
    VP9AccessUnitBuilder builder;
    auto frame = fake_vp9_frame(200, 0x40);
    std::vector<uint8_t> buffer;
    buffer.insert(buffer.end(), 16, 0xEE); /* junk prefix */
    buffer.insert(buffer.end(), frame.begin(), frame.end());
    buffer.insert(buffer.end(), 8, 0xCC); /* junk suffix */
    builder.add_slice_data(buffer);
    VASliceParameterBufferVP9 slice = slice_param(16, frame.size());
    builder.add_slice_parameters({ &slice, 1 });

    auto access_unit = builder.finish();
    CHECK(access_unit == frame);
}

/* (A) No slice parameters: forward the whole accumulated slice-data buffer. */
void test_no_slice_params_forwards_whole_buffer()
{
    VP9AccessUnitBuilder builder;
    auto frame = fake_vp9_frame(150, 0x70);
    builder.add_slice_data(frame);

    auto access_unit = builder.finish();
    CHECK(access_unit == frame);
}

/* (A) Multiple slice parameters concatenate their regions in order. */
void test_multi_slice_concatenation()
{
    VP9AccessUnitBuilder builder;
    auto first = fake_vp9_frame(100, 0x01);
    auto second = fake_vp9_frame(60, 0x80);
    std::vector<uint8_t> buffer;
    buffer.insert(buffer.end(), first.begin(), first.end());
    buffer.insert(buffer.end(), second.begin(), second.end());
    builder.add_slice_data(buffer);
    VASliceParameterBufferVP9 slices[2] = { slice_param(0, first.size()), slice_param(first.size(), second.size()) };
    builder.add_slice_parameters({ slices, 2 });

    std::vector<uint8_t> expected = first;
    expected.insert(expected.end(), second.begin(), second.end());
    CHECK(builder.finish() == expected);
}

/* (A) finish() resets the per-picture state: the next frame is independent. */
void test_empty_and_reset()
{
    VP9AccessUnitBuilder builder;
    CHECK(builder.finish().empty());

    auto first = fake_vp9_frame(40, 0x22);
    builder.add_slice_data(first);
    CHECK(builder.finish() == first);

    auto second = fake_vp9_frame(55, 0x33);
    builder.add_slice_data(second);
    CHECK(builder.finish() == second); /* no bleed from the first frame */
}

/* (A) A slice range past the buffer end is rejected, not read out of bounds. */
void test_out_of_range_throws()
{
    VP9AccessUnitBuilder builder;
    auto frame = fake_vp9_frame(50, 0x10);
    builder.add_slice_data(frame);
    VASliceParameterBufferVP9 slice = slice_param(40, 20); /* 40 + 20 > 50 */
    builder.add_slice_parameters({ &slice, 1 });

    bool threw = false;
    try {
        builder.finish();
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK(threw);
}

/* (A) Picture dimensions are recorded but never enter the access unit. */
void test_picture_dims_not_in_access_unit()
{
    VP9AccessUnitBuilder builder;
    VADecPictureParameterBufferVP9 picture = {};
    picture.frame_width = 1280;
    picture.frame_height = 720;
    builder.set_picture_parameters(picture);
    CHECK_EQ(builder.frame_width(), 1280u);
    CHECK_EQ(builder.frame_height(), 720u);

    auto frame = fake_vp9_frame(30, 0x05);
    builder.add_slice_data(frame);
    CHECK(builder.finish() == frame);
}

StatefulSession::Options fast_options()
{
    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.output_ring_size = 4;
    options.sync_timeout_ms = 30;
    return options;
}

/* (B) The assembled VP9 frame reaches the device OUTPUT plane byte-for-byte
 * through a real session built for V4L2_PIX_FMT_VP9, and decodes 1 in / 1 out
 * (the VP9 threshold the survey measured: announce at 1 access unit). */
void test_session_receives_frame_verbatim()
{
    FakeDevice device;
    StatefulSession session(device, V4L2_PIX_FMT_VP9, 1280, 720, fast_options());
    CHECK(device.subscribed);
    CHECK(device.output_streaming);

    /* Assemble a frame the builder must extract from a junk-framed buffer. */
    VP9AccessUnitBuilder builder;
    auto frame = fake_vp9_frame(300, 0x5A);
    std::vector<uint8_t> buffer;
    buffer.insert(buffer.end(), 12, 0x99);
    buffer.insert(buffer.end(), frame.begin(), frame.end());
    builder.add_slice_data(buffer);
    VASliceParameterBufferVP9 slice = slice_param(12, frame.size());
    builder.add_slice_parameters({ &slice, 1 });
    auto access_unit = builder.finish();
    REQUIRE(access_unit == frame);

    /* Submit: the session copies the access unit into the OUTPUT plane. */
    session.submit(1, access_unit);
    REQUIRE(!device.pending_decodes.empty());
    CHECK_EQ(device.pending_decodes.back().bytes, static_cast<unsigned>(access_unit.size()));

    const unsigned index = device.pending_decodes.back().index;
    REQUIRE(index < device.output_memory.size());
    const auto& plane = device.output_memory[index];
    REQUIRE(plane.size() >= frame.size());
    /* Byte-for-byte: no start code, no parameter injection ahead of the frame. */
    CHECK(std::equal(frame.begin(), frame.end(), plane.begin()));

    /* The frame decodes: 1 in -> 1 out. */
    StatefulSession::Frame decoded;
    CHECK(session.sync(1, &decoded) == StatefulSession::SyncStatus::ok);
    CHECK(session.provisioned());
    session.release_frame(decoded);
}

} // namespace

int main()
{
    test_single_slice_verbatim();
    test_offset_and_size_honoured();
    test_no_slice_params_forwards_whole_buffer();
    test_multi_slice_concatenation();
    test_empty_and_reset();
    test_out_of_range_throws();
    test_picture_dims_not_in_access_unit();
    test_session_receives_frame_verbatim();

    return check_result("test_vp9_context");
}
