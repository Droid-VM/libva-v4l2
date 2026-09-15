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
#include <span>
#include <vector>

extern "C" {
#include <va/va.h>
#include <va/va_dec_vp9.h>
}

namespace stateful {

/*
 * The VP9 access-unit assembler (VPU_DESIGN.md 7.6; VA2-survey (C) "VP9 --
 * pure passthrough"). Unlike H.264, VP9 is NOT re-synthesised from separate VA
 * parameter structures: a VP9 frame carries its own uncompressed and compressed
 * headers in-band, so the VASliceDataBuffer already holds the complete frame
 * bitstream (uncompressed header + compressed header + tiles) that a
 * full-bitstream V4L2_PIX_FMT_VP9 decoder wants. ffmpeg-vaapi submits one VP9
 * frame per picture (superframes are split by the parser upstream), so
 * assembling the access unit is just forwarding the slice-data byte range(s)
 * VERBATIM -- no start code, no parameter injection, no synthesis.
 *
 * This is a pure, device-free unit so the verbatim property is host-testable
 * (test/stateful/test_vp9_context.cc) exactly as H264AccessUnitBuilder is.
 */
class VP9AccessUnitBuilder {
public:
    /* VADecPictureParameterBufferVP9: only the frame dimensions are kept (for
     * an optional sanity/log use); the compressed frame does not need them. */
    void set_picture_parameters(const VADecPictureParameterBufferVP9& picture);

    /* VASliceParameterBufferVP9 array: each entry's slice_data_offset /
     * slice_data_size bound one frame's bytes inside the slice-data buffer. */
    void add_slice_parameters(std::span<const VASliceParameterBufferVP9> slices);

    /* Raw VASliceDataBuffer bytes (the compressed VP9 frame). */
    void add_slice_data(std::span<const uint8_t> data);

    /*
     * The access unit for this picture: the slice-data byte range(s) forwarded
     * verbatim. With slice parameters, each slice's [slice_data_offset,
     * slice_data_offset + slice_data_size) region is appended in order (the
     * offsets are honoured); with no slice parameters, the whole accumulated
     * slice-data buffer is forwarded. Throws std::runtime_error if a slice's
     * offset/size runs past the accumulated data. Resets the per-picture
     * accumulators. Empty when no slice data was queued.
     */
    std::vector<uint8_t> finish();

    bool has_data() const { return !slice_data_.empty(); }
    unsigned frame_width() const { return frame_width_; }
    unsigned frame_height() const { return frame_height_; }

private:
    void reset_picture();

    std::vector<uint8_t> slice_data_;
    std::vector<VASliceParameterBufferVP9> slice_params_;
    unsigned frame_width_ = 0;
    unsigned frame_height_ = 0;
};

} // namespace stateful
