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
#include <optional>
#include <span>
#include <vector>

extern "C" {
#include <va/va.h>
}

#include "gsth264bitwriter_compat.h"

namespace stateful {

/*
 * H.264 bitstream synthesis for the stateful decode path (VPU_DESIGN.md 7.6
 * point 3): every vaEndPicture turns the VA parameter buffers plus the raw
 * slice NALs back into one Annex-B access unit for a stateful (full-bitstream)
 * V4L2 decoder.
 */

struct H264SliceHeaderInfo {
    uint32_t first_mb_in_slice;
    uint32_t slice_type;
    uint32_t pps_id;
    uint8_t nal_unit_type;
    uint8_t nal_ref_idc;
};

/* Parses the fixed leading part of a slice NAL (emulation prevention aware);
 * the NAL starts at the nal_unit_type byte, without a start code. */
std::optional<H264SliceHeaderInfo> parse_slice_header_info(std::span<const uint8_t> nal);

struct H264ParameterSets {
    GstH264SPS sps;
    GstH264PPS pps;
};

/*
 * Derive an SPS/PPS pair from what VA hands a decoder. Fields VA does not
 * carry are defaulted (see the .cc for the list). display_width/height come
 * from the VASurface and produce the frame cropping; pps_id and the
 * num_ref_idx defaults come from the first slice of the access unit.
 */
H264ParameterSets synthesize_parameter_sets(const VAPictureParameterBufferH264& picture,
    const VAIQMatrixBufferH264* iq_matrix, VAProfile profile, unsigned display_width, unsigned display_height,
    uint32_t pps_id, uint8_t num_ref_idx_l0_default_active_minus1, uint8_t num_ref_idx_l1_default_active_minus1);

/* Serialize [startcode SPS][startcode PPS] with emulation prevention bytes. */
std::vector<uint8_t> write_parameter_sets(const H264ParameterSets& sets);

/*
 * Accumulates one picture's VA buffers and assembles the access unit.
 * SPS/PPS are re-emitted only when the derived parameter sets change, and
 * always before an IDR (7.6 point 3).
 */
class H264AccessUnitBuilder {
public:
    explicit H264AccessUnitBuilder(VAProfile profile);

    void set_picture_parameters(const VAPictureParameterBufferH264& picture);
    void set_iq_matrix(const VAIQMatrixBufferH264& iq_matrix);
    void add_slice_parameters(std::span<const VASliceParameterBufferH264> slices);
    /* The VA slice data buffer holds whole NALs (header and emulation
     * prevention bytes included); slice_data_offset of each pending slice
     * parameter points at its nal_unit_type byte. */
    void add_slice_data(std::span<const uint8_t> data);

    /* Assemble the access unit for vaEndPicture and reset the per-picture
     * state. Empty when no slice was queued. */
    std::vector<uint8_t> finish(unsigned display_width, unsigned display_height);

    bool has_picture() const { return has_picture_; }

private:
    VAProfile profile_;
    bool has_picture_ = false;
    bool has_iq_matrix_ = false;
    VAPictureParameterBufferH264 picture_ {};
    VAIQMatrixBufferH264 iq_matrix_ {};
    std::vector<VASliceParameterBufferH264> pending_slice_params_;
    std::vector<uint8_t> slice_bytes_;
    std::optional<H264SliceHeaderInfo> first_slice_;
    uint8_t first_num_ref_idx_l0_ = 0;
    uint8_t first_num_ref_idx_l1_ = 0;
    std::vector<uint8_t> last_emitted_parameter_sets_;
};

} // namespace stateful
