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

#include "h264_bitstream.h"

#include <algorithm>
#include <cstring>
#include <stdexcept>

namespace stateful {

namespace {

    /*
     * Fields VA-API does not carry and the defaults this file fills in
     * (VPU_DESIGN.md 7.6 point 3 asks for this list):
     *
     *  - seq_parameter_set_id: 0 (we emit exactly one SPS).
     *  - pic_parameter_set_id: parsed from the first slice header of the access
     *    unit -- the slice NALs are forwarded verbatim, so the PPS id must match
     *    what the slices reference.
     *  - profile_idc / constraint_set flags: derived from the VAProfile of the
     *    context (ConstrainedBaseline -> 66 + set0 + set1, Main -> 77 + set1,
     *    High -> 100).
     *  - level_idc: not in VA; derived from the coded size and num_ref_frames via
     *    the Table A-1 MaxFS/MaxDpbMbs limits (smallest fitting level).
     *  - VUI: VA carries no VUI. We write one anyway because the reorder deadlock
     *    rule (7.6 points 3 and 5) requires bitstream_restriction_flag=1 with
     *    max_num_reorder_frames = num_ref_frames and max_dec_frame_buffering =
     *    max(num_ref_frames, 1); the remaining restriction fields use the spec
     *    defaults (motion_vectors_over_pic_boundaries=1, max_bytes_per_pic_denom=2,
     *    max_bits_per_mb_denom=1, log2_max_mv_length_h/v=15). No timing, HRD or
     *    colour information is written.
     *  - pic_order_cnt_type==1 cycle data (offset_for_non_ref_pic,
     *    offset_for_top_to_bottom_field, offset_for_ref_frame[]): VA does not
     *    carry them; they are written as an empty cycle (all zero). Streams using
     *    POC type 1 with a non-trivial cycle would decode wrongly -- such streams
     *    are vanishingly rare and are documented as out of scope for VA1.
     *  - pic_init_qs_minus26: 0 (SP/SI unsupported anyway).
     *  - num_ref_idx_l{0,1}_default_active_minus1: VA only carries the effective
     *    per-slice counts; the default is taken from the first slice of the
     *    access unit. A stream whose first slice overrides the PPS default while
     *    a later slice relies on it would mis-parse; encoders do not do this.
     *  - gaps_in_frame_num_value_allowed_flag, direct_8x8_inference_flag, etc.
     *    come from seq_fields verbatim.
     */

    struct LevelLimit {
        uint8_t level_idc;
        uint32_t max_fs; /* macroblocks per frame */
        uint32_t max_dpb_mbs;
    };

    constexpr LevelLimit kLevelLimits[] = {
        { 10, 99, 396 },
        { 11, 396, 900 },
        { 12, 396, 2376 },
        { 13, 396, 2376 },
        { 20, 396, 2376 },
        { 21, 792, 4752 },
        { 22, 1620, 8100 },
        { 30, 1620, 8100 },
        { 31, 3600, 18000 },
        { 32, 5120, 20480 },
        { 40, 8192, 32768 },
        { 41, 8192, 32768 },
        { 42, 8704, 34816 },
        { 50, 22080, 110400 },
        { 51, 36864, 184320 },
        { 52, 36864, 184320 },
        { 60, 139264, 696320 },
        { 61, 139264, 696320 },
        { 62, 139264, 696320 },
    };

    uint8_t derive_level_idc(uint32_t frame_size_in_mbs, uint32_t num_ref_frames)
    {
        for (const auto& limit : kLevelLimits) {
            if (limit.max_fs >= frame_size_in_mbs
                && limit.max_dpb_mbs >= frame_size_in_mbs * std::max(num_ref_frames, 1u)) {
                return limit.level_idc;
            }
        }
        return 62;
    }

    bool is_flat_16(const uint8_t* list, size_t size)
    {
        return std::all_of(list, list + size, [](uint8_t v) { return v == 16; });
    }

    /* Reads Exp-Golomb values from a NAL payload, removing emulation prevention
     * bytes on the fly. */
    class UEReader {
    public:
        explicit UEReader(std::span<const uint8_t> data)
            : data_(data)
        {
        }

        std::optional<uint32_t> read_bit()
        {
            if (byte_ >= data_.size()) {
                return std::nullopt;
            }
            /* 00 00 03: the 03 is an emulation prevention byte, skip it. */
            if (bit_ == 0 && byte_ >= 2 && data_[byte_] == 0x03 && data_[byte_ - 1] == 0x00
                && data_[byte_ - 2] == 0x00) {
                byte_ += 1;
                if (byte_ >= data_.size()) {
                    return std::nullopt;
                }
            }
            uint32_t bit = (data_[byte_] >> (7 - bit_)) & 1;
            bit_ += 1;
            if (bit_ == 8) {
                bit_ = 0;
                byte_ += 1;
            }
            return bit;
        }

        std::optional<uint32_t> read_ue()
        {
            unsigned zeros = 0;
            while (true) {
                auto bit = read_bit();
                if (!bit) {
                    return std::nullopt;
                }
                if (*bit != 0) {
                    break;
                }
                if (++zeros > 31) {
                    return std::nullopt;
                }
            }
            uint32_t value = 1;
            for (unsigned i = 0; i < zeros; i++) {
                auto bit = read_bit();
                if (!bit) {
                    return std::nullopt;
                }
                value = (value << 1) | *bit;
            }
            return value - 1;
        }

    private:
        std::span<const uint8_t> data_;
        size_t byte_ = 0;
        unsigned bit_ = 0;
    };

    /* Runs one bit-writer NAL through convert_to_nal to insert emulation
     * prevention bytes, and appends the result. */
    /* Turns one raw RBSP (nal header byte first, rbsp trailing bits already
     * present, no start code) into a 4-byte-start-code Annex-B NAL with emulation
     * prevention bytes, and appends it. The bit writer's own start_code=TRUE path
     * is avoided: it emits a malformed prefix on this GStreamer, so RBSP is
     * produced with start_code=FALSE and convert_to_nal adds the start code. */
    void append_nal_with_emulation_prevention(std::vector<uint8_t>& out, const uint8_t* raw, unsigned raw_bytes)
    {
        std::vector<uint8_t> nal(raw_bytes * 2 + 8);
        guint nal_size = nal.size();
        GstH264BitWriterResult result = gst_h264_bit_writer_convert_to_nal(4 /* nal_prefix_size */,
            FALSE /* packetized */, FALSE /* has_startcode */, FALSE /* add_trailings */, raw,
            static_cast<gsize>(raw_bytes) * 8, nal.data(), &nal_size);
        if (result != GST_H264_BIT_WRITER_OK) {
            throw std::runtime_error("failed to insert emulation prevention bytes");
        }
        out.insert(out.end(), nal.data(), nal.data() + nal_size);
    }

} // namespace

std::optional<H264SliceHeaderInfo> parse_slice_header_info(std::span<const uint8_t> nal)
{
    if (nal.size() < 2) {
        return std::nullopt;
    }

    H264SliceHeaderInfo info = {};
    info.nal_unit_type = nal[0] & 0x1f;
    info.nal_ref_idc = (nal[0] >> 5) & 0x3;

    UEReader reader(nal.subspan(1));
    auto first_mb = reader.read_ue();
    auto slice_type = reader.read_ue();
    auto pps_id = reader.read_ue();
    if (!first_mb || !slice_type || !pps_id || *pps_id > 255) {
        return std::nullopt;
    }
    info.first_mb_in_slice = *first_mb;
    info.slice_type = *slice_type;
    info.pps_id = *pps_id;
    return info;
}

H264ParameterSets synthesize_parameter_sets(const VAPictureParameterBufferH264& picture,
    const VAIQMatrixBufferH264* iq_matrix, VAProfile profile, unsigned display_width, unsigned display_height,
    uint32_t pps_id, uint8_t num_ref_idx_l0_default_active_minus1, uint8_t num_ref_idx_l1_default_active_minus1)
{
    H264ParameterSets sets = {};
    GstH264SPS& sps = sets.sps;
    GstH264PPS& pps = sets.pps;

    switch (profile) {
    case VAProfileH264ConstrainedBaseline:
        sps.profile_idc = 66;
        sps.constraint_set0_flag = 1;
        sps.constraint_set1_flag = 1;
        break;
    case VAProfileH264Main:
        sps.profile_idc = 77;
        sps.constraint_set1_flag = 1;
        break;
    case VAProfileH264High:
    default:
        sps.profile_idc = 100;
        break;
    }

    sps.id = 0;
    sps.chroma_format_idc = picture.seq_fields.bits.chroma_format_idc;
    sps.separate_colour_plane_flag = picture.seq_fields.bits.residual_colour_transform_flag;
    sps.bit_depth_luma_minus8 = picture.bit_depth_luma_minus8;
    sps.bit_depth_chroma_minus8 = picture.bit_depth_chroma_minus8;
    sps.qpprime_y_zero_transform_bypass_flag = 0;
    sps.log2_max_frame_num_minus4 = picture.seq_fields.bits.log2_max_frame_num_minus4;
    sps.pic_order_cnt_type = picture.seq_fields.bits.pic_order_cnt_type;
    sps.log2_max_pic_order_cnt_lsb_minus4 = picture.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4;
    sps.delta_pic_order_always_zero_flag = picture.seq_fields.bits.delta_pic_order_always_zero_flag;
    /* POC type 1 cycle data is not carried by VA: written as an empty cycle. */
    sps.offset_for_non_ref_pic = 0;
    sps.offset_for_top_to_bottom_field = 0;
    sps.num_ref_frames_in_pic_order_cnt_cycle = 0;
    sps.num_ref_frames = picture.num_ref_frames;
    sps.gaps_in_frame_num_value_allowed_flag = picture.seq_fields.bits.gaps_in_frame_num_value_allowed_flag;
    sps.pic_width_in_mbs_minus1 = picture.picture_width_in_mbs_minus1;
    sps.frame_mbs_only_flag = picture.seq_fields.bits.frame_mbs_only_flag;
    sps.mb_adaptive_frame_field_flag = picture.seq_fields.bits.mb_adaptive_frame_field_flag;
    sps.direct_8x8_inference_flag = picture.seq_fields.bits.direct_8x8_inference_flag;

    /* VA's picture_height_in_mbs is in frame macroblocks; map units halve for
     * field coding. */
    const uint32_t height_in_mbs = picture.picture_height_in_mbs_minus1 + 1u;
    const uint32_t map_unit_divisor = 2u - sps.frame_mbs_only_flag;
    sps.pic_height_in_map_units_minus1 = height_in_mbs / map_unit_divisor - 1u;

    const uint32_t coded_width = (picture.picture_width_in_mbs_minus1 + 1u) * 16u;
    const uint32_t coded_height = height_in_mbs * 16u;
    sps.level_idc
        = derive_level_idc((picture.picture_width_in_mbs_minus1 + 1u) * height_in_mbs, picture.num_ref_frames);

    /* Frame cropping from the surface (display) size, 4:2:0 crop units. */
    if (display_width > 0 && display_height > 0 && (display_width < coded_width || display_height < coded_height)) {
        sps.frame_cropping_flag = 1;
        sps.frame_crop_left_offset = 0;
        sps.frame_crop_right_offset = (coded_width - display_width) / 2u;
        sps.frame_crop_top_offset = 0;
        sps.frame_crop_bottom_offset = (coded_height - display_height) / (2u * map_unit_divisor);
    }

    /* Scaling lists: VA hands them in raster order, the writer wants the
     * bitstream (zigzag) order. All-16 lists mean "no scaling lists in the
     * stream" (the Flat_4x4/Flat_8x8 fallback) and are omitted entirely.
     * Non-flat lists are carried in the SPS; that requires profile_idc >= 100,
     * which always holds since only High profile streams use them. */
    bool have_scaling_lists = false;
    if (iq_matrix != nullptr) {
        for (const auto& list : iq_matrix->ScalingList4x4) {
            have_scaling_lists |= !is_flat_16(list, sizeof(list));
        }
        for (const auto& list : iq_matrix->ScalingList8x8) {
            have_scaling_lists |= !is_flat_16(list, sizeof(list));
        }
    }
    if (have_scaling_lists && sps.profile_idc >= 100) {
        sps.scaling_matrix_present_flag = 1;
        for (unsigned i = 0; i < 6; i++) {
            gst_h264_quant_matrix_4x4_get_zigzag_from_raster(sps.scaling_lists_4x4[i], iq_matrix->ScalingList4x4[i]);
        }
        /* VA carries the two 8x8 lists of 4:2:0 (Y intra, Y inter). */
        gst_h264_quant_matrix_8x8_get_zigzag_from_raster(sps.scaling_lists_8x8[0], iq_matrix->ScalingList8x8[0]);
        gst_h264_quant_matrix_8x8_get_zigzag_from_raster(sps.scaling_lists_8x8[1], iq_matrix->ScalingList8x8[1]);
        for (unsigned i = 2; i < 6; i++) {
            memset(sps.scaling_lists_8x8[i], 16, sizeof(sps.scaling_lists_8x8[i]));
        }
    }

    /* The reorder deadlock rule (7.6 points 3 and 5a): a VA client takes
     * frames in display order and stops feeding input while it waits, so the
     * decoder must not hold back more frames than the DPB implies.
     *
     * D91 is this one field. max_num_reorder_frames is how many pictures the
     * decoder is ALLOWED to sit on before its first output, and a decoder that
     * is allowed to sit on N produces nothing until access unit N + 1 (plus
     * its own pipeline). VA carries no reorder depth, so num_ref_frames was
     * used as a stand-in -- a safe upper bound that is, on real streams,
     * enormous. The adaptive High stream in the D91 record declares 5
     * reference frames against a true reorder depth of 2, so we told
     * c2.qti.avc.decoder it could hold 5 and it duly waited for the 6th access
     * unit: exactly the "the codec needs ~6 High-profile pictures before
     * SOURCE_CHANGE" threshold B26 measured and B23-B29 spent six rounds
     * attributing to Qualcomm. Firefox feeds 3 and blocks on vaSyncSurface, so
     * the grace expired, the sequence-1 sync failed, vaExportSurfaceHandle
     * failed, and it fell back to software. Constrained Baseline escaped only
     * because num_ref_frames is 1 there.
     *
     * 0 is the right value HERE, and it is a statement about this transport
     * rather than about the content. Output ORDER carries no information on
     * this path: every access unit is queued with its sequence as the V4L2
     * timestamp and the CAPTURE buffer is claimed by that tag (session.cc), so
     * the client is handed the picture it submitted whatever order the decoder
     * emits in -- and a VA client always reorders for itself, out of the DPB
     * it owns and whose POCs it has just handed us. What the client cannot
     * absorb is output LATENCY, because it blocks on the picture it has just
     * submitted. So ask the decoder to bump each picture as soon as it is
     * decoded.
     *
     * Measured, not assumed, on the phone. Sub-threshold browser-free repro
     * (B28's `-flags low_delay`: a 3-access-unit feeder, the Firefox shape) --
     * declared 0 or 1 gets past the first sync, declared 2, 3 or 5 stalls at 3
     * fed / 0 out. Full decode of the same fixture -- declared 0 gives
     * byte-identical output to declared 2, 3 and 5. Host-direct MediaCodec
     * probe against c2.qti.avc.decoder -- declared 0 delivers 300/300 in the
     * same delivery order as the native stream, so the decoder neither drops
     * pictures nor reorders differently; it only starts sooner.
     *
     * max_dec_frame_buffering is deliberately left at the full DPB: it governs
     * what may be RETAINED, which references depend on, not how long output
     * may be withheld. */
    sps.vui_parameters_present_flag = 1;
    GstH264VUIParams& vui = sps.vui_parameters;
    vui.bitstream_restriction_flag = 1;
    vui.motion_vectors_over_pic_boundaries_flag = 1;
    vui.max_bytes_per_pic_denom = 2;
    vui.max_bits_per_mb_denom = 1;
    vui.log2_max_mv_length_horizontal = 15;
    vui.log2_max_mv_length_vertical = 15;
    vui.num_reorder_frames = 0;
    vui.max_dec_frame_buffering = std::max<uint32_t>(picture.num_ref_frames, 1u);

    pps.id = static_cast<gint>(pps_id);
    pps.sequence = &sps;
    pps.entropy_coding_mode_flag = picture.pic_fields.bits.entropy_coding_mode_flag;
    pps.pic_order_present_flag = picture.pic_fields.bits.pic_order_present_flag;
    pps.num_slice_groups_minus1 = 0; /* FMO is not supported by VA. */
    pps.num_ref_idx_l0_active_minus1 = num_ref_idx_l0_default_active_minus1;
    pps.num_ref_idx_l1_active_minus1 = num_ref_idx_l1_default_active_minus1;
    pps.weighted_pred_flag = picture.pic_fields.bits.weighted_pred_flag;
    pps.weighted_bipred_idc = picture.pic_fields.bits.weighted_bipred_idc;
    pps.pic_init_qp_minus26 = picture.pic_init_qp_minus26;
    pps.pic_init_qs_minus26 = 0; /* not carried by VA (SP/SI unsupported) */
    pps.chroma_qp_index_offset = picture.chroma_qp_index_offset;
    pps.deblocking_filter_control_present_flag = picture.pic_fields.bits.deblocking_filter_control_present_flag;
    pps.constrained_intra_pred_flag = picture.pic_fields.bits.constrained_intra_pred_flag;
    pps.redundant_pic_cnt_present_flag = picture.pic_fields.bits.redundant_pic_cnt_present_flag;
    pps.transform_8x8_mode_flag = picture.pic_fields.bits.transform_8x8_mode_flag;
    pps.pic_scaling_matrix_present_flag = 0; /* scaling lists live in the SPS */
    pps.second_chroma_qp_index_offset = picture.second_chroma_qp_index_offset;

    return sets;
}

std::vector<uint8_t> write_parameter_sets(const H264ParameterSets& sets)
{
    std::vector<uint8_t> out;
    /* The GStreamer bit writer ORs its output into the destination, so the
     * buffer must be zeroed before every write. */
    uint8_t raw[1024] = {};

    guint size = sizeof(raw);
    if (gst_h264_bit_writer_sps(&sets.sps, FALSE, raw, &size) != GST_H264_BIT_WRITER_OK) {
        throw std::runtime_error("failed to write SPS");
    }
    append_nal_with_emulation_prevention(out, raw, size);

    /* The writer follows pps->sequence; re-point it at our SPS copy. */
    GstH264PPS pps = sets.pps;
    GstH264SPS sps = sets.sps;
    pps.sequence = &sps;

    memset(raw, 0, sizeof(raw));
    size = sizeof(raw);
    if (gst_h264_bit_writer_pps(&pps, FALSE, raw, &size) != GST_H264_BIT_WRITER_OK) {
        throw std::runtime_error("failed to write PPS");
    }
    append_nal_with_emulation_prevention(out, raw, size);

    return out;
}

H264AccessUnitBuilder::H264AccessUnitBuilder(VAProfile profile)
    : profile_(profile)
{
}

void H264AccessUnitBuilder::set_picture_parameters(const VAPictureParameterBufferH264& picture)
{
    picture_ = picture;
    has_picture_ = true;
}

void H264AccessUnitBuilder::set_iq_matrix(const VAIQMatrixBufferH264& iq_matrix)
{
    iq_matrix_ = iq_matrix;
    has_iq_matrix_ = true;
}

void H264AccessUnitBuilder::add_slice_parameters(std::span<const VASliceParameterBufferH264> slices)
{
    pending_slice_params_.insert(pending_slice_params_.end(), slices.begin(), slices.end());
}

void H264AccessUnitBuilder::add_slice_data(std::span<const uint8_t> data)
{
    static constexpr uint8_t start_code[] = { 0x00, 0x00, 0x00, 0x01 };

    for (const auto& params : pending_slice_params_) {
        if (params.slice_data_offset >= data.size()
            || data.size() - params.slice_data_offset < params.slice_data_size) {
            throw std::invalid_argument("slice parameters point outside the slice data buffer");
        }
        auto nal = data.subspan(params.slice_data_offset, params.slice_data_size);
        if (!first_slice_) {
            first_slice_ = parse_slice_header_info(nal);
            first_num_ref_idx_l0_ = params.num_ref_idx_l0_active_minus1;
            first_num_ref_idx_l1_ = params.num_ref_idx_l1_active_minus1;
        }
        slice_bytes_.insert(slice_bytes_.end(), std::begin(start_code), std::end(start_code));
        slice_bytes_.insert(slice_bytes_.end(), nal.begin(), nal.end());
    }
    pending_slice_params_.clear();
}

std::vector<uint8_t> H264AccessUnitBuilder::finish(unsigned display_width, unsigned display_height)
{
    std::vector<uint8_t> access_unit;

    if (!has_picture_ || slice_bytes_.empty() || !first_slice_) {
        pending_slice_params_.clear();
        slice_bytes_.clear();
        first_slice_.reset();
        return access_unit;
    }

    auto sets = synthesize_parameter_sets(picture_, has_iq_matrix_ ? &iq_matrix_ : nullptr, profile_, display_width,
        display_height, first_slice_->pps_id, first_num_ref_idx_l0_, first_num_ref_idx_l1_);
    auto parameter_bytes = write_parameter_sets(sets);

    const bool idr = first_slice_->nal_unit_type == 5 /* GST_H264_NAL_SLICE_IDR */;
    if (idr || parameter_bytes != last_emitted_parameter_sets_) {
        access_unit = parameter_bytes;
        last_emitted_parameter_sets_ = std::move(parameter_bytes);
    }
    access_unit.insert(access_unit.end(), slice_bytes_.begin(), slice_bytes_.end());

    has_picture_ = false;
    /* iq_matrix_ is sticky: clients may send it once per sequence. */
    pending_slice_params_.clear();
    slice_bytes_.clear();
    first_slice_.reset();
    return access_unit;
}

} // namespace stateful
