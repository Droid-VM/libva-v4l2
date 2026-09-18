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
 * VPU_DESIGN.md 7.6 host verification (a) and (b): SPS/PPS synthesis round
 * trips through the GStreamer H.264 parser, and access-unit assembly places
 * start codes and parameter sets correctly.
 */

#include <cstring>
#include <vector>

#include "../../src/stateful/h264_bitstream.h"
#include "check.h"

using stateful::H264AccessUnitBuilder;
using stateful::parse_slice_header_info;
using stateful::synthesize_parameter_sets;
using stateful::write_parameter_sets;

namespace {

/* Split an Annex-B byte stream on 4-byte start codes (the only 00 00 00 01
 * runs possible once emulation prevention bytes are in). */
std::vector<std::vector<uint8_t>> split_nals(const std::vector<uint8_t>& stream)
{
    std::vector<std::vector<uint8_t>> nals;
    std::vector<size_t> starts;
    for (size_t i = 0; i + 4 <= stream.size(); i++) {
        if (stream[i] == 0 && stream[i + 1] == 0 && stream[i + 2] == 0 && stream[i + 3] == 1) {
            starts.push_back(i);
        }
    }
    for (size_t i = 0; i < starts.size(); i++) {
        size_t begin = starts[i] + 4;
        size_t end = (i + 1 < starts.size()) ? starts[i + 1] : stream.size();
        nals.emplace_back(stream.begin() + begin, stream.begin() + end);
    }
    return nals;
}

struct ParsedSets {
    GstH264SPS sps;
    GstH264PPS pps;
};

ParsedSets parse_back(GstH264NalParser* parser, std::vector<uint8_t> stream)
{
    REQUIRE(split_nals(stream).size() == 2);
    /* A trailing filler NAL delimits the PPS for gst_h264_parser_identify_nalu. */
    const uint8_t trailer[] = { 0x00, 0x00, 0x00, 0x01, 0x0c, 0x80 };
    stream.insert(stream.end(), std::begin(trailer), std::end(trailer));

    ParsedSets parsed = {};
    GstH264NalUnit nalu = {};
    REQUIRE(gst_h264_parser_identify_nalu(parser, stream.data(), 0, stream.size(), &nalu) == GST_H264_PARSER_OK);
    REQUIRE(nalu.type == GST_H264_NAL_SPS);
    REQUIRE(gst_h264_parser_parse_sps(parser, &nalu, &parsed.sps) == GST_H264_PARSER_OK);

    REQUIRE(gst_h264_parser_identify_nalu(parser, stream.data(), nalu.offset + nalu.size, stream.size(), &nalu)
        == GST_H264_PARSER_OK);
    REQUIRE(nalu.type == GST_H264_NAL_PPS);
    REQUIRE(gst_h264_parser_parse_pps(parser, &nalu, &parsed.pps) == GST_H264_PARSER_OK);
    return parsed;
}

/* The [SPS][PPS] prefix of an assembled access unit, for parse_back(). */
std::vector<uint8_t> parameter_set_prefix(const std::vector<uint8_t>& access_unit)
{
    auto nals = split_nals(access_unit);
    REQUIRE(nals.size() >= 3);
    std::vector<uint8_t> prefix;
    for (unsigned i = 0; i < 2; i++) {
        const uint8_t start_code[] = { 0x00, 0x00, 0x00, 0x01 };
        prefix.insert(prefix.end(), std::begin(start_code), std::end(start_code));
        prefix.insert(prefix.end(), nals[i].begin(), nals[i].end());
    }
    return prefix;
}

VAPictureParameterBufferH264 high_1080p_picture()
{
    /* A High-profile 1080p stream with B-frames, CABAC, 8x8 transform,
     * POC type 0 (the B17 reference clip's shape). */
    VAPictureParameterBufferH264 picture = {};
    picture.picture_width_in_mbs_minus1 = 119; /* 1920 */
    picture.picture_height_in_mbs_minus1 = 67; /* 1088 coded */
    picture.num_ref_frames = 4;
    picture.seq_fields.bits.chroma_format_idc = 1;
    picture.seq_fields.bits.frame_mbs_only_flag = 1;
    picture.seq_fields.bits.direct_8x8_inference_flag = 1;
    picture.seq_fields.bits.log2_max_frame_num_minus4 = 4;
    picture.seq_fields.bits.pic_order_cnt_type = 0;
    picture.seq_fields.bits.log2_max_pic_order_cnt_lsb_minus4 = 6;
    picture.pic_fields.bits.entropy_coding_mode_flag = 1;
    picture.pic_fields.bits.weighted_bipred_idc = 2;
    picture.pic_fields.bits.transform_8x8_mode_flag = 1;
    picture.pic_fields.bits.deblocking_filter_control_present_flag = 1;
    picture.pic_init_qp_minus26 = -3;
    picture.chroma_qp_index_offset = -2;
    picture.second_chroma_qp_index_offset = -4;
    return picture;
}

VAPictureParameterBufferH264 main_720p_picture()
{
    VAPictureParameterBufferH264 picture = {};
    picture.picture_width_in_mbs_minus1 = 79; /* 1280 */
    picture.picture_height_in_mbs_minus1 = 44; /* 720 exactly */
    picture.num_ref_frames = 2;
    picture.seq_fields.bits.chroma_format_idc = 1;
    picture.seq_fields.bits.frame_mbs_only_flag = 1;
    picture.seq_fields.bits.direct_8x8_inference_flag = 1;
    picture.seq_fields.bits.log2_max_frame_num_minus4 = 0;
    picture.seq_fields.bits.pic_order_cnt_type = 2;
    picture.pic_fields.bits.entropy_coding_mode_flag = 1;
    picture.pic_fields.bits.weighted_pred_flag = 1;
    picture.pic_fields.bits.deblocking_filter_control_present_flag = 1;
    picture.pic_init_qp_minus26 = 2;
    picture.chroma_qp_index_offset = 1;
    return picture;
}

void test_round_trip_high_1080p(GstH264NalParser* parser)
{
    auto picture = high_1080p_picture();
    auto sets = synthesize_parameter_sets(picture, nullptr, VAProfileH264High, 1920, 1080, 0, 1, 0, 2);
    auto parsed = parse_back(parser, write_parameter_sets(sets));

    CHECK_EQ(parsed.sps.profile_idc, 100);
    CHECK_EQ(parsed.sps.constraint_set0_flag, 0);
    CHECK_EQ(parsed.sps.constraint_set1_flag, 0);
    CHECK_EQ(parsed.sps.chroma_format_idc, 1);
    CHECK_EQ(parsed.sps.bit_depth_luma_minus8, 0);
    CHECK_EQ(parsed.sps.bit_depth_chroma_minus8, 0);
    CHECK_EQ(parsed.sps.scaling_matrix_present_flag, 0);
    CHECK_EQ(parsed.sps.log2_max_frame_num_minus4, 4);
    CHECK_EQ(parsed.sps.pic_order_cnt_type, 0);
    CHECK_EQ(parsed.sps.log2_max_pic_order_cnt_lsb_minus4, 6);
    CHECK_EQ(parsed.sps.num_ref_frames, 4u);
    CHECK_EQ(parsed.sps.pic_width_in_mbs_minus1, 119u);
    CHECK_EQ(parsed.sps.pic_height_in_map_units_minus1, 67u);
    CHECK_EQ(parsed.sps.frame_mbs_only_flag, 1);
    CHECK_EQ(parsed.sps.direct_8x8_inference_flag, 1);
    CHECK_EQ(parsed.sps.frame_cropping_flag, 1);
    CHECK_EQ(parsed.sps.frame_crop_right_offset, 0u);
    CHECK_EQ(parsed.sps.frame_crop_bottom_offset, 4u); /* (1088-1080)/(CropUnitY=2) */
    CHECK_EQ(parsed.sps.width, 1920); /* coded */
    CHECK_EQ(parsed.sps.height, 1088); /* coded */
    CHECK_EQ(parsed.sps.crop_rect_width, 1920); /* displayed after cropping */
    CHECK_EQ(parsed.sps.crop_rect_height, 1080);

    /* The reorder deadlock rule (7.6 points 3/5a). */
    CHECK_EQ(parsed.sps.vui_parameters_present_flag, 1);
    CHECK_EQ(parsed.sps.vui_parameters.bitstream_restriction_flag, 1);
    /* D91: the reorder depth is what the caller observed, NOT num_ref_frames
     * (4 here); max_dec_frame_buffering still carries the whole DPB. */
    CHECK_EQ(parsed.sps.vui_parameters.num_reorder_frames, 2u);
    CHECK_EQ(parsed.sps.vui_parameters.max_dec_frame_buffering, 4u);
    CHECK_EQ(parsed.sps.vui_parameters.motion_vectors_over_pic_boundaries_flag, 1);

    CHECK_EQ(parsed.pps.id, 0);
    CHECK_EQ(parsed.pps.entropy_coding_mode_flag, 1);
    CHECK_EQ(parsed.pps.num_ref_idx_l0_active_minus1, 1);
    CHECK_EQ(parsed.pps.num_ref_idx_l1_active_minus1, 0);
    CHECK_EQ(parsed.pps.weighted_pred_flag, 0);
    CHECK_EQ(parsed.pps.weighted_bipred_idc, 2);
    CHECK_EQ(parsed.pps.pic_init_qp_minus26, -3);
    CHECK_EQ(parsed.pps.chroma_qp_index_offset, -2);
    CHECK_EQ(parsed.pps.second_chroma_qp_index_offset, -4);
    CHECK_EQ(parsed.pps.transform_8x8_mode_flag, 1);
    CHECK_EQ(parsed.pps.deblocking_filter_control_present_flag, 1);
    CHECK_EQ(parsed.pps.pic_scaling_matrix_present_flag, 0);
}

void test_round_trip_main_720p(GstH264NalParser* parser)
{
    auto picture = main_720p_picture();
    auto sets = synthesize_parameter_sets(picture, nullptr, VAProfileH264Main, 1280, 720, 5, 0, 0, 1);
    auto parsed = parse_back(parser, write_parameter_sets(sets));

    CHECK_EQ(parsed.sps.profile_idc, 77);
    CHECK_EQ(parsed.sps.constraint_set1_flag, 1);
    CHECK_EQ(parsed.sps.pic_order_cnt_type, 2);
    CHECK_EQ(parsed.sps.num_ref_frames, 2u);
    CHECK_EQ(parsed.sps.frame_cropping_flag, 0);
    CHECK_EQ(parsed.sps.width, 1280); /* no cropping: coded == displayed */
    CHECK_EQ(parsed.sps.height, 720);
    CHECK_EQ(parsed.sps.vui_parameters.bitstream_restriction_flag, 1);
    CHECK_EQ(parsed.sps.vui_parameters.num_reorder_frames, 1u);
    CHECK_EQ(parsed.sps.vui_parameters.max_dec_frame_buffering, 2u);

    CHECK_EQ(parsed.pps.id, 5); /* the pps_id the slices reference */
    CHECK_EQ(parsed.pps.weighted_pred_flag, 1);
    CHECK_EQ(parsed.pps.pic_init_qp_minus26, 2);
    /* Main profile: no transform_8x8/scaling/second_chroma extension. */
    CHECK_EQ(parsed.pps.transform_8x8_mode_flag, 0);
}

void test_round_trip_scaling_lists(GstH264NalParser* parser)
{
    auto picture = high_1080p_picture();

    VAIQMatrixBufferH264 iq_matrix = {};
    for (unsigned list = 0; list < 6; list++) {
        for (unsigned i = 0; i < 16; i++) {
            iq_matrix.ScalingList4x4[list][i] = static_cast<uint8_t>(8 + list + i);
        }
    }
    for (unsigned list = 0; list < 2; list++) {
        for (unsigned i = 0; i < 64; i++) {
            iq_matrix.ScalingList8x8[list][i] = static_cast<uint8_t>(9 + list + (i % 32));
        }
    }

    auto sets = synthesize_parameter_sets(picture, &iq_matrix, VAProfileH264High, 1920, 1080, 0, 1, 0, 2);
    auto parsed = parse_back(parser, write_parameter_sets(sets));

    CHECK_EQ(parsed.sps.scaling_matrix_present_flag, 1);
    for (unsigned list = 0; list < 6; list++) {
        uint8_t zigzag[16];
        gst_h264_quant_matrix_4x4_get_zigzag_from_raster(zigzag, iq_matrix.ScalingList4x4[list]);
        CHECK(memcmp(parsed.sps.scaling_lists_4x4[list], zigzag, sizeof(zigzag)) == 0);
    }
    for (unsigned list = 0; list < 2; list++) {
        uint8_t zigzag[64];
        gst_h264_quant_matrix_8x8_get_zigzag_from_raster(zigzag, iq_matrix.ScalingList8x8[list]);
        CHECK(memcmp(parsed.sps.scaling_lists_8x8[list], zigzag, sizeof(zigzag)) == 0);
    }
}

/* A minimal fake slice NAL: header byte, then first_mb_in_slice=0 ('1'),
 * slice_type ue, pps_id ue, padding, then opaque payload bytes. */
std::vector<uint8_t> fake_slice_nal(bool idr, uint8_t payload_byte)
{
    /* '1' (first_mb=0) '011' (slice_type=2) '1' (pps_id=0) -> 10111 000 */
    std::vector<uint8_t> nal = { static_cast<uint8_t>(idr ? 0x65 : 0x41), 0xb8 };
    for (unsigned i = 0; i < 8; i++) {
        nal.push_back(payload_byte);
    }
    return nal;
}

void test_slice_header_parsing()
{
    /* nal type 5, first_mb=0, slice_type=2, pps_id=3: '1' '011' '00100' */
    std::vector<uint8_t> nal = { 0x65, 0xb2, 0x00, 0xaa };
    auto info = parse_slice_header_info(nal);
    REQUIRE(info.has_value());
    CHECK_EQ(info->nal_unit_type, 5);
    CHECK_EQ(info->nal_ref_idc, 3);
    CHECK_EQ(info->first_mb_in_slice, 0u);
    CHECK_EQ(info->slice_type, 2u);
    CHECK_EQ(info->pps_id, 3u);

    /* Emulation prevention: a first_mb ue whose 22-zero prefix spells the
     * bytes 00 00 02 forces the encoder to insert 03 after the zero pair;
     * the parser must skip it. Raw bits: 22 zeros, '1', 22 zero value bits
     * (first_mb 4194303), then '1' (slice_type 0) and '1' (pps_id 0). */
    std::vector<uint8_t> epb_nal = { 0x41, 0x00, 0x00, 0x03, 0x02, 0x00, 0x00, 0x06 };
    auto epb_info = parse_slice_header_info(epb_nal);
    REQUIRE(epb_info.has_value());
    CHECK_EQ(epb_info->first_mb_in_slice, 4194303u);
    CHECK_EQ(epb_info->slice_type, 0u);
    CHECK_EQ(epb_info->pps_id, 0u);

    CHECK(!parse_slice_header_info(std::vector<uint8_t> { 0x65 }).has_value());
}

void run_picture(H264AccessUnitBuilder& builder, const VAPictureParameterBufferH264& picture, bool idr,
    unsigned slice_count, std::vector<uint8_t>* access_unit)
{
    builder.set_picture_parameters(picture);
    std::vector<uint8_t> data;
    std::vector<VASliceParameterBufferH264> params;
    for (unsigned i = 0; i < slice_count; i++) {
        auto nal = fake_slice_nal(idr, static_cast<uint8_t>(0x10 + i));
        VASliceParameterBufferH264 slice = {};
        slice.slice_data_offset = data.size();
        slice.slice_data_size = nal.size();
        slice.num_ref_idx_l0_active_minus1 = 1;
        data.insert(data.end(), nal.begin(), nal.end());
        params.push_back(slice);
    }
    builder.add_slice_parameters(params);
    builder.add_slice_data(data);
    auto finished = builder.finish(1920, 1080);
    REQUIRE(finished.had_picture);
    /* D91: the first picture is held until a client sync reveals the reorder
     * depth. These cases model a client that syncs after one picture. */
    if (finished.access_units.empty()) {
        finished.access_units = builder.flush_held(0);
    }
    REQUIRE(finished.access_units.size() == 1);
    *access_unit = finished.access_units[0];
}

unsigned count_nal_types(const std::vector<uint8_t>& access_unit, uint8_t type)
{
    unsigned count = 0;
    for (auto&& nal : split_nals(access_unit)) {
        if (!nal.empty() && (nal[0] & 0x1f) == type) {
            count += 1;
        }
    }
    return count;
}

void test_access_unit_assembly()
{
    H264AccessUnitBuilder builder(VAProfileH264High);
    auto picture = high_1080p_picture();
    std::vector<uint8_t> access_unit;

    /* IDR: SPS+PPS first, then both slices, all with 4-byte start codes. */
    run_picture(builder, picture, true, 2, &access_unit);
    auto nals = split_nals(access_unit);
    REQUIRE(nals.size() == 4);
    CHECK_EQ(nals[0][0] & 0x1f, 7); /* SPS */
    CHECK_EQ(nals[1][0] & 0x1f, 8); /* PPS */
    CHECK_EQ(nals[2][0] & 0x1f, 5); /* IDR slice */
    CHECK_EQ(nals[3][0] & 0x1f, 5);
    CHECK_EQ(nals[2][2], 0x10); /* slice payloads forwarded verbatim, in order */
    CHECK_EQ(nals[3][2], 0x11);
    CHECK(access_unit[0] == 0 && access_unit[1] == 0 && access_unit[2] == 0 && access_unit[3] == 1);

    /* Same parameters, non-IDR: no re-emission. */
    run_picture(builder, picture, false, 1, &access_unit);
    CHECK_EQ(count_nal_types(access_unit, 7), 0u);
    CHECK_EQ(count_nal_types(access_unit, 8), 0u);
    CHECK_EQ(count_nal_types(access_unit, 1), 1u);

    /* Changed derived parameters (log2_max_frame_num): re-emitted. */
    picture.seq_fields.bits.log2_max_frame_num_minus4 = 8;
    run_picture(builder, picture, false, 1, &access_unit);
    CHECK_EQ(count_nal_types(access_unit, 7), 1u);
    CHECK_EQ(count_nal_types(access_unit, 8), 1u);

    /* Unchanged again. */
    run_picture(builder, picture, false, 1, &access_unit);
    CHECK_EQ(count_nal_types(access_unit, 7), 0u);

    /* IDR always re-emits, changed or not. */
    run_picture(builder, picture, true, 1, &access_unit);
    CHECK_EQ(count_nal_types(access_unit, 7), 1u);
    CHECK_EQ(count_nal_types(access_unit, 8), 1u);

    /* No slices: nothing to assemble. */
    builder.set_picture_parameters(picture);
    CHECK(!builder.finish(1920, 1080).had_picture);
}

/*
 * D91. The VUI's max_num_reorder_frames is how many pictures the decoder may
 * hold before its first output; VA does not carry it, and the pre-D91 stand-in
 * num_ref_frames made c2.qti.avc.decoder sit on 6 High-profile access units
 * while Firefox fed 3 and blocked. The builder must instead hold its access
 * units until the client's first sync and then write the depth that sync
 * revealed.
 */
void test_reorder_depth_comes_from_the_client_feed(GstH264NalParser* parser)
{
    H264AccessUnitBuilder builder(VAProfileH264High);
    auto picture = high_1080p_picture();
    picture.num_ref_frames = 5; /* the D91 stream: 5 refs, real reorder depth 2 */

    /* Three pictures fed with no sync in between: all held, nothing emitted. */
    for (unsigned i = 0; i < 3; i++) {
        builder.set_picture_parameters(picture);
        auto nal = fake_slice_nal(i == 0, static_cast<uint8_t>(0x20 + i));
        VASliceParameterBufferH264 slice = {};
        slice.slice_data_offset = 0;
        slice.slice_data_size = nal.size();
        builder.add_slice_parameters({ &slice, 1 });
        builder.add_slice_data(nal);
        auto finished = builder.finish(1920, 1080);
        REQUIRE(finished.had_picture);
        CHECK(finished.access_units.empty());
        CHECK_EQ(builder.held_count(), i + 1u);
    }

    /* The client syncs: it fed 3, so its reorder depth is 2. */
    auto access_units = builder.flush_held(static_cast<unsigned>(builder.held_count() - 1));
    REQUIRE(access_units.size() == 3);
    CHECK_EQ(builder.held_count(), 0u);

    auto parsed = parse_back(parser, parameter_set_prefix(access_units[0]));
    CHECK_EQ(parsed.sps.vui_parameters.bitstream_restriction_flag, 1);
    /* The mutation this pins: writing num_ref_frames (5) here is exactly the
     * D91 defect -- the codec then wants 6 access units before its first
     * output and a 3-deep client never gets a frame. */
    CHECK_EQ(parsed.sps.vui_parameters.num_reorder_frames, 2u);
    /* References are unaffected: the full DPB is still declared. */
    CHECK_EQ(parsed.sps.vui_parameters.max_dec_frame_buffering, 5u);
    CHECK_EQ(parsed.sps.num_ref_frames, 5u);

    /* Only the first held access unit carries the parameter sets; the other
     * two are slices only, and the slice payloads kept their order. */
    CHECK_EQ(count_nal_types(access_units[0], 7), 1u);
    CHECK_EQ(count_nal_types(access_units[1], 7), 0u);
    CHECK_EQ(count_nal_types(access_units[2], 7), 0u);
    CHECK_EQ(split_nals(access_units[1])[0][2], 0x21);
    CHECK_EQ(split_nals(access_units[2])[0][2], 0x22);

    /* From here nothing is held: the depth is known. */
    std::vector<uint8_t> access_unit;
    run_picture(builder, picture, false, 1, &access_unit);
    CHECK_EQ(builder.held_count(), 0u);
    CHECK_EQ(count_nal_types(access_unit, 1), 1u);
}

/*
 * A client that keeps feeding without ever syncing is not waiting on us and
 * cannot deadlock. The hold must not grow without bound: at the cap it is
 * released with the conservative num_ref_frames, which is what the pre-D91
 * code always wrote.
 */
void test_hold_is_capped_for_a_feed_ahead_client(GstH264NalParser* parser)
{
    H264AccessUnitBuilder builder(VAProfileH264High);
    auto picture = high_1080p_picture();
    picture.num_ref_frames = 5;

    std::vector<std::vector<uint8_t>> released;
    for (size_t i = 0; i < H264AccessUnitBuilder::kMaxHeldAccessUnits; i++) {
        builder.set_picture_parameters(picture);
        auto nal = fake_slice_nal(i == 0, 0x30);
        VASliceParameterBufferH264 slice = {};
        slice.slice_data_offset = 0;
        slice.slice_data_size = nal.size();
        builder.add_slice_parameters({ &slice, 1 });
        builder.add_slice_data(nal);
        auto finished = builder.finish(1920, 1080);
        REQUIRE(finished.had_picture);
        released = std::move(finished.access_units);
    }
    REQUIRE(released.size() == H264AccessUnitBuilder::kMaxHeldAccessUnits);
    CHECK_EQ(builder.held_count(), 0u);

    auto parsed = parse_back(parser, parameter_set_prefix(released[0]));
    CHECK_EQ(parsed.sps.vui_parameters.num_reorder_frames, 5u);
}

} // namespace

int main()
{
    GstH264NalParser* parser = gst_h264_nal_parser_new();
    REQUIRE(parser != nullptr);

    test_round_trip_high_1080p(parser);
    test_round_trip_main_720p(parser);
    test_round_trip_scaling_lists(parser);
    test_slice_header_parsing();
    test_access_unit_assembly();
    test_reorder_depth_comes_from_the_client_feed(parser);
    test_hold_is_capped_for_a_feed_ahead_client(parser);

    gst_h264_nal_parser_free(parser);
    return check_result("test_h264_bitstream");
}
