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
 * VA2b (VPU_DESIGN.md 7.6; VA2-survey (C) "AV1 -- must rebuild OBU bitstream").
 * The AV1 stateful context re-synthesises the OBU temporal unit from the parsed
 * VA buffers. This host test asserts, without a device:
 *
 *  (A) the LEB128, OBU-wrapper and tile_group (5.11.1) primitives;
 *  (B) a synthesised key frame's OBU stream parses back with the GStreamer AV1
 *      parser (temporal delimiter + sequence header + frame header + tile
 *      group), the frame type / dimensions round-trip, and the tile payload is
 *      forwarded byte-for-byte;
 *  (C) an inter frame's header parses back too;
 *  (D) loop restoration with lr_unit_shift 0/1/2 on a 64x64-superblock stream
 *      round-trips -- this directly guards the fix for the shipped gst bit
 *      writer bug (it can only encode lr_unit_shift == 2), which would otherwise
 *      make most real inter frames unparseable;
 *  (E) segmentation is rejected (the gst writer does not support it).
 */

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <exception>
#include <vector>

extern "C" {
#include <va/va.h>
#include <va/va_dec_av1.h>
}

#include "../../src/stateful/av1_bitstream.h"
#include "check.h"

using stateful::Av1AccessUnitBuilder;

namespace {

/* One-shot parse of a synthesised temporal unit. */
struct ParsedAu {
    bool ok = true;
    bool have_seq = false;
    bool have_frame = false;
    bool have_tile_group = false;
    int obu_count = 0;
    GstAV1SequenceHeaderOBU seq {};
    GstAV1FrameHeaderOBU frame {};
    std::vector<uint8_t> tile_bytes;
};

ParsedAu parse_au(GstAV1Parser* parser, const std::vector<uint8_t>& au)
{
    ParsedAu out;
    size_t off = 0;
    while (off < au.size()) {
        GstAV1OBU obu;
        guint32 consumed = 0;
        if (gst_av1_parser_identify_one_obu(parser, au.data() + off, au.size() - off, &obu, &consumed)
            != GST_AV1_PARSER_OK) {
            out.ok = false;
            break;
        }
        out.obu_count++;
        switch (obu.obu_type) {
        case GST_AV1_OBU_TEMPORAL_DELIMITER:
            if (gst_av1_parser_parse_temporal_delimiter_obu(parser, &obu) != GST_AV1_PARSER_OK) {
                out.ok = false;
            }
            break;
        case GST_AV1_OBU_SEQUENCE_HEADER:
            if (gst_av1_parser_parse_sequence_header_obu(parser, &obu, &out.seq) != GST_AV1_PARSER_OK) {
                out.ok = false;
            } else {
                out.have_seq = true;
            }
            break;
        case GST_AV1_OBU_FRAME_HEADER:
            if (gst_av1_parser_parse_frame_header_obu(parser, &obu, &out.frame) != GST_AV1_PARSER_OK) {
                out.ok = false;
            } else {
                out.have_frame = true;
                /* A real decoder updates its reference state after each frame so
                 * the next inter frame's references are valid. */
                gst_av1_parser_reference_frame_update(parser, &out.frame);
            }
            break;
        case GST_AV1_OBU_TILE_GROUP: {
            GstAV1TileGroupOBU tg;
            memset(&tg, 0, sizeof(tg));
            if (gst_av1_parser_parse_tile_group_obu(parser, &obu, &tg) != GST_AV1_PARSER_OK) {
                out.ok = false;
            } else {
                out.have_tile_group = true;
                for (guint32 i = 0; i < tg.num_tiles; i++) {
                    out.tile_bytes.insert(out.tile_bytes.end(), obu.data + tg.entry[i].tile_offset,
                        obu.data + tg.entry[i].tile_offset + tg.entry[i].tile_size);
                }
            }
            break;
        }
        default:
            break;
        }
        off += consumed;
    }
    return out;
}

/* A minimal, self-consistent VA picture parameter buffer for a single-tile
 * 8-bit 4:2:0 frame with no loop filter / CDEF / restoration by default. */
VADecPictureParameterBufferAV1 make_pic(unsigned width, unsigned height, unsigned frame_type, uint8_t base_q)
{
    VADecPictureParameterBufferAV1 p;
    memset(&p, 0, sizeof(p));
    p.profile = 0;
    p.order_hint_bits_minus_1 = 6;
    p.bit_depth_idx = 0;
    p.frame_width_minus1 = static_cast<uint16_t>(width - 1);
    p.frame_height_minus1 = static_cast<uint16_t>(height - 1);
    p.seq_info_fields.fields.enable_order_hint = 1;
    p.seq_info_fields.fields.subsampling_x = 1;
    p.seq_info_fields.fields.subsampling_y = 1;
    p.pic_info_fields.bits.frame_type = frame_type;
    p.pic_info_fields.bits.show_frame = 1;
    p.pic_info_fields.bits.uniform_tile_spacing_flag = 1;
    p.tile_cols = 1;
    p.tile_rows = 1;
    p.base_qindex = base_q;
    p.superres_scale_denominator = 8;
    p.primary_ref_frame = 7; /* PRIMARY_REF_NONE */
    return p;
}

std::vector<uint8_t> fake_tile(size_t size, uint8_t seed)
{
    std::vector<uint8_t> t(size);
    for (size_t i = 0; i < size; i++) {
        t[i] = static_cast<uint8_t>(seed + i * 7);
    }
    return t;
}

void test_leb128()
{
    CHECK(stateful::leb128(0) == std::vector<uint8_t> { 0x00 });
    CHECK(stateful::leb128(1) == std::vector<uint8_t> { 0x01 });
    CHECK(stateful::leb128(127) == std::vector<uint8_t> { 0x7f });
    CHECK(stateful::leb128(128) == (std::vector<uint8_t> { 0x80, 0x01 }));
    CHECK(stateful::leb128(300) == (std::vector<uint8_t> { 0xac, 0x02 }));
}

void test_wrap_obu()
{
    std::vector<uint8_t> payload { 0xaa, 0xbb, 0xcc };
    auto obu = stateful::wrap_obu(GST_AV1_OBU_TILE_GROUP, payload);
    /* obu_header: type(4)<<3 | has_size(0x02); size LEB; payload. */
    CHECK_EQ(obu[0], static_cast<uint8_t>((GST_AV1_OBU_TILE_GROUP << 3) | 0x02));
    CHECK_EQ(obu[1], 3);
    CHECK(std::equal(payload.begin(), payload.end(), obu.begin() + 2));
}

void test_tile_group_single()
{
    auto tile = fake_tile(40, 5);
    VASliceParameterBufferAV1 sp;
    memset(&sp, 0, sizeof(sp));
    sp.slice_data_offset = 0;
    sp.slice_data_size = 40;
    auto payload = stateful::build_tile_group_payload({ &sp, 1 }, tile, 1, 1, 1);
    /* Single tile: the payload is exactly the tile bytes. */
    CHECK(payload == tile);
}

void test_tile_group_multi()
{
    auto t0 = fake_tile(10, 1);
    auto t1 = fake_tile(20, 100);
    std::vector<uint8_t> data;
    data.insert(data.end(), t0.begin(), t0.end());
    data.insert(data.end(), t1.begin(), t1.end());
    VASliceParameterBufferAV1 sp[2];
    memset(sp, 0, sizeof(sp));
    sp[0].slice_data_offset = 0;
    sp[0].slice_data_size = 10;
    sp[0].tile_column = 0;
    sp[1].slice_data_offset = 10;
    sp[1].slice_data_size = 20;
    sp[1].tile_column = 1;
    auto payload = stateful::build_tile_group_payload({ sp, 2 }, data, 2, 1, 1);
    /* flag byte(0) + tile0_size_minus_1(1 byte) + tile0 + tile1. */
    CHECK_EQ(payload[0], 0x00);
    CHECK_EQ(payload[1], 9); /* tile0 size (10) - 1, tile_size_bytes = 1 */
    CHECK(std::equal(t0.begin(), t0.end(), payload.begin() + 2));
    CHECK(std::equal(t1.begin(), t1.end(), payload.begin() + 2 + 10));
}

void test_keyframe_roundtrip()
{
    GstAV1Parser* parser = gst_av1_parser_new();
    Av1AccessUnitBuilder builder;

    auto tile = fake_tile(64, 3);
    auto pic = make_pic(320, 240, 0 /* KEY */, 100);
    VASliceParameterBufferAV1 sp;
    memset(&sp, 0, sizeof(sp));
    sp.slice_data_size = static_cast<uint32_t>(tile.size());
    builder.set_picture_parameters(pic);
    builder.add_tile_parameters({ &sp, 1 });
    builder.add_tile_data(tile);
    auto au = builder.finish();
    CHECK(!au.empty());

    auto parsed = parse_au(parser, au);
    CHECK(parsed.ok);
    CHECK(parsed.have_seq);
    CHECK(parsed.have_frame);
    CHECK(parsed.have_tile_group);
    CHECK_EQ(parsed.frame.frame_type, GST_AV1_KEY_FRAME);
    CHECK_EQ(parsed.frame.frame_width, 320);
    CHECK_EQ(parsed.frame.frame_height, 240);
    CHECK_EQ(parsed.seq.seq_profile, GST_AV1_PROFILE_0);
    /* The tile payload is forwarded byte-for-byte. */
    CHECK(parsed.tile_bytes == tile);
    gst_av1_parser_free(parser);
}

void test_inter_roundtrip()
{
    GstAV1Parser* parser = gst_av1_parser_new();
    Av1AccessUnitBuilder builder;

    /* First a key frame so the DPB has a reference. */
    auto ktile = fake_tile(48, 9);
    auto kpic = make_pic(320, 240, 0, 100);
    VASliceParameterBufferAV1 ksp;
    memset(&ksp, 0, sizeof(ksp));
    ksp.slice_data_size = static_cast<uint32_t>(ktile.size());
    builder.set_picture_parameters(kpic);
    builder.add_tile_parameters({ &ksp, 1 });
    builder.add_tile_data(ktile);
    (void)parse_au(parser, builder.finish());

    /* Then an inter frame referencing it. */
    auto tile = fake_tile(80, 4);
    auto pic = make_pic(320, 240, 1 /* INTER */, 100);
    pic.order_hint = 1;
    pic.primary_ref_frame = 0;
    pic.mode_control_fields.bits.reference_select = 1;
    pic.pic_info_fields.bits.use_ref_frame_mvs = 1;
    for (int i = 0; i < 7; i++) {
        pic.ref_frame_idx[i] = 0;
    }
    /* current_frame surfaces are opaque ids; the key frame used the default 0,
     * so give the inter frame's refs a valid map entry. */
    for (int i = 0; i < 8; i++) {
        pic.ref_frame_map[i] = 0;
    }
    VASliceParameterBufferAV1 sp;
    memset(&sp, 0, sizeof(sp));
    sp.slice_data_size = static_cast<uint32_t>(tile.size());
    builder.set_picture_parameters(pic);
    builder.add_tile_parameters({ &sp, 1 });
    builder.add_tile_data(tile);
    auto au = builder.finish();
    auto parsed = parse_au(parser, au);
    CHECK(parsed.ok);
    CHECK(parsed.have_frame);
    CHECK_EQ(parsed.frame.frame_type, GST_AV1_INTER_FRAME);
    CHECK(parsed.tile_bytes == tile);
    gst_av1_parser_free(parser);
}

/* The core guard for the shipped gst bit-writer bug: with a 64x64 superblock
 * (use_128x128_superblock == 0) every lr_unit_shift must round-trip. The gst
 * writer can only encode shift == 2; our serialiser encodes all three. */
void test_loop_restoration_unit_shift_fix()
{
    for (unsigned shift = 0; shift <= 2; shift++) {
        GstAV1Parser* parser = gst_av1_parser_new();
        Av1AccessUnitBuilder builder;

        auto tile = fake_tile(56, static_cast<uint8_t>(shift + 1));
        auto pic = make_pic(256, 256, 0 /* KEY */, 100);
        /* Wiener restoration on the luma plane, unit shift under test. */
        pic.loop_restoration_fields.bits.yframe_restoration_type = 1; /* WIENER */
        pic.loop_restoration_fields.bits.lr_unit_shift = shift;
        VASliceParameterBufferAV1 sp;
        memset(&sp, 0, sizeof(sp));
        sp.slice_data_size = static_cast<uint32_t>(tile.size());
        builder.set_picture_parameters(pic);
        builder.add_tile_parameters({ &sp, 1 });
        builder.add_tile_data(tile);
        auto au = builder.finish();

        auto parsed = parse_au(parser, au);
        CHECK(parsed.ok);
        CHECK(parsed.have_frame);
        CHECK_EQ(parsed.frame.loop_restoration_params.frame_restoration_type[0], GST_AV1_FRAME_RESTORE_WIENER);
        CHECK_EQ(parsed.frame.loop_restoration_params.lr_unit_shift, shift);
        gst_av1_parser_free(parser);
    }
}

void test_segmentation_rejected()
{
    Av1AccessUnitBuilder builder;
    auto tile = fake_tile(32, 2);
    auto pic = make_pic(320, 240, 0, 100);
    pic.seg_info.segment_info_fields.bits.enabled = 1;
    VASliceParameterBufferAV1 sp;
    memset(&sp, 0, sizeof(sp));
    sp.slice_data_size = static_cast<uint32_t>(tile.size());
    builder.set_picture_parameters(pic);
    builder.add_tile_parameters({ &sp, 1 });
    builder.add_tile_data(tile);
    bool threw = false;
    try {
        (void)builder.finish();
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK(threw);
}

} // namespace

int main()
{
    test_leb128();
    test_wrap_obu();
    test_tile_group_single();
    test_tile_group_multi();
    test_keyframe_roundtrip();
    test_inter_roundtrip();
    test_loop_restoration_unit_shift_fix();
    test_segmentation_rejected();

    if (g_check_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_check_failures);
        return 1;
    }
    printf("all AV1 bitstream checks passed\n");
    return 0;
}
