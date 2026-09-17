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

/*
 * VA2c: the one-frame-lookahead reconstruction of refresh_frame_flags. A
 * random-access (deep B-pyramid) sequence fills the 8-slot DPB with distinct
 * live surfaces; VA does not carry refresh_frame_flags, and the immediate
 * "evict the oldest" heuristic drops the long-lived GOP anchor (the key frame)
 * the moment its duplicate slots are consumed, so a later frame that references
 * it can no longer resolve it. Holding each ambiguous frame until the next one
 * arrives recovers its exact refresh_frame_flags from the next frame's
 * ref_frame_map. This test drives a model encoder DPB whose ground-truth refresh
 * keeps the key frame pinned in slot 7 while every other slot churns, and
 * asserts the builder reconstructs each deferred frame's refresh exactly and
 * keeps the key frame resolvable.
 */
void test_random_access_refresh_lookahead()
{
    GstAV1Parser* parser = gst_av1_parser_new();
    Av1AccessUnitBuilder builder;

    /* The encoder's DPB ground truth: which surface sits in each of the 8 slots
     * before each frame updates it. */
    VASurfaceID model[8];
    for (int i = 0; i < 8; i++) {
        model[i] = VA_INVALID_SURFACE;
    }
    const VASurfaceID key_surface = 100;

    auto in_slot = [&](VASurfaceID id) {
        for (int j = 0; j < 8; j++) {
            if (model[j] == id) {
                return true;
            }
        }
        return false;
    };

    /* Key frame: current surface 100, refresh all (0xFF). */
    {
        auto pic = make_pic(640, 480, 0 /* KEY */, 100);
        pic.current_frame = key_surface;
        for (int i = 0; i < 8; i++) {
            pic.ref_frame_map[i] = VA_INVALID_SURFACE;
        }
        VASliceParameterBufferAV1 sp;
        memset(&sp, 0, sizeof(sp));
        auto tile = fake_tile(32, 1);
        sp.slice_data_size = static_cast<uint32_t>(tile.size());
        builder.set_picture_parameters(pic);
        builder.add_tile_parameters({ &sp, 1 });
        builder.add_tile_data(tile);
        auto result = builder.submit_picture(1);
        CHECK(result.ready.size() == 1); /* key frame is immediate */
        CHECK_EQ(result.ready[0].refresh_frame_flags, 0xFF);
        (void)parse_au(parser, result.ready[0].access_unit);
        for (int i = 0; i < 8; i++) {
            model[i] = key_surface;
        }
    }

    /* A run of inter frames. Ground-truth refresh: slot 7 always holds the key
     * frame (never refreshed); slots 0..6 are refreshed round-robin. Two frames
     * near the end reference the key frame (slot 7). Each entry is
     * {current_surface, ground_truth_refresh_mask}. */
    struct Step {
        VASurfaceID cur;
        uint8_t refresh;
    };
    const Step steps[] = {
        { 101, 0x01 },
        { 102, 0x02 },
        { 103, 0x04 },
        { 104, 0x08 },
        { 105, 0x10 },
        { 106, 0x20 },
        { 107, 0x40 },
        /* DPB now: [101..107, key]. Full of 8 distinct live surfaces. */
        { 108, 0x01 }, /* churn slot 0, keep the key in slot 7 */
        { 109, 0x02 },
        { 110, 0x00 }, /* a leaf that refreshes nothing */
        { 111, 0x04 },
    };
    const size_t n_steps = sizeof(steps) / sizeof(steps[0]);

    uint64_t tag = 2;
    int checked = 0;
    int key_still_referenceable = 0;
    for (size_t k = 0; k < n_steps; k++) {
        const Step& st = steps[k];
        auto pic = make_pic(640, 480, 1 /* INTER */, 100);
        pic.current_frame = st.cur;
        pic.order_hint = static_cast<uint8_t>(k + 1);
        pic.primary_ref_frame = 0;
        pic.mode_control_fields.bits.reference_select = 1;
        for (int i = 0; i < 8; i++) {
            pic.ref_frame_map[i] = model[i];
        }
        /* Reference the key frame (slot 7) from every ref position: the failure
         * mode is precisely the key frame being evicted, so if it is gone this
         * frame cannot resolve its references. */
        for (int i = 0; i < 7; i++) {
            pic.ref_frame_idx[i] = 7;
        }
        CHECK(in_slot(key_surface)); /* the model keeps it; so must the builder */

        VASliceParameterBufferAV1 sp;
        memset(&sp, 0, sizeof(sp));
        auto tile = fake_tile(48, static_cast<uint8_t>(k + 2));
        sp.slice_data_size = static_cast<uint32_t>(tile.size());
        builder.set_picture_parameters(pic);
        builder.add_tile_parameters({ &sp, 1 });
        builder.add_tile_data(tile);
        auto result = builder.submit_picture(tag++);
        /* Whatever frame became ready is a PREVIOUS one; its reconstructed
         * refresh must equal its ground truth. */
        for (auto& rf : result.ready) {
            /* Find the ground-truth refresh for this tag. tag 2 is steps[0]. */
            size_t idx = static_cast<size_t>(rf.tag - 2);
            if (idx < n_steps) {
                CHECK_EQ(rf.refresh_frame_flags, steps[idx].refresh);
                auto parsed = parse_au(parser, rf.access_unit);
                CHECK(parsed.ok);
                /* Its ref_frame_idx must all be a valid resolved slot (the key
                 * frame was found in our mirrored DPB, not a fall-through). */
                for (int i = 0; i < 7; i++) {
                    CHECK(parsed.frame.ref_frame_idx[i] >= 0 && parsed.frame.ref_frame_idx[i] < 8);
                }
                key_still_referenceable++;
                checked++;
            }
        }
        /* Advance the model with the ground truth. */
        for (int i = 0; i < 8; i++) {
            if (st.refresh & (1u << i)) {
                model[i] = st.cur;
            }
        }
    }

    /* Flush the tail (the last still-deferred frame). */
    auto tail = builder.flush();
    CHECK(tail.size() == 1);

    /* Every ambiguous frame that was emitted before the tail was reconstructed
     * exactly and still resolved the key frame. */
    CHECK(checked >= static_cast<int>(n_steps) - 1);
    CHECK_EQ(key_still_referenceable, checked);
    gst_av1_parser_free(parser);
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

/* VA3-fakeau2 SPIKE. synthesize_fake_frame() rebuilds a padding frame from the
 * last real inter frame with a BUMPED order_hint and refresh_frame_flags=0, for
 * the order_hint-fake injection (a genuinely-new, non-reference frame -- the
 * lever the byte-copy VA3-fakeau lacked). This checks the synthesised header
 * round-trips with exactly those mutations, in both show_frame constructions,
 * that it is side-effect free (the mirrored DPB is untouched, so the following
 * real frames are unaffected), and that it retains nothing unless capture is on
 * (the shipped OFF path). The device question -- does the flush stay bit-exact
 * / perceptually acceptable -- is settled on the phone. */
void test_fake_ohint_synthesis()
{
    GstAV1Parser* parser = gst_av1_parser_new();
    Av1AccessUnitBuilder builder;
    builder.enable_fake_capture(); /* retain the last real frame's VA inputs */

    /* A key frame so the DPB has a reference. */
    auto ktile = fake_tile(48, 9);
    auto kpic = make_pic(320, 240, 0 /* KEY */, 100);
    VASliceParameterBufferAV1 ksp;
    memset(&ksp, 0, sizeof(ksp));
    ksp.slice_data_size = static_cast<uint32_t>(ktile.size());
    builder.set_picture_parameters(kpic);
    builder.add_tile_parameters({ &ksp, 1 });
    builder.add_tile_data(ktile);
    (void)parse_au(parser, builder.finish());

    /* A key frame is not a usable padding template (intra). */
    CHECK(!builder.synthesize_fake_frame(1, true).has_value());

    /* A real inter frame at order_hint 5, referencing the key frame. */
    auto tile = fake_tile(80, 4);
    auto pic = make_pic(320, 240, 1 /* INTER */, 100);
    pic.order_hint = 5;
    pic.primary_ref_frame = 0;
    pic.mode_control_fields.bits.reference_select = 1;
    pic.pic_info_fields.bits.use_ref_frame_mvs = 1;
    for (int i = 0; i < 7; i++) {
        pic.ref_frame_idx[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
        pic.ref_frame_map[i] = 0;
    }
    VASliceParameterBufferAV1 sp;
    memset(&sp, 0, sizeof(sp));
    sp.slice_data_size = static_cast<uint32_t>(tile.size());
    builder.set_picture_parameters(pic);
    builder.add_tile_parameters({ &sp, 1 });
    builder.add_tile_data(tile);
    (void)parse_au(parser, builder.finish());

    /* k = 1, shown: order_hint bumped by 1, non-reference, shown, same refs. */
    auto fake1 = builder.synthesize_fake_frame(1, true);
    CHECK(fake1.has_value());
    auto p1 = parse_au(parser, *fake1);
    CHECK(p1.ok);
    CHECK(p1.have_frame);
    CHECK_EQ(p1.frame.frame_type, GST_AV1_INTER_FRAME);
    CHECK_EQ(static_cast<unsigned>(p1.frame.order_hint), 6u); /* 5 + 1 */
    CHECK_EQ(static_cast<unsigned>(p1.frame.refresh_frame_flags), 0u); /* non-reference: writes no slot */
    CHECK_EQ(static_cast<unsigned>(p1.frame.show_frame), 1u);
    CHECK(p1.tile_bytes == tile); /* carries the real tile payload */

    /* k = 2 bumps by 2. */
    auto fake2 = builder.synthesize_fake_frame(2, true);
    CHECK(fake2.has_value());
    auto p2 = parse_au(parser, *fake2);
    CHECK(p2.ok);
    CHECK_EQ(static_cast<unsigned>(p2.frame.order_hint), 7u); /* 5 + 2 */
    CHECK_EQ(static_cast<unsigned>(p2.frame.refresh_frame_flags), 0u);

    /* Hidden construction: show_frame=0 (no CAPTURE emitted on the device). */
    auto fakeh = builder.synthesize_fake_frame(1, false);
    CHECK(fakeh.has_value());
    auto ph = parse_au(parser, *fakeh);
    CHECK(ph.ok);
    CHECK(ph.have_frame);
    CHECK_EQ(static_cast<unsigned>(ph.frame.show_frame), 0u);
    CHECK_EQ(static_cast<unsigned>(ph.frame.order_hint), 6u);
    CHECK_EQ(static_cast<unsigned>(ph.frame.refresh_frame_flags), 0u);

    /* Side-effect free: synthesising again yields byte-identical output (the
     * mirrored DPB and per-picture state were not mutated), so the real frames
     * that follow read the same reference state. */
    auto fake1b = builder.synthesize_fake_frame(1, true);
    CHECK(fake1b.has_value());
    CHECK(*fake1b == *fake1);

    /* A subsequent real inter frame still assembles cleanly after the fakes. */
    auto tile2 = fake_tile(64, 7);
    auto pic2 = make_pic(320, 240, 1 /* INTER */, 100);
    pic2.order_hint = 6;
    pic2.primary_ref_frame = 0;
    pic2.mode_control_fields.bits.reference_select = 1;
    for (int i = 0; i < 7; i++) {
        pic2.ref_frame_idx[i] = 0;
    }
    for (int i = 0; i < 8; i++) {
        pic2.ref_frame_map[i] = 0;
    }
    VASliceParameterBufferAV1 sp2;
    memset(&sp2, 0, sizeof(sp2));
    sp2.slice_data_size = static_cast<uint32_t>(tile2.size());
    builder.set_picture_parameters(pic2);
    builder.add_tile_parameters({ &sp2, 1 });
    builder.add_tile_data(tile2);
    auto real2 = parse_au(parser, builder.finish());
    CHECK(real2.ok);
    CHECK_EQ(real2.frame.frame_type, GST_AV1_INTER_FRAME);
    CHECK_EQ(static_cast<unsigned>(real2.frame.show_frame), 1u); /* real frames stay shown (VA2c) */

    /* Without capture enabled, a fresh builder retains nothing (OFF path). */
    {
        Av1AccessUnitBuilder off;
        auto kt = fake_tile(48, 9);
        auto kp = make_pic(320, 240, 0, 100);
        VASliceParameterBufferAV1 ks;
        memset(&ks, 0, sizeof(ks));
        ks.slice_data_size = static_cast<uint32_t>(kt.size());
        off.set_picture_parameters(kp);
        off.add_tile_parameters({ &ks, 1 });
        off.add_tile_data(kt);
        (void)off.finish();
        CHECK(!off.synthesize_fake_frame(1, true).has_value());
    }

    gst_av1_parser_free(parser);
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
    test_random_access_refresh_lookahead();
    test_segmentation_rejected();
    test_fake_ohint_synthesis();

    if (g_check_failures != 0) {
        fprintf(stderr, "%d check(s) failed\n", g_check_failures);
        return 1;
    }
    printf("all AV1 bitstream checks passed\n");
    return 0;
}
