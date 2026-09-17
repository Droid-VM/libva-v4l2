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

#include "av1_bitstream.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace stateful {

namespace {

    /* AV1 frame types (VADecPictureParameterBufferAV1 pic_info frame_type). */
    enum { AV1_KEY_FRAME = 0, AV1_INTER_FRAME = 1, AV1_INTRA_ONLY_FRAME = 2, AV1_SWITCH_FRAME = 3 };

    unsigned floor_log2(uint32_t x)
    {
        unsigned s = 0;
        while (x != 0) {
            x >>= 1;
            s++;
        }
        return s == 0 ? 0 : s - 1;
    }

    /* 5.9.16 tile_log2: smallest k such that (blkSize << k) >= target. */
    int tile_log2(int blk_size, int target)
    {
        int k = 0;
        while ((blk_size << k) < target) {
            k++;
        }
        return k;
    }

    int8_t bit_depth_from_idx(uint8_t idx)
    {
        return idx == 0 ? 8 : idx == 1 ? 10 : 12;
    }

    /* Serialise one gst header OBU (size field on) via the bit writer. */
    template <typename Writer> std::vector<uint8_t> write_gst_obu(Writer&& writer)
    {
        std::vector<uint8_t> buffer(4096);
        while (true) {
            guint size = static_cast<guint>(buffer.size());
            GstAV1BitWriterResult result = writer(buffer.data(), &size);
            if (result == GST_AV1_BIT_WRITER_OK) {
                buffer.resize(size);
                return buffer;
            }
            if (result == GST_AV1_BIT_WRITER_NO_MORE_SPACE && buffer.size() < (1u << 24)) {
                buffer.resize(buffer.size() * 2);
                continue;
            }
            throw std::runtime_error("gst AV1 bit writer failed");
        }
    }

} // namespace

std::vector<uint8_t> leb128(uint64_t value)
{
    std::vector<uint8_t> out;
    do {
        uint8_t byte = value & 0x7f;
        value >>= 7;
        if (value != 0) {
            byte |= 0x80;
        }
        out.push_back(byte);
    } while (value != 0);
    return out;
}

std::vector<uint8_t> wrap_obu(GstAV1OBUType type, std::span<const uint8_t> payload)
{
    std::vector<uint8_t> out;
    /* obu_header: forbidden(0) type(4) extension_flag(0) has_size_field(1) reserved(0). */
    out.push_back(static_cast<uint8_t>((static_cast<int>(type) << 3) | 0x02));
    auto size = leb128(payload.size());
    out.insert(out.end(), size.begin(), size.end());
    out.insert(out.end(), payload.begin(), payload.end());
    return out;
}

std::vector<uint8_t> build_tile_group_payload(std::span<const VASliceParameterBufferAV1> tiles,
    std::span<const uint8_t> tile_data, unsigned tile_cols, unsigned tile_rows, unsigned tile_size_bytes)
{
    const unsigned num_tiles = std::max(1u, tile_cols * tile_rows);

    /* Order tiles by TileNum = tile_row * TileCols + tile_column. */
    std::vector<const VASliceParameterBufferAV1*> ordered;
    ordered.reserve(tiles.size());
    for (const auto& tile : tiles) {
        ordered.push_back(&tile);
    }
    std::stable_sort(ordered.begin(), ordered.end(),
        [tile_cols](const VASliceParameterBufferAV1* a, const VASliceParameterBufferAV1* b) {
            return static_cast<unsigned>(a->tile_row) * tile_cols + a->tile_column
                < static_cast<unsigned>(b->tile_row) * tile_cols + b->tile_column;
        });

    std::vector<uint8_t> payload;

    if (num_tiles == 1) {
        /* tile_start_and_end_present_flag is not present; the single tile is the
         * rest of the OBU. Forward the one tile's bytes. */
        if (ordered.empty()) {
            if (tile_data.empty()) {
                throw std::runtime_error("AV1 tile group without tile data");
            }
            payload.assign(tile_data.begin(), tile_data.end());
            return payload;
        }
        const auto& t = *ordered.front();
        if (t.slice_data_offset > tile_data.size() || t.slice_data_size > tile_data.size() - t.slice_data_offset) {
            throw std::runtime_error("AV1 tile offset/size runs past the tile-data buffer");
        }
        payload.assign(
            tile_data.begin() + t.slice_data_offset, tile_data.begin() + t.slice_data_offset + t.slice_data_size);
        return payload;
    }

    if (ordered.size() != num_tiles) {
        throw std::runtime_error("AV1 tile count does not match tile_cols * tile_rows");
    }

    /* tile_start_and_end_present_flag = 0 (one tile group covers the frame); the
     * payload is byte-aligned already (the flag byte is a full byte here because
     * a tile group OBU payload starts byte aligned and only this one bit is
     * written before the alignment). */
    payload.push_back(0x00);
    for (unsigned i = 0; i < num_tiles; i++) {
        const auto& t = *ordered[i];
        if (t.slice_data_offset > tile_data.size() || t.slice_data_size > tile_data.size() - t.slice_data_offset) {
            throw std::runtime_error("AV1 tile offset/size runs past the tile-data buffer");
        }
        const bool last = (i + 1 == num_tiles);
        if (!last) {
            uint64_t size_minus_1 = t.slice_data_size - 1;
            for (unsigned b = 0; b < tile_size_bytes; b++) {
                payload.push_back(static_cast<uint8_t>((size_minus_1 >> (8 * b)) & 0xff));
            }
        }
        payload.insert(payload.end(), tile_data.begin() + t.slice_data_offset,
            tile_data.begin() + t.slice_data_offset + t.slice_data_size);
    }
    return payload;
}

namespace {

    /*
     * Frame-header serialiser (5.9 uncompressed_header), a faithful port of
     * GStreamer's gst_av1_bit_writer_frame_header_obu logic reading the same
     * GstAV1FrameHeaderOBU/GstAV1SequenceHeaderOBU structs -- WITH ONE FIX. The
     * shipped gst_av1_bit_writer (1.24 and 1.28, identical source) has a bug in
     * loop_restoration_params for the 64x64-superblock case: it hardcodes the first
     * lr_unit_shift bit to 1 and only ever emits the shift==2 pattern, so any frame
     * that uses loop restoration with 64x64 superblocks and unit shift 0 or 1
     * produces a frame header its own parser (and ffmpeg / the hardware) cannot
     * re-parse -- and loop-restoration unit params also live in the tile entropy
     * data, so restoration cannot simply be disabled. This is the common case for
     * real inter content, so we serialise the frame header ourselves and encode
     * lr_unit_shift per spec (5.9.20): 0 -> "0", 1 -> "10", 2 -> "11".
     * The temporal-delimiter and sequence-header OBUs still come from the gst
     * writer (those round-trip correctly).
     */
    class BitWriter {
    public:
        void put(uint64_t value, int nbits)
        {
            for (int i = nbits - 1; i >= 0; i--) {
                if ((nbits_ & 7) == 0) {
                    data_.push_back(0);
                }
                uint8_t bit = (value >> i) & 1u;
                data_.back() |= static_cast<uint8_t>(bit << (7 - (nbits_ & 7)));
                nbits_++;
            }
        }
        /* su(n): two's-complement value in (n) bits (matches _av1_write_su's n+1). */
        void put_signed(int32_t value, int nbits) { put(static_cast<uint64_t>(value) & mask(nbits), nbits); }
        size_t bit_size() const { return nbits_; }
        std::vector<uint8_t> finish_rbsp()
        {
            put(1, 1); /* trailing one bit */
            while ((nbits_ & 7) != 0) {
                put(0, 1);
            }
            return data_;
        }

    private:
        static uint64_t mask(int n) { return n >= 64 ? ~0ULL : ((1ULL << n) - 1); }
        std::vector<uint8_t> data_;
        size_t nbits_ = 0;
    };

    int helper_msb(uint32_t n)
    {
        int log = 0;
        for (int i = 4; i >= 0; --i) {
            int shift = 1 << i;
            uint32_t x = n >> shift;
            if (x != 0) {
                n = x;
                log += shift;
            }
        }
        return log;
    }

    void write_delta_q(BitWriter& bw, int32_t delta_q)
    {
        if (delta_q != 0) {
            bw.put(1, 1);
            bw.put_signed(delta_q, 7);
        } else {
            bw.put(0, 1);
        }
    }

    void write_uniform(BitWriter& bw, uint32_t max_value, uint32_t value)
    {
        int l = max_value ? helper_msb(max_value) + 1 : 0;
        uint32_t m = (1u << l) - max_value;
        if (l == 0) {
            return;
        }
        if (value < m) {
            bw.put(value, l - 1);
        } else {
            bw.put(m + ((value - m) >> 1), l - 1);
            bw.put((value - m) & 1, 1);
        }
    }

    uint16_t recenter_nonneg(uint16_t r, uint16_t v)
    {
        if (v > (r << 1)) {
            return v;
        } else if (v >= r) {
            return (v - r) << 1;
        } else {
            return ((r - v) << 1) - 1;
        }
    }

    uint16_t recenter_finite_nonneg(uint16_t n, uint16_t r, uint16_t v)
    {
        if ((r << 1) <= n) {
            return recenter_nonneg(r, v);
        }
        return recenter_nonneg(n - 1 - r, n - 1 - v);
    }

    void write_primitive_quniform(BitWriter& bw, uint16_t n, uint16_t v)
    {
        if (n <= 1) {
            return;
        }
        int l = helper_msb(n) + 1;
        uint16_t m = (1u << l) - n;
        if (v < m) {
            bw.put(v, l - 1);
        } else {
            bw.put(m + ((v - m) >> 1), l - 1);
            bw.put((v - m) & 1, 1);
        }
    }

    void write_primitive_subexpfin(BitWriter& bw, uint16_t n, uint16_t k, uint16_t v)
    {
        int i = 0;
        int mk = 0;
        while (true) {
            int b = i ? k + i - 1 : k;
            int a = 1 << b;
            if (n <= mk + 3 * a) {
                write_primitive_quniform(bw, n - mk, v - mk);
                break;
            }
            int t = (v >= mk + a);
            bw.put(t, 1);
            if (t) {
                i += 1;
                mk += a;
            } else {
                bw.put(v - mk, b);
                break;
            }
        }
    }

    void write_primitive_refsubexpfin(BitWriter& bw, uint16_t n, uint16_t k, uint16_t ref, uint16_t v)
    {
        write_primitive_subexpfin(bw, n, k, recenter_finite_nonneg(n, ref, v));
    }

    void write_signed_primitive_refsubexpfin(BitWriter& bw, uint16_t n, uint16_t k, int16_t ref, int16_t v)
    {
        uint16_t scaled_n = (n << 1) - 1;
        ref += n - 1;
        v += n - 1;
        write_primitive_refsubexpfin(bw, scaled_n, k, ref, v);
    }

    int helper_tile_log2(int blk_size_v, int target)
    {
        int k = 0;
        while ((blk_size_v << k) < target) {
            k++;
        }
        return k;
    }

    void write_superres_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        if (s.enable_superres) {
            bw.put(f.use_superres, 1);
        }
        if (f.use_superres) {
            uint8_t coded = f.superres_denom - GST_AV1_SUPERRES_DENOM_MIN;
            bw.put(coded, GST_AV1_SUPERRES_DENOM_BITS);
        }
    }

    void write_frame_size(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        if (f.frame_size_override_flag) {
            bw.put(f.frame_width - 1, s.frame_width_bits_minus_1 + 1);
            bw.put(f.frame_height - 1, s.frame_height_bits_minus_1 + 1);
        }
        write_superres_params(bw, f, s);
    }

    void write_render_size(BitWriter& bw, const GstAV1FrameHeaderOBU& f)
    {
        bw.put(f.render_and_frame_size_different, 1);
        if (f.render_and_frame_size_different) {
            bw.put(f.render_width - 1, 16);
            bw.put(f.render_height - 1, 16);
        }
    }

    void write_tile_info(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        const GstAV1TileInfo& ti = f.tile_info;
        uint32_t mi_cols = 2 * ((f.frame_width + 7) >> 3);
        uint32_t mi_rows = 2 * ((f.frame_height + 7) >> 3);
        int sb_cols = s.use_128x128_superblock ? ((mi_cols + 31) >> 5) : ((mi_cols + 15) >> 4);
        int sb_rows = s.use_128x128_superblock ? ((mi_rows + 31) >> 5) : ((mi_rows + 15) >> 4);
        int sb_shift = s.use_128x128_superblock ? 5 : 4;
        int sb_size = sb_shift + 2;
        int max_tile_width_sb = 4096 >> sb_size;
        int max_tile_area_sb = (4096 * 2304) >> (2 * sb_size);
        int min_log2_tile_cols = helper_tile_log2(max_tile_width_sb, sb_cols);
        int max_log2_tile_cols = helper_tile_log2(1, std::min(sb_cols, GST_AV1_MAX_TILE_COLS));
        int max_log2_tile_rows = helper_tile_log2(1, std::min(sb_rows, GST_AV1_MAX_TILE_ROWS));
        int min_log2_tiles = std::max(min_log2_tile_cols, helper_tile_log2(max_tile_area_sb, sb_rows * sb_cols));

        bw.put(ti.uniform_tile_spacing_flag, 1);
        if (ti.uniform_tile_spacing_flag) {
            int ones = ti.tile_cols_log2 - min_log2_tile_cols;
            while (ones-- > 0) {
                bw.put(1, 1);
            }
            if (ti.tile_cols_log2 < max_log2_tile_cols) {
                bw.put(0, 1);
            }
            int min_log2_tile_rows = std::max(min_log2_tiles - ti.tile_cols_log2, 0);
            ones = ti.tile_rows_log2 - min_log2_tile_rows;
            while (ones-- > 0) {
                bw.put(1, 1);
            }
            if (ti.tile_rows_log2 < max_log2_tile_rows) {
                bw.put(0, 1);
            }
        } else {
            int widest_tile_sb = 0;
            int width_sb = sb_cols;
            for (int i = 0; i < ti.tile_cols; i++) {
                int size_sb = (ti.mi_col_starts[i + 1] - ti.mi_col_starts[i]) >> sb_shift;
                widest_tile_sb = std::max(size_sb, widest_tile_sb);
                write_uniform(bw, std::min(width_sb, max_tile_width_sb), size_sb - 1);
                width_sb -= size_sb;
            }
            int mta_sb = (min_log2_tiles > 0) ? ((sb_rows * sb_cols) >> (min_log2_tiles + 1)) : (sb_rows * sb_cols);
            int max_tile_height_sb = std::max(mta_sb / std::max(widest_tile_sb, 1), 1);
            int height_sb = sb_rows;
            for (int i = 0; i < ti.tile_rows; i++) {
                int size_sb = (ti.mi_row_starts[i + 1] - ti.mi_row_starts[i]) >> sb_shift;
                write_uniform(bw, std::min(height_sb, max_tile_height_sb), size_sb - 1);
                height_sb -= size_sb;
            }
        }
        if (ti.tile_cols_log2 > 0 || ti.tile_rows_log2 > 0) {
            bw.put(ti.context_update_tile_id, ti.tile_cols_log2 + ti.tile_rows_log2);
            bw.put(ti.tile_size_bytes_minus_1, 2);
        }
    }

    void write_quantization_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        const GstAV1QuantizationParams& q = f.quantization_params;
        bw.put(q.base_q_idx, 8);
        write_delta_q(bw, q.delta_q_y_dc);
        if (s.num_planes > 1) {
            if (s.color_config.separate_uv_delta_q) {
                bw.put(q.diff_uv_delta, 1);
            }
            write_delta_q(bw, q.delta_q_u_dc);
            write_delta_q(bw, q.delta_q_u_ac);
            if (q.diff_uv_delta) {
                write_delta_q(bw, q.delta_q_v_dc);
                write_delta_q(bw, q.delta_q_v_ac);
            }
        }
        bw.put(q.using_qmatrix, 1);
        if (q.using_qmatrix) {
            bw.put(q.qm_y, 4);
            bw.put(q.qm_u, 4);
            if (s.color_config.separate_uv_delta_q) {
                bw.put(q.qm_v, 4);
            }
        }
    }

    void write_delta_q_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f)
    {
        if (f.quantization_params.base_q_idx > 0) {
            bw.put(f.quantization_params.delta_q_present, 1);
        }
        if (f.quantization_params.delta_q_present) {
            bw.put(f.quantization_params.delta_q_res, 2);
        }
    }

    void write_delta_lf_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f)
    {
        if (f.quantization_params.delta_q_present) {
            if (!f.allow_intrabc) {
                bw.put(f.loop_filter_params.delta_lf_present, 1);
            }
            if (f.loop_filter_params.delta_lf_present) {
                bw.put(f.loop_filter_params.delta_lf_res, 2);
                bw.put(f.loop_filter_params.delta_lf_multi, 1);
            }
        }
    }

    void write_loop_filter_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        if (f.coded_lossless || f.allow_intrabc) {
            return;
        }
        const GstAV1LoopFilterParams& lf = f.loop_filter_params;
        bw.put(lf.loop_filter_level[0], 6);
        bw.put(lf.loop_filter_level[1], 6);
        if (s.num_planes > 1) {
            if (lf.loop_filter_level[0] || lf.loop_filter_level[1]) {
                bw.put(lf.loop_filter_level[2], 6);
                bw.put(lf.loop_filter_level[3], 6);
            }
        }
        bw.put(lf.loop_filter_sharpness, 3);
        bw.put(lf.loop_filter_delta_enabled, 1);
        if (lf.loop_filter_delta_enabled) {
            bw.put(lf.loop_filter_delta_update, 1);
            if (lf.loop_filter_delta_update) {
                static const int8_t kDefaultRefDeltas[] = { 1, 0, 0, 0, -1, 0, -1, -1 };
                for (int i = 0; i < GST_AV1_TOTAL_REFS_PER_FRAME; i++) {
                    bool update = lf.loop_filter_ref_deltas[i] != kDefaultRefDeltas[i];
                    bw.put(update, 1);
                    if (update) {
                        bw.put_signed(lf.loop_filter_ref_deltas[i], 7);
                    }
                }
                for (int i = 0; i < 2; i++) {
                    bool update = lf.loop_filter_mode_deltas[i] != 0;
                    bw.put(update, 1);
                    if (update) {
                        bw.put_signed(lf.loop_filter_mode_deltas[i], 7);
                    }
                }
            }
        }
    }

    void write_cdef_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        if (f.coded_lossless || f.allow_intrabc || !s.enable_cdef) {
            return;
        }
        const GstAV1CDEFParams& c = f.cdef_params;
        bw.put(c.cdef_damping - 3, 2);
        bw.put(c.cdef_bits, 2);
        for (int i = 0; i < (1 << c.cdef_bits); i++) {
            bw.put(c.cdef_y_pri_strength[i], 4);
            bw.put(c.cdef_y_sec_strength[i], 2);
            if (s.num_planes > 1) {
                bw.put(c.cdef_uv_pri_strength[i], 4);
                bw.put(c.cdef_uv_sec_strength[i], 2);
            }
        }
    }

    /* 5.9.20 lr_params -- the FIXED loop-restoration writer. */
    void write_loop_restoration_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        if (f.all_lossless || f.allow_intrabc || !s.enable_restoration) {
            return;
        }
        const GstAV1LoopRestorationParams& lr = f.loop_restoration_params;
        static const GstAV1FrameRestorationType remap[4] = { GST_AV1_FRAME_RESTORE_NONE,
            GST_AV1_FRAME_RESTORE_SWITCHABLE, GST_AV1_FRAME_RESTORE_WIENER, GST_AV1_FRAME_RESTORE_SGRPROJ };
        uint8_t use_chroma_lr = 0;
        for (int i = 0; i < s.num_planes; i++) {
            int j = 0;
            for (; j < 4; j++) {
                if (lr.frame_restoration_type[i] == remap[j]) {
                    break;
                }
            }
            if (lr.frame_restoration_type[i] != GST_AV1_FRAME_RESTORE_NONE && i > 1) {
                use_chroma_lr = 1;
            }
            bw.put(j, 2);
        }
        if (lr.uses_lr) {
            if (s.use_128x128_superblock) {
                bw.put(lr.lr_unit_shift - 1, 1);
            } else {
                /* Spec 5.9.20: lr_unit_shift = f(1); if set, lr_unit_extra_shift =
                 * f(1). Final value 0 -> "0", 1 -> "10", 2 -> "11". (The gst writer
                 * hardcodes the first bit to 1 -- this is the fix.) */
                if (lr.lr_unit_shift == 0) {
                    bw.put(0, 1);
                } else {
                    bw.put(1, 1);
                    bw.put(lr.lr_unit_shift - 1, 1);
                }
            }
            if (s.color_config.subsampling_x && s.color_config.subsampling_y && use_chroma_lr) {
                bw.put(lr.lr_uv_shift, 1);
            }
        }
    }

    void write_skip_mode_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f)
    {
        if (f.skip_mode_frame[0] > 0 || f.skip_mode_frame[1] > 0) {
            bw.put(f.skip_mode_present, 1);
        }
    }

    void write_global_motion_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f)
    {
        if (f.frame_is_intra) {
            return;
        }
        int32_t prev[GST_AV1_NUM_REF_FRAMES][6];
        if (f.primary_ref_frame != GST_AV1_PRIMARY_REF_NONE) {
            memcpy(prev, f.ref_global_motion_params.gm_params, sizeof(prev));
        } else {
            for (int ref = 0; ref < GST_AV1_NUM_REF_FRAMES; ref++) {
                for (int i = 0; i < 6; i++) {
                    prev[ref][i] = (i % 3 == 2) ? (1 << GST_AV1_WARPEDMODEL_PREC_BITS) : 0;
                }
            }
        }
        const GstAV1GlobalMotionParams& gm = f.global_motion_params;
        for (int ref = GST_AV1_REF_LAST_FRAME; ref <= GST_AV1_REF_ALTREF_FRAME; ref++) {
            bw.put(gm.gm_type[ref] != GST_AV1_WARP_MODEL_IDENTITY, 1);
            if (gm.gm_type[ref] != GST_AV1_WARP_MODEL_IDENTITY) {
                bw.put(gm.gm_type[ref] == GST_AV1_WARP_MODEL_ROTZOOM, 1);
                if (gm.gm_type[ref] != GST_AV1_WARP_MODEL_ROTZOOM) {
                    bw.put(gm.gm_type[ref] == GST_AV1_WARP_MODEL_TRANSLATION, 1);
                }
            }
            if (gm.gm_type[ref] >= GST_AV1_WARP_MODEL_ROTZOOM) {
                write_signed_primitive_refsubexpfin(bw, (1 << GST_AV1_GM_ABS_ALPHA_BITS) + 1, 3,
                    (prev[ref][2] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS))
                        - (1 << GST_AV1_GM_ALPHA_PREC_BITS),
                    (gm.gm_params[ref][2] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS))
                        - (1 << GST_AV1_GM_ALPHA_PREC_BITS));
                write_signed_primitive_refsubexpfin(bw, (1 << GST_AV1_GM_ABS_ALPHA_BITS) + 1, 3,
                    (prev[ref][3] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS)),
                    (gm.gm_params[ref][3] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS)));
            }
            if (gm.gm_type[ref] >= GST_AV1_WARP_MODEL_AFFINE) {
                write_signed_primitive_refsubexpfin(bw, (1 << GST_AV1_GM_ABS_ALPHA_BITS) + 1, 3,
                    (prev[ref][4] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS)),
                    (gm.gm_params[ref][4] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS)));
                write_signed_primitive_refsubexpfin(bw, (1 << GST_AV1_GM_ABS_ALPHA_BITS) + 1, 3,
                    (prev[ref][5] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS))
                        - (1 << GST_AV1_GM_ALPHA_PREC_BITS),
                    (gm.gm_params[ref][5] >> (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_ALPHA_PREC_BITS))
                        - (1 << GST_AV1_GM_ALPHA_PREC_BITS));
            }
            if (gm.gm_type[ref] >= GST_AV1_WARP_MODEL_TRANSLATION) {
                int trans_bits = (gm.gm_type[ref] == GST_AV1_WARP_MODEL_TRANSLATION)
                    ? GST_AV1_GM_ABS_TRANS_ONLY_BITS - !f.allow_high_precision_mv
                    : GST_AV1_GM_ABS_TRANS_BITS;
                int trans_prec_diff = (gm.gm_type[ref] == GST_AV1_WARP_MODEL_TRANSLATION)
                    ? GST_AV1_WARPEDMODEL_PREC_BITS - 3 + !f.allow_high_precision_mv
                    : (GST_AV1_WARPEDMODEL_PREC_BITS - GST_AV1_GM_TRANS_PREC_BITS);
                write_signed_primitive_refsubexpfin(bw, (1 << trans_bits) + 1, 3, (prev[ref][0] >> trans_prec_diff),
                    (gm.gm_params[ref][0] >> trans_prec_diff));
                write_signed_primitive_refsubexpfin(bw, (1 << trans_bits) + 1, 3, (prev[ref][1] >> trans_prec_diff),
                    (gm.gm_params[ref][1] >> trans_prec_diff));
            }
        }
    }

    void write_film_grain_params(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        const GstAV1FilmGrainParams& fg = f.film_grain_params;
        if (!s.film_grain_params_present || (!f.show_frame && !f.showable_frame)) {
            return;
        }
        bw.put(fg.apply_grain, 1);
        if (!fg.apply_grain) {
            return;
        }
        bw.put(fg.grain_seed, 16);
        if (f.frame_type == GST_AV1_INTER_FRAME) {
            bw.put(fg.update_grain, 1);
        }
        if (!fg.update_grain) {
            bw.put(fg.film_grain_params_ref_idx, 3);
            return;
        }
        bw.put(fg.num_y_points, 4);
        for (int i = 0; i < fg.num_y_points; i++) {
            bw.put(fg.point_y_value[i], 8);
            bw.put(fg.point_y_scaling[i], 8);
        }
        if (!s.color_config.mono_chrome) {
            bw.put(fg.chroma_scaling_from_luma, 1);
        }
        int num_cb = fg.num_cb_points;
        int num_cr = fg.num_cr_points;
        if (!(s.color_config.mono_chrome || fg.chroma_scaling_from_luma
                || (s.color_config.subsampling_x == 1 && s.color_config.subsampling_y == 1 && fg.num_y_points == 0))) {
            bw.put(fg.num_cb_points, 4);
            for (int i = 0; i < fg.num_cb_points; i++) {
                bw.put(fg.point_cb_value[i], 8);
                bw.put(fg.point_cb_scaling[i], 8);
            }
            bw.put(fg.num_cr_points, 4);
            for (int i = 0; i < fg.num_cr_points; i++) {
                bw.put(fg.point_cr_value[i], 8);
                bw.put(fg.point_cr_scaling[i], 8);
            }
        } else {
            num_cb = 0;
            num_cr = 0;
        }
        bw.put(fg.grain_scaling_minus_8, 2);
        bw.put(fg.ar_coeff_lag, 2);
        int num_pos_luma = 2 * fg.ar_coeff_lag * (fg.ar_coeff_lag + 1);
        int num_pos_chroma;
        if (fg.num_y_points) {
            num_pos_chroma = num_pos_luma + 1;
            for (int i = 0; i < num_pos_luma; i++) {
                bw.put(fg.ar_coeffs_y_plus_128[i], 8);
            }
        } else {
            num_pos_chroma = num_pos_luma;
        }
        if (fg.chroma_scaling_from_luma || num_cb) {
            for (int i = 0; i < num_pos_chroma; i++) {
                bw.put(fg.ar_coeffs_cb_plus_128[i], 8);
            }
        }
        if (fg.chroma_scaling_from_luma || num_cr) {
            for (int i = 0; i < num_pos_chroma; i++) {
                bw.put(fg.ar_coeffs_cr_plus_128[i], 8);
            }
        }
        bw.put(fg.ar_coeff_shift_minus_6, 2);
        bw.put(fg.grain_scale_shift, 2);
        if (num_cb) {
            bw.put(fg.cb_mult, 8);
            bw.put(fg.cb_luma_mult, 8);
            bw.put(fg.cb_offset, 9);
        }
        if (num_cr) {
            bw.put(fg.cr_mult, 8);
            bw.put(fg.cr_luma_mult, 8);
            bw.put(fg.cr_offset, 9);
        }
        bw.put(fg.overlap_flag, 1);
        bw.put(fg.clip_to_restricted_range, 1);
    }

    void write_uncompressed_frame_header(BitWriter& bw, const GstAV1FrameHeaderOBU& f, const GstAV1SequenceHeaderOBU& s)
    {
        int id_len = 0;
        if (s.frame_id_numbers_present_flag) {
            id_len = s.additional_frame_id_length_minus_1 + 1 + s.delta_frame_id_length_minus_2 + 2;
        }

        if (!s.reduced_still_picture_header) {
            bw.put(f.show_existing_frame, 1);
            if (f.show_existing_frame) {
                bw.put(f.frame_to_show_map_idx, 3);
                if (s.frame_id_numbers_present_flag) {
                    bw.put(f.display_frame_id, id_len);
                }
                return;
            }
            bw.put(f.frame_type, 2);
            bw.put(f.show_frame, 1);
            if (!f.show_frame) {
                bw.put(f.showable_frame, 1);
            }
            if (!(f.frame_type == GST_AV1_SWITCH_FRAME || (f.frame_type == GST_AV1_KEY_FRAME && f.show_frame))) {
                bw.put(f.error_resilient_mode, 1);
            }
        }

        bw.put(f.disable_cdf_update, 1);
        if (s.seq_force_screen_content_tools == GST_AV1_SELECT_SCREEN_CONTENT_TOOLS) {
            bw.put(f.allow_screen_content_tools, 1);
        }
        if (f.allow_screen_content_tools && s.seq_force_integer_mv == GST_AV1_SELECT_INTEGER_MV) {
            bw.put(f.force_integer_mv, 1);
        }
        if (s.frame_id_numbers_present_flag) {
            bw.put(f.current_frame_id, id_len);
        }
        if (f.frame_type != GST_AV1_SWITCH_FRAME && !s.reduced_still_picture_header) {
            bw.put(f.frame_size_override_flag, 1);
        }
        bw.put(f.order_hint, s.order_hint_bits_minus_1 + 1);
        if (!(f.frame_is_intra || f.error_resilient_mode)) {
            bw.put(f.primary_ref_frame, 3);
        }

        if (f.frame_type == GST_AV1_INTRA_ONLY_FRAME) {
            /* refresh_frame_flags != 0xff enforced by the caller. */
        }
        if (!(f.frame_type == GST_AV1_SWITCH_FRAME || (f.frame_type == GST_AV1_KEY_FRAME && f.show_frame))) {
            bw.put(f.refresh_frame_flags, 8);
        }

        if (!f.frame_is_intra || f.refresh_frame_flags != 0xFF) {
            if (f.error_resilient_mode && s.enable_order_hint) {
                for (int i = 0; i < GST_AV1_NUM_REF_FRAMES; i++) {
                    bw.put(f.ref_order_hint[i], s.order_hint_bits_minus_1 + 1);
                }
            }
        }

        if (f.frame_is_intra) {
            write_frame_size(bw, f, s);
            write_render_size(bw, f);
            if (f.allow_screen_content_tools && f.upscaled_width == f.frame_width) {
                bw.put(f.allow_intrabc, 1);
            }
        } else {
            if (s.enable_order_hint) {
                bw.put(f.frame_refs_short_signaling, 1);
                if (f.frame_refs_short_signaling) {
                    bw.put(f.last_frame_idx, 3);
                    bw.put(f.gold_frame_idx, 3);
                }
            }
            for (int i = 0; i < GST_AV1_REFS_PER_FRAME; i++) {
                if (!f.frame_refs_short_signaling) {
                    bw.put(f.ref_frame_idx[i], 3);
                }
                if (s.frame_id_numbers_present_flag) {
                    bw.put(0, s.delta_frame_id_length_minus_2 + 2);
                }
            }
            if (f.frame_size_override_flag && !f.error_resilient_mode) {
                for (int i = 0; i < GST_AV1_REFS_PER_FRAME; i++) {
                    bw.put(0, 1);
                }
            }
            write_frame_size(bw, f, s);
            write_render_size(bw, f);
            if (!f.force_integer_mv) {
                bw.put(f.allow_high_precision_mv, 1);
            }
            bw.put(f.is_filter_switchable, 1);
            if (!f.is_filter_switchable) {
                bw.put(f.interpolation_filter, 2);
            }
            bw.put(f.is_motion_mode_switchable, 1);
            if (!(f.error_resilient_mode || !s.enable_ref_frame_mvs)) {
                bw.put(f.use_ref_frame_mvs, 1);
            }
        }

        if (!(s.reduced_still_picture_header || f.disable_cdf_update)) {
            bw.put(f.disable_frame_end_update_cdf, 1);
        }

        write_tile_info(bw, f, s);
        write_quantization_params(bw, f, s);
        bw.put(f.segmentation_params.segmentation_enabled, 1); /* segmentation rejected upstream */
        write_delta_q_params(bw, f);
        write_delta_lf_params(bw, f);
        write_loop_filter_params(bw, f, s);
        write_cdef_params(bw, f, s);
        write_loop_restoration_params(bw, f, s);

        if (f.coded_lossless != 1) {
            bw.put(f.tx_mode == GST_AV1_TX_MODE_SELECT ? 1 : 0, 1);
        }
        if (!f.frame_is_intra) {
            bw.put(f.reference_select, 1);
        }
        write_skip_mode_params(bw, f);
        if (!(f.frame_is_intra || f.error_resilient_mode || !s.enable_warped_motion)) {
            bw.put(f.allow_warped_motion, 1);
        }
        bw.put(f.reduced_tx_set, 1);
        write_global_motion_params(bw, f);
        write_film_grain_params(bw, f, s);
    }

} // namespace

std::vector<uint8_t> serialize_frame_header_rbsp(const GstAV1SequenceHeaderOBU& seq, const GstAV1FrameHeaderOBU& frame)
{
    BitWriter bw;
    write_uncompressed_frame_header(bw, frame, seq);
    return bw.finish_rbsp();
}

void Av1Dpb::reset()
{
    slots = {};
}

void Av1AccessUnitBuilder::set_picture_parameters(const VADecPictureParameterBufferAV1& picture)
{
    picture_ = picture;
    has_picture_ = true;
}

void Av1AccessUnitBuilder::add_tile_parameters(std::span<const VASliceParameterBufferAV1> tiles)
{
    tile_params_.insert(tile_params_.end(), tiles.begin(), tiles.end());
}

void Av1AccessUnitBuilder::add_tile_data(std::span<const uint8_t> data)
{
    tile_data_.insert(tile_data_.end(), data.begin(), data.end());
}

void Av1AccessUnitBuilder::build_sequence_header(const VADecPictureParameterBufferAV1& pic)
{
    GstAV1SequenceHeaderOBU& s = seq_;
    s = {};

    s.seq_profile = static_cast<GstAV1Profile>(pic.profile);
    s.still_picture = pic.seq_info_fields.fields.still_picture;
    s.reduced_still_picture_header = 0;

    /* One operating point, no timing / decoder model / display-delay. A valid
     * seq_level_idx is required by the writer; use a high level (the stateful
     * decoder does not gate decode on it). */
    s.timing_info_present_flag = 0;
    s.decoder_model_info_present_flag = 0;
    s.initial_display_delay_present_flag = 0;
    s.operating_points_cnt_minus_1 = 0;
    s.operating_points[0].idc = 0;
    s.operating_points[0].seq_level_idx = 0; /* level 2.0: <= 3.3 so no seq_tier bit */
    s.operating_points[0].seq_tier = 0;

    const uint16_t max_w = pic.frame_width_minus1;
    const uint16_t max_h = pic.frame_height_minus1;
    s.max_frame_width_minus_1 = max_w;
    s.max_frame_height_minus_1 = max_h;
    s.frame_width_bits_minus_1 = static_cast<guint8>(floor_log2(max_w));
    s.frame_height_bits_minus_1 = static_cast<guint8>(floor_log2(max_h));

    s.frame_id_numbers_present_flag = 0;

    s.use_128x128_superblock = pic.seq_info_fields.fields.use_128x128_superblock;
    s.enable_filter_intra = pic.seq_info_fields.fields.enable_filter_intra;
    s.enable_intra_edge_filter = pic.seq_info_fields.fields.enable_intra_edge_filter;
    s.enable_interintra_compound = pic.seq_info_fields.fields.enable_interintra_compound;
    s.enable_masked_compound = pic.seq_info_fields.fields.enable_masked_compound;
    /* VA does not carry these seq flags; set them so the per-frame value VA
     * *does* carry is re-emitted and read back consistently. */
    s.enable_warped_motion = 1;
    s.enable_dual_filter = pic.seq_info_fields.fields.enable_dual_filter;
    s.enable_order_hint = pic.seq_info_fields.fields.enable_order_hint;
    s.enable_jnt_comp = pic.seq_info_fields.fields.enable_jnt_comp;
    s.enable_ref_frame_mvs = 1;

    s.seq_choose_screen_content_tools = 1; /* SELECT: frame writes allow_screen_content_tools */
    s.seq_force_screen_content_tools = GST_AV1_SELECT_SCREEN_CONTENT_TOOLS;
    s.seq_choose_integer_mv = 1; /* SELECT: frame writes force_integer_mv */
    s.seq_force_integer_mv = GST_AV1_SELECT_INTEGER_MV;

    if (s.enable_order_hint) {
        s.order_hint_bits_minus_1 = static_cast<gint8>(pic.order_hint_bits_minus_1);
        s.order_hint_bits = static_cast<guint8>(pic.order_hint_bits_minus_1 + 1);
    } else {
        s.order_hint_bits_minus_1 = -1; /* frame writes order_hint with 0 bits */
        s.order_hint_bits = 0;
    }

    s.enable_superres = 1;
    s.enable_cdef = pic.seq_info_fields.fields.enable_cdef;
    s.enable_restoration = 1;

    const int8_t bit_depth = bit_depth_from_idx(pic.bit_depth_idx);
    s.bit_depth = static_cast<guint8>(bit_depth);
    s.num_planes = pic.seq_info_fields.fields.mono_chrome ? 1 : 3;

    GstAV1ColorConfig& c = s.color_config;
    c.high_bitdepth = bit_depth > 8;
    c.twelve_bit = bit_depth == 12;
    c.mono_chrome = pic.seq_info_fields.fields.mono_chrome;
    /* VA carries only matrix_coefficients, not primaries/transfer. Leave the
     * colour description absent: it does not affect the decoded YUV samples. */
    c.color_description_present_flag = 0;
    c.color_primaries = GST_AV1_CP_UNSPECIFIED;
    c.transfer_characteristics = GST_AV1_TC_UNSPECIFIED;
    c.matrix_coefficients = static_cast<GstAV1MatrixCoefficients>(pic.matrix_coefficients);
    c.color_range = pic.seq_info_fields.fields.color_range;
    c.subsampling_x = pic.seq_info_fields.fields.subsampling_x;
    c.subsampling_y = pic.seq_info_fields.fields.subsampling_y;
    c.chroma_sample_position = GST_AV1_CSP_UNKNOWN;
    c.separate_uv_delta_q = 1;

    s.film_grain_params_present = pic.seq_info_fields.fields.film_grain_params_present;
}

void Av1AccessUnitBuilder::build_frame_header(
    const VADecPictureParameterBufferAV1& pic, uint8_t refresh_frame_flags, bool shown)
{
    const GstAV1SequenceHeaderOBU& s = seq_;
    GstAV1FrameHeaderOBU& f = frame_;
    f = {};

    const unsigned frame_type = pic.pic_info_fields.bits.frame_type;
    const bool frame_is_intra = (frame_type == AV1_KEY_FRAME || frame_type == AV1_INTRA_ONLY_FRAME);

    f.show_existing_frame = 0;
    f.frame_type = static_cast<GstAV1FrameType>(frame_type);
    f.frame_is_intra = frame_is_intra;
    /* Emit every reconstructed frame as shown (VA2c). The stateful V4L2 decoder
     * yields a CAPTURE frame only when the OBU stream DISPLAYS one (show_frame=1,
     * or a show_existing_frame), but a stateless-VA client (ffmpeg, Firefox)
     * drives one vaBeginPicture/EndPicture per CODED frame and expects the
     * decoded surface back for each -- it does its own display reordering. VA
     * never carries show_existing_frame, so a random-access stream's non-shown
     * alt-refs would be decoded but never output, and the client would stall
     * waiting for a surface the decoder is holding for a show_existing that never
     * comes (the codec drains after only the shown leaves). Marking every frame
     * shown makes the decoder emit exactly one output per decode op, in decode
     * order; show_frame does not affect the decoded samples, and the client
     * reorders for display. Low-delay / all-intra streams are already all-shown,
     * so this is a no-op there. showable_frame is then not coded (5.9.2). */
    f.show_frame = shown ? 1 : 0;
    /* A hidden DPB catch-up frame is the ONE frame we emit unshown: it re-decodes
     * a frame the client already has, purely to land it in the slots the encoder
     * refreshed (see the header). Nothing ever show_existing's it. */
    f.showable_frame = 0;
    f.error_resilient_mode = pic.pic_info_fields.bits.error_resilient_mode;
    f.disable_cdf_update = pic.pic_info_fields.bits.disable_cdf_update;
    f.allow_screen_content_tools = pic.pic_info_fields.bits.allow_screen_content_tools;
    f.force_integer_mv = pic.pic_info_fields.bits.force_integer_mv;
    f.order_hint = pic.order_hint;
    f.primary_ref_frame = pic.primary_ref_frame;

    const uint32_t upscaled_width = static_cast<uint32_t>(pic.frame_width_minus1) + 1;
    const uint32_t frame_height = static_cast<uint32_t>(pic.frame_height_minus1) + 1;
    f.upscaled_width = upscaled_width;
    f.frame_width = upscaled_width;
    f.frame_height = frame_height;
    f.render_and_frame_size_different = 0;
    f.render_width = upscaled_width;
    f.render_height = frame_height;
    f.use_superres = pic.pic_info_fields.bits.use_superres;
    f.superres_denom = f.use_superres ? pic.superres_scale_denominator : GST_AV1_SUPERRES_NUM;
    f.frame_size_override_flag = (pic.frame_width_minus1 != s.max_frame_width_minus_1
        || pic.frame_height_minus1 != s.max_frame_height_minus_1);

    f.allow_high_precision_mv = pic.pic_info_fields.bits.allow_high_precision_mv;
    f.is_motion_mode_switchable = pic.pic_info_fields.bits.is_motion_mode_switchable;
    f.use_ref_frame_mvs = pic.pic_info_fields.bits.use_ref_frame_mvs;
    f.disable_frame_end_update_cdf = pic.pic_info_fields.bits.disable_frame_end_update_cdf;
    f.allow_warped_motion = pic.pic_info_fields.bits.allow_warped_motion;
    f.reduced_tx_set = pic.mode_control_fields.bits.reduced_tx_set_used;
    f.allow_intrabc = pic.pic_info_fields.bits.allow_intrabc;

    f.interpolation_filter = static_cast<GstAV1InterpolationFilter>(pic.interp_filter);
    f.is_filter_switchable = (pic.interp_filter == GST_AV1_INTERPOLATION_FILTER_SWITCHABLE);
    f.frame_refs_short_signaling = 0;

    /* Reference resolution against the mirrored DPB (see update_dpb): map each
     * reference position to the slot in our DPB that holds the surface VA points
     * at, so ref_frame_idx resolves to the correct decoded frame. */
    for (int i = 0; i < GST_AV1_REFS_PER_FRAME; i++) {
        f.ref_frame_idx[i] = pic.ref_frame_idx[i];
    }
    if (!frame_is_intra) {
        for (int i = 0; i < GST_AV1_REFS_PER_FRAME; i++) {
            const uint8_t va_slot = pic.ref_frame_idx[i];
            VASurfaceID want = (va_slot < GST_AV1_NUM_REF_FRAMES) ? pic.ref_frame_map[va_slot] : VA_INVALID_SURFACE;
            int8_t found = -1;
            for (int j = 0; j < GST_AV1_NUM_REF_FRAMES; j++) {
                if (dpb_.slots[j].valid && dpb_.slots[j].surface == want) {
                    found = static_cast<int8_t>(j);
                    break;
                }
            }
            f.ref_frame_idx[i] = (found >= 0) ? found : static_cast<gint8>(va_slot);
            if (found < 0 && getenv("AV1DBG")) {
                fprintf(stderr, "REMAP-MISS oh=%u ref[%d] wants surface %u not in DPB\n", pic.order_hint, i, want);
            }
        }
    }

    /* ref_order_hint / order hints from the mirrored DPB. */
    for (int i = 0; i < GST_AV1_NUM_REF_FRAMES; i++) {
        f.ref_order_hint[i] = dpb_.slots[i].order_hint;
    }

    /* refresh_frame_flags is not carried by VA; the caller supplies the mask it
     * chose (an immediate heuristic, or the VA2c one-frame-lookahead
     * reconstruction). A shown key frame implies 0xFF regardless. */
    f.refresh_frame_flags
        = (frame_type == AV1_KEY_FRAME && f.show_frame) ? static_cast<uint8_t>(0xFF) : refresh_frame_flags;

    /* Quantization. */
    GstAV1QuantizationParams& q = f.quantization_params;
    q.base_q_idx = pic.base_qindex;
    q.delta_q_y_dc = pic.y_dc_delta_q;
    q.delta_q_u_dc = pic.u_dc_delta_q;
    q.delta_q_u_ac = pic.u_ac_delta_q;
    q.delta_q_v_dc = pic.v_dc_delta_q;
    q.delta_q_v_ac = pic.v_ac_delta_q;
    q.diff_uv_delta = (pic.u_dc_delta_q != pic.v_dc_delta_q || pic.u_ac_delta_q != pic.v_ac_delta_q);
    q.using_qmatrix = pic.qmatrix_fields.bits.using_qmatrix;
    q.qm_y = pic.qmatrix_fields.bits.qm_y;
    q.qm_u = pic.qmatrix_fields.bits.qm_u;
    q.qm_v = pic.qmatrix_fields.bits.qm_v;
    q.delta_q_present = pic.mode_control_fields.bits.delta_q_present_flag;
    q.delta_q_res = pic.mode_control_fields.bits.log2_delta_q_res;

    /* CodedLossless / AllLossless (5.9.12) -- with segmentation rejected below,
     * the only segment is segment 0. */
    const bool seg0_lossless = (q.base_q_idx == 0 && q.delta_q_y_dc == 0 && q.delta_q_u_dc == 0 && q.delta_q_u_ac == 0
        && q.delta_q_v_dc == 0 && q.delta_q_v_ac == 0);
    f.coded_lossless = seg0_lossless ? 1 : 0;
    f.all_lossless = (f.coded_lossless && f.frame_width == f.upscaled_width) ? 1 : 0;

    /* Segmentation: the GStreamer writer does not support it. */
    f.segmentation_params.segmentation_enabled = pic.seg_info.segment_info_fields.bits.enabled;

    /* Loop filter. */
    GstAV1LoopFilterParams& lf = f.loop_filter_params;
    lf.loop_filter_level[0] = pic.filter_level[0];
    lf.loop_filter_level[1] = pic.filter_level[1];
    lf.loop_filter_level[2] = pic.filter_level_u;
    lf.loop_filter_level[3] = pic.filter_level_v;
    lf.loop_filter_sharpness = pic.loop_filter_info_fields.bits.sharpness_level;
    lf.loop_filter_delta_enabled = pic.loop_filter_info_fields.bits.mode_ref_delta_enabled;
    lf.loop_filter_delta_update = pic.loop_filter_info_fields.bits.mode_ref_delta_update;
    for (int i = 0; i < GST_AV1_TOTAL_REFS_PER_FRAME; i++) {
        lf.loop_filter_ref_deltas[i] = pic.ref_deltas[i];
    }
    lf.loop_filter_mode_deltas[0] = pic.mode_deltas[0];
    lf.loop_filter_mode_deltas[1] = pic.mode_deltas[1];
    lf.delta_lf_present = pic.mode_control_fields.bits.delta_lf_present_flag;
    lf.delta_lf_res = pic.mode_control_fields.bits.log2_delta_lf_res;
    lf.delta_lf_multi = pic.mode_control_fields.bits.delta_lf_multi;

    /* CDEF. */
    GstAV1CDEFParams& cdef = f.cdef_params;
    cdef.cdef_damping = pic.cdef_damping_minus_3 + 3;
    cdef.cdef_bits = pic.cdef_bits;
    for (int i = 0; i < (1 << pic.cdef_bits) && i < GST_AV1_CDEF_MAX; i++) {
        cdef.cdef_y_pri_strength[i] = pic.cdef_y_strengths[i] >> 2;
        cdef.cdef_y_sec_strength[i] = pic.cdef_y_strengths[i] & 0x03;
        cdef.cdef_uv_pri_strength[i] = pic.cdef_uv_strengths[i] >> 2;
        cdef.cdef_uv_sec_strength[i] = pic.cdef_uv_strengths[i] & 0x03;
    }

    /* Loop restoration: VA stores the already-remapped FrameRestorationType,
     * whose values match GstAV1FrameRestorationType. */
    GstAV1LoopRestorationParams& lr = f.loop_restoration_params;
    lr.frame_restoration_type[0]
        = static_cast<GstAV1FrameRestorationType>(pic.loop_restoration_fields.bits.yframe_restoration_type);
    lr.frame_restoration_type[1]
        = static_cast<GstAV1FrameRestorationType>(pic.loop_restoration_fields.bits.cbframe_restoration_type);
    lr.frame_restoration_type[2]
        = static_cast<GstAV1FrameRestorationType>(pic.loop_restoration_fields.bits.crframe_restoration_type);
    lr.lr_unit_shift = pic.loop_restoration_fields.bits.lr_unit_shift;
    lr.lr_uv_shift = pic.loop_restoration_fields.bits.lr_uv_shift;
    lr.uses_lr = (lr.frame_restoration_type[0] != GST_AV1_FRAME_RESTORE_NONE
        || lr.frame_restoration_type[1] != GST_AV1_FRAME_RESTORE_NONE
        || lr.frame_restoration_type[2] != GST_AV1_FRAME_RESTORE_NONE);

    /* Tiles. */
    GstAV1TileInfo& ti = f.tile_info;
    ti.uniform_tile_spacing_flag = pic.pic_info_fields.bits.uniform_tile_spacing_flag;
    ti.tile_cols = std::max<uint8_t>(1, pic.tile_cols);
    ti.tile_rows = std::max<uint8_t>(1, pic.tile_rows);
    ti.tile_cols_log2 = static_cast<guint8>(tile_log2(1, ti.tile_cols));
    ti.tile_rows_log2 = static_cast<guint8>(tile_log2(1, ti.tile_rows));
    ti.context_update_tile_id = pic.context_update_tile_id;
    {
        /* tile_size_bytes for the multi-tile group: enough to hold the largest
         * tile's size minus one. */
        uint32_t max_tile = 0;
        for (const auto& t : tile_params_) {
            max_tile = std::max(max_tile, t.slice_data_size);
        }
        unsigned bytes = 1;
        if (max_tile > 1) {
            bytes = (floor_log2(max_tile - 1) / 8) + 1;
        }
        ti.tile_size_bytes = bytes;
        ti.tile_size_bytes_minus_1 = static_cast<int>(bytes) - 1;
    }
    if (!ti.uniform_tile_spacing_flag) {
        const int sb_shift = s.use_128x128_superblock ? 5 : 4;
        uint32_t mi_cols = 2 * ((f.frame_width + 7) >> 3);
        uint32_t mi_rows = 2 * ((f.frame_height + 7) >> 3);
        uint32_t start = 0;
        for (int i = 0; i < ti.tile_cols; i++) {
            ti.mi_col_starts[i] = start;
            ti.width_in_sbs_minus_1[i] = pic.width_in_sbs_minus_1[i];
            start += (static_cast<uint32_t>(pic.width_in_sbs_minus_1[i]) + 1) << sb_shift;
        }
        ti.mi_col_starts[ti.tile_cols] = mi_cols;
        start = 0;
        for (int i = 0; i < ti.tile_rows; i++) {
            ti.mi_row_starts[i] = start;
            ti.height_in_sbs_minus_1[i] = pic.height_in_sbs_minus_1[i];
            start += (static_cast<uint32_t>(pic.height_in_sbs_minus_1[i]) + 1) << sb_shift;
        }
        ti.mi_row_starts[ti.tile_rows] = mi_rows;
    }

    f.tx_mode = static_cast<GstAV1TXModes>(pic.mode_control_fields.bits.tx_mode);
    f.reference_select = pic.mode_control_fields.bits.reference_select;
    f.skip_mode_present = pic.mode_control_fields.bits.skip_mode_present;

    /* Global motion (inter only). */
    for (int ref = GST_AV1_REF_LAST_FRAME; ref <= GST_AV1_REF_ALTREF_FRAME; ref++) {
        const VAWarpedMotionParamsAV1& wm = pic.wm[ref - GST_AV1_REF_LAST_FRAME];
        f.global_motion_params.gm_type[ref] = static_cast<GstAV1WarpModelType>(wm.wmtype);
        f.global_motion_params.invalid[ref] = wm.invalid;
        for (int j = 0; j < 6; j++) {
            f.global_motion_params.gm_params[ref][j] = wm.wmmat[j];
        }
    }
    /* Global-motion delta base: the primary reference's saved params. */
    if (!frame_is_intra && f.primary_ref_frame != GST_AV1_PRIMARY_REF_NONE) {
        const uint8_t slot = f.ref_frame_idx[f.primary_ref_frame];
        if (slot < GST_AV1_NUM_REF_FRAMES) {
            for (int ref = 0; ref < GST_AV1_NUM_REF_FRAMES; ref++) {
                for (int j = 0; j < 6; j++) {
                    f.ref_global_motion_params.gm_params[ref][j] = dpb_.slots[slot].gm_params[ref][j];
                }
            }
        }
    }

    /* skip_mode_frame (5.9.22): its presence must match skipModeAllowed so the
     * writer emits skip_mode_present iff the decoder will read it. */
    compute_skip_mode_frame(pic);

    /* Film grain. */
    if (s.film_grain_params_present && pic.film_grain_info.film_grain_info_fields.bits.apply_grain) {
        GstAV1FilmGrainParams& fg = f.film_grain_params;
        const VAFilmGrainStructAV1& v = pic.film_grain_info;
        fg.apply_grain = 1;
        fg.grain_seed = v.grain_seed;
        fg.update_grain = 1;
        fg.num_y_points = v.num_y_points;
        for (int i = 0; i < v.num_y_points && i < 14; i++) {
            fg.point_y_value[i] = v.point_y_value[i];
            fg.point_y_scaling[i] = v.point_y_scaling[i];
        }
        fg.chroma_scaling_from_luma = v.film_grain_info_fields.bits.chroma_scaling_from_luma;
        fg.num_cb_points = v.num_cb_points;
        for (int i = 0; i < v.num_cb_points && i < 10; i++) {
            fg.point_cb_value[i] = v.point_cb_value[i];
            fg.point_cb_scaling[i] = v.point_cb_scaling[i];
        }
        fg.num_cr_points = v.num_cr_points;
        for (int i = 0; i < v.num_cr_points && i < 10; i++) {
            fg.point_cr_value[i] = v.point_cr_value[i];
            fg.point_cr_scaling[i] = v.point_cr_scaling[i];
        }
        fg.grain_scaling_minus_8 = v.film_grain_info_fields.bits.grain_scaling_minus_8;
        fg.ar_coeff_lag = v.film_grain_info_fields.bits.ar_coeff_lag;
        for (int i = 0; i < 24; i++) {
            fg.ar_coeffs_y_plus_128[i] = static_cast<guint8>(v.ar_coeffs_y[i] + 128);
        }
        for (int i = 0; i < 25; i++) {
            fg.ar_coeffs_cb_plus_128[i] = static_cast<guint8>(v.ar_coeffs_cb[i] + 128);
            fg.ar_coeffs_cr_plus_128[i] = static_cast<guint8>(v.ar_coeffs_cr[i] + 128);
        }
        fg.ar_coeff_shift_minus_6 = v.film_grain_info_fields.bits.ar_coeff_shift_minus_6;
        fg.grain_scale_shift = v.film_grain_info_fields.bits.grain_scale_shift;
        fg.cb_mult = v.cb_mult;
        fg.cb_luma_mult = v.cb_luma_mult;
        fg.cb_offset = v.cb_offset;
        fg.cr_mult = v.cr_mult;
        fg.cr_luma_mult = v.cr_luma_mult;
        fg.cr_offset = v.cr_offset;
        fg.overlap_flag = v.film_grain_info_fields.bits.overlap_flag;
        fg.clip_to_restricted_range = v.film_grain_info_fields.bits.clip_to_restricted_range;
    }
}

int Av1AccessUnitBuilder::get_relative_dist(int a, int b) const
{
    if (!seq_.enable_order_hint) {
        return 0;
    }
    int diff = a - b;
    int m = 1 << (seq_.order_hint_bits - 1);
    diff = (diff & (m - 1)) - (diff & m);
    return diff;
}

void Av1AccessUnitBuilder::compute_skip_mode_frame(const VADecPictureParameterBufferAV1& pic)
{
    GstAV1FrameHeaderOBU& f = frame_;
    f.skip_mode_frame[0] = 0;
    f.skip_mode_frame[1] = 0;

    if (f.frame_is_intra || !f.reference_select || !seq_.enable_order_hint) {
        return;
    }

    const int order_hint = static_cast<int>(pic.order_hint);
    int forward_idx = -1, backward_idx = -1;
    int forward_hint = 0, backward_hint = 0;
    for (int i = 0; i < GST_AV1_REFS_PER_FRAME; i++) {
        const int ref_hint = static_cast<int>(dpb_.slots[f.ref_frame_idx[i]].order_hint);
        if (get_relative_dist(ref_hint, order_hint) < 0) {
            if (forward_idx < 0 || get_relative_dist(ref_hint, forward_hint) > 0) {
                forward_idx = i;
                forward_hint = ref_hint;
            }
        } else if (get_relative_dist(ref_hint, order_hint) > 0) {
            if (backward_idx < 0 || get_relative_dist(ref_hint, backward_hint) < 0) {
                backward_idx = i;
                backward_hint = ref_hint;
            }
        }
    }

    if (forward_idx < 0) {
        return;
    }
    if (backward_idx >= 0) {
        f.skip_mode_frame[0] = GST_AV1_REF_LAST_FRAME + std::min(forward_idx, backward_idx);
        f.skip_mode_frame[1] = GST_AV1_REF_LAST_FRAME + std::max(forward_idx, backward_idx);
        return;
    }
    int second_forward_idx = -1, second_forward_hint = 0;
    for (int i = 0; i < GST_AV1_REFS_PER_FRAME; i++) {
        const int ref_hint = static_cast<int>(dpb_.slots[f.ref_frame_idx[i]].order_hint);
        if (get_relative_dist(ref_hint, forward_hint) < 0) {
            if (second_forward_idx < 0 || get_relative_dist(ref_hint, second_forward_hint) > 0) {
                second_forward_idx = i;
                second_forward_hint = ref_hint;
            }
        }
    }
    if (second_forward_idx >= 0) {
        f.skip_mode_frame[0] = GST_AV1_REF_LAST_FRAME + std::min(forward_idx, second_forward_idx);
        f.skip_mode_frame[1] = GST_AV1_REF_LAST_FRAME + std::max(forward_idx, second_forward_idx);
    }
}

uint8_t Av1AccessUnitBuilder::choose_refresh_flags(const VADecPictureParameterBufferAV1& pic) const
{
    /*
     * BEST EFFORT ONLY -- used when there is NO successor frame to derive the
     * exact mask from (flush(): the end-of-stream tail, or a client that syncs a
     * frame before decoding the next; and the immediate finish() test API).
     * While a successor exists the builder derives the mask exactly instead
     * (derive_refresh_frame_flags), which is what keeps the mirrored DPB equal to
     * the encoder's.
     *
     * Here VA does not (yet) tell us which slot this frame took, so we choose one
     * ourselves and remap ref_frame_idx into our mirrored DPB. The
     * invariant that keeps every future reference resolvable is: our DPB must
     * contain every surface still live in VA's reference map. We therefore
     * refresh ONE slot (matching the encoder's usual single-slot pattern),
     * never one the current frame references, choosing in priority order:
     *   (1) an invalid/empty slot;
     *   (2) a dead slot (surface no longer in VA's ref map);
     *   (3) the least-recently-used live slot (LRU) -- consumed leaf frames age
     *       out while long-lived anchors (the key frame, GOP alt-refs) stay.
     * Evicting a still-live surface (3) is only forced when the 8-slot DPB is
     * full of distinct live surfaces; LRU is the heuristic that best matches a
     * hierarchical B-pyramid without a one-frame lookahead.
     */
    auto in_map = [&](VASurfaceID id) {
        for (int j = 0; j < GST_AV1_NUM_REF_FRAMES; j++) {
            if (pic.ref_frame_map[j] == id) {
                return true;
            }
        }
        return false;
    };

    /* Refresh every dead slot (surface no longer live in VA's ref map) with the
     * current frame, so it stays maximally available for later references while
     * every VA-live surface -- held only in non-dead slots -- is preserved. */
    uint8_t mask = 0;
    for (int j = 0; j < GST_AV1_NUM_REF_FRAMES; j++) {
        if (!dpb_.slots[j].valid || !in_map(dpb_.slots[j].surface)) {
            mask |= static_cast<uint8_t>(1u << j);
        }
    }
    if (mask != 0) {
        return mask;
    }
    /*
     * The 8-slot DPB is full of distinct VA-live surfaces and no successor is
     * available, so evict the oldest by order hint. Reaching this line at all
     * means a random-access frame was flushed without its successor (sync before
     * the next decode); the derived path never does.
     */
    int oldest = 0;
    for (int j = 1; j < GST_AV1_NUM_REF_FRAMES; j++) {
        if (get_relative_dist(
                static_cast<int>(dpb_.slots[j].order_hint), static_cast<int>(dpb_.slots[oldest].order_hint))
            < 0) {
            oldest = j;
        }
    }
    if (getenv("AV1DBG")) {
        fprintf(stderr, "REFRESH-FULL oh=%u full DPB, evicting oldest slot %d (oh=%u)\n", pic.order_hint, oldest,
            dpb_.slots[oldest].order_hint);
    }
    return static_cast<uint8_t>(1u << oldest);
}

void Av1AccessUnitBuilder::update_dpb(const VADecPictureParameterBufferAV1& pic)
{
    const unsigned frame_type = pic.pic_info_fields.bits.frame_type;
    if (frame_type == AV1_KEY_FRAME && frame_.show_frame) {
        dpb_.reset();
    }
    for (int j = 0; j < GST_AV1_NUM_REF_FRAMES; j++) {
        if (frame_.refresh_frame_flags & (1u << j)) {
            dpb_.slots[j].surface = pic.current_frame;
            dpb_.slots[j].order_hint = pic.order_hint;
            dpb_.slots[j].valid = true;
            for (int ref = 0; ref < GST_AV1_NUM_REF_FRAMES; ref++) {
                for (int k = 0; k < 6; k++) {
                    dpb_.slots[j].gm_params[ref][k] = frame_.global_motion_params.gm_params[ref][k];
                }
            }
        }
    }
}

std::vector<uint8_t> Av1AccessUnitBuilder::assemble_au(const VADecPictureParameterBufferAV1& pic,
    std::span<const VASliceParameterBufferAV1> tiles, std::span<const uint8_t> data, uint8_t refresh_frame_flags,
    bool shown, bool bare)
{
    if (pic.seg_info.segment_info_fields.bits.enabled) {
        throw std::runtime_error("AV1 segmentation is not supported by the GStreamer bit writer (VA2b limitation)");
    }

    const unsigned frame_type = pic.pic_info_fields.bits.frame_type;
    const bool is_key = (frame_type == AV1_KEY_FRAME);

    /* (Re)build the sequence header on the first frame / on a key frame. */
    if (!have_sequence_ || is_key) {
        build_sequence_header(pic);
        have_sequence_ = true;
    }
    build_frame_header(pic, refresh_frame_flags, shown);

    std::vector<uint8_t> au;
    if (!bare) {
        /* temporal_delimiter OBU. */
        au = write_gst_obu(
            [](guint8* d, guint* size) { return gst_av1_bit_writer_temporal_delimiter_obu(TRUE, d, size); });

        /* sequence_header OBU: emit on change (deduped like the H.264 SPS). */
        std::vector<uint8_t> seq_obu = write_gst_obu(
            [&](guint8* d, guint* size) { return gst_av1_bit_writer_sequence_header_obu(&seq_, TRUE, d, size); });
        if (seq_obu != last_emitted_sequence_header_) {
            au.insert(au.end(), seq_obu.begin(), seq_obu.end());
            last_emitted_sequence_header_ = seq_obu;
        }

        /* A hidden DPB catch-up frame waiting for a temporal unit rides in this
         * one, ahead of the frame that may reference it. A temporal unit may
         * carry several coded frames as long as exactly one of them is shown
         * (5.6); the catch-up frame is the unshown one. */
        if (!catch_up_.empty()) {
            au.insert(au.end(), catch_up_.begin(), catch_up_.end());
            catch_up_.clear();
        }
    }

    /* frame_header OBU: serialised here (not by the gst writer) to fix the gst
     * loop_restoration lr_unit_shift bug (see serialize_frame_header_rbsp). */
    std::vector<uint8_t> frame_rbsp = serialize_frame_header_rbsp(seq_, frame_);
    std::vector<uint8_t> frame_obu = wrap_obu(GST_AV1_OBU_FRAME_HEADER, frame_rbsp);
    au.insert(au.end(), frame_obu.begin(), frame_obu.end());

    /* tile_group OBU (hand-wrapped). */
    std::vector<uint8_t> tg_payload = build_tile_group_payload(
        tiles, data, frame_.tile_info.tile_cols, frame_.tile_info.tile_rows, frame_.tile_info.tile_size_bytes);
    std::vector<uint8_t> tg_obu = wrap_obu(GST_AV1_OBU_TILE_GROUP, tg_payload);
    au.insert(au.end(), tg_obu.begin(), tg_obu.end());

    return au;
}

std::vector<uint8_t> Av1AccessUnitBuilder::finish()
{
    if (!has_picture_) {
        reset_picture();
        return {};
    }

    const unsigned frame_type = picture_.pic_info_fields.bits.frame_type;
    const bool shown_key = (frame_type == AV1_KEY_FRAME) && picture_.pic_info_fields.bits.show_frame;
    const uint8_t refresh = shown_key ? static_cast<uint8_t>(0xFF) : choose_refresh_flags(picture_);

    std::vector<uint8_t> au;
    try {
        au = assemble_au(picture_, tile_params_, tile_data_, refresh);
    } catch (...) {
        reset_picture();
        throw;
    }
    update_dpb(picture_);
    reset_picture();
    return au;
}

uint8_t derive_refresh_frame_flags(VASurfaceID current_frame, const VASurfaceID next_ref_frame_map[8])
{
    /*
     * THE exact derivation (5.9.2 decode_frame_wrapup). After frame N decodes,
     * the decoder sets RefFrameMap[i] = current_frame for every i in
     * refresh_frame_flags and leaves the other slots untouched. Frame N+1 is the
     * next frame in DECODE order, and VA hands it that very map (ref_frame_map,
     * the DPB BEFORE N+1 updates it), so:
     *
     *   refresh_frame_flags(N) = OR{ 1 << i : ref_frame_map_{N+1}[i] == current_frame_N }
     *
     * A surface only appears in the map by being written there, and a frame's own
     * surface cannot be live in the map it is handed (a client never decodes into
     * a picture its own DPB still holds), so every set bit is one this frame
     * wrote. A mask of 0 is a legitimate answer: a never-referenced leaf of a
     * B-pyramid refreshes nothing.
     */
    uint8_t mask = 0;
    for (int i = 0; i < GST_AV1_NUM_REF_FRAMES; i++) {
        if (next_ref_frame_map[i] == current_frame) {
            mask |= static_cast<uint8_t>(1u << i);
        }
    }
    return mask;
}

Av1AccessUnitBuilder::PictureResult Av1AccessUnitBuilder::submit_picture(uint64_t tag)
{
    PictureResult result;

    /* 0. A frame flush() had to emit before its mask could be derived went out
     * with refresh_frame_flags = 0, so it is NOT in the codec's DPB. This frame's
     * ref_frame_map now says which slots the encoder put it in: re-submit it as a
     * HIDDEN catch-up frame (same bytes, so the same picture and CDFs) spliced
     * into the next temporal unit, ahead of any frame that references it. A mask
     * of 0 means the encoder kept it out of the DPB too -- nothing to do -- and a
     * key frame resets the DPB, which makes the catch-up pointless. */
    if (provisional_) {
        if (has_picture_) {
            const uint8_t refresh = derive_refresh_frame_flags(provisional_->pic.current_frame, picture_.ref_frame_map);
            const bool key_resets = (picture_.pic_info_fields.bits.frame_type == AV1_KEY_FRAME);
            if (refresh != 0 && !key_resets) {
                catch_up_ = assemble_au(provisional_->pic, provisional_->tiles, provisional_->data, refresh,
                    /*shown=*/false, /*bare=*/true);
                update_dpb(provisional_->pic);
            }
        }
        provisional_.reset();
    }

    /* 1. A frame held for the lookahead: this frame's ref_frame_map is the DPB
     * after the held frame updated it, so it pins down what the held frame
     * refreshed. Build and release it before touching the current frame so it is
     * never lost (even if the current frame fails). */
    if (pending_) {
        const uint8_t refresh = has_picture_
            ? derive_refresh_frame_flags(pending_->pic.current_frame, picture_.ref_frame_map) /* EXACT */
            : choose_refresh_flags(pending_->pic); /* no successor: best effort */
        std::vector<uint8_t> au = assemble_au(pending_->pic, pending_->tiles, pending_->data, refresh);
        update_dpb(pending_->pic);
        result.ready.push_back({ pending_->tag, std::move(au), refresh, pending_->pic.order_hint });
        pending_.reset();
    }

    if (!has_picture_) {
        return result;
    }

    /* 2. The current frame. */
    if (picture_.seg_info.segment_info_fields.bits.enabled) {
        reset_picture();
        result.current_failed = true;
        return result;
    }

    const unsigned frame_type = picture_.pic_info_fields.bits.frame_type;
    const bool is_key = (frame_type == AV1_KEY_FRAME);

    if (!is_key) {
        /* Hold it: refresh_frame_flags is only EXACT once the next frame's
         * ref_frame_map arrives, and choosing one now (the old heuristic) writes
         * this frame into slots the encoder never wrote -- which is what made the
         * re-synthesised stream invalid AV1. Its surface is already bound to
         * `tag`; flush() emits it if no successor ever comes. A key frame needs no
         * successor: a shown key refreshes all 8 slots by definition. */
        PendingFrame pf;
        pf.pic = picture_;
        pf.tiles.assign(tile_params_.begin(), tile_params_.end());
        pf.data.assign(tile_data_.begin(), tile_data_.end());
        pf.tag = tag;
        pending_ = std::move(pf);
        reset_picture();
        result.current_deferred = true;
        return result;
    }

    const bool shown_key = is_key && picture_.pic_info_fields.bits.show_frame;
    const uint8_t refresh = shown_key ? static_cast<uint8_t>(0xFF) : choose_refresh_flags(picture_);
    std::vector<uint8_t> au = assemble_au(picture_, tile_params_, tile_data_, refresh);
    update_dpb(picture_);
    result.ready.push_back({ tag, std::move(au), refresh, picture_.order_hint });
    reset_picture();
    return result;
}

std::vector<Av1AccessUnitBuilder::ReadyFrame> Av1AccessUnitBuilder::flush()
{
    std::vector<ReadyFrame> out;
    if (pending_) {
        /* No successor yet, so the mask cannot be derived. Emit the frame
         * PROVISIONALLY with refresh_frame_flags = 0 -- the only mask that cannot
         * evict a live picture -- and keep it: the next picture derives its true
         * mask and re-submits it as a hidden DPB catch-up frame. At end of stream
         * no successor ever comes, and then 0 is also the right answer (nothing
         * can reference the tail frame). */
        std::vector<uint8_t> au = assemble_au(pending_->pic, pending_->tiles, pending_->data, 0);
        update_dpb(pending_->pic);
        out.push_back({ pending_->tag, std::move(au), 0, pending_->pic.order_hint });
        provisional_ = std::move(*pending_);
        pending_.reset();
    }
    return out;
}

void Av1AccessUnitBuilder::reset_picture()
{
    has_picture_ = false;
    picture_ = {};
    tile_data_.clear();
    tile_params_.clear();
}

} // namespace stateful
