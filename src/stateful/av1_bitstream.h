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

#include <array>
#include <cstdint>
#include <span>
#include <vector>

extern "C" {
#include <va/va.h>
#include <va/va_dec_av1.h>
}

#include "gstav1bitwriter_compat.h"

namespace stateful {

/*
 * AV1 OBU synthesis for the stateful decode path (VPU_DESIGN.md 7.6; VA2b of
 * VA2-survey (C), "AV1 -- must rebuild OBU bitstream"). Unlike VP9 (whose whole
 * frame is in-band in the slice-data buffer and is forwarded verbatim), a
 * VA-API AV1 decoder is handed the ALREADY-PARSED sequence and frame headers as
 * a VADecPictureParameterBufferAV1, and only the raw tile (entropy) payloads in
 * the slice-data buffer -- the OBU framing has been stripped by the host
 * (ffmpeg cbs_av1 / Firefox). A stateful V4L2_PIX_FMT_AV1 decoder
 * (c2.qti.av1.decoder) wants the original temporal-unit OBU stream, so every
 * vaEndPicture must REBUILD it:
 *
 *   temporal_delimiter OBU
 *   [ sequence_header OBU ]   (first frame / on change; deduped like h264 SPS)
 *   frame_header OBU          (the uncompressed_header(), re-synthesised)
 *   tile_group OBU            (the tile payloads, with per-tile size fields)
 *
 * The two headers are serialised with the GStreamer AV1 bit writer
 * (gst_av1_bit_writer_sequence_header_obu / _frame_header_obu, exported by
 * libgstcodecparsers since 1.24 -- the same library the H.264 path uses), by
 * filling a GstAV1SequenceHeaderOBU / GstAV1FrameHeaderOBU from the VA buffer.
 * The temporal delimiter comes from the writer too; the tile_group OBU wrapper
 * (OBU header + LEB128 size + tile size fields) is written here (5.11.1).
 *
 * The VA AV1 picture buffer does not carry every field the OBU syntax needs
 * (it was designed for stateless accelerators that also get the reference
 * surfaces): the missing SEQUENCE flags (enable_superres / enable_restoration /
 * enable_warped_motion / enable_ref_frame_mvs) are reconstructed as SELECT/"1"
 * so the frame header re-emits the corresponding per-frame bit VA *does* carry
 * (self-consistent: our seq says "may be present", our frame writes the value,
 * the decoder reads it back the same way). frame_id numbering and the decoder
 * model are turned off (VA carries neither and a stateful decode needs neither).
 *
 * INTER frames additionally need decoder DPB state the VA buffer omits
 * (refresh_frame_flags, per-reference order hints, saved global-motion params);
 * this unit mirrors that state across frames (Av1Dpb) so skip-mode presence and
 * the global-motion delta base match what the decoder derives. This is a pure,
 * device-free unit so it is host-testable exactly like H264AccessUnitBuilder.
 */

/* Mirrors the AV1 reference DPB (8 slots) the way a decoder would, so the
 * per-frame state the VA buffer does not carry can be reconstructed. */
struct Av1Dpb {
    struct Slot {
        VASurfaceID surface = VA_INVALID_SURFACE;
        uint32_t order_hint = 0;
        std::array<std::array<int32_t, 6>, GST_AV1_NUM_REF_FRAMES> gm_params {};
        bool valid = false;
    };
    std::array<Slot, GST_AV1_NUM_REF_FRAMES> slots {};

    void reset();
};

class Av1AccessUnitBuilder {
public:
    Av1AccessUnitBuilder() = default;

    void set_picture_parameters(const VADecPictureParameterBufferAV1& picture);
    void add_tile_parameters(std::span<const VASliceParameterBufferAV1> tiles);
    /* The VA slice-data buffer holds the concatenated tile payloads; each tile
     * parameter's slice_data_offset/size bounds one tile inside it. */
    void add_tile_data(std::span<const uint8_t> data);

    /*
     * Assemble the temporal unit for this picture and reset the per-picture
     * accumulators. Throws std::runtime_error when the header cannot be
     * re-synthesised (e.g. segmentation, which the GStreamer writer does not
     * support, or an unresolved reference). Empty when no picture was set.
     */
    std::vector<uint8_t> finish();

    bool has_picture() const { return has_picture_; }

    /* Test hooks: the last synthesised gst structures (for host tests). */
    const GstAV1SequenceHeaderOBU& last_sequence_header() const { return seq_; }
    const GstAV1FrameHeaderOBU& last_frame_header() const { return frame_; }

private:
    void reset_picture();
    void build_sequence_header(const VADecPictureParameterBufferAV1& pic);
    void build_frame_header(const VADecPictureParameterBufferAV1& pic);
    void compute_skip_mode_frame(const VADecPictureParameterBufferAV1& pic);
    uint8_t choose_refresh_flags(const VADecPictureParameterBufferAV1& pic) const;
    void update_dpb(const VADecPictureParameterBufferAV1& pic);
    int get_relative_dist(int a, int b) const;

    bool has_picture_ = false;
    bool have_sequence_ = false;
    VADecPictureParameterBufferAV1 picture_ {};
    std::vector<uint8_t> tile_data_;
    std::vector<VASliceParameterBufferAV1> tile_params_;

    GstAV1SequenceHeaderOBU seq_ {};
    GstAV1FrameHeaderOBU frame_ {};
    std::vector<uint8_t> last_emitted_sequence_header_;

    Av1Dpb dpb_;
};

/* 5.11.1 tile_group_obu payload (without the OBU header/size): with one tile it
 * is just the tile bytes; with several it is a leading flag byte then each
 * tile's LEB-fixed size and bytes, honouring tile_size_bytes. Exposed for the
 * host test. */
std::vector<uint8_t> build_tile_group_payload(std::span<const VASliceParameterBufferAV1> tiles,
    std::span<const uint8_t> tile_data, unsigned tile_cols, unsigned tile_rows, unsigned tile_size_bytes);

/* Wrap a payload in an OBU (obu_has_size_field = 1, no extension). Exposed for
 * the host test. */
std::vector<uint8_t> wrap_obu(GstAV1OBUType type, std::span<const uint8_t> payload);

/* LEB128 encode (exposed for the host test). */
std::vector<uint8_t> leb128(uint64_t value);

/* Serialise the uncompressed_header() RBSP (5.9) from the gst structs, with the
 * loop_restoration lr_unit_shift fix (the shipped gst bit writer is buggy for
 * 64x64 superblocks). Exposed for the host test. */
std::vector<uint8_t> serialize_frame_header_rbsp(const GstAV1SequenceHeaderOBU& seq, const GstAV1FrameHeaderOBU& frame);

} // namespace stateful
