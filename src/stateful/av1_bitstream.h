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
#include <optional>
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
 *
 * refresh_frame_flags (the 8-bit mask of DPB slots a frame writes ITSELF into
 * after decoding) is the one field VA does not carry -- and it is EXACTLY
 * DERIVABLE from the next frame's picture parameters; it is never guessed. VA
 * hands every frame both its own surface (current_frame) and ref_frame_map[8],
 * the whole DPB as that frame sees it BEFORE it updates it. Frames N and N+1 are
 * consecutive in DECODE order, so N+1's map is exactly N's map with N's refreshes
 * applied (5.9.2 decode_frame_wrapup):
 *
 *   refresh_frame_flags(N) = OR{ 1 << i : ref_frame_map_{N+1}[i] == current_frame_N }
 *
 * (derive_refresh_frame_flags()). submit_picture() therefore holds EVERY coded
 * frame for a ONE-FRAME LOOKAHEAD and emits it as soon as the next frame pins its
 * mask down.
 *
 * The heuristic this replaced ("refresh every slot VA no longer lists, else evict
 * the least-recently-used one") was applied whenever the mirrored DPB still had a
 * dead slot -- i.e. to most frames -- and it writes a frame into slots the encoder
 * never wrote, so from the first mini-GOP on our DPB drifts from the encoder's and
 * inter frames end up referencing the wrong PICTURE. Measured on a 300-frame
 * random-access (deep-B) AV1 stream (VA3-mcmatrix): 245 of 305 inter frames
 * referenced a wrong picture, and the re-synthesised stream was INVALID AV1 --
 * desktop libdav1d decoded 40 of 300 frames with 265 decode errors, the very
 * number the phone's hardware decoder produced. With the exact derivation our
 * mirrored DPB IS the encoder's DPB, the reference remap below is the identity,
 * and the emitted stream is self-consistent.
 *
 * A client that SYNCS a frame before decoding the next one (ffmpeg and Firefox
 * both do, for every frame the stream shows: vaSyncSurface(N) comes before
 * vaBeginPicture(N+1)) cannot be made to wait -- the held frame has to go to the
 * codec to produce the pixels it is waiting for, before its successor exists.
 * That frame is emitted PROVISIONALLY with refresh_frame_flags = 0: a mask of 0
 * is the only choice that cannot destroy anything, since the DPB at that point
 * holds 8 distinct live pictures and any other mask evicts one of them -- and
 * evicting a picture a later frame references is unrecoverable. The successor
 * then reveals the frame's true mask, and the builder re-submits that same frame
 * as a HIDDEN "DPB catch-up" frame (show_frame = 0, refresh = the derived mask,
 * the same tile payload) prepended INTO the next access unit's temporal unit.
 * The codec decodes it a second time -- identical bytes, so identical pixels and
 * identical CDFs -- and lands it in exactly the slots the encoder used, which
 * restores mirror == encoder DPB. Being hidden it produces NO extra CAPTURE
 * frame, so the session's one-submit / one-output mapping is untouched; the cost
 * is the second decode of the frames the client syncs eagerly (about half a
 * random-access stream). update_dpb() then evicts exactly the picture the
 * encoder evicted, which is dead by construction.
 *
 * choose_refresh_flags() -- the old heuristic -- survives only for finish(), the
 * immediate single-frame API the host tests drive.
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
     * accumulators, choosing refresh_frame_flags immediately (no lookahead).
     * Throws std::runtime_error when the header cannot be re-synthesised (e.g.
     * segmentation, which the GStreamer writer does not support). Empty when no
     * picture was set. Retained for the single-frame / immediate host tests;
     * the stateful context drives submit_picture()/flush() instead.
     */
    std::vector<uint8_t> finish();

    /* A temporal unit ready to submit, tagged with the caller's per-frame tag
     * (the stateful session sequence). refresh_frame_flags / order_hint are
     * exposed for host ground-truth checks. */
    struct ReadyFrame {
        uint64_t tag = 0;
        std::vector<uint8_t> access_unit;
        uint8_t refresh_frame_flags = 0;
        uint32_t order_hint = 0;
    };

    /* The outcome of accepting one picture (the accumulated set_picture /
     * add_tile_* state). `ready` holds the temporal units that became
     * submittable now, in decode order (0..2: a previously deferred frame that
     * this frame's ref_frame_map just resolved, and/or this frame itself).
     * `current_deferred` means this frame is held for a one-frame lookahead (its
     * surface is bound but no AU was produced yet). `current_failed` means it
     * cannot be re-synthesised (segmentation) and the client must fall back. */
    struct PictureResult {
        std::vector<ReadyFrame> ready;
        bool current_deferred = false;
        bool current_failed = false;
    };

    /*
     * Accept the accumulated picture, tag it, and resolve the one-frame
     * lookahead. This frame's ref_frame_map DERIVES the held frame's
     * refresh_frame_flags exactly (derive_refresh_frame_flags); the held frame
     * becomes ready and this one is held in turn. A key frame is exact without a
     * successor (a shown key refreshes all 8 slots by definition) and is emitted
     * immediately. Never throws for segmentation (it reports current_failed so a
     * previously deferred frame is not lost).
     */
    PictureResult submit_picture(uint64_t tag);

    /* Emit any still-deferred frame PROVISIONALLY, with refresh_frame_flags = 0
     * (end of stream, or a client that syncs a frame before decoding the next).
     * The mask cannot be derived yet and 0 is the only non-destructive choice;
     * the next submit_picture() derives the true one and re-submits the frame as
     * a hidden DPB catch-up frame. */
    std::vector<ReadyFrame> flush();

    bool has_picture() const { return has_picture_; }
    bool has_pending() const { return pending_.has_value(); }
    uint64_t pending_tag() const { return pending_ ? pending_->tag : 0; }

    /* Test hooks: the last synthesised gst structures (for host tests). */
    const GstAV1SequenceHeaderOBU& last_sequence_header() const { return seq_; }
    const GstAV1FrameHeaderOBU& last_frame_header() const { return frame_; }

private:
    /* A frame held for a one-frame lookahead: its parsed VA state, plus the
     * session tag its surface is already bound to. */
    struct PendingFrame {
        VADecPictureParameterBufferAV1 pic {};
        std::vector<VASliceParameterBufferAV1> tiles;
        std::vector<uint8_t> data;
        uint64_t tag = 0;
    };

    void reset_picture();
    void build_sequence_header(const VADecPictureParameterBufferAV1& pic);
    /* `tiles` are THIS frame's tile parameters (not the live accumulator): the
     * header's tile_size_bytes is derived from them and the tile group writes its
     * size fields in that width. */
    void build_frame_header(const VADecPictureParameterBufferAV1& pic, std::span<const VASliceParameterBufferAV1> tiles,
        uint8_t refresh_frame_flags, bool shown);
    /* Assemble TD [+ seq] [+ a queued catch-up frame] + frame header + tile group
     * for one frame with an explicit refresh_frame_flags; leaves frame_ populated
     * for update_dpb. `shown` writes show_frame (a catch-up frame is hidden, so
     * the codec does not output it a second time); `bare` emits just the frame
     * header + tile group, for a catch-up frame that is spliced into the NEXT
     * temporal unit rather than starting one. Throws on segmentation. */
    std::vector<uint8_t> assemble_au(const VADecPictureParameterBufferAV1& pic,
        std::span<const VASliceParameterBufferAV1> tiles, std::span<const uint8_t> data, uint8_t refresh_frame_flags,
        bool shown = true, bool bare = false);
    void compute_skip_mode_frame(const VADecPictureParameterBufferAV1& pic);
    /* Best-effort mask for a frame with NO successor to derive from (flush() /
     * finish()): refresh every slot whose surface VA no longer lists, else evict
     * the oldest by order hint. Self-consistent, NOT encoder-exact -- it is never
     * used while a successor is available. */
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
    std::optional<PendingFrame> pending_;
    /* A frame flush() had to emit before its mask could be derived (emitted with
     * refresh_frame_flags = 0). The next picture derives its true mask and turns
     * it into the catch-up frame below. */
    std::optional<PendingFrame> provisional_;
    /* A hidden DPB catch-up frame waiting to be spliced into the next temporal
     * unit, ahead of the frame that may reference it. */
    std::vector<uint8_t> catch_up_;
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

/* The EXACT refresh_frame_flags of frame N, derived from frame N+1's pre-decode
 * ref_frame_map: N wrote itself into exactly the slots that hold N's own surface
 * once N+1 sees the DPB (5.9.2 decode_frame_wrapup). Pure, no heuristic, no
 * mirrored state. Exposed for the host test. */
uint8_t derive_refresh_frame_flags(VASurfaceID current_frame, const VASurfaceID next_ref_frame_map[8]);

/* LEB128 encode (exposed for the host test). */
std::vector<uint8_t> leb128(uint64_t value);

/* Serialise the uncompressed_header() RBSP (5.9) from the gst structs, with the
 * loop_restoration lr_unit_shift fix (the shipped gst bit writer is buggy for
 * 64x64 superblocks). Exposed for the host test. */
std::vector<uint8_t> serialize_frame_header_rbsp(const GstAV1SequenceHeaderOBU& seq, const GstAV1FrameHeaderOBU& frame);

} // namespace stateful
