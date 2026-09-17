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
 * after decoding) is the one field VA does not carry and cannot be inferred from
 * a single frame once the 8-slot DPB is full of distinct live surfaces (a deep
 * B-pyramid -- random-access AV1, what YouTube serves). It IS recoverable with a
 * ONE-FRAME LOOKAHEAD: VA gives every frame its ref_frame_map[8] (the DPB as that
 * frame sees it BEFORE it updates it), so the slots frame N refreshed are exactly
 * those where frame N+1's ref_frame_map now holds N's own surface but N's did
 * not (reconstruct_refresh_flags()). submit_picture() therefore HOLDS a frame
 * whose refresh cannot be chosen unambiguously now (the DPB is full and the
 * least-recently-used slot is a still-live surface's last copy -- evicting it
 * would drop a reference a later frame needs) until the next frame arrives and
 * pins down which slot it really refreshed. Low-delay / P-frame streams never
 * reach that ambiguity (the DPB always has a dead slot, or the LRU victim is a
 * duplicated anchor), so they are emitted immediately and bit-exactly, exactly
 * as before -- the one-frame delay is confined to the random-access case. The
 * unresolved tail frame at end-of-stream is emitted by flush() with a
 * best-effort mask (nothing references it, so the choice does not matter).
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
     * lookahead (VA2c). Uses this frame's ref_frame_map to recover the deferred
     * frame's refresh_frame_flags exactly. Never throws for segmentation (it
     * reports current_failed so a previously deferred frame is not lost).
     */
    PictureResult submit_picture(uint64_t tag);

    /* Emit any still-deferred frame with a best-effort refresh mask (end of
     * stream, or a client that syncs the tail before the next frame arrives). */
    std::vector<ReadyFrame> flush();

    bool has_picture() const { return has_picture_; }
    bool has_pending() const { return pending_.has_value(); }
    uint64_t pending_tag() const { return pending_ ? pending_->tag : 0; }

    /* VA3-fakeau2 SPIKE. Turn on retention of the last real frame's VA inputs
     * (pic + tiles + data) so synthesize_fake_frame() can rebuild a padding
     * frame from them. Off by default so the normal path allocates nothing
     * extra and is byte-identical to r402's OFF path (only the AV1 context with
     * LIBVA_V4L2_FAKE_AU_OHINT enables it). */
    void enable_fake_capture() { capture_last_real_ = true; }

    /* VA3-fakeau2 SPIKE. Synthesise a NON-reference padding frame from the last
     * real inter frame's VA inputs, with a BUMPED order_hint (last real +k) so
     * the QTI AV1 decoder treats it as a genuinely NEW frame -- advancing the
     * reorder and flushing the held real frame -- and refresh_frame_flags=0 so
     * it never overwrites a real DPB reference slot (VA3-reorder-probe: the
     * decoder emits a held display frame only when fed a later order_hint;
     * reorder depth is bounded at 4). show_frame selects a shown padding frame
     * (the codec emits a CAPTURE the session drops by sentinel tag) or a hidden
     * one (no CAPTURE emitted). Returns std::nullopt when nothing usable was
     * retained (no real inter frame yet, order_hint disabled, or assembly
     * failed). Does NOT mutate the mirrored DPB (refresh=0, no update_dpb), so
     * the following real frames read the unchanged reference state -- the
     * bit-exactness question 关卡二 is settled on the phone. */
    std::optional<std::vector<uint8_t>> synthesize_fake_frame(unsigned k, bool show_frame);

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
    /* show_frame overridable for the VA3-fakeau2 hidden-padding construction;
     * the real path always emits shown (VA2c), so it defaults to true. */
    void build_frame_header(
        const VADecPictureParameterBufferAV1& pic, uint8_t refresh_frame_flags, bool show_frame = true);
    /* Assemble TD [+ seq] + frame header + tile group for one frame with an
     * explicit refresh_frame_flags; leaves frame_ populated for update_dpb.
     * Throws on segmentation. */
    std::vector<uint8_t> assemble_au(const VADecPictureParameterBufferAV1& pic,
        std::span<const VASliceParameterBufferAV1> tiles, std::span<const uint8_t> data, uint8_t refresh_frame_flags,
        bool show_frame = true);
    /* VA3-fakeau2 SPIKE: retain a copy of a real frame's VA inputs when
     * capture_last_real_ is on, so a later padding frame can be built from
     * them. No-op otherwise. */
    void save_last_real(const VADecPictureParameterBufferAV1& pic, std::span<const VASliceParameterBufferAV1> tiles,
        std::span<const uint8_t> data);
    void compute_skip_mode_frame(const VADecPictureParameterBufferAV1& pic);
    uint8_t choose_refresh_flags(const VADecPictureParameterBufferAV1& pic) const;
    /* True when refresh_frame_flags is ambiguous without a lookahead: the DPB is
     * full of live surfaces AND the least-recently-used slot the immediate
     * heuristic would evict holds a still-live surface's last copy. */
    bool refresh_is_ambiguous(const VADecPictureParameterBufferAV1& pic) const;
    /* Recover a held frame's refresh_frame_flags from the NEXT frame's
     * ref_frame_map: a slot is refreshed iff it now holds the held frame's own
     * surface and did not before (VA2c one-frame lookahead). */
    uint8_t reconstruct_refresh_flags(
        const VADecPictureParameterBufferAV1& held, const VASurfaceID next_ref_frame_map[8]) const;
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

    /* VA3-fakeau2 SPIKE: the last real frame's VA inputs, retained only when
     * capture_last_real_ is set, so synthesize_fake_frame() can rebuild a
     * padding frame from a genuine frame (same tiles/refs) with a bumped
     * order_hint. Off by default -> zero extra work on the shipped path. */
    bool capture_last_real_ = false;
    bool have_last_real_ = false;
    VADecPictureParameterBufferAV1 last_real_pic_ {};
    std::vector<VASliceParameterBufferAV1> last_real_tiles_;
    std::vector<uint8_t> last_real_data_;
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
