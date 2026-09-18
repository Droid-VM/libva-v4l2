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

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <span>
#include <vector>

#include "allocator.h"
#include "device.h"

namespace stateful {

/*
 * The stateful M2M decode session (VPU_DESIGN.md 7.6 points 4-6): access
 * units go into a small OUTPUT ring tagged with a 64-bit sequence; the device
 * copies the tag to the decoded CAPTURE buffer; sync() collects CAPTURE
 * buffers into a sequence-keyed stash until the requested one arrives, with
 * the bounded-wait DEC_CMD_STOP/drain/START recovery of point 5b.
 *
 * Thread safety (D83): ffmpeg's vaapi hwaccel is HWACCEL_CAP_ASYNC_SAFE, so
 * frame threads call vaBeginPicture/vaEndPicture/vaSyncSurface concurrently
 * on one context. Every public entry point serialises on an internal mutex;
 * a thread that must sleep for the device does so through a single elected
 * harvester with the mutex released (7.6 point 5: a sync waiting for frame S
 * while holding the lock would starve the thread submitting more input into
 * a deadlock).
 */
class StatefulSession {
public:
    struct Options {
        unsigned num_surfaces = 0;
        /* D84: the live surface count, re-read at every CAPTURE
         * provisioning; the larger of it and num_surfaces sets the pool
         * share. vaCreateContext's render-target list is empty in modern
         * clients (ffmpeg passes none), so the count must come from the
         * surfaces vaCreateSurfaces actually made. Called with the session
         * mutex held: it must not take the driver-wide mutex (read an
         * atomic counter instead). */
        std::function<unsigned()> surface_count;
        unsigned output_ring_size = 8;
        /* < 0: LIBVA_V4L2_SYNC_TIMEOUT_MS or the 500 ms default (point 5b). */
        int sync_timeout_ms = -1;
        /* D85: how long the pipeline must be QUIET -- no new access unit
         * submitted AND no new decoded frame dequeued -- before a waiting sync
         * drains. VA-API has no EOS call, so a tail the codec holds can only be
         * shaken loose by a drain, and a quiet pipeline is the only end-of-
         * stream signal this interface has. That makes it a GUESS; the hard
         * cap alone never drains (it only feeds the backstop), because a drain
         * in the middle of a live stream resets the codec and cuts the
         * reference chain. Default 50 ms: on the AVC path that is now dormant
         * (crosvm puts the codec in decode order, so it holds nothing -- D91),
         * and where no mid-stream drain can fire (AV1/VP9) it only sets the
         * poll cadence. < 0: LIBVA_V4L2_SYNC_IDLE_MS or the default. */
        int sync_idle_ms = -1;
        /* D85/D86: the mid-stream idle/timeout DEC_CMD_STOP drain
         * (recover_locked) exists for H.264, whose reorder tail the codec holds
         * back until a drain shakes it loose -- VA-API has no flush/EOS call, so
         * a mid-stream sync must drain to extract it (B18/B19). VP9 clears it:
         * its display order is in-band (show_existing_frame), so there is no held
         * tail mid-stream. AV1 clears it too: on a correctly reconstructed
         * stream the decoder emits a frame per access unit and keeps up with a
         * client that syncs every frame before feeding the next (measured: a
         * 166 s browser session, 5113 access units in / 5113 frames out), so
         * there is no mid-stream tail for a drain to extract and a DEC_CMD_STOP
         * would only seek a live reference chain. (An earlier note here claimed
         * the AV1 decoder held a decode-order tail the drain dropped. That was
         * measured while this backend was rebuilding the AV1 bitstream
         * incorrectly -- heuristic refresh_frame_flags and a missing lr_uv_shift
         * bit -- so the decoder was failing on an invalid stream, not holding
         * frames back. The claim is withdrawn; the setting is unchanged.)
         * When false, a mid-stream sync waits out the hard cap without a reset and
         * fails only THIS surface as a last resort. finish() (EOS) still drains
         * the true tail in every codec. */
        bool allow_midstream_drain = true;
        /* D88/post-crash: a fresh client's REQBUFS/STREAMON can be refused
         * EBUSY while the device is still reaping a crashed client's session
         * (B18/B19: the next process hits it within milliseconds). Provisioning
         * retries EBUSY with a bounded backoff for up to this long before
         * failing the context/first begin cleanly. < 0: 2000 ms. */
        int provision_retry_ms = -1;
        /* VA3 zero-copy (7.7): when set and the device advertises
         * V4L2_BUF_CAP_SUPPORTS_DMABUF, CAPTURE is provisioned from these GBM
         * dma-bufs instead of device MMAP buffers, so vaExportSurfaceHandle
         * can hand the browser a GPU-importable fd. Null (or an unusable
         * allocator, or an r22 device that masks the cap) -> VA1 MMAP. Not
         * owned by the session. */
        SurfaceAllocator* allocator = nullptr;
    };

    enum class SyncStatus {
        ok,
        decode_error,
        dead,
    };

    /* Decided once per context at the first CAPTURE provisioning and logged on
     * one line (7.7 point 5): gbm-dmabuf (a surface is a GBM bo) or mmap. */
    enum class CaptureMode {
        mmap,
        gbm_dmabuf,
    };

    /* A claimed decoded frame: the CAPTURE index plus the provisioning
     * generation it belongs to (D83). A re-provision (mid-stream
     * SOURCE_CHANGE) increments the generation; releasing a stale binding is
     * a logged no-op instead of a QBUF of an index the pool no longer has. */
    struct Frame {
        unsigned index = 0;
        uint64_t generation = 0;
        friend bool operator==(const Frame&, const Frame&) = default;
    };

    StatefulSession(StatefulDevice& device, uint32_t coded_pixelformat, unsigned coded_width, unsigned coded_height,
        const Options& options, std::function<void(const char*)> log = {});

    /* Copy one access unit into the ring and queue it; throws DeviceLost or
     * std::runtime_error. */
    void submit(uint64_t sequence, std::span<const uint8_t> access_unit);

    /* Wait (bounded) for the frame with this sequence; other frames dequeued
     * on the way are stashed. */
    SyncStatus sync(uint64_t sequence, Frame* frame);

    /* Return a claimed CAPTURE buffer to the queue (surface reuse/destroy,
     * point 4); a binding from a previous provisioning generation is a
     * logged no-op. */
    void release_frame(const Frame& frame);

    /* Forget a submitted-but-never-synced sequence; its frame is re-queued
     * on arrival. */
    void drop_sequence(uint64_t sequence);

    /* vaDestroyContext path (point 6): DEC_CMD_STOP + drain + STREAMOFF both
     * + REQBUFS(0). */
    void finish();

    /* Serialise an external reader of decoded frame memory (the vaGetImage /
     * vaDeriveImage copies) against re-provisioning (D83). Never take the
     * driver-wide mutex while holding this: the established order is
     * driver mutex, then session mutex (destroySurfaces -> release_frame). */
    std::unique_lock<std::mutex> hold() { return std::unique_lock<std::mutex>(mutex_); }

    /* The CAPTURE mode chosen at provisioning (7.7 point 5); mmap until the
     * first SOURCE_CHANGE decides. Stable once provisioned; under churn read it
     * holding hold(). */
    CaptureMode capture_mode() const { return capture_mode_; }

    /* The GBM buffer backing a claimed CAPTURE index (gbm-dmabuf mode), for
     * vaExportSurfaceHandle and the vaDeriveImage/vaGetImage CPU views. Null in
     * MMAP mode or for an out-of-range index. Call holding hold(), and while
     * the surface's binding is still the current generation. */
    SurfaceBuffer* capture_buffer(unsigned index);

    bool dead() const { return dead_.load(std::memory_order_relaxed); }
    bool provisioned() const { return provisioned_.load(std::memory_order_relaxed); }
    /* Stable once provisioned; under churn read it holding hold(). */
    const CaptureFormat& capture_format() const { return capture_format_; }
    /* Bumped by every CAPTURE provisioning; under churn read it holding
     * hold(). */
    uint64_t generation() const { return generation_; }
    unsigned timeout_recoveries() const { return timeout_recoveries_; }
    unsigned idle_drains() const { return idle_drains_; }
    int sync_timeout_ms() const { return sync_timeout_ms_; }
    int sync_idle_ms() const { return sync_idle_ms_; }
    int midstream_stall_ms() const { return midstream_stall_ms_; }
    bool allow_midstream_drain() const { return allow_midstream_drain_; }
    StatefulDevice& device() { return device_; }

private:
    /* _locked members run with mutex_ held. */
    void pump_locked();
    void handle_source_change_locked();
    /* 7.7 point 5: choose gbm-dmabuf vs mmap once, honouring the
     * LIBVA_V4L2_SURFACES override and the SUPPORTS_DMABUF capability, and log
     * the one-line reason. */
    CaptureMode decide_capture_mode_locked();
    /* Provision the CAPTURE pool in the decided mode. The gbm variant returns
     * false when it must fall back (allocation, stride or REQBUFS refusal),
     * having left nothing allocated, so the caller retries as mmap. */
    void provision_capture_mmap_locked(unsigned count);
    bool provision_capture_gbm_locked(unsigned count);
    void release_capture_pool_locked();
    /* QBUF one CAPTURE buffer in whichever mode is live (7.6/7.7). */
    void requeue_capture_locked(unsigned index);
    void handle_capture_locked(const DequeuedCapture& frame);
    bool claim_locked(uint64_t sequence, Frame* frame);
    /* Drop the lock around one device wait; exactly one thread is the
     * harvester at a time (the others sleep on the condvar). */
    void wait_for_progress(std::unique_lock<std::mutex>& lock, int timeout_ms, bool include_output);
    /* Caller owns harvesting_; true when LAST was seen. */
    bool drain_capture_locked(std::unique_lock<std::mutex>& lock, int timeout_ms);
    SyncStatus recover_locked(std::unique_lock<std::mutex>& lock, uint64_t sequence, Frame* frame, bool idle);
    /* Run a provisioning device call, retrying EBUSY with a bounded backoff
     * (post-crash: the device is still reaping a dead client). Throws
     * std::runtime_error if the budget is exhausted; other errors propagate. */
    void retry_provision(const char* what, const std::function<void()>& op);
    void grow_output_buffers_locked(std::unique_lock<std::mutex>& lock, size_t needed);
    int acquire_output_buffer_locked(std::unique_lock<std::mutex>& lock, size_t needed); /* -1 on timeout */
    void log(const char* message);

    StatefulDevice& device_;
    std::function<void(const char*)> log_;
    unsigned num_surfaces_;
    std::function<unsigned()> surface_count_;
    SurfaceAllocator* allocator_; /* not owned; null -> MMAP only */
    unsigned output_ring_size_;
    int sync_timeout_ms_;
    int sync_idle_ms_;
    /* D85: the mid-stream drain's anti-deadlock backstop. sync_timeout_ms_ is
     * measured from the start of ONE sync, so on the drain path it cannot be
     * the drain trigger: a paced client legitimately needs longer than it to
     * feed the access units a waiting sync's frame depends on, and draining
     * there is exactly the reference-chain cut of D85. The trigger is the quiet
     * window (sync_idle_ms_); this is only the cap that stops a sync waiting
     * forever when the pipeline keeps MOVING and our frame never comes -- the
     * 'sync timeout' of 7.6 point 5b, whose bar stays 0 on every clip. Derived
     * as max(sync_timeout_ms_, 3 x sync_idle_ms_): strictly above the quiet
     * window, so an end-of-stream tail is always counted as the idle drain it
     * is. */
    int midstream_stall_ms_;
    bool allow_midstream_drain_;
    int provision_retry_ms_;
    uint32_t output_pixelformat_;
    uint32_t coded_width_;
    uint32_t coded_height_;
    uint32_t output_size_ = 0;

    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool harvesting_ = false; /* one device waiter/drainer at a time */

    std::vector<unsigned> free_outputs_;
    std::set<unsigned> usable_outputs_; /* current ring; retired buffers are dropped on dequeue */
    unsigned queued_outputs_ = 0;

    bool capture_streaming_ = false;
    std::atomic<bool> provisioned_ { false };
    std::atomic<bool> dead_ { false };
    CaptureFormat capture_format_ {};
    unsigned capture_count_ = 0;
    uint64_t generation_ = 0; /* CAPTURE provisioning generation (D83) */
    bool stale_release_logged_ = false;

    CaptureMode capture_mode_ = CaptureMode::mmap;
    bool capture_mode_decided_ = false;
    /* gbm-dmabuf mode: one GBM bo per CAPTURE index (the surfaces plus any
     * internal spares up to the codec minimum), owned for the context's life
     * (7.7 point 2). Empty in MMAP mode. */
    std::vector<std::unique_ptr<SurfaceBuffer>> capture_bos_;

    std::map<uint64_t, unsigned> stash_; /* decoded, not yet claimed */
    std::set<uint64_t> unwanted_sequences_;
    std::set<unsigned> client_owned_; /* claimed CAPTURE indices */
    uint64_t submit_count_ = 0; /* total submits; syncs watch it for input flow (D85) */
    /* D85: total CAPTURE buffers dequeued from the device. A sync watches it
     * alongside submit_count_: a device still handing frames back is not at
     * end of stream, whoever the frames belong to. */
    uint64_t decoded_count_ = 0;
    uint64_t claim_count_ = 0; /* frames actually handed to the client */
    unsigned timeout_recoveries_ = 0;
    unsigned idle_drains_ = 0;

    /* LIBVA_V4L2_SYNC_STATS=1 only (default off, and then this whole block is
     * dead): the inter-submit gap distribution of the client and the longest
     * quiet window a sync sat through, logged once per session by finish().
     * This is the measurement the D85 threshold is chosen from -- a paced
     * client's real pacing, from the backend's own side of the interface. */
    bool sync_stats_ = false;
    bool have_last_submit_ = false;
    std::chrono::steady_clock::time_point last_submit_at_ {};
    static constexpr unsigned kGapBuckets = 13;
    unsigned gap_histogram_[kGapBuckets] = {};
    unsigned gap_count_ = 0;
    long long gap_sum_ms_ = 0;
    int gap_max_ms_ = 0;
    int quiet_max_ms_ = 0; /* longest no-input-no-output window seen inside a sync */
    void record_submit_gap_locked();
    void note_quiet_locked(int quiet_ms);
    void log_sync_stats_locked();
};

} // namespace stateful
