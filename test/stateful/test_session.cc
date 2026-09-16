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
 * VPU_DESIGN.md 7.6 host verification (c): the stateful session state machine
 * against a scripted fake of the device seam -- CAPTURE provisioning after
 * SOURCE_CHANGE, the timestamp/sequence mapping and the stash, out-of-order
 * delivery, the timeout drain, OUTPUT ring growth, and the dead session.
 */

#include <chrono>
#include <set>
#include <string>
#include <vector>

#include "../../src/stateful/session.h"
#include "check.h"
#include "fake_device.h"

using stateful::StatefulSession;

namespace {

StatefulSession::Options fast_options()
{
    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.output_ring_size = 4;
    options.sync_timeout_ms = 30; /* keep the stall tests fast */
    return options;
}

std::vector<uint8_t> fake_au(size_t size = 512)
{
    return std::vector<uint8_t>(size, 0xab);
}

void test_provisioning_and_mapping()
{
    FakeDevice device;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    CHECK(device.subscribed);
    CHECK(device.output_streaming);
    CHECK_EQ(device.output_count, 4u);
    CHECK(!session.provisioned());

    session.submit(1, fake_au());
    StatefulSession::Frame index;
    CHECK(session.sync(1, &index) == StatefulSession::SyncStatus::ok);

    /* SOURCE_CHANGE provisioned the pool: MIN_BUFFERS(4) + min(surfaces 6, 8)
     * = 10, all queued, minus the one the frame came back in (7.6 point 4). */
    CHECK(session.provisioned());
    CHECK_EQ(device.capture_count, 10u);
    CHECK(device.capture_streaming);
    CHECK_EQ(device.free_captures.size(), 9u);
    CHECK_EQ(session.capture_format().bytesperline, 1920u);
    CHECK_EQ(session.timeout_recoveries(), 0u);

    /* Releasing the frame re-queues its CAPTURE buffer. */
    session.release_frame(index);
    CHECK_EQ(device.free_captures.size(), 10u);
}

void test_out_of_order_delivery_and_stash()
{
    FakeDevice device;
    device.manual_delivery = true;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    session.submit(1, fake_au());
    session.submit(2, fake_au());
    session.submit(3, fake_au());

    /* The decoder finishes 2, 3, 1: sync(1) stashes the first two. */
    device.deliver(2);
    device.deliver(3);
    device.deliver(1);

    StatefulSession::Frame index1;
    CHECK(session.sync(1, &index1) == StatefulSession::SyncStatus::ok);

    /* 2 and 3 now come straight from the stash. */
    StatefulSession::Frame index2;
    CHECK(session.sync(2, &index2) == StatefulSession::SyncStatus::ok);
    StatefulSession::Frame index3;
    CHECK(session.sync(3, &index3) == StatefulSession::SyncStatus::ok);
    CHECK(index1.index != index2.index && index2.index != index3.index && index1.index != index3.index);
    CHECK_EQ(session.timeout_recoveries(), 0u);

    /* All three OUTPUT buffers were reclaimed (ring of 4 still has room). */
    session.submit(4, fake_au());
    session.submit(5, fake_au());
    session.submit(6, fake_au());
    session.submit(7, fake_au());
    CHECK_EQ(device.pending_decodes.size(), 4u);
}

void test_reorder_absorbed_in_any_sync_order()
{
    /* VA3-sync-reorder: the reorder-tolerance contract. A stateful codec may
     * deliver CAPTURE frames in an order that is neither submit order nor the
     * client's sync order (a deep-B decoder reorders by order_hint; a VA-API
     * client syncs in display order). The backend keys the stash by the sequence
     * carried in the OUTPUT timestamp (TIMESTAMP_COPY), so sync(seq) returns THAT
     * sequence's frame whenever it has been produced, regardless of order, and
     * releasing recycles the CAPTURE buffer with no leak. The wedge investigation
     * confirmed this mapping is ALREADY correct -- the deep-B AV1 wedge is an
     * input-starvation stall (session.h allow_midstream_drain doc), NOT a
     * mis-delivery. Named mutation this fails under: claim_locked() returning
     * stash_.begin()->second (the front of the stash / FIFO) instead of
     * stash_.find(sequence) -- a scrambled sync then gets the wrong frame. */
    FakeDevice device;
    device.manual_delivery = true;
    /* An OUTPUT ring deep enough to hold every submit before the codec produces
     * anything, so all six sit in the stash when the scrambled syncs begin. */
    StatefulSession::Options options = fast_options();
    options.output_ring_size = 8;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    constexpr int n = 6;
    for (uint64_t s = 1; s <= n; s++) {
        session.submit(s, fake_au());
    }

    /* The codec finishes in a scrambled order. The fake pops CAPTURE indices
     * (0,1,2,...) in delivery order, so the seq->index mapping is known and the
     * "right frame" assertion below is exact. */
    const uint64_t delivery[n] = { 2, 5, 1, 4, 3, 6 };
    unsigned expected_index[n + 1] = { 0 };
    for (int i = 0; i < n; i++) {
        device.deliver(delivery[i]);
        expected_index[delivery[i]] = static_cast<unsigned>(i);
    }

    /* The client syncs in yet another order (its display order). Every sync must
     * return its OWN sequence's frame -- a distinct CAPTURE buffer each time. */
    const uint64_t sync_order[n] = { 4, 1, 6, 2, 5, 3 };
    std::vector<StatefulSession::Frame> claimed;
    std::set<unsigned> seen;
    for (uint64_t s : sync_order) {
        StatefulSession::Frame frame;
        CHECK(session.sync(s, &frame) == StatefulSession::SyncStatus::ok);
        CHECK_EQ(frame.index, expected_index[s]); /* the RIGHT frame, not a FIFO front */
        CHECK(seen.insert(frame.index).second); /* distinct buffers, never handed out twice */
        claimed.push_back(frame);
    }
    CHECK_EQ(session.timeout_recoveries(), 0u);
    CHECK_EQ(session.idle_drains(), 0u);

    /* No surface leak: releasing every claimed frame returns its CAPTURE buffer
     * to the device's free pool -- all provisioned buffers (min 4 + share
     * min(6,8) = 10) are free again. */
    for (const auto& f : claimed) {
        session.release_frame(f);
    }
    CHECK_EQ(device.capture_count, 10u);
    CHECK_EQ(device.free_captures.size(), static_cast<size_t>(device.capture_count));
}

void test_drop_sequence_recycles()
{
    FakeDevice device;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    session.submit(1, fake_au());
    StatefulSession::Frame index;
    CHECK(session.sync(1, &index) == StatefulSession::SyncStatus::ok);
    session.release_frame(index);
    const auto captures_free = device.free_captures.size();

    /* Sequence 2 is submitted, then its surface is destroyed before any
     * sync: the decoded frame must be re-queued, not stashed (7.6 point 4). */
    session.submit(2, fake_au());
    session.drop_sequence(2);
    session.submit(3, fake_au());
    StatefulSession::Frame index3;
    CHECK(session.sync(3, &index3) == StatefulSession::SyncStatus::ok);
    /* 2's buffer was recycled; only 3's is claimed. */
    CHECK_EQ(device.free_captures.size(), captures_free - 1);
}

void test_timeout_drain_recovery()
{
    FakeDevice device;
    device.manual_delivery = true; /* the decoder holds everything back */
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    session.submit(1, fake_au());
    session.submit(2, fake_au());
    session.submit(3, fake_au());

    /* Only DEC_CMD_STOP shakes the frames loose (7.6 point 5b). */
    device.on_decoder_stop = [&device]() {
        device.deliver(2);
        device.deliver(1);
        device.deliver(3, /*last=*/true);
    };

    StatefulSession::Frame index1;
    CHECK(session.sync(1, &index1) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(session.timeout_recoveries(), 1u);
    CHECK_EQ(device.decoder_stops, 1u);
    CHECK_EQ(device.decoder_starts, 1u);

    /* The drain harvested every frame into the stash. */
    StatefulSession::Frame index2;
    CHECK(session.sync(2, &index2) == StatefulSession::SyncStatus::ok);
    StatefulSession::Frame index3;
    CHECK(session.sync(3, &index3) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(session.timeout_recoveries(), 1u);

    /* A drain that yields only an empty LAST buffer: decode error. */
    session.submit(4, fake_au());
    device.on_decoder_stop = [&device]() { device.deliver_empty_last(); };
    StatefulSession::Frame index4;
    CHECK(session.sync(4, &index4) == StatefulSession::SyncStatus::decode_error);
    CHECK_EQ(session.timeout_recoveries(), 2u);
}

void test_capture_pool_share()
{
    /* D84: the B18 strace showed REQBUFS(CAPTURE) = the announced minimum
     * (21) because vaCreateContext carried no render targets, so the
     * client's held surfaces came out of the codec's own slots. The pool
     * must be min + min(surfaces, 8) capped at 32 (7.6 point 4), with the
     * surface count read live at provisioning time. Named mutation this
     * fails under: drop the surface_count re-read in handle_source_change
     * (num_surfaces stays 0 and the ask collapses to the bare minimum). */
    auto decode_one = [](FakeDevice& device, StatefulSession& session) {
        session.submit(1, fake_au());
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);
        session.release_frame(frame);
    };

    {
        std::vector<std::string> lines;
        FakeDevice device;
        device.scripted_min_buffers = 21;
        StatefulSession::Options options = fast_options();
        options.num_surfaces = 0; /* what vaCreateContext really passes */
        options.surface_count = [] { return 12u; };
        StatefulSession session(
            device, 0x34363248, 1920, 1088, options, [&lines](const char* m) { lines.emplace_back(m); });
        decode_one(device, session);
        CHECK_EQ(device.capture_count, 29u); /* 21 + min(12, 8) */
        bool logged = false;
        for (auto&& line : lines) {
            logged = logged || line.find("CAPTURE pool: min 21 + share 8 = 29 (surfaces 12") != std::string::npos;
        }
        CHECK(logged);
    }
    {
        FakeDevice device;
        device.scripted_min_buffers = 21;
        StatefulSession::Options options = fast_options();
        options.num_surfaces = 0;
        options.surface_count = [] { return 4u; };
        StatefulSession session(device, 0x34363248, 1920, 1088, options);
        decode_one(device, session);
        CHECK_EQ(device.capture_count, 25u); /* 21 + min(4, 8) */
    }
    {
        FakeDevice device;
        device.scripted_min_buffers = 30;
        StatefulSession::Options options = fast_options();
        options.num_surfaces = 0;
        options.surface_count = [] { return 12u; };
        StatefulSession session(device, 0x34363248, 1920, 1088, options);
        decode_one(device, session);
        CHECK_EQ(device.capture_count, 32u); /* 30 + 8 capped at 32 */
    }
}

void test_stale_release_after_reprovision()
{
    FakeDevice device;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    session.submit(1, fake_au());
    StatefulSession::Frame frame1;
    CHECK(session.sync(1, &frame1) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(frame1.generation, 1u);

    /* A mid-stream SOURCE_CHANGE re-provisions the pool: every binding
     * handed out before it is stale (D83). */
    device.pending_events.push_back(stateful::DeviceEvent::source_change);
    session.submit(2, fake_au());
    StatefulSession::Frame frame2;
    CHECK(session.sync(2, &frame2) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(frame2.generation, 2u);

    /* Releasing the old binding is a no-op: no QBUF, no throw (B18's abort
     * was exactly a stale index reaching the device from release_frame).
     * Named mutation this fails under: remove the generation check in
     * StatefulSession::release_frame -- the stale index is re-queued and
     * the free count below moves. */
    const auto free_before = device.free_captures.size();
    session.release_frame(frame1);
    CHECK_EQ(device.free_captures.size(), free_before);

    /* The current-generation binding still releases normally. */
    session.release_frame(frame2);
    CHECK_EQ(device.free_captures.size(), free_before + 1);
}

void test_idle_drain_on_stream_tail()
{
    /* D85: a B-frame stream's tail is held by the codec until a drain --
     * VA-API has no EOS call -- so B18 saw a flat 500 ms 'sync timeout' on
     * EVERY -bf 3 run at end of stream. Once no new submission arrives for
     * the idle budget, the sync must drain immediately instead of sitting
     * out the hard cap, and the tail frame must come out. Named mutation
     * this fails under: force idle_expired to false in sync() (the frame
     * then arrives only after the 500 ms hard cap, as a 'sync timeout'). */
    FakeDevice device;
    device.manual_delivery = true;
    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.sync_timeout_ms = 500; /* the hard cap the tail must NOT wait out */
    options.sync_idle_ms = 30;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    for (uint64_t sequence = 1; sequence <= 5; sequence++) {
        session.submit(sequence, fake_au());
    }
    /* The codec emits the first two and holds the last three (the reorder
     * tail). */
    device.deliver(1);
    device.deliver(2);
    StatefulSession::Frame frame;
    CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);
    CHECK(session.sync(2, &frame) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(session.idle_drains(), 0u);

    device.on_decoder_stop = [&device]() {
        device.deliver(3);
        device.deliver(4);
        device.deliver(5, /*last=*/true);
    };

    /* Sync of frame N-2 after the last submission: one idle drain within
     * the idle budget, and the frame comes out -- not a 500 ms stall. */
    const auto before = std::chrono::steady_clock::now();
    CHECK(session.sync(3, &frame) == StatefulSession::SyncStatus::ok);
    const auto elapsed_ms
        = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - before).count();
    CHECK(elapsed_ms < 250);
    CHECK_EQ(session.idle_drains(), 1u);
    CHECK_EQ(session.timeout_recoveries(), 0u);
    CHECK_EQ(device.decoder_stops, 1u);

    /* The drain harvested the rest of the tail into the stash. */
    CHECK(session.sync(4, &frame) == StatefulSession::SyncStatus::ok);
    CHECK(session.sync(5, &frame) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(session.idle_drains(), 1u);
}

void test_reenterable_recovery()
{
    /* D86: B18 found recovery was one-shot -- a second drain+restart in a
     * session, or one before the first SOURCE_CHANGE, ended in a dead
     * session. Two consecutive recoveries must both yield their frames.
     * The fake pauses OUTPUT on DEC_CMD_STOP, so a recovery that fails to
     * re-STREAMON OUTPUT starves the next decode (the one-shot). Named
     * mutation this fails under: drop the stream_output(true) re-assert in
     * recover_locked (the second submit throws on the paused queue). */
    FakeDevice device;
    device.manual_delivery = true;
    device.stop_pauses_output = true;
    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.sync_timeout_ms = 30;
    options.sync_idle_ms = 10;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    /* First stall: submit 1, deliver nothing until DEC_CMD_STOP. */
    session.submit(1, fake_au());
    device.on_decoder_stop = [&device]() { device.deliver(1, /*last=*/true); };
    StatefulSession::Frame frame;
    CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(device.decoder_stops, 1u);
    CHECK(!session.dead());
    session.release_frame(frame);

    /* The recovery must have resumed OUTPUT: a second submit succeeds. */
    session.submit(2, fake_au());
    device.on_decoder_stop = [&device]() { device.deliver(2, /*last=*/true); };
    CHECK(session.sync(2, &frame) == StatefulSession::SyncStatus::ok);
    CHECK_EQ(device.decoder_stops, 2u); /* the SECOND recovery fired and worked */
    CHECK_EQ(device.decoder_starts, 2u);
    CHECK(!session.dead());
    session.release_frame(frame);

    /* A dropped sequence keeps the session alive and streaming: sync of a
     * sequence the drain never delivers is a per-surface decode error, not
     * a dead session, and later frames still decode. */
    session.submit(3, fake_au());
    device.on_decoder_stop = [&device]() { device.deliver_empty_last(); };
    CHECK(session.sync(3, &frame) == StatefulSession::SyncStatus::decode_error);
    CHECK(!session.dead());
    session.submit(4, fake_au());
    device.on_decoder_stop = nullptr;
    device.deliver(4);
    CHECK(session.sync(4, &frame) == StatefulSession::SyncStatus::ok);
    CHECK(!session.dead());
}

void test_midstream_drain_is_codec_aware()
{
    /* The mid-stream idle/timeout DEC_CMD_STOP drain is H.264-only. VP9 emits
     * every frame as shown (in-band show_existing_frame), so it never holds a
     * mid-stream tail. AV1 deep-B DOES hold a decode-order pipeline tail, but a
     * mid-stream DEC_CMD_STOP cannot rescue it (VA3-sync-reorder): on the device
     * the drain DROPS the held frame and resets, so draining would only lose the
     * frame and break the chain -- AV1 keeps the drain OFF and falls back to
     * software for deep-B, VP9 carries browser zero-copy. With
     * Options.allow_midstream_drain = false a stalled sync waits out the hard cap
     * and fails THIS surface WITHOUT a DEC_CMD_STOP; with it true (H.264) the
     * same stall must still drain (its EOS-shaped tail flushes cleanly). finish()
     * drains the genuine tail at EOS in both. Named mutations this fails under:
     * default allow_midstream_drain false (H.264 stops draining -- Part B) or
     * dropping the !allow_midstream_drain_ gate in sync() (AV1/VP9 drain and
     * reset again -- Part A). */

    /* Part A -- AV1/VP9 (drain suppressed): a mid-stream idle stall issues NO
     * DEC_CMD_STOP; the sync waits out the hard cap and reports a per-surface
     * decode error, session alive and provisioned, chain never reset. The
     * on_decoder_stop script would deliver the frame IF a drain fired -- it
     * must never run. */
    {
        FakeDevice device;
        device.manual_delivery = true;
        device.on_decoder_stop = [&device]() { device.deliver(1, /*last=*/true); };
        StatefulSession::Options options;
        options.num_surfaces = 6;
        options.sync_timeout_ms = 40;
        options.sync_idle_ms = 5;
        options.allow_midstream_drain = false;
        StatefulSession session(device, 0x34363248, 1920, 1088, options);

        session.submit(1, fake_au());
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::decode_error);
        CHECK_EQ(device.decoder_stops, 0u); /* the reset that breaks AV1 never fired */
        CHECK_EQ(device.decoder_starts, 0u);
        CHECK_EQ(session.idle_drains(), 0u);
        CHECK_EQ(session.timeout_recoveries(), 0u);
        CHECK(!session.dead());
        CHECK(session.provisioned()); /* alive for the frames that follow */
    }

    /* Part B -- H.264 (default, drain allowed): the same idle stall STILL
     * drains, and the tail comes out. This is what the mid-stream drain is FOR
     * (D85/D86, B18/B19); it must not regress. */
    {
        FakeDevice device;
        device.manual_delivery = true;
        device.on_decoder_stop = [&device]() { device.deliver(1, /*last=*/true); };
        StatefulSession::Options options;
        options.num_surfaces = 6;
        options.sync_timeout_ms = 500;
        options.sync_idle_ms = 5;
        /* allow_midstream_drain defaults true (H.264) */
        StatefulSession session(device, 0x34363248, 1920, 1088, options);

        session.submit(1, fake_au());
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);
        CHECK_EQ(session.idle_drains(), 1u);
        CHECK_EQ(session.timeout_recoveries(), 0u);
        CHECK_EQ(device.decoder_stops, 1u);
        CHECK_EQ(device.decoder_starts, 1u);
    }

    /* Part C -- AV1/VP9 normal decode still works with the drain off: a frame
     * the decoder produces on its own is claimed with no DEC_CMD_STOP. */
    {
        FakeDevice device; /* automatic FIFO delivery */
        StatefulSession::Options options;
        options.num_surfaces = 6;
        options.sync_timeout_ms = 30;
        options.allow_midstream_drain = false;
        StatefulSession session(device, 0x34363248, 1920, 1088, options);

        session.submit(1, fake_au());
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);
        CHECK_EQ(device.decoder_stops, 0u);
    }

    /* Part D -- EOS drain is independent of the flag: finish() still issues
     * DEC_CMD_STOP to flush the genuine tail even with mid-stream drain off. */
    {
        FakeDevice device;
        device.manual_delivery = true;
        StatefulSession::Options options;
        options.num_surfaces = 6;
        options.sync_timeout_ms = 30;
        options.allow_midstream_drain = false;
        StatefulSession session(device, 0x34363248, 1920, 1088, options);

        session.submit(1, fake_au());
        device.deliver(1);
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);

        session.submit(2, fake_au());
        device.on_decoder_stop = [&device]() { device.deliver(2, /*last=*/true); };
        session.finish();
        CHECK_EQ(device.decoder_stops, 1u); /* EOS drain fires regardless of the flag */
    }
}

void test_provision_retry_on_ebusy()
{
    /* Post-crash (D88): the device refuses REQBUFS/STREAMON with EBUSY while it
     * reaps a dead client's session; the next process hits it within
     * milliseconds (B18/B19). Provisioning must retry with a bounded backoff
     * and, only on exhaustion, fail the context/first begin cleanly -- never
     * turn it into "surface is in use" on individual pictures. Named mutation
     * this fails under: drop the retry_provision wrapper (the first EBUSY
     * throws straight out). */

    /* OUTPUT REQBUFS refused twice at construction, then granted: the session
     * constructs. */
    {
        FakeDevice device;
        device.ebusy_output_provisions = 2;
        StatefulSession::Options options = fast_options();
        options.provision_retry_ms = 200;
        StatefulSession session(device, 0x34363248, 1920, 1088, options);
        CHECK_EQ(device.output_count, 4u);
        CHECK(device.output_streaming);
    }

    /* OUTPUT REQBUFS refused past the budget: construction fails (which
     * vaCreateContext turns into VA_STATUS_ERROR_OPERATION_FAILED). */
    {
        std::vector<std::string> lines;
        FakeDevice device;
        device.ebusy_output_provisions = 1000000;
        StatefulSession::Options options = fast_options();
        options.provision_retry_ms = 40;
        bool threw = false;
        try {
            StatefulSession session(
                device, 0x34363248, 1920, 1088, options, [&lines](const char* m) { lines.emplace_back(m); });
        } catch (const std::exception&) {
            threw = true;
        }
        CHECK(threw);
        bool logged = false;
        for (auto&& line : lines) {
            logged = logged || line.find("still EBUSY after") != std::string::npos;
        }
        CHECK(logged);
    }

    /* CAPTURE REQBUFS refused twice at the first SOURCE_CHANGE, then granted:
     * the decode goes through. */
    {
        FakeDevice device;
        device.ebusy_capture_provisions = 2;
        StatefulSession::Options options = fast_options();
        options.provision_retry_ms = 200;
        StatefulSession session(device, 0x34363248, 1920, 1088, options);
        session.submit(1, fake_au());
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::ok);
        CHECK(session.provisioned());
        CHECK_EQ(device.capture_count, 10u);
    }

    /* CAPTURE REQBUFS refused past the budget: the first sync fails as a
     * per-surface decode error (not "surface is in use"), the session is not
     * dead, and no drain was issued. */
    {
        std::vector<std::string> lines;
        FakeDevice device;
        device.ebusy_capture_provisions = 1000000;
        StatefulSession::Options options = fast_options();
        options.provision_retry_ms = 40;
        StatefulSession session(
            device, 0x34363248, 1920, 1088, options, [&lines](const char* m) { lines.emplace_back(m); });
        session.submit(1, fake_au());
        StatefulSession::Frame frame;
        CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::decode_error);
        CHECK(!session.dead());
        CHECK(!session.provisioned());
        CHECK_EQ(device.decoder_stops, 0u);
        bool logged = false;
        for (auto&& line : lines) {
            logged = logged || line.find("still EBUSY after") != std::string::npos;
        }
        CHECK(logged);
    }
}

void test_no_drain_before_source_change()
{
    /* D86: no timeout/idle drain may fire before the first SOURCE_CHANGE
     * has been handled -- before provisioning a drain has nowhere to
     * deliver. A sync that expires while unprovisioned must NOT issue
     * DEC_CMD_STOP; it waits for the announce and then reports a decode
     * error, session still alive. Named mutation this fails under: let the
     * sync loop enter recover_locked while !provisioned_ (a DEC_CMD_STOP
     * fires at sequence 1). */
    FakeDevice device;
    device.manual_delivery = true;
    device.source_change_pending_on_submit = false; /* the announce never comes */
    StatefulSession::Options options;
    options.num_surfaces = 6;
    options.sync_timeout_ms = 20;
    options.sync_idle_ms = 5;
    StatefulSession session(device, 0x34363248, 1920, 1088, options);

    session.submit(1, fake_au());
    StatefulSession::Frame frame;
    CHECK(session.sync(1, &frame) == StatefulSession::SyncStatus::decode_error);
    CHECK_EQ(device.decoder_stops, 0u); /* never drained before the announce */
    CHECK(!session.provisioned());
    CHECK(!session.dead());
}

void test_output_ring_growth()
{
    FakeDevice device;
    StatefulSession session(device, 0x34363248, 64, 64, fast_options());
    const uint32_t initial_size = device.output_size;

    /* An access unit larger than the ring buffers: CREATE_BUFS grows the
     * ring while streaming (7.6 point 4). */
    session.submit(1, fake_au(initial_size + 1));
    CHECK_EQ(device.create_bufs_calls, 1u);
    CHECK(device.output_size > initial_size);
    CHECK_EQ(device.stream_off_output_calls, 0u);
    CHECK_EQ(device.pending_decodes.front().bytes, initial_size + 1);
    CHECK(device.pending_decodes.front().index >= 4); /* one of the new, larger buffers */

    /* Without CREATE_BUFS the ring is reallocated once idle (STREAMOFF). */
    FakeDevice device2;
    device2.fail_create_bufs = true;
    StatefulSession session2(device2, 0x34363248, 64, 64, fast_options());
    session2.submit(1, fake_au(device2.output_size + 1));
    CHECK_EQ(device2.stream_off_output_calls, 1u);
    CHECK(device2.output_streaming);
    CHECK_EQ(device2.pending_decodes.size(), 1u);
}

void test_device_lost()
{
    FakeDevice device;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    device.manual_delivery = true;
    session.submit(1, fake_au());
    device.deliver(1);
    StatefulSession::Frame index;
    CHECK(session.sync(1, &index) == StatefulSession::SyncStatus::ok);

    /* A second picture is in flight when the device disappears. */
    session.submit(2, fake_au());
    device.lose_device = true; /* ENODEV from here on (7.6 point 6) */
    StatefulSession::Frame index2;
    CHECK(session.sync(2, &index2) == StatefulSession::SyncStatus::dead);
    CHECK(session.dead());
    /* Later syncs fail immediately. */
    CHECK(session.sync(2, &index2) == StatefulSession::SyncStatus::dead);
    /* And teardown survives a dead device. */
    session.release_frame(index);
    session.finish();
}

void test_submit_on_dead_device_throws()
{
    FakeDevice device;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());
    session.submit(1, fake_au());
    StatefulSession::Frame index;
    CHECK(session.sync(1, &index) == StatefulSession::SyncStatus::ok);

    device.lose_device = true;
    bool threw = false;
    try {
        session.submit(2, fake_au());
    } catch (const std::exception&) {
        threw = true;
    }
    CHECK(threw);
    CHECK(session.dead());
}

void test_finish_drains()
{
    FakeDevice device;
    device.manual_delivery = true;
    StatefulSession session(device, 0x34363248, 1920, 1088, fast_options());

    session.submit(1, fake_au());
    device.deliver(1);
    StatefulSession::Frame index;
    CHECK(session.sync(1, &index) == StatefulSession::SyncStatus::ok);

    session.submit(2, fake_au());
    device.on_decoder_stop = [&device]() { device.deliver(2, /*last=*/true); };
    session.finish();

    /* 7.6 point 6: DEC_CMD_STOP + drain, then STREAMOFF both + REQBUFS(0). */
    CHECK_EQ(device.decoder_stops, 1u);
    CHECK(!device.output_streaming);
    CHECK(!device.capture_streaming);
    CHECK_EQ(device.output_count, 0u);
    CHECK_EQ(device.capture_count, 0u);
}

} // namespace

int main()
{
    test_provisioning_and_mapping();
    test_out_of_order_delivery_and_stash();
    test_reorder_absorbed_in_any_sync_order();
    test_drop_sequence_recycles();
    test_capture_pool_share();
    test_stale_release_after_reprovision();
    test_timeout_drain_recovery();
    test_idle_drain_on_stream_tail();
    test_reenterable_recovery();
    test_midstream_drain_is_codec_aware();
    test_provision_retry_on_ebusy();
    test_no_drain_before_source_change();
    test_output_ring_growth();
    test_device_lost();
    test_submit_on_dead_device_throws();
    test_finish_drains();
    return check_result("test_session");
}
