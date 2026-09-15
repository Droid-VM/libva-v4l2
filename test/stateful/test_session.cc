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
    test_drop_sequence_recycles();
    test_stale_release_after_reprovision();
    test_timeout_drain_recovery();
    test_output_ring_growth();
    test_device_lost();
    test_submit_on_dead_device_throws();
    test_finish_drains();
    return check_result("test_session");
}
