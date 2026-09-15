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
 * D83: ffmpeg's vaapi hwaccel is HWACCEL_CAP_ASYNC_SAFE, so one frame thread
 * runs vaBeginPicture/vaEndPicture (submit + release_frame) while another
 * runs vaSyncSurface on the same context. B18 caught std::out_of_range in
 * queue_capture from exactly that overlap (~1 run in 9). This test drives
 * StatefulSession from two threads for 1000 frames: one submitting and
 * releasing (the begin-picture path), one syncing in display order the way
 * every real client claims (the decode itself still runs ahead into the
 * stash). It must finish with no throw, no recovery, and every CAPTURE
 * buffer back on the queue. Run it under -fsanitize=thread to catch the
 * races the timing hides.
 *
 * Named mutation this test fails under: remove the mutex_ lock from
 * StatefulSession::release_frame -- under -fsanitize=thread the run aborts
 * on the data race (verified; the plain build corrupts the maps only
 * intermittently, which is exactly D83's 1-run-in-9).
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <thread>
#include <vector>

#include "../../src/stateful/session.h"
#include "check.h"
#include "fake_device.h"

using stateful::StatefulSession;

namespace {

/* FakeDevice itself is single-threaded on purpose; this wrapper serialises
 * every device call on one internal mutex so the test exercises
 * StatefulSession's locking, not the fake's. */
class LockedFakeDevice final : public stateful::StatefulDevice {
public:
    FakeDevice inner;
    std::mutex m;

    void set_output_format(uint32_t pf, uint32_t w, uint32_t h, uint32_t size) override
    {
        std::lock_guard<std::mutex> g(m);
        inner.set_output_format(pf, w, h, size);
    }
    uint32_t output_buffer_size() override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.output_buffer_size();
    }
    stateful::CaptureFormat capture_format() override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.capture_format();
    }
    unsigned request_output_buffers(unsigned count) override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.request_output_buffers(count);
    }
    std::pair<unsigned, unsigned> create_output_buffers(unsigned count, uint32_t size) override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.create_output_buffers(count, size);
    }
    std::span<uint8_t> output_plane(unsigned index) override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.output_plane(index);
    }
    unsigned request_capture_buffers(unsigned count) override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.request_capture_buffers(count);
    }
    std::span<uint8_t> capture_plane(unsigned index, unsigned plane) override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.capture_plane(index, plane);
    }
    void queue_output(unsigned index, uint64_t sequence, unsigned bytes) override
    {
        std::lock_guard<std::mutex> g(m);
        inner.queue_output(index, sequence, bytes);
    }
    std::optional<uint32_t> dequeue_output() override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.dequeue_output();
    }
    void queue_capture(unsigned index) override
    {
        std::lock_guard<std::mutex> g(m);
        inner.queue_capture(index);
    }
    std::optional<stateful::DequeuedCapture> dequeue_capture() override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.dequeue_capture();
    }
    int min_buffers_for_capture() override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.min_buffers_for_capture();
    }
    void subscribe_events() override
    {
        std::lock_guard<std::mutex> g(m);
        inner.subscribe_events();
    }
    std::optional<stateful::DeviceEvent> dequeue_event() override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.dequeue_event();
    }
    void decoder_stop() override
    {
        std::lock_guard<std::mutex> g(m);
        inner.decoder_stop();
    }
    void decoder_start() override
    {
        std::lock_guard<std::mutex> g(m);
        inner.decoder_start();
    }
    void stream_output(bool enable) override
    {
        std::lock_guard<std::mutex> g(m);
        inner.stream_output(enable);
    }
    void stream_capture(bool enable) override
    {
        std::lock_guard<std::mutex> g(m);
        inner.stream_capture(enable);
    }
    bool wait(int timeout_ms, bool include_output) override
    {
        std::lock_guard<std::mutex> g(m);
        return inner.wait(timeout_ms, include_output);
    }
};

std::vector<uint8_t> fake_au(size_t size = 512)
{
    return std::vector<uint8_t>(size, 0xab);
}

constexpr uint64_t kIterations = 1000;

void test_concurrent_submit_and_sync()
{
    LockedFakeDevice device;
    StatefulSession::Options options;
    options.num_surfaces = 8;
    options.output_ring_size = 4;
    options.sync_timeout_ms = 5000; /* generous: no recovery may fire */
    StatefulSession session(device, 0x34363248, 320, 240, options);

    std::atomic<bool> failed { false };
    std::atomic<uint64_t> synced { 0 };
    std::mutex release_mutex;
    std::condition_variable release_cv;
    std::deque<StatefulSession::Frame> to_release;
    /* A real client cycles a finite surface pool; an unbounded backlog of
     * unreleased frames would starve the decoder of CAPTURE buffers. */
    constexpr size_t kMaxUnreleased = 6;

    /* The begin-picture thread: submit every access unit, and re-queue the
     * frames the sync thread finished with (release_frame from a different
     * thread than sync -- the exact D83 overlap). */
    std::thread producer([&] {
        try {
            for (uint64_t sequence = 1; sequence <= kIterations; sequence++) {
                for (;;) {
                    StatefulSession::Frame frame;
                    {
                        std::lock_guard<std::mutex> g(release_mutex);
                        if (to_release.empty()) {
                            break;
                        }
                        frame = to_release.front();
                        to_release.pop_front();
                    }
                    release_cv.notify_all();
                    session.release_frame(frame);
                }
                session.submit(sequence, fake_au());
            }
        } catch (...) {
            failed = true;
        }
    });

    /* The sync thread: claim frames in display order (what ffmpeg, mpv and
     * gst all do); hand most releases to the producer thread, but never let
     * the backlog exceed the surface pool a real client would cycle --
     * on overflow release directly, as a client whose submit thread is busy
     * still returns surfaces from the consuming side. */
    std::thread consumer([&] {
        try {
            StatefulSession::Frame index;
            for (uint64_t sequence = 1; sequence <= kIterations; sequence++) {
                if (session.sync(sequence, &index) != StatefulSession::SyncStatus::ok) {
                    failed = true;
                    return;
                }
                bool release_directly = false;
                {
                    std::unique_lock<std::mutex> g(release_mutex);
                    release_cv.wait_for(
                        g, std::chrono::milliseconds(2), [&] { return to_release.size() < kMaxUnreleased; });
                    if (to_release.size() < kMaxUnreleased) {
                        to_release.push_back(index);
                    } else {
                        release_directly = true;
                    }
                }
                if (release_directly) {
                    session.release_frame(index);
                }
                synced += 1;
            }
        } catch (...) {
            failed = true;
        }
    });

    producer.join();
    consumer.join();
    CHECK(!failed);
    CHECK_EQ(synced.load(), kIterations);
    CHECK_EQ(session.timeout_recoveries(), 0u);

    /* Consistent maps at the end: release the tail claims, then every
     * CAPTURE buffer must be back on the device queue exactly once. */
    for (;;) {
        StatefulSession::Frame frame;
        {
            std::lock_guard<std::mutex> g(release_mutex);
            if (to_release.empty()) {
                break;
            }
            frame = to_release.front();
            to_release.pop_front();
        }
        session.release_frame(frame);
    }
    {
        std::lock_guard<std::mutex> g(device.m);
        CHECK_EQ(device.inner.capture_count, 12u); /* min 4 + min(8, 8) */
        CHECK_EQ(device.inner.free_captures.size(), 12u);
        CHECK_EQ(device.inner.pending_decodes.size(), 0u);
    }

    session.finish();
    {
        std::lock_guard<std::mutex> g(device.m);
        CHECK_EQ(device.inner.capture_count, 0u);
        CHECK_EQ(device.inner.output_count, 0u);
    }
}

} // namespace

int main()
{
    test_concurrent_submit_and_sync();
    return check_result("test_session_threads");
}
