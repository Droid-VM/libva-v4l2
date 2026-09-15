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
 * runs vaRenderPicture/vaEndPicture -- which call vaCreateBuffer/
 * vaDestroyBuffer per picture and read driver_data->buffers via '.at()' --
 * while another thread runs vaSyncSurface/vaDeriveImage/vaGetImage/vaMapBuffer,
 * which also read those maps. B18/B19 caught the lock-free 'contains() then
 * .at()' racing the concurrent insert/erase and aborting with
 * std::out_of_range 'map::at' (~1 default-budget decode in 15 in B19 §6.1).
 *
 * This test drives the real driver vtable entry points against one DriverData
 * from two threads for 2000 iterations: thread A churns the buffers map
 * (vaCreateBuffer then vaDestroyBuffer, the exact per-picture insert/erase),
 * thread B reads it (vaMapBuffer/vaBufferInfo/vaUnmapBuffer on a stable set of
 * buffers). It must finish with no exception and every read consistent.
 *
 * Run under -fsanitize=thread. Named mutation this test fails under: drop the
 * std::shared_lock from mapBuffer (buffer.cc) -- TSan reports the data race
 * between vaCreateBuffer/vaDestroyBuffer's std::map insert/erase (which
 * rebalances the tree) and mapBuffer's find. The same shared lock guards
 * renderPicture's buffers.at() and syncSurface/deriveImage's surfaces.at();
 * those readers need a live decode context (a real V4L2 device) and are
 * exercised on the phone by B20, but they share this map and this lock.
 */

#include <atomic>
#include <thread>
#include <vector>

extern "C" {
#include <va/va.h>
#include <va/va_backend.h>
}

#include "../../src/buffer.h"
#include "../../src/driver.h"
#include "check.h"

namespace {

constexpr int kIterations = 20000;
constexpr int kStableBuffers = 16;
/* The churn thread keeps up to this many buffers live at once, so its
 * insert/erase rebalances the map through the same interior nodes the reader
 * traverses -- keys interleave with the stable set (smallest_free_key reuses
 * the low ids the stable set leaves free). */
constexpr int kChurnLive = 24;

void test_concurrent_buffer_map()
{
    /* No devices: the buffers map is independent of any V4L2 node, so an
     * empty DriverData is enough to exercise the map's locking. */
    DriverData driver_data({});
    VADriverContext va_context = {};
    va_context.pDriverData = &driver_data;

    /* A stable set of buffers thread B always finds. */
    std::vector<VABufferID> stable;
    for (int i = 0; i < kStableBuffers; i++) {
        VABufferID id = VA_INVALID_ID;
        CHECK(createBuffer(&va_context, 0, VASliceDataBufferType, 512, 1, nullptr, &id) == VA_STATUS_SUCCESS);
        stable.push_back(id);
    }
    /* The id range the reader sweeps: the stable ids plus the range the churn
     * thread cycles through, so find() walks the mutating region. */
    const VABufferID max_id = static_cast<VABufferID>(kStableBuffers + kChurnLive + 4);

    std::atomic<bool> failed { false };

    /* The begin/render/end frame thread: create per-picture buffers and
     * destroy them (vaCreateBuffer/vaDestroyBuffer, the exact per-picture
     * insert/erase), keeping a rolling window live so the tree keeps
     * rebalancing under thread B's reads. */
    std::thread churn([&] {
        try {
            std::vector<VABufferID> live;
            for (int i = 0; i < kIterations; i++) {
                VABufferID id = VA_INVALID_ID;
                if (createBuffer(&va_context, 0, VASliceParameterBufferType, 128, 1, nullptr, &id)
                    != VA_STATUS_SUCCESS) {
                    failed = true;
                    return;
                }
                live.push_back(id);
                if (static_cast<int>(live.size()) > kChurnLive) {
                    if (destroyBuffer(&va_context, live.front()) != VA_STATUS_SUCCESS) {
                        failed = true;
                        return;
                    }
                    live.erase(live.begin());
                }
            }
            for (VABufferID id : live) {
                destroyBuffer(&va_context, id);
            }
        } catch (...) {
            failed = true;
        }
    });

    /* The consumer thread: map/info/unmap over the whole id range, including
     * ids the churn thread is inserting/erasing. A read that misses a churn id
     * is fine (INVALID_BUFFER); the stable ids must always resolve. This is the
     * exact 'find/at on a std::map another thread mutates' overlap of D83. */
    std::thread reader([&] {
        try {
            for (int i = 0; i < kIterations; i++) {
                for (VABufferID id = 1; id <= max_id; id++) {
                    void* map = nullptr;
                    VAStatus st = mapBuffer(&va_context, id, &map);
                    if (st != VA_STATUS_SUCCESS && st != VA_STATUS_ERROR_INVALID_BUFFER) {
                        failed = true;
                        return;
                    }
                    VABufferType type;
                    unsigned size = 0;
                    unsigned count = 0;
                    bufferInfo(&va_context, id, &type, &size, &count);
                    unmapBuffer(&va_context, id);
                }
                bool stable_ok = true;
                for (VABufferID id : stable) {
                    void* map = nullptr;
                    if (mapBuffer(&va_context, id, &map) != VA_STATUS_SUCCESS || map == nullptr) {
                        stable_ok = false;
                    }
                }
                if (!stable_ok) {
                    failed = true;
                    return;
                }
            }
        } catch (...) {
            failed = true;
        }
    });

    churn.join();
    reader.join();
    CHECK(!failed);

    /* The stable buffers are still there and are the only ones left. */
    for (VABufferID id : stable) {
        CHECK(destroyBuffer(&va_context, id) == VA_STATUS_SUCCESS);
    }
    CHECK(driver_data.buffers.empty());
}

} // namespace

int main()
{
    test_concurrent_buffer_map();
    return check_result("test_driver_maps");
}
