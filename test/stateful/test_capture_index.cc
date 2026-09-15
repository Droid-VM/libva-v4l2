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
 * D83: V4L2StatefulDevice::capture_plane resolves a CAPTURE index and plane
 * against the provisioned pool. A stale index (a binding surviving a
 * re-provision) must fail with a runtime_error that names the bug, never the
 * bare container exception (std::out_of_range "map::at" from a container, or
 * vector::_M_range_check) that terminates the process. These are the checked
 * lookups; this test drives them directly (no device needed).
 *
 * Named mutation this test fails under: make ensure_capture_index a no-op --
 * an out-of-range index no longer throws the named error.
 */

#include <string>

#include "../../src/stateful/v4l2_device.h"
#include "check.h"

namespace {

std::string caught_message(void (*fn)(unsigned, std::size_t), unsigned value, std::size_t bound)
{
    try {
        fn(value, bound);
    } catch (const std::runtime_error& e) {
        return e.what();
    } catch (...) {
        return "<non-runtime_error>";
    }
    return "<no throw>";
}

void test_capture_index_bounds()
{
    /* In range: no throw. */
    bool threw = false;
    try {
        stateful::ensure_capture_index(0, 27);
        stateful::ensure_capture_index(26, 27);
    } catch (...) {
        threw = true;
    }
    CHECK(!threw);

    /* Index == pool size and beyond: the named runtime_error, never a bare
     * container exception. */
    const std::string at_bound = caught_message(stateful::ensure_capture_index, 27, 27);
    CHECK(at_bound.find("CAPTURE index 27 outside the provisioned pool of 27") != std::string::npos);
    const std::string beyond = caught_message(stateful::ensure_capture_index, 40, 27);
    CHECK(beyond.find("CAPTURE index 40 outside the provisioned pool of 27") != std::string::npos);
    CHECK(beyond.find("map::at") == std::string::npos);

    /* An empty pool (the re-provision-to-zero window): index 0 is out of
     * range and fails cleanly rather than indexing an empty container. */
    const std::string empty_pool = caught_message(stateful::ensure_capture_index, 0, 0);
    CHECK(empty_pool.find("CAPTURE index 0 outside the provisioned pool of 0") != std::string::npos);
}

void test_capture_plane_bounds()
{
    bool threw = false;
    try {
        stateful::ensure_capture_plane(0, 1); /* NV12 single plane */
        stateful::ensure_capture_plane(1, 2); /* NV12 two planes */
    } catch (...) {
        threw = true;
    }
    CHECK(!threw);

    const std::string msg = caught_message(stateful::ensure_capture_plane, 1, 1);
    CHECK(msg.find("CAPTURE plane 1 outside the provisioned pool of 1") != std::string::npos);
    CHECK(msg.find("map::at") == std::string::npos);
}

} // namespace

int main()
{
    test_capture_index_bounds();
    test_capture_plane_bounds();
    return check_result("test_capture_index");
}
