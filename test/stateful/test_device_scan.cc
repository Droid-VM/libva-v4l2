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
 * P-4 (E2E-vpu.md 11.1 layer 2): the udev-free device scan. udev needs /sys
 * and /run/udev, which Firefox's RDD (media) process does not have, so
 * enumerate_devices() falls back to probing node PATHS. This test drives the
 * pure selection with a fake device list -- no /dev, no ioctl -- and pins the
 * two rules that keep the fallback picking the same node the udev walk picks:
 * an M2M capability word, AND coded OUTPUT formats that say stateful decoder.
 *
 * The fake list is the DroidVM guest's own shape (video0 camera, video1
 * decoder, video2 encoder) plus two synthetic nodes that isolate one guard
 * each, so a dropped guard cannot hide behind the other.
 *
 * Named mutations this test fails under:
 *   M1  drop the `capabilities & required_capabilities` test in
 *       select_decode_nodes  -> /dev/video7 (not an M2M node) is selected.
 *   M2  drop the `stateful_decoder` test                      -> /dev/video2
 *       (the ENCODER node) is selected, and the driver would drive the encoder
 *       as a decoder.
 *   M3  make candidate_paths() start at 1, or return the paths in any other
 *       order -> the first usable node is no longer the one udev would have
 *       returned first.
 */

#include <map>
#include <string>
#include <vector>

extern "C" {
#include <linux/videodev2.h>
}

#include "../../src/device_scan.h"
#include "check.h"

namespace {

/* V4L2M2MDevice::required_capabilities, spelled out so this test does not pull
 * in v4l2.h (and with it libva and libudev). */
constexpr uint32_t kRequired = V4L2_CAP_VIDEO_M2M | V4L2_CAP_VIDEO_M2M_MPLANE;

constexpr uint32_t kCamera = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_STREAMING;
constexpr uint32_t kM2M = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING;

/* The fake /dev: what probing each path would have found. A path that is
 * absent from the map is a node that does not exist or that the sandbox
 * refuses -- probe() returns nullopt for it, as the real one does on a failed
 * open(). */
const std::map<std::string, device_scan::NodeProbe>& fake_devices()
{
    static const std::map<std::string, device_scan::NodeProbe> devices = {
        /* The guest's three virtio-media nodes. */
        { "/dev/video0", { kCamera, false } }, /* camera */
        { "/dev/video1", { kM2M, true } }, /* stateful decoder -- the one we want */
        { "/dev/video2", { kM2M, false } }, /* encoder: M2M, but not a decoder */
        /* video3..video6 absent. */
        /* Isolates the capability guard: claims to be a stateful decoder but is
         * not an M2M node, so it can never be driven as one. */
        { "/dev/video7", { kCamera, true } },
        /* A second decoder, to pin the order. */
        { "/dev/video9", { kM2M, true } },
    };
    return devices;
}

std::optional<device_scan::NodeProbe> fake_probe(const std::string& path)
{
    auto it = fake_devices().find(path);
    if (it == fake_devices().end()) {
        return std::nullopt;
    }
    return it->second;
}

void test_candidate_paths()
{
    const auto paths = device_scan::candidate_paths(64);
    CHECK_EQ(paths.size(), 64u);
    CHECK(paths.front() == "/dev/video0");
    CHECK(paths[1] == "/dev/video1");
    CHECK(paths.back() == "/dev/video63");
    CHECK(device_scan::candidate_paths(0).empty());
}

void test_selection_picks_the_decoder_nodes()
{
    const auto selected = device_scan::select_decode_nodes(device_scan::candidate_paths(64), kRequired, fake_probe);

    /* Exactly the two stateful M2M decoders, in ascending node order: the
     * camera, the encoder and the non-M2M impostor are all out. */
    CHECK_EQ(selected.size(), 2u);
    REQUIRE(selected.size() == 2);
    CHECK(selected[0] == "/dev/video1");
    CHECK(selected[1] == "/dev/video9");

    for (auto&& path : selected) {
        CHECK(path != "/dev/video0");
        CHECK(path != "/dev/video2");
        CHECK(path != "/dev/video7");
    }
}

void test_no_devices_is_empty_not_a_crash()
{
    /* A host with no video nodes at all: the scan must return an empty list so
     * vaInitialize fails cleanly with "No usable V4L2 M2M decode device
     * found", exactly as the udev path does. */
    const auto selected = device_scan::select_decode_nodes(device_scan::candidate_paths(64), kRequired,
        [](const std::string&) { return std::optional<device_scan::NodeProbe>(); });
    CHECK(selected.empty());
}

void test_selection_is_reusable_for_the_udev_walk()
{
    /* enumerate_devices() runs the SAME selection over the node names udev
     * handed it, so a udev list that is not sorted still yields the udev order
     * (the caller's order is preserved, never re-sorted). */
    const std::vector<std::string> udev_order = { "/dev/video9", "/dev/video2", "/dev/video1" };
    const auto selected = device_scan::select_decode_nodes(udev_order, kRequired, fake_probe);
    CHECK_EQ(selected.size(), 2u);
    REQUIRE(selected.size() == 2);
    CHECK(selected[0] == "/dev/video9");
    CHECK(selected[1] == "/dev/video1");
}

} // namespace

int main()
{
    test_candidate_paths();
    test_selection_picks_the_decoder_nodes();
    test_no_devices_is_empty_not_a_crash();
    test_selection_is_reusable_for_the_udev_walk();
    return check_result("test_device_scan");
}
