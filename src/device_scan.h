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

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

/*
 * The udev-free device scan (P-4, E2E-vpu.md 11.1 layer 2).
 *
 * V4L2M2MDevice::enumerate_devices() walks udev, and udev needs /sys and
 * /run/udev. Firefox's RDD (media) process has neither: its file broker
 * (SandboxBrokerPolicyFactory::GetRDDPolicy) grants /dev/video* M2M nodes --
 * it enumerates and QUERYCAPs them in the PARENT and adds the M2M ones by
 * path -- but no /sys tree and no /run/udev. So udev enumerates nothing
 * there, vaInitialize fails with "No usable V4L2 M2M decode device found",
 * and a shipped browser silently decodes in software.
 *
 * The fallback therefore probes a fixed, bounded set of node PATHS with
 * open()+VIDIOC_QUERYCAP, which is exactly what that broker is prepared to
 * answer. Listing /dev would in fact also be allowed -- the same function ends
 * with AddPath(rdonly, "/dev"), "FFmpeg V4L2 needs to list /dev to find V4L2
 * devices" -- but nothing here depends on that, which is the point: the sweep
 * works in any process that can open the node at all, including one whose
 * broker grants the node and nothing else. The selection below is the pure
 * part -- the same capability and stateful-decoder test the udev walk applies
 * -- so it can be driven from a unit test with a fake device list and no /dev
 * at all.
 */
namespace device_scan {

/* What probing one node found: the QUERYCAP capability word the udev walk
 * would have read, and whether the node's coded OUTPUT formats mark it a
 * stateful decoder (V4L2M2MDevice::stateful_decoder()'s rule). */
struct NodeProbe {
    uint32_t capabilities;
    bool stateful_decoder;
};

/* nullopt = the node does not exist, cannot be opened (the sandbox refuses a
 * camera node), or does not answer VIDIOC_QUERYCAP. */
using ProbeFn = std::function<std::optional<NodeProbe>(const std::string&)>;

/* /dev/video0 .. /dev/video<count-1>, in ascending order: the same order
 * udev's video4linux walk returns, so the node the driver picks first does
 * not change when the fallback is the one that found it. */
std::vector<std::string> candidate_paths(unsigned count);

/* The nodes to hand to DriverData, in candidate order. A node is kept when it
 * is an M2M node (capabilities & required_capabilities) AND its coded OUTPUT
 * formats say stateful decoder -- the encoder node fails the second test (its
 * OUTPUT formats are raw and carry no V4L2_FMT_FLAG_DYN_RESOLUTION) and a
 * camera node fails the first, which is how the udev walk already tells the
 * three virtio-media nodes apart. */
std::vector<std::string> select_decode_nodes(
    const std::vector<std::string>& candidates, uint32_t required_capabilities, const ProbeFn& probe);

} // namespace device_scan
