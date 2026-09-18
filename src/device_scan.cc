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

#include "device_scan.h"

namespace device_scan {

std::vector<std::string> candidate_paths(unsigned count)
{
    std::vector<std::string> result;
    result.reserve(count);
    for (unsigned i = 0; i < count; i += 1) {
        result.push_back("/dev/video" + std::to_string(i));
    }
    return result;
}

std::vector<std::string> select_decode_nodes(
    const std::vector<std::string>& candidates, uint32_t required_capabilities, const ProbeFn& probe)
{
    std::vector<std::string> result;
    for (auto&& path : candidates) {
        const auto node = probe(path);
        if (!node) {
            continue;
        }
        if ((node->capabilities & required_capabilities) == 0) {
            continue; /* not an M2M node: a camera, or a bare capture device */
        }
        if (!node->stateful_decoder) {
            continue; /* an M2M node that is not a stateful decoder: the encoder */
        }
        result.push_back(path);
    }
    return result;
}

} // namespace device_scan
