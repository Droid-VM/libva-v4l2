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
#include <span>

namespace stateful {

/*
 * A claimed decoded frame's NV12 planes for the image path (copy or derive).
 * This describes the decoded CAPTURE buffer, which is codec-agnostic (7.6/7.7):
 * every stateful context -- H.264, VP9 -- produces the same NV12 container, so
 * the vaGetImage/vaDeriveImage plumbing in image.cc consumes one shared view
 * regardless of which codec filled it. StatefulH264Context historically nested
 * its own identical struct; VA2a introduces this shared one for the VP9 path
 * (VA2-survey (C): the surface/image plumbing is codec-agnostic and reused).
 */
struct FrameView {
    std::span<uint8_t> luma;
    std::span<uint8_t> chroma;
    unsigned pitch;
    unsigned coded_width;
    unsigned coded_height;
    /* Non-empty when both planes live in one contiguous mapping
     * (single-plane NV12): the whole mapping, for vaDeriveImage. */
    std::span<uint8_t> contiguous;
};

} // namespace stateful
