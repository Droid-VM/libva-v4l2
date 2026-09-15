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

#include "vp9_bitstream.h"

#include <cstddef>
#include <stdexcept>

namespace stateful {

void VP9AccessUnitBuilder::set_picture_parameters(const VADecPictureParameterBufferVP9& picture)
{
    frame_width_ = picture.frame_width;
    frame_height_ = picture.frame_height;
}

void VP9AccessUnitBuilder::add_slice_parameters(std::span<const VASliceParameterBufferVP9> slices)
{
    slice_params_.insert(slice_params_.end(), slices.begin(), slices.end());
}

void VP9AccessUnitBuilder::add_slice_data(std::span<const uint8_t> data)
{
    slice_data_.insert(slice_data_.end(), data.begin(), data.end());
}

std::vector<uint8_t> VP9AccessUnitBuilder::finish()
{
    std::vector<uint8_t> access_unit;

    if (slice_data_.empty()) {
        reset_picture();
        return access_unit;
    }

    if (slice_params_.empty()) {
        /* No slice parameters: the whole slice-data buffer is the frame. */
        access_unit = slice_data_;
        reset_picture();
        return access_unit;
    }

    /* Honour each slice's declared byte range; forward the bytes verbatim. */
    for (const auto& slice : slice_params_) {
        const size_t offset = slice.slice_data_offset;
        const size_t size = slice.slice_data_size;
        if (offset > slice_data_.size() || size > slice_data_.size() - offset) {
            reset_picture();
            throw std::runtime_error("VP9 slice_data_offset/size runs past the slice-data buffer");
        }
        access_unit.insert(access_unit.end(), slice_data_.begin() + offset, slice_data_.begin() + offset + size);
    }

    reset_picture();
    return access_unit;
}

void VP9AccessUnitBuilder::reset_picture()
{
    slice_data_.clear();
    slice_params_.clear();
    /* frame_width_/frame_height_ are sticky: a client may repeat them, and they
     * are never part of the forwarded access unit. */
}

} // namespace stateful
