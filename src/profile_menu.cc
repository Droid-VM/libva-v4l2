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

#include "profile_menu.h"

namespace profile_menu {

std::optional<uint32_t> control_for_format(uint32_t pixelformat)
{
    switch (pixelformat) {
    case V4L2_PIX_FMT_H264:
        return V4L2_CID_MPEG_VIDEO_H264_PROFILE;
    case V4L2_PIX_FMT_VP9:
        return V4L2_CID_MPEG_VIDEO_VP9_PROFILE;
    case V4L2_PIX_FMT_AV1:
        return V4L2_CID_MPEG_VIDEO_AV1_PROFILE;
    default:
        return std::nullopt;
    }
}

std::optional<VAProfile> profile_for_menu_value(uint32_t control_id, uint32_t value)
{
    switch (control_id) {
    case V4L2_CID_MPEG_VIDEO_H264_PROFILE:
        switch (value) {
        case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
            return VAProfileH264ConstrainedBaseline;
        case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
            return VAProfileH264Main;
        case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
            return VAProfileH264High;
        default:
            /* BASELINE, EXTENDED, HIGH_10, HIGH_422, the intra and scalable
             * profiles: not implemented here. VAProfileH264Baseline is
             * deprecated in libva and this backend never claimed it, so even
             * plain BASELINE stays out -- a device that lists it and nothing
             * else advertises nothing, which is the honest answer. */
            return std::nullopt;
        }

    case V4L2_CID_MPEG_VIDEO_VP9_PROFILE:
        switch (value) {
        case V4L2_MPEG_VIDEO_VP9_PROFILE_0:
            return VAProfileVP9Profile0;
        case V4L2_MPEG_VIDEO_VP9_PROFILE_1:
            return VAProfileVP9Profile1;
        case V4L2_MPEG_VIDEO_VP9_PROFILE_2:
            return VAProfileVP9Profile2;
        case V4L2_MPEG_VIDEO_VP9_PROFILE_3:
            return VAProfileVP9Profile3;
        default:
            return std::nullopt;
        }

    case V4L2_CID_MPEG_VIDEO_AV1_PROFILE:
        switch (value) {
        case V4L2_MPEG_VIDEO_AV1_PROFILE_MAIN:
            return VAProfileAV1Profile0;
        case V4L2_MPEG_VIDEO_AV1_PROFILE_HIGH:
            return VAProfileAV1Profile1;
        default:
            /* PROFESSIONAL is AV1 profile 2, which libva has no VAProfile
             * for at all. */
            return std::nullopt;
        }

    default:
        return std::nullopt;
    }
}

std::set<VAProfile> device_profiles(uint32_t control_id, const std::set<uint32_t>& menu_values)
{
    std::set<VAProfile> result;
    for (auto&& value : menu_values) {
        if (const auto profile = profile_for_menu_value(control_id, value); profile) {
            result.insert(*profile);
        }
    }
    return result;
}

std::set<VAProfile> supported_profiles(
    uint32_t control_id, const std::set<VAProfile>& implemented, const DeviceMenu& menu)
{
    /* No control, or a control that listed no item: the device told us
     * nothing, so the implemented set stands (the VA1 behaviour). Note this
     * is decided on the RAW menu -- a device that listed items we cannot map
     * did tell us something, and that something is "none of these". */
    if (!menu || menu->empty()) {
        return implemented;
    }

    const auto device = device_profiles(control_id, *menu);

    std::set<VAProfile> result;
    for (auto&& profile : implemented) {
        if (device.contains(profile)) {
            result.insert(profile);
        }
    }
    return result;
}

} // namespace profile_menu
