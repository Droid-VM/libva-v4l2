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
#include <optional>
#include <set>

extern "C" {
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <va/va.h>
}

/* The AV1 profile control postdates this tree's sanitised kernel headers
 * (include/linux/v4l2-controls.h stops at HEVC). The values are the host's
 * /usr/include/linux/v4l2-controls.h ones -- the same numbers the device side
 * publishes -- and the guard means the day the guest headers catch up, theirs
 * win with no edit here. */
#ifndef V4L2_CID_MPEG_VIDEO_AV1_PROFILE
#define V4L2_CID_MPEG_VIDEO_AV1_PROFILE (V4L2_CID_CODEC_BASE + 655)
enum v4l2_mpeg_video_av1_profile {
    V4L2_MPEG_VIDEO_AV1_PROFILE_MAIN = 0,
    V4L2_MPEG_VIDEO_AV1_PROFILE_HIGH = 1,
    V4L2_MPEG_VIDEO_AV1_PROFILE_PROFESSIONAL = 2,
};
#endif

/*
 * VA1b (VPU_DESIGN.md 7.6 point 2): the profiles the stateful bridge
 * advertises are (what the bridge implements) INTERSECT (what the device's
 * profile menu lists) -- never the hardcoded list alone, and never what the
 * device lists alone.
 *
 * VA1 shipped a knowingly temporary hardcode: the fourcc was present on the
 * OUTPUT queue, so the bridge claimed H.264 ConstrainedBaseline/Main/High
 * whatever the MediaCodec behind the device could actually do. The device now
 * publishes the truth as a read-only V4L2 menu control per coded format
 * (V4L2_CID_MPEG_VIDEO_{H264,HEVC,VP9,AV1}_PROFILE, the way the encoder device
 * already does: QUERYMENU lists only the supported items and answers EINVAL
 * for the rest), and this module turns that menu into the VAProfile set.
 *
 * Two rules hold in both directions:
 *
 *   - the device can only NARROW. A menu item the bridge does not implement
 *     (H.264 High 10, VP9 Profile 2, AV1 Professional) must not reach a
 *     client just because the codec behind the device could decode it -- the
 *     bitstream synthesis (7.6 point 3) is what would have to be written
 *     first. profile_for_menu_value() therefore maps only the implemented
 *     values and drops the rest;
 *
 *   - a device that says nothing cannot narrow anything. An older device has
 *     no such control at all (QUERY_EXT_CTRL answers ENOTTY/EINVAL, which the
 *     probe reports as nullopt) and a device can have the control while
 *     listing no item; both are "no information", and the implemented set
 *     stands, exactly as it did in VA1.
 *
 * Everything here is pure: menu values in, VAProfiles out, no ioctl and no
 * device. V4L2M2MDevice does the probing (src/v4l2.cc) and caches the menu per
 * coded fourcc; the stateful contexts call supported_profiles() with the set
 * they implement.
 */
namespace profile_menu {

/* What one QUERYMENU sweep found: the menu values the device accepted.
 * nullopt = the device has no such control (an older device), which is NOT
 * the same as a control that lists nothing -- but both leave the implemented
 * set untouched. */
using DeviceMenu = std::optional<std::set<uint32_t>>;

/* The profile control that describes one coded OUTPUT format, for the coded
 * formats this bridge implements. nullopt = a format with no profile control
 * here, which is either a raw format or a codec the bridge does not decode
 * yet: when VA2's HEVC context lands, V4L2_PIX_FMT_HEVC ->
 * V4L2_CID_MPEG_VIDEO_HEVC_PROFILE joins this table together with its menu
 * values below, and not before -- an entry here with no mapping below would
 * advertise nothing anyway, but an entry with a mapping and no context would
 * advertise a profile nothing can decode. */
std::optional<uint32_t> control_for_format(uint32_t pixelformat);

/* One menu value -> the VAProfile it means. nullopt = a profile this bridge
 * does not implement; it is dropped, never advertised. */
std::optional<VAProfile> profile_for_menu_value(uint32_t control_id, uint32_t value);

/* The whole menu, mapped. Values the bridge does not implement drop out, so
 * an empty result can mean either "the device listed nothing" or "the device
 * listed only profiles we cannot decode" -- which is why the fallback below
 * is decided on the raw menu, not on this. */
std::set<VAProfile> device_profiles(uint32_t control_id, const std::set<uint32_t>& menu_values);

/* The rule. `implemented` is what the bridge can actually decode for this
 * coded format (the VA1 list); `menu` is what the device reported. nullopt or
 * an empty menu -> `implemented` unchanged (the fallback); otherwise the
 * intersection, which can legitimately be empty when the device lists only
 * profiles this bridge does not implement. */
std::set<VAProfile> supported_profiles(
    uint32_t control_id, const std::set<VAProfile>& implemented, const DeviceMenu& menu);

} // namespace profile_menu
