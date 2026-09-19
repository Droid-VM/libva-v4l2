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
 * VA1b (VPU_DESIGN.md 7.6 point 2): the profiles the stateful bridge
 * advertises = (what the bridge implements) INTERSECT (what the device's
 * profile menu listed). This drives the pure half -- menu values in,
 * VAProfiles out -- with no device and no ioctl.
 *
 * Named mutations this test fails under:
 *   M1  return the device's set instead of the intersection -> a device that
 *       lists H.264 High 10 (or VP9 Profile 2, or AV1 HIGH) makes the bridge
 *       advertise a profile nothing here can synthesise, and the client gets
 *       a black picture instead of a clean software fallback.
 *   M2  return the implemented set instead of the intersection -> VA1's
 *       hardcode survives: a device that lists only Constrained Baseline
 *       still gets Main and High claimed for it.
 *   M3  decide the "device said nothing" fallback on the MAPPED set instead
 *       of the raw menu -> a device that lists only profiles we do not
 *       implement is mistaken for a silent device, and the implemented set is
 *       advertised in full: the exact opposite of what that device said.
 *   M4  drop the nullopt (no control) fallback -> an older device with no
 *       profile menu advertises nothing at all and every client falls back to
 *       software.
 *   M5  map H.264 menu value 0 (BASELINE) or 5 (HIGH_10), or AV1
 *       PROFESSIONAL -> profiles this bridge does not implement leak out.
 */

#include <optional>
#include <set>

extern "C" {
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>

#include <va/va.h>
}

#include "../../src/profile_menu.h"
#include "check.h"

namespace {

/* What each stateful context implements today (h264_context.cc,
 * vp9_context.cc, av1_context.cc), spelled out so this test pins the rule
 * even if a context's own list changes. */
const std::set<VAProfile> kH264Implemented { VAProfileH264ConstrainedBaseline, VAProfileH264Main, VAProfileH264High };
const std::set<VAProfile> kVP9Implemented { VAProfileVP9Profile0 };
const std::set<VAProfile> kAV1Implemented { VAProfileAV1Profile0 };

void test_control_for_format()
{
    CHECK(profile_menu::control_for_format(V4L2_PIX_FMT_H264)
        == std::optional<uint32_t>(V4L2_CID_MPEG_VIDEO_H264_PROFILE));
    CHECK(
        profile_menu::control_for_format(V4L2_PIX_FMT_VP9) == std::optional<uint32_t>(V4L2_CID_MPEG_VIDEO_VP9_PROFILE));
    CHECK(
        profile_menu::control_for_format(V4L2_PIX_FMT_AV1) == std::optional<uint32_t>(V4L2_CID_MPEG_VIDEO_AV1_PROFILE));

    /* A coded format no context implements yet (HEVC is VA2's) and a raw
     * format: nothing to probe, so nothing is probed. */
    CHECK(!profile_menu::control_for_format(V4L2_PIX_FMT_HEVC).has_value());
    CHECK(!profile_menu::control_for_format(V4L2_PIX_FMT_NV12).has_value());
}

void test_menu_value_mapping()
{
    /* The three H.264 values this bridge implements. */
    CHECK(profile_menu::profile_for_menu_value(
              V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE)
        == std::optional<VAProfile>(VAProfileH264ConstrainedBaseline));
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_MAIN)
        == std::optional<VAProfile>(VAProfileH264Main));
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_H264_PROFILE, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH)
        == std::optional<VAProfile>(VAProfileH264High));

    /* Everything else the menu can carry is dropped, not guessed at. */
    for (auto&& value : { V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE, V4L2_MPEG_VIDEO_H264_PROFILE_EXTENDED,
             V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_422,
             V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH, V4L2_MPEG_VIDEO_H264_PROFILE_STEREO_HIGH }) {
        CHECK(!profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_H264_PROFILE, value).has_value());
    }

    /* VP9: the menu index IS the profile number. */
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, V4L2_MPEG_VIDEO_VP9_PROFILE_0)
        == std::optional<VAProfile>(VAProfileVP9Profile0));
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, V4L2_MPEG_VIDEO_VP9_PROFILE_1)
        == std::optional<VAProfile>(VAProfileVP9Profile1));
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, V4L2_MPEG_VIDEO_VP9_PROFILE_2)
        == std::optional<VAProfile>(VAProfileVP9Profile2));
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, V4L2_MPEG_VIDEO_VP9_PROFILE_3)
        == std::optional<VAProfile>(VAProfileVP9Profile3));
    CHECK(!profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, 4).has_value());

    /* AV1: MAIN/HIGH are VA's Profile0/Profile1; PROFESSIONAL has no VAProfile. */
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_AV1_PROFILE, V4L2_MPEG_VIDEO_AV1_PROFILE_MAIN)
        == std::optional<VAProfile>(VAProfileAV1Profile0));
    CHECK(profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_AV1_PROFILE, V4L2_MPEG_VIDEO_AV1_PROFILE_HIGH)
        == std::optional<VAProfile>(VAProfileAV1Profile1));
    CHECK(
        !profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_AV1_PROFILE, V4L2_MPEG_VIDEO_AV1_PROFILE_PROFESSIONAL)
             .has_value());

    /* A control this module knows nothing about maps nothing. */
    CHECK(!profile_menu::profile_for_menu_value(V4L2_CID_MPEG_VIDEO_HEVC_PROFILE, 0).has_value());
}

void test_device_lists_more_than_implemented()
{
    /* The interesting device: MediaCodec's AVC decoder can do more than this
     * bridge synthesises. The extra items must not leak out. */
    const profile_menu::DeviceMenu menu = std::set<uint32_t> {
        V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE,
        V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
        V4L2_MPEG_VIDEO_H264_PROFILE_MAIN,
        V4L2_MPEG_VIDEO_H264_PROFILE_HIGH,
        V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10,
        V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_HIGH,
    };
    const auto result = profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, kH264Implemented, menu);
    CHECK(result == kH264Implemented);
    /* BASELINE, HIGH_10 and CONSTRAINED_HIGH mapped to nothing at all, so even
     * before the intersection the device set is already those three. (The
     * bridge never claimed the deprecated VAProfileH264Baseline, which is why
     * plain BASELINE is dropped rather than folded into Constrained
     * Baseline.) */
    CHECK_EQ(profile_menu::device_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, *menu).size(), 3u);

    /* Same rule for the other two codecs: a device listing every VP9 profile,
     * or AV1 HIGH as well as MAIN, still advertises Profile 0 only. */
    const profile_menu::DeviceMenu vp9_all = std::set<uint32_t> { V4L2_MPEG_VIDEO_VP9_PROFILE_0,
        V4L2_MPEG_VIDEO_VP9_PROFILE_1, V4L2_MPEG_VIDEO_VP9_PROFILE_2, V4L2_MPEG_VIDEO_VP9_PROFILE_3 };
    const auto vp9 = profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, kVP9Implemented, vp9_all);
    CHECK(vp9 == kVP9Implemented);
    CHECK(!vp9.contains(VAProfileVP9Profile2));

    const profile_menu::DeviceMenu av1_both
        = std::set<uint32_t> { V4L2_MPEG_VIDEO_AV1_PROFILE_MAIN, V4L2_MPEG_VIDEO_AV1_PROFILE_HIGH };
    const auto av1 = profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_AV1_PROFILE, kAV1Implemented, av1_both);
    CHECK(av1 == kAV1Implemented);
    CHECK(!av1.contains(VAProfileAV1Profile1));
}

void test_device_lists_fewer_than_implemented()
{
    /* A codec that only does Constrained Baseline and Main: High goes away,
     * which is the whole point of VA1b -- a client that would have opened
     * High and got garbage now sees it unsupported and falls back cleanly. */
    const profile_menu::DeviceMenu menu
        = std::set<uint32_t> { V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE, V4L2_MPEG_VIDEO_H264_PROFILE_MAIN };
    const auto result = profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, kH264Implemented, menu);
    CHECK_EQ(result.size(), 2u);
    CHECK(result.contains(VAProfileH264ConstrainedBaseline));
    CHECK(result.contains(VAProfileH264Main));
    CHECK(!result.contains(VAProfileH264High));

    /* A VP9 decoder that starts at Profile 2 (a 10-bit-only device) says "not
     * Profile 0", and that is an empty advertisement -- NOT the fallback. */
    const profile_menu::DeviceMenu vp9_menu
        = std::set<uint32_t> { V4L2_MPEG_VIDEO_VP9_PROFILE_2, V4L2_MPEG_VIDEO_VP9_PROFILE_3 };
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, kVP9Implemented, vp9_menu).empty());
}

void test_device_lists_only_unimplemented_profiles()
{
    /* The case that separates "said nothing" from "said none of yours": every
     * listed item maps to no VAProfile we implement, so the mapped set is
     * empty -- but the device DID speak, and the answer is none. */
    const profile_menu::DeviceMenu menu
        = std::set<uint32_t> { V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_444_PREDICTIVE };
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, kH264Implemented, menu).empty());

    /* And the mapping underneath it drops those values rather than mapping
     * them to something adjacent. */
    CHECK(profile_menu::device_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, *menu).empty());
}

void test_device_lists_nothing_falls_back()
{
    /* The control exists but QUERYMENU accepted no index: nothing was learned,
     * so the implemented set stands. */
    const profile_menu::DeviceMenu empty_menu = std::set<uint32_t> {};
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, kH264Implemented, empty_menu)
        == kH264Implemented);
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, kVP9Implemented, empty_menu)
        == kVP9Implemented);
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_AV1_PROFILE, kAV1Implemented, empty_menu)
        == kAV1Implemented);
}

void test_device_without_the_control_falls_back()
{
    /* An older device: QUERY_EXT_CTRL answered ENOTTY/EINVAL, the probe
     * reported nullopt. This is exactly VA1's behaviour, which is what keeps
     * a driver built after VA1b working against a device deployed before it. */
    const profile_menu::DeviceMenu absent;
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, kH264Implemented, absent)
        == kH264Implemented);
    CHECK(
        profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_VP9_PROFILE, kVP9Implemented, absent) == kVP9Implemented);
    CHECK(
        profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_AV1_PROFILE, kAV1Implemented, absent) == kAV1Implemented);

    /* A context that implements nothing for a format advertises nothing, menu
     * or no menu -- the fallback can never invent a profile. */
    CHECK(profile_menu::supported_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, {}, absent).empty());
}

void test_mapped_device_set()
{
    /* device_profiles() is the mapping alone, before any intersection: the
     * implemented-set clamp above is a separate step, and this is what it
     * clamps. */
    const std::set<uint32_t> menu { V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE,
        V4L2_MPEG_VIDEO_H264_PROFILE_HIGH, V4L2_MPEG_VIDEO_H264_PROFILE_HIGH_10 };
    const auto mapped = profile_menu::device_profiles(V4L2_CID_MPEG_VIDEO_H264_PROFILE, menu);
    CHECK_EQ(mapped.size(), 2u);
    CHECK(mapped.contains(VAProfileH264ConstrainedBaseline));
    CHECK(mapped.contains(VAProfileH264High));
}

} // namespace

int main()
{
    test_control_for_format();
    test_menu_value_mapping();
    test_device_lists_more_than_implemented();
    test_device_lists_fewer_than_implemented();
    test_device_lists_only_unimplemented_profiles();
    test_device_lists_nothing_falls_back();
    test_device_without_the_control_falls_back();
    test_mapped_device_set();
    return check_result("test_profile_menu");
}
