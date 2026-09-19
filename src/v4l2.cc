/*
 * Copyright (C) 2018 Paul Kocialkowski <paul.kocialkowski@bootlin.com>
 * Copyright (C) 2023 Max Schettler <max.schettler@posteo.de>
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

#include "v4l2.h"

#include <algorithm>
#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <stdexcept>
#include <system_error>

extern "C" {
#include <fcntl.h>
#include <linux/media.h>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/sysmacros.h>
#include <unistd.h>

#include <libudev.h>
}

#include "device_scan.h"
#include "utils.h"

namespace {

uint32_t query_capabilities(int video_fd)
{
    v4l2_capability capability = {};
    errno_wrapper(ioctl, video_fd, VIDIOC_QUERYCAP, &capability);

    if ((capability.capabilities & V4L2_CAP_DEVICE_CAPS) != 0) {
        return capability.device_caps;
    } else {
        return capability.capabilities;
    }
}

v4l2_format get_format(int video_fd, v4l2_buf_type type)
{
    v4l2_format result = { .type = type };
    errno_wrapper(ioctl, video_fd, VIDIOC_G_FMT, &result);
    return result;
}

std::vector<std::span<uint8_t>> map_buffer(int video_fd, v4l2_buf_type type, unsigned index)
{
    v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    v4l2_buffer buffer = {
        .index = index,
        .type = type,
        .m = { .planes = planes },
        .length = VIDEO_MAX_PLANES,
    };
    errno_wrapper(ioctl, video_fd, VIDIOC_QUERYBUF, &buffer);

    if (!V4L2_TYPE_IS_MULTIPLANAR(type)) { // reduce singleplanar API to single-plane in multiplanar buffer
        const auto offset = buffer.m.offset;
        buffer.m.planes = planes;
        buffer.m.planes[0].length = buffer.length;
        buffer.m.planes[0].m.mem_offset = offset;
        buffer.length = 1;
    }

    std::vector<std::span<uint8_t>> result(buffer.length);
    for (unsigned i = 0; i < buffer.length; i++) {
        result[i] = { static_cast<uint8_t*>(mmap(NULL, buffer.m.planes[i].length, PROT_READ | PROT_WRITE, MAP_SHARED,
                          video_fd, buffer.m.planes[i].m.mem_offset)),
            buffer.m.planes[i].length };
        if (result[i].data() == MAP_FAILED) {
            throw std::system_error(errno, std::generic_category());
        }
    }
    return result;
}

std::vector<std::string> enumerate_video_devices(udev* ctx, const std::string& media_device)
{
    int fd = errno_wrapper(open, media_device.c_str(), O_RDONLY);

    media_device_info device_info = {};
    errno_wrapper(ioctl, fd, MEDIA_IOC_DEVICE_INFO, &device_info);

    media_v2_topology topology = {};
    errno_wrapper(ioctl, fd, MEDIA_IOC_G_TOPOLOGY, &topology);

    std::vector<media_v2_entity> entities(topology.num_entities);
    std::vector<media_v2_interface> interfaces(topology.num_interfaces);
    topology.ptr_entities = reinterpret_cast<uint64_t>(entities.data());
    topology.ptr_interfaces = reinterpret_cast<uint64_t>(interfaces.data());

    errno_wrapper(ioctl, fd, MEDIA_IOC_G_TOPOLOGY, &topology);
    close(fd);

    if (std::ranges::find_if(entities, [](auto&& entity) { return entity.function == MEDIA_ENT_F_PROC_VIDEO_DECODER; })
        == entities.end()) {
        return {};
    }

    std::vector<std::string> result;
    for (auto&& interface : interfaces) {
        auto devnum = makedev(interface.devnode.major, interface.devnode.minor);
        std::unique_ptr<udev_device, decltype(&udev_device_unref)> device(
            udev_device_new_from_devnum(ctx, 'c', devnum), &udev_device_unref);
        if (device && interface.intf_type == MEDIA_INTF_T_V4L_VIDEO) {
            result.push_back(udev_device_get_property_value(device.get(), "DEVNAME"));
        }
    }

    return result;
}

std::vector<std::string> enumerate_media_devices(udev* ctx)
{
    std::unique_ptr<udev_enumerate, decltype(&udev_enumerate_unref)> enumerate(
        udev_enumerate_new(ctx), &udev_enumerate_unref);

    udev_enumerate_add_match_subsystem(enumerate.get(), "media");
    udev_enumerate_scan_devices(enumerate.get());

    std::vector<std::string> result;
    for (auto entry = udev_enumerate_get_list_entry(enumerate.get()); entry != nullptr;
         entry = udev_list_entry_get_next(entry)) {
        auto name = udev_list_entry_get_name(entry);

        std::unique_ptr<udev_device, decltype(&udev_device_unref)> device(
            udev_device_new_from_syspath(ctx, name), &udev_device_unref);

        if (device) {
            result.push_back(udev_device_get_property_value(device.get(), "DEVNAME"));
        }
    }

    return result;
}

bool is_stateful_decoder_node(int video_fd, v4l2_buf_type output_type)
{
    bool dynamic_resolution = false;
    bool slice_or_frame = false;

    for (v4l2_fmtdesc fmtdesc = { .type = static_cast<uint32_t>(output_type) };
         ioctl(video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0; fmtdesc.index += 1) {
        if (fmtdesc.flags & V4L2_FMT_FLAG_DYN_RESOLUTION) {
            dynamic_resolution = true;
        }
        switch (fmtdesc.pixelformat) {
        case V4L2_PIX_FMT_MPEG2_SLICE:
        case V4L2_PIX_FMT_H264_SLICE:
        case V4L2_PIX_FMT_HEVC_SLICE:
        case V4L2_PIX_FMT_VP8_FRAME:
        case V4L2_PIX_FMT_VP9_FRAME:
            slice_or_frame = true;
            break;
        default:
            break;
        }
    }

    return dynamic_resolution && !slice_or_frame;
}

/* The udev-free fallback probes /dev/video0 .. /dev/video63: a bounded sweep
 * of PATHS rather than a readdir, so it needs nothing but the right to open
 * the node itself -- which is exactly what Firefox's RDD broker grants, node
 * by node, for the M2M devices it found in the parent. 64 covers every DroidVM
 * guest (three nodes: camera, decoder, encoder) with room for a host that
 * renumbers them. */
constexpr unsigned kFallbackScanNodes = 64;

/* One node, opened read-only and non-blocking so a camera node cannot stall
 * the probe, and closed again. nullopt when the node is absent, refused, or
 * does not answer VIDIOC_QUERYCAP. */
std::optional<device_scan::NodeProbe> probe_video_node(const std::string& path)
{
    int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return std::nullopt;
    }

    std::optional<device_scan::NodeProbe> result;
    try {
        const uint32_t capabilities = query_capabilities(fd);
        const v4l2_buf_type output_type
            = (capabilities & V4L2_CAP_VIDEO_M2M) ? V4L2_BUF_TYPE_VIDEO_OUTPUT : V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
        result = device_scan::NodeProbe { capabilities, is_stateful_decoder_node(fd, output_type) };
    } catch (const std::exception&) {
        /* Not a usable node; keep probing the others. */
    }
    close(fd);
    return result;
}

/*
 * VA1b (VPU_DESIGN.md 7.6 point 2): read one coded format's profile menu.
 *
 * The menu is per coded format -- one control id, a different answer once the
 * OUTPUT queue is set to H.264 than to AV1 -- so the format has to be selected
 * before the control is queried, and that is a change of session state. Doing
 * it on the display-wide fd would S_FMT a queue that a context may already own
 * (P-6b: one V4L2 open is one codec session), so the probe opens the node once
 * more, sets the format with no buffers on it, sweeps the menu and closes.
 * Nothing it does can be seen by any other session.
 *
 * nullopt on every failure -- the node cannot be opened a second time, the
 * format is refused, the control does not exist (ENOTTY/EINVAL from an older
 * device that never learned to publish profiles), or it exists but is not a
 * menu. "No information" is not "no profiles": the caller keeps what the
 * bridge implements.
 */
profile_menu::DeviceMenu probe_profile_menu(
    const std::string& path, v4l2_buf_type output_type, fourcc pixelformat, uint32_t control_id)
{
    int fd = open(path.c_str(), O_RDWR | O_NONBLOCK | O_CLOEXEC);
    if (fd < 0) {
        return std::nullopt;
    }

    profile_menu::DeviceMenu result;
    do {
        /* Start from the driver's own OUTPUT format and change only the coded
         * format, so nothing here depends on guessing a width or a plane
         * layout the device would have to correct. */
        v4l2_format format = { .type = static_cast<uint32_t>(output_type) };
        if (ioctl(fd, VIDIOC_G_FMT, &format) < 0) {
            break;
        }
        format.fmt.pix_mp.pixelformat = pixelformat;
        format.fmt.pix_mp.plane_fmt[0].sizeimage = SOURCE_SIZE_MAX;
        if (ioctl(fd, VIDIOC_S_FMT, &format) < 0) {
            break;
        }
        if (format.fmt.pix_mp.pixelformat != pixelformat) {
            break; /* the device substituted another format: it is not set up for this one */
        }

        v4l2_query_ext_ctrl query = { .id = control_id };
        if (ioctl(fd, VIDIOC_QUERY_EXT_CTRL, &query) < 0) {
            break; /* ENOTTY/EINVAL: an older device, with no profile menu at all */
        }
        if (query.type != V4L2_CTRL_TYPE_MENU || (query.flags & V4L2_CTRL_FLAG_DISABLED)) {
            break; /* not a menu, or a menu the device says does not apply here */
        }

        /* QUERYMENU over the control's own range: an item the device supports
         * answers, the rest answer EINVAL. The cap only bounds a device that
         * reports a nonsense range -- a real profile menu is a handful of
         * items (the widest upstream one, H.264, has 18). */
        constexpr int64_t kMaxMenuItems = 64;
        std::set<uint32_t> values;
        const int64_t last = std::min<int64_t>(query.maximum, query.minimum + kMaxMenuItems - 1);
        for (int64_t value = std::max<int64_t>(query.minimum, 0); value <= last; value += 1) {
            v4l2_querymenu menu = { .id = control_id, .index = static_cast<uint32_t>(value) };
            if (ioctl(fd, VIDIOC_QUERYMENU, &menu) == 0) {
                values.insert(menu.index);
            }
        }
        result = std::move(values);
    } while (false);

    close(fd);
    return result;
}

} // namespace

profile_menu::DeviceMenu V4L2M2MDevice::profile_menu(fourcc pixelformat) const
{
    const auto it = profile_menus.find(pixelformat);
    return (it == profile_menus.end()) ? std::nullopt : it->second;
}

std::string V4L2M2MDevice::profile_menu_log_line() const
{
    if (profile_menus.empty()) {
        return {};
    }

    std::string reported;
    std::string silent;
    for (auto&& [pixelformat, menu] : profile_menus) {
        const char name[] = { static_cast<char>(pixelformat & 0xff), static_cast<char>((pixelformat >> 8) & 0xff),
            static_cast<char>((pixelformat >> 16) & 0xff), static_cast<char>((pixelformat >> 24) & 0xff), '\0' };
        if (menu && !menu->empty()) {
            reported += reported.empty() ? "" : ", ";
            reported += name;
            reported += " (" + std::to_string(menu->size()) + " item(s))";
        } else {
            silent += silent.empty() ? "" : ", ";
            silent += name;
        }
    }

    std::string line = video_path + ": profile menus reported for " + (reported.empty() ? "nothing" : reported);
    if (!silent.empty()) {
        /* The fallback, said once and plainly: these formats keep the profile
         * set this bridge implements because the device did not report one. */
        line += "; " + silent + " did not report profiles, keeping the implemented set";
    }
    return line;
}

bool V4L2M2MDevice::stateful_decoder() const
{
    return is_stateful_decoder_node(video_fd, output_buf_type);
}

std::vector<std::pair<std::string, std::optional<std::string>>> V4L2M2MDevice::enumerate_devices()
{
    std::vector<std::pair<std::string, std::optional<std::string>>> result;

    /* udev_new() fails in a process with no /sys and no /run/udev; the walks
     * below would then enumerate nothing anyway, so skip them rather than
     * hand libudev a null context. */
    std::unique_ptr<udev, decltype(&udev_unref)> ctx(udev_new(), &udev_unref);
    if (ctx) {
        for (auto&& media_device : enumerate_media_devices(ctx.get())) {
            for (auto&& video_device : enumerate_video_devices(ctx.get(), media_device)) {
                int fd = errno_wrapper(open, video_device.c_str(), O_RDONLY);
                if (query_capabilities(fd) & required_capabilities) {
                    result.emplace_back(video_device, media_device);
                }
                close(fd);
            }
        }

        /* Stateful decoders (VPU_DESIGN.md 7.6 point 1) have no media controller
         * node, so the media-driven walk above cannot find them: also probe the
         * video4linux subsystem directly and keep M2M nodes whose coded formats
         * mark them stateful. */
        std::unique_ptr<udev_enumerate, decltype(&udev_enumerate_unref)> video_enumerate(
            udev_enumerate_new(ctx.get()), &udev_enumerate_unref);
        udev_enumerate_add_match_subsystem(video_enumerate.get(), "video4linux");
        udev_enumerate_scan_devices(video_enumerate.get());

        std::vector<std::string> udev_nodes;
        for (auto entry = udev_enumerate_get_list_entry(video_enumerate.get()); entry != nullptr;
             entry = udev_list_entry_get_next(entry)) {
            std::unique_ptr<udev_device, decltype(&udev_device_unref)> device(
                udev_device_new_from_syspath(ctx.get(), udev_list_entry_get_name(entry)), &udev_device_unref);
            if (!device) {
                continue;
            }
            const char* devname = udev_device_get_property_value(device.get(), "DEVNAME");
            if (devname == nullptr) {
                continue;
            }
            const std::string video_device(devname);
            if (std::ranges::any_of(result, [&](auto&& r) { return r.first == video_device; })) {
                continue;
            }
            udev_nodes.push_back(video_device);
        }

        for (auto&& video_device :
            device_scan::select_decode_nodes(udev_nodes, required_capabilities, probe_video_node)) {
            result.emplace_back(video_device, std::nullopt);
        }
    }

    if (!result.empty()) {
        return result;
    }

    /* P-4 (E2E-vpu.md 11.1 layer 2): udev found nothing. In a normal session
     * that means there is nothing to find; in Firefox's RDD (media) process it
     * means the sandbox has no /sys and no /run/udev, while the decoder node
     * itself IS reachable -- that process's file broker enumerates /dev/video*
     * in the PARENT and grants the M2M ones by path
     * (SandboxBrokerPolicyFactory::AddV4l2Dependencies, used only by
     * GetRDDPolicy). So repeat the same capability + stateful-decoder test over
     * a bounded sweep of node paths. Selection and order are the udev walk's,
     * which is why both go through device_scan::select_decode_nodes. */
    for (auto&& video_device : device_scan::select_decode_nodes(
             device_scan::candidate_paths(kFallbackScanNodes), required_capabilities, probe_video_node)) {
        result.emplace_back(video_device, std::nullopt);
    }

    return result;
}

V4L2M2MDevice::Buffer::Buffer(V4L2M2MDevice& owner, v4l2_buf_type type, unsigned index)
    : owner_(owner)
    , type_(type)
    , index_(index)
    , mapping_(map_buffer(owner.video_fd, type, index))
{
}

V4L2M2MDevice::Buffer::Buffer(V4L2M2MDevice::Buffer&& other)
    : owner_(other.owner_)
    , type_(other.type_)
    , index_(other.index_)
    , mapping_(other.mapping_)
{
    other.mapping_.clear();
}

V4L2M2MDevice::Buffer& V4L2M2MDevice::Buffer::operator=(V4L2M2MDevice::Buffer&& other)
{
    this->~Buffer();
    new (this) V4L2M2MDevice::Buffer(std::move(other));
    return *this;
}

V4L2M2MDevice::Buffer::~Buffer()
{
    for (auto&& map : mapping_) {
        munmap(map.data(), map.size());
    }
}

void V4L2M2MDevice::Buffer::queue(int request_fd, timeval* timestamp, unsigned size) const
{
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    struct v4l2_buffer buffer = {
        .index = index_,
        .type = type_,
        .memory = V4L2_MEMORY_MMAP,
        .m = { .planes = planes },
        .length = static_cast<uint32_t>(mapping_.size()),
    };

    if (V4L2_TYPE_IS_MULTIPLANAR(type_)) {
        for (unsigned i = 0; i < mapping_.size(); i++) {
            buffer.m.planes[i].bytesused = size;
        }
    } else {
        buffer.bytesused = size;
    }

    if (request_fd >= 0) {
        buffer.flags = V4L2_BUF_FLAG_REQUEST_FD;
        buffer.request_fd = request_fd;
    }

    if (timestamp != NULL)
        buffer.timestamp = *timestamp;

    errno_wrapper(ioctl, owner_.video_fd, VIDIOC_QBUF, &buffer);
}

void V4L2M2MDevice::Buffer::dequeue() const
{
    struct v4l2_plane planes[VIDEO_MAX_PLANES] = {};
    struct v4l2_buffer buffer = {
        .index = index_,
        .type = type_,
        .memory = V4L2_MEMORY_MMAP,
        .m = { .planes = planes },
        .length = VIDEO_MAX_PLANES,
    };

    errno_wrapper(ioctl, owner_.video_fd, VIDIOC_DQBUF, &buffer);
    if (buffer.flags & V4L2_BUF_FLAG_ERROR) {
        throw std::runtime_error("Dequeued buffer marked erroneous by driver.");
    }
}

std::vector<int> V4L2M2MDevice::Buffer::export_(unsigned flags) const
{
    std::vector<int> result;
    for (unsigned i = 0; i < mapping_.size(); i++) {
        v4l2_exportbuffer exportbuffer = {
            .type = type_,
            .index = index_,
            .plane = i,
            .flags = flags,
        };

        errno_wrapper(ioctl, owner_.video_fd, VIDIOC_EXPBUF, &exportbuffer);
        result.push_back(exportbuffer.fd);
    }
    return result;
}

V4L2M2MDevice::V4L2M2MDevice(const std::string& video_path, const std::optional<std::string>& media_path)
    : video_path(video_path)
    , video_fd(errno_wrapper(open, video_path.c_str(), O_RDWR | O_NONBLOCK))
    , media_fd((media_path) ? errno_wrapper(open, media_path->c_str(), O_RDWR | O_NONBLOCK) : -1)
    , capabilities(query_capabilities(video_fd))
    , capture_buf_type(
          (capabilities & V4L2_CAP_VIDEO_M2M) ? V4L2_BUF_TYPE_VIDEO_CAPTURE : V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
    , output_buf_type(
          (capabilities & V4L2_CAP_VIDEO_M2M) ? V4L2_BUF_TYPE_VIDEO_OUTPUT : V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE)
    , capture_format(get_format(video_fd, capture_buf_type))
    , output_format(get_format(video_fd, output_buf_type))
{
    if (!(capabilities & required_capabilities)) {
        std::runtime_error("Missing device capabilities");
    }

    /* VA1b: read the device's profile menus once, here, for each coded format
     * this bridge implements AND the device supports. A format the device does
     * not list is not probed at all (no entry), which reads the same as "no
     * information" -- the contexts already refuse such a format on the fourcc
     * test alone. */
    for (auto&& pixelformat : { V4L2_PIX_FMT_H264, V4L2_PIX_FMT_VP9, V4L2_PIX_FMT_AV1 }) {
        const auto control_id = profile_menu::control_for_format(pixelformat);
        if (!control_id || !format_supported(output_buf_type, pixelformat)) {
            continue;
        }
        profile_menus.emplace(pixelformat, probe_profile_menu(video_path, output_buf_type, pixelformat, *control_id));
    }
}

V4L2M2MDevice::V4L2M2MDevice(V4L2M2MDevice&& other)
    : video_path(std::move(other.video_path))
    , video_fd(std::move(other.video_fd))
    , media_fd(std::move(other.media_fd))
    , capabilities(std::move(other.capabilities))
    , capture_buf_type(std::move(other.capture_buf_type))
    , output_buf_type(std::move(other.output_buf_type))
    , capture_format(std::move(other.capture_format))
    , output_format(std::move(other.output_format))
    , supported_output_formats(std::move(other.supported_output_formats))
    , supported_capture_formats(std::move(other.supported_capture_formats))
    , profile_menus(std::move(other.profile_menus))
    , capture_buffers(std::move(other.capture_buffers))
    , output_buffers(std::move(other.output_buffers))
{
    other.capture_buffers.clear();
    other.output_buffers.clear();
    other.video_fd = -1;
    other.media_fd = -1;
}

V4L2M2MDevice& V4L2M2MDevice::operator=(V4L2M2MDevice&& other)
{
    this->~V4L2M2MDevice();
    new (this) V4L2M2MDevice(std::move(other));
    return *this;
}

V4L2M2MDevice::~V4L2M2MDevice()
{
    if (video_fd >= 0) {
        close(video_fd);
    }
    if (media_fd >= 0) {
        close(media_fd);
    }
}

void V4L2M2MDevice::set_format(v4l2_buf_type type, unsigned int pixelformat, unsigned int width, unsigned int height)
{
    struct v4l2_format* format = V4L2_TYPE_IS_CAPTURE(type) ? &capture_format : &output_format;

    format->type = type;
    format->fmt.pix_mp.pixelformat = pixelformat;
    format->fmt.pix_mp.width = width;
    format->fmt.pix_mp.height = height;

    // Automatic size is insufficient for data buffers
    format->fmt.pix_mp.plane_fmt[0].sizeimage = V4L2_TYPE_IS_OUTPUT(type) ? SOURCE_SIZE_MAX : 0;

    errno_wrapper(ioctl, video_fd, VIDIOC_S_FMT, format);
}

unsigned V4L2M2MDevice::request_buffers(v4l2_buf_type type, unsigned count)
{
    struct v4l2_requestbuffers req_buffers = {
        .count = count,
        .type = type,
        .memory = V4L2_MEMORY_MMAP,
    };

    errno_wrapper(ioctl, video_fd, VIDIOC_REQBUFS, &req_buffers);
    buffer_capabilities = req_buffers.capabilities;

    auto& buffers = V4L2_TYPE_IS_CAPTURE(type) ? capture_buffers : output_buffers;

    buffers.clear();
    for (unsigned i = 0; i < req_buffers.count; i += 1) {
        buffers.emplace_back(*this, type, i);
    }

    return buffers.size(); // Actual amount may differ
}

bool V4L2M2MDevice::format_supported(v4l2_buf_type type, unsigned pixelformat) const
{
    for (v4l2_fmtdesc fmtdesc = { .type = type }; ioctl(video_fd, VIDIOC_ENUM_FMT, &fmtdesc) >= 0; fmtdesc.index += 1) {
        if (fmtdesc.pixelformat == pixelformat) {
            return true;
        }
    }
    return false;
}

const V4L2M2MDevice::Buffer& V4L2M2MDevice::buffer(v4l2_buf_type type, unsigned index)
{
    return (V4L2_TYPE_IS_CAPTURE(type) ? capture_buffers : output_buffers)[index];
}

int32_t V4L2M2MDevice::get_control(uint32_t id) const
{
    v4l2_control ctrl = {
        .id = id,
    };
    errno_wrapper(ioctl, video_fd, VIDIOC_G_CTRL, &ctrl);
    return ctrl.value;
}

void V4L2M2MDevice::set_ext_control(int request_fd, unsigned id, void* data, unsigned size)
{
    v4l2_ext_control control = {
        .id = id,
        .size = size,
        .ptr = data,
    };
    set_ext_controls(request_fd, std::span(&control, 1));
}

void V4L2M2MDevice::set_ext_controls(int request_fd, std::span<v4l2_ext_control> controls)
{
    struct v4l2_ext_controls meta = {
        .count = static_cast<uint32_t>(controls.size()),
        .controls = controls.data(),
    };

    if (request_fd >= 0) {
        meta.which = V4L2_CTRL_WHICH_REQUEST_VAL;
        meta.request_fd = request_fd;
    }

    errno_wrapper(ioctl, video_fd, VIDIOC_S_EXT_CTRLS, &meta);
}

void V4L2M2MDevice::set_streaming(bool enable)
{
    errno_wrapper(ioctl, video_fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &capture_buf_type);
    errno_wrapper(ioctl, video_fd, enable ? VIDIOC_STREAMON : VIDIOC_STREAMOFF, &output_buf_type);
}
