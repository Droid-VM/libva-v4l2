# Copyright (C) 2019 Bootlin
#
# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the
# "Software"), to deal in the Software without restriction, including
# without limitation the rights to use, copy, modify, merge, publish,
# distribute, sub license, and/or sell copies of the Software, and to
# permit persons to whom the Software is furnished to do so, subject to
# the following conditions:
#
# The above copyright notice and this permission notice (including the
# next paragraph) shall be included in all copies or substantial portions
# of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
# OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
# MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT.
# IN NO EVENT SHALL PRECISION INSIGHT AND/OR ITS SUPPLIERS BE LIABLE FOR
# ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
# TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


# The stateful decode path (VPU_DESIGN.md 7.6 point 3) synthesises H.264
# SPS/PPS with gsth264bitwriter, exported by libgstcodecparsers since 1.22:
# the codecparsers dependency is required for it.
if not libgstcodecparsers_dep.found()
	error('gstreamer-codecparsers-1.0 is required: the stateful decode path synthesises H.264 parameter sets with gsth264bitwriter')
endif
if not libgstcodecparsers_dep.version().version_compare('>= 1.22')
	error('gstreamer-codecparsers-1.0 >= 1.22 is required for gsth264bitwriter')
endif

sources = [
	'driver.cc',
	'config.cc',
	'surface.cc',
	'context.cc',
	'buffer.cc',
	'picture.cc',
	'subpicture.cc',
	'image.cc',
	'utils.cc',
	'format.cc',
	'media.cc',
	'v4l2.cc',
	'mpeg2.cc',
	'h264.cc',
	'vp8.cc',
	'stateful/h264_bitstream.cc',
	'stateful/session.cc',
	'stateful/v4l2_device.cc',
	'stateful/h264_context.cc',
]

headers = [
	'driver.h',
	'config.h',
	'surface.h',
	'context.h',
	'buffer.h',
	'picture.h',
	'subpicture.h',
	'image.h',
	'utils.h',
	'format.h',
	'media.h',
	'v4l2.h',
	'mpeg2.h',
	'h264.h',
	'vp8.h',
	'stateful/device.h',
	'stateful/gsth264bitwriter_compat.h',
	'stateful/h264_bitstream.h',
	'stateful/session.h',
	'stateful/v4l2_device.h',
	'stateful/h264_context.h',
]
cpp_args = [
	'-Wall',
	'-fvisibility=hidden',
	'-DVA_DRIVER_INIT_FUNC=' + va_driver_init_func,
	'-D__STDC_WANT_LIB_EXT2__',
	'-DGST_USE_UNSTABLE_API',
	'-std=c++20',
]

if libgstcodecparsers_dep.found() and libgstcodecs_dep.found()
	sources += 'vp9.cc'
	headers += 'vp9.h'
	cpp_args += '-DENABLE_VP9'
endif

v4l2_drv_video = shared_module('v4l2_drv_video',
	name_prefix: '',
	install: true,
	install_dir: join_paths(get_option('libdir'), 'dri'),
	cpp_args: cpp_args,
	sources: [ sources, headers ],
	dependencies: [
		libva_dep,
		libdrm_dep,
		libgstcodecparsers_dep,
		libgstcodecs_dep,
		libudev_dep,
		kernel_dep,
	])

# Host-side unit tests for the stateful path (VPU_DESIGN.md 7.6, host
# verification): no device needed.
test_bitstream = executable('test_h264_bitstream',
	[ '../test/stateful/test_h264_bitstream.cc', 'stateful/h264_bitstream.cc' ],
	cpp_args: cpp_args,
	dependencies: [ libva_dep, libgstcodecparsers_dep, kernel_dep ])
test('h264-bitstream', test_bitstream)

test_session = executable('test_stateful_session',
	[ '../test/stateful/test_session.cc', 'stateful/session.cc' ],
	cpp_args: cpp_args,
	dependencies: [ kernel_dep ])
test('stateful-session', test_session)

thread_dep = dependency('threads')

test_session_threads = executable('test_stateful_session_threads',
	[ '../test/stateful/test_session_threads.cc', 'stateful/session.cc' ],
	cpp_args: cpp_args,
	dependencies: [ kernel_dep, thread_dep ])
test('stateful-session-threads', test_session_threads)

dl_dep = cc.find_library('dl', required: false)

test_probe = executable('test_driver_probe',
	[ '../test/stateful/test_driver_probe.cc' ],
	cpp_args: cpp_args,
	dependencies: [ libva_dep, dl_dep ])
test('driver-probe', test_probe, args: [ v4l2_drv_video ])
