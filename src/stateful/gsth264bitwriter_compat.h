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

/*
 * The H.264 bit writer (gst-plugins-bad >= 1.22) is exported by
 * libgstcodecparsers-1.0, but several distributions (Ubuntu 24.04 among them)
 * do not ship its unstable-API header gst/codecparsers/gsth264bitwriter.h.
 * Include the real header when present, otherwise declare the small subset of
 * the 1.22 API the stateful path links against (the symbols are exported by
 * the shared library either way).
 */

extern "C" {
#include <gst/codecparsers/gsth264parser.h>

#if defined(__has_include) && __has_include(<gst/codecparsers/gsth264bitwriter.h>)
#include <gst/codecparsers/gsth264bitwriter.h>
#else

typedef enum {
    GST_H264_BIT_WRITER_OK,
    GST_H264_BIT_WRITER_INVALID_DATA,
    GST_H264_BIT_WRITER_NO_MORE_SPACE,
    GST_H264_BIT_WRITER_ERROR
} GstH264BitWriterResult;

GstH264BitWriterResult gst_h264_bit_writer_sps(const GstH264SPS* sps, gboolean start_code, guint8* data, guint* size);

GstH264BitWriterResult gst_h264_bit_writer_pps(const GstH264PPS* pps, gboolean start_code, guint8* data, guint* size);

GstH264BitWriterResult gst_h264_bit_writer_convert_to_nal(guint nal_prefix_size, gboolean packetized,
    gboolean has_startcode, gboolean add_trailings, const guint8* raw_data, gsize raw_size, guint8* nal_data,
    guint* nal_size);

#endif
}
