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
 * The AV1 bit writer (gst-plugins-bad >= 1.24) is exported by
 * libgstcodecparsers-1.0, but -- exactly like gsth264bitwriter -- several
 * distributions ship the shared library without its unstable-API header
 * gst/codecparsers/gstav1bitwriter.h (Ubuntu 24.04 host: only gstav1parser.h;
 * the Ubuntu 26.04 arm64 cross image DOES ship it and exports the symbols).
 * Include the real header when present, otherwise declare the small subset of
 * the API the stateful path links against (the symbols are exported by the
 * shared library either way -- verified with nm on the arm64 lib).
 */

extern "C" {
#include <gst/codecparsers/gstav1parser.h>

#if defined(__has_include) && __has_include(<gst/codecparsers/gstav1bitwriter.h>)
#include <gst/codecparsers/gstav1bitwriter.h>
#else

typedef enum {
    GST_AV1_BIT_WRITER_OK,
    GST_AV1_BIT_WRITER_INVALID_DATA,
    GST_AV1_BIT_WRITER_NO_MORE_SPACE,
    GST_AV1_BIT_WRITER_ERROR
} GstAV1BitWriterResult;

GstAV1BitWriterResult gst_av1_bit_writer_sequence_header_obu(
    const GstAV1SequenceHeaderOBU* seq_hdr, gboolean size_field, guint8* data, guint* size);

GstAV1BitWriterResult gst_av1_bit_writer_frame_header_obu(const GstAV1FrameHeaderOBU* frame_hdr,
    const GstAV1SequenceHeaderOBU* seq_hdr, guint8 temporal_id, guint8 spatial_id, gboolean size_field, guint8* data,
    guint* size);

GstAV1BitWriterResult gst_av1_bit_writer_temporal_delimiter_obu(gboolean size_field, guint8* data, guint* size);

#endif
}
