/*
 * Copyright (C) 2007 Intel Corporation
 * Copyright (C) 2016 Florent Revest <florent.revest@free-electrons.com>
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

#pragma once

#include <cstdint>
#include <cstdlib>
#include <memory>

extern "C" {
#include <va/va.h>
#include <va/va_backend.h>
}

struct Buffer {
    /* data is calloc()/reallocarray()'d, so it must be free()'d, not delete'd:
     * a std::unique_ptr<uint8_t> would call operator delete on malloc'd memory
     * (UB; ASan flags alloc-dealloc-mismatch, benign only because glibc's
     * operator delete happens to call free). Carry the C free deleter. */
    struct FreeDeleter {
        void operator()(uint8_t* p) const { free(p); }
    };

    Buffer(VABufferType type, unsigned count, unsigned size, VASurfaceID derived_surface_id);
    /* A buffer over externally owned memory (a CAPTURE buffer mmap for a
     * derived image on the stateful path); nothing is allocated or freed. */
    Buffer(VABufferType type, unsigned count, unsigned size, VASurfaceID derived_surface_id, uint8_t* external_data);

    uint8_t* map() const { return external_data != nullptr ? external_data : data.get(); }

    VABufferType type;
    unsigned count;
    std::unique_ptr<uint8_t, FreeDeleter> data;
    uint8_t* external_data = nullptr;
    unsigned int size;
    VASurfaceID derived_surface_id;
    VABufferInfo info;
};

VAStatus createBuffer(VADriverContextP context, VAContextID context_id, VABufferType type, unsigned int size,
    unsigned int count, void* data, VABufferID* buffer_id);
VAStatus destroyBuffer(VADriverContextP context, VABufferID buffer_id);
VAStatus mapBuffer(VADriverContextP context, VABufferID buffer_id, void** data_map);
VAStatus unmapBuffer(VADriverContextP context, VABufferID buffer_id);
VAStatus bufferSetNumElements(VADriverContextP context, VABufferID buffer_id, unsigned int count);
VAStatus bufferInfo(
    VADriverContextP context, VABufferID buffer_id, VABufferType* type, unsigned int* size, unsigned int* count);
VAStatus acquireBufferHandle(VADriverContextP context, VABufferID buffer_id, VABufferInfo* buffer_info);
VAStatus releaseBufferHandle(VADriverContextP context, VABufferID buffer_id);
