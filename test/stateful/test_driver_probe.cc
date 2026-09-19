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
 * VPU_DESIGN.md 7.6 host verification: confirm the driver loads and probes
 * cleanly with no /dev/videoN present (vainfo-style dlopen), i.e.
 * vaInitialize fails without a crash rather than segfaulting.
 *
 * VA1b (7.6 point 2) put more work in that probe -- V4L2M2MDevice's
 * constructor now opens the node a second time per coded format to read the
 * profile menus -- so this runs the init twice: once against a node that does
 * not exist (open fails), and once against a node that opens but is not a V4L2
 * device at all (/dev/null: QUERYCAP fails, and the profile probe must never
 * be reached, let alone crash or hang on it). Both must return the same clean
 * VA_STATUS_ERROR_OPERATION_FAILED.
 */

#include <cstdio>
#include <cstring>
#include <initializer_list>

extern "C" {
#include <dlfcn.h>

#include <va/va.h>
#include <va/va_backend.h>
}

#include "check.h"

namespace {

void fake_error(VADriverContextP, const char*) { }
void fake_info(VADriverContextP, const char*) { }

} // namespace

int main(int argc, char** argv)
{
    REQUIRE(argc >= 2);
    const char* module_path = argv[1];

    void* handle = dlopen(module_path, RTLD_NOW | RTLD_LOCAL);
    if (handle == nullptr) {
        fprintf(stderr, "FATAL: dlopen(%s) failed: %s\n", module_path, dlerror());
        return 1;
    }

    /* The init symbol name embeds the libva ABI version this was built for. */
    char symbol[64];
    snprintf(symbol, sizeof(symbol), "__vaDriverInit_%d_%d", VA_MAJOR_VERSION, VA_MINOR_VERSION);
    auto init = reinterpret_cast<VAStatus (*)(VADriverContextP)>(dlsym(handle, symbol));
    if (init == nullptr) {
        fprintf(stderr, "FATAL: dlsym(%s) failed: %s\n", symbol, dlerror());
        return 1;
    }

    /* A node that cannot be opened, then a node that opens and answers no V4L2
     * ioctl. Both make enumeration find nothing usable. */
    for (auto&& video_path : { "/dev/does-not-exist-videoN", "/dev/null" }) {
        setenv("LIBVA_V4L2_VIDEO_PATH", video_path, 1);

        VADriverContext context = {};
        VADriverVTable vtable = {};
        context.vtable = &vtable;
        context.version_major = VA_MAJOR_VERSION;
        context.version_minor = VA_MINOR_VERSION;
        context.error_callback = fake_error;
        context.info_callback = fake_info;

        /* The whole point: this returns (any status) without crashing. */
        VAStatus status = init(&context);
        printf("vaDriverInit with %s returned 0x%x\n", video_path, status);

        if (status == VA_STATUS_SUCCESS && vtable.vaTerminate != nullptr) {
            vtable.vaTerminate(&context);
        }

        CHECK(status == VA_STATUS_ERROR_OPERATION_FAILED);
    }

    dlclose(handle);
    return check_result("test_driver_probe");
}
