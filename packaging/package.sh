#!/bin/bash
# Turn the staged `meson install` tree into a guest .deb.
#
#   package.sh <version> <stagedir> <outdir> <driverdir>
#
# Called by build-in-container.sh once the cross build is done, so the payload and the
# environment it ships are described in exactly one place. Two files go in:
#
#   <driverdir>/v4l2_drv_video.so    the backend libva dlopen()s
#   /etc/profile.d/droidvm-va.sh     the environment that makes libva pick it
#
# WHY THE ENVIRONMENT IS PART OF THE PACKAGE (design VPU_DESIGN.md 7.6 point 1)
#
# libva's default lookup asks DRM for the kernel driver's name and loads <name>_drv_video.so. In
# a DroidVM guest that name is `virtio_gpu`, there is no virtio_gpu_drv_video.so and there never
# will be one -- the guest Mesa is built -Dgallium-drivers=zink,llvmpipe with no VA state tracker
# at all -- so vaInitialize fails and every VA client falls back to software while a perfectly
# good decoder sits on /dev/video*. The backend is named `v4l2` (upstream's name, kept so this
# tree stays offerable upstream), so LIBVA_DRIVER_NAME=v4l2 is what connects the two. It is
# shipped rather than documented because "export this or nothing works" is not a tuning knob.
#
# GST_VAAPI_ALL_DRIVERS=1 is for the OLD gstreamer-vaapi elements, which carry a whitelist of
# driver vendor strings and refuse anything else (upstream README says the same). GStreamer
# 1.28's newer `va` plugin (vah264dec) has no such list, so this line is only about not having to
# explain a silent element-not-found to whoever reaches for vaapidecode.
set -euo pipefail

PKGVER=${1:?version}
STAGE=${2:?stagedir}
OUT=${3:?outdir}
DRIVERDIR=${4:?driverdir}

PKG=libva-v4l2
root=$(mktemp -d)
trap 'rm -rf "$root"' EXIT
# mktemp makes its directory 0700, and that directory IS the package's `./` entry -- dpkg would
# apply the mode to the guest's root filesystem on unpack. 0755, as / already is.
chmod 0755 "$root"

so=$(find "$STAGE" -name 'v4l2_drv_video.so' -type f | head -1)
[ -n "$so" ] || { echo "error: no v4l2_drv_video.so in $STAGE" >&2; exit 1; }

# Installed explicitly at libva's own driverdir rather than wherever meson's libdir landed: the
# package's contract is "the guest's libva can find it", and that is a property of the guest's
# libva, not of this build's prefix.
install -d -m 0755 "$root${DRIVERDIR}"
install -m 0644 "$so" "$root${DRIVERDIR}/v4l2_drv_video.so"
aarch64-linux-gnu-strip --strip-unneeded "$root${DRIVERDIR}/v4l2_drv_video.so"

install -d -m 0755 "$root/etc/profile.d"
cat > "$root/etc/profile.d/droidvm-va.sh" <<'EOF'
# Installed by libva-v4l2 (DroidVM guest VA-API backend).
#
# LIBVA_DRIVER_NAME: libva's default lookup asks DRM for the kernel driver's name and loads
# <name>_drv_video.so. Here that name is `virtio_gpu`, no virtio_gpu_drv_video.so exists (the
# guest Mesa is built without any VA state tracker), and vaInitialize fails -- so every VA-API
# client falls back to software with the decoder sitting unused on /dev/video*. Naming the
# driver explicitly is what connects libva to this package.
export LIBVA_DRIVER_NAME=v4l2
# GST_VAAPI_ALL_DRIVERS: the old gstreamer-vaapi elements keep a whitelist of driver vendor
# strings and refuse anything not on it. GStreamer 1.28's newer `va` plugin (vah264dec) does not,
# so this only matters to whoever reaches for vaapidecode.
export GST_VAAPI_ALL_DRIVERS=1
# LIBVA_V4L2_VIDEO_PATH=/dev/videoN pins one node if the probe picks the wrong one.
EOF
chmod 0644 "$root/etc/profile.d/droidvm-va.sh"

# ---------------------------------------------------------------------------
# Depends, derived from what the binary actually links against rather than typed out. Each
# DT_NEEDED soname is resolved to the arm64 package that owns it in THIS image -- the same
# Ubuntu release the guest runs -- so the list cannot drift from the build, and a new dependency
# added in src/ turns up here instead of as a dlopen failure in the guest. It is also the only
# way to get these names right on this release: the GStreamer codecparsers the stateful bitstream
# writer will pull in live in libgstreamer-plugins-EXTRA1.0-0 on Ubuntu 26.04, not in the
# -bad1.0-0 the design text guessed from earlier releases (the -dev package IS still
# libgstreamer-plugins-bad1.0-dev; only the runtime package was renamed).
#
# dpkg-shlibdeps is the usual tool for this and is deliberately not used: it wants a debian/
# directory and a package build tree that this upstream source tree does not have. It would also
# pin minimum versions from the BUILD host, which is the same Ubuntu release as the guest today
# but is not guaranteed to be; an unversioned dependency on the soname's package is the honest
# statement of what was verified.
# ---------------------------------------------------------------------------
# libva2 and libva-drm2 are added by hand on top of the derived list, and they are the one
# hand-written part: the .so does NOT link against libva -- libva dlopen()s IT and hands it a
# vtable -- so nothing in DT_NEEDED can express that this file is meaningless without the loader
# that opens it. The coupling is real and versioned: the entry point is named
# __vaDriverInit_<major>_<minor> from the libva this was built against (root meson.build), so a
# libva of a different major would not call it at all.
pkg_of() {  # pkg_of <path> -- the arm64 package owning it, arch qualifier dropped: on the guest
            # these are the native architecture and a qualified dependency is noise at best.
    dpkg -S "$(readlink -f "$1")" | cut -d: -f1
}
deps=$(
    {
        aarch64-linux-gnu-objdump -p "$root${DRIVERDIR}/v4l2_drv_video.so" |
            awk '/NEEDED/ {print $2}' |
            while read -r soname; do
                path=/usr/lib/aarch64-linux-gnu/$soname
                [ -e "$path" ] || { echo "error: $soname is not in the image" >&2; exit 1; }
                pkg_of "$path"
            done
        pkg_of /usr/lib/aarch64-linux-gnu/libva.so.2
        pkg_of /usr/lib/aarch64-linux-gnu/libva-drm.so.2
    } | sort -u | paste -sd, - | sed 's/,/, /g'
)
[ -n "$deps" ] || { echo "error: derived an empty Depends" >&2; exit 1; }
echo "==> Depends: $deps"

install -d -m 0755 "$root/DEBIAN"
cat > "$root/DEBIAN/control" <<EOF
Package: $PKG
Version: $PKGVER
Section: libs
Priority: optional
Architecture: arm64
Installed-Size: $(du -sk "$root" | cut -f1)
Maintainer: Droid-VM <noreply@github.com>
Depends: $deps
Homepage: https://github.com/Droid-VM/libva-v4l2
Description: VA-API backend for DroidVM's virtio-media V4L2 decoder
 A libva backend (v4l2_drv_video.so) that drives a V4L2 memory-to-memory
 decoder, so VA-API-only clients -- Chromium and Firefox, mpv --hwdec=vaapi,
 ffmpeg -hwaccel vaapi, GStreamer's va* elements -- reach DroidVM's hardware
 decoder. Clients that speak V4L2 M2M directly (ffmpeg h264_v4l2m2m,
 GStreamer v4l2videodec) never needed it and are unaffected.
 .
 Installs /etc/profile.d/droidvm-va.sh, which exports LIBVA_DRIVER_NAME=v4l2:
 libva would otherwise look for a driver named after the DRM device
 (virtio_gpu), which does not exist in this guest.
EOF

deb="${PKG}_${PKGVER}_arm64.deb"
mkdir -p "$OUT"
dpkg-deb --root-owner-group --build "$root" "$OUT/$deb" >/dev/null
echo "==> wrote $deb"
