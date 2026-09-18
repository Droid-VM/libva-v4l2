#!/bin/bash
# Turn the staged `meson install` tree into a guest .deb.
#
#   package.sh <version> <stagedir> <outdir> <driverdir>
#
# Called by build-in-container.sh once the cross build is done, so the payload and the
# environment it ships are described in exactly one place. Three things go in:
#
#   <driverdir>/v4l2_drv_video.so    the backend libva dlopen()s
#   /etc/profile.d/droidvm-va.sh     the environment that makes libva pick it
#   .../defaults/pref/droidvm-vaapi.js  the Firefox default that makes it ASK for hardware
#
# WHY THE ENVIRONMENT IS PART OF THE PACKAGE (design VPU_DESIGN.md 7.6 point 1)
#
# libva's default lookup asks DRM for the kernel driver's name and loads <name>_drv_video.so. In
# a DroidVM guest that name is `virtio_gpu`, and -- B18 §2 corrects the guess this comment used
# to make -- /usr/lib/aarch64-linux-gnu/dri/virtio_gpu_drv_video.so DOES exist: the distro Mesa
# ships it as a symlink into libgallium. It is Gallium's virgl video driver, it has nothing to do
# with this guest's virtio_gpu, and libva picking it is exactly why LIBVA_DRIVER_NAME=v4l2 is
# REQUIRED rather than merely helpful -- without it vaInitialize goes to the wrong file and every
# VA client falls back to software while a perfectly good decoder sits on /dev/video*. The
# backend is named `v4l2` (upstream's name, kept so this tree stays offerable upstream), so
# LIBVA_DRIVER_NAME=v4l2 is what connects the two. It is shipped rather than documented because
# "export this or nothing works" is not a tuning knob.
#
# TWO whitelist overrides, because there are two plugins and each reads its OWN variable.
# GST_VA_ALL_DRIVERS=1 is for GStreamer 1.28's `va` plugin (vah264dec): B18 §4.4 measured it
# logging `Unsupported driver: DroidVM libva-v4l2 (stateful virtio-media)` (gstvadisplay.c:186)
# and registering 0 features with only the old name exported. GST_VAAPI_ALL_DRIVERS=1 is the same
# whitelist in the OLD gstreamer-vaapi elements (vaapidecode). Both are exported: a guest may
# carry either plugin, and a missing override reads as a missing package, not as a refusal.
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
# <name>_drv_video.so. Here that name is `virtio_gpu`, and a virtio_gpu_drv_video.so DOES exist
# in this guest -- the distro Mesa ships it as a symlink into libgallium -- but it is Gallium's
# virgl video driver, not ours, and it decodes nothing here. That is precisely why this line is
# required: left alone, libva opens the wrong driver and every VA-API client falls back to
# software with the decoder sitting unused on /dev/video*.
export LIBVA_DRIVER_NAME=v4l2
# GST_VA_ALL_DRIVERS: GStreamer 1.28's `va` plugin (vah264dec) keeps its OWN vendor whitelist and
# reads THIS variable. Without it the plugin logs `Unsupported driver: DroidVM libva-v4l2
# (stateful virtio-media)` and registers 0 features, so vah264dec does not exist at all.
export GST_VA_ALL_DRIVERS=1
# GST_VAAPI_ALL_DRIVERS: the same whitelist in the OLD gstreamer-vaapi elements (vaapidecode),
# which read the other name. Both are exported because a guest may carry either plugin.
export GST_VAAPI_ALL_DRIVERS=1
# LIBVA_V4L2_VIDEO_PATH=/dev/videoN pins one node if the probe picks the wrong one.
EOF
chmod 0644 "$root/etc/profile.d/droidvm-va.sh"

# ---------------------------------------------------------------------------
# THE FIREFOX DEFAULT (P-4 layer 1, E2E-vpu.md 11.1)
#
# A stock firefox-esr launched from the guest's desktop menu decoded every
# video in SOFTWARE while the hardware decoder sat idle, and nothing on screen
# said so -- about:support reported "Hardware Decoding: Supported" for H264,
# VP9, AV1 and HEVC the whole time, and playback was smooth because four ARM
# cores can carry 1080p. The phone's device ledger was the only witness: not
# one codec session for the entire playback.
#
# The reason is upstream policy, not a bug: Firefox only ATTEMPTS VA-API on a
# driver its own allowlist recognises, and a backend that is not Mesa/Intel/AMD
# is not on that list. `media.hardware-video-decoding.force-enabled` is the
# documented escape hatch, and it defaults to false. So the backend is never
# even asked.
#
# This is shipped rather than documented for the same reason
# LIBVA_DRIVER_NAME=v4l2 is: "set this or nothing works" is not a tuning knob.
# It is a DEFAULT pref, not a lock -- about:config still wins -- and it does
# NOT weaken the sandbox in any way; the sandbox is a separate problem that no
# pref should be used to solve.
#
# WHY THIS PATH. Mozilla's own .deb (packages.mozilla.org, the repo the guest
# uses -- not the Ubuntu snap) installs to /usr/lib/firefox-esr and already
# ships pref() defaults exactly here: firefox-esr 153.3.0esr~build1 arm64
# carries defaults/pref/channel-prefs.js and defaults/pref/package-prefs.js
# (the latter sets dom.ipc.forkserver.enable), verified by unpacking that .deb
# [V]. So the mechanism is the vendor's own, not a guess. The file name is
# ours, so no dpkg path collides and a firefox-esr upgrade leaves it alone.
#
# Both channel directories get a copy: the ESR deb is what the guest image
# uses, /usr/lib/firefox is the rapid-release deb from the same repo. Ubuntu's
# own `firefox` is a SNAP and cannot be reached this way at all -- a snap reads
# neither this file nor /etc/profile.d/droidvm-va.sh, so the backend is not
# usable from it and that is a packaging limitation, not a configuration one.
# ---------------------------------------------------------------------------
for ffdir in /usr/lib/firefox-esr /usr/lib/firefox; do
    install -d -m 0755 "$root$ffdir/defaults/pref"
    cat > "$root$ffdir/defaults/pref/droidvm-vaapi.js" <<'EOF'
// Installed by libva-v4l2 (DroidVM guest VA-API backend).
//
// Firefox only attempts VA-API on a driver its own allowlist recognises. This
// backend is not on that list, so without this line Firefox never calls
// vaInitialize: every video decodes in software while DroidVM's hardware
// decoder sits idle, and about:support still claims "Hardware Decoding:
// Supported" (it reports what the platform could do, not what was tried).
//
// A default, not a lock: about:config overrides it. It does not disable or
// weaken any sandbox.
pref("media.hardware-video-decoding.force-enabled", true);

// The VA-API decode module itself. Already true on Linux in current builds;
// stated so a build that ships it off does not silently turn the backend into
// dead weight.
pref("media.ffmpeg.vaapi.enabled", true);
EOF
    chmod 0644 "$root$ffdir/defaults/pref/droidvm-vaapi.js"
done

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
 Installs /etc/profile.d/droidvm-va.sh, which exports LIBVA_DRIVER_NAME=v4l2
 (libva would otherwise load the driver named after the DRM device,
 virtio_gpu, which in this guest is Mesa's virgl video driver and decodes
 nothing) plus GST_VA_ALL_DRIVERS=1 and GST_VAAPI_ALL_DRIVERS=1, the vendor
 whitelist overrides GStreamer's two VA-API plugins each read separately.
 .
 Also installs defaults/pref/droidvm-vaapi.js into the Mozilla firefox-esr and
 firefox .deb directories, setting
 media.hardware-video-decoding.force-enabled -- Firefox otherwise never
 attempts VA-API on a driver outside its own allowlist, and decodes in
 software without saying so. It is a default pref, not a lock, and it changes
 nothing about the browser's sandbox.
EOF

deb="${PKG}_${PKGVER}_arm64.deb"
mkdir -p "$OUT"
dpkg-deb --root-owner-group --build "$root" "$OUT/$deb" >/dev/null
echo "==> wrote $deb"
