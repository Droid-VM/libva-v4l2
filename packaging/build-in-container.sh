#!/bin/bash
# Configure, build and package the guest VA-API backend inside the cross container.
#
# Bind mounts (build-packages.sh sets them up):
#   /work/src   = this checkout                 /work/pkg = packaging/ (the recipe), READ-ONLY
#   /work/out   = where the .deb goes
#
# Nothing else is mounted, so a build cannot write anywhere except its own tree and the output
# directory it was given.
#
#   build-in-container.sh <package-version>
set -e
PKGVER=${1:?usage: build-in-container.sh <package-version>}
PKGDIR=${WORK_PKG:-/work/pkg}
OUT=${WORK_OUT:-/work/out}
SRC=${WORK_SRC:-/work/src}

cd "$SRC"

# Resolve every dependency against the arm64 packages only. Debian multiarch puts them in the
# compiler's normal search path, so this plus the cross file is the whole of the cross setup --
# no sysroot is involved.
export PKG_CONFIG_LIBDIR=/usr/lib/aarch64-linux-gnu/pkgconfig:/usr/share/pkgconfig

# The driver has to land where the guest's own libva looks, and that path is libva's own
# `driverdir` variable rather than a guess: asking the arm64 libva.pc for it means a libva that
# ever moves its search path moves the package with it. A guess that is merely plausible
# (.../dri) fails as "vaInitialize failed with error code -1 (unknown libva error)", which names
# nothing.
DRIVERDIR=$(pkg-config --variable=driverdir libva)
[ -n "$DRIVERDIR" ] || { echo "error: arm64 libva.pc has no driverdir" >&2; exit 1; }
case "$DRIVERDIR" in
    /usr/lib/aarch64-linux-gnu/*) ;;
    *) echo "error: driverdir '$DRIVERDIR' is not under the arm64 multiarch dir -- wrong libva.pc?" >&2; exit 1 ;;
esac
echo "==> libva driverdir: $DRIVERDIR"

# meson reconfigures itself when the options change, so the build dir is kept by default -- a
# failed packaging step should not cost a full rebuild. VA_CLEAN=1 forces a fresh one.
[ -z "${VA_CLEAN:-}" ] || rm -rf build-cross install-cross
cross=(--cross-file "$PKGDIR/aarch64-linux-gnu.meson")
if [ -d build-cross ]; then
    meson setup --reconfigure build-cross "${cross[@]}" --buildtype=release
else
    meson setup build-cross "${cross[@]}" --buildtype=release
fi

# VP9 is compiled in only when BOTH gstreamer-codecparsers-1.0 and the unadvertised
# libgstcodecs-1.0 are found (root meson.build). Both are in the -bad -dev package the image
# installs, so a miss here means the image lost a dependency and the package would silently ship
# a smaller driver than the one that was reviewed -- say so rather than find out in the guest.
grep -Eq "Library gstcodecs-1\\.0 found: +YES" build-cross/meson-logs/meson-log.txt ||
    echo "note: libgstcodecs-1.0 not found -- VP9 is NOT compiled in (Ubuntu does not ship gst/codecs/*, upstream README says the same)" >&2

ninja -C build-cross
rm -rf install-cross
DESTDIR="$PWD/install-cross" ninja -C build-cross install

# Sanity before packaging: the shipped .so must be aarch64, not x86. readelf from the cross
# toolchain rather than file(1), which the image does not carry.
so=$(find install-cross -name 'v4l2_drv_video.so' -type f | head -1)
[ -n "$so" ] || { echo "error: the build installed no v4l2_drv_video.so" >&2; exit 1; }
aarch64-linux-gnu-readelf -h "$so" | grep -q 'AArch64' || {
    echo "error: $so is not aarch64:" >&2; aarch64-linux-gnu-readelf -h "$so" | grep Machine >&2; exit 1; }
echo "arch check: $(aarch64-linux-gnu-readelf -h "$so" | awk -F: '/Machine/{print $2}' | xargs)"

# THE CHECK THAT A SHARED MODULE NEEDS. `shared_module()` links with undefined symbols allowed --
# that is what a dlopen'd plugin is -- so a dependency meson failed to find does NOT fail the
# build here. It fails in the guest, as "vaInitialize failed" with the real cause
# (`undefined symbol: gst_h264_bit_writer_sps`) only visible under LIBVA_MESSAGING_LEVEL=2.
# So resolve every non-weak undefined symbol against the libraries the .so actually records as
# NEEDED, and refuse to package one that nothing provides. Version suffixes are stripped: an
# undefined symbol reads `sym@GLIBCXX_3.4` where its definition reads `sym@@GLIBCXX_3.4`.
for l in $(aarch64-linux-gnu-objdump -p "$so" | awk '/NEEDED/{print $2}'); do
    for d in /usr/lib/aarch64-linux-gnu /lib/aarch64-linux-gnu; do
        [ -e "$d/$l" ] && aarch64-linux-gnu-nm -D --defined-only "$d/$l"
    done
done | awk '{print $3}' | sed 's/@.*//' | sort -u > /tmp/va-defined
aarch64-linux-gnu-nm -D --undefined-only "$so" | awk '$1=="U"{print $2}' | sed 's/@.*//' | sort -u > /tmp/va-undefined
unresolved=$(comm -23 /tmp/va-undefined /tmp/va-defined)
if [ -n "$unresolved" ]; then
    echo "error: the driver would dlopen with undefined symbols -- a dependency is missing from the image:" >&2
    # shellcheck disable=SC2086  # deliberate: one symbol per word, one line each
    printf '    %s\n' $unresolved >&2
    exit 1
fi
echo "symbol check: every undefined symbol is provided by a NEEDED library"

# Packaging is package.sh's job, from this one staged tree.
exec bash "$PKGDIR/package.sh" "$PKGVER" "$PWD/install-cross" "$OUT" "$DRIVERDIR"
