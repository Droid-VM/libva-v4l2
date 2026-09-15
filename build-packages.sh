#!/bin/bash
# Package the DroidVM guest VA-API backend:
#
#   libva-v4l2_<ver>_arm64.deb          Debian / Ubuntu aarch64 guest
#
#   ./build-packages.sh [deb]           (default: deb)
#
# Output goes to $OUTDIR (default: this directory).
#
# Self-contained, like Droid-VM/droidvm-guest-additions' script of the same name: it needs
# nothing from the meta repo, so this checkout can be built on its own. Everything specific to
# cross-compiling lives in packaging/ (Dockerfile.va-cross, the meson cross file,
# build-in-container.sh, package.sh) and is modelled on Droid-VM/mesa-cross.
#
# WHY A CONTAINER AND NOT THE HOST. The .so is a GUEST binary: it has to be linked against the
# guest's libva, libdrm, GStreamer codecparsers and libudev, which are Ubuntu 26.04 arm64
# libraries. The container is an x86 toolchain emitting aarch64 against Debian multiarch :arm64
# -dev packages -- minutes, and nothing emulated (VPU_DESIGN.md 7.6 point 8).
#
#   VA_IMG=<tag>            image name (default droidvm-va-cross)
#   VA_NO_IMAGE_BUILD=1     use $VA_IMG as it is instead of (re)building it here
#   BASE=<image>            base image for the environment (default ubuntu:26.04)
#   VA_CLEAN=1              throw the kept build dir away first
#   OUTDIR=<dir>            where the .deb lands (default: this directory)
set -euo pipefail
cd "$(dirname "$0")"

WHAT=${1:-deb}
PKG=libva-v4l2
OUTDIR=${OUTDIR:-$PWD}
IMG=${VA_IMG:-droidvm-va-cross}

msg() { echo "==> $*"; }
die() { echo "error: $*" >&2; exit 1; }

[ "$WHAT" = deb ] || die "usage: $0 [deb]"
[ -f meson.build ] || die "run this from the libva-v4l2 checkout"

# ---------------------------------------------------------------------------
# Version.
#
# `git describe` is what the design text names, and it cannot be used: this fork carries
# upstream's full history and upstream has never tagged a release, so describe exits with "No
# names found, cannot describe anything". Upstream's own version (root meson.build) is the short
# commit hash for exactly that reason.
#
# A hash alone is not a package version: dpkg COMPARES versions, and a hash does not order --
# "g74b336a1" ranks below "gcf8f9fe4" whichever was built first, so an upgrade to a newer build
# would need --allow-downgrades and `apt upgrade` could quietly keep the older one. So the same
# scheme droidvm-guest-additions arrived at: a commit COUNT leads (it only goes up on a branch
# that is never rewritten) and the hash only identifies. A modified tree gets a dirty suffix,
# because otherwise an uncommitted change rebuilds to the same filename with different contents.
# ---------------------------------------------------------------------------
count=$(git rev-list --count HEAD 2>/dev/null || echo 0)
sha=$(git rev-parse --short=8 HEAD 2>/dev/null || echo unknown)
dirty=""
git diff --quiet HEAD -- 2>/dev/null || dirty="+dirty$(LC_ALL=C date -u '+%Y%m%d%H%M%S')"
VER="0+droidvm.r${count}.g${sha}${dirty}"
msg "version $VER"

# docker, as mesa-cross's build.sh uses (podman is not installed on the DroidVM build host).
command -v docker >/dev/null || die "docker is required"

if [ -n "${VA_NO_IMAGE_BUILD:-}" ]; then
    msg "using cross env image $IMG as is (VA_NO_IMAGE_BUILD)"
    docker image inspect "$IMG" >/dev/null 2>&1 || die "no image $IMG"
else
    msg "building cross env image ($IMG, base ${BASE:-ubuntu:26.04})"
    # --network=host: BuildKit runs RUN steps in its own network namespace, where DNS resolution
    # of archive.ubuntu.com / ports.ubuntu.com intermittently fails on some hosts even though a
    # plain `docker run` resolves them fine. The symptom is a wall of apt "Temporary failure
    # resolving", which reads like a mirror outage (mesa-cross hit this first).
    docker build --network=host -t "$IMG" --build-arg BASE="${BASE:-ubuntu:26.04}" \
        -f packaging/Dockerfile.va-cross packaging
fi

mkdir -p "$OUTDIR"
OUTDIR=$(cd "$OUTDIR" && pwd)

msg "cross-building $PKG for arm64"
# Three mounts, three roles: the source tree, the recipe (read-only), and the output directory.
# The .deb goes to its own mount so the output location is an argument rather than "wherever the
# recipe happens to live". git is deliberately not usable in there -- the version is passed in.
docker run --rm \
    -e VA_CLEAN="${VA_CLEAN:-}" \
    -v "$PWD:/work/src" \
    -v "$PWD/packaging:/work/pkg:ro" \
    -v "$OUTDIR:/work/out" \
    "$IMG" bash /work/pkg/build-in-container.sh "$VER"

deb="$OUTDIR/${PKG}_${VER}_arm64.deb"
[ -f "$deb" ] || die "the build produced no $deb"
msg "wrote $(basename "$deb")"
