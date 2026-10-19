#!/bin/bash
# Builds the RG40XXH (aarch64, glibc 2.31) PortMaster packages:
#   dist/flightsim-probe-portmaster.zip   the device probe port ("Flight Sim Probe")
#   dist/flightsim-portmaster.zip         the game port ("Flight Sim"), once WP-00 has landed
#
#   tools/build_device.sh                 build in Docker (default when a Docker daemon is reachable)
#   tools/build_device.sh --docker        force Docker
#   tools/build_device.sh --chroot DIR    no Docker: use a Debian 11 chroot in DIR (root needed)
#   tools/build_device.sh --setup-chroot DIR   create that chroot with debootstrap + install_toolchain.sh
#   tools/build_device.sh --native        already inside the build environment (CI container, chroot)
#
# Options:
#   --jsbsim-src DIR    use a local JSBSim v1.3.3 checkout instead of fetching it (offline builds)
#   --require-main      fail if the game (WP-00) cannot be built, instead of packaging the probe only
#   --skip-probe        do not build the probe
#   --mirror URL        Debian mirror for the Docker image / chroot (default http://deb.debian.org/debian,
#                       use http://archive.debian.org/debian if bullseye has moved there)
#
# Both modes run tools/docker/build_in_container.sh, which also runs port/check_package.sh
# (GLIBC <= 2.31, allowed dynamic dependencies, package layout).
set -euo pipefail

FLIGHTSIM_DIR="$(cd "$(dirname "$0")/.." && pwd)"
REPO_ROOT="$(cd "$FLIGHTSIM_DIR/.." && pwd)"
REL_FLIGHTSIM="${FLIGHTSIM_DIR#$REPO_ROOT/}"
MODE=auto
CHROOT=""
JSBSIM_SRC="${JSBSIM_SRC:-}"
MIRROR="${DEBIAN_MIRROR:-http://deb.debian.org/debian}"
IMAGE="flightsim-device-build"

die() { echo "build_device: $*" >&2; exit 1; }

while [ $# -gt 0 ]; do
  case "$1" in
    --docker) MODE=docker; shift ;;
    --native) MODE=native; shift ;;
    --chroot) MODE=chroot; CHROOT="$2"; shift 2 ;;
    --setup-chroot) MODE=setup-chroot; CHROOT="$2"; shift 2 ;;
    --jsbsim-src) JSBSIM_SRC="$(cd "$2" && pwd)"; shift 2 ;;
    --require-main) export REQUIRE_MAIN=1; shift ;;
    --skip-probe) export SKIP_PROBE=1; shift ;;
    --mirror) MIRROR="$2"; shift 2 ;;
    -h|--help) sed -n '2,22p' "$0"; exit 0 ;;
    *) die "unknown option $1" ;;
  esac
done

if [ "$MODE" = auto ]; then
  if command -v docker >/dev/null 2>&1 && docker info >/dev/null 2>&1; then MODE=docker; else
    die "no Docker daemon reachable. Use --chroot DIR (see --help), or run inside the build environment with --native"
  fi
fi

case "$MODE" in
  native)
    JSBSIM_SRC="$JSBSIM_SRC" exec "$FLIGHTSIM_DIR/tools/docker/build_in_container.sh"
    ;;

  docker)
    docker build --build-arg "DEBIAN_MIRROR=$MIRROR" -t "$IMAGE" -f "$FLIGHTSIM_DIR/tools/docker/Dockerfile" \
        "$FLIGHTSIM_DIR/tools/docker"
    EXTRA=()
    JSB_IN=""
    if [ -n "$JSBSIM_SRC" ]; then EXTRA+=(-v "$JSBSIM_SRC:/jsbsim-src:ro"); JSB_IN=/jsbsim-src; fi
    docker run --rm --user "$(id -u):$(id -g)" -e HOME=/tmp \
        -e "JSBSIM_SRC=$JSB_IN" -e REQUIRE_MAIN="${REQUIRE_MAIN:-0}" -e SKIP_PROBE="${SKIP_PROBE:-0}" \
        -v "$REPO_ROOT:/src" "${EXTRA[@]+"${EXTRA[@]}"}" -w "/src/$REL_FLIGHTSIM" "$IMAGE" \
        tools/docker/build_in_container.sh
    ;;

  setup-chroot)
    [ "$(id -u)" = 0 ] || die "--setup-chroot needs root"
    [ -n "$CHROOT" ] || die "--setup-chroot DIR"
    command -v debootstrap >/dev/null || die "install debootstrap and debian-archive-keyring"
    if [ ! -x "$CHROOT/bin/bash" ]; then
      debootstrap --variant=minbase --arch=amd64 bullseye "$CHROOT" "$MIRROR"
    fi
    mount -t proc proc "$CHROOT/proc" 2>/dev/null || true
    mount --bind /dev "$CHROOT/dev" 2>/dev/null || true
    cp /etc/resolv.conf "$CHROOT/etc/resolv.conf"
    cp "$FLIGHTSIM_DIR/tools/docker/install_toolchain.sh" "$CHROOT/root/install_toolchain.sh"
    chroot "$CHROOT" env DEBIAN_MIRROR="$MIRROR" /root/install_toolchain.sh
    umount "$CHROOT/dev" 2>/dev/null || true
    umount "$CHROOT/proc" 2>/dev/null || true
    echo "chroot ready: tools/build_device.sh --chroot $CHROOT"
    exit 0
    ;;

  chroot)
    [ "$(id -u)" = 0 ] || die "--chroot needs root"
    [ -x "$CHROOT/bin/bash" ] || die "$CHROOT is not a chroot (create it with --setup-chroot)"
    CHROOT="$(cd "$CHROOT" && pwd)"
    mkdir -p "$CHROOT/src"
    cleanup() {
      umount "$CHROOT/jsbsim-src" 2>/dev/null || true
      umount "$CHROOT/src" 2>/dev/null || true
      umount "$CHROOT/dev" 2>/dev/null || true
      umount "$CHROOT/proc" 2>/dev/null || true
    }
    trap cleanup EXIT
    mount -t proc proc "$CHROOT/proc"
    mount --bind /dev "$CHROOT/dev"
    mount --bind "$REPO_ROOT" "$CHROOT/src"
    cp /etc/resolv.conf "$CHROOT/etc/resolv.conf"
    JSB_IN=""
    if [ -n "$JSBSIM_SRC" ]; then
      mkdir -p "$CHROOT/jsbsim-src"
      mount --bind "$JSBSIM_SRC" "$CHROOT/jsbsim-src"
      JSB_IN=/jsbsim-src
    fi
    chroot "$CHROOT" env -i PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin HOME=/root \
        "JSBSIM_SRC=$JSB_IN" "REQUIRE_MAIN=${REQUIRE_MAIN:-0}" "SKIP_PROBE=${SKIP_PROBE:-0}" \
        "FSIM_BUILD=${FSIM_BUILD:-build-device}" "FSIM_DIST=${FSIM_DIST:-dist}" \
        "http_proxy=${http_proxy:-}" "https_proxy=${https_proxy:-}" "no_proxy=${no_proxy:-}" \
        /bin/bash -c "cd /src/$REL_FLIGHTSIM && tools/docker/build_in_container.sh"
    ;;
esac

echo "build_device: artifacts in $FLIGHTSIM_DIR/dist/"
