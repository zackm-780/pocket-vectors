#!/bin/bash
# Installs the aarch64 cross-build toolchain into a Debian 11 (bullseye) amd64 system.
#
# Used by tools/docker/Dockerfile and by `tools/build_device.sh --chroot` so that the Docker image and
# the no-Docker local chroot are the same environment. Run as root inside bullseye.
#
# Result: cross g++ 10.2 (aarch64-linux-gnu-g++), glibc 2.31 headers and libs for arm64,
# libsdl2-dev:arm64 (SDL2 2.0.14), cmake 3.18, ninja, git, zip, qemu-user (for smoke tests).
set -euo pipefail

# bullseye left regular support in 2024 and LTS in Aug 2026, so the mirror can move to
# archive.debian.org. Override with DEBIAN_MIRROR=http://archive.debian.org/debian if the default fails.
MIRROR="${DEBIAN_MIRROR:-http://deb.debian.org/debian}"

# bullseye-security is required: the debian:bullseye Docker image already carries security updates
# (e.g. libc6 2.31-13+deb11u14), and without the same repo the matching libc6-dev and libc6:arm64
# versions are not installable. Since bullseye LTS ended, deb.debian.org and security.debian.org
# still serve the bullseye-security index but its pool files are gone (404), so the default is the
# archive, which has both.
SECURITY_MIRROR="${DEBIAN_SECURITY_MIRROR:-http://archive.debian.org/debian-security}"
cat > /etc/apt/sources.list <<EOF
deb ${MIRROR} bullseye main
deb ${MIRROR} bullseye-updates main
deb ${SECURITY_MIRROR} bullseye-security main
EOF
# bullseye-updates does not exist on archive.debian.org; drop it there.
case "${MIRROR}" in *archive.debian.org*) sed -i '/bullseye-updates/d' /etc/apt/sources.list ;;
esac
echo 'Acquire::Check-Valid-Until "false";' > /etc/apt/apt.conf.d/99archive
echo 'APT::Install-Recommends "false";' > /etc/apt/apt.conf.d/98norecommends

export DEBIAN_FRONTEND=noninteractive
dpkg --add-architecture arm64
apt-get update

apt-get install -y \
    build-essential \
    crossbuild-essential-arm64 \
    binutils-aarch64-linux-gnu \
    cmake \
    ninja-build \
    pkg-config \
    git \
    ca-certificates \
    curl \
    zip \
    unzip \
    jq \
    libxml2-utils \
    file \
    qemu-user \
    libsdl2-dev:arm64

apt-get clean
rm -rf /var/lib/apt/lists/*

echo "---- toolchain ----"
aarch64-linux-gnu-g++ --version | head -1
cmake --version | head -1
ninja --version
dpkg -s libsdl2-dev:arm64 | grep -E '^(Package|Version)'
dpkg -s libc6:arm64 | grep -E '^Version'
