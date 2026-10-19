# Device build image: Debian 11 (glibc 2.31) + aarch64 cross g++ + libsdl2-dev:arm64.
#
#   docker build -t flightsim-device-build -f flightsim/tools/docker/Dockerfile flightsim/tools/docker
#
# tools/build_device.sh builds and runs this image for you.
#
# If deb.debian.org no longer serves bullseye (it is past LTS), build with
#   --build-arg DEBIAN_MIRROR=http://archive.debian.org/debian
FROM debian:bullseye

ARG DEBIAN_MIRROR=http://deb.debian.org/debian
ENV DEBIAN_MIRROR=${DEBIAN_MIRROR}

COPY install_toolchain.sh /usr/local/sbin/install_toolchain.sh
RUN chmod +x /usr/local/sbin/install_toolchain.sh && /usr/local/sbin/install_toolchain.sh

# Build scripts are bind-mounted from the repo at run time (see tools/build_device.sh).
WORKDIR /src/flightsim
