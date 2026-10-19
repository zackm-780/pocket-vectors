# CMake toolchain file: cross-compile for the Anbernic RG40XXH (aarch64 Linux, glibc).
#
# Intended environment: Debian 11 "bullseye" (glibc 2.31) with crossbuild-essential-arm64 and
# libsdl2-dev:arm64 (see tools/docker/). Using the oldest practical glibc keeps the binary loadable on the
# stock OS (decision D15). tools/build_device.sh sets everything up.
#
# Usage:
#   cmake -S . -B build-device -G Ninja -DCMAKE_BUILD_TYPE=Release \
#         -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-aarch64.cmake
#
# Optional cache/env variables:
#   AARCH64_TRIPLE   compiler prefix, default aarch64-linux-gnu  (env or -D)
#   AARCH64_SYSROOT  extra sysroot (only when the cross libs are NOT installed system-wide via multiarch)
#   AARCH64_CPU      value for -mcpu, default cortex-a53 (the H700); use "generic" to opt out

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

if(NOT AARCH64_TRIPLE)
  if(DEFINED ENV{AARCH64_TRIPLE})
    set(AARCH64_TRIPLE "$ENV{AARCH64_TRIPLE}")
  else()
    set(AARCH64_TRIPLE "aarch64-linux-gnu")
  endif()
endif()
if(NOT AARCH64_SYSROOT AND DEFINED ENV{AARCH64_SYSROOT})
  set(AARCH64_SYSROOT "$ENV{AARCH64_SYSROOT}")
endif()
if(NOT AARCH64_CPU)
  if(DEFINED ENV{AARCH64_CPU})
    set(AARCH64_CPU "$ENV{AARCH64_CPU}")
  else()
    set(AARCH64_CPU "cortex-a53")
  endif()
endif()

set(CMAKE_C_COMPILER   ${AARCH64_TRIPLE}-gcc)
set(CMAKE_CXX_COMPILER ${AARCH64_TRIPLE}-g++)
set(CMAKE_AR           ${AARCH64_TRIPLE}-ar CACHE FILEPATH "")
set(CMAKE_RANLIB       ${AARCH64_TRIPLE}-ranlib CACHE FILEPATH "")
set(CMAKE_STRIP        ${AARCH64_TRIPLE}-strip CACHE FILEPATH "")
set(CMAKE_OBJDUMP      ${AARCH64_TRIPLE}-objdump CACHE FILEPATH "")

if(AARCH64_SYSROOT)
  set(CMAKE_SYSROOT "${AARCH64_SYSROOT}")
  set(CMAKE_FIND_ROOT_PATH "${AARCH64_SYSROOT}")
  # Debian multiarch layout inside the sysroot.
  set(CMAKE_LIBRARY_ARCHITECTURE ${AARCH64_TRIPLE})
  list(APPEND CMAKE_PREFIX_PATH "${AARCH64_SYSROOT}/usr")
endif()

# Search target libs/headers/packages in the target tree, programs on the host.
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)

set(_fsim_cpu_flag "")
if(NOT AARCH64_CPU STREQUAL "generic")
  set(_fsim_cpu_flag "-mcpu=${AARCH64_CPU}")
endif()
set(CMAKE_C_FLAGS_INIT   "${_fsim_cpu_flag}")
set(CMAKE_CXX_FLAGS_INIT "${_fsim_cpu_flag}")

# Do not depend on the device's libstdc++/libgcc (decision D15). The GLIBC floor is checked by
# tools/check_device_binary.sh. --as-needed drops unused DT_NEEDED entries.
set(_fsim_ldflags "-static-libstdc++ -static-libgcc -Wl,--as-needed")
set(CMAKE_EXE_LINKER_FLAGS_INIT    "${_fsim_ldflags}")
set(CMAKE_SHARED_LINKER_FLAGS_INIT "${_fsim_ldflags}")
set(CMAKE_MODULE_LINKER_FLAGS_INIT "${_fsim_ldflags}")

# Binaries find the shipped libs next to them (<port>/libs.aarch64) even when started without the launch
# script. The launch script also exports LD_LIBRARY_PATH. Set as defaults so projects can override.
set(CMAKE_BUILD_WITH_INSTALL_RPATH ON CACHE BOOL "")
set(CMAKE_INSTALL_RPATH "\$ORIGIN/libs.aarch64" CACHE STRING "")

# Used by project CMake files to enable device-only behaviour.
set(FSIM_DEVICE_BUILD ON CACHE BOOL "Cross-compiling for the RG40XXH")
