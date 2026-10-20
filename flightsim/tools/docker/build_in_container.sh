#!/bin/bash
# Runs INSIDE the Debian 11 build environment (Docker image or chroot, see tools/build_device.sh).
# The repo's flightsim/ directory is the working directory's parent of this script's ../..
#
# Builds the aarch64 probe and (when WP-00 has landed) the game, packages both PortMaster zips into
# $FSIM_DIST and runs port/check_package.sh on them.
#
# Environment:
#   FSIM_BUILD       build directory, default build-device (relative to flightsim/)
#   FSIM_DIST        output directory, default dist
#   JSBSIM_SRC       use this JSBSim checkout instead of fetching v1.3.3 (offline builds)
#   REQUIRE_MAIN=1   fail if the game itself cannot be built (CI after WP-00)
#   JOBS             parallel jobs, default: nproc
#   SKIP_PROBE=1     do not build the probe
set -euo pipefail

cd "$(dirname "$0")/../.."
ROOT="$PWD"
BUILD="${FSIM_BUILD:-build-device}"
DIST="${FSIM_DIST:-dist}"
JOBS="${JOBS:-$(nproc)}"
TOOLCHAIN="$ROOT/cmake/toolchain-aarch64.cmake"
TRIPLE="${AARCH64_TRIPLE:-aarch64-linux-gnu}"
STRIP_CMD="${TRIPLE}-strip"
export HOME="${HOME:-/tmp}"
git config --global --add safe.directory '*' 2>/dev/null || true

mkdir -p "$BUILD" "$DIST"
echo "== environment"
"${TRIPLE}-g++" --version | sed -n 1p
cmake --version | sed -n 1p
ldd --version | sed -n 1p
JSBSIM_ARGS=()
[ -n "${JSBSIM_SRC:-}" ] && JSBSIM_ARGS=("-DPROBE_JSBSIM_SOURCE_DIR=$JSBSIM_SRC")
ZIPS=()

# ---------------------------------------------------------------------------------------------
# Probe
# ---------------------------------------------------------------------------------------------
if [ "${SKIP_PROBE:-0}" != 1 ]; then
  echo "== building the device probe"
  cmake -S tools/probe -B "$BUILD/probe" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DPROBE_WERROR=ON "${JSBSIM_ARGS[@]+"${JSBSIM_ARGS[@]}"}"
  cmake --build "$BUILD/probe" -j "$JOBS"

  PROBE_LIBS="$BUILD/probe-libs"
  rm -rf "$PROBE_LIBS"; mkdir -p "$PROBE_LIBS"
  # libJSBSim.so, libJSBSim.so.1, libJSBSim.so.1.3.3 (symlinks kept)
  find "$BUILD/probe" -name 'libJSBSim.so*' \( -type f -o -type l \) -exec cp -a {} "$PROBE_LIBS/" \;
  [ -n "$(find "$PROBE_LIBS" -name "libJSBSim.so*" -print -quit)" ] || { echo "libJSBSim.so not found after the probe build" >&2; exit 1; }

  PROBE_ARGS=(--port-dir tools/probe/port --script tools/probe/port/FlightSimProbe.sh --name flightsimprobe --binary "$BUILD/probe/flightsim_probe"
              --libs-dir "$PROBE_LIBS" --licenses-dir port/licenses --strip "$STRIP_CMD")
  if [ -d "$BUILD/probe/jsbsim" ]; then PROBE_ARGS+=(--runtime "$BUILD/probe/jsbsim:jsbsim"); fi
  port/package_port.sh "${PROBE_ARGS[@]}" --out "$DIST/flightsim-probe-portmaster.zip"
  ZIPS+=("$DIST/flightsim-probe-portmaster.zip")
fi

# ---------------------------------------------------------------------------------------------
# Main game (WP-00 provides CMakeLists.txt and src/). Contract with WP-00 (documented in
# tools/docker/README section "Main game contract"):
#   * target `flightsim`, output <build>/flightsim
#   * JSBSim built shared: libJSBSim.so* anywhere under <build>
#   * runtime layout assembled in <build>/jsbsim/ and <build>/assets/ (BUILD_AND_DEPLOY section 2)
# ---------------------------------------------------------------------------------------------
if [ -f CMakeLists.txt ] && [ -f src/main.cpp ]; then
  echo "== building flightsim (main game)"
  MAIN_ARGS=()
  [ -n "${JSBSIM_SRC:-}" ] && MAIN_ARGS=("-DFETCHCONTENT_SOURCE_DIR_JSBSIM=$JSBSIM_SRC")
  cmake -S . -B "$BUILD/main" -G Ninja -DCMAKE_BUILD_TYPE=Release \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN" -DFSIM_WERROR=ON "${MAIN_ARGS[@]+"${MAIN_ARGS[@]}"}"
  cmake --build "$BUILD/main" --target flightsim -j "$JOBS"
  MAIN_BIN="$(find "$BUILD/main" -maxdepth 2 -name flightsim -type f -print -quit)"
  [ -n "$MAIN_BIN" ] || { echo "flightsim binary not found in $BUILD/main" >&2; exit 1; }
  MAIN_LIBS="$BUILD/main-libs"
  rm -rf "$MAIN_LIBS"; mkdir -p "$MAIN_LIBS"
  find "$BUILD/main" -name 'libJSBSim.so*' \( -type f -o -type l \) -exec cp -a {} "$MAIN_LIBS/" \;
  [ -n "$(find "$MAIN_LIBS" -name "libJSBSim.so*" -print -quit)" ] || { echo "libJSBSim.so not found after the main build" >&2; exit 1; }
  MAIN_PKG=(--port-dir port --script port/FlightSim.sh --name flightsim --binary "$MAIN_BIN" --libs-dir "$MAIN_LIBS"
            --licenses-dir port/licenses --strip "$STRIP_CMD")
  MAIN_DIR="$(dirname "$MAIN_BIN")"
  [ -d "$MAIN_DIR/jsbsim" ] && MAIN_PKG+=(--runtime "$MAIN_DIR/jsbsim:jsbsim")
  if [ -d "$MAIN_DIR/assets" ]; then MAIN_PKG+=(--runtime "$MAIN_DIR/assets:assets");
  elif [ -d assets ]; then MAIN_PKG+=(--runtime "assets:assets"); fi
  port/package_port.sh "${MAIN_PKG[@]}" --out "$DIST/flightsim-portmaster.zip"
  ZIPS+=("$DIST/flightsim-portmaster.zip")
else
  echo "== main game not present yet (no CMakeLists.txt + src/main.cpp): only the probe is packaged"
  if [ "${REQUIRE_MAIN:-0}" = 1 ]; then
    echo "REQUIRE_MAIN=1 but the game cannot be built" >&2
    exit 1
  fi
fi

# ---------------------------------------------------------------------------------------------
# Checks
# ---------------------------------------------------------------------------------------------
echo "== checking packages"
port/check_package.sh "${ZIPS[@]}"

# Smoke test: run the aarch64 probe's CPU benchmark under qemu-user when it is available.
if [ "${SKIP_QEMU:-0}" != 1 ] && command -v qemu-aarch64 >/dev/null 2>&1 && [ "${SKIP_PROBE:-0}" != 1 ]; then
  echo "== qemu-user smoke test: probe --cpu-only --quick"
  SMOKE="$BUILD/smoke"
  rm -rf "$SMOKE"; mkdir -p "$SMOKE"
  unzip -q "$DIST/flightsim-probe-portmaster.zip" -d "$SMOKE"
  # Use the Debian cross sysroot for ld.so/libc, and the multiarch dir for libSDL2.
  QEMU_LD_PREFIX="/usr/${TRIPLE}" \
  LD_LIBRARY_PATH="/usr/lib/${TRIPLE}" \
  qemu-aarch64 -E "LD_LIBRARY_PATH=$SMOKE/flightsimprobe/libs.aarch64:/usr/lib/${TRIPLE}" \
      "$SMOKE/flightsimprobe/flightsimprobe.aarch64" --cpu-only --quick --out "$SMOKE" > "$SMOKE/stdout.txt" 2>&1 \
      || { tail -20 "$SMOKE/stdout.txt"; echo "qemu smoke test FAILED" >&2; exit 1; }
  grep -E "RESULT: c172p" "$SMOKE/stdout.txt" || { tail -20 "$SMOKE/stdout.txt"; echo "no CPU result in the smoke test" >&2; exit 1; }
  echo "smoke test ok (timings under emulation are meaningless)"
fi

# Smoke test: fly the packaged aarch64 game headless (no display needed) under qemu-user.
if [ "${SKIP_QEMU:-0}" != 1 ] && command -v qemu-aarch64 >/dev/null 2>&1 && [ -f "$DIST/flightsim-portmaster.zip" ]; then
  echo "== qemu-user smoke test: flightsim --headless (c172p 6 NM final, 30 s)"
  GSMOKE="$BUILD/smoke-game"
  rm -rf "$GSMOKE"; mkdir -p "$GSMOKE"
  unzip -q "$DIST/flightsim-portmaster.zip" -d "$GSMOKE"
  ( cd "$GSMOKE/flightsim" &&
    QEMU_LD_PREFIX="/usr/${TRIPLE}" \
    qemu-aarch64 -E "LD_LIBRARY_PATH=$PWD/libs.aarch64:/usr/lib/${TRIPLE}" \
        ./flightsim.aarch64 --headless --scenario assets/scenarios/c172_final_xtst.ini --duration 30 ) \
      > "$GSMOKE/stdout.txt" 2>&1 \
      || { tail -20 "$GSMOKE/stdout.txt"; echo "qemu game smoke test FAILED" >&2; exit 1; }
  grep -E "^c172p: simulated 30\.0 s .*\| ok$" "$GSMOKE/stdout.txt" \
      || { tail -20 "$GSMOKE/stdout.txt"; echo "game smoke test did not end in a valid, uncrashed state" >&2; exit 1; }
  echo "game smoke test ok"
fi
echo "== done: $(printf '%s ' "${ZIPS[@]}")"
