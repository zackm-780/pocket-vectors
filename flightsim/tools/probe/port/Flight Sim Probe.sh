#!/bin/bash
# Flight Sim Probe - launch script (aarch64).
#
# Works with PortMaster (official template: https://portmaster.games/packaging.html) and WITHOUT it
# (e.g. the Anbernic stock OS, which does not ship PortMaster): the port folder is then the
# "flightsimprobe" folder next to this script. Kept POSIX-sh compatible on purpose, because some
# launchers run scripts with sh instead of bash.
#
# Writes probe_report.txt into the port folder, and ALWAYS writes flightsimprobe_launch_log.txt NEXT
# TO THIS SCRIPT, so a launch that fails before the probe starts can still be diagnosed.
# gptokeyb (PortMaster only) provides only the Select+Start exit hotkey.

# Started by a plain sh? Re-run under bash when it exists (PortMaster's control.txt needs bash).
if [ -z "$BASH_VERSION" ] && command -v bash >/dev/null 2>&1; then exec bash "$0" "$@"; fi

PORTNAME=flightsimprobe
SCRIPT_DIR="$(cd "$(dirname "$0")" 2>/dev/null && pwd)"
LAUNCHLOG="${SCRIPT_DIR:-/tmp}/${PORTNAME}_launch_log.txt"
{
  echo "== Flight Sim Probe launch $(date 2>/dev/null)"
  echo "script: $0 (dir: $SCRIPT_DIR) shell: ${BASH_VERSION:-sh} user: $(id 2>/dev/null)"
  uname -a 2>&1
} > "$LAUNCHLOG" 2>&1

XDG_DATA_HOME=${XDG_DATA_HOME:-$HOME/.local/share}

controlfolder=""
for d in "/opt/system/Tools/PortMaster" "/opt/tools/PortMaster" "$XDG_DATA_HOME/PortMaster" \
         "/roms/ports/PortMaster" "$SCRIPT_DIR/PortMaster"; do
  if [ -f "$d/control.txt" ]; then controlfolder="$d"; break; fi
done

GAMEDIR=""
if [ -n "$controlfolder" ]; then
  . "$controlfolder/control.txt"
  [ -f "${controlfolder}/mod_${CFW_NAME}.txt" ] && . "${controlfolder}/mod_${CFW_NAME}.txt"
  get_controls
  GAMEDIR=/$directory/ports/flightsimprobe
fi
# No PortMaster, or its ports folder is elsewhere: use the folder that sits next to this script.
if [ -z "$GAMEDIR" ] || [ ! -d "$GAMEDIR" ]; then
  GAMEDIR="$SCRIPT_DIR/$PORTNAME"
fi

# PortMaster helpers become no-ops when PortMaster is not installed.
command -v pm_message >/dev/null 2>&1 || pm_message() { echo "$*"; }
command -v pm_platform_helper >/dev/null 2>&1 || pm_platform_helper() { :; }
command -v pm_finish >/dev/null 2>&1 || pm_finish() { :; }

DEVICE_ARCH="${DEVICE_ARCH:-aarch64}"
BIN="$GAMEDIR/flightsimprobe.${DEVICE_ARCH}"
CONFDIR="$GAMEDIR/conf"
{
  echo "controlfolder=${controlfolder:-<none: PortMaster not found>}"
  echo "CFW_NAME=${CFW_NAME:-} DEVICE_NAME=${DEVICE_NAME:-} directory=${directory:-}"
  echo "GAMEDIR=$GAMEDIR"
  ls -la "$GAMEDIR" 2>&1 | sed -n '1,30p'
  echo "--- loader / libraries ---"
  ls -la /lib/ld-linux-aarch64.so.1 2>&1
  (ldd --version 2>&1 || /lib/ld-linux-aarch64.so.1 --version 2>&1) | sed -n 1p
  for l in /usr/lib/libSDL2-2.0.so.0 /usr/lib/aarch64-linux-gnu/libSDL2-2.0.so.0 /usr/lib64/libSDL2-2.0.so.0; do
    [ -e "$l" ] && echo "found $l"
  done
} >> "$LAUNCHLOG" 2>&1

if ! cd "$GAMEDIR"; then
  echo "ERROR: cannot enter $GAMEDIR (is the 'flightsimprobe' folder next to the script?)" >> "$LAUNCHLOG"
  exit 1
fi
mkdir -p "$CONFDIR"

# From here on, all output (including the probe's own) goes to the launch log next to the script.
exec >> "$LAUNCHLOG" 2>&1

if [ ! -f "$BIN" ]; then
  pm_message "Flight Sim Probe: flightsimprobe.${DEVICE_ARCH} is missing from $GAMEDIR"
  sleep 5
  exit 1
fi
$ESUDO chmod +x "$BIN" 2>/dev/null

export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"
[ -n "$sdl_controllerconfig" ] && export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"
export PROBE_PLATFORM=portmaster
export FLIGHTSIM_PLATFORM=portmaster

# Hand PortMaster's view of the device to the probe so it lands in probe_report.txt
# (answers "PortMaster install path" and "ports folder" in DEVICE.md).
export PM_CONTROLFOLDER="${controlfolder:-none}"
export PM_DIRECTORY="$directory"
export PM_GAMEDIR="$GAMEDIR"
export PM_GPTOKEYB="$GPTOKEYB"
export CFW_NAME DEVICE_NAME DEVICE_ARCH DEVICE_CPU DEVICE_RAM DEVICE_HAS_ARMHF ANALOG_STICKS \
       DISPLAY_WIDTH DISPLAY_HEIGHT param_device ESUDO
{
  echo "controlfolder=${controlfolder:-<none>}"
  echo "directory=$directory"
  echo "CFW_NAME=$CFW_NAME DEVICE_NAME=$DEVICE_NAME DEVICE_ARCH=$DEVICE_ARCH DEVICE_RAM=$DEVICE_RAM"
  echo "ANALOG_STICKS=$ANALOG_STICKS DISPLAY=${DISPLAY_WIDTH}x${DISPLAY_HEIGHT} param_device=$param_device"
  echo "--- ls controlfolder ---"
  [ -n "$controlfolder" ] && ls "$controlfolder" 2>&1 | sed -n '1,60p'
} > "$GAMEDIR/portmaster_env.txt" 2>&1

[ -n "$GPTOKEYB" ] && $GPTOKEYB "flightsimprobe.${DEVICE_ARCH}" &

pm_platform_helper "$BIN"

echo "--- starting $BIN ---"
# PROBE_EXTRA_ARGS is for development (e.g. PROBE_EXTRA_ARGS="--quick --skip gpu").
if [ -x "$BIN" ]; then
  "$BIN" --fullscreen --out "$GAMEDIR" $PROBE_EXTRA_ARGS
else
  # SD cards formatted FAT/exFAT may not keep the execute bit: start through the dynamic loader.
  echo "(not executable here; starting via /lib/ld-linux-aarch64.so.1)"
  /lib/ld-linux-aarch64.so.1 "$BIN" --fullscreen --out "$GAMEDIR" $PROBE_EXTRA_ARGS
fi
echo "probe exited with status $?"

pm_finish
