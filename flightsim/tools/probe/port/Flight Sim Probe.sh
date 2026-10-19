#!/bin/bash
# Flight Sim Probe - PortMaster launch script (aarch64).
# Follows the official template: https://portmaster.games/packaging.html
#
# Writes probe_report.txt (and log.txt) into this port's folder. gptokeyb provides only the
# Select+Start exit hotkey.

XDG_DATA_HOME=${XDG_DATA_HOME:-$HOME/.local/share}

if [ -d "/opt/system/Tools/PortMaster/" ]; then
  controlfolder="/opt/system/Tools/PortMaster"
elif [ -d "/opt/tools/PortMaster/" ]; then
  controlfolder="/opt/tools/PortMaster"
elif [ -d "$XDG_DATA_HOME/PortMaster/" ]; then
  controlfolder="$XDG_DATA_HOME/PortMaster"
else
  controlfolder="/roms/ports/PortMaster"
fi

source $controlfolder/control.txt
[ -f "${controlfolder}/mod_${CFW_NAME}.txt" ] && source "${controlfolder}/mod_${CFW_NAME}.txt"
get_controls

DEVICE_ARCH="${DEVICE_ARCH:-aarch64}"
GAMEDIR=/$directory/ports/flightsimprobe
CONFDIR="$GAMEDIR/conf"

mkdir -p "$CONFDIR"
cd "$GAMEDIR" || exit 1

> "$GAMEDIR/log.txt" && exec > >(tee "$GAMEDIR/log.txt") 2>&1

if [ ! -f "$GAMEDIR/flightsimprobe.${DEVICE_ARCH}" ]; then
  pm_message "Flight Sim Probe: flightsimprobe.${DEVICE_ARCH} is missing from $GAMEDIR"
  sleep 5
  exit 1
fi
$ESUDO chmod +x "$GAMEDIR/flightsimprobe.${DEVICE_ARCH}"

export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"
export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"
export PROBE_PLATFORM=portmaster
export FLIGHTSIM_PLATFORM=portmaster

# Hand PortMaster's view of the device to the probe so it lands in probe_report.txt
# (answers "PortMaster install path" and "ports folder" in DEVICE.md).
export PM_CONTROLFOLDER="$controlfolder"
export PM_DIRECTORY="$directory"
export PM_GAMEDIR="$GAMEDIR"
export PM_GPTOKEYB="$GPTOKEYB"
export CFW_NAME DEVICE_NAME DEVICE_ARCH DEVICE_CPU DEVICE_RAM DEVICE_HAS_ARMHF ANALOG_STICKS \
       DISPLAY_WIDTH DISPLAY_HEIGHT param_device ESUDO
{
  echo "controlfolder=$controlfolder"
  echo "directory=$directory"
  echo "CFW_NAME=$CFW_NAME DEVICE_NAME=$DEVICE_NAME DEVICE_ARCH=$DEVICE_ARCH DEVICE_RAM=$DEVICE_RAM"
  echo "ANALOG_STICKS=$ANALOG_STICKS DISPLAY=${DISPLAY_WIDTH}x${DISPLAY_HEIGHT} param_device=$param_device"
  echo "--- ls controlfolder ---"
  ls "$controlfolder" 2>&1 | head -60
} > "$GAMEDIR/portmaster_env.txt" 2>&1

$GPTOKEYB "flightsimprobe.${DEVICE_ARCH}" &

pm_platform_helper "$GAMEDIR/flightsimprobe.${DEVICE_ARCH}"

# PROBE_EXTRA_ARGS is for development (e.g. PROBE_EXTRA_ARGS="--quick --skip gpu").
./flightsimprobe.${DEVICE_ARCH} --fullscreen --out "$GAMEDIR" $PROBE_EXTRA_ARGS
echo "probe exited with status $?"

pm_finish
