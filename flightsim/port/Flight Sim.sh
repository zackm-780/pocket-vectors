#!/bin/bash
# Flight Sim - PortMaster launch script (aarch64).
# Follows the official template: https://portmaster.games/packaging.html
#
# gptokeyb is used ONLY for its Select+Start exit hotkey: it is started without a mapping file (-c), so
# it injects no keys. The game reads the gamepad itself through SDL's GameController API and ignores
# keyboard events when FLIGHTSIM_PLATFORM=portmaster.

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
GAMEDIR=/$directory/ports/flightsim
CONFDIR="$GAMEDIR/conf"

mkdir -p "$CONFDIR"
cd "$GAMEDIR" || exit 1

# The log is overwritten on every launch.
> "$GAMEDIR/log.txt" && exec > >(tee "$GAMEDIR/log.txt") 2>&1

if [ ! -f "$GAMEDIR/flightsim.${DEVICE_ARCH}" ]; then
  pm_message "Flight Sim: flightsim.${DEVICE_ARCH} is missing from $GAMEDIR"
  sleep 5
  exit 1
fi
$ESUDO chmod +x "$GAMEDIR/flightsim.${DEVICE_ARCH}"

# Shipped libraries (libJSBSim.so) live in libs.aarch64.
export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"
export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"
# Tells the game to use the gamepad only and to ignore keyboard events.
export FLIGHTSIM_PLATFORM=portmaster

# Exit hotkey only (Select+Start). No -c mapping file: gptokeyb must not inject keys.
$GPTOKEYB "flightsim.${DEVICE_ARCH}" &

pm_platform_helper "$GAMEDIR/flightsim.${DEVICE_ARCH}"

./flightsim.${DEVICE_ARCH} --fullscreen --conf-dir "$CONFDIR"
echo "flightsim exited with status $?"

pm_finish
