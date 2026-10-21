#!/bin/bash
# Pocket Vectors (flightsim) - launch script (aarch64).
#
# Works with PortMaster (official template: https://portmaster.games/packaging.html) and without it.
# The port folder is the "flightsim" folder next to this script when it holds the binary, else
# PortMaster's ports folder (/$directory/ports/flightsim). Kept POSIX-sh compatible on purpose, because
# some launchers run scripts with sh instead of bash.
#
# Always writes flightsim_launch_log.txt NEXT TO THIS SCRIPT (and flightsim/log.txt), so a launch
# that fails before the game starts can still be diagnosed.
#
# gptokeyb (PortMaster only) is used ONLY for its Select+Start exit hotkey: it is started without a
# mapping file (-c), so it injects no keys. The game reads the gamepad itself through SDL's
# GameController API, ignores keyboard events when FLIGHTSIM_PLATFORM=portmaster, and also quits on
# Select+Start by itself (needed when gptokeyb is absent).

# Started by a plain sh? Re-run under bash when it exists (PortMaster's control.txt needs bash).
if [ -z "$BASH_VERSION" ] && command -v bash >/dev/null 2>&1; then exec bash "$0" "$@"; fi

PORTNAME=flightsim
# Shell builtins only (no dirname): this must work even on a minimal PATH.
case "$0" in */*) SCRIPT_DIR="${0%/*}" ;; *) SCRIPT_DIR=. ;; esac
SCRIPT_DIR="$(cd "$SCRIPT_DIR" 2>/dev/null && pwd)"
LAUNCHLOG="${SCRIPT_DIR:-/tmp}/${PORTNAME}_launch_log.txt"
{
  echo "== Pocket Vectors launch $(date 2>/dev/null)"
  echo "script: $0 (dir: $SCRIPT_DIR) shell: ${BASH_VERSION:-sh} user: $(id 2>/dev/null)"
  uname -a 2>&1
} > "$LAUNCHLOG" 2>&1

XDG_DATA_HOME=${XDG_DATA_HOME:-$HOME/.local/share}

controlfolder=""
for d in "/opt/system/Tools/PortMaster" "/opt/tools/PortMaster" "$XDG_DATA_HOME/PortMaster" \
         "/roms/ports/PortMaster" "$SCRIPT_DIR/PortMaster"; do
  if [ -f "$d/control.txt" ]; then controlfolder="$d"; break; fi
done

PM_PORTDIR=""
if [ -n "$controlfolder" ]; then
  . "$controlfolder/control.txt"
  [ -f "${controlfolder}/mod_${CFW_NAME}.txt" ] && . "${controlfolder}/mod_${CFW_NAME}.txt"
  get_controls
  PM_PORTDIR="/$directory/ports/flightsim"
fi
DEVICE_ARCH="${DEVICE_ARCH:-aarch64}"
# Use the folder that holds the binary, preferring the one next to this script. PortMaster's ports
# folder is not always where the launcher reads scripts from: on the RG40XXH stock OS PortMaster
# lives in /roms/ports (internal storage) while the Ports menu also runs scripts from the SD card
# (/mnt/sdcard/Roms/PORTS). A folder without the binary (e.g. one an older script created) is skipped.
GAMEDIR=""
for d in "$SCRIPT_DIR/$PORTNAME" "$PM_PORTDIR"; do
  if [ -n "$d" ] && [ -f "$d/$PORTNAME.${DEVICE_ARCH}" ]; then GAMEDIR="$d"; break; fi
done
# Neither holds it: report the folder next to this script, where the install note puts it.
[ -n "$GAMEDIR" ] || GAMEDIR="$SCRIPT_DIR/$PORTNAME"

# PortMaster helpers become no-ops when PortMaster is not installed.
command -v pm_message >/dev/null 2>&1 || pm_message() { echo "$*"; }
command -v pm_platform_helper >/dev/null 2>&1 || pm_platform_helper() { :; }
command -v pm_finish >/dev/null 2>&1 || pm_finish() { :; }

BIN="$GAMEDIR/flightsim.${DEVICE_ARCH}"
CONFDIR="$GAMEDIR/conf"
{
  echo "controlfolder=${controlfolder:-<none: PortMaster not found>}"
  echo "CFW_NAME=${CFW_NAME:-} DEVICE_NAME=${DEVICE_NAME:-} directory=${directory:-}"
  for d in "$SCRIPT_DIR/$PORTNAME" "$PM_PORTDIR"; do
    [ -n "$d" ] || continue
    if [ -f "$d/$PORTNAME.${DEVICE_ARCH}" ]; then echo "candidate $d: has $PORTNAME.${DEVICE_ARCH}"
    elif [ -d "$d" ]; then echo "candidate $d: exists, no $PORTNAME.${DEVICE_ARCH}"
    else echo "candidate $d: absent"; fi
  done
  echo "GAMEDIR=$GAMEDIR"
  ls -la "$GAMEDIR" 2>&1 | sed -n '1,30p'
  echo "--- loader / libraries ---"
  ls -la /lib/ld-linux-aarch64.so.1 2>&1
  (ldd --version 2>&1 || /lib/ld-linux-aarch64.so.1 --version 2>&1) | sed -n 1p
  for l in /usr/lib/libSDL2-2.0.so.0 /usr/lib/aarch64-linux-gnu/libSDL2-2.0.so.0 /usr/lib64/libSDL2-2.0.so.0; do
    [ -e "$l" ] && echo "found $l"
  done
  # The aarch64 build needs a 64-bit userspace; some Anbernic stock OS releases are 32-bit (armhf)
  # even though uname reports an aarch64 kernel. Byte 5 of an ELF file is 1 (32-bit) or 2 (64-bit).
  echo "userspace LONG_BIT: $(getconf LONG_BIT 2>&1)"
  for f in /bin/sh /bin/busybox /bin/bash; do
    [ -f "$f" ] || continue
    c="$(dd if="$f" bs=1 skip=4 count=1 2>/dev/null)"
    if [ "$c" = "$(printf '\002')" ]; then echo "$f: 64-bit ELF"; elif [ "$c" = "$(printf '\001')" ]; then echo "$f: 32-bit ELF"; fi
  done
  [ -e /lib/ld-linux-armhf.so.3 ] && echo "found /lib/ld-linux-armhf.so.3 (32-bit loader)"
  [ -e /lib/ld-linux-aarch64.so.1 ] || echo "ERROR: /lib/ld-linux-aarch64.so.1 is missing: no 64-bit ARM userspace (32-bit OS?), the aarch64 build cannot run"
} >> "$LAUNCHLOG" 2>&1

if ! cd "$GAMEDIR"; then
  echo "ERROR: cannot enter $GAMEDIR (is the 'flightsim' folder next to the script?)" >> "$LAUNCHLOG"
  exit 1
fi
mkdir -p "$CONFDIR"

# From here on, all output (including the game's own) goes to the launch log next to the script.
exec >> "$LAUNCHLOG" 2>&1

if [ ! -f "$BIN" ]; then
  pm_message "Pocket Vectors: flightsim.${DEVICE_ARCH} is missing from $GAMEDIR"
  sleep 5
  exit 1
fi
$ESUDO chmod +x "$BIN" 2>/dev/null

# Shipped libraries (libJSBSim.so.1 as a regular file) live in libs.aarch64.
export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"
[ -n "$sdl_controllerconfig" ] && export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"
# Tells the game to use the gamepad only and to ignore keyboard events.
export FLIGHTSIM_PLATFORM=portmaster

# Exit hotkey only (Select+Start). No -c mapping file: gptokeyb must not inject keys.
[ -n "$GPTOKEYB" ] && $GPTOKEYB "flightsim.${DEVICE_ARCH}" &

pm_platform_helper "$BIN"

echo "--- starting $BIN ---"
# --asset-dir: started through the loader, the game would take the loader's folder (/lib) for its
# own and miss jsbsim/, assets/ and data/.
if [ -x "$BIN" ]; then
  "$BIN" --fullscreen --asset-dir "$GAMEDIR" --conf-dir "$CONFDIR"
else
  # SD cards formatted FAT/exFAT may not keep the execute bit: start through the dynamic loader.
  echo "(not executable here; starting via /lib/ld-linux-aarch64.so.1)"
  /lib/ld-linux-aarch64.so.1 "$BIN" --fullscreen --asset-dir "$GAMEDIR" --conf-dir "$CONFDIR"
fi
echo "flightsim exited with status $?"

pm_finish
