#!/bin/bash
# Runs both launch scripts against fake device layouts and checks which port folder they start the
# binary from. The binary is a stand-in shell script, so this runs on any Linux host (CI included).
#
#   port/test_launch_scripts.sh        (from flightsim/)
#
# Layouts:
#   rg40xxh-stock  PortMaster's ports folder holds a leftover folder without the binary (what the
#                  first sideload's script created); the real port sits next to the script.
#   pm-only        The binary is only in PortMaster's ports folder (scripts and data kept apart).
#   no-portmaster  No PortMaster at all; the port sits next to the script.
set -u
HERE="$(cd "$(dirname "$0")/.." && pwd)"
WORK="$(mktemp -d "${TMPDIR:-/tmp}/launchtest.XXXXXX")"
trap 'rm -rf "$WORK"' EXIT
failures=0

# $1 script path, $2 port name, $3 layout. Prints the folder the fake binary ran in, or nothing.
run_layout() {
  local script="$1" port="$2" layout="$3"
  local root="$WORK/$layout-$port"
  local sd="$root/sdcard/Roms/PORTS" pmroot="$root/roms"
  mkdir -p "$sd"
  cp "$script" "$sd/"
  local launcher="$sd/${script##*/}"
  if [ "$layout" != no-portmaster ]; then
    # Found through the $SCRIPT_DIR/PortMaster candidate; points $directory at the fake /roms.
    mkdir -p "$sd/PortMaster" "$pmroot/ports"
    cat > "$sd/PortMaster/control.txt" <<EOF
directory="${pmroot#/}"
CFW_NAME=""
get_controls() { :; }
EOF
  fi
  local portdir="$sd/$port"
  case "$layout" in
    rg40xxh-stock) mkdir -p "$pmroot/ports/$port/conf"; echo stale > "$pmroot/ports/$port/log.txt" ;;
    pm-only) portdir="$pmroot/ports/$port" ;;
  esac
  mkdir -p "$portdir"
  printf '#!/bin/sh\necho "FAKEBIN ran in $(pwd)"\n' > "$portdir/$port.aarch64"
  chmod +x "$portdir/$port.aarch64"
  env -i PATH="$PATH" HOME="$root/home" XDG_DATA_HOME="$root/xdg" bash "$launcher" >/dev/null 2>&1
  sed -n 's/^FAKEBIN ran in //p' "$sd/${port}_launch_log.txt" 2>/dev/null
}

check() {
  local script="$1" port="$2" layout="$3" want="$4" got
  got="$(run_layout "$script" "$port" "$layout")"
  if [ "$got" = "$want" ]; then
    echo "ok   $port $layout -> ${got#"$WORK"/}"
  else
    echo "FAIL $port $layout: started in '${got:-<binary not started>}', expected '$want'"
    sed 's/^/     | /' "$WORK/$layout-$port/sdcard/Roms/PORTS/${port}_launch_log.txt" 2>/dev/null
    failures=$((failures + 1))
  fi
}

for pair in "port/FlightSim.sh flightsim" "tools/probe/port/FlightSimProbe.sh flightsimprobe"; do
  set -- $pair
  script="$HERE/$1" port="$2"
  check "$script" "$port" rg40xxh-stock "$WORK/rg40xxh-stock-$port/sdcard/Roms/PORTS/$port"
  check "$script" "$port" pm-only "$WORK/pm-only-$port/roms/ports/$port"
  check "$script" "$port" no-portmaster "$WORK/no-portmaster-$port/sdcard/Roms/PORTS/$port"
done

[ "$failures" -eq 0 ] && echo "launch scripts: all layouts pass" || echo "launch scripts: $failures failure(s)"
[ "$failures" -eq 0 ]
