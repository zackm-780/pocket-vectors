#!/usr/bin/env bash
# Build the complete flightsim data pack (terrain + nav + FAA approaches) with one command.
# Output is a folder to copy into flightsim/data/ on the SD card. Never commit it (git-ignored).
# See flightsim/tools/datapack/README.md and docs/DATA.md.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TOOLS="$(cd "$HERE/.." && pwd)"

OUT="$PWD/datapack-out"
JOBS="$(nproc 2>/dev/null || echo 2)"
PRESET="pdx_oth"
BBOX=""
MIN_LEVEL=7
MAX_LEVEL=10
WORLD_MAX_LEVEL=6
SKIP_WORLD=0
KEEP_WORLD_CACHE=0
VENV="$TOOLS/.datapack-venv"
CIFP_AIRPORTS=""
TERRAIN_CACHE="$TOOLS/terrain/cache"
WORLD_CACHE=""

usage() {
  cat <<USAGE
Usage: build_datapack.sh [options]
  --out DIR            output folder (default ./datapack-out); its contents go into flightsim/data/
  --jobs N             terrain worker processes (default: CPU count)
  --skip-world         skip the world coarse layer (levels 0-$WORLD_MAX_LEVEL, ~50 GB of downloads)
  --preset NAME        region preset (default pdx_oth; see build_terrain.py --list-presets)
  --bbox W,S,E,N       custom region box instead of a preset (e.g. -123.4,43.1,-123.0,43.4)
  --min-level N        region min level (default 7)
  --max-level N        region max level (default 10)
  --cifp-airports L    only convert procedures for these ICAOs (quick test), e.g. KOTH,KPDX
  --venv DIR           python venv to create/use (default tools/.datapack-venv)
  --terrain-cache DIR  terrain download cache (default tools/terrain/cache)
  --world-cache DIR    GLO-90 cache for the world layer (default <terrain-cache>-world, deleted afterwards)
  --keep-world-cache   keep the ~50 GB GLO-90 download cache after the world layer
USAGE
}

while [ $# -gt 0 ]; do
  case "$1" in
    --out) OUT="$2"; shift 2 ;;
    --jobs) JOBS="$2"; shift 2 ;;
    --skip-world) SKIP_WORLD=1; shift ;;
    --preset) PRESET="$2"; shift 2 ;;
    --bbox) BBOX="$2"; shift 2 ;;
    --min-level) MIN_LEVEL="$2"; shift 2 ;;
    --max-level) MAX_LEVEL="$2"; shift 2 ;;
    --cifp-airports) CIFP_AIRPORTS="$2"; shift 2 ;;
    --venv) VENV="$2"; shift 2 ;;
    --terrain-cache) TERRAIN_CACHE="$2"; shift 2 ;;
    --world-cache) WORLD_CACHE="$2"; shift 2 ;;
    --keep-world-cache) KEEP_WORLD_CACHE=1; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
  esac
done

mkdir -p "$OUT"
OUT="$(cd "$OUT" && pwd)"
START=$(date +%s)
step() { echo; echo "==== $* ($(( $(date +%s) - START )) s elapsed)"; }

# --- python environment -------------------------------------------------------------------------
step "python venv: $VENV"
if [ ! -x "$VENV/bin/python" ]; then
  python3 -m venv "$VENV"
fi
"$VENV/bin/pip" install -q --disable-pip-version-check \
  -r "$TOOLS/terrain/requirements.txt" -r "$TOOLS/nav/requirements.txt"
PY="$VENV/bin/python"

if [ -n "$BBOX" ]; then
  REGION=("--bbox=$BBOX")
  REGION_NAME="bbox $BBOX"
else
  REGION=(--preset "$PRESET")
  REGION_NAME="$PRESET"
fi

# --- terrain: region ----------------------------------------------------------------------------
step "terrain: $REGION_NAME levels $MIN_LEVEL-$MAX_LEVEL (heights + land class)"
"$PY" "$TOOLS/terrain/build_terrain.py" "${REGION[@]}" --min-level "$MIN_LEVEL" --max-level "$MAX_LEVEL" \
  --jobs "$JOBS" --cache-dir "$TERRAIN_CACHE" --out "$OUT" 2>&1 | tail -n 15

# --- terrain: world coarse layer ----------------------------------------------------------------
if [ "$SKIP_WORLD" -eq 0 ]; then
  # Separate cache so the ~50 GB of GLO-90 can be deleted afterwards (CI disks are small).
  WORLD_CACHE="${WORLD_CACHE:-${TERRAIN_CACHE}-world}"
  step "terrain: world levels 0-$WORLD_MAX_LEVEL (heights only; large download)"
  "$PY" "$TOOLS/terrain/build_terrain.py" --world --max-level "$WORLD_MAX_LEVEL" \
    --jobs "$JOBS" --cache-dir "$WORLD_CACHE" --out "$OUT" 2>&1 | tail -n 15
  if [ "$KEEP_WORLD_CACHE" -eq 0 ]; then rm -rf "$WORLD_CACHE"; fi
fi

step "terrain: validate"
"$PY" "$TOOLS/terrain/validate_pack.py" --pack "$OUT" --source-cache "$TERRAIN_CACHE" 2>&1 | tail -n 12

# --- nav ----------------------------------------------------------------------------------------
step "nav: OurAirports + WMM + FAA NASR + CIFP"
NAV_ARGS=(--out "$OUT/nav" --faa-nasr auto --cifp auto)
if [ -n "$CIFP_AIRPORTS" ]; then NAV_ARGS+=(--cifp-airports "$CIFP_AIRPORTS"); fi
"$PY" "$TOOLS/nav/build_navdb.py" "${NAV_ARGS[@]}" 2>&1 | tail -n 15
step "nav: validate"
"$PY" "$TOOLS/nav/validate_navdb.py" "$OUT/nav" --spot-checks --wmm-selftest 2>&1 | tail -n 8

# --- attribution + readme -----------------------------------------------------------------------
NASR_CYCLE="$(grep -h -i -E '^(faa_nasr|nasr)' "$OUT/nav/nav.ini" 2>/dev/null | head -n 1 || true)"
CIFP_CYCLE="$(grep -h -i -E '^faa_cifp' "$OUT/nav/nav.ini" 2>/dev/null | head -n 1 || true)"
BUILT="$(date -u +%Y-%m-%d)"

cat > "$OUT/ATTRIBUTION.txt" <<ATTR
flightsim data pack, built $BUILT
Nav data cycle: ${NASR_CYCLE:-n/a}; ${CIFP_CYCLE:-n/a}

This pack is a derived product of the open datasets below. It is for flight simulation only and
MUST NOT be used for real navigation. Procedures and data may be incomplete or out of date.

Elevation (terrain/h): Copernicus DEM GLO-30 and GLO-90.
  "Produced using Copernicus WorldDEM-30 / GLO-90 (c) DLR e.V. 2010-2014 and (c) Airbus Defence
  and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights
  reserved." Accessed from the AWS Open Data registry (copernicus-dem-30m, copernicus-dem-90m).
  Licence: Copernicus DEM licence, free of charge for any use with this attribution.

Land cover (terrain/c): ESA WorldCover 10 m 2021 v200.
  "(c) ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021)
  processed by ESA WorldCover consortium." Licence: CC BY 4.0
  https://creativecommons.org/licenses/by/4.0/  (modified: resampled to 256 x 256 class tiles)

Airports, runways, navaids (nav/): OurAirports, https://ourairports.com/data/ (David Megginson).
  Public domain (no attribution required; credited here as a courtesy). No accuracy guarantee.

ILS, fixes, airways (nav/): FAA National Airspace System Resources (NASR) 28-day subscription.
  US Government work, public domain.

Instrument procedures (nav/procedures/): FAA Coded Instrument Flight Procedures (CIFP, ARINC 424).
  US Government work, public domain. Converted by this project; not an FAA product and not
  suitable for navigation.

Magnetic variation (nav/magvar.bin): World Magnetic Model WMM2025, NOAA NCEI and the British
  Geological Survey. Public domain (US Government work).
  https://www.ncei.noaa.gov/products/world-magnetic-model
ATTR

cat > "$OUT/README.txt" <<'RDM'
flightsim data pack
===================
Copy the CONTENTS of this folder (terrain/, nav/, ATTRIBUTION.txt) into the data folder next to
the game on the SD card:

    <ports folder>/flightsim/data/terrain/...
    <ports folder>/flightsim/data/nav/...

Do not nest it (no flightsim/data/datapack/terrain). If you keep the pack elsewhere, set data_dir
in flightsim/conf/settings.ini instead.

Tip: copy with the card mounted on a PC (a file manager is fine; many small files take a few
minutes). Licences and credits: ATTRIBUTION.txt. For simulation use only, not for real navigation.
RDM

# --- summary ------------------------------------------------------------------------------------
step "summary"
echo "pack: $OUT"
du -sh "$OUT" "$OUT/terrain" "$OUT/nav" 2>/dev/null | sed 's/^/  size /'
echo "files total: $(find "$OUT" -type f | wc -l)"
echo "  terrain/h tiles: $(find "$OUT/terrain/h" -type f 2>/dev/null | wc -l)"
echo "  terrain/c tiles: $(find "$OUT/terrain/c" -type f 2>/dev/null | wc -l)"
echo "  nav/procedures:  $(find "$OUT/nav/procedures" -type f 2>/dev/null | wc -l)"
echo "directories: $(find "$OUT" -type d | wc -l)"
echo "max entries in one directory (FAT32 limit 65,534 per folder; warn above 16,000):"
MAXLINE="$(find "$OUT" -type d -print0 | while IFS= read -r -d '' d; do
  printf '%s %s\n' "$(find "$d" -mindepth 1 -maxdepth 1 | wc -l)" "${d#"$OUT"/}"
done | sort -rn | head -n 1)"
echo "  $MAXLINE"
MAXN="${MAXLINE%% *}"
if [ "$MAXN" -gt 16000 ]; then
  echo "  WARNING: a directory exceeds 16,000 entries (slow on FAT32/exFAT, hard limit 65,534 on FAT32)"
fi
echo "done in $(( $(date +%s) - START )) s"
