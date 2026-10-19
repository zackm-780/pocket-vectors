#!/bin/bash
# Assembles a PortMaster port zip from a binary, shipped libraries and runtime data.
#
#   port/package_port.sh --port-dir port --name flightsim --binary build/flightsim \
#       --libs-dir build/libs --runtime build/jsbsim:jsbsim --runtime assets:assets \
#       --licenses-dir port/licenses --out dist/flightsim-portmaster.zip
#
# Result layout (BUILD_AND_DEPLOY section 6), zip root = the ports folder:
#   <Script>.sh                      launch script (the one *.sh in --port-dir)
#   <name>/<name>.aarch64            binary
#   <name>/libs.aarch64/...          shipped shared libraries
#   <name>/<runtime dest>/...        runtime data (jsbsim/, assets/, ...)
#   <name>/licenses/...              licence files
#   <name>/port.json gameinfo.xml README.md screenshot.png
#
# Options:
#   --port-dir DIR      folder with the launch script, port.json, gameinfo.xml, README.md, screenshot.png
#   --script FILE       the launch script (default: the only *.sh in --port-dir)
#   --name NAME         port folder name; PortMaster rule: [a-z0-9][a-z0-9._]*
#   --binary FILE       the executable; stored as <name>.<arch>
#   --arch ARCH         default aarch64
#   --libs-dir DIR      repeatable: every lib file/symlink in DIR goes to libs.<arch>/ (use cp -a semantics)
#   --runtime SRC:DEST  repeatable: copy directory SRC to <name>/DEST
#   --licenses-dir DIR  repeatable: copy every file into <name>/licenses/
#   --extra SRC:DEST    repeatable: copy a single file to <name>/DEST
#   --strip CMD         strip the binary and libs with CMD (e.g. aarch64-linux-gnu-strip)
#   --out ZIP           output zip
#   --keep-stage        keep the staging directory (printed at the end)
set -euo pipefail

die() { echo "package_port: $*" >&2; exit 1; }

PORT_DIR=""; SCRIPT=""; NAME=""; BINARY=""; ARCH="aarch64"; OUT=""; STRIP_CMD=""; KEEP=0
LIBS_DIRS=(); RUNTIME=(); LICENSE_DIRS=(); EXTRA=()
while [ $# -gt 0 ]; do
  case "$1" in
    --port-dir) PORT_DIR="$2"; shift 2 ;;
    --script) SCRIPT="$2"; shift 2 ;;
    --name) NAME="$2"; shift 2 ;;
    --binary) BINARY="$2"; shift 2 ;;
    --arch) ARCH="$2"; shift 2 ;;
    --libs-dir) LIBS_DIRS+=("$2"); shift 2 ;;
    --runtime) RUNTIME+=("$2"); shift 2 ;;
    --licenses-dir) LICENSE_DIRS+=("$2"); shift 2 ;;
    --extra) EXTRA+=("$2"); shift 2 ;;
    --strip) STRIP_CMD="$2"; shift 2 ;;
    --out) OUT="$2"; shift 2 ;;
    --keep-stage) KEEP=1; shift ;;
    -h|--help) sed -n '2,32p' "$0"; exit 0 ;;
    *) die "unknown option $1" ;;
  esac
done

[ -n "$PORT_DIR" ] && [ -d "$PORT_DIR" ] || die "--port-dir missing or not a directory"
[ -n "$NAME" ] || die "--name required"
[[ "$NAME" =~ ^[a-z0-9][a-z0-9._]*$ ]] || die "port name '$NAME' violates the PortMaster rule [a-z0-9][a-z0-9._]*"
[ -n "$BINARY" ] && [ -f "$BINARY" ] || die "--binary missing or not a file: $BINARY"
[ -n "$OUT" ] || die "--out required"

shopt -s nullglob
if [ -z "$SCRIPT" ]; then
  scripts=("$PORT_DIR"/*.sh)
  [ ${#scripts[@]} -eq 1 ] || die "expected exactly one launch script (*.sh) in $PORT_DIR, found ${#scripts[@]}; pass --script"
  SCRIPT="${scripts[0]}"
fi
[ -f "$SCRIPT" ] || die "launch script $SCRIPT not found"
SCRIPT_NAME="$(basename "$SCRIPT")"
for f in port.json gameinfo.xml README.md; do
  [ -f "$PORT_DIR/$f" ] || die "$PORT_DIR/$f missing"
done

STAGE="$(mktemp -d "${TMPDIR:-/tmp}/portstage.XXXXXX")"
trap '[ "$KEEP" = 1 ] || rm -rf "$STAGE"' EXIT
DEST="$STAGE/$NAME"
mkdir -p "$DEST/licenses" "$DEST/libs.$ARCH"

install -m 0755 "$SCRIPT" "$STAGE/$SCRIPT_NAME"
install -m 0755 "$BINARY" "$DEST/$NAME.$ARCH"
for f in port.json gameinfo.xml README.md; do install -m 0644 "$PORT_DIR/$f" "$DEST/$f"; done
for img in screenshot.png screenshot.jpg cover.png cover.jpg; do
  [ -f "$PORT_DIR/$img" ] && install -m 0644 "$PORT_DIR/$img" "$DEST/$img"
done
if [ ! -f "$DEST/screenshot.png" ] && [ ! -f "$DEST/screenshot.jpg" ]; then
  echo "package_port: WARNING no screenshot.png in $PORT_DIR (PortMaster requires one for a catalogue submission)" >&2
fi

for d in "${LIBS_DIRS[@]+"${LIBS_DIRS[@]}"}"; do
  [ -d "$d" ] || die "libs dir $d not found"
  cp -a "$d"/. "$DEST/libs.$ARCH/"
done
for r in "${RUNTIME[@]+"${RUNTIME[@]}"}"; do
  src="${r%%:*}"; dst="${r#*:}"
  [ -d "$src" ] || die "runtime dir $src not found"
  mkdir -p "$DEST/$dst"
  cp -a "$src"/. "$DEST/$dst/"
done
for d in "${LICENSE_DIRS[@]+"${LICENSE_DIRS[@]}"}"; do
  [ -d "$d" ] || die "licenses dir $d not found"
  cp -a "$d"/. "$DEST/licenses/"
done
for e in "${EXTRA[@]+"${EXTRA[@]}"}"; do
  src="${e%%:*}"; dst="${e#*:}"
  [ -f "$src" ] || die "extra file $src not found"
  mkdir -p "$(dirname "$DEST/$dst")"
  cp -a "$src" "$DEST/$dst"
done
rmdir "$DEST/libs.$ARCH" 2>/dev/null || true

if [ -n "$STRIP_CMD" ]; then
  "$STRIP_CMD" --strip-unneeded "$DEST/$NAME.$ARCH" || true
  if [ -d "$DEST/libs.$ARCH" ]; then
    find "$DEST/libs.$ARCH" -type f -name '*.so*' -exec "$STRIP_CMD" --strip-unneeded {} + || true
  fi
fi

# Deterministic timestamps (zip cannot store dates before 1980).
EPOCH="${SOURCE_DATE_EPOCH:-1704067200}"
find "$STAGE" -exec touch -h -d "@$EPOCH" {} +

mkdir -p "$(dirname "$OUT")"
OUT_ABS="$(cd "$(dirname "$OUT")" && pwd)/$(basename "$OUT")"
rm -f "$OUT_ABS"
( cd "$STAGE" && find . -type f -o -type l | sed 's|^\./||' | LC_ALL=C sort | zip -X -q -y "$OUT_ABS" -@ )
echo "package_port: wrote $OUT ($(du -h "$OUT_ABS" | cut -f1)), $(unzip -Z1 "$OUT_ABS" | wc -l) entries"
[ "$KEEP" = 1 ] && echo "package_port: staging kept in $STAGE"
exit 0
