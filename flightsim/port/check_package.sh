#!/bin/bash
# Verifies a PortMaster port zip (or an unpacked folder) built for the RG40XXH:
#   * every ELF is aarch64
#   * the highest GLIBC_ symbol version referenced is <= MAX_GLIBC (default 2.31)   [objdump -T]
#   * dynamic dependencies (DT_NEEDED) are limited to libc, libm, libdl, libpthread, librt,
#     ld-linux-aarch64, libSDL2-2.0.so.0 and libraries shipped inside the package
#   * no dependency on libstdc++/libgcc_s (they must be linked statically)
#   * the launch script, port.json, gameinfo.xml, README.md, licenses/ are present and well formed
#   * SD-card / FAT safety: no symlinks in the zip or tree, no FAT-illegal names, no case collisions
#
#   port/check_package.sh dist/flightsim-portmaster.zip [more zips or folders...]
#
# Env: MAX_GLIBC (2.31), AARCH64_TRIPLE (aarch64-linux-gnu), CHECK_SCREENSHOT=1 to fail on a missing screenshot.
set -uo pipefail

MAX_GLIBC="${MAX_GLIBC:-2.31}"
TRIPLE="${AARCH64_TRIPLE:-aarch64-linux-gnu}"
ALLOWED_BASE="libc.so.6 libm.so.6 libdl.so.2 libpthread.so.0 librt.so.1 ld-linux-aarch64.so.1 libSDL2-2.0.so.0"

if command -v "${TRIPLE}-objdump" >/dev/null 2>&1; then
  OBJDUMP="${TRIPLE}-objdump"; READELF="${TRIPLE}-readelf"
elif objdump -i 2>/dev/null | grep -i aarch64 >/dev/null; then
  OBJDUMP=objdump; READELF=readelf
else
  echo "check_package: no aarch64-capable objdump/readelf (install binutils-aarch64-linux-gnu)" >&2
  exit 2
fi

FAIL=0
fail() { echo "  FAIL: $*"; FAIL=1; }
ok() { echo "  ok:   $*"; }
warn() { echo "  warn: $*"; }

# version compare: prints the larger of two dotted versions
ver_max() { printf '%s\n%s\n' "$1" "$2" | sort -V | tail -n1; }

check_elf() {
  local f="$1" rel="$2" machine glibc_max=0 v needed n
  machine="$($READELF -h "$f" 2>/dev/null | awk -F: '/Machine/ {gsub(/^ +/,"",$2); print $2}')"
  case "$machine" in
    *AArch64*) ;;
    *) fail "$rel: not an aarch64 ELF (machine: $machine)"; return ;;
  esac
  # Highest GLIBC_x.y among undefined (imported) dynamic symbols.
  for v in $($OBJDUMP -T "$f" 2>/dev/null | awk '/\*UND\*/' | grep -o 'GLIBC_[0-9][0-9.]*' | sed 's/GLIBC_//' | sort -uV); do
    glibc_max="$(ver_max "$glibc_max" "$v")"
  done
  if [ "$glibc_max" = "0" ]; then
    ok "$rel: no versioned GLIBC symbols"
  elif [ "$(ver_max "$glibc_max" "$MAX_GLIBC")" = "$MAX_GLIBC" ]; then
    ok "$rel: max GLIBC_$glibc_max (<= $MAX_GLIBC)"
  else
    fail "$rel: needs GLIBC_$glibc_max, newer than $MAX_GLIBC"
    $OBJDUMP -T "$f" | awk '/\*UND\*/' | grep "GLIBC_$glibc_max" | sed -n 1,5p | sed 's/^/        /'
  fi
  # Anything the C++ runtime was supposed to supply statically.
  if $OBJDUMP -T "$f" 2>/dev/null | awk '/\*UND\*/' | grep -E 'GLIBCXX_|CXXABI_|GCC_[0-9]' >/dev/null; then
    fail "$rel: imports GLIBCXX/CXXABI/GCC symbols (libstdc++/libgcc not linked statically)"
  fi
  needed="$($READELF -d "$f" 2>/dev/null | awk '/\(NEEDED\)/ {gsub(/[\[\]]/,"",$NF); print $NF}')"
  for n in $needed; do
    if [[ " $ALLOWED_BASE " == *" $n "* ]]; then continue; fi
    if [[ " $SHIPPED_SONAMES " == *" $n "* ]]; then continue; fi
    fail "$rel: dependency $n is not allowed (allowed: $ALLOWED_BASE + shipped libs)"
  done
  ok "$rel: NEEDED = $(echo $needed)"
}

# SD-card safety: FAT/exFAT cannot store symlinks; Windows/macOS unzip often drops them.
check_fat_safe() {
  local root="$1" rel base
  if [ -n "$(find "$root" -type l -print -quit 2>/dev/null)" ]; then
    fail "tree contains symlinks (not FAT/exFAT safe):"
    find "$root" -type l -printf '        %P -> %l\n' | sed -n '1,20p'
  else
    ok "no symlinks in tree"
  fi
  while IFS= read -r -d '' p; do
    rel="${p#"$root"/}"
    [ "$rel" = "$p" ] && continue
    base="${rel##*/}"
    case "$base" in ""|.|..) continue ;; esac
    if printf '%s' "$base" | grep -q $'[\t<>:"\\\\|?*\001-\037]'; then
      fail "FAT-illegal character in path: $rel"
    fi
    case "$base" in
      " "*|*" ") fail "FAT-unsafe leading/trailing space: $rel" ;;
    esac
    case "$base" in
      *.) fail "FAT-unsafe trailing dot: $rel" ;;
    esac
  done < <(find "$root" -print0)
  if ! find "$root" -mindepth 1 -printf '%P\n' | while IFS= read -r rel; do
      printf '%s\t%s\n' "$(printf '%s' "$rel" | tr '[:upper:]' '[:lower:]')" "$rel"
    done | LC_ALL=C sort | awk -F'\t' '
      NR>1 && $1==prev_key && $2!=prev_path {
        printf "        case collision: \"%s\" vs \"%s\"\n", prev_path, $2
        err=1
      }
      { prev_key=$1; prev_path=$2 }
      END { exit err+0 }'; then
    fail "case-insensitive path collision (FAT/exFAT)"
  else
    ok "no FAT case collisions"
  fi
}

# Zip central directory: fail if any entry is a stored symlink (zip -y).
check_zip_no_symlinks() {
  local zip="$1" hits
  if ! command -v zipinfo >/dev/null 2>&1; then
    warn "zipinfo not installed: skipping zip symlink attribute check"
    return
  fi
  # zipinfo long listing: symlink entries start with 'l' in the Unix permissions column.
  hits="$(zipinfo -l "$zip" 2>/dev/null | awk 'NR>1 && $1 ~ /^l/ { print }')"
  if [ -n "$hits" ]; then
    fail "zip contains symlink entries (zip -y / FAT-unsafe):"
    printf '%s\n' "$hits" | sed 's/^/        /' | sed -n '1,20p'
  else
    ok "zip stores no symlink entries"
  fi
}

check_tree() {
  local root="$1" label="$2" script port_dir portname f rel sonames
  echo "== $label"
  check_fat_safe "$root"
  # Layout: <root>/<Script>.sh and <root>/<portname>/
  script=""; for f in "$root"/*.sh; do [ -f "$f" ] && script="$f"; done
  [ -n "$script" ] || { fail "no launch script (*.sh) at the zip root"; return; }
  port_dir=""; for f in "$root"/*/; do port_dir="${f%/}"; done
  [ -n "$port_dir" ] || { fail "no port folder at the zip root"; return; }
  portname="$(basename "$port_dir")"
  ok "launch script '$(basename "$script")', port folder '$portname'"
  # Stock launchers may not quote paths: a space in the script name breaks the launch silently.
  case "$(basename "$script")" in *" "*) fail "launch script name contains a space" ;; esac
  [[ "$portname" =~ ^[a-z0-9][a-z0-9._]*$ ]] || fail "port folder name '$portname' violates PortMaster naming"
  bash -n "$script" && ok "launch script passes bash -n" || fail "launch script has a syntax error"
  # Some launchers (e.g. the Anbernic stock OS) run scripts with a plain POSIX sh.
  if command -v dash >/dev/null 2>&1; then
    dash -n "$script" && ok "launch script passes POSIX sh -n (dash)" || fail "launch script is not POSIX-sh parseable"
  fi
  grep -q 'command -v pm_finish' "$script" && grep -q '_launch_log.txt' "$script" \
    && ok "launch script runs without PortMaster and writes a launch log" \
    || fail "launch script lacks the no-PortMaster fallback / launch log"
  head -n1 "$script" | grep -q '^#!/bin/bash' || fail "launch script must start with #!/bin/bash"
  grep -q 'control.txt' "$script" && grep -q 'get_controls' "$script" && grep -q 'pm_finish' "$script" \
    && grep -q 'pm_platform_helper' "$script" && grep -q 'LD_LIBRARY_PATH' "$script" \
    && grep -q 'SDL_GAMECONTROLLERCONFIG' "$script" && grep -q 'GPTOKEYB' "$script" \
    && ok "launch script has the PortMaster template calls" || fail "launch script lacks template calls (control.txt/get_controls/pm_platform_helper/pm_finish/LD_LIBRARY_PATH/SDL_GAMECONTROLLERCONFIG/GPTOKEYB)"
  if grep -E '\$GPTOKEYB[^&]*-c ' "$script" >/dev/null; then fail "gptokeyb started with -c: it must be the exit hotkey only"; fi
  [ -x "$script" ] && ok "launch script is executable" || fail "launch script not executable"
  grep -q "GAMEDIR=/\$directory/ports/$portname" "$script" && ok "GAMEDIR matches port folder" || fail "GAMEDIR in the launch script does not point at ports/$portname"
  grep -q "\.\${DEVICE_ARCH}" "$script" && ok "binary referenced as \${DEVICE_ARCH}" || fail "launch script does not use \${DEVICE_ARCH}"

  [ -x "$port_dir/$portname.aarch64" ] && ok "binary $portname.aarch64 present and executable" || fail "binary $portname.aarch64 missing or not executable"
  for f in port.json gameinfo.xml README.md; do [ -s "$port_dir/$f" ] && ok "$f present" || fail "$f missing"; done
  if [ -f "$port_dir/screenshot.png" ] || [ -f "$port_dir/screenshot.jpg" ]; then ok "screenshot present"; \
    elif [ "${CHECK_SCREENSHOT:-0}" = 1 ]; then fail "screenshot.png missing"; else warn "screenshot.png missing (required for a PortMaster catalogue submission)"; fi
  if command -v jq >/dev/null 2>&1; then
    jq -e --arg n "$portname.zip" --arg s "$(basename "$script")" --arg d "$portname" \
       '.version==4 and .name==$n and (.items|index($s)!=null) and (.items|index($d)!=null) and (.attr.arch|index("aarch64")!=null)' \
       "$port_dir/port.json" >/dev/null && ok "port.json valid, name/items match" || fail "port.json invalid or name/items do not match ($portname.zip, $(basename "$script"), $portname)"
  else warn "jq not installed: port.json not validated"; fi
  if command -v xmllint >/dev/null 2>&1; then
    xmllint --noout "$port_dir/gameinfo.xml" && ok "gameinfo.xml well-formed" || fail "gameinfo.xml malformed"
  else warn "xmllint not installed: gameinfo.xml not validated"; fi
  grep -q "<path>./$(basename "$script")</path>" "$port_dir/gameinfo.xml" && ok "gameinfo.xml path matches launch script" || fail "gameinfo.xml <path> does not match the launch script"

  # Licences.
  if ls "$port_dir"/licenses/*LGPL* >/dev/null 2>&1; then ok "JSBSim LGPL text present in licenses/"; else fail "licenses/ lacks the LGPL text (JSBSim)"; fi
  [ -f "$port_dir/licenses/SDL2-zlib.txt" ] && ok "SDL2 zlib licence present" || fail "licenses/SDL2-zlib.txt missing"
  [ -f "$port_dir/licenses/gptokeyb-NOTE.txt" ] && ok "gptokeyb licence note present" || fail "licenses/gptokeyb-NOTE.txt missing"

  # ELF checks. Collect shipped sonames first.
  SHIPPED_SONAMES=""
  list="$(mktemp "${TMPDIR:-/tmp}/portlist.XXXXXX")"
  find "$root" -type f -name '*.so*' > "$list"
  while IFS= read -r f; do
    sonames="$($READELF -d "$f" 2>/dev/null | awk '/SONAME/ {gsub(/[\[\]]/,"",$NF); print $NF}')"
    SHIPPED_SONAMES="$SHIPPED_SONAMES $sonames $(basename "$f")"
  done < "$list"
  find "$root" -type f \( -name '*.so*' -o -name "*.aarch64" -o -perm -u+x \) | LC_ALL=C sort > "$list"
  nelf=0
  while IFS= read -r f; do
    rel="${f#$root/}"
    if [ "$(head -c4 "$f" | od -An -tx1 | tr -d ' ')" = "7f454c46" ]; then check_elf "$f" "$rel"; nelf=$((nelf + 1)); fi
  done < "$list"
  rm -f "$list"
  [ "$nelf" -ge 1 ] || fail "no ELF files found to check"
  # Every NEEDED of the main binary that is shipped must really be present as a regular file
  # (exact basename match — not a broken symlink left after a Windows unzip).
  for n in $($READELF -d "$port_dir/$portname.aarch64" 2>/dev/null | awk '/\(NEEDED\)/ {gsub(/[\[\]]/,"",$NF); print $NF}'); do
    if [[ " $ALLOWED_BASE " == *" $n "* ]]; then continue; fi
    if [ -f "$port_dir/libs.aarch64/$n" ] && [ ! -L "$port_dir/libs.aarch64/$n" ]; then
      ok "libs.aarch64/$n is a regular file (matches DT_NEEDED)"
    else
      fail "binary needs $n but libs.aarch64/$n is missing or not a regular file"
    fi
  done
}

[ $# -ge 1 ] || { sed -n '2,14p' "$0"; exit 2; }
for input in "$@"; do
  if [ -d "$input" ]; then
    check_tree "$(cd "$input" && pwd)" "$input"
  elif [ -f "$input" ]; then
    check_zip_no_symlinks "$input"
    tmp="$(mktemp -d "${TMPDIR:-/tmp}/portcheck.XXXXXX")"
    unzip -q -o "$input" -d "$tmp" || { fail "cannot unzip $input"; continue; }
    check_tree "$tmp" "$input"
    rm -rf "$tmp"
  else
    fail "$input not found"
  fi
done

if [ "$FAIL" = 0 ]; then echo "RESULT: all checks passed"; else echo "RESULT: FAILED"; fi
exit "$FAIL"
