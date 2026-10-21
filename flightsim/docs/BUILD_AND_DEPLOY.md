# Build and deploy

## 1. Desktop prerequisites
- **Linux (reference, used by CI):** Ubuntu 22.04+ with
  `sudo apt install build-essential cmake ninja-build git libsdl2-dev xvfb mesa-utils libgl1-mesa-dri`.
- **macOS:** Xcode CLT, `brew install cmake ninja sdl2`. Uses the desktop GL 2.1 fallback.
- **Windows:** Visual Studio 2022 + CMake, with SDL2 via vcpkg (`vcpkg install sdl2`). Uses the
  GL 2.1 fallback.
- CMake ≥ 3.18. JSBSim, doctest and stb are fetched by CMake (network needed on the first configure).
- Optional and recommended when several builds run on one machine: `ccache`, enabled with
  `-DCMAKE_CXX_COMPILER_LAUNCHER=ccache -DCMAKE_C_COMPILER_LAUNCHER=ccache`. JSBSim is then
  compiled only once.

## 2. Desktop build
```bash
cd flightsim
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
./build/flightsim                         # window 1280×960 (2× the device resolution)
./build/flightsim --aircraft 737 --scenario assets/scenarios/737_final_xtst.ini
```
The runtime layout is assembled in `build/` (`jsbsim/`, `assets/`), so the binary runs in place.

## 3. CLI flags
See ARCHITECTURE §9. All flags work on desktop and device.

## 4. Headless and visual testing (no display)
```bash
./build/flightsim --headless --scenario assets/scenarios/c172_final_xtst.ini --duration 120 --trace /tmp/t.csv
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -s "-screen 0 1280x960x24" \
  ./build/flightsim --scenario assets/scenarios/c172_final_xtst.ini --frames 90 --screenshot /tmp/s.png
```
Mesa's llvmpipe provides GLES2 and GL 2.1 in software, which is enough for screenshots. Performance
numbers from it mean nothing. Device performance comes from the P1 probe and X3.

## 5. Device build (aarch64)
- **Baseline (D15):** Debian 11 bullseye (glibc 2.31), cross g++ for `aarch64-linux-gnu`, and
  `libsdl2-dev:arm64` headers and libs for linking. Link flags: `-static-libstdc++ -static-libgcc`.
  JSBSim is built as a shared library; the device zip ships **one regular file**
  `libs.aarch64/libJSBSim.so.1` (the ELF SONAME / `DT_NEEDED` name). The CMake install's
  `libJSBSim.so` → `libJSBSim.so.1` → `libJSBSim.so.1.3.3` symlink chain is collapsed at package
  time because FAT/exFAT SD cards (and many Windows/macOS unzip tools) cannot store symlinks.
- `tools/build_device.sh` runs the build inside the Docker image from `tools/docker/` and writes
  `dist/flightsim-portmaster.zip`. With no Docker (or on CI), `.github/workflows/device.yml` does the
  same and uploads the zip as a workflow artifact.
- Checks (`port/check_package.sh`, run in the device workflow): no `GLIBC_` symbol version above 2.31;
  dynamic deps limited to libc, libm, libdl, libpthread, librt, `libSDL2-2.0.so.0`, plus what we ship;
  **no symlinks** in the zip or unpacked tree; no FAT-illegal path characters or case collisions;
  each shipped `DT_NEEDED` is a regular file under `libs.aarch64/`.
- If the probe shows the device lacks `libSDL2-2.0.so.0` outside PortMaster's environment, ship SDL2
  in `libs.aarch64/` (zlib license) and record a decision.

## 6. PortMaster package layout (sideload)

```
FlightSim.sh                       # launch script (PortMaster template; no spaces in the name)
flightsim/
  flightsim.aarch64                # main binary
  libs.aarch64/libJSBSim.so.1      # shipped shared lib (real file; no soname symlinks)
  jsbsim/{aircraft,engine,systems} # stock files for the fleet + overlay
  assets/                          # profiles, scenarios, sample plans, defaults
  conf/                            # created at runtime: settings.ini, controls.ini, log
  data/                            # user-built data pack (terrain/, nav/, plans/) — not in the zip
  licenses/                        # JSBSim (LGPL-2.1), SDL2 (zlib), gptokeyb, data attributions
  port.json  gameinfo.xml  README.md  screenshot.png
```

Launch script requirements (P1). Follow the official template at
<https://portmaster.games/packaging.html>:
- Locate `controlfolder`, `source control.txt`, source `mod_${CFW_NAME}.txt` if present, call `get_controls`.
- `GAMEDIR` is the `flightsim/` folder next to the script if it holds the binary, else PortMaster's
  `/$directory/ports/flightsim`. The template uses only the second, but on the RG40XXH stock OS
  PortMaster's ports folder (`/roms/ports`) is internal storage, and ports copied to the SD card
  run from `/mnt/sdcard/Roms/PORTS`. Then `cd "$GAMEDIR"`, and log to `<port>_launch_log.txt` next
  to the script. `port/test_launch_scripts.sh` checks the folder choice on fake layouts.
- `export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"`,
  `export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"`, `export FLIGHTSIM_PLATFORM=portmaster`.
- `$GPTOKEYB "flightsim.${DEVICE_ARCH}" &`: used **only** for the Select+Start exit hotkey. The game
  reads the gamepad itself and ignores keyboard events on this platform (gptokeyb may inject keys).
- `pm_platform_helper "$GAMEDIR/flightsim.${DEVICE_ARCH}"`, run the binary with
  `--fullscreen --conf-dir "$GAMEDIR/conf"`, then `pm_finish`.

## 7. Installing on the RG40XXH (human steps)
PortMaster is **optional**. The RG40XXH stock OS ships it (`/roms/ports/PortMaster`), but the
launch scripts also work without it. Either way they use the port folder next to the script when
it holds the binary. Without PortMaster the game handles Select+Start itself.

### 7.1 Extract the port zip (SD-card safe)
1. Download `flightsim-portmaster.zip` (and optionally `flightsim-probe-portmaster.zip`) from the
   `device` CI workflow artifact or from `tools/build_device.sh` → `flightsim/dist/`.
2. Unzip **onto the SD card** (or into a folder you then copy onto the card) so that these sit
   **side by side** in the ports/scripts folder your launcher uses:
   - `FlightSim.sh`
   - `flightsim/` (directory)
   The same pattern for the probe: `FlightSimProbe.sh` next to `flightsimprobe/`.
3. Prefer unzipping with a tool that keeps Unix paths as written (the zip itself contains **no
   symlinks**). Safe options: the handheld's own file manager, 7-Zip, or `unzip` on Linux/macOS.
   Avoid re-zipping the port folder on Ubuntu with tools that turn libraries into symlinks again.
4. After extract, confirm `flightsim/libs.aarch64/libJSBSim.so.1` is a **normal file** (not a
   shortcut/symlink). The binary's `DT_NEEDED` is exactly that name; `LD_LIBRARY_PATH` points at
   `libs.aarch64`.

### 7.2 Data pack and launch
1. Copy your data pack into `flightsim/data/`. Alternatively, put it anywhere on the card and set
   `data_dir` in `flightsim/conf/settings.ini`.
   **Getting the data pack (no PC pipeline needed):** on GitHub open the Actions tab, pick the
   **datapack** workflow, "Run workflow" (defaults: preset `pdx_oth` plus a Pacific Northwest coarse layer for the far view; world layer off). When it
   finishes, download the artifact `flightsim-data-<preset>-<date>` (a zip), **unzip it into
   `flightsim/data/`** (the zip's top level is `terrain/`, `nav/`, `ATTRIBUTION.txt`, `README.txt`)
   and copy that folder to the card. Terrain is built from Copernicus DEM and ESA WorldCover, nav
   from OurAirports, FAA NASR and CIFP (real approaches) and WMM. To build it on a PC instead:
   `flightsim/tools/datapack/build_datapack.sh --out flightsim/data` (needs **Python ≥ 3.12**; see its README). The pack is
   about 0.4 GB by default and about 3 GB with the world layer (`include_world` on; hours of
   downloads, only needed to fly far outside the region).
2. Launch "Flight Sim". Exit with **Select + Start**.
3. **Logs:** every launch writes `flightsim_launch_log.txt` **next to `FlightSim.sh`**, even when
   the game never starts. It records PortMaster detection, the folder used, the libraries found, and the
   game's own output and exit status, and whether the OS userspace is 64-bit (the build needs it;
   some Anbernic stock OS releases are 32-bit). The game also keeps `flightsim/conf/log.txt`.
4. Run the probe port ("Flight Sim Probe") once and send back `flightsimprobe/probe_report.txt` and
   `flightsimprobe_launch_log.txt`.

### 7.3 Linux / handheld notes (stock Anbernic)
- **Filesystem:** microSD is usually FAT32 or exFAT — no symlinks, case-insensitive names, and the
  execute bit is often lost. Launch scripts detect a non-executable binary and start it via
  `/lib/ld-linux-aarch64.so.1` (same for the probe).
- **Line endings:** keep `FlightSim.sh` / `FlightSimProbe.sh` as Unix LF. CRLF from a Windows editor
  can break `#!/bin/bash`.
- **Userspace:** the build is **aarch64 (64-bit)** glibc. Some stock images are 32-bit armhf; those
  cannot run this port (`/lib/ld-linux-aarch64.so.1` missing). The launch log records this.
- **SDL2:** linked dynamically to `libSDL2-2.0.so.0` from the system / PortMaster runtime; JSBSim is
  the only library we ship under `libs.aarch64/`.
- **RPATH / loader:** the binary has `RUNPATH=$ORIGIN/libs.aarch64`; the launch script also sets
  `LD_LIBRARY_PATH=$GAMEDIR/libs.${DEVICE_ARCH}:…` as a belt-and-braces measure.

## 8. Device test checklist (per milestone, human)
- Launches, renders at the native 640×480, exits cleanly with Select+Start.
- Every control in CONTROLS.md does what it says (the probe log helps if names are swapped).
- FPS overlay (`--show-fps` or the setting) is ≥ 30 in the milestone's test scenario.
- No stutter while flying fast over detailed terrain (tile streaming).
- Note anything odd in DEVICE.md under "Field notes".
