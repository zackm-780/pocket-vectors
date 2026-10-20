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
  JSBSim is built as `libJSBSim.so` and shipped in `libs.aarch64/`.
- `tools/build_device.sh` runs the build inside the Docker image from `tools/docker/` and writes
  `dist/flightsim-portmaster.zip`. With no Docker (or on CI), `.github/workflows/device.yml` does the
  same and uploads the zip as a workflow artifact.
- Checks (P1 adds them to CI): no `GLIBC_` symbol version above 2.31, and dynamic deps limited to
  libc, libm, libdl, libpthread, librt, `libSDL2-2.0.so.0`, plus what we ship.
- If the probe shows the device lacks `libSDL2-2.0.so.0` outside PortMaster's environment, ship SDL2
  in `libs.aarch64/` (zlib license) and record a decision.

## 6. PortMaster package layout (sideload)

```
FlightSim.sh                       # launch script (PortMaster template; no spaces in the name)
flightsim/
  flightsim.aarch64                # main binary
  libs.aarch64/libJSBSim.so*       # shipped shared libs
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
- `GAMEDIR="/$directory/ports/flightsim"`, `cd "$GAMEDIR"`, tee the log to `$GAMEDIR/log.txt`.
- `export LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"`,
  `export SDL_GAMECONTROLLERCONFIG="$sdl_controllerconfig"`, `export FLIGHTSIM_PLATFORM=portmaster`.
- `$GPTOKEYB "flightsim.${DEVICE_ARCH}" &`: used **only** for the Select+Start exit hotkey. The game
  reads the gamepad itself and ignores keyboard events on this platform (gptokeyb may inject keys).
- `pm_platform_helper "$GAMEDIR/flightsim.${DEVICE_ARCH}"`, run the binary with
  `--fullscreen --conf-dir "$GAMEDIR/conf"`, then `pm_finish`.

## 7. Installing on the RG40XXH (human steps)
PortMaster is **optional**. The Anbernic stock OS does not ship it, and the launch scripts work
without it: they then use the folder next to the script and the game handles Select+Start itself.
1. Copy `FlightSim.sh` **and** the `flightsim/` folder side by side into the folder your launcher
   runs ports or scripts from (the same folder as each other). The exact path on the stock OS goes
   into DEVICE.md once confirmed. The script names have no spaces on purpose: a launcher that does
   not quote paths would fail on `Flight Sim.sh` before the script even starts.
2. Copy your data pack into `flightsim/data/`. Alternatively, put it anywhere on the card and set
   `data_dir` in `flightsim/conf/settings.ini`.
   **Getting the data pack (no PC pipeline needed):** on GitHub open the Actions tab, pick the
   **datapack** workflow, "Run workflow" (defaults: preset `pdx_oth`, world layer on). When it
   finishes, download the artifact `flightsim-data-<preset>-<date>` (a zip), **unzip it into
   `flightsim/data/`** (the zip's top level is `terrain/`, `nav/`, `ATTRIBUTION.txt`, `README.txt`)
   and copy that folder to the card. Terrain is built from Copernicus DEM and ESA WorldCover, nav
   from OurAirports, FAA NASR and CIFP (real approaches) and WMM. To build it on a PC instead:
   `flightsim/tools/datapack/build_datapack.sh --out flightsim/data` (see its README). The pack is
   about 3 GB with the world layer, 0.4 GB without (`include_world` off).
3. Launch "Flight Sim". Exit with **Select + Start**.
4. **Logs:** every launch writes `flightsim_launch_log.txt` **next to `FlightSim.sh`**, even when
   the game never starts. It records PortMaster detection, the folder used, the libraries found, and the
   game's own output and exit status, and whether the OS userspace is 64-bit (the build needs it;
   some Anbernic stock OS releases are 32-bit). The game also keeps `flightsim/conf/log.txt`.
5. Run the probe port ("Flight Sim Probe") once and send back `flightsimprobe/probe_report.txt` and
   `flightsimprobe_launch_log.txt`.

## 8. Device test checklist (per milestone, human)
- Launches, renders at the native 640×480, exits cleanly with Select+Start.
- Every control in CONTROLS.md does what it says (the probe log helps if names are swapped).
- FPS overlay (`--show-fps` or the setting) is ≥ 30 in the milestone's test scenario.
- No stutter while flying fast over detailed terrain (tile streaming).
- Note anything odd in DEVICE.md under "Field notes".
