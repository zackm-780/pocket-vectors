# Device: Anbernic RG40XXH (stock OS)

## 1. Known specs (manufacturer)
| Item | Value |
|---|---|
| SoC | Allwinner H700, 4× Cortex-A53 @ 1.5 GHz (aarch64) |
| GPU | Mali-G31 MP2 |
| RAM | 1 GB LPDDR4 (shared with GPU) |
| Display | 4.0" IPS, **640×480** (4:3) |
| Controls | D-pad, A/B/X/Y, L1/R1, L2/R2, Select, Start, Menu/function key, volume ±, power, **two analog sticks** |
| Connectivity | Wi-Fi, Bluetooth, mini-HDMI out, USB-C |
| Storage | microSD (TF) card(s) |
| OS | Anbernic stock Linux (64-bit userspace, Ubuntu 22.04 base). Ships PortMaster. |

## 2. Unknowns: measure with the P1 probe, then fill in
| Fact | How | Value |
|---|---|---|
| glibc version | `ldd --version` / probe | 2.35 (Ubuntu GLIBC 2.35-0ubuntu3), launch log 2026-10-20 |
| Kernel | probe | Linux 4.9.170 aarch64 (`#6 SMP PREEMPT Sat Dec 21 16:10:36 CST 2024`), device tree `sun50iw9` / `allwinner,h616`. Ubuntu 22.04 userspace. PortMaster 6.55, `param_device=rg35xxh`. |
| PortMaster install path (`controlfolder`) | probe / launch script log | `/roms/ports/PortMaster` (ships with the stock OS; `directory=roms`, `CFW_NAME` empty), launch log |
| Ports folder path on the SD card | launch path | `/mnt/sdcard/Roms/PORTS` (SD card in slot 2). PortMaster's own ports folder `/roms/ports` is a different, internal folder. |
| SDL2 version and video driver | probe | **runtime SDL 2.0.12** (`/usr/lib/libSDL2-2.0.so.0.12.0`; the build compiles against 2.0.14 headers, so new code must not call anything newer than 2.0.12). Video driver `mali` (fbdev), audio `alsa`. |
| GL_VERSION / GL_RENDERER / extensions | probe | OpenGL ES 3.2 driver (`v1.r20p0`), Mali-G31, ES 2.0 context requested and granted. 94 extensions incl. OES_element_index_uint, OES_vertex_array_object, OES_depth24, OES_depth_texture, EXT_multisampled_render_to_texture, ETC1, ASTC. No float/half-float textures, no S3TC, no anisotropic filtering. |
| Depth bits / MSAA available | probe | Always depth 24 (16 is upgraded). Stencil 8 on request. MSAA 2x/4x requests both give 4 samples. Default window: RGBA8888, depth 24, no MSAA; the game itself reports `msaa 4`. |
| Max texture size | probe | 8192 (cube maps 4096) |
| L2/R2 analog or digital? | probe input log | **Digital**: SDL `lefttrigger`/`righttrigger` axes jump 0 → 32767 (raw buttons 10/11) |
| L3/R3 (stick clicks) present? | probe input log | Both present (SDL `leftstick` b9, `rightstick` b12). L3 is clean. **R3 jolts the right stick X axis to about −0.32** while clicking, so do not bind R3 to anything used in flight (rudder is on right stick X). |
| Face-button names as SDL reports them (A/B swap?) | probe input log | PortMaster mapping `a:b0,b:b1,x:b3,y:b2`. The probe's A/B/X/Y prompts got no presses (missed), but in the game A and B were swapped with the old label table (printed A = SDL `b`), so **printed labels equal SDL names** for A/B; X/Y very likely the same (PortMaster swaps raw b2/b3 to make the names match), to be confirmed in game. **Select produces SDL `guide`** (b6), not `back`. The default label table now uses `a = a` … `y = y` and `select = back, guide`. |
| Menu key usable by ports? | probe input log | Yes, it reaches the app as SDL `back` (b8) plus raw button 13. Volume keys arrive as raw buttons 15/16 and SDL VolumeUp/Down keys. |
| Free RAM at idle in a port | probe | MemAvailable 857 MB of 973 MB (no swap). Probe peak RSS 101 MB. |
| GPU: triangles/frame at 30 fps (fog shader) | probe benchmark | 40k tris = 2.5 ms GPU, 80k = 4.2 ms (fit: 1.23 ms + 0.035 ms per 1k tris). **Draw calls cost more than triangles**: the same 40k in 150 draw calls = 5.7 ms. Fill: 2086 Mpix/s opaque, 1205 blended, 311 blended + fog shader (about 17 full-screen layers per 16.7 ms). Vsync on works (16.76 ms frames). |
| CPU: JSBSim µs/step (c172p, 737) | probe benchmark | c172p 84 µs, 737 94 µs per 120 Hz step (0.34 / 0.38 ms per 30 fps frame). 4× Cortex-A53 @ 1512 MHz, governor `performance`. memcpy 1177 MB/s. |
| Audio device / sample rate | probe | ALSA `audiocodec` (plus HDMI). Probe got 44.1 kHz stereo, 1024 samples. The game opens 22.05 kHz mono. With 512 samples it logged ALSA underruns (one in a 1.0 flight, two in the 2026-11-05 run); from 1.2.0 it asks for 1024 on the device (`audio_buffer_samples`) and logs the size it got. |

## 3. Field notes
Add observations from device tests here (date, build, what was seen).

- **2026-10-19, first sideload (stock OS):** both the probe and the game opened and exited
  immediately. The guess at the time was that the stock OS has no PortMaster; the 2026-10-20 logs
  show it does (see below), so the real cause was the wrong port folder. The scripts now run without
  PortMaster (and under a plain `sh`) and always write `<port>_launch_log.txt` next to the script. They were also renamed
  to `FlightSim.sh` / `FlightSimProbe.sh` (no spaces, in case the launcher does not quote paths),
  and the log now records whether userspace is 64-bit (some stock OS releases are 32-bit,
  which the aarch64 build cannot run on). The 2026-10-20 run below is the retest.
- **2026-10-20, libJSBSim.so symlinks vs FAT/exFAT:** device zips originally stored the CMake/JSBSim
  soname chain (`libJSBSim.so` → `libJSBSim.so.1` → `libJSBSim.so.1.3.3`) with `cp -a` and `zip -y`.
  Unzipping onto a FAT/exFAT SD card (or via many Windows/macOS unzippers) drops those symlinks, so
  `DT_NEEDED: libJSBSim.so.1` fails to load. Packaging now ships one regular file
  `libs.aarch64/libJSBSim.so.1`, and `check_package.sh` fails the build if any symlink remains in
  the zip. Re-copy ports from a zip built after that fix.
- **2026-10-20, second sideload (build with the fixes above):** both ports exited again, but this
  time wrote launch logs. They show a 64-bit Ubuntu 22.04 userspace (glibc 2.35, kernel 4.9.170),
  `libSDL2-2.0.so.0` in `/usr/lib` and `/usr/lib/aarch64-linux-gnu`, and PortMaster at
  `/roms/ports/PortMaster` with `directory=roms`. The scripts were run from
  `/mnt/sdcard/Roms/PORTS`, but they used PortMaster's ports folder `/roms/ports/<port>`. That
  folder existed but was empty apart from `conf/` and `log.txt`, which the first sideload's script
  had created there, so the scripts stopped with "flightsim.aarch64 is missing". The launch scripts
  now use the folder that holds the binary, preferring the one next to the script, and log every
  folder they tried. `port/test_launch_scripts.sh` (run in the device workflow) recreates this layout.
  The leftover `/roms/ports/flightsim*` folders are harmless. The next note is the retest.
- **2026-10-20, third sideload (main `fab11fc`, one zip with the pdx_oth data pack):** first real
  run. The probe completed every section (results in section 2). The game loaded the nav database
  (49,374 airports, 1,509 real + 20,341 synthesized ILS, 1.9 s) and terrain levels 5–10, opened the
  menus, and a test flight in the Global 5000 at KPDX covered a runway start and two cruise starts.
  **A and B were swapped** with that build's label table (printed labels equal SDL names; Select is
  SDL `guide`). The default table was updated afterwards. Frame rate was not measured
  (`show_fps = true` in `flightsim/conf/settings.ini`).
  The probe was SIGKILLed by gptokeyb (`-sudokill`, exit 137) when it was quit with Select+Start.
- **2026-11-05, 1.2 build (PR #11), first frame timing run (section 6), Mali-G31:**
  the owner flew the section 6 steps and sent `flightsim_launch_log.txt` (it can be re-attached).

  | View | Preset | fps | Frame (ms) | CPU (ms) | HUD (ms) | Terrain tris / draws | Notes |
  |---|---|---|---|---|---|---|---|
  | Cockpit | MED | 30, locked | 33.5 | 11.6 | 8.0 | 44k / 70 | swap 22 ms: GPU-bound |
  | Chase | MED | 30 | | 4.5 | 0.7 | | dips to 20 fps while orbiting (GPU); terrain mesh builds are rare, 0.4–0.7 ms each |
  | Cockpit inside OVC | MED | 30 ↔ 60, avg 55 | | | | 47k | the same triangles flip between 30 and 60: the GPU sits at the 16.7 ms edge, or GPU DVFS (the GPU clock is not logged yet) |
  | Cockpit | LOW | 60, locked | p95 17.0 | 9.4 | 6.6 | 37k | |
  | Cockpit | HIGH | 30 | | | | | |

  - **The HUD is the largest CPU item:** 8.0 ms of the 11.6 ms in the cockpit (PLAN section 3
    budgets 2 ms), against 0.7 ms in the chase view. The A53 is far from the 33 ms limit, so it
    does not cost frames at 30 fps, but it is the first thing between MED and 60 fps.
  - **Looks:** LOW looks rough, MED decent. Owner decisions (2026-11-06): MED stays the default,
    HIGH stays (other devices), 1.2 ships now, and X3c follows as 1.2.1: HUD CPU, the GPU clock
    in the log, MED toward 60 fps.
  - **Audio:** two ALSA underruns with the 512-sample buffer. 1.2.0 asks for 1024 on the device.
  - **Autopilot:** L1 + R2 (ALT) worked only once the AP was on. From 1.2.0 a mode button turns
    the FD on when the AP and the FD are both off, like TO/GA.
  - The PR #11 "Needs device test" checklist was run separately; its results are in the 1.2.0
    pull request.

## 4. How to run the probe
The probe is the PortMaster port "Flight Sim Probe" (source in `tools/probe/`, full description in
`tools/probe/README.md`). Every row of the table in section 2 has a matching entry in the report's
KEY FACTS block.
1. Get `flightsim-probe-portmaster.zip` from the `device` CI workflow artifact (or `tools/build_device.sh`).
2. Unzip it so `FlightSimProbe.sh` and `flightsimprobe/` sit **side by side** in the folder your
   launcher runs ports from (PortMaster's ports folder, or the stock OS equivalent). Confirm
   `flightsimprobe/libs.aarch64/libJSBSim.so.1` is a normal file (not a symlink/shortcut).
3. Run "Flight Sim Probe" from the Ports menu. Leave the controls alone for the first 1.5 s, then
   press what each prompt asks (never Select and Start together: that exits). About 2 minutes in total.
4. The results are `flightsimprobe/probe_report.txt` and `flightsimprobe/log.txt`.
5. Record the KEY FACTS block under section 5 and fill in the table in section 2.

## 5. Probe results
2026-10-20, probe 1.0 built from `fab11fc`, stock OS (Ubuntu 22.04, kernel 4.9.170, PortMaster 6.55).
The numbers are in the table in section 2. The KEY FACTS block is also in
`flightsimprobe_launch_log.txt`. The face-button prompts (A, B, X, Y, L1) got no presses because
each prompt waits only a few seconds. A rerun that presses each button as soon as its prompt
appears would confirm D1.

## 6. Frame timing (X3)
The game times every frame, split into its passes, and writes the results to the log
(`flightsim_launch_log.txt` next to `FlightSim.sh`, and `flightsim/conf/log.txt`). This is the input for
X3's tuning and for the budgets in PLAN section 3.

**What is recorded.**
- **On screen**, with Settings → SHOW FPS ON: top left, the frames per second and the mean frame
  time over the last second, and `CPU` (the frame minus the swap).
- **Once a second in the log**, with SHOW FPS ON:
  `29.9 fps | frame 33.4 ms max 41.0 cpu 14.1 | update 2.1 terrain 6.3 airports 0.4 clouds 0.0
  model 0.0 overlay 0.1 hud 1.2 swap 19.3 | cockpit`, followed by the terrain line (nodes,
  triangles, draws).
- **A session table**, whether SHOW FPS is on or not. It is written when the pause menu opens (with at
  least 10 s of new frames), every 2 minutes, when GRAPHICS QUALITY changes and when the game exits.
  There is one row per view (cockpit, chase, map, menu, each also "clouds" when a cloud layer is drawn and
  "in cloud" inside one), with the frame count, fps, mean, 95th percentile and worst frame
  in ms, the CPU time, each pass's mean in ms, and mean terrain triangles and draw calls. The table
  is labelled with the quality preset: `perf session (MED): ...`. The first 10 frames after the window
  opens or a flight starts are left out (shader compiles, first tile uploads).

**Passes.** `update` = input, menus, flight model, GPWS, audio and tile requests. `terrain` = clear,
sky and terrain. Then `airports`, `clouds`, `model` (chase view), `overlay` (W6), `hud` (HUD, map
page and menus), and `swap`. A pass's time is when the CPU issued its GL calls. With vsync, `swap`
holds the GPU work still queued and the wait for the next refresh. So `cpu` above about 16 ms
means the CPU limits the frame rate, and a low `cpu` at 30 fps means the GPU or vsync does.
`perf_gpu_sync = true` in `conf/settings.ini` (file only, read at start) calls `glFinish` after each
pass, so each pass also holds its GPU time. On the Mali this splits the frame into several
render passes, so the total grows: compare the passes with each other, not with an unsynced run.

**Device results.** The first run (1.2, 2026-11-05) is the field note in section 3: MED holds
30 fps in the cockpit (GPU-bound, HUD 8 ms of CPU), LOW 60, HIGH 30.

**The device run X3 needs.** About 6 minutes:
1. Settings: SHOW FPS ON, GRAPHICS QUALITY MED.
2. New flight: any aircraft at KPDX, start IN THE AIR at 3,000 ft. Fly 1 minute in the cockpit
   view, toward Mt Hood (east) if you can.
3. L2 + L3: chase view for 30 s, then L2 + L3 back.
4. Pause → Settings: CLOUDS OVC, CLOUD BASE 4,000 FT (a layer above you). Fly 30 s. Then CLOUD
   BASE 2,000 FT (you are inside the layer, which is 2,500 ft thick). Fly 30 s. Then CLOUDS AUTO.
5. Pause → Settings: GRAPHICS QUALITY LOW, fly 30 s; HIGH, fly 30 s. Say which looked acceptable.
6. Quit from the pause menu (QUIT), not with Select + Start: gptokeyb can kill the game
   (exit 137) before it writes the last table.
7. Send `flightsim_launch_log.txt`. The `perf session` tables are the numbers X3 needs. Note the
   on-screen FPS at each step too, in case the log is cut short.

A second run with `perf_gpu_sync = true` shows which pass costs the GPU the most.

**Desktop reference (not device numbers).** Mesa llvmpipe (software GL, 4 threads on a 2.1 GHz
Xeon) at 640 × 480 with MSAA 4, 2026-11-05. Terrain is a KPDX–Mt Hood pack (bbox
-123.1,44.9,-121.3,45.9, levels 7–10) seen from the `FSIM_W2_CAMERA` pose. Means over 389
frames. "Synced" is the terrain pass with `perf_gpu_sync`, which on llvmpipe is the raster time.

| View | Preset | Triangles | Terrain draws | fps | Terrain pass, synced (ms) |
|---|---|---|---|---|---|
| 3,000 ft east of KPDX, toward Mt Hood | LOW | 24,080 | 36 | 59.5 | 13.3 |
| | MED | 35,600 | 54 | 46.6 | 18.2 |
| | HIGH | 52,240 | 80 | 39.0 | 23.1 |
| 9,000 ft near Mt Hood, 6° down | LOW | 26,640 | 40 | 60.1 | 14.3 |
| | MED | 33,680 | 51 | 52.7 | 18.4 |
| | HIGH | 43,280 | 66 | 44.2 | 18.6 |
| 1,500 ft over Portland, west | LOW | 33,680 | 51 | 52.4 | 15.3 |
| | MED | 35,600 | 54 | 49.8 | 16.8 |
| | HIGH | 51,600 | 79 | 39.3 | 21.8 |

The same 3,000 ft view at MED, synced (ms): an OVC layer above adds 3.6 ms of clouds; inside the
layer, 3.3 ms; a BKN top seen from 9,000 ft, 3.6 ms. The chase model is 0.9 ms; the HUD is 2.7–3.1 ms.
Detail noise is 0.3 ms of the terrain pass in the Mt Hood view and 1.3 ms over Portland (MED's
detail at LOW's LOD). Near the ground the view is grazing, so LOW saves only about 5 % of the triangles there.
Most of LOW's saving at low altitude has to come from the shader, and the Mali's cost for it is
the open question. CPU per frame, unsynced: update 0.05 ms, terrain submit 2.8–5.3 ms (it scales
with the draws), HUD 0.8–1.3 ms. The A53 is several times slower, so the device's `cpu` column
matters as much as its fps.
