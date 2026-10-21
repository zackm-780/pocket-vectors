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
| Ports folder path on the SD card | user | `/mnt/sdcard/Roms/PORTS` (SD card in slot 2). PortMaster's own ports folder `/roms/ports` is a different, internal folder. |
| SDL2 version and video driver | probe | **runtime SDL 2.0.12** (`/usr/lib/libSDL2-2.0.so.0.12.0`; the build compiles against 2.0.14 headers, so new code must not call anything newer than 2.0.12). Video driver `mali` (fbdev), audio `alsa`. |
| GL_VERSION / GL_RENDERER / extensions | probe | OpenGL ES 3.2 driver (`v1.r20p0`), Mali-G31, ES 2.0 context requested and granted. 94 extensions incl. OES_element_index_uint, OES_vertex_array_object, OES_depth24, OES_depth_texture, EXT_multisampled_render_to_texture, ETC1, ASTC. No float/half-float textures, no S3TC, no anisotropic filtering. |
| Depth bits / MSAA available | probe | Always depth 24 (16 is upgraded). Stencil 8 on request. MSAA 2x/4x requests both give 4 samples. Default window: RGBA8888, depth 24, no MSAA; the game itself reports `msaa 4`. |
| Max texture size | probe | 8192 (cube maps 4096) |
| L2/R2 analog or digital? | probe input log | **Digital**: SDL `lefttrigger`/`righttrigger` axes jump 0 → 32767 (raw buttons 10/11) |
| L3/R3 (stick clicks) present? | probe input log | Both present (SDL `leftstick` b9, `rightstick` b12). L3 is clean. **R3 jolts the right stick X axis to about −0.32** while clicking, so do not bind R3 to anything used in flight (rudder is on right stick X). |
| Face-button names as SDL reports them (A/B swap?) | probe input log | PortMaster mapping `a:b0,b:b1,x:b3,y:b2`. The probe's A/B/X/Y prompts got no presses (missed), but in the game A and B were swapped with the old label table (printed A = SDL `b`), so **printed labels equal SDL names** for A/B; X/Y very likely the same (PortMaster swaps raw b2/b3 to make the names match), to be confirmed. **Select produces SDL `guide`** (b6), not `back`. See WORK_PACKAGES D1. |
| Menu key usable by ports? | probe input log | Yes, it reaches the app as SDL `back` (b8) plus raw button 13. Volume keys arrive as raw buttons 15/16 and SDL VolumeUp/Down keys. |
| Free RAM at idle in a port | probe | MemAvailable 857 MB of 973 MB (no swap). Probe peak RSS 101 MB. |
| GPU: triangles/frame at 30 fps (fog shader) | probe benchmark | 40k tris = 2.5 ms GPU, 80k = 4.2 ms (fit: 1.23 ms + 0.035 ms per 1k tris). **Draw calls cost more than triangles**: the same 40k in 150 draw calls = 5.7 ms. Fill: 2086 Mpix/s opaque, 1205 blended, 311 blended + fog shader (about 17 full-screen layers per 16.7 ms). Vsync on works (16.76 ms frames). |
| CPU: JSBSim µs/step (c172p, 737) | probe benchmark | c172p 84 µs, 737 94 µs per 120 Hz step (0.34 / 0.38 ms per 30 fps frame). 4× Cortex-A53 @ 1512 MHz, governor `performance`. memcpy 1177 MB/s. |
| Audio device / sample rate | probe | ALSA `audiocodec` (plus HDMI). Probe got 44.1 kHz stereo, 1024 samples. The game opens 22.05 kHz mono with 512 samples and logged one ALSA underrun in a flight. |

## 3. Field notes
Add observations from device tests here (date, build, what was seen).

- **2026-10-19, first sideload (stock OS):** both the probe and the game opened and exited
  immediately. The guess at the time was that the stock OS has no PortMaster; the 2026-10-20 logs
  show it does (see below), so the real cause was the wrong port folder. The scripts now run without
  PortMaster (and under a plain `sh`) and always write `<port>_launch_log.txt` next to the script. They were also renamed
  to `FlightSim.sh` / `FlightSimProbe.sh` (no spaces, in case the launcher does not quote paths),
  and the log now records whether the userspace is 64-bit (some stock OS releases are 32-bit,
  which the aarch64 build cannot run on). Awaiting a retest.
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
  The leftover `/roms/ports/flightsim*` folders are harmless. Awaiting a retest.
- **2026-10-20, third sideload (main `fab11fc`, one zip with the pdx_oth data pack):** first real
  run. The probe completed every section (results in section 2). The game loaded the nav database
  (49,374 airports, 1,509 real + 20,341 synthesized ILS, 1.9 s) and terrain levels 5–10, opened the
  menus, and the user flew the Global 5000 at KPDX from a runway start and two cruise starts. Problems
  reported: **A and B swapped**, maybe more buttons (see D1; the probe shows Select = SDL `guide`).
  Not yet measured: frame rate (set `show_fps = true` in `flightsim/conf/settings.ini`).
  The probe was SIGKILLed by gptokeyb (`-sudokill`, exit 137) when it was quit with Select+Start.

## 4. How to run the probe
The probe is the PortMaster port "Flight Sim Probe" (source in `tools/probe/`, full description in
`tools/probe/README.md`). Every row of the table in section 2 has a matching entry in the report's
KEY FACTS block.
1. Get `flightsim-probe-portmaster.zip` from the `device` CI workflow artifact (or `tools/build_device.sh`).
2. Unzip it so `FlightSimProbe.sh` and `flightsimprobe/` sit **side by side** in the folder your
   launcher runs ports from (PortMaster's ports folder, or the stock OS equivalent). Confirm
   `flightsimprobe/libs.aarch64/libJSBSim.so.1` is a normal file (not a symlink/shortcut).
3. Run "Flight Sim Probe" from the Ports menu. Hands off for the first 1.5 s, then press what each prompt
   asks (never Select and Start together: that exits). About 2 minutes in total.
4. Send back `flightsimprobe/probe_report.txt` and `flightsimprobe/log.txt`.
5. Paste the KEY FACTS block under section 5 and fill in the table in section 2.

## 5. Probe results
2026-10-20, probe 1.0 built from `fab11fc`, stock OS (Ubuntu 22.04, kernel 4.9.170, PortMaster 6.55).
The numbers are in the table in section 2. The KEY FACTS block of `probe_report.txt` was not sent back
yet; the launch log `flightsimprobe_launch_log.txt` carries the same sections. The face-button
prompts (A, B, X, Y, L1) got no presses because each prompt waits only a few seconds; rerun the
input test and press each button as soon as its prompt appears to confirm D1.
