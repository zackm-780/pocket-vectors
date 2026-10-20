# Device probe

A small standalone SDL2 + OpenGL ES 2.0 program (own CMake project, no dependency on the game) that
answers every open question in [DEVICE.md](../../docs/DEVICE.md) section 2 on the real RG40XXH. It is
packaged as the PortMaster port **"Flight Sim Probe"** and writes `probe_report.txt` next to its binary.
It also builds and runs on desktop Linux, so it can be verified without the device.

## What it measures

| Phase | What lands in the report |
|---|---|
| System | glibc (`gnu_get_libc_version`), kernel, OS, device-tree model, CPU (`/proc/cpuinfo`, cpufreq governor/frequencies, thermal zones), RAM (`MemAvailable`, CMA), PortMaster environment handed over by the launch script (`controlfolder`, `$directory`, `CFW_NAME`, ...), free disk space, `/dev` nodes, `/proc/bus/input/devices`, ALSA cards |
| SDL | compiled vs linked version, revision, video drivers compiled in, driver in use, every display mode, DPI |
| OpenGL ES | `GL_VENDOR/RENDERER/VERSION/SHADING_LANGUAGE_VERSION`, every extension (plus a yes/no list of the ones we care about, e.g. `GL_OES_element_index_uint`), max texture size and the other limits, shader float precision, libraries really mapped into the process (libmali / Mesa / EGL / gbm) |
| Framebuffer | colour, depth, stencil and MSAA bits actually obtained, window and drawable size, whether vsync can be switched |
| Audio | SDL audio driver, devices, the format and sample rate you really get when asking for 44.1 kHz S16 stereo, plus a short quiet test tone |
| Controllers | name, GUID, mapping string, every button/axis binding, axis/button/hat counts, battery level |
| Input test | prompts you through A/B/X/Y, L1/R1, **L2/R2** (squeeze slowly), **L3/R3**, D-pad, Select, Start, **Menu key**, volume keys and both sticks, logging every SDL controller, joystick, hat and keyboard event with names and raw values. The summary answers: analog or digital triggers, L3/R3 present, A/B swapped, Menu key visible |
| CPU benchmark | JSBSim v1.3.3 `c172p` and `737`, 2400 steps at 120 Hz after warm-up: mean/p50/p95/p99/max microseconds per step, process CPU time, the cost per 30 fps frame against the 4 ms budget; memory copy bandwidth |
| GPU benchmark | Terrain-like meshes of 10k, 40k and 80k triangles (plus 40k split into 150 draw calls, and 80k with 32-bit indices if available) with per-vertex lighting and a per-fragment fog shader. Measured **offscreen** (FBO + `glFinish`: pure GPU time, immune to vsync) and **on screen** with vsync (frame pacing, dropped frames). Fill rate (opaque, blended, blended with the fog shader). A linear fit gives "triangles per frame at 30 fps" and at 60 fps |
| Config sweep | Eight window requests (depth 16/24, stencil 8, MSAA 2x/4x, RGB565, RGBA8888), each in a fresh window and context, with what SDL really granted. Runs last, because recreating contexts is what is most likely to upset a driver |

The report starts with a **KEY FACTS** block with one entry per row of the DEVICE.md table, in the same
order. The report file is rewritten atomically after every phase, so it is useful even if the probe
crashes, hangs, or you quit early with Select+Start (the `status:` line and the last "trying ..." line
show where it stopped). Everything printed also goes to stdout (so into `log.txt` on the device).

## How to run the probe on the device (human steps)

1. Get `flightsim-probe-portmaster.zip` (CI artifact of the `device` workflow, or `tools/build_device.sh`).
2. Unzip it into the folder PortMaster uses for ports on the SD card, so that `FlightSimProbe.sh` and the
   `flightsimprobe/` folder sit next to your other ports. (Or install it with PortMaster's sideload feature.)
3. Start **Flight Sim Probe** from the Ports menu. Leave the sticks and triggers alone during the first
   1.5 s ("HANDS OFF": rest values are measured). Then do what each prompt asks. **Never press Select and
   Start together** during the input test, that is the exit chord. The whole run takes about 2 minutes.
4. At the end the summary stays on screen; press A, B or Start to leave (or wait 2 minutes).
5. Copy `flightsimprobe/probe_report.txt` and `flightsimprobe/log.txt` from the SD card and send them back.
   Paste the KEY FACTS block into DEVICE.md section 2.

Developer switches (also usable on the device through the `PROBE_EXTRA_ARGS` environment variable of the
launch script): see `flightsim_probe --help`.

## Build and run on desktop

```bash
cd flightsim
cmake -S tools/probe -B build-probe -G Ninja        # fetches JSBSim v1.3.3 (needs network the first time)
cmake --build build-probe
./build-probe/flightsim_probe                       # 640x480 window, interactive
# without a display (Mesa llvmpipe), automatic input events injected so the whole pipeline runs:
LIBGL_ALWAYS_SOFTWARE=1 xvfb-run -s "-screen 0 1280x960x24" \
    ./build-probe/flightsim_probe --auto --selftest-input --quick
# CPU benchmark only, no SDL window at all (also what the qemu smoke test runs):
./build-probe/flightsim_probe --cpu-only --quick
```

Options: `-DPROBE_WITH_JSBSIM=OFF` (skip the CPU benchmark), `-DPROBE_JSBSIM_SOURCE_DIR=<checkout>` (offline),
`-DPROBE_WERROR=ON`. The runtime data for the CPU benchmark (`jsbsim/aircraft/{c172p,737}`, `engine/`,
`systems/`) is copied next to the binary at build time.

## Design notes

- **No libGLESv2/libEGL link.** GL entry points come from `SDL_GL_GetProcAddress`; prototypes and constants
  from SDL's bundled `SDL_opengles2.h`. The binary's dynamic dependencies are `libSDL2-2.0.so.0`,
  `libJSBSim.so.1`, libc and libm only.
- **ES 2.0 first, desktop GL 2.1 as fallback** (reported in the file). Shaders use GLSL 1.00 syntax with a
  `#ifdef GL_ES` precision preamble that works on both.
- **SDL API level:** only functions that exist in SDL 2.0.4 or older are called (the device's SDL version is
  unknown), e.g. `SDL_QueueAudio`, `SDL_GetDisplayDPI`, `SDL_JoystickCurrentPowerLevel`.
- The 8x8 bitmap font is the one in SDL's `SDL_test_font.c` (zlib licence, see `port/licenses/SDL2-zlib.txt`).
- JSBSim is LGPL-2.1: built as a shared library and shipped as `libs.aarch64/libJSBSim.so*` with its licence.

## Verification done without the device

Desktop run under Xvfb + Mesa llvmpipe through every phase (with `--selftest-input` injecting a fake pad that
has swapped A/B, an analog L2, a digital R2, no R3 and no Menu key, which the summary reports correctly); an
aarch64 cross build in a Debian 11 chroot; `port/check_package.sh` (GLIBC and dependency whitelist, with
negative tests); a qemu-user run of the CPU benchmark; and the launch script run against a fake PortMaster
control folder. **Timings from Mesa or qemu mean nothing**: only the on-device numbers count.
