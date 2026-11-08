Status: done (needs device test: the DEVICE.md §6 1.2.1 run and the chase-view checks below; ships as 1.2.1)
# X3c: Device tuning: HUD CPU, GPU clock, MED toward 60, chase-view flight data

Phase 2b · Effort: high · Depends: T2 (done) · Branch: `wp/X3c-device-tuning` · Release: **1.2.1**

## Why

The first device timing run (RG40XXH, Mali-G31, 2026-11-05; DEVICE.md §3, PLAN §3):

| View | Preset | fps | CPU (ms) | HUD (ms) | Terrain | Notes |
|---|---|---|---|---|---|---|
| Cockpit | MED | 30 locked (33.5 ms) | 11.6 | 8.0 | 44k tris / 70 draws | swap 22 ms: GPU-bound |
| Chase | MED | 30 | 4.5 | 0.7 | | dips to 20 fps while orbiting (GPU) |
| Cockpit in OVC | MED | 30 ↔ 60, avg 55 | | | 47k tris | GPU at the 16.7 ms edge, or GPU DVFS |
| Cockpit | LOW | 60 locked (p95 17.0) | 9.4 | 6.6 | 37k tris | |
| Cockpit | HIGH | 30 | | | | |

Owner decisions (2026-11-06): **MED stays the default** (LOW looks rough, MED decent) and
**HIGH stays** (other devices). 2026-11-07: the chase view must show the flight data ("impossible
to see that info in chase").

Planner findings (desktop, 2026-11-07), so the builder doesn't have to repeat them:

- **HUD CPU.** callgrind of `Hud::draw` + `Renderer2D::end` with the fake GL of
  `tests/hud/hudRenderTest.cpp` and the `loc_gs` nav state: the cockpit batch is **33.8k vertices**
  (half of them the outline pass), 209 µs a frame on a desktop core; the chase batch 7.2k, 36 µs.
  String formatting and HUD logic are under 2 %. The cost is per vertex in `Renderer2D`:
  `emitTriangle` 21 %, `emitStroke` 21 %, `pushVertex` (`std::vector::push_back`) 24 %, and the
  `merged_` copy in `end()` 15 %. `emitStroke` runs twice per stroke (outline, then colour) and
  recomputes every direction (`sqrt`). So the fix is in `Renderer2D`, it can be exact (the same
  floats in the same order), and the screenshots can match bit for bit.
- **HUD sync.** The HUD pass is 8.0 ms at MED and 6.6 ms at LOW with the same HUD, so part of it
  is waiting on the GPU. `end()` re-specifies **one** VBO every frame with `BufferData`; if the
  driver doesn't orphan it, that waits for the GPU to finish the previous frame's HUD draw.
- **GPU.** LOW (no detail noise, 37k tris) holds 60 fps, MED (detail noise, 44k tris) holds 30.
  7k triangles are about 0.25 ms on the G31 (DEVICE.md §2: 0.035 ms per 1k), so the gap is mostly
  the terrain fragment shader: two octaves of 3D value noise, 8 `hash3` each, in highp, on every
  terrain pixel within 7.5 km (`uDetail` fades 150–1500 m and 600–7500 m). The sky pass shades
  all 307k pixels with a highp shader before the terrain covers half of them.
- Airport draws are not in the perf draw count yet (`perf.drawCalls` in App.cpp), so the 70 draws
  are terrain + HUD. Merging terrain nodes into fewer draws is a larger change than this card;
  70 draws are about 1.5 ms of GPU (DEVICE.md §2).

## Read

- This card. Then, in the order you need them:
- `src/hud/Renderer2D.h` (whole file, 228 lines) and `src/hud/Renderer2D.cpp`:
  `grep -n 'void Renderer2D::end\|emitTriangle\|emitStroke\|strokePath\|fillPath\|drawGlyphStrokes\|void Renderer2D::text' src/hud/Renderer2D.cpp`
- `src/gfx/GlApi.h`: `grep -n 'FSIM_GL_FUNCTIONS\|BufferData' src/gfx/GlApi.h`
- `src/hud/Hud.cpp`: `grep -n 'options.externalView\|void drawSpeedTape\|void drawAltTape\|void drawHeadingTape\|Current value box\|Value box\|Current heading box' src/hud/Hud.cpp`
- `src/hud/HudLogic.h`: `grep -n 'struct HudOptions\|struct HudVisibility\|namespace hudzones\|kSpeedTapeGeom\|formatAltimeterSetting\|formatVs' src/hud/HudLogic.h`
- `src/hud/NavHud.cpp`: `NavHud::draw` and `NavHud::drawFma` (lines 51–120)
- `src/app/App.cpp`: `grep -n 'logPerfSession\|windowLine\|perf.drawCalls\|hudOptions.externalView\|airports.render' src/app/App.cpp`
- `src/app/FrameStats.h` (whole file) and `FrameStats::sessionSummary` in `src/app/FrameStats.cpp`
- `src/world/TerrainRenderer.cpp`: `grep -n 'kFragmentBody\|vnoise\|hash3\|cfg_.detailNoise\|setNoiseUniforms\|fogUniforms_.locate' src/world/TerrainRenderer.cpp`
- `src/app/RenderQuality.{h,cpp}` (whole, 80 lines) and `TerrainRenderConfig` in `src/world/TerrainRenderer.h`
- `src/gfx/DebugRenderer.cpp` around `sky_->draw` (`grep -n 'sky_->draw' src/gfx/DebugRenderer.cpp`) and `Sky::draw` in `src/world/Sky.cpp`
- `src/world/AirportRenderer.h` (`struct Stats`) and `grep -n 'DrawElements\|DrawArrays\|stats_\.' src/world/AirportRenderer.cpp`
- `docs/HUD.md` §3 (zone table) and §7a; `docs/DEVICE.md` §6

## Owns

`src/hud/Renderer2D.{h,cpp}`, `src/hud/Hud.cpp`, the chase-readout part of `src/hud/HudLogic.{h,cpp}`,
new `src/app/GpuClock.{h,cpp}`, `src/app/FrameStats.{h,cpp}`, `src/app/RenderQuality.{h,cpp}`, the
detail-noise part of `src/world/TerrainRenderer.{h,cpp}`, new tests `tests/hud/hudChaseTest.cpp`,
`tests/hud/renderer2DBatchTest.cpp`, `tests/app/gpuClockTest.cpp`, the X3c rows of `docs/DEVICE.md`
§6, `docs/HUD.md` §7a, and the 1.2.1 release lines (CMakeLists `project(... VERSION)`, CHANGELOG,
ROADMAP, WORK_PACKAGES X3, this card).

**Also touches:** `src/gfx/GlApi.h` (one line: `BufferSubData`), `src/hud/NavHud.cpp` (FMA in the
chase view), `src/app/App.cpp` (GPU clock lines, airport draws in the count), `src/gfx/DebugRenderer.cpp`
and `src/world/Sky.cpp` (sky after the terrain), `src/world/AirportRenderer.{h,cpp}` (a draw counter),
the fake GLs in `tests/hud/hudRenderTest.cpp` and `tests/hud/fontRenderer2DTest.cpp` (`BufferSubData`).

## Do

**Before any change:** build, then save the screenshot set below to a scratch folder (deterministic:
two runs give 0 differing pixels). After each part, rerun it and diff (numpy/PIL, max channel
difference and the count of differing pixels per image).
`--window 640x480 --frames 90 --screenshot`, a `--conf-dir` with `show_fps = false`:
c172_final_xtst `--hud-fake loc_gs`; 737_cruise_fl350_xtst `--hud-fake gps_enr`; ap_ils_c172;
c172_runway_xtst; the same final and runway with `external_view = true`; `--glyph-sheet`;
ap_ils_c172 `--map 10`; c172_final_night_xtst `--hud-fake fd`; ap_lnav_c172 with `hud_mini_nav`
and all four overlays on; `--menu pause`. With the pdx_oth data pack (`--data-dir`): c172_final_kpdx,
c172_final_koth and c172_final_kpdx_night at LOW, MED and HIGH, and the KPDX final in the chase view.

**A. HUD CPU (no visible change).**
1. `Renderer2D` vertex storage: replace the three `std::vector<Vertex>` with a small growable array
   (`Vertex* grow(int n)` returns n slots, capacity kept across frames, no per-vertex capacity
   check), and write vertices through the pointer. Fills keep going through `emitTriangle`.
2. One stroke, both passes: dedupe the points once and compute each segment's direction once
   (the same `dx / len, dy / len` as today), then emit the outline pass (width + 2 × outline) into
   the outline array and the colour pass into the colour array, in today's order (segments, then
   joins). Same floats, same order, so the batch is bit-identical.
3. Clip at stroke level: bound the stroke's points, grow the box by 1.5 × the outline half-width
   (square caps reach √2 × half-width). Box inside the clip, or no clip: write without
   per-triangle tests. Box outside: skip the stroke. Otherwise the per-triangle path as today.
4. Pack the text colour and its outline colour once per `text()` call, not per glyph stroke.
5. `end()`: no `merged_` copy. `BufferData(size, nullptr)` and three `BufferSubData` (back,
   outline, colour). Add `BufferSubData` to `FSIM_GL_FUNCTIONS`; fall back to the merged upload if
   the pointer is null. `batchData()` (tests only) builds the merged copy on demand.
6. Rotate through 4 VBOs, one per `end()` (the HUD and the menu are two batches in some frames),
   so `BufferData` never re-specifies a buffer the GPU may still read.
7. Keep a vertex-count guard: a test that the `loc_gs` cockpit batch stays under 40k vertices and
   that the batch is identical before and after a `begin()`/`end()` pair with the clip on.

**B. GPU clock in the log.**
1. New `GpuClock` (`src/app/GpuClock.{h,cpp}`): `open(root = "/sys/class/devfreq")` scans the
   entries once at start and picks the one whose name contains `gpu` or `mali` (case-insensitive),
   else the only entry; none or no folder means "not available" (desktop), not an error. It reads
   `cur_freq`, `max_freq`, `min_freq`, `available_frequencies` and `governor` (Hz in sysfs, MHz
   in the log).
2. Log once at start: `gpu clock: <entry> <governor> cur 648 MHz (min 216 max 648; available 216 312 432 648)`.
   Append ` | gpu 648/648 MHz` (cur/max) to every SHOW FPS log line (once a second, not per frame)
   and a `gpu clock:` line under every `perf session` table. Nothing is printed when not available.
3. Test with a temporary fake sysfs tree: entry choice, Hz → MHz, missing files, empty folder.

**C. MED toward 60 fps (keep MED's look; the owner judges on the device).**
1. Detail noise from a texture on MED: a 256 × 256 RGBA8 table of random bytes, built once at
   init from a fixed seed, `GL_LINEAR`, `GL_REPEAT`, no mipmaps; G at (x, y) holds R at
   (x + 37, y + 17). `vnoise(p)`: `i = floor(p); f = fract(p); f = f*f*(3-2f);
   uv = i.xy + vec2(37,17) * i.z + f.xy; rg = texture2D(t, (uv + 0.5) / 256.0).rg;
   return mix(rg.r, rg.g, f.z)`. That is the same smoothstep 3D value noise as today (other random
   values), periodic in 256 cells like the `mod(i, 256)` it replaces, at one texture fetch per
   octave instead of 8 `hash3`. Same octaves, fades and amplitude.
2. `RenderQuality::terrainNoise` (`Off`, `Texture`, `Procedural`) replaces `terrainDetailNoise`:
   LOW Off, MED Texture, **HIGH Procedural** (the 1.2.0 shader, the reference look and an A/B on
   the device: flip MED ↔ HIGH in the pause menu). Build both programs at init.
3. Sky after the terrain: draw the sky in `DebugRenderer::render` after `terrain_->render`, with
   `GL_DEPTH_TEST` on, `GL_LEQUAL`, depth writes off (the sky triangle is at z = w, depth 1.0, so
   it only fills pixels the terrain left at the cleared depth). Pixels must not change.
4. Count the airport draws (`AirportRenderer::Stats::drawCalls`) and add them to `perf.drawCalls`,
   so the next device table shows every draw.

**D. Flight data in the chase view (owner, 2026-11-07).** Today `Hud::draw` returns after the
status rows when `options.externalView` is set (HUD.md §7a).
1. Pure layout in HudLogic: `ChaseReadout buildChaseReadout(const AircraftState&, const HudOptions&,
   const HudVisibility&, float virtualWidth)` with the strings and boxes of each item:
   - IAS box: the speed tape's value box (66 × 26 at x 18, centred on y 240), text `%d` kt;
   - altitude box: the altitude tape's value box (70 × 28, right edge at `rightX() - 18`), indicated
     altitude (`altMslFt + altBaroOffsetFt`, thousands large, the rest in 20 ft steps);
   - heading box: the heading tape's box (40 × 18 at y 24, centred), magnetic, `formatHeadingBox`;
   - `GS %d` (and Mach above 0.40) at (16, 388), `formatVs` at (rightX() − 96, 388) and the altimeter
     setting (`formatAltimeterSetting`, the inHg / hPa setting) at (rightX() − 96, 420): the
     cockpit's own spots, so nothing moves when the view changes.
   Declutter as in the cockpit: GS and Mach follow `speedAltReadouts`, the heading box follows
   `headingTape`; speed, altitude, VS and the altimeter setting always show. No tapes, ticks,
   bands, bugs, trend arrow or pointers; the ladder, horizon, FPV, bank scale, RA and nav box stay
   hidden (cockpit eye point).
2. `Hud.cpp`: factor the value-box drawing out of `drawSpeedTape`, `drawAltTape` and
   `drawHeadingTape` (fill 0.8 / 0.7 black, 1.5 px stroke, the same text sizes) with a flag for the
   pointer, and draw the chase readout from `buildChaseReadout` before the status rows. The tapes
   keep calling the same helper (the cockpit screenshots must not change).
3. FMA: in the chase view call the overlay too; `NavHud::draw` draws only the FMA row there, and
   only when `ap_.apEngaged || ap_.fdOn` and `visibility.fma`.
4. Tests (`tests/hud/hudChaseTest.cpp`): every readout box stays clear of the status panel
   (y ≥ 430), the warning lines (y 264–300, the widest 39-character L warning), the three message
   lines (y 336–378, 31 M characters), the FMA row (y 0–22) and the SHOW FPS block (16, 27, 140, 26),
   at 640 × 480 and 853 × 480; the values (IAS rounding, indicated altitude with a baro offset,
   magnetic heading with a magnetic variation, VS, `STD` / `29.92` / `1013`); MIN declutter
   hides GS and the heading box; the FMA is drawn only with the AP or FD on; the chase batch stays
   under 12k vertices.
5. Xvfb screenshots in the chase view: on the runway (c172_runway_xtst), climbing in a bank
   (ap_hdg90_c172, AP on, so the FMA shows) and on final (c172_final_kpdx with the pack, or
   c172_final_xtst). Update HUD.md §7a and the §3 zone table.

**E. Release 1.2.1.** `project(... VERSION 1.2.1)` in `flightsim/CMakeLists.txt`; CHANGELOG `## 1.2.1`;
ROADMAP (1.2.1 row); WORK_PACKAGES X3 status; this card's status; DEVICE.md §6 (the GPU clock line,
the SHOW FPS line format, the 1.2.1 timing run). `release.yml` attaches datapack run 42155295255,
whose artifact expires 2026-12-06: if v1.2.1 will be tagged after that, run the datapack workflow on
main first and update `DATA_RUN_ID` / `DATA_ARTIFACT` in this pull request.

## Acceptance

- [ ] HUD screenshots before and after A: 0 differing pixels on every image of the set above
- [ ] HUD CPU per frame (fake-GL benchmark, `loc_gs` cockpit) at most half of the planner's
      209 µs on the same machine; state both numbers and the callgrind instruction counts in the PR
- [ ] Batch tests: identical vertex data for a clipped stroke on the fast and the per-triangle
      path; the cockpit batch under 40k vertices; one draw call per batch
- [ ] `GpuClock` tests pass; on desktop nothing is logged and nothing fails
- [ ] Sky after terrain: 0 differing pixels on the terrain screenshots (LOW, which has no noise,
      shows it cleanly)
- [ ] Texture noise: MED terrain screenshots side by side with 1.2.0's MED in the PR; HIGH
      unchanged (0 differing pixels)
- [ ] Chase readout: layout and value tests pass; three chase screenshots in the PR
- [ ] Full `ctest` passes, clang-format clean, no per-frame allocation (the arrays keep their
      capacity), no file IO per frame (the GPU clock is read once a second)

## Device checks

- [ ] The DEVICE.md §6 timing run at MED, then LOW and HIGH: cockpit, chase (orbit with L2 +
      right stick), under an OVC layer and inside it. Send the launch log: the `perf session`
      tables now carry `gpu clock:` lines and the SHOW FPS lines end in `gpu <cur>/<max> MHz`.
- [ ] HUD pass in the cockpit at MED well under the 8.0 ms of 1.2.0 (target ≤ 3 ms)
- [ ] MED cockpit fps above 30 (target 60) at KPDX, in cruise and on the KOTH final
- [ ] MED terrain looks like 1.2.0 (detail noise on the near ground; compare with HIGH, which keeps
      the 1.2.0 shader)
- [ ] Chase view: speed, altitude, VS, heading and (with the AP or FD on) the FMA are legible on the
      4" screen in daylight and at night, and the aircraft stays the focus
