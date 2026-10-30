Status: ready
# X3a: Frame timing breakdown and a bench run

Phase 1 · Effort: high · Depends: none · Branch: `wp/X3a-frame-timing`

## Why

The only number on the device is frames per second, once a second (`App.cpp`, the `fpsFrames`
block). The performance pass (X3b, X3c) needs to know where a 33 ms frame goes. On the Mali-G31,
draw calls cost more than triangles: the same 40k triangles in 150 draws cost 5.7 ms instead of
2.5 ms (DEVICE.md §2). W6 added up to 4 overlay draws.

## Read

- `docs/ARCHITECTURE.md` §3 "Frame loop and threading" (`grep -n '^## 3' docs/ARCHITECTURE.md`)
- `src/app/App.cpp`: `grep -n 'fpsFrames\|renderer.render\|airports.render\|worldOverlay\|hud.draw\|window.swap\|sim.step\|update(' src/app/App.cpp`
  and read a window around the frame loop
- `src/world/TerrainRenderer.h` (`struct` with `drawCalls`, `triangles`, `summary()`)
- `src/app/Cli.cpp` (flag parsing pattern) and `docs/BUILD_AND_DEPLOY.md` §3
- `src/gfx/GlApi.h`: does every draw go through one `GlApi` object? (`grep -n 'DrawArrays\|DrawElements'`)

## Owns

New `src/app/FrameStats.{h,cpp}`, new `tests/app/frameStatsTest.cpp`, `src/app/Cli.{h,cpp}` (new
flags), new `assets/scenarios/bench_xtst_final.ini`, `docs/BUILD_AND_DEPLOY.md` §3 (one line per
flag).

**Also touches:** `src/app/App.cpp` (timing scopes and the log line), `src/gfx/GlApi.*` (draw
counter, only if every draw already passes through it).

## Do

1. `FrameStats`: a fixed ring buffer of frame times (no allocation per frame) with avg, p50, p95,
   p99, max, and the share of frames over 33.3 ms and over 50 ms.
2. CPU time per phase with `SDL_GetPerformanceCounter`: input and menu, sim steps, terrain update,
   terrain render, airports and night lights, world overlays, HUD and menu 2D, swap. Under vsync,
   swap is where GPU wait shows up. All of it is off unless `show_fps` or a bench is on.
3. Draw calls and triangles per frame. Use the terrain stats that already exist. Count the other
   draws in `GlApi` if every draw passes through it, otherwise per renderer. No new globals.
4. With `show_fps` on, log one line every 5 s, for example:
   `perf: 31.2 fps | avg 32.0 p95 41.0 max 66 ms | >33ms 18% | sim 0.4 terr 6.1 rend 12.3 apt 1.2 ovl 0.3 hud 2.0 swap 8.0 | draws 142 tris 46k`.
   Keep the existing once-a-second `fps` line.
5. `--bench <seconds>`: run the given `--scenario` windowed for that long and print one summary
   line at the end, then exit 0. Add `bench_xtst_final.ini` (c172p, XTST, 6 NM final, AP on)
   for CI-safe comparisons.
6. Optional `--gpu-sync`: `glFinish` after each pass so the per-phase times include GPU work.
   Debug only; say in the help text that it distorts the total.

## Acceptance

- [ ] Unit tests: percentiles on known data, ring-buffer wrap, over-budget share
- [ ] `xvfb-run … --scenario assets/scenarios/bench_xtst_final.ini --bench 20` prints the summary
      line (paste it in the PR)
- [ ] Off by default: no measurable cost when `show_fps` is off (compare `--bench` total with the
      timing scopes compiled out or skipped, and state the difference)
- [ ] No per-frame allocations in the new code

## Device checks

- [ ] With SHOW FPS on, `conf/log.txt` gets a `perf:` line every 5 s on the KPDX runway and in
      cruise
