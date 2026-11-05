Status: blocked (X3a merged and T1 frame numbers)
# X3b: Quality presets (wire `graphics_quality`)

Phase 2 · Effort: high · Depends: X3a, T1 · Branch: `wp/X3b-quality-presets`

## Why

`graphics_quality = low | med | high` is parsed and saved (`src/app/Settings.cpp`, `kQuality`),
but nothing reads it. On a 1 GB handheld with a Mali-G31, a preset is the main lever between
smooth and pretty. The game asks for 4x MSAA and gets it on the device today (DEVICE.md §2).

## Read

- `docs/DEVICE.md` §2 GPU row and the T1 field note in §3 (frame numbers per case)
- `src/world/TerrainRenderer.h` lines 20–45 (`TerrainRenderConfig`, `LodConfig` in
  `TerrainRendererLod.h` lines 49–60)
- `src/platform/Window.h` lines 15–30 (`WindowConfig::preferMsaa`) and its use:
  `grep -n 'WindowConfig wc' -A12 src/app/App.cpp`
- `src/world/AirportRenderer.h` line 33 (`kMaxActiveAirports`), `src/world/NightLights.h` (public
  knobs), `src/world/WorldOverlayLogic.h` lines 38–45 (`selectOverlayAirports` `maxCount`)
- `src/app/Settings.cpp`: `grep -n 'graphics_quality\|kQuality' src/app/Settings.cpp`
- `src/ui/FlightScreens.cpp` lines 40–140 (`kSettingDefs`)

## Owns

New `src/app/Quality.{h,cpp}` (a pure function from preset to knobs), new
`tests/app/qualityTest.cpp`.

**Also touches:** `App.cpp` (apply the knobs at start and on a settings change),
`TerrainRenderer.*`, `AirportRenderer.*`, `NightLights.*` (accept a config value instead of a
constant), `FlightScreens.cpp` (a GRAPHICS row), `assets/settings_default.ini` (comment).

## Do

1. `QualityKnobs qualityKnobs(GraphicsQuality)`: terrain `pixelsPerCell`, `uploadsPerFrame`,
   maximum view distance, MSAA on or off, night-light speckle density, active airports, overlay
   airport cap. A starting point, to be tuned with `--bench`:

   | Knob | Low | Med | High |
   |---|---|---|---|
   | Terrain `pixelsPerCell` | 16 | 12 | 10 (today) |
   | Uploads per frame | 2 | 3 | 4 |
   | MSAA | off | off | 4x |
   | Night speckle | off | half | full |
   | Active airports | 3 | 4 | 6 |
   | Overlay airports | 6 | 10 | 12 |

2. Apply them live where possible. MSAA takes effect at the next launch; the menu row says
   `RESTART` next to the value when it changes.
3. The default stays `med` unless the T1 numbers show `med` misses 30 fps at the KPDX runway, in
   which case make `low` the default for the `portmaster` platform only.

## Acceptance

- [ ] Unit test for the knob table and for unknown values falling back to `med`
- [ ] `--bench 20` summary lines for low, med and high on `bench_xtst_final.ini`, pasted in the PR
- [ ] Screenshots of the same view at low and high, 640×480 (two images)
- [ ] Changing GRAPHICS in the pause menu applies without a restart, except MSAA

## Device checks

- [ ] `perf:` lines at the KPDX runway for each preset
- [ ] Low still reads well: runway edges, HUD strokes without MSAA
