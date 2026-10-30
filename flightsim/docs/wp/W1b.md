Status: ready
# W1b: Runways flush with the terrain at KPDX

Phase 2 · Effort: high · Depends: none · Branch: `wp/W1b-runway-terrain-step`

## Why

An open F3 follow-up: a step remains between the DEM terrain and the runway surface at the KPDX
pads. Real DEM heights near a runway differ from the published runway elevations, and the
flatten areas blend them, but not enough. It shows from the air and is felt on the takeoff roll.

## Read

- `docs/ARCHITECTURE.md` §5, the `TerrainQuery` contract and flatten areas
  (`grep -n -i 'flatten' docs/ARCHITECTURE.md`)
- `src/world/terrain/FlattenSet.h` and the blend code it points to; `tests/world/terrain/FlattenTest.cpp`
- How runways register flatten areas: `grep -n -i 'flatten' src/world/AirportRenderer.cpp src/world/AirportGeometry.*`
- `docs/DATA.md` §5.3 (`runways.csv`: threshold elevations per end)
- `tests/world/terrain/SyntheticPack.h` (synthetic packs are built in memory by the tests)

## Owns

`src/world/terrain/`, `tests/world/terrain/`.

**Also touches:** `src/world/AirportGeometry.*` (where the runway surface height comes from), only
if the fix needs it.

## Do

1. Reproduce on desktop with the real pack. Download `pocket-vectors-1.0.0-data-pdx_oth.zip` from
   the v1.0.0 release into a folder outside the repo and pass `--data-dir`. Log the terrain height
   and the runway surface height every 10 m along both KPDX centerlines (28L/10R, 28R/10L) and
   on the pads. Record the worst step in the PR.
2. Fix it so the runway surface follows a straight grade between its two threshold elevations,
   and the flatten area blends to the DEM over at least 150 m beyond the runway edges and ends.
   Pads and the area around the thresholds get the same treatment.
3. Extend `SyntheticPack` with a deliberate 4 m DEM step under a runway end, and test against it.

## Acceptance

- [ ] Synthetic test: the surface along the centerline is within 0.3 m of the graded line, and
      the blend has no slope steeper than 1:20
- [ ] Real data (desktop, not in CI): the worst step at KPDX before and after, in the PR
- [ ] `c172_runway_kpdx.ini` takeoff trace shows no vertical-acceleration spike on the roll
      (excerpt in the PR)
- [ ] Screenshots of the 28L threshold before and after (two images)

## Device checks

- [ ] Takeoff and landing at KPDX roll smoothly, and from a 3 NM final no step is visible
