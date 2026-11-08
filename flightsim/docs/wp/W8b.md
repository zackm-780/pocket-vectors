Status: ready
# W8b: Smooth ground under taxiways and aprons

Phase 4 · Effort: high · Depends: W8a (`Airport::taxiways`, `Airport::aprons`), W1b (#15) · Branch: `wp/W8b-taxiway-ground`

## Why

Taxiways need ground as smooth as a runway's, and it must meet the runway without a step. W1b gave
flatten areas priorities and composes them into one continuous surface (`world/terrain/FlattenSet`).
Taxiways reuse that: every straight taxiway piece becomes a graded area below the runways, and
every apron becomes a flat area. KPDX would then hold about 1,400 areas instead of 3, and
`FlattenSet::elevationM` tests every area on each call (per wheel per sim step, and per terrain
vertex when a mesh is baked). A grid index has to come first.

Sizes measured from OSM on 2026-11-08: KPDX has 553 taxiway segments and 2,205 straight pieces
(41.7 km). Douglas–Peucker at 5 m leaves 781 pieces. Its 15 aprons have 528 ring vertices. KOTH has
36 segments and 43 pieces at 5 m.

**Heights come from the runways, not the DEM** (TAXIWAYS.md §3, revised with this plan).
Flatten areas are built from the `Airport` alone, by the terrain service and by the airport
renderer (its own `FlattenSet` in `buildAirport`). The DEM at that moment depends on which tiles
are loaded, and W1b made it a rule that loading tiles never moves the pavement. The runway
elevations are within a few metres of each other at KPDX and KOTH, so the field between them is
nearly flat.

## Read

- `src/world/terrain/FlattenSet.h` and `FlattenSet.cpp` (whole, 110 and 118 lines)
- `src/core/TerrainQuery.h` `struct FlattenArea` (`grep -n 'struct FlattenArea' -A16 src/core/TerrainQuery.h`)
- `tests/world/terrain/FlattenTest.cpp`: `grep -n 'TEST_CASE' tests/world/terrain/FlattenTest.cpp`,
  then read the test "crossing runways at different elevations" (line 251 on)
- `src/world/AirportGeometry.cpp`: `grep -n 'double runwayGradeElevM\|FlattenArea runwayFlattenArea\|airportFlattenAreas(const\|void registerAirportFlatten' src/world/AirportGeometry.cpp`
  and read those functions; `src/world/AirportGeometry.h` `struct RunwayInfo` and `makeRunwayInfo`
- `src/world/AirportRenderer.cpp`: `grep -n 'surfaceSet\|clearFlattenAreas' src/world/AirportRenderer.cpp`
- `src/world/TerrainRendererLod.cpp`: `grep -n 'void TerrainSelector::updateFlattenState' -A35 src/world/TerrainRendererLod.cpp`
- `tests/world/terrain/RealPackBench.cpp` lines 1–60 (a `doctest::skip()` bench) and
  `tests/world/terrain/RunwaySurfaceRealDataTest.cpp` lines 1–60 (the `FSIM_REAL_DATA_DIR` pattern)
- `src/core/NavTypes.h` `struct Taxiway`, `struct Apron` (W8a)

## Owns

`src/world/terrain/`, `tests/world/terrain/`, new `src/world/TaxiwayGeometry.{h,cpp}`, new
`tests/world/taxiwayFlattenTest.cpp`.

**Also touches:** `src/world/AirportGeometry.cpp` (`airportFlattenAreas` appends the taxiway
areas), `src/world/TerrainRendererLod.cpp` (`updateFlattenState` merges boxes).

## Do

1. **Lazy index in `FlattenSet`.** `add()` only appends and marks the set dirty. Today it
   re-sorts `order_` on every add, which is O(n² log n) for 1,400 areas. A private
   `ensureIndex() const` (mutable members; the set is main-thread only) sorts `order_` with the
   same comparator and builds the grid. It runs at the top of `elevationM` and of any new query.
   `clear()` and `revision()` behave as before.
2. **Per-area reach.** Areas with `priority < 0` cap the pavement blend at
   `kMaxLowPaveBlendM = 120` m instead of `kMaxPaveBlendM = 600`. Store the cap per area and use
   it both in `add()` (the reach box) and in `elevationM` (`wantM`). Runways (priority 0) keep 600
   m, so every W1b number stays the same. Without this, every taxiway area reaches 600 m and a
   grid cell would list most of the field.
3. **Grid.** Fixed cells of about 250 m (for example 0.0025° of latitude by 0.0035° of longitude).
   Each cell lists every area whose reach box overlaps it, in composition order. Store the lists
   flat (CSR: a `std::vector<uint32_t>` plus an `std::unordered_map<uint64_t, {offset, count}>`
   keyed by cell). An area whose box crosses ±180° goes in both edge ranges. `elevationM` walks
   only the point's list (no list: `natural()`). The lists keep the global order, so the result
   equals the old loop. Add `size_t candidateCount(lat, lon) const` (the list length) and a test
   switch `setUseGridForTest(bool)`. `distanceM` may compare squared distances and take one square
   root, with the same result to float precision.
4. **Taxiway areas** (new `world/TaxiwayGeometry.{h,cpp}`, GL-free; W8c adds the meshes there):
   - `double taxiwayFieldElevM(...)`, the height of any point. Take a `FlattenSet` of the
     airport's runway areas (`runwayFlattenArea` for each primary) and evaluate it at the point,
     with this natural function: the mean of each runway's `runwayGradeElevM` at the point,
     weighted by `1 / (d² + 200²)`, where `d` is the distance in metres from the point to that
     runway's grade segment (an airport without runways uses `elevFt`). Inside a runway's area
     this is that runway's surface. Elsewhere it blends to the weighted field, which always stays
     within the airport's runway elevations.
   - `std::vector<FlattenArea> taxiwayFlattenAreas(const Airport&)`, empty without taxiways and
     aprons. For taxiways, simplify each segment with Douglas–Peucker at 5 m. Each straight piece
     (skip pieces shorter than 1 m) becomes a rectangle around it: the half-width plus 40 m to each
     side, extended the same amount past both ends. It is graded from `taxiwayFieldElevM` at the
     two ends, with `blendM = 60` and `priority = -1`. The 40 m margin keeps the terrain vertices
     next to the pavement (about 30 m apart at level 10) on the flat core, and covers the 5 m
     simplification.
   - Aprons: simplify the ring with Douglas–Peucker at 3 m and ear-clip it into triangles. Put the
     ear clipping in this file, because W8c needs it too. Register each triangle as a quad whose
     4th corner repeats the 3rd, flat at the mean of `taxiwayFieldElevM` over the ring, with
     `blendM = 60` and `priority = -2`. Also register one rectangle per ring edge (30 m to each
     side and past each end) at the same height, which gives a flat margin around concave aprons.
     If ear clipping fails (a self-intersecting ring), log it and skip that apron.
   - `airportFlattenAreas()` appends `taxiwayFlattenAreas(airport)`. Airports without OSM data
     return exactly what they return today.
5. **Terrain renderer.** `TerrainSelector::updateFlattenState` builds one box per area and then
   tests every terrain record against every box, so taxiways would multiply its cost by about 500.
   Merge each area's box into the running box while they overlap or lie within 500 m (areas arrive
   grouped by airport), then test records against the merged boxes. A few more nodes rebuild near
   an airport, which is fine.
6. **Tests.**
   - `FlattenTest.cpp`: the grid equals the old loop (`setUseGridForTest(false)`) to 1e-4 m at
     10,000 random points over the W1b sets and over the KPDX-sized set below. On that set, the
     mean `candidateCount` over a 50 m lattice across the field is under 5 % of `size()`.
   - A KPDX-sized synthetic set, built in code: 3 runways (two parallel ones of 3,000 m and a
     2,000 m one crossing them), about 800 taxiway pieces in about 550 segments (parallel
     taxiways, exits every 300 m including 30° high-speed exits, and a dense taxilane lattice at a
     terminal), and 15 aprons with about 300 ring vertices in all, some of them concave.
   - `taxiwayFlattenTest.cpp`: a small airport (two crossing runways at different graded
     elevations, as in the W1b crossing test, a parallel taxiway with a 90° and a 30° exit, a
     junction, a concave apron) over a natural surface with a 4 m step under the taxiway. Sampled
     every 1 m along every taxiway centerline from the apron onto the runway centerline, the
     surface changes by at most 0.01 m per metre (1:100, so no step either). The runway centerline
     profiles match the runway-only surface to 1 mm. The apron interior, away from taxiways, stays
     within 0.01 m of its height. An airport without taxiways gets the same flatten areas as before.
   - A bench (`doctest::skip()`): 10⁶ lookups over the KPDX-sized set, printing µs per lookup and
     the time to build `airportFlattenAreas` for it.
   - Real data (`FSIM_REAL_DATA_DIR`, desktop only, skipped without it): KPDX from the v1.2.0 pack
     plus the W8a tool's `taxiways.csv` and `aprons.csv`. Print the area count, the build time, and
     the worst slope and worst 1 m step along every taxiway centerline.

## Acceptance

- [ ] Every W1b test passes unchanged (`ctest -R terrain` and `ctest -R world`)
- [ ] The grid equals the brute-force loop (test), and the candidate share is under 5 % (test)
- [ ] No step where a taxiway meets a runway, and no slope steeper than 1:100 on taxiway or runway
      pavement (synthetic test). Paste the KPDX real-data worst slope and step in the PR
- [ ] Under 2 µs per lookup on the KPDX-sized set (bench, desktop RelWithDebInfo; paste the number
      in the PR, with the brute-force time for comparison)
- [ ] KPDX: the area count and the `airportFlattenAreas` build time in the PR
- [ ] Airports without OSM data register exactly the areas they did before (test)

## Device checks

- [ ] KPDX start on 28L with the W8a pack: the spawn takes no longer than with the 1.2 pack, and
      the takeoff roll is as smooth as in 1.2
- [ ] Flying across KPDX at 1,500 ft with SHOW FPS on, the `perf:` max frame time shows no new
      hitch when the active airport set changes
