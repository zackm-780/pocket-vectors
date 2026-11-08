Status: ready
# W8c: Draw taxiways and aprons

Phase 4 · Effort: high · Depends: W8a (data), W8b (the composed surface, `world/TaxiwayGeometry`) · Branch: `wp/W8c-taxiway-drawing`

## Why

After W8a and W8b the geometry is loaded and the ground under it is smooth, but nothing is drawn.
Airports still show a generic apron and a short taxiway stub (`buildAirportExtras`). This card
draws the OSM taxiways and aprons with their yellow markings, on the same surface the wheels feel,
and credits OpenStreetMap on the About page.

Budget (TAXIWAYS.md §4, DEVICE.md §2): draws cost more than triangles on the Mali-G31. KPDX has
553 segments and 2,205 straight pieces (41.7 km), with 15 aprons and 528 ring vertices. Aim for at
most 2 draws per airport (pavement, then markings), under about 25k triangles at KPDX, and nothing
drawn beyond 5 NM.

## Read

- `src/world/AirportRenderer.h` (whole, 120 lines) and `src/world/AirportRenderer.cpp`:
  `grep -n 'void AirportRenderer::buildAirport\|void AirportRenderer::render\|layer 1: apron\|layer 2: markings\|const AirportExtras extras' src/world/AirportRenderer.cpp`,
  then read `buildAirport` and the surfaces part of `render`
- `src/world/AirportGeometry.h`: `struct SurfaceVertex`, `struct SurfaceMesh`,
  `class SurfaceMeshBuilder`, `struct AirportExtras` (lines 210–281)
- `src/world/AirportGeometry.cpp`: the colour constants (lines 15–25), `SurfaceMeshBuilder::current`
  and `finish` (`grep -n 'SurfaceMeshBuilder::' src/world/AirportGeometry.cpp`) and
  `buildAirportExtras` (`grep -n '^AirportExtras buildAirportExtras' src/world/AirportGeometry.cpp`, about 95 lines)
- `src/world/TaxiwayGeometry.h` (W8b: `taxiwayFieldElevM`, the ear clipping, Douglas–Peucker)
- `src/ui/HelpPages.cpp` `helpAboutLines` (`grep -n 'helpAboutLines' src/ui/HelpPages.cpp`) and
  `tests/ui/HelpTest.cpp` around line 176; `src/hud/StrokeFont.h` lines 1–25 (`FSIM_SYM_COPYRIGHT`)
- `src/app/Scenario.h` `struct Scenario` and `src/app/Scenario.cpp`:
  `grep -n 'bool parseScenario\|if (sc.startKind == "runway")\|} else {  // cruise' src/app/Scenario.cpp`
- `tests/world/airportGeometryTest.cpp`: `grep -n 'TEST_CASE' tests/world/airportGeometryTest.cpp`
  (read "apron and buildings sit beside a hard runway")

## Owns

`src/world/TaxiwayGeometry.{h,cpp}` (the mesh part), new `tests/world/taxiwayMeshTest.cpp`, new
`assets/scenarios/c172_ramp_kpdx_chase.ini` and `assets/scenarios/c172_taxiways_koth.ini`.

**Also touches:** `src/world/AirportRenderer.{h,cpp}` (meshes, draws, range, stats),
`src/world/AirportGeometry.{h,cpp}` (`buildAirportExtras` skips the generic apron and stub where
OSM data exists), `src/ui/HelpPages.cpp` and `tests/ui/HelpTest.cpp` (the About line),
`src/app/Scenario.{h,cpp}` (an optional start position, additive), `docs/TAXIWAYS.md` §3–4
(measured numbers).

## Do

1. **Meshes** (`TaxiwayGeometry`, GL-free, RTC anchor at the airport reference point; 2D work in a
   local east/north frame in metres; heights from a `SurfaceElevFn`, the airport's composed surface):
   `TaxiwayMeshes buildTaxiwayMeshes(const Airport&, const std::vector<RunwayInfo>&, const SurfaceElevFn&)`
   returns `pavement` and `markings` (each a `std::vector<SurfaceMesh>`) plus `centerEcef` and
   `radiusM` for culling.
   - Ribbons: each segment simplified with Douglas–Peucker at 0.5 m, then one quad per piece,
     split along the piece only every 40 m at most (the surface along a piece is close to its
     straight grade). Use **mitre** joins at interior bends under 30° (the mitre at most 2× the
     half-width), and **round fillets** (a fan of 8 segments, radius the widest half-width there) at
     nodes shared by two or more segments and at bends of 30° or more.
   - Aprons: the W8b ear clipping, then each triangle subdivided until no edge is longer than 40 m,
     every vertex on the surface.
   - Taxiways and aprons share **one** pavement colour (`kApronRgb`; grass and dirt taxiways use a
     darker green-brown) and the up normal at the anchor. Where they overlap on a ramp, the same
     colour and normal hide any depth fight.
   - **Runways stay clean:** remove the part of every pavement piece and fillet that lies inside a
     hard runway's pavement rectangle. A quad minus a rectangle comes out as at most 4 convex
     pieces: clip successively against the rectangle's 4 edge half-planes, taking the outer part
     each time. Taxiway and runway pavement then never overlap, so both can stay in polygon-offset
     layer 1.
   - Markings, yellow (about `{0.85, 0.68, 0.12}`): a 0.40 m centerline per segment, and 0.45 m
     edge lines along both ribbon edges. Cut both at runway rectangles the same way. Drop the edge
     line wherever it lies inside another ribbon or an apron: sample it every 5 m, and use a small
     grid of segments for the test. Build the lines as strips that share 2 vertices per sample, so
     KPDX fits in one 16-bit mesh.
   - Lift: pavement 0.08 m, markings 0.10 m, like the generic apron today.
2. **Renderer** (`AirportRenderer`): in `buildAirport`, when the airport has taxiways or aprons,
   build the meshes with the same `surfaceSet` lambda the runways use, falling back to
   `taxiwayFieldElevM` outside every area. Upload them as `taxiPavement` and `taxiMarkings`. Draw
   the pavement in layer 1 and the markings in layer 2, only while the eye is within 5 NM
   (`nmToM(5)`) of the taxiway bounding circle (`distToEye(center) - radiusM`). Free them in
   `destroy`. Add `taxiwayTriangles` (drawn last frame) to `Stats`. There is no allocation in
   `render`. Log the mesh build time for each airport at INFO.
3. **Generic extras:** at an airport with OSM taxiways or aprons, `buildAirportExtras` adds no
   generic apron and no stub. It keeps a building only if no footprint corner lies inside an
   apron or within 30 m of a taxiway centerline. Other airports are unchanged.
4. **About page** (`helpAboutLines`): after `"JSBSIM  LGPL-2.1"`, always add
   `"Taxiways " FSIM_SYM_COPYRIGHT " OpenStreetMap contributors (ODbL)"`. Players who only copy
   the new `data/nav/` keep their old `ATTRIBUTION.txt`, so the line can't depend on it. Update
   `HelpTest.cpp`.
5. **Scenarios** (additive keys in `[scenario]`): `lat_deg`, `lon_deg` and `heading_true_deg`
   override the start position and heading for `start = runway` (on the ground) and
   `start = cruise`. Without them nothing changes. Add `c172_ramp_kpdx_chase.ini` (cruise at
   about 600 ft MSL over the KPDX terminal ramp, the centroid of the largest KPDX apron, heading
   about 300°) and `c172_taxiways_koth.ini` (on the ground at a KOTH apron, facing the taxiway).
6. **Tests** (`taxiwayMeshTest.cpp`, a synthetic airport like W8b's): the meshes are RTC and
   16-bit indexed. Pavement has no gap at a 90° junction or at a 45° bend: points sampled every 0.5
   m across the join are covered. No pavement or marking vertex lies inside a runway rectangle
   (beyond 0.05 m). Edge lines are absent inside an apron. Every vertex sits on the surface
   function plus its lift. KPDX-sized synthetic set: at most 1 pavement mesh and 1 marking mesh,
   with the triangle counts printed. An airport without OSM data gets no taxiway meshes and the
   same `buildAirportExtras` output as before. The draw range is under 5 NM.
7. **Screenshots** (desktop, not CI; the v1.2.0 pack plus the W8a tool's KPDX and KOTH files, as
   in W8a and W8b): `xvfb-run -a -s "-screen 0 640x480x24" ./build/flightsim --data-dir <data> --scenario <ini> --frames 120 --screenshot build/<name>.png`
   for `c172_threshold_kpdx.ini` (the 28L threshold), `c172_ramp_kpdx_chase.ini` with
   `external_view = true` in a scratch `--conf-dir`, and `c172_taxiways_koth.ini`. Look at all
   three.

## Acceptance

- [ ] `ctest` passes: the new mesh tests, the updated About test, and every existing airport test
- [ ] KPDX: triangle counts (pavement, markings), draw calls added (≤ 2) and the mesh build time
      in the PR; KOTH the same
- [ ] No allocation per frame in the new draw path; nothing drawn beyond 5 NM (test of the range check)
- [ ] Screenshots in the PR: the KPDX 28L threshold, the KPDX terminal ramp from above in the chase
      view, and KOTH. Taxiways meet runways at the edge with no overlap, and ramps have no gaps
- [ ] The generic apron and stub still appear at an airport without OSM taxiways (screenshot at
      XTST or a test)
- [ ] The About page shows "Taxiways © OpenStreetMap contributors (ODbL)"

## Device checks

- [ ] Taxi from the KPDX terminal ramp to 28L along the taxiways: the pavement is continuous, no
      step where a taxiway meets the runway, no flicker
- [ ] Frame rate at the KPDX ramp within 2 fps of v1.2.1 at the same spot (DEVICE.md §3 method)
- [ ] Taxiways and aprons show at KOTH, and the generic apron still shows at an airport without OSM
      taxiways
- [ ] The About page shows the OpenStreetMap line
