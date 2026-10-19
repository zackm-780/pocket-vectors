# Nav database runtime (WP N2)

`CsvNavDatabase` (`NavDb.h`) implements `core/NavDatabase` over the CSV nav pack
(DATA.md section 5) and `magvar.bin` (section 6). `MagVar` (`MagVar.h`) is the declination lookup,
`NavDbIls.h` holds the synthesized-ILS geometry.

## Usage
```cpp
CsvNavDatabase nav;                       // XTST only until load() succeeds
std::string err;
nav.load(paths.data("nav"), err);         // airports, runways, ils, navaids, fixes, magvar
const Airport* koth = nav.findAirport("KOTH");
const Runway* r05 = findRunway(*koth, "05");   // idents are zero-padded ("05", "10L")
```
`App` uses it when `<data root>/nav/nav.ini` exists and falls back to `SyntheticNavDb` otherwise
(or when `load()` fails). XTST is always present. After `load()` the object is read-only, every
query is const and thread-safe, and returned pointers stay valid for its lifetime.

## Behaviour
- **Ident lookup** is case-insensitive. `findNavaids` / `findFixes` return every match
  (ambiguous idents, e.g. a DME and an NDB both called `2K`), in file order.
- **nearestAirports**: 1 degree grid, exact great-circle filter, sorted by distance then ident.
  `maxDistNm = 0` returns only an airport at the exact position.
- **airportsInBox**: inclusive edges, sorted by ident, `west > east` means the box crosses the
  antimeridian.
- **searchAirports** ranking: exact ident, exact IATA, ident prefix, IATA prefix, municipality
  prefix, name prefix, any word prefix in name or municipality. Ties: large before medium before
  small, then by ident.
- **Real ILS** rows attach by airport + runway ident. Empty `loc_width_deg` is derived from the
  geometry (below), empty `tch_ft` is 50, empty `gs_angle_deg` is 3, empty `gs_elev_ft` is the
  threshold elevation. `has_gs = 0` gives a localizer-only `Ils` (`hasGs = false`).
- **Synthesized ILS** (`synthesized = true`, ident `I<runway>`, `freqMhz = 0`): for every runway
  that is hard, not `approx` and >= 3,000 ft and has no real ILS row. LOC antenna 1,000 ft beyond
  the far end on the extended centerline, course = runway heading, half-width such that full scale
  is +-350 ft at the threshold clamped to 1.5..3.0 degrees (`locWidthDeg` is the total width, so
  3.0..6.0), GS antenna `TCH / tan(3 deg)` (954 ft) past the threshold on the centerline, GS
  elevation = threshold elevation, TCH 50 ft, angle 3 degrees. `approx` runways never get one.
- **MagVar**: bilinear on the int16 centidegree grid, longitude wraps for a global grid, latitude
  clamps. Without `magvar.bin` declination reads 0 and a warning is logged. Navaids without a
  `magvar_deg` value take the grid value.
- **Airways** (`airway("V23")`) are loaded lazily on first call, ordered by `seq`. Do not connect
  points across a `seq / 1000` boundary (`AirwayPoint::segment()`).

## Performance (desktop, 4 cores, release-with-debuginfo, warm file cache)
Full pack (49,374 airports, 67,820 runway rows, 11,008 navaids, 70,129 fixes, 1,509 real ILS,
20,341 synthesized ILS, 19k airway points not loaded): **about 330-350 ms** to load everything
except airways, single thread, on the calling thread. Whole-process peak RSS of a headless run is
about 59 MB including JSBSim. Because that is an order of magnitude under the 3 s target, nothing
is lazy or threaded except airways; revisit with the device measurement (the A53 is roughly
4-6x slower than the desktop, so expect 1.5-2 s) and move fixes to a background thread only if
that is too slow.

## Tests and fixtures
`tests/nav/navdb*_test.cpp` use `tests/fixtures/nav/` (real rows for 12 airports, their ILS, 5
navaids, a few fixes, 2 airways; plus one synthetic duplicate fix `AARON` and a synthetic
`magvar.bin` whose value is `lat/10 + lon/30` degrees). Regenerate with
`tests/fixtures/nav/make_fixtures.py <full nav dir>`.
Manual check against the real pack (not run in CI):
`FSIM_NAV_DATA_DIR=<data>/nav ./flightsim_tests --test-case="nav: full*" -s`.
