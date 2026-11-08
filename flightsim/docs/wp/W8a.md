Status: ready
# W8a: Taxiways from OpenStreetMap: extraction, pack format, loader

Phase 4 · Effort: high · Depends: OD6 (answered: OpenStreetMap, PLAN D23) · Branch: `wp/W8a-osm-taxiways`

## Why

The owner wants taxiways (2026-11-08, ships in **1.3.0**). The packs have runways only. This card
gets the geometry into the pack and into `Airport`. W8b shapes the ground under it and W8c draws
it. Nothing is drawn yet after this card.

[../TAXIWAYS.md](../TAXIWAYS.md) §1–3 has the source, the ODbL terms and the format. Measured on
2026-11-08 from the OSM API: KPDX has 145 taxiway and 85 taxilane ways, which split into 553
segments at shared nodes, with 2,758 points and 15 aprons (528 ring vertices, all closed ways). KOTH
has 17 taxiway ways (36 segments), plus 2 `area=yes` taxiways that are dropped, and 7 aprons.
Neither field maps an apron as a multipolygon relation.

## Read

- [../TAXIWAYS.md](../TAXIWAYS.md) §1 "What OSM lacks", §2, §3 "Extraction" and "Format"
- `docs/DATA.md` §2 (the "planned, W8" row), §3, and §5 through §5.7 (`grep -n '^## 2\|^## 3\|^### 5' docs/DATA.md`)
- `tools/nav/README.md` (the Files table), `tools/nav/ourairports.py` `norm_surface`
  (`grep -n 'def norm_surface' tools/nav/ourairports.py`), `tools/nav/navgeo.py` (`distance_m`)
- `tools/nav/tests/test_navdb.py` lines 1–30 (test style: `unittest`, fixtures in `tests/fixtures/`)
- `tools/datapack/build_datapack.sh`: `grep -n 'usage()\|--cifp-airports) \|step "nav: validate"\|cat > "\$OUT/ATTRIBUTION.txt"\|^# --- summary' tools/datapack/build_datapack.sh`
- `../.github/workflows/datapack.yml` (whole file, 125 lines) and `../.github/workflows/ci.yml` (the `sdcard` job)
- `src/core/NavTypes.h` `struct Airport` and `src/core/Geo.h` `struct LatLon`
- `src/nav/NavDb.h` (whole) and `src/nav/NavDb.cpp`: `grep -n 'CsvNavDatabase::loadFixes\|CsvNavDatabase::load(\|LOG_INFO("nav: loaded' src/nav/NavDb.cpp`
  (read `loadFixes` as the pattern for an optional file); `src/core/Csv.h` (`CsvReader`)
- `tests/nav/navdb_test.cpp` lines 1–58 and 99–110 (fixture dir, the "no data pack" test)
- `docs/ARCHITECTURE.md` §5: `grep -n 'Later additive changes\|^struct Airport {' docs/ARCHITECTURE.md`

## Owns

New `tools/nav/osm_taxiways.py`, new `tools/nav/tests/test_osm_taxiways.py`, new
`tools/nav/tests/fixtures/osm/`, `tools/nav/README.md`, `tools/datapack/build_datapack.sh`,
`tools/datapack/README.md`, `../.github/workflows/datapack.yml`, `docs/DATA.md` §2, §3 and new
§5.8–5.10, `src/nav/NavDb.{h,cpp}`, new `tests/nav/navdb_taxiways_test.cpp`, new
`tests/fixtures/nav/taxiways.csv` and `tests/fixtures/nav/aprons.csv`.

**Also touches:** `src/core/NavTypes.h` (additive), `docs/ARCHITECTURE.md` §5 (one line per new
type), `../.github/workflows/ci.yml` (one step in the `sdcard` job that runs the new tool tests).

## Do

1. **`tools/nav/osm_taxiways.py`**, standard library only (Python ≥ 3.10, like the other nav tools):
   - CLI: `--nav-dir DIR` (required; reads `airports.csv` and `runways.csv` there and writes the
     outputs there), `--airports KPDX,KOTH` (subset), `--osm-file F.osm` (repeatable: parse saved
     OSM XML instead of querying, for tests and offline desktop data), `--cache-dir` (default
     `tools/nav/cache/osm/`, already git-ignored), `--offline` (cache only),
     `--overpass-url` (default `https://overpass-api.de/api/interpreter`), `--sleep 1.0`,
     `--margin-m 1500`, `--attribution FILE` (append the ODbL line to an existing
     `ATTRIBUTION.txt` unless it is there already).
   - Airports: every row of `airports.csv` with at least one `hard=1` runway. Box: the bounding box
     of its runways' `start_*` and `end_*` points grown by `--margin-m`.
   - Overpass, one POST per airport, `[out:xml][timeout:90]`, `way["aeroway"~"^(taxiway|taxilane|apron)$"](box); out body; >; out skel qt;`.
     User-Agent `pocket-vectors-datapack/1.3 (+https://github.com/zackm-780/pocket-vectors)`. At least
     `--sleep` seconds between requests. HTTP 429, 5xx and timeouts retry 4 times (5, 10, 20, 40 s).
     An airport that still fails is logged and skipped. Cache each answer as
     `<cache>/<ICAO>-<YYYYMMDD>.osm` and reuse it the same day; `--offline` takes the newest.
   - Parse OSM XML with `xml.etree.ElementTree`, in any element order (Overpass puts nodes after
     ways; the OSM API `map` call puts them first). One parser serves both.
   - Keep `aeroway=taxiway|taxilane` ways with ≥ 2 nodes, unless `area=yes`, or `disused`,
     `abandoned`, `construction` or `proposed` has a value other than `no`. Keep `aeroway=apron`
     closed ways with ≥ 3 distinct nodes and the same lifecycle filter. Skip relations, but count
     them in the log.
   - Each way goes to one airport: the one whose box holds the way's mean point; on a tie, the one
     with the nearest runway. Deduplicate by way id.
   - Split taxiway ways into **segments** at nodes that two or more kept taxiway or taxilane ways of
     the airport share, or that one way visits twice. Each segment becomes one row, and `osm_way_id`
     repeats across a way's segments.
   - Width: the `width` tag (`23`, `23 m`, `75 ft`, `75'`), kept when 3–80 m. Otherwise taxilanes
     get 7.5 m and taxiways 23 m at `large`, 15 m at `medium` and 10.5 m at other airports.
     Surface: the `surface` tag through `ourairports.norm_surface`, else the surface of the
     airport's longest runway.
   - Write `taxiways.csv` (`airport_ident,osm_way_id,kind,ref,width_m,surface,points`) and
     `aprons.csv` (`airport_ident,osm_way_id,surface,points`). Points are `lat lon` pairs with 6
     decimals, space separated. Aprons store the outer ring without repeating the first point. Rows
     are sorted by airport, way id, then segment order, and an unchanged input gives identical
     files. Always write both files, with a header even when empty.
   - Write `TAXIWAYS-ODbL.txt`: these two files contain data from OpenStreetMap, © OpenStreetMap
     contributors, extracted on <date>, available under the Open Database License 1.0
     (`https://opendatacommons.org/licenses/odbl/1-0/`, see `https://www.openstreetmap.org/copyright`).
     Add `osm_taxiways = <date>` under `[sources]` in `nav.ini` when the file exists.
   - Print one summary line per run, for example
     `osm: 188/190 airports answered, 2 failed; 1,234 taxiway rows, 210 aprons; taxiways.csv 412 KB, aprons.csv 61 KB; KPDX 553 rows 76 KB`.
     Exit 1 when it tried at least one airport and none answered. A partial answer still exits 0.
2. **Fixtures and tests** (`tools/nav/tests/test_osm_taxiways.py`, `unittest`, offline):
   - `fixtures/osm/koth_aeroway.osm`, under 100 KB: the KOTH aeroway ways and their nodes,
     filtered from one OSM API `map` call (`api.openstreetmap.org/api/0.6/map?bbox=-124.270,43.400,-124.225,43.432`),
     kept as OSM XML. Put the ODbL attribution in an XML comment and in `fixtures/osm/README.md`.
     Don't use the OSM API for anything bigger. The full extract runs in GitHub Actions.
   - `fixtures/osm/edge_cases.osm`, synthetic, in Overpass element order (ways before nodes): a
     `disused=yes` taxiway, an `area=yes` taxiway, a way with `width=75 ft`, two ways crossing at a
     shared node, a way that ends on another way's middle node, an unclosed apron, and a closed apron.
   - Tests: the filters; splitting at shared nodes (segment counts and endpoints); width and surface
     defaults; the CSV format (header, 6 decimals, open apron ring); deterministic output; the
     summary counts; KOTH from the fixture gives 17 taxiway ways and 7 aprons; `--attribution` does
     not repeat the line.
   - CI: add a step `python3 -m unittest -v tests.test_osm_taxiways` (working dir
     `flightsim/tools/nav`) to the `sdcard` job in `ci.yml`.
3. **Data pack** (`build_datapack.sh`): `--osm-taxiways` / `--no-osm-taxiways` (default on). After
   `step "nav: validate"`, add `step "nav: OSM taxiways"` that runs the tool on `$OUT/nav`. When the
   tool wrote rows, add an OpenStreetMap paragraph to the `ATTRIBUTION.txt` heredoc:
   `Taxiways and aprons (nav/taxiways.csv, nav/aprons.csv): © OpenStreetMap contributors, ODbL 1.0, see nav/TAXIWAYS-ODbL.txt.`
   Print the two file sizes in the summary. `datapack.yml`: a boolean input `osm_taxiways`
   (default true) that passes `--no-osm-taxiways` when false. The existing nav cache step already
   covers `tools/nav/cache/osm/`.
4. **Docs.** `DATA.md` §2: drop "planned" from the OSM row (tool `tools/nav/osm_taxiways.py`,
   W8a). §3: list the three files under `nav/`. New §5.8 `taxiways.csv` (one row per segment: an
   OSM way split where it meets another taxiway, columns and units), §5.9 `aprons.csv`, §5.10
   `TAXIWAYS-ODbL.txt`, each saying the files are optional and ODbL. `tools/nav/README.md`: a
   section with the commands, the Overpass endpoint, the fair-use throttle and the cache.
   `tools/datapack/README.md`: one line for the switch.
5. **Types** (`core/NavTypes.h`, additive; include `core/Geo.h` for `LatLon`):
   `enum class TaxiwayKind { Taxiway, Taxilane };`
   `struct Taxiway { long long osmWayId = 0; TaxiwayKind kind = TaxiwayKind::Taxiway; std::string ref; float widthM = 0; bool hard = true; std::vector<LatLon> points; };`
   `struct Apron { long long osmWayId = 0; bool hard = true; std::vector<LatLon> points; };`
   (open ring, ≥ 3 points), and in `Airport`: `std::vector<Taxiway> taxiways;` and
   `std::vector<Apron> aprons;`, both empty without OSM data. One line each in ARCHITECTURE §5.
6. **Loader** (`NavDb.{h,cpp}`): `loadTaxiways(path)` and `loadAprons(path)`, called from
   `load()` after `loadFixes`, both optional, in the `loadFixes` style. A missing file logs at INFO
   and nothing changes. Rows with an unknown airport, an odd or unparsable point list, or fewer
   than 2 points (taxiway) or 3 points (apron) count in `skippedRows`. `hard` means the surface is
   `asphalt` or `concrete`. An empty or bad `width_m` gets the airport-type default from step 1.
   Add `taxiways` and `aprons` counts to `NavDbStats` and the `nav: loaded …` log line. Parse with
   `strtod` over the field, without building one string per point.
7. **Loader tests** (`tests/nav/navdb_taxiways_test.cpp`): add `taxiways.csv` and `aprons.csv` to
   `tests/fixtures/nav` with a few synthetic rows at KOTH and KPDX (both are in the fixture). Check
   that kinds, refs, widths, `hard`, point order and the open ring load, that bad rows are skipped
   and counted, and that a pack without the files loads the same airports and runways with empty
   lists (copy `airports.csv` and `runways.csv` into a temp dir in the test).

## Acceptance

- [ ] `python3 -m unittest -v tests.test_osm_taxiways` passes offline, and CI runs it
- [ ] The tool on the KPDX and KOTH OSM API extracts (desktop, `--osm-file`, written into the
      v1.2.0 pack's `data/nav/` outside git) prints the summary. Paste it in the PR: KPDX about
      553 rows and 76 KB, KOTH about 36 rows
- [ ] `ctest -R nav` passes, including the new loader tests and the old-pack case
- [ ] `bash -n` and `shellcheck -S warning` pass on `build_datapack.sh`
- [ ] No generated data in git: only the two fixtures (< 100 KB together) under `tools/nav/tests/fixtures/osm/`
- [ ] `src/core/` changes are additive, with one ARCHITECTURE §5 line each

## Device checks

- [ ] With the new pack's `data/nav/` copied over an install, the game starts, `conf/log.txt` shows
      the `nav: loaded …` line with taxiway and apron counts, and the nav load time grows by less
      than 150 ms
- [ ] A pack without the taxiway files (the 1.2 pack) still starts and flies as before
