# tools/nav: nav data pipeline (WP N1)

Builds the nav database of DATA.md sections 5-6 on a PC. Standard library only (Python >= 3.10);
`requirements.txt` documents that. Generated data and downloads are never committed (`cache/` is
git-ignored; write output outside git or to `flightsim/data/nav`).

```bash
cd flightsim/tools/nav
python3 build_navdb.py --out ../../data/nav --faa-nasr auto      # OurAirports + WMM2025 + current FAA cycle
python3 build_navdb.py --out ../../data/nav                      # OurAirports + WMM2025 only
python3 build_navdb.py --out ../../data/nav --offline --faa-nasr auto   # cache only, no network
python3 build_navdb.py --out ../../data/nav --faa-nasr NASR_CSV.zip     # a zip/dir you downloaded yourself
python3 validate_navdb.py ../../data/nav --spot-checks --wmm-selftest   # schema + KSFO/KPDX/KOTH checks
python3 -m unittest -v                                           # offline unit tests (fixtures)
```

The build validates its own output at the end (disable with `--no-validate`) and prints counts,
file sizes and timing. Output is deterministic (only `nav.ini` `created_utc` changes).

## Files

| File | Purpose |
|---|---|
| `build_navdb.py` | CLI: downloads (cached), orchestrates, writes `airports/runways/navaids/ils/fixes/airways.csv`, `nav.ini`, `magvar.bin` |
| `ourairports.py` | OurAirports rows -> airports, runways (one row per landing direction), navaids |
| `nasr.py` | FAA NASR CSV -> ils, fixes, airways; cycle/URL helpers |
| `wmm.py` | WMM2025 in pure Python, `magvar.bin` writer/reader, NOAA test-value self-test |
| `navgeo.py`, `schema.py` | geodesy helpers; column lists (single source of truth) |
| `validate_navdb.py` | schema/consistency validation + spot checks |
| `wmm/` | vendored NOAA files: `WMM2025.COF`, `WMM2025_TEST_VALUES.txt`, README (public domain) |
| `tests/` | `unittest` suite with tiny fixtures in `tests/fixtures/` |

## Verified sources (2026-10-19)

| Data | URL | Notes |
|---|---|---|
| OurAirports | `https://davidmegginson.github.io/ourairports-data/{airports,runways,navaids}.csv` | 200 OK. Re-downloaded when the cached copy is older than 7 days |
| WMM2025 coefficients | `https://www.ncei.noaa.gov/sites/default/files/2025-01/WMM2025COF.zip` (contains `WMM.COF`, `WMM2025_TEST_VALUES.txt`) | The old `ngdc.noaa.gov/geomag/WMM/data/WMM2025/...` path is 404. Test values: `https://www.ncei.noaa.gov/sites/default/files/2025-02/WMM2025_TEST_VALUES.txt`. Vendored in `wmm/`; epoch 2025.0, valid to 2030.0 |
| FAA NASR landing page | `https://www.faa.gov/air_traffic/flight_info/aeronav/aero_data/NASR_Subscription/` | Links the current and next cycle as `.../NASR_Subscription/YYYY-MM-DD` |
| FAA NASR CSV per subject | `https://nfdc.faa.gov/webContent/28DaySub/extra/DD_Mon_YYYY_{ILS,FIX,AWY}_CSV.zip` | e.g. `01_Oct_2026_ILS_CSV.zip` (0.7 MB), `..._FIX_CSV.zip` (3.6 MB), `..._AWY_CSV.zip` (1.0 MB). Cycle 2026-10-01 downloaded and parsed (real data) |

`--faa-nasr auto` reads the landing page, picks the newest cycle whose effective date is <= today
(falls back to 28-day arithmetic from 2026-10-01 if the page is unavailable, and to the previous
cycle if the newest is not downloadable), downloads the three zips into `cache/`.
`--faa-nasr <path>` accepts a zip or directory, searched recursively including nested zips, for
`ILS_BASE.csv ILS_GS.csv ILS_DME.csv FIX_BASE.csv AWY_SEG_ALT.csv`. The other cycle bundles
(`DD_Mon_YYYY_CSV.zip`) also work. The FAA unzips of the `.txt` flavour are NOT supported.

## Semantics worth knowing

### runways.csv
* **Runway idents are zero-padded** to two digits (`01L`, not `1L`), matching FAA/CIFP. `rwy_ident`
  is unique per airport; where an airport reuses an ident, the hard-surface/longest runway wins.
* `start` = LE/HE coordinates as given (physical start of pavement in the landing direction),
  `thr` = start moved `displaced_ft` along the heading, `end` = far end. `heading_true_deg` is the
  great-circle **initial bearing of the rounded published start->end coordinates** (2 decimals).
* `length_ft` is the full pavement length (published, else computed). Landing distance available is
  `length_ft - displaced_ft`. `thr_elev_ft` is the elevation of that runway end (else the opposite
  end, else the airport), `end_elev_ft` that of the far end.
* `approx = 1` when the geometry was synthesized: one end known -> other end from published heading
  + length; no coordinates -> centered on the airport reference point with published heading +
  length. **Extension:** when no coordinates AND no heading exist (about 18 000 small strips) the
  heading is `ident*10 + WMM declination at the airport` (error up to about +/-5 deg, still
  `approx = 1`). `--no-ident-heading` skips such runways instead (as DATA.md literally says).
* OurAirports quirks handled: ends further than 40 km from the airport or closer than 30 m to each
  other are discarded (then synthesized); ends listed swapped relative to the idents (bearing ~180
  deg off the ident) are swapped back; one-ended entries (helipads, `N`/`S`) are skipped.
* `surface`: first recognised token wins (`ASPH-TURF` -> asphalt); `PEM`/`PAVED`/`HARD` -> asphalt;
  metal mats, unknown one-letter codes (Australia) -> other. `hard` = asphalt or concrete.

### navaids.csv
`VOR-DME -> VORDME`, `NDB-DME -> NDBDME`. `magvar_deg` is OurAirports' published value, else WMM at
the station. `range_nm` is nominal from OurAirports `power`: VOR/TACAN/DME HIGH 130 / MEDIUM 40 /
LOW 25 (default 40); NDB HIGH 75 / MEDIUM 50 / LOW 25 (default 25). `freq_khz` empty if <= 0.
Note that OurAirports calls the San Francisco station `VOR-DME` (not VORTAC); OAK, BTG and OTH
are VORTACs.

### ils.csv (NASR `ILS_BASE`, `ILS_GS`, `ILS_DME`)
* Operational components only (`OPERATIONAL IFR/RESTRICTED`). FAA `ARPT_ID` is mapped to the
  OurAirports ident through `local_code`/`gps_code` (`SFO -> KSFO`, `ANC -> PANC`).
* `loc_course_true_deg` = published magnetic `APCH_BEAR` + published `MAG_VAR` (E +).
* **`loc_width_deg` and `tch_ft` are left empty**: the ILS files do not contain them. The runtime
  defaults (5 deg, 50 ft, ARCHITECTURE section 5) should apply.
* `has_gs` from `ILS_GS` (types `GS`, `GD`); `category` is the NASR value (`I`, `II`, `III`,
  `IIIB`, empty for localizer-only). All systems (ILS, LDA, SDF, localizer) are included.
* `airports.csv has_ils` = 1 if the airport has any ils.csv row, 0 for other US-territory
  airports, empty (unknown) elsewhere and when no NASR data was used.
* OurAirports and NASR occasionally disagree on runway numbering (about 45 ILS rows cannot be
  joined to a runway row; reported as a validation warning).

### fixes.csv / airways.csv
`fixes.csv` = `FIX_BASE` (`region` = ICAO region code such as `K2`). `airways.csv` = one row per
point in route order from `AWY_SEG_ALT`; coordinates come from fixes or, for VOR/NDB points, from
OurAirports navaids (same-name ambiguity: region code, then proximity to the previous point).
`mea_ft` = `MIN_ENROUTE_ALT` of the leg leaving that point (blank on the last point). If the same
designator exists in two regions (e.g. `V1` contiguous and Hawaii) the second chain's `seq` starts
at 1001 so chains stay separable. About 220 points (US-border pseudo points, VORs absent from
OurAirports) cannot be resolved and are skipped.

### magvar.bin
Exactly DATA section 6: `MAGVAR01`, `<f` epoch, six `<h` (-90, 90, -180, 180, 1, 0), then
181 x 361 little-endian int16 centidegrees, east positive, rows from -90 upward, columns from -180
(column 180 repeats -180). Height 0 km; poles are evaluated at +/-89.99 deg. Epoch = build date.

## Verification
`python3 validate_navdb.py <dir> --wmm-selftest` compares 12 official NOAA WMM2025 test vectors
(declination, inclination, X/Y/Z) -- max declination error 0.005 deg (limit 0.05 deg). The
validator checks headers, enumerations, ranges, reciprocal runways, heading == start->end bearing,
threshold == displaced distance, `hard` vs surface, magvar.bin layout, and (with `--spot-checks`)
KSFO, KPDX and KOTH runways/headings/ILS records looked up in the data.
