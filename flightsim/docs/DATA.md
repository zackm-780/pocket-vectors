# Data: sources, pipeline, formats

## 1. Principles
- All heavy processing happens **offline on a PC** with the Python tools in `tools/`. The device only
  reads prebuilt files.
- The output is a **data pack** directory copied to the SD card (default `<port>/data/`, see
  ARCHITECTURE §8). Storage is not a constraint, so prefer fast-to-read, uncompressed layouts.
- Generated data is **never committed**. Tests use synthetic fixtures.
- Every format has a version. Readers reject unknown major versions with a clear log message.
- **Datum (D16):** all elevations and altitudes are MSL. Geoid undulation is ignored consistently.

## 2. Sources and licenses

| Data | Source | License / terms | Used by |
|---|---|---|---|
| Elevation 30 m | Copernicus DEM **GLO-30**, AWS Open Data bucket `copernicus-dem-30m` (1°×1° Cloud-Optimized GeoTIFFs) | Free worldwide use, attribution required (see the Copernicus DEM license) | W3 |
| Elevation 90 m | Copernicus DEM **GLO-90**, bucket `copernicus-dem-90m` | Same as GLO-30 | W3 (world layer, GLO-30 gaps) |
| Land cover | **ESA WorldCover 10 m 2021 v200**, bucket `esa-worldcover` (3°×3° tiles) | CC BY 4.0 | W3 |
| Airports, runways, navaids | **OurAirports** CSVs: `airports.csv`, `runways.csv`, `navaids.csv` from `https://davidmegginson.github.io/ourairports-data/` | Public domain | N1 |
| ILS, fixes, airways (US) | **FAA NASR** 28-day subscription, CSV format | US Government, public domain | N1 |
| Instrument procedures (US) | **FAA CIFP**, ARINC 424 | US Government, public domain | N4 |
| Magnetic variation | **WMM2025** (NOAA NCEI) | Public domain | N1 |

URLs, bucket layouts and tile naming must be **verified by the implementing WP** before relying on
them. Record the verified details in the tool's `README.md`. If a source is unreachable from the
build environment, fall back to the alternative listed in the tool's README, or report it.

## 3. Data pack layout

```
data/
  terrain/
    pack.ini                     # §4.1
    index/L{k}.bit               # §4.4 tile presence bitmaps
    h/{k}/{x}/{y}.h16            # §4.2 heights
    c/{k}/{x}/{y}.lc8            # §4.3 land class (optional per tile)
  nav/
    nav.ini                      # §5.1
    airports.csv  runways.csv  navaids.csv
    ils.csv  fixes.csv  airways.csv          # US/FAA, optional
    magvar.bin                   # §6
    procedures/{ICAO}.proc       # §7 (N4)
  plans/                         # user flight plans, *.fms (NAVIGATION §2.1)
```

## 4. Terrain format v1

### 4.1 Tile scheme
A geographic quadtree. At **level `k`** (k ≥ 0):
- `spanDeg = 90 / 2^k`. Tiles are `spanDeg × spanDeg` in lat/lon.
- `x ∈ [0, 4·2^k)`, `y ∈ [0, 2·2^k)`. Tile `(k,x,y)` covers lon `[-180 + x·span, -180 + (x+1)·span]`
  and lat `[-90 + y·span, -90 + (y+1)·span]`. `y` grows **northward**.
- Children of `(k,x,y)` are `(k+1, 2x+i, 2y+j)` for i,j ∈ {0,1}.

| k | span | sample spacing | ≈ spacing at equator |
|---|---|---|---|
| 0 | 90° | 0.3516° | 39 km |
| 3 | 11.25° | 0.0439° | 4.9 km |
| 5 | 2.8125° | 0.01099° | 1.2 km |
| 6 | 1.40625° | 0.00549° | 611 m |
| 7 | 0.703° | 0.00275° | 305 m |
| 8 | 0.3516° | 0.00137° | 153 m |
| 9 | 0.1758° | 0.000687° | 76 m |
| 10 | 0.0879° | 0.000343° | 38 m |
| 11 | 0.0439° | 0.000172° | 19 m (only with sources finer than 30 m) |

`pack.ini`:
```ini
[pack]
format = flightsim-terrain
version = 1
samples = 257
min_level = 0
max_level = 10
created_utc = 2026-10-19T12:00:00Z
[attribution]
dem = <exact Copernicus DEM attribution string>
landcover = ESA WorldCover 2021 v200, CC BY 4.0
landcover_credit = © ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium
; informational, one line per build
[regions]
world = levels 0-6
pdx_oth = bbox -124.7,43.0,-121.4,46.3 levels 7-10
```
**INI rule (applies to all our INI files):** values may contain `;`, `#`, `©` and other UTF-8. A
comment is a line whose first non-blank character is `;` or `#`. There are no inline comments.
Values are kept verbatim, with surrounding whitespace trimmed.

### 4.2 Heights `.h16`
- **257 × 257** samples, **little-endian int16**, meters MSL, no header. File size exactly 132,098 bytes.
- **Row-major, north-up:** row `r` (0..256) is at lat `north − r·step`. Column `c` (0..256) is at
  lon `west + c·step`, where `step = span/256`.
- Samples sit at **grid corners**, and edges are **shared** with neighbours (column 256 of tile x
  equals column 0 of tile x+1). The pipeline produces bit-identical edge values.
- `-32768` = no data. W3 never writes it (voids are filled), but readers must handle it by falling
  back to the parent tile. Valid range is −500..9000.
- Ocean = 0 (with land class Water). **All-ocean tiles are omitted.** A missing tile means "ocean
  or not built": the runtime uses the finest ancestor present, or elevation 0 / Water if none exists.
- **Samples are area averages:** each corner sample is the box average (width = that level's
  spacing) of the bilinear source surface. Every level is built from the source (GLO-90 for
  k ≤ 6, GLO-30 above, each falling back to the other), never by point decimation. Parents
  therefore match their children only up to smoothing, which is expected.

### 4.3 Land class `.lc8`
- **256 × 256** cells, `uint8`, row-major north-up, **cell-centered**: cell `(r,c)` covers
  lat `[north−(r+1)·step, north−r·step]` and lon `[west+c·step, west+(c+1)·step]`. No header.
  File size 65,536 bytes.
- Values follow `fsim::LandClass` (ARCHITECTURE §5): 0 Unknown, 1 Water, 2 Tree, 3 Shrub, 4 Grass,
  5 Crop, 6 Urban, 7 Bare, 8 SnowIce, 9 Wetland, 10 Mangrove, 11 Moss.
- WorldCover mapping: 10→Tree, 20→Shrub, 30→Grass, 40→Crop, 50→Urban, 60→Bare, 70→SnowIce,
  80→Water, 90→Wetland, 95→Mangrove, 100→Moss. No data with height ≤ 0 → Water, otherwise Unknown.
- Downsampling uses the **mode**, except that a cell that is ≥ 50% water is Water.
- Optional: a missing `.lc8` means the renderer colours by elevation and slope.

### 4.4 Presence index `index/L{k}.bit`
- Bitmap of the tiles present at level k: bit `i = y·(4·2^k) + x`, LSB-first within each byte.
  Size = `ceil(8·4^k / 8)` bytes (1 MiB at k=10).
- The pipeline writes or updates it and **merges** with existing bits when a pack is extended with a
  new region.
- Runtime: the finest present level covering a point is found from the bitmaps. A missing bitmap
  means "no tiles at this level".

### 4.5 Size guide (measured by W3 where noted)
| Coverage | Levels | Size |
|---|---|---|
| `pdx_oth` (heights + land class) | 7–10 | **326 MB measured** (1,650 + 1,650 tiles, 104 s on 4 cores) |
| `bayarea` (heights + land class) | 7–10 | 88 MB measured |
| Pacific NW far-range layer, bbox −128,40,−114,50 | 5–6 | ~18 MB (1.2 GB of source downloads) |
| World, heights only (default for `--world`) | 0–6 | ≤ 2.85 GB (~50 GB GLO-90 download) |
| World with land class | 0–6 | ≤ 4.3 GB (plus ~150 GB WorldCover download) |

**Far-range coverage:** without the world layer, the terrain beyond a region renders as flat sea
level. A cheap fix is a coarse layer (levels 5–6) over a wider box around the region, as in the
Pacific NW row. Levels below 5 need very large GLO-90 footprints (a level-0 tile is 90° wide), so
always `--dry-run` first.

## 5. Nav database format v1 (CSV)

All files: UTF-8, header row, comma-separated, RFC 4180 quoting, `.` decimal separator, empty
field = unknown. Columns may be **appended** in later versions, and readers index by header name.

### 5.1 `nav.ini`
```ini
[nav]
format = flightsim-nav
version = 1
created_utc = …
[sources]
ourairports = <download date>
faa_nasr = <cycle effective date or empty>
faa_cifp = <cycle or empty>
wmm = WMM2025, epoch used <decimal year>
```

### 5.2 `airports.csv`
`ident,type,name,lat_deg,lon_deg,elev_ft,iso_country,iso_region,municipality,iata,has_ils`
- Excludes closed airports, heliports and balloonports. `type` ∈ `large,medium,small,seaplane`.

### 5.3 `runways.csv`: one row per **landing direction**
`airport_ident,rwy_ident,opposite_ident,thr_lat_deg,thr_lon_deg,thr_elev_ft,start_lat_deg,start_lon_deg,end_lat_deg,end_lon_deg,end_elev_ft,heading_true_deg,length_ft,width_ft,displaced_ft,surface,hard,lighted,approx`
- `start_*` = physical start of pavement. `thr_*` = landing threshold (start moved by
  `displaced_ft` along the heading). `end_*` = far end. Heading is computed from start→end coordinates.
- `surface` is normalized to `asphalt,concrete,grass,dirt,gravel,water,other`. `hard` = asphalt or concrete.
- Runway idents are **zero-padded** (`01L`, `09`), matching FAA/CIFP usage.
- `approx = 1` when end coordinates were missing and geometry was synthesized from the airport
  reference point, published heading and length. If no heading is published either, the heading
  is derived from the runway number (×10° + WMM declination, typically ±5° off). N1's
  `--no-ident-heading` turns this off. Skip the row if even that is impossible.
- **Runtime rule:** `approx` runways are drawn and usable for takeoff and landing, but get **no
  synthesized ILS, PAPI or approach lights**, because their geometry is only approximate.

### 5.4 `navaids.csv`
`ident,type,name,freq_khz,lat_deg,lon_deg,elev_ft,magvar_deg,range_nm,dme_lat_deg,dme_lon_deg,iso_country`
- `type` ∈ `VOR,VORDME,VORTAC,TACAN,NDB,NDBDME,DME`. `magvar_deg` + = east.

### 5.5 `ils.csv` (US from NASR; other regions are synthesized at runtime by N2)
`airport_ident,rwy_ident,loc_ident,freq_mhz,loc_lat_deg,loc_lon_deg,loc_course_true_deg,loc_width_deg,has_gs,gs_lat_deg,gs_lon_deg,gs_elev_ft,gs_angle_deg,tch_ft,dme_lat_deg,dme_lon_deg,category`
- NASR has no localizer width or TCH, so those fields are usually empty. The runtime then uses the
  `Ils` defaults (width from NAVIGATION §3.2, TCH 50 ft). Rows may be localizer-only (`has_gs = 0`).

### 5.6 `fixes.csv` (US)
`ident,region,lat_deg,lon_deg`

### 5.7 `airways.csv` (US, optional)
`airway,seq,fix_ident,fix_lat_deg,fix_lon_deg,mea_ft`
- When one designator has disjoint segments (for example V1 in the contiguous US and Hawaii), `seq`
  restarts at 1001, 2001, … for each further segment. Consumers must not connect points across a
  thousand boundary.

## 6. `magvar.bin`
- Header: 8 bytes ASCII `MAGVAR01`, then little-endian `float32 epochYear`, `int16 latMinDeg(-90)`,
  `int16 latMaxDeg(90)`, `int16 lonMinDeg(-180)`, `int16 lonMaxDeg(180)`, `int16 stepDeg(1)`, `int16 reserved`.
- Data: `int16` declination in **centidegrees** (+ = east), row-major from latMin to latMax, and
  lonMin to lonMax within a row. 181 × 361 values for a 1° grid.
- Runtime: bilinear interpolation (N2).

## 7. Procedures `procedures/{ICAO}.proc` (owned by N4)
Real instrument approaches of the US (FAA CIFP, ARINC 424-18), one file per airport that has any
(KOTH.proc, KPDX.proc, PANC.proc, ...; Alaska, Hawaii and the territories included: 3,018 airports,
10,232 approaches, 15 MB for cycle 2610). Built by `tools/nav/cifp.py` (also `build_navdb.py --cifp`),
read by `src/nav/Procedures.cpp` (`loadApproaches(navDir, icao)`), which produces `fsim::Approach`
objects (core/NavTypes.h). Every leg is **self-contained**: the fix name and its coordinates are
inline, no database lookups are needed at runtime. All courses are **true** degrees (the CIFP
magnetic courses are converted offline, see below), altitudes feet MSL, speeds knots, distances NM.

### 7.1 Source and cycle
- Download: `https://aeronav.faa.gov/Upload_313-d/cifp/CIFP_YYMMDD.zip` where `YYMMDD` is the cycle's
  effective date (`CIFP_261001.zip` = AIRAC 2610, effective 2026-10-01, 9.1 MB; it contains
  `FAACIFP18`, a 132-column ARINC 424-18 text file, plus the readme PDF). The links are listed on
  `https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/cifp/download/` (current and
  next cycle). `cifp.py --cifp auto` picks the newest cycle effective on or before today. US
  Government work, public domain. The cycle goes into `nav.ini` (`faa_cifp = 2610`) and into each
  file (`CYCLE 2610`). Keep it equal to the NASR cycle (`faa_nasr`).

### 7.2 File format `flightsim-proc` version 1
Plain UTF-8 text, one directive per line, tokens separated by whitespace, `#` starts a whole-line
comment, `.` decimal separator. Readers reject a different major version, ignore unknown
directives between approaches and unknown `KEY=value` options on LEG lines, and fail with a line
number on anything else malformed.

```
FORMAT flightsim-proc 1
# FAA CIFP cycle 2610 (effective 2026-10-01); generated by tools/nav/cifp.py; public domain
CYCLE 2610
AIRPORT KOTH
APPROACH I05-Z ILS 05 ILS Z RWY 05
TRANSITION RARES
LEG IF RARES 43.6602583 -124.0228583 IAF
LEG RF MITUE 43.3136111 -124.5063167 ALT ATORABOVE 2600 ARC 43.4155056 -124.1685222 15.963 L TURN L ORIG AF
FINAL
LEG IF MITUE 43.3136111 -124.5063167 ALT ATORABOVE 1600
LEG CF OXVAK 43.3830722 -124.3377528 CRS 61.0 ALT ATORABOVE 1300 FAF
LEG CF RW05 43.4147278 -124.2605972 CRS 61.0 ALT AT 67 FLYOVER VANG -3 WK RWY MAP
MISSED
LEG CA - CRS 60.7 ALT ATORABOVE 500
LEG CF YICBU 43.5113639 -124.2958028 CRS 316.0 ALT ATORABOVE 2600 FLYOVER
LEG HM YICBU 43.5113639 -124.2958028 CRS 136.0 ALT ATORABOVE 2600 TURN R HOLD T 1
END
APPROACH ...
```

| Line | Meaning |
|---|---|
| `FORMAT flightsim-proc 1` | Must be the first directive. |
| `CYCLE <yynn>` / `AIRPORT <ICAO>` | Source cycle and airport (the CIFP ident, same as `airports.csv`). |
| `APPROACH <id> <kind> <rwy or -> <title...>` | Starts an approach. `id` = CIFP ident without "RW": `I05-Z`, `R28L-Y`, `H05-Z`, `L13`, `VOR-B` (the `-Y`/`-Z` suffix is the FAA multiple-procedure indicator, "ILS Z RWY 5"). `kind` = `ILS LOC RNAV VOR NDB`. `rwy` = DB style (`05`, `28L`) or `-` for circling-only approaches. `title` = rest of the line, derived from the ident. |
| `LOS <name>...` | Optional. RNAV levels of service authorised at the FAF: `LPV LNAV/VNAV LNAV`. Informational. |
| `TRANSITION <name>` | Zero or more. An approach transition (IAF route); `name` = its CIFP transition ident (the IAF fix, or `OTH1`...). It runs up to or just before the first fix of FINAL. Vectors to final = no transition. |
| `FINAL` | Legs from the initial fix of the final segment **up to and including the MAP** (the runway threshold `RWxx` for runway-aligned approaches). |
| `MISSED` | Optional. Legs after the MAP (first leg flagged `M` in CIFP). All become `missedApproach = true`. |
| `END` | Closes the approach. |

`LEG <TYPE> <IDENT|-> [<LAT> <LON>] [options...]`: `TYPE` is one of `IF TF CF DF CA VA FA RF HM
HA HF` (= `fsim::LegType`). `IDENT` is `-` when the leg has no fix (CA, VA), and then no `LAT LON`
follow. Options, any order:

| Option | Meaning | `fsim::Leg` field |
|---|---|---|
| `CRS <deg>` | course, **true** (CF, FA, CA, VA; the **inbound** course of HM/HA/HF) | `courseTrueDeg` |
| `ALT <KIND> <ft> [<ft2>]` | `AT`, `ATORABOVE`, `ATORBELOW`, `BETWEEN <low> <high>` | `alt` (Between: `altFt` = low, `alt2Ft` = high) |
| `SPD <kt>` | speed limit | `speedLimitKt` |
| `FLYOVER` | fly-over fix (waypoint description code, 2nd character `Y`) | `flyOver` |
| `ARC <clat> <clon> <nm> <L\|R>` | RF arc centre, radius, turn direction | `arcCenterLatDeg/LonDeg`, `arcRadiusNm`, `arcRight` |
| `TURN <L\|R>` | hold turn direction (HM/HA/HF), DF turn | `arcRight` (true = right turns) |
| `WK <VOR\|NDB\|RWY\|APT\|SYN>` | waypoint kind when not an ordinary fix; `SYN` = computed by the converter | `to.kind` |
| `HOLD <T\|D> <value>` | hold leg length, minutes or NM | `ProcLegExtra::holdLegTimeS/holdLegDistNm` |
| `VANG <deg>` | published vertical angle (negative = descent, e.g. `-3`) | `ProcLegExtra::vertAngleDeg` |
| `IAF`, `FAF`, `MAP` | role flags (waypoint description code, 4th character A/C/D, F, M) | `ProcLegExtra`, `ApproachInfo::fafFinalLeg/mapFinalLeg` |
| `ORIG <ARINC type>` | the leg was approximated from this path terminator | `ProcLegExtra::approximatedFrom` |

`Leg` has no field for the hold leg length, the vertical angle or the FAF/MAP flags, so the loader
puts them into `ApproachInfo`/`ProcLegExtra` (parallel to the leg vectors); N5 should use
`ApproachInfo::fafFinalLeg` for the APR CDI scaling (2 NM before the FAF).

### 7.3 Conversion rules (`tools/nav/cifp.py`)
- **Records.** Approach records are section P, subsection F, grouped by approach ident, then
  route type (`A` = approach transition by transition ident; any other letter = the final approach
  route) and sequence number. Fix coordinates come from the terminal waypoint (PC), enroute
  waypoint (EA), VHF navaid (D), NDB (DB, PN), runway (PG) and airport (PA) records of the same
  file. Alaska is area code CAN and Hawaii/Guam PAC: every area is read. A fix that cannot be
  resolved drops its leg (1 of 122,173 records in cycle 2610: `CMY` at KDAF).
- **Missed approach** starts after the first leg whose description code marks the MAP (4th
  character `M`); fallback: the runway fix. No approach lacks a MAP in cycle 2610.
- **Courses.** CIFP courses are magnetic. The converter adds the declination FAA used for that leg:
  the station declination of the recommended navaid when it is a VHF navaid (or of the leg's own
  fix when it is a VOR), otherwise the airport's. Checked against 16,810 legs that carry theta/rho
  (fix position relative to a navaid): median error 0.08 degrees (waypoint-record variations were
  1.2 degrees off). Against the nav database, 2,523 final CF legs to a runway fix have a median
  course error of 0.27 degrees to the runway heading, and the runway fix lies a median 16 ft from
  `runways.csv`'s threshold. Note that CIFP uses legacy station declinations (UBG 21 E, OTH 15 E),
  so a hold at a fix that is not a VOR may differ by a few degrees from the radial geometry.
- **Altitudes.** Description codes: blank/`@` AT; `+` AT_OR_ABOVE; `-` AT_OR_BELOW; `B` BETWEEN
  (low, high); `G`, `I`, `X` AT; `H`, `J`, `V` AT_OR_ABOVE; `Y` AT_OR_BELOW, applied to altitude 1.
  The glide-slope intercept altitude (altitude 2 of `G H I J`) is not kept (the vertical angle is).
  `FLxxx` becomes feet. Runway fixes carry threshold elevation + TCH (e.g. 67 ft at KOTH 05).
- **Speed limits** are copied (displayed, never enforced). **Minimums** are not in CIFP: the loader
  leaves `minimumsFt = 0` and N5 falls back to its synthesized values (NAVIGATION 3.1).
- **Native path terminators:** IF TF CF DF CA VA FA RF HM HA HF.
- **Approximated** (`ORIG` / dropped, all counted in `procedures/coverage_report.txt`, logged by the
  converter):

| ARINC | Becomes | Cycle 2610 records |
|---|---|---|
| `AF` (DME arc) | `RF` about the recommended navaid, radius = distance navaid to fix | 1,150 |
| `PI` (procedure turn) | `TF` to the PT fix; dropped when the previous leg already ends there. **The course reversal is not flown.** | 1,037 (192 TF, 845 dropped) |
| `VI`, `CI` (heading/course to intercept) | dropped, the next CF leg intercepts (a following TF becomes DF) | 1,330 + 12 |
| `VM`, `FM` (vectors) | dropped | 2 |
| `FC` (track from fix for a distance) | `TF` fix + `TF` to the computed point (`WK SYN`) | 593 |
| `VD`, `CD`, `FD` (to a DME distance) | `CF` to the computed DME-distance point (`WK SYN`); dropped without a start | 13 + 5 |
| `VR`, `CR` (to a radial) | `CF` to the computed radial intersection (`WK SYN`) | 6 |

  Overall 96.6% of the 122,173 source records are native, 1.6% approximated and 1.8% dropped (mostly
  the VI/PI legs above); 22% of the approaches contain at least one approximated or dropped leg.
- **Not marked in CIFP:** helicopter-only procedures (KOTH `I05-Y` is the "COPTER ILS Y" chart),
  and circling-only approaches (no runway) are written with `rwy -`.

### 7.4 Runtime (`src/nav/Procedures.h`)
`loadApproaches(dataDir, icao)` (cached, thread-safe; accepts the nav directory or a data root),
`findApproach(approaches, id)` (exact id, or the short form `I05` for `I05-Y`), `approachesForRunway`,
`assembleApproachLegs(approach, transitionName)` (transition + final + missed, duplicate IF at the
join dropped, ready for `insertApproach`), and `loadProcedureSet()` for the extras. Parsing all
3,018 files takes about 120 ms.

## 8. Pipeline CLIs (to be implemented by W3/N1/N4)

```bash
python3 -m venv .venv && . .venv/bin/activate
pip install -r flightsim/tools/terrain/requirements.txt -r flightsim/tools/nav/requirements.txt
# world coarse layer (once)
python3 flightsim/tools/terrain/build_terrain.py --world --max-level 6 --out data/
# detailed region
python3 flightsim/tools/terrain/build_terrain.py --preset pdx_oth --min-level 7 --max-level 10 --out data/
python3 flightsim/tools/terrain/build_terrain.py --bbox -124.7,43.0,-121.4,46.3 --max-level 10 --out data/ --dry-run
# nav database (OurAirports + WMM; FAA optional)
python3 flightsim/tools/nav/build_navdb.py --out data/nav [--faa-nasr auto|NASR.zip] [--cifp auto|CIFP.zip]
# instrument procedures only, into an existing nav dir (or --airports KOTH,KPDX for a quick run)
python3 flightsim/tools/nav/cifp.py --cifp auto --out data/nav [--airports KOTH,KPDX]
```
Each tool caches downloads under its `cache/` dir, can run offline from the cache, prints a summary
(tiles written, size, time), and validates its output.
