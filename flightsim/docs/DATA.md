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
[regions]                ; informational, one line per build
world = levels 0-6
pdx_oth = bbox -124.7,43.0,-121.4,46.3 levels 7-10
```

### 4.2 Heights `.h16`
- **257 × 257** samples, **little-endian int16**, meters MSL, no header. File size exactly 132,098 bytes.
- **Row-major, north-up:** row `r` (0..256) is at lat `north − r·step`. Column `c` (0..256) is at
  lon `west + c·step`, where `step = span/256`.
- Samples are **grid-corner point samples**, and edges are **shared** with neighbours (column 256 of
  tile x equals column 0 of tile x+1). The pipeline must produce identical edge values.
- `-32768` = no data (runtime falls back to the parent tile for that tile). Valid range is −500..9000.
- Ocean = 0 (with land class Water).
- Resampling: bilinear from the source DEM at each tile's own spacing. Coarser levels are built from
  the source (or by 2× downsampling the finer level), never by point decimation.

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

### 4.5 Size guide (heights only, land tiles, plus about ⅓ for coarser levels)
| Coverage | Levels | ≈ Size |
|---|---|---|
| World coarse layer | 0–6 | ~1.5–2 GB |
| 500 × 500 km region | 7–10 | ~0.5 GB |
| Contiguous US | 7–9 | ~5 GB |

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
N4 defines this text format here before implementing the runtime loader. Requirements: per airport,
one block per approach with its transitions, final segment and missed approach. Legs map 1:1 onto
`fsim::Leg` (ARCHITECTURE §5) with fix ident and coordinates inline (self-contained, no lookups),
altitude and speed constraints, fly-over flag, RF arc data, and the source cycle in a header comment.
Sketch:
```
# FAA CIFP cycle 2610
APPROACH I28R ILS RWY 28R
TRANSITION <name or ALL>
LEG IF ARCHI 37.xxxxx -122.xxxxx ALT AT 4000
LEG TF CEPIN 37.xxxxx -122.xxxxx ALT ATORABOVE 3000 SPD 210
LEG CF RW28R 37.xxxxx -122.xxxxx CRS 297.0 ALT AT 63 FLYOVER
MISSED
LEG CA CRS 297.0 ALT ATORABOVE 520
…
END
```

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
python3 flightsim/tools/nav/build_navdb.py --out data/nav [--faa-nasr NASR.zip] [--cifp CIFP.zip]
```
Each tool caches downloads under its `cache/` dir, can run offline from the cache, prints a summary
(tiles written, size, time), and validates its output.
