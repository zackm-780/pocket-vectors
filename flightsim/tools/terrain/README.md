# Terrain data pipeline (W3)

Builds the terrain part of the data pack (`<out>/terrain/`, format in
[`docs/DATA.md` section 4](../../docs/DATA.md)) from Copernicus DEM and ESA WorldCover. Runs offline
on a PC; the device only reads the output. Output is never committed.

```bash
cd flightsim/tools/terrain
python3 -m venv .venv && .venv/bin/pip install -r requirements.txt     # numpy, rasterio (bundles GDAL), requests

# 1. what will it cost?  (downloads only two small tile lists)
.venv/bin/python build_terrain.py --preset pdx_oth --min-level 7 --max-level 10 --out ../../data --dry-run
# 2. build the primary region (PLAN D19): heights + land class, levels 7-10
.venv/bin/python build_terrain.py --preset pdx_oth --min-level 7 --max-level 10 --out ../../data
# 3. world coarse layer, levels 0-6 from GLO-90 (heights only; see "World layer")
.venv/bin/python build_terrain.py --world --max-level 6 --out ../../data
# 4. check it
.venv/bin/python validate_pack.py --pack ../../data
.venv/bin/python preview_hillshade.py --pack ../../data --level 10 --bbox -121.85,45.25,-121.55,45.50 --out hood.png
# unit tests (synthetic data, no network)
.venv/bin/python -m unittest discover -s tests
```

Files: `build_terrain.py` (CLI), `validate_pack.py`, `preview_hillshade.py`, `fsterrain/` (library:
`tilegrid` tile scheme and bitmaps, `sources` downloads, `dem` resampling, `landcover`, `pack`
ini/bitmaps/presets, `build` planner and workers, `packread` runtime-style reader), `tests/`.

## CLI

`build_terrain.py (--world | --bbox W,S,E,N | --preset NAME) [--min-level A] [--max-level B] --out DIR`

| Option | Meaning |
|---|---|
| `--preset NAME` | `pdx_oth` (primary, PLAN D19: -124.7,43.0,-121.4,46.3), `bayarea` (-123.2,36.9,-121.2,38.4), plus small test boxes `mthood`, `tam_diablo`. `--list-presets` prints them. |
| `--bbox W,S,E,N` | any box in degrees. `W > E` crosses the antimeridian. |
| `--world` | world coarse layer. Defaults: levels 0-6, GLO-90, heights only. Alone it means the whole globe. Together with `--bbox`/`--preset` it keeps those defaults but builds only tiles touching the box (cheap test of the world path). |
| `--min-level`, `--max-level` | defaults 7 and 10 for a region, 0 and 6 for `--world` |
| `--out DIR` | data pack root; files go to `DIR/terrain/` |
| `--dry-run` | print tile counts, size upper bound and download estimate per level, then exit |
| `--force` | rebuild existing tiles. Without it the build is **resumable/idempotent**: existing tiles are skipped and the bitmaps and `pack.ini` are merged with the existing pack |
| `--jobs N` | worker processes (default: CPU count) |
| `--landcover` / `--no-landcover` | land class tiles on/off (default on for regions, off for `--world`) |
| `--source auto\|glo30\|glo90`, `--glo90-max-level 6` | auto = GLO-90 for levels <= 6, GLO-30 above; each falls back to the other product for cells that one lacks |
| `--offline` | never touch the network (fails if a needed file is not cached) |
| `--cache-dir` | default `tools/terrain/cache/` (git-ignored) |
| `--keep-ocean-tiles` | also write tiles that are entirely sea level (default: dropped) |
| `--reindex` | rebuild `index/L*.bit` from the `.h16` files on disk |

`--bbox -123.2,...` works with a leading minus sign (the script works around argparse).

## How it works

* **Tile math** (`tilegrid.py`) is exactly DATA 4.1. Sample positions are computed as
  `-180 + (x*256 + c) * (90/256/2^k)` from integer indices; all values are dyadic rationals, so they are
  exact in float64 and identical whichever tile (or process) computes them. The east edge of the last
  column of tiles wraps to -180, so edges also match across the antimeridian.
* **Heights.** A tile corner sample is the **box average (width = the level's sample spacing) of the
  bilinearly interpolated source surface**, centred on the sample. That is "bilinear at the tile's own
  spacing" with an area prefilter: coarse levels are area averages of the source, never point
  decimation, and at level 10 on GLO-30 (spacing 1.23 source pixels) it is almost plain bilinear.
  The weights are closed-form integrals of the bilinear kernel, accumulated in a fixed order, so a
  sample depends only on its position and the source pixels around it. **Neighbouring tiles therefore
  produce bit-identical shared edges**, and a rebuild gives byte-identical files regardless of
  `--jobs`, tile order or cache state (checked, see below).
* **Georeferencing is read from the files.** Each 1 degree Copernicus cell has its own pixel counts
  (GLO-30: 3600 rows, 3600 columns below 50 N, 2400/1800/1200/720/360 columns further north;
  GLO-90 a third of that) and is a *point lattice* (`AREA_OR_POINT=Point`: pixel centres lie on integer
  degrees; GDAL reports bounds half a pixel outside the cell). Cells do **not** overlap and do not share
  boundary pixels: cell `N37 W123` holds lon in `[-123, -122)` and lat in `(37, 38]`. Border taps are
  taken from the neighbouring cell's pixels (resampled if its grid differs), so there are no seams and
  no assumption about pixel widths. Cells absent from the bucket's `tileList.txt` are open ocean = 0 m.
* **Levels.** Every level is computed directly from the source (GLO-90 for levels <= 6, GLO-30 above),
  never from the finer level, so there is no error build-up. For speed, each cell is first block-averaged
  by an integer divisor of its pixel counts so that the reduced pixel is <= 1/4 of the spacing.
* **Void fill.** Voids (NaN or values outside -1000..12000 m) are filled with GDAL's inverse-distance fill;
  a cell with no valid pixel becomes 0. The Copernicus tiles checked here contain no voids.
* **Ocean = 0.** Copernicus ocean pixels are already 0 and unpublished ocean cells are treated as 0.
  Final values are rounded half up and clamped to -500..9000. A tile whose samples are all 0 and whose
  land class is all Water/Unknown is **not written** (the runtime falls back to the parent).
* **Land class** (`landcover.py`). At the finest level the 10 m WorldCover pixels are counted per
  256 x 256 cell (a pixel belongs to the cell containing its centre); cell = Water if >= 50% water,
  otherwise the mode (ties -> lowest class id). WorldCover "no data" counts as Water where the cell's
  mean corner height is <= 0, else Unknown. Coarser levels are derived from the four child tiles with the
  same rule (a child that was dropped as pure ocean counts as Water, a child outside the build as
  Unknown). A land-class file that would be all Unknown is not written.
* **Output merge.** `index/L{k}.bit` is read, extended and rewritten; `pack.ini` keeps the union of the
  levels and all `[regions]` lines. Writes are atomic (temp file + rename).

### World layer

`--world` is implemented for the whole globe (26,475 GLO-90 cells, about 50 GB of downloads, levels 0-6
of roughly 21.5k land tiles, <= 2.8 GB). It was **not run in full here**; it was tested with
`--world --bbox ... --min-level 2 --max-level 6` (that builds the whole GLO-90 footprint of the
coarse tiles that touch the box, e.g. level 2 needs about 110 cells). Land class for the world layer is
off by default: WorldCover at 10 m is about 2,650 tiles / 150+ GB. `--world --landcover` works but
reads all of it.

Coarse tiles in a *region* build (`--min-level` below 6) need GLO-90 for the whole tile footprint
(level 0 is 90 x 90 degrees), so `--dry-run` first.

## Verified source layouts (checked 2026-10-19)

Bucket listings were checked with the S3 REST API (`GET https://<bucket>.s3.amazonaws.com/?list-type=2&prefix=...`).
All three buckets are anonymous and reachable over HTTPS (also through the agent proxy).

| Data | URL pattern | Notes |
|---|---|---|
| GLO-30 DEM | `https://copernicus-dem-30m.s3.amazonaws.com/Copernicus_DSM_COG_10_{N\|S}{lat:02d}_00_{E\|W}{lon:03d}_00_DEM/Copernicus_DSM_COG_10_{N\|S}{lat:02d}_00_{E\|W}{lon:03d}_00_DEM.tif` | lat/lon = SW corner of the 1 degree cell, e.g. `N37_00_W123_00`. float32, deflate COG, 20-50 MB per land cell. The folder also holds `AUXFILES/` (EDM, FLM, HEM, WBM masks), `PREVIEW/` and the metadata XML. |
| GLO-30 availability | `https://copernicus-dem-30m.s3.amazonaws.com/tileList.txt` | one folder name per published cell (26,450 lines). A missing cell = open ocean (HTTP 404). |
| GLO-90 DEM | `https://copernicus-dem-90m.s3.amazonaws.com/Copernicus_DSM_COG_30_{N\|S}{lat:02d}_00_{E\|W}{lon:03d}_00_DEM/<same>.tif`, list at `.../tileList.txt` (26,475 lines) | 1200 x 1200 px per cell (fewer columns at high latitude), about 2.4 MB per land cell. GLO-90 has cells that GLO-30 lacks (the fallback). |
| WorldCover 2021 v200 | `https://esa-worldcover.s3.amazonaws.com/v200/2021/map/ESA_WorldCover_10m_2021_v200_{N\|S}{lat:02d}{E\|W}{lon:03d}_Map.tif` | 3 x 3 degree tiles named by SW corner (lat/lon multiples of 3), 36000 x 36000 uint8 COG, 4-115 MB each, 0 = no data. Index: list prefix `v200/2021/map/` (2,650 tiles), cached as `cache/lists/worldcover_tiles.txt`. |

Measured raster facts: GLO-30 N37 W123: 3600 x 3600, pixel 1/3600 deg, `AREA_OR_POINT=Point`, bounds
`-123.000139 .. -122.000139` (pixel 0 centre exactly -123.0). N61 W150: 1800 x 3600. GLO-90 N37 W123: 1200 x 1200;
N61 W150: 600 x 1200. WorldCover: `AREA_OR_POINT=Area`, bounds exactly on the 3 degree grid.

Cache layout: `cache/glo30/`, `cache/glo90/`, `cache/worldcover/` (original file names) and `cache/lists/`.
Downloads are streamed to `*.part` and renamed, with retries; a rerun reuses the cache and
`--offline` runs entirely from it.

### Fallbacks
If WorldCover is unreachable, run with `--no-landcover` (heights-only pack; the renderer then colours by
elevation and slope). If a GLO-30 cell is missing but GLO-90 has it, GLO-90 is used automatically.
The AWS Open Data copies are the only sources used; there is no second mirror.

## Attribution (written to `pack.ini`)
* `dem`: "Produced using Copernicus WorldDEM-30 and WorldDEM-90 (c) DLR e.V. 2010-2014 and (c) Airbus
  Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European Union and ESA; all rights
  reserved" (with the real (c) sign, UTF-8). Note the `;` inside the value: INI readers must treat `;`
  and `#` as comments only at the start of a line.
* `landcover` = "ESA WorldCover 2021 v200, CC BY 4.0" and `landcover_credit` = the ESA copyright line
  ("(c) ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA
  WorldCover consortium"), written when land class tiles are present.

## Validation

`validate_pack.py --pack DIR` checks, and exits non-zero if any fails:
`files` (pack.ini keys, sizes 132,098 / 65,536 B, value ranges, orphan land class),
`index` (bitmaps <-> files both ways), `edges` (all shared east/north edges bit-identical, wrap included),
`parents` (child vs parent at the shared points; see below), `spots` (Mt Diablo 1173 m, Mt Tamalpais East
Peak 784 m, Mt Hood 3429 m, only those the pack covers; add more with `--spot "Name:lat:lon:m"`; prints the
raw DEM maximum within 100 m for context) and `source` (random samples re-computed independently, by a
brute-force box average over a dense sub-grid of the raw GeoTIFFs read as a global point lattice, half of
them drawn near 1 degree cell borders). The `source` check reads GLO-30 as a 3600 x 3600 lattice, so it
is meaningful below 50 N; reduced-width cells (checked by hand on N50 W101/W100, 2400 x 3600, across the
lat-50 resolution change: max difference 0.49 m, edges and parents pass) are handled by the pipeline itself.

`parents`: a parent sample averages a box twice as wide as its children's, so the raw difference at shared
points measures roughness, not error. The check compares the parent with the `[1 2 1]x[1 2 1]/16`
smoothing of the children and judges the mismatch relative to the local relief; defaults mean <= 0.10,
p99 <= 0.40 of the relief (measured about 0.03 and 0.15 at every level).

## Measured results (4 cores, this container)

Measured on this container (4 cores, downloads about 20 MB/s). "Written" tiles exclude the all-ocean
tiles that are dropped.

| Build | Levels | Tiles written (h + lc) | Pack size | Time | Downloaded |
|---|---|---|---|---|---|
| `mthood` (0.3 x 0.25 deg test box) | 7-10 | 32 + 32 | 6.3 MB | 6.7 s from cache (about 17 s with downloads) | 6 GLO-30 cells, 1 WorldCover tile |
| `tam_diablo` (0.8 x 0.25 deg) | 7-10 | 62 + 62 (3 ocean dropped) | 12.3 MB | 15 s | 4 GLO-30 cells |
| `bayarea` | 7-10 | 447 + 447 (155 ocean dropped) | 88.3 MB | 69 s incl. a few downloads | 11 GLO-30 cells, 2 WorldCover tiles |
| **`pdx_oth`** | 7-10 | 1,650 + 1,650 (324 ocean dropped) | **326.1 MB** | **104 s** incl. 19 new GLO-30 cells and 3 WorldCover tiles; a resume with everything present takes 31 s | 25 GLO-30 cells, 5 WorldCover tiles; cache about 2.0 GB total with the test regions |
| `--world --bbox tam_diablo --min-level 2 --max-level 6` | 2-6 | 8 | 1 MB | 38 s | 191 GLO-90 cells (about 0.4 GB) |

`--dry-run` estimates (upper bounds: they count tiles touching a published cell):

| Preset / mode | Levels | Tiles | Size bound | Downloads |
|---|---|---|---|---|
| `pdx_oth` | 7-10 | 1,974 (30 / 100 / 400 / 1,444) | 390 MB (actual 326 MB) | 25 GLO-30 cells, 4 WorldCover tiles |
| `bayarea` | 7-10 | 602 | 119 MB (actual 88 MB) | 11 GLO-30 cells, 2 WorldCover tiles |
| `--world` (heights only) | 0-6 | 21,560 (8 / 32 / 121 / 396 / 1,259 / 4,285 / 15,459) | 2.85 GB | 26,475 GLO-90 cells, about 50 GB |
| `--world --landcover` | 0-6 | 21,560 | 4.26 GB | plus 2,650 WorldCover tiles, about 150 GB |

Validation of the real builds (`validate_pack.py`): all checks pass on `pdx_oth`, `bayarea`, `mthood`, and the
mixed `tam_diablo` + world levels 2-6 pack. On `pdx_oth`: 3,154 shared edges (810,578 samples) all bit-identical;
parent check on 27 million sample pairs, smoothed mismatch about 0.03 of local relief (p99 0.15);
300 random samples against the independent brute-force computation: max difference 0.52 m (the int16 rounding).
Spot elevations (pack at level 10 vs published; the pack is a 38 m box average so summits read lower than the
true peak): Mount Diablo 1162 vs 1173 m, Mount Tamalpais East Peak 766 vs 784 m, Mount Hood 3390 vs 3429 m,
Mount Adams 3737 vs 3743 m, KPDX field 6 vs 9 m, KOTH field 2 vs 5 m.
