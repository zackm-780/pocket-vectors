# tools/datapack (WP P4)

One command that builds the whole data pack (DATA.md section 3): terrain for a region (levels 7-10,
heights + land class), the world coarse layer (levels 0-6, heights), and the nav database with FAA
NASR and CIFP (real approaches), then validates both and writes `ATTRIBUTION.txt` and `README.txt`.
The output is never committed (`flightsim/data/` is git-ignored).

```bash
flightsim/tools/datapack/build_datapack.sh --out /path/to/pack            # pdx_oth + world + nav
flightsim/tools/datapack/build_datapack.sh --out flightsim/data --skip-world --jobs 4
# quick test: small box, two airports (this is how the script is tested)
flightsim/tools/datapack/build_datapack.sh --out /tmp/pack --bbox -124.4,43.3,-124.1,43.55 \
    --min-level 9 --max-level 10 --skip-world --cifp-airports KOTH,KPDX
```

Options: `--out DIR`, `--jobs N`, `--skip-world`, `--preset NAME`, `--bbox W,S,E,N`, `--min-level`,
`--max-level`, `--cifp-airports`, `--venv DIR`, `--terrain-cache DIR`, `--world-cache DIR`,
`--keep-world-cache`. It creates `tools/.datapack-venv` (git-ignored), caches downloads in `tools/terrain/cache` and `tools/nav/cache`, and is resumable (existing
tiles are skipped). At the end it prints pack size, file counts and the largest directory.

## GitHub Actions: workflow `datapack`

Actions tab, "datapack", "Run workflow". Inputs: `preset` (default `pdx_oth`), `include_world`
(default on). It uploads `flightsim-data-<preset>-<date>` (a zip with `terrain/`, `nav/`,
`ATTRIBUTION.txt`, `README.txt` at the top level), kept 30 days. Unzip it into `flightsim/data/`.

* Caches: terrain downloads (key by preset) and nav downloads (key by preset and 28-day cycle).
  The world layer's GLO-90 downloads (about 50 GB) are not cached (the repository cache limit is
  10 GB); they go to `/mnt/world-cache` and are deleted after use.
* Zip method: deflate (-6). On a coastal sample the terrain tiles shrank to 21 percent and the nav CSVs
  to about 25 percent; mountainous tiles compress less, but always some. Store would only save a minute.
* **Estimated runtime (not yet measured on a runner):** region `pdx_oth` 7-10 about 2-5 minutes of
  compute (104 s on 4 cores locally) plus about 1.5 GB of downloads; nav about 1 minute; world layer
  about 50 GB of S3 downloads (roughly 15-40 minutes at 20-60 MB/s) plus level 0-6 resampling, guess
  1-2 hours on 4 cores. Total about 1.5-3 h, under the 6 h limit (the job has a 345 min timeout). If
  the world layer runs out of disk or time, rerun with `include_world` off (about 10 minutes).
* Expected size: region 326 MB + nav about 29 MB; world heights up to 2.85 GB (DATA.md 4.5);
  zip about 1.5-2.5 GB. Without the world layer about 0.35 GB (zip about 0.15-0.25 GB).

## FAT32 / exFAT

Measured on the sample pack, and derived from the tile scheme for the full one: the deepest folders are
`terrain/h/{k}/{x}/` with at most one file per tile row (at most 128 at level 6, a few dozen at level
10), and `terrain/h/{k}/` has one folder per tile column (at most 256 at level 6, about 40 for the
region at 10). The biggest folder is `nav/procedures/` (one `.proc` per airport with approaches,
3,018 files measured from the real CIFP, 16 MB), far below the FAT32 limit of 65,534 entries per folder and below 16,000.
The whole pack is roughly 25-30 thousand files (about 21.5k land tiles in the world layer, 3.3k region
tiles, about 3,000 procedures). Every file is far below 4 GB. The build prints the real numbers and warns
above 16,000 entries in one folder.

## Licences

`ATTRIBUTION.txt` in the pack carries the credits: Copernicus DEM (the required "Produced using
Copernicus WorldDEM-30 / GLO-90 ..." notice), ESA WorldCover (CC BY 4.0, "(c) ESA WorldCover project
2021 / Contains modified Copernicus Sentinel data (2021) processed by ESA WorldCover consortium"),
OurAirports (public domain), FAA NASR and CIFP (US Government, public domain), NOAA WMM2025 (public
domain). Check the current Copernicus DEM licence text before redistributing the pack publicly; the
artifact is for personal use on your own device.
