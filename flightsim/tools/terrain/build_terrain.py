#!/usr/bin/env python3
"""Build the flight sim terrain data pack (DATA.md section 4) from Copernicus DEM and ESA WorldCover.

Examples
  # what would a region cost? (no download, no output)
  build_terrain.py --preset pdx_oth --min-level 7 --max-level 10 --out data/ --dry-run
  # detailed region (heights + land class), levels 7-10
  build_terrain.py --preset pdx_oth --min-level 7 --max-level 10 --out data/
  # world coarse layer, levels 0-6 from GLO-90 (heights only unless --landcover)
  build_terrain.py --world --max-level 6 --out data/
  # the same pipeline restricted to a box, to test the world layer cheaply
  build_terrain.py --world --bbox -122.65,37.75,-121.85,38.0 --min-level 3 --max-level 6 --out data/
  # any box
  build_terrain.py --bbox -121.85,45.25,-121.55,45.50 --min-level 7 --max-level 10 --out data/

The tool is resumable: tiles that already exist are skipped (use --force to rebuild) and the
presence bitmaps and pack.ini are merged with whatever the pack already contains.
"""
from __future__ import annotations

import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from fsterrain.build import (BuildConfig, estimate, format_estimate, plan_levels,  # noqa: E402
                             run_build)
from fsterrain.pack import PRESETS, rebuild_bitmaps  # noqa: E402
from fsterrain.sources import Sources  # noqa: E402


def parse_bbox(text: str):
    try:
        w, s, e, n = (float(v) for v in text.split(","))
    except ValueError:
        raise argparse.ArgumentTypeError("--bbox must be west,south,east,north in degrees")
    if not (-180 <= w <= 180 and -180 <= e <= 180 and -90 <= s < n <= 90):
        raise argparse.ArgumentTypeError("--bbox out of range (west/east -180..180, south<north)")
    return (w, s, e, n)


def build_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_argument_group("region (choose one)")
    g.add_argument("--world", action="store_true",
                   help="world coarse layer: default levels 0-6 from GLO-90, heights only. Alone: the "
                        "whole globe; together with --bbox/--preset: only tiles touching that box")
    g.add_argument("--bbox", type=parse_bbox, metavar="W,S,E,N",
                   help="region box in degrees (west,south,east,north); west>east crosses 180")
    g.add_argument("--preset", choices=sorted(PRESETS), help="named region box")
    g.add_argument("--list-presets", action="store_true", help="print the presets and exit")
    ap.add_argument("--min-level", type=int, help="coarsest level (default 7 for a region, 0 for --world)")
    ap.add_argument("--max-level", type=int, help="finest level (default 10 for a region, 6 for --world)")
    ap.add_argument("--out", default="data", help="data pack root; files go to <out>/terrain (default: data)")
    ap.add_argument("--cache-dir", default=os.path.join(HERE, "cache"),
                    help="download cache (default: tools/terrain/cache, git-ignored)")
    ap.add_argument("--dry-run", action="store_true",
                    help="print tile counts, size and download estimates, then exit")
    ap.add_argument("--force", action="store_true", help="rebuild tiles that already exist")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 1,
                    help="worker processes (default: CPU count)")
    ap.add_argument("--offline", action="store_true",
                    help="never touch the network; fail if a needed file is not in the cache")
    lc = ap.add_mutually_exclusive_group()
    lc.add_argument("--landcover", dest="landcover", action="store_true", default=None,
                    help="build land class tiles (default for regions; needs the WorldCover 10 m tiles)")
    lc.add_argument("--no-landcover", dest="landcover", action="store_false",
                    help="heights only (default for --world)")
    ap.add_argument("--source", choices=["auto", "glo30", "glo90"], default="auto",
                    help="auto: GLO-90 up to --glo90-max-level, GLO-30 above, each falling back to the other")
    ap.add_argument("--glo90-max-level", type=int, default=6,
                    help="finest level built from GLO-90 when --source auto (default 6)")
    ap.add_argument("--keep-ocean-tiles", action="store_true",
                    help="also write tiles that are entirely sea level (default: drop them)")
    ap.add_argument("--cell-cache-mb", type=float, default=400.0,
                    help="DEM cell cache per worker process in MB (default 400)")
    ap.add_argument("--name", help="region name recorded in pack.ini (default: preset name or 'bbox')")
    ap.add_argument("--reindex", action="store_true",
                    help="rebuild index/L*.bit from the .h16 files on disk, then exit")
    ap.add_argument("--quiet", action="store_true", help="less output")
    return ap


def _fix_negative_bbox(argv: list[str]) -> list[str]:
    """argparse rejects `--bbox -123.2,36.9,...` (a value starting with '-'); accept it anyway."""
    out, i = [], 0
    while i < len(argv):
        if argv[i] == "--bbox" and i + 1 < len(argv):
            out.append("--bbox=" + argv[i + 1])
            i += 2
        else:
            out.append(argv[i])
            i += 1
    return out


def main(argv=None) -> int:
    ap = build_parser()
    a = ap.parse_args(_fix_negative_bbox(list(sys.argv[1:] if argv is None else argv)))
    if a.list_presets:
        for name, bb in PRESETS.items():
            print(f"{name:12s} bbox {bb[0]},{bb[1]},{bb[2]},{bb[3]}")
        return 0
    terrain_dir = os.path.join(a.out, "terrain")
    if a.reindex:
        print("rebuilt bitmaps (tiles per level):", rebuild_bitmaps(terrain_dir))
        return 0
    if a.bbox and a.preset:
        ap.error("use either --bbox or --preset, not both")
    if not (a.world or a.bbox or a.preset):
        ap.error("choose a region: --world, --bbox or --preset")
    if a.world:
        # --world alone is the whole globe; with --bbox/--preset it keeps the world-layer defaults
        # (levels 0-6, GLO-90, heights only) but only builds the tiles that touch the box (testing).
        bbox = PRESETS[a.preset] if a.preset else a.bbox
        name = a.name or ("world_bbox" if bbox else "world")
        min_level = 0 if a.min_level is None else a.min_level
        max_level = 6 if a.max_level is None else a.max_level
        landcover = bool(a.landcover)
    else:
        bbox = PRESETS[a.preset] if a.preset else a.bbox
        name = a.name or a.preset or "bbox"
        min_level = 7 if a.min_level is None else a.min_level
        max_level = 10 if a.max_level is None else a.max_level
        landcover = True if a.landcover is None else a.landcover
    if not (0 <= min_level <= max_level <= 14):
        ap.error("need 0 <= min-level <= max-level <= 14")
    cfg = BuildConfig(out=a.out, cache=a.cache_dir, min_level=min_level, max_level=max_level,
                      bbox=bbox, landcover=landcover, jobs=max(1, a.jobs), force=a.force,
                      offline=a.offline, glo90_max_level=a.glo90_max_level, source=a.source,
                      keep_ocean_tiles=a.keep_ocean_tiles, cell_cache_mb=a.cell_cache_mb,
                      region_name=name, verbose=not a.quiet)
    sources = Sources(cfg.cache, offline=cfg.offline, verbose=cfg.verbose)
    region_txt = "whole world" if bbox is None else "bbox " + ",".join(f"{v:g}" for v in bbox)
    print(f"region: {region_txt}  levels {min_level}-{max_level}  land class: "
          f"{'on' if landcover else 'off'}  out: {terrain_dir}")
    if bbox is not None and min_level < 6:
        print("note: coarse levels need GLO-90 for the whole (up to 90 degree wide) tile footprint, "
              "so this downloads far more than the box itself; see --dry-run", file=sys.stderr)
    plan = plan_levels(cfg, sources)
    est = estimate(cfg, sources, plan)
    print(format_estimate(est, cfg))
    if a.dry_run:
        return 0
    desc = (f"levels {min_level}-{max_level}" if bbox is None else
            f"{region_txt} levels {min_level}-{max_level}")
    res = run_build(cfg, desc)
    print(f"done in {res['seconds']:.1f} s: {res['totals']}")
    print(f"pack now holds {res['pack_files']:,} files, {res['pack_bytes'] / 1e6:,.1f} MB under {terrain_dir}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
