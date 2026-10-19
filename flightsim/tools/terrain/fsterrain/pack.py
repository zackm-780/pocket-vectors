"""Pack output: atomic file writes, pack.ini with merge-on-update, presence bitmaps, presets."""
from __future__ import annotations

import configparser
import datetime
import os

import numpy as np

from .tilegrid import (Tile, bitmap_bytes, read_bitmap, set_bit, write_bitmap)

DEM_ATTRIBUTION = (
    "Produced using Copernicus WorldDEM-30 and WorldDEM-90 © DLR e.V. 2010-2014 and "
    "© Airbus Defence and Space GmbH 2014-2018 provided under COPERNICUS by the European "
    "Union and ESA; all rights reserved"
)
LC_ATTRIBUTION = "ESA WorldCover 2021 v200, CC BY 4.0"
LC_CREDIT = ("© ESA WorldCover project 2021 / Contains modified Copernicus Sentinel data "
             "(2021) processed by ESA WorldCover consortium")

# name -> (west, south, east, north). The first entry is the primary preset (PLAN D19).
PRESETS = {
    "pdx_oth": (-124.7, 43.0, -121.4, 46.3),      # Portland KPDX .. North Bend KOTH, Mt Hood
    "bayarea": (-123.2, 36.9, -121.2, 38.4),      # San Francisco Bay Area
    "mthood": (-121.85, 45.25, -121.55, 45.50),   # small Mt Hood test box
    "tam_diablo": (-122.65, 37.75, -121.85, 38.0),  # small Bay Area test box
}


def write_atomic(path: str, data: bytes) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = f"{path}.{os.getpid()}.tmp"
    with open(tmp, "wb") as f:
        f.write(data)
    os.replace(tmp, path)


def utc_now() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _read_ini(path: str) -> configparser.ConfigParser:
    cp = configparser.ConfigParser(interpolation=None, strict=False)
    cp.optionxform = str
    if os.path.exists(path):
        cp.read(path, encoding="utf-8")
    return cp


def update_pack_ini(terrain_dir: str, min_level: int, max_level: int, region_name: str,
                    region_desc: str, has_landcover: bool) -> dict:
    """Create or merge pack.ini. Levels are the union with the existing pack."""
    path = os.path.join(terrain_dir, "pack.ini")
    cp = _read_ini(path)
    for sec in ("pack", "attribution", "regions"):
        if not cp.has_section(sec):
            cp.add_section(sec)
    p = cp["pack"]
    old_min = int(p.get("min_level", min_level))
    old_max = int(p.get("max_level", max_level))
    p["format"] = "flightsim-terrain"
    p["version"] = "1"
    p["samples"] = "257"
    p["min_level"] = str(min(old_min, min_level))
    p["max_level"] = str(max(old_max, max_level))
    p.setdefault("created_utc", utc_now())
    cp["attribution"]["dem"] = DEM_ATTRIBUTION
    if has_landcover or cp["attribution"].get("landcover"):
        cp["attribution"]["landcover"] = LC_ATTRIBUTION
        cp["attribution"]["landcover_credit"] = LC_CREDIT
    cp["regions"][region_name] = region_desc
    order = ["pack", "attribution", "regions"]
    lines = []
    for sec in order:
        lines.append(f"[{sec}]")
        for k, v in cp[sec].items():
            lines.append(f"{k} = {v}")
    write_atomic(path, ("\n".join(lines) + "\n").encode("utf-8"))
    return {s: dict(cp[s]) for s in order}


def update_bitmaps(terrain_dir: str, tiles) -> int:
    """Set the presence bit of every given tile whose .h16 exists, merging with the bitmaps
    already on disk. Returns the number of bits newly set."""
    by_level: dict[int, list[Tile]] = {}
    for t in tiles:
        by_level.setdefault(t.k, []).append(t)
    new = 0
    for k, ts in by_level.items():
        path = os.path.join(terrain_dir, "index", f"L{k}.bit")
        bm = read_bitmap(path, k)
        before = int(np.unpackbits(np.frombuffer(bytes(bm), dtype=np.uint8)).sum())
        for t in ts:
            if os.path.exists(os.path.join(terrain_dir, t.rel_path_h())):
                set_bit(bm, t)
        after = int(np.unpackbits(np.frombuffer(bytes(bm), dtype=np.uint8)).sum())
        new += after - before
        write_bitmap(path, bm)
    return new


def rebuild_bitmaps(terrain_dir: str) -> dict:
    """Rebuild every index/L{k}.bit from the .h16 files on disk."""
    counts = {}
    hdir = os.path.join(terrain_dir, "h")
    if not os.path.isdir(hdir):
        return counts
    for kname in sorted(os.listdir(hdir), key=lambda s: int(s) if s.isdigit() else 99):
        if not kname.isdigit():
            continue
        k = int(kname)
        bm = bytearray(bitmap_bytes(k))
        n = 0
        for xname in os.listdir(os.path.join(hdir, kname)):
            xdir = os.path.join(hdir, kname, xname)
            for fn in os.listdir(xdir):
                if fn.endswith(".h16"):
                    set_bit(bm, Tile(k, int(xname), int(fn[:-4])))
                    n += 1
        write_bitmap(os.path.join(terrain_dir, "index", f"L{k}.bit"), bm)
        counts[k] = n
    return counts
