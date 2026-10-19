#!/usr/bin/env python3
"""Render a PNG hillshade (optionally tinted by land class) of the tiles of one level of a pack.

  preview_hillshade.py --pack data --level 9 --out hood.png
  preview_hillshade.py --pack data --level 10 --bbox -121.85,45.25,-121.55,45.50 --out hood10.png

Pure numpy + stdlib (own PNG writer), so it needs no extra dependency.
"""
from __future__ import annotations

import argparse
import os
import struct
import sys
import zlib

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from fsterrain.packread import Pack  # noqa: E402
from fsterrain.tilegrid import CELLS, SAMPLES, Tile, step_deg  # noqa: E402

# land class palette (index = fsim::LandClass value), RGB
LC_RGB = np.array([
    (128, 128, 128),  # Unknown
    (40, 90, 160),    # Water
    (30, 90, 40),     # Tree
    (110, 130, 70),   # Shrub
    (140, 170, 90),   # Grass
    (200, 190, 100),  # Crop
    (150, 150, 150),  # Urban
    (160, 140, 110),  # Bare
    (245, 245, 250),  # SnowIce
    (80, 140, 120),   # Wetland
    (60, 120, 80),    # Mangrove
    (150, 160, 130),  # Moss
], dtype=np.float64)


def write_png(path: str, rgb: np.ndarray) -> None:
    h, w, _ = rgb.shape
    raw = b"".join(b"\x00" + rgb[y].tobytes() for y in range(h))

    def chunk(tag: bytes, data: bytes) -> bytes:
        return struct.pack(">I", len(data)) + tag + data + struct.pack(">I", zlib.crc32(tag + data))

    png = (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 2, 0, 0, 0))
           + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
    with open(path, "wb") as f:
        f.write(png)


def hypsometric(h: np.ndarray) -> np.ndarray:
    stops = [(-500, (30, 60, 120)), (0, (60, 110, 80)), (300, (110, 150, 80)), (900, (170, 160, 100)),
             (1800, (140, 110, 80)), (2800, (200, 200, 200)), (4500, (255, 255, 255))]
    xs = [s[0] for s in stops]
    out = np.zeros(h.shape + (3,))
    for ch in range(3):
        out[..., ch] = np.interp(h, xs, [s[1][ch] for s in stops])
    return out


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pack", required=True)
    ap.add_argument("--level", type=int, help="level to render (default: finest present)")
    ap.add_argument("--bbox", help="west,south,east,north: only tiles touching this box")
    ap.add_argument("--out", required=True, help="PNG path")
    ap.add_argument("--max-px", type=int, default=1400, help="longest image side (default 1400)")
    ap.add_argument("--landcover", action="store_true", help="tint by land class instead of elevation")
    ap.add_argument("--exaggeration", type=float, default=2.0, help="vertical exaggeration (default 2)")
    a = ap.parse_args(argv)
    pack = Pack(a.pack)
    level = a.level if a.level is not None else max(pack.levels_on_disk())
    tiles = pack.tiles_at_level(level)
    if a.bbox:
        w, s, e, n = (float(v) for v in a.bbox.split(","))
        tiles = [t for t in tiles if t.east_deg > w and t.west_deg < e and t.north_deg > s and t.south_deg < n]
    if not tiles:
        print("no tiles", file=sys.stderr)
        return 1
    x0, x1 = min(t.x for t in tiles), max(t.x for t in tiles)
    y0, y1 = min(t.y for t in tiles), max(t.y for t in tiles)
    nxt, nyt = x1 - x0 + 1, y1 - y0 + 1
    H = np.zeros((nyt * CELLS + 1, nxt * CELLS + 1), dtype=np.float64)
    L = np.zeros((nyt * CELLS, nxt * CELLS), dtype=np.uint8)
    have = np.zeros((nyt, nxt), dtype=bool)
    for t in tiles:
        c0 = (t.x - x0) * CELLS
        r0 = (y1 - t.y) * CELLS            # north-up image
        H[r0:r0 + SAMPLES, c0:c0 + SAMPLES] = pack.heights(t)
        have[y1 - t.y, t.x - x0] = True
        lc = pack.landclass(t)
        if lc is not None:
            L[r0:r0 + CELLS, c0:c0 + CELLS] = lc
    stride = max(1, int(np.ceil(max(H.shape) / a.max_px)))
    Hs = H[::stride, ::stride]
    lat_mid = np.radians((tiles[0].south_deg + tiles[0].north_deg) / 2)
    step_m = step_deg(level) * stride * 111320.0
    dy = step_m
    dx = step_m * max(np.cos(lat_mid), 0.05)
    gy, gx = np.gradient(Hs * a.exaggeration, dy, dx)
    az, alt = np.radians(315.0), np.radians(45.0)
    slope = np.arctan(np.hypot(gx, gy))
    aspect = np.arctan2(-gx, gy)
    shade = np.sin(alt) * np.cos(slope) + np.cos(alt) * np.sin(slope) * np.cos(az - aspect)
    shade = np.clip(shade, 0.0, 1.0)
    if a.landcover and L.any():
        Ls = L[::stride, ::stride][:Hs.shape[0], :Hs.shape[1]]
        pad = np.zeros(Hs.shape, dtype=np.uint8)
        pad[:Ls.shape[0], :Ls.shape[1]] = Ls
        base = LC_RGB[pad]
    else:
        base = hypsometric(Hs)
    rgb = np.clip(base * (0.35 + 0.75 * shade[..., None]), 0, 255).astype(np.uint8)
    # tiles that are absent from the pack (dropped ocean or outside the build) are dark grey
    miss = np.kron(~have, np.ones((CELLS, CELLS), dtype=bool))
    miss = np.pad(miss, ((0, 1), (0, 1)), mode="edge")[::stride, ::stride]
    rgb[miss] = (24, 24, 28)
    write_png(a.out, rgb)
    print(f"wrote {a.out}: {rgb.shape[1]}x{rgb.shape[0]} px from {len(tiles)} tiles at level {level} "
          f"(1 px = {step_m:.0f} m)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
