"""Land class tiles (DATA.md section 4.3).

* Finest level: counted directly from ESA WorldCover 10 m pixels. A pixel belongs to the 256 x 256
  cell that contains its centre.
* Coarser levels: derived from the four child tiles (cells nest exactly in the dyadic grid) by the
  same rule.

Rule per cell: Water if at least 50% of its pixels are water, otherwise the mode of the known
classes (ties go to the lowest class id). WorldCover "no data" pixels count as Water when the
cell's mean corner height is <= 0 (open ocean), otherwise they are Unknown and ignored by the mode.
"""
from __future__ import annotations

import math

import numpy as np

from .sources import WC_SPAN_DEG, Sources
from .tilegrid import CELLS, Tile, step_deg

UNKNOWN, WATER, TREE, SHRUB, GRASS, CROP, URBAN, BARE, SNOWICE, WETLAND, MANGROVE, MOSS = range(12)
N_CLASSES = 12
NODATA_BUCKET = 15

WC_TO_CLASS = np.full(256, NODATA_BUCKET, dtype=np.uint8)
for _wc, _cls in {10: TREE, 20: SHRUB, 30: GRASS, 40: CROP, 50: URBAN, 60: BARE, 70: SNOWICE,
                  80: WATER, 90: WETLAND, 95: MANGROVE, 100: MOSS}.items():
    WC_TO_CLASS[_wc] = _cls


def resolve_cells(counts: np.ndarray, n0: np.ndarray, ocean: np.ndarray) -> np.ndarray:
    """counts: (N, 12) class pixel counts (column 0 unused), n0: (N,) no-data pixel counts,
    ocean: (N,) bool. Returns uint8 class per cell."""
    counts = counts.astype(np.int64).copy()
    counts[:, UNKNOWN] = 0
    counts[:, WATER] += np.where(ocean, n0, 0)
    total = counts.sum(axis=1) + np.where(ocean, 0, n0)
    water_frac_ok = (total > 0) & (counts[:, WATER] * 2 >= total)
    mode = np.argmax(counts, axis=1).astype(np.uint8)
    mode = np.where(counts.max(axis=1) == 0, UNKNOWN, mode).astype(np.uint8)
    out = np.where(water_frac_ok, WATER, mode).astype(np.uint8)
    # cells with no pixels at all (outside every WorldCover tile): ocean -> Water
    out = np.where((total == 0) & ocean, WATER, out).astype(np.uint8)
    return out


def cell_ocean_mask(h: np.ndarray) -> np.ndarray:
    """(256,256) bool: mean of the cell's four corner heights <= 0 (h is the 257x257 array)."""
    mean = 0.25 * (h[:-1, :-1] + h[1:, :-1] + h[:-1, 1:] + h[1:, 1:])
    return mean <= 0.0


def lc_from_worldcover(sources: Sources, tile: Tile, h: np.ndarray, strip_px: int = 2048):
    """(256,256) uint8 land class for a tile from WorldCover pixels; None if no WorldCover tile
    overlaps and the tile is not ocean (all Unknown)."""
    import rasterio

    step = step_deg(tile.k)
    west, north = tile.west_deg, tile.north_deg
    south, east = tile.south_deg, tile.east_deg
    counts = np.zeros((CELLS * CELLS, 16), dtype=np.int64)
    for lat0 in range(math.floor(south / WC_SPAN_DEG) * WC_SPAN_DEG, math.ceil(north), WC_SPAN_DEG):
        for lon0 in range(math.floor(west / WC_SPAN_DEG) * WC_SPAN_DEG, math.ceil(east),
                          WC_SPAN_DEG):
            if lat0 >= north or lat0 + WC_SPAN_DEG <= south:
                continue
            path = sources.get_wc(lat0, lon0)
            if path is None:
                continue
            with rasterio.open(path) as src:
                _count_tile(src, counts, west, north, step, south, east, strip_px)
    ocean = cell_ocean_mask(h).reshape(-1)
    res = resolve_cells(counts[:, :N_CLASSES], counts[:, NODATA_BUCKET], ocean)
    return res.reshape(CELLS, CELLS)


def _count_tile(src, counts, west, north, step, south, east, strip_px):
    tr = src.transform
    dx, dy = tr.a, -tr.e
    # pixel index ranges whose centres are inside the tile footprint
    j0 = max(int(math.floor((west - tr.c) / dx - 0.5)), 0)
    j1 = min(int(math.ceil((east - tr.c) / dx - 0.5)) + 1, src.width)
    i0 = max(int(math.floor((tr.f - north) / dy - 0.5)), 0)
    i1 = min(int(math.ceil((tr.f - south) / dy - 0.5)) + 1, src.height)
    if j0 >= j1 or i0 >= i1:
        return
    lon_c = tr.c + (np.arange(j0, j1) + 0.5) * dx
    col = np.floor((lon_c - west) / step).astype(np.int64)
    col_ok = (col >= 0) & (col < CELLS)
    if not col_ok.any():
        return
    strip = max(1, min(strip_px, 4_000_000 // (j1 - j0)))    # bound the working set
    for s0 in range(i0, i1, strip):
        s1 = min(s0 + strip, i1)
        lat_c = tr.f - (np.arange(s0, s1) + 0.5) * dy
        row = np.floor((north - lat_c) / step).astype(np.int64)
        row_ok = (row >= 0) & (row < CELLS)
        if not row_ok.any():
            continue
        data = src.read(1, window=((s0, s1), (j0, j1)))
        cls = WC_TO_CLASS[data]
        idx = (row[:, None] * CELLS + col[None, :]) * 16 + cls
        ok = row_ok[:, None] & col_ok[None, :]
        counts += np.bincount(idx[ok], minlength=CELLS * CELLS * 16).reshape(CELLS * CELLS, 16)


def derive_from_children(children: list[np.ndarray]) -> np.ndarray:
    """Parent (256,256) from child arrays ordered as Tile.children(): SW, SE, NW, NE."""
    sw, se, nw, ne = children
    mosaic = np.empty((2 * CELLS, 2 * CELLS), dtype=np.uint8)      # north-up
    mosaic[:CELLS, :CELLS] = nw
    mosaic[:CELLS, CELLS:] = ne
    mosaic[CELLS:, :CELLS] = sw
    mosaic[CELLS:, CELLS:] = se
    blocks = mosaic.reshape(CELLS, 2, CELLS, 2).transpose(0, 2, 1, 3).reshape(CELLS, CELLS, 4)
    counts = np.zeros((CELLS, CELLS, N_CLASSES), dtype=np.int64)
    for cls in range(1, N_CLASSES):
        counts[:, :, cls] = (blocks == cls).sum(axis=2)
    counts[:, :, UNKNOWN] = 0
    water_ok = counts[:, :, WATER] * 2 >= 4
    mode = np.argmax(counts, axis=2).astype(np.uint8)
    mode = np.where(counts.max(axis=2) == 0, UNKNOWN, mode).astype(np.uint8)
    return np.where(water_ok, WATER, mode).astype(np.uint8)
