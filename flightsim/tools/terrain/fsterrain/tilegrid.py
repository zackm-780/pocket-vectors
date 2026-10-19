"""Tile scheme of DATA.md section 4.1 and the presence index of section 4.4.

All sample positions are computed from integer grid indices with dyadic step sizes, so every
position is exactly representable in float64 and identical no matter which tile computes it.
"""
from __future__ import annotations

import math
import os
from dataclasses import dataclass
from typing import Iterator

import numpy as np

SAMPLES = 257                 # corner samples per tile edge
CELLS = SAMPLES - 1           # 256 land-class cells per tile edge
H16_BYTES = SAMPLES * SAMPLES * 2   # 132,098
LC8_BYTES = CELLS * CELLS           # 65,536
NODATA_H16 = -32768
H_MIN_M = -500
H_MAX_M = 9000
MAX_LEVEL = 14                # sanity limit (bitmaps beyond ~12 are unreasonable)


def span_deg(k: int) -> float:
    """Tile span in degrees at level k (exact in float64)."""
    return 90.0 / (1 << k)


def step_deg(k: int) -> float:
    """Sample spacing in degrees at level k: span / 256 (exact in float64)."""
    return 90.0 / 256.0 / (1 << k)


def tiles_x(k: int) -> int:
    return 4 << k


def tiles_y(k: int) -> int:
    return 2 << k


@dataclass(frozen=True, order=True)
class Tile:
    k: int
    x: int
    y: int

    @property
    def west_deg(self) -> float:
        return -180.0 + self.x * span_deg(self.k)

    @property
    def east_deg(self) -> float:
        return -180.0 + (self.x + 1) * span_deg(self.k)

    @property
    def south_deg(self) -> float:
        return -90.0 + self.y * span_deg(self.k)

    @property
    def north_deg(self) -> float:
        return -90.0 + (self.y + 1) * span_deg(self.k)

    def lon_samples_deg(self) -> np.ndarray:
        """Longitudes of columns 0..256 (float64). Exact: -180 + (x*256 + c) * step.

        Values are wrapped into [-180, 180), so the east edge of the last column of tiles
        equals the west edge of column 0 across the antimeridian.
        """
        g = self.x * CELLS + np.arange(SAMPLES, dtype=np.int64)
        lon = -180.0 + g * step_deg(self.k)
        return np.where(lon >= 180.0, lon - 360.0, lon)

    def lat_samples_deg(self) -> np.ndarray:
        """Latitudes of rows 0..256, north-up (row r at north - r*step). Exact."""
        g = (self.y + 1) * CELLS - np.arange(SAMPLES, dtype=np.int64)
        return -90.0 + g * step_deg(self.k)

    def children(self) -> list["Tile"]:
        return [Tile(self.k + 1, 2 * self.x + i, 2 * self.y + j) for j in (0, 1) for i in (0, 1)]

    def parent(self) -> "Tile":
        return Tile(self.k - 1, self.x >> 1, self.y >> 1)

    def rel_path_h(self) -> str:
        return f"h/{self.k}/{self.x}/{self.y}.h16"

    def rel_path_c(self) -> str:
        return f"c/{self.k}/{self.x}/{self.y}.lc8"


def tile_containing(k: int, lat_deg: float, lon_deg: float) -> Tile:
    span = span_deg(k)
    x = int(math.floor((lon_deg + 180.0) / span))
    y = int(math.floor((lat_deg + 90.0) / span))
    return Tile(k, min(max(x, 0), tiles_x(k) - 1), min(max(y, 0), tiles_y(k) - 1))


def bbox_tile_range(k: int, west: float, south: float, east: float, north: float):
    """Inclusive (x0, x1, y0, y1) of tiles with positive-area overlap with the bbox."""
    span = span_deg(k)
    x0 = int(math.floor((west + 180.0) / span))
    x1 = int(math.ceil((east + 180.0) / span)) - 1
    y0 = int(math.floor((south + 90.0) / span))
    y1 = int(math.ceil((north + 90.0) / span)) - 1
    x0 = max(x0, 0)
    y0 = max(y0, 0)
    x1 = min(max(x1, x0), tiles_x(k) - 1)
    y1 = min(max(y1, y0), tiles_y(k) - 1)
    return x0, x1, y0, y1


def split_bbox(west: float, south: float, east: float, north: float):
    """Split an antimeridian-crossing bbox (west > east) into two."""
    if west <= east:
        return [(west, south, east, north)]
    return [(west, south, 180.0, north), (-180.0, south, east, north)]


def tiles_for_bbox(k: int, bbox) -> list[Tile]:
    out: dict[Tile, None] = {}
    for w, s, e, n in split_bbox(*bbox):
        x0, x1, y0, y1 = bbox_tile_range(k, w, s, e, n)
        for y in range(y0, y1 + 1):
            for x in range(x0, x1 + 1):
                out[Tile(k, x, y)] = None
    return sorted(out, key=lambda t: (t.y, t.x))


def all_tiles(k: int) -> Iterator[Tile]:
    for y in range(tiles_y(k)):
        for x in range(tiles_x(k)):
            yield Tile(k, x, y)


# ---------------------------------------------------------------- presence index (4.4)

def bitmap_bytes(k: int) -> int:
    return -(-(8 * 4 ** k) // 8)


def bit_index(t: Tile) -> int:
    return t.y * tiles_x(t.k) + t.x


def read_bitmap(path: str, k: int) -> bytearray:
    """Existing bitmap (merge base), or zeros. A wrong-sized file is treated as corrupt."""
    n = bitmap_bytes(k)
    try:
        with open(path, "rb") as f:
            data = f.read()
        if len(data) == n:
            return bytearray(data)
    except FileNotFoundError:
        pass
    return bytearray(n)


def set_bit(bm: bytearray, t: Tile) -> None:
    i = bit_index(t)
    bm[i >> 3] |= 1 << (i & 7)


def get_bit(bm: bytes | bytearray, t: Tile) -> bool:
    i = bit_index(t)
    return bool(bm[i >> 3] >> (i & 7) & 1)


def write_bitmap(path: str, bm: bytearray) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    tmp = path + ".tmp"
    with open(tmp, "wb") as f:
        f.write(bm)
    os.replace(tmp, path)


def bitmap_tiles(bm: bytes | bytearray, k: int) -> Iterator[Tile]:
    arr = np.unpackbits(np.frombuffer(bytes(bm), dtype=np.uint8), bitorder="little")
    nx = tiles_x(k)
    for i in np.flatnonzero(arr):
        yield Tile(k, int(i) % nx, int(i) // nx)
