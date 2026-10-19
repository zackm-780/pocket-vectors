"""Reader for the terrain pack, mirroring what the runtime does (DATA.md section 4)."""
from __future__ import annotations

import math
import os

import numpy as np

from .tilegrid import (CELLS, SAMPLES, Tile, bitmap_bytes, bitmap_tiles, get_bit, read_bitmap,
                       step_deg, tile_containing, tiles_x, tiles_y)


class Pack:
    def __init__(self, root: str):
        """root: the data pack root (containing terrain/) or the terrain directory itself."""
        t = os.path.join(root, "terrain")
        self.dir = t if os.path.isdir(t) else root
        self._bm: dict[int, bytearray] = {}
        self._cache: dict[Tile, np.ndarray] = {}

    def levels_on_disk(self) -> list[int]:
        h = os.path.join(self.dir, "h")
        if not os.path.isdir(h):
            return []
        return sorted(int(n) for n in os.listdir(h) if n.isdigit())

    def bitmap(self, k: int) -> bytearray:
        if k not in self._bm:
            self._bm[k] = read_bitmap(os.path.join(self.dir, "index", f"L{k}.bit"), k)
        return self._bm[k]

    def has(self, t: Tile) -> bool:
        return get_bit(self.bitmap(t.k), t)

    def h_path(self, t: Tile) -> str:
        return os.path.join(self.dir, t.rel_path_h())

    def c_path(self, t: Tile) -> str:
        return os.path.join(self.dir, t.rel_path_c())

    def heights(self, t: Tile) -> np.ndarray:
        """(257,257) int16 array, row 0 = north edge."""
        if t not in self._cache:
            if len(self._cache) > 64:
                self._cache.clear()
            a = np.fromfile(self.h_path(t), dtype="<i2")
            if a.size != SAMPLES * SAMPLES:
                raise ValueError(f"{self.h_path(t)}: wrong size")
            self._cache[t] = a.reshape(SAMPLES, SAMPLES)
        return self._cache[t]

    def landclass(self, t: Tile):
        p = self.c_path(t)
        if not os.path.exists(p):
            return None
        return np.fromfile(p, dtype=np.uint8).reshape(CELLS, CELLS)

    def finest_tile_at(self, lat: float, lon: float, max_level: int = 14):
        """Finest present tile covering the point, using the bitmaps (runtime behaviour)."""
        for k in range(min(max(self.levels_on_disk() + [0]), max_level), -1, -1):
            if not os.path.exists(os.path.join(self.dir, "index", f"L{k}.bit")):
                continue
            t = tile_containing(k, lat, lon)
            if self.has(t):
                return t
        return None

    def elevation(self, lat: float, lon: float, level: int | None = None):
        """Bilinear elevation (m) at a point from the finest present tile (or a given level)."""
        t = (self.finest_tile_at(lat, lon) if level is None else tile_containing(level, lat, lon))
        if t is None or not os.path.exists(self.h_path(t)):
            return None, None
        h = self.heights(t)
        step = step_deg(t.k)
        fc = (lon - t.west_deg) / step
        fr = (t.north_deg - lat) / step
        c0 = min(int(math.floor(fc)), CELLS - 1)
        r0 = min(int(math.floor(fr)), CELLS - 1)
        c0, r0 = max(c0, 0), max(r0, 0)
        dc, dr = fc - c0, fr - r0
        v = (h[r0, c0] * (1 - dc) * (1 - dr) + h[r0, c0 + 1] * dc * (1 - dr)
             + h[r0 + 1, c0] * (1 - dc) * dr + h[r0 + 1, c0 + 1] * dc * dr)
        return float(v), t

    def tiles_at_level(self, k: int):
        return list(bitmap_tiles(self.bitmap(k), k))
