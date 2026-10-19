"""DEM access and resampling to the tile grid (DATA.md section 4.2).

Method
------
Each Copernicus tile is a 1 degree "cell" with its own pixel grid. Georeferencing is read from
the file (pixel counts differ with latitude, and the lattice is a point lattice offset half a pixel
from the area bounds that GDAL reports), never assumed.

A tile corner sample is the **box average (width = the level's sample spacing, in degrees) of the
bilinearly interpolated source surface**, centred on the sample. This is "bilinear resampling at
the tile's own spacing" with a prefilter, so coarse levels are area averages of the source and never
point decimations. When the box is only slightly wider than one source pixel (level 10 on GLO-30)
it reduces to almost plain bilinear interpolation.

To keep the cost bounded at coarse levels each cell is first block-averaged by an integer divisor
of its pixel counts (so the reduced pixel is at most a quarter of the sample spacing).

Exactness: a sample's value depends only on its own position and on the source pixels around it,
through weights computed from that position and accumulated in a fixed order. Neighbouring tiles
therefore compute bit-identical values for the samples they share, and the same point on the
same level gives the same answer whichever tile (or process) computes it. Cell borders are
handled by pulling taps from the neighbouring cell's pixels, so there are no seams.
"""
from __future__ import annotations

import math
import os
from collections import OrderedDict
from dataclasses import dataclass

import numpy as np

from .tilegrid import CELLS, SAMPLES, Tile, step_deg

EPS_DEG = 1e-9          # snaps positions that are on an integer degree up to float noise
VOID_LO = -1000.0       # raw values outside [VOID_LO, VOID_HI] are treated as voids
VOID_HI = 12000.0


# ----------------------------------------------------------------------------- helpers

def cell_of(lat, lon):
    """1-degree cell (lat0, lon0) owning a point. Cells hold lat in (lat0, lat0+1], lon in
    [lon0, lon0+1) because the Copernicus point lattice includes the north and west lines."""
    lat0 = np.clip(np.ceil(np.asarray(lat) - EPS_DEG).astype(np.int64) - 1, -90, 89)
    lon0 = np.floor(np.asarray(lon) + EPS_DEG).astype(np.int64)
    lon0 = np.where(lon0 >= 180, lon0 - 360, lon0)
    return lat0, lon0


def wrap_lon(lon):
    lon = np.asarray(lon, dtype=np.float64)
    return np.where(lon >= 180.0, lon - 360.0, np.where(lon < -180.0, lon + 360.0, lon))


def best_divisor(n: int, limit: float) -> int:
    """Largest divisor d of n with d <= limit (at least 1)."""
    best = 1
    for d in range(1, int(max(limit, 1)) + 1):
        if n % d == 0:
            best = d
    return best


def tent_cdf(x: np.ndarray) -> np.ndarray:
    """Integral from -inf to x of the unit tent (bilinear kernel) centred on 0."""
    x = np.clip(x, -1.0, 1.0)
    return np.where(x < 0.0, (x + 1.0) ** 2 * 0.5, 0.5 + x - 0.5 * x * x)


def axis_taps(pos: np.ndarray, w: float):
    """Taps for the box average (width w pixels) centred at pos of the bilinear surface.

    Returns (idx[n,T] int64, wts[n,T] float64). Pixel i has weight
    (G(b-i) - G(a-i)) / w with [a, b] the box; weights are zero outside the support and sum to 1.
    """
    a = pos - 0.5 * w
    b = pos + 0.5 * w
    t = int(math.ceil(w)) + 3
    i0 = np.floor(a).astype(np.int64)
    idx = i0[:, None] + np.arange(t, dtype=np.int64)[None, :]
    wts = (tent_cdf(b[:, None] - idx) - tent_cdf(a[:, None] - idx)) / w
    return idx, wts


# ----------------------------------------------------------------------------- cells

@dataclass
class Cell:
    """A (possibly block-reduced) 1-degree DEM cell."""
    lat0: int
    lon0: int
    product: str
    nx: int            # source pixel counts
    ny: int
    relx: float        # source pixel i centre = lon0 + (i + relx) / nx
    rely: float        # source pixel j centre = lat0 + 1 - (j + rely) / ny
    fx: int            # block reduction factors
    fy: int
    arr: np.ndarray    # float32 (ny // fy, nx // fx)

    @property
    def nbytes(self) -> int:
        return self.arr.nbytes

    def col_pos(self, lon):
        """Continuous reduced-column coordinate of longitude(s) (pixel centres at integers)."""
        p = (np.asarray(lon, dtype=np.float64) - self.lon0) * self.nx - self.relx
        return (p - (self.fx - 1) * 0.5) / self.fx

    def row_pos(self, lat):
        q = ((self.lat0 + 1) - np.asarray(lat, dtype=np.float64)) * self.ny - self.rely
        return (q - (self.fy - 1) * 0.5) / self.fy

    def col_lon(self, idx):
        p = np.asarray(idx, dtype=np.float64) * self.fx + (self.fx - 1) * 0.5
        return wrap_lon(self.lon0 + (p + self.relx) / self.nx)

    def row_lat(self, idx):
        q = np.asarray(idx, dtype=np.float64) * self.fy + (self.fy - 1) * 0.5
        return (self.lat0 + 1) - (q + self.rely) / self.ny


def fill_voids(arr: np.ndarray) -> np.ndarray:
    """Fill voids (NaN or out-of-range values) by GDAL's inverse-distance fill; any void that
    remains (no valid pixel within reach) becomes 0 (sea level)."""
    bad = ~np.isfinite(arr) | (arr < VOID_LO) | (arr > VOID_HI)
    if not bad.any():
        return arr
    work = np.where(bad, 0.0, arr).astype(np.float32)
    if (~bad).any():
        from rasterio.fill import fillnodata
        work = fillnodata(work, mask=(~bad).astype(np.uint8), max_search_distance=100,
                          smoothing_iterations=0)
    return np.where(np.isfinite(work), work, 0.0).astype(np.float32)


def read_raw_cell(path: str, lat0: int, lon0: int):
    """Read a Copernicus GeoTIFF. Returns (arr float32, nx, ny, relx, rely)."""
    import rasterio
    with rasterio.open(path) as src:
        arr = src.read(1).astype(np.float32)
        tr = src.transform
        ny, nx = arr.shape
        if abs(tr.a * nx - 1.0) > 1e-6 or abs(-tr.e * ny - 1.0) > 1e-6:
            raise ValueError(f"{path}: not a 1 degree cell ({nx}x{ny}, res {tr.a},{tr.e})")
        if src.nodata is not None:
            arr = np.where(arr == np.float32(src.nodata), np.nan, arr)
        centre_lon0 = tr.c + 0.5 * tr.a
        centre_lat0 = tr.f + 0.5 * tr.e
        relx = round((centre_lon0 - lon0) * nx, 6)
        rely = round(((lat0 + 1) - centre_lat0) * ny, 6)
    return fill_voids(arr), nx, ny, relx, rely


class CellStore:
    """Loads cells on demand, with a byte-bounded LRU of reduced cells.

    `locate(lat0, lon0, k)` decides which product serves a cell at level k:
    the level's primary product, or the other one when the primary has no such cell.
    """

    def __init__(self, sources, glo90_max_level: int = 6, order: str = "auto",
                 cache_mb: float = 400.0, reader=None):
        self.sources = sources
        self.glo90_max_level = glo90_max_level
        self.order = order
        self.cache_bytes = int(cache_mb * 1e6)
        self._reduced: OrderedDict = OrderedDict()
        self._reduced_bytes = 0
        self._raw: OrderedDict = OrderedDict()
        self._locate: dict = {}
        self.reader = reader or read_raw_cell     # injectable for tests

    # ---------------------------------------------------------------- product selection
    def products_for_level(self, k: int):
        if self.order == "glo30":
            return ["glo30"]
        if self.order == "glo90":
            return ["glo90"]
        return ["glo90", "glo30"] if k <= self.glo90_max_level else ["glo30", "glo90"]

    def locate(self, lat0: int, lon0: int, k: int):
        """Returns (product, path) or None when the cell is open ocean / unpublished."""
        key = (lat0, lon0, tuple(self.products_for_level(k)))
        if key in self._locate:
            return self._locate[key]
        res = None
        for prod in self.products_for_level(k):
            p = self.sources.get_dem(prod, lat0, lon0)
            if p:
                res = (prod, p)
                break
        self._locate[key] = res
        return res

    # ---------------------------------------------------------------- cell loading
    def _get_raw(self, prod: str, path: str, lat0: int, lon0: int):
        key = (prod, lat0, lon0)
        if key in self._raw:
            self._raw.move_to_end(key)
            return self._raw[key]
        data = self.reader(path, lat0, lon0)
        self._raw[key] = data
        while len(self._raw) > 2:
            self._raw.popitem(last=False)
        return data

    def cell(self, lat0: int, lon0: int, k: int):
        """Reduced Cell for level k, or None if there is no data (treated as sea level)."""
        loc = self.locate(lat0, lon0, k)
        if loc is None:
            return None
        prod, path = loc
        step = step_deg(k)
        # reduction factors are a function of the file's pixel counts, known after the read
        key0 = (prod, lat0, lon0, k)
        if key0 in self._reduced:
            self._reduced.move_to_end(key0)
            return self._reduced[key0]
        arr, nx, ny, relx, rely = self._get_raw(prod, path, lat0, lon0)
        fx = best_divisor(nx, step * nx * 0.25)
        fy = best_divisor(ny, step * ny * 0.25)
        if fx == 1 and fy == 1:
            red = arr
        else:
            red = arr.reshape(ny // fy, fy, nx // fx, fx).mean(axis=(1, 3), dtype=np.float64)
            red = red.astype(np.float32)
        c = Cell(lat0, lon0, prod, nx, ny, relx, rely, fx, fy, red)
        # cache keyed by level (the factor depends on the level); evict by bytes
        self._reduced[key0] = c
        self._reduced_bytes += c.nbytes
        while self._reduced_bytes > self.cache_bytes and len(self._reduced) > 1:
            _, old = self._reduced.popitem(last=False)
            self._reduced_bytes -= old.nbytes
        return c

    # ---------------------------------------------------------------- point sampling
    def sample_points(self, lat: np.ndarray, lon: np.ndarray, k: int) -> np.ndarray:
        """Plain bilinear sample of the reduced surface (used to fill window borders)."""
        out = np.zeros(lat.shape, dtype=np.float32)
        lat0, lon0 = cell_of(lat, lon)
        lon = wrap_lon(lon)
        keys = lat0 * 1000 + (lon0 + 180)
        for kv in np.unique(keys):
            m = keys == kv
            la0, lo0 = int(lat0[m][0]), int(lon0[m][0])
            c = self.cell(la0, lo0, k)
            if c is None:
                continue
            out[m] = bilinear(c, lat[m], lon[m])
        return out

    def window(self, c: Cell, r0: int, r1: int, c0: int, c1: int, k: int) -> np.ndarray:
        """Reduced pixels [r0:r1, c0:c1] of cell c; indices outside the cell are taken from
        the neighbouring cells (zero where a neighbour has no data)."""
        ny_o, nx_o = c.arr.shape
        out = np.zeros((r1 - r0, c1 - c0), dtype=np.float32)
        ir0, ir1 = max(r0, 0), min(r1, ny_o)
        ic0, ic1 = max(c0, 0), min(c1, nx_o)
        if ir0 < ir1 and ic0 < ic1:
            out[ir0 - r0:ir1 - r0, ic0 - c0:ic1 - c0] = c.arr[ir0:ir1, ic0:ic1]
        if r0 >= 0 and c0 >= 0 and r1 <= ny_o and c1 <= nx_o:
            return out
        rows = np.arange(r0, r1)
        cols = np.arange(c0, c1)
        rin = (rows >= 0) & (rows < ny_o)
        cin = (cols >= 0) & (cols < nx_o)
        rr, cc = np.nonzero(~(rin[:, None] & cin[None, :]))
        lat = c.row_lat(rows[rr])
        lon = c.col_lon(cols[cc])
        out[rr, cc] = self.sample_points(lat, lon, k)
        return out


def bilinear(c: Cell, lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
    """Bilinear sample of a reduced cell, clamped at its own border, with lattice snapping so
    that points on a pixel centre return that pixel exactly."""
    ny_o, nx_o = c.arr.shape
    px = np.clip(c.col_pos(lon), 0.0, nx_o - 1.0)
    py = np.clip(c.row_pos(lat), 0.0, ny_o - 1.0)
    for p in (px, py):
        r = np.rint(p)
        snap = np.abs(p - r) < 1e-7
        p[snap] = r[snap]
    x0 = np.minimum(np.floor(px).astype(np.int64), max(nx_o - 2, 0))
    y0 = np.minimum(np.floor(py).astype(np.int64), max(ny_o - 2, 0))
    x1 = np.minimum(x0 + 1, nx_o - 1)
    y1 = np.minimum(y0 + 1, ny_o - 1)
    fx = px - x0
    fy = py - y0
    a = c.arr
    top = a[y0, x0] * (1 - fx) + a[y0, x1] * fx
    bot = a[y1, x0] * (1 - fx) + a[y1, x1] * fx
    return (top * (1 - fy) + bot * fy).astype(np.float32)


# ----------------------------------------------------------------------------- tile heights

def height_tile(store: CellStore, tile: Tile) -> np.ndarray:
    """257 x 257 float64 heights (metres) of a tile: box-averaged bilinear surface (see module
    docstring). Cells without data count as sea level."""
    k = tile.k
    step = step_deg(k)
    lons = tile.lon_samples_deg()
    lats = tile.lat_samples_deg()
    out = np.zeros((SAMPLES, SAMPLES), dtype=np.float64)
    lat0s = cell_of(lats, 0.0)[0]
    lon0s = cell_of(0.0, lons)[1]
    for la0 in np.unique(lat0s):
        rsel = np.flatnonzero(lat0s == la0)
        for lo0 in np.unique(lon0s):
            csel = np.flatnonzero(lon0s == lo0)
            c = store.cell(int(la0), int(lo0), k)
            if c is None:
                continue
            out[np.ix_(rsel, csel)] = _eval_cell(store, c, lats[rsel], lons[csel], step, k)
    return out


def _eval_cell(store: CellStore, c: Cell, lats, lons, step: float, k: int) -> np.ndarray:
    wx = step * c.nx / c.fx
    wy = step * c.ny / c.fy
    ic, wc = axis_taps(c.col_pos(lons), wx)
    ir, wr = axis_taps(c.row_pos(lats), wy)
    r0, r1 = int(ir.min()), int(ir.max()) + 1
    c0, c1 = int(ic.min()), int(ic.max()) + 1
    win = store.window(c, r0, r1, c0, c1, k).astype(np.float64)
    # pass 1: along columns, for every needed row (fixed accumulation order over taps)
    tmp = np.zeros((r1 - r0, len(lons)), dtype=np.float64)
    for t in range(ic.shape[1]):
        tmp += win[:, ic[:, t] - c0] * wc[:, t][None, :]
    # pass 2: along rows
    res = np.zeros((len(lats), len(lons)), dtype=np.float64)
    for t in range(ir.shape[1]):
        res += tmp[ir[:, t] - r0, :] * wr[:, t][:, None]
    return res


def heights_to_int16(h: np.ndarray, h_min: int = -500, h_max: int = 9000) -> np.ndarray:
    """Round half up and clamp to the valid range (DATA 4.2)."""
    return np.clip(np.floor(h + 0.5), h_min, h_max).astype("<i2")
