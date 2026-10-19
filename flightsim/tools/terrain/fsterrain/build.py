"""Planning and building of the tile pyramid."""
from __future__ import annotations

import math
import multiprocessing as mp
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import asdict, dataclass, field

import numpy as np

from . import landcover as lcmod
from .dem import EPS_DEG, CellStore, height_tile, heights_to_int16
from .pack import update_bitmaps, update_pack_ini, write_atomic
from .sources import EST_MB_PER_CELL, EST_MB_PER_WC_TILE, WC_SPAN_DEG, Sources
from .tilegrid import (CELLS, H16_BYTES, LC8_BYTES, SAMPLES, Tile, all_tiles, split_bbox,
                       span_deg, step_deg, tiles_for_bbox, tiles_x, tiles_y)


@dataclass
class BuildConfig:
    out: str                              # data pack root; terrain is written to <out>/terrain
    cache: str
    min_level: int
    max_level: int
    bbox: tuple | None = None             # None = whole world
    landcover: bool = True
    jobs: int = 1
    force: bool = False
    offline: bool = False
    glo90_max_level: int = 6
    source: str = "auto"                  # auto | glo30 | glo90
    keep_ocean_tiles: bool = False
    cell_cache_mb: float = 400.0
    region_name: str = "region"
    verbose: bool = True

    @property
    def terrain_dir(self) -> str:
        return os.path.join(self.out, "terrain")


# ------------------------------------------------------------------------------ planning

def _avail_grid(cells: set) -> np.ndarray:
    """(180, 360) bool, row = lat0 + 90, col = lon0 + 180."""
    g = np.zeros((180, 360), dtype=bool)
    for lat0, lon0 in cells:
        if -90 <= lat0 < 90 and -180 <= lon0 < 180:
            g[lat0 + 90, lon0 + 180] = True
    return g


def _integral(g: np.ndarray) -> np.ndarray:
    ii = np.zeros((g.shape[0] + 1, g.shape[1] + 1), dtype=np.int64)
    ii[1:, 1:] = g.cumsum(0).cumsum(1)
    return ii


def _any_in_ranges(ii, r0, r1, c0, c1):
    """Any True in inclusive row/col ranges (arrays), no wrap."""
    r0 = np.clip(r0, 0, 179)
    r1 = np.clip(r1, 0, 179)
    c0 = np.clip(c0, 0, 359)
    c1 = np.clip(c1, 0, 359)
    s = ii[r1 + 1, c1 + 1] - ii[r0, c1 + 1] - ii[r1 + 1, c0] + ii[r0, c0]
    return s > 0


def candidate_mask(k: int, grid_ii: np.ndarray) -> np.ndarray:
    """(tiles_y, tiles_x) bool: tile footprint touches at least one published DEM cell."""
    span = span_deg(k)
    xs = np.arange(tiles_x(k))
    ys = np.arange(tiles_y(k))
    west = -180.0 + xs * span
    east = west + span
    south = -90.0 + ys * span
    north = south + span
    c0 = np.floor(west + EPS_DEG).astype(np.int64) + 180
    c1 = np.floor(east + EPS_DEG).astype(np.int64) + 180
    r0 = np.ceil(south - EPS_DEG).astype(np.int64) - 1 + 90
    r1 = np.ceil(north - EPS_DEG).astype(np.int64) - 1 + 90
    m = np.zeros((len(ys), len(xs)), dtype=bool)
    # tiles that wrap (east edge at +180 -> cell -180) are covered by the clip: harmless
    for j in range(len(ys)):
        m[j] = _any_in_ranges(grid_ii, np.full(len(xs), r0[j]), np.full(len(xs), r1[j]), c0, np.minimum(c1, 359))
    return m


def plan_levels(cfg: BuildConfig, sources: Sources) -> dict[int, list[Tile]]:
    """Tiles to build per level: tiles over the region that touch published DEM cells."""
    plan: dict[int, list[Tile]] = {}
    avail = {p: _integral(_avail_grid(sources.dem_cells(p))) for p in ("glo30", "glo90")}
    for k in range(cfg.min_level, cfg.max_level + 1):
        if cfg.source == "glo30":
            prods = ["glo30"]
        elif cfg.source == "glo90":
            prods = ["glo90"]
        else:
            prods = ["glo90", "glo30"]
        ii = sum(avail[p] for p in prods)
        mask = candidate_mask(k, ii)
        if cfg.bbox is None:
            ys, xs = np.nonzero(mask)
            tiles = [Tile(k, int(x), int(y)) for y, x in zip(ys, xs)]
        else:
            tiles = [t for t in tiles_for_bbox(k, cfg.bbox) if mask[t.y, t.x]]
        plan[k] = sorted(tiles, key=lambda t: (t.y, t.x))
    return plan


def cells_needed(tiles: list[Tile], k: int) -> set:
    """1-degree cells touched by the tiles' footprints, grown by the filter reach."""
    out = set()
    margin = 1.5 * step_deg(k) + 0.005
    for t in tiles:
        s, n = t.south_deg - margin, t.north_deg + margin
        w, e = t.west_deg - margin, t.east_deg + margin
        for la in range(max(math.ceil(s - EPS_DEG) - 1, -90), min(math.ceil(n - EPS_DEG) - 1, 89) + 1):
            for lo in range(math.floor(w + EPS_DEG), math.floor(e + EPS_DEG) + 1):
                out.add((la, ((lo + 180) % 360) - 180))
    return out


def products_for(cfg: BuildConfig, k: int) -> list[str]:
    if cfg.source == "glo30":
        return ["glo30"]
    if cfg.source == "glo90":
        return ["glo90"]
    return ["glo90", "glo30"] if k <= cfg.glo90_max_level else ["glo30", "glo90"]


def estimate(cfg: BuildConfig, sources: Sources, plan: dict[int, list[Tile]]) -> dict:
    per_level = {}
    need = {"glo30": set(), "glo90": set()}
    for k, tiles in plan.items():
        n = len(tiles)
        h_bytes = n * H16_BYTES
        c_bytes = n * LC8_BYTES if cfg.landcover else 0
        per_level[k] = dict(tiles=n, bytes=h_bytes + c_bytes)
        for c in cells_needed(tiles, k):
            for prod in products_for(cfg, k):
                if c in sources.dem_cells(prod):
                    need[prod].add(c)
                    break
    wc = set()
    if cfg.landcover and plan:
        kmax = max(plan)
        for t in plan[kmax]:
            for la in range(math.floor(t.south_deg / WC_SPAN_DEG) * WC_SPAN_DEG, math.ceil(t.north_deg), WC_SPAN_DEG):
                for lo in range(math.floor(t.west_deg / WC_SPAN_DEG) * WC_SPAN_DEG, math.ceil(t.east_deg), WC_SPAN_DEG):
                    if la < t.north_deg and la + WC_SPAN_DEG > t.south_deg:
                        wc.add((la, lo))
        wc &= sources.wc_tiles()
    dl = {}
    for prod in need:
        missing = [c for c in need[prod] if not os.path.exists(sources.dem_path(prod, *c))]
        dl[prod] = dict(cells=len(need[prod]), missing=len(missing),
                        mb=len(missing) * EST_MB_PER_CELL[prod])
    wc_missing = [c for c in wc if not os.path.exists(sources.wc_path(*c))]
    return dict(per_level=per_level, download=dl,
                wc=dict(tiles=len(wc), missing=len(wc_missing), mb=len(wc_missing) * EST_MB_PER_WC_TILE),
                total_tiles=sum(v["tiles"] for v in per_level.values()),
                total_bytes=sum(v["bytes"] for v in per_level.values()))


def format_estimate(est: dict, cfg: BuildConfig) -> str:
    lines = []
    lines.append(f"{'level':>5} {'tiles':>9} {'size (upper bound)':>20}")
    for k, v in est["per_level"].items():
        lines.append(f"{k:>5} {v['tiles']:>9,} {v['bytes'] / 1e6:>17,.1f} MB")
    lines.append(f"{'total':>5} {est['total_tiles']:>9,} {est['total_bytes'] / 1e6:>17,.1f} MB"
                 f"  ({est['total_bytes'] / 2**30:.2f} GiB)")
    lines.append("Tile counts are tiles that touch at least one published DEM cell; all-ocean tiles "
                 "are dropped during the build, so the real size is smaller.")
    for prod, d in est["download"].items():
        if d["cells"]:
            lines.append(f"{prod}: {d['cells']:,} cells needed, {d['missing']:,} not cached "
                         f"(~{d['mb'] / 1000:,.2f} GB to download, average-size guess)")
    if cfg.landcover:
        w = est["wc"]
        lines.append(f"worldcover: {w['tiles']:,} 3-degree tiles needed, {w['missing']:,} not cached "
                     f"(~{w['mb'] / 1000:,.2f} GB to download, average-size guess)")
    else:
        lines.append("land class: off (heights only)")
    return "\n".join(lines)


# ------------------------------------------------------------------------------ worker

_W: dict = {}


def _init_worker(cfg_dict: dict, planned_next: set | None):
    cfg = BuildConfig(**cfg_dict)
    sources = Sources(cfg.cache, offline=cfg.offline, verbose=cfg.verbose)
    _W["cfg"] = cfg
    _W["sources"] = sources
    _W["store"] = CellStore(sources, cfg.glo90_max_level, cfg.source, cfg.cell_cache_mb)
    _W["planned_next"] = planned_next or set()


def _read_h16(path: str) -> np.ndarray:
    a = np.fromfile(path, dtype="<i2")
    if a.size != SAMPLES * SAMPLES:
        raise ValueError(f"{path}: bad size")
    return a.reshape(SAMPLES, SAMPLES).astype(np.float64)


def _children_lc(cfg: BuildConfig, t: Tile, planned_next: set):
    out = []
    for ch in t.children():
        cpath = os.path.join(cfg.terrain_dir, ch.rel_path_c())
        hpath = os.path.join(cfg.terrain_dir, ch.rel_path_h())
        if os.path.exists(cpath):
            out.append(np.fromfile(cpath, dtype=np.uint8).reshape(CELLS, CELLS))
        elif os.path.exists(hpath):
            out.append(np.zeros((CELLS, CELLS), dtype=np.uint8))          # unknown
        elif (ch.x, ch.y) in planned_next:
            out.append(np.full((CELLS, CELLS), lcmod.WATER, dtype=np.uint8))  # dropped ocean
        else:
            out.append(np.zeros((CELLS, CELLS), dtype=np.uint8))          # outside the build
    return out


def process_tile(tt: tuple) -> dict:
    cfg: BuildConfig = _W["cfg"]
    t = Tile(*tt)
    t0 = time.time()
    hpath = os.path.join(cfg.terrain_dir, t.rel_path_h())
    cpath = os.path.join(cfg.terrain_dir, t.rel_path_c())
    res = dict(tile=tt, h="", c="")
    have_h = os.path.exists(hpath) and not cfg.force
    have_c = os.path.exists(cpath) and not cfg.force
    if have_h and (have_c or not cfg.landcover):
        res["h"] = "existing"
        res["c"] = "existing" if have_c else "off"
        return res
    if have_h:
        h = _read_h16(hpath)
        hi = h.astype("<i2")
    else:
        h = height_tile(_W["store"], t)
        hi = heights_to_int16(h)
        h = hi.astype(np.float64)
    ocean = bool((hi == 0).all())
    lc = None
    if cfg.landcover:
        if t.k == cfg.max_level:
            lc = lcmod.lc_from_worldcover(_W["sources"], t, h)
        else:
            lc = lcmod.derive_from_children(_children_lc(cfg, t, _W["planned_next"]))
    lc_all_flat = lc is None or bool(np.isin(lc, (lcmod.UNKNOWN, lcmod.WATER)).all())
    if ocean and lc_all_flat and not cfg.keep_ocean_tiles:
        res["h"] = "ocean"
        res["c"] = "ocean"
        return res
    if not have_h:
        write_atomic(hpath, hi.astype("<i2").tobytes())
        res["h"] = "written"
    else:
        res["h"] = "existing"
    if lc is not None:
        if (lc == lcmod.UNKNOWN).all():
            res["c"] = "unknown"
        else:
            write_atomic(cpath, lc.astype(np.uint8).tobytes())
            res["c"] = "written"
    else:
        res["c"] = "off"
    res["secs"] = time.time() - t0
    return res


# ------------------------------------------------------------------------------ driver

class Progress:
    def __init__(self, label: str, total: int, enabled: bool = True):
        self.label, self.total, self.enabled = label, total, enabled
        self.done = 0
        self.t0 = time.time()
        self.last = 0.0
        self.counts: dict[str, int] = {}

    def update(self, res: dict | None):
        self.done += 1
        if res:
            key = "h:" + res["h"]
            self.counts[key] = self.counts.get(key, 0) + 1
            if res["c"] == "written":
                self.counts["c:written"] = self.counts.get("c:written", 0) + 1
        now = time.time()
        if self.enabled and (now - self.last > 2.0 or self.done == self.total):
            self.last = now
            el = now - self.t0
            rate = self.done / el if el > 0 else 0
            eta = (self.total - self.done) / rate if rate > 0 else 0
            cs = " ".join(f"{k}={v}" for k, v in sorted(self.counts.items()))
            print(f"[{self.label}] {self.done}/{self.total} tiles  {rate:.1f} t/s  eta {eta:.0f}s  {cs}",
                  file=sys.stderr, flush=True)


def prefetch(cfg: BuildConfig, sources: Sources, tiles: list[Tile], k: int):
    """Download the DEM cells (and the WorldCover tiles at the finest level) in parallel."""
    todo = []
    for c in cells_needed(tiles, k):
        for prod in products_for(cfg, k):
            if c in sources.dem_cells(prod):
                if not os.path.exists(sources.dem_path(prod, *c)):
                    todo.append((prod, c))
                break
    wc_todo = []
    if cfg.landcover and k == cfg.max_level:
        seen = set()
        for t in tiles:
            for la in range(math.floor(t.south_deg / WC_SPAN_DEG) * WC_SPAN_DEG, math.ceil(t.north_deg), WC_SPAN_DEG):
                for lo in range(math.floor(t.west_deg / WC_SPAN_DEG) * WC_SPAN_DEG, math.ceil(t.east_deg), WC_SPAN_DEG):
                    if la < t.north_deg and la + WC_SPAN_DEG > t.south_deg and (la, lo) in sources.wc_tiles():
                        seen.add((la, lo))
        wc_todo = [c for c in seen if not os.path.exists(sources.wc_path(*c))]
    if not todo and not wc_todo:
        return
    print(f"downloading {len(todo)} DEM cells and {len(wc_todo)} WorldCover tiles to {cfg.cache}",
          file=sys.stderr, flush=True)
    with ThreadPoolExecutor(max_workers=6) as ex:
        futs = [ex.submit(sources.get_dem, p, *c) for p, c in todo]
        futs += [ex.submit(sources.get_wc, *c) for c in wc_todo]
        for f in futs:
            f.result()


def run_build(cfg: BuildConfig, region_desc: str) -> dict:
    sources = Sources(cfg.cache, offline=cfg.offline, verbose=cfg.verbose)
    plan = plan_levels(cfg, sources)
    t_start = time.time()
    totals: dict[str, int] = {}
    written_tiles: list[Tile] = []
    cfg_dict = asdict(cfg)
    try:
        for k in sorted(plan, reverse=True):
            tiles = plan[k]
            if not tiles:
                continue
            prefetch(cfg, sources, tiles, k)
            planned_next = {(t.x, t.y) for t in plan.get(k + 1, [])}
            prog = Progress(f"L{k}", len(tiles), cfg.verbose)
            tasks = [(t.k, t.x, t.y) for t in tiles]
            if cfg.jobs > 1 and len(tasks) > 1:
                chunk = max(1, min(8, len(tasks) // (cfg.jobs * 8)))
                with mp.get_context("fork" if sys.platform != "win32" else "spawn").Pool(
                        cfg.jobs, _init_worker, (cfg_dict, planned_next)) as pool:
                    for r in pool.imap_unordered(process_tile, tasks, chunksize=chunk):
                        prog.update(r)
            else:
                _init_worker(cfg_dict, planned_next)
                for tk in tasks:
                    prog.update(process_tile(tk))
            for key, v in prog.counts.items():
                totals[key] = totals.get(key, 0) + v
            written_tiles += tiles
            update_bitmaps(cfg.terrain_dir, tiles)
    finally:
        if written_tiles:
            update_bitmaps(cfg.terrain_dir, written_tiles)
            update_pack_ini(cfg.terrain_dir, cfg.min_level, cfg.max_level, cfg.region_name,
                            region_desc, cfg.landcover)
    secs = time.time() - t_start
    size = 0
    nfiles = 0
    for root, _, files in os.walk(cfg.terrain_dir):
        for fn in files:
            if fn.endswith((".h16", ".lc8")):
                size += os.path.getsize(os.path.join(root, fn))
                nfiles += 1
    return dict(seconds=secs, totals=totals, pack_files=nfiles, pack_bytes=size)
