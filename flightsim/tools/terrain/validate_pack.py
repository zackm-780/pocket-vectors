#!/usr/bin/env python3
"""Validate a terrain pack against DATA.md section 4.

Checks (each prints PASS/FAIL, exit code 1 if any FAIL):
  files    pack.ini keys, file sizes (132,098 / 65,536 bytes), value ranges, orphan land class
  index    presence bitmaps match the .h16 files on disk, both directions
  edges    shared edges are bit-identical with the east and north neighbours (incl. antimeridian wrap)
  parents  child corner samples vs the parent's samples at the same points (statistics + tolerance)
  spots    spot elevations of known peaks (reports the pack value and the raw DEM maximum nearby)
  source   independent re-computation of random samples straight from the cached GeoTIFFs
           (own brute-force box-average of the bilinear surface, via rasterio.merge). Needs the cache.

Examples
  validate_pack.py --pack data
  validate_pack.py --pack data --spot "Mt Hood:45.3736:-121.6959:3429" --source-cache tools/terrain/cache
"""
from __future__ import annotations

import argparse
import configparser
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

from fsterrain.packread import Pack  # noqa: E402
from fsterrain.tilegrid import (CELLS, H16_BYTES, LC8_BYTES, NODATA_H16, SAMPLES, Tile,  # noqa: E402
                                bitmap_bytes, bitmap_tiles, step_deg, tiles_x, tiles_y)

DEFAULT_SPOTS = [
    # name, lat, lon, expected m, tolerance below (DEM reads low on peaks), tolerance above
    ("Mount Diablo", 37.8816, -121.9142, 1173.0, 60.0, 25.0),
    ("Mount Tamalpais East Peak", 37.9293, -122.5780, 784.0, 60.0, 25.0),
    ("Mount Hood", 45.3736, -121.6959, 3429.0, 150.0, 25.0),
]

results: list[tuple[str, bool, str]] = []


def report(name: str, ok: bool, msg: str):
    results.append((name, ok, msg))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {msg}", flush=True)


def check_files(pack: Pack) -> list[Tile]:
    ini = os.path.join(pack.dir, "pack.ini")
    cp = configparser.ConfigParser(interpolation=None)
    ok = os.path.exists(ini)
    msg = "pack.ini present"
    if ok:
        cp.read(ini, encoding="utf-8")
        need = {"pack": ["format", "version", "samples", "min_level", "max_level", "created_utc"],
                "attribution": ["dem"]}
        for sec, keys in need.items():
            for k in keys:
                if not cp.has_section(sec) or k not in cp[sec]:
                    ok, msg = False, f"missing [{sec}] {k}"
        if ok and (cp["pack"]["format"] != "flightsim-terrain" or cp["pack"]["version"] != "1"
                   or cp["pack"]["samples"] != "257"):
            ok, msg = False, "unexpected format/version/samples"
        if ok:
            msg = (f"format {cp['pack']['format']} v{cp['pack']['version']}, levels "
                   f"{cp['pack']['min_level']}-{cp['pack']['max_level']}, "
                   f"regions: {dict(cp['regions']) if cp.has_section('regions') else {}}")
            ok2 = bool(cp["attribution"]["dem"].strip())
            if not ok2:
                ok, msg = False, "empty DEM attribution"
    report("pack.ini", ok, msg)
    tiles: list[Tile] = []
    bad_size, bad_range, orphans, n_h, n_c = [], [], [], 0, 0
    hmin, hmax = 32767, -32768
    for k in pack.levels_on_disk():
        hdir = os.path.join(pack.dir, "h", str(k))
        for xn in os.listdir(hdir):
            for fn in os.listdir(os.path.join(hdir, xn)):
                if not fn.endswith(".h16"):
                    continue
                t = Tile(k, int(xn), int(fn[:-4]))
                tiles.append(t)
                n_h += 1
                p = pack.h_path(t)
                if os.path.getsize(p) != H16_BYTES:
                    bad_size.append(p)
                    continue
                a = pack.heights(t)
                valid = a[a != NODATA_H16]
                if valid.size:
                    hmin, hmax = min(hmin, int(valid.min())), max(hmax, int(valid.max()))
                    if valid.min() < -500 or valid.max() > 9000:
                        bad_range.append(p)
    for k in pack.levels_on_disk():
        cdir = os.path.join(pack.dir, "c", str(k))
        if not os.path.isdir(cdir):
            continue
        for xn in os.listdir(cdir):
            for fn in os.listdir(os.path.join(cdir, xn)):
                if not fn.endswith(".lc8"):
                    continue
                t = Tile(k, int(xn), int(fn[:-4]))
                n_c += 1
                p = pack.c_path(t)
                if os.path.getsize(p) != LC8_BYTES:
                    bad_size.append(p)
                elif np.fromfile(p, dtype=np.uint8).max() > 11:
                    bad_range.append(p)
                if not os.path.exists(pack.h_path(t)):
                    orphans.append(p)
    report("files", not (bad_size or bad_range or orphans),
           f"{n_h} .h16 + {n_c} .lc8; height range {hmin}..{hmax} m; bad size {len(bad_size)}, "
           f"out of range {len(bad_range)}, lc without heights {len(orphans)}")
    return tiles


def check_index(pack: Pack, tiles: list[Tile]):
    on_disk = set(tiles)
    problems = []
    levels = set(t.k for t in tiles)
    idx_dir = os.path.join(pack.dir, "index")
    if os.path.isdir(idx_dir):
        for fn in os.listdir(idx_dir):
            if fn.startswith("L") and fn.endswith(".bit"):
                levels.add(int(fn[1:-4]))
    n_bits = 0
    for k in sorted(levels):
        p = os.path.join(idx_dir, f"L{k}.bit")
        if not os.path.exists(p):
            problems.append(f"L{k}.bit missing")
            continue
        if os.path.getsize(p) != bitmap_bytes(k):
            problems.append(f"L{k}.bit wrong size {os.path.getsize(p)} != {bitmap_bytes(k)}")
            continue
        bits = set(bitmap_tiles(pack.bitmap(k), k))
        n_bits += len(bits)
        files = {t for t in on_disk if t.k == k}
        if bits - files:
            problems.append(f"L{k}: {len(bits - files)} bits without a file")
        if files - bits:
            problems.append(f"L{k}: {len(files - bits)} files without a bit")
    report("index", not problems, f"{n_bits} bits over {len(levels)} levels; "
           + ("consistent with files" if not problems else "; ".join(problems[:6])))


def check_edges(pack: Pack, tiles: list[Tile]):
    present = set(tiles)
    n_pairs = n_bad = 0
    n_samples = 0
    worst = 0
    for t in tiles:
        a = pack.heights(t)
        east = Tile(t.k, (t.x + 1) % tiles_x(t.k), t.y)
        if east in present:
            b = pack.heights(east)
            n_pairs += 1
            n_samples += SAMPLES
            d = np.abs(a[:, SAMPLES - 1].astype(int) - b[:, 0].astype(int))
            if d.max() > 0:
                n_bad += 1
                worst = max(worst, int(d.max()))
        if t.y + 1 < tiles_y(t.k):
            north = Tile(t.k, t.x, t.y + 1)
            if north in present:
                b = pack.heights(north)
                n_pairs += 1
                n_samples += SAMPLES
                d = np.abs(a[0, :].astype(int) - b[SAMPLES - 1, :].astype(int))
                if d.max() > 0:
                    n_bad += 1
                    worst = max(worst, int(d.max()))
    report("edges", n_bad == 0,
           f"{n_pairs} shared edges compared ({n_samples} samples), {n_bad} differ"
           + (f", worst {worst} m" if n_bad else ", all bit-identical"))


def check_parents(pack: Pack, tiles: list[Tile], tol_mean: float, tol_p99: float):
    """Parent/child consistency.

    A parent sample is the average of the surface over a box twice as wide as a child's, so it
    equals the child samples only up to smoothing. Two statistics per level pair:
      raw       |child - parent| at the shared points (informational: it measures terrain
                roughness at the child's scale, large in mountains and for coarse levels)
      smoothed  |[1 2 1]x[1 2 1]/16 smoothing of the children - parent| (the real check: a
                parent must be consistent with the children it summarises). Interior points only.
                Judged relative to the local relief of the children (range over a 3x3 stencil,
                plus 2 m), because the absolute mismatch scales with terrain roughness.
    """
    present = set(tiles)
    raw, smooth, rel = {}, {}, {}
    cr = np.arange(0, SAMPLES, 2)
    ci = np.arange(2, SAMPLES - 2, 2)
    for t in tiles:
        if t.k == 0:
            continue
        p = t.parent()
        if p not in present:
            continue
        ph = pack.heights(p).astype(np.float64)
        ch = pack.heights(t).astype(np.float64)
        # child sample (r, c) with even r, c sits on parent sample (128*(1-(Y&1)) + r/2, 128*(X&1) + c/2)
        pr = 128 * (1 - (t.y & 1)) + cr // 2
        pc = 128 * (t.x & 1) + cr // 2
        raw.setdefault(t.k, []).append((ch[np.ix_(cr, cr)] - ph[np.ix_(pr, pc)]).ravel())
        sm = 0.25 * ch[:, :-2] + 0.5 * ch[:, 1:-1] + 0.25 * ch[:, 2:]            # along columns
        sm = 0.25 * sm[:-2, :] + 0.5 * sm[1:-1, :] + 0.25 * sm[2:, :]            # along rows
        # sm[i, j] is centred on child (i + 1, j + 1)
        pr_i = 128 * (1 - (t.y & 1)) + ci // 2
        pc_i = 128 * (t.x & 1) + ci // 2
        d_sm = sm[np.ix_(ci - 1, ci - 1)] - ph[np.ix_(pr_i, pc_i)]
        # local relief of the children around each point (range over the 3x3 stencil), metres
        win = [ch[1 + a:SAMPLES - 1 + a, 1 + b:SAMPLES - 1 + b] for a in (-1, 0, 1) for b in (-1, 0, 1)]
        relief = (np.max(win, axis=0) - np.min(win, axis=0))[np.ix_(ci - 1, ci - 1)]
        smooth.setdefault(t.k, []).append(d_sm.ravel())
        rel.setdefault(t.k, []).append((np.abs(d_sm) / (relief + 2.0)).ravel())
    if not raw:
        report("parents", True, "no parent/child pairs present (nothing to compare)")
        return
    lines, ok = [], True

    def st(d):
        d = np.abs(np.concatenate(d))
        return d, (f"n={d.size} mean={d.mean():.2f} p50={np.percentile(d, 50):.2f} "
                   f"p95={np.percentile(d, 95):.2f} p99={np.percentile(d, 99):.2f} max={d.max():.1f}")

    for k in sorted(raw):
        _, s_raw = st(raw[k])
        d2, s_sm = st(smooth[k])
        r = np.concatenate(rel[k])
        lines.append(f"L{k}->L{k-1}  raw: {s_raw} | smoothed: {s_sm} | smoothed/relief: "
                     f"mean={r.mean():.3f} p99={np.percentile(r, 99):.3f}")
        if r.mean() > tol_mean or np.percentile(r, 99) > tol_p99:
            ok = False
    report("parents", ok, f"smoothed/relief tolerance mean<={tol_mean}, p99<={tol_p99}\n    " + "\n    ".join(lines))


def check_spots(pack: Pack, spots, source_cache: str | None):
    ok_all = True
    lines = []
    for name, lat, lon, exp, tol_lo, tol_hi in spots:
        v, t = pack.elevation(lat, lon)
        if v is None:
            lines.append(f"{name}: no tile covers {lat},{lon} (skipped)")
            continue
        raw = raw_max_near(source_cache, lat, lon, 100.0) if source_cache else None
        ok = (exp - tol_lo) <= v <= (exp + tol_hi)
        ok_all &= ok
        extra = f", raw GLO-30 max within 100 m = {raw:.1f}" if raw is not None else ""
        lines.append(f"{name}: pack {v:.1f} m (L{t.k}) vs reference {exp:.0f} m, diff {v - exp:+.1f} "
                     f"(allowed -{tol_lo:g}/+{tol_hi:g}){extra} -> {'ok' if ok else 'OUT OF TOLERANCE'}")
    if not lines:
        report("spots", True, "no spots requested")
    else:
        report("spots", ok_all, "\n    " + "\n    ".join(lines))


class RawLattice:
    """Independent reader of the cached GLO-30 GeoTIFFs as a global point lattice.

    Verified from the files: pixel centres lie on integer multiples of 1/3600 degree (below 50 N), so
    the value at lattice point (i, j) is lon = i/3600, lat = j/3600. The row/column inside a 1 degree
    cell is derived here from the lattice, not from the pipeline's own georeferencing code.
    """
    N = 3600

    def __init__(self, cache: str):
        self.dir = os.path.join(cache, "glo30")
        self.ds: dict = {}

    def _open(self, lat0: int, lon0: int):
        key = (lat0, lon0)
        if key not in self.ds:
            import rasterio
            from fsterrain.sources import cell_name
            p = os.path.join(self.dir, cell_name("glo30", lat0, lon0) + ".tif")
            ds = rasterio.open(p) if os.path.exists(p) else None
            if ds is not None:
                tr = ds.transform
                assert abs((tr.c + tr.a / 2) - lon0) < 1e-9 and abs((tr.f + tr.e / 2) - (lat0 + 1)) < 1e-9, \
                    f"{p}: not the expected point lattice"
                assert ds.width == self.N and ds.height == self.N, f"{p}: not 3600 x 3600"
            self.ds[key] = ds
        return self.ds[key]

    def block(self, i0: int, i1: int, j0: int, j1: int) -> np.ndarray:
        """Lattice values for i in [i0, i1], j in [j0, j1] -> array[j - j0, i - i0] (NaN if absent)."""
        out = np.full((j1 - j0 + 1, i1 - i0 + 1), np.nan)
        N = self.N
        for lon0 in range(math.floor(i0 / N), math.floor(i1 / N) + 1):
            for lat0 in range(math.ceil(j0 / N) - 1, math.ceil(j1 / N)):
                ds = self._open(lat0, lon0)
                if ds is None:
                    continue
                ia, ib = max(i0, lon0 * N), min(i1, lon0 * N + N - 1)
                ja, jb = max(j0, lat0 * N + 1), min(j1, lat0 * N + N)
                if ia > ib or ja > jb:
                    continue
                # row 0 of the cell is lattice j = (lat0 + 1) * N
                rows = ((lat0 + 1) * N - jb, (lat0 + 1) * N - ja + 1)
                cols = (ia - lon0 * N, ib - lon0 * N + 1)
                a = ds.read(1, window=(rows, cols)).astype(np.float64)   # north-up rows
                out[ja - j0:jb - j0 + 1, ia - i0:ib - i0 + 1] = a[::-1, :]
        return out

    def bilinear(self, lat: np.ndarray, lon: np.ndarray) -> np.ndarray:
        u, v = lon * self.N, lat * self.N
        i0, j0 = int(math.floor(u.min())), int(math.floor(v.min()))
        i1, j1 = int(math.floor(u.max())) + 1, int(math.floor(v.max())) + 1
        blk = self.block(i0, i1, j0, j1)
        ii, jj = np.floor(u).astype(int) - i0, np.floor(v).astype(int) - j0
        fu, fv = u - np.floor(u), v - np.floor(v)
        return (blk[jj, ii] * (1 - fu) * (1 - fv) + blk[jj, ii + 1] * fu * (1 - fv)
                + blk[jj + 1, ii] * (1 - fu) * fv + blk[jj + 1, ii + 1] * fu * fv)

    def close(self):
        for d in self.ds.values():
            if d is not None:
                d.close()


def raw_max_near(cache: str, lat: float, lon: float, radius_m: float):
    lat_ = RawLattice(cache)
    dlat = radius_m / 111320.0
    dlon = radius_m / (111320.0 * math.cos(math.radians(lat)))
    n = 2 * int(math.ceil(dlat * 3600)) + 1
    la, lo = np.meshgrid(lat + np.linspace(-dlat, dlat, n), lon + np.linspace(-dlon, dlon, n), indexing="ij")
    try:
        v = lat_.bilinear(la, lo)
    except Exception:
        return None
    finally:
        lat_.close()
    return None if np.isnan(v).all() else float(np.nanmax(v))


def check_source(pack: Pack, cache: str, n: int, tol_m: float, seed: int, level: int | None):
    """Brute-force, independent re-computation of random samples from the raw GeoTIFFs: the mean of
    the bilinear source surface over a dense sub-grid across the sample's box (width = spacing)."""
    rng = np.random.default_rng(seed)
    tiles = []
    for k in pack.levels_on_disk():
        if (level is not None and k != level) or k < 7:
            continue
        tiles += pack.tiles_at_level(k)
    if not tiles:
        report("source", True, "no GLO-30 level (>= 7) tiles to compare")
        return
    raw = RawLattice(cache)
    diffs, by_level, near_border = [], {}, 0
    for trial in range(n):
        t = tiles[int(rng.integers(len(tiles)))]
        lats_t, lons_t = t.lat_samples_deg(), t.lon_samples_deg()
        r, c = int(rng.integers(0, SAMPLES)), int(rng.integers(0, SAMPLES))
        if trial % 2 == 1:
            # half of the draws prefer samples within 2 steps of a 1-degree cell border
            s2 = 2 * step_deg(t.k)
            cs = np.flatnonzero(np.abs(lons_t - np.round(lons_t)) < s2)
            rs = np.flatnonzero(np.abs(lats_t - np.round(lats_t)) < s2)
            if cs.size:
                c = int(rng.choice(cs))
                near_border += 1
            elif rs.size:
                r = int(rng.choice(rs))
                near_border += 1
        lat, lon = lats_t[r], lons_t[c]
        step = step_deg(t.k)
        m = int(math.ceil(step * 3600)) * 4 + 1
        offs = (np.arange(m) + 0.5) / m * step - step / 2
        la, lo = np.meshgrid(lat + offs, lon + offs, indexing="ij")
        vals = raw.bilinear(la, lo)
        if np.isnan(vals).any():
            vals = np.nan_to_num(vals, nan=0.0)       # absent cell = open ocean = 0 (as the pipeline)
        d = float(pack.heights(t)[r, c]) - float(vals.mean())
        diffs.append(d)
        by_level.setdefault(t.k, []).append(abs(d))
    raw.close()
    if not diffs:
        report("source", True, "no comparable samples")
        return
    d = np.abs(np.array(diffs))
    lv = "; ".join(f"L{k}: n={len(v)} max={max(v):.2f}" for k, v in sorted(by_level.items()))
    report("source", float(d.max()) <= tol_m,
           f"{d.size} random samples vs independent brute-force box average of the raw GeoTIFFs "
           f"({near_border} drawn near a 1-degree cell border): mean|d|={d.mean():.3f} "
           f"p95={np.percentile(d, 95):.3f} max={d.max():.3f} m (tolerance {tol_m} m; includes the "
           f"0.5 m int16 rounding) [{lv}]")


def parse_spot(text: str):
    parts = text.split(":")
    if len(parts) < 4:
        raise argparse.ArgumentTypeError('spot is "name:lat:lon:expected_m[:tol_below:tol_above]"')
    lo, hi = (float(parts[4]), float(parts[5])) if len(parts) >= 6 else (150.0, 25.0)
    return (parts[0], float(parts[1]), float(parts[2]), float(parts[3]), lo, hi)


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--pack", required=True, help="data pack root (contains terrain/)")
    ap.add_argument("--spot", action="append", type=parse_spot, metavar="NAME:LAT:LON:M[:TOL_BELOW:TOL_ABOVE]",
                    help="spot elevation to check (repeatable). Defaults: Mt Diablo, Mt Tamalpais, "
                         "Mt Hood (only those covered by the pack are checked)")
    ap.add_argument("--no-default-spots", action="store_true")
    ap.add_argument("--source-cache", default=os.path.join(HERE, "cache"),
                    help="download cache, used for the raw-DEM comparisons (default: tools/terrain/cache)")
    ap.add_argument("--no-source-check", action="store_true", help="skip the independent source check")
    ap.add_argument("--source-samples", type=int, default=60)
    ap.add_argument("--source-tol-m", type=float, default=3.0)
    ap.add_argument("--source-level", type=int, help="restrict the source check to one level")
    ap.add_argument("--parent-tol-mean", type=float, default=0.10,
                    help="max mean of |smoothed child - parent| / local relief per level pair (default 0.10)")
    ap.add_argument("--parent-tol-p99", type=float, default=0.4,
                    help="max p99 of the same ratio per level pair (default 0.4)")
    ap.add_argument("--seed", type=int, default=1)
    a = ap.parse_args(argv)
    pack = Pack(a.pack)
    if not os.path.isdir(pack.dir):
        print(f"no terrain directory under {a.pack}", file=sys.stderr)
        return 2
    spots = list(a.spot or []) + ([] if a.no_default_spots else DEFAULT_SPOTS)
    tiles = check_files(pack)
    check_index(pack, tiles)
    check_edges(pack, tiles)
    check_parents(pack, tiles, a.parent_tol_mean, a.parent_tol_p99)
    check_spots(pack, spots, a.source_cache if os.path.isdir(a.source_cache) else None)
    if not a.no_source_check and os.path.isdir(os.path.join(a.source_cache, "glo30")):
        check_source(pack, a.source_cache, a.source_samples, a.source_tol_m, a.seed, a.source_level)
    bad = [r for r in results if not r[1]]
    print(f"\n{len(results) - len(bad)}/{len(results)} checks passed")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
