"""Unit tests for the terrain pipeline. Synthetic data only: no network, no real DEM.

Run:  cd flightsim/tools/terrain && .venv/bin/python -m unittest discover -s tests -v
"""
import os
import sys
import tempfile
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from fsterrain import dem, landcover as lc                      # noqa: E402
from fsterrain.pack import update_bitmaps, update_pack_ini      # noqa: E402
from fsterrain.tilegrid import (H16_BYTES, SAMPLES, Tile, bitmap_bytes, bitmap_tiles,  # noqa: E402
                                get_bit, read_bitmap, set_bit, step_deg, tiles_for_bbox,
                                tiles_x)


class FakeSources:
    """Pretends a set of 1-degree cells exists; the reader below synthesizes their pixels."""

    def __init__(self, cells):
        self.cells = set(cells)

    def get_dem(self, product, lat0, lon0):
        return f"fake:{lat0}:{lon0}" if (lat0, lon0) in self.cells else None


def plane_reader(n, fn):
    """Reader of a synthetic point-lattice cell with n x n pixels: value = fn(lat, lon)."""
    def read(path, lat0, lon0):
        j = np.arange(n)[:, None]
        i = np.arange(n)[None, :]
        lat = lat0 + 1 - j / n
        lon = lon0 + i / n
        return fn(lat + 0 * lon, lon + 0 * lat).astype(np.float32), n, n, 0.0, 0.0
    return read


class TileGridTests(unittest.TestCase):
    def test_sample_positions_exact_and_shared(self):
        for k in (0, 3, 7, 10):
            t = Tile(k, 5 % tiles_x(k), 1)
            e = Tile(k, (t.x + 1) % tiles_x(k), 1)
            n = Tile(k, t.x, 2)
            self.assertEqual(t.lon_samples_deg()[256], e.lon_samples_deg()[0])
            self.assertEqual(t.lat_samples_deg()[0], n.lat_samples_deg()[256])
            self.assertEqual(t.lat_samples_deg()[0], t.north_deg)
            self.assertEqual(t.lon_samples_deg()[0], t.west_deg)
            self.assertAlmostEqual(t.lon_samples_deg()[1] - t.lon_samples_deg()[0], step_deg(k), 15)

    def test_antimeridian_wrap(self):
        k = 4
        last = Tile(k, tiles_x(k) - 1, 3)
        first = Tile(k, 0, 3)
        self.assertEqual(last.lon_samples_deg()[256], first.lon_samples_deg()[0])  # both -180

    def test_levels_table(self):
        self.assertAlmostEqual(step_deg(0), 0.3515625)
        self.assertAlmostEqual(step_deg(10), 0.000343323, 8)
        self.assertEqual(H16_BYTES, 132098)
        self.assertEqual(bitmap_bytes(10), 1 << 20)

    def test_bbox_tiles(self):
        ts = tiles_for_bbox(10, (-121.85, 45.25, -121.55, 45.50))
        self.assertTrue(all(t.k == 10 for t in ts))
        # every tile overlaps the bbox, and the bbox corners are covered
        for t in ts:
            self.assertTrue(t.east_deg > -121.85 and t.west_deg < -121.55)
        self.assertTrue(any(t.west_deg <= -121.85 < t.east_deg for t in ts))
        self.assertEqual(len(tiles_for_bbox(0, (-124.7, 43.0, -121.4, 46.3))), 1)


class BitmapTests(unittest.TestCase):
    def test_merge_on_update(self):
        with tempfile.TemporaryDirectory() as d:
            for t in (Tile(3, 1, 1), Tile(3, 7, 4)):
                p = os.path.join(d, t.rel_path_h())
                os.makedirs(os.path.dirname(p), exist_ok=True)
                with open(p, "wb") as f:
                    f.write(b"\0" * H16_BYTES)
            update_bitmaps(d, [Tile(3, 1, 1)])
            update_bitmaps(d, [Tile(3, 7, 4), Tile(3, 2, 2)])    # (3,2,2) has no file: not set
            bm = read_bitmap(os.path.join(d, "index", "L3.bit"), 3)
            self.assertEqual(len(bm), bitmap_bytes(3))
            self.assertEqual(set(bitmap_tiles(bm, 3)), {Tile(3, 1, 1), Tile(3, 7, 4)})
            self.assertTrue(get_bit(bm, Tile(3, 7, 4)))
            # bit i = y * 4*2^k + x, LSB first
            i = 1 * 32 + 1
            self.assertTrue(bm[i >> 3] >> (i & 7) & 1)

    def test_pack_ini_merge(self):
        with tempfile.TemporaryDirectory() as d:
            update_pack_ini(d, 0, 6, "world", "levels 0-6", False)
            r = update_pack_ini(d, 7, 10, "pdx_oth", "bbox -124.7,43.0,-121.4,46.3 levels 7-10", True)
            self.assertEqual(r["pack"]["min_level"], "0")
            self.assertEqual(r["pack"]["max_level"], "10")
            self.assertEqual(set(r["regions"]), {"world", "pdx_oth"})
            self.assertIn("CC BY 4.0", r["attribution"]["landcover"])
            self.assertIn("Copernicus", r["attribution"]["dem"])


class ResampleTests(unittest.TestCase):
    N = 1800

    def store(self, fn, cells, **kw):
        return dem.CellStore(FakeSources(cells), reader=plane_reader(self.N, fn), **kw)

    def test_plane_is_reproduced_across_cell_borders(self):
        # a tile that straddles lat 37 / lon -122: the box average of a plane equals the plane
        fn = lambda la, lo: 100.0 + 300.0 * (la - 37.0) - 200.0 * (lo + 122.0)   # noqa: E731
        cells = {(la, lo) for la in (36, 37, 38) for lo in (-123, -122, -121)}
        for k in (6, 8):                      # k=6 exercises the block reduction (f=2), k=8 f=1
            st = self.store(fn, cells)
            t = Tile(k, int((-122 + 180) / (90 / 2 ** k)), int((37 + 90) / (90 / 2 ** k)))
            h = dem.height_tile(st, t)
            lat = t.lat_samples_deg()[:, None]
            lon = t.lon_samples_deg()[None, :]
            expect = fn(lat, lon)
            self.assertLess(np.abs(h - expect).max(), 1e-3, f"level {k}")

    def test_shared_edges_bit_identical_with_noise(self):
        rng = np.random.default_rng(3)
        noise = {}

        def reader(path, lat0, lon0):
            key = (lat0, lon0)
            if key not in noise:
                noise[key] = rng.normal(500, 80, (self.N, self.N)).astype(np.float32)
            return noise[key], self.N, self.N, 0.0, 0.0
        cells = {(la, lo) for la in (36, 37, 38) for lo in (-123, -122, -121)}
        st = dem.CellStore(FakeSources(cells), reader=reader)
        for k in (6, 7):
            t = Tile(k, int((-122 + 180) / (90 / 2 ** k)) - 1, int((37 + 90) / (90 / 2 ** k)))
            e = Tile(k, t.x + 1, t.y)
            n = Tile(k, t.x, t.y + 1)
            a, b, c = (dem.heights_to_int16(dem.height_tile(st, q)) for q in (t, e, n))
            self.assertTrue((a[:, 256] == b[:, 0]).all(), "east edge")
            self.assertTrue((a[0, :] == c[256, :]).all(), "north edge")
            # a fresh store (different cache state, other tile order) gives the same values
            st2 = dem.CellStore(FakeSources(cells), reader=reader)
            self.assertTrue((dem.heights_to_int16(dem.height_tile(st2, e)) == b).all())

    def test_missing_cells_are_sea_level(self):
        fn = lambda la, lo: 50.0 + 0 * la    # noqa: E731
        st = self.store(fn, {(37, -122)})
        t = Tile(8, int((-122 + 180) / (90 / 256)), int((37 + 90) / (90 / 256)))
        h = dem.height_tile(st, t)
        self.assertTrue(((h > 0) & (h <= 50.0001)).any())
        far = Tile(8, 10, 10)
        self.assertEqual(dem.height_tile(self.store(fn, set()), far).max(), 0.0)

    def test_box_average_uses_footprint(self):
        # a one-pixel spike must not survive coarse sampling the way point decimation would keep it
        def reader(path, lat0, lon0):
            a = np.zeros((self.N, self.N), dtype=np.float32)
            a[900, 901] = 1000.0
            return a, self.N, self.N, 0.0, 0.0
        st = dem.CellStore(FakeSources({(37, -122)}), reader=reader)
        t = Tile(6, int((-122 + 180) / (90 / 64)), int((37 + 90) / (90 / 64)))
        h = dem.height_tile(st, t)
        self.assertLess(h.max(), 400.0)
        w = dem.step_deg(6) * self.N                      # box width in source pixels
        # partition of unity: every source pixel's mass is distributed over the samples exactly once
        self.assertAlmostEqual(h.sum() * w * w / 1000.0, 1.0, delta=0.02)

    def test_taps_sum_to_one(self):
        for w in (0.7, 1.235, 4.94):
            idx, wts = dem.axis_taps(np.array([10.0, 10.3, 11.5]), w)
            self.assertTrue(np.allclose(wts.sum(axis=1), 1.0))

    def test_void_fill(self):
        a = np.full((60, 60), 100.0, dtype=np.float32)
        a[20:30, 20:30] = np.nan
        a[0, 0] = -32767.0
        f = dem.fill_voids(a)
        self.assertTrue(np.isfinite(f).all())
        self.assertAlmostEqual(float(f[25, 25]), 100.0, delta=1.0)
        self.assertAlmostEqual(float(f[0, 0]), 100.0, delta=1.0)
        self.assertTrue((dem.fill_voids(np.full((8, 8), np.nan, dtype=np.float32)) == 0).all())

    def test_int16_rounding_and_clamp(self):
        h = np.array([-1000.0, -499.6, 0.49, 0.5, 1.5, 9000.4, 12000.0])
        self.assertEqual(dem.heights_to_int16(h).tolist(), [-500, -500, 0, 1, 2, 9000, 9000])
        self.assertEqual(dem.heights_to_int16(h).dtype, np.dtype("<i2"))


class LandClassTests(unittest.TestCase):
    def counts(self, **kw):
        c = np.zeros((1, 16), dtype=np.int64)
        for k, v in kw.items():
            c[0, int(k[1:])] = v
        return c

    def resolve(self, ocean=False, **kw):
        c = self.counts(**kw)
        return int(lc.resolve_cells(c[:, :12], c[:, 15], np.array([ocean]))[0])

    def test_mode_and_water_rule(self):
        self.assertEqual(self.resolve(c2=6, c4=4), lc.TREE)
        self.assertEqual(self.resolve(c1=5, c2=3, c4=2), lc.WATER)       # exactly 50% water
        self.assertEqual(self.resolve(c1=4, c2=3, c4=3), lc.WATER)       # water is also the mode
        self.assertEqual(self.resolve(c1=3, c2=4, c4=3), lc.TREE)        # 30% water, tree is the mode
        self.assertEqual(self.resolve(c1=4, c2=4, c4=2), lc.WATER)       # tie -> lowest class id
        self.assertEqual(self.resolve(c3=2, c4=2), lc.SHRUB)             # tie -> lowest id

    def test_nodata_rule(self):
        self.assertEqual(self.resolve(ocean=True, c15=10), lc.WATER)     # no data, height <= 0
        self.assertEqual(self.resolve(ocean=False, c15=10), lc.UNKNOWN)  # no data on land
        self.assertEqual(self.resolve(ocean=False, c15=6, c2=4), lc.TREE)

    def test_wc_mapping(self):
        m = lc.WC_TO_CLASS
        self.assertEqual([int(m[v]) for v in (10, 20, 30, 40, 50, 60, 70, 80, 90, 95, 100)],
                         [2, 3, 4, 5, 6, 7, 8, 1, 9, 10, 11])
        self.assertEqual(int(m[0]), lc.NODATA_BUCKET)

    def test_derive_from_children(self):
        z = np.zeros((256, 256), dtype=np.uint8)
        sw, se, nw, ne = (z + lc.TREE, z + lc.WATER, z + lc.WATER, z + lc.TREE)
        out = lc.derive_from_children([sw, se, nw, ne])
        # north-up mosaic: NW quadrant is rows 0..255 / cols 0..255 of the 512 grid, so the parent
        # cells are mixed only where child cells differ; here every 2x2 block is uniform
        self.assertTrue((out[:128, :128] == lc.WATER).all())     # NW child
        self.assertTrue((out[:128, 128:] == lc.TREE).all())      # NE child
        self.assertTrue((out[128:, :128] == lc.TREE).all())      # SW child
        self.assertTrue((out[128:, 128:] == lc.WATER).all())     # SE child
        # 2 of 4 water -> water; 1 of 4 water -> majority class
        m = np.zeros((512, 512), dtype=np.uint8) + lc.GRASS
        m[0, 0] = m[0, 1] = lc.WATER
        m[2, 2] = lc.WATER
        ch = [m[256:, :256], m[256:, 256:], m[:256, :256], m[:256, 256:]]
        o = lc.derive_from_children([c.copy() for c in ch])
        self.assertEqual(int(o[0, 0]), lc.WATER)
        self.assertEqual(int(o[1, 1]), lc.GRASS)


if __name__ == "__main__":
    unittest.main()
