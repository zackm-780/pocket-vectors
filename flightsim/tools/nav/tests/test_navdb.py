"""Offline unit tests for the nav pipeline. Run from tools/nav:  python3 -m unittest -v"""
from __future__ import annotations

import csv
import io
import math
import sys
import tempfile
import unittest
import zipfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import build_navdb  # noqa: E402
import nasr  # noqa: E402
import navgeo  # noqa: E402
import ourairports as oa  # noqa: E402
import schema  # noqa: E402
import validate_navdb  # noqa: E402
import wmm  # noqa: E402

FIX = HERE / "fixtures"


def rows(path):
    with open(path, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


class GeoTests(unittest.TestCase):
    def test_bearing_cardinals(self):
        self.assertAlmostEqual(navgeo.initial_bearing_true_deg(0, 0, 1, 0), 0.0, 6)
        self.assertAlmostEqual(navgeo.initial_bearing_true_deg(0, 0, 0, 1), 90.0, 6)
        self.assertAlmostEqual(navgeo.initial_bearing_true_deg(1, 0, 0, 0), 180.0, 6)
        self.assertAlmostEqual(navgeo.initial_bearing_true_deg(0, 1, 0, 0), 270.0, 6)

    def test_bearing_convergence_at_high_latitude(self):
        # a long east-west-ish line at 60N: initial bearing is NOT 90
        b = navgeo.initial_bearing_true_deg(60, 0, 60, 10)
        self.assertGreater(b, 80.0)
        self.assertLess(b, 89.0)

    def test_destination_roundtrip(self):
        lat, lon = navgeo.destination(37.0, -122.0, 73.0, 5000.0)
        self.assertAlmostEqual(navgeo.distance_m(37.0, -122.0, lat, lon), 5000.0, 1)
        self.assertAlmostEqual(navgeo.initial_bearing_true_deg(37.0, -122.0, lat, lon), 73.0, 2)


class WmmTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.model = wmm.Wmm()

    def test_official_test_values_within_0_05_deg(self):
        res = wmm.selftest(self.model, tolDeg=0.05)
        self.assertEqual(len(res), 12)
        for r, err, ok in res:
            self.assertTrue(ok, f"{r} {err}")
            self.assertLess(abs(err["declDeg"]), 0.05)

    def test_san_francisco_declination(self):
        d = self.model.declination_deg(37.62, -122.37, 2026.8)
        self.assertGreater(d, 12.0)
        self.assertLess(d, 14.0)

    def test_magvar_bin_roundtrip(self):
        with tempfile.TemporaryDirectory() as td:
            p = Path(td) / "magvar.bin"
            wmm.write_magvar_bin(p, self.model, 2026.5)
            raw = p.read_bytes()
            self.assertEqual(raw[:8], b"MAGVAR01")
            self.assertEqual(len(raw), 24 + 2 * 181 * 361)
            g = wmm.read_magvar_bin(p)
            self.assertEqual(g[1:6], (-90, 90, -180, 180, 1))
            self.assertAlmostEqual(g[0], 2026.5, 3)
            # row order: latMin..latMax, lon -180..180 (first value is lat -90, lon -180)
            v = wmm.lookup_magvar_deg(g, 37.62, -122.37)
            self.assertAlmostEqual(v, self.model.declination_deg(37.62, -122.37, 2026.5), delta=0.15)
            # grid point is exact to the centidegree
            gridVal = g[6][(37 + 90) * 361 + (-122 + 180)] / 100.0
            self.assertAlmostEqual(gridVal, self.model.declination_deg(37, -122, 2026.5), delta=0.006)


class OurAirportsTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.apRows = oa.build_airports(rows(FIX / "airports.csv"))
        cls.ap = {a["ident"]: a for a in cls.apRows}
        cls.rw, cls.stats = oa.build_runways(rows(FIX / "runways.csv"), cls.ap, lambda lat, lon: 10.0)
        cls.byKey = {(r["airport_ident"], r["rwy_ident"]): r for r in cls.rw}

    def test_airport_filter(self):
        self.assertEqual(sorted(self.ap), ["KGRS", "KSML", "KSWP", "KTST"])
        self.assertEqual({a["type"] for a in self.apRows}, {"large", "small", "seaplane"})

    def test_rwy_ident_and_surface_normalization(self):
        self.assertEqual(oa.norm_rwy_ident("1l"), "01L")
        self.assertEqual(oa.norm_rwy_ident("28R"), "28R")
        self.assertEqual(oa.norm_rwy_ident("H1"), "H1")
        for raw, want in [("ASP", "asphalt"), ("ASPH-G", "asphalt"), ("PEM", "asphalt"), ("CONC-F", "concrete"),
                          ("TURF", "grass"), ("GRASS / SOD", "grass"), ("GVL", "gravel"), ("WATER", "water"),
                          ("DIRT", "dirt"), ("ASPH-TURF", "asphalt"), ("TURF-GRVL", "grass"),
                          ("G", "other"), ("", "other"), ("MATS", "other")]:
            self.assertEqual(oa.norm_surface(raw), want, raw)

    def test_exact_runway_both_directions(self):
        a, b = self.byKey[("KTST", "01L")], self.byKey[("KTST", "19R")]
        self.assertEqual(a["opposite_ident"], "19R")
        self.assertEqual(b["opposite_ident"], "01L")
        self.assertEqual(a["approx"], 0)
        self.assertAlmostEqual(float(a["heading_true_deg"]), 0.0, delta=0.01)
        self.assertAlmostEqual(float(b["heading_true_deg"]), 180.0, delta=0.01)
        self.assertEqual(a["surface"], "asphalt")
        self.assertEqual(a["hard"], 1)
        self.assertEqual(a["lighted"], 1)
        # displaced threshold: start moved 500 ft north along the heading
        self.assertEqual(a["displaced_ft"], "500")
        d = navgeo.distance_m(float(a["start_lat_deg"]), float(a["start_lon_deg"]),
                              float(a["thr_lat_deg"]), float(a["thr_lon_deg"])) / navgeo.M_PER_FT
        self.assertAlmostEqual(d, 500.0, delta=1.0)
        self.assertGreater(float(a["thr_lat_deg"]), float(a["start_lat_deg"]))
        # the other direction has no displacement: threshold == start
        self.assertEqual((b["thr_lat_deg"], b["thr_lon_deg"]), (b["start_lat_deg"], b["start_lon_deg"]))
        self.assertEqual((a["end_lat_deg"], a["end_lon_deg"]), (b["start_lat_deg"], b["start_lon_deg"]))
        self.assertEqual(a["thr_elev_ft"], "90")
        self.assertEqual(a["end_elev_ft"], "110")

    def test_synthesized_from_published_heading_centered_on_airport(self):
        a, b = self.byKey[("KTST", "10")], self.byKey[("KTST", "28")]
        self.assertEqual(a["approx"], 1)
        self.assertAlmostEqual(float(a["heading_true_deg"]), 100.0, delta=0.1)
        mid = ((float(a["start_lat_deg"]) + float(a["end_lat_deg"])) / 2,
               (float(a["start_lon_deg"]) + float(a["end_lon_deg"])) / 2)
        self.assertLess(navgeo.distance_m(mid[0], mid[1], 37.0, -122.0), 5.0)
        self.assertAlmostEqual(navgeo.distance_m(float(a["start_lat_deg"]), float(a["start_lon_deg"]),
                                                 float(a["end_lat_deg"]), float(a["end_lon_deg"])) / navgeo.M_PER_FT,
                               5000.0, delta=2.0)
        self.assertEqual(a["surface"], "concrete")
        self.assertEqual((b["start_lat_deg"], b["start_lon_deg"]), (a["end_lat_deg"], a["end_lon_deg"]))

    def test_one_end_known(self):
        a = self.byKey[("KSWP", "13")]
        self.assertEqual((a["start_lat_deg"], a["start_lon_deg"]), ("45", "-120"))
        self.assertEqual(a["approx"], 1)
        self.assertEqual(a["surface"], "water")
        self.assertEqual(a["hard"], 0)
        self.assertAlmostEqual(float(a["heading_true_deg"]), 130.0, delta=0.2)

    def test_heading_from_ident_uses_declination(self):
        a = self.byKey[("KGRS", "09")]  # no coords, no heading: 90 mag + 10 E
        self.assertEqual(a["approx"], 1)
        self.assertAlmostEqual(float(a["heading_true_deg"]), 100.0, delta=0.1)
        self.assertEqual(a["surface"], "grass")
        self.assertEqual(self.stats["heading_from_ident"], 2)  # KGRS 09, KSML turf 09

    def test_ident_heading_can_be_disabled(self):
        rw, st = oa.build_runways(rows(FIX / "runways.csv"), self.ap, lambda a, b: 10.0, useIdentHeading=False)
        self.assertNotIn(("KGRS", "09"), {(r["airport_ident"], r["rwy_ident"]) for r in rw})
        self.assertEqual(st["skip_no_heading"], 3)  # KGRS 09/27, KSML 09/27 (turf dup), KSML N/S

    def test_swapped_ends_are_fixed_by_ident(self):
        # 18/36 with the 18 end listed at the SOUTH (bearing 0 vs expected ~190)
        a = self.byKey[("KSML", "18")]
        self.assertAlmostEqual(float(a["heading_true_deg"]), 180.0, delta=1.0)
        self.assertGreater(float(a["start_lat_deg"]), float(a["end_lat_deg"]))
        self.assertEqual(self.stats["swapped_ends"], 1)

    def test_duplicate_ident_keeps_hard_runway(self):
        a = self.byKey[("KSML", "09")]
        self.assertEqual(a["surface"], "asphalt")
        self.assertEqual(self.stats["skip_duplicate_ident"], 1)
        self.assertEqual(len([k for k in self.byKey if k[0] == "KSML"]), 4)  # 09/27 + 18/36

    def test_skips(self):
        keys = set(self.byKey)
        self.assertNotIn(("KTST", "05"), keys)   # closed
        self.assertFalse(any(k[0] == "KHEL" for k in keys))  # heliport airport excluded
        self.assertFalse(any(k[1] in ("N", "S") for k in keys))  # one-ended without heading
        self.assertEqual(len(self.rw) % 2, 0)

    def test_navaids(self):
        nav = oa.build_navaids(rows(FIX / "navaids.csv"), lambda lat, lon: 12.5)
        by = {n["ident"]: n for n in nav}
        self.assertEqual(sorted(by), ["NDB1", "OAK", "SFO", "TCN"])
        self.assertEqual(by["SFO"]["type"], "VORDME")
        self.assertEqual(by["SFO"]["freq_khz"], "115800")
        self.assertEqual(by["SFO"]["magvar_deg"], "14.42")
        self.assertEqual(by["NDB1"]["magvar_deg"], "12.5")        # WMM fallback
        self.assertEqual(by["NDB1"]["range_nm"], 25)
        self.assertEqual(by["OAK"]["range_nm"], 130)
        self.assertEqual(by["OAK"]["dme_lat_deg"], "37.72")
        self.assertEqual(by["TCN"]["freq_khz"], "")               # -1 => unknown


class NasrTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        raw = rows(FIX / "airports.csv")
        cls.apRows = oa.build_airports(raw)
        cls.idents = {a["ident"] for a in cls.apRows}
        cls.idMap = nasr.build_airport_id_map(raw, cls.idents)

    def test_airport_id_map(self):
        self.assertEqual(nasr.map_airport("TST", self.idMap, self.idents), "KTST")
        self.assertIsNone(nasr.map_airport("ZZZ", self.idMap, self.idents))

    def test_ils(self):
        ils, st = nasr.build_ils(rows(FIX / "nasr/ILS_BASE.csv"), rows(FIX / "nasr/ILS_GS.csv"),
                                 rows(FIX / "nasr/ILS_DME.csv"), self.idMap, self.idents)
        self.assertEqual(len(ils), 1)
        r = ils[0]
        self.assertEqual((r["airport_ident"], r["rwy_ident"], r["loc_ident"]), ("KTST", "01L", "ITS"))
        self.assertEqual(r["freq_mhz"], "109.55")
        self.assertEqual(r["loc_course_true_deg"], "340")   # 350 mag, 10 W => 340 true
        self.assertEqual((r["has_gs"], r["gs_angle_deg"], r["gs_elev_ft"]), (1, "2.85", "92.5"))
        self.assertEqual(r["dme_lat_deg"], "36.9801")
        self.assertEqual(r["category"], "IIIB")
        self.assertEqual(st["skip_not_operational"], 1)
        self.assertEqual(st["skip_unknown_airport"], 1)
        self.assertEqual(list(r), schema.ILS_COLS)

    def test_fixes_and_airways(self):
        fixes = nasr.build_fixes(rows(FIX / "nasr/FIX_BASE.csv"))
        self.assertEqual(len(fixes), 4)
        nav = oa.build_navaids(rows(FIX / "navaids.csv"))
        aw, st = nasr.build_airways(rows(FIX / "nasr/AWY_SEG_ALT.csv"), fixes, nav)
        c = [(r["airway"], r["seq"], r["fix_ident"], r["mea_ft"]) for r in aw]
        self.assertEqual(c, [("V9", 1, "ALPHA", "3000"), ("V9", 2, "BRAVO", "4000"), ("V9", 3, "SFO", "5000"),
                             ("V9", 4, "CHRLI", ""), ("V9", 1000 + 1, "CHRLI", "6000"),
                             ("V9", 1000 + 2, "ALPHA", "")])
        bravo = aw[1]
        self.assertEqual((bravo["fix_lat_deg"], bravo["fix_lon_deg"]), ("37.6", "-122"))  # K2, not K6
        self.assertEqual(aw[2]["fix_lat_deg"], "37.6195")  # resolved from navaids

    def test_source_finds_nested_zips_and_dirs(self):
        with tempfile.TemporaryDirectory() as td:
            inner = io.BytesIO()
            with zipfile.ZipFile(inner, "w") as z:
                for n in ("ILS_BASE.csv", "ILS_GS.csv"):
                    z.write(FIX / "nasr" / n, n)
            outer = Path(td) / "all.zip"
            with zipfile.ZipFile(outer, "w") as z:
                z.writestr("01_Oct_2026_ILS_CSV.zip", inner.getvalue())
                z.write(FIX / "nasr/FIX_BASE.csv", "sub/FIX_BASE.csv")
            src = nasr.NasrSource(outer)
            self.assertTrue(src.has("ils_base.csv"))
            self.assertTrue(src.has("FIX_BASE.csv"))
            self.assertFalse(src.has("AWY_SEG_ALT.csv"))
            self.assertEqual(src.effective_date(), "2026-10-01")
            self.assertTrue(nasr.NasrSource(FIX / "nasr").has("AWY_SEG_ALT.csv"))

    def test_cycle_helpers(self):
        self.assertEqual(nasr.csv_zip_url("2026-10-01", "ILS"),
                         "https://nfdc.faa.gov/webContent/28DaySub/extra/01_Oct_2026_ILS_CSV.zip")
        html = 'a href="./../NASR_Subscription/2026-10-01" b href="./../NASR_Subscription/2026-10-29"'
        dates = nasr.parse_cycle_dates(html)
        self.assertEqual(dates, ["2026-10-01", "2026-10-29"])
        self.assertEqual(nasr.pick_current_cycle(dates, "2026-10-19"), "2026-10-01")
        self.assertEqual(nasr.pick_current_cycle(dates, "2026-10-29"), "2026-10-29")
        self.assertIsNone(nasr.pick_current_cycle(dates, "2026-09-01"))


class EndToEndTests(unittest.TestCase):
    """Runs build() entirely offline from a cache populated with the fixtures."""

    def test_build_offline_validates(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            cache = td / "cache"
            cache.mkdir()
            for n in ("airports.csv", "runways.csv", "navaids.csv"):
                (cache / n).write_bytes((FIX / n).read_bytes())
            nz = td / "nasr.zip"
            with zipfile.ZipFile(nz, "w") as z:
                for p in (FIX / "nasr").glob("*.csv"):
                    z.write(p, p.name)
            old = build_navdb.CACHE
            build_navdb.CACHE = cache
            try:
                out = td / "nav"
                res = build_navdb.build(out, str(nz), True, "2026-10-19T00:00:00Z", quiet=True)
            finally:
                build_navdb.CACHE = old
            self.assertEqual(res["airports"], 4)
            ini = (out / "nav.ini").read_text()
            self.assertIn("format = flightsim-nav", ini)
            self.assertIn("faa_nasr = 2026-10-01", ini)
            self.assertIn("wmm = WMM2025, epoch used 2026.797", ini)
            rep = validate_navdb.validate(out)
            self.assertEqual(rep.errors, [], rep.errors)
            ap = {a["ident"]: a for a in rows(out / "airports.csv")}
            self.assertEqual(ap["KTST"]["has_ils"], "1")
            self.assertEqual(ap["KGRS"]["has_ils"], "0")
            self.assertTrue((out / "ils.csv").exists())
            self.assertTrue((out / "airways.csv").exists())

    def test_validator_catches_bad_data(self):
        with tempfile.TemporaryDirectory() as td:
            td = Path(td)
            (td / "airports.csv").write_text(",".join(schema.AIRPORTS_COLS) + "\nX,heli,n,95,0,0,US,US-CA,m,,\n")
            rep = validate_navdb.validate(td)
            self.assertTrue(any("bad type" in e for e in rep.errors))
            self.assertTrue(any("lat_deg" in e for e in rep.errors))
            self.assertTrue(any("nav.ini missing" in e for e in rep.errors))
            self.assertTrue(any("magvar.bin missing" in e for e in rep.errors))


if __name__ == "__main__":
    unittest.main()
