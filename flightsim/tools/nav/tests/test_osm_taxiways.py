"""Offline tests for osm_taxiways.py (WP W8a). Run from tools/nav:
    python3 -m unittest -v tests.test_osm_taxiways
"""
from __future__ import annotations

import contextlib
import csv
import io
import tempfile
import unittest
from pathlib import Path
from unittest import mock

import sys

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import osm_taxiways as ot  # noqa: E402

FIX = HERE / "fixtures" / "osm"

AIRPORTS = """ident,type,name,lat_deg,lon_deg,elev_ft,iso_country,iso_region,municipality,iata,has_ils
KOTH,medium,Southwest Oregon Regional Airport,43.417099,-124.246002,17,US,US-OR,North Bend,OTH,1
XEDG,small,Edge Case Field,10.0,20.0,100,ZZ,ZZ-1,Nowhere,,0
XGRS,small,Grass Only,10.5,20.5,100,ZZ,ZZ-1,Nowhere,,0
"""
RUNWAYS = """airport_ident,rwy_ident,opposite_ident,thr_lat_deg,thr_lon_deg,thr_elev_ft,start_lat_deg,start_lon_deg,end_lat_deg,end_lon_deg,end_elev_ft,heading_true_deg,length_ft,width_ft,displaced_ft,surface,hard,lighted,approx
KOTH,05,23,43.4147,-124.261002,16,43.4147,-124.261002,43.421902,-124.242996,13,61.15,5980,150,0,asphalt,1,1,0
KOTH,13,31,43.4198,-124.246002,13,43.4198,-124.246002,43.409302,-124.237999,17,151.02,4470,150,0,asphalt,1,1,0
KOTH,23,05,43.421902,-124.242996,13,43.421902,-124.242996,43.4147,-124.261002,16,241.17,5980,150,0,asphalt,1,1,0
KOTH,31,13,43.409302,-124.237999,17,43.409302,-124.237999,43.4198,-124.246002,13,331.03,4470,150,0,asphalt,1,1,0
XEDG,09,27,10.0,19.995,100,10.0,19.995,10.0,20.005,100,90,3600,100,0,asphalt,1,1,0
XEDG,27,09,10.0,20.005,100,10.0,20.005,10.0,19.995,100,270,3600,100,0,asphalt,1,1,0
XGRS,09,27,10.5,20.495,100,10.5,20.495,10.5,20.505,100,90,3600,100,0,grass,0,0,0
"""
NAV_INI = "[nav]\nformat = flightsim-nav\nversion = 1\n[sources]\nourairports = 2026-11-01\n"


def rows(path: Path) -> list[dict]:
    with open(path, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def points(row: dict) -> list[tuple[float, float]]:
    v = [float(x) for x in row["points"].split()]
    return list(zip(v[0::2], v[1::2]))


class NavDirCase(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory()
        self.nav = Path(self.tmp.name)
        (self.nav / "airports.csv").write_text(AIRPORTS, encoding="utf-8")
        (self.nav / "runways.csv").write_text(RUNWAYS, encoding="utf-8")
        (self.nav / "nav.ini").write_text(NAV_INI, encoding="utf-8")

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def run_tool(self, *args: str) -> tuple[int, str]:
        out = io.StringIO()
        with contextlib.redirect_stdout(out), contextlib.redirect_stderr(io.StringIO()):
            code = ot.run(["--nav-dir", str(self.nav), "--date", "2026-11-08", *args])
        return code, out.getvalue()


class EdgeCaseTests(NavDirCase):
    def setUp(self) -> None:
        super().setUp()
        self.code, self.log = self.run_tool("--osm-file", str(FIX / "edge_cases.osm"))
        self.tax = rows(self.nav / "taxiways.csv")
        self.apr = rows(self.nav / "aprons.csv")

    def by_way(self, wid: int) -> list[dict]:
        return [r for r in self.tax if r["osm_way_id"] == str(wid)]

    def test_exit_code_and_summary(self):
        self.assertEqual(self.code, 0)
        self.assertIn("1 OSM files read; 10 taxiway rows, 1 aprons, 1 airports", self.log)
        self.assertIn("1 apron relations skipped", self.log)

    def test_filters(self):
        ways = {r["osm_way_id"] for r in self.tax}
        self.assertNotIn("104", ways)  # disused=yes
        self.assertNotIn("105", ways)  # area=yes
        self.assertNotIn("111", ways)  # outside every airport box
        self.assertIn("110", ways)  # construction=no is live
        self.assertEqual([r["osm_way_id"] for r in self.apr], ["108"])  # 107 is not closed

    def test_split_at_shared_nodes(self):
        a = self.by_way(101)  # nodes 1-2-3-4; 2 shared with C, 3 shared with B
        self.assertEqual(len(a), 3)
        self.assertEqual([len(points(r)) for r in a], [2, 2, 2])
        self.assertEqual(points(a[0])[-1], points(a[1])[0])
        self.assertEqual(points(a[1])[-1], points(a[2])[0])
        self.assertEqual(points(a[0])[0], (10.003, 19.996))
        self.assertEqual(points(a[2])[-1], (10.003, 20.002))
        self.assertEqual(len(self.by_way(102)), 1)  # ends on A's middle node: not split itself
        c = self.by_way(103)  # crosses A at node 2
        self.assertEqual(len(c), 2)
        self.assertEqual(points(c[0])[-1], (10.003, 19.998))

    def test_split_where_a_way_visits_a_node_twice(self):
        loop = self.by_way(110)  # 24-25-26-24-27
        self.assertEqual([len(points(r)) for r in loop], [4, 2])
        self.assertEqual(points(loop[0])[0], points(loop[0])[-1])

    def test_widths_and_surfaces(self):
        self.assertEqual(self.by_way(101)[0]["width_m"], "10.5")  # small airport default
        self.assertEqual(self.by_way(106)[0]["width_m"], "22.9")  # width=75 ft
        lane = self.by_way(109)[0]
        self.assertEqual((lane["kind"], lane["width_m"], lane["surface"]),
                         ("taxilane", "7.5", "concrete"))
        self.assertEqual(self.by_way(101)[0]["surface"], "asphalt")  # the runway's surface
        self.assertEqual(self.by_way(101)[0]["ref"], "A")
        self.assertEqual(self.by_way(106)[0]["ref"], "")

    def test_csv_format(self):
        text = (self.nav / "taxiways.csv").read_text(encoding="utf-8")
        self.assertTrue(text.startswith("airport_ident,osm_way_id,kind,ref,width_m,surface,points\n"))
        self.assertIn("XEDG,101,taxiway,A,10.5,asphalt,10.003000 19.996000 10.003000 19.998000\n",
                      text)
        apron = self.apr[0]
        self.assertEqual(apron["surface"], "concrete")
        ring = points(apron)
        self.assertEqual(len(ring), 4)  # open ring: the first point is not repeated
        self.assertNotEqual(ring[0], ring[-1])
        self.assertEqual((self.nav / "aprons.csv").read_text().splitlines()[0],
                         "airport_ident,osm_way_id,surface,points")

    def test_rows_sorted_and_deterministic(self):
        first = (self.nav / "taxiways.csv").read_bytes()
        keys = [(r["airport_ident"], int(r["osm_way_id"])) for r in self.tax]
        self.assertEqual(keys, sorted(keys))
        self.run_tool("--osm-file", str(FIX / "edge_cases.osm"))
        self.assertEqual((self.nav / "taxiways.csv").read_bytes(), first)

    def test_licence_note_and_nav_ini(self):
        note = (self.nav / "TAXIWAYS-ODbL.txt").read_text(encoding="utf-8")
        self.assertIn("© OpenStreetMap contributors", note)
        self.assertIn("2026-11-08", note)
        self.assertIn("https://opendatacommons.org/licenses/odbl/1-0/", note)
        ini = (self.nav / "nav.ini").read_text()
        self.assertIn("[sources]\nosm_taxiways = 2026-11-08\nourairports", ini)
        self.run_tool("--osm-file", str(FIX / "edge_cases.osm"))
        self.assertEqual((self.nav / "nav.ini").read_text().count("osm_taxiways"), 1)


class KothFixtureTests(NavDirCase):
    def test_koth(self):
        code, log = self.run_tool("--airports", "KOTH", "--osm-file", str(FIX / "koth_aeroway.osm"))
        self.assertEqual(code, 0)
        tax = rows(self.nav / "taxiways.csv")
        apr = rows(self.nav / "aprons.csv")
        self.assertEqual(len({r["osm_way_id"] for r in tax}), 17)  # 19 ways, 2 are area=yes
        self.assertEqual(len(apr), 7)
        self.assertEqual(len(tax), 36)
        self.assertTrue(all(r["airport_ident"] == "KOTH" for r in tax + apr))
        self.assertTrue(all(r["width_m"] == "15.0" for r in tax))  # medium, no width tags
        self.assertTrue(all(r["kind"] == "taxiway" and r["ref"] == "" for r in tax))
        for r in tax:
            for lat, lon in points(r):
                self.assertTrue(43.39 < lat < 43.44 and -124.28 < lon < -124.22)
        self.assertIn("KOTH 36 rows", log)


class UnitTests(unittest.TestCase):
    def test_parse_width(self):
        self.assertEqual(ot.parse_width_m("23"), 23.0)
        self.assertEqual(ot.parse_width_m("23 m"), 23.0)
        self.assertAlmostEqual(ot.parse_width_m("75 ft"), 22.86)
        self.assertAlmostEqual(ot.parse_width_m("75'"), 22.86)
        self.assertEqual(ot.parse_width_m("22,5"), 22.5)
        self.assertIsNone(ot.parse_width_m("2"))  # below 3 m
        self.assertIsNone(ot.parse_width_m("120"))  # above 80 m
        self.assertIsNone(ot.parse_width_m("wide"))
        self.assertIsNone(ot.parse_width_m(None))

    def test_default_widths(self):
        self.assertEqual(ot.default_width_m("taxiway", "large"), 23.0)
        self.assertEqual(ot.default_width_m("taxiway", "medium"), 15.0)
        self.assertEqual(ot.default_width_m("taxiway", "small"), 10.5)
        self.assertEqual(ot.default_width_m("taxilane", "large"), 7.5)

    def test_surface(self):
        self.assertEqual(ot.surface_of({"surface": "paved"}, "grass"), "asphalt")
        self.assertEqual(ot.surface_of({"surface": "concrete:plates"}, "grass"), "concrete")
        self.assertEqual(ot.surface_of({"surface": "grass"}, "asphalt"), "grass")
        self.assertEqual(ot.surface_of({}, "concrete"), "concrete")
        self.assertEqual(ot.surface_of({"surface": "weird"}, "asphalt"), "asphalt")

    def test_query_uses_the_box(self):
        box = ot.AirportBox("KXYZ", "small", 1, 2, "asphalt", south=1.0, west=2.0, north=1.5,
                            east=2.5)
        q = ot.overpass_query(box)
        self.assertIn('way["aeroway"~"^(taxiway|taxilane|apron)$"](1.000000,2.000000,1.500000,2.500000)', q)
        self.assertIn("out body;>;out skel qt;", q)


class AttributionAndFetchTests(NavDirCase):
    def test_attribution_line_added_once(self):
        attr = self.nav / "ATTRIBUTION.txt"
        attr.write_text("flightsim data pack\nElevation: Copernicus\n", encoding="utf-8")
        self.run_tool("--osm-file", str(FIX / "edge_cases.osm"), "--attribution", str(attr))
        self.run_tool("--osm-file", str(FIX / "edge_cases.osm"), "--attribution", str(attr))
        text = attr.read_text(encoding="utf-8")
        self.assertEqual(text.count("OpenStreetMap"), 1)
        self.assertIn(ot.ATTRIBUTION_LINE, text)

    def test_offline_uses_cache_and_exits_1_without_answers(self):
        cache = self.nav / "cache"
        code, log = self.run_tool("--airports", "XEDG", "--offline", "--cache-dir", str(cache))
        self.assertEqual(code, 1)
        self.assertIn("0/1 airports answered, 1 failed", log)
        cache.mkdir()
        (cache / "XEDG-20261101.osm").write_bytes((FIX / "edge_cases.osm").read_bytes())
        code, log = self.run_tool("--airports", "XEDG", "--offline", "--cache-dir", str(cache))
        self.assertEqual(code, 0)
        self.assertIn("1/1 airports answered, 0 failed; 10 taxiway rows", log)

    def test_region_filter_and_refusal(self):
        boxes = ot.read_airports(self.nav, None, 1500.0, ot.parse_bbox("19,9,21,11"))
        self.assertEqual([b.ident for b in boxes], ["XEDG"])  # KOTH is outside the region
        with self.assertRaises(ValueError):
            ot.parse_bbox("21,9,19,11")
        # Querying Overpass needs a region or a subset: airports.csv is worldwide.
        with mock.patch.object(ot.urllib.request, "urlopen") as op:
            code, _ = self.run_tool("--cache-dir", str(self.nav / "cache"))
            self.assertEqual(code, 2)
            self.assertEqual(op.call_count, 0)
        cache = self.nav / "cache"
        cache.mkdir()
        (cache / "XEDG-20261101.osm").write_bytes((FIX / "edge_cases.osm").read_bytes())
        code, log = self.run_tool("--bbox=19,9,21,11", "--offline", "--cache-dir", str(cache))
        self.assertEqual(code, 0)
        self.assertIn("1/1 airports answered, 0 failed (region 19,9,21,11); 10 taxiway rows", log)

    def test_only_airports_with_a_hard_runway(self):
        boxes = ot.read_airports(self.nav, None, 1500.0)
        self.assertEqual([b.ident for b in boxes], ["KOTH", "XEDG"])  # XGRS is grass only
        koth = boxes[0]
        self.assertLess(koth.south, 43.4093 - 0.013)
        self.assertGreater(koth.east, -124.2380 + 0.018)

    def test_stops_early_when_overpass_is_down(self):
        # Seven airports, every request fails: it gives up after the first five.
        rows = "\n".join(f"XA{i:02d},small,A{i},{10 + i},20,100,ZZ,ZZ-1,X,,0" for i in range(7))
        rwys = "\n".join(f"XA{i:02d},09,27,{10 + i},19.995,100,{10 + i},19.995,{10 + i},20.005,100,"
                         f"90,3600,100,0,asphalt,1,1,0" for i in range(7))
        (self.nav / "airports.csv").write_text(AIRPORTS.splitlines()[0] + "\n" + rows + "\n")
        (self.nav / "runways.csv").write_text(RUNWAYS.splitlines()[0] + "\n" + rwys + "\n")
        err = ot.urllib.error.URLError("unreachable")
        with mock.patch.object(ot.urllib.request, "urlopen", side_effect=err) as op, \
                mock.patch.object(ot.time, "sleep"):
            code, log = self.run_tool("--bbox=19,9,21,17", "--cache-dir", str(self.nav / "c"),
                                      "--sleep", "0")
        self.assertEqual(code, 1)
        self.assertIn("the first 5 airports all failed; stopping", log)
        self.assertEqual(op.call_count, 5 * (len(ot.RETRY_DELAYS_S) + 1))

    def test_overpass_answer_is_cached(self):
        cache = self.nav / "cache"
        answer = (FIX / "edge_cases.osm").read_bytes()

        class Resp(io.BytesIO):
            def __enter__(self):
                return self

            def __exit__(self, *a):
                return False

        with mock.patch.object(ot.urllib.request, "urlopen", return_value=Resp(answer)) as op:
            code, log = self.run_tool("--airports", "XEDG", "--cache-dir", str(cache),
                                      "--sleep", "0")
            self.assertEqual(op.call_count, 1)
            req = op.call_args[0][0]
            self.assertIn(b"aeroway", req.data)
            self.assertIn("pocket-vectors-datapack", req.get_header("User-agent"))
        self.assertEqual(code, 0)
        self.assertTrue((cache / "XEDG-20261108.osm").is_file())
        with mock.patch.object(ot.urllib.request, "urlopen") as op:  # same day: from the cache
            self.run_tool("--airports", "XEDG", "--cache-dir", str(cache), "--sleep", "0")
            self.assertEqual(op.call_count, 0)


if __name__ == "__main__":
    unittest.main()
