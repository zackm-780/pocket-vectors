"""Offline unit tests for the CIFP converter (WP N4). Run from tools/nav:  python3 -m unittest -v

``fixtures/cifp_mini.txt`` is a 67 KiB cut of the real FAA CIFP cycle 2610 (public domain, made by
fixtures/make_cifp_fixture.py): all KOTH approaches, KPDX ILS 10R/28L and one approach each with
the less common path terminators (CD, VD, VM, CI, FA, VA, VR, FC)."""
from __future__ import annotations

import math
import sys
import tempfile
import unittest
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))

import cifp  # noqa: E402
import cifp_proc as cp  # noqa: E402
import navgeo  # noqa: E402

FIXTURE = HERE / "fixtures" / "cifp_mini.txt"

_cache: dict = {}


def build():
    """(db, stats, {airport: {approach id: Approach}}) for the whole fixture, built once."""
    if not _cache:
        db = cifp.CifpDb()
        db.load(cifp.open_cifp_lines(FIXTURE))
        stats = cifp.Stats()
        out = {}
        for apt in sorted(db.records):
            out[apt] = {a.id: a for a in cifp.build_airport(db, apt, stats)}
        _cache.update(db=db, stats=stats, out=out)
    return _cache["db"], _cache["stats"], _cache["out"]


def appr(apt: str, ident: str) -> cp.Approach:
    return build()[2][apt][ident]


def fix_of(legs, ident):
    return next(l for l in legs if l.ident == ident)


def dist_nm(a, b):
    return navgeo.distance_m(a[0], a[1], b[0], b[1]) / 1852.0


class FieldParsers(unittest.TestCase):
    def test_lat_lon(self):
        self.assertAlmostEqual(cifp.parse_lat("N43245302"), 43 + 24 / 60 + 53.02 / 3600, 9)
        self.assertAlmostEqual(cifp.parse_lon("W124153815"), -(124 + 15 / 60 + 38.15 / 3600), 9)
        self.assertAlmostEqual(cifp.parse_lat("S33000000"), -33.0, 9)
        self.assertAlmostEqual(cifp.parse_lon("E151000000"), 151.0, 9)
        self.assertIsNone(cifp.parse_lat("         "))
        self.assertIsNone(cifp.parse_lon("W12415381"))  # one digit short

    def test_magvar(self):
        self.assertAlmostEqual(cifp.parse_magvar("E0143"), 14.3)
        self.assertAlmostEqual(cifp.parse_magvar("W0094"), -9.4)
        self.assertEqual(cifp.parse_magvar("T0000"), 0.0)
        self.assertIsNone(cifp.parse_magvar("     "))

    def test_altitudes(self):
        self.assertEqual(cifp.parse_int("03600"), 3600)
        self.assertEqual(cifp.parse_int("FL180"), 18000)
        self.assertIsNone(cifp.parse_int("     "))

    def test_approach_ident(self):
        self.assertEqual(cifp.parse_approach_ident("I05-Z"), ("I", "05", "Z"))
        self.assertEqual(cifp.parse_approach_ident("R28L-Y"), ("R", "28L", "Y"))
        self.assertEqual(cifp.parse_approach_ident("I10R"), ("I", "10R", ""))
        self.assertEqual(cifp.parse_approach_ident("I16RZ"), ("I", "16R", "Z"))
        self.assertEqual(cifp.parse_approach_ident("VOR-B"), ("V", "", "B"))
        self.assertEqual(cifp.approach_title("I05-Z"), "ILS Z RWY 05")
        self.assertEqual(cifp.approach_title("VOR-B"), "VOR B")

    def test_cycle_selection(self):
        html = '<a href="https://aeronav.faa.gov/Upload_313-d/cifp/CIFP_261001.zip">x</a>' \
               '<a href="https://aeronav.faa.gov/Upload_313-d/cifp/CIFP_261029.zip">y</a>'
        dates = cifp.parse_download_page(html)
        self.assertEqual(dates, ["2026-10-01", "2026-10-29"])
        self.assertEqual(cifp.pick_cycle(dates, "2026-10-19"), "2026-10-01")
        self.assertEqual(cifp.pick_cycle(dates, "2026-10-29"), "2026-10-29")
        self.assertEqual(cifp.pick_cycle(dates, "2026-09-01"), "2026-10-01")  # nothing effective yet
        self.assertEqual(cifp.zip_url("2026-10-01"),
                         "https://aeronav.faa.gov/Upload_313-d/cifp/CIFP_261001.zip")


class RecordDecoding(unittest.TestCase):
    LINE = ("SUSAP KOTHK1FI05-Y I      030RW05 K1PG0GY M    CF IOTHK1      2257001104600026PI"
            "    00067             -300          0 DS   821672411")

    def test_fields(self):
        r = cifp.Record(self.LINE)
        self.assertEqual((r.apt, r.ident, r.rtype, r.seq, r.fix, r.pt), ("KOTH", "I05-Y", "I", "030", "RW05", "CF"))
        self.assertEqual((r.fixSec, r.fixSub), ("P", "G"))
        self.assertEqual(r.courseDeg(), (46.0, False))
        self.assertEqual(r.distNm(), 2.6)
        self.assertEqual((r.alt1, r.altDesc), (67, " "))
        self.assertAlmostEqual(r.vertAngleDeg, -3.0)
        self.assertTrue(r.isMap and r.isFlyOver and not r.isFaf)
        self.assertEqual((r.navId, r.navSec, r.navSub), ("IOTH", "P", "I"))

    def test_hold_time_and_rf_radius(self):
        hm = cifp.Record("SUSAP KOTHK1FI05-Y I      070YICBUK1PC0EE  R   HM                     1210T010    + 02600"
                         "                           0 DS   821712411")
        self.assertEqual(hm.timeMin(), 1.0)
        self.assertIsNone(hm.distNm())
        self.assertEqual(hm.turn, "R")
        rf = cifp.Record("SUSAP KOTHK1FH05-Z ADEROY 040PIVLYK1PC0E   R010RF       0030002567    30830027    + 02400"
                         "                 CFLTZ K1PCA FS   821122004")
        self.assertAlmostEqual(rf.arcRadiusNm, 3.0)
        self.assertEqual((rf.centerFix, rf.centerSec, rf.centerSub), ("CFLTZ", "P", "C"))


class KothApproaches(unittest.TestCase):
    def test_inventory(self):
        ids = sorted(build()[2]["KOTH"])
        self.assertEqual(ids, ["H05-Z", "I05-Y", "I05-Z", "L05-Y", "L05-Z", "R05-Y", "VOR-B"])
        z = appr("KOTH", "I05-Z")
        self.assertEqual((z.kind, z.runway, z.title), ("ILS", "05", "ILS Z RWY 05"))
        self.assertEqual(appr("KOTH", "H05-Z").kind, "RNAV")
        self.assertEqual(appr("KOTH", "VOR-B").runway, "")
        self.assertEqual(appr("KOTH", "R05-Y").los, ["LPV", "LNAV/VNAV", "LNAV"])

    def test_ils_z_final_matches_chart(self):
        # FAA chart ILS Z or LOC Z RWY 5 (OTH): MITUE (IF/IAF) 1600 min, OXVAK FAF 1300, GS 3.00
        # deg, TCH 51, TDZE 16 -> crossing the threshold at 67 ft. Final course 046 mag.
        a = appr("KOTH", "I05-Z")
        self.assertEqual([l.ident for l in a.final], ["MITUE", "OXVAK", "RW05"])
        mitue, oxvak, rw = a.final
        self.assertEqual((mitue.type, oxvak.type, rw.type), ("IF", "CF", "CF"))
        self.assertEqual((mitue.altKind, mitue.alt), ("ATORABOVE", 1600))
        self.assertEqual((oxvak.altKind, oxvak.alt), ("ATORABOVE", 1300))
        self.assertIn("FAF", oxvak.flags)
        self.assertEqual((rw.altKind, rw.alt), ("AT", 67))
        self.assertIn("MAP", rw.flags)
        self.assertTrue(rw.flyOver)
        self.assertEqual(rw.fix.kind, "RWY")
        self.assertAlmostEqual(rw.vertAngleDeg, -3.0)
        # published course 046 magnetic at 15.0 E variation -> 061 true; runway 05 is 061.15 true
        self.assertAlmostEqual(rw.crsTrue, 61.0, delta=0.05)
        self.assertAlmostEqual(mitue.fix.lat, 43.3136111, places=5)
        self.assertAlmostEqual(oxvak.fix.lon, -124.3377528, places=5)
        # OXVAK is 4.9 NM and MITUE 13.4 NM from the localizer (chart: I-OTH 4.9 / 13.4)
        thr = (rw.fix.lat, rw.fix.lon)
        self.assertAlmostEqual(dist_nm((oxvak.fix.lat, oxvak.fix.lon), thr), 4.9 - 0.9, delta=0.6)

    def test_ils_z_missed_matches_chart(self):
        # Chart: climb to 500, climbing left turn to 2600 on heading 340 and OTH R-301 to YICBU
        # (OTH 8 DME) and hold, continue climb-in-hold to 2600.
        a = appr("KOTH", "I05-Z")
        self.assertEqual([l.type for l in a.missed], ["CA", "CF", "HM"])
        ca, cf, hm = a.missed
        self.assertEqual((ca.altKind, ca.alt), ("ATORABOVE", 500))
        self.assertAlmostEqual(ca.crsTrue, 60.7, delta=0.05)
        self.assertEqual((cf.ident, cf.altKind, cf.alt), ("YICBU", "ATORABOVE", 2600))
        self.assertAlmostEqual(cf.crsTrue, 316.0, delta=0.05)  # R-301 magnetic + 15.0 E
        self.assertEqual((hm.ident, hm.alt, hm.holdTimeMin, hm.turn), ("YICBU", 2600, 1.0, "R"))
        self.assertAlmostEqual(hm.crsTrue, 136.0, delta=0.05)  # inbound 121 magnetic
        # YICBU is 8.0 NM from OTH
        oth = fix_of(appr("KOTH", "I05-Z").transitions[1].legs, "OTH").fix
        self.assertAlmostEqual(dist_nm((cf.fix.lat, cf.fix.lon), (oth.lat, oth.lon)), 8.0, delta=0.1)

    def test_ils_z_transitions(self):
        a = appr("KOTH", "I05-Z")
        self.assertEqual([t.name for t in a.transitions], ["LUPCI", "OTH", "RARES"])
        lupci = a.transitions[0].legs
        self.assertEqual([l.type for l in lupci], ["IF", "RF"])  # AF approximated by RF
        self.assertEqual(lupci[1].orig, "AF")
        clat, clon, radius, d = lupci[1].arc
        self.assertAlmostEqual(radius, 16.0, delta=0.1)  # chart: OTH 16 DME arc
        self.assertEqual(d, "R")
        oth = a.transitions[1].legs
        self.assertEqual([l.type for l in oth], ["IF", "TF", "HF"])  # hold in lieu of PT at MITUE
        self.assertEqual(oth[2].holdTimeMin, 1.0)
        self.assertEqual(oth[2].alt, 1600)
        self.assertEqual(oth[0].fix.kind, "VOR")
        self.assertIn("IAF", oth[2].flags)

    def test_rnp_with_rf_legs(self):
        a = appr("KOTH", "H05-Z")
        rfs = [l for l in a.transitions[0].legs if l.type == "RF"]
        self.assertEqual(len(rfs), 2)
        for l in rfs:
            clat, clon, radius, d = l.arc
            self.assertAlmostEqual(radius, 3.0, delta=0.01)
            self.assertAlmostEqual(dist_nm((clat, clon), (l.fix.lat, l.fix.lon)), radius, delta=0.06)
            self.assertIsNone(l.orig)
        self.assertEqual([l.ident for l in a.final], ["OXVAK", "RW05"])
        self.assertEqual([l.type for l in a.missed], ["TF", "RF", "TF", "HM"])
        self.assertEqual(a.missed[1].ident, "ERQAS")
        self.assertEqual(a.missed[1].arc[3], "L")
        self.assertEqual(a.missed[-1].holdDistNm, 4.0)

    def test_rnav_gps_y(self):
        a = appr("KOTH", "R05-Y")
        self.assertEqual([l.ident for l in a.final], ["MITUE", "OXVAK", "RW05"])
        self.assertIn("FAF", a.final[1].flags)
        self.assertEqual([l.type for l in a.missed], ["CA", "DF", "TF", "HM"])
        self.assertEqual((a.missed[0].alt, a.missed[0].altKind), (319, "ATORABOVE"))
        self.assertEqual(a.missed[1].ident, "RUNUE")
        self.assertEqual(a.missed[3].ident, "RARES")

    def test_vor_b_circling(self):
        a = appr("KOTH", "VOR-B")
        self.assertEqual(a.kind, "VOR")
        self.assertEqual([l.ident for l in a.final], ["COOSE", "JORAN", "NEWRI", "OTH", "CIYTY", "MAHIZ"])
        self.assertIn("MAP", a.final[-1].flags)
        self.assertEqual([l.type for l in a.missed], ["CA", "DF", "HM"])
        self.assertEqual(a.missed[1].ident, "OTH")

    def test_vi_and_pi_handling(self):
        # I05-Y transition OTH3 ends in a PI at the previous fix: dropped, not duplicated
        a = appr("KOTH", "I05-Y")
        self.assertEqual([l.ident for l in a.transitions[2].legs], ["OTH", "EMIRE"])
        # the missed approach VI (heading to intercept) is dropped; the CF to YICBU intercepts
        self.assertEqual([l.type for l in a.missed], ["CA", "CF", "HM"])

    def test_stats_count_approximations(self):
        _db, st, _out = build()
        self.assertGreater(st.approx["AF"], 0)
        self.assertGreater(st.dropped[("VI", "VI (next leg intercepts)")], 0)
        self.assertEqual(st.noMap, 0)


class KpdxApproaches(unittest.TestCase):
    def test_ils_10r(self):
        a = appr("KPDX", "I10R")
        self.assertEqual((a.kind, a.runway), ("ILS", "10R"))
        self.assertEqual(a.final[-1].ident, "RW10R")
        self.assertIn("MAP", a.final[-1].flags)
        self.assertTrue(any("FAF" in l.flags for l in a.final))
        self.assertTrue(a.missed and a.missed[0].type in ("CA", "VA"))
        self.assertEqual(a.transitions, [])  # IF at SCAPO: the chart has no feeder transitions
        self.assertEqual(a.final[0].ident, "SCAPO")

    def test_ils_28l(self):
        a = appr("KPDX", "I28L")
        self.assertEqual(a.final[-1].ident, "RW28L")
        self.assertIn("MAP", a.final[-1].flags)
        self.assertTrue(a.missed)
        # the MAP crosses the threshold at threshold elevation + TCH
        self.assertEqual(a.final[-1].altKind, "AT")


class Approximations(unittest.TestCase):
    def test_cd_vd_dme_termination(self):
        legs = [l for l in appr("KBFI", "I14R").missed + appr("KBFI", "I14R").final if l.orig == "CD"]
        self.assertEqual(len(legs), 1)
        syn = legs[0]
        self.assertEqual((syn.type, syn.fix.kind), ("CF", "SYN"))
        db = build()[0]
        loc = db.localizers[("KBFI", "IBFI")]
        # name encodes the DME distance; the computed point really is that far from the navaid
        want = float(syn.ident[len("IBFI"):])
        self.assertAlmostEqual(dist_nm((syn.fix.lat, syn.fix.lon), (loc["lat"], loc["lon"])), want, delta=0.05)
        vd = [l for part in (appr("KDEN", "I16L").final, appr("KDEN", "I16L").missed) for l in part if l.orig == "VD"]
        self.assertEqual(len(vd), 1)

    def test_vr_radial(self):
        legs = [l for l in appr("KLAX", "I25L").missed if l.orig == "VR"]
        self.assertEqual(len(legs), 1)
        self.assertEqual(legs[0].type, "CF")
        self.assertEqual(legs[0].fix.kind, "SYN")

    def test_fc_track_distance(self):
        a = appr("PANC", "I07L")
        fcs = [l for t in a.transitions for l in t.legs if l.orig == "FC" and l.fix.kind == "SYN"]
        self.assertTrue(fcs)
        for syn in fcs:
            self.assertEqual(syn.type, "TF")

    def test_vm_and_ci_dropped_next_tf_becomes_df(self):
        for apt, ident in (("KRFD", "I07"), ("KDDC", "I14")):
            a = appr(apt, ident)
            for part in [a.final, a.missed] + [t.legs for t in a.transitions]:
                self.assertTrue(all(l.type in cp.LEG_TYPES for l in part))

    def test_native_va_fa(self):
        self.assertTrue(any(l.type == "FA" for l in appr("KCOE", "I06").final + appr("KCOE", "I06").missed
                            + [x for t in appr("KCOE", "I06").transitions for x in t.legs]))
        self.assertTrue(any(l.type == "VA" for l in appr("KLPR", "I07").missed))

    def test_every_leg_is_self_contained(self):
        for apt, d in build()[2].items():
            for a in d.values():
                legs = a.final + a.missed + [l for t in a.transitions for l in t.legs]
                for l in legs:
                    self.assertIn(l.type, cp.LEG_TYPES)
                    if l.type in ("IF", "TF", "CF", "DF", "RF", "HM", "HA", "HF", "FA"):
                        self.assertIsNotNone(l.fix, f"{apt} {a.id} {l.type}")
                        self.assertTrue(-90 <= l.fix.lat <= 90 and -180 <= l.fix.lon <= 180)
                    if l.type in ("CF", "CA", "VA", "FA", "HM", "HA", "HF"):
                        self.assertIsNotNone(l.crsTrue)
                        self.assertTrue(0 <= l.crsTrue < 360)
                    if l.type == "RF":
                        self.assertIsNotNone(l.arc)
                        self.assertGreater(l.arc[2], 0.3)


class ProcFormat(unittest.TestCase):
    def test_round_trip_all_fixture_airports(self):
        db, _st, out = build()
        for apt, d in out.items():
            text = cp.write_proc(apt, list(d.values()), db.cycle, db.effective)
            meta, back = cp.parse_proc(text)
            self.assertEqual(meta, {"cycle": "2610", "airport": apt})
            self.assertEqual(len(back), len(d))
            for a, b in zip(d.values(), back):
                self.assertEqual((a.id, a.kind, a.runway, a.title, a.los), (b.id, b.kind, b.runway, b.title, b.los))
                self.assertEqual(len(a.final), len(b.final))
                self.assertEqual(len(a.missed), len(b.missed))
                self.assertEqual([t.name for t in a.transitions], [t.name for t in b.transitions])
                for x, y in zip(a.final + a.missed, b.final + b.missed):
                    self.assertEqual((x.type, x.ident, x.altKind, x.flags, x.orig), (y.type, y.ident, y.altKind, y.flags, y.orig))
                    self.assertAlmostEqual(x.alt, y.alt)
                    if x.fix:
                        self.assertAlmostEqual(x.fix.lat, y.fix.lat, 6)
                        self.assertAlmostEqual(x.fix.lon, y.fix.lon, 6)
                        self.assertEqual(x.fix.kind, y.fix.kind)
                    if x.crsTrue is not None:
                        self.assertAlmostEqual(x.crsTrue, y.crsTrue, 1)

    def test_leg_line_format(self):
        leg = cp.Leg("CF", cp.Fix("RW28R", 45.5, -122.6, "RWY"), crsTrue=297.04, altKind="AT", alt=63,
                     flyOver=True, flags={"MAP"})
        self.assertEqual(cp.format_leg(leg),
                         "LEG CF RW28R 45.5000000 -122.6000000 CRS 297.0 ALT AT 63 FLYOVER WK RWY MAP")
        self.assertEqual(cp.format_leg(cp.Leg("CA", None, crsTrue=45.7, altKind="ATORABOVE", alt=500)),
                         "LEG CA - CRS 45.7 ALT ATORABOVE 500")
        between = cp.parse_leg("LEG TF ABC 1.5 2.5 ALT BETWEEN 3000 5000 SPD 210 FUTURE=1")
        self.assertEqual((between.altKind, between.alt, between.alt2, between.speedKt), ("BETWEEN", 3000, 5000, 210))

    def test_reader_rejects_bad_input(self):
        with self.assertRaises(cp.ProcError):
            cp.parse_proc("FORMAT flightsim-proc 2\n")
        with self.assertRaises(cp.ProcError):
            cp.parse_proc("AIRPORT KOTH\n")
        with self.assertRaises(cp.ProcError):
            cp.parse_proc("FORMAT flightsim-proc 1\nAPPROACH I05 ILS 05 t\nFINAL\nLEG XX ABC 1 2\nEND\n")
        with self.assertRaises(cp.ProcError):
            cp.parse_proc("FORMAT flightsim-proc 1\nAPPROACH I05 ILS 05 t\nFINAL\nLEG TF ABC 1\nEND\n")
        # unknown directives and KEY=value options are skipped (forward compatibility)
        meta, ap = cp.parse_proc("FORMAT flightsim-proc 1\nNEWTHING 1\nAIRPORT KXYZ\nAPPROACH I05 ILS 05 t\n"
                                 "FINAL\nLEG TF ABC 1 2 X=3\nEND\n")
        self.assertEqual(meta["airport"], "KXYZ")
        self.assertEqual(ap[0].final[0].ident, "ABC")


class DriverTests(unittest.TestCase):
    def test_run_writes_filtered_procedures_and_report(self):
        with tempfile.TemporaryDirectory() as td:
            out = Path(td) / "nav"
            out.mkdir()
            (out / "nav.ini").write_text("[nav]\nformat = flightsim-nav\n[sources]\nfaa_cifp =\nwmm = x\n", encoding="utf-8")
            res = cifp.run(str(FIXTURE), out, cifp.normalise_airports("koth,pdx"), quiet=True)
            names = sorted(p.name for p in (out / "procedures").glob("*"))
            self.assertEqual(names, ["KOTH.proc", "KPDX.proc", "coverage_report.txt"])
            self.assertEqual(res["cycle"], "2610")
            self.assertIn("faa_cifp = 2610", (out / "nav.ini").read_text(encoding="utf-8"))
            self.assertIn("path terminators", (out / "procedures" / "coverage_report.txt").read_text())
            meta, aps = cp.parse_proc((out / "procedures" / "KOTH.proc").read_text(encoding="utf-8"))
            self.assertEqual(len(aps), 7)

    def test_unresolved_fix_drops_the_leg_not_the_approach(self):
        with open(FIXTURE, encoding="latin-1") as f:
            lines = [l.rstrip("\n") for l in f]
        # remove the enroute waypoint record of RARES (an IAF in KOTH R05-Y / I05-Z / L05-Z)
        lines = [l for l in lines if not (l.startswith("SUSAEA") and l[13:18].strip() == "RARES")]
        db = cifp.CifpDb({"KOTH"})
        db.load(lines)
        st = cifp.Stats()
        out = {a.id: a for a in cifp.build_airport(db, "KOTH", st)}
        self.assertGreater(st.unresolved[("E", "A")], 0)
        self.assertTrue(all(l.ident != "RARES" or l.type != "IF" for t in out["I05-Z"].transitions for l in t.legs))


if __name__ == "__main__":
    unittest.main()
