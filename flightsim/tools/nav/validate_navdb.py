#!/usr/bin/env python3
"""Validate a built nav database directory against DATA.md section 5-6 and run KSFO spot checks.

    python3 validate_navdb.py <navdir> [--spot-checks] [--wmm-selftest] [--strict]

Exit status 0 when there are no errors. Spot checks (``--spot-checks``) assume a database that
contains San Francisco (KSFO); they are skipped with a notice when KSFO is absent.
"""
from __future__ import annotations

import argparse
import configparser
import csv
import math
import sys
from pathlib import Path

import schema
import wmm as wmmmod
from navgeo import distance_m, initial_bearing_true_deg, M_PER_FT


class Report:
    def __init__(self):
        self.errors: list[str] = []
        self.warnings: list[str] = []
        self.info: list[str] = []

    def err(self, m):
        self.errors.append(m)

    def warn(self, m):
        self.warnings.append(m)

    def note(self, m):
        self.info.append(m)


def _read(path: Path):
    with open(path, encoding="utf-8", newline="") as f:
        return list(csv.DictReader(f))


def _num(rep: Report, table: str, rowNo: int, row: dict, col: str, lo=None, hi=None,
         required=True) -> float | None:
    v = row.get(col, "")
    if v == "":
        if required:
            rep.err(f"{table} row {rowNo}: {col} is empty")
        return None
    try:
        x = float(v)
    except ValueError:
        rep.err(f"{table} row {rowNo}: {col}={v!r} is not a number")
        return None
    if (lo is not None and x < lo) or (hi is not None and x > hi):
        rep.err(f"{table} row {rowNo}: {col}={x} outside [{lo}, {hi}]")
    return x


MAX_ERRORS_PER_TABLE = 25


def validate(navDir: Path | str) -> Report:
    d = Path(navDir)
    rep = Report()
    # ---- nav.ini
    ini = d / "nav.ini"
    if not ini.exists():
        rep.err("nav.ini missing")
    else:
        cp = configparser.ConfigParser()
        cp.read(ini, encoding="utf-8")
        if cp.get("nav", "format", fallback="") != schema.FORMAT_NAME:
            rep.err("nav.ini: [nav] format != flightsim-nav")
        if cp.get("nav", "version", fallback="") != str(schema.FORMAT_VERSION):
            rep.err("nav.ini: [nav] version != 1")
        for k in ("ourairports", "wmm"):
            if not cp.get("sources", k, fallback=""):
                rep.err(f"nav.ini: [sources] {k} missing/empty")
    # ---- tables
    tables: dict[str, list[dict]] = {}
    for name, cols in schema.TABLES.items():
        p = d / name
        if not p.exists():
            if name in ("airports.csv", "runways.csv", "navaids.csv"):
                rep.err(f"{name} missing")
            continue
        with open(p, encoding="utf-8", newline="") as f:
            header = next(csv.reader(f), [])
        if header != cols:
            rep.err(f"{name}: header mismatch\n   got      {header}\n   expected {cols}")
            continue
        tables[name] = _read(p)
        rep.note(f"{name}: {len(tables[name])} rows")

    ap = {a["ident"]: a for a in tables.get("airports.csv", [])}
    _check_airports(rep, tables.get("airports.csv", []))
    _check_runways(rep, tables.get("runways.csv", []), ap)
    _check_navaids(rep, tables.get("navaids.csv", []))
    if "ils.csv" in tables:
        _check_ils(rep, tables["ils.csv"], ap, tables.get("runways.csv", []))
    if "fixes.csv" in tables:
        _check_fixes(rep, tables["fixes.csv"])
    if "airways.csv" in tables:
        _check_airways(rep, tables["airways.csv"])
    _check_magvar(rep, d / "magvar.bin")
    return rep


def _limit(rep: Report, before: int, table: str):
    n = len(rep.errors) - before
    if n > MAX_ERRORS_PER_TABLE:
        del rep.errors[before + MAX_ERRORS_PER_TABLE:]
        rep.err(f"{table}: ... {n - MAX_ERRORS_PER_TABLE} more errors suppressed")


def _check_airports(rep, rows):
    e0 = len(rep.errors)
    seen = set()
    for i, r in enumerate(rows, 2):
        if not r["ident"]:
            rep.err(f"airports.csv row {i}: empty ident")
        if r["ident"] in seen:
            rep.err(f"airports.csv row {i}: duplicate ident {r['ident']}")
        seen.add(r["ident"])
        if r["type"] not in schema.AIRPORT_TYPES:
            rep.err(f"airports.csv row {i}: bad type {r['type']!r}")
        _num(rep, "airports.csv", i, r, "lat_deg", -90, 90)
        _num(rep, "airports.csv", i, r, "lon_deg", -180, 180)
        _num(rep, "airports.csv", i, r, "elev_ft", -1500, 30000, required=False)
        if r["has_ils"] not in ("", "0", "1"):
            rep.err(f"airports.csv row {i}: has_ils={r['has_ils']!r}")
    _limit(rep, e0, "airports.csv")


def _check_runways(rep, rows, ap):
    e0 = len(rep.errors)
    byKey = {(r["airport_ident"], r["rwy_ident"]): r for r in rows}
    if len(byKey) != len(rows):
        rep.err("runways.csv: duplicate (airport_ident, rwy_ident)")
    warnMismatch = 0
    for i, r in enumerate(rows, 2):
        if r["airport_ident"] not in ap:
            rep.err(f"runways.csv row {i}: unknown airport {r['airport_ident']}")
        opp = byKey.get((r["airport_ident"], r["opposite_ident"]))
        if opp is None or opp["opposite_ident"] != r["rwy_ident"]:
            rep.err(f"runways.csv row {i}: {r['airport_ident']} {r['rwy_ident']} has no reciprocal row")
        if r["surface"] not in schema.SURFACES:
            rep.err(f"runways.csv row {i}: bad surface {r['surface']!r}")
        if r["hard"] != ("1" if r["surface"] in ("asphalt", "concrete") else "0"):
            rep.err(f"runways.csv row {i}: hard flag inconsistent with surface")
        for c in ("lighted", "approx"):
            if r[c] not in ("0", "1"):
                rep.err(f"runways.csv row {i}: {c}={r[c]!r}")
        v = {}
        for c, lo, hi in (("thr_lat_deg", -90, 90), ("thr_lon_deg", -180, 180),
                          ("start_lat_deg", -90, 90), ("start_lon_deg", -180, 180),
                          ("end_lat_deg", -90, 90), ("end_lon_deg", -180, 180),
                          ("heading_true_deg", 0, 360), ("length_ft", 1, 40000),
                          ("displaced_ft", 0, 40000)):
            v[c] = _num(rep, "runways.csv", i, r, c, lo, hi)
        _num(rep, "runways.csv", i, r, "width_ft", 1, 10000, required=False)
        _num(rep, "runways.csv", i, r, "thr_elev_ft", -1500, 30000, required=False)
        _num(rep, "runways.csv", i, r, "end_elev_ft", -1500, 30000, required=False)
        if None in v.values():
            continue
        brg = initial_bearing_true_deg(v["start_lat_deg"], v["start_lon_deg"],
                                       v["end_lat_deg"], v["end_lon_deg"])
        dh = abs((brg - v["heading_true_deg"] + 180) % 360 - 180)
        if dh > 0.02:
            rep.err(f"runways.csv row {i}: heading {v['heading_true_deg']} != start->end bearing {brg:.2f}")
        dThr = distance_m(v["start_lat_deg"], v["start_lon_deg"], v["thr_lat_deg"], v["thr_lon_deg"]) / M_PER_FT
        if abs(dThr - v["displaced_ft"]) > max(3.0, 0.002 * v["displaced_ft"]):
            rep.err(f"runways.csv row {i}: threshold is {dThr:.0f} ft from start, displaced_ft={v['displaced_ft']}")
        if r["approx"] == "0":
            gm = distance_m(v["start_lat_deg"], v["start_lon_deg"], v["end_lat_deg"], v["end_lon_deg"]) / M_PER_FT
            if abs(gm - v["length_ft"]) > 0.25 * v["length_ft"] + 100:
                warnMismatch += 1
    if warnMismatch:
        rep.warn(f"runways.csv: {warnMismatch} exact rows whose coordinate length differs >25% "
                 "from the published length_ft")
    _limit(rep, e0, "runways.csv")


def _check_navaids(rep, rows):
    e0 = len(rep.errors)
    for i, r in enumerate(rows, 2):
        if r["type"] not in schema.NAVAID_TYPES:
            rep.err(f"navaids.csv row {i}: bad type {r['type']!r}")
        _num(rep, "navaids.csv", i, r, "lat_deg", -90, 90)
        _num(rep, "navaids.csv", i, r, "lon_deg", -180, 180)
        _num(rep, "navaids.csv", i, r, "freq_khz", 100, 1_000_000, required=False)
        _num(rep, "navaids.csv", i, r, "magvar_deg", -180, 180, required=False)
        _num(rep, "navaids.csv", i, r, "range_nm", 1, 500)
        if bool(r["dme_lat_deg"]) != bool(r["dme_lon_deg"]):
            rep.err(f"navaids.csv row {i}: dme lat/lon must both be set or both empty")
    _limit(rep, e0, "navaids.csv")


def _check_ils(rep, rows, ap, rwys):
    e0 = len(rep.errors)
    rw = {(r["airport_ident"], r["rwy_ident"]): r for r in rwys}
    badCourse = missingRwy = 0
    for i, r in enumerate(rows, 2):
        if r["airport_ident"] not in ap:
            rep.err(f"ils.csv row {i}: unknown airport {r['airport_ident']}")
        _num(rep, "ils.csv", i, r, "freq_mhz", 108.0, 112.0)
        _num(rep, "ils.csv", i, r, "loc_lat_deg", -90, 90)
        _num(rep, "ils.csv", i, r, "loc_lon_deg", -180, 180)
        c = _num(rep, "ils.csv", i, r, "loc_course_true_deg", 0, 360)
        if r["has_gs"] not in ("0", "1"):
            rep.err(f"ils.csv row {i}: has_gs={r['has_gs']!r}")
        if r["has_gs"] == "1":
            _num(rep, "ils.csv", i, r, "gs_lat_deg", -90, 90)
            _num(rep, "ils.csv", i, r, "gs_lon_deg", -180, 180)
            _num(rep, "ils.csv", i, r, "gs_angle_deg", 1.5, 9.0)
        run = rw.get((r["airport_ident"], r["rwy_ident"]))
        if run is None:
            missingRwy += 1
        elif c is not None:
            dh = abs((c - float(run["heading_true_deg"]) + 180) % 360 - 180)
            if dh > 12.0:
                badCourse += 1
    if missingRwy:
        rep.warn(f"ils.csv: {missingRwy} rows reference a runway not in runways.csv")
    if badCourse:
        rep.warn(f"ils.csv: {badCourse} rows whose localizer course differs >12 deg from the runway heading")
    _limit(rep, e0, "ils.csv")


def _check_fixes(rep, rows):
    e0 = len(rep.errors)
    for i, r in enumerate(rows, 2):
        if not r["ident"]:
            rep.err(f"fixes.csv row {i}: empty ident")
        _num(rep, "fixes.csv", i, r, "lat_deg", -90, 90)
        _num(rep, "fixes.csv", i, r, "lon_deg", -180, 180)
    _limit(rep, e0, "fixes.csv")


def _check_airways(rep, rows):
    e0 = len(rep.errors)
    last: dict[str, int] = {}
    for i, r in enumerate(rows, 2):
        s = _num(rep, "airways.csv", i, r, "seq", 1, 100000)
        _num(rep, "airways.csv", i, r, "fix_lat_deg", -90, 90)
        _num(rep, "airways.csv", i, r, "fix_lon_deg", -180, 180)
        _num(rep, "airways.csv", i, r, "mea_ft", 0, 100000, required=False)
        if s is not None:
            if r["airway"] in last and s <= last[r["airway"]]:
                rep.err(f"airways.csv row {i}: seq not increasing for {r['airway']}")
            last[r["airway"]] = int(s)
    _limit(rep, e0, "airways.csv")


def _check_magvar(rep, p: Path):
    if not p.exists():
        rep.err("magvar.bin missing")
        return
    try:
        g = wmmmod.read_magvar_bin(p)
    except Exception as ex:  # noqa: BLE001
        rep.err(f"magvar.bin: {ex}")
        return
    epoch, la0, la1, lo0, lo1, step, vals = g
    if (la0, la1, lo0, lo1, step) != (-90, 90, -180, 180, 1):
        rep.err(f"magvar.bin: unexpected grid {(la0, la1, lo0, lo1, step)}")
    if len(vals) != 181 * 361:
        rep.err(f"magvar.bin: {len(vals)} values, expected {181 * 361}")
    if vals and (min(vals) < -18000 or max(vals) > 18000):
        rep.err("magvar.bin: values outside +/-180 deg")
    # rows at lon -180 and +180 must agree (wrap)
    nLon = 361
    for r in range(0, 181, 30):
        if vals[r * nLon] != vals[r * nLon + 360]:
            rep.err("magvar.bin: lon -180 and +180 columns differ")
            break
    rep.note(f"magvar.bin: epoch {epoch:.3f}, {len(vals)} values")


# --------------------------------------------------------------------------------------------
# spot checks
# --------------------------------------------------------------------------------------------
def _angdiff(a: float, b: float) -> float:
    return abs((a - b + 180.0) % 360.0 - 180.0)


def spot_checks(navDir: Path | str, rep: Report) -> list[str]:
    """KSFO, KPDX and KOTH checks. Appends to rep.errors on failure and returns report lines.

    Nothing about ILS frequencies is hardcoded: they are looked up in ils.csv, printed, and
    sanity-checked (band 108.0-111.95 MHz, odd-tenth localizer channels, course vs runway).
    """
    d = Path(navDir)
    out: list[str] = []

    def check(cond: bool, ok: str, bad: str):
        if cond:
            out.append("PASS  " + ok)
        else:
            out.append("FAIL  " + bad)
            rep.err("spot check: " + bad)

    aps = {a["ident"]: a for a in _read(d / "airports.csv")}
    allRw = _read(d / "runways.csv")
    navaids = _read(d / "navaids.csv")
    ilsRows = _read(d / "ils.csv") if (d / "ils.csv").exists() else None
    grid = wmmmod.read_magvar_bin(d / "magvar.bin")

    check(not any(x["type"] not in schema.AIRPORT_TYPES for x in aps.values()),
          "only large/medium/small/seaplane airports (no heliports/closed/balloonports)",
          "excluded airport types present")

    def airport_checks(icao: str, iata: str, wantRunways: list[str] | None):
        a = aps.get(icao)
        check(a is not None, f"{icao} exists in airports.csv", f"{icao} missing from airports.csv")
        if a is None:
            return {}
        out.append(f"INFO  {icao}: {a['name']}, type {a['type']}, elev {a['elev_ft']} ft, has_ils={a['has_ils'] or '?'}")
        check(a["iata"] == iata, f"{icao} iata={iata}", f"{icao} iata={a['iata']!r}")
        rw = {r["rwy_ident"]: r for r in allRw if r["airport_ident"] == icao}
        if wantRunways is not None:
            miss = [w for w in wantRunways if w not in rw]
            check(not miss, f"{icao} runways present: {', '.join(sorted(rw))}",
                  f"{icao} runways missing: {miss} (have {sorted(rw)})")
        else:
            check(bool(rw), f"{icao} runways in data: {', '.join(sorted(rw))}", f"{icao} has no runways")
        mv = wmmmod.lookup_magvar_deg(grid, float(a["lat_deg"]), float(a["lon_deg"]))
        for ident, r in sorted(rw.items()):
            h = float(r["heading_true_deg"])
            m = int("".join(c for c in ident if c.isdigit())) * 10.0
            diff = _angdiff(h, m + mv)
            check(diff <= 8.0,
                  f"{icao} {ident}: heading_true {h:.2f}, thr {r['thr_lat_deg']},{r['thr_lon_deg']}, "
                  f"elev {r['thr_elev_ft']} ft, disp {r['displaced_ft']} ft, {r['length_ft']}x{r['width_ft']} ft "
                  f"(ident+magvar {mv:+.1f} = {m + mv:.1f}, diff {diff:.1f})",
                  f"{icao} {ident}: heading_true {h:.2f} vs ident+magvar {m + mv:.1f} (diff {diff:.1f})")
            check(r["approx"] == "0" and r["hard"] == "1", f"{icao} {ident}: approx=0 hard=1",
                  f"{icao} {ident}: approx={r['approx']} hard={r['hard']}")
            if float(r["displaced_ft"]) > 0:
                dThr = distance_m(float(r["start_lat_deg"]), float(r["start_lon_deg"]),
                                  float(r["thr_lat_deg"]), float(r["thr_lon_deg"])) / M_PER_FT
                check(abs(dThr - float(r["displaced_ft"])) < 5,
                      f"{icao} {ident}: threshold {dThr:.0f} ft from pavement start = displaced_ft",
                      f"{icao} {ident}: threshold {dThr:.0f} ft from start but displaced {r['displaced_ft']}")
        if ilsRows is not None:
            mine = sorted((r for r in ilsRows if r["airport_ident"] == icao), key=lambda x: x["rwy_ident"])
            out.append(f"INFO  {icao} ILS/LOC records in ils.csv: " + ("; ".join(
                f"{r['rwy_ident']} {r['loc_ident']} {r['freq_mhz']} MHz crs {r['loc_course_true_deg']}T "
                f"GS {'%s deg' % r['gs_angle_deg'] if r['has_gs'] == '1' else 'none'} cat {r['category'] or '-'}"
                for r in mine) or "none"))
            check(a["has_ils"] == ("1" if mine else "0"), f"{icao} has_ils={a['has_ils']} consistent with ils.csv",
                  f"{icao} has_ils={a['has_ils']} but ils.csv has {len(mine)} rows")
            for r in mine:
                f10 = int(float(r["freq_mhz"]) * 10 + 1e-6)  # tenths digit odd: 108.10..111.95
                check(1081 <= f10 <= 1119 and f10 % 2 == 1,
                      f"{icao} {r['rwy_ident']} LOC {r['freq_mhz']} MHz is a valid localizer channel",
                      f"{icao} {r['rwy_ident']} LOC {r['freq_mhz']} MHz is not a valid localizer channel")
                run = rw.get(r["rwy_ident"])
                if run:
                    dh = _angdiff(float(r["loc_course_true_deg"]), float(run["heading_true_deg"]))
                    check(dh < 3.0, f"{icao} {r['rwy_ident']} LOC course within {dh:.1f} deg of runway heading",
                          f"{icao} {r['rwy_ident']} LOC course {r['loc_course_true_deg']} vs runway {run['heading_true_deg']}")
                else:
                    check(False, "", f"{icao} ILS {r['rwy_ident']} has no matching runway row")
        else:
            out.append(f"SKIP  {icao}: ils.csv not present (no FAA NASR data)")
        return rw

    # ---- KSFO
    rw = airport_checks("KSFO", "SFO", ["01L", "01R", "10L", "10R", "19L", "19R", "28L", "28R"])
    if rw:
        for w, e in (("28L", 298), ("28R", 298), ("10L", 118), ("10R", 118)):
            h = float(rw[w]["heading_true_deg"])
            check(abs(h - e) <= 1.5, f"KSFO {w} heading_true {h:.2f} ~ {e}", f"KSFO {w} heading {h:.2f}, expected ~{e}")
        check(-5 <= float(rw["28L"]["thr_elev_ft"]) <= 30, "KSFO 28L thr elevation ~13 ft",
              f"KSFO 28L thr elev {rw['28L']['thr_elev_ft']}")
        check(float(aps["KSFO"]["elev_ft"]) in range(10, 17), "KSFO airport elevation ~13 ft",
              f"KSFO elevation {aps['KSFO']['elev_ft']}")
        sep = distance_m(float(rw["28R"]["thr_lat_deg"]), float(rw["28R"]["thr_lon_deg"]),
                         float(rw["10L"]["thr_lat_deg"]), float(rw["10L"]["thr_lon_deg"])) / M_PER_FT
        check(9000 < sep < 12000, f"KSFO 28R/10L thresholds {sep:.0f} ft apart", f"KSFO 28R/10L separation {sep:.0f} ft")
        mv = wmmmod.lookup_magvar_deg(grid, 37.62, -122.37)
        check(11.5 <= mv <= 14.5, f"magvar.bin at KSFO = {mv:+.2f} deg (expect ~ +13 E)", f"magvar.bin at KSFO = {mv:+.2f}")
    sfo = [n for n in navaids if n["ident"] == "SFO" and n["iso_country"] == "US"]
    check(any(n["freq_khz"] == "115800" for n in sfo),
          "SFO VOR 115.8 MHz present: " + ", ".join(f"{n['type']} {n['freq_khz']}" for n in sfo),
          f"SFO 115800 navaid missing: {sfo}")
    check(any(n["type"] == "VORTAC" for n in navaids), "VORTAC navaids exist (e.g. OAK/BTG/OTH)", "no VORTAC navaids")

    # ---- KPDX (Portland): 10L/28R, 10R/28L, 03/21
    airport_checks("KPDX", "PDX", ["10L", "10R", "28L", "28R", "03", "21"])
    # ---- KOTH (North Bend): runways come from the data, whatever they are
    airport_checks("KOTH", "OTH", None)
    for ident in ("PDX", "OTH"):
        n = [x for x in navaids if x["ident"] == ident and x["iso_country"] == "US"]
        out.append(f"INFO  navaid {ident}: " + ", ".join(f"{x['type']} {x['freq_khz']} kHz" for x in n))
    return out


def wmm_selftest() -> list[str]:
    lines = []
    ok = True
    for r, err, good in wmmmod.selftest():
        lines.append(f"{'PASS' if good else 'FAIL'}  {r['year']} h={r['hKm']:5.1f} km lat={r['latDeg']:6.1f} "
                     f"lon={r['lonDeg']:6.1f}  dDecl={err['declDeg']:+.3f} dIncl={err['inclDeg']:+.3f} deg  "
                     f"dX={err['xNt']:+.2f} dY={err['yNt']:+.2f} dZ={err['zNt']:+.2f} nT")
        ok &= good
    lines.append("WMM2025 test values: " + ("all within 0.05 deg" if ok else "FAILED"))
    return lines if ok else lines + ["__FAIL__"]


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("navdir", nargs="?", type=Path, help="directory containing nav.ini and the CSV files")
    ap.add_argument("--spot-checks", action="store_true", help="run the KSFO / SFO VOR / ILS spot checks")
    ap.add_argument("--wmm-selftest", action="store_true",
                    help="compare the WMM2025 implementation with the official NOAA test values")
    ap.add_argument("--strict", action="store_true", help="treat warnings as errors")
    a = ap.parse_args(argv)
    rc = 0
    if a.wmm_selftest:
        lines = wmm_selftest()
        print("\n".join(l for l in lines if l != "__FAIL__"))
        if "__FAIL__" in lines:
            rc = 1
    if a.navdir:
        rep = validate(a.navdir)
        if a.spot_checks:
            print("\n".join(spot_checks(a.navdir, rep)))
        print("\n".join(rep.info))
        for w in rep.warnings:
            print("WARNING", w)
        for e in rep.errors:
            print("ERROR  ", e)
        print(f"validation: {len(rep.errors)} errors, {len(rep.warnings)} warnings")
        if rep.errors or (a.strict and rep.warnings):
            rc = 1
    elif not a.wmm_selftest:
        ap.error("give a nav directory and/or --wmm-selftest")
    return rc


if __name__ == "__main__":
    sys.exit(main())
