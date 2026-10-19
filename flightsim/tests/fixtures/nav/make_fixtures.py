#!/usr/bin/env python3
"""Regenerates the small nav fixtures in this directory from a full nav database.

Usage: make_fixtures.py <full nav dir> [out dir]

Copies a handful of real rows (OurAirports and FAA NASR data, both public domain) so unit tests
need no data pack. The CSVs are checked in; only rerun this when the schema changes.
magvar.bin here is NOT real data: it is a synthetic 30 degree grid with a known formula
(declination in degrees = lat/10 + lon/30 at every grid node) used to test interpolation.
"""
import argparse
import csv
import os
import struct

AIRPORTS = ["KPDX", "KOTH", "KSFO", "KEUG", "KHIO", "KTTD", "KVUO", "KSEA", "KLAX", "04W", "04WN",
            "00AL"]
NAVAIDS = ["PDX", "OTH", "SFO", "BTG", "UBG"]
FIX_NEAR = [(45.6, -122.6, 1.5), (43.4, -124.2, 1.5)]  # lat, lon, half-box deg
AIRWAYS = ["V23", "V287"]


def rows(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        return r.fieldnames, list(r)


def write(path, header, data):
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=header, lineterminator="\n")
        w.writeheader()
        w.writerows(data)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument("src", help="full nav database directory")
    ap.add_argument("out", nargs="?", default=os.path.dirname(os.path.abspath(__file__)),
                    help="output directory (default: this directory)")
    a = ap.parse_args()

    h, apts = rows(os.path.join(a.src, "airports.csv"))
    apts = [r for r in apts if r["ident"] in AIRPORTS]
    write(os.path.join(a.out, "airports.csv"), h, apts)

    h, rw = rows(os.path.join(a.src, "runways.csv"))
    write(os.path.join(a.out, "runways.csv"), h, [r for r in rw if r["airport_ident"] in AIRPORTS])

    h, ils = rows(os.path.join(a.src, "ils.csv"))
    write(os.path.join(a.out, "ils.csv"), h, [r for r in ils if r["airport_ident"] in AIRPORTS])

    h, nav = rows(os.path.join(a.src, "navaids.csv"))
    keep = [r for r in nav if r["ident"] in NAVAIDS]
    seen = {}
    for r in nav:  # two ambiguous idents (same ident, different place)
        seen.setdefault(r["ident"], []).append(r)
    amb = [k for k, v in seen.items() if len(v) == 2 and k not in NAVAIDS][:2]
    for k in amb:
        keep += seen[k]
    write(os.path.join(a.out, "navaids.csv"), h, keep)

    h, fx = rows(os.path.join(a.src, "fixes.csv"))
    keepf = [r for r in fx if any(abs(float(r["lat_deg"]) - la) < d and abs(float(r["lon_deg"]) - lo) < d
                                  for la, lo, d in FIX_NEAR)][:60]
    cnt = {}
    for r in fx:
        cnt.setdefault(r["ident"], []).append(r)
    # Real fix idents are unique in the US data, so add one SYNTHETIC duplicate of the first fix
    # (same ident, different place) to exercise ambiguous-ident handling.
    keepf.append({"ident": keepf[0]["ident"], "region": "K2", "lat_deg": "35.000000",
                  "lon_deg": "-100.000000"})
    write(os.path.join(a.out, "fixes.csv"), h, keepf)

    h, aw = rows(os.path.join(a.src, "airways.csv"))
    write(os.path.join(a.out, "airways.csv"), h, [r for r in aw if r["airway"] in AIRWAYS])

    with open(os.path.join(a.out, "nav.ini"), "w") as f:
        f.write("[nav]\nformat = flightsim-nav\nversion = 1\ncreated_utc = fixture\n"
                "[sources]\nourairports = fixture\nfaa_nasr = fixture\nfaa_cifp =\n"
                "wmm = synthetic test grid, epoch used 2026.0\n")

    # Synthetic magvar: 30 degree grid, value (deg) = lat/10 + lon/30.
    lats = list(range(-90, 91, 30))
    lons = list(range(-180, 181, 30))
    with open(os.path.join(a.out, "magvar.bin"), "wb") as f:
        f.write(b"MAGVAR01" + struct.pack("<fhhhhhh", 2026.0, -90, 90, -180, 180, 30, 0))
        for la in lats:
            for lo in lons:
                f.write(struct.pack("<h", round((la / 10.0 + lo / 30.0) * 100)))


if __name__ == "__main__":
    main()
