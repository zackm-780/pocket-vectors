#!/usr/bin/env python3
"""Download raw METARs into <data root>/weather/<ICAO>.txt (X1).

Standard library only. Run it on a PC (the handheld has no TLS library) and copy the weather/
folder to the device data root, or let the launch script call it where python3 exists.

    fetch_metar.py --out data/weather KPDX KOTH KEUG
    fetch_metar.py --out data/weather --plan assets/plans/kpdx_koth.fms

Each file holds the newest METAR line of that station, as served by
https://aviationweather.gov/api/data/metar?ids=KPDX&format=raw (one report per line).
Stations without a report are skipped (an old file is left in place).
"""
import argparse
import os
import re
import sys
import urllib.error
import urllib.parse
import urllib.request

API = "https://aviationweather.gov/api/data/metar"
ICAO = re.compile(r"^[A-Z][A-Z0-9]{3}$")


def fetch(ids, timeout_s=20.0):
    """Returns {ICAO: newest raw METAR line} for the stations that reported."""
    query = urllib.parse.urlencode({"ids": ",".join(ids), "format": "raw"}, safe=",")
    request = urllib.request.Request(
        API + "?" + query, headers={"User-Agent": "retro-games-flightsim-fetch-metar/1.0"}
    )
    with urllib.request.urlopen(request, timeout=timeout_s) as response:
        text = response.read().decode("utf-8", "replace")
    found = {}
    for line in text.splitlines():
        words = line.split()
        if not words:
            continue
        # "METAR KPDX ..." / "SPECI KPDX ..." / "KPDX ..."
        station = words[1] if words[0] in ("METAR", "SPECI") and len(words) > 1 else words[0]
        if ICAO.match(station) and station not in found:  # the API lists newest first
            found[station] = line.strip()
    return found


def plan_idents(path):
    """Airport idents mentioned in a Garmin .fms flight plan (best effort)."""
    idents = []
    with open(path, encoding="utf-8", errors="replace") as f:
        for line in f:
            words = line.split()
            # waypoint rows: type ident ... ; type 1 = airport
            if len(words) >= 2 and words[0] == "1" and ICAO.match(words[1]):
                idents.append(words[1])
    return idents


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("stations", nargs="*", help="ICAO idents, e.g. KPDX KOTH")
    ap.add_argument("--out", default="weather", help="output folder (default: ./weather)")
    ap.add_argument("--plan", action="append", default=[], help="also fetch the airports of an .fms plan")
    ap.add_argument("--timeout", type=float, default=20.0)
    args = ap.parse_args(argv)

    ids = [s.upper() for s in args.stations]
    for plan in args.plan:
        ids += plan_idents(plan)
    ids = sorted(set(ids))
    bad = [s for s in ids if not ICAO.match(s)]
    if bad:
        print("not ICAO idents: " + " ".join(bad), file=sys.stderr)
        return 2
    if not ids:
        ap.print_usage(sys.stderr)
        return 2

    try:
        reports = fetch(ids, args.timeout)
    except (urllib.error.URLError, OSError) as e:
        print("download failed: %s" % e, file=sys.stderr)
        return 1

    os.makedirs(args.out, exist_ok=True)
    for ident in ids:
        line = reports.get(ident)
        if not line:
            print("%s: no report" % ident)
            continue
        target = os.path.join(args.out, ident + ".txt")
        tmp = target + ".tmp"
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(line + "\n")
        os.replace(tmp, target)
        print("%s: %s" % (ident, line))
    return 0


if __name__ == "__main__":
    sys.exit(main())
