#!/usr/bin/env python3
"""Build the flight sim nav database (DATA.md section 5-6).

Sources: OurAirports CSVs (airports, runways, navaids), WMM2025 (magnetic variation grid) and,
optionally, the FAA NASR 28-day subscription (ILS, fixes, airways). Downloads are cached under
``cache/`` next to this script and the build can run entirely offline from that cache.

Examples:
    build_navdb.py --out data/nav                         # OurAirports + WMM
    build_navdb.py --out data/nav --faa-nasr auto         # + current FAA cycle, downloaded
    build_navdb.py --out data/nav --faa-nasr NASR.zip     # + a NASR CSV zip or directory
    build_navdb.py --out data/nav --offline --faa-nasr auto   # cache only, no network
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import json
import sys
import time
import urllib.error
import urllib.request
from collections import Counter
from pathlib import Path

import nasr
import ourairports as oa
import schema
import validate_navdb
import wmm as wmmmod

HERE = Path(__file__).resolve().parent
CACHE = HERE / "cache"
OURAIRPORTS_BASE = "https://davidmegginson.github.io/ourairports-data/"
OURAIRPORTS_FILES = ("airports.csv", "runways.csv", "navaids.csv")
USER_AGENT = "flightsim-nav-pipeline/1 (+https://github.com/; offline data builder)"
CACHE_MAX_AGE_DAYS = 7  # OurAirports is refreshed after this age (unless --offline)


class BuildError(Exception):
    pass


# --------------------------------------------------------------------------------------------
# downloads and cache
# --------------------------------------------------------------------------------------------
def _manifest() -> dict:
    p = CACHE / "manifest.json"
    try:
        return json.loads(p.read_text())
    except (OSError, ValueError):
        return {}


def _save_manifest(m: dict) -> None:
    CACHE.mkdir(parents=True, exist_ok=True)
    (CACHE / "manifest.json").write_text(json.dumps(m, indent=1, sort_keys=True))


def http_get(url: str, timeout: float = 180.0, retries: int = 3) -> bytes:
    last = None
    for attempt in range(retries):
        try:
            req = urllib.request.Request(url, headers={"User-Agent": USER_AGENT})
            with urllib.request.urlopen(req, timeout=timeout) as r:
                return r.read()
        except urllib.error.HTTPError as e:
            if e.code in (400, 401, 403, 404, 410):
                raise
            last = e
        except (urllib.error.URLError, TimeoutError, ConnectionError) as e:
            last = e
        time.sleep(1.5 * (attempt + 1))
    raise BuildError(f"download failed: {url}: {last}")


def cached_download(name: str, url: str, offline: bool, maxAgeDays: float | None) -> Path:
    """Return the cached path for `name`, downloading when missing/stale (unless offline)."""
    dest = CACHE / name
    man = _manifest()
    age = None
    if dest.exists():
        age = (time.time() - dest.stat().st_mtime) / 86400.0
    fresh = dest.exists() and (maxAgeDays is None or (age is not None and age <= maxAgeDays))
    if fresh:
        return dest
    if offline:
        if dest.exists():
            return dest  # stale but usable
        raise BuildError(f"--offline: {name} is not in the cache ({CACHE}); run once online first")
    try:
        data = http_get(url)
    except Exception:
        if dest.exists():
            print(f"  warning: could not refresh {name}; using cached copy", file=sys.stderr)
            return dest
        raise
    CACHE.mkdir(parents=True, exist_ok=True)
    tmp = dest.with_suffix(dest.suffix + ".part")
    tmp.write_bytes(data)
    tmp.replace(dest)
    man[name] = {"url": url, "downloaded_utc": dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")}
    _save_manifest(man)
    return dest


def download_date(name: str) -> str:
    """UTC date (YYYY-MM-DD) the cached file was downloaded."""
    m = _manifest().get(name)
    if m:
        return m["downloaded_utc"][:10]
    p = CACHE / name
    return dt.datetime.fromtimestamp(p.stat().st_mtime, dt.timezone.utc).strftime("%Y-%m-%d")


def read_csv_rows(path: Path) -> list[dict]:
    with open(path, encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def write_csv(path: Path, cols: list[str], rows: list[dict]) -> None:
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.DictWriter(f, fieldnames=cols, lineterminator="\n", extrasaction="raise")
        w.writeheader()
        w.writerows(rows)


# --------------------------------------------------------------------------------------------
# FAA NASR acquisition
# --------------------------------------------------------------------------------------------
def _anchor_cycles(todayIso: str) -> list[str]:
    """28-day AIRAC-style cycle dates computed from the known 2026-10-01 effective date."""
    anchor = dt.date(2026, 10, 1)
    today = dt.date.fromisoformat(todayIso)
    k = (today - anchor).days // 28
    return [(anchor + dt.timedelta(days=28 * (k - i))).isoformat() for i in range(0, 3)]


def locate_nasr(arg: str, offline: bool, todayIso: str) -> tuple[nasr.NasrSource, str]:
    """Return (NasrSource, effective date) for --faa-nasr <path|auto>."""
    if arg != "auto":
        p = Path(arg)
        if not p.exists():
            raise BuildError(f"--faa-nasr {arg}: no such file or directory")
        src = nasr.NasrSource(p)
        return src, src.effective_date()
    if offline:
        # newest complete set in the cache
        cands = sorted({f.name[:11] for f in CACHE.glob("*_ILS_CSV.zip")})
        for pre in reversed(cands):
            if all((CACHE / f"{pre}_{s}_CSV.zip").exists() for s in nasr.SUBJECTS):
                break
        else:
            raise BuildError("--offline --faa-nasr auto: no complete NASR set in the cache")
        d = dt.datetime.strptime(pre, "%d_%b_%Y").date().isoformat()
    else:
        dates: list[str] = []
        try:
            html = http_get(nasr.INDEX_URL, timeout=60).decode("utf-8", "replace")
            dates = nasr.parse_cycle_dates(html)
        except Exception as ex:  # noqa: BLE001
            print(f"  note: FAA index page unavailable ({ex}); computing cycle date", file=sys.stderr)
        cur = nasr.pick_current_cycle(dates, todayIso) if dates else None
        cycles = [cur] if cur else _anchor_cycles(todayIso)
        d = None
        for c in cycles + [x for x in _anchor_cycles(todayIso) if x not in cycles]:
            try:
                for s in nasr.SUBJECTS:
                    url = nasr.csv_zip_url(c, s)
                    name = url.rsplit("/", 1)[1]
                    cached_download(name, url, False, None)
                d = c
                break
            except urllib.error.HTTPError as ex:
                print(f"  note: NASR cycle {c} not downloadable ({ex}); trying previous", file=sys.stderr)
        if d is None:
            raise BuildError("could not download an FAA NASR cycle (use --faa-nasr <zip> or omit)")
    # assemble a source over the three zips
    paths = [CACHE / nasr.csv_zip_url(d, s).rsplit("/", 1)[1] for s in nasr.SUBJECTS]
    holder = _MultiSource([nasr.NasrSource(p) for p in paths])
    return holder, d  # type: ignore[return-value]


class _MultiSource:
    """Several NasrSource objects presented as one."""

    def __init__(self, sources):
        self.sources = sources

    def has(self, name):
        return any(s.has(name) for s in self.sources)

    def rows(self, name):
        for s in self.sources:
            if s.has(name):
                return s.rows(name)
        raise FileNotFoundError(name)

    def effective_date(self):
        for s in self.sources:
            e = s.effective_date()
            if e:
                return e
        return ""


# --------------------------------------------------------------------------------------------
# main build
# --------------------------------------------------------------------------------------------
def build(out: Path, faaNasr: str | None, offline: bool, createdUtc: str | None,
          identHeading: bool = True, wmmYear: float | None = None, quiet: bool = False) -> dict:
    t0 = time.time()
    log = (lambda *a: None) if quiet else (lambda *a: print(*a, flush=True))
    now = dt.datetime.now(dt.timezone.utc)
    created = createdUtc or now.strftime("%Y-%m-%dT%H:%M:%SZ")
    todayIso = created[:10]
    out.mkdir(parents=True, exist_ok=True)

    # ---- OurAirports
    log("[1/5] OurAirports")
    paths = {}
    for fn in OURAIRPORTS_FILES:
        paths[fn] = cached_download(fn, OURAIRPORTS_BASE + fn, offline, CACHE_MAX_AGE_DAYS)
    ourDate = min(download_date(fn) for fn in OURAIRPORTS_FILES)
    rawAirports = read_csv_rows(paths["airports.csv"])
    rawRunways = read_csv_rows(paths["runways.csv"])
    rawNavaids = read_csv_rows(paths["navaids.csv"])

    # ---- WMM
    log("[2/5] WMM2025 magnetic variation")
    model = wmmmod.Wmm()
    year = wmmYear if wmmYear is not None else wmmmod.decimal_year(dt.datetime.fromisoformat(created.replace("Z", "+00:00")))
    if not (model.validFromYear <= year <= model.validToYear):
        print(f"  warning: epoch {year:.2f} is outside the WMM2025 validity "
              f"[{model.validFromYear}, {model.validToYear}]; results degrade", file=sys.stderr)
    declFn = lambda lat, lon: model.declination_deg(lat, lon, year)  # noqa: E731
    wmmmod.write_magvar_bin(out / "magvar.bin", model, year)

    # ---- tables
    log("[3/5] Airports, runways, navaids")
    airports = oa.build_airports(rawAirports)
    apByIdent = {a["ident"]: a for a in airports}
    runways, rstats = oa.build_runways(rawRunways, apByIdent, declFn, identHeading)
    navaids = oa.build_navaids(rawNavaids, declFn)

    # ---- FAA NASR
    nasrDate = ""
    ils: list[dict] = []
    fixes: list[dict] = []
    airways: list[dict] = []
    istats: Counter = Counter()
    astats: Counter = Counter()
    wrote = {}
    if faaNasr:
        log(f"[4/5] FAA NASR ({faaNasr})")
        src, nasrDate = locate_nasr(faaNasr, offline, todayIso)
        idents = set(apByIdent)
        idMap = nasr.build_airport_id_map(rawAirports, idents)
        if src.has("ILS_BASE.csv"):
            ils, istats = nasr.build_ils(src.rows("ILS_BASE.csv"),
                                         src.rows("ILS_GS.csv") if src.has("ILS_GS.csv") else [],
                                         src.rows("ILS_DME.csv") if src.has("ILS_DME.csv") else [],
                                         idMap, idents)
        else:
            print("  warning: ILS_BASE.csv not found; ils.csv not written", file=sys.stderr)
        if src.has("FIX_BASE.csv"):
            fixes = nasr.build_fixes(src.rows("FIX_BASE.csv"))
        else:
            print("  warning: FIX_BASE.csv not found; fixes.csv not written", file=sys.stderr)
        if fixes and src.has("AWY_SEG_ALT.csv"):
            airways, astats = nasr.build_airways(src.rows("AWY_SEG_ALT.csv"), fixes, navaids)
        elif src.has("AWY_SEG_ALT.csv"):
            print("  warning: airways need fixes.csv; airways.csv not written", file=sys.stderr)
        # has_ils: 1 if the airport has an NASR ILS/LOC record, 0 for other US-territory airports
        usC = {"US", "PR", "VI", "GU", "MP", "AS", "UM"}
        withIls = {r["airport_ident"] for r in ils}
        for a in airports:
            if a["ident"] in withIls:
                a["has_ils"] = 1
            elif ils and a["iso_country"] in usC:
                a["has_ils"] = 0
    else:
        log("[4/5] FAA NASR: skipped (no --faa-nasr)")
        for stale in ("ils.csv", "fixes.csv", "airways.csv"):
            (out / stale).unlink(missing_ok=True)

    # ---- write
    log("[5/5] Writing")
    write_csv(out / "airports.csv", schema.AIRPORTS_COLS, airports)
    write_csv(out / "runways.csv", schema.RUNWAYS_COLS, runways)
    write_csv(out / "navaids.csv", schema.NAVAIDS_COLS, navaids)
    if faaNasr:
        if ils:
            write_csv(out / "ils.csv", schema.ILS_COLS, ils)
        if fixes:
            write_csv(out / "fixes.csv", schema.FIXES_COLS, fixes)
        if airways:
            write_csv(out / "airways.csv", schema.AIRWAYS_COLS, airways)
    (out / "nav.ini").write_text(
        "[nav]\n"
        f"format = {schema.FORMAT_NAME}\n"
        f"version = {schema.FORMAT_VERSION}\n"
        f"created_utc = {created}\n"
        "[sources]\n"
        f"ourairports = {ourDate}\n"
        f"faa_nasr = {nasrDate}\n"
        "faa_cifp =\n"
        f"wmm = WMM2025, epoch used {year:.3f}\n"
        "[attribution]\n"
        "airports = OurAirports (public domain), https://ourairports.com/data/\n"
        "faa = FAA NASR (US Government, public domain)\n"
        "wmm = NOAA NCEI World Magnetic Model 2025 (public domain)\n",
        encoding="utf-8")

    # ---- summary
    elapsed = time.time() - t0
    files = sorted(p for p in out.iterdir() if p.is_file())
    log("")
    log("=" * 64)
    log(f"nav database written to {out}")
    log(f"  airports.csv : {len(airports):>7} rows")
    log(f"  runways.csv  : {len(runways):>7} rows ({len(runways) // 2} runways; "
        f"{sum(1 for r in runways if r['approx'] == 1) // 2} approx, "
        f"{sum(1 for r in runways if r['hard'] == 1) // 2} hard)")
    log(f"  navaids.csv  : {len(navaids):>7} rows")
    if faaNasr:
        log(f"  ils.csv      : {len(ils):>7} rows ({sum(1 for r in ils if r['has_gs'] == 1)} with glideslope)")
        log(f"  fixes.csv    : {len(fixes):>7} rows")
        log(f"  airways.csv  : {len(airways):>7} rows ({astats['airways']} airways, "
            f"{astats['unresolved_points']} unresolved points)")
    log("  magvar.bin   : 181 x 361 grid, epoch %.3f" % year)
    log("runway stats: " + ", ".join(f"{k}={v}" for k, v in sorted(rstats.items())))
    if istats:
        log("ils stats   : " + ", ".join(f"{k}={v}" for k, v in sorted(istats.items())))
    log("file sizes  : " + ", ".join(f"{p.name} {p.stat().st_size / 1024:.0f} KiB" for p in files))
    log(f"sources     : ourairports {ourDate}, faa_nasr {nasrDate or '-'}, WMM2025")
    log(f"elapsed     : {elapsed:.1f} s")
    return {"airports": len(airports), "runways": len(runways), "navaids": len(navaids),
            "ils": len(ils), "fixes": len(fixes), "airways": len(airways), "stats": rstats}


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True, type=Path, help="output directory (created), e.g. data/nav")
    ap.add_argument("--faa-nasr", metavar="ZIP|DIR|auto", default=None,
                    help="add FAA NASR data (ils.csv, fixes.csv, airways.csv). 'auto' downloads the "
                         "current 28-day cycle CSV zips; or give a zip/directory that contains "
                         "ILS_BASE.csv, FIX_BASE.csv, AWY_SEG_ALT.csv (nested zips are searched)")
    ap.add_argument("--offline", action="store_true",
                    help="never touch the network: use only files already in cache/")
    ap.add_argument("--no-ident-heading", action="store_true",
                    help="do not synthesize runways that have no coordinates and no published "
                         "heading from the runway number (they are skipped instead)")
    ap.add_argument("--magvar-epoch", type=float, default=None, metavar="YEAR",
                    help="decimal year for WMM (default: now, or the --created-utc date)")
    ap.add_argument("--created-utc", default=None, metavar="ISO",
                    help="override the created timestamp (YYYY-MM-DDTHH:MM:SSZ), for reproducible builds")
    ap.add_argument("--no-validate", action="store_true", help="skip the post-build validation")
    ap.add_argument("--cifp", default=None, help=argparse.SUPPRESS)  # owned by WP N4
    a = ap.parse_args(argv)
    try:
        build(a.out, a.faa_nasr, a.offline, a.created_utc, not a.no_ident_heading, a.magvar_epoch)
    except BuildError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if not a.no_validate:
        rep = validate_navdb.validate(a.out)
        for w in rep.warnings:
            print("WARNING", w)
        for e in rep.errors:
            print("ERROR  ", e)
        print(f"validation  : {len(rep.errors)} errors, {len(rep.warnings)} warnings")
        if rep.errors:
            return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
