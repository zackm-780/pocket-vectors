#!/usr/bin/env python3
"""FAA CIFP (ARINC 424-18) -> flight sim instrument procedures (DATA.md section 7).

Reads the FAA Coded Instrument Flight Procedures file ``FAACIFP18`` (a fixed-width, 132 column
ARINC 424-18 text file) and writes one ``procedures/{ICAO}.proc`` per airport with approaches
(airport section P, subsection F). Every leg is self-contained: the fix name AND its coordinates are
copied into the leg, so the runtime never has to look anything up.

ARINC 424 path terminators IF, TF, CF, DF, CA, VA, FA, RF, HM, HA and HF map one to one onto the
runtime leg types. AF, PI, VD, VI, VM, CI, CD, CR, VR, FC, FD and FM are approximated with a logged
mapping (see ``approximate_legs``), and every approximation is counted in the coverage report.

Examples:
    cifp.py --cifp auto --out data/nav                       # download current cycle, all airports
    cifp.py --cifp auto --out data/nav --airports KOTH,KPDX  # quick run
    cifp.py --cifp CIFP_261001.zip --out /tmp/nav --report /tmp/cov.txt
    cifp.py --cifp auto --offline --out data/nav             # cache only, no network

The same code is reachable from the nav pipeline as ``build_navdb.py --out data/nav --cifp auto``.
Standard library only (Python >= 3.10).
"""
from __future__ import annotations

import argparse
import datetime as dt
import io
import math
import re
import sys
import time
import urllib.error
import urllib.request
import zipfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Iterable, Iterator

import cifp_proc as cp
import navgeo

HERE = Path(__file__).resolve().parent
CACHE = HERE / "cache"
CIFP_INDEX_URL = "https://www.faa.gov/air_traffic/flight_info/aeronav/digital_products/cifp/download/"
CIFP_ZIP_URL = "https://aeronav.faa.gov/Upload_313-d/cifp/CIFP_{yymmdd}.zip"
CIFP_MEMBER = "FAACIFP18"
USER_AGENT = "flightsim-nav-pipeline/1 (+https://github.com/; offline data builder)"

NM_PER_M = 1.0 / navgeo.M_PER_NM


class CifpError(Exception):
    pass


# --------------------------------------------------------------------------------------------
# download and cycle selection
# --------------------------------------------------------------------------------------------
def parse_download_page(html: str) -> list[str]:
    """Effective dates (ISO) of the CIFP zips linked from the FAA download page, ascending."""
    dates = set()
    for m in re.finditer(r"CIFP_(\d{2})(\d{2})(\d{2})\.zip", html):
        dates.add(f"20{m.group(1)}-{m.group(2)}-{m.group(3)}")
    return sorted(dates)


def pick_cycle(dates: list[str], todayIso: str) -> str | None:
    """Newest cycle effective on or before today, else the earliest listed one."""
    if not dates:
        return None
    past = [d for d in dates if d <= todayIso]
    return past[-1] if past else dates[0]


def zip_url(effectiveIso: str) -> str:
    return CIFP_ZIP_URL.format(yymmdd=effectiveIso[2:4] + effectiveIso[5:7] + effectiveIso[8:10])


def http_get(url: str, timeout: float = 300.0, retries: int = 3) -> bytes:
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
    raise CifpError(f"download failed: {url}: {last}")


def locate_cifp(arg: str, offline: bool, todayIso: str) -> tuple[Path, str]:
    """Resolve --cifp <auto|zip|file|dir> to (path of zip or FAACIFP18 file, effective date ISO).
    The date is '' when it cannot be determined (a bare file; the header still gives the cycle)."""
    if arg != "auto":
        p = Path(arg)
        if p.is_dir():
            cands = sorted(p.glob("CIFP_*.zip")) or sorted(p.glob("**/FAACIFP18"))
            if not cands:
                raise CifpError(f"--cifp {arg}: no CIFP_*.zip or FAACIFP18 inside")
            p = cands[-1]
        if not p.exists():
            raise CifpError(f"--cifp {arg}: no such file or directory")
        m = re.search(r"CIFP_(\d{2})(\d{2})(\d{2})", p.name)
        return p, (f"20{m.group(1)}-{m.group(2)}-{m.group(3)}" if m else "")
    CACHE.mkdir(parents=True, exist_ok=True)
    if offline:
        cached = sorted(CACHE.glob("CIFP_??????.zip"))
        if not cached:
            raise CifpError("--offline --cifp auto: no CIFP_*.zip in the cache; run once online first")
        dates = []
        for c in cached:
            m = re.search(r"CIFP_(\d{2})(\d{2})(\d{2})", c.name)
            dates.append(f"20{m.group(1)}-{m.group(2)}-{m.group(3)}")
        d = pick_cycle(sorted(dates), todayIso) or dates[-1]
        return CACHE / f"CIFP_{d[2:4]}{d[5:7]}{d[8:10]}.zip", d
    dates: list[str] = []
    try:
        dates = parse_download_page(http_get(CIFP_INDEX_URL, timeout=60).decode("utf-8", "replace"))
    except Exception as ex:  # noqa: BLE001
        print(f"  note: FAA CIFP page unavailable ({ex}); trying the cache / 28-day arithmetic",
              file=sys.stderr)
    if not dates:
        anchor = dt.date(2026, 10, 1)
        today = dt.date.fromisoformat(todayIso)
        k = (today - anchor).days // 28
        dates = [(anchor + dt.timedelta(days=28 * (k - i))).isoformat() for i in (1, 0)]
    order = []
    cur = pick_cycle(dates, todayIso)
    if cur:
        order.append(cur)
    order += [d for d in reversed(dates) if d != cur]
    last: Exception | None = None
    for d in order:
        dest = CACHE / f"CIFP_{d[2:4]}{d[5:7]}{d[8:10]}.zip"
        if dest.exists():
            return dest, d
        try:
            data = http_get(zip_url(d))
        except urllib.error.HTTPError as ex:
            last = ex
            print(f"  note: CIFP cycle {d} not downloadable ({ex}); trying another", file=sys.stderr)
            continue
        tmp = dest.with_suffix(".zip.part")
        tmp.write_bytes(data)
        tmp.replace(dest)
        return dest, d
    raise CifpError(f"could not download an FAA CIFP cycle ({last}); use --cifp <zip>")


def open_cifp_lines(path: Path) -> Iterator[str]:
    """Lines of FAACIFP18 from a zip, a bare file, or (for tests) any ARINC text file."""
    if path.suffix.lower() == ".zip":
        with zipfile.ZipFile(path) as z:
            name = next((n for n in z.namelist() if n.split("/")[-1].upper() == CIFP_MEMBER), None)
            if name is None:
                raise CifpError(f"{path}: no {CIFP_MEMBER} inside")
            with z.open(name) as f:
                for raw in io.TextIOWrapper(f, encoding="latin-1", newline=None):
                    yield raw.rstrip("\r\n")
    else:
        with open(path, encoding="latin-1", newline=None) as f:
            for raw in f:
                yield raw.rstrip("\r\n")


# --------------------------------------------------------------------------------------------
# ARINC 424 field helpers (column numbers below are 0-based string indexes)
# --------------------------------------------------------------------------------------------
def parse_lat(s: str) -> float | None:
    """'N43245302' -> 43.41473 (DD MM SS.ss)."""
    s = s.strip()
    if len(s) != 9 or s[0] not in "NS" or not s[1:].isdigit():
        return None
    v = int(s[1:3]) + int(s[3:5]) / 60.0 + int(s[5:9]) / 100.0 / 3600.0
    return -v if s[0] == "S" else v


def parse_lon(s: str) -> float | None:
    """'W124153815' -> -124.2605 (DDD MM SS.ss)."""
    s = s.strip()
    if len(s) != 10 or s[0] not in "EW" or not s[1:].isdigit():
        return None
    v = int(s[1:4]) + int(s[4:6]) / 60.0 + int(s[6:10]) / 100.0 / 3600.0
    return -v if s[0] == "W" else v


def parse_magvar(s: str) -> float | None:
    """'E0143' -> +14.3, 'W0094' -> -9.4, 'T0000' (true) -> 0.0. East positive."""
    s = s.strip()
    if len(s) != 5 or s[0] not in "EWT" or not s[1:].isdigit():
        return None
    v = int(s[1:]) / 10.0
    return -v if s[0] == "W" else v


def parse_int(s: str) -> int | None:
    s = s.strip()
    if not s:
        return None
    if s.startswith("FL") and s[2:].isdigit():
        return int(s[2:]) * 100
    try:
        return int(s)
    except ValueError:
        return None


def norm360(d: float) -> float:
    return navgeo.norm360(d)


class Record:
    """One decoded primary approach record (airport section P, subsection F)."""

    __slots__ = ("apt", "region", "ident", "rtype", "trans", "seq", "fix", "fixRegion", "fixSec",
                 "fixSub", "desc", "turn", "pt", "navId", "navRegion", "navSec", "navSub", "arcRadiusNm",
                 "thetaDeg", "rhoNm", "crsRaw", "distRaw", "altDesc", "alt1", "alt2", "speedKt",
                 "speedDesc", "vertAngleDeg", "centerFix", "centerRegion", "centerSec", "centerSub",
                 "line", "los")

    def __init__(self, line: str):
        g = line
        self.line = line
        self.apt = g[6:10].strip()
        self.region = g[10:12]
        self.ident = g[13:19].strip()
        self.rtype = g[19]
        self.trans = g[20:25].strip()
        self.seq = g[26:29]
        self.fix = g[29:34].strip()
        self.fixRegion = g[34:36]
        self.fixSec = g[36]
        self.fixSub = g[37]
        self.desc = g[39:43]
        self.turn = g[43].strip()
        self.pt = g[47:49]
        self.navId = g[50:54].strip()
        self.navRegion = g[54:56]
        self.navSec = g[78].strip() if len(g) > 78 else ""
        self.navSub = g[79].strip() if len(g) > 79 else ""
        rad = g[56:62].strip()
        self.arcRadiusNm = int(rad) / 1000.0 if rad.isdigit() else None
        th = g[62:66].strip()
        self.thetaDeg = int(th) / 10.0 if th.isdigit() else None
        rho = g[66:70].strip()
        self.rhoNm = int(rho) / 10.0 if rho.isdigit() else None
        self.crsRaw = g[70:74]
        self.distRaw = g[74:78]
        self.altDesc = g[82]
        self.alt1 = parse_int(g[84:89])
        self.alt2 = parse_int(g[89:94])
        sp = g[99:102].strip()
        self.speedKt = int(sp) if sp.isdigit() else 0
        self.speedDesc = g[117] if len(g) > 117 else " "
        va = g[102:106].strip()
        self.vertAngleDeg = None
        if va and re.fullmatch(r"[+-]?\d{3,4}", va):
            self.vertAngleDeg = int(va) / 100.0
        self.centerFix = g[106:111].strip()
        self.centerRegion = g[112:114]
        self.centerSec = g[114].strip()
        self.centerSub = g[115].strip()
        self.los: list[str] = []

    # ---- derived
    @property
    def isTransition(self) -> bool:
        return self.rtype == "A"

    def courseDeg(self) -> tuple[float, bool] | None:
        """(course degrees, isTrue) from the magnetic-course field, None if blank."""
        s = self.crsRaw
        if re.fullmatch(r"\d{4}", s):
            return int(s) / 10.0, False
        if re.fullmatch(r"\d{3}T", s):  # whole degrees, true
            return float(int(s[:3])), True
        return None

    def distNm(self) -> float | None:
        s = self.distRaw
        return int(s) / 10.0 if re.fullmatch(r"\d{4}", s) else None

    def timeMin(self) -> float | None:
        s = self.distRaw
        return int(s[1:]) / 10.0 if re.fullmatch(r"T\d{3}", s) else None

    @property
    def isFaf(self) -> bool:
        return self.desc[3] == "F"

    @property
    def isMap(self) -> bool:
        return self.desc[3] == "M"

    @property
    def isIaf(self) -> bool:
        return self.desc[3] in "ACD"

    @property
    def isFlyOver(self) -> bool:
        return self.desc[1] == "Y"


# --------------------------------------------------------------------------------------------
# the CIFP database (coordinates for every fix a procedure can reference + approach records)
# --------------------------------------------------------------------------------------------
class CifpDb:
    """Indexes of fix coordinates plus the raw approach records grouped by airport."""

    def __init__(self, airportFilter: set[str] | None = None):
        self.filter = airportFilter
        self.cycle = ""  # 'YYNN' from the header volume
        self.effective = ""  # ISO date from the header
        self.airports: dict[str, dict] = {}
        self.runways: dict[tuple[str, str], dict] = {}
        self.termWpts: dict[tuple[str, str], list[dict]] = defaultdict(list)  # (ident, region)
        self.enrWpts: dict[tuple[str, str], dict] = {}
        self.vhf: dict[tuple[str, str], dict] = {}
        self.ndb: dict[tuple[str, str], dict] = {}
        self.termNdb: dict[tuple[str, str], list[dict]] = defaultdict(list)
        # fallbacks when the leg names another ICAO region code than the fix record: ident -> rows
        self._byIdent: dict[str, dict[str, list[dict]]] = {
            "term": defaultdict(list), "enr": defaultdict(list), "vhf": defaultdict(list),
            "ndb": defaultdict(list), "tndb": defaultdict(list)}
        self.localizers: dict[tuple[str, str], dict] = {}
        self.records: dict[str, list[Record]] = defaultdict(list)  # airport -> approach records
        self.counts: Counter = Counter()

    # ---- loading
    def load(self, lines: Iterable[str]) -> None:
        pending: Record | None = None
        for line in lines:
            if len(line) < 40:
                continue
            if line.startswith("HDR04"):
                m = re.search(r"VOLUME\s+(\d{4})\s+EFFECTIVE\s+(\d{2}) (\w{3}) (\d{4})", line)
                if m:
                    self.cycle = m.group(1)
                    try:
                        self.effective = dt.datetime.strptime(
                            f"{m.group(2)} {m.group(3)} {m.group(4)}", "%d %b %Y").date().isoformat()
                    except ValueError:
                        pass
                continue
            if line[0] not in "ST":  # any customer/area code: Alaska is CAN, Hawaii and Guam are PAC
                continue
            sec = line[4]
            if sec == "P":
                self._load_terminal(line)
            elif sec == "E" and line[5] == "A":
                self._load_enroute_wpt(line)
            elif sec == "D":
                self._load_navaid(line)
        self.counts["airports"] = len(self.airports)

    def _load_terminal(self, g: str) -> None:
        sub = g[12]
        apt = g[6:10].strip()
        if g[5] == "N":  # terminal NDB: "PN" sits in columns 5-6 and the airport follows
            if g[21] not in "01":
                return
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:
                return
            ident = g[13:17].strip()
            row = {"apt": apt, "lat": lat, "lon": lon, "magvar": parse_magvar(g[74:79])}
            self.termNdb[(ident, g[19:21])].append(row)
            self._byIdent["tndb"][ident].append(row)
            return
        if sub == "F":
            if self.filter is not None and apt not in self.filter:
                return
            cont = g[38]
            if cont in "01":
                r = Record(g)
                self.records[apt].append(r)
                self.counts["records"] += 1
            elif cont == "2" and g[39] == "W" and self.records[apt]:
                self._attach_los(g)
            return
        if g[21] not in "01" and sub != "I":
            return  # continuation records of waypoints etc. carry nothing we need
        if sub == "A":
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:
                return
            self.airports[apt] = {"lat": lat, "lon": lon, "magvar": parse_magvar(g[51:56]),
                                  "elevFt": parse_int(g[56:61]), "name": g[93:123].strip()}
        elif sub == "G":
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:
                return
            ident = g[13:18].strip()
            brg = g[27:31].strip()
            self.runways[(apt, ident)] = {
                "lat": lat, "lon": lon, "magBrg": int(brg) / 10.0 if brg.isdigit() else None,
                "elevFt": parse_int(g[66:71]), "tchFt": parse_int(g[75:77])}
        elif sub == "C":
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:
                return
            ident = g[13:18].strip()
            row = {"apt": apt, "lat": lat, "lon": lon, "magvar": parse_magvar(g[74:79])}
            self.termWpts[(ident, g[19:21])].append(row)
            self._byIdent["term"][ident].append(row)
        elif sub == "I":
            if g[21] not in "01":
                return
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:
                return
            self.localizers[(apt, g[13:17].strip())] = {"lat": lat, "lon": lon}

    def _attach_los(self, g: str) -> None:
        """Continuation 'W' records (RNAV approaches): levels of service authorised at that fix."""
        los = []
        for a in (40, 51, 62):
            flag, name = g[a], g[a + 1:a + 11].strip()
            if flag == "A" and name:
                los.append(name)
        self.records[g[6:10].strip()][-1].los = los

    def _load_enroute_wpt(self, g: str) -> None:
        lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
        if lat is None or lon is None or g[21] not in "01":
            return
        ident = g[13:18].strip()
        row = {"lat": lat, "lon": lon, "magvar": parse_magvar(g[74:79])}
        self.enrWpts[(ident, g[19:21])] = row
        self._byIdent["enr"][ident].append(row)

    def _load_navaid(self, g: str) -> None:
        sub = g[5]
        if g[21] not in "01":
            return
        if sub == " ":  # VHF navaid
            ident = g[13:17].strip()
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:  # DME-only / TACAN: the DME position is the station
                lat, lon = parse_lat(g[55:64]), parse_lon(g[64:74])
            if lat is None or lon is None:
                return
            row = {"lat": lat, "lon": lon, "magvar": parse_magvar(g[74:79])}
            self.vhf[(ident, g[19:21])] = row
            self._byIdent["vhf"][ident].append(row)
        elif sub == "B":  # enroute NDB
            ident = g[13:17].strip()
            lat, lon = parse_lat(g[32:41]), parse_lon(g[41:51])
            if lat is None or lon is None:
                return
            row = {"lat": lat, "lon": lon, "magvar": parse_magvar(g[74:79])}
            self.ndb[(ident, g[19:21])] = row
            self._byIdent["ndb"][ident].append(row)

    # ---- fix resolution
    def resolve(self, apt: str, ident: str, region: str, sec: str, sub: str) -> tuple[cp.Fix, float | None] | None:
        """Fix (with kind) and its local magnetic variation for a leg's fix reference."""
        if not ident:
            return None
        bi = self._byIdent
        if sec == "P" and sub == "C":
            c = self._pick(self.termWpts.get((ident, region)) or bi["term"].get(ident), apt)
            if c:
                return cp.Fix(ident, c["lat"], c["lon"], "FIX"), c["magvar"]
        elif sec == "E" and sub == "A":
            c = self.enrWpts.get((ident, region)) or self._pick(bi["enr"].get(ident), apt)
            if c:
                return cp.Fix(ident, c["lat"], c["lon"], "FIX"), c["magvar"]
        elif sec == "D" and sub == "":
            c = self.vhf.get((ident, region)) or self._pick(bi["vhf"].get(ident), apt)
            if c:
                return cp.Fix(ident, c["lat"], c["lon"], "VOR"), c["magvar"]
        elif sec == "D" and sub == "B":
            c = self.ndb.get((ident, region)) or self._pick(bi["ndb"].get(ident), apt)
            if c:
                return cp.Fix(ident, c["lat"], c["lon"], "NDB"), c["magvar"]
        elif sec == "P" and sub == "N":
            c = self._pick(self.termNdb.get((ident, region)) or bi["tndb"].get(ident), apt)
            if c:
                return cp.Fix(ident, c["lat"], c["lon"], "NDB"), c["magvar"]
        elif sec == "P" and sub == "G":
            r = self.runways.get((apt, ident))
            if r:
                return cp.Fix(ident, r["lat"], r["lon"], "RWY"), None
        elif sec == "P" and sub == "A":
            a = self.airports.get(ident)
            if a:
                return cp.Fix(ident, a["lat"], a["lon"], "APT"), a["magvar"]
        elif sec == "P" and sub == "I":
            c = self.localizers.get((apt, ident))
            if c:
                return cp.Fix(ident, c["lat"], c["lon"], "FIX"), None
        return None

    def navaid_position(self, apt: str, rec: Record) -> tuple[float, float, float | None] | None:
        """Recommended navaid of a leg (VOR/NDB/localizer): lat, lon, declination."""
        if not rec.navId:
            return None
        r = self.resolve(apt, rec.navId, rec.navRegion, rec.navSec, rec.navSub)
        if r is None:
            return None
        return r[0].lat, r[0].lon, r[1]

    @staticmethod
    def _pick(cands: list[dict] | None, apt: str) -> dict | None:
        if not cands:
            return None
        for c in cands:
            if c.get("apt") == apt:
                return c
        return cands[0]


# --------------------------------------------------------------------------------------------
# approach identifiers
# --------------------------------------------------------------------------------------------
KIND_BY_TYPE = {
    "I": "ILS", "M": "ILS", "W": "ILS", "Y": "ILS",
    "L": "LOC", "B": "LOC", "X": "LOC", "U": "LOC", "G": "LOC",
    "R": "RNAV", "H": "RNAV", "P": "RNAV", "J": "RNAV", "F": "RNAV",
    "V": "VOR", "S": "VOR", "D": "VOR", "T": "VOR",
    "N": "NDB", "Q": "NDB",
}
TITLE_BY_TYPE = {
    "I": "ILS", "M": "MLS", "W": "MLS", "Y": "MLS", "L": "LOC", "B": "LOC BC", "X": "LDA",
    "U": "SDF", "G": "IGS", "R": "RNAV (GPS)", "H": "RNAV (RNP)", "P": "GPS", "J": "GLS",
    "F": "FMS", "V": "VOR", "S": "VOR/DME", "D": "VOR/DME", "T": "TACAN", "N": "NDB", "Q": "NDB/DME",
}
_IDENT_RE = re.compile(r"^([A-Z])(\d{2})([LRC]?)(?:-?([A-Z]))?$")


def parse_approach_ident(ident: str) -> tuple[str, str, str]:
    """ARINC approach ident -> (type letter, runway in DB style or '', multiple indicator)."""
    m = _IDENT_RE.match(ident)
    if m:
        return m.group(1), m.group(2) + m.group(3), m.group(4) or ""
    return ident[:1], "", (ident[-1] if re.search(r"-[A-Z]$", ident) else "")


def approach_title(ident: str) -> str:
    """Human title derived from the ARINC ident: 'I05-Z' -> 'ILS Z RWY 05', 'VOR-B' -> 'VOR B'."""
    t, rwy, mult = parse_approach_ident(ident)
    if rwy:
        return f"{TITLE_BY_TYPE.get(t, t)}{(' ' + mult) if mult else ''} RWY {rwy}"
    return ident.replace("-", " ").strip()


# --------------------------------------------------------------------------------------------
# leg conversion
# --------------------------------------------------------------------------------------------
NATIVE_TYPES = ("IF", "TF", "CF", "DF", "CA", "VA", "FA", "RF", "HM", "HA", "HF")
APPROX_TYPES = ("AF", "PI", "VD", "VI", "VM", "CI", "CD", "CR", "VR", "FC", "FD", "FM")
# What each approximated ARINC path terminator becomes (documented in NAVIGATION 3.5 / DATA 7).
APPROX_NOTES = {
    "AF": "-> RF (DME arc about the recommended navaid)",
    "PI": "-> TF to the PT fix (course reversal NOT flown; dropped when the fix repeats)",
    "VI": "-> dropped (the next CF/DF leg intercepts)",
    "CI": "-> dropped (the next CF/DF leg intercepts)",
    "VM": "-> dropped (vectors to the next leg)",
    "FM": "-> dropped (vectors to the next leg)",
    "VD": "-> CF to the computed DME-distance point",
    "CD": "-> CF to the computed DME-distance point",
    "FD": "-> TF fix + CF to the computed DME-distance point",
    "VR": "-> CF to the computed radial-intercept point",
    "CR": "-> CF to the computed radial-intercept point",
    "FC": "-> TF fix + TF to the point `distance` along the track",
}
ALT_AT, ALT_ABOVE, ALT_BELOW, ALT_BETWEEN = "AT", "ATORABOVE", "ATORBELOW", "BETWEEN"
_ALT_DESC = {" ": ALT_AT, "@": ALT_AT, "+": ALT_ABOVE, "-": ALT_BELOW, "B": ALT_BETWEEN,
             "C": ALT_ABOVE, "G": ALT_AT, "I": ALT_AT, "H": ALT_ABOVE, "J": ALT_ABOVE,
             "V": ALT_ABOVE, "X": ALT_AT, "Y": ALT_BELOW}


class Stats:
    """Coverage numbers over every approach of every processed airport."""

    def __init__(self) -> None:
        self.airports = 0
        self.approaches = 0
        self.byKind: Counter = Counter()
        self.transitions = 0
        self.legsOut = 0
        self.records: Counter = Counter()  # ARINC terminator -> source records
        self.native: Counter = Counter()
        self.approx: Counter = Counter()
        self.dropped: Counter = Counter()  # (terminator, reason)
        self.unresolved: Counter = Counter()  # (section, subsection)
        self.unresolvedExamples: list[str] = []
        self.noMap = 0
        self.noMapExamples: list[str] = []
        self.approachesWithApprox = 0
        self.altCodes: Counter = Counter()
        self.skippedApproaches: Counter = Counter()
        self.logLines: list[str] = []

    def log(self, msg: str) -> None:
        if len(self.logLines) < 5000:
            self.logLines.append(msg)


class Ctx:
    """Per-approach conversion context."""

    def __init__(self, db: CifpDb, apt: str, stats: Stats, approachId: str):
        self.db, self.apt, self.stats, self.approachId = db, apt, stats, approachId
        a = db.airports.get(apt)
        self.aptMagVar = a["magvar"] if a and a["magvar"] is not None else 0.0
        self.approxUsed = False

    def where(self, rec: Record) -> str:
        return f"{self.apt} {rec.ident} {rec.trans or 'FINAL'} {rec.seq} {rec.pt}"


def local_magvar(ctx: Ctx, rec: Record) -> float:
    """Variation that FAA used to publish this leg's magnetic course: the station declination of
    the recommended navaid when it is a VHF navaid, otherwise the airport's. (Verified against the
    theta/rho fields of ~17 000 legs: median error 0.1 deg; fix-record variations were 1.2 deg off.)"""
    if rec.navId:
        nv = ctx.db.navaid_position(ctx.apt, rec)
        if nv is not None and nv[2] is not None and rec.navSec == "D":
            return nv[2]
    return ctx.aptMagVar


def course_true(ctx: Ctx, rec: Record) -> float | None:
    c = rec.courseDeg()
    if c is None:
        return None
    deg, isTrue = c
    return norm360(deg if isTrue else deg + local_magvar(ctx, rec))


def alt_constraint(ctx: Ctx, rec: Record) -> tuple[str | None, float, float]:
    if rec.alt1 is None:
        return None, 0.0, 0.0
    kind = _ALT_DESC.get(rec.altDesc)
    if kind is None:
        ctx.stats.altCodes[rec.altDesc] += 1
        kind = ALT_AT
    if kind == ALT_BETWEEN:
        if rec.alt2 is None:
            return ALT_AT, float(rec.alt1), 0.0
        lo, hi = sorted((rec.alt1, rec.alt2))
        return ALT_BETWEEN, float(lo), float(hi)
    return kind, float(rec.alt1), 0.0


def _base_leg(ctx: Ctx, rec: Record, ltype: str, fix: cp.Fix | None) -> cp.Leg:
    leg = cp.Leg(ltype, fix)
    leg.altKind, leg.alt, leg.alt2 = alt_constraint(ctx, rec)
    leg.speedKt = rec.speedKt
    leg.flyOver = rec.isFlyOver
    if rec.vertAngleDeg is not None:
        leg.vertAngleDeg = rec.vertAngleDeg
    if rec.isIaf:
        leg.flags.add("IAF")
    if rec.isFaf:
        leg.flags.add("FAF")
    if rec.isMap:
        leg.flags.add("MAP")
    return leg


def _move(lat: float, lon: float, brgTrueDeg: float, distNm: float) -> tuple[float, float]:
    return navgeo.destination(lat, lon, brgTrueDeg, distNm * navgeo.M_PER_NM)


def _dist_nm(a: tuple[float, float], b: tuple[float, float]) -> float:
    return navgeo.distance_m(a[0], a[1], b[0], b[1]) * NM_PER_M


def find_along_track(start: tuple[float, float], crsTrueDeg: float,
                     f, maxNm: float = 80.0, stepNm: float = 0.1) -> tuple[float, float] | None:
    """First point along the great-circle course from `start` where f(point) changes sign."""
    prevPt = start
    prevV = f(start)
    d = stepNm
    while d <= maxNm:
        pt = _move(start[0], start[1], crsTrueDeg, d)
        v = f(pt)
        if prevV == 0.0:
            return prevPt
        if (prevV < 0.0) != (v < 0.0):
            t = prevV / (prevV - v) if prevV != v else 0.5  # refine linearly between samples
            return _move(start[0], start[1], crsTrueDeg, d - stepNm + t * stepNm)
        prevPt, prevV = pt, v
        d += stepNm
    return None


def _fmt_nm(v: float) -> str:
    return f"{v:.1f}".rstrip("0").rstrip(".")


def convert_record(ctx: Ctx, rec: Record, prevFix: cp.Fix | None) -> tuple[list[cp.Leg], str]:
    """Convert one ARINC record. Returns (legs, outcome) where outcome is 'native', 'approx[:..]',
    'dropped:<reason>' or 'unresolved'. The legs of a dropped record are empty."""
    db, apt, st = ctx.db, ctx.apt, ctx.stats
    pt = rec.pt
    fixInfo = None
    if rec.fix:
        fixInfo = db.resolve(apt, rec.fix, rec.fixRegion, rec.fixSec, rec.fixSub.strip())
        if fixInfo is None:
            st.unresolved[(rec.fixSec, rec.fixSub.strip())] += 1
            if len(st.unresolvedExamples) < 30:
                st.unresolvedExamples.append(
                    f"{ctx.where(rec)} fix {rec.fix} ({rec.fixSec}{rec.fixSub.strip()})")
            return [], "unresolved"
    fix = fixInfo[0] if fixInfo else None
    crs = course_true(ctx, rec)

    # ---- native types
    if pt in ("IF", "TF", "DF"):
        if fix is None:
            return [], "dropped:no fix"
        leg = _base_leg(ctx, rec, pt, fix)
        if rec.turn in ("L", "R"):
            leg.turn = rec.turn
        return [leg], "native"
    if pt == "CF":
        if fix is None:
            return [], "dropped:no fix"
        leg = _base_leg(ctx, rec, "CF", fix)
        leg.crsTrue = crs
        if crs is None:
            leg.type = "TF"
            st.log(f"{ctx.where(rec)}: CF without course -> TF")
        return [leg], "native"
    if pt in ("CA", "VA"):
        if crs is None:
            return [], "dropped:no course"
        leg = _base_leg(ctx, rec, pt, None)
        leg.crsTrue = crs
        if rec.turn in ("L", "R"):
            leg.turn = rec.turn
        return [leg], "native"
    if pt == "FA":
        if fix is None or crs is None:
            return [], "dropped:no fix/course"
        leg = _base_leg(ctx, rec, "FA", fix)
        leg.crsTrue = crs
        return [leg], "native"
    if pt == "RF":
        if fix is None:
            return [], "dropped:no fix"
        c = db.resolve(apt, rec.centerFix, rec.centerRegion, rec.centerSec, rec.centerSub)
        if c is None or rec.turn not in ("L", "R"):
            leg = _base_leg(ctx, rec, "TF", fix)
            leg.orig = "RF"
            st.log(f"{ctx.where(rec)}: RF without centre/turn -> TF")
            return [leg], "approx:RF->TF"
        leg = _base_leg(ctx, rec, "RF", fix)
        radius = rec.arcRadiusNm or _dist_nm((c[0].lat, c[0].lon), (fix.lat, fix.lon))
        leg.arc = (c[0].lat, c[0].lon, radius, rec.turn)
        leg.turn = rec.turn
        return [leg], "native"
    if pt in ("HM", "HA", "HF"):
        if fix is None or crs is None:
            return [], "dropped:no fix/course"
        leg = _base_leg(ctx, rec, pt, fix)
        leg.crsTrue = crs
        leg.turn = rec.turn if rec.turn in ("L", "R") else "R"
        t, d = rec.timeMin(), rec.distNm()
        if t is not None:
            leg.holdTimeMin = t
        elif d is not None:
            leg.holdDistNm = d
        return [leg], "native"

    # ---- approximated types
    ctx.approxUsed = True
    if pt == "AF":
        nv = db.navaid_position(apt, rec)
        if fix is None:
            return [], "dropped:no fix"
        if nv is None or rec.turn not in ("L", "R"):
            leg = _base_leg(ctx, rec, "TF", fix)
            leg.orig = "AF"
            st.log(f"{ctx.where(rec)}: AF without navaid/turn -> TF")
            return [leg], "approx:AF->TF"
        leg = _base_leg(ctx, rec, "RF", fix)
        radius = _dist_nm((nv[0], nv[1]), (fix.lat, fix.lon))  # puts the fix exactly on the arc
        if rec.rhoNm and abs(radius - rec.rhoNm) > 0.3:
            st.log(f"{ctx.where(rec)}: AF rho {rec.rhoNm} vs geometry {radius:.2f} NM")
        leg.arc = (nv[0], nv[1], radius, rec.turn)
        leg.turn = rec.turn
        leg.orig = "AF"
        return [leg], "approx"
    if pt == "PI":
        if fix is None:
            return [], "dropped:no fix"
        if prevFix is not None and prevFix.ident == fix.ident:
            return [], "dropped:PI at the previous fix (procedure turn not flown)"
        leg = _base_leg(ctx, rec, "TF", fix)
        leg.orig = "PI"
        return [leg], "approx"
    if pt in ("VI", "CI"):
        return [], "dropped:" + pt + " (next leg intercepts)"
    if pt in ("VM", "FM"):
        return [], "dropped:" + pt + " (vectors)"
    if pt in ("VD", "CD", "FD", "VR", "CR"):
        nv = db.navaid_position(apt, rec)
        start = None
        legs: list[cp.Leg] = []
        if pt == "FD":
            if fix is None:
                return [], "dropped:no fix"
            start = (fix.lat, fix.lon)
            if prevFix is None or prevFix.ident != fix.ident:
                first = _base_leg(ctx, rec, "TF", fix)
                first.altKind, first.alt, first.alt2 = None, 0.0, 0.0
                first.flags.clear()
                first.orig = pt
                legs.append(first)
        elif prevFix is not None:
            start = (prevFix.lat, prevFix.lon)
        if start is None or nv is None or crs is None:
            return [], f"dropped:{pt} without start/navaid/course"
        navPt = (nv[0], nv[1])
        if pt in ("VD", "CD", "FD"):
            dme = rec.distNm()
            if dme is None:
                return [], f"dropped:{pt} without distance"
            end = find_along_track(start, crs, lambda p: _dist_nm(p, navPt) - dme)
            name = f"{rec.navId}{_fmt_nm(dme)}"
        else:
            if rec.thetaDeg is None:
                return [], f"dropped:{pt} without radial"
            radialTrue = norm360(rec.thetaDeg + local_magvar(ctx, rec))

            def f(p, navPt=navPt, radialTrue=radialTrue):
                b = navgeo.initial_bearing_true_deg(navPt[0], navPt[1], p[0], p[1])
                return (b - radialTrue + 540.0) % 360.0 - 180.0

            end = find_along_track(start, crs, f)
            name = f"{rec.navId}R{int(round(rec.thetaDeg)):03d}"
        if end is None:
            return [], f"dropped:{pt} termination not found"
        leg = _base_leg(ctx, rec, "CF", cp.Fix(name, end[0], end[1], "SYN"))
        leg.crsTrue = crs
        leg.orig = pt
        legs.append(leg)
        return legs, "approx"
    if pt == "FC":
        dist = rec.distNm()
        if fix is None or crs is None or dist is None:
            return [], "dropped:FC without fix/course/distance"
        legs = []
        if prevFix is None or prevFix.ident != fix.ident:
            first = _base_leg(ctx, rec, "TF", fix)
            first.altKind, first.alt, first.alt2 = None, 0.0, 0.0
            first.flags.clear()
            first.orig = "FC"
            legs.append(first)
        lat, lon = _move(fix.lat, fix.lon, crs, dist)
        leg = _base_leg(ctx, rec, "TF", cp.Fix(f"{fix.ident}{_fmt_nm(dist)}", lat, lon, "SYN"))
        leg.orig = "FC"
        legs.append(leg)
        return legs, "approx"
    return [], f"dropped:unknown terminator {pt}"


# --------------------------------------------------------------------------------------------
# approach assembly
# --------------------------------------------------------------------------------------------
def convert_segment(ctx: Ctx, recs: list[Record]) -> list[cp.Leg]:
    """Convert the records of one transition or final segment into runtime legs."""
    st = ctx.stats
    legs: list[cp.Leg] = []
    prevFix: cp.Fix | None = None
    bridged = False  # a VI/CI/VM/FM record was dropped: the next TF becomes a DF
    for rec in recs:
        pt = rec.pt
        st.records[pt] += 1
        out, outcome = convert_record(ctx, rec, prevFix)
        if outcome == "native":
            st.native[pt] += 1
        elif outcome.startswith("approx"):
            st.approx[pt] += 1
        else:
            reason = "unresolved fix" if outcome == "unresolved" else outcome.split(":", 1)[1]
            st.dropped[(pt, reason)] += 1
            if outcome == "unresolved":
                ctx.approxUsed = True
            if pt in APPROX_TYPES:
                st.approx[pt] += 0  # keeps the terminator listed in the report
        if not out:
            # a dropped record can still carry the FAF / MAP flag: hand it to the previous leg
            if legs:
                if rec.isMap:
                    legs[-1].flags.add("MAP")
                if rec.isFaf:
                    legs[-1].flags.add("FAF")
            if pt in ("VI", "CI", "VM", "FM"):
                bridged = True
            continue
        if bridged and out[0].type == "TF":
            out[0].type = "DF"
            st.log(f"{ctx.where(rec)}: TF after a dropped {pt}-style leg became DF")
        bridged = False
        legs.extend(out)
        prevFix = legs[-1].fix
    return legs


def split_missed(ctx: Ctx, legs: list[cp.Leg], label: str) -> tuple[list[cp.Leg], list[cp.Leg]]:
    """Final segment -> (legs up to and including the MAP, missed approach legs)."""
    idx = next((i for i, l in enumerate(legs) if "MAP" in l.flags), None)
    if idx is None:  # no MAP flag: fall back to the runway fix
        idx = next((i for i, l in enumerate(legs) if l.fix and l.fix.kind == "RWY"), None)
        if idx is not None:
            legs[idx].flags.add("MAP")
    if idx is None:
        ctx.stats.noMap += 1
        if len(ctx.stats.noMapExamples) < 30:
            ctx.stats.noMapExamples.append(label)
        return legs, []
    return legs[:idx + 1], legs[idx + 1:]


def build_approach(db: CifpDb, apt: str, ident: str, recs: list[Record], stats: Stats) -> cp.Approach | None:
    typ, rwy, _mult = parse_approach_ident(ident)
    ctx = Ctx(db, apt, stats, ident)
    trans: dict[str, list[Record]] = {}
    finals: dict[str, list[Record]] = {}
    for r in sorted(recs, key=lambda r: (r.seq, )):
        if r.isTransition:
            trans.setdefault(r.trans, []).append(r)
        else:
            finals.setdefault(r.rtype, []).append(r)
    if not finals:
        stats.skippedApproaches["no final segment"] += 1
        return None
    if len(finals) > 1:  # several final route types under one ident: keep the first in file order
        keep = next(r.rtype for r in recs if not r.isTransition)
        stats.log(f"{apt} {ident}: several final route types {sorted(finals)}; keeping {keep}")
        finals = {keep: finals[keep]}
    finalRecs = next(iter(finals.values()))
    appr = cp.Approach(ident, KIND_BY_TYPE.get(typ, "NONE"), rwy, approach_title(ident), apt)
    los: list[str] = []
    for r in finalRecs:
        for name in r.los:
            if name not in los:
                los.append(name)
    appr.los = los
    finalLegs = convert_segment(ctx, finalRecs)
    if not finalLegs:
        stats.skippedApproaches["final segment empty after conversion"] += 1
        return None
    appr.final, appr.missed = split_missed(ctx, finalLegs, f"{apt} {ident}")
    for name, rs in trans.items():
        legs = convert_segment(ctx, rs)
        if legs:
            appr.transitions.append(cp.Transition(name, legs))
            stats.transitions += 1
    appr.transitions.sort(key=lambda t: t.name)
    stats.approaches += 1
    stats.byKind[appr.kind] += 1
    if ctx.approxUsed:
        stats.approachesWithApprox += 1
    stats.legsOut += (len(appr.final) + len(appr.missed)
                      + sum(len(t.legs) for t in appr.transitions))
    return appr


def build_airport(db: CifpDb, apt: str, stats: Stats) -> list[cp.Approach]:
    groups: dict[str, list[Record]] = {}
    for r in db.records.get(apt, []):
        groups.setdefault(r.ident, []).append(r)
    out = []
    for ident in sorted(groups):
        a = build_approach(db, apt, ident, groups[ident], stats)
        if a is not None:
            out.append(a)
    return out


# --------------------------------------------------------------------------------------------
# report
# --------------------------------------------------------------------------------------------
def format_report(db: CifpDb, stats: Stats, seconds: float, airportsWritten: int) -> str:
    L: list[str] = []
    A = L.append
    A("CIFP coverage report (tools/nav/cifp.py)")
    A(f"cycle {db.cycle} effective {db.effective or '?'}; airports with procedures: {airportsWritten}"
      f" (of {len(db.records)} with approach records); {seconds:.1f} s")
    A(f"approaches written: {stats.approaches}  by kind: "
      + ", ".join(f"{k} {v}" for k, v in sorted(stats.byKind.items())))
    A(f"transitions: {stats.transitions}; legs written: {stats.legsOut}")
    A(f"approaches with at least one approximated or dropped leg: {stats.approachesWithApprox}"
      f" ({100.0 * stats.approachesWithApprox / max(1, stats.approaches):.1f}%)")
    A(f"approaches without a flagged MAP (no missed approach emitted): {stats.noMap}")
    if stats.skippedApproaches:
        A("approaches skipped: " + ", ".join(f"{k}: {v}" for k, v in stats.skippedApproaches.items()))
    A("")
    A("path terminators (source records)")
    A(f"  {'type':<5}{'records':>9}{'native':>9}{'approx':>9}{'dropped':>9}  handling")
    total = sum(stats.records.values())
    for t in sorted(stats.records, key=lambda k: -stats.records[k]):
        dropped = sum(v for (tt, _r), v in stats.dropped.items() if tt == t)
        if t in NATIVE_TYPES:
            note = "native"
        elif t in APPROX_NOTES:
            note = "approximated " + APPROX_NOTES[t]
        else:
            note = "unsupported (dropped)"
        A(f"  {t:<5}{stats.records[t]:>9}{stats.native[t]:>9}{stats.approx[t]:>9}{dropped:>9}  {note}")
    nat = sum(stats.native.values())
    app = sum(stats.approx.values())
    drp = sum(stats.dropped.values())
    A(f"  total {total} records: {nat} native ({100.0 * nat / max(1, total):.2f}%), "
      f"{app} approximated ({100.0 * app / max(1, total):.2f}%), {drp} dropped "
      f"({100.0 * drp / max(1, total):.2f}%; includes legs whose fix could not be resolved)")
    A("")
    A("dropped records by reason")
    for (t, reason), n in sorted(stats.dropped.items(), key=lambda kv: -kv[1]):
        A(f"  {n:>6}  {t}: {reason}")
    if stats.unresolved:
        A("")
        A("unresolved fix references (section+subsection: count)")
        for k, v in stats.unresolved.items():
            A(f"  {k[0]}{k[1]}: {v}")
        for e in stats.unresolvedExamples[:10]:
            A("    e.g. " + e)
    if stats.altCodes:
        A("")
        A("unknown altitude description codes (treated as AT): "
          + ", ".join(f"{k!r}: {v}" for k, v in stats.altCodes.items()))
    if stats.noMapExamples:
        A("")
        A("approaches without a MAP flag (first 30): " + ", ".join(stats.noMapExamples))
    return "\n".join(L) + "\n"


# --------------------------------------------------------------------------------------------
# driver
# --------------------------------------------------------------------------------------------
def normalise_airports(spec: str | None) -> set[str] | None:
    if not spec:
        return None
    out: set[str] = set()
    for a in spec.replace(";", ",").split(","):
        a = a.strip().upper()
        if a:
            out.add(a)
            if len(a) == 3:
                out.add("K" + a)
    return out


def run(source: str, outDir: Path, airports: set[str] | None = None, offline: bool = False,
        todayIso: str | None = None, reportPath: Path | None = None, quiet: bool = False,
        updateNavIni: bool = True) -> dict:
    """Parse CIFP and write ``outDir/procedures/*.proc``. Returns a summary dict."""
    log = (lambda *a: None) if quiet else (lambda *a: print(*a, flush=True))
    t0 = time.time()
    today = todayIso or dt.datetime.now(dt.timezone.utc).date().isoformat()
    path, dateIso = locate_cifp(source, offline, today)
    log(f"CIFP source: {path}")
    db = CifpDb(airports)
    db.load(open_cifp_lines(path))
    if not db.cycle:
        raise CifpError(f"{path}: no HDR04 volume line found (is this FAACIFP18?)")
    log(f"  cycle {db.cycle} (effective {db.effective}); {len(db.airports)} airports, "
        f"{sum(len(v) for v in db.termWpts.values())} terminal and {len(db.enrWpts)} enroute waypoints, "
        f"{len(db.vhf)} VHF navaids, {len(db.runways)} runways, {db.counts['records']} approach records")
    stats = Stats()
    procDir = outDir / "procedures"
    procDir.mkdir(parents=True, exist_ok=True)
    if airports is None:
        for old in procDir.glob("*.proc"):
            old.unlink()
    written = 0
    for apt in sorted(db.records):
        if airports is not None and apt not in airports:
            continue
        approaches = build_airport(db, apt, stats)
        stats.airports += 1
        if not approaches:
            continue
        text = cp.write_proc(apt, approaches, db.cycle, db.effective)
        (procDir / f"{apt}.proc").write_text(text, encoding="utf-8")
        written += 1
    seconds = time.time() - t0
    report = format_report(db, stats, seconds, written)
    rp = reportPath or (procDir / "coverage_report.txt")
    rp.parent.mkdir(parents=True, exist_ok=True)
    rp.write_text(report, encoding="utf-8")
    if updateNavIni:
        update_nav_ini(outDir / "nav.ini", db.cycle)
    if airports is not None:
        missing = sorted(a for a in airports if a not in db.records)
        if missing:
            log("  note: no approach records for " + ", ".join(missing))
    log(f"  {stats.approaches} approaches at {written} airports written to {procDir} ({seconds:.1f} s)")
    log(f"  report: {rp}")
    return {"cycle": db.cycle, "effective": db.effective, "airports": written,
            "approaches": stats.approaches, "stats": stats, "report": report, "db": db}


def update_nav_ini(path: Path, cycle: str) -> None:
    """Set `faa_cifp = <cycle>` in an existing nav.ini (written by build_navdb.py)."""
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return
    if re.search(r"^faa_cifp\s*=.*$", text, re.M):
        text = re.sub(r"^faa_cifp\s*=.*$", f"faa_cifp = {cycle}", text, flags=re.M)
    else:
        text = re.sub(r"^(\[sources\]\s*)$", rf"\1\nfaa_cifp = {cycle}", text, count=1, flags=re.M)
    path.write_text(text, encoding="utf-8")


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cifp", required=True, metavar="ZIP|FILE|DIR|auto",
                    help="FAA CIFP source: 'auto' downloads the current cycle (CIFP_YYMMDD.zip from "
                         "aeronav.faa.gov) into cache/; or a zip / a bare FAACIFP18 file / a directory")
    ap.add_argument("--out", required=True, type=Path,
                    help="nav directory (as written by build_navdb.py); procedures/ is created inside")
    ap.add_argument("--airports", default=None, metavar="KOTH,KPDX",
                    help="only these airports (comma separated, 3-letter idents get a K prefix too)")
    ap.add_argument("--report", type=Path, default=None,
                    help="where to write the coverage report (default <out>/procedures/coverage_report.txt)")
    ap.add_argument("--offline", action="store_true", help="never touch the network (cache only)")
    ap.add_argument("--today", default=None, metavar="YYYY-MM-DD",
                    help="date used to pick the current cycle (default: today, UTC)")
    ap.add_argument("--print-report", action="store_true", help="also print the coverage report")
    ap.add_argument("--quiet", action="store_true")
    a = ap.parse_args(argv)
    try:
        res = run(a.cifp, a.out, normalise_airports(a.airports), a.offline, a.today, a.report, a.quiet)
    except CifpError as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if a.print_report:
        print(res["report"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
