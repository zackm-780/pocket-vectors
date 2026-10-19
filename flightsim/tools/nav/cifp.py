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
            if line[0] not in "ST" or line[1:4] != "USA":
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
