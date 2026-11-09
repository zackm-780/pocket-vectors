#!/usr/bin/env python3
"""OpenStreetMap taxiways and aprons for the nav pack (WP W8a; DATA.md 5.8-5.10; PLAN D23).

Reads airports.csv and runways.csv from --nav-dir, takes the airports with at least one hard
runway inside the pack's region (--bbox; airports.csv is worldwide, about 12,700 such airports),
fetches their aeroway=taxiway / taxilane / apron ways from the Overpass API (one query per airport
box, cached under cache/osm/), and writes next to them:

    taxiways.csv      one row per segment: an OSM way split at nodes it shares with other taxiways
    aprons.csv        one row per apron (outer ring, not closed)
    TAXIWAYS-ODbL.txt the licence note: the two files are a derivative database of OpenStreetMap

Standard library only. The taxiway files are ODbL 1.0 (c) OpenStreetMap contributors; the rest of
the pack keeps its own terms.

Examples:
    osm_taxiways.py --nav-dir ../../data/nav --bbox=-124.7,43.0,-121.4,46.3   # a region, Overpass
    osm_taxiways.py --nav-dir ../../data/nav --airports KPDX,KOTH             # a subset
    osm_taxiways.py --nav-dir ../../data/nav --osm-file kpdx.osm --osm-file koth.osm  # saved XML
    osm_taxiways.py --nav-dir ../../data/nav --offline              # cached answers only
"""
from __future__ import annotations

import argparse
import csv
import datetime as dt
import io
import math
import re
import sys
import time
import urllib.error
import urllib.parse
import urllib.request
import xml.etree.ElementTree as ET
from dataclasses import dataclass, field
from pathlib import Path

import ourairports as oa

HERE = Path(__file__).resolve().parent
DEFAULT_CACHE = HERE / "cache" / "osm"
DEFAULT_OVERPASS = "https://overpass-api.de/api/interpreter"
USER_AGENT = "pocket-vectors-datapack/1.3 (+https://github.com/zackm-780/pocket-vectors)"
RETRY_DELAYS_S = (5, 10, 20, 40)
REQUEST_TIMEOUT_S = 100  # the query asks the server for at most 90 s
FAIL_FAST_AIRPORTS = 5   # the first this many all failing: Overpass is down, stop
M_PER_DEG_LAT = 111320.0

TAXIWAY_COLS = ["airport_ident", "osm_way_id", "kind", "ref", "width_m", "surface", "points"]
APRON_COLS = ["airport_ident", "osm_way_id", "surface", "points"]
LIFECYCLE_KEYS = ("disused", "abandoned", "construction", "proposed")

# Default widths (TAXIWAYS.md section 1): a `width` tag wins.
TAXILANE_WIDTH_M = 7.5
TAXIWAY_WIDTH_M = {"large": 23.0, "medium": 15.0}
SMALL_TAXIWAY_WIDTH_M = 10.5

ATTRIBUTION_LINE = ("Taxiways and aprons (nav/taxiways.csv, nav/aprons.csv): © OpenStreetMap "
                    "contributors, ODbL 1.0, see nav/TAXIWAYS-ODbL.txt.")


# --------------------------------------------------------------------------------------------
# input: airports with hard runways
# --------------------------------------------------------------------------------------------
@dataclass
class AirportBox:
    ident: str
    type: str
    lat_deg: float
    lon_deg: float
    surface: str  # of the longest runway
    runways: list[tuple[float, float, float, float]] = field(default_factory=list)  # lat/lon ends
    south: float = 0.0
    west: float = 0.0
    north: float = 0.0
    east: float = 0.0

    def contains(self, lat: float, lon: float) -> bool:
        return self.south <= lat <= self.north and self.west <= lon <= self.east


def _float(s: str) -> float | None:
    try:
        v = float(s)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def parse_bbox(text: str) -> tuple[float, float, float, float]:
    """'W,S,E,N' in degrees."""
    parts = [float(v) for v in text.split(",")]
    if len(parts) != 4 or not (parts[0] < parts[2] and parts[1] < parts[3]):
        raise ValueError(f"bad --bbox {text!r}: want west,south,east,north")
    return parts[0], parts[1], parts[2], parts[3]


def read_airports(nav_dir: Path, subset: set[str] | None, margin_m: float,
                  region: tuple[float, float, float, float] | None = None) -> list[AirportBox]:
    """Airports of airports.csv with at least one hard runway (and their reference point inside
    `region` when given), boxed by their runways + margin."""
    with open(nav_dir / "airports.csv", encoding="utf-8-sig", newline="") as f:
        apts = {r["ident"].upper(): r for r in csv.DictReader(f)}
    runways: dict[str, list[dict]] = {}
    with open(nav_dir / "runways.csv", encoding="utf-8-sig", newline="") as f:
        for r in csv.DictReader(f):
            runways.setdefault(r["airport_ident"].upper(), []).append(r)
    out = []
    for ident in sorted(runways):
        if subset is not None and ident not in subset:
            continue
        a = apts.get(ident)
        rows = runways[ident]
        if a is None or not any(r.get("hard") == "1" for r in rows):
            continue
        lat, lon = _float(a.get("lat_deg")), _float(a.get("lon_deg"))
        if lat is None or lon is None:
            continue
        if region and not (region[0] <= lon <= region[2] and region[1] <= lat <= region[3]):
            continue
        box = AirportBox(ident, (a.get("type") or "").lower(), lat, lon, "other")
        best_len = -1.0
        for r in rows:
            ends = [_float(r.get(k)) for k in ("start_lat_deg", "start_lon_deg", "end_lat_deg",
                                               "end_lon_deg")]
            if any(v is None for v in ends):
                continue
            box.runways.append(tuple(ends))  # type: ignore[arg-type]
            length = _float(r.get("length_ft")) or 0.0
            if length > best_len:
                best_len = length
                box.surface = r.get("surface") or "other"
        if not box.runways:
            continue
        lats = [p for rw in box.runways for p in (rw[0], rw[2])]
        lons = [p for rw in box.runways for p in (rw[1], rw[3])]
        dlat = margin_m / M_PER_DEG_LAT
        dlon = margin_m / (M_PER_DEG_LAT * max(0.05, math.cos(math.radians(lat))))
        box.south, box.north = min(lats) - dlat, max(lats) + dlat
        box.west, box.east = min(lons) - dlon, max(lons) + dlon
        out.append(box)
    return out


# --------------------------------------------------------------------------------------------
# OSM XML (Overpass answers and OSM API `map` calls share the format)
# --------------------------------------------------------------------------------------------
@dataclass
class Way:
    id: int
    tags: dict[str, str]
    refs: list[int]


class OsmData:
    def __init__(self) -> None:
        self.nodes: dict[int, tuple[float, float]] = {}
        self.ways: dict[int, Way] = {}
        self.relations = 0  # aeroway=apron relations seen (multipolygons; skipped)

    def add_xml(self, data: bytes) -> None:
        """Adds one OSM XML document; elements may come in any order."""
        root = ET.fromstring(data)
        for el in root:
            if el.tag == "node":
                lat, lon = _float(el.get("lat")), _float(el.get("lon"))
                if lat is not None and lon is not None:
                    self.nodes[int(el.get("id"))] = (lat, lon)
            elif el.tag == "way":
                tags = {t.get("k"): t.get("v") for t in el.findall("tag")}
                refs = [int(n.get("ref")) for n in el.findall("nd")]
                self.ways[int(el.get("id"))] = Way(int(el.get("id")), tags, refs)
            elif el.tag == "relation":
                if any(t.get("k") == "aeroway" and t.get("v") == "apron"
                       for t in el.findall("tag")):
                    self.relations += 1


def overpass_query(box: AirportBox) -> str:
    bbox = f"{box.south:.6f},{box.west:.6f},{box.north:.6f},{box.east:.6f}"
    return ("[out:xml][timeout:90];"
            f'way["aeroway"~"^(taxiway|taxilane|apron)$"]({bbox});'
            "out body;>;out skel qt;")


def cache_files(cache_dir: Path, ident: str) -> list[Path]:
    return sorted(cache_dir.glob(f"{ident}-*.osm"))


def fetch_airport(box: AirportBox, cache_dir: Path, url: str, offline: bool, today: str,
                  sleep_s: float, state: dict) -> bytes | None:
    """The Overpass answer for one airport: today's cache, else a query (throttled, retried)."""
    todays = cache_dir / f"{box.ident}-{today}.osm"
    if todays.is_file():
        return todays.read_bytes()
    if offline:
        cached = cache_files(cache_dir, box.ident)
        return cached[-1].read_bytes() if cached else None
    body = urllib.parse.urlencode({"data": overpass_query(box)}).encode()
    for attempt in range(len(RETRY_DELAYS_S) + 1):
        wait = state.get("last", 0.0) + sleep_s - time.monotonic()
        if wait > 0:
            time.sleep(wait)
        state["last"] = time.monotonic()
        req = urllib.request.Request(url, data=body, headers={"User-Agent": USER_AGENT})
        try:
            with urllib.request.urlopen(req, timeout=REQUEST_TIMEOUT_S) as resp:
                data = resp.read()
            if b"<osm" not in data[:2000]:
                raise ValueError("not an OSM XML answer")
            cache_dir.mkdir(parents=True, exist_ok=True)
            todays.write_bytes(data)
            return data
        except urllib.error.HTTPError as e:
            retry = e.code == 429 or e.code >= 500
            err = f"HTTP {e.code}"
            # Overpass names its own wait on 429 (rate limit per IP; runners share IPs).
            after = _float(e.headers.get("Retry-After") if e.headers else None)
            if e.code == 429 and after and attempt < len(RETRY_DELAYS_S):
                print(f"osm: {box.ident}: {err}; Overpass asks for {after:.0f} s", flush=True)
                time.sleep(min(after, 120.0))
                continue
        except (urllib.error.URLError, TimeoutError, OSError, ValueError) as e:
            retry = True
            err = str(e)
        if not retry or attempt == len(RETRY_DELAYS_S):
            print(f"osm: {box.ident}: {err}; skipped", flush=True)
            return None
        print(f"osm: {box.ident}: {err}; retry in {RETRY_DELAYS_S[attempt]} s", flush=True)
        time.sleep(RETRY_DELAYS_S[attempt])
    return None


# --------------------------------------------------------------------------------------------
# filters, assignment, segments
# --------------------------------------------------------------------------------------------
def is_live(tags: dict[str, str]) -> bool:
    return all(tags.get(k, "no") == "no" for k in LIFECYCLE_KEYS)


def taxiway_kind(way: Way) -> str | None:
    kind = way.tags.get("aeroway")
    if kind not in ("taxiway", "taxilane") or way.tags.get("area") == "yes":
        return None
    if len(way.refs) < 2 or not is_live(way.tags):
        return None
    return kind


def is_apron(way: Way) -> bool:
    if way.tags.get("aeroway") != "apron" or not is_live(way.tags):
        return False
    return len(way.refs) >= 4 and way.refs[0] == way.refs[-1] and len(set(way.refs[:-1])) >= 3


_WIDTH_RE = re.compile(r"([0-9]*\.?[0-9]+)\s*(m|meters?|metres?|ft|feet|foot|')?")


def parse_width_m(value: str | None) -> float | None:
    """'23', '23 m', '75 ft', "75'" -> metres; None when missing or outside 3-80 m."""
    if not value:
        return None
    m = _WIDTH_RE.fullmatch(value.strip().lower().replace(",", "."))
    if not m:
        return None
    w = float(m.group(1))
    if m.group(2) in ("ft", "feet", "foot", "'"):
        w *= 0.3048
    return w if 3.0 <= w <= 80.0 else None


def default_width_m(kind: str, airport_type: str) -> float:
    if kind == "taxilane":
        return TAXILANE_WIDTH_M
    return TAXIWAY_WIDTH_M.get(airport_type, SMALL_TAXIWAY_WIDTH_M)


_OSM_SURFACE = {
    "asphalt": "asphalt", "paved": "asphalt", "chipseal": "asphalt",
    "concrete": "concrete", "concrete:plates": "concrete", "concrete:lanes": "concrete",
    "paving_stones": "concrete", "sett": "concrete", "bricks": "concrete", "metal": "concrete",
    "grass": "grass", "grass_paver": "grass",
    "dirt": "dirt", "earth": "dirt", "ground": "dirt", "mud": "dirt", "sand": "dirt",
    "unpaved": "dirt", "compacted": "dirt",
    "gravel": "gravel", "fine_gravel": "gravel", "pebblestone": "gravel",
}


def surface_of(tags: dict[str, str], airport_surface: str) -> str:
    raw = (tags.get("surface") or "").strip().lower()
    if not raw:
        return airport_surface
    s = _OSM_SURFACE.get(raw) or oa.norm_surface(raw)
    return airport_surface if s == "other" else s


def _dist_to_segment_m(lat: float, lon: float, rw: tuple[float, float, float, float]) -> float:
    k = M_PER_DEG_LAT * math.cos(math.radians(lat))
    ax, ay = (rw[1] - lon) * k, (rw[0] - lat) * M_PER_DEG_LAT
    bx, by = (rw[3] - lon) * k, (rw[2] - lat) * M_PER_DEG_LAT
    ex, ey = bx - ax, by - ay
    l2 = ex * ex + ey * ey
    t = 0.0 if l2 < 1e-9 else max(0.0, min(1.0, -(ax * ex + ay * ey) / l2))
    return math.hypot(ax + t * ex, ay + t * ey)


def assign_airport(points: list[tuple[float, float]], boxes: list[AirportBox]) -> AirportBox | None:
    """The airport whose box holds the way's mean point (else any of its nodes); nearest runway
    wins a tie."""
    lat = sum(p[0] for p in points) / len(points)
    lon = sum(p[1] for p in points) / len(points)
    cands = [b for b in boxes if b.contains(lat, lon)]
    if not cands:
        cands = [b for b in boxes if any(b.contains(p[0], p[1]) for p in points)]
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]
    return min(cands, key=lambda b: (min(_dist_to_segment_m(lat, lon, rw) for rw in b.runways),
                                     b.ident))


def split_segments(ways: list[Way]) -> list[tuple[Way, list[int]]]:
    """Splits ways at nodes shared by two or more of them, or visited twice by one way."""
    occ: dict[int, int] = {}
    for w in ways:
        for n in set(w.refs):
            occ[n] = occ.get(n, 0) + 1
    out = []
    for w in ways:
        seen: dict[int, int] = {}
        for n in w.refs:
            seen[n] = seen.get(n, 0) + 1
        cur = [w.refs[0]]
        for i in range(1, len(w.refs)):
            n = w.refs[i]
            cur.append(n)
            if i < len(w.refs) - 1 and (occ[n] > 1 or seen[n] > 1):
                out.append((w, cur))
                cur = [n]
        out.append((w, cur))
    return out


def fmt_points(points: list[tuple[float, float]]) -> str:
    return " ".join(f"{lat:.6f} {lon:.6f}" for lat, lon in points)


@dataclass
class Result:
    taxiways: list[dict] = field(default_factory=list)
    aprons: list[dict] = field(default_factory=list)


def extract(osm: OsmData, boxes: list[AirportBox]) -> Result:
    """Filters, assigns and splits the OSM ways into taxiway and apron rows (sorted)."""
    by_airport: dict[str, list[Way]] = {}
    aprons: list[dict] = []
    box_of = {b.ident: b for b in boxes}
    for wid in sorted(osm.ways):
        w = osm.ways[wid]
        kind = taxiway_kind(w)
        apron = kind is None and is_apron(w)
        if kind is None and not apron:
            continue
        if any(n not in osm.nodes for n in w.refs):
            continue  # incomplete way (outside the downloaded area)
        pts = [osm.nodes[n] for n in w.refs]
        box = assign_airport(pts, boxes)
        if box is None:
            continue
        if apron:
            aprons.append({"airport_ident": box.ident, "osm_way_id": w.id,
                           "surface": surface_of(w.tags, box.surface),
                           "points": fmt_points(pts[:-1])})
        else:
            by_airport.setdefault(box.ident, []).append(w)
    res = Result()
    for ident in sorted(by_airport):
        box = box_of[ident]
        for w, refs in split_segments(by_airport[ident]):
            kind = w.tags["aeroway"]
            width = parse_width_m(w.tags.get("width")) or default_width_m(kind, box.type)
            res.taxiways.append({
                "airport_ident": ident, "osm_way_id": w.id, "kind": kind,
                "ref": (w.tags.get("ref") or "").strip(), "width_m": f"{width:.1f}",
                "surface": surface_of(w.tags, box.surface),
                "points": fmt_points([osm.nodes[n] for n in refs])})
    # Stable sort keeps the segment order within a way.
    res.taxiways.sort(key=lambda r: (r["airport_ident"], r["osm_way_id"]))
    aprons.sort(key=lambda r: (r["airport_ident"], r["osm_way_id"]))
    res.aprons = aprons
    return res


# --------------------------------------------------------------------------------------------
# output
# --------------------------------------------------------------------------------------------
def csv_text(cols: list[str], rows: list[dict]) -> str:
    buf = io.StringIO()
    w = csv.DictWriter(buf, fieldnames=cols, lineterminator="\n", extrasaction="raise")
    w.writeheader()
    for r in rows:
        w.writerow(r)
    return buf.getvalue()


def odbl_text(date: str) -> str:
    return (
        "taxiways.csv and aprons.csv in this folder contain data from OpenStreetMap,\n"
        "© OpenStreetMap contributors, extracted on " + date + ".\n"
        "\n"
        "They are a derivative database of OpenStreetMap and are made available under the Open\n"
        "Database License (ODbL) 1.0: https://opendatacommons.org/licenses/odbl/1-0/\n"
        "See https://www.openstreetmap.org/copyright for the OpenStreetMap terms.\n"
        "\n"
        "Only these two files are under the ODbL. The rest of this data pack keeps the terms in\n"
        "ATTRIBUTION.txt, and the game itself is Apache-2.0.\n")


def set_nav_ini_source(nav_ini: Path, date: str) -> None:
    """Sets `osm_taxiways = <date>` under [sources] in an existing nav.ini."""
    if not nav_ini.is_file():
        return
    text = nav_ini.read_text(encoding="utf-8")
    if re.search(r"^osm_taxiways\s*=.*$", text, re.M):
        text = re.sub(r"^osm_taxiways\s*=.*$", f"osm_taxiways = {date}", text, flags=re.M)
    elif re.search(r"^\[sources\]\s*$", text, re.M):
        text = re.sub(r"^(\[sources\]\s*)$", rf"\1\nosm_taxiways = {date}", text, count=1,
                      flags=re.M)
    else:
        text = text.rstrip("\n") + f"\n[sources]\nosm_taxiways = {date}\n"
    nav_ini.write_text(text, encoding="utf-8")


def append_attribution(path: Path) -> bool:
    """Appends the ODbL line to ATTRIBUTION.txt unless it mentions OpenStreetMap already."""
    text = path.read_text(encoding="utf-8") if path.is_file() else ""
    if "OpenStreetMap" in text:
        return False
    with open(path, "a", encoding="utf-8") as f:
        if text and not text.endswith("\n"):
            f.write("\n")
        f.write("\n" + ATTRIBUTION_LINE + "\n")
    return True


def kib(n: int) -> str:
    return f"{n / 1024:.0f} KB"


def summary_line(source: str, res: Result, tax_bytes: int, apron_bytes: int) -> str:
    per: dict[str, list[int]] = {}
    for r in res.taxiways:
        e = per.setdefault(r["airport_ident"], [0, 0])
        e[0] += 1
        e[1] += len(csv_text(TAXIWAY_COLS, [r])) - len(",".join(TAXIWAY_COLS)) - 1
    for r in res.aprons:
        e = per.setdefault(r["airport_ident"], [0, 0])
        e[1] += len(csv_text(APRON_COLS, [r])) - len(",".join(APRON_COLS)) - 1
    line = (f"osm: {source}; {len(res.taxiways)} taxiway rows, {len(res.aprons)} aprons, "
            f"{len(per)} airports; taxiways.csv {kib(tax_bytes)}, aprons.csv {kib(apron_bytes)}")
    if per:
        big = max(per, key=lambda k: (per[k][1], k))
        line += f"; largest {big} {per[big][0]} rows {kib(per[big][1])}"
        for ident in ("KPDX", "KOTH"):
            if ident in per and ident != big:
                line += f"; {ident} {per[ident][0]} rows {kib(per[ident][1])}"
    return line


# --------------------------------------------------------------------------------------------
# main
# --------------------------------------------------------------------------------------------
def run(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0],
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--nav-dir", required=True, type=Path,
                    help="nav folder with airports.csv and runways.csv; outputs go there")
    ap.add_argument("--airports", default="", help="only these idents, e.g. KPDX,KOTH")
    ap.add_argument("--bbox", default="",
                    help="west,south,east,north: only airports in the pack's region (write "
                         "--bbox=-124.7,43.0,-121.4,46.3 so the minus sign is not an option)")
    ap.add_argument("--osm-file", action="append", type=Path, default=[],
                    help="parse this saved OSM XML instead of querying Overpass (repeatable)")
    ap.add_argument("--cache-dir", type=Path, default=DEFAULT_CACHE)
    ap.add_argument("--offline", action="store_true", help="use cached answers only")
    ap.add_argument("--overpass-url", default=DEFAULT_OVERPASS)
    ap.add_argument("--sleep", type=float, default=1.0, help="seconds between Overpass queries")
    ap.add_argument("--margin-m", type=float, default=1500.0, help="airport box margin")
    ap.add_argument("--attribution", type=Path, help="ATTRIBUTION.txt to append the ODbL line to")
    ap.add_argument("--date", default="", help=argparse.SUPPRESS)  # tests: fixed date
    args = ap.parse_args(argv)

    nav_dir: Path = args.nav_dir
    subset = {s.strip().upper() for s in args.airports.split(",") if s.strip()} or None
    try:
        region = parse_bbox(args.bbox) if args.bbox else None
    except ValueError as e:
        print(f"osm: {e}", file=sys.stderr)
        return 2
    if region is None and subset is None and not args.osm_file:
        # airports.csv is worldwide: never send thousands of queries by accident.
        print("osm: give --bbox (the pack's region) or --airports; refusing to query every airport "
              "in airports.csv", file=sys.stderr)
        return 2
    boxes = read_airports(nav_dir, subset, args.margin_m, region)
    today = args.date or dt.datetime.now(dt.timezone.utc).strftime("%Y-%m-%d")
    osm = OsmData()
    t0 = time.monotonic()
    if args.osm_file:
        for p in args.osm_file:
            osm.add_xml(p.read_bytes())
        if osm.nodes and region is None and subset is None:
            # Only the airports the files can touch (assignment is per way and box).
            lats = [ll[0] for ll in osm.nodes.values()]
            lons = [ll[1] for ll in osm.nodes.values()]
            s_, n_, w_, e_ = min(lats), max(lats), min(lons), max(lons)
            boxes = [b for b in boxes
                     if b.south <= n_ and b.north >= s_ and b.west <= e_ and b.east >= w_]
        source = f"{len(args.osm_file)} OSM files read"
        answered, tried = len(args.osm_file), 0
    else:
        state: dict = {}
        answered = tried = 0
        stamp = today.replace("-", "")
        for i, box in enumerate(boxes):
            tried += 1
            data = fetch_airport(box, args.cache_dir, args.overpass_url, args.offline, stamp,
                                 args.sleep, state)
            if data is not None:
                try:
                    osm.add_xml(data)
                    answered += 1
                except ET.ParseError as e:
                    print(f"osm: {box.ident}: bad XML ({e}); skipped", flush=True)
            if answered == 0 and tried >= FAIL_FAST_AIRPORTS and tried < len(boxes):
                print(f"osm: the first {tried} airports all failed; stopping (Overpass down?)",
                      flush=True)
                break
            if (i + 1) % 20 == 0:
                print(f"osm: {i + 1}/{len(boxes)} airports, {answered} answered "
                      f"({time.monotonic() - t0:.0f} s)", flush=True)
        source = f"{answered}/{tried} airports answered, {tried - answered} failed"
        if region:
            source += f" (region {args.bbox})"

    res = extract(osm, boxes)
    tax = csv_text(TAXIWAY_COLS, res.taxiways)
    apr = csv_text(APRON_COLS, res.aprons)
    (nav_dir / "taxiways.csv").write_text(tax, encoding="utf-8")
    (nav_dir / "aprons.csv").write_text(apr, encoding="utf-8")
    (nav_dir / "TAXIWAYS-ODbL.txt").write_text(odbl_text(today), encoding="utf-8")
    set_nav_ini_source(nav_dir / "nav.ini", today)
    if args.attribution:
        append_attribution(args.attribution)
    if osm.relations:
        print(f"osm: {osm.relations} apron relations skipped (multipolygon aprons are not read)")
    print(summary_line(source, res, len(tax.encode()), len(apr.encode())), flush=True)
    if tried > 0 and answered == 0:
        print("osm: no airport answered; is the Overpass API reachable?", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(run())
