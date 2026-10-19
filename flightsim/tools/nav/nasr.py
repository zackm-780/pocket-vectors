"""FAA NASR 28-day subscription (CSV flavour) -> ils.csv, fixes.csv, airways.csv.

Input files used (all inside the per-subject zips ``<DD>_<Mon>_<YYYY>_<ILS|FIX|AWY>_CSV.zip``):
    ILS_BASE.csv, ILS_GS.csv, ILS_DME.csv      -> ils.csv
    FIX_BASE.csv                               -> fixes.csv
    AWY_SEG_ALT.csv  (+ FIX_BASE.csv, navaids) -> airways.csv
See README.md for the verified download URLs and column meanings.
"""
from __future__ import annotations

import csv
import io
import re
import zipfile
from collections import Counter, defaultdict
from pathlib import Path
from typing import Callable, Iterable

from navgeo import distance_m, norm360
from ourairports import fmt, fnum, norm_rwy_ident, valid_latlon

WANTED = ("ILS_BASE.csv", "ILS_GS.csv", "ILS_DME.csv", "FIX_BASE.csv", "AWY_SEG_ALT.csv")


# --------------------------------------------------------------------------------------------
# locating files inside a zip / nested zips / directory
# --------------------------------------------------------------------------------------------
class NasrSource:
    """Finds NASR CSV members by base name inside a directory, a zip, or zips nested in zips."""

    def __init__(self, path: Path | str):
        self.path = Path(path)
        self._index: dict[str, Callable[[], bytes]] = {}
        if self.path.is_dir():
            for p in sorted(self.path.rglob("*")):
                if p.is_file():
                    self._add_file(p)
        else:
            self._add_zip(zipfile.ZipFile(self.path), 0)

    def _add_file(self, p: Path) -> None:
        if p.suffix.lower() == ".zip":
            self._add_zip(zipfile.ZipFile(p), 1)
        else:
            self._index.setdefault(p.name.lower(), p.read_bytes)

    def _add_zip(self, z: zipfile.ZipFile, depth: int) -> None:
        for info in z.infolist():
            if info.is_dir():
                continue
            name = info.filename.rsplit("/", 1)[-1].lower()
            if name.endswith(".zip") and depth < 3:
                self._add_zip(zipfile.ZipFile(io.BytesIO(z.read(info))), depth + 1)
            else:
                self._index.setdefault(name, (lambda zz=z, ii=info: zz.read(ii)))

    def has(self, name: str) -> bool:
        return name.lower() in self._index

    def rows(self, name: str) -> list[dict]:
        if not self.has(name):
            raise FileNotFoundError(f"{name} not found in {self.path}")
        data = self._index[name.lower()]().decode("utf-8-sig")
        return list(csv.DictReader(io.StringIO(data)))

    def effective_date(self) -> str:
        """EFF_DATE of the first ILS_BASE/FIX_BASE row as YYYY-MM-DD ('' if unknown)."""
        for n in ("ILS_BASE.csv", "FIX_BASE.csv", "AWY_SEG_ALT.csv"):
            if self.has(n):
                rows = self.rows(n)
                if rows:
                    return rows[0].get("EFF_DATE", "").replace("/", "-")
        return ""


# --------------------------------------------------------------------------------------------
# airport id mapping (FAA location id -> OurAirports ident)
# --------------------------------------------------------------------------------------------
def build_airport_id_map(rawAirports: Iterable[dict], includedIdents: set[str]) -> dict[str, str]:
    """FAA ARPT_ID ('SFO', 'ANC', '00A') -> OurAirports ident ('KSFO', 'PANC', '00A').

    Uses local_code / gps_code of US-style airports; falls back at lookup time to 'K'+id.
    """
    m: dict[str, str] = {}
    for r in rawAirports:
        ident = (r.get("ident") or "").strip()
        if ident not in includedIdents:
            continue
        if (r.get("iso_country") or "") not in ("US", "PR", "VI", "GU", "MP", "AS", "UM"):
            continue
        for key in ((r.get("local_code") or "").strip(), (r.get("gps_code") or "").strip()):
            if key and key not in m:
                m[key] = ident
    return m


def map_airport(arptId: str, idMap: dict[str, str], idents: set[str]) -> str | None:
    a = arptId.strip().upper()
    if a in idMap:
        return idMap[a]
    if "K" + a in idents:
        return "K" + a
    if a in idents:
        return a
    return None


# --------------------------------------------------------------------------------------------
# ils.csv
# --------------------------------------------------------------------------------------------
_OK_STATUS = ("OPERATIONAL IFR", "OPERATIONAL RESTRICTED")


def _signed_magvar(r: dict) -> float:
    v = fnum(r.get("MAG_VAR")) or 0.0
    return -v if (r.get("MAG_VAR_HEMIS") or "").strip().upper() == "W" else v


def build_ils(base: Iterable[dict], gs: Iterable[dict], dme: Iterable[dict],
              idMap: dict[str, str], idents: set[str]) -> tuple[list[dict], Counter]:
    """loc_course_true_deg = APCH_BEAR (magnetic, as published) + MAG_VAR (E+). loc_width_deg and
    tch_ft are not published in the ILS files and are left empty (runtime defaults apply)."""
    stats: Counter = Counter()

    def key(r):
        return (r["ARPT_ID"].strip(), norm_rwy_ident(r["RWY_END_ID"]), r["ILS_LOC_ID"].strip())

    gsMap, dmeMap = {}, {}
    for r in gs:
        if r.get("COMPONENT_STATUS") in _OK_STATUS:
            gsMap[key(r)] = r
    for r in dme:
        if r.get("COMPONENT_STATUS") in _OK_STATUS:
            dmeMap[key(r)] = r
    out = []
    for r in base:
        stats["base_rows"] += 1
        if r.get("COMPONENT_STATUS") not in _OK_STATUS:
            stats["skip_not_operational"] += 1
            continue
        ap = map_airport(r["ARPT_ID"], idMap, idents)
        if ap is None:
            stats["skip_unknown_airport"] += 1
            continue
        lat, lon = fnum(r.get("LAT_DECIMAL")), fnum(r.get("LONG_DECIMAL"))
        bear, freq = fnum(r.get("APCH_BEAR")), fnum(r.get("LOC_FREQ"))
        if not valid_latlon(lat, lon) or bear is None or freq is None:
            stats["skip_incomplete"] += 1
            continue
        k = key(r)
        g, d = gsMap.get(k), dmeMap.get(k)
        glat = fnum(g.get("LAT_DECIMAL")) if g else None
        glon = fnum(g.get("LONG_DECIMAL")) if g else None
        hasGs = bool(g) and valid_latlon(glat, glon)
        gsElev = fnum(g.get("SITE_ELEVATION")) if g else None
        dlat = fnum(d.get("LAT_DECIMAL")) if d else None
        dlon = fnum(d.get("LONG_DECIMAL")) if d else None
        dOk = valid_latlon(dlat, dlon)
        out.append({
            "airport_ident": ap, "rwy_ident": k[1], "loc_ident": k[2], "freq_mhz": fmt(freq, 3),
            "loc_lat_deg": fmt(lat, 6), "loc_lon_deg": fmt(lon, 6),
            "loc_course_true_deg": fmt(norm360(bear + _signed_magvar(r)), 2),
            "loc_width_deg": "",
            "has_gs": 1 if hasGs else 0,
            "gs_lat_deg": fmt(glat, 6) if hasGs else "", "gs_lon_deg": fmt(glon, 6) if hasGs else "",
            "gs_elev_ft": fmt(gsElev, 1) if hasGs and gsElev is not None else "",
            "gs_angle_deg": fmt(fnum(g.get("G_S_ANGLE")), 2) if hasGs else "",
            "tch_ft": "",
            "dme_lat_deg": fmt(dlat, 6) if dOk else "", "dme_lon_deg": fmt(dlon, 6) if dOk else "",
            "category": (r.get("CATEGORY") or "").strip(),
        })
    out.sort(key=lambda x: (x["airport_ident"], x["rwy_ident"], x["loc_ident"]))
    stats["rows_out"] = len(out)
    return out, stats


# --------------------------------------------------------------------------------------------
# fixes.csv
# --------------------------------------------------------------------------------------------
def build_fixes(fixBase: Iterable[dict]) -> list[dict]:
    out = []
    seen = set()
    for r in fixBase:
        ident = (r.get("FIX_ID") or "").strip()
        lat, lon = fnum(r.get("LAT_DECIMAL")), fnum(r.get("LONG_DECIMAL"))
        if not ident or not valid_latlon(lat, lon):
            continue
        region = (r.get("ICAO_REGION_CODE") or "").strip()
        k = (ident, region, fmt(lat, 6), fmt(lon, 6))
        if k in seen:
            continue
        seen.add(k)
        out.append({"ident": ident, "region": region, "lat_deg": fmt(lat, 6), "lon_deg": fmt(lon, 6)})
    out.sort(key=lambda x: (x["ident"], x["region"], x["lat_deg"], x["lon_deg"]))
    return out


# --------------------------------------------------------------------------------------------
# airways.csv
# --------------------------------------------------------------------------------------------
def build_airways(segs: Iterable[dict], fixes: list[dict], navaids: list[dict]
                  ) -> tuple[list[dict], Counter]:
    """One row per airway point, in route order. A segment contributes its FROM point; the last
    segment of each contiguous chain also contributes its TO point. Coordinates come from
    fixes.csv rows (waypoints) or navaids.csv rows (VOR/NDB/DME points); ambiguity between
    same-named points is resolved by ICAO region code, then by proximity to the previous point.
    MEA is MIN_ENROUTE_ALT (direction of travel as published; blank when not published)."""
    stats: Counter = Counter()
    fixIdx: dict[str, list[tuple[str, float, float]]] = defaultdict(list)
    for f in fixes:
        fixIdx[f["ident"]].append((f["region"], float(f["lat_deg"]), float(f["lon_deg"])))
    navIdx: dict[str, list[tuple[str, float, float]]] = defaultdict(list)
    for n in navaids:
        navIdx[n["ident"]].append((n["iso_country"], float(n["lat_deg"]), float(n["lon_deg"])))

    def resolve(ident: str, region: str, ptType: str, prev):
        isNav = ptType.strip() not in ("WP", "RP", "CN", "")
        order = (navIdx, fixIdx) if isNav else (fixIdx, navIdx)
        for idx in order:
            c = idx.get(ident)
            if not c:
                continue
            if len(c) > 1 and idx is fixIdx:
                cr = [x for x in c if x[0] == region]
                if cr:
                    c = cr
            if len(c) > 1 and prev is not None:
                c = sorted(c, key=lambda x: distance_m(prev[0], prev[1], x[1], x[2]))
            return c[0][1], c[0][2]
        return None

    chains: dict[tuple[str, str], list[dict]] = defaultdict(list)
    for r in segs:
        chains[((r.get("AWY_LOCATION") or "").strip(), (r.get("AWY_ID") or "").strip())].append(r)
    out = []
    chainCount: dict[str, int] = {}
    for (loc, awy), rows in sorted(chains.items(), key=lambda kv: (kv[0][1], kv[0][0])):
        rows.sort(key=lambda r: fnum(r.get("POINT_SEQ")) or 0.0)
        # points: [ident, point type, region, mea]. A segment FROM->TO gives its FROM point the
        # segment's MEA. The TO point of one segment is the FROM point of the next, unless the
        # airway has a gap or dogleg, in which case both are emitted.
        points: list[list] = []
        prevTo = None
        for i, r in enumerate(rows):
            frm = (r.get("FROM_POINT") or "").strip()
            to = (r.get("TO_POINT") or "").strip()
            region = (r.get("ICAO_REGION_CODE") or "").strip()
            mea = fnum(r.get("MIN_ENROUTE_ALT"))
            if i == 0 or frm != prevTo:
                points.append([frm, r.get("FROM_PT_TYPE", ""), region, mea])
            else:
                points[-1][1], points[-1][3] = r.get("FROM_PT_TYPE", ""), mea
            points.append([to, "", region, None])
            prevTo = to
        # The same designator can exist in several regions (e.g. V1 in the contiguous US and in
        # Hawaii). Their chains are kept apart by starting each region at a multiple of 1000.
        k = chainCount.get(awy, 0)
        chainCount[awy] = k + 1
        seq = 1000 * k
        prev = None
        for ident, ptType, region, mea in points:
            if not ident:
                continue
            pos = resolve(ident, region, ptType, prev)
            if pos is None:
                stats["unresolved_points"] += 1
                continue
            seq += 1
            prev = pos
            out.append({"airway": awy, "seq": seq, "fix_ident": ident,
                        "fix_lat_deg": fmt(pos[0], 6), "fix_lon_deg": fmt(pos[1], 6),
                        "mea_ft": fmt(round(mea)) if mea else ""})
        stats["airways"] += 1
    stats["rows_out"] = len(out)
    return out, stats


# --------------------------------------------------------------------------------------------
# cycle helpers
# --------------------------------------------------------------------------------------------
_MON = ["Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"]
INDEX_URL = "https://www.faa.gov/air_traffic/flight_info/aeronav/aero_data/NASR_Subscription/"
CSV_URL = "https://nfdc.faa.gov/webContent/28DaySub/extra/{dd}_{mon}_{yyyy}_{subject}_CSV.zip"
SUBJECTS = ("ILS", "FIX", "AWY")


def csv_zip_url(effective: str, subject: str) -> str:
    y, m, d = effective.split("-")
    return CSV_URL.format(dd=d, mon=_MON[int(m) - 1], yyyy=y, subject=subject)


def parse_cycle_dates(indexHtml: str) -> list[str]:
    """Cycle effective dates (YYYY-MM-DD) linked from the FAA NASR subscription landing page."""
    return sorted(set(re.findall(r"NASR_Subscription/(\d{4}-\d{2}-\d{2})", indexHtml)))


def pick_current_cycle(dates: list[str], todayIso: str) -> str | None:
    """Latest cycle whose effective date is on or before today."""
    ok = [d for d in dates if d <= todayIso]
    return ok[-1] if ok else None
