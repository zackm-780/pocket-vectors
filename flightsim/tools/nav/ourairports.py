"""OurAirports CSV -> airports.csv / runways.csv / navaids.csv rows (DATA.md sections 5.2-5.4).

Pure functions on lists of dict rows (as produced by csv.DictReader) so they are easy to test.
"""
from __future__ import annotations

import math
import re
from collections import Counter
from typing import Callable, Iterable

from navgeo import FT_PER_M, M_PER_FT, destination, distance_m, initial_bearing_true_deg, norm360

# --------------------------------------------------------------------------------------------
# small helpers
# --------------------------------------------------------------------------------------------


def fnum(s) -> float | None:
    """Parse a float, returning None for empty or unparsable input."""
    if s is None:
        return None
    s = str(s).strip()
    if not s:
        return None
    try:
        v = float(s)
    except ValueError:
        return None
    return v if math.isfinite(v) else None


def fmt(v, nd: int = 6) -> str:
    """Format a number with at most nd decimals (trailing zeros stripped). None -> ''."""
    if v is None:
        return ""
    if isinstance(v, bool):
        return "1" if v else "0"
    if isinstance(v, int):
        return str(v)
    s = f"{v:.{nd}f}".rstrip("0").rstrip(".")
    return "0" if s in ("-0", "") else s


def valid_latlon(lat: float | None, lon: float | None) -> bool:
    if lat is None or lon is None:
        return False
    if not (-90.0 <= lat <= 90.0 and -180.0 <= lon <= 180.0):
        return False
    return not (abs(lat) < 1e-9 and abs(lon) < 1e-9)  # (0,0) is a placeholder, not a place


_RWY_RE = re.compile(r"^(\d{1,2})([LRCWGlrcwg]?)$")


def norm_rwy_ident(s: str) -> str:
    """'1l' -> '01L', '28R' -> '28R'. Non-numeric idents (helipads, 'N', 'NE') are kept upper-case."""
    s = (s or "").strip().upper().replace(" ", "")
    m = _RWY_RE.match(s)
    if m:
        return f"{int(m.group(1)):02d}{m.group(2)}"
    return s


def rwy_ident_heading_mag_deg(ident: str) -> float | None:
    """Nominal magnetic heading from a numeric runway ident ('28R' -> 280)."""
    m = _RWY_RE.match(ident)
    if not m:
        return None
    n = int(m.group(1))
    if not 1 <= n <= 36:
        return None
    return n * 10.0


# --------------------------------------------------------------------------------------------
# surface normalisation
# --------------------------------------------------------------------------------------------
_SURF_TOKENS = {
    "asphalt": {"ASP", "ASPH", "ASPHALT", "ASFALT", "BIT", "BITUMEN", "BITUMINOUS", "TAR",
                "TARMAC", "TARSEAL", "MAC", "MACADAM", "PEM", "PAVED", "PAV", "HARD", "ASPHALTIC"},
    "concrete": {"CON", "CONC", "CONCRETE", "CEMENT"},
    "grass": {"GRS", "GRE", "GRAS", "GRASS", "GRASSED", "TURF", "SOD", "GRAMA"},
    "dirt": {"DIRT", "EARTH", "GROUND", "SOIL", "CLAY", "CLA", "SAND", "SAN", "UNPAVED", "CALICHE",
             "GRADED", "LATERITE", "LAT", "LOAM", "MUD"},
    "gravel": {"GVL", "GRVL", "GRAVEL", "GRV", "CRUSHED", "ROCK", "CORAL", "COR", "CINDERS", "SLAG",
               "TRTD", "TREATED", "SHELLS", "STONE"},
    "water": {"WATER", "WAT", "WATE"},
}
_TOKEN_TO_SURF = {t: s for s, ts in _SURF_TOKENS.items() for t in ts}
_SPLIT_RE = re.compile(r"[^A-ZÀ-Ý]+")


def norm_surface(raw: str) -> str:
    """Map the free-text OurAirports surface to asphalt/concrete/grass/dirt/gravel/water/other.

    Mixed surfaces ('ASPH-TURF', 'GRVL/DIRT') take the FIRST recognised component; one-letter
    condition suffixes ('-G', '-F', '-P', '-E') and unknown one-letter codes are ignored.
    """
    toks = [t for t in _SPLIT_RE.split((raw or "").upper()) if len(t) > 1]
    for t in toks:
        s = _TOKEN_TO_SURF.get(t)
        if s:
            return s
    return "other"


# --------------------------------------------------------------------------------------------
# airports
# --------------------------------------------------------------------------------------------
_AIRPORT_TYPE = {"large_airport": "large", "medium_airport": "medium", "small_airport": "small",
                 "seaplane_base": "seaplane"}


def build_airports(rows: Iterable[dict]) -> list[dict]:
    """Excludes closed airports, heliports, balloonports (anything not in _AIRPORT_TYPE)."""
    out = []
    for r in rows:
        t = _AIRPORT_TYPE.get(r.get("type", ""))
        if t is None:
            continue
        ident = (r.get("ident") or "").strip()
        lat, lon = fnum(r.get("latitude_deg")), fnum(r.get("longitude_deg"))
        if not ident or not valid_latlon(lat, lon):
            continue
        elev = fnum(r.get("elevation_ft"))
        out.append({
            "ident": ident, "type": t, "name": (r.get("name") or "").strip(),
            "lat_deg": fmt(lat, 6), "lon_deg": fmt(lon, 6),
            "elev_ft": fmt(round(elev)) if elev is not None else "",
            "iso_country": (r.get("iso_country") or "").strip(),
            "iso_region": (r.get("iso_region") or "").strip(),
            "municipality": (r.get("municipality") or "").strip(),
            "iata": (r.get("iata_code") or "").strip(),
            "has_ils": "",
        })
    out.sort(key=lambda a: a["ident"])
    return out


# --------------------------------------------------------------------------------------------
# runways
# --------------------------------------------------------------------------------------------
MIN_RUNWAY_LEN_M = 30.0        # geometric length below this means the two ends are bogus
MAX_END_FROM_AIRPORT_M = 40_000.0  # an end further than this from the airport ref is bogus


def _end(r: dict, p: str) -> dict:
    lat, lon = fnum(r.get(f"{p}_latitude_deg")), fnum(r.get(f"{p}_longitude_deg"))
    return {"ident": norm_rwy_ident(r.get(f"{p}_ident", "")), "lat": lat, "lon": lon,
            "ok": valid_latlon(lat, lon),
            "elev": fnum(r.get(f"{p}_elevation_ft")),
            "hdg": fnum(r.get(f"{p}_heading_degT")),
            "disp": fnum(r.get(f"{p}_displaced_threshold_ft"))}


def _direction_row(apIdent: str, me: dict, other: dict, start: tuple[float, float],
                   end: tuple[float, float], lengthFt: float, widthFt: float | None,
                   surface: str, lighted: int, approx: int, apElevFt: float | None) -> dict:
    # round first so the published heading is exactly the bearing of the published coordinates
    start = (round(start[0], 6), round(start[1], 6))
    end = (round(end[0], 6), round(end[1], 6))
    hdg = initial_bearing_true_deg(start[0], start[1], end[0], end[1])
    dispFt = me["disp"] if me["disp"] and me["disp"] > 0 else 0.0
    if dispFt >= lengthFt * 0.9:  # nonsense displacement, ignore
        dispFt = 0.0
    thr = destination(start[0], start[1], hdg, dispFt * M_PER_FT) if dispFt else start
    elevMe = me["elev"] if me["elev"] is not None else (other["elev"] if other["elev"] is not None else apElevFt)
    elevOther = other["elev"] if other["elev"] is not None else (me["elev"] if me["elev"] is not None else apElevFt)
    return {
        "airport_ident": apIdent, "rwy_ident": me["ident"], "opposite_ident": other["ident"],
        "thr_lat_deg": fmt(thr[0], 6), "thr_lon_deg": fmt(thr[1], 6),
        "thr_elev_ft": fmt(round(elevMe)) if elevMe is not None else "",
        "start_lat_deg": fmt(start[0], 6), "start_lon_deg": fmt(start[1], 6),
        "end_lat_deg": fmt(end[0], 6), "end_lon_deg": fmt(end[1], 6),
        "end_elev_ft": fmt(round(elevOther)) if elevOther is not None else "",
        "heading_true_deg": fmt(hdg, 2), "length_ft": fmt(round(lengthFt)),
        "width_ft": fmt(round(widthFt)) if widthFt else "",
        "displaced_ft": fmt(round(dispFt)), "surface": surface,
        "hard": 1 if surface in ("asphalt", "concrete") else 0,
        "lighted": lighted, "approx": approx,
    }


def build_runways(rows: Iterable[dict], airports: dict[str, dict],
                  declEastDegFn: Callable[[float, float], float] | None = None,
                  useIdentHeading: bool = True) -> tuple[list[dict], Counter]:
    """One output row per landing direction.

    airports: ident -> airport output row (only included airports get runways).
    declEastDegFn(lat, lon) -> magnetic declination (east +), used only to turn a runway
    ident (magnetic) into a true heading when NO coordinates and NO published heading exist.
    Returns (rows, stats).
    """
    stats: Counter = Counter()
    out: list[dict] = []
    cands: list[tuple] = []
    for r in rows:
        stats["runways_in"] += 1
        apIdent = (r.get("airport_ident") or "").strip()
        ap = airports.get(apIdent)
        if ap is None:
            stats["skip_airport_excluded"] += 1
            continue
        if (r.get("closed") or "").strip() == "1":
            stats["skip_closed"] += 1
            continue
        le, he = _end(r, "le"), _end(r, "he")
        if not le["ident"] or not he["ident"] or le["ident"] == he["ident"]:
            stats["skip_one_ended"] += 1  # helipads and single-ended strips
            continue
        apLat, apLon = float(ap["lat_deg"]), float(ap["lon_deg"])
        apElev = fnum(ap["elev_ft"])
        for e in (le, he):  # drop ends that are far from the airport (bad data)
            if e["ok"] and distance_m(apLat, apLon, e["lat"], e["lon"]) > MAX_END_FROM_AIRPORT_M:
                e["ok"] = False
                stats["end_too_far_from_airport"] += 1
        lengthFt = fnum(r.get("length_ft"))
        widthFt = fnum(r.get("width_ft"))
        surface = norm_surface(r.get("surface", ""))
        lighted = 1 if (r.get("lighted") or "").strip() == "1" else 0

        approx = 0
        start = end = None  # start = LE end position, end = HE end position
        if le["ok"] and he["ok"]:
            gm = distance_m(le["lat"], le["lon"], he["lat"], he["lon"])
            if gm >= MIN_RUNWAY_LEN_M:
                start, end = (le["lat"], le["lon"]), (he["lat"], he["lon"])
                # Some OurAirports rows have the LE/HE coordinates swapped relative to the
                # idents (bearing ~180 deg off the ident's magnetic heading): trust the ident.
                if declEastDegFn is not None:
                    mag = rwy_ident_heading_mag_deg(le["ident"])
                    if mag is not None:
                        want = norm360(mag + declEastDegFn(apLat, apLon))
                        got = initial_bearing_true_deg(start[0], start[1], end[0], end[1])
                        if abs((got - want + 180.0) % 360.0 - 180.0) > 150.0:
                            start, end = end, start
                            stats["swapped_ends"] += 1
                if lengthFt is None or lengthFt <= 0:
                    lengthFt = gm * FT_PER_M
            else:
                stats["degenerate_coords"] += 1
                le["ok"] = he["ok"] = False
        if start is None:
            # synthesize: need length and a heading (le -> he direction)
            if lengthFt is None or lengthFt <= 0:
                stats["skip_no_length"] += 1
                continue
            hdg = None
            if le["hdg"] is not None:
                hdg = le["hdg"]
            elif he["hdg"] is not None:
                hdg = norm360(he["hdg"] + 180.0)
            if hdg is None and useIdentHeading and declEastDegFn is not None:
                mag = rwy_ident_heading_mag_deg(le["ident"])
                if mag is None:
                    m2 = rwy_ident_heading_mag_deg(he["ident"])
                    mag = None if m2 is None else norm360(m2 + 180.0)
                if mag is not None:
                    hdg = norm360(mag + declEastDegFn(apLat, apLon))
                    stats["heading_from_ident"] += 1
            if hdg is None:
                stats["skip_no_heading"] += 1
                continue
            lenM = lengthFt * M_PER_FT
            if le["ok"]:
                start = (le["lat"], le["lon"])
                end = destination(start[0], start[1], hdg, lenM)
            elif he["ok"]:
                end = (he["lat"], he["lon"])
                start = destination(end[0], end[1], hdg + 180.0, lenM)
            else:
                start = destination(apLat, apLon, hdg + 180.0, lenM / 2.0)
                end = destination(apLat, apLon, hdg, lenM / 2.0)
            approx = 1
            stats["synthesized"] += 1
        else:
            stats["exact"] += 1

        cands.append((apIdent, 0 if surface in ("asphalt", "concrete") else 1, -lengthFt, len(cands), (
            _direction_row(apIdent, le, he, start, end, lengthFt, widthFt, surface, lighted, approx, apElev),
            _direction_row(apIdent, he, le, end, start, lengthFt, widthFt, surface, lighted, approx, apElev))))
    # (airport, rwy_ident) must be unique: where two runways reuse an ident, keep the hard/longer one
    used: set[tuple[str, str]] = set()
    for apIdent, _h, _l, _i, pair in sorted(cands, key=lambda c: c[:4]):
        keys = [(apIdent, pair[0]["rwy_ident"]), (apIdent, pair[1]["rwy_ident"])]
        if keys[0] in used or keys[1] in used:
            stats["skip_duplicate_ident"] += 1
            continue
        used.update(keys)
        out.extend(pair)
    out.sort(key=lambda x: (x["airport_ident"], x["rwy_ident"]))
    stats["rows_out"] = len(out)
    return out, stats


# --------------------------------------------------------------------------------------------
# navaids
# --------------------------------------------------------------------------------------------
_NAVAID_TYPE = {"VOR": "VOR", "VOR-DME": "VORDME", "VORTAC": "VORTAC", "TACAN": "TACAN",
                "NDB": "NDB", "NDB-DME": "NDBDME", "DME": "DME"}
# Nominal service range (NM) from OurAirports 'power' (HIGH/MEDIUM/LOW) and navaid class.
_RANGE_VOR = {"HIGH": 130, "MEDIUM": 40, "LOW": 25}
_RANGE_NDB = {"HIGH": 75, "MEDIUM": 50, "LOW": 25}


def navaid_range_nm(t: str, power: str) -> int:
    p = (power or "").strip().upper()
    if t in ("NDB", "NDBDME"):
        return _RANGE_NDB.get(p, 25)
    return _RANGE_VOR.get(p, 40)


def build_navaids(rows: Iterable[dict], declEastDegFn: Callable[[float, float], float] | None = None
                  ) -> list[dict]:
    out = []
    for r in rows:
        t = _NAVAID_TYPE.get((r.get("type") or "").strip())
        if t is None:
            continue
        ident = (r.get("ident") or "").strip()
        lat, lon = fnum(r.get("latitude_deg")), fnum(r.get("longitude_deg"))
        if not ident or not valid_latlon(lat, lon):
            continue
        freq = fnum(r.get("frequency_khz"))
        if not freq or freq <= 0:
            freq = fnum(r.get("dme_frequency_khz"))
        if freq is not None and freq <= 0:
            freq = None
        elev = fnum(r.get("elevation_ft"))
        mv = fnum(r.get("magnetic_variation_deg"))
        if mv is None and declEastDegFn is not None:
            mv = declEastDegFn(lat, lon)
        dlat, dlon = fnum(r.get("dme_latitude_deg")), fnum(r.get("dme_longitude_deg"))
        dmeOk = valid_latlon(dlat, dlon)
        out.append({
            "ident": ident, "type": t, "name": (r.get("name") or "").strip(),
            "freq_khz": fmt(round(freq)) if freq else "",
            "lat_deg": fmt(lat, 6), "lon_deg": fmt(lon, 6),
            "elev_ft": fmt(round(elev)) if elev is not None else "",
            "magvar_deg": fmt(mv, 2),
            "range_nm": navaid_range_nm(t, r.get("power", "")),
            "dme_lat_deg": fmt(dlat, 6) if dmeOk else "",
            "dme_lon_deg": fmt(dlon, 6) if dmeOk else "",
            "iso_country": (r.get("iso_country") or "").strip(),
        })
    out.sort(key=lambda n: (n["ident"], n["type"], n["lat_deg"], n["lon_deg"]))
    return out
