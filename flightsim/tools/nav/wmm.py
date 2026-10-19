"""World Magnetic Model (WMM2025) in pure Python, plus the magvar.bin writer.

The model is the standard spherical-harmonic expansion (degree/order 12) described in the NOAA
WMM2025 technical report. Coefficients are read from the official NOAA ``WMM.COF`` file
(vendored as ``wmm/WMM2025.COF``). Declination is positive EAST.

Verified against the official NOAA WMM2025 test values (``wmm/WMM2025_TEST_VALUES.txt``) by
``tests/test_wmm.py`` and ``validate_navdb.py --wmm-selftest``.
"""
from __future__ import annotations

import math
import struct
from pathlib import Path

HERE = Path(__file__).resolve().parent
DEFAULT_COF = HERE / "wmm" / "WMM2025.COF"
DEFAULT_TEST_VALUES = HERE / "wmm" / "WMM2025_TEST_VALUES.txt"

# WGS84 and WMM reference constants (km).
_A_KM = 6378.137
_F = 1.0 / 298.257223563
_E2 = _F * (2.0 - _F)
_RE_KM = 6371.2  # geomagnetic reference radius

MAGVAR_MAGIC = b"MAGVAR01"


class WmmError(Exception):
    pass


def _parse_cof(path: Path):
    lines = path.read_text(encoding="ascii").splitlines()
    head = lines[0].split()
    epoch = float(head[0])
    model = head[1] if len(head) > 1 else "WMM"
    nmax = 12
    g = [[0.0] * (nmax + 2) for _ in range(nmax + 2)]
    h = [[0.0] * (nmax + 2) for _ in range(nmax + 2)]
    gd = [[0.0] * (nmax + 2) for _ in range(nmax + 2)]
    hd = [[0.0] * (nmax + 2) for _ in range(nmax + 2)]
    count = 0
    for ln in lines[1:]:
        t = ln.split()
        if not t or t[0].startswith("9999"):
            continue
        n, m = int(t[0]), int(t[1])
        g[n][m], h[n][m], gd[n][m], hd[n][m] = (float(v) for v in t[2:6])
        count += 1
    if count != 90:
        raise WmmError(f"{path}: expected 90 coefficient rows, found {count}")
    return epoch, model, nmax, g, h, gd, hd


class Wmm:
    """WMM spherical-harmonic model evaluated at a decimal-year time."""

    def __init__(self, cofPath: Path | str = DEFAULT_COF):
        (self.epochYear, self.modelName, self.nmax,
         self._g, self._h, self._gd, self._hd) = _parse_cof(Path(cofPath))

    @property
    def validFromYear(self) -> float:
        return self.epochYear

    @property
    def validToYear(self) -> float:
        return self.epochYear + 5.0

    def _coeffs(self, year: float):
        dt = year - self.epochYear
        n1 = self.nmax + 1
        g = [[self._g[n][m] + dt * self._gd[n][m] for m in range(n1)] for n in range(n1)]
        h = [[self._h[n][m] + dt * self._hd[n][m] for m in range(n1)] for n in range(n1)]
        return g, h

    @staticmethod
    def _legendre(x: float, s: float, nmax: int):
        """Schmidt semi-normalised P[n][m](x=sin(phi')) and dP/dphi' (no Condon-Shortley phase).
        Pole-safe: derivatives come from recursion, not from division by cos(phi')."""
        n1 = nmax + 2
        P = [[0.0] * n1 for _ in range(n1)]
        dP = [[0.0] * n1 for _ in range(n1)]
        P[0][0] = 1.0
        for n in range(1, nmax + 1):
            for m in range(0, n + 1):
                if m == n:
                    k = math.sqrt((2.0 * n - 1.0) / (2.0 * n)) if n >= 2 else 1.0
                    P[n][n] = k * s * P[n - 1][n - 1]
                    dP[n][n] = k * (s * dP[n - 1][n - 1] - x * P[n - 1][n - 1])
                else:
                    den = math.sqrt(float(n * n - m * m))
                    c2 = math.sqrt(float((n - 1) * (n - 1) - m * m)) if m <= n - 2 else 0.0
                    P[n][m] = ((2.0 * n - 1.0) * x * P[n - 1][m] - c2 * P[n - 2][m]) / den
                    dP[n][m] = ((2.0 * n - 1.0) * (s * P[n - 1][m] + x * dP[n - 1][m])
                                - c2 * dP[n - 2][m]) / den
        return P, dP

    def _lat_terms(self, latDeg: float, hKm: float, g, h):
        """Everything that depends on latitude/height only. Returns per-order-m coefficient
        lists so that, for a longitude, X' = sum_m (xc[m] cos(m lon) + xs[m] sin(m lon)), etc."""
        lat = math.radians(latDeg)
        sl, cl = math.sin(lat), math.cos(lat)
        rc = _A_KM / math.sqrt(1.0 - _E2 * sl * sl)
        p = (rc + hKm) * cl
        z = (rc * (1.0 - _E2) + hKm) * sl
        r = math.hypot(p, z)
        sphi = z / r  # sin(geocentric latitude)
        cphi = p / r
        dphi = math.asin(max(-1.0, min(1.0, sphi))) - lat  # phi' - phi
        P, dP = self._legendre(sphi, cphi, self.nmax)
        nm = self.nmax
        xc = [0.0] * (nm + 1)
        xs = [0.0] * (nm + 1)
        yc = [0.0] * (nm + 1)
        ys = [0.0] * (nm + 1)
        zc = [0.0] * (nm + 1)
        zs = [0.0] * (nm + 1)
        ar = _RE_KM / r
        arn = ar * ar  # (a/r)^(n+2) built up from n=0
        for n in range(1, nm + 1):
            arn *= ar
            for m in range(0, n + 1):
                gg, hh = g[n][m], h[n][m]
                pnm, dpnm = P[n][m], dP[n][m]
                # X' = -sum arn (g cos + h sin) dP
                xc[m] -= arn * gg * dpnm
                xs[m] -= arn * hh * dpnm
                # Y' = 1/cos(phi') sum arn m (g sin - h cos) P
                yc[m] -= arn * m * hh * pnm
                ys[m] += arn * m * gg * pnm
                # Z' = -sum (n+1) arn (g cos + h sin) P
                zc[m] -= (n + 1) * arn * gg * pnm
                zs[m] -= (n + 1) * arn * hh * pnm
        icp = 1.0 / cphi if abs(cphi) > 1e-9 else 0.0
        yc = [v * icp for v in yc]
        ys = [v * icp for v in ys]
        return (xc, xs, yc, ys, zc, zs), dphi

    @staticmethod
    def _assemble(terms, dphi: float, lonDeg: float):
        xc, xs, yc, ys, zc, zs = terms
        lam = math.radians(lonDeg)
        xp = yp = zp = 0.0
        for m in range(len(xc)):
            cm, sm = math.cos(m * lam), math.sin(m * lam)
            xp += xc[m] * cm + xs[m] * sm
            yp += yc[m] * cm + ys[m] * sm
            zp += zc[m] * cm + zs[m] * sm
        cd, sd = math.cos(dphi), math.sin(dphi)
        return xp * cd - zp * sd, yp, xp * sd + zp * cd

    def components(self, latDeg: float, lonDeg: float, hKm: float, year: float):
        """Return (X north, Y east, Z down) in nT at geodetic position and height above WGS84."""
        g, h = self._coeffs(year)
        terms, dphi = self._lat_terms(latDeg, hKm, g, h)
        return self._assemble(terms, dphi, lonDeg)

    def declination_deg(self, latDeg: float, lonDeg: float, year: float, hKm: float = 0.0) -> float:
        """Magnetic declination in degrees, positive EAST (mag = true - dec)."""
        x, y, _ = self.components(latDeg, lonDeg, hKm, year)
        return math.degrees(math.atan2(y, x))

    def declination_grid_centideg(self, year: float, hKm: float = 0.0,
                                  latMinDeg: int = -90, latMaxDeg: int = 90,
                                  lonMinDeg: int = -180, lonMaxDeg: int = 180,
                                  stepDeg: int = 1) -> list[int]:
        """Row-major (latMin..latMax, lonMin..lonMax) declination in centidegrees (east +).
        The poles are evaluated at +/-89.99 deg since declination is singular there."""
        g, h = self._coeffs(year)
        out: list[int] = []
        for lat in range(latMinDeg, latMaxDeg + 1, stepDeg):
            latEval = max(-89.99, min(89.99, float(lat)))
            terms, dphi = self._lat_terms(latEval, hKm, g, h)
            for lon in range(lonMinDeg, lonMaxDeg + 1, stepDeg):
                x, y, _ = self._assemble(terms, dphi, float(lon))
                out.append(int(round(math.degrees(math.atan2(y, x)) * 100.0)))
        return out


def decimal_year(dt) -> float:
    """Decimal year for a datetime (UTC)."""
    import datetime as _dt
    start = _dt.datetime(dt.year, 1, 1, tzinfo=dt.tzinfo)
    end = _dt.datetime(dt.year + 1, 1, 1, tzinfo=dt.tzinfo)
    return dt.year + (dt - start).total_seconds() / (end - start).total_seconds()


# ---------------------------------------------------------------------------------------------
# magvar.bin (DATA.md section 6)
# header: "MAGVAR01", float32 epochYear, int16 latMin, latMax, lonMin, lonMax, step, reserved
# data  : int16 centidegrees, east +, rows latMin..latMax, columns lonMin..lonMax
# ---------------------------------------------------------------------------------------------
_HEADER = struct.Struct("<8sf6h")


def write_magvar_bin(path: Path | str, wmm: Wmm, year: float, stepDeg: int = 1) -> None:
    vals = wmm.declination_grid_centideg(year, stepDeg=stepDeg)
    nLat = (90 - -90) // stepDeg + 1
    nLon = (180 - -180) // stepDeg + 1
    assert len(vals) == nLat * nLon
    with open(path, "wb") as f:
        f.write(_HEADER.pack(MAGVAR_MAGIC, year, -90, 90, -180, 180, stepDeg, 0))
        f.write(struct.pack(f"<{len(vals)}h", *vals))


def read_magvar_bin(path: Path | str):
    """Returns (epochYear, latMin, latMax, lonMin, lonMax, stepDeg, values list[int])."""
    raw = Path(path).read_bytes()
    magic, epoch, la0, la1, lo0, lo1, step, _res = _HEADER.unpack_from(raw, 0)
    if magic != MAGVAR_MAGIC:
        raise WmmError(f"{path}: bad magic {magic!r}")
    n = ((la1 - la0) // step + 1) * ((lo1 - lo0) // step + 1)
    if len(raw) != _HEADER.size + 2 * n:
        raise WmmError(f"{path}: size {len(raw)} != expected {_HEADER.size + 2 * n}")
    vals = list(struct.unpack_from(f"<{n}h", raw, _HEADER.size))
    return epoch, la0, la1, lo0, lo1, step, vals


def lookup_magvar_deg(grid, latDeg: float, lonDeg: float) -> float:
    """Bilinear lookup in a grid returned by read_magvar_bin (same method the runtime uses)."""
    _epoch, la0, la1, lo0, lo1, step, vals = grid
    nLon = (lo1 - lo0) // step + 1
    fy = (min(max(latDeg, la0), la1 - 1e-9) - la0) / step
    lon = (lonDeg - lo0) % (lo1 - lo0)  # wrap
    fx = lon / step
    y0, x0 = int(fy), int(fx)
    ty, tx = fy - y0, fx - x0

    def v(yy, xx):
        return vals[yy * nLon + xx] / 100.0

    return ((v(y0, x0) * (1 - tx) + v(y0, x0 + 1) * tx) * (1 - ty)
            + (v(y0 + 1, x0) * (1 - tx) + v(y0 + 1, x0 + 1) * tx) * ty)


def load_test_values(path: Path | str = DEFAULT_TEST_VALUES):
    """Parse the NOAA test-value file into dicts."""
    rows = []
    for ln in Path(path).read_text(encoding="ascii").splitlines():
        if not ln.strip() or ln.lstrip().startswith("#"):
            continue
        t = ln.split()
        f = [float("nan") if v.upper() == "NAN" else float(v) for v in t]
        rows.append({"year": f[0], "hKm": f[1], "latDeg": f[2], "lonDeg": f[3],
                     "xNt": f[4], "yNt": f[5], "zNt": f[6], "hNt": f[7], "fNt": f[8],
                     "inclDeg": f[9], "declDeg": f[10]})
    return rows


def selftest(wmm: Wmm | None = None, tolDeg: float = 0.05, tolNt: float = 0.3):
    """Compare against the official NOAA test values. Returns list of (row, errors dict)."""
    wmm = wmm or Wmm()
    results = []
    for r in load_test_values():
        x, y, z = wmm.components(r["latDeg"], r["lonDeg"], r["hKm"], r["year"])
        hh = math.hypot(x, y)
        decl = math.degrees(math.atan2(y, x))
        incl = math.degrees(math.atan2(z, hh))
        err = {"declDeg": decl - r["declDeg"], "inclDeg": incl - r["inclDeg"],
               "xNt": x - r["xNt"], "yNt": y - r["yNt"], "zNt": z - r["zNt"]}
        ok = (abs(err["declDeg"]) <= tolDeg and abs(err["inclDeg"]) <= tolDeg
              and max(abs(err["xNt"]), abs(err["yNt"]), abs(err["zNt"])) <= tolNt)
        results.append((r, err, ok))
    return results
