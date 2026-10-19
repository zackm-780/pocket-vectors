"""Small geodesy helpers shared by the nav pipeline (pure standard library).

Angles in function arguments are degrees unless the name says Rad. Bearings are TRUE.
Positions use a spherical earth (R = 6371008.8 m); over runway-scale distances the error
against the WGS84 ellipsoid is well below the precision of the source data.
"""
from __future__ import annotations

import math

FT_PER_M = 1.0 / 0.3048
M_PER_FT = 0.3048
M_PER_NM = 1852.0
EARTH_RADIUS_M = 6371008.8


def norm360(deg: float) -> float:
    """Wrap an angle to [0, 360)."""
    v = math.fmod(deg, 360.0)
    return v + 360.0 if v < 0.0 else v


def initial_bearing_true_deg(lat1Deg: float, lon1Deg: float, lat2Deg: float, lon2Deg: float) -> float:
    """Initial great-circle bearing (true) from point 1 to point 2, degrees in [0, 360)."""
    p1 = math.radians(lat1Deg)
    p2 = math.radians(lat2Deg)
    dl = math.radians(lon2Deg - lon1Deg)
    y = math.sin(dl) * math.cos(p2)
    x = math.cos(p1) * math.sin(p2) - math.sin(p1) * math.cos(p2) * math.cos(dl)
    return norm360(math.degrees(math.atan2(y, x)))


def distance_m(lat1Deg: float, lon1Deg: float, lat2Deg: float, lon2Deg: float) -> float:
    """Great-circle (haversine) distance in metres."""
    p1 = math.radians(lat1Deg)
    p2 = math.radians(lat2Deg)
    dp = p2 - p1
    dl = math.radians(lon2Deg - lon1Deg)
    a = math.sin(dp / 2) ** 2 + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2
    return 2.0 * EARTH_RADIUS_M * math.asin(min(1.0, math.sqrt(a)))


def destination(latDeg: float, lonDeg: float, bearingTrueDeg: float, distM: float) -> tuple[float, float]:
    """Point reached from (lat, lon) travelling distM along a great circle with the given
    initial true bearing. Returns (lat, lon) in degrees, lon wrapped to [-180, 180)."""
    p1 = math.radians(latDeg)
    l1 = math.radians(lonDeg)
    th = math.radians(bearingTrueDeg)
    d = distM / EARTH_RADIUS_M
    sp2 = math.sin(p1) * math.cos(d) + math.cos(p1) * math.sin(d) * math.cos(th)
    p2 = math.asin(max(-1.0, min(1.0, sp2)))
    l2 = l1 + math.atan2(math.sin(th) * math.sin(d) * math.cos(p1),
                         math.cos(d) - math.sin(p1) * sp2)
    lon2 = (math.degrees(l2) + 540.0) % 360.0 - 180.0
    return math.degrees(p2), lon2
