#include "core/Geo.h"

#include <cmath>

namespace fsim {

Ecef geodeticToEcef(const GeoPos& pos) {
    const double lat = degToRad(pos.latDeg), lon = degToRad(pos.lonDeg);
    const double sinLat = std::sin(lat), cosLat = std::cos(lat);
    const double n = kWgs84AM / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
    Ecef out;
    out.x = (n + pos.altMslM) * cosLat * std::cos(lon);
    out.y = (n + pos.altMslM) * cosLat * std::sin(lon);
    out.z = (n * (1.0 - kWgs84E2) + pos.altMslM) * sinLat;
    return out;
}

GeoPos ecefToGeodetic(const Ecef& e) {
    // Fixed-point iteration; converges to sub-millimeter in a few steps near the surface.
    const double p = std::hypot(e.x, e.y);
    GeoPos out;
    out.lonDeg = radToDeg(std::atan2(e.y, e.x));
    double lat = std::atan2(e.z, p * (1.0 - kWgs84E2));
    double alt = 0;
    for (int i = 0; i < 8; ++i) {
        const double sinLat = std::sin(lat);
        const double n = kWgs84AM / std::sqrt(1.0 - kWgs84E2 * sinLat * sinLat);
        alt = (std::fabs(lat) < 1.4) ? p / std::cos(lat) - n : e.z / sinLat - n * (1.0 - kWgs84E2);
        lat = std::atan2(e.z, p * (1.0 - kWgs84E2 * n / (n + alt)));
    }
    out.latDeg = radToDeg(lat);
    out.altMslM = alt;
    return out;
}

double greatCircleDistM(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    const double lat1 = degToRad(lat1Deg), lat2 = degToRad(lat2Deg);
    const double dLat = lat2 - lat1, dLon = degToRad(lon2Deg - lon1Deg);
    const double s1 = std::sin(dLat * 0.5), s2 = std::sin(dLon * 0.5);
    const double a = s1 * s1 + std::cos(lat1) * std::cos(lat2) * s2 * s2;
    return 2.0 * kEarthRadiusM * std::asin(std::fmin(1.0, std::sqrt(a)));
}

double initialBearingTrueRad(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    const double lat1 = degToRad(lat1Deg), lat2 = degToRad(lat2Deg);
    const double dLon = degToRad(lon2Deg - lon1Deg);
    const double y = std::sin(dLon) * std::cos(lat2);
    const double x =
        std::cos(lat1) * std::sin(lat2) - std::sin(lat1) * std::cos(lat2) * std::cos(dLon);
    return wrap2Pi(std::atan2(y, x));
}

void destinationPoint(double latDeg, double lonDeg, double bearingTrueRad, double distM,
                      double& outLatDeg, double& outLonDeg) {
    const double lat1 = degToRad(latDeg), lon1 = degToRad(lonDeg);
    const double ang = distM / kEarthRadiusM;
    const double sinLat2 =
        std::sin(lat1) * std::cos(ang) + std::cos(lat1) * std::sin(ang) * std::cos(bearingTrueRad);
    const double lat2 = std::asin(std::fmax(-1.0, std::fmin(1.0, sinLat2)));
    const double lon2 = lon1 + std::atan2(std::sin(bearingTrueRad) * std::sin(ang) * std::cos(lat1),
                                          std::cos(ang) - std::sin(lat1) * sinLat2);
    outLatDeg = radToDeg(lat2);
    outLonDeg = radToDeg(wrapPi(lon2));
}

double crossTrackM(const LatLon& p, const LatLon& a, const LatLon& b) {
    const double d13 = greatCircleDistM(a.latDeg, a.lonDeg, p.latDeg, p.lonDeg) / kEarthRadiusM;
    const double t13 = initialBearingTrueRad(a.latDeg, a.lonDeg, p.latDeg, p.lonDeg);
    const double t12 = initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg);
    return std::asin(std::sin(d13) * std::sin(t13 - t12)) * kEarthRadiusM;
}

double alongTrackM(const LatLon& p, const LatLon& a, const LatLon& b) {
    const double d13 = greatCircleDistM(a.latDeg, a.lonDeg, p.latDeg, p.lonDeg) / kEarthRadiusM;
    const double t13 = initialBearingTrueRad(a.latDeg, a.lonDeg, p.latDeg, p.lonDeg);
    const double t12 = initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg);
    const double dxt = std::asin(std::sin(d13) * std::sin(t13 - t12));
    const double cosDxt = std::cos(dxt);
    if (cosDxt == 0.0) return 0.0;
    const double dat = std::acos(std::fmax(-1.0, std::fmin(1.0, std::cos(d13) / cosDxt)));
    return (std::cos(t13 - t12) >= 0.0 ? dat : -dat) * kEarthRadiusM;
}

Mat3d nedToEcef(double latDeg, double lonDeg) {
    const double lat = degToRad(latDeg), lon = degToRad(lonDeg);
    const double sLat = std::sin(lat), cLat = std::cos(lat);
    const double sLon = std::sin(lon), cLon = std::cos(lon);
    Mat3d r;
    // Columns: north, east, down expressed in ECEF.
    r.m[0][0] = -sLat * cLon;
    r.m[0][1] = -sLon;
    r.m[0][2] = -cLat * cLon;
    r.m[1][0] = -sLat * sLon;
    r.m[1][1] = cLon;
    r.m[1][2] = -cLat * sLon;
    r.m[2][0] = cLat;
    r.m[2][1] = 0.0;
    r.m[2][2] = -sLat;
    return r;
}

Mat3d bodyToNed(double rollRad, double pitchRad, double headingTrueRad) {
    const double sp = std::sin(rollRad), cp = std::cos(rollRad);
    const double st = std::sin(pitchRad), ct = std::cos(pitchRad);
    const double ss = std::sin(headingTrueRad), cs = std::cos(headingTrueRad);
    Mat3d r;
    r.m[0][0] = ct * cs;
    r.m[0][1] = sp * st * cs - cp * ss;
    r.m[0][2] = cp * st * cs + sp * ss;
    r.m[1][0] = ct * ss;
    r.m[1][1] = sp * st * ss + cp * cs;
    r.m[1][2] = cp * st * ss - sp * cs;
    r.m[2][0] = -st;
    r.m[2][1] = sp * ct;
    r.m[2][2] = cp * ct;
    return r;
}

Mat3d operator*(const Mat3d& a, const Mat3d& b) {
    Mat3d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            r.m[i][j] = a.m[i][0] * b.m[0][j] + a.m[i][1] * b.m[1][j] + a.m[i][2] * b.m[2][j];
    return r;
}

Vec3d operator*(const Mat3d& a, const Vec3d& v) {
    return {a.m[0][0] * v.x + a.m[0][1] * v.y + a.m[0][2] * v.z,
            a.m[1][0] * v.x + a.m[1][1] * v.y + a.m[1][2] * v.z,
            a.m[2][0] * v.x + a.m[2][1] * v.y + a.m[2][2] * v.z};
}

Mat3d transpose(const Mat3d& a) {
    Mat3d r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r.m[i][j] = a.m[j][i];
    return r;
}

double wrapPi(double rad) {
    double r = std::fmod(rad + kPi, kTwoPi);
    if (r <= 0.0) r += kTwoPi;
    return r - kPi;
}
double wrap2Pi(double rad) {
    double r = std::fmod(rad, kTwoPi);
    if (r < 0.0) r += kTwoPi;
    return r;
}
double wrap360(double deg) {
    double r = std::fmod(deg, 360.0);
    if (r < 0.0) r += 360.0;
    return r;
}
double wrap180(double deg) {
    double r = std::fmod(deg + 180.0, 360.0);
    if (r <= 0.0) r += 360.0;
    return r - 180.0;
}

}  // namespace fsim
