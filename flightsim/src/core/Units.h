#pragma once
// Unit constants and conversion helpers. All conversions go through here (CONVENTIONS section 2):
// no magic numbers like 0.3048 inline anywhere else. Math is done in SI and radians.

namespace fsim {

constexpr double kPi = 3.14159265358979323846;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kDegPerRad = 180.0 / kPi;
constexpr double kRadPerDeg = kPi / 180.0;

constexpr double kMPerFt = 0.3048;
constexpr double kFtPerM = 1.0 / kMPerFt;
constexpr double kMPerNm = 1852.0;
constexpr double kNmPerM = 1.0 / kMPerNm;
constexpr double kFtPerNm = kMPerNm / kMPerFt;
constexpr double kMpsPerKt = kMPerNm / 3600.0;
constexpr double kKtPerMps = 1.0 / kMpsPerKt;
constexpr double kFpmPerMps = 60.0 * kFtPerM;
constexpr double kMpsPerFpm = 1.0 / kFpmPerMps;
constexpr double kKgPerLb = 0.45359237;
constexpr double kLbPerKg = 1.0 / kKgPerLb;
constexpr double kHpaPerInHg = 33.8638866667;

constexpr double kStdGravityMps2 = 9.80665;

// Earth: mean radius for spherical formulas, WGS84 ellipsoid for geodetic <-> ECEF.
constexpr double kEarthRadiusM = 6371008.8;
constexpr double kWgs84AM = 6378137.0;                   // semi-major axis
constexpr double kWgs84F = 1.0 / 298.257223563;          // flattening
constexpr double kWgs84BM = kWgs84AM * (1.0 - kWgs84F);  // semi-minor axis
constexpr double kWgs84E2 = kWgs84F * (2.0 - kWgs84F);   // first eccentricity squared

constexpr double degToRad(double deg) {
    return deg * kRadPerDeg;
}
constexpr double radToDeg(double rad) {
    return rad * kDegPerRad;
}
constexpr double ftToM(double ft) {
    return ft * kMPerFt;
}
constexpr double mToFt(double m) {
    return m * kFtPerM;
}
constexpr double nmToM(double nm) {
    return nm * kMPerNm;
}
constexpr double mToNm(double m) {
    return m * kNmPerM;
}
constexpr double nmToFt(double nm) {
    return nm * kFtPerNm;
}
constexpr double ftToNm(double ft) {
    return ft / kFtPerNm;
}
constexpr double ktToMps(double kt) {
    return kt * kMpsPerKt;
}
constexpr double mpsToKt(double mps) {
    return mps * kKtPerMps;
}
constexpr double fpmToMps(double fpm) {
    return fpm * kMpsPerFpm;
}
constexpr double mpsToFpm(double mps) {
    return mps * kFpmPerMps;
}
constexpr double fpsToMps(double fps) {
    return fps * kMPerFt;
}
constexpr double mpsToFps(double mps) {
    return mps * kFtPerM;
}
constexpr double fpsToKt(double fps) {
    return fps * kMPerFt * kKtPerMps;
}
constexpr double ktToFps(double kt) {
    return kt * kMpsPerKt * kFtPerM;
}
constexpr double lbsToKg(double lbs) {
    return lbs * kKgPerLb;
}
constexpr double kgToLbs(double kg) {
    return kg * kLbPerKg;
}
constexpr double inHgToHpa(double inHg) {
    return inHg * kHpaPerInHg;
}
constexpr double hpaToInHg(double hpa) {
    return hpa / kHpaPerInHg;
}

}  // namespace fsim
