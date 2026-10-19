#include "world/Atmosphere.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/Units.h"

namespace fsim::world {
namespace {

float clamp01(float v) {
    return std::min(1.0f, std::max(0.0f, v));
}

float smoothstep(float e0, float e1, float x) {
    const float t = clamp01((x - e0) / (e1 - e0));
    return t * t * (3.0f - 2.0f * t);
}

void mix3(const float a[3], const float b[3], float t, float out[3]) {
    for (int i = 0; i < 3; ++i) out[i] = a[i] + (b[i] - a[i]) * t;
}

// Days since 1970-01-01 of a proleptic Gregorian date (Howard Hinnant's algorithm).
long daysFromCivil(int y, int m, int d) {
    y -= m <= 2;
    const long era = (y >= 0 ? y : y - 399) / 400;
    const long yoe = y - era * 400;
    const long doy = (153L * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const long doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    return era * 146097 + doe - 719468;
}

constexpr char kFogGlsl[] =
    "uniform vec3 uFogColor;\n"
    "uniform float uFogDensity;\n"
    "float fsimFogFactor(float distM) { return exp(-uFogDensity * distM); }\n"
    "vec3 fsimApplyFog(vec3 rgb, float distM) {\n"
    "  return mix(uFogColor, rgb, fsimFogFactor(distM));\n"
    "}\n";

}  // namespace

bool parseUtc(const std::string& text, UtcTime& out) {
    int y, mo, d, h, mi;
    double s = 0;
    char sep = 0;
    const int n = std::sscanf(text.c_str(), "%d-%d-%d%c%d:%d:%lf", &y, &mo, &d, &sep, &h, &mi, &s);
    if (n < 6 || (sep != 'T' && sep != ' ')) return false;
    if (mo < 1 || mo > 12 || d < 1 || d > 31 || h < 0 || h > 23 || mi < 0 || mi > 59 || s < 0 ||
        s >= 61 || y < 1900 || y > 2100)
        return false;
    out.year = y;
    out.month = mo;
    out.day = d;
    out.hour = h;
    out.minute = mi;
    out.second = s;
    return true;
}

double julianDay(const UtcTime& t) {
    const double days = static_cast<double>(daysFromCivil(t.year, t.month, t.day));
    const double secs = t.hour * 3600.0 + t.minute * 60.0 + t.second;
    return 2440587.5 + days + secs / 86400.0;  // JD of the Unix epoch is 2440587.5
}

SolarPosition solarPosition(const UtcTime& t) {
    const double jd = julianDay(t);
    const double T = (jd - 2451545.0) / 36525.0;  // Julian centuries since J2000
    const double l0 = std::fmod(280.46646 + T * (36000.76983 + T * 0.0003032), 360.0);
    const double mDeg = 357.52911 + T * (35999.05029 - T * 0.0001537);
    const double e = 0.016708634 - T * (0.000042037 + T * 0.0000001267);
    const double mRad = degToRad(mDeg);
    const double c = std::sin(mRad) * (1.914602 - T * (0.004817 + 0.000014 * T)) +
                     std::sin(2 * mRad) * (0.019993 - 0.000101 * T) + std::sin(3 * mRad) * 0.000289;
    const double trueLong = l0 + c;
    const double omega = 125.04 - 1934.136 * T;
    const double lambda = trueLong - 0.00569 - 0.00478 * std::sin(degToRad(omega));
    const double eps0 =
        23.0 + (26.0 + (21.448 - T * (46.815 + T * (0.00059 - T * 0.001813))) / 60.0) / 60.0;
    const double eps = degToRad(eps0 + 0.00256 * std::cos(degToRad(omega)));

    SolarPosition sp;
    sp.declinationRad = std::asin(std::sin(eps) * std::sin(degToRad(lambda)));
    const double y = std::tan(eps / 2) * std::tan(eps / 2);
    const double l0Rad = degToRad(l0);
    sp.equationOfTimeMin =
        4.0 * radToDeg(y * std::sin(2 * l0Rad) - 2 * e * std::sin(mRad) +
                       4 * e * y * std::sin(mRad) * std::cos(2 * l0Rad) -
                       0.5 * y * y * std::sin(4 * l0Rad) - 1.25 * e * e * std::sin(2 * mRad));
    sp.subsolarLatDeg = radToDeg(sp.declinationRad);
    const double utcMin = t.hour * 60.0 + t.minute + t.second / 60.0;
    sp.subsolarLonDeg = wrap180(-(utcMin + sp.equationOfTimeMin - 720.0) / 4.0);
    return sp;
}

Vec3d sunDirEcef(const SolarPosition& sun) {
    const double lat = degToRad(sun.subsolarLatDeg), lon = degToRad(sun.subsolarLonDeg);
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

Vec3d sunDirNed(const SolarPosition& sun, double latDeg, double lonDeg) {
    return transpose(nedToEcef(latDeg, lonDeg)) * sunDirEcef(sun);
}

SunAngles sunAngles(const SolarPosition& sun, double latDeg, double lonDeg) {
    const Vec3d ned = sunDirNed(sun, latDeg, lonDeg);
    SunAngles a;
    a.elevationRad = std::asin(std::max(-1.0, std::min(1.0, -ned.z)));
    a.azimuthTrueRad = wrap2Pi(std::atan2(ned.y, ned.x));
    return a;
}

float fogDensityPerM(float visibilityM) {
    const float v =
        std::min(Atmosphere::kMaxVisibilityM, std::max(Atmosphere::kMinVisibilityM, visibilityM));
    return 3.912f / v;
}

float fogTransmittance(float densityPerM, float distM) {
    return std::exp(-densityPerM * std::max(0.0f, distM));
}

double horizonDipRad(double heightM) {
    const double h = std::max(0.0, heightM);
    return std::acos(kEarthRadiusM / (kEarthRadiusM + h));
}

Atmosphere::Atmosphere() {
    solar_ = solarPosition(utc_);
    update();
}

void Atmosphere::setVisibilityM(float visibilityM) {
    visibilityM_ = std::min(kMaxVisibilityM, std::max(kMinVisibilityM, visibilityM));
    update();
}

void Atmosphere::setUtc(const UtcTime& t) {
    utc_ = t;
    solar_ = solarPosition(utc_);
    update();
}

bool Atmosphere::setUtcText(const std::string& text) {
    UtcTime t;
    if (!parseUtc(text, t)) return false;
    setUtc(t);
    return true;
}

void Atmosphere::setObserver(double latDeg, double lonDeg, double heightAglM) {
    latDeg_ = latDeg;
    lonDeg_ = lonDeg;
    heightAglM_ = heightAglM;
    update();
}

void Atmosphere::update() {
    sunAngles_ = sunAngles(solar_, latDeg_, lonDeg_);
    const float elevDeg = static_cast<float>(radToDeg(sunAngles_.elevationRad));

    // Sun-driven brightness: 0 once the sun is 6 deg below the horizon, 1 above 10 deg.
    const float dayF = smoothstep(-6.0f, 10.0f, elevDeg);
    // Warm tint near the horizon (sunrise/sunset), strongest around 0..4 deg elevation.
    const float warm = std::exp(-((elevDeg - 2.0f) / 7.0f) * ((elevDeg - 2.0f) / 7.0f));
    // 0 at the ground, 1 at FL350 (10,668 m).
    const float altF = clamp01(static_cast<float>(heightAglM_ / 10668.0));

    // Fog and horizon haze: ONE color, used for sky horizon, clear color and surface fog.
    static const float kNightHaze[3] = {0.020f, 0.028f, 0.055f};
    static const float kDayHaze[3] = {0.72f, 0.80f, 0.90f};
    static const float kWarmHaze[3] = {0.95f, 0.68f, 0.50f};
    float lit[3];
    mix3(kDayHaze, kWarmHaze, warm * 0.65f, lit);
    mix3(kNightHaze, lit, dayF, fog_.colorRgb);
    fog_.visibilityM = visibilityM_;
    fog_.densityPerM = fogDensityPerM(visibilityM_);

    // Sky above the haze band: deeper blue with altitude, dark at night.
    static const float kNightZenith[3] = {0.008f, 0.012f, 0.035f};
    static const float kDayZenithLow[3] = {0.24f, 0.44f, 0.84f};
    static const float kDayZenithHigh[3] = {0.07f, 0.16f, 0.48f};
    static const float kTwilightZenith[3] = {0.20f, 0.26f, 0.50f};
    float dayZenith[3];
    mix3(kDayZenithLow, kDayZenithHigh, altF, dayZenith);
    mix3(dayZenith, kTwilightZenith, warm * 0.35f, dayZenith);
    mix3(kNightZenith, dayZenith, dayF, sky_.zenithRgb);
    sky_.bandSharpness = 5.0f + 7.0f * altF;
    sky_.horizonDipRad = static_cast<float>(horizonDipRad(heightAglM_));
    static const float kSunWhite[3] = {1.0f, 0.97f, 0.88f};
    static const float kSunRed[3] = {1.0f, 0.60f, 0.30f};
    mix3(kSunWhite, kSunRed, warm, sky_.sunDiscRgb);
    static const float kGlowWhite[3] = {1.0f, 0.90f, 0.70f};
    static const float kGlowRed[3] = {1.0f, 0.55f, 0.25f};
    mix3(kGlowWhite, kGlowRed, warm, sky_.glowRgb);
    sky_.glowStrength = 0.30f * smoothstep(-8.0f, 4.0f, elevDeg);

    // Light terms for terrain shading.
    const float sunI = smoothstep(-2.0f, 12.0f, elevDeg);
    float sunColor[3];
    mix3(kSunWhite, kSunRed, warm * 0.8f, sunColor);
    for (int i = 0; i < 3; ++i) light_.diffuseRgb[i] = sunColor[i] * 0.85f * sunI;
    static const float kNightAmbient[3] = {0.035f, 0.045f, 0.085f};
    static const float kDayAmbient[3] = {0.34f, 0.40f, 0.50f};
    mix3(kNightAmbient, kDayAmbient, dayF, light_.ambientRgb);
    const Vec3d ecef = sunDirEcef(solar_);
    const Vec3d ned = sunDirNed(solar_, latDeg_, lonDeg_);
    light_.sunDirEcef[0] = static_cast<float>(ecef.x);
    light_.sunDirEcef[1] = static_cast<float>(ecef.y);
    light_.sunDirEcef[2] = static_cast<float>(ecef.z);
    light_.sunDirNed[0] = static_cast<float>(ned.x);
    light_.sunDirNed[1] = static_cast<float>(ned.y);
    light_.sunDirNed[2] = static_cast<float>(ned.z);
}

const char* fogGlsl() {
    return kFogGlsl;
}

void FogUniforms::locate(const gfx::ShaderProgram& program) {
    color = program.uniform("uFogColor");
    density = program.uniform("uFogDensity");
}

void FogUniforms::set(const gfx::GlApi& gl, const FogParams& fog) const {
    if (color >= 0) gl.Uniform3f(color, fog.colorRgb[0], fog.colorRgb[1], fog.colorRgb[2]);
    if (density >= 0) gl.Uniform1f(density, fog.densityPerM);
}

}  // namespace fsim::world
