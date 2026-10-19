// W4: solar position, fog math, horizon dip and the Atmosphere service (no GL context needed).
#include <doctest/doctest.h>

#include <cmath>
#include <string>

#include "core/Units.h"
#include "world/Atmosphere.h"

using namespace fsim;
using namespace fsim::world;

namespace {

UtcTime utc(int y, int mo, int d, int h, int mi = 0, double s = 0) {
    UtcTime t;
    t.year = y;
    t.month = mo;
    t.day = d;
    t.hour = h;
    t.minute = mi;
    t.second = s;
    return t;
}

double elevDeg(const UtcTime& t, double latDeg, double lonDeg) {
    return radToDeg(sunAngles(solarPosition(t), latDeg, lonDeg).elevationRad);
}

double azDeg(const UtcTime& t, double latDeg, double lonDeg) {
    return radToDeg(sunAngles(solarPosition(t), latDeg, lonDeg).azimuthTrueRad);
}

}  // namespace

TEST_CASE("sky: parseUtc") {
    UtcTime t;
    CHECK(parseUtc("2026-06-21T18:30:15", t));
    CHECK(t.year == 2026);
    CHECK(t.month == 6);
    CHECK(t.day == 21);
    CHECK(t.hour == 18);
    CHECK(t.minute == 30);
    CHECK(t.second == doctest::Approx(15.0));
    CHECK(parseUtc("2026-06-21 06:05", t));
    CHECK(t.hour == 6);
    CHECK(t.minute == 5);
    CHECK_FALSE(parseUtc("", t));
    CHECK_FALSE(parseUtc("2026-13-01T00:00:00", t));
    CHECK_FALSE(parseUtc("2026-06-21T25:00:00", t));
    CHECK_FALSE(parseUtc("noon", t));
}

TEST_CASE("sky: julian day") {
    CHECK(julianDay(utc(2000, 1, 1, 12)) == doctest::Approx(2451545.0));  // J2000.0
    CHECK(julianDay(utc(1970, 1, 1, 0)) == doctest::Approx(2440587.5));   // Unix epoch
    CHECK(julianDay(utc(2024, 3, 1, 0)) - julianDay(utc(2024, 2, 28, 0)) ==
          doctest::Approx(2.0));  // leap year
}

TEST_CASE("sky: declination and equation of time") {
    // June solstice 2026: declination 23.44 deg. Early November: equation of time about +16.4 min.
    CHECK(radToDeg(solarPosition(utc(2026, 6, 21, 9)).declinationRad) ==
          doctest::Approx(23.44).epsilon(0.002));
    CHECK(radToDeg(solarPosition(utc(2026, 12, 21, 16)).declinationRad) ==
          doctest::Approx(-23.44).epsilon(0.002));
    CHECK(solarPosition(utc(2026, 11, 3, 12)).equationOfTimeMin ==
          doctest::Approx(16.4).epsilon(0.02));
    // The equinox (2026-03-20 14:46 UTC) has declination ~0.
    CHECK(std::fabs(radToDeg(solarPosition(utc(2026, 3, 20, 14, 46)).declinationRad)) < 0.05);
}

TEST_CASE("sky: solar elevation at known places and times") {
    // Greenwich Observatory (51.4769 N, 0), 2000-01-01 12:00 UTC: declination -23.03 deg, equation
    // of time -3.6 min, so the sun is 3.6 min before its meridian passage. Meridian altitude is
    // 90 - 51.4769 - 23.03 = 15.49 deg; the NOAA calculator gives 15.4 deg at this instant.
    CHECK(elevDeg(utc(2000, 1, 1, 12), 51.4769, 0.0) == doctest::Approx(15.4).epsilon(0.01));
    // Solar noon at the Tropic latitude on the solstice: the sun is at the zenith.
    // Equation of time on 2026-06-21 is about -1.7 min, so the meridian passage over lon 0 is at
    // 12:01:42 UTC.
    CHECK(elevDeg(utc(2026, 6, 21, 12, 1, 42), 23.44, 0.0) > 89.8);
    // Same instant at the Arctic circle edge: 90 - (66.56 - 23.44) = 46.9 deg.
    CHECK(elevDeg(utc(2026, 6, 21, 12, 1, 42), 66.56, 0.0) == doctest::Approx(46.88).epsilon(0.01));
    // Midnight sun checks: below the horizon at local midnight in the tropics.
    CHECK(elevDeg(utc(2026, 6, 21, 0, 0), 23.44, 0.0) < -40.0);
}

TEST_CASE("sky: solar azimuth") {
    // Northern mid-latitude, local morning: sun in the east; afternoon: west; noon: south.
    // XTST is at 40 N, 100 W: local mean solar time = UTC - 6.667 h.
    CHECK(azDeg(utc(2026, 3, 20, 18, 47), 40.0, -100.0) == doctest::Approx(180.0).epsilon(0.01));
    const double morning = azDeg(utc(2026, 3, 20, 12), 40.0, -100.0);
    const double evening = azDeg(utc(2026, 3, 20, 23), 40.0, -100.0);
    CHECK(morning > 80.0);
    CHECK(morning < 130.0);
    CHECK(evening > 230.0);
    CHECK(evening < 280.0);
    // Southern hemisphere summer noon: sun to the north.
    const double north = azDeg(utc(2026, 12, 21, 12), -35.0, 0.0);
    CHECK((north < 20.0 || north > 340.0));
}

TEST_CASE("sky: sun direction frames agree") {
    const UtcTime t = utc(2026, 6, 22, 2);
    const SolarPosition sp = solarPosition(t);
    const Vec3d ecef = sunDirEcef(sp);
    CHECK(std::sqrt(ecef.x * ecef.x + ecef.y * ecef.y + ecef.z * ecef.z) == doctest::Approx(1.0));
    const Vec3d ned = sunDirNed(sp, 40.0, -100.0);
    CHECK(std::sqrt(ned.x * ned.x + ned.y * ned.y + ned.z * ned.z) == doctest::Approx(1.0));
    const SunAngles a = sunAngles(sp, 40.0, -100.0);
    CHECK(-ned.z == doctest::Approx(std::sin(a.elevationRad)));
    CHECK(ned.x == doctest::Approx(std::cos(a.elevationRad) * std::cos(a.azimuthTrueRad)));
    CHECK(ned.y == doctest::Approx(std::cos(a.elevationRad) * std::sin(a.azimuthTrueRad)));
    // Subsolar point sees the sun overhead.
    const SunAngles overhead = sunAngles(sp, sp.subsolarLatDeg, sp.subsolarLonDeg);
    CHECK(radToDeg(overhead.elevationRad) > 89.99);
}

TEST_CASE("sky: fog density from visibility") {
    CHECK(fogDensityPerM(3912.0f) == doctest::Approx(1.0e-3f));
    CHECK(fogDensityPerM(25000.0f) == doctest::Approx(3.912f / 25000.0f));
    // Contrast falls to 2 % at the visibility distance.
    CHECK(fogTransmittance(fogDensityPerM(10000.0f), 10000.0f) ==
          doctest::Approx(0.02f).epsilon(0.01));
    CHECK(fogTransmittance(1e-4f, 0.0f) == doctest::Approx(1.0f));
    CHECK(fogTransmittance(1e-4f, -5.0f) == doctest::Approx(1.0f));  // negative distance clamps
    // Monotonic: poorer visibility, denser fog; farther, more fogged.
    CHECK(fogDensityPerM(1000.0f) > fogDensityPerM(5000.0f));
    CHECK(fogTransmittance(1e-4f, 20000.0f) < fogTransmittance(1e-4f, 10000.0f));
    // Clamped to 200 m .. 200 km.
    CHECK(fogDensityPerM(1.0f) == doctest::Approx(3.912f / 200.0f));
    CHECK(fogDensityPerM(1e9f) == doctest::Approx(3.912f / 200000.0f));
}

TEST_CASE("sky: horizon dip") {
    CHECK(horizonDipRad(0.0) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(horizonDipRad(-50.0) == doctest::Approx(0.0).epsilon(1e-9));
    // FL350 = 10,668 m: dip = acos(R / (R + h)) = 3.28 deg.
    CHECK(radToDeg(horizonDipRad(ftToM(35000.0))) == doctest::Approx(3.28).epsilon(0.01));
    // Small-angle check at 1,000 m: sqrt(2h/R) = 0.0177 rad.
    CHECK(horizonDipRad(1000.0) ==
          doctest::Approx(std::sqrt(2.0 * 1000.0 / kEarthRadiusM)).epsilon(0.002));
}

TEST_CASE("sky: atmosphere service") {
    Atmosphere atm;
    CHECK(atm.setUtcText("2026-06-21T18:00:00"));
    atm.setVisibilityM(25000.0f);
    atm.setObserver(40.0, -100.0, 2.0);
    CHECK(atm.fog().densityPerM == doctest::Approx(3.912f / 25000.0f));
    CHECK(atm.fog().visibilityM == doctest::Approx(25000.0f));
    // Late morning local solar time: sun high.
    CHECK(radToDeg(atm.sun().elevationRad) > 60.0);
    CHECK(atm.light().sunDirNed[2] < -0.85f);  // down component negative = sun above the horizon
    CHECK(atm.light().diffuseRgb[0] > 0.6f);

    const float dayHaze = atm.fog().colorRgb[2];
    const float dayZenithGround = atm.sky().zenithRgb[2];

    // Altitude deepens the sky and thins the haze band; the fog color is unchanged.
    atm.setObserver(40.0, -100.0, ftToM(33000.0));
    CHECK(atm.sky().zenithRgb[2] < dayZenithGround);
    CHECK(atm.sky().bandSharpness > 5.0f);
    CHECK(atm.sky().horizonDipRad > 0.05f);
    CHECK(atm.fog().colorRgb[2] == doctest::Approx(dayHaze));

    // Night: haze and light drop, the sun is below the horizon.
    CHECK(atm.setUtcText("2026-06-22T08:00:00"));
    atm.setObserver(40.0, -100.0, 2.0);
    CHECK(atm.sun().elevationRad < 0.0);
    CHECK(atm.fog().colorRgb[2] < 0.2f * dayHaze);
    CHECK(atm.light().diffuseRgb[0] < 0.01f);
    CHECK(atm.light().ambientRgb[0] < 0.1f);

    // A bad time string leaves the previous time in place.
    CHECK_FALSE(atm.setUtcText("not a time"));
    CHECK(atm.utc().hour == 8);

    // Visibility is clamped.
    atm.setVisibilityM(10.0f);
    CHECK(atm.fog().visibilityM == doctest::Approx(Atmosphere::kMinVisibilityM));
}

TEST_CASE("sky: fog GLSL snippet uses the shared uniform names") {
    const std::string glsl = fogGlsl();
    CHECK(glsl.find("uniform vec3 uFogColor;") != std::string::npos);
    CHECK(glsl.find("uniform float uFogDensity;") != std::string::npos);
    CHECK(glsl.find("fsimApplyFog") != std::string::npos);
}
