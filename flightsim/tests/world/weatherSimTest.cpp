// X1: weather in the flight model: crosswind final keeps its track with a crab, gusts, QNH wiring.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "SimFixture.h"

using namespace fsim;
using namespace fsim::test;

namespace {
double angDiffDeg(double a, double b) {
    double d = std::fmod(a - b, 360.0);
    if (d > 180) d -= 360;
    if (d < -180) d += 360;
    return d;
}
}  // namespace

TEST_CASE("sim: crosswind final starts crabbed and keeps its ground track") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("weather_xwind_final_xtst.ini"), f.error);
    f.run(2.0);
    const AircraftState& s = f.sim.aircraft();
    const double track = radToDeg(s.trackTrueRad), hdg = radToDeg(s.headingTrueRad);
    MESSAGE("t=2 s: heading ", hdg, " track ", track, " wind ", s.windFromTrueDeg, "/", s.windKt);
    // surface wind 180/15, veered and stronger at the ~950 ft start height (profile)
    CHECK(s.windFromTrueDeg > 180.0f);
    CHECK(s.windFromTrueDeg < 205.0f);
    CHECK(s.windKt > 15.0f);
    CHECK(s.windKt < 24.0f);
    CHECK(std::fabs(angDiffDeg(track, 270.0)) < 3.0);  // ground track follows the centerline
    CHECK(std::fabs(angDiffDeg(hdg, track)) > 8.0);    // with a crab into the wind
    CHECK(angDiffDeg(hdg, track) < 0);                 // wind from the south: nose south of west
    f.run(18.0);
    const AircraftState& e = f.sim.aircraft();
    const double drift = angDiffDeg(radToDeg(e.trackTrueRad), 270.0);
    MESSAGE("t=20 s: heading ", radToDeg(e.headingTrueRad), " track ", radToDeg(e.trackTrueRad),
            " gs ", e.gsKt);
    CHECK(std::fabs(drift) < 12.0);
    CHECK_FALSE(e.crashed);
}

TEST_CASE("sim: gusts and turbulence move the wind around the mean") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("weather_lowvis_final_xtst.ini"), f.error);
    float lo = 1e9f, hi = 0;
    for (int i = 0; i < 40; ++i) {
        f.run(1.0);
        lo = std::min(lo, f.sim.aircraft().windKt);
        hi = std::max(hi, f.sim.aircraft().windKt);
    }
    MESSAGE("wind ", lo, " .. ", hi, " kt");
    CHECK(hi - lo > 2.0f);
    CHECK(hi < 22.0f);
    CHECK(f.sim.weather().conditions().visibilityM == doctest::Approx(1500.0f));
}

TEST_CASE("sim: altimeter offset follows the setting, not the weather") {
    SimFixture f;
    // QNH 1005, altimeter follows QNH: indicated equals true
    REQUIRE_MESSAGE(f.initFile("weather_lowvis_final_xtst.ini"), f.error);
    f.run(0.1);
    CHECK(std::fabs(f.sim.aircraft().altBaroOffsetFt) < 1.0f);
    // standard setting on a low-pressure day: reads high by about 27 ft per hPa (1013.25 - 1005)
    REQUIRE_MESSAGE(
        f.initText("[scenario]\naircraft = c172p\nairport = XTST\nrunway = 27\nstart = final\n"
                   "[weather]\nqnh_hpa = 1005\naltimeter_hpa = 1013.25\nmetar = off\n"),
        f.error);
    f.run(0.1);
    const float off = f.sim.aircraft().altBaroOffsetFt;
    MESSAGE("offset ", off);
    CHECK(off > 150.0f);
    CHECK(off < 300.0f);
}
