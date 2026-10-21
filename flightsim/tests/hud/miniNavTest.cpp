// Mini-nav: range clamp, track-up projection, readout, and the CPU cost of one inset.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstring>

#include "core/AircraftState.h"
#include "core/Geo.h"
#include "core/Units.h"
#include "gfx/GlApi.h"
#include "hud/MiniNav.h"
#include "hud/Renderer2D.h"

using namespace fsim;

namespace {

FlightPlan samplePlan() {
    FlightPlan p;
    p.activeLeg = 1;
    p.approachStartLeg = 3;
    const double lats[] = {45.0, 45.2, 45.2, 45.05};
    const double lons[] = {-122.0, -122.0, -121.7, -121.7};
    const char* ids[] = {"AAAA", "BBBB", "CCCC", "DDDD"};
    for (int i = 0; i < 4; ++i) {
        Leg leg;
        leg.to.ident = ids[i];
        leg.to.latDeg = lats[i];
        leg.to.lonDeg = lons[i];
        if (i == 3) leg.missedApproach = true;
        p.legs.push_back(leg);
    }
    return p;
}

AircraftState aircraftAt(double lat, double lon, float trackRad, float gs) {
    AircraftState ac;
    ac.latDeg = lat;
    ac.lonDeg = lon;
    ac.trackTrueRad = trackRad;
    ac.headingTrueRad = trackRad;
    ac.gsKt = gs;
    return ac;
}

}  // namespace

TEST_CASE("mininav: range follows the next waypoint or a fixed setting") {
    CHECK(miniNavAutoRangeNm(0) == doctest::Approx(2));
    CHECK(miniNavAutoRangeNm(0.4f) == doctest::Approx(2));
    CHECK(miniNavAutoRangeNm(8) == doctest::Approx(9.6f));
    CHECK(miniNavAutoRangeNm(100) == doctest::Approx(20));
    CHECK(miniNavRangeNm(8, 0) == doctest::Approx(9.6f));
    CHECK(miniNavRangeNm(8, 10) == doctest::Approx(10));
    CHECK(miniNavRangeNm(8, 1) == doctest::Approx(2));
    CHECK(miniNavRangeNm(8, 50) == doctest::Approx(20));
    CHECK(miniNavShown(false, Declutter::Full, false) == false);
    CHECK(miniNavShown(true, Declutter::Full, false));
    CHECK(miniNavShown(true, Declutter::Std, false));
    CHECK(miniNavShown(true, Declutter::Min, false) == false);
    CHECK(miniNavShown(true, Declutter::Full, true) == false);
}

TEST_CASE("mininav: track-up projection puts the track ahead and up") {
    const MiniNavBox box = kMiniNavBox;
    const MapProjection north = miniNavProjection(45.0, -122.0, 0.0f, 10.0f, box);
    double aheadLat = 0, aheadLon = 0;
    destinationPoint(45.0, -122.0, 0.0, nmToM(10.0), aheadLat, aheadLon);
    float x, y;
    north.toScreen(aheadLat, aheadLon, x, y);
    CHECK(x == doctest::Approx(north.cx).epsilon(0.02));
    CHECK(y < north.cy - north.ringRadiusPx * 0.8f);

    double rightLat = 0, rightLon = 0;
    destinationPoint(45.0, -122.0, degToRad(90.0), nmToM(10.0), rightLat, rightLon);
    north.toScreen(rightLat, rightLon, x, y);
    CHECK(x > north.cx + north.ringRadiusPx * 0.8f);
    CHECK(y == doctest::Approx(north.cy).epsilon(0.05));

    const MapProjection east =
        miniNavProjection(45.0, -122.0, static_cast<float>(degToRad(90.0)), 10.0f, box);
    east.toScreen(rightLat, rightLon, x, y);
    CHECK(y < east.cy - east.ringRadiusPx * 0.8f);
    CHECK(x == doctest::Approx(east.cx).epsilon(0.05));

    // A wider range pulls the same point closer to the aircraft.
    const MapProjection wide = miniNavProjection(45.0, -122.0, 0.0f, 20.0f, box);
    float yNear = 0, yFar = 0, dummy = 0;
    north.toScreen(aheadLat, aheadLon, dummy, yNear);
    wide.toScreen(aheadLat, aheadLon, dummy, yFar);
    CHECK(std::fabs(yFar - wide.cy) < std::fabs(yNear - north.cy) * 0.7f);
}

TEST_CASE("mininav: readout uses nav state, else the active leg") {
    const FlightPlan plan = samplePlan();
    NavState nav;
    std::snprintf(nav.activeWptIdent, sizeof nav.activeWptIdent, "BBBB");
    nav.distToWptNm = 6.0f;
    nav.eteToWptS = 180;
    nav.crossTrackNm = -0.4f;
    nav.activeLegIndex = 1;
    const MiniNavReadout fromNav = miniNavReadout(45.0, -122.0, 120, &nav, &plan, 0);
    CHECK(fromNav.hasPlan);
    CHECK(std::strcmp(fromNav.ident, "BBBB") == 0);
    CHECK(fromNav.distNm == doctest::Approx(6));
    CHECK(fromNav.eteS == doctest::Approx(180));
    CHECK(fromNav.crossTrackNm == doctest::Approx(-0.4f));
    CHECK(fromNav.rangeNm == doctest::Approx(7.2f));

    const MiniNavReadout fromPlan = miniNavReadout(45.0, -122.0, 120, nullptr, &plan, 5);
    CHECK(std::strcmp(fromPlan.ident, "BBBB") == 0);
    CHECK(fromPlan.distNm == doctest::Approx(12).epsilon(0.05));  // 0.2 deg * 60 NM
    CHECK(fromPlan.rangeNm == doctest::Approx(5));                // fixed wins
    CHECK(fromPlan.crossTrackNm == doctest::Approx(0).epsilon(0.05));

    const MiniNavReadout empty = miniNavReadout(45.0, -122.0, 0, nullptr, nullptr, 0);
    CHECK_FALSE(empty.hasPlan);
    CHECK(empty.rangeNm == doctest::Approx(2));
}

TEST_CASE("mininav: draw stays inside the box and the active leg is magenta") {
    gfx::GlApi api{};
    Renderer2D r(api, gfx::GlFlavor::Es2);
    MiniNav mini;
    const FlightPlan plan = samplePlan();
    AircraftState ac = aircraftAt(45.05, -122.0, 0, 110);
    NavState nav;
    std::snprintf(nav.activeWptIdent, sizeof nav.activeWptIdent, "BBBB");
    nav.distToWptNm = 9;
    nav.eteToWptS = 300;
    nav.activeLegIndex = 1;
    r.begin(640, 480);
    mini.draw(r, ac, nav, &plan, 0);
    r.end();
    REQUIRE(r.batchVertexCount() > 0);
    int magenta = 0;
    const MiniNavBox box = kMiniNavBox;
    for (int i = 0; i < r.batchVertexCount(); ++i) {
        const Renderer2D::Vertex& v = r.batchData()[i];
        CHECK(v.x >= box.x - 1.0f);
        CHECK(v.x <= box.x + box.w + 1.0f);
        CHECK(v.y >= box.y - 1.0f);
        CHECK(v.y <= box.y + box.h + 1.0f);
        if (v.r > 200 && v.b > 200 && v.g < 120 && v.a > 100) ++magenta;
    }
    CHECK(magenta > 0);

    // Warm the vertex buffers, then time the geometry build. Under 0.3 ms on desktop.
    for (int i = 0; i < 3; ++i) {
        r.begin(640, 480);
        mini.draw(r, ac, nav, &plan, 0);
        r.end();
    }
    constexpr int kFrames = 40;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) {
        r.begin(640, 480);
        mini.draw(r, ac, nav, &plan, 0);
        r.end();
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / kFrames;
    MESSAGE("mini nav frame ", ms, " ms");
    CHECK(ms < 0.3);
}
