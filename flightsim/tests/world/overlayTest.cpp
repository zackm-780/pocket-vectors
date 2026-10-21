// World overlays: presets, label placement, airport culling, marker altitude, runway width,
// batch count and the CPU cost of one build.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "core/Geo.h"
#include "core/Units.h"
#include "world/WorldOverlayLogic.h"

using namespace fsim;
using namespace fsim::world;

namespace {

OverlayEye eyeAt(double lat, double lon, double altM, float headingRad) {
    OverlayEye eye;
    overlayEyeAt(lat, lon, altM, headingRad, 55.0f, 4.0f / 3.0f, eye);
    return eye;
}

FlightPlan linePlan(int legs, int active) {
    FlightPlan p;
    p.cruiseAltFt = 8000;
    p.activeLeg = active;
    for (int i = 0; i < legs; ++i) {
        Leg leg;
        char id[8];
        std::snprintf(id, sizeof id, "W%02d", i);
        leg.to.ident = id;
        leg.to.latDeg = 45.0 + 0.05 * i;
        leg.to.lonDeg = -122.0;
        if (i == legs - 1) {
            leg.alt.kind = AltConstraintKind::At;
            leg.alt.altFt = 3000;
        }
        p.legs.push_back(leg);
    }
    return p;
}

Airport fieldAt(const char* ident, double lat, double lon) {
    Airport ap;
    ap.ident = ident;
    ap.latDeg = lat;
    ap.lonDeg = lon;
    ap.elevFt = 30;
    Runway rw;
    rw.ident = "10";
    rw.oppositeIdent = "28";
    rw.startLatDeg = lat;
    rw.startLonDeg = lon;
    rw.endLatDeg = lat + 0.02;
    rw.endLonDeg = lon;
    rw.widthFt = 150;
    rw.thrElevFt = 30;
    rw.headingTrueDeg = 10;
    Runway back = rw;
    back.ident = "28";
    back.oppositeIdent = "10";
    back.startLatDeg = rw.endLatDeg;
    back.startLonDeg = rw.endLonDeg;
    back.endLatDeg = rw.startLatDeg;
    back.endLonDeg = rw.startLonDeg;
    back.headingTrueDeg = 190;
    ap.runways.push_back(rw);
    ap.runways.push_back(back);
    return ap;
}

}  // namespace

TEST_CASE("overlay: presets cycle and a custom mix returns to off") {
    OverlayToggles t;
    CHECK(overlayMatchingPreset(t) == 0);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "ROUTE");
    CHECK(t.route);
    CHECK_FALSE(t.vpath);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "ROUTE+PATH");
    CHECK(t.route);
    CHECK(t.vpath);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "AIRPORTS");
    CHECK(t.airports);
    CHECK(t.runways);
    CHECK_FALSE(t.route);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "ALL");
    CHECK(t.route);
    CHECK(t.vpath);
    CHECK(t.airports);
    CHECK(t.runways);
    CHECK(overlayCycle(t) == 0);
    CHECK_FALSE(t.route);

    t = {};
    t.vpath = true;
    CHECK(overlayMatchingPreset(t) == -1);
    CHECK(std::string(overlayPresetName(-1)) == "CUSTOM");
    CHECK(overlayCycle(t) == 0);
    CHECK_FALSE(t.vpath);
}

TEST_CASE("overlay: marker altitude prefers a constraint, then cruise, then a pole") {
    AltConstraint none;
    AltConstraint at;
    at.kind = AltConstraintKind::At;
    at.altFt = 6000;
    CHECK(overlayMarkerAltFt(at, 8000, 100) == doctest::Approx(6000));
    CHECK(overlayMarkerAltFt(none, 8000, 100) == doctest::Approx(8000));
    CHECK(overlayMarkerAltFt(none, 150, 100) ==
          doctest::Approx(500));  // cruise is not above ground
    AltConstraint low;
    low.kind = AltConstraintKind::AtOrAbove;
    low.altFt = 0.5f;
    CHECK(overlayMarkerAltFt(low, 0, 20) == doctest::Approx(420));
}

TEST_CASE("overlay: airports inside 1 NM and past the radius are dropped, nearest 12 kept") {
    std::vector<double> dist = {0.4, 0.9, 1.0, 1.01, 5.0, 25.0, 25.01, 40.0};
    std::vector<int> keep;
    selectOverlayAirports(dist, 25.0, 1.0, 12, keep);
    REQUIRE(keep.size() == 3);
    CHECK(keep[0] == 3);  // 1.01 NM
    CHECK(keep[1] == 4);  // 5 NM
    CHECK(keep[2] == 5);  // 25 NM, the radius edge

    std::vector<double> many;
    for (int i = 0; i < 20; ++i) many.push_back(2.0 + i);
    selectOverlayAirports(many, 80.0, 1.0, 12, keep);
    REQUIRE(keep.size() == 12);
    CHECK(keep.front() == 0);
    CHECK(many[static_cast<size_t>(keep.back())] < many[static_cast<size_t>(keep.front())] + 12.0);
}

TEST_CASE("overlay: a label nudges off a neighbour and stays on the canvas") {
    OverlayBox placed{60.0f, 92.0f, 80.0f, 16.0f};
    float x = 110, y = 100, w = 80, h = 16;
    REQUIRE(placeOverlayLabel(x, y, w, h, &placed, 1, 640, 480));
    CHECK(y < 100);
    const float left = x - w * 0.5f, top = y - h * 0.5f;
    CHECK(left >= 2);
    CHECK(top >= 2);
    CHECK(left + w <= 638);
    CHECK(top + h <= 478);
    const bool overlap = left < placed.x + placed.w && left + w > placed.x &&
                         top < placed.y + placed.h && top + h > placed.y;
    CHECK_FALSE(overlap);
}

TEST_CASE("overlay: runway highlight grows with distance and never shrinks below the pavement") {
    const float vFov = static_cast<float>(degToRad(55.0));
    const float near = overlayRunwayHalfWidthM(10.0f, 500.0f, 4.0f, 480.0f, vFov);
    const float far = overlayRunwayHalfWidthM(10.0f, 20000.0f, 4.0f, 480.0f, vFov);
    CHECK(near == doctest::Approx(10.0f));
    CHECK(far > 100.0f);
    CHECK(metersForPixels(1000.0f, 0.0f, 480.0f, vFov) == doctest::Approx(0.0f));
}

TEST_CASE("overlay: a point ahead of the eye projects onto the canvas") {
    const OverlayEye eye = eyeAt(45.0, -122.0, 2000.0, 0.0f);
    double lat = 0, lon = 0;
    destinationPoint(45.0, -122.0, 0.0, nmToM(2.0), lat, lon);
    FlightPlan plan = linePlan(2, 1);
    plan.legs[1].to.latDeg = lat;
    plan.legs[1].to.lonDeg = lon;
    plan.legs[1].to.ident = "AHEAD";
    plan.legs[0].to.latDeg = 45.0;
    plan.legs[0].to.lonDeg = -122.0;
    OverlayToggles t;
    t.route = true;
    std::vector<float> route, vpath, ap, rw;
    OverlayLabel labels[8];
    int n = 0;
    buildWorldOverlay(t, &plan, nullptr, 0, nullptr, 0, nullptr, eye, route, vpath, ap, rw, labels,
                      8, n);
    bool ahead = false;
    for (int i = 0; i < n; ++i) {
        if (std::string(labels[i].text).find("AHEAD") == std::string::npos) continue;
        ahead = true;
        CHECK(labels[i].x > 200.0f);
        CHECK(labels[i].x < 440.0f);
        CHECK(labels[i].y > 40.0f);
        CHECK(labels[i].y < 470.0f);
    }
    CHECK(ahead);
}

TEST_CASE("overlay: opposite runway direction is drawn once, and four layers are four batches") {
    const OverlayEye eye = eyeAt(45.0, -122.2, 1500.0, 0.0f);
    Airport both = fieldAt("KPDX", 45.05, -122.2);
    Airport one = both;
    one.runways.pop_back();
    OverlayAirportRef refs[] = {{&both, 4.0}, {&one, 4.0}};
    OverlayToggles t;
    t.runways = true;
    std::vector<float> route, vpath, ap, rwBoth, rwOne;
    OverlayLabel labels[4];
    int n = 0;
    buildWorldOverlay(t, nullptr, nullptr, 0, &refs[0], 1, nullptr, eye, route, vpath, ap, rwBoth,
                      labels, 4, n);
    buildWorldOverlay(t, nullptr, nullptr, 0, &refs[1], 1, nullptr, eye, route, vpath, ap, rwOne,
                      labels, 4, n);
    CHECK(rwBoth.size() == rwOne.size());
    CHECK(rwBoth.size() > 0);
    CHECK(rwBoth.size() % 6 == 0);

    FlightPlan plan = linePlan(8, 3);
    std::vector<Airport> fields;
    std::vector<OverlayAirportRef> fieldRefs;
    for (int i = 0; i < 12; ++i) {
        fields.push_back(fieldAt("A", 45.1 + 0.02 * i, -122.3));
        char id[8];
        std::snprintf(id, sizeof id, "K%02d", i);
        fields.back().ident = id;
    }
    for (int i = 0; i < 12; ++i)
        fieldRefs.push_back(OverlayAirportRef{&fields[static_cast<size_t>(i)], 3.0 + i});
    t = {};
    t.route = t.vpath = t.airports = t.runways = true;
    route.clear();
    vpath.clear();
    ap.clear();
    std::vector<float> rw;
    const int batches = buildWorldOverlay(t, &plan, fieldRefs.data(), 12, fieldRefs.data(), 12,
                                          nullptr, eye, route, vpath, ap, rw, labels, 4, n);
    CHECK(batches == 4);
    CHECK(batches <= 4);
    int magenta = 0;
    for (std::size_t i = 0; i + 5 < route.size(); i += 6) {
        if (route[i + 3] > 0.9f && route[i + 4] < 0.4f && route[i + 5] > 0.9f) ++magenta;
    }
    CHECK(magenta > 0);

    t = {};
    std::vector<float> empty;
    CHECK(buildWorldOverlay(t, &plan, fieldRefs.data(), 12, fieldRefs.data(), 12, nullptr, eye,
                            empty, empty, empty, empty, nullptr, 0, n) == 0);
}

TEST_CASE("overlay: building an 8-leg plan and 12 airports stays under 0.5 ms") {
    const OverlayEye eye = eyeAt(45.2, -122.2, 1800.0, 0.2f);
    FlightPlan plan = linePlan(8, 2);
    plan.approachStartLeg = 6;
    plan.legs[6].alt.kind = AltConstraintKind::At;
    plan.legs[6].alt.altFt = 2500;
    std::vector<Airport> fields;
    fields.reserve(12);
    for (int i = 0; i < 12; ++i) fields.push_back(fieldAt("K", 45.0 + 0.03 * i, -122.1 - 0.01 * i));
    std::vector<OverlayAirportRef> refs;
    for (int i = 0; i < 12; ++i)
        refs.push_back(OverlayAirportRef{&fields[static_cast<size_t>(i)], 2.0 + i});
    OverlayToggles t;
    t.route = t.vpath = t.airports = t.runways = true;
    std::vector<float> route, vpath, ap, rw;
    OverlayLabel labels[48];
    int n = 0;
    for (int i = 0; i < 3; ++i)
        buildWorldOverlay(t, &plan, refs.data(), 12, refs.data(), 12, nullptr, eye, route, vpath,
                          ap, rw, labels, 48, n);
    constexpr int kFrames = 30;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kFrames; ++i) {
        route.clear();
        vpath.clear();
        ap.clear();
        rw.clear();
        buildWorldOverlay(t, &plan, refs.data(), 12, refs.data(), 12, nullptr, eye, route, vpath,
                          ap, rw, labels, 48, n);
    }
    const auto t1 = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count() / kFrames;
    MESSAGE("overlay build ", ms, " ms, draw batches <= 4");
    CHECK(ms < 0.5);
    CHECK(n > 0);
}
