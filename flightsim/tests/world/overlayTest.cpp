// World overlays: presets, label placement, airport culling, marker altitude, runway width,
// batch count and the CPU cost of one build.
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "core/Geo.h"
#include "core/Units.h"
#include "nav/FlyBy.h"
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

Leg fixLeg(const char* ident, double lat, double lon,
           AltConstraintKind kind = AltConstraintKind::None, float altFt = 0,
           WaypointKind wk = WaypointKind::Fix) {
    Leg leg;
    leg.to.ident = ident;
    leg.to.kind = wk;
    leg.to.latDeg = lat;
    leg.to.lonDeg = lon;
    leg.alt.kind = kind;
    leg.alt.altFt = altFt;
    return leg;
}

std::vector<float> profileOf(const FlightPlan& p, float groundFt) {
    std::vector<float> ground(p.legs.size(), groundFt), alt;
    overlayRouteProfileFt(p, ground, alt);
    return alt;
}

double legNm(const FlightPlan& p, int a, int b) {
    const Waypoint& wa = p.legs[static_cast<size_t>(a)].to;
    const Waypoint& wb = p.legs[static_cast<size_t>(b)].to;
    return mToNm(greatCircleDistM(wa.latDeg, wa.lonDeg, wb.latDeg, wb.lonDeg));
}

std::vector<float> routeVerts(const FlightPlan& p, const OverlayEye& eye) {
    OverlayToggles t;
    t.route = true;
    std::vector<float> route, vpath, ap, rw;
    int n = 0;
    buildWorldOverlay(t, &p, nullptr, 0, nullptr, 0, nullptr, eye, route, vpath, ap, rw, nullptr, 0,
                      n);
    return route;
}

// Highest route vertex (m MSL) within `radiusM` of a point, -1 when none is that close.
double maxAltNearM(const std::vector<float>& verts, const OverlayEye& eye, double lat, double lon,
                   double radiusM) {
    double best = -1.0;
    for (std::size_t i = 0; i + 5 < verts.size(); i += 6) {
        const Ecef e{eye.ecef.x + verts[i], eye.ecef.y + verts[i + 1], eye.ecef.z + verts[i + 2]};
        const GeoPos g = ecefToGeodetic(e);
        if (greatCircleDistM(g.latDeg, g.lonDeg, lat, lon) <= radiusM)
            best = std::max(best, g.altMslM);
    }
    return best;
}

}  // namespace

TEST_CASE(
    "overlay: presets cycle AUTO, ROUTE+PATH, AIRPORTS, ALL, OFF; a custom mix goes to AUTO") {
    OverlayToggles t;  // the defaults: route and path auto, airports and runways off
    CHECK(overlayMatchingPreset(t) == 0);
    CHECK(std::string(overlayPresetName(0)) == "AUTO");
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "ROUTE+PATH");
    CHECK(t.routeMode == OverlayMode::On);
    CHECK(t.vpathMode == OverlayMode::On);
    CHECK_FALSE(t.airports);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "AIRPORTS");
    CHECK(t.routeMode == OverlayMode::Auto);
    CHECK(t.vpathMode == OverlayMode::Auto);
    CHECK(t.airports);
    CHECK(t.runways);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "ALL");
    CHECK(t.routeMode == OverlayMode::On);
    CHECK(t.vpathMode == OverlayMode::On);
    CHECK(t.airports);
    CHECK(t.runways);
    CHECK(std::string(overlayPresetName(overlayCycle(t))) == "OFF");
    CHECK(t.routeMode == OverlayMode::Off);
    CHECK(t.vpathMode == OverlayMode::Off);
    CHECK_FALSE(t.airports);
    CHECK(overlayCycle(t) == 0);
    CHECK(t.routeMode == OverlayMode::Auto);

    t = {};
    t.vpathMode = OverlayMode::On;  // route auto, path on: no preset
    CHECK(overlayMatchingPreset(t) == -1);
    CHECK(std::string(overlayPresetName(-1)) == "CUSTOM");
    CHECK(overlayCycle(t) == 0);
    CHECK(t.vpathMode == OverlayMode::Auto);
}

TEST_CASE("overlay: route and path follow the declutter level in AUTO, ON and OFF force them") {
    for (const OverlayMode route : {OverlayMode::Auto, OverlayMode::On, OverlayMode::Off}) {
        for (const bool full : {true, false}) {  // FULL, or STD / MIN
            OverlayToggles t;
            t.routeMode = route;
            t.vpathMode = OverlayMode::Off;
            t.airports = true;
            const OverlayToggles r = resolveOverlay(t, full);
            INFO("mode " << overlayModeName(route) << (full ? " at FULL" : " at STD/MIN"));
            const bool expected = route == OverlayMode::On || (route == OverlayMode::Auto && full);
            CHECK(r.route == expected);
            CHECK_FALSE(r.vpath);
            CHECK(r.airports);  // airports and runways do not depend on the declutter level
            CHECK(r.routeMode == route);
        }
    }
    OverlayToggles t;  // both auto
    CHECK(resolveOverlay(t, true).route);
    CHECK(resolveOverlay(t, true).vpath);
    CHECK_FALSE(resolveOverlay(t, false).route);
    CHECK_FALSE(resolveOverlay(t, false).vpath);
    CHECK(std::string(overlayModeName(OverlayMode::Auto)) == "AUTO");
    CHECK(std::string(overlayModeName(OverlayMode::On)) == "ON");
    CHECK(std::string(overlayModeName(OverlayMode::Off)) == "OFF");
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

TEST_CASE("overlay: runway outlines clear below 250 ft above the field") {
    // Field elevation 30 ft; the eye is on a 2 NM final to runway 10 (heading north).
    Airport field = fieldAt("KPDX", 45.05, -122.2);
    OverlayAirportRef refs[] = {{&field, 2.0}};
    OverlayToggles t;
    t.runways = true;
    auto outlineVerts = [&](double heightAboveFieldFt) {
        const OverlayEye eye =
            eyeAt(45.05 - 2.0 / 60.0, -122.2, ftToM(30.0 + heightAboveFieldFt), 0.0f);
        std::vector<float> route, vpath, ap, rw;
        int n = 0;
        buildWorldOverlay(t, nullptr, nullptr, 0, refs, 1, nullptr, eye, route, vpath, ap, rw,
                          nullptr, 0, n);
        return rw.size();
    };
    CHECK(outlineVerts(1500.0) > 0);
    CHECK(outlineVerts(260.0) > 0);
    CHECK(outlineVerts(240.0) == 0);
    CHECK(outlineVerts(8.0) == 0);  // on the runway
    // Height above the field, not distance: far out but low still hides them.
    CHECK(overlayHiddenNearField(ftToM(30.0 + 249.0), 30.0f));
    CHECK_FALSE(overlayHiddenNearField(ftToM(30.0 + 251.0), 30.0f));
}

TEST_CASE("overlay: the approach path clears below 250 ft, an enroute path does not") {
    // FAF at 1500 ft, then the runway fix (ground 0 ft without terrain). Eye on that last leg.
    FlightPlan plan;
    for (int i = 0; i < 3; ++i) {
        Leg leg;
        leg.to.ident = i == 2 ? "RW10" : (i == 1 ? "FAF" : "IAF");
        leg.to.latDeg = 45.0 + 0.05 * i;
        leg.to.lonDeg = -122.0;
        if (i == 1) {
            leg.alt.kind = AltConstraintKind::At;
            leg.alt.altFt = 1500;
        }
        plan.legs.push_back(leg);
    }
    plan.activeLeg = 2;
    plan.approachStartLeg = 1;
    OverlayToggles t;
    t.vpath = true;
    auto pathVerts = [&](const FlightPlan& p, double altFt) {
        const OverlayEye eye = eyeAt(45.07, -122.0, ftToM(altFt), 0.0f);
        std::vector<float> route, vpath, ap, rw;
        int n = 0;
        buildWorldOverlay(t, &p, nullptr, 0, nullptr, 0, nullptr, eye, route, vpath, ap, rw,
                          nullptr, 0, n);
        return vpath.size();
    };
    CHECK(pathVerts(plan, 1000.0) > 0);
    CHECK(pathVerts(plan, 200.0) == 0);
    FlightPlan enroute = plan;
    enroute.approachStartLeg = -1;
    CHECK(pathVerts(enroute, 200.0) > 0);
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

TEST_CASE("overlay: the route descends to the runway instead of dropping from cruise") {
    // The owner's report: the plan's last legs sat at cruise and the line fell straight down onto
    // the runway. Departure runway, a free fix 3 NM out, cruise, a free fix 6 NM before the
    // destination runway, the runway; ground 30 ft everywhere.
    FlightPlan p;
    p.cruiseAltFt = 9000;
    p.legs.push_back(
        fixLeg("RW05", 45.0, -122.0, AltConstraintKind::None, 0, WaypointKind::Runway));
    p.legs.push_back(fixLeg("CLIMB", 45.05, -122.0));
    p.legs.push_back(fixLeg("ENR", 45.5, -122.0, AltConstraintKind::At, 9000));
    p.legs.push_back(fixLeg("DESC", 45.9, -122.0));
    p.legs.push_back(
        fixLeg("RW10L", 46.0, -122.0, AltConstraintKind::None, 0, WaypointKind::Runway));
    p.activeLeg = 4;
    REQUIRE(overlayLandingLeg(p) == 4);
    const std::vector<float> alt = profileOf(p, 30.0f);
    REQUIRE(alt.size() == 5);
    const double grad = std::tan(degToRad(3.0)) * 6076.12;  // ft per NM
    CHECK(alt[0] == doctest::Approx(30.0f));                // on the departure runway
    CHECK(alt[1] == doctest::Approx(30.0 + grad * legNm(p, 0, 1)).epsilon(0.01));  // 3 deg climb
    CHECK(alt[2] == doctest::Approx(9000.0f));
    CHECK(alt[3] == doctest::Approx(80.0 + grad * legNm(p, 3, 4)).epsilon(0.01));  // 3 deg descent
    CHECK(alt[4] == doctest::Approx(80.0f));  // 50 ft over the threshold
    // The old per-fix marker put both of the last fixes at cruise.
    CHECK(overlayMarkerAltFt(p.legs[3].alt, p.cruiseAltFt, 30.0f) == doctest::Approx(9000.0f));
    CHECK(overlayMarkerAltFt(p.legs[4].alt, p.cruiseAltFt, 30.0f) == doctest::Approx(9000.0f));
    // No leg of the final descent is steeper than the 3 degree path.
    const double lastDeg = radToDeg(std::atan((alt[3] - alt[4]) / (legNm(p, 3, 4) * 6076.12)));
    CHECK(lastDeg == doctest::Approx(3.0).epsilon(0.02));
}

TEST_CASE("overlay: approach altitudes are flown as charted, missed legs climb from the runway") {
    // ILS 10L at KPDX: IF at or above 3000, TRAYL 2300, FAF 2000, RW10L at 79; missed: a CA leg
    // without a fix, then BTG at or above 4200.
    FlightPlan p;
    p.cruiseAltFt = 9000;
    p.legs.push_back(fixLeg("UBG", 45.3532, -122.9780, AltConstraintKind::At, 9000));
    p.legs.push_back(fixLeg("BUXOM", 45.6976, -122.8607, AltConstraintKind::AtOrAbove, 3000));
    p.legs.push_back(fixLeg("TRAYL", 45.6677, -122.7833, AltConstraintKind::AtOrAbove, 2300));
    p.legs.push_back(fixLeg("BLAZR", 45.6435, -122.7209, AltConstraintKind::AtOrAbove, 2000));
    p.legs.push_back(
        fixLeg("RW10L", 45.5948, -122.5956, AltConstraintKind::At, 79, WaypointKind::Runway));
    Leg ca = fixLeg("", 0.0, 0.0, AltConstraintKind::AtOrAbove, 800);
    ca.type = LegType::CA;
    ca.missedApproach = true;
    p.legs.push_back(ca);
    Leg btg = fixLeg("BTG", 45.7478, -122.5915, AltConstraintKind::AtOrAbove, 4200);
    btg.missedApproach = true;
    p.legs.push_back(btg);
    Leg free = fixLeg("FREE", 45.70, -122.40);  // an unconstrained missed leg
    free.missedApproach = true;
    p.legs.push_back(free);
    p.approachStartLeg = 1;
    p.activeLeg = 4;
    REQUIRE(overlayLandingLeg(p) == 4);
    const std::vector<float> alt = profileOf(p, 30.0f);
    CHECK(alt[0] == doctest::Approx(9000.0f));
    CHECK(alt[1] == doctest::Approx(3000.0f));
    CHECK(alt[2] == doctest::Approx(2300.0f));
    CHECK(alt[3] == doctest::Approx(2000.0f));
    CHECK(alt[4] == doctest::Approx(79.0f));
    CHECK(alt[6] == doctest::Approx(4200.0f));
    // The free missed leg climbs at 3 degrees from the runway, under its 4200 ft neighbour's cone.
    const double grad = std::tan(degToRad(3.0)) * 6076.12;
    const float viaRunway = static_cast<float>(79.0 + grad * (legNm(p, 4, 6) + legNm(p, 6, 7)));
    const float viaBtg = static_cast<float>(4200.0 + grad * legNm(p, 6, 7));
    CHECK(alt[7] == doctest::Approx(std::min({9000.0f, viaRunway, viaBtg})).epsilon(0.01));
    // A plan that ends at a fix has no landing leg: its last fix keeps the cruise.
    FlightPlan enroute;
    enroute.cruiseAltFt = 7000;
    enroute.legs.push_back(fixLeg("A", 45.0, -122.0));
    enroute.legs.push_back(fixLeg("B", 45.5, -122.0));
    CHECK(overlayLandingLeg(enroute) == -1);
    CHECK(profileOf(enroute, 100.0f)[1] == doctest::Approx(7000.0f));
}

TEST_CASE("overlay: constraints outside the approach clamp the profile") {
    FlightPlan p;
    p.cruiseAltFt = 9000;
    p.legs.push_back(
        fixLeg("DEP", 44.0, -122.0, AltConstraintKind::None, 0, WaypointKind::Airport));
    p.legs.push_back(fixLeg("CAP", 44.8, -122.0, AltConstraintKind::AtOrBelow, 12000));
    p.legs.push_back(fixLeg("FLOOR", 45.2, -122.0, AltConstraintKind::AtOrAbove, 11000));
    p.legs.push_back(fixLeg("WIN", 45.6, -122.0, AltConstraintKind::Between, 5000));
    p.legs[3].alt.alt2Ft = 7000;
    p.legs.push_back(
        fixLeg("DEST", 46.0, -122.0, AltConstraintKind::None, 0, WaypointKind::Airport));
    const std::vector<float> alt = profileOf(p, 100.0f);
    CHECK(alt[0] == doctest::Approx(100.0f));
    // An at-or-below never lifts the line above the plan; an at-or-above floor above cruise does.
    CHECK(alt[1] <= 12000.0f);
    CHECK(alt[1] > 9000.0f);
    CHECK(alt[2] == doctest::Approx(11000.0f));
    // 24 NM before the airport the 3 degree descent is above 7000: the window caps it.
    CHECK(alt[3] == doctest::Approx(7000.0f));
    CHECK(alt[4] == doctest::Approx(150.0f));
}

TEST_CASE("overlay: the route clears below 250 ft on final, and the landing fix has no pole") {
    // A fix, then the runway 3 NM north; the eye flies the landing leg toward it.
    FlightPlan p;
    p.cruiseAltFt = 3000;
    p.legs.push_back(fixLeg("WPT", 45.0, -122.0));
    p.legs.push_back(
        fixLeg("RW36", 45.05, -122.0, AltConstraintKind::None, 0, WaypointKind::Runway));
    p.activeLeg = 1;
    auto onFinal = [&](const FlightPlan& plan, double heightFt) {
        return routeVerts(plan, eyeAt(45.03, -122.003, ftToM(heightFt), 0.0f)).size();
    };
    CHECK(onFinal(p, 1000.0) > 0);
    CHECK(onFinal(p, 260.0) > 0);
    CHECK(onFinal(p, 240.0) == 0);
    // A plan that ends at a fix is not a landing: the route stays.
    FlightPlan toFix = p;
    toFix.legs[1].to.kind = WaypointKind::Fix;
    CHECK(onFinal(toFix, 240.0) > 0);

    // From the side: the fix gets its pole, the runway does not (the old pole stood 80 m tall on
    // the threshold; the line ends 50 ft over it).
    FlightPlan three = p;
    three.legs.push_back(fixLeg("BEYOND", 45.10, -122.0));
    three.legs[1].to.kind = WaypointKind::Fix;
    three.legs[2].to.kind = WaypointKind::Runway;  // BEYOND is the landing fix now
    three.activeLeg = 2;
    const OverlayEye side = eyeAt(45.05, -122.08, ftToM(3000.0), static_cast<float>(kPi * 0.5));
    const std::vector<float> v = routeVerts(three, side);
    REQUIRE_FALSE(v.empty());
    const double fixTop = maxAltNearM(v, side, 45.05, -122.0, 60.0);
    const double rwyTop = maxAltNearM(v, side, 45.10, -122.0, 60.0);
    CHECK(fixTop > 100.0);  // the pole: ground + 25 m to at least 80 m above that
    CHECK(rwyTop >= 0.0);   // the line still reaches the threshold
    CHECK(rwyTop < 70.0);   // 50 ft + the line's own half height, no pole
}

TEST_CASE("overlay: missed approach legs show only once the missed approach is active") {
    FlightPlan p;
    p.cruiseAltFt = 5000;
    p.legs.push_back(fixLeg("IAF", 45.00, -122.0, AltConstraintKind::At, 3000));
    p.legs.push_back(fixLeg("FAF", 45.08, -122.0, AltConstraintKind::At, 1600));
    p.legs.push_back(
        fixLeg("RW36", 45.16, -122.0, AltConstraintKind::At, 50, WaypointKind::Runway));
    Leg ca = fixLeg("CA", 45.24, -122.0, AltConstraintKind::AtOrAbove, 1500);
    ca.missedApproach = true;
    p.legs.push_back(ca);
    Leg back = fixLeg("IF", 45.00, -122.05, AltConstraintKind::At, 3000);
    back.missedApproach = true;
    p.legs.push_back(back);
    p.approachStartLeg = 0;
    FlightPlan noMissed = p;
    noMissed.legs.resize(3);
    const OverlayEye eye = eyeAt(45.10, -122.0, ftToM(1300.0), 0.0f);
    p.activeLeg = noMissed.activeLeg = 2;
    CHECK(routeVerts(p, eye).size() == routeVerts(noMissed, eye).size());
    p.activeLeg = 3;  // went around: the navigator is on the CA leg
    const OverlayEye climbing = eyeAt(45.18, -122.0, ftToM(900.0), 0.0f);
    CHECK(routeVerts(p, climbing).size() > routeVerts(noMissed, climbing).size());
}

TEST_CASE("overlay: N10 fly-by turns are drawn as arcs that cut the corner") {
    // East 5 NM, then north 5 NM: a 90 degree left turn at B, seen from above and south of it.
    FlightPlan plan;
    plan.legs = {fixLeg("A", 45.0, -122.0), fixLeg("B", 45.0, -121.8823),
                 fixLeg("C", 45.0833, -121.8823)};
    plan.legs[0].type = LegType::IF;
    plan.activeLeg = 1;
    const OverlayEye eye = eyeAt(44.95, -121.95, 3000.0, 30.0f);
    OverlayToggles t;
    t.route = true;
    auto build = [&](float radiusNm) {
        std::vector<float> route, vpath, ap, rw;
        int n = 0;
        buildWorldOverlay(t, &plan, nullptr, 0, nullptr, 0, nullptr, eye, route, vpath, ap, rw,
                          nullptr, 0, n, radiusNm);
        return route;
    };
    const std::vector<float> sharp = build(0.0f), arcs = build(1.0f);
    const FlyByTurn turn = planTurnAt(plan, 1, 1.0);
    REQUIRE(turn.valid);
    LatLon pts[16];
    const int np = arcPoints(turn, pts, 16);
    const LatLon mid = pts[np / 2];
    CHECK(arcs.size() > sharp.size());  // the arc's pieces
    // The straight leg now ends at T1, 1 NM before B, where the arc begins (vertices sit at the
    // segment ends: the sharp ribbon has none there).
    CHECK(maxAltNearM(arcs, eye, turn.start.latDeg, turn.start.lonDeg, 60.0) >= 0.0);
    CHECK(maxAltNearM(sharp, eye, turn.start.latDeg, turn.start.lonDeg, 60.0) < 0.0);
    CHECK(maxAltNearM(arcs, eye, mid.latDeg, mid.lonDeg, 150.0) >= 0.0);  // along the arc
    CHECK(maxAltNearM(sharp, eye, mid.latDeg, mid.lonDeg, 150.0) < 0.0);
}
