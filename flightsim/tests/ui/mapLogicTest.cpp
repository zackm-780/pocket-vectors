// MapPage logic: projection (north-up / track-up / zoom), declutter by zoom, leg styling,
// waypoint info values, formatting, action handling, and a recording-only draw smoke test.
#include <doctest/doctest.h>

#include <cmath>
#include <cstring>

#include "core/Geo.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "ui/MapPage.h"
#include "ui/MapPageLogic.h"

using namespace fsim;

namespace {
FlightPlan makePlan() {
    FlightPlan p;
    p.activeLeg = 1;
    const double lats[] = {45.0, 45.5, 46.0, 46.0};
    const double lons[] = {-122.0, -122.0, -122.0, -121.0};
    const char* ids[] = {"AAAA", "BBBB", "CCCC", "DDDD"};
    for (int i = 0; i < 4; ++i) {
        Leg l;
        l.to.ident = ids[i];
        l.to.latDeg = lats[i];
        l.to.lonDeg = lons[i];
        p.legs.push_back(l);
    }
    p.legs[2].alt = {AltConstraintKind::AtOrAbove, 3000, 0};
    return p;
}
}  // namespace

TEST_CASE("map: zoom steps") {
    CHECK(mapZoomIndexForNm(10) == 2);
    CHECK(mapZoomIndexForNm(1) == 0);
    CHECK(mapZoomIndexForNm(1000) == kMapZoomCount - 1);
    CHECK(mapZoomStep(0, -1) == 0);
    CHECK(mapZoomStep(kMapZoomCount - 1, 1) == kMapZoomCount - 1);
    CHECK(mapZoomStep(2, 1) == 3);
}

TEST_CASE("map: north-up projection and zoom scale") {
    const MapProjection p = makeMapProjection(45, -122, 1.0f, MapOrientation::NorthUp, 10, 640);
    float x, y;
    p.toScreen(45, -122, x, y);
    CHECK(x == doctest::Approx(320));
    CHECK(y == doctest::Approx(240));
    // 10 NM north = outer ring radius straight up.
    p.toScreen(45 + 10.0 / 60.0, -122, x, y);
    CHECK(x == doctest::Approx(320).epsilon(0.001));
    CHECK(y == doctest::Approx(240 - p.ringRadiusPx).epsilon(0.001));
    // East is right.
    p.toScreen(45, -122 + 5.0 / (60.0 * std::cos(degToRad(45))), x, y);
    CHECK(x == doctest::Approx(320 + p.ringRadiusPx * 0.5f).epsilon(0.002));
    // Doubling the zoom halves the scale.
    const MapProjection p2 = makeMapProjection(45, -122, 0, MapOrientation::NorthUp, 20, 640);
    CHECK(p2.pxPerNm() == doctest::Approx(p.pxPerNm() * 0.5f));
}

TEST_CASE("map: track-up puts the track vector up and the aircraft low") {
    const float track = static_cast<float>(degToRad(90));  // flying east
    const MapProjection p = makeMapProjection(45, -122, track, MapOrientation::TrackUp, 10, 640);
    CHECK(p.cy > 240);
    float x, y;
    p.toScreen(45, -122 + 5.0 / (60.0 * std::cos(degToRad(45))), x, y);  // 5 NM east
    CHECK(x == doctest::Approx(p.cx).epsilon(0.002));
    CHECK(y == doctest::Approx(p.cy - p.ringRadiusPx * 0.5f).epsilon(0.002));
    p.toScreen(45 + 5.0 / 60.0, -122, x, y);  // 5 NM north is to the LEFT when flying east
    CHECK(x < p.cx);
    CHECK(x == doctest::Approx(p.cx - p.ringRadiusPx * 0.5f).epsilon(0.002));
    // Inverse.
    double lat, lon;
    p.toLatLon(x, y, lat, lon);
    CHECK(lat == doctest::Approx(45 + 5.0 / 60.0).epsilon(1e-6));
    CHECK(lon == doctest::Approx(-122).epsilon(1e-6));
}

TEST_CASE("map: declutter rules by zoom") {
    CHECK(mapShowAirport(AirportType::Small, 10));
    CHECK_FALSE(mapShowAirport(AirportType::Small, 40));
    CHECK(mapShowAirport(AirportType::Medium, 40));
    CHECK_FALSE(mapShowAirport(AirportType::Medium, 160));
    CHECK(mapShowAirport(AirportType::Large, 160));
    CHECK(mapShowAirportLabel(AirportType::Large, 160));
    CHECK_FALSE(mapShowAirportLabel(AirportType::Medium, 80));
    CHECK(mapShowRunways(AirportType::Large, 40));
    CHECK_FALSE(mapShowRunways(AirportType::Large, 80));
    CHECK(mapShowRunways(AirportType::Small, 5));
    CHECK_FALSE(mapShowRunways(AirportType::Small, 10));
    CHECK(mapShowNavaid(NavaidType::Vor, 80));
    CHECK_FALSE(mapShowNavaid(NavaidType::Vor, 160));
    CHECK(mapShowNavaid(NavaidType::Ndb, 20));
    CHECK_FALSE(mapShowNavaid(NavaidType::Ndb, 40));
    CHECK(mapShowNavaidLabel(NavaidType::Vor, 40));
    CHECK_FALSE(mapShowNavaidLabel(NavaidType::Vor, 80));
    CHECK(mapShowConstraints(40));
    CHECK_FALSE(mapShowConstraints(80));
    CHECK(mapDeclutterCaps(160).maxAirports < mapDeclutterCaps(10).maxAirports);
    CHECK(mapAirportRank(AirportType::Large) < mapAirportRank(AirportType::Small));
}

TEST_CASE("map: leg styling by state") {
    // active = 3, approach starts at 4, leg 6 is missed.
    CHECK(mapLegStyle(1, 3, 4, false).kind == MapLegKind::Past);
    CHECK(mapLegStyle(3, 3, 4, false).kind == MapLegKind::Active);
    CHECK(mapLegStyle(3, 3, 4, false).widthPx > mapLegStyle(1, 3, 4, false).widthPx);
    CHECK(mapLegStyle(4, 3, 4, false).kind == MapLegKind::Approach);
    CHECK(mapLegStyle(2, 1, -1, false).kind == MapLegKind::Future);
    const MapLegStyle m = mapLegStyle(6, 3, 4, true);
    CHECK(m.kind == MapLegKind::Missed);
    CHECK(m.dashed);
    CHECK_FALSE(mapLegStyle(4, 3, 4, false).dashed);
    CHECK(mapLegStyle(6, 6, 4, true).kind == MapLegKind::Active);
    CHECK(mapLegStyle(2, -1, -1, false).kind == MapLegKind::Future);
}

TEST_CASE("map: waypoint info distance, bearing, ETE, DTK") {
    const FlightPlan plan = makePlan();
    // Aircraft at the origin of leg 1 heading north; leg 1 ends 30 NM north.
    const MapWptInfo w = mapWaypointInfo(plan, 1, 45.0, -122.0, 120, 0, 0);
    REQUIRE(w.valid);
    CHECK(std::strcmp(w.ident, "BBBB") == 0);
    CHECK(w.distNm == doctest::Approx(30).epsilon(0.01));
    CHECK(w.bearingTrueDeg == doctest::Approx(0).epsilon(0.01));
    CHECK(w.eteS == doctest::Approx(30.0 / 120.0 * 3600).epsilon(0.01));
    REQUIRE(w.hasDtk);
    CHECK(w.dtkMagDeg == doctest::Approx(0).epsilon(0.01));
    // 10 degrees east variation: magnetic = true - 10.
    const MapWptInfo wm = mapWaypointInfo(plan, 1, 45.0, -122.0, 120, 10, 0);
    CHECK(wm.bearingMagDeg == doctest::Approx(350).epsilon(0.001));
    // Leg 3 DTK is roughly east.
    const MapWptInfo e = mapWaypointInfo(plan, 3, 46.0, -122.0, 100, 0, 0);
    CHECK(e.dtkMagDeg == doctest::Approx(90).epsilon(0.01));
    // Zero speed: no ETE.
    CHECK(mapWaypointInfo(plan, 1, 45.0, -122.0, 0, 0, 0).eteS == 0);
    CHECK_FALSE(mapWaypointInfo(plan, 9, 45, -122, 100, 0, 0).valid);
    // Rows accumulate along the route.
    MapWptInfo rows[8];
    const int n = mapWaypointRows(plan, 45.0, -122.0, 120, 0, rows, 8);
    CHECK(n == 3);
    CHECK(rows[0].distNm == doctest::Approx(30).epsilon(0.01));
    CHECK(rows[1].distNm == doctest::Approx(60).epsilon(0.01));
    CHECK(rows[2].distNm > rows[1].distNm);
    CHECK(mapWaypointRows(plan, 45.0, -122.0, 120, 0, rows, 2) == 2);
}

TEST_CASE("map: formatting") {
    char b[32];
    AltConstraint a{AltConstraintKind::AtOrAbove, 3000, 0};
    mapFormatAlt(a, b, sizeof b);
    CHECK(std::strcmp(b, "3000A") == 0);
    a.kind = AltConstraintKind::AtOrBelow;
    mapFormatAlt(a, b, sizeof b);
    CHECK(std::strcmp(b, "3000B") == 0);
    a.kind = AltConstraintKind::Between;
    a.alt2Ft = 5000;
    mapFormatAlt(a, b, sizeof b);
    CHECK(std::strcmp(b, "3000-5000") == 0);
    a.kind = AltConstraintKind::None;
    CHECK(mapFormatAlt(a, b, sizeof b) == 0);
    mapFormatEte(372, b, sizeof b);
    CHECK(std::strcmp(b, "6:12") == 0);
    mapFormatEte(3910, b, sizeof b);
    CHECK(std::strcmp(b, "1:05:10") == 0);
    mapFormatEte(0, b, sizeof b);
    CHECK(std::strcmp(b, "--:--") == 0);
}

TEST_CASE("map: page actions") {
    MapPage page;
    const FlightPlan plan = makePlan();
    ActionState a;
    CHECK(page.zoomNm() == 10);
    a.pressed[static_cast<int>(Action::MenuDown)] = true;  // zoom out
    page.update(a, &plan);
    CHECK(page.zoomNm() == 20);
    a = ActionState();
    a.pressed[static_cast<int>(Action::MenuUp)] = true;
    page.update(a, &plan);
    page.update(a, &plan);
    CHECK(page.zoomNm() == 5);
    a = ActionState();
    a.pressed[static_cast<int>(Action::HudDeclutter)] = true;
    page.update(a, &plan);
    CHECK(page.orientation() == MapOrientation::TrackUp);
    a = ActionState();
    a.pressed[static_cast<int>(Action::MenuRight)] = true;
    page.update(a, &plan);
    CHECK(page.selectedLeg() == 2);
    page.update(a, &plan);
    page.update(a, &plan);
    CHECK(page.selectedLeg() == 3);  // clamped to the last leg
    a = ActionState();
    a.pressed[static_cast<int>(Action::MenuAccept)] = true;
    MapPageResult r = page.update(a, &plan);
    CHECK(r.directToLeg == 3);
    CHECK_FALSE(r.close);
    a = ActionState();
    a.pressed[static_cast<int>(Action::MapToggle)] = true;
    CHECK(page.update(a, &plan).close);
    a = ActionState();
    a.pressed[static_cast<int>(Action::MenuBack)] = true;
    CHECK(page.update(a, nullptr).close);
}

TEST_CASE("map: draw records geometry without GL") {
    gfx::GlApi api{};
    Renderer2D r(api, gfx::GlFlavor::Desktop21);
    SyntheticNavDb db;
    AircraftState ac;
    ac.valid = true;
    ac.latDeg = 45;
    ac.lonDeg = -122;
    ac.gsKt = 100;
    ac.windKt = 10;
    ac.windFromTrueDeg = 270;
    NavState nav;
    const FlightPlan plan = makePlan();
    MapPage page;
    r.begin(640, 480);
    page.draw(r, ac, nav, &plan, db, 15);
    CHECK(r.pendingVertexCount() > 500);
    r.begin(640, 480);
    page.setOrientation(MapOrientation::TrackUp);
    page.draw(r, ac, nav, nullptr, db, 15);
    CHECK(r.pendingVertexCount() > 100);
}
