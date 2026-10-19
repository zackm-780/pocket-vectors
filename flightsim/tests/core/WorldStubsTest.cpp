// FlatTerrain, SyntheticNavDb (airport XTST) and Paths.
#include <doctest/doctest.h>

#include <cstdlib>

#include "core/FlatTerrain.h"
#include "core/Geo.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"

using namespace fsim;

namespace {

// A ~1 km square flatten area around (40, -100), CCW (east, north).
FlattenArea squareArea(float elevM, float blendM) {
    FlattenArea a;
    const double d = 0.005;  // deg, ~500 m
    const double lat[4] = {40 - d, 40 - d, 40 + d, 40 + d};
    const double lon[4] = {-100 - d, -100 + d, -100 + d, -100 - d};
    for (int i = 0; i < 4; ++i) {
        a.latDeg[i] = lat[i];
        a.lonDeg[i] = lon[i];
    }
    a.elevM = elevM;
    a.blendM = blendM;
    return a;
}

}  // namespace

TEST_CASE("terrain: flat terrain returns the constant elevation and is always ready") {
    FlatTerrain t(609.6f);
    CHECK(t.elevationM(40, -100) == doctest::Approx(609.6f));
    CHECK(t.elevationM(-10, 20) == doctest::Approx(609.6f));
    CHECK(t.isDetailReady(0, 0));
    CHECK(t.landClass(1, 1) == LandClass::Grass);
    t.setElevationM(0);
    CHECK(t.elevationM(5, 5) == doctest::Approx(0.0f));
}

TEST_CASE("terrain: flatten area forces elevation inside and blends outside") {
    FlatTerrain t(100.0f);
    t.addFlattenArea(squareArea(160.0f, 60.0f));
    CHECK(t.elevationM(40, -100) == doctest::Approx(160.0f));          // center
    CHECK(t.elevationM(40.004, -100.004) == doctest::Approx(160.0f));  // still inside

    // ~30 m outside the north edge (0.005 deg = 556 m): half way through the 60 m blend.
    const double outsideLat = 40.005 + 30.0 / (kEarthRadiusM * kRadPerDeg);
    const float mid = t.elevationM(outsideLat, -100.0);
    CHECK(mid > 100.0f);
    CHECK(mid < 160.0f);
    CHECK(mid == doctest::Approx(130.0f).epsilon(0.03));  // smoothstep(0.5) = 0.5

    CHECK(t.elevationM(40.02, -100) == doctest::Approx(100.0f));  // far outside
    t.clearFlattenAreas();
    CHECK(t.elevationM(40, -100) == doctest::Approx(100.0f));
}

TEST_CASE("nav: synthetic database knows only XTST") {
    SyntheticNavDb db;
    const Airport* a = db.findAirport("XTST");
    REQUIRE(a != nullptr);
    CHECK(db.findAirport("xtst") == a);
    CHECK(db.findAirport("KSFO") == nullptr);
    CHECK(a->latDeg == doctest::Approx(40.0));
    CHECK(a->lonDeg == doctest::Approx(-100.0));
    CHECK(a->elevFt == doctest::Approx(2000.0f));
    REQUIRE(a->runways.size() == 2);

    const Runway* r27 = findRunway(*a, "27");
    const Runway* r09 = findRunway(*a, "09");
    REQUIRE(r27 != nullptr);
    REQUIRE(r09 != nullptr);
    CHECK(findRunway(*a, "18") == nullptr);
    CHECK(r27->headingTrueDeg == doctest::Approx(270.0f));
    CHECK(r09->headingTrueDeg == doctest::Approx(90.0f));
    CHECK(r27->lengthFt == doctest::Approx(10000.0f));
    CHECK(r27->widthFt == doctest::Approx(150.0f));
    CHECK(r27->oppositeIdent == "09");

    // The 27 threshold is the east end; thresholds are one runway length apart.
    CHECK(r27->thrLonDeg > r09->thrLonDeg);
    CHECK(greatCircleDistM(r27->thrLatDeg, r27->thrLonDeg, r09->thrLatDeg, r09->thrLonDeg) ==
          doctest::Approx(ftToM(10000.0)).epsilon(1e-6));
    // The airport reference point is the runway midpoint.
    CHECK(greatCircleDistM(a->latDeg, a->lonDeg, r27->thrLatDeg, r27->thrLonDeg) ==
          doctest::Approx(ftToM(5000.0)).epsilon(1e-6));

    // Both ends have a synthesized ILS with a 3 degree glidepath along the runway heading.
    for (const Runway* r : {r27, r09}) {
        REQUIRE(r->ils.has_value());
        CHECK(r->ils->synthesized);
        CHECK(r->ils->hasGs);
        CHECK(r->ils->gsAngleDeg == doctest::Approx(3.0f));
        CHECK(r->ils->locCourseTrueDeg == doctest::Approx(r->headingTrueDeg));
    }
    CHECK(db.magVarDeg(40, -100) == doctest::Approx(0.0f));
}

TEST_CASE("nav: synthetic database queries") {
    SyntheticNavDb db;
    CHECK(db.nearestAirports(40.1, -100.1, 5, 50).size() == 1);
    CHECK(db.nearestAirports(45, -100, 5, 50).empty());
    CHECK(db.searchAirports("xt", 3).size() == 1);
    CHECK(db.searchAirports("zzz", 3).empty());
    CHECK(db.airportsInBox(39, -101, 41, -99).size() == 1);
    CHECK(db.airportsInBox(10, 10, 11, 11).empty());
    CHECK(db.findNavaids("SFO").empty());
    CHECK(db.findFixes("ABCDE").empty());
}

TEST_CASE("paths: join, overrides and input resolution") {
    CHECK(Paths::join("/a/b", "c") == "/a/b/c");
    CHECK(Paths::join("/a/b/", "c") == "/a/b/c");
    CHECK(Paths::join("", "c") == "c");

    PathOverrides o;
    o.assetDir = "/opt/fsim";
    o.dataDir = "/mnt/data";
    o.confDir = "/tmp/fsim-conf";
    const Paths p = Paths::resolve(o);
    CHECK(p.assetRoot() == "/opt/fsim");
    CHECK(p.asset("jsbsim") == "/opt/fsim/jsbsim");
    CHECK(p.dataRoot() == "/mnt/data");
    CHECK(p.conf("log.txt") == "/tmp/fsim-conf/log.txt");

    PathOverrides defaults;
    defaults.assetDir = "/opt/fsim";
    defaults.confDir = "/c";
    CHECK(Paths::resolve(defaults).dataRoot() == "/opt/fsim/data");  // default data root
    CHECK(Paths::resolve({}).assetRoot() == executableDir());
    CHECK(Paths::resolve(o).resolveInputFile("/definitely/not/here.ini") ==
          "/definitely/not/here.ini");
    CHECK(fileExists(executableDir()));
}
