// WP F3 tests: the JSBSim ground callback over TerrainQuery, ground contact on synthetic terrain,
// the terrain and water crash rules with real TerrainService data, and spawn gating.
// "ground: ..." tests are pure (no flight model); "sim: ground ..." tests fly JSBSim.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <math/FGColumnVector3.h>
#include <math/FGLocation.h>

#include "app/Simulation.h"
#include "core/FlatTerrain.h"
#include "groundSupport.h"
#include "sim/GroundCallback.h"

using namespace fsim;
using namespace fsim::test;
using fsim::terrain::testing::HeightFn;

namespace {

constexpr double kWgs84SemiMajorFt = 20925646.32546;
constexpr double kWgs84SemiMinorFt = 20855486.5951;

JSBSim::FGLocation locationAt(double latDeg, double lonDeg, double altFt) {
    JSBSim::FGLocation l;
    l.SetEllipse(kWgs84SemiMajorFt, kWgs84SemiMinorFt);
    l.SetPositionGeodetic(degToRad(lonDeg), degToRad(latDeg), altFt);
    return l;
}

// Terrain rising toward the west at `slopeWest` and toward the north at `slopeNorth` (rise/run).
FunctionTerrain rampTerrain(double slopeWest, double slopeNorth, double baseM = kXtstElevM) {
    return FunctionTerrain([=](double lat, double lon) {
        return baseM - slopeWest * xtstEastM(lat, lon) + slopeNorth * xtstNorthM(lat);
    });
}

// Unit vector pointing up in ECEF at a geodetic position (what a flat ground's normal is).
JSBSim::FGColumnVector3 upEcef(double latDeg, double lonDeg) {
    const double la = degToRad(latDeg), lo = degToRad(lonDeg);
    return JSBSim::FGColumnVector3(std::cos(la) * std::cos(lo), std::cos(la) * std::sin(lo),
                                   std::sin(la));
}

double dot(const JSBSim::FGColumnVector3& a, const JSBSim::FGColumnVector3& b) {
    return a(1) * b(1) + a(2) * b(2) + a(3) * b(3);
}

}  // namespace

// ---- the callback itself -----------------------------------------------------------------------

TEST_CASE("ground: flat terrain gives vertical AGL, the contact point below and an up normal") {
    FunctionTerrain flat([](double, double) { return 1000.0; });
    TerrainGroundCallback cb(&flat, kWgs84SemiMajorFt, kWgs84SemiMinorFt);
    const double lat = 45.5, lon = -122.6;
    const JSBSim::FGLocation at = locationAt(lat, lon, mToFt(1000.0) + 37.5);
    JSBSim::FGLocation contact;
    JSBSim::FGColumnVector3 normal, v(1, 2, 3), w(4, 5, 6);
    const double aglFt = cb.GetAGLevel(at, contact, normal, v, w);
    CHECK(aglFt == doctest::Approx(37.5).epsilon(1e-6));  // D16: meters MSL = geodetic feet
    contact.SetEllipse(kWgs84SemiMajorFt, kWgs84SemiMinorFt);
    CHECK(contact.GetGeodAltitude() == doctest::Approx(mToFt(1000.0)).epsilon(1e-6));
    CHECK(radToDeg(contact.GetGeodLatitudeRad()) == doctest::Approx(lat).epsilon(1e-9));
    CHECK(radToDeg(contact.GetLongitude()) == doctest::Approx(lon).epsilon(1e-9));
    CHECK(dot(normal, upEcef(lat, lon)) == doctest::Approx(1.0).epsilon(1e-9));
    CHECK(normal.Magnitude() == doctest::Approx(1.0).epsilon(1e-9));
    CHECK(v.Magnitude() == 0.0);  // the ground does not move
    CHECK(w.Magnitude() == 0.0);
}

TEST_CASE("ground: AGL is negative below the surface and ignores the IC terrain elevation") {
    FunctionTerrain flat([](double, double) { return 200.0; });
    TerrainGroundCallback cb(&flat, kWgs84SemiMajorFt, kWgs84SemiMinorFt);
    cb.SetTerrainElevation(99999.0);  // JSBSim's IC writes here; the terrain is the truth
    JSBSim::FGLocation contact;
    JSBSim::FGColumnVector3 n, v, w;
    CHECK(cb.GetAGLevel(locationAt(10, 20, mToFt(200.0) - 2.0), contact, n, v, w) ==
          doctest::Approx(-2.0).epsilon(1e-6));
}

TEST_CASE("ground: the normal tilts away from the uphill direction by the terrain slope") {
    const double lat = 40.0, lon = -100.0;
    // 10% rise toward the north: the normal leans 10% to the south.
    {
        FunctionTerrain t = rampTerrain(0.0, 0.10);
        TerrainGroundCallback cb(&t, kWgs84SemiMajorFt, kWgs84SemiMinorFt);
        JSBSim::FGLocation contact;
        JSBSim::FGColumnVector3 n, v, w;
        cb.GetAGLevel(locationAt(lat, lon, 3000), contact, n, v, w);
        const Mat3d ned = nedToEcef(lat, lon);
        auto comp = [&](int col) {  // component along NED axis col (0 N, 1 E, 2 D)
            return n(1) * ned.m[0][col] + n(2) * ned.m[1][col] + n(3) * ned.m[2][col];
        };
        const double len = std::sqrt(1.0 + 0.1 * 0.1);
        CHECK(comp(0) == doctest::Approx(-0.10 / len).epsilon(1e-3));
        CHECK(comp(1) == doctest::Approx(0.0).epsilon(1e-3));
        CHECK(comp(2) == doctest::Approx(-1.0 / len).epsilon(1e-3));
    }
    // 10% rise toward the west: the normal leans east.
    {
        FunctionTerrain t = rampTerrain(0.10, 0.0);
        TerrainGroundCallback cb(&t, kWgs84SemiMajorFt, kWgs84SemiMinorFt);
        JSBSim::FGLocation contact;
        JSBSim::FGColumnVector3 n, v, w;
        cb.GetAGLevel(locationAt(lat, lon, 3000), contact, n, v, w);
        const Mat3d ned = nedToEcef(lat, lon);
        const double east = n(1) * ned.m[0][1] + n(2) * ned.m[1][1] + n(3) * ned.m[2][1];
        CHECK(east == doctest::Approx(0.10 / std::sqrt(1.01)).epsilon(2e-3));
    }
}

TEST_CASE("ground: a cliff does not give a horizontal normal (slope limit)") {
    FunctionTerrain cliff([](double lat, double) { return lat < 40.0 ? 0.0 : 5000.0; });
    GroundCallbackConfig cfg;
    cfg.maxSlopeDeg = 60.0f;
    TerrainGroundCallback cb(&cliff, kWgs84SemiMajorFt, kWgs84SemiMinorFt, cfg);
    JSBSim::FGLocation contact;
    JSBSim::FGColumnVector3 n, v, w;
    cb.GetAGLevel(locationAt(40.0, -100.0, 20000), contact, n, v, w);
    CHECK(dot(n, upEcef(40.0, -100.0)) == doctest::Approx(std::cos(degToRad(60.0))).epsilon(1e-3));
}

TEST_CASE("ground: repeated queries at one location in one step cost one terrain sample set") {
    FunctionTerrain t = rampTerrain(0.02, 0.0);
    TerrainGroundCallback cb(&t, kWgs84SemiMajorFt, kWgs84SemiMinorFt);
    const JSBSim::FGLocation at = locationAt(40.0, -100.0, 3000);
    JSBSim::FGLocation contact;
    JSBSim::FGColumnVector3 n, v, w;
    cb.SetTime(1.0);
    cb.GetAGLevel(at, contact, n, v, w);
    CHECK(cb.elevationLookupCount() == 5);  // centre + four for the gradient
    cb.GetAGLevel(at, contact, n, v, w);
    cb.GetAGLevel(at, contact, n, v, w);
    CHECK(cb.elevationLookupCount() == 5);
    CHECK(cb.memoHitCount() == 2);
    cb.SetTime(1.0 + kSimDtS);  // next step: the terrain may have changed (tiles arrive)
    cb.GetAGLevel(at, contact, n, v, w);
    CHECK(cb.elevationLookupCount() == 10);
    cb.invalidate();
    cb.GetAGLevel(at, contact, n, v, w);
    CHECK(cb.elevationLookupCount() == 15);
}

TEST_CASE("ground: surface follows a TerrainService pack and its runway flatten area") {
    // Rough terrain: 8 m hills with a 150 m wavelength around XTST.
    const HeightFn rough = [](double lat, double lon) {
        return kXtstElevM +
               8.0 * std::sin(xtstEastM(lat, lon) / 24.0) * std::cos(xtstNorthM(lat) / 24.0);
    };
    SynthWorld world(rough);
    const SyntheticNavDb nav;
    const Runway* rwy = findRunway(nav.airport(), "27");
    REQUIRE(rwy != nullptr);
    TerrainGroundCallback cb(&world.terrain(), kWgs84SemiMajorFt, kWgs84SemiMinorFt);

    // Away from the runway the terrain is rough: slopes are not zero.
    double maxSlopeOff = 0;
    for (int i = 0; i < 40; ++i) {
        const double lat = 40.03 + 0.0004 * i, lon = -100.0 + 0.0003 * i;
        const auto s = cb.surfaceAt(degToRad(lat), degToRad(lon));
        maxSlopeOff = std::max(maxSlopeOff, std::hypot(s.slopeNorth, s.slopeEast));
    }
    CHECK(maxSlopeOff > 0.05);

    // On the runway pad the surface is dead flat at the runway elevation, normal straight up.
    world.terrain().addFlattenArea(runwayFlattenArea(*rwy));
    double maxSlopeOn = 0, maxDevM = 0;
    for (int i = 0; i <= 50; ++i) {
        double lat = 0, lon = 0;
        destinationPoint(rwy->thrLatDeg, rwy->thrLonDeg, degToRad(rwy->headingTrueDeg),
                         i * 0.02 * ftToM(rwy->lengthFt), lat, lon);
        const auto s = cb.surfaceAt(degToRad(lat), degToRad(lon));
        maxSlopeOn = std::max(maxSlopeOn, std::hypot(s.slopeNorth, s.slopeEast));
        maxDevM = std::max(maxDevM, std::fabs(s.elevM - kXtstElevM));
    }
    CHECK(maxSlopeOn == doctest::Approx(0.0).epsilon(1e-6));
    CHECK(maxDevM < 1e-3);
}

TEST_CASE("ground: callback cost per query on a terrain pack") {
    SynthWorld world([](double, double) { return kXtstElevM; });
    TerrainGroundCallback cb(&world.terrain(), kWgs84SemiMajorFt, kWgs84SemiMinorFt);
    JSBSim::FGLocation contact;
    JSBSim::FGColumnVector3 n, v, w;
    const int kN = 100000;
    // Walk like a landing roll: 0.25 m steps along the runway, one fresh location per query.
    std::vector<JSBSim::FGLocation> locs;
    locs.reserve(kN);
    for (int i = 0; i < kN; ++i)
        locs.push_back(locationAt(40.0, -100.0 + 0.0000029 * (i % 5000), mToFt(kXtstElevM) + 5.0));
    double sum = 0;
    const double t0 = nowS();
    for (int i = 0; i < kN; ++i) {
        cb.SetTime(i * kSimDtS);
        sum += cb.GetAGLevel(locs[static_cast<size_t>(i)], contact, n, v, w);
    }
    const double perQueryUs = (nowS() - t0) / kN * 1e6;
    std::printf("[ground] GetAGLevel on a TerrainService pack: %.2f us per query (%.0f)\n",
                perQueryUs, sum);
    CHECK(perQueryUs < 100.0);  // generous: the budget is 11 queries per 8.3 ms step
}
