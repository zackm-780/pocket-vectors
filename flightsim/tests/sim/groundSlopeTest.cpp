// WP F3: the c172p on sloped and rough synthetic terrain (parking, landing, rolling, drops).
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/FlatTerrain.h"
#include "groundSupport.h"

using namespace fsim;
using namespace fsim::test;
using fsim::terrain::testing::HeightFn;

namespace {

struct Rest {
    float pitchDeg, rollDeg, aglFt, gsKt;
    bool wowAll, crashed;
};

// c172p parked (parking brake) on the terrain for 12 s.
Rest parkOn(TerrainQuery& terrain) {
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "runway"), f.error);
    f.model.setTerrain(&terrain);
    placeStartOnTerrain(f, terrain);
    REQUIRE_MESSAGE(f.begin(), f.error);
    f.run(12.0);
    const AircraftState& s = f.s();
    return {static_cast<float>(radToDeg(s.pitchRad)), static_cast<float>(radToDeg(s.rollRad)),
            s.altAglFt, s.gsKt, s.wowAll, s.crashed};
}

}  // namespace

TEST_CASE("sim: ground c172p parks on a slope and takes the slope's attitude") {
    // Facing west (runway 27). Reference: the same park on level ground.
    FunctionTerrain level([](double, double) { return kXtstElevM; });
    const Rest flat = parkOn(level);
    CHECK(flat.wowAll);
    CHECK_FALSE(flat.crashed);

    const double slope = 0.05, slopeDeg = radToDeg(std::atan(slope));
    {
        // Rising ahead (west): the nose goes up by the slope angle.
        SynthWorld w(slopedPlane(slope, 0.0));
        const Rest r = parkOn(w.terrain());
        std::printf("[ground] uphill park: pitch %.2f (flat %.2f) roll %.2f agl %.2f gs %.2f\n",
                    r.pitchDeg, flat.pitchDeg, r.rollDeg, r.aglFt, r.gsKt);
        CHECK(r.wowAll);
        CHECK_FALSE(r.crashed);
        CHECK(r.gsKt < 0.5f);  // the parking brake holds it
        // The nose rises by the slope angle; the brake force at the main wheels (below the CG)
        // shifts load off the nose strut and adds up to about 2 degrees on top of it.
        CHECK(r.pitchDeg - flat.pitchDeg > slopeDeg - 0.5);
        CHECK(r.pitchDeg - flat.pitchDeg < slopeDeg + 2.5);
        CHECK(std::fabs(r.rollDeg) < 1.0f);
        CHECK(r.aglFt == doctest::Approx(flat.aglFt).epsilon(0.1));
    }
    {
        // Rising to the north = the right side when facing west: it banks left by the slope angle.
        SynthWorld w(slopedPlane(0.0, slope));
        const Rest r = parkOn(w.terrain());
        std::printf("[ground] cross-slope park: pitch %.2f roll %.2f agl %.2f gs %.2f\n",
                    r.pitchDeg, r.rollDeg, r.aglFt, r.gsKt);
        CHECK(r.wowAll);
        CHECK_FALSE(r.crashed);
        CHECK(std::fabs(r.rollDeg + slopeDeg) < 1.0);
        CHECK(std::fabs(r.pitchDeg - flat.pitchDeg) < 1.0f);
    }
}

TEST_CASE("sim: ground the AGL the model reports equals the height above the terrain") {
    SynthWorld w(slopedPlane(0.03, 0.0));
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "cruise", "altitude_ft = 3500\n"), f.error);
    f.model.setTerrain(&w.terrain());
    placeStartOnTerrain(f, w.terrain());
    REQUIRE_MESSAGE(f.begin(), f.error);
    f.run(5.0);
    const AircraftState& s = f.s();
    const double terrainFt = mToFt(w.terrain().elevationM(s.latDeg, s.lonDeg));
    CHECK(s.altAglFt == doctest::Approx(s.altMslFt - terrainFt).epsilon(1e-3));
    CHECK(s.terrainElevFt == doctest::Approx(terrainFt).epsilon(1e-3));
}
