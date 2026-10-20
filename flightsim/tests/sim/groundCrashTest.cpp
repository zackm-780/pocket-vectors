// WP F3: F1's terrain-closure and water crash rules with real TerrainService data.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "core/FlatTerrain.h"
#include "groundSupport.h"

using namespace fsim;
using namespace fsim::test;
using fsim::terrain::testing::HeightFn;

namespace {

// A north-south ridge west of XTST: level at the field elevation, then a ramp with the given
// grade starting `footM` west of XTST and rising to a plateau `heightM` higher.
HeightFn ridge(double footM, double grade, double heightM) {
    return [=](double lat, double lon) {
        const double westM = -xtstEastM(lat, lon);
        return kXtstElevM + std::clamp((westM - footM) * grade, 0.0, heightM);
    };
}

struct CruiseRun {
    bool crashed = false;
    std::string reason;
    double crashWestM = 0, crashAglFt = 0, crashIasKt = 0, simTimeS = 0;
    double minAglFt = 1e9;
};

// c172p cruising west at `altFt` MSL, 100 kt, hands-off, toward the ridge.
CruiseRun flyWestInto(TerrainQuery& terrain, double altFt, double durationS) {
    CruiseRun r;
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "cruise", "altitude_ft = " + std::to_string(altFt) + "\n"),
                    f.error);
    f.start.headingTrueDeg = 270;
    f.start.iasKt = 100;
    f.start.latDeg = SyntheticNavDb::kLatDeg;
    f.start.lonDeg = SyntheticNavDb::kLonDeg + 0.0125;  // ~1 km east of the field
    f.model.setTerrain(&terrain);
    placeStartOnTerrain(f, terrain);
    REQUIRE_MESSAGE(f.begin(), f.error);
    for (long i = 0; i < static_cast<long>(durationS * kSimHz) && !f.s().crashed; ++i) {
        f.step();
        r.minAglFt = std::min<double>(r.minAglFt, f.s().altAglFt);
    }
    const AircraftState& s = f.s();
    r.crashed = s.crashed;
    r.reason = s.crashReason;
    r.crashWestM = -xtstEastM(s.latDeg, s.lonDeg);
    r.crashAglFt = s.altAglFt;
    r.crashIasKt = s.iasKt;
    r.simTimeS = s.simTimeS;
    return r;
}

}  // namespace

TEST_CASE("sim: ground flying into a ridge is a terrain crash") {
    // 2 km of level ground, then a 15% ramp to 500 m above the field. The aircraft flies level at
    // 250 ft above the field, so the ramp meets it about 600 m after the foot.
    SynthWorld w(ridge(2000.0, 0.15, 500.0));
    const CruiseRun r = flyWestInto(w.terrain(), SyntheticNavDb::kElevFt + 250.0, 120.0);
    std::printf(
        "[ground] ridge: crashed=%d '%s' at %.0f m west, agl %.1f ft, ias %.0f kt, t=%.1f\n",
        r.crashed, r.reason.c_str(), r.crashWestM, r.crashAglFt, r.crashIasKt, r.simTimeS);
    REQUIRE(r.crashed);
    CHECK(r.crashAglFt < 30.0);
    CHECK(r.crashWestM > 2000.0);  // it hit the ramp, not the level ground
    CHECK(r.crashWestM < 3500.0);
    CHECK(!r.reason.empty());
    CHECK(r.reason != "water impact");
    CHECK(r.minAglFt > -1.0);  // frozen at the surface, not below it
}

TEST_CASE("sim: ground the same flight clears the ridge with enough altitude or without it") {
    SynthWorld w(ridge(2000.0, 0.15, 500.0));
    // Well above the plateau (500 m = 1640 ft above the field): no contact.
    const CruiseRun high = flyWestInto(w.terrain(), SyntheticNavDb::kElevFt + 2400.0, 90.0);
    CHECK_FALSE(high.crashed);
    CHECK(high.minAglFt > 300.0);
    // The same low flight over flat ground (no ridge) is fine.
    SynthWorld flat(ridge(1e9, 0.0, 0.0));
    const CruiseRun low = flyWestInto(flat.terrain(), SyntheticNavDb::kElevFt + 250.0, 90.0);
    CHECK_FALSE(low.crashed);
}

TEST_CASE("sim: ground a gentle slope is landable, a steep one is not (closure rate)") {
    // Same speed, different grade: 5% closes at ~500 fpm (fine), 20% at ~2000 fpm (crash).
    for (const double grade : {0.05, 0.20}) {
        SynthWorld w(ridge(2000.0, grade, 800.0));
        const CruiseRun r = flyWestInto(w.terrain(), SyntheticNavDb::kElevFt + 40.0, 110.0);
        std::printf("[ground] grade %.2f: crashed=%d '%s' ias %.0f\n", grade, r.crashed,
                    r.reason.c_str(), r.crashIasKt);
        if (grade > 0.1)
            CHECK(r.crashed);
        else
            CHECK_FALSE(r.crashed);  // it climbs the slope (slowly) or touches down softly
    }
}

// ---- water ---------------------------------------------------------------------------------------

TEST_CASE("sim: ground touching down on a Water land-class tile is a water impact") {
    const HeightFn flat = [](double, double) { return kXtstElevM; };
    const terrain::testing::LandFn water = [](double, double) {
        return static_cast<uint8_t>(LandClass::Water);
    };
    const terrain::testing::LandFn grass = [](double, double) {
        return static_cast<uint8_t>(LandClass::Grass);
    };
    auto fly = [&](const terrain::testing::LandFn& land) {
        SynthWorld w(flat, land);
        REQUIRE(w.terrain().landClass(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg) ==
                (land(0, 0) == static_cast<uint8_t>(LandClass::Water) ? LandClass::Water
                                                                      : LandClass::Grass));
        Flight f;
        REQUIRE_MESSAGE(f.resolve("c172p", "final", "distance_nm = 0.5\n"), f.error);
        f.model.setTerrain(&w.terrain());
        shiftFinalToTerrain(f, w.terrain());
        REQUIRE_MESSAGE(f.begin(), f.error);
        for (long i = 0; i < 40 * kSimHz && !f.s().crashed; ++i) {
            flaredLandingStep(f, 270.0);
            f.step();
        }
        return std::make_pair(f.s().crashed, std::string(f.s().crashReason));
    };
    const auto onWater = fly(water);
    const auto onGrass = fly(grass);
    std::printf("[ground] water: crashed=%d '%s'; grass: crashed=%d '%s'\n", onWater.first,
                onWater.second.c_str(), onGrass.first, onGrass.second.c_str());
    CHECK(onWater.first);
    CHECK(onWater.second == "water impact");
    CHECK_FALSE(onGrass.first);
}
