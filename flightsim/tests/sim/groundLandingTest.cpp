// WP F3: landing, rolling and dropping the c172p on synthetic terrain.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/FlatTerrain.h"
#include "groundSupport.h"
#include "world/AirportGeometry.h"

using namespace fsim;
using namespace fsim::test;
using fsim::terrain::testing::HeightFn;

namespace {

struct LandingResult {
    bool touched = false, crashed = false;
    std::string reason;
    double touchdownVsFpm = 0;
    int groundTransitions = 0;  // wowAny edges after the first touchdown (bounces)
    double maxBounceAglFt = 0;  // highest wheel clearance after the first touchdown
    double maxPitchDeg = 0, maxAbsRollDeg = 0;
    double stopGsKt = 99;
    double maxHeadingErrDeg = 0;
    double endAglFt = 0;
};

// Flared landing with brakes on the rollout, 60 s total.
LandingResult land(TerrainQuery& terrain, double distNm, double headingTrueDeg = 270.0) {
    LandingResult r;
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "final", "distance_nm = " + std::to_string(distNm) + "\n"),
                    f.error);
    f.model.setTerrain(&terrain);
    shiftFinalToTerrain(f, terrain);
    REQUIRE_MESSAGE(f.begin(), f.error);
    bool wasOnGround = false;
    double touchdownT = -1;
    for (long i = 0; i < 70 * kSimHz && !f.s().crashed; ++i) {
        const bool brake = touchdownT >= 0 && f.s().simTimeS > touchdownT + 8.0;
        flaredLandingStep(f, headingTrueDeg, brake);
        const float vsBefore = f.s().vsFpm;
        f.step();
        const AircraftState& s = f.s();
        const bool onGround = s.wowAny;
        if (onGround && !r.touched) {
            r.touched = true;
            r.touchdownVsFpm = vsBefore;
            touchdownT = s.simTimeS;
        } else if (r.touched && onGround != wasOnGround) {
            ++r.groundTransitions;
        }
        wasOnGround = onGround;
        if (r.touched) {
            r.maxBounceAglFt = std::max(r.maxBounceAglFt, s.altAglFt - 4.3);
            r.maxPitchDeg = std::max(r.maxPitchDeg, radToDeg(s.pitchRad));
            r.maxAbsRollDeg = std::max(r.maxAbsRollDeg, std::fabs(radToDeg(s.rollRad)));
            if (s.simTimeS > touchdownT + 3.0)
                r.maxHeadingErrDeg =
                    std::max(r.maxHeadingErrDeg,
                             std::fabs(wrap180(radToDeg(s.headingTrueRad) - headingTrueDeg)));
        }
    }
    r.crashed = f.s().crashed;
    r.reason = f.s().crashReason;
    r.stopGsKt = f.s().gsKt;
    r.endAglFt = f.s().altAglFt;
    return r;
}

void checkGoodLanding(const char* what, const LandingResult& r) {
    std::printf(
        "[ground] %s: touchdown %.0f fpm, bounces %d (max %.1f ft), pitch<=%.1f roll<=%.1f, "
        "heading err %.1f, stop gs %.2f kt, crash '%s'\n",
        what, r.touchdownVsFpm, r.groundTransitions, r.maxBounceAglFt, r.maxPitchDeg,
        r.maxAbsRollDeg, r.maxHeadingErrDeg, r.stopGsKt, r.reason.c_str());
    INFO(what << " crash reason '" << r.reason << "'");
    CHECK(r.touched);
    CHECK_FALSE(r.crashed);
    CHECK(r.touchdownVsFpm > -600.0);
    CHECK(r.groundTransitions <= 4);  // a small skip is fine, a bouncing aircraft is not
    CHECK(r.maxBounceAglFt < 6.0);
    CHECK(r.maxPitchDeg < 12.0);
    CHECK(r.maxAbsRollDeg < 10.0);
    CHECK(r.stopGsKt < 2.0);  // braked to a stop on the slope
}

}  // namespace

TEST_CASE("sim: ground landing on level synthetic terrain is the reference") {
    SynthWorld w(slopedPlane(0.0, 0.0));
    checkGoodLanding("level", land(w.terrain(), 0.6));
}

TEST_CASE("sim: ground landing on an upslope, a downslope and a cross slope behaves") {
    // Level-8 packs (117 m samples east-west): 3 m per column is a 2.6% slope, held exactly.
    for (const int mPerColumn : {3, -3}) {
        const GridPlane g = gridPlane(8, mPerColumn, 0);  // + rises ahead (west), - falls away
        SynthWorld w(g.fn, nullptr, 0.15, 0, true, 8);
        checkGoodLanding(mPerColumn > 0 ? "upslope" : "downslope", land(w.terrain(), 0.6));
    }
    const GridPlane g = gridPlane(8, 0, 3);  // 2.0% rising to the north
    SynthWorld w(g.fn, nullptr, 0.15, 0, true, 8);
    checkGoodLanding("cross slope", land(w.terrain(), 0.6));
}

TEST_CASE("sim: ground a c172p rolls on a slope without a bounce explosion") {
    // Free roll downhill from rest: the plane keeps contact, accelerates gently, never launches.
    const GridPlane g = gridPlane(8, -4, 0);  // falls toward the west (3.4%), the way it faces
    SynthWorld w(g.fn, nullptr, 0.15, 0, true, 8);
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "runway"), f.error);
    f.model.setTerrain(&w.terrain());
    placeStartOnTerrain(f, w.terrain());
    REQUIRE_MESSAGE(f.begin(), f.error);
    f.ctl.parkingBrake = false;
    f.ctl.throttleNorm[0] = 0.0f;
    double maxAgl = 0, minAgl = 99;
    for (long i = 0; i < 40 * kSimHz; ++i) {
        f.ctl.rudderNorm = headingHoldRudder(f.s(), 270.0);
        f.ctl.steeringNorm = f.ctl.rudderNorm;
        f.step();
        if (i > 2 * kSimHz) {
            maxAgl = std::max<double>(maxAgl, f.s().altAglFt);
            minAgl = std::min<double>(minAgl, f.s().altAglFt);
        }
    }
    std::printf("[ground] downhill roll: gs %.1f kt, agl %.2f..%.2f ft\n", f.s().gsKt, minAgl,
                maxAgl);
    CHECK_FALSE(f.s().crashed);
    CHECK(f.s().gsKt > 5.0);  // it does roll downhill
    CHECK(f.s().gsKt < 40.0);
    CHECK(f.s().wowAll);
    CHECK(maxAgl - minAgl < 1.0);  // glued to the surface (rolling over 1 m quantised ground)
    CHECK(minAgl > 3.0);
}

// ---- runway pad on rough terrain ---------------------------------------------------------------

namespace {

HeightFn roughHills(double amplitudeM) {
    return [=](double lat, double lon) {
        return kXtstElevM +
               amplitudeM * std::sin(xtstEastM(lat, lon) / 24.0) * std::cos(xtstNorthM(lat) / 24.0);
    };
}

struct RollStats {
    double liftoffKt = -1;
    double maxAglDevFt = 0;  // CG height deviation from its standing value while rolling
    double maxPitchDevDeg = 0;
    double maxAbsVsFpm = 0;
    double maxHdgErrDeg = 0;
    bool crashed = false;
};

RollStats takeoffRoll(TerrainQuery& terrain) {
    RollStats r;
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "runway"), f.error);
    f.model.setTerrain(&terrain);
    placeStartOnTerrain(f, terrain);
    REQUIRE_MESSAGE(f.begin(), f.error);
    const double agl0 = f.s().altAglFt, pitch0 = radToDeg(f.s().pitchRad);
    f.ctl.parkingBrake = false;
    f.ctl.throttleNorm[0] = 1.0f;
    for (long i = 0; i < 40 * kSimHz && r.liftoffKt < 0 && !f.s().crashed; ++i) {
        takeoffRollStep(f, 270.0, 1000.0, 0.0);  // no rotation: measure the roll on the ground
        f.step();
        const AircraftState& s = f.s();
        if (s.simTimeS < 3.0) continue;
        if (s.iasKt > 52.0) break;  // stop before rotation speed
        r.maxAglDevFt = std::max(r.maxAglDevFt, std::fabs(s.altAglFt - agl0));
        r.maxPitchDevDeg = std::max(r.maxPitchDevDeg, std::fabs(radToDeg(s.pitchRad) - pitch0));
        r.maxAbsVsFpm = std::max<double>(r.maxAbsVsFpm, std::fabs(s.vsFpm));
        r.maxHdgErrDeg =
            std::max(r.maxHdgErrDeg, std::fabs(wrap180(radToDeg(s.headingTrueRad) - 270.0)));
        if (!s.wowAny) r.liftoffKt = s.iasKt;
    }
    r.crashed = f.s().crashed;
    return r;
}

}  // namespace

TEST_CASE("sim: ground a runway pad on rough terrain rolls flat") {
    const SyntheticNavDb nav;
    const Runway* rwy = findRunway(nav.airport(), "27");
    REQUIRE(rwy != nullptr);

    SynthWorld rough(roughHills(3.0));
    // Control: the same hills without a pad make a visibly bumpy roll.
    const RollStats bumpy = takeoffRoll(rough.terrain());
    world::registerAirportFlatten(nav.airport(), rough.terrain());  // as App does (W5)
    const RollStats flat = takeoffRoll(rough.terrain());
    FunctionTerrain level([](double, double) { return kXtstElevM; });
    const RollStats ref = takeoffRoll(level);  // the same roll on perfectly level ground
    std::printf(
        "[ground] roll on hills without pad: agl dev %.2f ft, pitch dev %.2f deg, |vs| %.0f fpm\n",
        bumpy.maxAglDevFt, bumpy.maxPitchDevDeg, bumpy.maxAbsVsFpm);
    std::printf(
        "[ground] roll on the flattened pad:  agl dev %.2f ft, pitch dev %.2f deg, |vs| %.0f fpm, "
        "heading err %.1f\n",
        flat.maxAglDevFt, flat.maxPitchDevDeg, flat.maxAbsVsFpm, flat.maxHdgErrDeg);
    CHECK_FALSE(flat.crashed);
    CHECK(flat.liftoffKt < 0);  // stayed on the ground up to 52 kt
    std::printf("[ground] roll on level ground:          agl dev %.2f ft, pitch dev %.2f deg, |vs| "
                "%.0f fpm\n",
                ref.maxAglDevFt, ref.maxPitchDevDeg, ref.maxAbsVsFpm);
    CHECK(flat.maxAglDevFt < ref.maxAglDevFt + 0.1);  // as flat as level ground
    CHECK(std::fabs(flat.maxPitchDevDeg - ref.maxPitchDevDeg) < 0.5);
    CHECK(flat.maxAbsVsFpm < ref.maxAbsVsFpm + 20.0);
    CHECK(flat.maxHdgErrDeg < 5.0);
    CHECK(bumpy.maxPitchDevDeg > 2.0 * flat.maxPitchDevDeg);  // the pad makes the difference
}

// ---- it cannot sink through the ground -----------------------------------------------------------

namespace {

struct DropResult {
    double impactFpm = 0;
    double minAglFt = 99;  // lowest CG height above the surface during the drop
    double minNoseGearClearFt = 99;
    bool crashed = false;
    std::string reason;
    bool touched = false;
};

// Free-fall drop of the parked c172p from `heightFt` of CG clearance (extra to the standing 4.3 ft).
DropResult drop(TerrainQuery& terrain, double heightFt, bool forgiving) {
    DropResult r;
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "runway"), f.error);
    if (forgiving) {  // judge the contact physics, not the crash rules
        f.profile.properties["crash.hard_landing_fpm"] = "100000";
        f.profile.properties["crash.touchdown_bank_deg"] = "180";
        f.profile.properties["crash.touchdown_pitch_up_deg"] = "90";
        f.profile.properties["crash.touchdown_pitch_down_deg"] = "90";
    }
    f.model.setTerrain(&terrain);
    placeStartOnTerrain(f, terrain);
    REQUIRE_MESSAGE(f.begin(), f.error);
    f.ctl.parkingBrake = true;
    f.run(1.2);  // crash rules are off in the first second after a start
    f.model.setProperty("position/h-agl-ft", f.s().altAglFt + heightFt);
    for (long i = 0; i < 6 * kSimHz && !f.s().crashed; ++i) {
        f.step();
        const AircraftState& s = f.s();
        r.minAglFt = std::min<double>(r.minAglFt, s.altAglFt);
        if (s.wowAny && !r.touched) {
            r.touched = true;
        }
        if (!r.touched) r.impactFpm = std::min<double>(r.impactFpm, s.vsFpm);
    }
    r.crashed = f.s().crashed;
    r.reason = f.s().crashReason;
    return r;
}

}  // namespace

TEST_CASE("sim: ground the c172p cannot sink through terrain at 120 Hz") {
    const GridPlane g = gridPlane(8, 4, 2);  // 3.4% up to the west and 1.3% to the north
    SynthWorld w(g.fn, nullptr, 0.15, 0, true, 8);
    for (const double heightFt : {0.5, 1.0, 2.0, 4.0, 8.0}) {
        const DropResult r = drop(w.terrain(), heightFt, true);
        std::printf("[ground] drop from %.1f ft: impact %.0f fpm, min CG agl %.2f ft, crash '%s'\n",
                    heightFt, r.impactFpm, r.minAglFt, r.reason.c_str());
        INFO("height " << heightFt << " ft, impact " << r.impactFpm << " fpm");
        CHECK(r.touched);
        CHECK(r.minAglFt > 1.5);  // the CG never goes near the surface (standing height 4.3 ft)
    }
}

TEST_CASE("sim: ground hard drops are judged a crash, soft ones are not") {
    FunctionTerrain level([](double, double) { return kXtstElevM; });
    const DropResult soft = drop(level, 0.5, false);  // ~ 340 fpm
    const DropResult hard = drop(level, 8.0, false);  // ~ 1350 fpm
    CHECK_FALSE(soft.crashed);
    CHECK(hard.crashed);
    CHECK(hard.reason == "hard landing");
    CHECK(hard.minAglFt > 1.5);
}
