#pragma once
// Support for the F3 ground-contact tests: synthetic terrain packs around the XTST airport,
// analytic terrains, and a flared-landing pilot. No real data is used anywhere.

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <string>

#include "FlightHarness.h"
#include "app/SpawnGate.h"
#include "core/SyntheticNavDb.h"
#include "world/terrain/SyntheticPack.h"
#include "world/terrain/TerrainService.h"

namespace fsim::test {

// Local metric coordinates around XTST (40 N, 100 W): east and north in meters.
inline double xtstEastM(double latDeg, double lonDeg) {
    return (lonDeg - SyntheticNavDb::kLonDeg) * kRadPerDeg * kEarthRadiusM *
           std::cos(degToRad(SyntheticNavDb::kLatDeg));
}
inline double xtstNorthM(double latDeg) {
    return (latDeg - SyntheticNavDb::kLatDeg) * kRadPerDeg * kEarthRadiusM;
}
constexpr double kXtstElevM = SyntheticNavDb::kElevFt * kMPerFt;  // 609.6 m

// Pure function terrain (no packs): elevation from a callable, land class fixed. Flatten areas are
// not supported (use a pack for those).
class FunctionTerrain : public TerrainQuery {
public:
    using Fn = std::function<double(double latDeg, double lonDeg)>;
    explicit FunctionTerrain(Fn fn, LandClass land = LandClass::Grass)
        : fn_(std::move(fn)), land_(land) {}
    float elevationM(double latDeg, double lonDeg) const override {
        ++calls;
        return static_cast<float>(fn_(latDeg, lonDeg));
    }
    LandClass landClass(double, double) const override { return land_; }
    void addFlattenArea(const FlattenArea&) override {}
    void clearFlattenAreas() override {}
    void prefetch(double, double, float) override {}
    bool isDetailReady(double, double) const override { return true; }
    mutable long calls = 0;

private:
    Fn fn_;
    LandClass land_;
};

// A terrain that never has its detail: for the spawn-gate timeout test.
class NeverReadyTerrain : public FunctionTerrain {
public:
    NeverReadyTerrain() : FunctionTerrain([](double, double) { return kXtstElevM; }) {}
    bool isDetailReady(double, double) const override { return false; }
    void prefetch(double, double, float) override { ++prefetches; }
    int prefetches = 0;
};

// A synthetic level-10 pack around XTST (tiles of 0.088 degrees, ~30 m height samples) served by
// a real TerrainService, so the tests go through the same bilinear / quantised / flattened path as
// real data.
struct SynthWorld {
    terrain::testing::TempDir dir;
    std::unique_ptr<terrain::TerrainService> svc;

    // halfSpanDeg: the pack covers XTST +- this many degrees. loadDelayMs slows the worker (gating
    // tests). load = false leaves the tiles unloaded.
    SynthWorld(const terrain::testing::HeightFn& height,
               const terrain::testing::LandFn& land = nullptr, double halfSpanDeg = 0.15,
               int loadDelayMs = 0, bool load = true) {
        constexpr int kLevel = 10;
        terrain::testing::PackWriter w(dir.path, kLevel, kLevel);
        w.writeBox(kLevel, SyntheticNavDb::kLatDeg - halfSpanDeg,
                   SyntheticNavDb::kLonDeg - halfSpanDeg, SyntheticNavDb::kLatDeg + halfSpanDeg,
                   SyntheticNavDb::kLonDeg + halfSpanDeg, height, land);
        w.finish();
        terrain::TerrainConfig cfg;
        cfg.debugLoadDelayMs = loadDelayMs;
        svc = std::make_unique<terrain::TerrainService>(dir.path, cfg);
        if (load) loadAll();
    }

    // Prefetch around XTST, integrate, and wait for the worker (tests only).
    bool loadAll(double radiusM = 12000.0f) {
        svc->prefetch(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg,
                      static_cast<float>(radiusM));
        svc->update(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg);
        return svc->waitForIdle(30.0);
    }
    terrain::TerrainService& terrain() { return *svc; }
};

// Puts a resolved runway start on the terrain below it.
inline void placeStartOnTerrain(Flight& f, const TerrainQuery& terrain) {
    f.start.fieldElevFt = mToFt(terrain.elevationM(f.start.latDeg, f.start.lonDeg));
}

// Shifts a resolved Final start (resolved against the flat 2,000 ft field) so the glidepath
// reference is the terrain at the runway threshold, keeping the 3 degree path and TCH.
inline void shiftFinalToTerrain(Flight& f, const TerrainQuery& terrain) {
    const Runway* rwy = findRunway(f.nav.airport(), "27");
    const double thrElevFt = mToFt(terrain.elevationM(rwy->thrLatDeg, rwy->thrLonDeg));
    f.start.altMslFt += thrElevFt - SyntheticNavDb::kElevFt;
    f.start.fieldElevFt = mToFt(terrain.elevationM(f.start.latDeg, f.start.lonDeg));
}

// c172p flared landing and rollout on a runway heading (the F1 landing recipe): hands-off down the
// glidepath, idle and a progressive flare below 30 ft, then keep the nose light and track the
// heading. Returns true when the wheels have touched at some point during the call.
inline void flaredLandingStep(Flight& f, double headingTrueDeg, bool brakeOnRollout = false) {
    const AircraftState& s = f.s();
    const double aglFt = s.altAglFt - 4.3;  // CG height on the wheels is 4.3 ft
    f.ctl.aileronNorm = bankHoldAileron(s);
    if (!s.wowAny) {
        if (aglFt < 30.0) {
            f.ctl.throttleNorm[0] = 0.0f;
            const double pitchTargetDeg = -0.6 + (30.0 - std::max(aglFt, 0.0)) / 30.0 * 5.5;
            f.ctl.elevatorNorm = pitchHoldElevator(s, pitchTargetDeg, 0.05, 0.05);
        }
    } else {
        f.ctl.throttleNorm[0] = 0.0f;
        f.ctl.elevatorNorm = 0.1f;
        f.ctl.rudderNorm = headingHoldRudder(s, headingTrueDeg);
        f.ctl.steeringNorm = f.ctl.rudderNorm;
        if (brakeOnRollout) f.ctl.brakeLeftNorm = f.ctl.brakeRightNorm = 0.6f;
    }
}

// Pilot for the c172p takeoff roll (as in FlightModelTest): rudder and steering hold the heading.
inline void takeoffRollStep(Flight& f, double headingTrueDeg, double rotateKt, double pitchDeg) {
    const AircraftState& s = f.s();
    const float rud = headingHoldRudder(s, headingTrueDeg);
    f.ctl.rudderNorm = rud;
    f.ctl.steeringNorm = rud;
    f.ctl.aileronNorm = bankHoldAileron(s);
    f.ctl.elevatorNorm = s.iasKt > rotateKt ? pitchHoldElevator(s, pitchDeg, 0.5, 0.1) : 0.f;
}

inline double nowS() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

}  // namespace fsim::test
