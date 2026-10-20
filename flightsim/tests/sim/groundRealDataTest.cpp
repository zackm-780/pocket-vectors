// WP F3: manual real-data checks (KPDX runway start, KOTH 05 final, Mt Hood). They need the real
// nav and terrain packs, so they only run when FSIM_REAL_DATA_DIR points at a data root
// (<root>/nav, <root>/terrain); otherwise they return immediately. Tests must not need real data
// (CONVENTIONS section 4), so CI never runs the body.
//
//   FSIM_REAL_DATA_DIR=/path/to/flightsim/data ./flightsim_tests --test-case='*real data*'
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "app/Simulation.h"
#include "groundSupport.h"
#include "nav/NavDb.h"
#include "world/AirportGeometry.h"

using namespace fsim;
using namespace fsim::test;

namespace {

struct RealWorld {
    std::string root;
    Paths paths = Paths::resolve({});
    CsvNavDatabase nav;
    std::unique_ptr<terrain::TerrainService> svc;
    std::string error;

    bool open() {
        const char* env = std::getenv("FSIM_REAL_DATA_DIR");
        if (!env || !*env) return false;
        root = env;
        if (!nav.load(Paths::join(root, "nav"), error)) return false;
        svc = std::make_unique<terrain::TerrainService>(Paths::join(root, "terrain"));
        return svc->isOpen();
    }
    TerrainPump pump() {
        return [this](double lat, double lon) { svc->update(lat, lon); };
    }
};

struct ApproachRef {
    double thrLatDeg, thrLonDeg, headingTrueDeg, thrElevFt;
};

// Crosstrack (m, + = right of the extended centerline) and distance to the threshold along it
// (m, + = before the threshold).
void runwayFrame(const ApproachRef& a, const AircraftState& s, double& xtkM, double& beforeThrM) {
    const LatLon start{a.thrLatDeg, a.thrLonDeg};
    double lat2 = 0, lon2 = 0;
    destinationPoint(a.thrLatDeg, a.thrLonDeg, degToRad(a.headingTrueDeg), 1000.0, lat2, lon2);
    const LatLon end{lat2, lon2};
    const LatLon p{s.latDeg, s.lonDeg};
    xtkM = crossTrackM(p, start, end);
    beforeThrM = -alongTrackM(p, start, end);
}

// A simple pilot for a 3 degree ILS-like approach: pitch tracks the path (via vertical speed),
// throttle holds 65 kt (PI), ailerons follow the centerline; below 30 ft the F1 flare takes over.
struct ApproachPilot {
    ApproachRef a;
    double trimPitchDeg = 0;
    double throttleI = 0.45;

    void update(Simulation& sim) {
        const AircraftState& s = sim.aircraft();
        ControlInputs& c = sim.pilot();
        double xtkM = 0, beforeM = 0;
        runwayFrame(a, s, xtkM, beforeM);
        // Lateral: steer the track toward the centerline (up to 15 degrees of intercept), bank up
        // to 15 degrees.
        const double trackTarget = a.headingTrueDeg - std::clamp(xtkM * 0.08, -15.0, 15.0);
        const double bank =
            std::clamp(0.6 * wrap180(trackTarget - radToDeg(s.trackTrueRad)), -15.0, 15.0);
        c.aileronNorm = bankHoldAileron(s, bank);
        c.rudderNorm = 0;
        const double aglOverRunwayFt = s.altMslFt - a.thrElevFt - 4.3;
        if (s.wowAny || aglOverRunwayFt < 30.0) {
            // flare / rollout (F1 recipe), heading hold on the runway heading
            c.throttleNorm[0] = 0.0f;
            if (!s.wowAny) {
                const double pitchTargetDeg =
                    -0.6 + (30.0 - std::max(aglOverRunwayFt, 0.0)) / 30.0 * 5.5;
                c.elevatorNorm = pitchHoldElevator(s, pitchTargetDeg, 0.05, 0.05);
            } else {
                c.elevatorNorm = 0.1f;
                c.rudderNorm = headingHoldRudder(s, a.headingTrueDeg);
                c.steeringNorm = c.rudderNorm;
                c.aileronNorm = bankHoldAileron(s);
                if (s.gsKt < 50.0f) c.brakeLeftNorm = c.brakeRightNorm = 0.6f;
            }
            return;
        }
        // Vertical: the path is 50 ft over the threshold at 3 degrees.
        const double pathAltFt =
            a.thrElevFt + 50.0 + std::max(beforeM, 0.0) * kFtPerM * std::tan(degToRad(3.0));
        const double errFt = s.altMslFt - pathAltFt;  // + = above the path
        const double vsTargetFpm =
            std::clamp(-s.gsKt * 101.3 * std::tan(degToRad(3.0)) - 3.0 * errFt, -700.0, 0.0);
        const double pitchTarget =
            trimPitchDeg + std::clamp(0.012 * (vsTargetFpm - s.vsFpm), -4.0, 4.0);
        c.elevatorNorm = pitchHoldElevator(s, pitchTarget, 0.08, 0.06);
        const double ias = s.iasKt;
        throttleI = std::clamp(throttleI + 0.0004 * (65.0 - ias) * kSimDtS * 60.0 / 60.0, 0.1, 0.9);
        c.throttleNorm[0] =
            static_cast<float>(std::clamp(throttleI + 0.03 * (65.0 - ias), 0.0, 1.0));
    }
};

}  // namespace

TEST_CASE("sim: ground real data: KPDX 28L runway start sits on the runway and takes off") {
    RealWorld w;
    if (!w.open()) return;
    Simulation sim(w.paths, w.nav, *w.svc);
    world::registerAirportFlatten(*w.nav.findAirport("KPDX"), *w.svc);  // as App does (W5)
    sim.setTerrainPump(w.pump());
    Scenario sc;
    std::string err;
    REQUIRE_MESSAGE(
        parseScenario("[scenario]\nairport = KPDX\nrunway = 28L\nstart = runway\n", sc, err), err);
    REQUIRE_MESSAGE(sim.init(sc, "", err), err);
    const Airport* kpdx = w.nav.findAirport("KPDX");
    const Runway* rwy = findRunway(*kpdx, "28L");
    const double padFt = 0.5 * (rwy->thrElevFt + rwy->endElevFt);
    std::printf(
        "[real] KPDX 28L: gate waited %.2f s; pad %.1f ft; natural terrain at start %.1f ft\n",
        sim.spawnGate().waitedS, padFt,
        mToFt(w.svc->naturalElevationM(sim.startCondition().latDeg, sim.startCondition().lonDeg)));
    for (long i = 0; i < 3 * kSimHz; ++i) sim.stepOnce();
    const AircraftState& s0 = sim.aircraft();
    std::printf("[real] KPDX 28L parked: msl %.2f ft, agl %.2f ft, terrain %.2f ft, wow %d\n",
                s0.altMslFt, s0.altAglFt, s0.terrainElevFt, s0.wowAll);
    CHECK(s0.wowAll);
    CHECK(s0.terrainElevFt == doctest::Approx(padFt).epsilon(1e-3));
    CHECK(s0.altAglFt == doctest::Approx(4.37).epsilon(0.05));

    // Takeoff: full power, hold the centerline, rotate at 55 kt.
    sim.pilot().parkingBrake = false;
    sim.pilot().throttleNorm[0] = 1.0f;
    double liftoffKt = -1, maxXtkM = 0, liftoffAlongM = 0;
    const LatLon thr{rwy->thrLatDeg, rwy->thrLonDeg}, end{rwy->endLatDeg, rwy->endLonDeg};
    for (long i = 0; i < 60 * kSimHz && !sim.aircraft().crashed; ++i) {
        const AircraftState& s = sim.aircraft();
        const double hdg = rwy->headingTrueDeg;
        sim.pilot().rudderNorm = headingHoldRudder(s, hdg);
        sim.pilot().steeringNorm = sim.pilot().rudderNorm;
        sim.pilot().aileronNorm = bankHoldAileron(s);
        sim.pilot().elevatorNorm = s.iasKt > 55.0f ? pitchHoldElevator(s, 7.0, 0.5, 0.1) : 0.f;
        sim.stepOnce();
        const AircraftState& t = sim.aircraft();
        if (t.wowAny)  // centerline error during the ground roll
            maxXtkM = std::max(maxXtkM, std::fabs(crossTrackM({t.latDeg, t.lonDeg}, thr, end)));
        if (liftoffKt < 0 && !t.wowAny) {
            liftoffKt = t.iasKt;
            liftoffAlongM = alongTrackM({t.latDeg, t.lonDeg}, thr, end);
        }
    }
    const AircraftState& s = sim.aircraft();
    std::printf(
        "[real] KPDX 28L takeoff: liftoff %.1f kt after %.0f m, max centerline error %.1f m, "
        "after 60 s: agl %.0f ft, vs %.0f fpm, crashed %d\n",
        liftoffKt, liftoffAlongM, maxXtkM, s.altAglFt, s.vsFpm, s.crashed);
    CHECK_FALSE(s.crashed);
    CHECK(liftoffKt > 50.0);
    CHECK(liftoffKt < 65.0);
    CHECK(maxXtkM < 15.0);
    CHECK(s.altAglFt > 300.0);
}

TEST_CASE("sim: ground real data: KOTH 05 final lands on the runway without a crash") {
    RealWorld w;
    if (!w.open()) return;
    Simulation sim(w.paths, w.nav, *w.svc);
    world::registerAirportFlatten(*w.nav.findAirport("KOTH"), *w.svc);  // as App does (W5)
    sim.setTerrainPump(w.pump());
    Scenario sc;
    std::string err;
    REQUIRE_MESSAGE(parseScenario("[scenario]\nairport = KOTH\nrunway = 05\nstart = final\n"
                                  "distance_nm = 6\n",
                                  sc, err),
                    err);
    REQUIRE_MESSAGE(sim.init(sc, "", err), err);
    const Runway* rwy = findRunway(*w.nav.findAirport("KOTH"), "05");
    const ApproachRef a{rwy->thrLatDeg, rwy->thrLonDeg, rwy->headingTrueDeg,
                        0.5 * (rwy->thrElevFt + rwy->endElevFt)};
    ApproachPilot pilot{a, radToDeg(sim.aircraft().pitchRad)};
    pilot.throttleI = sim.pilot().throttleNorm[0];
    std::printf("[real] KOTH 05 final: start msl %.0f ft agl %.0f ft; pad %.1f ft; gate %.2f s\n",
                sim.aircraft().altMslFt, sim.aircraft().altAglFt, a.thrElevFt,
                sim.spawnGate().waitedS);
    bool touched = false;
    double touchdownVsFpm = 0, touchdownBeforeM = 0, touchdownXtk = 0, minAglApproach = 1e9;
    for (long i = 0; i < 6 * 60 * kSimHz && !sim.aircraft().crashed; ++i) {
        pilot.update(sim);
        const float vsBefore = sim.aircraft().vsFpm;
        w.svc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
        sim.stepOnce();
        const AircraftState& s = sim.aircraft();
        if (!touched) minAglApproach = std::min<double>(minAglApproach, s.altAglFt);
        if (std::getenv("FSIM_VERBOSE") && i % (5 * kSimHz) == 0) {
            double x = 0, b = 0;
            runwayFrame(a, s, x, b);
            std::printf("  t=%5.1f before=%7.0f m xtk=%6.1f alt=%6.0f agl=%6.0f gnd=%5.0f "
                        "ias=%5.1f vs=%5.0f pitch=%4.1f thr=%.2f wow=%d\n",
                        s.simTimeS, b, x, s.altMslFt, s.altAglFt, s.terrainElevFt, s.iasKt, s.vsFpm,
                        radToDeg(s.pitchRad), sim.pilot().throttleNorm[0], s.wowAny);
        }
        if (s.wowAny && !touched) {
            touched = true;
            touchdownVsFpm = vsBefore;
            double xtk = 0, before = 0;
            runwayFrame(a, s, xtk, before);
            touchdownBeforeM = before;
            touchdownXtk = xtk;
        }
        if (touched && s.gsKt < 1.0f) break;
    }
    const AircraftState& s = sim.aircraft();
    double xtk = 0, before = 0;
    runwayFrame(a, s, xtk, before);
    std::printf("[real] KOTH 05 landing: touched %d at %.0f m past the threshold, %.1f m off the "
                "centerline, sink %.0f fpm; stopped at %.0f m past, gs %.1f kt, crashed %d '%s'; "
                "min agl on approach %.0f ft; sim time %.0f s\n",
                touched, -touchdownBeforeM, touchdownXtk, touchdownVsFpm, -before, s.gsKt,
                s.crashed, s.crashReason, minAglApproach, s.simTimeS);
    CHECK(touched);
    CHECK_FALSE(s.crashed);
    CHECK(touchdownVsFpm > -500.0);
    CHECK(-touchdownBeforeM > 0.0);                         // past the threshold
    CHECK(-touchdownBeforeM < ftToM(rwy->lengthFt) * 0.5);  // in the first half
    CHECK(std::fabs(touchdownXtk) < 0.5 * ftToM(rwy->widthFt));
    CHECK(s.gsKt < 2.0);
}

TEST_CASE("sim: ground real data: flying into Mt Hood's slope is a terrain crash") {
    RealWorld w;
    if (!w.open()) return;
    // 25 NM west-northwest of the summit (45.3735 N, 121.6959 W, 11,249 ft), level at 7,000 ft,
    // 100 kt, heading straight for it.
    const double summitLat = 45.3735, summitLon = -121.6959;
    double lat = 0, lon = 0;
    destinationPoint(summitLat, summitLon, degToRad(289.0), nmToM(25.0), lat, lon);
    const SpawnGateResult gate =
        waitForSpawnDetail(*w.svc, {{lat, lon}, {summitLat, summitLon}}, 15000.0f, 20.0, w.pump());
    Flight f;
    REQUIRE_MESSAGE(f.resolve("c172p", "cruise", "altitude_ft = 7000\n"), f.error);
    f.start.latDeg = lat;
    f.start.lonDeg = lon;
    f.start.headingTrueDeg = 109.0;
    f.start.altMslFt = 7000.0;
    f.start.iasKt = 100.0;
    f.model.setTerrain(w.svc.get());
    f.start.fieldElevFt = mToFt(w.svc->elevationM(lat, lon));
    REQUIRE_MESSAGE(f.begin(), f.error);
    double minAglFt = 1e9;
    for (long i = 0; i < 20 * 60 * kSimHz && !f.s().crashed; ++i) {
        w.svc->update(f.s().latDeg, f.s().lonDeg);
        f.step();
        minAglFt = std::min<double>(minAglFt, f.s().altAglFt);
    }
    const AircraftState& s = f.s();
    const double distToSummitKm = greatCircleDistM(s.latDeg, s.lonDeg, summitLat, summitLon) / 1000;
    std::printf(
        "[real] Mt Hood: gate %.2f s; crashed=%d '%s' at %.0f ft MSL (ground %.0f ft), %.1f km "
        "from the summit, ias %.0f kt, t=%.0f s, min agl %.0f ft\n",
        gate.waitedS, s.crashed, s.crashReason, s.altMslFt, s.terrainElevFt, distToSummitKm,
        s.iasKt, s.simTimeS, minAglFt);
    CHECK(s.crashed);
    CHECK(distToSummitKm < 15.0);
    CHECK(s.altMslFt > 6500.0);
    CHECK(s.altAglFt < 60.0);
}
