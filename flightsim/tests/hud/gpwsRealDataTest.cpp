// X2: manual real-data checks for GPWS-lite (Mt Hood head-on, KOTH ILS 05 final). They need the
// real nav and terrain packs and only run when FSIM_REAL_DATA_DIR points at a data root
// (<root>/nav, <root>/terrain); otherwise they return immediately (tests never need real data).
//
//   FSIM_REAL_DATA_DIR=/path/to/flightsim/data ./flightsim_tests --test-case='*gpws real data*'
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>

#include "app/Simulation.h"
#include "hud/GpwsMonitor.h"
#include "hud/HudLogic.h"
#include "nav/NavDb.h"
#include "sim/groundSupport.h"
#include "world/AirportGeometry.h"

using namespace fsim;
using namespace fsim::test;

namespace {

struct RealWorld {
    Paths paths = Paths::resolve({});
    CsvNavDatabase nav;
    std::unique_ptr<terrain::TerrainService> svc;
    std::string error;

    bool open() {
        const char* env = std::getenv("FSIM_REAL_DATA_DIR");
        if (!env || !*env) return false;
        if (!nav.load(Paths::join(env, "nav"), error)) return false;
        svc = std::make_unique<terrain::TerrainService>(Paths::join(env, "terrain"));
        return svc->isOpen();
    }
};

}  // namespace

TEST_CASE("sim: gpws real data: Mt Hood head-on gets TERRAIN and PULL UP before impact") {
    RealWorld w;
    if (!w.open()) return;
    const double summitLat = 45.3735, summitLon = -121.6959;
    double lat = 0, lon = 0;
    destinationPoint(summitLat, summitLon, degToRad(289.0), nmToM(25.0), lat, lon);
    waitForSpawnDetail(*w.svc, {{lat, lon}, {summitLat, summitLon}}, 15000.0f, 20.0,
                       [&](double la, double lo) { w.svc->update(la, lo); });
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
    GpwsMonitor mon;
    double tTerr = -1, tPull = -1, tImpact = -1;
    for (long i = 0; i < 20 * 60 * kSimHz && !f.s().crashed; ++i) {
        w.svc->update(f.s().latDeg, f.s().lonDeg);
        f.step();
        mon.update(f.s(), f.profile, nullptr, *w.svc, &w.nav);
        const double t = f.s().simTimeS;
        if (tTerr < 0 && (mon.warnings() & warningBit(WarningId::Terrain))) tTerr = t;
        if (tPull < 0 && (mon.warnings() & warningBit(WarningId::PullUp))) tPull = t;
    }
    tImpact = f.s().simTimeS;
    std::printf("[gpws real] Mt Hood: TERRAIN %.1f s, PULL UP %.1f s, impact %.1f s\n", tTerr,
                tPull, tImpact);
    CHECK(tTerr > 0);
    CHECK(tPull > tTerr);
    CHECK(tImpact - tTerr > 25.0);
    CHECK(tImpact - tPull > 10.0);
}

TEST_CASE("sim: gpws real data: KOTH ILS 05 final gives callouts but no terrain warnings") {
    RealWorld w;
    if (!w.open()) return;
    Simulation sim(w.paths, w.nav, *w.svc);
    world::registerAirportFlatten(*w.nav.findAirport("KOTH"), *w.svc);
    sim.setTerrainPump([&](double la, double lo) { w.svc->update(la, lo); });
    Scenario sc;
    std::string err;
    REQUIRE_MESSAGE(parseScenario("[scenario]\nairport = KOTH\nrunway = 05\nstart = final\n"
                                  "distance_nm = 6\napproach = I05\n",
                                  sc, err),
                    err);
    REQUIRE_MESSAGE(sim.init(sc, "", err), err);
    const Runway* rwy = findRunway(*w.nav.findAirport("KOTH"), "05");
    const double padFt = 0.5 * (rwy->thrElevFt + rwy->endElevFt);
    const double pitchTrimDeg = radToDeg(sim.aircraft().pitchRad);
    double throttleI = sim.pilot().throttleNorm[0];
    GpwsMonitor mon;
    std::uint32_t seen = 0;
    int callouts = 0;
    std::string seq;
    for (long i = 0; i < 6 * 60 * kSimHz && !sim.aircraft().crashed; ++i) {
        const AircraftState& s = sim.aircraft();
        if (s.wowAny || s.altMslFt - padFt < 25.0) break;
        // Simple 3 degree path follower along the runway extended centerline (as the F3 test).
        const LatLon a{rwy->thrLatDeg, rwy->thrLonDeg};
        double lat2 = 0, lon2 = 0;
        destinationPoint(a.latDeg, a.lonDeg, degToRad(rwy->headingTrueDeg), 1000.0, lat2, lon2);
        const double xtkM = crossTrackM({s.latDeg, s.lonDeg}, a, {lat2, lon2});
        const double beforeM = -alongTrackM({s.latDeg, s.lonDeg}, a, {lat2, lon2});
        const double trackTarget = rwy->headingTrueDeg - std::clamp(xtkM * 0.08, -15.0, 15.0);
        const double bank =
            std::clamp(0.6 * wrap180(trackTarget - radToDeg(s.trackTrueRad)), -15.0, 15.0);
        sim.pilot().aileronNorm = bankHoldAileron(s, bank);
        const double pathFt =
            padFt + 50.0 + std::max(beforeM, 0.0) * kFtPerM * std::tan(degToRad(3.0));
        const double vsTarget = std::clamp(
            -s.gsKt * 101.3 * std::tan(degToRad(3.0)) - 3.0 * (s.altMslFt - pathFt), -700.0, 0.0);
        sim.pilot().elevatorNorm = pitchHoldElevator(
            s, pitchTrimDeg + std::clamp(0.012 * (vsTarget - s.vsFpm), -4.0, 4.0), 0.08, 0.06);
        throttleI = std::clamp(throttleI + 0.0004 * (65.0 - s.iasKt) * kSimDtS, 0.1, 0.9);
        sim.pilot().throttleNorm[0] =
            static_cast<float>(std::clamp(throttleI + 0.03 * (65.0 - s.iasKt), 0.0, 1.0));
        sim.stepOnce();
        mon.update(sim.aircraft(), sim.profile(), &sim.navState(), *w.svc, &w.nav);
        seen |= mon.warnings();
        for (GpwsEvent e; mon.consumeEvent(e);)
            if (e.kind == GpwsEventKind::Callout) ++callouts, seq += std::string(e.text) + " ";
    }
    std::printf("[gpws real] KOTH final: warnings 0x%x, callouts: %s\n", seen, seq.c_str());
    CHECK((seen & (warningBit(WarningId::Terrain) | warningBit(WarningId::PullUp))) == 0);
    CHECK(callouts >= 2);
}
