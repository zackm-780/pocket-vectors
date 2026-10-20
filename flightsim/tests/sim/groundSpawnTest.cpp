// WP F3: spawn gating and runway placement through Simulation::init.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdio>

#include "app/Simulation.h"
#include "core/FlatTerrain.h"
#include "groundSupport.h"

using namespace fsim;
using namespace fsim::test;
using fsim::terrain::testing::HeightFn;

namespace {

struct SimWorld {
    Paths paths = Paths::resolve({});
    SyntheticNavDb nav;
    std::string error;
};

HeightFn hills(double amplitudeM) {
    return [=](double lat, double lon) {
        return kXtstElevM + amplitudeM * std::sin(xtstEastM(lat, lon) / 24.0) *
                                std::cos(xtstNorthM(lat) / 24.0);
    };
}

bool initSim(Simulation& sim, const std::string& scenarioText, std::string& error) {
    Scenario sc;
    return parseScenario(scenarioText, sc, error) && sim.init(sc, "", error);
}

}  // namespace

TEST_CASE("ground: the spawn gate waits until the detail around the start is resident") {
    // The worker needs ~40 ms per tile; the start area spans several tiles.
    SynthWorld world(hills(5.0), nullptr, 0.15, 40, /*load=*/false);
    terrain::TerrainService& svc = world.terrain();
    const SpawnPoint p{SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg};
    REQUIRE_FALSE(svc.isDetailReady(p.latDeg, p.lonDeg));
    const SpawnGateResult r = waitForSpawnDetail(
        svc, {p}, 5000.0f, 10.0, [&](double lat, double lon) { svc.update(lat, lon); });
    std::printf("[ground] gate waited %.3f s (%d polls)\n", r.waitedS, r.polls);
    CHECK(r.ready);
    CHECK_FALSE(r.timedOut);
    CHECK(r.polls > 0);
    CHECK(r.waitedS > 0.03);
    CHECK(r.waitedS < 10.0);
    CHECK(svc.isDetailReady(p.latDeg, p.lonDeg));
}

TEST_CASE("ground: the spawn gate is bounded when detail never arrives") {
    NeverReadyTerrain never;
    const double t0 = nowS();
    int pumps = 0;
    const SpawnGateResult r = waitForSpawnDetail(
        never, {{40.0, -100.0}}, 5000.0f, 0.25, [&](double, double) { ++pumps; });
    const double elapsedS = nowS() - t0;
    CHECK_FALSE(r.ready);
    CHECK(r.timedOut);
    CHECK(elapsedS >= 0.25);
    CHECK(elapsedS < 0.6);
    CHECK(pumps > 10);
    CHECK(never.prefetches == 1);
    // Without a pump nothing could change, so it does not sit there waiting.
    const double t1 = nowS();
    const SpawnGateResult noPump = waitForSpawnDetail(never, {{40.0, -100.0}}, 5000.0f, 5.0, nullptr);
    CHECK(nowS() - t1 < 0.1);
    CHECK_FALSE(noPump.ready);
    CHECK_FALSE(noPump.timedOut);
    // Flat terrain is always ready: no waiting at all.
    FlatTerrain flat;
    const SpawnGateResult immediate =
        waitForSpawnDetail(flat, {{40.0, -100.0}}, 5000.0f, 5.0, [](double, double) {});
    CHECK(immediate.ready);
    CHECK(immediate.polls == 0);
}

TEST_CASE("sim: ground Simulation::init gates the start on terrain detail and sits on the pad") {
    SimWorld env;
    SynthWorld world(hills(6.0), nullptr, 0.15, 25, /*load=*/false);
    terrain::TerrainService& svc = world.terrain();
    Simulation sim(env.paths, env.nav, svc);
    sim.setTerrainPump([&](double lat, double lon) { svc.update(lat, lon); });
    REQUIRE_FALSE(svc.isDetailReady(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg));
    REQUIRE_MESSAGE(initSim(sim, "[scenario]\nstart = runway\nrunway = 27\n", env.error), env.error);
    const SpawnGateResult& gate = sim.spawnGate();
    std::printf("[ground] init waited %.3f s\n", gate.waitedS);
    CHECK(gate.ready);
    CHECK(gate.polls > 0);
    CHECK(svc.isDetailReady(sim.startCondition().latDeg, sim.startCondition().lonDeg));

    // The start sits on the flat runway pad (2,000 ft), not on a hill and not at sea level.
    CHECK(sim.startCondition().fieldElevFt == doctest::Approx(SyntheticNavDb::kElevFt).epsilon(1e-4));
    for (long i = 0; i < 6 * kSimHz; ++i) sim.stepOnce();
    const AircraftState& s = sim.aircraft();
    CHECK(s.wowAll);
    CHECK_FALSE(s.crashed);
    CHECK(s.terrainElevFt == doctest::Approx(SyntheticNavDb::kElevFt).epsilon(1e-3));
    CHECK(s.altAglFt == doctest::Approx(4.37).epsilon(0.05));
    CHECK(s.altMslFt == doctest::Approx(SyntheticNavDb::kElevFt + 4.37).epsilon(1e-3));
}

TEST_CASE("sim: ground Simulation::init starts anyway after the gate times out") {
    SimWorld env;
    NeverReadyTerrain never;
    Simulation sim(env.paths, env.nav, never);
    sim.setTerrainPump([](double, double) {});
    sim.setSpawnGateTimeoutS(0.2);
    const double t0 = nowS();
    REQUIRE_MESSAGE(initSim(sim, "[scenario]\nstart = runway\nrunway = 27\n", env.error), env.error);
    CHECK(nowS() - t0 < 5.0);
    CHECK(sim.spawnGate().timedOut);
    sim.stepOnce();
    CHECK(sim.aircraft().valid);
}

TEST_CASE("sim: ground a final start gates on both the start point and the runway") {
    SimWorld env;
    SynthWorld world(hills(4.0), nullptr, 0.15, 10, /*load=*/false);
    terrain::TerrainService& svc = world.terrain();
    Simulation sim(env.paths, env.nav, svc);
    sim.setTerrainPump([&](double lat, double lon) { svc.update(lat, lon); });
    REQUIRE_MESSAGE(initSim(sim, "[scenario]\nstart = final\nrunway = 27\ndistance_nm = 3\n", env.error),
                    env.error);
    const Runway* rwy = findRunway(env.nav.airport(), "27");
    CHECK(sim.spawnGate().ready);
    CHECK(svc.isDetailReady(rwy->thrLatDeg, rwy->thrLonDeg));
    CHECK(svc.isDetailReady(sim.startCondition().latDeg, sim.startCondition().lonDeg));
    // Hands-off for 40 s: still airborne, not crashed, ground below follows the terrain.
    for (long i = 0; i < 40 * kSimHz; ++i) sim.stepOnce();
    const AircraftState& s = sim.aircraft();
    CHECK_FALSE(s.crashed);
    CHECK(s.altAglFt > 100.0f);
    CHECK(s.altAglFt == doctest::Approx(s.altMslFt - s.terrainElevFt).epsilon(1e-3));
}

TEST_CASE("sim: ground a runway start takes off from the pad on rough terrain") {
    SimWorld env;
    SynthWorld world(hills(3.0));
    terrain::TerrainService& svc = world.terrain();
    Simulation sim(env.paths, env.nav, svc);
    REQUIRE_MESSAGE(initSim(sim,
                            "[scenario]\nstart = runway\nrunway = 27\n[inputs]\n0.0 = parkingbrake 0\n"
                            "0.0 = throttle 1.0\n",
                            env.error),
                    env.error);
    double liftoffKt = -1, maxHdgErr = 0;
    for (long i = 0; i < 40 * kSimHz && liftoffKt < 0; ++i) {
        // rudder and steering to hold the heading, rotate at 55 kt
        const AircraftState& s0 = sim.aircraft();
        sim.pilot().rudderNorm = headingHoldRudder(s0, 270.0);
        sim.pilot().steeringNorm = sim.pilot().rudderNorm;
        sim.pilot().aileronNorm = bankHoldAileron(s0);
        sim.pilot().elevatorNorm = s0.iasKt > 55.0f ? pitchHoldElevator(s0, 7.0, 0.5, 0.1) : 0.f;
        sim.stepOnce();
        const AircraftState& s = sim.aircraft();
        maxHdgErr = std::max(maxHdgErr, std::fabs(wrap180(radToDeg(s.headingTrueRad) - 270.0)));
        if (!s.wowAny) liftoffKt = s.iasKt;
    }
    std::printf("[ground] pad takeoff: liftoff %.1f kt, heading err %.1f deg\n", liftoffKt, maxHdgErr);
    CHECK(liftoffKt > 50.0);
    CHECK(liftoffKt < 65.0);
    CHECK(maxHdgErr < 5.0);
    CHECK_FALSE(sim.aircraft().crashed);
}

TEST_CASE("sim: ground the flat fallback world still works through the callback") {
    // No pack: FlatTerrain at the field elevation. Same results as the old default ground.
    SimWorld env;
    FlatTerrain flat{static_cast<float>(kXtstElevM)};
    Simulation sim(env.paths, env.nav, flat);
    REQUIRE_MESSAGE(initSim(sim, "[scenario]\nstart = runway\nrunway = 27\n", env.error), env.error);
    for (long i = 0; i < 5 * kSimHz; ++i) sim.stepOnce();
    CHECK(sim.aircraft().wowAll);
    CHECK(sim.aircraft().terrainElevFt == doctest::Approx(SyntheticNavDb::kElevFt).epsilon(1e-4));
    CHECK(sim.spawnGate().polls == 0);
}
