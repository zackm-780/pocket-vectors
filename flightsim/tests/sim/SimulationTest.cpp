// Headless flight tests (they run the real JSBSim, so they carry the "sim:" prefix that ctest uses
// to group them).
#include <doctest/doctest.h>

#include <cmath>
#include <cstdio>
#include <initializer_list>

#include "SimFixture.h"
#include "app/Trace.h"
#include "core/Csv.h"

using namespace fsim;
using namespace fsim::test;

TEST_CASE("sim: smoke test flies 30 s from c172_runway_xtst.ini") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("c172_runway_xtst.ini"), f.error);

    const AircraftState& s0 = f.sim.aircraft();
    REQUIRE(s0.valid);
    CHECK(s0.wowAll);        // sitting on the runway
    CHECK(s0.parkingBrake);  // brakes set at start
    CHECK(s0.altAglFt == doctest::Approx(4.4f).epsilon(0.1));
    CHECK(s0.engines[0].running);
    CHECK(s0.iasKt == doctest::Approx(0.0f).epsilon(0.5));
    CHECK(radToDeg(s0.headingTrueRad) == doctest::Approx(270.0).epsilon(0.002));

    float maxIasKt = 0;
    for (long i = 0; i < 30 * kSimHz; ++i) {
        f.sim.stepOnce();
        maxIasKt = std::max(maxIasKt, f.sim.aircraft().iasKt);
    }
    const AircraftState& s = f.sim.aircraft();
    CHECK(s.valid);
    CHECK_FALSE(s.crashed);
    CHECK(s.simTimeS == doctest::Approx(30.0).epsilon(0.001));
    CHECK(maxIasKt > 55.0f);  // full throttle on the runway: IAS passes 55 kt
    CHECK(s.engines[0].throttleCmdNorm == doctest::Approx(1.0f));
    CHECK(std::isfinite(s.latDeg));
    CHECK(s.lonDeg < -99.98);    // rolled west along runway 27, not east
    CHECK(s.altMslFt > 2000.0);  // above the field
}

TEST_CASE("sim: runway start holds still with the parking brake and idle throttle") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText("[scenario]\nstart = runway\n"), f.error);
    f.run(10.0);
    const AircraftState& s = f.sim.aircraft();
    CHECK(s.gsKt < 1.0f);
    CHECK(s.wowAll);
    CHECK_FALSE(s.crashed);
}

TEST_CASE("sim: JSBSim control signs are converted to our conventions") {
    // Start trimmed in the air, then apply each control for a second and check the response.
    struct Case {
        const char* command;
        int which;  // 0 = pitch rate, 1 = roll rate, 2 = yaw rate
    };
    for (const Case c : {Case{"elevator 0.3", 0}, Case{"aileron 0.3", 1}, Case{"rudder 0.3", 2}}) {
        SimFixture f;
        const std::string scenario =
            std::string("[scenario]\nstart = cruise\naltitude_ft = 6000\n[inputs]\n1.0 = ") +
            c.command + "\n";
        REQUIRE_MESSAGE(f.initText(scenario), f.error);
        f.run(2.0);
        const AircraftState& s = f.sim.aircraft();
        const float rate = c.which == 0 ? s.qRadps : c.which == 1 ? s.pRadps : s.rRadps;
        INFO(c.command << " rate " << rate);
        CHECK(rate > 0.02f);  // + elevator = nose up, + aileron = roll right, + rudder = yaw right
    }
}

TEST_CASE("sim: c172p final start is trimmed and holds the 3 degree path hands-off") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("c172_final_xtst.ini"), f.error);
    const AircraftState& s0 = f.sim.aircraft();
    const StartCondition& sc = f.sim.startCondition();
    CHECK(sc.altMslFt == doctest::Approx(3960.7).epsilon(1e-4));
    CHECK(s0.iasKt == doctest::Approx(65.0f).epsilon(0.01));
    CHECK(s0.vsFpm == doctest::Approx(-365.0f).epsilon(0.1));        // 3 degrees at 65 KCAS
    CHECK(s0.flapPosNorm == doctest::Approx(0.667f).epsilon(0.01));  // approach flaps
    CHECK(s0.gearPosNorm == doctest::Approx(1.0f));
    CHECK_FALSE(s0.wowAny);

    // The trimmed elevator lives in the trim control; the stick is centered.
    const ControlInputs& c = f.sim.pilot();
    CHECK(c.elevatorNorm == doctest::Approx(0.0f));
    CHECK(c.aileronNorm == doctest::Approx(0.0f));
    CHECK(std::fabs(c.elevTrimNorm) > 0.01f);
    CHECK(c.elevTrimNorm > 0.0f);  // c172p needs nose-up trim at 65 kt, flaps 20
    CHECK(c.throttleNorm[0] > 0.2f);
    CHECK(c.throttleNorm[0] < 0.9f);
    CHECK(c.flapDetent == 2);
    CHECK(s0.elevTrimNorm == doctest::Approx(c.elevTrimNorm));

    double maxDevFt = 0;
    for (long i = 0; i < 60 * kSimHz; ++i) {
        f.sim.stepOnce();
        if (i % kSimHz == 0) {
            const AircraftState& s = f.sim.aircraft();
            const PathPoint p = glidepathAt(f.nav, s);
            maxDevFt = std::max(maxDevFt, std::fabs(s.altMslFt - p.pathAltFt));
        }
    }
    INFO("max deviation from the glidepath over 60 s: " << maxDevFt << " ft");
    CHECK(maxDevFt < 150.0);
    CHECK_FALSE(f.sim.aircraft().crashed);
    CHECK(f.sim.aircraft().iasKt == doctest::Approx(65.0f).epsilon(0.02));
}

TEST_CASE("sim: 737 final start (best effort) flies 60 s without crashing") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initFile("737_final_xtst.ini"), f.error);
    const AircraftState& s0 = f.sim.aircraft();
    CHECK(s0.engineCount == 2);
    CHECK(s0.gearPosNorm == doctest::Approx(1.0f));
    CHECK(s0.flapPosNorm > 0.7f);
    CHECK(s0.engines[0].running);
    CHECK(s0.engines[1].running);
    CHECK(s0.engines[0].n1Pct > 30.0f);
    f.run(60.0);
    const AircraftState& s = f.sim.aircraft();
    CHECK_FALSE(s.crashed);
    CHECK(s.iasKt == doctest::Approx(145.0f).epsilon(0.03));
    CHECK(s.vsFpm < -400.0f);  // descending on the glidepath
    CHECK(s.vsFpm > -1100.0f);
    const PathPoint p = glidepathAt(f.nav, s);
    CHECK(std::fabs(s.altMslFt - p.pathAltFt) < 400.0);
}

TEST_CASE("sim: wind from the scenario shows up in the state") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText("[scenario]\nstart = final\ndistance_nm = 3\n"
                               "[wind]\nfrom_true_deg = 270\nspeed_kt = 10\n"),
                    f.error);
    f.run(2.0);
    const AircraftState& s = f.sim.aircraft();
    CHECK(s.windKt == doctest::Approx(10.0f).epsilon(0.05));
    CHECK(s.windFromTrueDeg == doctest::Approx(270.0f).epsilon(0.02));
    CHECK(s.iasKt == doctest::Approx(65.0f).epsilon(0.02));
    // Flying west into a westerly: ground speed is about 10 kt below true airspeed.
    CHECK(s.gsKt < s.tasKt - 8.0f);
}

TEST_CASE("sim: the flight model is deterministic") {
    auto finalState = [] {
        SimFixture f;
        REQUIRE_MESSAGE(f.initFile("c172_runway_xtst.ini"), f.error);
        f.run(20.0);
        return f.sim.aircraft();
    };
    const AircraftState a = finalState(), b = finalState();
    CHECK(a.latDeg == b.latDeg);
    CHECK(a.lonDeg == b.lonDeg);
    CHECK(a.altMslFt == b.altMslFt);
    CHECK(a.iasKt == b.iasKt);
}

TEST_CASE("sim: gear-up is ignored while any wheel carries weight") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText("[scenario]\nstart = runway\n[inputs]\n1.0 = gear 0\n"), f.error);
    f.run(3.0);
    CHECK(f.sim.aircraft().wowAny);
    CHECK(f.sim.aircraft().gearPosNorm == doctest::Approx(1.0f));
}

TEST_CASE("sim: a hard impact sets the crashed flag and freezes the model") {
    SimFixture f;
    // Dive the c172p into the ground: full nose-down elevator in a cruise start at 400 ft AGL.
    REQUIRE_MESSAGE(f.initText("[scenario]\nstart = cruise\naltitude_ft = 2400\n"
                               "[inputs]\n0.0 = elevator -1.0\n0.0 = throttle 1.0\n"),
                    f.error);
    f.run(60.0);
    const AircraftState& s = f.sim.aircraft();
    CHECK(s.crashed);
    CHECK(std::string(s.crashReason).size() > 0);
    const double t = s.simTimeS;
    f.run(2.0);
    CHECK(f.sim.aircraft().simTimeS == doctest::Approx(t));  // frozen after the crash
}

TEST_CASE("sim: trace CSV has a header and one row per 0.1 s with sensible values") {
    const std::string path = "trace_test_output.csv";
    {
        SimFixture f;
        REQUIRE_MESSAGE(f.initFile("c172_runway_xtst.ini"), f.error);
        REQUIRE(f.sim.openTrace(path));
        f.run(5.0);
    }  // the fixture's destructor closes (flushes) the trace

    CsvReader csv;
    REQUIRE(csv.open(path));
    for (const char* col :
         {"t_s", "lat_deg", "lon_deg", "alt_msl_ft", "ias_kt", "gs_kt", "vs_fpm",
          "heading_true_deg", "pitch_deg", "roll_deg", "crashed", "nav_source", "ap_engaged"})
        CHECK_MESSAGE(csv.column(col) >= 0, col);

    int rows = 0;
    double lastT = -1, lastIas = 0;
    while (csv.next()) {
        const double t = csv.getDouble("t_s");
        CHECK(t > lastT);
        lastT = t;
        lastIas = csv.getDouble("ias_kt");
        CHECK(csv.getInt("crashed") == 0);
        ++rows;
    }
    CHECK(rows >= 49);
    CHECK(rows <= 52);
    CHECK(lastT == doctest::Approx(5.0).epsilon(0.03));
    CHECK(lastIas > 5.0);  // accelerating
    std::remove(path.c_str());
}
