// F6: fuel load and planning. FuelPlan logic, the JSBSim tank load at the start (default, full,
// a fraction, pounds, AUTO from the plan), fuel flow in flight and the HUD endurance.
#include <doctest/doctest.h>

#include <cstring>
#include <string>

#include "SimFixture.h"
#include "core/ControlInputs.h"
#include "hud/HudLogic.h"
#include "nav/FlightPlan.h"
#include "sim/FuelPlan.h"

using namespace fsim;
using namespace fsim::test;

TEST_CASE("fuel: settings parse and print back") {
    FuelSetting s;
    REQUIRE(parseFuelSetting("FULL", s));
    CHECK(s.kind == FuelSetting::Kind::Full);
    REQUIRE(parseFuelSetting("50%", s));
    CHECK(s.kind == FuelSetting::Kind::Fraction);
    CHECK(s.value == doctest::Approx(0.5f));
    CHECK(fuelSettingText(s) == "50%");
    REQUIRE(parseFuelSetting(" 3000 ", s));
    CHECK(s.kind == FuelSetting::Kind::Lbs);
    CHECK(s.value == doctest::Approx(3000.f));
    REQUIRE(parseFuelSetting("", s));
    CHECK(s.kind == FuelSetting::Kind::Default);
    REQUIRE(parseFuelSetting("auto", s));
    CHECK(fuelSettingText(s) == "auto");
    for (const char* bad : {"half", "150%", "-5", "0", "12abc"})
        CHECK_FALSE(parseFuelSetting(bad, s));
}

TEST_CASE("fuel: AUTO plans trip + 10 % + 45 min; endurance and fuel at destination") {
    // 160 NM at 120 kt and 100 lb/h: trip 133.3 lb, +10 % = 146.7, reserve 75 lb.
    CHECK(plannedFuelLbs(160, 120, 100) == doctest::Approx(221.7f).epsilon(0.001));
    CHECK(plannedFuelLbs(0, 120, 100) == 0);
    CHECK(plannedFuelLbs(160, 120, 0) == 0);
    CHECK(enduranceMin(100, 60) == doctest::Approx(100.f));
    CHECK(enduranceMin(100, 0) >= 6000.f);
    CHECK(fuelAtDestinationLbs(500, 120, 100, 100) == doctest::Approx(380.f));
    char b[16];
    formatEndurance(b, sizeof b, 125.4f);
    CHECK(std::string(b) == "2:05");
    formatEndurance(b, sizeof b, 9000.f);
    CHECK(std::string(b) == "-");
}

TEST_CASE("fuel: LOW FUEL fires once below 30 min and once below 15, only airborne") {
    LowFuelMonitor m;
    CHECK(m.update(20, false) == 0);  // on the ground
    CHECK(m.update(40, true) == 0);
    CHECK(m.update(29, true) == 30);
    CHECK(m.update(25, true) == 0);
    CHECK(m.update(14, true) == 15);
    CHECK(m.update(10, true) == 0);
    m.reset();
    CHECK(m.update(10, true) == 15);  // straight past both: one message
    CHECK(m.update(29, true) == 0);
}

TEST_CASE("sim: fuel load at the start follows the setting (c172p, 737)") {
    for (const std::string ac : {"c172p", "737"}) {
        CAPTURE(ac);
        auto loaded = [&](const std::string& fuel) {
            SimFixture f;
            const std::string sc = "[scenario]\naircraft = " + ac +
                                   "\nairport = XTST\nrunway = 27\nstart = runway\n" +
                                   (fuel.empty() ? "" : "fuel = " + fuel + "\n");
            REQUIRE_MESSAGE(f.initText(sc), f.error);
            return f.sim.aircraft();
        };
        const AircraftState def = loaded("");
        REQUIRE(def.fuelCapacityLbs > 0);
        const AircraftState full = loaded("full");
        CHECK(full.fuelTotalLbs == doctest::Approx(full.fuelCapacityLbs).epsilon(0.01));
        const AircraftState half = loaded("50%");
        CHECK(half.fuelTotalLbs == doctest::Approx(0.5f * half.fuelCapacityLbs).epsilon(0.01));
        const float wantLbs = ac == "737" ? 3000.f : 100.f;  // a realistic small load each
        const AircraftState lbs = loaded(std::to_string(static_cast<int>(wantLbs)));
        CHECK(lbs.fuelTotalLbs == doctest::Approx(wantLbs).epsilon(0.02));
        CHECK(def.fuelTotalLbs != doctest::Approx(full.fuelTotalLbs).epsilon(0.001));
    }
}

TEST_CASE("sim: AUTO fuel carries the plan's trip and reserve; the flow shows in flight") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText("[scenario]\naircraft = c172p\nairport = XTST\nrunway = 27\n"
                               "start = cruise\nplan = assets/plans/XTST-lnav.fms\nfuel = auto\n"),
                    f.error);
    const AircraftProfile& p = f.sim.profile();
    REQUIRE(p.cruiseFuelFlowLbsHr > 0);
    const double tripNm = planDistNm(f.sim.navigator().plan());
    const float tas = p.cruiseKt * (1.0f + 0.02f * p.cruiseAltFt / 1000.0f);
    const float expect = plannedFuelLbs(tripNm, tas, p.cruiseFuelFlowLbsHr);
    MESSAGE("AUTO: " << tripNm << " NM, " << f.sim.aircraft().fuelTotalLbs << " lb on board");
    CHECK(f.sim.aircraft().fuelTotalLbs == doctest::Approx(expect).epsilon(0.03));
    f.run(5);
    const AircraftState& s = f.sim.aircraft();
    CHECK(s.fuelFlowLbsPerHr > 0.3f * p.cruiseFuelFlowLbsHr);
    CHECK(s.fuelFlowLbsPerHr < 3.0f * p.cruiseFuelFlowLbsHr);
}

TEST_CASE("fuel: HUD row 2 shows the endurance, amber under 30 min, red under 15") {
    AircraftState s;
    s.valid = true;
    s.fuelTotalLbs = 50;
    s.fuelFlowLbsPerHr = 60;  // 50 min
    ControlInputs c;
    AircraftProfile p;
    StatusInputs in;
    in.state = &s;
    in.controls = &c;
    in.profile = &p;
    StatusField f[kMaxStatusFields];
    auto fuelText = [&](Role& role) {
        buildStatusRow2(in, f);
        std::string t;
        role = Role::Normal;
        for (int i = 0; i < f[kFuel].segCount; ++i) {
            if (i > 0) t += ' ';  // segments are drawn one cell apart
            t += f[kFuel].segs[i].text;
            if (i > 0) role = f[kFuel].segs[i].role;
        }
        return t;
    };
    Role role;
    CHECK(fuelText(role) == "FUEL 50 LB 0:50");
    CHECK(role == Role::Normal);
    s.fuelTotalLbs = 25;  // 25 min
    fuelText(role);
    CHECK(role == Role::Amber);
    s.fuelTotalLbs = 10;  // 10 min
    fuelText(role);
    CHECK(role == Role::Red);
    s.fuelFlowLbsPerHr = 0;  // engine off: no endurance shown
    CHECK(fuelText(role) == "FUEL 10 LB");
    s.fuelTotalLbs = 300;  // 3 h at 100 lb/h: plenty, not shown
    s.fuelFlowLbsPerHr = 100;
    CHECK(fuelText(role) == "FUEL 300 LB");
    // A five-digit load with the endurance still fits its slot, so WIND keeps its gap.
    s.fuelTotalLbs = 11740;
    s.fuelFlowLbsPerHr = 6000;
    CHECK(fuelText(role) == "FUEL 11 740 LB 1:57");
    CHECK(fieldCells10(f[kFuel]) <= f[kFuel].slotCells * 10);
}
