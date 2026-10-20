// WP F4: the N6 autopilot checks (90 degree heading change, ILS from 10 NM) for the extended fleet.
// Same flights and pass bands as tests/autopilot/AutopilotFlightTest.cpp, run through Simulation
// (scenario -> autopilot -> flight model -> navigator). Numbers go to the README table through
// "FLEET <id> ap_* <value>" lines.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "SimFixture.h"
#include "core/Units.h"

using namespace fsim;
using namespace fsim::test;

namespace {

struct ApCase {
    const char* id;
    bool jet;
    double hdgAltFt;  // altitude of the heading change flight
    double vappKt;    // approach speed bug
    double armAtS;    // when APR is armed on the ILS flight
    double descentAltFt, descentVsFpm;
    double interceptHdgMagDeg;  // heading bug when APR is armed
};

constexpr ApCase kCases[] = {
    {"A320", true, 10000, 173, 30, 4400, -1500, 250},
    {"787-8", true, 10000, 165, 30, 4400, -1500, 250},
    {"MD11", true, 10000, 135, 30, 4600, -1500, 250},
    {"c182", false, 4500, 70, 45, 4800, -1000, 250},
    {"pa28", false, 4500, 80, 45, 4800, -1000, 250},
};

double headingDeg(const AircraftState& s) {
    return wrap360(radToDeg(s.headingTrueRad));
}

void hdg90(const ApCase& c) {
    SimFixture f;
    const std::string scenario =
        std::string("[scenario]\naircraft = ") + c.id +
        "\nairport = XTST\nrunway = 27\nstart = cruise\naltitude_ft = " +
        std::to_string(c.hdgAltFt) + "\n[wind]\nfrom_true_deg = 0\nspeed_kt = 0\n[autopilot]\n" +
        (c.jet ? "engage = ap at hdg alt\n" : "engage = ap hdg alt\n") +
        "hdg_bug_mag_deg = 360\nalt_bug_ft = " + std::to_string(c.hdgAltFt) + "\n";
    REQUIRE_MESSAGE(f.initText(scenario), f.error);
    double maxOvershootDeg = 0, minAltFt = 1e9, maxAltFt = 0;
    for (long i = 0; i < 120 * kSimHz; ++i) {
        f.sim.stepOnce();
        const AircraftState& s = f.sim.aircraft();
        if (s.simTimeS > 20) maxOvershootDeg = std::max(maxOvershootDeg, wrap180(headingDeg(s)));
        minAltFt = std::min(minAltFt, s.altMslFt);
        maxAltFt = std::max(maxAltFt, s.altMslFt);
    }
    const AircraftState& s = f.sim.aircraft();
    std::printf("FLEET %s ap_hdg90_overshoot_deg %.1f\n", c.id, maxOvershootDeg);
    std::printf("FLEET %s ap_hdg90_alt_dev_ft %.0f\n", c.id,
                std::max(maxAltFt - c.hdgAltFt, c.hdgAltFt - minAltFt));
    CHECK(std::fabs(wrap180(headingDeg(s))) < 1.0);
    CHECK(maxOvershootDeg < 5.0);
    CHECK(minAltFt > c.hdgAltFt - 100);
    CHECK(maxAltFt < c.hdgAltFt + 100);
    CHECK(f.sim.autopilotState().apEngaged);
    CHECK_FALSE(s.crashed);
}

void ils(const ApCase& c) {
    SimFixture f;
    // 10 NM final, 10 kt crosswind. The AP first descends in VS mode on heading 290 (drifting off
    // the centerline), then the heading bug goes to 250 and APR is armed: LOC captures on the
    // intercept and GS from below.
    const std::string scenario =
        std::string("[scenario]\naircraft = ") + c.id +
        "\nairport = XTST\nrunway = 27\nstart = final\ndistance_nm = 10\napproach = I27\n"
        "[wind]\nfrom_true_deg = 360\nspeed_kt = 10\n[autopilot]\nengage = ap at hdg vs\n"
        "hdg_bug_mag_deg = 290\nalt_bug_ft = " +
        std::to_string(c.descentAltFt) + "\nvs_bug_fpm = " + std::to_string(c.descentVsFpm) +
        "\nspd_bug_kt = " + std::to_string(c.vappKt) + "\n";
    REQUIRE_MESSAGE(f.initText(scenario), f.error);
    Autopilot& ap = f.sim.autopilot();
    double locCaptureS = -1, gsCaptureS = -1, maxCdi = 0, maxGs = 0, lowestAglFt = 1e9;
    double aglAtDisconnect = -1;
    bool armed = false;
    for (long i = 0; i < 700 * kSimHz; ++i) {
        f.sim.stepOnce();
        const AircraftState& s = f.sim.aircraft();
        const NavState& n = f.sim.navState();
        const AutopilotState& st = f.sim.autopilotState();
        if (!armed && s.simTimeS >= c.armAtS) {
            ap.setHdgBugMagDeg(c.interceptHdgMagDeg);
            ap.selectApr(n);
            armed = true;
        }
        if (st.latActive == LatMode::Loc && locCaptureS < 0) locCaptureS = s.simTimeS;
        if (st.vertActive == VertMode::Gs && gsCaptureS < 0) gsCaptureS = s.simTimeS;
        if (!st.apEngaged && armed && aglAtDisconnect < 0 && s.simTimeS > 30) {
            aglAtDisconnect = s.altAglFt;
            break;
        }
        if (st.vertActive == VertMode::Gs && s.altAglFt >= 200) {
            maxGs = std::max(maxGs, static_cast<double>(std::fabs(n.gsDeflection)));
            lowestAglFt = std::min(lowestAglFt, static_cast<double>(s.altAglFt));
        }
        if (st.latActive == LatMode::Loc && s.altAglFt >= 200)
            maxCdi = std::max(maxCdi, static_cast<double>(std::fabs(n.cdiDeflection)));
        if (s.crashed) break;
    }
    std::printf("FLEET %s ap_ils_loc_capture_s %.0f\n", c.id, locCaptureS);
    std::printf("FLEET %s ap_ils_gs_capture_s %.0f\n", c.id, gsCaptureS);
    std::printf("FLEET %s ap_ils_max_cdi_dots %.2f\n", c.id, maxCdi * 2.0);
    std::printf("FLEET %s ap_ils_max_gs_dots %.2f\n", c.id, maxGs * 2.0);
    std::printf("FLEET %s ap_ils_disconnect_agl_ft %.0f\n", c.id, aglAtDisconnect);
    CHECK(locCaptureS > 0);
    CHECK(gsCaptureS >= locCaptureS);
    CHECK(maxCdi <= 0.5);  // within 1 dot after LOC capture, down to 200 ft AGL
    CHECK(maxGs <= 0.5);
    CHECK(lowestAglFt < 215.0);      // still coupled at 200 ft AGL
    CHECK(aglAtDisconnect > 100.0);  // minimums - 50 ft disconnect (DA = threshold + 200 ft)
    CHECK(aglAtDisconnect < 160.0);
}

}  // namespace

#define FLEET_AP_CASES(ID, IDX)                                                                    \
    TEST_CASE("sim: fleet " ID " autopilot 90 degree heading change") {                            \
        hdg90(kCases[IDX]);                                                                        \
    }                                                                                              \
    TEST_CASE("sim: fleet " ID " autopilot ILS from 10 NM") {                                      \
        ils(kCases[IDX]);                                                                          \
    }

FLEET_AP_CASES("A320", 0)
FLEET_AP_CASES("787-8", 1)
FLEET_AP_CASES("MD11", 2)
FLEET_AP_CASES("c182", 3)
FLEET_AP_CASES("pa28", 4)
