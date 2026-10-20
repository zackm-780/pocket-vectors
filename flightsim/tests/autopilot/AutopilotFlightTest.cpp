// N6 acceptance flights: the autopilot flying the real JSBSim model (hence the "sim:" prefix) in
// the c172p and the 737, through Simulation (scenario -> autopilot -> flight model -> navigator).
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

double headingDeg(const AircraftState& s) {
    return wrap360(radToDeg(s.headingTrueRad));
}

// Scenario text for a cruise start at XTST (heading 270) with the given autopilot section.
std::string cruiseScenario(const std::string& aircraft, double altFt, const std::string& extra,
                           const std::string& autopilot, const std::string& wind = "") {
    return "[scenario]\naircraft = " + aircraft +
           "\nairport = XTST\nrunway = 27\nstart = cruise\n" +
           "altitude_ft = " + std::to_string(altFt) + "\n" + extra + "\n[wind]\n" + wind +
           "\n[autopilot]\n" + autopilot;
}

}  // namespace

TEST_CASE("sim: autopilot 90 degree heading change, overshoot < 5 degrees (c172p, 737)") {
    for (const char* ac : {"c172p", "737"}) {
        SimFixture f;
        const bool jet = std::string(ac) == "737";
        const double altFt = jet ? 10000 : 4500;
        REQUIRE_MESSAGE(
            f.initText(cruiseScenario(
                ac, altFt, "",
                std::string(jet ? "engage = ap at hdg alt\n" : "engage = ap hdg alt\n") +
                    "hdg_bug_mag_deg = 360\nalt_bug_ft = " + std::to_string(altFt) + "\n")),
            f.error);
        double maxOvershootDeg = 0, minAltFt = 1e9, maxAltFt = 0;
        for (long i = 0; i < 120 * kSimHz; ++i) {
            f.sim.stepOnce();
            const AircraftState& s = f.sim.aircraft();
            // heading relative to the target 360 (= 0): overshoot is going past it to the right
            const double rel = wrap180(headingDeg(s) - 0.0);
            if (s.simTimeS > 20) maxOvershootDeg = std::max(maxOvershootDeg, rel);
            minAltFt = std::min(minAltFt, s.altMslFt);
            maxAltFt = std::max(maxAltFt, s.altMslFt);
        }
        const AircraftState& s = f.sim.aircraft();
        std::printf("[N6] %s hdg90: final hdg %.2f, max overshoot %.2f deg, alt %.0f..%.0f ft\n",
                    ac, headingDeg(s), maxOvershootDeg, minAltFt, maxAltFt);
        CHECK(std::fabs(wrap180(headingDeg(s))) < 1.0);
        CHECK(maxOvershootDeg < 5.0);
        CHECK(minAltFt > altFt - 100);
        CHECK(maxAltFt < altFt + 100);
        CHECK(f.sim.autopilotState().apEngaged);
        CHECK_FALSE(s.crashed);
    }
}

TEST_CASE(
    "sim: autopilot altitude capture, overshoot < 100 ft (c172p climb, 737 climb and descent)") {
    struct Case {
        const char* aircraft;
        double startFt, targetFt;
        const char* ap;
        double durationS;
    };
    const Case cases[] = {
        {"c172p", 4500, 6500, "engage = ap hdg alt\nhdg_bug_mag_deg = 270\n", 330},
        {"737", 10000, 12000, "engage = ap at hdg vs\nhdg_bug_mag_deg = 270\nvs_bug_fpm = 2000\n",
         150},
        {"737", 12000, 10000, "engage = ap at hdg vs\nhdg_bug_mag_deg = 270\nvs_bug_fpm = -1500\n",
         150},
    };
    for (const Case& c : cases) {
        SimFixture f;
        const std::string extra =
            std::string(c.ap) + "alt_bug_ft = " + std::to_string(c.targetFt) + "\n";
        const bool pistonCase = std::string(c.aircraft) == "c172p";
        REQUIRE_MESSAGE(
            f.initText(cruiseScenario(c.aircraft, c.startFt,
                                      pistonCase ? "[inputs]\n0.0 = throttle 0.85\n" : "",
                                      extra + "spd_bug_kt = 250\n")),
            f.error);
        const double dir = c.targetFt > c.startFt ? 1.0 : -1.0;
        double maxOvershootFt = -1e9;
        bool sawCapture = false;
        for (long i = 0; i < c.durationS * kSimHz; ++i) {
            f.sim.stepOnce();
            const AircraftState& s = f.sim.aircraft();
            maxOvershootFt = std::max(maxOvershootFt, dir * (s.altMslFt - c.targetFt));
            if (f.sim.autopilotState().vertActive == VertMode::AltCapture) sawCapture = true;
        }
        const AircraftState& s = f.sim.aircraft();
        std::printf("[N6] %s %.0f -> %.0f ft: max overshoot %.1f ft, final %.1f ft, vs %.0f\n",
                    c.aircraft, c.startFt, c.targetFt, maxOvershootFt, s.altMslFt, s.vsFpm);
        CHECK(sawCapture);
        CHECK(maxOvershootFt < 100.0);
        CHECK(std::fabs(s.altMslFt - c.targetFt) < 25.0);
        CHECK(f.sim.autopilotState().vertActive == VertMode::Alt);
        CHECK_FALSE(s.crashed);
    }
}

TEST_CASE("sim: autopilot ILS from 10 NM, within 1 dot of LOC and GS to 200 ft AGL (c172p, 737)") {
    for (const char* ac : {"c172p", "737"}) {
        SimFixture f;
        const bool jet = std::string(ac) == "737";
        // 10 NM final, 10 kt crosswind. The AP first descends to 4,800 ft (about 430 ft under the
        // path) in VS mode on heading 290 (drifting off the centerline); then the heading bug goes
        // to 250 and APR is armed: LOC captures on the intercept, GS from below.
        const std::string scenario =
            std::string("[scenario]\naircraft = ") + ac +
            "\nairport = XTST\nrunway = 27\nstart = final\ndistance_nm = 10\napproach = I27\n"
            "[inputs]\n" +
            "" + "[wind]\nfrom_true_deg = 360\nspeed_kt = 10\n[autopilot]\nengage = ap " +
            "at hdg vs\nhdg_bug_mag_deg = 290\nalt_bug_ft = " + (jet ? "4600" : "4800") +
            "\nvs_bug_fpm = " + (jet ? "-1500" : "-1000") + "\n" +
            (jet ? "spd_bug_kt = 145\n" : "spd_bug_kt = 65\n");
        REQUIRE_MESSAGE(f.initText(scenario), f.error);
        Autopilot& ap = f.sim.autopilot();
        const double armAtS = jet ? 30.0 : 45.0;

        double locCaptureS = -1, gsCaptureS = -1, maxCdi = 0, maxGs = 0, lowestAglFt = 1e9;
        double maxXtkAtCaptureNm = 0, aglAtDisconnect = -1;
        bool armed = false;
        for (long i = 0; i < 700 * kSimHz; ++i) {
            f.sim.stepOnce();
            const AircraftState& s = f.sim.aircraft();
            const NavState& n = f.sim.navState();
            const AutopilotState& st = f.sim.autopilotState();
            if (!armed && s.simTimeS >= armAtS) {
                ap.setHdgBugMagDeg(250);
                ap.selectApr(n);
                armed = true;
            }
            if (st.latActive == LatMode::Loc && locCaptureS < 0) {
                locCaptureS = s.simTimeS;
                maxXtkAtCaptureNm = std::fabs(n.crossTrackNm);
            }
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
        std::printf("[N6] %s ILS: LOC captured %.0f s (xtk %.2f NM), GS captured %.0f s, max |cdi| "
                    "%.3f, max "
                    "|gs| %.3f (1 dot = 0.5), AP off at %.0f ft AGL, lowest coupled %.0f ft AGL\n",
                    ac, locCaptureS, maxXtkAtCaptureNm, gsCaptureS, maxCdi, maxGs, aglAtDisconnect,
                    lowestAglFt);
        CHECK(locCaptureS > 0);
        CHECK(gsCaptureS >= locCaptureS);
        CHECK(maxCdi <= 0.5);  // within 1 dot after LOC capture, down to 200 ft AGL
        CHECK(maxGs <= 0.5);
        CHECK(lowestAglFt < 215.0);      // still coupled at 200 ft AGL
        CHECK(aglAtDisconnect > 100.0);  // minimums - 50 ft disconnect (DA = threshold + 200 ft)
        CHECK(aglAtDisconnect < 160.0);
    }
}

TEST_CASE("sim: autopilot LNAV through a six-leg plan, XTK < 0.3 NM after each intercept") {
    for (const char* ac : {"c172p", "737"}) {
        SimFixture f;
        const bool jet = std::string(ac) == "737";
        REQUIRE_MESSAGE(f.initFile(jet ? "ap_lnav_737.ini" : "ap_lnav_c172.ini"), f.error);
        int lastLeg = f.sim.navState().activeLegIndex;
        bool intercepted = false;
        double maxXtkNm = 0, maxAltDevFt = 0;
        int legsFlown = 0;
        const double altFt = jet ? 6000 : 4500;
        for (long i = 0; i < 1500 * kSimHz; ++i) {
            f.sim.stepOnce();
            const NavState& n = f.sim.navState();
            const AircraftState& s = f.sim.aircraft();
            if (n.activeLegIndex != lastLeg) {  // leg change: a new intercept starts
                lastLeg = n.activeLegIndex;
                intercepted = false;
                ++legsFlown;
            }
            if (!intercepted && std::fabs(n.crossTrackNm) < 0.1f) intercepted = true;
            if (intercepted && legsFlown <= 4) {
                maxXtkNm = std::max(maxXtkNm, static_cast<double>(std::fabs(n.crossTrackNm)));
                maxAltDevFt = std::max(maxAltDevFt, std::fabs(s.altMslFt - altFt));
            }
            if (legsFlown >= 5 || s.crashed) break;
        }
        std::printf(
            "[N6] %s LNAV: %d leg changes, max XTK after intercept %.3f NM, alt dev %.0f ft\n", ac,
            legsFlown, maxXtkNm, maxAltDevFt);
        CHECK(legsFlown >= 5);  // 5 legs flown (origin leg + 4 turns + the leg to the destination)
        CHECK(maxXtkNm < 0.3);
        CHECK(maxAltDevFt < 150.0);
        CHECK(f.sim.autopilotState().latActive == LatMode::Nav);
    }
}

TEST_CASE("sim: autopilot NAV arms off course and captures the leg on a closing intercept") {
    for (const char* ac : {"c172p", "737"}) {
        SimFixture f;
        const bool jet = std::string(ac) == "737";
        const std::string plan = "plan = assets/plans/XTST-lnav.fms\n";
        REQUIRE_MESSAGE(
            f.initText(cruiseScenario(
                ac, jet ? 6000 : 4500, plan,
                std::string(jet ? "engage = ap at hdg alt\n" : "engage = ap hdg alt\n") +
                    "hdg_bug_mag_deg = 360\nalt_bug_ft = " + (jet ? "6000" : "4500") +
                    "\nspd_bug_kt = 250\n")),
            f.error);
        Autopilot& ap = f.sim.autopilot();
        bool armed = false, captured = false;
        double captureXtkNm = 0, maxXtkAfterNm = 0, captureS = 0;
        int captureLeg = -1;
        long samples = 0;
        for (long i = 0; i < 600 * kSimHz; ++i) {
            f.sim.stepOnce();
            const AircraftState& s = f.sim.aircraft();
            const NavState& n = f.sim.navState();
            const AutopilotState& st = f.sim.autopilotState();
            if (!armed && s.simTimeS >= (jet ? 14.0 : 40.0)) {
                // well off the first leg, now heading back toward it (southwest)
                ap.setHdgBugMagDeg(jet ? 250 : 240);
                ap.selectNav(n);
                armed = true;
                REQUIRE(st.latArmed == LatMode::Nav);
            }
            if (armed && !captured && st.latActive == LatMode::Nav) {
                captured = true;
                captureS = s.simTimeS;
                captureXtkNm = std::fabs(n.crossTrackNm);
                captureLeg = n.activeLegIndex;
            }
            if (captured && s.simTimeS > captureS + (jet ? 3.0 : 30.0) &&
                n.activeLegIndex == captureLeg) {
                maxXtkAfterNm =
                    std::max(maxXtkAfterNm, static_cast<double>(std::fabs(n.crossTrackNm)));
                ++samples;
            }
        }
        std::printf(
            "[N6] %s NAV intercept: captured at %.0f s with XTK %.2f NM, later max XTK %.3f\n", ac,
            captureS, captureXtkNm, maxXtkAfterNm);
        CHECK(captured);
        CHECK(captureXtkNm > 0.1);    // a genuine intercept, not an on-course capture
        CHECK(samples > 3 * kSimHz);  // the check below is not vacuous
        CHECK(maxXtkAfterNm < 0.3);
    }
}

TEST_CASE("sim: AP-managed trim keeps a disconnect gentle (c172p)") {
    SimFixture f;
    REQUIRE_MESSAGE(f.initText(cruiseScenario("c172p", 4500, "[inputs]\n0.0 = throttle 0.85\n",
                                              "engage = ap hdg alt\nhdg_bug_mag_deg = 270\n"
                                              "alt_bug_ft = 6500\n")),
                    f.error);
    f.run(60);  // climbing at the default 500 fpm: the elevator load moves into trim
    const AircraftState before = f.sim.aircraft();
    const float elevCmd = f.sim.finalControls().elevatorNorm;
    CHECK(std::fabs(elevCmd) < 0.06f);  // trim carries the load, not the elevator integrator
    f.sim.autopilot().disengageAp(true);
    f.run(3);
    const AircraftState& after = f.sim.aircraft();
    CHECK(f.sim.autopilotState().disconnectWarning);
    CHECK(std::fabs(radToDeg(after.pitchRad - before.pitchRad)) < 3.0);
    CHECK(after.nzG > 0.8f);
    CHECK(after.nzG < 1.3f);
}
