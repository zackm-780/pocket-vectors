// Mode logic of the autopilot (arming, capture, reversion, disconnect rules, bug handling, A/T)
// driven with synthetic AircraftState / NavState: no JSBSim.
#include <doctest/doctest.h>

#include <cmath>

#include "autopilot/Autopilot.h"
#include "core/SimTime.h"
#include "core/Units.h"

using namespace fsim;

namespace {

struct Rig {
    Autopilot ap;
    AircraftProfile profile;
    AircraftState ac;
    NavState nav;
    ControlInputs pilot;

    explicit Rig(AircraftCategory cat = AircraftCategory::Piston, bool hasAt = false) {
        profile.id = "test";
        profile.category = cat;
        profile.hasAutothrottle = hasAt;
        profile.engineCount = 1;
        profile.vs1Kt = 45;
        profile.approachFlapDetent = 1;
        ap.configure(profile);
        ac.valid = true;
        ac.altMslFt = 5000;
        ac.altAglFt = 3000;
        ac.iasKt = ac.tasKt = ac.gsKt = 100;
        ac.engineCount = 1;
        ac.flapDetent = 0;
        nav.magVarDeg = 0;
        nav.source = NavSource::Gps;
        nav.activeLegIndex = 1;
    }
    void setHeadingDeg(double deg) {
        ac.headingTrueRad = static_cast<float>(degToRad(deg));
        ac.trackTrueRad = ac.headingTrueRad;
    }
    void run(double seconds) {
        const long n = static_cast<long>(seconds * kSimHz + 0.5);
        for (long i = 0; i < n; ++i) {
            ac.simTimeS += kSimDtS;
            out = ap.update(pilot, ac, nav, kSimDtS);
            ap.syncPilot(pilot);
        }
    }
    void engage() { REQUIRE(ap.engageAp(ac, nav)); }
    ControlInputs out;
};

ActionState press(Action a, int repeats = 0) {
    ActionState s;
    s.pressed[static_cast<int>(a)] = true;
    s.down[static_cast<int>(a)] = true;
    s.repeatCount[static_cast<int>(a)] = repeats;
    return s;
}

}  // namespace

TEST_CASE("autopilot: engage with no mode selected gives ROL + PIT, refuses on the ground") {
    Rig r;
    r.ac.wowAny = true;
    CHECK_FALSE(r.ap.engageAp(r.ac, r.nav));
    CHECK_FALSE(r.ap.state().apEngaged);
    r.ac.wowAny = false;
    r.ac.rollRad = static_cast<float>(degToRad(3));
    r.engage();
    const AutopilotState& st = r.ap.state();
    CHECK(st.apEngaged);
    CHECK(st.latActive == LatMode::Roll);
    CHECK(st.vertActive == VertMode::Pitch);
    r.run(1.0);
    CHECK(r.ap.state().fdValid);
    CHECK(r.ap.state().fdRollCmdDeg == doctest::Approx(0.f));  // <= 6 deg: wings level
}

TEST_CASE("autopilot: ROL holds a bank above 6 degrees") {
    Rig r;
    r.ac.rollRad = static_cast<float>(degToRad(12));
    r.engage();
    r.run(0.5);
    CHECK(r.ap.state().fdRollCmdDeg == doctest::Approx(12.f).epsilon(0.01));
}

TEST_CASE("autopilot: HDG banks toward the bug, limited to 20 (light) and 25 (jet) degrees") {
    for (auto cat : {AircraftCategory::Piston, AircraftCategory::Jet}) {
        Rig r(cat);
        r.setHeadingDeg(0);
        r.engage();
        r.ap.setHdgBugMagDeg(90);
        r.ap.selectHdg(r.ac);
        r.run(0.5);
        const float limit = cat == AircraftCategory::Jet ? 25.f : 20.f;
        CHECK(r.ap.state().latActive == LatMode::Hdg);
        CHECK(r.ap.state().fdRollCmdDeg == doctest::Approx(limit));  // + = right
        r.ap.setHdgBugMagDeg(270);
        r.run(0.5);
        CHECK(r.ap.state().fdRollCmdDeg == doctest::Approx(-limit));
        r.ap.setHdgBugMagDeg(3);  // small error: proportional, not saturated
        r.run(0.5);
        CHECK(std::fabs(r.ap.state().fdRollCmdDeg) < limit);
        CHECK(r.ap.state().fdRollCmdDeg > 0.f);
    }
}

TEST_CASE("autopilot: heading bug is magnetic, converted with the magnetic variation") {
    Rig r;
    r.nav.magVarDeg = 15;  // east: true = magnetic + 15
    r.setHeadingDeg(105);  // exactly on the bug
    r.engage();
    r.ap.setHdgBugMagDeg(90);
    r.ap.selectHdg(r.ac);
    r.run(0.5);
    CHECK(std::fabs(r.ap.state().fdRollCmdDeg) < 0.5f);
    r.ap.syncBugs(r.ac, r.nav);
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(90.f));
}

TEST_CASE("autopilot: pressing HDG again with HDG active returns to ROL") {
    Rig r;
    r.engage();
    r.ap.selectHdg(r.ac);
    CHECK(r.ap.state().latActive == LatMode::Hdg);
    r.ap.selectHdg(r.ac);
    CHECK(r.ap.state().latActive == LatMode::Roll);
}

TEST_CASE("autopilot: NAV arms off course and captures on a closing intercept") {
    Rig r;
    r.setHeadingDeg(0);
    r.nav.desiredTrackTrueDeg = 0;
    r.nav.crossTrackNm = 2.0f;  // 2 NM right of course
    r.engage();
    r.ap.selectNav(r.nav);
    r.run(0.2);
    CHECK(r.ap.state().latArmed == LatMode::Nav);
    CHECK(r.ap.state().latActive == LatMode::Roll);  // parallel to the course: stays armed

    r.ac.trackTrueRad = static_cast<float>(degToRad(30));  // diverging
    r.nav.crossTrackNm = 0.15f;
    r.run(0.2);
    CHECK(r.ap.state().latArmed == LatMode::Nav);

    r.ac.trackTrueRad = static_cast<float>(degToRad(330));  // closing, inside the lead distance
    r.run(0.2);
    CHECK(r.ap.state().latActive == LatMode::Nav);
    CHECK(r.ap.state().latArmed == LatMode::Off);
}

TEST_CASE("autopilot: NAV on course captures at once, NAV without an active leg is ignored") {
    Rig r;
    r.nav.desiredTrackTrueDeg = 0;
    r.nav.crossTrackNm = 0.02f;
    r.engage();
    r.ap.selectNav(r.nav);
    r.run(0.1);
    CHECK(r.ap.state().latActive == LatMode::Nav);

    Rig q;
    q.nav.source = NavSource::None;
    q.nav.activeLegIndex = -1;
    q.engage();
    q.ap.selectNav(q.nav);
    CHECK(q.ap.state().latArmed == LatMode::Off);
}

TEST_CASE("autopilot: NAV steers toward the course: right of course turns left") {
    Rig r;
    r.setHeadingDeg(0);
    r.nav.desiredTrackTrueDeg = 0;
    r.nav.crossTrackNm = 0.05f;
    r.engage();
    r.ap.selectNav(r.nav);
    r.run(0.1);
    REQUIRE(r.ap.state().latActive == LatMode::Nav);
    r.nav.crossTrackNm = 0.8f;  // right of course
    r.run(0.1);
    CHECK(r.ap.state().fdRollCmdDeg < -1.f);
    r.nav.crossTrackNm = -0.8f;  // left of course
    r.run(0.1);
    CHECK(r.ap.state().fdRollCmdDeg > 1.f);
}

TEST_CASE("autopilot: NAV drops to ROL when guidance disappears") {
    Rig r;
    r.nav.crossTrackNm = 0.01f;
    r.engage();
    r.ap.selectNav(r.nav);
    r.run(0.1);
    REQUIRE(r.ap.state().latActive == LatMode::Nav);
    r.nav.source = NavSource::None;
    r.nav.activeLegIndex = -1;
    r.run(0.1);
    CHECK(r.ap.state().latActive == LatMode::Roll);
}

namespace {
void setIls(Rig& r, float cdi, float gsDefl, bool gsValid = true) {
    r.nav.source = NavSource::Loc;
    r.nav.locValid = true;
    r.nav.gsValid = gsValid;
    r.nav.cdiDeflection = cdi;
    r.nav.gsDeflection = gsDefl;
    r.nav.crossTrackNm = cdi * 0.2f;
    r.nav.desiredTrackTrueDeg = 0;
    std::snprintf(r.nav.approachName, sizeof r.nav.approachName, "ILS 27");
    r.nav.phase = NavPhase::Approach;
}
}  // namespace

TEST_CASE("autopilot: APR arms LOC and GS, LOC needs the needle inside full scale") {
    Rig r;
    setIls(r, 1.1f, 0.0f);
    r.setHeadingDeg(0);
    r.engage();
    r.ap.selectApr(r.nav);
    CHECK(r.ap.state().latArmed == LatMode::Loc);
    CHECK(r.ap.state().vertArmed == VertMode::Gs);
    r.run(0.2);
    CHECK(r.ap.state().latActive == LatMode::Roll);  // cdi 1.1 > 1.0

    r.nav.cdiDeflection = 0.6f;
    r.nav.crossTrackNm = 0.05f;
    r.run(0.2);
    CHECK(r.ap.state().latActive == LatMode::Loc);
    CHECK(r.ap.state().latArmed == LatMode::Off);
}

TEST_CASE("autopilot: GS captures only after LOC, from below or within one dot") {
    Rig r;
    setIls(r, 0.0f, 0.9f);  // glidepath above the aircraft by 1.8 dots
    r.engage();
    r.ap.selectApr(r.nav);
    r.run(0.2);
    REQUIRE(r.ap.state().latActive == LatMode::Loc);
    CHECK(r.ap.state().vertActive != VertMode::Gs);
    CHECK(r.ap.state().vertArmed == VertMode::Gs);

    r.nav.gsDeflection = -0.8f;  // aircraft above the path by 1.6 dots: no capture
    r.run(0.2);
    CHECK(r.ap.state().vertActive != VertMode::Gs);

    r.nav.gsDeflection = 0.45f;  // path comes down to within a dot
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Gs);
    CHECK(r.ap.state().vertArmed == VertMode::Off);
}

TEST_CASE("autopilot: GS does not capture while LOC is still armed") {
    Rig r;
    setIls(r, 0.9f, 0.0f);
    r.ac.trackTrueRad = static_cast<float>(degToRad(45));  // diverging: LOC will not capture
    r.nav.crossTrackNm = 0.8f;
    r.engage();
    r.ap.selectApr(r.nav);
    r.run(0.2);
    CHECK(r.ap.state().latActive != LatMode::Loc);
    CHECK(r.ap.state().vertActive != VertMode::Gs);
}

TEST_CASE("autopilot: loss of LOC or GS reverts the modes; missed approach flies NAV") {
    Rig r;
    setIls(r, 0.0f, 0.0f);
    r.engage();
    r.ap.selectApr(r.nav);
    r.run(0.2);
    REQUIRE(r.ap.state().latActive == LatMode::Loc);
    REQUIRE(r.ap.state().vertActive == VertMode::Gs);
    r.nav.gsValid = false;
    r.run(0.1);
    CHECK(r.ap.state().vertActive == VertMode::Pitch);

    Rig q;
    setIls(q, 0.0f, 0.0f);
    q.engage();
    q.ap.selectApr(q.nav);
    q.run(0.2);
    REQUIRE(q.ap.state().latActive == LatMode::Loc);
    q.nav.source = NavSource::Gps;
    q.nav.locValid = false;
    q.nav.phase = NavPhase::Missed;
    q.run(0.1);
    CHECK(q.ap.state().latActive == LatMode::Nav);
    CHECK(q.ap.state().vertActive != VertMode::Gs);
}

TEST_CASE("autopilot: APR on a non-ILS approach arms NAV (GPS), pressing APR again cancels") {
    Rig r;
    std::snprintf(r.nav.approachName, sizeof r.nav.approachName, "RNAV 27");
    r.nav.crossTrackNm = 1.5f;
    r.engage();
    r.ap.selectApr(r.nav);
    CHECK(r.ap.state().latArmed == LatMode::Nav);
    CHECK(r.ap.state().vertArmed == VertMode::Off);

    Rig q;
    setIls(q, 0.9f, 0.9f);
    q.nav.crossTrackNm = 1.5f;
    q.engage();
    q.ap.selectApr(q.nav);
    REQUIRE(q.ap.state().latArmed == LatMode::Loc);
    q.ap.selectApr(q.nav);
    CHECK(q.ap.state().latArmed == LatMode::Off);
    CHECK(q.ap.state().vertArmed == VertMode::Off);
}

TEST_CASE("autopilot: VS climb arms ALT, captures near the bug and settles in ALT") {
    Rig r;
    r.ac.vsFpm = 1000;
    r.engage();
    r.ap.setAltBugFt(6000);
    r.ap.setVsBugFpm(1000);
    r.ap.selectVs(r.ac);
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Vs);
    CHECK(r.ap.state().vertArmed == VertMode::Alt);

    r.ac.altMslFt = 5900;  // 100 ft to go; capture distance at 1000 fpm is max(100, 167) ft
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::AltCapture);
    CHECK(r.ap.state().vertArmed == VertMode::Off);

    r.ac.altMslFt = 5990;
    r.ac.vsFpm = 200;
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Alt);
}

TEST_CASE("autopilot: ALT capture starts at |VS|/6 feet (min 100) and only toward the bug") {
    Rig r(AircraftCategory::Jet);
    r.ac.vsFpm = 2400;
    r.engage();
    r.ap.setAltBugFt(10000);
    r.ac.altMslFt = 9500;  // 500 ft: outside 2400/6 = 400
    r.ap.setVsBugFpm(2400);
    r.ap.selectVs(r.ac);
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Vs);
    r.ac.altMslFt = 9620;  // 380 ft: inside
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::AltCapture);

    Rig q;  // climbing away from a bug below: never captures
    q.ac.vsFpm = 800;
    q.engage();
    q.ap.setAltBugFt(4900);
    q.ap.setVsBugFpm(800);
    q.ap.selectVs(q.ac);
    q.run(0.2);
    CHECK(q.ap.state().vertActive == VertMode::Vs);
    CHECK(q.ap.state().vertArmed == VertMode::Off);
}

TEST_CASE("autopilot: PIT with the bug above and climbing arms and captures ALT*") {
    Rig r;
    r.ac.vsFpm = 600;
    r.ac.altMslFt = 5000;
    r.engage();
    r.ap.setAltBugFt(5500);
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Pitch);
    CHECK(r.ap.state().vertArmed == VertMode::Alt);
    r.ac.altMslFt = 5450;
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::AltCapture);
}

TEST_CASE("autopilot: bug, capture and hold use the indicated altitude (altimeter 1 inHg low)") {
    Rig r;
    r.ac.altBaroOffsetFt = -910;  // setting 1 inHg too low reads about 910 ft low
    r.ac.altMslFt = 5000 + 910;   // true 5910, indicated 5000
    r.ac.vsFpm = 1000;
    r.engage();
    r.ap.setAltBugFt(6000);
    r.ap.setVsBugFpm(1000);
    r.ap.selectVs(r.ac);
    r.run(0.2);
    CHECK(r.ap.state().vertArmed == VertMode::Alt);  // 1000 ft to go on the tape

    r.ac.altMslFt = 5900 + 910;  // indicated 5900: capture
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::AltCapture);

    r.ac.altMslFt = 5990 + 910;
    r.ac.vsFpm = 200;
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Alt);
    // At the bug on the tape the hold asks for level flight, not the 910 ft the true altitude is off.
    r.ac.altMslFt = 6000 + 910;
    r.ac.vsFpm = 0;
    r.run(2.0);
    CHECK(std::fabs(r.ap.state().fdPitchCmdDeg) < 1.0f);
}

TEST_CASE("autopilot: ALT mode holds the present altitude; GS stays armed behind it") {
    Rig r;
    setIls(r, 0.9f, 0.9f);
    r.nav.crossTrackNm = 2.0f;
    r.engage();
    r.ap.selectApr(r.nav);
    r.ap.selectAltHold(r.ac);
    r.run(0.2);
    CHECK(r.ap.state().vertActive == VertMode::Alt);
    CHECK(r.ap.state().vertArmed == VertMode::Gs);
}

TEST_CASE("autopilot: VS mode picks a sensible initial rate") {
    Rig r;
    r.engage();
    r.ap.setAltBugFt(7000);
    r.ap.selectVs(r.ac);  // level, bug above
    CHECK(r.ap.state().vsBugFpm > 0.f);
    r.ap.setAltBugFt(3000);
    r.ap.selectVs(r.ac);  // level, bug below
    CHECK(r.ap.state().vsBugFpm < 0.f);
    r.ac.vsFpm = -730;
    r.ap.selectVs(r.ac);
    CHECK(r.ap.state().vsBugFpm == doctest::Approx(-700.f));
}

TEST_CASE("autopilot: FD works with the AP off and never drives the controls") {
    Rig r;
    r.ac.rollRad = static_cast<float>(degToRad(10));
    r.pilot.aileronNorm = 0.2f;
    r.pilot.elevatorNorm = -0.1f;
    r.ap.setFd(true, r.ac);
    r.ap.setHdgBugMagDeg(90);
    r.ap.selectHdg(r.ac);
    r.run(0.5);
    const AutopilotState& st = r.ap.state();
    CHECK(st.fdOn);
    CHECK_FALSE(st.apEngaged);
    CHECK(st.fdValid);
    CHECK(st.fdRollCmdDeg > 5.f);
    CHECK(r.out.aileronNorm == doctest::Approx(0.2f));
    CHECK(r.out.elevatorNorm == doctest::Approx(-0.1f));
    r.ap.setFd(false, r.ac);
    r.run(0.1);
    CHECK_FALSE(r.ap.state().fdValid);
    CHECK(r.ap.state().latActive == LatMode::Off);
}

TEST_CASE("autopilot: FD stays on after the AP disconnects, modes persist") {
    Rig r;
    r.ap.setFd(true, r.ac);
    r.engage();
    r.ap.selectHdg(r.ac);
    r.ap.disengageAp(true);
    CHECK(r.ap.state().latActive == LatMode::Hdg);
    CHECK(r.ap.state().fdOn);
}

TEST_CASE("autopilot: AP drives the surfaces with the right signs") {
    Rig r;
    r.setHeadingDeg(0);
    r.engage();
    r.ap.setHdgBugMagDeg(90);
    r.ap.selectHdg(r.ac);
    r.run(2.0);
    CHECK(r.out.aileronNorm > 0.05f);  // roll right
    r.ap.setHdgBugMagDeg(270);
    r.run(8.0);
    CHECK(r.out.aileronNorm < -0.05f);

    Rig q;  // below the pitch target: nose up (positive elevator)
    q.ac.pitchRad = 0;
    q.engage();
    q.ap.setAltBugFt(7000);
    q.ap.setVsBugFpm(1000);
    q.ap.selectVs(q.ac);
    q.run(1.0);
    CHECK(q.out.elevatorNorm > 0.f);
}

TEST_CASE("autopilot: sustained elevator is offloaded into trim, slowly") {
    Rig r;
    r.engage();
    r.ap.setAltBugFt(7000);
    r.ap.setVsBugFpm(1000);
    r.ap.selectVs(r.ac);
    r.run(0.1);
    const float trim0 = r.ap.apElevTrimNorm();
    float prev = trim0;
    for (int i = 0; i < 20; ++i) {
        r.run(1.0);  // the aircraft never reaches the pitch: a constant elevator demand
        CHECK(r.ap.apElevTrimNorm() >= prev - 1e-6f);
        CHECK(r.ap.apElevTrimNorm() - prev <= 0.021f);  // rate limited
        prev = r.ap.apElevTrimNorm();
    }
    CHECK(prev > trim0 + 0.02f);
    CHECK(r.pilot.elevTrimNorm == doctest::Approx(r.ap.apElevTrimNorm()));  // pilot follows
}

TEST_CASE("autopilot: engaging with an elevator trim and stick offset is bumpless") {
    Rig r;
    r.pilot.elevTrimNorm = 0.3f;
    r.pilot.elevatorNorm = 0.05f;
    r.engage();
    r.run(0.05);
    CHECK(r.out.elevTrimNorm == doctest::Approx(0.3f).epsilon(0.01));
    CHECK(r.out.elevatorNorm == doctest::Approx(0.05f).epsilon(0.2));
}

TEST_CASE("autopilot: stick override disconnects after 0.5 s, a short blip does not") {
    Rig r;
    r.engage();
    r.pilot.aileronNorm = 0.8f;
    r.run(0.3);
    CHECK(r.ap.state().apEngaged);
    r.pilot.aileronNorm = 0.0f;
    r.run(0.5);
    CHECK(r.ap.state().apEngaged);  // timer restarted
    r.pilot.elevatorNorm = -0.7f;
    r.run(0.6);
    CHECK_FALSE(r.ap.state().apEngaged);
    CHECK(r.ap.state().disconnectWarning);
}

TEST_CASE("autopilot: small stick inputs are ignored") {
    Rig r;
    r.engage();
    r.pilot.aileronNorm = 0.4f;
    r.run(2.0);
    CHECK(r.ap.state().apEngaged);
}

TEST_CASE("autopilot: stall warning and extreme attitude disconnect") {
    Rig r;
    r.engage();
    r.ac.stallWarning = true;
    r.run(0.1);
    CHECK_FALSE(r.ap.state().apEngaged);
    CHECK(r.ap.state().disconnectWarning);

    Rig q;
    q.engage();
    q.ac.rollRad = static_cast<float>(degToRad(46));
    q.run(0.1);
    CHECK_FALSE(q.ap.state().apEngaged);

    Rig p;
    p.engage();
    p.ac.pitchRad = static_cast<float>(degToRad(-26));
    p.run(0.1);
    CHECK_FALSE(p.ap.state().apEngaged);
}

TEST_CASE("autopilot: disconnect below minimums minus 50 ft on an approach") {
    Rig r;
    setIls(r, 0.0f, 0.0f);
    r.nav.minimumsFt = 2200;
    r.nav.minimumsIsDa = true;
    r.nav.onFinal = true;
    r.ac.altMslFt = 2300;
    r.engage();
    r.ap.selectApr(r.nav);
    r.run(0.2);
    CHECK(r.ap.state().apEngaged);
    r.ac.altMslFt = 2160;  // above minimums - 50
    r.run(0.1);
    CHECK(r.ap.state().apEngaged);
    r.ac.altMslFt = 2149;
    r.run(0.1);
    CHECK_FALSE(r.ap.state().apEngaged);
    CHECK(r.ap.state().disconnectWarning);
}

TEST_CASE("autopilot: no minimums disconnect off the approach or in a missed approach") {
    Rig r;
    r.nav.minimumsFt = 2200;
    r.nav.phase = NavPhase::Missed;
    r.ac.altMslFt = 2100;
    r.nav.onFinal = true;
    r.engage();
    r.run(0.2);
    CHECK(r.ap.state().apEngaged);
}

TEST_CASE("autopilot: AP toggle disconnects with a warning; the next press acknowledges") {
    Rig r;
    r.engage();
    r.ap.handleActions(press(Action::ApToggle), r.ac, r.nav, kSimDtS);
    CHECK_FALSE(r.ap.state().apEngaged);
    CHECK(r.ap.state().disconnectWarning);
    r.ap.handleActions(press(Action::ApToggle), r.ac, r.nav, kSimDtS);
    CHECK_FALSE(r.ap.state().apEngaged);  // acknowledged, not re-engaged
    CHECK_FALSE(r.ap.state().disconnectWarning);
    r.ap.handleActions(press(Action::ApToggle), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().apEngaged);
}

TEST_CASE("autopilot: the disconnect warning clears itself after 5 s") {
    Rig r;
    r.engage();
    r.ap.disengageAp(true);
    r.run(4.5);
    CHECK(r.ap.state().disconnectWarning);
    r.run(1.0);
    CHECK_FALSE(r.ap.state().disconnectWarning);
}

TEST_CASE("autopilot: weight on wheels disconnects silently") {
    Rig r;
    r.engage();
    r.ac.wowAny = true;
    r.run(0.1);
    CHECK_FALSE(r.ap.state().apEngaged);
    CHECK_FALSE(r.ap.state().disconnectWarning);
}

TEST_CASE("autopilot: bug actions") {
    Rig r;
    r.ac.altMslFt = 5130;
    r.setHeadingDeg(87);
    r.ap.handleActions(press(Action::SyncBugs), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(87.f));
    CHECK(r.ap.state().altBugFt == doctest::Approx(5100.f));

    r.ap.handleActions(press(Action::HdgBugInc), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(88.f));
    r.ap.handleActions(press(Action::HdgBugInc, 40), r.ac, r.nav, kSimDtS);  // fast repeat
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(98.f));
    r.ap.setHdgBugMagDeg(359);
    r.ap.handleActions(press(Action::HdgBugInc), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(360.f));
    r.ap.handleActions(press(Action::HdgBugInc), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(1.f));
    r.ap.handleActions(press(Action::HdgBugDec), r.ac, r.nav, kSimDtS);
    r.ap.handleActions(press(Action::HdgBugDec), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().hdgBugMagDeg == doctest::Approx(359.f));

    r.ap.handleActions(press(Action::AltBugInc), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().altBugFt == doctest::Approx(5200.f));
    r.ap.handleActions(press(Action::AltBugDec), r.ac, r.nav, kSimDtS);
    r.ap.handleActions(press(Action::AltBugDec), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().altBugFt == doctest::Approx(5000.f));
    r.ap.handleActions(press(Action::AltBugInc, 40), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().altBugFt == doctest::Approx(6000.f));
}

TEST_CASE("autopilot: mode actions need AP or FD, then select modes") {
    Rig r;
    r.ap.handleActions(press(Action::HdgMode), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().latActive == LatMode::Off);  // nothing engaged: ignored
    r.ap.handleActions(press(Action::FdToggle), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().fdOn);
    r.ap.handleActions(press(Action::HdgMode), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().latActive == LatMode::Hdg);
    r.ap.handleActions(press(Action::AltMode), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().vertActive == VertMode::Alt);
    r.ap.handleActions(press(Action::VsMode), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().vertActive == VertMode::Vs);
    r.ap.handleActions(press(Action::ApToggle), r.ac, r.nav, kSimDtS);
    CHECK(r.ap.state().apEngaged);
    CHECK(r.ap.state().latActive == LatMode::Hdg);  // FD modes carry over to the AP
}

TEST_CASE("autopilot: VS and speed bug follow the bug-rate axis") {
    Rig r(AircraftCategory::Jet, true);
    r.engage();
    r.ap.setAltBugFt(9000);
    r.ap.selectVs(r.ac);  // level, bug above: +default
    ActionState a;
    a.axis[static_cast<int>(Axis::BugRate)] = 1.0f;
    for (int i = 0; i < 60; ++i) r.ap.handleActions(a, r.ac, r.nav, 1.0 / 60.0);
    CHECK(r.ap.state().vsBugFpm > 1000.f);  // 5 steps per second at full deflection
    r.ap.selectPitch(r.ac);
    REQUIRE(r.ap.setAt(true, r.ac));
    const float spd0 = r.ap.state().spdBugKt;
    for (int i = 0; i < 60; ++i) r.ap.handleActions(a, r.ac, r.nav, 1.0 / 60.0);
    CHECK(r.ap.state().spdBugKt >= spd0 + 8.f);
}

TEST_CASE("autopilot: autothrottle needs an A/T aircraft and is refused on the ground") {
    Rig piston;
    CHECK_FALSE(piston.ap.setAt(true, piston.ac));
    Rig jet(AircraftCategory::Jet, true);
    jet.ac.wowAny = true;
    CHECK_FALSE(jet.ap.setAt(true, jet.ac));
    jet.ac.wowAny = false;
    CHECK(jet.ap.setAt(true, jet.ac));
    CHECK(jet.ap.state().atEngaged);
    CHECK(jet.ap.state().spdActive == SpdMode::Speed);
    CHECK(jet.ap.state().spdBugKt == doctest::Approx(100.f));  // starts at the present speed
}

TEST_CASE("autopilot: A/T opens the throttle when slow and closes it when fast") {
    Rig r(AircraftCategory::Jet, true);
    r.pilot.throttleNorm[0] = 0.5f;
    REQUIRE(r.ap.setAt(true, r.ac));
    r.ap.setSpdBugKt(150);
    r.ac.iasKt = 120;
    r.run(5.0);
    CHECK(r.out.throttleNorm[0] > 0.55f);
    CHECK(r.pilot.throttleNorm[0] == doctest::Approx(r.out.throttleNorm[0]));  // lever follows
    r.ap.setSpdBugKt(80);
    r.ac.iasKt = 120;
    r.run(20.0);
    CHECK(r.out.throttleNorm[0] < 0.5f);
    CHECK(r.ap.state().atEngaged);
}

TEST_CASE("autopilot: moving the throttle lever turns the A/T off") {
    Rig r(AircraftCategory::Jet, true);
    r.pilot.throttleNorm[0] = 0.5f;
    REQUIRE(r.ap.setAt(true, r.ac));
    r.run(1.0);
    r.pilot.throttleNorm[0] += 0.05f;  // pilot pushes the lever
    r.run(0.1);
    CHECK_FALSE(r.ap.state().atEngaged);
    CHECK(r.ap.state().spdActive == SpdMode::Off);
}

TEST_CASE("autopilot: RETARD below 30 ft in the landing configuration, A/T off at touchdown") {
    Rig r(AircraftCategory::Jet, true);
    r.pilot.throttleNorm[0] = 0.5f;
    r.ac.flapDetent = 1;
    r.ac.gearRetractable = true;
    r.ac.gearPosNorm = 1;
    REQUIRE(r.ap.setAt(true, r.ac));
    r.ac.altAglFt = 100;
    r.ac.vsFpm = -700;
    r.run(1.0);
    CHECK(r.ap.state().spdActive == SpdMode::Speed);
    r.ac.altAglFt = 25;
    r.run(0.5);
    CHECK(r.ap.state().spdActive == SpdMode::Retard);
    r.run(2.0);
    CHECK(r.out.throttleNorm[0] < 0.2f);
    r.run(2.0);
    CHECK(r.out.throttleNorm[0] == doctest::Approx(0.f));
    r.ac.wowAny = true;
    r.ac.altAglFt = 0;
    r.run(0.1);
    CHECK_FALSE(r.ap.state().atEngaged);
}

TEST_CASE("autopilot: no RETARD without the landing configuration") {
    Rig r(AircraftCategory::Jet, true);
    r.pilot.throttleNorm[0] = 0.5f;
    r.ac.flapDetent = 0;
    REQUIRE(r.ap.setAt(true, r.ac));
    r.ac.altAglFt = 20;
    r.ac.vsFpm = -700;
    r.run(1.0);
    CHECK(r.ap.state().spdActive == SpdMode::Speed);
}

TEST_CASE("autopilot: config defaults by category and profile overrides") {
    AircraftProfile p;
    p.category = AircraftCategory::Piston;
    CHECK(ApConfig::fromProfile(p).bankLimitDeg == doctest::Approx(20.f));
    p.category = AircraftCategory::Jet;
    CHECK(ApConfig::fromProfile(p).bankLimitDeg == doctest::Approx(25.f));
    p.autopilot["bank_limit_deg"] = 15;
    p.autopilot["vs_default_fpm"] = 1234;
    p.autopilot["unknown_key"] = 7;  // ignored
    const ApConfig c = ApConfig::fromProfile(p);
    CHECK(c.bankLimitDeg == doctest::Approx(15.f));
    CHECK(c.vsDefaultFpm == doctest::Approx(1234.f));
}

TEST_CASE("autopilot: scenario tokens engage and select modes") {
    Rig r(AircraftCategory::Jet, true);
    r.ap.setAltBugFt(r.ac.altMslFt + 2000.f);
    r.ap.applyScenario({"ap", "at", "hdg", "alt"}, r.ac, r.nav);
    const AutopilotState& st = r.ap.state();
    CHECK(st.apEngaged);
    CHECK(st.atEngaged);
    CHECK(st.latActive == LatMode::Hdg);
    CHECK(st.vertActive == VertMode::Vs);  // flies to the bug
    CHECK(st.vsBugFpm > 0.f);

    Rig q;
    q.ap.applyScenario({"fd"}, q.ac, q.nav);
    CHECK(q.ap.state().fdOn);
    CHECK_FALSE(q.ap.state().apEngaged);
}
