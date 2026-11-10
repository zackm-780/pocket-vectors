// WP F9: pitch handling as the pilot feels it. A raw left-stick deflection goes through the game's
// shaping (the [axes] radial deadzone and expo of assets/controls_default.ini, InputCurves.h) and
// PilotControls (the user's stick tuning, the trim buttons), then into the flight model. Prints
//   FLEET <id> handling_<start>_<what> <value>
// for the stick steps (pitch rate at 1 s, VS change at 3 s and 6 s), one trim tap and one second
// of trim (VS change after 10 s, stick free), and the trim travel end to end.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "FleetPhases.h"
#include "core/Actions.h"
#include "input/ControlsConfig.h"
#include "input/InputCurves.h"
#include "input/PilotControls.h"

using namespace fsim;
using namespace fsim::test;
using namespace fsim::test::fleet;

namespace {

constexpr double kFrameS = 1.0 / 60.0;  // the game's frame (PilotControls runs once a frame)
constexpr int kStepsPerFrame = kSimHz / 60;

int ai(Action a) {
    return static_cast<int>(a);
}

// The game's input path for the left stick's Y and the trim buttons.
struct GamePilot {
    ControlsConfig cfg;
    PilotControls pc;
    ActionState actions;
    const AircraftProfile* profile = nullptr;

    GamePilot() {
        const Paths paths = Paths::resolve({});
        cfg.loadFiles(paths.asset("assets/controls_default.ini"), "");
    }
    // Raw stick Y (+ = back, nose up) as the InputSystem shapes it: [axes] left_pitch_expo, or
    // the aircraft's [controls] pitch_expo (InputSystem::setPitchExpoOverride).
    float expoOverride = -1.0f;  // >= 0: this pitch expo (the traces' "before": 0.35)
    float shapePitch(float raw, const AircraftProfile& p) const {
        float x = 0.0f, y = raw;
        const float e = expoOverride >= 0.0f  ? expoOverride
                        : p.pitchExpo >= 0.0f ? p.pitchExpo
                                              : cfg.leftPitchExpo;
        input::applyRadialStick(x, y, cfg.leftDeadzone, cfg.leftExpo, e);
        return y;
    }
    void frame(Flight& f, float rawPitch, bool trimUp, bool trimDown) {
        const bool upWas = actions.down[ai(Action::TrimNoseUp)];
        const bool downWas = actions.down[ai(Action::TrimNoseDown)];
        actions.axis[static_cast<int>(Axis::Pitch)] = shapePitch(rawPitch, f.profile);
        actions.axis[static_cast<int>(Axis::Roll)] = bankHoldAileron(f.s());
        actions.down[ai(Action::TrimNoseUp)] = trimUp;
        actions.down[ai(Action::TrimNoseDown)] = trimDown;
        actions.pressed[ai(Action::TrimNoseUp)] = trimUp && !upWas;
        actions.pressed[ai(Action::TrimNoseDown)] = trimDown && !downWas;
        pc.update(actions, f.profile, f.s(), kFrameS, f.ctl, nullptr);
        for (int i = 0; i < kStepsPerFrame; ++i) f.step();
    }
};

struct StartKind {
    const char* label;  // "cruise" or "approach"
    const char* kind;   // scenario start
    const char* extra;  // scenario lines
};
const StartKind kStarts[] = {{"cruise", "cruise", ""}, {"approach", "final", "distance_nm = 8\n"}};
// The 737's profile cruise is FL330 at 270 KIAS, where a small pitch change is a large VS; the
// owner's check is at FL100.
const StartKind kFl100 = {"fl100", "cruise", "altitude_ft = 10000\nias_kt = 250\n"};

bool startFlight(Flight& f, const char* id, const StartKind& st) {
    if (!f.begin(id, st.kind, st.extra)) return false;
    // Let the trimmed start settle a moment with the stick free.
    GamePilot settle;
    for (int i = 0; i < 60; ++i) settle.frame(f, 0.0f, false, false);
    return true;
}

struct StepResult {
    double q1sDegPs = 0, dvs3Fpm = 0, dvs6Fpm = 0;
};

// The raw stick held at `raw` for 3 s, then released for 3 s.
StepResult stickStep(const char* id, const StartKind& st, float raw) {
    Flight f;
    StepResult r;
    REQUIRE_MESSAGE(startFlight(f, id, st), f.error);
    GamePilot p;
    const double vs0 = f.s().vsFpm;
    for (int k = 1; k <= 360; ++k) {
        p.frame(f, k <= 180 ? raw : 0.0f, false, false);
        if (k == 60) r.q1sDegPs = radToDeg(f.s().qRadps);
        if (k == 180) r.dvs3Fpm = f.s().vsFpm - vs0;
        if (k == 360) r.dvs6Fpm = f.s().vsFpm - vs0;
    }
    return r;
}

// Trim nose up for `frames` frames (1 = a tap), stick free: the mean VS change from 3 to 10 s
// after the press (the phugoid makes one sample at 10 s noisy).
double trimResponse(const char* id, const StartKind& st, int frames) {
    Flight f;
    REQUIRE_MESSAGE(startFlight(f, id, st), f.error);
    GamePilot p;
    const double vs0 = f.s().vsFpm;
    double sum = 0;
    int n = 0;
    for (int k = 1; k <= 600; ++k) {
        p.frame(f, 0.0f, k <= frames, false);
        if (k >= 180) {
            sum += f.s().vsFpm - vs0;
            ++n;
        }
    }
    return sum / n;
}

// Seconds to run the trim from full nose down to full nose up with the button held, on the
// ground (the speed factor is 1 there).
double trimTravelS(const char* id) {
    Flight f;
    REQUIRE_MESSAGE(f.begin(id, "runway"), f.error);
    GamePilot p;
    f.ctl.elevTrimNorm = -1.0f;
    ActionState& a = p.actions;
    double t = 0;
    for (int k = 1; k <= 60 * 120 && f.ctl.elevTrimNorm < 1.0f; ++k) {
        a.pressed[ai(Action::TrimNoseUp)] = k == 1;
        a.down[ai(Action::TrimNoseUp)] = true;
        p.pc.update(a, f.profile, f.s(), kFrameS, f.ctl, nullptr);
        t = k * kFrameS;
    }
    return t;
}

void reportKey(const char* id, const std::string& key, double v) {
    std::printf("FLEET %s handling_%s %.1f\n", id, key.c_str(), v);
}

struct Measured {
    StepResult step[4];  // raw 0.1, 0.2, 0.3, 0.5
    double tapFpm = 0, holdFpm = 0;
};

Measured handlingAt(const char* id, const StartKind& st) {
    Measured m;
    int si = 0;
    for (float raw : {0.1f, 0.2f, 0.3f, 0.5f}) {
        const StepResult r = stickStep(id, st, raw);
        m.step[si++] = r;
        char tag[48];
        std::snprintf(tag, sizeof tag, "%s_r%.1f", st.label, raw);
        reportKey(id, std::string(tag) + "_q1s_degps", r.q1sDegPs);
        reportKey(id, std::string(tag) + "_dvs3_fpm", r.dvs3Fpm);
        reportKey(id, std::string(tag) + "_dvs6_fpm", r.dvs6Fpm);
    }
    m.tapFpm = trimResponse(id, st, 1);
    m.holdFpm = trimResponse(id, st, 60);
    reportKey(id, std::string(st.label) + "_trim_tap_dvs_fpm", m.tapFpm);
    reportKey(id, std::string(st.label) + "_trim_1s_dvs_fpm", m.holdFpm);
    return m;
}

// The F9 targets (the card's, with one moved, see the PR): raw 0.2 held 3 s changes the VS by
// 150-500 fpm at under 2.5 deg/s of pitch rate (the 737 on approach: raw 0.3, its elevator is a
// fifth as strong there as at FL100), raw 0.1 under 250 fpm, one trim tap 30-150 fpm and one
// second of trim 150-600 fpm.
void checkTargets(const char* id, const char* where, const Measured& m, int stepIndex) {
    INFO(id << " " << where);
    const StepResult& s = m.step[stepIndex];
    CHECK(std::fabs(s.dvs3Fpm) >= 150.0);
    CHECK(std::fabs(s.dvs3Fpm) <= 500.0);
    CHECK(std::fabs(m.step[1].q1sDegPs) < 2.5);
    CHECK(std::fabs(m.step[0].dvs3Fpm) < 250.0);
    CHECK(m.tapFpm >= 30.0);
    CHECK(m.tapFpm <= 150.0);
    CHECK(m.holdFpm >= 150.0);
    CHECK(m.holdFpm <= 600.0);
}

void handling(const char* id, bool check) {
    const bool jet = std::string(id) == "737";
    const Measured cruise = handlingAt(id, kStarts[0]);
    const Measured approach = handlingAt(id, kStarts[1]);
    if (jet) {
        const Measured fl100 = handlingAt(id, kFl100);
        if (check) checkTargets(id, "fl100", fl100, 1);
    }
    reportKey(id, "trim_travel_s", trimTravelS(id));
    if (!check) return;
    checkTargets(id, "cruise", cruise, 1);
    checkTargets(id, "approach", approach, jet ? 2 : 1);
    // Full stick is still full elevator (rotation, flare and stall recovery keep their authority).
    GamePilot p;
    Flight f;
    REQUIRE(f.loadProfile(id));
    CHECK(p.shapePitch(1.0f, f.profile) == doctest::Approx(1.0f));
    CHECK(p.shapePitch(-1.0f, f.profile) == doctest::Approx(-1.0f));
}

// The pitch damper and auto trim aids (off by default): after the raw 0.2 step is released, the
// VS swing from 6 to 40 s (the phugoid), the pitch rate at 1 s, and the VS 20-40 s after.
void aids(const char* id, float autoKp = -1, float autoKd = -1) {
    for (int mode = 0; mode < 3; ++mode) {  // off, damper, auto trim
        Flight f;
        REQUIRE_MESSAGE(startFlight(f, id, kStarts[0]), f.error);
        GamePilot p;
        if (autoKp >= 0) p.pc.tuning().autoTrimPerDegS = autoKp;
        if (autoKd >= 0) p.pc.tuning().autoTrimDampPerDegPs = autoKd;
        StickTuning t;
        t.pitchDamper = mode == 1;
        t.autoTrim = mode == 2;
        p.pc.setStickTuning(t);
        const double vs0 = f.s().vsFpm;
        double q1 = 0, lo = 1e9, hi = -1e9, late = 0;
        int nLate = 0;
        for (int k = 1; k <= 60 * 40; ++k) {
            p.frame(f, k <= 180 ? 0.2f : 0.0f, false, false);
            const double vs = f.s().vsFpm - vs0;
            if (k == 60) q1 = radToDeg(f.s().qRadps);
            if (k >= 360) {
                lo = std::min(lo, vs);
                hi = std::max(hi, vs);
            }
            if (k >= 1200) {
                late += vs;
                ++nLate;
            }
        }
        static const char* const kNames[] = {"off", "damper", "autotrim"};
        std::printf("FLEET %s handling_aid_%s q1s %.1f phugoid_pp_fpm %.0f vs_20_40s_fpm %.0f\n",
                    id, kNames[mode], q1, hi - lo, late / nLate);
    }
}

}  // namespace

TEST_CASE("sim: fleet c172p handling") {
    handling("c172p", true);
    aids("c172p");
}
TEST_CASE("sim: fleet pa28 handling") {
    handling("pa28", true);
    aids("pa28");
}
TEST_CASE("sim: fleet 737 handling") {
    handling("737", true);
    aids("737");
}
TEST_CASE("sim: fleet auto trim tuning (print only)" * doctest::skip()) {
    for (const char* id : {"c172p", "pa28", "737"})
        for (float kp : {0.001f, 0.002f, 0.004f})
            for (float kd : {0.0f, 0.01f, 0.03f, 0.06f}) {
                std::printf("kp %.3f kd %.2f: ", kp, kd);
                aids(id, kp, kd);
            }
}
// The raw 0.2 step as a CSV (t, raw, elevator, q, VS change), with the 1.3 shaping (pitch expo
// 0.35, "before") and today's. Run by hand: --test-case='*handling trace*' --no-skip.
TEST_CASE("sim: fleet handling trace (print only)" * doctest::skip()) {
    for (const char* id : {"c172p", "pa28", "737"}) {
        for (const bool before : {true, false}) {
            Flight f;
            REQUIRE_MESSAGE(startFlight(f, id, kStarts[0]), f.error);
            GamePilot p;
            if (before) p.expoOverride = 0.35f;
            const double vs0 = f.s().vsFpm;
            std::printf("TRACE %s %s t_s,raw,elevator,q_degps,dvs_fpm\n", id,
                        before ? "before" : "after");
            for (int k = 1; k <= 360; ++k) {
                const float raw = k <= 180 ? 0.2f : 0.0f;
                p.frame(f, raw, false, false);
                if (k % 30 == 0)
                    std::printf("TRACE %s %s %.1f,%.1f,%.3f,%.2f,%.0f\n", id,
                                before ? "before" : "after", k * kFrameS, raw, f.ctl.elevatorNorm,
                                radToDeg(f.s().qRadps), f.s().vsFpm - vs0);
            }
        }
    }
}

TEST_CASE("sim: fleet F9 [controls] keys load from the profiles") {
    Flight f;
    REQUIRE(f.loadProfile("c172p"));
    CHECK(f.profile.pitchExpo == doctest::Approx(0.83f));
    CHECK(f.profile.trimStepNorm == doctest::Approx(0.005f));
    CHECK(f.profile.trimRateNormPerS == 0.0f);  // the default
    REQUIRE(f.loadProfile("pa28"));
    CHECK(f.profile.trimStepNorm == doctest::Approx(0.009f));
    CHECK(f.profile.trimRateNormPerS == doctest::Approx(0.08f));
    REQUIRE(f.loadProfile("c182"));
    CHECK(f.profile.pitchExpo < 0.0f);  // no [controls]: the [axes] default
}
// Print-only for the rest of the fleet (the [axes] default pitch expo and trim).
TEST_CASE("sim: fleet other aircraft handling (print only)") {
    for (const char* id : {"c182", "c310", "global5000", "B747", "A320", "787-8", "MD11"})
        handling(id, false);
}
