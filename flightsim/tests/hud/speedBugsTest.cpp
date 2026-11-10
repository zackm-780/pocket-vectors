// H9: V-speed and flap bugs (HUD.md section 3): weight scaling, the speeds every jet profile gets at
// its reference weight and with full fuel, the stall-speed fallback, and the phase sequence.
#include <doctest/doctest.h>

#include <cmath>
#include <cstring>
#include <string>

#include "core/MessageQueue.h"
#include "core/Paths.h"
#include "gfx/Camera.h"
#include "hud/Hud.h"
#include "hud/SpeedBugs.h"
#include "sim/FlightHarness.h"
#include "sim/ProfileLoader.h"

using namespace fsim;

namespace {

const char* const kAllProfiles[] = {"c172p", "c182", "pa28", "c310",  "global5000",
                                    "737",   "B747", "A320", "787-8", "MD11"};

AircraftProfile loadProfile(const char* id) {
    AircraftProfile p;
    std::string err;
    const Paths paths = Paths::resolve({});
    REQUIRE_MESSAGE(
        loadAircraftProfile(paths.asset(std::string("assets/aircraft/") + id + ".ini"), p, err),
        err);
    return p;
}

float vfeOf(const AircraftProfile& p, int detent) {
    const float v = p.flapDetents[static_cast<size_t>(detent)].vfeKt;
    return v > 0 ? v : p.vfeKt;
}

const SpeedBug* find(const SpeedBugSet& set, const char* label) {
    for (int i = 0; i < set.count; ++i)
        if (std::strcmp(set.bugs[i].label, label) == 0) return &set.bugs[i];
    return nullptr;
}

std::string labels(const SpeedBugSet& set) {
    std::string s;
    for (int i = 0; i < set.count; ++i)
        s += (s.empty() ? "" : " ") + std::string(set.bugs[i].label);
    return s;
}

}  // namespace

TEST_CASE("speedbugs: weight scaling is sqrt(weight / reference), clamped") {
    CHECK(speedBugWeightScale(1.21f * 100000.0f, 100000.0f) == doctest::Approx(1.1f));
    CHECK(speedBugWeightScale(100000.0f, 100000.0f) == doctest::Approx(1.0f));
    CHECK(speedBugWeightScale(0.25f * 100000.0f, 100000.0f) == doctest::Approx(0.8f));  // clamp
    CHECK(speedBugWeightScale(4.0f * 100000.0f, 100000.0f) == doctest::Approx(1.25f));  // clamp
    CHECK(speedBugWeightScale(0.0f, 100000.0f) == doctest::Approx(1.0f));  // unknown weight
    CHECK(speedBugWeightScale(100000.0f, 0.0f) == doctest::Approx(1.0f));  // no reference
    AircraftProfile p = loadProfile("737");
    const VSpeeds ref = computeVSpeeds(p, p.refWeightLbs);
    const VSpeeds heavy = computeVSpeeds(p, 1.21f * p.refWeightLbs);
    CHECK(ref.k == doctest::Approx(1.0f));
    CHECK(heavy.vrKt == doctest::Approx(1.1f * ref.vrKt));
    CHECK(heavy.vrefKt == doctest::Approx(1.1f * ref.vrefKt));
    CHECK(flapBugKt(p, 0, 1.1f) >= flapBugKt(p, 0, 1.0f));
}

TEST_CASE("speedbugs: every profile orders its V-speeds and keeps flap bugs under the next Vfe") {
    for (const char* id : kAllProfiles) {
        CAPTURE(id);
        const AircraftProfile p = loadProfile(id);
        REQUIRE(p.refWeightLbs > 0.0f);
        // At the reference weight and with full fuel (the weight the flight model reports).
        test::Flight f;
        REQUIRE(f.begin(id, "runway", "fuel = full\n"));
        const float fullLbs = f.s().grossWeightLbs;
        CHECK(fullLbs >= p.refWeightLbs * 0.99f);
        for (const float w : {p.refWeightLbs, fullLbs}) {
            CAPTURE(w);
            const VSpeeds v = computeVSpeeds(p, w);
            CHECK(v.vrKt > 0.0f);
            if (p.category == AircraftCategory::Piston) {
                CHECK(v.v1Kt == 0.0f);
                CHECK(v.v2Kt == 0.0f);
            } else {
                CHECK(v.v1Kt < v.vrKt);
                CHECK(v.vrKt < v.v2Kt);
                CHECK(v.v2Kt >= 1.2f * stallSpeedForDetentKt(p, p.takeoffFlapDetent) * v.k - 0.01f);
            }
            CHECK(v.vrefKt <= v.vappKt);
            const int n = static_cast<int>(p.flapDetents.size());
            for (int d = 0; d + 1 < n; ++d) {
                CAPTURE(d);
                const float bug = flapBugKt(p, d, v.k);
                CHECK(bug > 0.0f);
                CHECK(bug <= vfeOf(p, d + 1) + 0.01f);
            }
        }
    }
}

TEST_CASE("speedbugs: without vs_by_flap the stall speed runs from Vs1 to Vs0 by flap command") {
    AircraftProfile p = loadProfile("c172p");
    REQUIRE(p.vsByFlapKt.empty());
    const int n = static_cast<int>(p.flapDetents.size());
    REQUIRE(n >= 3);
    CHECK(stallSpeedForDetentKt(p, 0) == doctest::Approx(p.vs1Kt));
    CHECK(stallSpeedForDetentKt(p, n - 1) == doctest::Approx(p.vs0Kt));
    for (int d = 1; d < n - 1; ++d) {
        const float vs = stallSpeedForDetentKt(p, d);
        CHECK(vs <= p.vs1Kt);
        CHECK(vs >= p.vs0Kt);
        CHECK(vs <= stallSpeedForDetentKt(p, d - 1));
    }
    // With vs_by_flap (F5), the measured values.
    const AircraftProfile j = loadProfile("737");
    REQUIRE(j.vsByFlapKt.size() == j.flapDetents.size());
    CHECK(stallSpeedForDetentKt(j, 3) == doctest::Approx(j.vsByFlapKt[3]));
}

TEST_CASE(
    "speedbugs: phase sequence ground, takeoff roll, climb, cruise, descent, approach, landed") {
    const AircraftProfile p = loadProfile("737");
    SpeedBugTracker t;
    AircraftState s;
    s.valid = true;
    s.grossWeightLbs = p.refWeightLbs;
    s.wowAny = true;
    s.iasKt = 0;
    s.flapDetent = p.takeoffFlapDetent;
    s.flapPosNorm = p.flapDetents[static_cast<size_t>(p.takeoffFlapDetent)].cmdNorm;
    ControlInputs c;
    for (int i = 0; i < kMaxEngines; ++i) c.throttleNorm[i] = 0.1f;

    // Taxi: only the takeoff detent's flap bug.
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Other);
    CHECK(labels(t.bugs()) == p.flapDetents[static_cast<size_t>(s.flapDetent)].label);

    // Takeoff roll: V1, VR, V2 and the next lower detent's bug.
    for (int i = 0; i < kMaxEngines; ++i) c.throttleNorm[i] = 1.0f;
    s.iasKt = 80;
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Takeoff);
    REQUIRE(find(t.bugs(), "V1"));
    REQUIRE(find(t.bugs(), "VR"));
    REQUIRE(find(t.bugs(), "V2"));
    CHECK(find(t.bugs(), "V1")->kt < find(t.bugs(), "VR")->kt);
    CHECK(find(t.bugs(), p.flapDetents[static_cast<size_t>(s.flapDetent) - 1].label));

    // Climb after liftoff: V2 stays until the flaps are up; V1 and VR go.
    s.wowAny = false;
    s.iasKt = 170;
    s.altAglFt = 800;
    s.vsFpm = 2500;
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Takeoff);
    CHECK(find(t.bugs(), "V2"));
    CHECK_FALSE(find(t.bugs(), "V1"));
    CHECK_FALSE(find(t.bugs(), "VR"));

    // Flaps up: cruise climb, no bugs.
    s.flapDetent = 0;
    s.flapPosNorm = 0;
    s.iasKt = 250;
    s.altAglFt = 6000;
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Other);
    CHECK(t.bugs().count == 0);

    // Cruise, then descent: the UP bug (minimum clean speed) and the next detent's.
    s.vsFpm = 0;
    t.update(s, c, p, false, false);
    CHECK(t.bugs().count == 0);
    s.vsFpm = -1500;
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Descent);
    CHECK(labels(t.bugs()) == std::string("UP ") + p.flapDetents[1].label);

    // An approach loaded: REF, APP and the flap bugs.
    t.update(s, c, p, true, false);
    CHECK(t.phase() == SpeedBugPhase::Approach);
    REQUIRE(find(t.bugs(), "REF"));
    REQUIRE(find(t.bugs(), "APP"));
    CHECK(find(t.bugs(), "REF")->kt == doctest::Approx(p.vrefKt));
    CHECK(find(t.bugs(), "UP"));

    // Flaps 30 on final: the detent's bug and the next one.
    s.flapDetent = p.approachFlapDetent;
    s.iasKt = p.vappKt;
    s.vsFpm = -700;
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Approach);
    CHECK(find(t.bugs(), p.flapDetents[static_cast<size_t>(s.flapDetent)].label));

    // MIN declutter hides them, the phase still follows.
    t.update(s, c, p, false, true);
    CHECK(t.bugs().count == 0);
    CHECK(t.phase() == SpeedBugPhase::Approach);

    // Landed, idle: only the flap bug while the flaps are out.
    s.wowAny = true;
    s.vsFpm = 0;
    for (int i = 0; i < kMaxEngines; ++i) c.throttleNorm[i] = 0.0f;
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Other);
    CHECK(labels(t.bugs()) == p.flapDetents[static_cast<size_t>(s.flapDetent)].label);
}

TEST_CASE("speedbugs: pistons show VR only on takeoff; a slow climb is not an approach") {
    const AircraftProfile p = loadProfile("c172p");
    SpeedBugTracker t;
    AircraftState s;
    s.valid = true;
    s.wowAny = true;
    ControlInputs c;
    c.throttleNorm[0] = 1.0f;
    t.update(s, c, p, false, false);
    CHECK(labels(t.bugs()) == "VR");
    s.wowAny = false;
    s.iasKt = 75;
    s.vsFpm = 700;
    s.altAglFt = 300;
    t.update(s, c, p, false, false);  // flaps up at liftoff: the takeoff phase ends
    CHECK(t.phase() == SpeedBugPhase::Other);
    CHECK(t.bugs().count == 0);
    s.vsFpm = -500;  // slow and descending: an approach
    t.update(s, c, p, false, false);
    CHECK(t.phase() == SpeedBugPhase::Approach);
    CHECK(find(t.bugs(), "REF"));
}

TEST_CASE("speedbugs: the next bug follows the speed trend") {
    SpeedBugSet set;
    set.count = 3;
    set.bugs[0].kt = 140;
    std::strcpy(set.bugs[0].label, "REF");
    set.bugs[1].kt = 145;
    std::strcpy(set.bugs[1].label, "APP");
    set.bugs[2].kt = 178;
    std::strcpy(set.bugs[2].label, "5");
    CHECK(nextSpeedBug(set, 160, 0.0f) == nullptr);  // steady: none
    REQUIRE(nextSpeedBug(set, 160, -5.0f));
    CHECK(std::string(nextSpeedBug(set, 160, -5.0f)->label) == "APP");
    REQUIRE(nextSpeedBug(set, 160, 5.0f));
    CHECK(std::string(nextSpeedBug(set, 160, 5.0f)->label) == "5");
    CHECK(nextSpeedBug(set, 130, -5.0f) == nullptr);  // nothing below
}

TEST_CASE("speedbugs: the cockpit HUD with bugs stays under 40k vertices") {
    gfx::GlApi noGl;  // the renderer records geometry without init(); no GL calls
    Renderer2D r2d{noGl, gfx::GlFlavor::Es2};
    gfx::Camera cam;
    const AircraftProfile p = loadProfile("737");
    AircraftState s;
    s.valid = true;
    s.latDeg = 45.5;
    s.lonDeg = -122.6;
    s.altMslFt = 3000;
    s.altAglFt = 1000;
    s.headingTrueRad = 1.5f;
    s.iasKt = 150;
    s.gsKt = 150;
    s.vsFpm = -800;
    s.flapDetent = p.approachFlapDetent;
    s.grossWeightLbs = p.refWeightLbs;
    s.engineCount = 2;
    ControlInputs c;
    MessageQueue msgs;
    HudOptions opt;
    opt.approachLoaded = true;
    cam.setLens(55.0f);
    cam.setViewport(640, 480);
    cam.setPoseFromAircraft(s);
    Hud hud;
    for (const Declutter d : {Declutter::Full, Declutter::Min}) {
        opt.declutter = d;
        r2d.begin(640, 480);
        hud.draw(r2d, cam, s, c, p, msgs, opt);
        const int vertices = r2d.pendingVertexCount();
        r2d.end();
        CAPTURE(declutterName(d));

        CHECK(vertices < 40000);
    }
}
