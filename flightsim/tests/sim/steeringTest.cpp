// F11: nose-wheel steering for taxiing. Every aircraft on the XTST runway: a speed hold on the
// throttle and brakes at 8, 15, 25 and 35 kt, then 25 %, 50 % and full right stick (steering):
// the steady turn radius and yaw rate, and the nose-wheel angle reached. Prints
//   STEER <id> <kt>kt_<pct> radius_m <r> yaw_degps <y> nose_deg <a> lat_g <g>
// plus the asserted taxi checks.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "FlightHarness.h"

using namespace fsim;
using namespace fsim::test;

namespace {

const char* const kFleet[] = {"c172p", "c182", "pa28", "c310",  "global5000",
                              "737",   "B747", "A320", "787-8", "MD11"};

void setThrottles(Flight& f, float v) {
    for (int i = 0; i < f.s().engineCount; ++i) f.ctl.throttleNorm[i] = v;
}

// Throttle and brakes hold the ground speed (simple PI on thrust, brakes when too fast).
struct SpeedHold {
    double integ = 0;
    void apply(Flight& f, double targetKt) {
        const double err = targetKt - f.s().gsKt;
        integ = std::clamp(integ + 0.02 * err / kSimHz, 0.0, 0.6);
        setThrottles(f, static_cast<float>(std::clamp(integ + 0.05 * err, 0.0, 0.8)));
        const float b =
            err < -0.5 ? static_cast<float>(std::clamp(-0.3 * (err + 0.5), 0.0, 1.0)) : 0.f;
        f.ctl.brakeLeftNorm = f.ctl.brakeRightNorm = b;
    }
};

struct TurnResult {
    double radiusM = 0, yawDegPs = 0, noseDeg = 0, maxLatG = 0;
    bool crashed = false;
    std::string reason;
};

double noseAngleDeg(const Flight& f) {
    return f.model.getProperty("fcs/steer-pos-deg[0]");
}

// From the runway start: accelerate straight to `kt`, then hold `stick` right for 15 s at that
// speed; the mean yaw rate over the last 5 s gives the radius (ground speed / yaw rate).
TurnResult turn(const char* id, double kt, float stick, float rudderFrac = 1.0f,
                float steerFrac = 1.0f) {
    Flight f;
    TurnResult r;
    REQUIRE_MESSAGE(f.begin(id, "runway"), f.error);
    f.ctl.parkingBrake = false;
    SpeedHold hold;
    const double hdg0 = radToDeg(f.s().headingTrueRad);
    for (long i = 0; i < 90L * kSimHz && std::fabs(f.s().gsKt - kt) > 0.5; ++i) {
        hold.apply(f, kt);
        f.ctl.steeringNorm = f.ctl.rudderNorm = headingHoldRudder(f.s(), hdg0);
        f.step();
    }
    for (long i = 0; i < 5L * kSimHz; ++i) {  // settle the speed
        hold.apply(f, kt);
        f.ctl.steeringNorm = f.ctl.rudderNorm = headingHoldRudder(f.s(), hdg0);
        f.step();
    }
    double sumR = 0, sumGs = 0;
    long n = 0;
    for (long i = 0; i < 15L * kSimHz; ++i) {
        hold.apply(f, kt);
        f.ctl.steeringNorm = stick * steerFrac;
        f.ctl.rudderNorm = stick * rudderFrac;
        f.step();
        const AircraftState& s = f.s();
        if (s.crashed) {
            r.crashed = true;
            r.reason = s.crashReason;
            break;
        }
        const double latG = std::fabs(s.gsKt * 0.514444 * s.rRadps) / 9.80665;
        r.maxLatG = std::max(r.maxLatG, latG);
        if (i >= 10L * kSimHz) {
            sumR += s.rRadps;
            sumGs += s.gsKt;
            ++n;
        }
    }
    r.noseDeg = noseAngleDeg(f);
    if (n > 0) {
        const double yaw = sumR / n;  // rad/s
        r.yawDegPs = radToDeg(yaw);
        r.radiusM = std::fabs(yaw) > 1e-4 ? (sumGs / n) * 0.514444 / std::fabs(yaw) : 1e9;
    }
    return r;
}

}  // namespace

TEST_CASE("sim: F11 steering table: turn radius, yaw rate and nose-wheel angle (print only)") {
    for (const char* id : kFleet) {
        for (double kt : {8.0, 15.0, 25.0, 35.0}) {
            for (float stick : {0.25f, 0.5f, 1.0f}) {
                const TurnResult r = turn(id, kt, stick);
                std::printf("STEER %s %.0fkt_%.0f radius_m %.0f yaw_degps %.1f nose_deg %.1f "
                            "lat_g %.2f%s\n",
                            id, kt, stick * 100, r.radiusM, r.yawDegPs, r.noseDeg, r.maxLatG,
                            r.crashed ? (" CRASH " + r.reason).c_str() : "");
            }
        }
    }
}

TEST_CASE("sim: F11 full stick at 8 kt turns tight, at 30 kt without a skid or a strike") {
    struct Limit {
        const char* id;
        double maxRadiusM;
    };
    const Limit kLimits[] = {{"c172p", 15},      {"c182", 15}, {"pa28", 15}, {"c310", 15},
                             {"global5000", 30}, {"737", 30},  {"A320", 30}, {"787-8", 40},
                             {"MD11", 40},       {"B747", 50}};
    for (const Limit& l : kLimits) {
        INFO(l.id);
        const TurnResult slow = turn(l.id, 8.0, 1.0f);
        CHECK_FALSE(slow.crashed);
        CHECK(slow.radiusM <= l.maxRadiusM);
        // Full stick at 30 kt (rudder and nose wheel together, as the stick drives both): no
        // crash or strike. The nose wheel's own share stays under 0.35 g; in the light aircraft
        // the rudder and propwash alone give 0.2-0.4 g more at this speed, so the 0.35 g bound is
        // asserted on the full stick for the jets only.
        const TurnResult fast = turn(l.id, 30.0, 1.0f);
        const TurnResult nose = turn(l.id, 30.0, 1.0f, 0.0f, 1.0f);
        CHECK_FALSE(fast.crashed);
        CHECK_FALSE(nose.crashed);
        CHECK(nose.maxLatG <= 0.35);
        const bool light = l.maxRadiusM <= 15;
        if (!light) CHECK(fast.maxLatG <= 0.35);
        std::printf("STEER %s check 8kt_100 radius_m %.1f; 30kt_100 lat_g %.2f (nose wheel only "
                    "%.2f) radius_m %.0f\n",
                    l.id, slow.radiusM, fast.maxLatG, nose.maxLatG, fast.radiusM);
    }
}

namespace {

// Inverse of the ground steering curve x (1 - e + e x^2) on 0..1 (bisection).
double unshape(double y, double e) {
    const double a = std::clamp(std::fabs(y), 0.0, 1.0);
    double lo = 0, hi = 1;
    for (int k = 0; k < 40; ++k) {
        const double m = 0.5 * (lo + hi);
        (m * (1 - e + e * m * m) < a ? lo : hi) = m;
    }
    return y < 0 ? -lo : lo;
}

double groundKey(const AircraftProfile& p, const char* key, double def) {
    const auto it = p.properties.find(std::string("ground.") + key);
    return it == p.properties.end() ? def : std::strtod(it->second.c_str(), nullptr);
}

}  // namespace

// A 90 degree right turn onto a centerline arc of radius `radiusM` at 11 kt, flown by a simple
// steering pilot: the nose-wheel angle the arc needs (atan(wheelbase / R)), started a little
// before the arc, plus cross-track and heading feedback, turned back into a stick position
// through the tiller's limits. The main gear (what a pilot of a long jet keeps on the line, and
// the point the tiller geometry turns about) stays within 3 m of the line.
TEST_CASE("sim: F11 a 90 degree taxiway turn holds the centerline within 3 m") {
    struct Case {
        const char* id;
        double radiusM;
    };
    const double kXte = 2.0,
                 kHdg = 1.5;    // deg of nose wheel per m off the line, per deg of heading
    const double kLeadS = 1.6;  // the turn starts this long before the arc
    const double kTaxiKt = 10.0;
    for (const Case& c : {Case{"c172p", 15}, Case{"737", 30}, Case{"A320", 30}, Case{"B747", 30}}) {
        INFO(c.id);
        Flight f;
        REQUIRE_MESSAGE(f.begin(c.id, "runway"), f.error);
        f.ctl.parkingBrake = false;
        const AircraftProfile& p = f.profile;
        const double wheelbaseM = ftToM(groundKey(p, "wheelbase_ft", 0));
        const double tillerDeg = groundKey(p, "tiller_max_deg", 0);
        const double latG = groundKey(p, "steer_lat_g", 0.3);
        const double expo = groundKey(p, "steer_expo", 0.5);
        REQUIRE(wheelbaseM > 0);
        const double lat0 = f.s().latDeg, lon0 = f.s().lonDeg;
        const double h0 = radToDeg(f.s().headingTrueRad);
        // Local frame: x along the start heading, y to the right (m).
        // The main gear is this far behind the reference point (JSBSim x grows aft).
        const double mainAftM = ftToM((f.model.getProperty("gear/unit[1]/x-position") -
                                       f.model.getProperty("inertia/cg-x-in")) /
                                      12.0);
        std::printf("STEER %s main gear %.1f m behind the reference point\n", c.id, mainAftM);
        auto local = [&](double& x, double& y) {
            const double dM = greatCircleDistM(lat0, lon0, f.s().latDeg, f.s().lonDeg);
            const double brg =
                radToDeg(initialBearingTrueRad(lat0, lon0, f.s().latDeg, f.s().lonDeg));
            const double rel = degToRad(brg - h0);
            const double hdg = f.s().headingTrueRad - degToRad(h0);
            x = dM * std::cos(rel) - mainAftM * std::cos(hdg);
            y = dM * std::sin(rel) - mainAftM * std::sin(hdg);
        };
        SpeedHold hold;
        const double straightM = 60.0, R = c.radiusM;
        double maxErrM = 0;
        bool done = false;
        for (long i = 0; i < 240L * kSimHz && !done; ++i) {
            double x = 0, y = 0;
            local(x, y);
            const double hdgRel = wrap180(radToDeg(f.s().headingTrueRad) - h0);  // + = right
            double xte = 0, pathHdg = 0, curv = 0;  // xte + = right of the path
            if (x < straightM) {
                xte = y;
            } else {
                const double cx = straightM, cy = R;            // the arc's centre, to the right
                const double ang = std::atan2(x - cx, cy - y);  // swept angle, 0 at the entry
                if (ang < kPi / 2) {
                    xte = R - std::hypot(x - cx, y - cy);  // inside = right of the path
                    pathHdg = radToDeg(ang);
                    curv = 1.0 / R;
                } else {
                    xte = -(x - (straightM + R));  // after the turn, heading +90: right is -x
                    pathHdg = 90.0;
                    if (y > R + 40.0) done = true;
                }
            }
            if (f.s().gsKt > 9.0 || x > 5.0) maxErrM = std::max(maxErrM, std::fabs(xte));
            const double v = std::max(ktToMps(f.s().gsKt), 1.0);
            // The curvature a pilot looks ahead to: from 1.2 s before the arc to 1.2 s before
            // its end.
            const double leadM = kLeadS * v;
            if (x < straightM && x > straightM - leadM) curv = 1.0 / R;
            if (curv > 0 && pathHdg > 90.0 - radToDeg(leadM / R)) curv = 0.0;
            if (std::getenv("TAXI_TRACE") && i % (kSimHz / 2) == 0)
                std::printf(
                    "TAXI %s t %.1f x %.1f y %.1f hdg %.1f xte %.2f gs %.1f stick %.2f nose %.1f\n",
                    c.id, f.s().simTimeS, x, y, hdgRel, xte, f.s().gsKt, f.ctl.steeringNorm,
                    f.model.getProperty("fcs/steer-pos-deg[0]"));
            const double limDeg = std::min(
                tillerDeg, radToDeg(std::atan(wheelbaseM * latG * kStdGravityMps2 / (v * v))));
            // A taxiing pilot uses the tiller the arc needs and a little more (far past it the
            // A320's nose tyre slides and turns less).
            const double ffDeg = radToDeg(std::atan(wheelbaseM * (1.0 / R)));
            const double wantDeg = std::clamp(radToDeg(std::atan(wheelbaseM * curv)) - kXte * xte -
                                                  kHdg * wrap180(hdgRel - pathHdg),
                                              -ffDeg - 15.0, ffDeg + 15.0);
            f.ctl.steeringNorm = f.ctl.rudderNorm =
                static_cast<float>(unshape(std::clamp(wantDeg / limDeg, -1.0, 1.0), expo));
            hold.apply(f, kTaxiKt);
            f.step();
            REQUIRE_FALSE(f.s().crashed);
        }
        CHECK(done);
        std::printf("STEER %s taxi turn R %.0f m at %.0f kt: max off the line %.2f m\n", c.id, R,
                    kTaxiKt, maxErrM);
        CHECK(maxErrM < 3.0);
    }
}
