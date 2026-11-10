// WP F8: balked landing. Full power with the flaps out, holding a target IAS with the elevator: the
// climb (or sink) that is left. A real PA-28 meets CAR 3 / FAR 23.77 (a 3.3 % gradient with takeoff
// power and landing flaps, about 200 fpm at 65 kt). Prints
//   FLEET <id> balked_vs_fpm_f<detent label>_<kt> <value>
// with the elevator, alpha and pitch it took. Flown near sea level (flat ground at 0 ft, 1,000 ft
// MSL), so the numbers compare with the certification rule.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include "FleetPhases.h"
#include "FleetSpecs.h"
#include "FleetSpecsExt.h"

using namespace fsim;
using namespace fsim::test;
using namespace fsim::test::fleet;

namespace {

struct BalkedResult {
    double vsFpm = 0;     // mean over the last 20 s
    double iasKt = 0;     // mean IAS over the same window
    double elevator = 0;  // mean total elevator (stick + trim), +1 = full nose up
    double alphaDeg = 0, pitchDeg = 0;
    bool elevatorAtStop = false;  // the elevator sat at its stop and the speed was not held
    bool ok = false;
};

// Full power at flap detent `detent`, wings level, the IAS held at `targetKt` with a slow pitch
// loop (speed to pitch attitude, then the elevator holds the attitude). 30 s to settle, then 20 s
// of means.
BalkedResult measureBalked(const char* id, int detent, double targetKt) {
    Flight f;
    BalkedResult r;
    if (!f.resolve(id, "cruise")) return r;
    f.start.altMslFt = 1000.0;
    f.start.fieldElevFt = 0.0;
    f.start.iasKt = targetKt;
    f.start.flapDetent = detent;
    f.start.gearDown = true;
    if (!f.begin()) return r;
    setAllThrottles(f, 1.0);
    double pitchCmd = radToDeg(f.s().pitchRad), integ = pitchCmd;
    double sumVs = 0, sumIas = 0, sumElev = 0, sumAlpha = 0, sumPitch = 0;
    long n = 0, atStop = 0;
    const long settle = static_cast<long>(30.0 * kSimHz), total = static_cast<long>(50.0 * kSimHz);
    for (long i = 0; i < total && !f.s().crashed; ++i) {
        const AircraftState& s = f.s();
        const double err = s.iasKt - targetKt;  // fast: pitch up
        integ = std::clamp(integ + 0.08 * err / kSimHz, -10.0, 25.0);
        pitchCmd = std::clamp(integ + 0.6 * err, -10.0, 25.0);
        f.ctl.elevatorNorm = pitchHoldElevator(s, pitchCmd, 0.1, 0.1);
        f.ctl.aileronNorm = bankHoldAileron(s);
        f.ctl.rudderNorm = headingHoldRudder(s, f.start.headingTrueDeg, 0.02, 0.02);
        f.step();
        if (i < settle) continue;
        const double elev = std::clamp(static_cast<double>(f.ctl.elevatorNorm) +
                                           static_cast<double>(f.ctl.elevTrimNorm),
                                       -1.0, 1.0);
        sumVs += f.s().vsFpm;
        sumIas += f.s().iasKt;
        sumElev += elev;
        sumAlpha += f.s().alphaDeg;
        sumPitch += radToDeg(f.s().pitchRad);
        if (elev > 0.98) ++atStop;
        ++n;
    }
    if (n == 0 || f.s().crashed) return r;
    r.vsFpm = sumVs / n;
    r.iasKt = sumIas / n;
    r.elevator = sumElev / n;
    r.alphaDeg = sumAlpha / n;
    r.pitchDeg = sumPitch / n;
    r.elevatorAtStop = atStop > n / 2 && r.iasKt > targetKt + 2.0;
    r.ok = true;
    return r;
}

const FleetSpec& specOf(const char* id) {
    for (const FleetSpec& sp : kFleetExt)
        if (std::string(sp.id) == id) return sp;
    for (const FleetSpec& sp : kFleet)
        if (std::string(sp.id) == id) return sp;
    FAIL("no fleet spec for " << id);
    return kFleet[0];
}

// The best (highest) VS over the three speeds, after printing each.
double balkedTable(const char* id, int detent, const char* label) {
    const FleetSpec& sp = specOf(id);
    double best = -1e9;
    for (double kt : {65.0, 70.0, 75.0}) {
        const BalkedResult r = measureBalked(id, detent, kt);
        INFO(id << " flaps " << label << " " << kt << " kt");
        REQUIRE(r.ok);
        char key[64];
        std::snprintf(key, sizeof key, "balked_vs_fpm_f%s_%.0f", label, kt);
        report(sp, key, r.vsFpm);
        std::printf("FLEET %s balked_f%s_%.0f ias %.1f elevator %.2f%s alpha %.1f pitch %.1f\n", id,
                    label, kt, r.iasKt, r.elevator, r.elevatorAtStop ? " (stop)" : "", r.alphaDeg,
                    r.pitchDeg);
        if (!r.elevatorAtStop) best = std::max(best, r.vsFpm);
    }
    return best;
}

}  // namespace

// The clean climb of the stock model (before the F8 overlay), best of 65-75 kt: the overlay must
// not change the clean airplane.
constexpr double kPa28StockCleanBestFpm = 460.0;

TEST_CASE("sim: fleet pa28 balked landing") {
    const double f40 = balkedTable("pa28", 3, "40");
    const double f25 = balkedTable("pa28", 2, "25");
    const double f10 = balkedTable("pa28", 1, "10");
    const double f0 = balkedTable("pa28", 0, "UP");
    std::printf("FLEET pa28 balked_best_fpm f40 %.0f f25 %.0f f10 %.0f up %.0f\n", f40, f25, f10,
                f0);
    // F8 (the overlay in assets/jsbsim-overlay/aircraft/pa28): full flaps climb, weakly, as the
    // certification rule asks; each flap step up climbs better; the clean climb is unchanged.
    CHECK(f40 >= 100.0);
    CHECK(f40 <= 400.0);
    CHECK(f25 > f40);
    CHECK(f10 > f25);
    CHECK(std::fabs(f0 - kPa28StockCleanBestFpm) <= 0.1 * kPa28StockCleanBestFpm);
}

// A go-around from a flaps-40 final: full power at once, the pilot holds 70 kt. CSV for the PR.
TEST_CASE("sim: fleet pa28 balked landing trace") {
    Flight f;
    REQUIRE_MESSAGE(f.begin("pa28", "final", "distance_nm = 3\n"), f.error);
    REQUIRE(f.ctl.flapDetent == 3);
    setAllThrottles(f, 1.0);
    double integ = radToDeg(f.s().pitchRad);
    std::printf("TRACE t_s,ias_kt,vs_fpm,pitch_deg,alpha_deg,elevator\n");
    for (long i = 0; i <= static_cast<long>(20.0 * kSimHz) && !f.s().crashed; ++i) {
        const AircraftState& s = f.s();
        if (i % static_cast<long>(2.0 * kSimHz) == 0)
            std::printf("TRACE %.0f,%.1f,%.0f,%.1f,%.1f,%.2f\n", s.simTimeS, s.iasKt, s.vsFpm,
                        radToDeg(s.pitchRad), s.alphaDeg,
                        std::clamp(static_cast<double>(f.ctl.elevatorNorm + f.ctl.elevTrimNorm),
                                   -1.0, 1.0));
        const double err = s.iasKt - 70.0;
        integ = std::clamp(integ + 0.08 * err / kSimHz, -10.0, 25.0);
        f.ctl.elevatorNorm =
            pitchHoldElevator(s, std::clamp(integ + 0.6 * err, -10.0, 25.0), 0.1, 0.1);
        f.ctl.aileronNorm = bankHoldAileron(s);
        f.step();
    }
    CHECK_FALSE(f.s().crashed);
}

TEST_CASE("sim: fleet c172p balked landing (reference)") {
    const double f30 = balkedTable("c172p", 3, "30");
    const double f10 = balkedTable("c172p", 1, "10");
    std::printf("FLEET c172p balked_best_fpm f30 %.0f f10 %.0f\n", f30, f10);
}
