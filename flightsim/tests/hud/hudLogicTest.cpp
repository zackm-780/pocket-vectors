// HudLogic: tape scaling and label stepping, heading wrap, declutter rules, status-row field
// formatting, warning priority and flash timing, speed trend and bands, message fading.
#include <doctest/doctest.h>

#include <cmath>
#include <cstring>
#include <string>

#include "core/Actions.h"
#include "core/MessageQueue.h"
#include "core/Units.h"
#include "hud/Hud.h"
#include "hud/HudLogic.h"
#include "hud/StrokeFont.h"

using namespace fsim;

namespace {

AircraftProfile makeC172() {
    AircraftProfile p;
    p.id = "c172p";
    p.category = AircraftCategory::Piston;
    p.engineCount = 1;
    const char* labels[] = {"UP", "10", "20", "30"};
    const float cmd[] = {0.0f, 0.3333f, 0.6667f, 1.0f};
    for (int i = 0; i < 4; ++i) {
        FlapDetent d;
        d.cmdNorm = cmd[i];
        std::strncpy(d.label, labels[i], sizeof d.label - 1);
        p.flapDetents.push_back(d);
    }
    p.vs0Kt = 40;
    p.vs1Kt = 48;
    p.vfeKt = 85;
    p.vnoKt = 129;
    p.vneKt = 163;
    p.stallWarnAlphaDeg = 13.5f;
    p.approachFlapDetent = 2;
    return p;
}

AircraftProfile makeJet() {
    AircraftProfile p;
    p.id = "737";
    p.category = AircraftCategory::Jet;
    p.engineCount = 2;
    p.hasRetractableGear = true;
    p.hasSpeedbrake = true;
    p.hasReversers = true;
    const char* labels[] = {"UP", "1", "5", "10", "15", "25", "30", "40"};
    for (int i = 0; i < 8; ++i) {
        FlapDetent d;
        d.cmdNorm = static_cast<float>(i) / 7.0f;
        std::strncpy(d.label, labels[i], sizeof d.label - 1);
        p.flapDetents.push_back(d);
    }
    p.vs0Kt = 105;
    p.vs1Kt = 130;
    p.vfeKt = 190;
    p.vmoKt = 340;
    p.mmo = 0.82f;
    p.vappKt = 145;
    p.approachFlapDetent = 6;
    p.stallWarnAlphaDeg = 12;
    return p;
}

AircraftState airborneState() {
    AircraftState s;
    s.valid = true;
    s.altMslFt = 5000;
    s.altAglFt = 3000;
    s.iasKt = 120;
    s.gearPosNorm = 1;
    s.engineCount = 1;
    return s;
}

std::string joined(const StatusField& f) {
    std::string out;
    for (int i = 0; i < f.segCount; ++i) {
        if (i) out += " ";
        out += f.segs[i].text;
    }
    return out;
}

}  // namespace

TEST_CASE("hud: speed and altitude tape value to y mapping") {
    const TapeGeom& sp = kSpeedTapeGeom;
    CHECK(sp.yOf(132, 132) == doctest::Approx(240.0f));
    CHECK(sp.yOf(142, 132) == doctest::Approx(240.0f - 20.0f));  // 10 kt = 20 px, up is faster
    CHECK(sp.yOf(122, 132) == doctest::Approx(240.0f + 20.0f));
    CHECK(sp.valueAt(sp.yOf(171.5f, 132), 132) == doctest::Approx(171.5f));
    const TapeGeom& al = kAltTapeGeom;
    CHECK(al.yOf(3200, 3200) == doctest::Approx(240.0f));
    CHECK(al.yOf(3300, 3200) == doctest::Approx(240.0f - 16.0f));  // 100 ft = 16 px
    CHECK(al.valueAt(al.yOf(2750, 3200), 3200) == doctest::Approx(2750.0f));
}

TEST_CASE("hud: tape ticks and label stepping") {
    // Speed: ticks every 10 kt that fit between y 70 and y 370 around 132 kt.
    const TickRange t = tickRange(kSpeedTapeGeom, 132, 10);
    REQUIRE_FALSE(t.empty());
    CHECK(t.first % 10 == 0);
    CHECK(t.last % 10 == 0);
    CHECK(kSpeedTapeGeom.yOf(static_cast<float>(t.first), 132) <= 370.0f);
    CHECK(kSpeedTapeGeom.yOf(static_cast<float>(t.first - 10), 132) > 370.0f);
    CHECK(kSpeedTapeGeom.yOf(static_cast<float>(t.last), 132) >= 70.0f);
    CHECK(kSpeedTapeGeom.yOf(static_cast<float>(t.last + 10), 132) < 70.0f);
    // Labels on every second speed tick, every fifth altitude tick.
    CHECK(isLabelTick(120, 20));
    CHECK_FALSE(isLabelTick(130, 20));
    CHECK(isLabelTick(0, 20));
    CHECK(isLabelTick(-20, 20));
    CHECK_FALSE(isLabelTick(-10, 20));
    CHECK(isLabelTick(3500, 500));
    CHECK_FALSE(isLabelTick(3600, 500));
    const TickRange a = tickRange(kAltTapeGeom, 3217, 100);
    CHECK(a.first % 100 == 0);
    CHECK(a.last % 100 == 0);
    CHECK(a.last > a.first);
}

TEST_CASE("hud: heading is magnetic and wraps 359 to 000") {
    CHECK(headingMagDeg(0.0f, 10.0f) == doctest::Approx(350.0f));  // 10 E variation
    CHECK(headingMagDeg(static_cast<float>(degToRad(90)), -5.0f) == doctest::Approx(95.0f));
    CHECK(angleDiffDeg(359, 1) == doctest::Approx(-2.0f));
    CHECK(angleDiffDeg(1, 359) == doctest::Approx(2.0f));
    // 001 sits 2 degrees to the right of 359 on the tape.
    const float x = headingTapeX(1, 359, 320, kHeadingTapePxPerDeg);
    CHECK(x == doctest::Approx(320.0f + 2.0f * kHeadingTapePxPerDeg));
    CHECK(headingTapeLabel(270) == 27);
    CHECK(headingTapeLabel(0) == 36);
    CHECK(headingTapeLabel(360) == 36);
    CHECK(headingTapeLabel(350) == 35);
    char box[8];
    formatHeadingBox(box, sizeof box, 359.6f);
    CHECK(std::string(box) == "000");
    formatHeadingBox(box, sizeof box, 359.4f);
    CHECK(std::string(box) == "359");
    formatHeadingBox(box, sizeof box, 5.0f);
    CHECK(std::string(box) == "005");
    formatHeadingBox(box, sizeof box, -1.0f);
    CHECK(std::string(box) == "359");
}

TEST_CASE("hud: declutter levels decide what is visible") {
    const HudVisibility full = visibilityFor(Declutter::Full);
    const HudVisibility std_ = visibilityFor(Declutter::Std);
    const HudVisibility min = visibilityFor(Declutter::Min);
    // FULL shows everything.
    CHECK(full.headingTape);
    CHECK(full.statusRow1);
    CHECK(full.statusRow2);
    CHECK(full.row2FuelAndTime);
    CHECK(full.navBoxLines == 2);
    CHECK(ladderLabelVisible(full, 10));
    // STD: ladder numbers near the horizon go, nav box shrinks, row 2 loses fuel and time.
    CHECK(std_.statusRow1);
    CHECK(std_.statusRow2);
    CHECK_FALSE(std_.row2FuelAndTime);
    CHECK(std_.navBoxLines == 1);
    CHECK_FALSE(ladderLabelVisible(std_, 10));
    CHECK_FALSE(ladderLabelVisible(std_, -10));
    CHECK(ladderLabelVisible(std_, 20));
    CHECK(ladderLabelVisible(std_, -30));
    // MIN: attitude, speed, altitude, FPV and warnings only.
    CHECK_FALSE(min.headingTape);
    CHECK_FALSE(min.statusRow1);
    CHECK_FALSE(min.statusRow2);
    CHECK_FALSE(min.fma);
    CHECK(min.navBoxLines == 0);
    // Cycle FULL -> STD -> MIN -> FULL.
    CHECK(nextDeclutter(Declutter::Full) == Declutter::Std);
    CHECK(nextDeclutter(Declutter::Std) == Declutter::Min);
    CHECK(nextDeclutter(Declutter::Min) == Declutter::Full);
    CHECK(std::string(declutterName(Declutter::Std)) == "STD");
}

TEST_CASE("hud: HudDeclutter action cycles the level and posts feedback") {
    HudOptions opt;
    MessageQueue q;
    ActionState a;
    CHECK_FALSE(Hud::applyActions(a, opt, &q));
    CHECK(opt.declutter == Declutter::Full);
    a.pressed[static_cast<int>(Action::HudDeclutter)] = true;
    CHECK(Hud::applyActions(a, opt, &q));
    CHECK(opt.declutter == Declutter::Std);
    CHECK(std::string(q.newest(0).text) == "HUD STD");
    CHECK(Hud::applyActions(a, opt, nullptr));
    CHECK(opt.declutter == Declutter::Min);
    CHECK(Hud::applyActions(a, opt, &q));
    CHECK(opt.declutter == Declutter::Full);
}

TEST_CASE("hud: throttle field formatting") {
    ControlInputs c;
    char buf[32];
    c.throttleNorm[0] = 0.72f;
    formatThrottle(buf, sizeof buf, c, 1);
    CHECK(std::string(buf) == "THR 72%");
    c.throttleNorm[1] = 0.70f;
    formatThrottle(buf, sizeof buf, c, 2);
    CHECK(std::string(buf) == "THR 72/70%");
    c.throttleNorm[1] = 0.72f;
    formatThrottle(buf, sizeof buf, c, 2);
    CHECK(std::string(buf) == "THR 72%");  // levers together: one number
    c.reverseThrust = true;
    c.throttleNorm[0] = c.throttleNorm[1] = 0.60f;
    formatThrottle(buf, sizeof buf, c, 2);
    CHECK(std::string(buf) == "THR REV 60%");
    c.reverseThrust = false;
    c.throttleNorm[0] = 1.0f;
    c.throttleNorm[1] = 0.5f;
    c.throttleNorm[2] = 0.0f;
    c.throttleNorm[3] = 1.0f;
    formatThrottle(buf, sizeof buf, c, 4);
    CHECK(std::string(buf) == "THR 100/50/0/100%");
}

TEST_CASE("hud: flap label, moving blink and Vfe caution") {
    const AircraftProfile p = makeJet();
    AircraftState s = airborneState();
    char buf[16];
    s.flapDetent = 5;
    formatFlapLabel(buf, sizeof buf, p, s.flapDetent);
    CHECK(std::string(buf) == "FLAP 25");
    s.flapDetent = 0;
    formatFlapLabel(buf, sizeof buf, p, s.flapDetent);
    CHECK(std::string(buf) == "FLAP UP");
    // Surface still travelling toward the selected detent: label blinks.
    s.flapDetent = 3;
    s.flapPosNorm = 0.2f;
    CHECK(flapsMoving(s, p));
    s.flapPosNorm = p.flapDetents[3].cmdNorm;
    CHECK_FALSE(flapsMoving(s, p));
    // Amber above Vfe with flaps out; never with flaps up.
    s.iasKt = 200;
    CHECK(flapsOverVfe(s, p));
    s.flapDetent = 0;
    s.flapPosNorm = 0;
    CHECK_FALSE(flapsOverVfe(s, p));

    // Row 1 carries the blink flag, the amber role and the detent bar.
    s.flapDetent = 3;
    s.flapPosNorm = 0.2f;
    s.iasKt = 150;
    const ControlInputs c;
    StatusField f[kMaxStatusFields];
    const StatusInputs in{&s, &c, &p, 0};
    const int n = buildStatusRow1(in, f);
    int flapIdx = -1;
    for (int i = 0; i < n; ++i)
        if (std::string(f[i].segs[0].text).rfind("FLAP", 0) == 0) flapIdx = i;
    REQUIRE(flapIdx >= 0);
    CHECK(f[flapIdx].segs[0].blinks);
    CHECK(f[flapIdx].segs[0].role == Role::Normal);
    CHECK(f[flapIdx].segs[1].kind == StatusSeg::Kind::FlapBar);
    CHECK(f[flapIdx].segs[1].a == 7);  // 8 detents -> 7 boxes
    CHECK(f[flapIdx].segs[1].b == 3);
    s.iasKt = 210;
    buildStatusRow1(in, f);
    CHECK(f[flapIdx].segs[0].role == Role::Amber);
}

TEST_CASE("hud: gear states") {
    const AircraftProfile jet = makeJet();
    const AircraftProfile c172 = makeC172();
    AircraftState s = airborneState();
    const ControlInputs c;
    StatusField f[kMaxStatusFields];
    auto gearField = [&](const AircraftProfile& p) -> StatusField {
        const StatusInputs in{&s, &c, &p, 0};
        const int n = buildStatusRow1(in, f);
        for (int i = 0; i < n; ++i)
            if (std::string(f[i].segs[0].text).rfind("GEAR", 0) == 0) return f[i];
        return StatusField{};
    };
    s.gearRetractable = false;
    StatusField g = gearField(c172);
    CHECK(std::string(g.segs[0].text) == "GEAR FIXED");
    CHECK(g.segs[0].role == Role::Dim);

    s.gearRetractable = true;
    s.gearPosNorm = 1.0f;
    g = gearField(jet);
    CHECK(std::string(g.segs[0].text) == "GEAR DN");
    CHECK(g.segs[0].role == Role::Normal);
    REQUIRE(g.segCount == 2);
    CHECK(g.segs[1].kind == StatusSeg::Kind::GearLights);
    CHECK(g.segs[1].a == 3);

    s.gearPosNorm = 0.5f;
    g = gearField(jet);
    CHECK(g.segs[0].role == Role::Amber);
    CHECK(std::string(g.segs[0].text).rfind("GEAR", 0) == 0);

    s.gearPosNorm = 0.0f;
    g = gearField(jet);
    CHECK(std::string(g.segs[0].text) == "GEAR UP");
    CHECK(g.segs[0].role == Role::White);
}

TEST_CASE("hud: trim text and side trims") {
    char buf[32];
    formatTrim(buf, sizeof buf, 0.12f);
    CHECK(std::string(buf) == "TRIM " FSIM_SYM_ARROW_UP " +0.12");
    formatTrim(buf, sizeof buf, -0.30f);
    CHECK(std::string(buf) == "TRIM " FSIM_SYM_ARROW_DOWN " -0.30");
    formatTrim(buf, sizeof buf, -0.001f);
    CHECK(std::string(buf).find("-0.00") == std::string::npos);
    formatSideTrim(buf, sizeof buf, -0.05f);
    CHECK(std::string(buf) == "L0.05");
    formatSideTrim(buf, sizeof buf, 0.2f);
    CHECK(std::string(buf) == "R0.20");
    formatSideTrim(buf, sizeof buf, 0.001f);
    CHECK(std::string(buf).empty());

    // Rudder trim appears in row 1 only when non-zero.
    const AircraftProfile p = makeC172();
    AircraftState s = airborneState();
    const ControlInputs c;
    StatusField f[kMaxStatusFields];
    const StatusInputs in{&s, &c, &p, 0};
    int n = buildStatusRow1(in, f);
    CHECK(joined(f[n - 1]).find("RTRIM") == std::string::npos);
    s.rudderTrimNorm = -0.05f;
    n = buildStatusRow1(in, f);
    CHECK(joined(f[n - 1]).find("RTRIM L0.05") != std::string::npos);
}

TEST_CASE("hud: engine readouts per type, stopped engines show OFF") {
    const AircraftProfile jet = makeJet();
    const AircraftProfile pis = makeC172();
    const ControlInputs c;
    StatusField f[kMaxStatusFields];

    AircraftState s = airborneState();
    s.engineCount = 2;
    s.engines[0].running = s.engines[1].running = true;
    s.engines[0].n1Pct = 84.2f;
    s.engines[1].n1Pct = 84.0f;
    StatusInputs in{&s, &c, &jet, 0};
    buildStatusRow1(in, f);
    CHECK(joined(f[1]) == "N1 84.2 84.0");

    s.engines[1].running = false;
    buildStatusRow1(in, f);
    CHECK(joined(f[1]) == "N1 84.2 OFF");
    CHECK(f[1].segs[2].role == Role::Amber);

    AircraftState sp = airborneState();
    sp.engineCount = 1;
    sp.engines[0].running = true;
    sp.engines[0].rpm = 2347;
    in = StatusInputs{&sp, &c, &pis, 0};
    buildStatusRow1(in, f);
    CHECK(joined(f[1]) == "RPM 2350");
    sp.engines[0].running = false;
    buildStatusRow1(in, f);
    CHECK(joined(f[1]) == "RPM OFF");
}

TEST_CASE("hud: row 2 fields and formatting") {
    const AircraftProfile jet = makeJet();
    AircraftState s = airborneState();
    s.engineCount = 2;
    s.fuelTotalLbs = 9843;
    s.windFromTrueDeg = 270;
    s.windKt = 15.2f;
    s.simTimeS = 42 * 60 + 10;
    ControlInputs c;
    StatusField f[kMaxStatusFields];
    StatusInputs in{&s, &c, &jet, 10.0f};  // 10 E variation: 270 true = 260 magnetic
    CHECK(buildStatusRow2(in, f) == kRow2Fields);
    CHECK(joined(f[kFuel]) == "FUEL 9 840 LB");
    CHECK(joined(f[kWind]) == "WIND 260/15");
    CHECK(joined(f[kTime]) == "T+00:42:10");
    // Speedbrake is dim at 0 on an aircraft that has one, hidden fields are absent.
    CHECK(f[kSpdBrk].present);
    CHECK(f[kSpdBrk].segs[0].role == Role::Dim);
    CHECK_FALSE(f[kRev].present);
    CHECK_FALSE(f[kBrk].present);
    CHECK_FALSE(f[kPark].present);

    s.speedbrakePosNorm = 0.5f;
    buildStatusRow2(in, f);
    CHECK(joined(f[kSpdBrk]) == "SPDBRK 50%");
    CHECK(f[kSpdBrk].segs[0].role == Role::White);
    c.throttleNorm[0] = 0.8f;
    buildStatusRow2(in, f);
    CHECK(f[kSpdBrk].segs[0].role == Role::Amber);  // extended with more than 50% throttle

    // Reversers: amber in flight, green on the ground.
    s.engines[0].reverserDeployed = true;
    buildStatusRow2(in, f);
    CHECK(f[kRev].present);
    CHECK(f[kRev].segs[0].role == Role::Amber);
    s.wowAny = true;
    buildStatusRow2(in, f);
    CHECK(f[kRev].segs[0].role == Role::Normal);

    // Wheel brakes vs parking brake.
    s.brakeLeftNorm = 0.6f;
    buildStatusRow2(in, f);
    CHECK(f[kBrk].present);
    s.brakeLeftNorm = 1.0f;  // the sim reports parking brake pressure as brake
    s.parkingBrake = true;
    buildStatusRow2(in, f);
    CHECK(f[kPark].present);
    CHECK_FALSE(f[kBrk].present);

    s.windKt = 0.2f;
    buildStatusRow2(in, f);
    CHECK(joined(f[kWind]) == "WIND CALM");
}

TEST_CASE("hud: readout formatting helpers") {
    char b[24];
    formatMach(b, sizeof b, 0.78f);
    CHECK(std::string(b) == "M.78");
    formatMach(b, sizeof b, 1.02f);
    CHECK(std::string(b) == "M1.02");
    formatVs(b, sizeof b, 720.0f);
    CHECK(std::string(b) == "VS +700");
    formatVs(b, sizeof b, -1480.0f);
    CHECK(std::string(b) == "VS -1500");
    formatVs(b, sizeof b, 10.0f);
    CHECK(std::string(b) == "VS 0");
    formatThousands(b, sizeof b, 9840);
    CHECK(std::string(b) == "9 840");
    formatThousands(b, sizeof b, 123456);
    CHECK(std::string(b) == "123 456");
    formatThousands(b, sizeof b, 200);
    CHECK(std::string(b) == "200");
    formatFlightTime(b, sizeof b, 3 * 3600 + 5 * 60 + 9.9);
    CHECK(std::string(b) == "T+03:05:09");
    CHECK(roundRadioAltFt(452) == 450);
    CHECK(roundRadioAltFt(87) == 85);
    CHECK(roundRadioAltFt(234) == 230);
    CHECK(roundRadioAltFt(-3) == 0);
    char th[8], rest[8];
    formatAltBoxThousands(th, sizeof th, 35010);
    formatAltBoxRest(rest, sizeof rest, 35010);
    CHECK(std::string(th) == "35");
    CHECK(std::string(rest) == "020");  // 20 ft steps
    formatAltBoxThousands(th, sizeof th, 840);
    CHECK(std::string(th).empty());
}

TEST_CASE("hud: warning priority order") {
    const AircraftProfile p = makeJet();
    AircraftState s = airborneState();
    ControlInputs c;
    HudOptions opt;
    CHECK(activeWarnings(s, c, p, opt) == 0);

    s.crashed = true;
    std::strncpy(s.crashReason, "hard landing", sizeof s.crashReason - 1);
    s.stallWarning = true;
    s.overspeed = true;
    opt.paused = true;
    opt.extraWarnings = warningBit(WarningId::PullUp) | warningBit(WarningId::ApOff);
    const std::uint32_t mask = activeWarnings(s, c, p, opt);
    WarningId list[10];
    const int n = sortedWarnings(mask, list, 10);
    REQUIRE(n == 6);
    CHECK(list[0] == WarningId::Crash);
    CHECK(list[1] == WarningId::PullUp);
    CHECK(list[2] == WarningId::Stall);
    CHECK(list[3] == WarningId::Overspeed);
    CHECK(list[4] == WarningId::ApOff);
    CHECK(list[5] == WarningId::Paused);
    char text[40];
    formatWarning(text, sizeof text, WarningId::Crash, s);
    CHECK(std::string(text) == "CRASH: hard landing");
    CHECK(warningSeverity(WarningId::Stall) == Severity::Red);
    CHECK(warningSeverity(WarningId::Gear) == Severity::Amber);
    CHECK(warningSeverity(WarningId::ParkBrk) == Severity::Amber);
    CHECK(warningSeverity(WarningId::Paused) == Severity::White);
    CHECK(sortedWarnings(mask, list, 2) == 2);  // respects the output limit
}

TEST_CASE("hud: stall and overspeed come from the state or from the profile limits") {
    const AircraftProfile p = makeC172();
    AircraftState s = airborneState();
    const ControlInputs c;
    const HudOptions opt;
    s.alphaDeg = 15;
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::Stall)) != 0);
    s.wowAny = true;  // on the ground alpha means nothing
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::Stall)) == 0);
    s.wowAny = false;
    s.alphaDeg = 5;
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::Stall)) == 0);
    s.iasKt = 170;  // above Vne 163
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::Overspeed)) != 0);
}

TEST_CASE("hud: gear warning rules") {
    const AircraftProfile p = makeJet();
    AircraftState s = airborneState();
    s.gearRetractable = true;
    s.gearPosNorm = 0;
    ControlInputs c;
    c.gearDown = false;
    // High and clean: no warning.
    s.altAglFt = 5000;
    s.iasKt = 250;
    CHECK_FALSE(gearWarningActive(s, c, p));
    // Gear up with flaps at the landing detent.
    s.flapDetent = p.approachFlapDetent;
    CHECK(gearWarningActive(s, c, p));
    s.flapDetent = 0;
    // Low and slow and not climbing.
    s.altAglFt = 600;
    s.iasKt = 150;
    s.vsFpm = -600;
    CHECK(gearWarningActive(s, c, p));
    s.vsFpm = 1500;  // climbing out after takeoff: no nuisance warning
    CHECK_FALSE(gearWarningActive(s, c, p));
    s.vsFpm = -600;
    // Gear commanded down (in transit) or on the ground: no warning.
    c.gearDown = true;
    CHECK_FALSE(gearWarningActive(s, c, p));
    c.gearDown = false;
    s.wowAny = true;
    CHECK_FALSE(gearWarningActive(s, c, p));
    // Fixed gear never warns.
    s.wowAny = false;
    AircraftProfile fixed = makeC172();
    s.gearRetractable = false;
    CHECK_FALSE(gearWarningActive(s, c, fixed));
}

TEST_CASE("hud: parking brake warning needs throttle above 50 percent") {
    const AircraftProfile p = makeC172();
    AircraftState s = airborneState();
    s.parkingBrake = true;
    s.wowAny = true;
    ControlInputs c;
    const HudOptions opt;
    c.throttleNorm[0] = 0.4f;
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::ParkBrk)) == 0);
    c.throttleNorm[0] = 0.6f;
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::ParkBrk)) != 0);
    s.parkingBrake = false;
    CHECK((activeWarnings(s, c, p, opt) & warningBit(WarningId::ParkBrk)) == 0);
}

TEST_CASE("hud: new warnings flash at 2 Hz for 3 s, then stay steady") {
    // On for the first half of each 0.5 s cycle.
    CHECK(warningFlashVisible(0.0));
    CHECK(warningFlashVisible(0.2));
    CHECK_FALSE(warningFlashVisible(0.3));
    CHECK_FALSE(warningFlashVisible(0.45));
    CHECK(warningFlashVisible(0.5));
    CHECK(warningFlashVisible(0.7));
    CHECK_FALSE(warningFlashVisible(0.8));
    CHECK_FALSE(warningFlashVisible(2.8));
    CHECK(warningFlashVisible(2.99 - 0.24));
    // After 3 s: steady.
    CHECK(warningFlashVisible(3.0));
    CHECK(warningFlashVisible(3.3));
    CHECK(warningFlashVisible(10.3));

    WarningTracker t;
    const std::uint32_t stall = warningBit(WarningId::Stall);
    t.update(0, 100.0);
    CHECK(t.ageS(WarningId::Stall, 100.0) > 1e6);
    t.update(stall, 101.0);
    CHECK(t.ageS(WarningId::Stall, 101.0) == doctest::Approx(0.0));
    t.update(stall, 101.4);  // still active: the age keeps counting
    CHECK(t.ageS(WarningId::Stall, 101.4) == doctest::Approx(0.4));
    CHECK_FALSE(t.visible(WarningId::Stall, 101.4));
    CHECK(t.visible(WarningId::Stall, 104.5));
    t.update(0, 105.0);  // cleared
    t.update(stall, 106.0);  // raised again: flashes again
    CHECK(t.ageS(WarningId::Stall, 106.0) == doctest::Approx(0.0));
}

TEST_CASE("hud: speed trend predicts the 10 s change") {
    SpeedTrend tr;
    float ias = 100;
    double t = 0;
    for (int i = 0; i < 600; ++i) {  // 10 s at 60 Hz, +1 kt/s
        tr.update(ias, t);
        ias += 1.0f / 60.0f;
        t += 1.0 / 60.0;
    }
    CHECK(tr.trend10sKt() == doctest::Approx(10.0f).epsilon(0.1));
    // Steady speed: no trend. A time jump resets it.
    SpeedTrend flat;
    for (int i = 0; i < 300; ++i) flat.update(120, i / 60.0);
    CHECK(std::fabs(flat.trend10sKt()) < 0.1f);
    tr.update(50, t + 5.0);
    CHECK(tr.trend10sKt() == doctest::Approx(0.0f));
    // Same sim time (paused) keeps the filter untouched.
    SpeedTrend p;
    for (int i = 0; i < 120; ++i) p.update(100 + i * 0.1f, i / 60.0);
    const float before = p.trend10sKt();
    p.update(500, 119 / 60.0);
    CHECK(p.trend10sKt() == doctest::Approx(before));
}

TEST_CASE("hud: speed bands come from the profile") {
    const AircraftProfile p = makeC172();
    SpeedBands b = computeSpeedBands(p, 0);
    CHECK(b.redBelowKt == doctest::Approx(48.0f));  // Vs1 clean
    CHECK(b.whiteLowKt == doctest::Approx(48.0f));
    CHECK(b.whiteHighKt == doctest::Approx(85.0f));
    CHECK(b.yellowLowKt == doctest::Approx(129.0f));
    CHECK(b.yellowHighKt == doctest::Approx(163.0f));
    CHECK(b.redAboveKt == doctest::Approx(163.0f));
    b = computeSpeedBands(p, 2);
    CHECK(b.redBelowKt == doctest::Approx(40.0f));  // Vs0 with flaps
    CHECK(b.whiteLowKt == doctest::Approx(40.0f));
    const AircraftProfile jet = makeJet();
    b = computeSpeedBands(jet, 0);
    CHECK(b.redAboveKt == doctest::Approx(340.0f));  // Vmo
    CHECK(b.yellowHighKt == b.yellowLowKt);          // no caution band
}

TEST_CASE("hud: messages fade out near the end of their life") {
    CHECK(messageAlpha(0.0) == doctest::Approx(1.0f));
    CHECK(messageAlpha(1.4) == doctest::Approx(1.0f));
    CHECK(messageAlpha(1.75) == doctest::Approx(0.5f));
    CHECK(messageAlpha(kMessageLifeS) == doctest::Approx(0.0f));
    CHECK(messageAlpha(5.0) == doctest::Approx(0.0f));
    CHECK(kMaxShownMessages == 3);
}
