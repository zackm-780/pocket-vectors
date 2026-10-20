#include "hud/HudLogic.h"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "core/Geo.h"
#include "core/Units.h"
#include "hud/StrokeFont.h"

namespace fsim {
namespace {

constexpr float kSideTrimThreshold = 0.005f;
constexpr float kFlapMovingToleranceNorm = 0.02f;
constexpr float kGearDownNorm = 0.99f;
constexpr float kGearUpNorm = 0.01f;

float maxThrottleNorm(const ControlInputs& c, int engines) {
    float m = 0;
    for (int i = 0; i < engines && i < kMaxEngines; ++i) m = std::max(m, c.throttleNorm[i]);
    return m;
}

int engineCountOf(const AircraftState& s, const AircraftProfile& p) {
    int n = s.engineCount > 0 ? s.engineCount : p.engineCount;
    return std::max(1, std::min(n, kMaxEngines));
}

void setText(StatusSeg& seg, Role role, const char* fmt, ...)
#if defined(__GNUC__) || defined(__clang__)
    __attribute__((format(printf, 3, 4)))
#endif
    ;
void setText(StatusSeg& seg, Role role, const char* fmt, ...) {
    seg.kind = StatusSeg::Kind::Text;
    seg.role = role;
    va_list ap;
    va_start(ap, fmt);
    std::vsnprintf(seg.text, sizeof seg.text, fmt, ap);
    va_end(ap);
}

StatusSeg& addSeg(StatusField& f) {
    StatusSeg& s = f.segs[std::min(f.segCount, kMaxSegs - 1)];
    if (f.segCount < kMaxSegs) ++f.segCount;
    s = StatusSeg{};
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// Options and declutter
// ---------------------------------------------------------------------------------------------

Declutter nextDeclutter(Declutter d) {
    switch (d) {
    case Declutter::Full: return Declutter::Std;
    case Declutter::Std: return Declutter::Min;
    case Declutter::Min: break;
    }
    return Declutter::Full;
}

const char* declutterName(Declutter d) {
    switch (d) {
    case Declutter::Full: return "FULL";
    case Declutter::Std: return "STD";
    case Declutter::Min: return "MIN";
    }
    return "FULL";
}

Color hudBaseColor(HudPalette c) {
    switch (c) {
    case HudPalette::White: return colors::kWhite;
    case HudPalette::Amber: return colors::kAmber;
    case HudPalette::Green: break;
    }
    return colors::kHudGreen;
}

HudVisibility visibilityFor(Declutter d) {
    HudVisibility v;
    switch (d) {
    case Declutter::Full: break;
    case Declutter::Std:
        v.row2FuelAndTime = false;
        v.navBoxLines = 1;
        v.ladderLabelMinAbsDeg = 10;
        break;
    case Declutter::Min:
        v.headingTape = false;
        v.speedAltReadouts = false;
        v.statusRow1 = v.statusRow2 = false;
        v.row2FuelAndTime = false;
        v.fma = false;
        v.navBoxLines = 0;
        v.ladderLabelMinAbsDeg = 10;
        break;
    }
    return v;
}

bool ladderLabelVisible(const HudVisibility& v, int pitchDeg) {
    return std::abs(pitchDeg) > v.ladderLabelMinAbsDeg;
}

// ---------------------------------------------------------------------------------------------
// Tapes and heading
// ---------------------------------------------------------------------------------------------

TickRange tickRange(const TapeGeom& g, float centerValue, int step) {
    TickRange r;
    if (step <= 0 || g.pxPerUnit <= 0) return r;
    const float lo = centerValue - (g.bottomY - g.centerY) / g.pxPerUnit;
    const float hi = centerValue + (g.centerY - g.topY) / g.pxPerUnit;
    r.first = static_cast<int>(std::ceil(lo / step)) * step;
    r.last = static_cast<int>(std::floor(hi / step)) * step;
    return r;
}

bool isLabelTick(int value, int labelStep) {
    if (labelStep <= 0) return false;
    return ((value % labelStep) + labelStep) % labelStep == 0;
}

float headingMagDeg(float headingTrueRad, float magVarDeg) {
    return static_cast<float>(wrap360(radToDeg(headingTrueRad) - magVarDeg));
}

float trackMagDeg(float trackTrueRad, float magVarDeg) {
    return headingMagDeg(trackTrueRad, magVarDeg);
}

float angleDiffDeg(float aDeg, float bDeg) {
    return static_cast<float>(wrap180(static_cast<double>(aDeg) - bDeg));
}

float headingTapeX(float valueMagDeg, float centerMagDeg, float centerXpx, float pxPerDeg) {
    return centerXpx + angleDiffDeg(valueMagDeg, centerMagDeg) * pxPerDeg;
}

int headingTapeLabel(int deg) {
    const int n = ((deg / 10) % 36 + 36) % 36;
    return n == 0 ? 36 : n;
}

void formatHeadingBox(char* out, int n, float headingMagDegIn) {
    const int d = static_cast<int>(std::lround(wrap360(headingMagDegIn))) % 360;
    std::snprintf(out, static_cast<size_t>(n), "%03d", d);
}

// ---------------------------------------------------------------------------------------------
// Speed bands and trend
// ---------------------------------------------------------------------------------------------

SpeedBands computeSpeedBands(const AircraftProfile& p, int flapDetent) {
    SpeedBands b;
    const float vsClean = p.vs1Kt > 0 ? p.vs1Kt : p.vs0Kt;
    b.redBelowKt = (flapDetent > 0 && p.vs0Kt > 0) ? p.vs0Kt : vsClean;
    float vfeKt = p.vfeKt;
    const int count = static_cast<int>(p.flapDetents.size());
    if (flapDetent > 0 && flapDetent < count && p.flapDetents[flapDetent].vfeKt > 0) {
        vfeKt = p.flapDetents[flapDetent].vfeKt;  // limit with the selected flaps
    } else {
        float maxKt = 0;
        for (const FlapDetent& d : p.flapDetents) maxKt = std::max(maxKt, d.vfeKt);
        if (maxKt > 0) vfeKt = maxKt;  // first flap notch allows the highest speed
    }
    if (vfeKt > 0 && count > 1) {
        b.whiteLowKt = std::max(p.vs0Kt, b.redBelowKt);
        b.whiteHighKt = vfeKt;
        if (b.whiteHighKt <= b.whiteLowKt) b.whiteLowKt = b.whiteHighKt = 0;
    }
    if (p.vnoKt > 0 && p.vneKt > p.vnoKt) {
        b.yellowLowKt = p.vnoKt;
        b.yellowHighKt = p.vneKt;
    }
    b.redAboveKt = p.vneKt > 0 ? p.vneKt : p.vmoKt;
    return b;
}

void SpeedTrend::update(float iasKt, double simTimeS) {
    constexpr double kTauS = 1.5, kMaxGapS = 0.5;
    const double dtS = simTimeS - lastTimeS_;
    if (!have_) {
        have_ = true;
        accelKtPerS_ = 0;
    } else if (dtS < 0 || dtS > kMaxGapS) {
        accelKtPerS_ = 0;  // time jump (restart, long frame): start over
    } else if (dtS <= 1e-6) {
        return;  // same sim time (paused): keep the last sample
    } else {
        const float raw = static_cast<float>((iasKt - lastIasKt_) / dtS);
        const float alpha = static_cast<float>(1.0 - std::exp(-dtS / kTauS));
        accelKtPerS_ += alpha * (raw - accelKtPerS_);
    }
    lastIasKt_ = iasKt;
    lastTimeS_ = simTimeS;
}

// ---------------------------------------------------------------------------------------------
// Readout formatting
// ---------------------------------------------------------------------------------------------

int roundToStep(float value, int step) {
    if (step <= 0) return static_cast<int>(std::lround(value));
    return static_cast<int>(std::lround(value / static_cast<float>(step))) * step;
}

void formatThousands(char* out, int n, int value) {
    char digits[16];
    std::snprintf(digits, sizeof digits, "%d", std::abs(value));
    const int len = static_cast<int>(std::strlen(digits));
    char buf[24];
    int k = 0;
    if (value < 0) buf[k++] = '-';
    for (int i = 0; i < len; ++i) {
        if (i > 0 && (len - i) % 3 == 0) buf[k++] = ' ';
        buf[k++] = digits[i];
    }
    buf[k] = 0;
    std::snprintf(out, static_cast<size_t>(n), "%s", buf);
}

void formatMach(char* out, int n, float mach) {
    const int hundredths = static_cast<int>(std::lround(mach * 100.0f));
    if (hundredths >= 100)
        std::snprintf(out, static_cast<size_t>(n), "M%d.%02d", hundredths / 100, hundredths % 100);
    else
        std::snprintf(out, static_cast<size_t>(n), "M.%02d", hundredths);
}

void formatVs(char* out, int n, float vsFpm) {
    const int v = roundToStep(vsFpm, 50);
    if (v == 0)
        std::snprintf(out, static_cast<size_t>(n), "VS 0");
    else
        std::snprintf(out, static_cast<size_t>(n), "VS %+d", v);
}

int roundRadioAltFt(float agl) {
    if (agl <= 0) return 0;
    if (agl < 100) return roundToStep(agl, 5);
    if (agl < 500) return roundToStep(agl, 10);
    return roundToStep(agl, 50);
}

void formatAltBoxThousands(char* out, int n, float altFt) {
    const int v = roundToStep(altFt, 20);
    const int thousands = std::abs(v) / 1000;
    if (thousands > 0)
        std::snprintf(out, static_cast<size_t>(n), "%s%d", v < 0 ? "-" : "", thousands);
    else
        std::snprintf(out, static_cast<size_t>(n), "%s", v < 0 ? "-" : "");
}

void formatAltBoxRest(char* out, int n, float altFt) {
    const int v = roundToStep(altFt, 20);
    std::snprintf(out, static_cast<size_t>(n), "%03d", std::abs(v) % 1000);
}

void formatFlightTime(char* out, int n, double simTimeS) {
    const long t = simTimeS > 0 ? static_cast<long>(simTimeS) : 0;
    std::snprintf(out, static_cast<size_t>(n), "T+%02ld:%02ld:%02ld", t / 3600, (t / 60) % 60,
                  t % 60);
}

// ---------------------------------------------------------------------------------------------
// Status rows
// ---------------------------------------------------------------------------------------------

void formatThrottle(char* out, int n, const ControlInputs& c, int engineCount) {
    engineCount = std::max(1, std::min(engineCount, kMaxEngines));
    int pct[kMaxEngines];
    bool same = true;
    for (int i = 0; i < engineCount; ++i) {
        pct[i] = roundToStep(std::max(0.0f, std::min(1.0f, c.throttleNorm[i])) * 100.0f, 1);
        same = same && pct[i] == pct[0];
    }
    const char* prefix = c.reverseThrust ? "THR REV " : "THR ";
    char vals[24] = {0};
    int k = 0;
    if (same) {
        k = std::snprintf(vals, sizeof vals, "%d", pct[0]);
    } else {
        for (int i = 0; i < engineCount && k < static_cast<int>(sizeof vals) - 4; ++i)
            k += std::snprintf(vals + k, sizeof vals - static_cast<size_t>(k), i ? "/%d" : "%d",
                               pct[i]);
    }
    std::snprintf(out, static_cast<size_t>(n), "%s%s%%", prefix, vals);
}

void formatFlapLabel(char* out, int n, const AircraftProfile& p, int detent) {
    const int count = static_cast<int>(p.flapDetents.size());
    if (detent < 0 || detent >= count) {
        std::snprintf(out, static_cast<size_t>(n), "FLAP --");
        return;
    }
    std::snprintf(out, static_cast<size_t>(n), "FLAP %s", p.flapDetents[detent].label);
}

bool flapsMoving(const AircraftState& s, const AircraftProfile& p) {
    const int count = static_cast<int>(p.flapDetents.size());
    if (s.flapDetent < 0 || s.flapDetent >= count) return false;
    return std::fabs(s.flapPosNorm - p.flapDetents[s.flapDetent].cmdNorm) > kFlapMovingToleranceNorm;
}

float flapVfeLimitKt(const AircraftState& s, const AircraftProfile& p) {
    const int count = static_cast<int>(p.flapDetents.size());
    int idx = std::max(0, std::min(s.flapDetent, count - 1));
    for (int i = count - 1; i > idx; --i) {  // the surface may still be further out than selected
        if (p.flapDetents[i].cmdNorm <= s.flapPosNorm + kFlapMovingToleranceNorm) {
            idx = i;
            break;
        }
    }
    if (idx > 0 && p.flapDetents[idx].vfeKt > 0) return p.flapDetents[idx].vfeKt;
    return p.vfeKt;
}

bool flapsOverVfe(const AircraftState& s, const AircraftProfile& p) {
    const float limitKt = flapVfeLimitKt(s, p);
    return limitKt > 0 && s.iasKt > limitKt && (s.flapDetent > 0 || s.flapPosNorm > 0.02f);
}

void formatTrim(char* out, int n, float elevTrimNorm) {
    const char* arrow = elevTrimNorm > 0.005f ? FSIM_SYM_ARROW_UP
                                              : (elevTrimNorm < -0.005f ? FSIM_SYM_ARROW_DOWN : " ");
    const float shown = std::fabs(elevTrimNorm) < 0.005f ? 0.0f : elevTrimNorm;  // no "-0.00"
    std::snprintf(out, static_cast<size_t>(n), "TRIM %s %+.2f", arrow, static_cast<double>(shown));
}

void formatSideTrim(char* out, int n, float trimNorm) {
    if (std::fabs(trimNorm) < kSideTrimThreshold) {
        if (n > 0) out[0] = 0;
        return;
    }
    std::snprintf(out, static_cast<size_t>(n), "%c%.2f", trimNorm < 0 ? 'L' : 'R',
                  std::fabs(trimNorm));
}

void formatWind(char* out, int n, float windFromTrueDeg, float windKt, float magVarDeg) {
    if (windKt < 0.5f) {
        std::snprintf(out, static_cast<size_t>(n), "WIND CALM");
        return;
    }
    int dir = static_cast<int>(std::lround(wrap360(windFromTrueDeg - magVarDeg))) % 360;
    if (dir == 0) dir = 360;
    std::snprintf(out, static_cast<size_t>(n), "WIND %03d/%d", dir,
                  static_cast<int>(std::lround(windKt)));
}

int segCells10(const StatusSeg& s) {
    switch (s.kind) {
    case StatusSeg::Kind::Text: return StrokeFont::cellCount(s.text) * 10;
    case StatusSeg::Kind::FlapBar: return s.a * 7;
    case StatusSeg::Kind::GearLights: return 24;
    case StatusSeg::Kind::TrimScale: return 40;
    }
    return 0;
}

int fieldCells10(const StatusField& f) {
    int w = 0;
    for (int i = 0; i < f.segCount; ++i) w += segCells10(f.segs[i]) + (i > 0 ? 10 : 0);
    return w;
}

int buildStatusRow1(const StatusInputs& in, StatusField* out) {
    const AircraftState& s = *in.state;
    const ControlInputs& c = *in.controls;
    const AircraftProfile& p = *in.profile;
    const int engines = engineCountOf(s, p);
    int count = 0;
    for (int i = 0; i < kMaxStatusFields; ++i) out[i] = StatusField{};

    // Throttle (commanded).
    {
        StatusField& f = out[count++];
        f.present = true;
        StatusSeg& seg = addSeg(f);
        seg.kind = StatusSeg::Kind::Text;
        formatThrottle(seg.text, sizeof seg.text, c, engines);
    }
    // Engines: jets N1 (turboprops too: no TQ/NP data in AircraftState), pistons RPM. A stopped
    // engine shows amber OFF in place of its number.
    {
        StatusField& f = out[count++];
        f.present = true;
        const bool piston = p.category == AircraftCategory::Piston;
        setText(addSeg(f), Role::Normal, piston ? "RPM" : "N1");
        for (int i = 0; i < engines; ++i) {
            const EngineState& e = s.engines[i];
            StatusSeg& seg = addSeg(f);
            if (!e.running) {
                setText(seg, Role::Amber, "OFF");
            } else if (piston) {
                setText(seg, Role::Normal, "%d", roundToStep(e.rpm, 10));
            } else {
                setText(seg, Role::Normal, "%.1f", static_cast<double>(e.n1Pct));
            }
        }
    }
    // Flaps: label + detent bar. Blinks while moving, amber above Vfe.
    if (p.flapDetents.size() > 1) {
        StatusField& f = out[count++];
        f.present = true;
        StatusSeg& label = addSeg(f);
        label.kind = StatusSeg::Kind::Text;
        formatFlapLabel(label.text, sizeof label.text, p, s.flapDetent);
        label.blinks = flapsMoving(s, p);
        label.role = flapsOverVfe(s, p) ? Role::Amber : Role::Normal;
        StatusSeg& bar = addSeg(f);
        bar.kind = StatusSeg::Kind::FlapBar;
        bar.a = static_cast<int>(p.flapDetents.size()) - 1;
        bar.b = std::max(0, std::min(s.flapDetent, bar.a));
        bar.role = label.role;
    }
    // Gear.
    {
        StatusField& f = out[count++];
        f.present = true;
        const bool retractable = s.gearRetractable || p.hasRetractableGear;
        if (!retractable) {
            setText(addSeg(f), Role::Dim, "GEAR FIXED");
        } else if (s.gearPosNorm >= kGearDownNorm) {
            setText(addSeg(f), Role::Normal, "GEAR DN");
            StatusSeg& dots = addSeg(f);
            dots.kind = StatusSeg::Kind::GearLights;
            dots.a = 3;
        } else if (s.gearPosNorm > kGearUpNorm) {
            setText(addSeg(f), Role::Amber, "GEAR " FSIM_SYM_ARROW_UP FSIM_SYM_ARROW_DOWN);
        } else {
            setText(addSeg(f), Role::White, "GEAR UP");
        }
    }
    // Trim: elevator text + mini scale, then rudder/aileron trim only when non-zero.
    {
        StatusField& f = out[count++];
        f.present = true;
        StatusSeg& t = addSeg(f);
        t.kind = StatusSeg::Kind::Text;
        formatTrim(t.text, sizeof t.text, s.elevTrimNorm);
        StatusSeg& scale = addSeg(f);
        scale.kind = StatusSeg::Kind::TrimScale;
        scale.a = static_cast<int>(std::lround(std::max(-1.0f, std::min(1.0f, s.elevTrimNorm)) * 100));
        char side[12];
        formatSideTrim(side, sizeof side, s.rudderTrimNorm);
        if (side[0]) setText(addSeg(f), Role::Normal, "RTRIM %s", side);
        formatSideTrim(side, sizeof side, s.aileronTrimNorm);
        if (side[0]) setText(addSeg(f), Role::Normal, "ATRIM %s", side);
    }
    return count;
}

int buildStatusRow2(const StatusInputs& in, StatusField* out) {
    const AircraftState& s = *in.state;
    const ControlInputs& c = *in.controls;
    const AircraftProfile& p = *in.profile;
    const int engines = engineCountOf(s, p);
    for (int i = 0; i < kMaxStatusFields; ++i) out[i] = StatusField{};
    static constexpr int kSlots[kRow2Fields] = {10, 3, 3, 4, 14, 12, 10};
    for (int i = 0; i < kRow2Fields; ++i) out[i].slotCells = kSlots[i];

    // Speedbrake / spoilers: shown dim at 0 on aircraft that have them, white when out, amber when
    // extended with more than 50 % throttle.
    if (p.hasSpeedbrake || s.speedbrakePosNorm > 0.01f) {
        StatusField& f = out[kSpdBrk];
        f.present = true;
        const int pct = roundToStep(s.speedbrakePosNorm * 100.0f, 1);
        Role role = Role::Dim;
        if (s.speedbrakePosNorm > 0.01f)
            role = maxThrottleNorm(c, engines) > 0.5f ? Role::Amber : Role::White;
        setText(addSeg(f), role, "SPDBRK %d%%", pct);
    }
    // Reversers: amber in flight, green on the ground.
    bool reverser = false;
    for (int i = 0; i < engines; ++i) reverser = reverser || s.engines[i].reverserDeployed;
    if (reverser) {
        StatusField& f = out[kRev];
        f.present = true;
        setText(addSeg(f), s.wowAny ? Role::Normal : Role::Amber, "REV");
    }
    // BRK = wheel brakes in use; the parking brake alone shows only PARK.
    const bool parked = s.parkingBrake || c.parkingBrake;
    const float pilotBrake = std::max(c.brakeLeftNorm, c.brakeRightNorm);
    const float stateBrake = std::max(s.brakeLeftNorm, s.brakeRightNorm);
    if (pilotBrake > 0.05f || (!parked && stateBrake > 0.05f)) {
        StatusField& f = out[kBrk];
        f.present = true;
        setText(addSeg(f), Role::Normal, "BRK");
    }
    if (parked) {
        StatusField& f = out[kPark];
        f.present = true;
        setText(addSeg(f), Role::White, "PARK");
    }
    {
        StatusField& f = out[kFuel];
        f.present = true;
        char num[16];
        formatThousands(num, sizeof num, roundToStep(s.fuelTotalLbs, 10));
        setText(addSeg(f), Role::Normal, "FUEL %s LB", num);
    }
    {
        StatusField& f = out[kWind];
        f.present = true;
        StatusSeg& seg = addSeg(f);
        seg.kind = StatusSeg::Kind::Text;
        formatWind(seg.text, sizeof seg.text, s.windFromTrueDeg, s.windKt, in.magVarDeg);
    }
    {
        StatusField& f = out[kTime];
        f.present = true;
        StatusSeg& seg = addSeg(f);
        seg.kind = StatusSeg::Kind::Text;
        formatFlightTime(seg.text, sizeof seg.text, s.simTimeS);
    }
    return kRow2Fields;
}

// ---------------------------------------------------------------------------------------------
// Warnings
// ---------------------------------------------------------------------------------------------

bool gearWarningActive(const AircraftState& s, const ControlInputs& c, const AircraftProfile& p) {
    const bool retractable = s.gearRetractable || p.hasRetractableGear;
    if (!retractable || c.gearDown || s.gearPosNorm >= 0.5f || s.wowAny) return false;
    // Landing flaps with the gear up.
    if (p.approachFlapDetent > 0 && s.flapDetent >= p.approachFlapDetent) return true;
    // Low and slow, not climbing away: below 1,000 ft AGL and under approach speed + 20 kt.
    const float slowKt = p.vappKt > 0 ? p.vappKt + 20.0f : (p.vs1Kt > 0 ? 1.5f * p.vs1Kt : 0.0f);
    return slowKt > 0 && s.altAglFt < 1000.0f && s.iasKt < slowKt && s.vsFpm < 200.0f;
}

std::uint32_t activeWarnings(const AircraftState& s, const ControlInputs& c,
                             const AircraftProfile& p, const HudOptions& opt) {
    std::uint32_t m = 0;
    const int engines = engineCountOf(s, p);
    if (s.crashed) m |= warningBit(WarningId::Crash);
    if (s.stallWarning ||
        (p.stallWarnAlphaDeg > 0 && !s.wowAny && s.alphaDeg > p.stallWarnAlphaDeg))
        m |= warningBit(WarningId::Stall);
    const float vneKt = p.vmoKt > 0 ? p.vmoKt : p.vneKt;
    if (s.overspeed || (vneKt > 0 && s.iasKt > vneKt) || (p.mmo > 0 && s.mach > p.mmo))
        m |= warningBit(WarningId::Overspeed);
    if (gearWarningActive(s, c, p)) m |= warningBit(WarningId::Gear);
    if ((s.parkingBrake || c.parkingBrake) && maxThrottleNorm(c, engines) > 0.5f)
        m |= warningBit(WarningId::ParkBrk);
    if (opt.paused) m |= warningBit(WarningId::Paused);
    m |= opt.extraWarnings &
         (warningBit(WarningId::PullUp) | warningBit(WarningId::Terrain) |
          warningBit(WarningId::SinkRate) | warningBit(WarningId::ApOff));
    return m;
}

int sortedWarnings(std::uint32_t mask, WarningId* out, int max) {
    int n = 0;
    for (int i = 0; i < static_cast<int>(WarningId::Count) && n < max; ++i)
        if (mask & (1u << i)) out[n++] = static_cast<WarningId>(i);
    return n;
}

Severity warningSeverity(WarningId id) {
    switch (id) {
    case WarningId::Crash:
    case WarningId::PullUp:
    case WarningId::Stall:
    case WarningId::Terrain:
    case WarningId::SinkRate:
    case WarningId::Overspeed: return Severity::Red;
    case WarningId::Gear:
    case WarningId::ApOff:
    case WarningId::ParkBrk: return Severity::Amber;
    case WarningId::Paused:
    case WarningId::Count: break;
    }
    return Severity::White;
}

void formatWarning(char* out, int n, WarningId id, const AircraftState& s) {
    const char* text = "";
    switch (id) {
    case WarningId::Crash:
        std::snprintf(out, static_cast<size_t>(n), s.crashReason[0] ? "CRASH: %s" : "CRASH",
                      s.crashReason);
        return;
    case WarningId::PullUp: text = "PULL UP"; break;
    case WarningId::Stall: text = "STALL"; break;
    case WarningId::Terrain: text = "TERRAIN"; break;
    case WarningId::SinkRate: text = "SINK RATE"; break;
    case WarningId::Overspeed: text = "OVERSPEED"; break;
    case WarningId::Gear: text = "GEAR"; break;
    case WarningId::ApOff: text = "AP OFF"; break;
    case WarningId::ParkBrk: text = "PARK BRK"; break;
    case WarningId::Paused: text = "PAUSED"; break;
    case WarningId::Count: break;
    }
    std::snprintf(out, static_cast<size_t>(n), "%s", text);
}

bool warningFlashVisible(double ageS) {
    if (ageS < 0 || ageS >= kWarningFlashDurationS) return true;
    const double phase = ageS * kWarningFlashHz;  // 2 Hz: on for the first half of each cycle
    return phase - std::floor(phase) < 0.5;
}

void WarningTracker::update(std::uint32_t mask, double nowS) {
    for (int i = 0; i < static_cast<int>(WarningId::Count); ++i) {
        const bool on = (mask & (1u << i)) != 0;
        if (on && !active_[i]) since_[i] = nowS;
        active_[i] = on;
    }
}

double WarningTracker::ageS(WarningId id, double nowS) const {
    const int i = static_cast<int>(id);
    return active_[i] ? nowS - since_[i] : 1e9;
}

float messageAlpha(double ageS) {
    if (ageS < 0) return 1.0f;
    if (ageS >= kMessageLifeS) return 0.0f;
    if (ageS > kMessageLifeS - kMessageFadeS)
        return static_cast<float>((kMessageLifeS - ageS) / kMessageFadeS);
    return 1.0f;
}

}  // namespace fsim
