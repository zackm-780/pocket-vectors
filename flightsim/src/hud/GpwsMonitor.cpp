#include "hud/GpwsMonitor.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/Geo.h"
#include "core/Units.h"
#include "hud/HudLogic.h"

namespace fsim {
namespace {

struct Pt {
    float x, y;
};

float interp(const Pt* pts, int n, float x) {
    if (x <= pts[0].x) return pts[0].y;
    for (int i = 1; i < n; ++i) {
        if (x <= pts[i].x) {
            const float f = (x - pts[i - 1].x) / (pts[i].x - pts[i - 1].x);
            return pts[i - 1].y + f * (pts[i].y - pts[i - 1].y);
        }
    }
    return pts[n - 1].y;
}

// Mode 1 envelopes: radio altitude ft -> sink rate fpm. Roughly Honeywell Mode 1, tuned so a
// normal 3 degree final (500 to 800 fpm) never alerts.
const Pt kSinkCaution[] = {{30, 1200}, {100, 1200}, {500, 1500}, {1000, 2000}, {2450, 4000}};
const Pt kSinkWarning[] = {{30, 1800}, {100, 1800}, {500, 2300}, {1000, 3000}, {2450, 5500}};

constexpr int kCalloutFt[9] = {2500, 1000, 500, 100, 50, 40, 30, 20, 10};

}  // namespace

float sinkCautionFpm(float raFt) {
    return interp(kSinkCaution, 5, raFt);
}
float sinkWarningFpm(float raFt) {
    return interp(kSinkWarning, 5, raFt);
}
// Clearance floors: caution 200 ft + 5 ft per kt (700 ft at 100 kt, capped 1,500 ft); warning is half.
float lookaheadCautionFloorFt(float gsKt) {
    return std::min(1500.0f, 200.0f + 5.0f * std::max(0.0f, gsKt));
}
float lookaheadWarningFloorFt(float gsKt) {
    return 0.5f * lookaheadCautionFloorFt(gsKt);
}

bool GpwsMonitor::Latch::step(bool cond, float dtS, float persistS, float holdS) {
    if (cond) {
        onS += dtS;
        offS = 0;
        if (onS >= persistS) active = true;
    } else {
        onS = 0;
        if (active) {
            offS += dtS;
            if (offS >= holdS) active = false, offS = 0;
        }
    }
    return active;
}

void GpwsMonitor::reset() {
    *this = GpwsMonitor();
}

bool GpwsMonitor::consumeEvent(GpwsEvent& out) {
    if (qSize_ == 0) return false;
    out = queue_[qHead_];
    qHead_ = (qHead_ + 1) % 16;
    --qSize_;
    return true;
}

void GpwsMonitor::push(GpwsEventKind kind, const char* text, int valueFt) {
    if (qSize_ == 16) {  // drop the oldest
        qHead_ = (qHead_ + 1) % 16;
        --qSize_;
    }
    GpwsEvent& e = queue_[(qHead_ + qSize_) % 16];
    e.kind = kind;
    e.valueFt = valueFt;
    std::strncpy(e.text, text, sizeof e.text - 1);
    e.text[sizeof e.text - 1] = 0;
    ++qSize_;
}

void GpwsMonitor::probeAirport(const AircraftState& s, const NavDatabase* navDb) {
    if (!navDb || clockS_ - lastProbeS_ < 1.0) return;
    lastProbeS_ = clockS_;
    airportNm_ = -1;
    const auto near = navDb->nearestAirports(s.latDeg, s.lonDeg, 1, kAirportFullFloorNm);
    if (!near.empty())
        airportNm_ = static_cast<float>(
            mToNm(greatCircleDistM(s.latDeg, s.lonDeg, near[0]->latDeg, near[0]->lonDeg)));
}

void GpwsMonitor::lookahead(const AircraftState& s, const TerrainQuery& terrain, float floorScale,
                            bool& caution, bool& warning) const {
    caution = warning = false;
    const float gsMps = static_cast<float>(ktToMps(s.gsKt));
    const float cosLat = static_cast<float>(std::cos(degToRad(s.latDeg)));
    const float northPerS = gsMps * std::cos(s.trackTrueRad);
    const float eastPerS = gsMps * std::sin(s.trackTrueRad);
    const float cautionFloor = lookaheadCautionFloorFt(s.gsKt) * floorScale;
    const float warnFloor = lookaheadWarningFloorFt(s.gsKt) * floorScale;
    const float climbFtPerS = vsSmoothFpm_ / 60.0f;
    constexpr double kMPerDeg = kEarthRadiusM * kRadPerDeg;
    for (int i = 1; i <= kLookaheadMaxSamples; ++i) {
        const float t = i * kLookaheadStepS;
        const double lat = s.latDeg + northPerS * t / kMPerDeg;
        const double lon = s.lonDeg + eastPerS * t / (kMPerDeg * std::max(0.05f, cosLat));
        const float terrFt = static_cast<float>(mToFt(terrain.elevationM(lat, lon)));
        const float pathFt = static_cast<float>(s.altMslFt) + climbFtPerS * t;
        if (pathFt < terrFt + cautionFloor) {
            caution = true;
            if (t <= kLookaheadWarningS && pathFt < terrFt + warnFloor) {
                warning = true;
                return;
            }
        }
    }
}

void GpwsMonitor::callouts(const AircraftState& s) {
    const float ra = s.altAglFt;
    if (s.wowAny) {
        prevRaFt_ = -1;
        return;
    }
    int lowest = -1;
    for (int i = 0; i < 9; ++i) {
        const float L = static_cast<float>(kCalloutFt[i]);
        if (ra > L * 1.25f + 15.0f) calloutFired_[i] = false;
        if (prevRaFt_ >= L && ra < L && !calloutFired_[i]) {
            calloutFired_[i] = true;
            lowest = i;
        }
    }
    if (lowest >= 0) {
        char buf[16];
        std::snprintf(buf, sizeof buf, "%d", kCalloutFt[lowest]);
        push(GpwsEventKind::Callout, buf, kCalloutFt[lowest]);
    }
    prevRaFt_ = ra;
}

void GpwsMonitor::update(const AircraftState& s, const AircraftProfile& profile,
                         const NavState* nav, const TerrainQuery& terrain,
                         const NavDatabase* navDb) {
    float dtS = lastSimS_ < 0 ? 0.0f : static_cast<float>(s.simTimeS - lastSimS_);
    lastSimS_ = s.simTimeS;
    if (dtS < 0 || dtS > 0.5f) dtS = 0;
    clockS_ += dtS;
    mask_ = 0;
    if (!s.valid || s.crashed) return;

    // Smoothed vertical speed (tau 2 s) so turbulence does not jitter the projected path.
    if (!haveVs_) vsSmoothFpm_ = s.vsFpm, haveVs_ = true;
    else vsSmoothFpm_ += (s.vsFpm - vsSmoothFpm_) * std::min(1.0f, dtS / 2.0f);

    callouts(s);

    const float ra = s.altAglFt;
    const bool inhibit = s.wowAny || ra < kGpwsMinRaFt;
    bool sinkCaut = false, sinkWarn = false;
    if (!inhibit && ra <= kGpwsSinkMaxRaFt) {
        const float sink = -s.vsFpm;
        sinkCaut = sink > sinkCautionFpm(ra);
        sinkWarn = sink > sinkWarningFpm(ra);
    }

    // Mode 2-lite suppression: stabilized ILS (GS valid within 1.3 dots) or near an airport.
    probeAirport(s, navDb);
    float floorScale = 1.0f;
    if (airportNm_ >= 0)
        floorScale = std::clamp((airportNm_ - kAirportInhibitNm) /
                                    (kAirportFullFloorNm - kAirportInhibitNm), 0.0f, 1.0f);
    const bool stable = nav && nav->onFinal && nav->gsValid &&
                        std::fabs(nav->gsDeflection) <= kStableGsDeflection &&
                        std::fabs(nav->cdiDeflection) <= 1.0f;
    terrainSuppressed_ = stable || floorScale <= 0.0f;
    const bool lookActive = !inhibit && !terrainSuppressed_ && s.gsKt > 30.0f;
    if (!lookActive) {
        lookCaution_ = lookWarning_ = false;
        lastLookS_ = -1e9;
        terrCaution_.clear();
        terrWarning_.clear();
    } else if (clockS_ - lastLookS_ >= kLookaheadPeriodS) {
        lastLookS_ = clockS_;
        lookahead(s, terrain, floorScale, lookCaution_, lookWarning_);
    }

    if (inhibit) {
        sinkCaution_.clear();
        sinkWarning_.clear();
    }
    const bool sc = inhibit ? false : sinkCaution_.step(sinkCaut, dtS, 1.0f, kGpwsHoldOffS);
    const bool sw = inhibit ? false : sinkWarning_.step(sinkWarn, dtS, 0.5f, kGpwsHoldOffS);
    const bool tc = lookActive && terrCaution_.step(lookCaution_, dtS, 0.0f, kGpwsHoldOffS);
    const bool tw = lookActive && terrWarning_.step(lookWarning_, dtS, 0.0f, kGpwsHoldOffS);

    int level = 0;
    if (sw || tw) level = 3;
    else if (tc) level = 2;
    else if (sc) level = 1;
    if (level > prevLevel_) {
        if (level == 3) push(GpwsEventKind::PullUp, "PULL UP");
        else if (level == 2) push(GpwsEventKind::Terrain, "TERRAIN");
        else push(GpwsEventKind::SinkRate, "SINK RATE");
    }
    prevLevel_ = level;
    mask_ = level == 3   ? warningBit(WarningId::PullUp)
            : level == 2 ? warningBit(WarningId::Terrain)
            : level == 1 ? warningBit(WarningId::SinkRate)
                         : 0u;

    // Mode 4-lite: too low, not in landing configuration (descending, slow, not on the ground).
    if (s.wowAny || ra > 1000.0f) gearMsgFired_ = flapsMsgFired_ = false;
    const float slowKt = profile.vappKt > 0 ? profile.vappKt + 40.0f
                                            : (profile.vs1Kt > 0 ? 2.0f * profile.vs1Kt : 0.0f);
    const bool descending = s.vsFpm < -100.0f;
    if (!s.wowAny && ra >= kGpwsMinRaFt && descending && slowKt > 0 && s.iasKt < slowKt) {
        const bool retractable = s.gearRetractable || profile.hasRetractableGear;
        if (retractable && s.gearPosNorm < 0.5f && ra < 500.0f && !gearMsgFired_) {
            gearMsgFired_ = true;
            push(GpwsEventKind::TooLowGear, "TOO LOW GEAR");
        } else if (s.gearPosNorm >= 0.5f && profile.approachFlapDetent > 0 &&
                   s.flapDetent < profile.approachFlapDetent && ra < 245.0f && !flapsMsgFired_) {
            flapsMsgFired_ = true;
            push(GpwsEventKind::TooLowFlaps, "TOO LOW FLAPS");
        }
    }
}

}  // namespace fsim
