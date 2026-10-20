#include "hud/NavHudLogic.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "hud/StrokeFont.h"

namespace fsim {

namespace {

void copyText(char* out, int n, const char* s) {
    if (n <= 0) return;
    std::snprintf(out, static_cast<size_t>(n), "%s", s);
}

float safe(float v) {
    return std::isfinite(v) ? v : 0.0f;
}

DevPos clampDots(float units) {
    DevPos p;
    const float lim = kDeviationClamp;
    const float u = std::max(-lim, std::min(lim, safe(units)));
    p.dots = u * kDotsPerFullScale;
    p.pegged = std::fabs(safe(units)) >= lim - 1e-4f;
    return p;
}

void formatDist(char* out, int n, float nm) {
    if (nm >= 100.0f)
        std::snprintf(out, static_cast<size_t>(n), "%.0f", nm);
    else
        std::snprintf(out, static_cast<size_t>(n), "%.1f", nm);
}

int roundDeg3(float deg) {  // 1..360
    float w = std::fmod(safe(deg), 360.0f);
    if (w < 0) w += 360.0f;
    int v = static_cast<int>(std::lround(w));
    if (v <= 0 || v > 360) v = 360;
    return v;
}

}  // namespace

// ---------------------------------------------------------------------------------------------
// FMA
// ---------------------------------------------------------------------------------------------

const char* latModeName(LatMode m) {
    switch (m) {
    case LatMode::Roll: return "ROL";
    case LatMode::Hdg: return "HDG";
    case LatMode::Nav: return "NAV";
    case LatMode::Loc: return "LOC";
    case LatMode::Off: break;
    }
    return "";
}

const char* vertModeName(VertMode m) {
    switch (m) {
    case VertMode::Pitch: return "PIT";
    case VertMode::Alt: return "ALT";
    case VertMode::AltCapture: return "ALT*";
    case VertMode::Vs: return "VS";
    case VertMode::Gs: return "G/S";
    case VertMode::Flc: return "FLC";
    case VertMode::Off: break;
    }
    return "";
}

const char* spdModeName(SpdMode m) {
    switch (m) {
    case SpdMode::Speed: return "SPD";
    case SpdMode::Retard: return "RETARD";
    case SpdMode::Off: break;
    }
    return "";
}

void formatVertActive(char* out, int n, VertMode m, float vsBugFpm) {
    if (m == VertMode::Vs) {
        const int v = roundToStep(safe(vsBugFpm), 50);
        std::snprintf(out, static_cast<size_t>(n), "VS %+d", v);
    } else {
        copyText(out, n, vertModeName(m));
    }
}

void buildFma(const AutopilotState& ap, FmaText& out) {
    out = FmaText{};
    copyText(out.spd, sizeof out.spd, spdModeName(ap.spdActive));
    copyText(out.lat, sizeof out.lat, latModeName(ap.latActive));
    if (ap.latArmed != LatMode::Off)
        std::snprintf(out.latArmed, sizeof out.latArmed, "(%s)", latModeName(ap.latArmed));
    formatVertActive(out.vert, sizeof out.vert, ap.vertActive, ap.vsBugFpm);
    if (ap.vertArmed != VertMode::Off)
        std::snprintf(out.vertArmed, sizeof out.vertArmed, "(%s)", vertModeName(ap.vertArmed));
    char* p = out.status;
    const int cap = static_cast<int>(sizeof out.status);
    int used = 0;
    auto add = [&](const char* s) {
        used +=
            std::snprintf(p + used, static_cast<size_t>(cap - used), "%s%s", used ? " " : "", s);
    };
    if (ap.apEngaged) add("AP");
    if (ap.fdOn) add("FD");
    if (ap.atEngaged) add("A/T");
}

void FmaChangeTracker::update(const AutopilotState& ap, double nowS) {
    const int cur[3] = {static_cast<int>(ap.spdActive), static_cast<int>(ap.latActive),
                        static_cast<int>(ap.vertActive)};
    if (have_) {
        for (int i = 0; i < 3; ++i)
            if (cur[i] != last_[i]) since_[i] = nowS;
    }
    for (int i = 0; i < 3; ++i) last_[i] = cur[i];
    have_ = true;
}

bool FmaChangeTracker::boxVisible(FmaColumn c, double nowS) const {
    const double age = nowS - since_[static_cast<int>(c)];
    return age >= 0 && age < kFmaChangeBoxS;
}

// ---------------------------------------------------------------------------------------------
// Deviation scales
// ---------------------------------------------------------------------------------------------

DevPos deviationDots(float deflection) {
    return clampDots(deflection);
}

DevPos vnavDots(float vnavDevFt) {
    return clampDots(safe(vnavDevFt) / kVnavFullScaleFt);
}

const char* cdiScaleName(CdiScale s) {
    switch (s) {
    case CdiScale::Enr: return "ENR";
    case CdiScale::Term: return "TERM";
    case CdiScale::Apr: return "APR";
    case CdiScale::None: break;
    }
    return "";
}

CdiScale cdiScaleFor(const NavState& nav) {
    if (nav.source != NavSource::Gps) return CdiScale::None;
    if (nav.cdiFullScaleNm > 0.0f) {
        if (nav.cdiFullScaleNm >= 1.5f) return CdiScale::Enr;
        if (nav.cdiFullScaleNm >= 0.65f) return CdiScale::Term;
        return CdiScale::Apr;
    }
    switch (nav.phase) {
    case NavPhase::Approach: return CdiScale::Apr;
    case NavPhase::Terminal:
    case NavPhase::Missed: return CdiScale::Term;
    default: break;
    }
    return CdiScale::Enr;
}

const char* navSourceName(NavSource s) {
    switch (s) {
    case NavSource::Gps: return "GPS";
    case NavSource::Loc: return "LOC";
    case NavSource::Vor: return "VOR";
    case NavSource::None: break;
    }
    return "";
}

ScaleState buildScales(const NavState& nav, const AutopilotState& ap) {
    ScaleState s;
    s.locVisible =
        nav.locValid || (nav.source == NavSource::Gps && nav.phase == NavPhase::Approach);
    if (s.locVisible) {
        s.loc = deviationDots(nav.cdiDeflection);
        s.locFilled = !(ap.latArmed == LatMode::Loc && ap.latActive != LatMode::Loc);
    }
    if (nav.gsValid) {
        s.gsVisible = true;
        s.gs = deviationDots(nav.gsDeflection);
        s.gsFilled = !(ap.vertArmed == VertMode::Gs && ap.vertActive != VertMode::Gs);
    } else if (nav.vnavValid) {
        s.gsVisible = true;
        s.gsIsVnav = true;
        s.gs = vnavDots(nav.vnavDevFt);
        s.gsFilled = false;
    }
    return s;
}

// ---------------------------------------------------------------------------------------------
// Heading marks and bugs
// ---------------------------------------------------------------------------------------------

float trueToMagDeg(float trueDeg, float magVarDeg) {
    float v = std::fmod(safe(trueDeg) - safe(magVarDeg), 360.0f);
    if (v < 0) v += 360.0f;
    return v;
}

TapeMark headingMark(float valueMagDeg, float centerMagDeg) {
    TapeMark m;
    const float d = angleDiffDeg(safe(valueMagDeg), safe(centerMagDeg));
    m.offScale = std::fabs(d) > kHeadingTapeHalfRangeDeg;
    m.dxPx = std::max(-kTapeHalfWidthPx, std::min(kTapeHalfWidthPx, d * kHeadingTapePxPerDeg));
    m.valueMagDeg = roundDeg3(valueMagDeg);
    return m;
}

BugPos tapeBug(const TapeGeom& g, float value, float centerValue) {
    BugPos b;
    const float y = g.yOf(safe(value), safe(centerValue));
    b.offScale = y < g.topY || y > g.bottomY;
    b.above = y < g.topY;
    b.y = std::max(g.topY, std::min(g.bottomY, y));
    return b;
}

FdOffset fdCueOffset(float pitchCmdDeg, float pitchDeg, float rollCmdDeg, float rollDeg,
                     float pxPerDeg) {
    FdOffset o;
    o.dy = -(safe(pitchCmdDeg) - safe(pitchDeg)) * pxPerDeg;
    o.dx = (safe(rollCmdDeg) - safe(rollDeg)) * kFdRollPxPerDeg;
    o.dx = std::max(-kFdMaxXPx, std::min(kFdMaxXPx, o.dx));
    o.dy = std::max(-kFdMaxYPx, std::min(kFdMaxYPx, o.dy));
    return o;
}

// ---------------------------------------------------------------------------------------------
// Minimums
// ---------------------------------------------------------------------------------------------

void MinimumsTracker::reset() {
    haveAlt_ = below_ = alert_ = false;
    firedS_ = -1e9;
}

void MinimumsTracker::update(const NavState& nav, float altMslFt, double nowS) {
    if (nav.minimumsFt <= 0.0f) {
        haveAlt_ = below_ = false;
        return;
    }
    const bool atOrBelow = altMslFt <= nav.minimumsFt;
    if (!haveAlt_) {
        haveAlt_ = true;
        below_ = atOrBelow;
        return;
    }
    if (!below_ && atOrBelow) {
        below_ = true;
        alert_ = true;
        firedS_ = nowS;
    } else if (below_ && altMslFt > nav.minimumsFt + 100.0f) {
        below_ = false;
    }
}

bool MinimumsTracker::calloutVisible(double nowS) const {
    const double age = nowS - firedS_;
    return age >= 0 && age < kMinimumsCalloutS;
}

bool MinimumsTracker::consumeAlert() {
    const bool a = alert_;
    alert_ = false;
    return a;
}

void formatMinimums(char* out, int n, const NavState& nav) {
    if (nav.minimumsFt <= 0.0f) {
        if (n > 0) out[0] = 0;
        return;
    }
    std::snprintf(out, static_cast<size_t>(n), "%s %d", nav.minimumsIsDa ? "DA" : "MDA",
                  static_cast<int>(std::lround(nav.minimumsFt)));
}

// ---------------------------------------------------------------------------------------------
// Nav box
// ---------------------------------------------------------------------------------------------

void formatEte(char* out, int n, float eteS) {
    if (!(eteS > 0.0f) || eteS >= 100.0f * 3600.0f) {
        copyText(out, n, "--:--");
        return;
    }
    const int t = static_cast<int>(eteS + 0.5f);
    if (t >= 3600)
        std::snprintf(out, static_cast<size_t>(n), "%d:%02d:%02d", t / 3600, (t / 60) % 60, t % 60);
    else
        std::snprintf(out, static_cast<size_t>(n), "%d:%02d", t / 60, t % 60);
}

void buildNavBox(const NavState& nav, int navBoxLines, float magVarDeg, NavBoxText& out) {
    out = NavBoxText{};
    if (navBoxLines <= 0) return;
    const int maxLines = navBoxLines >= 2 ? 3 : 1;
    char dist[16], ete[16], mins[16];
    formatDist(dist, sizeof dist, nav.distToWptNm);
    formatEte(ete, sizeof ete, nav.eteToWptS);
    formatMinimums(mins, sizeof mins, nav);
    const int dtk = roundDeg3(trueToMagDeg(nav.desiredTrackTrueDeg, magVarDeg));

    int n = 0;
    auto line = [&](const char* fmt, auto... args) {
        if (n < 3) std::snprintf(out.line[n], sizeof out.line[n], fmt, args...);
        ++n;
    };

    switch (nav.source) {
    case NavSource::Gps:
        line("GPS" FSIM_SYM_ARROW_RIGHT "%s %sNM", nav.activeWptIdent, dist);
        line("DTK %03d XTK %.1f%c ETE %s", dtk, std::fabs(nav.crossTrackNm),
             nav.crossTrackNm < 0 ? 'L' : 'R', ete);
        if (nav.phase == NavPhase::Missed)
            line("%s", "MISSED APPR");
        else if (mins[0])
            line("%s  %s", nav.approachName, mins);
        else if (nav.nextWptIdent[0] && nav.vnavValid && nav.targetAltFt > 0)
            line("NEXT %s %d", nav.nextWptIdent, static_cast<int>(std::lround(nav.targetAltFt)));
        else if (nav.nextWptIdent[0])
            line("NEXT %s", nav.nextWptIdent);
        break;
    case NavSource::Loc:
        line("LOC %s CRS %03d", nav.approachName, dtk);
        if (mins[0])
            line("DME %.1f  %s", nav.distToThresholdNm, mins);
        else
            line("DME %.1f", nav.distToThresholdNm);
        if (nav.phase == NavPhase::Missed) line("%s", "MISSED APPR");
        break;
    case NavSource::Vor:
        line("VOR %s %s", nav.activeWptIdent, nav.toFlag ? "TO" : "FROM");
        line("CRS %03d DME %s", dtk, dist);
        break;
    case NavSource::None: line("%s", "NO NAV"); break;
    }
    out.lines = std::min(n, maxLines);
}

void buildFmaNavSummary(const NavState& nav, char* out, int n) {
    if (n > 0) out[0] = 0;
    if (nav.source == NavSource::None) return;
    char dist[16], ete[16];
    formatDist(dist, sizeof dist, nav.distToWptNm);
    formatEte(ete, sizeof ete, nav.eteToWptS);
    const char* ident = nav.activeWptIdent[0] ? nav.activeWptIdent : nav.approachName;
    if (!ident[0]) return;
    std::snprintf(out, static_cast<size_t>(n), "%s %s %s NM %s", navSourceName(nav.source), ident,
                  dist, ete);
}

}  // namespace fsim
