#include "hud/NavHudFake.h"

#include <cmath>
#include <cstring>

#include "hud/HudLogic.h"

namespace fsim {

namespace {

constexpr float kRadToDeg = 57.29578f;

void setText(char* dst, size_t n, const char* s) {
    std::strncpy(dst, s, n - 1);
    dst[n - 1] = 0;
}

}  // namespace

const char* hudFakeNames() {
    return "gps_enr gps_term gps_apr loc_gs loc_armed hdg_alt vs ap_off minimums fd";
}

bool hudFake(const std::string& name, int frame, const AircraftState& s, float magVarDeg,
             NavState& nav, AutopilotState& ap, std::uint32_t& extraWarnings) {
    NavState n;
    n.magVarDeg = magVarDeg;
    AutopilotState a;
    std::uint32_t warn = 0;
    const float hdgTrue = s.headingTrueRad * kRadToDeg;
    const float hdgMag = hdgTrue - magVarDeg;
    const float alt = static_cast<float>(s.altMslFt);
    auto gps = [&](NavPhase phase, float fullScaleNm, float cdi, float xtkNm) {
        n.source = NavSource::Gps;
        n.phase = phase;
        setText(n.activeWptIdent, sizeof n.activeWptIdent, "CEPIN");
        setText(n.nextWptIdent, sizeof n.nextWptIdent, "ARCHI");
        n.distToWptNm = 12.4f;
        n.eteToWptS = 372;
        n.desiredTrackTrueDeg = hdgTrue + 6;
        n.bearingToWptTrueDeg = hdgTrue + 14;
        n.crossTrackNm = xtkNm;
        n.cdiDeflection = cdi;
        n.cdiFullScaleNm = fullScaleNm;
        n.targetAltFt = 3000;
    };
    auto ils = [&](bool gs) {
        n.source = NavSource::Loc;
        n.phase = NavPhase::Approach;
        n.locValid = true;
        n.gsValid = gs;
        setText(n.approachName, sizeof n.approachName, "I05");
        n.desiredTrackTrueDeg = hdgTrue + 2;
        n.cdiDeflection = 0.35f;
        n.gsDeflection = -0.25f;
        n.distToThresholdNm = 3.2f;
        n.distToWptNm = 3.2f;
        setText(n.activeWptIdent, sizeof n.activeWptIdent, "FAF");
        n.minimumsFt = 1190;
        n.minimumsIsDa = true;
        n.onFinal = true;
    };
    auto autopilot = [&](LatMode lat, VertMode vert) {
        a.apEngaged = a.fdOn = a.atEngaged = true;
        a.latActive = lat;
        a.vertActive = vert;
        a.spdActive = SpdMode::Speed;
        a.hdgBugMagDeg = hdgMag + 20;
        a.altBugFt = std::round((alt + 800) / 100) * 100;
        a.spdBugKt = std::round(s.iasKt / 5) * 5 + 10;
        a.vsBugFpm = 700;
    };

    if (name == "gps_enr") {
        gps(NavPhase::Enroute, 2.0f, -0.4f, 0.8f);
        n.vnavValid = true;
        n.vnavDevFt = 120;
        autopilot(LatMode::Nav, VertMode::Alt);
    } else if (name == "gps_term") {
        gps(NavPhase::Terminal, 1.0f, 1.25f, -1.4f);  // pegged right
        autopilot(LatMode::Nav, VertMode::Vs);
        a.vsBugFpm = -500;
    } else if (name == "gps_apr") {
        gps(NavPhase::Approach, 0.3f, 0.6f, -0.2f);
        setText(n.approachName, sizeof n.approachName, "R05");
        n.minimumsFt = 1190;
        n.minimumsIsDa = false;
        n.vnavValid = true;
        n.vnavDevFt = -180;
        autopilot(LatMode::Nav, VertMode::Alt);
        a.vertArmed = VertMode::Gs;
    } else if (name == "loc_gs") {
        ils(true);
        autopilot(LatMode::Loc, VertMode::Gs);
    } else if (name == "loc_armed") {
        ils(true);
        n.cdiDeflection = 1.1f;
        autopilot(LatMode::Hdg, VertMode::Alt);
        a.latArmed = LatMode::Loc;
        a.vertArmed = VertMode::Gs;
    } else if (name == "hdg_alt") {
        autopilot(LatMode::Hdg, VertMode::Alt);
        a.fdPitchCmdDeg = 3;
        a.fdRollCmdDeg = 12;
        a.fdValid = true;
    } else if (name == "vs") {
        autopilot(LatMode::Roll, VertMode::Vs);
        a.vsBugFpm = -700;
        a.altBugFt = std::round((alt - 1500) / 100) * 100;
        a.vertArmed = VertMode::Alt;
        a.fdPitchCmdDeg = -4;
        a.fdRollCmdDeg = 0;
        a.fdValid = true;
    } else if (name == "ap_off") {
        a.disconnectWarning = true;
        warn = warningBit(WarningId::ApOff);
    } else if (name == "minimums") {
        ils(true);
        autopilot(LatMode::Loc, VertMode::Gs);
        // Starts above the minimums and crosses them at frame 60 (the callout lasts 3 s).
        n.minimumsFt = frame < 60 ? alt - 50.0f : alt + 50.0f;
    } else if (name == "fd") {
        a.fdOn = true;
        a.fdValid = true;
        a.latActive = LatMode::Hdg;
        a.vertActive = VertMode::Vs;
        a.vsBugFpm = 500;
        a.hdgBugMagDeg = hdgMag - 15;
        a.fdPitchCmdDeg = 6;
        a.fdRollCmdDeg = -15;
    } else {
        return false;
    }
    nav = n;
    ap = a;
    extraWarnings = warn;
    return true;
}

}  // namespace fsim
