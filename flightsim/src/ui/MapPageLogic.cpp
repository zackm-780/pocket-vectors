#include "ui/MapPageLogic.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/Geo.h"
#include "core/Units.h"
#include "nav/FlightPlan.h"

namespace fsim {

namespace {
constexpr double kNmPerDeg = 60.0;
}

int mapZoomIndexForNm(float nm) {
    int best = 0;
    float bestErr = 1e9f;
    for (int i = 0; i < kMapZoomCount; ++i) {
        const float err = std::fabs(std::log(kMapZoomStepsNm[i] / std::max(nm, 0.01f)));
        if (err < bestErr) {
            bestErr = err;
            best = i;
        }
    }
    return best;
}

int mapZoomStep(int index, int delta) { return std::clamp(index + delta, 0, kMapZoomCount - 1); }

void MapProjection::offsetsNm(double latDeg, double lonDeg, float& eastNm, float& northNm) const {
    const double midLat = degToRad(0.5 * (latDeg + centerLatDeg));
    eastNm = static_cast<float>(wrap180(lonDeg - centerLonDeg) * kNmPerDeg * std::cos(midLat));
    northNm = static_cast<float>((latDeg - centerLatDeg) * kNmPerDeg);
}

void MapProjection::offsetToScreen(float eastNm, float northNm, float& x, float& y) const {
    const float c = std::cos(rotationRad), s = std::sin(rotationRad);
    const float k = pxPerNm();
    x = cx + k * (eastNm * c - northNm * s);
    y = cy - k * (eastNm * s + northNm * c);
}

void MapProjection::toScreen(double latDeg, double lonDeg, float& x, float& y) const {
    float e, n;
    offsetsNm(latDeg, lonDeg, e, n);
    offsetToScreen(e, n, x, y);
}

void MapProjection::toLatLon(float x, float y, double& latDeg, double& lonDeg) const {
    const float k = pxPerNm();
    const float u = (x - cx) / k, v = -(y - cy) / k;  // rotated frame: right, up
    const float c = std::cos(rotationRad), s = std::sin(rotationRad);
    const float e = u * c + v * s, n = -u * s + v * c;
    latDeg = centerLatDeg + n / kNmPerDeg;
    lonDeg = centerLonDeg + e / (kNmPerDeg * std::cos(degToRad(0.5 * (latDeg + centerLatDeg))));
}

MapProjection makeMapProjection(double acLatDeg, double acLonDeg, float trackTrueRad,
                                MapOrientation mode, float zoomNm, float canvasWidthPx) {
    MapProjection p;
    p.centerLatDeg = acLatDeg;
    p.centerLonDeg = acLonDeg;
    p.zoomNm = zoomNm;
    p.cx = canvasWidthPx * 0.5f;
    if (mode == MapOrientation::TrackUp) {
        p.rotationRad = trackTrueRad;
        p.cy = 330.0f;
    } else {
        p.rotationRad = 0;
        p.cy = 240.0f;
    }
    return p;
}

// ---- declutter ---------------------------------------------------------------------------------

MapDeclutter mapDeclutterCaps(float zoomNm) {
    if (zoomNm <= 20) return {40, 30};
    if (zoomNm <= 40) return {30, 20};
    if (zoomNm <= 80) return {20, 14};
    return {14, 10};
}

int mapAirportRank(AirportType type) {
    switch (type) {
    case AirportType::Large: return 0;
    case AirportType::Medium: return 1;
    case AirportType::Small: return 2;
    default: return 3;
    }
}

bool mapShowAirport(AirportType type, float zoomNm) {
    switch (type) {
    case AirportType::Large: return true;
    case AirportType::Medium: return zoomNm <= 80;
    case AirportType::Small: return zoomNm <= 20;
    default: return zoomNm <= 10;  // seaplane / unknown (includes the synthetic XTST)
    }
}

bool mapShowAirportLabel(AirportType type, float zoomNm) {
    switch (type) {
    case AirportType::Large: return true;
    case AirportType::Medium: return zoomNm <= 40;
    default: return zoomNm <= 10;
    }
}

bool mapShowRunways(AirportType type, float zoomNm) {
    if (type == AirportType::Large) return zoomNm <= 40;
    if (type == AirportType::Medium) return zoomNm <= 20;
    return zoomNm <= 5;
}

bool mapShowNavaid(NavaidType type, float zoomNm) {
    switch (type) {
    case NavaidType::Vor:
    case NavaidType::VorDme:
    case NavaidType::Vortac: return zoomNm <= 80;
    case NavaidType::Ndb:
    case NavaidType::NdbDme: return zoomNm <= 20;
    default: return zoomNm <= 10;  // DME / TACAN only
    }
}

bool mapShowNavaidLabel(NavaidType type, float zoomNm) {
    switch (type) {
    case NavaidType::Vor:
    case NavaidType::VorDme:
    case NavaidType::Vortac: return zoomNm <= 40;
    default: return zoomNm <= 10;
    }
}

bool mapShowConstraints(float zoomNm) { return zoomNm <= 40; }

// ---- leg styling -------------------------------------------------------------------------------

MapLegStyle mapLegStyle(int legIndex, int activeLeg, int approachStartLeg, bool missedApproach) {
    MapLegStyle s;
    const bool isActive = legIndex == activeLeg;
    if (missedApproach) {
        s.dashed = true;
        s.kind = isActive ? MapLegKind::Active : MapLegKind::Missed;
    } else if (isActive) {
        s.kind = MapLegKind::Active;
    } else if (activeLeg >= 0 && legIndex < activeLeg) {
        s.kind = MapLegKind::Past;
    } else if (approachStartLeg >= 0 && legIndex >= approachStartLeg) {
        s.kind = MapLegKind::Approach;
    } else {
        s.kind = MapLegKind::Future;
    }
    s.widthPx = s.kind == MapLegKind::Active ? 3.0f : (s.kind == MapLegKind::Past ? 1.5f : 2.0f);
    return s;
}

// ---- waypoint info -----------------------------------------------------------------------------

MapWptInfo mapWaypointInfo(const FlightPlan& plan, int legIndex, double acLatDeg, double acLonDeg,
                           float gsKt, float magVarDeg, float routeDistNm) {
    MapWptInfo info;
    if (legIndex < 0 || legIndex >= static_cast<int>(plan.legs.size())) return info;
    const Leg& leg = plan.legs[static_cast<size_t>(legIndex)];
    info.valid = true;
    info.legIndex = legIndex;
    std::snprintf(info.ident, sizeof info.ident, "%s", leg.to.ident.c_str());
    const double directNm =
        mToNm(greatCircleDistM(acLatDeg, acLonDeg, leg.to.latDeg, leg.to.lonDeg));
    info.distNm = routeDistNm > 0 ? routeDistNm : static_cast<float>(directNm);
    info.bearingTrueDeg = static_cast<float>(
        radToDeg(initialBearingTrueRad(acLatDeg, acLonDeg, leg.to.latDeg, leg.to.lonDeg)));
    info.bearingMagDeg = static_cast<float>(wrap360(info.bearingTrueDeg - magVarDeg));
    if (auto dtk = legDtkTrueDeg(plan, legIndex)) {
        info.hasDtk = true;
        info.dtkMagDeg = static_cast<float>(wrap360(*dtk - magVarDeg));
    }
    info.eteS = gsKt > 5.0f ? static_cast<float>(eteS(info.distNm, gsKt)) : 0.0f;
    info.alt = leg.alt;
    return info;
}

int mapWaypointRows(const FlightPlan& plan, double acLatDeg, double acLonDeg, float gsKt,
                    float magVarDeg, MapWptInfo* rows, int maxRows, int firstLeg) {
    const int n = static_cast<int>(plan.legs.size());
    int start = firstLeg >= 0 ? firstLeg : plan.activeLeg;
    if (start < 1) start = 1;
    int count = 0;
    float acc = 0;
    for (int i = start; i < n && count < maxRows; ++i) {
        const Leg& leg = plan.legs[static_cast<size_t>(i)];
        if (count == 0)
            acc = static_cast<float>(
                mToNm(greatCircleDistM(acLatDeg, acLonDeg, leg.to.latDeg, leg.to.lonDeg)));
        else
            acc += static_cast<float>(legDistNm(plan, i));
        rows[count++] =
            mapWaypointInfo(plan, i, acLatDeg, acLonDeg, gsKt, magVarDeg, std::max(acc, 0.001f));
    }
    return count;
}

int mapFormatAlt(const AltConstraint& alt, char* out, size_t outSize) {
    const int a = static_cast<int>(alt.altFt + 0.5f), b = static_cast<int>(alt.alt2Ft + 0.5f);
    switch (alt.kind) {
    case AltConstraintKind::At: return std::snprintf(out, outSize, "%d", a);
    case AltConstraintKind::AtOrAbove: return std::snprintf(out, outSize, "%dA", a);
    case AltConstraintKind::AtOrBelow: return std::snprintf(out, outSize, "%dB", a);
    case AltConstraintKind::Between: return std::snprintf(out, outSize, "%d-%d", a, b);
    default: break;
    }
    if (outSize) out[0] = 0;
    return 0;
}

void mapFormatEte(float eteSeconds, char* out, size_t outSize) {
    if (!(eteSeconds > 0) || eteSeconds > 99.0f * 3600.0f) {
        std::snprintf(out, outSize, "--:--");
        return;
    }
    const int t = static_cast<int>(eteSeconds + 0.5f);
    if (t >= 3600)
        std::snprintf(out, outSize, "%d:%02d:%02d", t / 3600, (t / 60) % 60, t % 60);
    else
        std::snprintf(out, outSize, "%d:%02d", t / 60, t % 60);
}

}  // namespace fsim
