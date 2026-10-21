#include "hud/MiniNavLogic.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/Geo.h"
#include "core/Units.h"

namespace fsim {

float miniNavAutoRangeNm(float distToNextNm) {
    if (!(distToNextNm > 0.05f)) return 2.0f;
    return std::clamp(distToNextNm * 1.2f, 2.0f, 20.0f);
}

float miniNavRangeNm(float distToNextNm, float fixedRangeNm) {
    if (fixedRangeNm > 0.0f) return std::clamp(fixedRangeNm, 2.0f, 20.0f);
    return miniNavAutoRangeNm(distToNextNm);
}

bool miniNavShown(bool enabled, Declutter declutter, bool mapOpen) {
    return enabled && declutter != Declutter::Min && !mapOpen;
}

MiniNavReadout miniNavReadout(double latDeg, double lonDeg, float gsKt, const NavState* nav,
                              const FlightPlan* plan, float fixedRangeNm) {
    MiniNavReadout out;
    out.hasPlan = plan && !plan->legs.empty();
    if (nav && nav->activeWptIdent[0] != '\0') {
        std::snprintf(out.ident, sizeof out.ident, "%s", nav->activeWptIdent);
        out.distNm = std::max(0.0f, nav->distToWptNm);
        out.eteS = nav->eteToWptS;
        out.crossTrackNm = nav->crossTrackNm;
        out.activeLeg = nav->activeLegIndex;
    } else if (out.hasPlan) {
        int leg = plan->activeLeg;
        if (leg < 1) leg = 1;
        if (leg >= static_cast<int>(plan->legs.size()))
            leg = static_cast<int>(plan->legs.size()) - 1;
        const MapWptInfo info = mapWaypointInfo(*plan, leg, latDeg, lonDeg, gsKt, 0, -1);
        std::snprintf(out.ident, sizeof out.ident, "%s", info.ident);
        out.distNm = std::max(0.0f, info.distNm);
        out.eteS = info.eteS;
        out.activeLeg = leg;
        if (leg >= 1) {
            const Leg& prev = plan->legs[static_cast<size_t>(leg - 1)];
            const Leg& cur = plan->legs[static_cast<size_t>(leg)];
            out.crossTrackNm = static_cast<float>(
                mToNm(crossTrackM(LatLon{latDeg, lonDeg}, LatLon{prev.to.latDeg, prev.to.lonDeg},
                                  LatLon{cur.to.latDeg, cur.to.lonDeg})));
        }
    }
    if (out.eteS <= 0.0f && gsKt > 5.0f && out.distNm > 0.0f)
        out.eteS = static_cast<float>(eteS(out.distNm, gsKt));
    out.rangeNm = miniNavRangeNm(out.distNm, fixedRangeNm);
    return out;
}

MapProjection miniNavProjection(double latDeg, double lonDeg, float trackTrueRad, float rangeNm,
                                MiniNavBox box) {
    MapProjection p;
    p.centerLatDeg = latDeg;
    p.centerLonDeg = lonDeg;
    p.zoomNm = std::max(rangeNm, 0.5f);
    p.rotationRad = trackTrueRad;
    const float mapTop = box.y + 3.0f;
    const float mapH = box.h - 30.0f;
    const float mapW = box.w - 6.0f;
    p.cx = box.x + box.w * 0.5f;
    p.cy = mapTop + mapH * 0.72f;
    const float ahead = p.cy - mapTop - 4.0f;
    const float side = mapW * 0.5f - 4.0f;
    p.ringRadiusPx = std::max(8.0f, std::min(ahead, side));
    return p;
}

}  // namespace fsim
