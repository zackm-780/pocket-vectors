#pragma once
// HUD mini-nav logic (WP H7). No GL. The inset reuses MapProjection (track-up).

#include "core/NavState.h"
#include "hud/HudLogic.h"
#include "nav/FlightPlan.h"
#include "ui/MapPageLogic.h"

namespace fsim {

// Lower-left of the inner HUD: clear of the speed tape (the value box ends near x 96), below
// the heading tape, above the LOC scale (y 392) and the status rows (y 434). 140x140 virtual px.
struct MiniNavBox {
    float x = 118;
    float y = 214;
    float w = 140;
    float h = 140;
};

constexpr MiniNavBox kMiniNavBox{};

// 0 fixed range means auto (follow the next waypoint). Both results are clamped to 2..20 NM.
float miniNavAutoRangeNm(float distToNextNm);
float miniNavRangeNm(float distToNextNm, float fixedRangeNm);

// Off unless the setting is on. Hidden in HUD MIN and while the map page replaces the HUD.
bool miniNavShown(bool enabled, Declutter declutter, bool mapOpen);

struct MiniNavReadout {
    bool hasPlan = false;
    char ident[8] = {0};
    float distNm = 0;
    float eteS = 0;
    float crossTrackNm = 0;  // + = right of course
    float rangeNm = 2;
    int activeLeg = -1;
};

// `nav` may be null. Distance, ETE and cross-track come from NavState when it names a waypoint,
// otherwise from the active plan leg.
MiniNavReadout miniNavReadout(double latDeg, double lonDeg, float gsKt, const NavState* nav,
                              const FlightPlan* plan, float fixedRangeNm);

// Track-up. The aircraft sits low in the box so more of the route ahead is on screen.
// ringRadiusPx is the range ring (one ring = rangeNm).
MapProjection miniNavProjection(double latDeg, double lonDeg, float trackTrueRad, float rangeNm,
                                MiniNavBox box = kMiniNavBox);

}  // namespace fsim
