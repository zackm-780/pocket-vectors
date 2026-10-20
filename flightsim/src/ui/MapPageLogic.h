#pragma once
// Moving map page, pure logic (WP H4, HUD.md section 7). No GL.
//  - MapProjection: lat/lon -> virtual-canvas px, north-up or track-up, zoom = outer range ring.
//  - Declutter rules by zoom (what airports / navaids / labels are shown).
//  - Leg styling by plan state (past / active / future / approach / missed).
//  - Waypoint info rows (distance, bearing, DTK, ETE, altitude constraint).

#include <cstddef>

#include "core/NavTypes.h"

namespace fsim {

constexpr int kMapZoomCount = 7;
constexpr float kMapZoomStepsNm[kMapZoomCount] = {2, 5, 10, 20, 40, 80, 160};
constexpr int kMapDefaultZoomIndex = 2;  // 10 NM

// Index of the step closest to `nm` (used by the --map flag).
int mapZoomIndexForNm(float nm);
// Step by delta, clamped.
int mapZoomStep(int index, int delta);

enum class MapOrientation { NorthUp, TrackUp };

// Geometry of the map on the canvas: the outer range ring has radius ringRadiusPx for zoomNm.
struct MapProjection {
    double centerLatDeg = 0, centerLonDeg = 0;  // geographic point drawn at (cx, cy)
    float zoomNm = 10;
    float ringRadiusPx = 190;
    float cx = 320, cy = 240;
    float rotationRad = 0;  // 0 north-up; track-up: the aircraft track (true), up on screen

    float pxPerNm() const { return ringRadiusPx / zoomNm; }
    // Local tangent-plane offsets (NM) of a point from the center: east, north.
    void offsetsNm(double latDeg, double lonDeg, float& eastNm, float& northNm) const;
    void toScreen(double latDeg, double lonDeg, float& x, float& y) const;
    void offsetToScreen(float eastNm, float northNm, float& x, float& y) const;
    // Inverse: geographic point under a screen position.
    void toLatLon(float x, float y, double& latDeg, double& lonDeg) const;
    // Screen angle (rad, 0 = up, clockwise) of a true course / bearing.
    float screenAngleOfTrue(float trueRad) const { return trueRad - rotationRad; }
};

// North-up centers the aircraft; track-up puts it low so more of the route ahead is visible.
MapProjection makeMapProjection(double acLatDeg, double acLonDeg, float trackTrueRad,
                                MapOrientation mode, float zoomNm, float canvasWidthPx);

// ---- declutter ---------------------------------------------------------------------------------

struct MapDeclutter {
    int maxAirports = 0, maxNavaids = 0;
};
MapDeclutter mapDeclutterCaps(float zoomNm);
bool mapShowAirport(AirportType type, float zoomNm);
bool mapShowAirportLabel(AirportType type, float zoomNm);
bool mapShowRunways(AirportType type, float zoomNm);
bool mapShowNavaid(NavaidType type, float zoomNm);
bool mapShowNavaidLabel(NavaidType type, float zoomNm);
bool mapShowConstraints(float zoomNm);  // altitude constraints next to plan waypoints
// Lower = more important (airport sort for the cap).
int mapAirportRank(AirportType type);

// ---- leg styling -------------------------------------------------------------------------------

enum class MapLegKind { Past, Active, Future, Approach, Missed };
struct MapLegStyle {
    MapLegKind kind = MapLegKind::Future;
    bool dashed = false;  // missed approach legs
    float widthPx = 2;
};
// activeLeg / approachStartLeg: -1 = none.
MapLegStyle mapLegStyle(int legIndex, int activeLeg, int approachStartLeg, bool missedApproach);

// ---- waypoint info -----------------------------------------------------------------------------

struct MapWptInfo {
    bool valid = false;
    int legIndex = -1;
    char ident[8] = {0};
    float distNm = 0;          // along the route from the aircraft to this fix
    float bearingTrueDeg = 0;  // direct bearing from the aircraft
    float bearingMagDeg = 0;
    float dtkMagDeg = 0;  // desired track of the leg ending at this fix
    bool hasDtk = false;
    float eteS = 0;  // 0 when ground speed is too low
    AltConstraint alt;
};

// Info for one plan leg (the fix it ends at). routeDistNm > 0 overrides the direct distance.
MapWptInfo mapWaypointInfo(const FlightPlan& plan, int legIndex, double acLatDeg, double acLonDeg,
                           float gsKt, float magVarDeg, float routeDistNm);
// Fills up to maxRows rows from firstLeg (-1 = the active leg); returns the count. Distances
// accumulate along the route (direct to the first, then plus each leg length).
int mapWaypointRows(const FlightPlan& plan, double acLatDeg, double acLonDeg, float gsKt,
                    float magVarDeg, MapWptInfo* rows, int maxRows, int firstLeg = -1);

// "3000", "3000A" (at or above), "3000B" (at or below), "3000-5000". Empty for None.
int mapFormatAlt(const AltConstraint& alt, char* out, size_t outSize);
// "12:34" or "1:05:10"; "--:--" for non-positive / absurd times.
void mapFormatEte(float eteS, char* out, size_t outSize);

}  // namespace fsim
