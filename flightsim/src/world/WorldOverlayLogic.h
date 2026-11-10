#pragma once
// W6: world-overlay decisions with no GL. The renderer (WorldOverlay) draws the batches this
// builds: route markers, the vertical path, nearby airports and runway outlines. Each batch is
// eye-relative ECEF triangles (x y z r g b), so one draw call covers a layer.

#include <vector>

#include "core/Geo.h"
#include "core/NavTypes.h"
#include "core/TerrainQuery.h"

namespace fsim::world {

// H10: the route and vertical-path layers follow the HUD declutter level in Auto (shown only at
// FULL, hidden at STD and MIN, so the overlay is what separates FULL from STD); On and Off force them.
enum class OverlayMode { Auto, On, Off };
const char* overlayModeName(OverlayMode m);  // AUTO / ON / OFF

struct OverlayToggles {
    // What buildWorldOverlay draws this frame (route / vpath come from resolveOverlay).
    bool route = false;
    bool vpath = false;
    bool airports = false;
    bool runways = false;
    // H10: the settings behind route and vpath.
    OverlayMode routeMode = OverlayMode::Auto;
    OverlayMode vpathMode = OverlayMode::Auto;
};

// H10: route / vpath from their modes: On shows, Off hides, Auto shows only at FULL declutter.
OverlayToggles resolveOverlay(const OverlayToggles& t, bool fullDeclutter);

// H10 presets (L3 and the pause-menu OVERLAYS row), on the modes and the airport / runway flags:
// 0 AUTO (route and path auto), 1 ROUTE+PATH (both on), 2 AIRPORTS (route and path auto, airports
// and runways on), 3 ALL (all on), 4 OFF (all off). Anything else is custom.
constexpr int kOverlayPresetCount = 5;

int overlayMatchingPreset(const OverlayToggles& t);  // -1 when the settings match no preset
const char* overlayPresetName(int preset);  // AUTO / ROUTE+PATH / AIRPORTS / ALL / OFF / CUSTOM
void overlayApplyPreset(OverlayToggles& t, int preset);
// Steps to the next preset. A custom mix goes to AUTO, then the presets wrap.
int overlayCycle(OverlayToggles& t);

// Constraint altitude when one is set, else cruise when it is clearly above the ground, else a
// short pole (ground + 400 ft). The route uses it only as the ceiling of overlayRouteProfileFt.
float overlayMarkerAltFt(const AltConstraint& alt, float cruiseFt, float groundFt);

// The leg the plan lands on: the last leg before the missed approach when an approach is loaded,
// else the last leg when it ends at an airport or runway. -1 when the plan ends at a fix.
int overlayLandingLeg(const FlightPlan& plan);

// Route and path slope between the fixed altitudes of overlayRouteProfileFt (the 3 degree
// glidepath), and the height over the threshold when the landing leg has no altitude.
constexpr float kOverlayPathDeg = 3.0f;
constexpr float kOverlayLandingAboveFt = 50.0f;

// One altitude (ft MSL) per leg end, so the route climbs and descends instead of jumping between
// cruise and the ground. Fixed: the departure runway (its ground), the landing leg (its constraint,
// else ground + 50 ft), every AT constraint and every constraint on an approach leg. Other legs
// sit on the lowest 3 degree line through the fixed ones, capped at cruise; AT OR ABOVE /
// AT OR BELOW / BETWEEN outside the approach clamp that, and free legs stay 400 ft above their
// ground. groundFt[i] is the terrain under leg i's fix (missing entries count as 0).
void overlayRouteProfileFt(const FlightPlan& plan, const std::vector<float>& groundFt,
                           std::vector<float>& outAltFt);

// World length, in meters, of `px` vertical pixels at `distM` for this lens.
float metersForPixels(float distM, float px, float viewH, float vFovRad);
// Half-width of a runway highlight: the true half-width, or wider so it stays `minPx` pixels.
float overlayRunwayHalfWidthM(float trueHalfWidthM, float distM, float minPx, float viewH,
                              float vFovRad);

// Close to the field the runway outlines, the approach path and the route would box in the view:
// they clear once the eye is less than this height above the runway (or the approach leg's end;
// for the route, the landing leg's end while flying the approach or the landing leg).
constexpr float kOverlayNearFieldHideFt = 250.0f;
bool overlayHiddenNearField(double eyeAltMslM, float fieldElevFt);

// `distNm[i]` is the great-circle distance of candidate i. Keeps (hideInsideNm, radiusNm],
// nearest first, at most maxCount.
void selectOverlayAirports(const std::vector<double>& distNm, double radiusNm, double hideInsideNm,
                           int maxCount, std::vector<int>& outIndices);

// Axis-aligned box in virtual pixels (top-left). placeOverlayLabel nudges a center point so its
// box stays on the canvas and misses the boxes already placed. x/y are the box center.
struct OverlayBox {
    float x = 0, y = 0, w = 0, h = 0;
};
bool placeOverlayLabel(float& x, float& y, float w, float h, const OverlayBox* placed,
                       int placedCount, float canvasW, float canvasH);

// Eye used to place vertices and labels. Vertices are float(ecef - eye.ecef).
struct OverlayEye {
    double latDeg = 0, lonDeg = 0, altMslM = 0;
    Ecef ecef;
    Mat3d ecefToEye;  // ECEF vector -> eye axes (x right, y up, -z forward)
    float vFovRad = 0.96f;
    float aspect = 4.0f / 3.0f;
};

// Level attitude at headingTrueRad (0 = north). Tests and tools; the renderer copies the camera.
void overlayEyeAt(double latDeg, double lonDeg, double altMslM, float headingTrueRad, float vFovDeg,
                  float aspect, OverlayEye& out);

// One floating label. (x, y) is the center of the text in HUD pixels (y down).
struct OverlayLabel {
    float x = 0, y = 0;
    char text[40] = {};
    float r = 1, g = 1, b = 1;
};

struct OverlayAirportRef {
    const Airport* airport = nullptr;
    double distNm = 0;
};

// Appends triangles into the four batches (caller keeps the capacity). Writes labels[0, count).
// N10: turnRadiusNm > 0 draws the fly-by turns as arcs (NavState::turnRadiusNm).
// The route follows overlayRouteProfileFt, shows the missed approach only once a missed approach
// leg is active, and clears near the field on the approach (kOverlayNearFieldHideFt).
// Returns how many of the four batches received triangles (0..4).
int buildWorldOverlay(const OverlayToggles& toggles, const FlightPlan* plan,
                      const OverlayAirportRef* airports, int airportCount,
                      const OverlayAirportRef* runwayAirports, int runwayCount,
                      const TerrainQuery* terrain, const OverlayEye& eye, std::vector<float>& route,
                      std::vector<float>& vpath, std::vector<float>& airportVerts,
                      std::vector<float>& runwayVerts, OverlayLabel* labels, int labelCap,
                      int& labelCount, float turnRadiusNm = 0.0f);

}  // namespace fsim::world
