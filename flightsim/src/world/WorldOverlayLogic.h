#pragma once
// W6: world-overlay decisions with no GL. The renderer (WorldOverlay) draws the batches this
// builds: route markers, the vertical path, nearby airports and runway outlines. Each batch is
// eye-relative ECEF triangles (x y z r g b), so one draw call covers a layer.

#include <vector>

#include "core/Geo.h"
#include "core/NavTypes.h"
#include "core/TerrainQuery.h"

namespace fsim::world {

struct OverlayToggles {
    bool route = false;
    bool vpath = false;
    bool airports = false;
    bool runways = false;
};

// 0 off, 1 route, 2 route + vertical path, 3 airports + runways, 4 all. Anything else is custom.
constexpr int kOverlayPresetCount = 5;

int overlayMatchingPreset(const OverlayToggles& t);  // -1 when the four flags match no preset
const char* overlayPresetName(int preset);  // OFF / ROUTE / ROUTE+PATH / AIRPORTS / ALL / CUSTOM
void overlayApplyPreset(OverlayToggles& t, int preset);
// Steps to the next preset. A custom mix goes to off, then the presets wrap.
int overlayCycle(OverlayToggles& t);

// Constraint altitude when one is set, else cruise when it is clearly above the ground, else a
// short pole (ground + 400 ft).
float overlayMarkerAltFt(const AltConstraint& alt, float cruiseFt, float groundFt);

// World length, in meters, of `px` vertical pixels at `distM` for this lens.
float metersForPixels(float distM, float px, float viewH, float vFovRad);
// Half-width of a runway highlight: the true half-width, or wider so it stays `minPx` pixels.
float overlayRunwayHalfWidthM(float trueHalfWidthM, float distM, float minPx, float viewH,
                              float vFovRad);

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
// Returns how many of the four batches received triangles (0..4).
int buildWorldOverlay(const OverlayToggles& toggles, const FlightPlan* plan,
                      const OverlayAirportRef* airports, int airportCount,
                      const OverlayAirportRef* runwayAirports, int runwayCount,
                      const TerrainQuery* terrain, const OverlayEye& eye, std::vector<float>& route,
                      std::vector<float>& vpath, std::vector<float>& airportVerts,
                      std::vector<float>& runwayVerts, OverlayLabel* labels, int labelCap,
                      int& labelCount);

}  // namespace fsim::world
