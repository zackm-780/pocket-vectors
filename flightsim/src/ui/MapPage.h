#pragma once
// Moving map page (WP H4, HUD.md section 7). Self-contained: the app shows it when MapToggle is
// pressed, calls update() with the frame's actions while it is shown, and draw() into the open
// Renderer2D batch instead of the flight HUD. Draws an opaque full-screen map on the virtual
// 640x480 canvas; the simulation keeps running underneath.
//
// Controls while shown:  D-pad up / down = range in / out (2 5 10 20 40 80 160 NM),
//   D-pad left / right = select previous / next route waypoint (info panel, highlighted),
//   A = direct-to the selected waypoint, Y (HudDeclutter) = north-up / track-up,
//   B or Select (MapToggle) = close.
//
// draw() records geometry only and never allocates in steady state (the nav database is queried,
// and the lists rebuilt, only when the map center moves more than 30% of the range or the range
// changes).

#include <vector>

#include "core/Actions.h"
#include "core/AircraftState.h"
#include "core/NavDatabase.h"
#include "core/NavState.h"
#include "core/NavTypes.h"
#include "hud/Renderer2D.h"
#include "ui/MapPageLogic.h"

namespace fsim {

struct MapPageResult {
    bool close = false;
    int directToLeg = -1;  // >= 1: the app should call Navigator::directToLeg(directToLeg)
};

class MapPage {
public:
    // Applies this frame's actions. `plan` may be null. Call only while the page is shown.
    MapPageResult update(const ActionState& actions, const FlightPlan* plan);

    void draw(Renderer2D& r, const AircraftState& ac, const NavState& nav, const FlightPlan* plan,
              const NavDatabase& db, float magVarDeg);

    // State (also used by --map / --map-track and tests).
    void setZoomNm(float nm) { zoomIndex_ = mapZoomIndexForNm(nm); }
    float zoomNm() const { return kMapZoomStepsNm[zoomIndex_]; }
    void setOrientation(MapOrientation o) { orientation_ = o; }
    MapOrientation orientation() const { return orientation_; }
    // Selected plan leg (-1 = follow the active leg).
    int selectedLeg() const { return selectedLeg_; }
    void resetSelection() { selectedLeg_ = -1; }

private:
    static constexpr int kMaxRows = 24;
    static constexpr int kMaxLabels = 96;
    struct LabelRect {
        float x0, y0, x1, y1;
    };

    void refreshCaches(const NavDatabase& db, double latDeg, double lonDeg);
    bool placeLabel(float x, float y, float w, float h);
    void drawRings(Renderer2D& r, const MapProjection& p, const AircraftState& ac);
    void drawRoute(Renderer2D& r, const MapProjection& p, const FlightPlan& plan);
    void drawAirports(Renderer2D& r, const MapProjection& p);
    void drawNavaids(Renderer2D& r, const MapProjection& p);
    void drawAircraft(Renderer2D& r, const MapProjection& p, const AircraftState& ac);
    void drawWind(Renderer2D& r, const MapProjection& p, const AircraftState& ac, float magVar);
    void drawPanel(Renderer2D& r, const FlightPlan* plan, const AircraftState& ac, float magVar);

    int zoomIndex_ = kMapDefaultZoomIndex;
    MapOrientation orientation_ = MapOrientation::NorthUp;
    int selectedLeg_ = -1;
    int rowCount_ = 0;
    MapWptInfo rows_[kMaxRows];
    int effectiveSel_ = -1;  // selected leg actually highlighted this frame

    // Query cache.
    bool cacheValid_ = false;
    double cacheLat_ = 0, cacheLon_ = 0;
    float cacheZoomNm_ = 0;
    std::vector<const Airport*> airports_;
    std::vector<const Navaid*> navaids_;

    LabelRect labels_[kMaxLabels];
    int labelCount_ = 0;
    std::vector<Pt2> pts_;
    float vw_ = 640;
};

}  // namespace fsim
