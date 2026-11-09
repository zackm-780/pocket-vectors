#pragma once
// W8b / W8c (1.3): taxiway and apron geometry from OpenStreetMap (core/NavTypes.h Taxiway, Apron;
// DATA.md 5.8-5.9), without any GL, so it can be unit tested headless.
//
//   * LocalFrame       : east/north metres around the airport reference point (flat earth, fine
//                        within the few kilometres of an airport)
//   * polylines        : Douglas-Peucker simplification of lines and rings, ear clipping
//   * TaxiwayField     : the height of taxiways and aprons. It comes from the runways only, never
//                        from the DEM: flatten areas are built from the Airport alone (terrain
//                        service, airport renderer), and the DEM at that moment depends on which
//                        tiles are loaded (W1b: loading tiles never moves the pavement).
//   * flatten areas    : every straight taxiway piece is a graded area below the runways
//                        (priority -1), every apron flat (priority -2), so the W1b composition
//                        (world/terrain/FlattenSet) blends them into the runways without a step.
//
// TAXIWAYS.md section 3 has the plan; W8b the numbers.

#include <array>
#include <vector>

#include "core/Geo.h"
#include "core/NavTypes.h"
#include "core/TerrainQuery.h"
#include "world/AirportGeometry.h"
#include "world/terrain/FlattenSet.h"

namespace fsim::world {

// ---- constants (W8b) ---------------------------------------------------------------------------
constexpr double kTaxiwaySimplifyM = 5.0;  // Douglas-Peucker tolerance for the flatten pieces
constexpr double kTaxiwayMarginM = 40.0;   // flat core beyond the pavement edge and past the ends
constexpr float kTaxiwayBlendM = 60.0f;    // blend back to the natural terrain
constexpr int kTaxiwayPriority = -1;       // below the runways (0)
constexpr double kMinTaxiwayPieceM = 1.0;  // shorter pieces add nothing (neighbours cover them)
constexpr double kApronSimplifyM = 3.0;
constexpr double kApronMarginM = 30.0;  // flat strip around every apron edge
constexpr int kApronPriority = -2;  // below the taxiways: a taxilane across a ramp keeps its grade
constexpr double kFieldWeightM = 200.0;  // TaxiwayField: weight 1 / (d^2 + 200^2) per runway
constexpr float kFieldBlendM = 400.0f;   // TaxiwayField: runway surface to the weighted field

// ---- local frame -------------------------------------------------------------------------------
struct Pt2 {
    double eM = 0, nM = 0;  // east, north (metres)
};

class LocalFrame {
public:
    LocalFrame() = default;
    LocalFrame(double lat0Deg, double lon0Deg);
    Pt2 toLocal(const LatLon& p) const;
    LatLon toLatLon(const Pt2& p) const;

private:
    double lat0Deg_ = 0, lon0Deg_ = 0, mPerDegLon_ = 1;
};

// ---- polylines ---------------------------------------------------------------------------------
// Douglas-Peucker on an open polyline: keeps the first and last point and returns the indexes of
// the points kept, in order.
std::vector<int> simplifyPolyline(const std::vector<Pt2>& pts, double toleranceM);
// The same on a closed ring (open: the first point is not repeated); at least 3 points are kept
// when the ring has 3.
std::vector<int> simplifyRing(const std::vector<Pt2>& ring, double toleranceM);
// Ear clipping of a simple polygon given as an open ring, either winding. Triangles are index
// triples into `ring`, counter-clockwise. Collinear and repeated points are dropped. False when no
// ear can be found (a self-intersecting ring); `tris` is then empty.
bool triangulatePolygon(const std::vector<Pt2>& ring, std::vector<std::array<int, 3>>& tris);
// Signed area (m^2), positive when counter-clockwise.
double ringArea(const std::vector<Pt2>& ring);

// ---- heights -----------------------------------------------------------------------------------
// The height of taxiways and aprons (metres MSL). Inside a runway's flatten area (the runway plus
// its 50 m / 100 m margins) it is exactly the composed runway surface the wheels feel, so a
// taxiway end on a runway meets it without a step. Beyond, the runways blend over 400 m to the
// mean of the runway grades weighted by 1 / (d^2 + 200^2), d being the distance to each runway's
// grade segment; the wide blend keeps the field gentle between runways at different heights. It
// always stays within the airport's runway elevations. Airports without runways use elevFt.
class TaxiwayField {
public:
    explicit TaxiwayField(const Airport& airport);
    double elevM(double latDeg, double lonDeg) const;

private:
    double naturalM(double latDeg, double lonDeg) const;
    std::vector<RunwayInfo> runways_;  // primary directions
    terrain::FlattenSet runwaySet_;    // the runway areas as registered (150 m blend)
    terrain::FlattenSet wideSet_;      // the same with a 400 m blend, outside the runway areas
    double airportElevM_ = 0;
};

// ---- flatten areas -----------------------------------------------------------------------------
// Graded areas for the taxiways (one per straight piece after a 5 m simplification: the
// half-width plus 40 m to each side and past both ends, priority -1, 60 m blend) and flat areas
// for the aprons (ear-clipped triangles plus a 30 m strip along every edge, at the mean field
// height of the ring, priority -2). Empty without taxiways and aprons.
std::vector<FlattenArea> taxiwayFlattenAreas(const Airport& airport);

}  // namespace fsim::world
