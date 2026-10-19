#pragma once
// W5: airport geometry without any GL. Everything the AirportRenderer draws is derived here from
// the nav database `Runway` rows, so it can be unit tested headless:
//   * RunwayInfo   : the numbers of one landing direction (length, width, displaced threshold...)
//   * RunwayFrame  : runway-local coordinates (along from the threshold, lateral + right, up) to
//                    ECEF, following the curvature of the earth (ARCHITECTURE section 4)
//   * markings     : threshold stripes, designator (stroke font), centerline, touchdown zone,
//                    aiming point, edge lines, displaced-threshold arrows (NAVIGATION 3.6)
//   * lights       : edge / threshold / end lights, simplified MALSR, PAPI
//   * PAPI logic   : unit positions and the white/red decision from the eye elevation angle
//   * flatten area : the quad registered with TerrainQuery::addFlattenArea
//   * mesh builders: triangle meshes (RTC: double ECEF anchor + float offsets) and point lights
//
// Conventions: "along" is metres from the LANDING THRESHOLD of this direction toward the far end
// (negative = before the threshold, i.e. the displaced area and the approach), "lateral" is metres
// to the right of the centerline when looking along the landing direction, "up" is metres above
// the runway's flat elevation. Headings are true.

#include <cstdint>
#include <string>
#include <vector>

#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/NavTypes.h"
#include "core/TerrainQuery.h"

namespace fsim::world {

// ---- constants --------------------------------------------------------------------------------
constexpr double kPapiTchFt = 50.0;            // threshold crossing height the PAPI is set for
constexpr double kPapiGlidepathDeg = 3.0;      // on-path angle
constexpr double kPapiFirstUnitFromEdgeFt = 50.0;
constexpr double kPapiUnitSpacingFt = 30.0;
constexpr double kPapiMinRunwayFt = 3000.0;    // hard runways at least this long get a PAPI
constexpr int kPapiUnitCount = 4;
// Transition angles from the unit nearest the runway outward (NAVIGATION 3.6).
constexpr double kPapiTransitionDeg[kPapiUnitCount] = {3.50, 3.17, 2.83, 2.50};
constexpr double kPapiUnitHeightM = 0.5;

constexpr double kEdgeLightSpacingFt = 200.0;
constexpr double kEdgeCautionZoneFt = 2000.0;  // yellow edge lights on instrument runways
constexpr double kApproachBarSpacingFt = 200.0;
constexpr double kApproachLengthFt = 2400.0;
constexpr int kApproachBarCount = 12;          // 200 ft .. 2400 ft
constexpr int kApproachLightsPerBar = 5;
constexpr double kApproachLightSpacingM = 1.5;
constexpr double kFlasherInnerFt = 1400.0;     // sequenced flashers on the outer 1,000 ft
constexpr int kFlasherCount = 6;               // 1400, 1600, ... 2400 ft
constexpr double kFlasherHz = 2.0;

constexpr double kThresholdStripeStartM = 6.0;
constexpr double kThresholdStripeLengthM = 30.0;
constexpr double kDesignatorGapM = 12.0;

// ---- runway info ------------------------------------------------------------------------------
struct RunwayInfo {
    std::string airportIdent, ident, oppositeIdent;
    LatLon thr;                // landing threshold
    double headingTrueRad = 0; // landing direction (bearing pavement start -> end)
    double lengthM = 0;        // whole pavement, start to end
    double displacedM = 0;     // start -> threshold
    double widthM = 0;
    double elevM = 0;          // flat elevation of the whole pavement (mean of the two ends)
    bool hard = true, lighted = false, approx = false;
    bool instrument = false;          // this direction has an ILS and a trustworthy geometry
    bool oppositeInstrument = false;  // the other direction does
    bool primary = true;              // owns the pavement mesh of the pair
    bool papi = false;                // hard, >= 3,000 ft, not approx
    bool approachLights = false;      // ILS runway (hard, not approx)
    double usableM() const { return lengthM - displacedM; }  // threshold -> far end
};

// Builds the info for one landing direction. The opposite direction is looked up in `airport`.
RunwayInfo makeRunwayInfo(const Airport& airport, const Runway& runway);
// Flat elevation used for the pavement AND the terrain flatten area (mean of the two ends of the
// pair's primary direction).
double runwayFlatElevM(const Airport& airport, const Runway& runway);

// Runway-local frame -> ECEF.
class RunwayFrame {
public:
    RunwayFrame() = default;
    explicit RunwayFrame(const RunwayInfo& info);
    LatLon latLon(double alongM, double lateralM) const;
    Ecef toEcef(double alongM, double lateralM, double upM) const;
    Vec3d upEcef(double alongM, double lateralM) const;  // geodetic normal
    Vec3d alongDirEcef(double alongM) const;             // horizontal unit vector toward +along
    double headingTrueRad() const { return headingTrueRad_; }

private:
    LatLon thr_;
    double headingTrueRad_ = 0;
    double elevM_ = 0;
};

// ---- markings ---------------------------------------------------------------------------------
enum class MarkingKind : uint8_t {
    ThresholdStripe,
    Designator,
    Centerline,
    TouchdownZone,
    AimingPoint,
    EdgeLine,
    DisplacedArrow
};

struct Poly {  // convex quad (n = 4) or triangle (n = 3) in runway-local coordinates
    int n = 4;
    double alongM[4] = {0, 0, 0, 0};
    double lateralM[4] = {0, 0, 0, 0};
    MarkingKind kind = MarkingKind::Centerline;
};

struct RunwayMarkings {
    std::vector<Poly> polys;
    double designatorEndM = 0;  // along-coordinate of the far edge of the designator text
    int count(MarkingKind kind) const;
};

// Threshold stripes in total (both sides of the centerline) for a runway width.
int thresholdStripeCount(double widthFt);
// Hard runways only. Non-instrument runways get designator, centerline and threshold (plus the
// displaced-threshold arrows); instrument runways add touchdown zone, aiming point, edge lines.
RunwayMarkings buildMarkings(const RunwayInfo& runway);
// Splits "28L" into the number "28" and the suffix "L" (possibly empty).
void splitDesignator(const std::string& ident, std::string& number, std::string& suffix);

// Touchdown-zone station table: distance from the threshold (ft) and bars per side.
struct TdzStation {
    double alongFt;
    int barsPerSide;
};
const std::vector<TdzStation>& touchdownZoneStations();
constexpr double kAimingPointFt = 1000.0;

// ---- lights -----------------------------------------------------------------------------------
enum class LightKind : uint8_t {
    Edge,
    EdgeCaution,
    EdgeDisplaced,
    Threshold,
    End,
    Approach,
    Flasher,
    Papi
};

struct LightSpec {
    LightKind kind = LightKind::Edge;
    double alongM = 0, lateralM = 0, heightM = 0;
    float rgb[3] = {1, 1, 1};
    float sizeM = 1;   // apparent diameter used for the point sprite size
    int facing = -1;   // -1: visible from before the threshold (the approach side); 0: all around
    int seq = -1;      // flashers: 0 = outermost (fires first); PAPI: 100 + unit; else -1
};

// Edge/threshold/end lights need `lighted`; approach lights and PAPI follow the runway flags.
std::vector<LightSpec> buildRunwayLights(const RunwayInfo& runway);
// Number of edge-light intervals per side for a pavement length (200 ft nominal spacing).
int edgeLightIntervals(double lengthM);

// ---- PAPI -------------------------------------------------------------------------------------
struct PapiPosition {
    double alongM = 0, lateralM = 0;
};
PapiPosition papiUnitPosition(const RunwayInfo& runway, int unit);
// A unit is white when the eye's elevation angle above it exceeds its transition angle.
bool papiUnitIsWhite(int unit, double elevationDeg);
// Bit i set = unit i white.
unsigned papiWhiteMask(double elevationDeg);
// Elevation angle (deg) of the eye above the PAPI unit row, measured in the vertical plane along
// the runway axis. Negative or very large when the eye is past the units.
double papiEyeElevationDeg(const RunwayFrame& frame, const RunwayInfo& runway, const Ecef& eyeEcef);

// Sequenced flasher: index (0 = outermost) of the flasher that is on at timeS, or -1.
int flasherActiveIndex(double timeS);

// ---- terrain flattening -----------------------------------------------------------------------
FlattenArea runwayFlattenArea(const RunwayInfo& runway, double lateralMarginM = 50.0,
                              double endMarginM = 100.0, float blendM = 60.0f);
// Registers one flatten area per runway pair of the airport (primary directions only).
void registerAirportFlatten(const Airport& airport, TerrainQuery& terrain);
// Lowest flat runway elevation of the airport (m), or airport elevation when it has no runways.
double airportLowestFlatElevM(const Airport& airport);

// ---- airport selection ------------------------------------------------------------------------
// Airports within radiusM of the point (nearest first, at most maxCount), through airportsInBox.
std::vector<const Airport*> selectAirportsNear(const NavDatabase& nav, double latDeg,
                                               double lonDeg, double radiusM, int maxCount);

// ---- meshes (RTC: double anchor + float offsets) ----------------------------------------------
struct SurfaceVertex {
    float pos[3];     // offset from the mesh anchor, ECEF axes, metres
    float normal[3];  // ECEF axes
    float rgb[3];     // albedo
};
struct SurfaceMesh {
    Ecef anchor;
    std::vector<SurfaceVertex> verts;
    std::vector<uint16_t> indices;  // triangles, 16-bit
};

struct LightVertex {
    float pos[3];
    float rgb[3];
    float info[3];  // sizeM, facing sign, seq
};
struct LightMesh {
    Ecef anchor;
    std::vector<LightVertex> verts;
    float axisEcef[3] = {1, 0, 0};  // landing direction at the anchor, for directional lights
    bool hasPapi = false;
};

// Appends triangles to one or more 16-bit-indexed meshes sharing an anchor.
class SurfaceMeshBuilder {
public:
    SurfaceMeshBuilder(const Ecef& anchor, const float rgbFallback[3] = nullptr);
    void addPoly(const RunwayFrame& frame, const Poly& poly, double liftM, const float rgb[3]);
    // Strip [a0,a1] x [l0,l1], split along the runway so it follows the earth's curvature.
    void addStrip(const RunwayFrame& frame, double a0, double a1, double l0, double l1,
                  double liftM, const float rgb[3], double maxSegmentM = 250.0);
    // Axis-aligned (in runway coordinates) box from the ground to heightM, with shaded walls.
    void addBox(const RunwayFrame& frame, double a0, double a1, double l0, double l1,
                double bottomM, double heightM, const float wallRgb[3], const float roofRgb[3]);
    std::vector<SurfaceMesh> finish();

private:
    SurfaceMesh& current(int extraVerts);
    void pushVertex(const Ecef& p, const float normal[3], const float rgb[3]);
    Ecef anchor_;
    float up_[3];
    std::vector<SurfaceMesh> meshes_;
};

struct RunwayMeshes {
    std::vector<SurfaceMesh> pavement;  // drawn with the first polygon offset
    std::vector<SurfaceMesh> markings;  // drawn with the second (stronger) polygon offset
    LightMesh lights;
    RunwayMarkings markingData;
};
RunwayMeshes buildRunwayMeshes(const RunwayInfo& runway);

// Generic apron, taxiway stub and a few box buildings beside the primary runway of an airport
// (placed clear of every runway). Empty for airports without a hard runway.
struct AirportExtras {
    std::vector<SurfaceMesh> flat;   // apron and taxiway stub (polygon-offset layer)
    std::vector<SurfaceMesh> solid;  // building boxes
    bool empty() const { return flat.empty() && solid.empty(); }
};
AirportExtras buildAirportExtras(const Airport& airport);
// Mean point of the airport's pavement in ECEF (for culling).
Ecef runwayCenterEcef(const RunwayInfo& runway);

}  // namespace fsim::world
