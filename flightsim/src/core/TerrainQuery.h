#pragma once
// Terrain access contract. Implemented by world/terrain (W1); used by sim, renderers, HUD, nav.
// core/FlatTerrain.h is the always-available fallback.

#include <cstdint>

namespace fsim {

enum class LandClass : uint8_t {
    Unknown = 0,
    Water,
    Tree,
    Shrub,
    Grass,
    Crop,
    Urban,
    Bare,
    SnowIce,
    Wetland,
    Mangrove,
    Moss
};

// E.g. a runway rectangle plus margin, forced to one elevation.
struct FlattenArea {
    double latDeg[4] = {0, 0, 0, 0}, lonDeg[4] = {0, 0, 0, 0};  // convex quad, CCW
    float elevM = 0;
    float blendM = 60;  // blend distance back to natural terrain
    // Additive (W1b, 1.3): a graded area follows a straight grade along a centerline instead
    // of the flat elevM. The surface at a point is the grade at its projection onto the segment
    // gradeLatDeg/gradeLonDeg[0] -> [1], clamped beyond the ends. elevM stays the mean of the
    // two ends for consumers that ignore grades.
    bool graded = false;
    double gradeLatDeg[2] = {0, 0}, gradeLonDeg[2] = {0, 0};
    float gradeElevM[2] = {0, 0};
    // Additive (W1b): where areas overlap the higher priority wins (ties: the longer grade, then
    // the larger quad), and the lower one blends into it smoothly instead of meeting it in a step
    // (world/terrain/FlattenSet). Runways use 0; taxiways and aprons will sit below.
    int priority = 0;
};

class TerrainQuery {
public:
    virtual ~TerrainQuery() = default;
    // Surface elevation (after flattening), meters MSL. NEVER blocks: uses the best loaded LOD,
    // and returns 0 (sea level) if nothing is loaded.
    virtual float elevationM(double latDeg, double lonDeg) const = 0;
    virtual LandClass landClass(double latDeg, double lonDeg) const = 0;
    virtual void addFlattenArea(const FlattenArea& area) = 0;
    virtual void clearFlattenAreas() = 0;
    // Ask for full detail around a point (ground contact, start positions).
    virtual void prefetch(double latDeg, double lonDeg, float radiusM) = 0;
    // True when the finest available LOD around the point is resident (gate for spawning).
    virtual bool isDetailReady(double latDeg, double lonDeg) const = 0;
};

}  // namespace fsim
