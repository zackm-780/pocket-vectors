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
