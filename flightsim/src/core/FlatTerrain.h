#pragma once
// Constant-elevation terrain that honours FlattenArea. Used for tests, the synthetic world and as
// the fallback when no terrain pack is installed (ARCHITECTURE section 8). It follows a graded
// area's grade (W1b) but keeps its simple rules otherwise: the first area containing the point
// wins and `priority` is ignored (world/terrain/FlattenSet composes overlapping areas smoothly).

#include <vector>

#include "core/TerrainQuery.h"

namespace fsim {

class FlatTerrain : public TerrainQuery {
public:
    explicit FlatTerrain(float elevM = 0) : elevM_(elevM) {}

    void setElevationM(float elevM) { elevM_ = elevM; }

    float elevationM(double latDeg, double lonDeg) const override;
    LandClass landClass(double, double) const override { return LandClass::Grass; }
    void addFlattenArea(const FlattenArea& area) override;
    void clearFlattenAreas() override { areas_.clear(); }
    void prefetch(double, double, float) override {}
    bool isDetailReady(double, double) const override { return true; }

private:
    float elevM_;
    std::vector<FlattenArea> areas_;
};

}  // namespace fsim
