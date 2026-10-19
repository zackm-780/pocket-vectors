#pragma once
// Flatten areas (core/TerrainQuery.h FlattenArea) with the same semantics as core/FlatTerrain:
// inside the convex quad the surface is forced to elevM; outside it blends back to the natural
// terrain with a smoothstep over blendM. The one difference is that the blend distance is the true
// distance to the quad (point to nearest edge or corner), so the blend ring has rounded corners
// instead of FlatTerrain's farthest-edge-line distance.
//
// Main thread only. Used by TerrainService and exposed for the terrain renderer (W2), which needs
// the same surface when it bakes runway pads into meshes.

#include <cmath>
#include <vector>

#include "core/TerrainQuery.h"

namespace fsim::terrain {

class FlattenSet {
public:
    void add(const FlattenArea& area);
    void clear() {
        areas_.clear();
        ++revision_;
    }
    bool empty() const { return areas_.empty(); }
    size_t size() const { return areas_.size(); }
    // Increments on every add/clear so caches (renderer meshes) know when to rebuild.
    unsigned revision() const { return revision_; }
    // The areas as supplied by callers, in insertion order.
    std::vector<FlattenArea> areas() const;

    // Distance in meters from the point to area i: 0 inside or on the edge, otherwise the
    // distance to the nearest edge/corner. Returns +infinity when beyond area.blendM + margin
    // (cheap bounding-box rejection, so the exact value is only meaningful when < blendM).
    float distanceM(size_t i, double latDeg, double lonDeg) const;

    // Surface elevation at the point. `natural` is a callable returning the unflattened elevation
    // in meters; it is invoked at most once, and not at all when the point is inside an area.
    template <class NaturalFn>
    float elevationM(double latDeg, double lonDeg, NaturalFn&& natural) const {
        if (areas_.empty()) return natural();
        bool haveNatural = false;
        float elev = 0;
        for (size_t i = 0; i < areas_.size(); ++i) {
            const Area& a = areas_[i];
            const float d = distanceM(i, latDeg, lonDeg);
            if (d <= 0) return a.src.elevM;
            if (a.src.blendM > 0 && d < a.src.blendM) {
                if (!haveNatural) {
                    elev = natural();
                    haveNatural = true;
                }
                const float t = d / a.src.blendM;           // 0 at the edge, 1 at the blend end
                const float s = t * t * (3.0f - 2.0f * t);  // smoothstep
                elev = a.src.elevM + (elev - a.src.elevM) * s;
            }
        }
        return haveNatural ? elev : natural();
    }

private:
    struct Area {
        FlattenArea src;
        // Local east/north frame in meters around the centroid, vertices in CCW order.
        double lat0Deg = 0, lon0Deg = 0, mPerDegLon = 0;
        double x[4] = {}, y[4] = {};
        double minX = 0, maxX = 0, minY = 0, maxY = 0;  // bounding box grown by blendM
    };
    std::vector<Area> areas_;
    unsigned revision_ = 0;
};

}  // namespace fsim::terrain
