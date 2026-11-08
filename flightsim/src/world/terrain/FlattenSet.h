#pragma once
// Flatten areas (core/TerrainQuery.h FlattenArea): inside the convex quad the surface is forced to
// the area's elevation (elevM, or its straight grade when `graded`); outside it blends back to the
// natural terrain with a smoothstep over blendM. Unlike core/FlatTerrain the blend distance is the
// true distance to the quad (point to nearest edge or corner), so the blend ring has rounded
// corners instead of FlatTerrain's farthest-edge-line distance.
//
// W1b (1.2.3): overlapping areas compose into one continuous surface. The areas apply in order of
// priority (then grade length, then quad size), lowest first, each one blending the surface built
// so far toward its own: inside an area its surface wins, so the highest-priority area wins where
// cores overlap, and around it the lower ones meet it through its blend ring instead of a step.
// Where that ring lies over pavement (other areas) rather than natural terrain, the blend grows
// with the height difference (kPaveBlendPerM, up to kMaxPaveBlendM), which keeps a crossing
// runway's transition flatter than about 1:130.
//
// Main thread only. Used by TerrainService and exposed for the terrain renderer (W2), which needs
// the same surface when it bakes runway pads into meshes, and for the airport renderer, whose
// pavement follows it.

#include <algorithm>
#include <cmath>
#include <vector>

#include "core/TerrainQuery.h"

namespace fsim::terrain {

class FlattenSet {
public:
    void add(const FlattenArea& area);
    void clear() {
        areas_.clear();
        order_.clear();
        ++revision_;
    }
    bool empty() const { return areas_.empty(); }
    size_t size() const { return areas_.size(); }
    // Increments on every add/clear so caches (renderer meshes) know when to rebuild.
    unsigned revision() const { return revision_; }
    // The areas as supplied by callers, in insertion order.
    std::vector<FlattenArea> areas() const;

    // Distance in meters from the point to area i: 0 inside or on the edge, otherwise the
    // distance to the nearest edge/corner. Returns +infinity well beyond the area's influence
    // (cheap bounding-box rejection, so the exact value is only meaningful within
    // max(blendM, kMaxPaveBlendM)).
    float distanceM(size_t i, double latDeg, double lonDeg) const;
    // The area's own surface at the point: elevM, or the grade at the point's projection.
    float areaSurfaceM(size_t i, double latDeg, double lonDeg) const;

    // Pavement-to-pavement blend: metres of blend per metre of height difference (smoothstep
    // slope 1.5 / 200 = 1:133), and its cap.
    static constexpr float kPaveBlendPerM = 200.0f;
    static constexpr float kMaxPaveBlendM = 600.0f;

    // Surface elevation at the point. `natural` is a callable returning the unflattened elevation
    // in meters; it is invoked at most once, and not at all when the point is inside an area.
    template <class NaturalFn>
    float elevationM(double latDeg, double lonDeg, NaturalFn&& natural) const {
        if (areas_.empty()) return natural();
        bool have = false;  // `elev` holds the surface so far
        float elev = 0;
        float pave = 0;  // how much of `elev` comes from areas (1) rather than natural terrain (0)
        for (const size_t i : order_) {
            const Area& a = areas_[i];
            const float d = distanceM(i, latDeg, lonDeg);
            if (!(d < kFarM)) continue;
            if (d <= 0) {
                elev = areaSurfaceM(i, latDeg, lonDeg);
                have = true;
                pave = 1;
                continue;
            }
            if (!have) {
                elev = natural();
                have = true;
            }
            const float z = areaSurfaceM(i, latDeg, lonDeg);
            float blend = a.src.blendM;
            if (pave > 0) {
                const float wantM = std::min(kMaxPaveBlendM, kPaveBlendPerM * std::fabs(elev - z));
                blend += pave * std::max(0.0f, wantM - blend);
            }
            if (!(blend > 0) || d >= blend) continue;
            const float t = d / blend;                  // 0 at the edge, 1 at the blend end
            const float s = t * t * (3.0f - 2.0f * t);  // smoothstep
            elev = z + (elev - z) * s;
            pave = pave * s + (1.0f - s);
        }
        return have ? elev : natural();
    }

private:
    static constexpr float kFarM = 1e30f;
    struct Area {
        FlattenArea src;
        // Local east/north frame in meters around the centroid, vertices in CCW order.
        double lat0Deg = 0, lon0Deg = 0, mPerDegLon = 0;
        double x[4] = {}, y[4] = {};
        double minX = 0, maxX = 0, minY = 0, maxY = 0;  // bounding box grown by the reach
        // Grade (when src.graded): start, unit direction and length in the local frame.
        double gx = 0, gy = 0, ux = 0, uy = 0, gLenM = 0;
        double sizeM2 = 0;  // quad area, for the order
    };
    std::vector<Area> areas_;    // insertion order
    std::vector<size_t> order_;  // composition order: priority, grade length, size (ascending)
    unsigned revision_ = 0;
};

}  // namespace fsim::terrain
