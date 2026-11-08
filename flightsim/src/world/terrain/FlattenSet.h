#pragma once
// Flatten areas (core/TerrainQuery.h FlattenArea): inside the convex quad the surface is forced to
// the area's elevation (elevM, or its straight grade when `graded`); outside it blends back to the
// natural terrain with a smoothstep over blendM. Unlike core/FlatTerrain the blend distance is the
// true distance to the quad (point to nearest edge or corner), so the blend ring has rounded
// corners instead of FlatTerrain's farthest-edge-line distance.
//
// W1b (1.3): overlapping areas compose into one continuous surface. The areas apply in order of
// priority (then grade length, then quad size), lowest first, each one blending the surface built
// so far toward its own: inside an area its surface wins, so the highest-priority area wins where
// cores overlap, and around it the lower ones meet it through its blend ring instead of a step.
// Where that ring lies over pavement (other areas) rather than natural terrain, the blend grows
// with the height difference (kPaveBlendPerM, up to kMaxPaveBlendM), which keeps a crossing
// runway's transition flatter than about 1:130.
//
// W8b (1.3): taxiways and aprons add hundreds of areas per airport (priority < 0), so a lookup
// no longer walks every area. A uniform grid (cells of about 140 m) lists, for each cell, the
// areas whose reach box overlaps it, in composition order; a lookup walks only its cell's list,
// and starts at the last area whose core holds the point (it overwrites everything before it), so
// it gets the same result as the full loop. The order and the grid are rebuilt lazily on the first
// query after add()/clear(), so registering an airport costs O(n log n) once. Areas below the
// runways blend over pavement more gently (kLowPaveBlendPerM, 1:200: their rings stack on taxiway
// grades) and stop sooner (kMaxLowPaveBlendM), which keeps their reach and the lists short.
//
// Main thread only (the lazy index is mutable state behind const queries). Used by TerrainService
// and exposed for the terrain renderer (W2), which needs the same surface when it bakes runway pads
// into meshes, and for the airport renderer, whose pavement follows it.

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <unordered_map>
#include <utility>
#include <vector>

#include "core/TerrainQuery.h"

namespace fsim::terrain {

class FlattenSet {
public:
    void add(const FlattenArea& area);
    void clear() {
        areas_.clear();
        order_.clear();
        cells_.clear();
        cellAreas_.clear();
        dirty_ = false;
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
    // slope 1.5 / 200 = 1:133), and its cap: runways (priority >= 0) and the areas below them.
    static constexpr float kPaveBlendPerM = 200.0f;
    static constexpr float kMaxPaveBlendM = 600.0f;
    // W8b: areas below the runways (priority < 0: taxiways, aprons) blend gentler (1:200), because
    // their rings stack on taxiway grades, and stop sooner, which keeps the grid lists short.
    static constexpr float kLowPaveBlendPerM = 300.0f;
    static constexpr float kMaxLowPaveBlendM = 150.0f;

    // W8b grid: cell size in degrees (about 139 m north-south, 137 m east-west at 45 deg N).
    static constexpr double kCellLatDeg = 0.00125;
    static constexpr double kCellLonDeg = 0.00175;
    // Number of areas a lookup at the point examines (its cell's list; all areas without the grid).
    size_t candidateCount(double latDeg, double lonDeg) const;
    // Tests only: false walks every area in composition order (the pre-W8b loop).
    void setUseGridForTest(bool on) { useGrid_ = on; }

    // Surface elevation at the point. `natural` is a callable returning the unflattened elevation
    // in meters; it is invoked at most once, and not at all when the point is inside an area.
    template <class NaturalFn>
    float elevationM(double latDeg, double lonDeg, NaturalFn&& natural) const {
        if (areas_.empty()) return natural();
        const uint32_t* list = nullptr;
        size_t count = 0;
        candidates(latDeg, lonDeg, list, count);
        if (count == 0) return natural();
        const double lonN = lonDeg >= -180.0 && lonDeg < 180.0 ? lonDeg : normalizeLon(lonDeg);
        // Distance to an area, +infinity beyond its reach (cheap degree-box test first: the grid
        // cell only bounds the reach loosely).
        auto dist = [&](size_t i) {
            const Area& a = areas_[i];
            if (latDeg < a.minLatDeg || latDeg > a.maxLatDeg) return kFarM;
            if (!a.wrapsLon && (lonN < a.minLonDeg || lonN > a.maxLonDeg)) return kFarM;
            return distanceM(i, latDeg, lonDeg);
        };
        // W8b: the last area (in composition order) whose core holds the point overwrites
        // everything before it, so the walk starts there. The backward scan keeps its distances.
        constexpr size_t kMaxCached = 512;
        float cached[kMaxCached];
        const bool useCache = count <= kMaxCached;
        size_t start = 0;
        for (size_t k = count; k-- > 0;) {
            const float d = dist(list[k]);
            if (useCache) cached[k] = d;
            if (d <= 0) {
                start = k;
                break;
            }
        }
        bool have = false;  // `elev` holds the surface so far
        float elev = 0;
        float pave = 0;  // how much of `elev` comes from areas (1) rather than natural terrain (0)
        for (size_t k = start; k < count; ++k) {
            const size_t i = list[k];
            const Area& a = areas_[i];
            const float d = useCache ? cached[k] : dist(i);
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
                const float wantM = std::min(a.paveCapM, a.pavePerM * std::fabs(elev - z));
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
    static constexpr float kFarM = std::numeric_limits<float>::infinity();
    struct Area {
        FlattenArea src;
        // Local east/north frame in meters around the centroid, vertices in CCW order.
        double lat0Deg = 0, lon0Deg = 0, mPerDegLon = 0;
        double x[4] = {}, y[4] = {};
        double minX = 0, maxX = 0, minY = 0, maxY = 0;  // bounding box grown by the reach
        // Grade (when src.graded): start, unit direction and length in the local frame.
        double gx = 0, gy = 0, ux = 0, uy = 0, gLenM = 0;
        double sizeM2 = 0;                // quad area, for the order
        float paveCapM = kMaxPaveBlendM;  // W8b: kMaxLowPaveBlendM below the runways
        float pavePerM = kPaveBlendPerM;  // W8b: kLowPaveBlendPerM below the runways
        // W8b: the reach box in degrees (longitudes unwrapped around lon0Deg), for the grid.
        double minLatDeg = 0, maxLatDeg = 0, minLonDeg = 0, maxLonDeg = 0;
        bool wrapsLon = false;  // the box crosses +-180 deg (no cheap longitude test)
    };

    // The areas a lookup at the point walks, in composition order (rebuilds the index if needed).
    void candidates(double latDeg, double lonDeg, const uint32_t*& list, size_t& count) const;
    void rebuildIndex() const;
    static double normalizeLon(double lonDeg);
    static uint64_t cellKey(int64_t latCell, int64_t lonCell) {
        return (static_cast<uint64_t>(latCell + (int64_t{1} << 31)) << 32) |
               static_cast<uint64_t>(static_cast<uint32_t>(lonCell + (int64_t{1} << 31)));
    }

    std::vector<Area> areas_;  // insertion order
    // Lazy index (W8b), rebuilt on the first query after add()/clear().
    mutable std::vector<uint32_t> order_;  // composition order: priority, grade length, size
    mutable std::unordered_map<uint64_t, std::pair<uint32_t, uint32_t>> cells_;  // offset, count
    mutable std::vector<uint32_t> cellAreas_;  // the cells' lists, each in composition order
    mutable bool dirty_ = false;
    bool useGrid_ = true;
    unsigned revision_ = 0;
};

}  // namespace fsim::terrain
