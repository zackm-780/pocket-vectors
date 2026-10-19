#include "core/FlatTerrain.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace fsim {
namespace {

// Signed distance (meters, + = inside) from a point to a convex CCW quad, using a local
// equirectangular projection around the quad's first vertex. Accurate to well under a percent at
// airport scale, which is all flatten areas are used for.
float insideDistanceM(const FlattenArea& a, double latDeg, double lonDeg) {
    const double lat0 = a.latDeg[0], lon0 = a.lonDeg[0];
    const double mPerDegLat = kEarthRadiusM * kRadPerDeg;
    const double mPerDegLon = mPerDegLat * std::cos(degToRad(lat0));
    double xs[4], ys[4];
    for (int i = 0; i < 4; ++i) {
        xs[i] = (a.lonDeg[i] - lon0) * mPerDegLon;  // east
        ys[i] = (a.latDeg[i] - lat0) * mPerDegLat;  // north
    }
    const double px = (lonDeg - lon0) * mPerDegLon, py = (latDeg - lat0) * mPerDegLat;
    double minDist = 1e30;
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        const double ex = xs[j] - xs[i], ey = ys[j] - ys[i];
        const double len = std::hypot(ex, ey);
        if (len < 1e-6) continue;
        // For a CCW polygon the interior is to the left of each edge.
        const double cross = ex * (py - ys[i]) - ey * (px - xs[i]);
        minDist = std::min(minDist, cross / len);
    }
    return static_cast<float>(minDist);
}

}  // namespace

void FlatTerrain::addFlattenArea(const FlattenArea& area) {
    areas_.push_back(area);
}

float FlatTerrain::elevationM(double latDeg, double lonDeg) const {
    float elev = elevM_;
    for (const FlattenArea& a : areas_) {
        const float d = insideDistanceM(a, latDeg, lonDeg);
        if (d >= 0) return a.elevM;
        if (a.blendM > 0 && -d < a.blendM) {
            const float t = -d / a.blendM;              // 0 at the edge, 1 at the end of the blend
            const float s = t * t * (3.0f - 2.0f * t);  // smoothstep
            elev = a.elevM + (elev - a.elevM) * s;
        }
    }
    return elev;
}

}  // namespace fsim
