#include "world/terrain/FlattenSet.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/Units.h"
#include "world/terrain/TileScheme.h"

namespace fsim::terrain {
namespace {

constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

double distToSegment(double px, double py, double ax, double ay, double bx, double by) {
    const double ex = bx - ax, ey = by - ay;
    const double len2 = ex * ex + ey * ey;
    double t = len2 > 1e-12 ? ((px - ax) * ex + (py - ay) * ey) / len2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    return std::hypot(px - (ax + t * ex), py - (ay + t * ey));
}

}  // namespace

void FlattenSet::add(const FlattenArea& area) {
    Area a;
    a.src = area;
    double latSum = 0, lonRef = area.lonDeg[0], dLonSum = 0;
    for (int i = 0; i < 4; ++i) {
        latSum += area.latDeg[i];
        dLonSum += normalizeLonDeg(area.lonDeg[i] - lonRef);
    }
    a.lat0Deg = latSum / 4.0;
    a.lon0Deg = normalizeLonDeg(lonRef + dLonSum / 4.0);
    a.mPerDegLon = kMPerDegLat * std::cos(a.lat0Deg * kRadPerDeg);
    for (int i = 0; i < 4; ++i) {
        a.x[i] = normalizeLonDeg(area.lonDeg[i] - a.lon0Deg) * a.mPerDegLon;
        a.y[i] = (area.latDeg[i] - a.lat0Deg) * kMPerDegLat;
    }
    // The interior must be on the left of every edge. Accept clockwise input too.
    double signedArea2 = 0;
    for (int i = 0; i < 4; ++i) {
        const int j = (i + 1) % 4;
        signedArea2 += a.x[i] * a.y[j] - a.x[j] * a.y[i];
    }
    if (signedArea2 < 0) {
        std::swap(a.x[1], a.x[3]);
        std::swap(a.y[1], a.y[3]);
    }
    // Reach: the natural blend, or the longest pavement blend (FlattenSet.h).
    const double reachM = std::max(static_cast<double>(area.blendM), double{kMaxPaveBlendM});
    a.minX = *std::min_element(a.x, a.x + 4) - reachM;
    a.maxX = *std::max_element(a.x, a.x + 4) + reachM;
    a.minY = *std::min_element(a.y, a.y + 4) - reachM;
    a.maxY = *std::max_element(a.y, a.y + 4) + reachM;
    a.sizeM2 = 0.5 * std::fabs(signedArea2);
    if (area.graded) {
        a.gx = normalizeLonDeg(area.gradeLonDeg[0] - a.lon0Deg) * a.mPerDegLon;
        a.gy = (area.gradeLatDeg[0] - a.lat0Deg) * kMPerDegLat;
        const double ex = normalizeLonDeg(area.gradeLonDeg[1] - a.lon0Deg) * a.mPerDegLon - a.gx;
        const double ey = (area.gradeLatDeg[1] - a.lat0Deg) * kMPerDegLat - a.gy;
        a.gLenM = std::hypot(ex, ey);
        if (a.gLenM > 1e-3) {
            a.ux = ex / a.gLenM;
            a.uy = ey / a.gLenM;
        }
    }
    areas_.push_back(a);
    // Composition order: lowest first, so the highest priority (then the longest grade, then the
    // largest quad) is applied last and wins where cores overlap. Stable for exact ties.
    order_.resize(areas_.size());
    for (size_t i = 0; i < order_.size(); ++i) order_[i] = i;
    std::stable_sort(order_.begin(), order_.end(), [this](size_t l, size_t r) {
        const Area& p = areas_[l];
        const Area& q = areas_[r];
        if (p.src.priority != q.src.priority) return p.src.priority < q.src.priority;
        if (std::fabs(p.gLenM - q.gLenM) > 1.0) return p.gLenM < q.gLenM;
        return p.sizeM2 < q.sizeM2 - 1.0;
    });
    ++revision_;
}

float FlattenSet::areaSurfaceM(size_t i, double latDeg, double lonDeg) const {
    const Area& a = areas_[i];
    if (!a.src.graded || a.gLenM <= 1e-3) return a.src.elevM;
    const double px = normalizeLonDeg(lonDeg - a.lon0Deg) * a.mPerDegLon;
    const double py = (latDeg - a.lat0Deg) * kMPerDegLat;
    const double t =
        std::min(1.0, std::max(0.0, ((px - a.gx) * a.ux + (py - a.gy) * a.uy) / a.gLenM));
    return static_cast<float>(a.src.gradeElevM[0] +
                              (a.src.gradeElevM[1] - a.src.gradeElevM[0]) * t);
}

std::vector<FlattenArea> FlattenSet::areas() const {
    std::vector<FlattenArea> out;
    out.reserve(areas_.size());
    for (const Area& a : areas_) out.push_back(a.src);
    return out;
}

float FlattenSet::distanceM(size_t i, double latDeg, double lonDeg) const {
    const Area& a = areas_[i];
    const double px = normalizeLonDeg(lonDeg - a.lon0Deg) * a.mPerDegLon;
    const double py = (latDeg - a.lat0Deg) * kMPerDegLat;
    if (px < a.minX || px > a.maxX || py < a.minY || py > a.maxY)
        return std::numeric_limits<float>::infinity();
    bool inside = true;
    double nearest = std::numeric_limits<double>::infinity();
    for (int e = 0; e < 4; ++e) {
        const int n = (e + 1) % 4;
        const double ex = a.x[n] - a.x[e], ey = a.y[n] - a.y[e];
        if (ex * (py - a.y[e]) - ey * (px - a.x[e]) < 0) inside = false;
        nearest = std::min(nearest, distToSegment(px, py, a.x[e], a.y[e], a.x[n], a.y[n]));
    }
    return inside ? 0.0f : static_cast<float>(nearest);
}

}  // namespace fsim::terrain
