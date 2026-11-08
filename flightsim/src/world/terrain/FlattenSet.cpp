#include "world/terrain/FlattenSet.h"

#include <algorithm>
#include <cmath>
#include <limits>

#include "core/Units.h"
#include "world/terrain/TileScheme.h"

namespace fsim::terrain {
namespace {

constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

// Squared distance from the point to the segment (W8b: one square root per area, not four).
double dist2ToSegment(double px, double py, double ax, double ay, double bx, double by) {
    const double ex = bx - ax, ey = by - ay;
    const double len2 = ex * ex + ey * ey;
    double t = len2 > 1e-12 ? ((px - ax) * ex + (py - ay) * ey) / len2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    const double dx = px - (ax + t * ex), dy = py - (ay + t * ey);
    return dx * dx + dy * dy;
}

int64_t cellOf(double deg, double cellDeg) {
    return static_cast<int64_t>(std::floor(deg / cellDeg));
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
    // Reach: the natural blend, or the longest pavement blend (FlattenSet.h); areas below the
    // runways cap their pavement blend lower (W8b).
    a.paveCapM = area.priority < 0 ? kMaxLowPaveBlendM : kMaxPaveBlendM;
    a.pavePerM = area.priority < 0 ? kLowPaveBlendPerM : kPaveBlendPerM;
    const double reachM = std::max(static_cast<double>(area.blendM), double{a.paveCapM});
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
    // W8b: the reach box in degrees for the grid (longitudes unwrapped around lon0Deg).
    a.minLatDeg = a.lat0Deg + a.minY / kMPerDegLat;
    a.maxLatDeg = a.lat0Deg + a.maxY / kMPerDegLat;
    const double mPerDegLon = std::max(a.mPerDegLon, 1.0);
    a.minLonDeg = a.lon0Deg + a.minX / mPerDegLon;
    a.maxLonDeg = a.lon0Deg + a.maxX / mPerDegLon;
    a.wrapsLon = a.minLonDeg < -180.0 || a.maxLonDeg >= 180.0;
    areas_.push_back(a);
    dirty_ = true;  // the order and the grid are rebuilt on the next query
    ++revision_;
}

double FlattenSet::normalizeLon(double lonDeg) {
    return normalizeLonDeg(lonDeg);
}

void FlattenSet::rebuildIndex() const {
    dirty_ = false;
    // Composition order: lowest first, so the highest priority (then the longest grade, then the
    // largest quad) is applied last and wins where cores overlap. Stable for exact ties.
    order_.resize(areas_.size());
    for (size_t i = 0; i < order_.size(); ++i) order_[i] = static_cast<uint32_t>(i);
    std::stable_sort(order_.begin(), order_.end(), [this](uint32_t l, uint32_t r) {
        const Area& p = areas_[l];
        const Area& q = areas_[r];
        if (p.src.priority != q.src.priority) return p.src.priority < q.src.priority;
        if (std::fabs(p.gLenM - q.gLenM) > 1.0) return p.gLenM < q.gLenM;
        return p.sizeM2 < q.sizeM2 - 1.0;
    });
    // Grid: every cell the reach box overlaps lists the area. Walking the areas in composition
    // order and sorting stably by cell keeps each list in that order.
    std::vector<std::pair<uint64_t, uint32_t>> entries;
    entries.reserve(areas_.size() * 8);
    auto addRange = [&](uint32_t i, int64_t lat0, int64_t lat1, double lonLo, double lonHi) {
        const int64_t lon0 = cellOf(lonLo, kCellLonDeg), lon1 = cellOf(lonHi, kCellLonDeg);
        for (int64_t y = lat0; y <= lat1; ++y)
            for (int64_t x = lon0; x <= lon1; ++x) entries.emplace_back(cellKey(y, x), i);
    };
    for (const uint32_t i : order_) {
        const Area& a = areas_[i];
        const int64_t lat0 = cellOf(std::max(-90.0, a.minLatDeg), kCellLatDeg);
        const int64_t lat1 = cellOf(std::min(90.0, a.maxLatDeg), kCellLatDeg);
        // Split a box that crosses +-180 deg into its two sides.
        double lo = a.minLonDeg, hi = a.maxLonDeg;
        if (hi - lo >= 360.0) {
            lo = -180.0;
            hi = 180.0;
        }
        if (lo < -180.0) {
            addRange(i, lat0, lat1, lo + 360.0, 180.0);
            lo = -180.0;
        }
        if (hi >= 180.0) {
            addRange(i, lat0, lat1, -180.0, hi - 360.0);
            hi = std::nextafter(180.0, 0.0);
        }
        addRange(i, lat0, lat1, lo, hi);
    }
    std::stable_sort(entries.begin(), entries.end(),
                     [](const auto& l, const auto& r) { return l.first < r.first; });
    cells_.clear();
    cells_.reserve(entries.size() / 4 + 1);
    cellAreas_.resize(entries.size());
    for (size_t k = 0; k < entries.size(); ++k) {
        cellAreas_[k] = entries[k].second;
        auto [it, inserted] =
            cells_.try_emplace(entries[k].first, static_cast<uint32_t>(k), uint32_t{0});
        ++it->second.second;
        (void)inserted;
    }
}

void FlattenSet::candidates(double latDeg, double lonDeg, const uint32_t*& list,
                            size_t& count) const {
    if (dirty_) rebuildIndex();
    if (!useGrid_) {
        list = order_.data();
        count = order_.size();
        return;
    }
    const auto it = cells_.find(
        cellKey(cellOf(latDeg, kCellLatDeg), cellOf(normalizeLonDeg(lonDeg), kCellLonDeg)));
    if (it == cells_.end()) {
        list = nullptr;
        count = 0;
        return;
    }
    list = cellAreas_.data() + it->second.first;
    count = it->second.second;
}

size_t FlattenSet::candidateCount(double latDeg, double lonDeg) const {
    const uint32_t* list = nullptr;
    size_t count = 0;
    candidates(latDeg, lonDeg, list, count);
    return count;
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
    double nearest2 = std::numeric_limits<double>::infinity();
    for (int e = 0; e < 4; ++e) {
        const int n = (e + 1) % 4;
        const double ex = a.x[n] - a.x[e], ey = a.y[n] - a.y[e];
        if (ex * (py - a.y[e]) - ey * (px - a.x[e]) < 0) inside = false;
        nearest2 = std::min(nearest2, dist2ToSegment(px, py, a.x[e], a.y[e], a.x[n], a.y[n]));
    }
    return inside ? 0.0f : static_cast<float>(std::sqrt(nearest2));
}

}  // namespace fsim::terrain
