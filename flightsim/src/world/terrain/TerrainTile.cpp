#include "world/terrain/TerrainTile.h"

#include <algorithm>
#include <cmath>

namespace fsim::terrain {
namespace {

// Longitude offset east of the tile's west edge, correct across the antimeridian.
inline double lonOffsetDeg(double lonDeg, double westDeg) {
    double d = lonDeg - westDeg;
    if (d < -180.0)
        d += 360.0;
    else if (d > 180.0)
        d -= 360.0;
    return d;
}

}  // namespace

size_t TerrainTile::memoryBytes() const {
    return sizeof(TerrainTile) + heightsM.capacity() * sizeof(int16_t) + landClass.capacity();
}

bool TerrainTile::sampleHeightM(double latDeg, double lonDeg, float& outM) const {
    const double inv = 1.0 / stepDeg;
    double fc = lonOffsetDeg(lonDeg, westDeg) * inv;
    double fr = (northDeg - latDeg) * inv;
    fc = std::min<double>(kTileCells, std::max(0.0, fc));
    fr = std::min<double>(kTileCells, std::max(0.0, fr));
    const int c0 = std::min(kTileCells - 1, static_cast<int>(fc));
    const int r0 = std::min(kTileCells - 1, static_cast<int>(fr));
    const float tx = static_cast<float>(fc - c0), ty = static_cast<float>(fr - r0);
    const int16_t* p = heightsM.data() + r0 * kTileSamples + c0;
    const int16_t h00 = p[0], h01 = p[1], h10 = p[kTileSamples], h11 = p[kTileSamples + 1];
    if (h00 == kNoDataM || h01 == kNoDataM || h10 == kNoDataM || h11 == kNoDataM) return false;
    const float top = h00 + (h01 - h00) * tx;
    const float bottom = h10 + (h11 - h10) * tx;
    outM = top + (bottom - top) * ty;
    return true;
}

LandClass TerrainTile::landClassAt(double latDeg, double lonDeg) const {
    if (landClass.empty()) return LandClass::Unknown;
    const double inv = 1.0 / stepDeg;
    int col = static_cast<int>(std::floor(lonOffsetDeg(lonDeg, westDeg) * inv));
    int row = static_cast<int>(std::floor((northDeg - latDeg) * inv));
    col = std::min(kTileCells - 1, std::max(0, col));
    row = std::min(kTileCells - 1, std::max(0, row));
    const uint8_t v = landClass[row * kTileCells + col];
    return v <= static_cast<uint8_t>(LandClass::Moss) ? static_cast<LandClass>(v)
                                                      : LandClass::Unknown;
}

std::shared_ptr<const TerrainTile> TerrainTile::make(const TileId& id, std::vector<int16_t> heights,
                                                     std::vector<uint8_t> landClass) {
    if (heights.size() != static_cast<size_t>(kTileSamples) * kTileSamples) return nullptr;
    if (!landClass.empty() && landClass.size() != static_cast<size_t>(kTileCells) * kTileCells)
        return nullptr;
    auto t = std::make_shared<TerrainTile>();
    t->id = id;
    t->heightsM = std::move(heights);
    t->landClass = std::move(landClass);
    bool any = false;
    int16_t lo = 0, hi = 0;
    for (const int16_t h : t->heightsM) {
        if (h == kNoDataM) {
            t->hasVoids = true;
            continue;
        }
        if (!any) {
            lo = hi = h;
            any = true;
        } else {
            lo = std::min(lo, h);
            hi = std::max(hi, h);
        }
    }
    t->minHeightM = lo;
    t->maxHeightM = hi;
    const TileBounds b = bounds(id);
    t->westDeg = b.westDeg;
    t->northDeg = b.northDeg;
    t->stepDeg = terrain::stepDeg(id.k);
    return t;
}

}  // namespace fsim::terrain
