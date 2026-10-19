#pragma once
// One decoded terrain tile: 257 x 257 corner-sampled heights and an optional 256 x 256 land-class
// grid (DATA.md sections 4.2, 4.3). Tiles are immutable once built and shared between the cache
// and the renderer as std::shared_ptr<const TerrainTile>, so a tile evicted from the cache stays
// alive for as long as a renderer mesh still uses it.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include "core/TerrainQuery.h"
#include "world/terrain/TileScheme.h"

namespace fsim::terrain {

constexpr int16_t kNoDataM = -32768;  // height sample without data (DATA 4.2)

struct TerrainTile {
    TileId id;
    // Row-major, north-up: row r is at lat north - r*step, column c at lon west + c*step.
    // 257 * 257 int16 meters MSL; kNoDataM marks voids.
    std::vector<int16_t> heightsM;
    // Row-major, north-up, cell-centred: 256 * 256 LandClass values, or empty when the tile has
    // no land-class file.
    std::vector<uint8_t> landClass;

    // Derived at construction (valid samples only; 0/0 for an all-void tile).
    int16_t minHeightM = 0, maxHeightM = 0;
    bool hasVoids = false;
    double westDeg = 0, northDeg = 0, stepDeg = 0;  // tile origin (north-west corner) and spacing

    bool hasLandClass() const { return !landClass.empty(); }
    int16_t heightAtM(int row, int col) const { return heightsM[row * kTileSamples + col]; }
    uint8_t landClassCell(int row, int col) const { return landClass[row * kTileCells + col]; }
    size_t memoryBytes() const;

    // Bilinear height at a point (meters MSL). Positions outside the tile are clamped to its edge.
    // Returns false if any of the four surrounding samples is a void (callers then fall back to
    // the parent tile).
    bool sampleHeightM(double latDeg, double lonDeg, float& outM) const;
    // Land class of the cell containing the point (Unknown if the tile has none).
    LandClass landClassAt(double latDeg, double lonDeg) const;

    // Builds a tile from decoded arrays (sizes must be 257*257 and 0 or 256*256) and fills the
    // derived fields. Returns null on a size mismatch.
    static std::shared_ptr<const TerrainTile> make(const TileId& id, std::vector<int16_t> heights,
                                                   std::vector<uint8_t> landClass);
};

}  // namespace fsim::terrain
