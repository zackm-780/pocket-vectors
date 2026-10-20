#pragma once
// Writes small synthetic terrain packs in the DATA.md section 4 format, for tests that must not
// need real data. Heights are point samples of a function at the grid corners (shared edges are
// bit-identical because coordinates come from global grid indices); land class is sampled at cell
// centres. For a planar function the box-averaged real data and these point samples agree.

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include "world/terrain/TileScheme.h"

namespace fsim::terrain::testing {

// A temporary directory removed on destruction.
struct TempDir {
    TempDir();
    ~TempDir();
    TempDir(const TempDir&) = delete;
    TempDir& operator=(const TempDir&) = delete;
    std::string path;
    std::string sub(const std::string& rel) const { return path + "/" + rel; }
};

using HeightFn = std::function<double(double latDeg, double lonDeg)>;  // meters MSL
using LandFn = std::function<uint8_t(double latDeg, double lonDeg)>;   // LandClass value

class PackWriter {
public:
    PackWriter(const std::string& dir, int minLevel, int maxLevel);

    // Writes <dir>/h/k/x/y.h16 (and .lc8 when land != nullptr) and sets the presence bit.
    void writeTile(const TileId& id, const HeightFn& height, const LandFn& land = nullptr);
    // Same with explicit arrays (257*257 heights, optional 256*256 land class).
    void writeTileRaw(const TileId& id, const std::vector<int16_t>& heights,
                      const std::vector<uint8_t>* land = nullptr);
    // Writes every tile of level k that overlaps the box.
    void writeBox(int k, double southDeg, double westDeg, double northDeg, double eastDeg,
                  const HeightFn& height, const LandFn& land = nullptr);
    // Sets the presence bit without writing files (a listed-but-missing tile).
    void markPresent(const TileId& id);
    // Writes pack.ini and index/L{k}.bit for every level 0..maxLevel that has a tile.
    void finish();

    const std::string& dir() const { return dir_; }

    // The 257 x 257 corner samples of a function (rounded to int16, row-major north-up).
    static std::vector<int16_t> sampleHeights(const TileId& id, const HeightFn& height);

    // Grid sample coordinates (global grid indices, so neighbours share edge values exactly).
    static double sampleLatDeg(const TileId& id, int row);
    static double sampleLonDeg(const TileId& id, int col);

private:
    std::string dir_;
    int minLevel_, maxLevel_;
    std::vector<std::vector<uint8_t>> bitmaps_;
};

bool writeFile(const std::string& path, const void* data, size_t size);  // creates directories

}  // namespace fsim::terrain::testing
