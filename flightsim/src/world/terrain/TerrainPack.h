#pragma once
// Reader for a terrain pack on disk (DATA.md sections 3-4): pack.ini, the presence bitmaps
// index/L{k}.bit, and per-tile .h16 / .lc8 files. After open() the object is immutable, so
// hasTile() and loadTile() may be called from any thread (the loader thread uses them).

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "world/terrain/TerrainTile.h"
#include "world/terrain/TileScheme.h"

namespace fsim::terrain {

struct PackInfo {
    std::string format;
    int versionMajor = 0;
    int samples = 0;
    int minLevel = 0, maxLevel = 0;  // from pack.ini; presence bitmaps are authoritative
    std::string createdUtc;
    std::string demAttribution, landcoverAttribution, landcoverCredit;
    std::vector<std::string> regions;  // informational lines of [regions]
};

class TerrainPack {
public:
    // Reads <dir>/pack.ini and the presence bitmaps of every level in 0..maxLevel that has one.
    // On failure returns false and fills `error` (unknown format/version, wrong sample count,
    // missing pack.ini). A missing or wrong-sized bitmap only means "no tiles at that level".
    bool open(const std::string& dir, std::string& error);

    bool isOpen() const { return open_; }
    const std::string& dir() const { return dir_; }
    const PackInfo& info() const { return info_; }

    // Finest and coarsest level that has at least one tile (-1 if the pack is empty).
    int finestLevel() const { return finest_; }
    int coarsestLevel() const { return coarsest_; }
    bool levelHasTiles(int k) const;

    // True if the presence bitmap lists the tile (DATA 4.4).
    bool hasTile(const TileId& id) const {
        if (id.k < 0 || id.k > kMaxLevel) return false;
        const std::vector<uint8_t>& bits = bitmaps_[id.k];
        if (bits.empty() || id.x >= tilesWide(id.k) || id.y >= tilesHigh(id.k)) return false;
        const uint64_t i = bitIndex(id);
        return (bits[i >> 3] >> (i & 7)) & 1u;
    }
    // Number of tiles listed at a level.
    uint64_t tileCount(int k) const;
    // Finest level whose tile covering the point is listed, or -1 if none is.
    int finestLevelAt(double latDeg, double lonDeg) const;

    std::string heightPath(const TileId& id) const;
    std::string landClassPath(const TileId& id) const;

    // Reads and decodes a tile (blocking file IO; call from the loader thread). A missing .lc8 is
    // not an error: the tile is returned without land class. A missing or malformed .h16 returns
    // null and fills `error`.
    std::shared_ptr<const TerrainTile> loadTile(const TileId& id, bool wantLandClass,
                                                std::string& error) const;

private:
    bool open_ = false;
    std::string dir_;
    PackInfo info_;
    std::vector<uint8_t> bitmaps_[kMaxLevel + 1];
    uint64_t counts_[kMaxLevel + 1] = {};
    int finest_ = -1, coarsest_ = -1;
};

// ---- low-level decoders (exposed for tests and tools) --------------------------------------------
// Little-endian int16 x (257*257), exactly 132,098 bytes.
bool readHeightFile(const std::string& path, std::vector<int16_t>& out, std::string& error);
// uint8 x (256*256), exactly 65,536 bytes. Returns false with an empty `error` if the file does
// not exist (optional data), and false with a message if it exists but is malformed.
bool readLandClassFile(const std::string& path, std::vector<uint8_t>& out, std::string& error);

}  // namespace fsim::terrain
