#include "world/terrain/TerrainPack.h"

#include <cstdio>
#include <cstring>

#include "core/Ini.h"
#include "core/Log.h"

namespace fsim::terrain {
namespace {

constexpr size_t kHeightFileBytes = static_cast<size_t>(kTileSamples) * kTileSamples * 2;  // 132,098
constexpr size_t kLandFileBytes = static_cast<size_t>(kTileCells) * kTileCells;            // 65,536

bool readWholeFile(const std::string& path, std::vector<uint8_t>& out, bool& exists) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    exists = f != nullptr;
    if (!f) return false;
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    std::fseek(f, 0, SEEK_SET);
    if (size < 0) {
        std::fclose(f);
        return false;
    }
    out.resize(static_cast<size_t>(size));
    const size_t got = size > 0 ? std::fread(out.data(), 1, out.size(), f) : 0;
    std::fclose(f);
    return got == out.size();
}

std::string tileRel(const char* dir, const TileId& id, const char* ext) {
    return std::string(dir) + "/" + std::to_string(id.k) + "/" + std::to_string(id.x) + "/" +
           std::to_string(id.y) + ext;
}

}  // namespace

bool readHeightFile(const std::string& path, std::vector<int16_t>& out, std::string& error) {
    std::vector<uint8_t> raw;
    bool exists = false;
    if (!readWholeFile(path, raw, exists)) {
        error = exists ? "cannot read " + path : "missing " + path;
        return false;
    }
    if (raw.size() != kHeightFileBytes) {
        error = path + ": expected " + std::to_string(kHeightFileBytes) + " bytes, got " +
                std::to_string(raw.size());
        return false;
    }
    out.resize(static_cast<size_t>(kTileSamples) * kTileSamples);
    for (size_t i = 0; i < out.size(); ++i)  // little-endian regardless of the host
        out[i] = static_cast<int16_t>(static_cast<uint16_t>(raw[2 * i] | (raw[2 * i + 1] << 8)));
    return true;
}

bool readLandClassFile(const std::string& path, std::vector<uint8_t>& out, std::string& error) {
    error.clear();
    bool exists = false;
    if (!readWholeFile(path, out, exists)) {
        if (exists) error = "cannot read " + path;
        out.clear();
        return false;
    }
    if (out.size() != kLandFileBytes) {
        error = path + ": expected " + std::to_string(kLandFileBytes) + " bytes, got " +
                std::to_string(out.size());
        out.clear();
        return false;
    }
    return true;
}

bool TerrainPack::open(const std::string& dir, std::string& error) {
    open_ = false;
    dir_ = dir;
    info_ = PackInfo{};
    finest_ = coarsest_ = -1;
    for (auto& b : bitmaps_) b.clear();
    for (auto& c : counts_) c = 0;

    Ini ini;
    if (!ini.loadFile(dir + "/pack.ini")) {
        error = "cannot read " + dir + "/pack.ini";
        return false;
    }
    info_.format = ini.getString("pack", "format");
    if (info_.format != "flightsim-terrain") {
        error = "pack.ini: unknown format '" + info_.format + "'";
        return false;
    }
    info_.versionMajor = static_cast<int>(ini.getDouble("pack", "version", 0));
    if (info_.versionMajor != 1) {
        error = "pack.ini: unsupported terrain format version " +
                ini.getString("pack", "version", "(none)") + " (this build reads 1.x)";
        return false;
    }
    info_.samples = ini.getInt("pack", "samples", 0);
    if (info_.samples != kTileSamples) {
        error = "pack.ini: samples = " + std::to_string(info_.samples) + ", expected " +
                std::to_string(kTileSamples);
        return false;
    }
    info_.minLevel = ini.getInt("pack", "min_level", 0);
    info_.maxLevel = ini.getInt("pack", "max_level", 0);
    if (info_.maxLevel < info_.minLevel || info_.maxLevel > kMaxLevel || info_.minLevel < 0) {
        error = "pack.ini: bad level range " + std::to_string(info_.minLevel) + ".." +
                std::to_string(info_.maxLevel);
        return false;
    }
    info_.createdUtc = ini.getString("pack", "created_utc");
    info_.demAttribution = ini.getString("attribution", "dem");
    info_.landcoverAttribution = ini.getString("attribution", "landcover");
    info_.landcoverCredit = ini.getString("attribution", "landcover_credit");
    for (const Ini::Entry& e : ini.entries("regions"))
        info_.regions.push_back(e.first + " = " + e.second);

    for (int k = 0; k <= info_.maxLevel; ++k) {
        const std::string path = dir + "/index/L" + std::to_string(k) + ".bit";
        std::vector<uint8_t> bits;
        bool exists = false;
        if (!readWholeFile(path, bits, exists)) {
            if (exists) LOG_WARN("terrain: cannot read %s, level %d ignored", path.c_str(), k);
            continue;
        }
        const size_t expected = static_cast<size_t>((tileCountAtLevel(k) + 7) / 8);
        if (bits.size() != expected) {
            LOG_WARN("terrain: %s has %zu bytes, expected %zu; level %d ignored", path.c_str(),
                     bits.size(), expected, k);
            continue;
        }
        uint64_t n = 0;
        for (const uint8_t b : bits) n += static_cast<uint64_t>(__builtin_popcount(b));
        if (n == 0) continue;
        counts_[k] = n;
        bitmaps_[k] = std::move(bits);
        if (finest_ < 0 || k > finest_) finest_ = k;
        if (coarsest_ < 0 || k < coarsest_) coarsest_ = k;
    }
    open_ = true;
    return true;
}

bool TerrainPack::levelHasTiles(int k) const {
    return k >= 0 && k <= kMaxLevel && !bitmaps_[k].empty();
}

uint64_t TerrainPack::tileCount(int k) const {
    return k >= 0 && k <= kMaxLevel ? counts_[k] : 0;
}

int TerrainPack::finestLevelAt(double latDeg, double lonDeg) const {
    for (int k = finest_; k >= coarsest_ && k >= 0; --k)
        if (hasTile(tileAt(k, latDeg, lonDeg))) return k;
    return -1;
}

std::string TerrainPack::heightPath(const TileId& id) const {
    return dir_ + "/" + tileRel("h", id, ".h16");
}

std::string TerrainPack::landClassPath(const TileId& id) const {
    return dir_ + "/" + tileRel("c", id, ".lc8");
}

std::shared_ptr<const TerrainTile> TerrainPack::loadTile(const TileId& id, bool wantLandClass,
                                                         std::string& error) const {
    std::vector<int16_t> heights;
    if (!readHeightFile(heightPath(id), heights, error)) return nullptr;
    std::vector<uint8_t> land;
    if (wantLandClass) {
        std::string lcError;
        if (!readLandClassFile(landClassPath(id), land, lcError)) {
            land.clear();
            // A malformed land-class file is worth a log line; a missing one is normal.
            if (!lcError.empty()) LOG_WARN("terrain: %s", lcError.c_str());
        }
    }
    auto tile = TerrainTile::make(id, std::move(heights), std::move(land));
    if (!tile) error = "internal: bad tile arrays for " + heightPath(id);
    return tile;
}

}  // namespace fsim::terrain
