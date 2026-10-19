#pragma once
// Terrain tile scheme math (DATA.md section 4.1). Header-only and free of IO and threads so the
// terrain renderer (W2) and tools can reuse it.
//
// A geographic quadtree. At level k: spanDeg = 90 / 2^k, x in [0, 4*2^k), y in [0, 2*2^k).
// Tile (k,x,y) covers lon [-180 + x*span, -180 + (x+1)*span] and lat [-90 + y*span, ...+span];
// y grows NORTHWARD. Children of (k,x,y) are (k+1, 2x+i, 2y+j). Each tile carries 257 x 257
// corner-sampled heights (shared edges) and 256 x 256 cell-centred land-class cells.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <vector>

#include "core/Units.h"

namespace fsim::terrain {

constexpr int kTileSamples = 257;  // height samples per tile side (corner-sampled)
constexpr int kTileCells = 256;    // land-class cells per tile side (and height intervals)
constexpr int kMaxLevel = 20;      // levels above this are rejected (keeps keys within 64 bits)

struct TileId {
    int k = 0;
    uint32_t x = 0, y = 0;
    friend bool operator==(const TileId& a, const TileId& b) {
        return a.k == b.k && a.x == b.x && a.y == b.y;
    }
    friend bool operator!=(const TileId& a, const TileId& b) { return !(a == b); }
    friend bool operator<(const TileId& a, const TileId& b) {
        if (a.k != b.k) return a.k < b.k;
        if (a.y != b.y) return a.y < b.y;
        return a.x < b.x;
    }
};

struct TileBounds {
    double westDeg = 0, southDeg = 0, eastDeg = 0, northDeg = 0;
    double centerLatDeg() const { return 0.5 * (southDeg + northDeg); }
    double centerLonDeg() const { return 0.5 * (westDeg + eastDeg); }
};

// ---- level geometry ----------------------------------------------------------------------------
inline uint32_t tilesWide(int k) {
    return 4u << k;
}
inline uint32_t tilesHigh(int k) {
    return 2u << k;
}
inline double spanDeg(int k) {
    return 90.0 / static_cast<double>(1u << k);
}
// Spacing of height samples (and the width of a land-class cell), degrees.
inline double stepDeg(int k) {
    return spanDeg(k) / kTileCells;
}
// Approximate sample spacing in meters along a meridian.
inline double sampleSpacingM(int k) {
    return stepDeg(k) * kRadPerDeg * kEarthRadiusM;
}
inline double tileCountAtLevel(int k) {
    return static_cast<double>(tilesWide(k)) * tilesHigh(k);
}

// ---- identity ----------------------------------------------------------------------------------
inline bool isValid(const TileId& id) {
    return id.k >= 0 && id.k <= kMaxLevel && id.x < tilesWide(id.k) && id.y < tilesHigh(id.k);
}
// Unique 64-bit key: level in bits 56.., y in bits 28..55, x in bits 0..27.
inline uint64_t key(const TileId& id) {
    return (static_cast<uint64_t>(id.k) << 56) | (static_cast<uint64_t>(id.y) << 28) | id.x;
}
inline TileId fromKey(uint64_t k) {
    TileId id;
    id.k = static_cast<int>(k >> 56);
    id.y = static_cast<uint32_t>((k >> 28) & 0xFFFFFFFu);
    id.x = static_cast<uint32_t>(k & 0xFFFFFFFu);
    return id;
}
// Index into the presence bitmap of level id.k (DATA 4.4): y * (4 * 2^k) + x.
inline uint64_t bitIndex(const TileId& id) {
    return static_cast<uint64_t>(id.y) * tilesWide(id.k) + id.x;
}

// ---- position <-> tile -------------------------------------------------------------------------
// Wraps a longitude into [-180, 180).
inline double normalizeLonDeg(double lonDeg) {
    if (lonDeg >= -180.0 && lonDeg < 180.0) return lonDeg;
    double v = std::fmod(lonDeg + 180.0, 360.0);
    if (v < 0) v += 360.0;
    return v - 180.0;
}

// The tile of level k containing the point (latitude clamped to +-90, longitude wrapped). Points
// on a shared edge belong to the tile to the north / east.
inline TileId tileAt(int k, double latDeg, double lonDeg) {
    const double inv = static_cast<double>(1u << k) / 90.0;
    const double lat = std::min(90.0, std::max(-90.0, latDeg));
    const double fx = (normalizeLonDeg(lonDeg) + 180.0) * inv;
    const double fy = (lat + 90.0) * inv;
    TileId id;
    id.k = k;
    id.x = static_cast<uint32_t>(std::min<double>(std::floor(fx), tilesWide(k) - 1));
    id.y = static_cast<uint32_t>(std::min<double>(std::floor(fy), tilesHigh(k) - 1));
    return id;
}

inline TileBounds bounds(const TileId& id) {
    const double span = spanDeg(id.k);
    TileBounds b;
    b.westDeg = -180.0 + id.x * span;
    b.eastDeg = b.westDeg + span;
    b.southDeg = -90.0 + id.y * span;
    b.northDeg = b.southDeg + span;
    return b;
}

// ---- hierarchy ---------------------------------------------------------------------------------
inline TileId parent(const TileId& id) {
    return TileId{id.k - 1, id.x >> 1, id.y >> 1};
}
inline std::array<TileId, 4> children(const TileId& id) {
    const uint32_t x = id.x * 2, y = id.y * 2;
    return {TileId{id.k + 1, x, y}, TileId{id.k + 1, x + 1, y}, TileId{id.k + 1, x, y + 1},
            TileId{id.k + 1, x + 1, y + 1}};
}
// The ancestor of id at a coarser (or equal) level.
inline TileId ancestorAt(const TileId& id, int level) {
    const int d = id.k - level;
    return TileId{level, id.x >> d, id.y >> d};
}
inline bool isAncestorOf(const TileId& anc, const TileId& t) {
    return anc.k <= t.k && ancestorAt(t, anc.k) == anc;
}

// ---- distances and circle queries --------------------------------------------------------------
// Distance in meters from a point to the nearest point of the tile (0 when inside), using an
// equirectangular approximation (mean-latitude scaling). Error is well under 1% for distances up
// to ~1000 km, which is all LOD rings and prefetch circles need.
inline double distToTileM(const TileId& id, double latDeg, double lonDeg) {
    const TileBounds b = bounds(id);
    const double lat = std::min(90.0, std::max(-90.0, latDeg));
    const double clampedLat = std::min(b.northDeg, std::max(b.southDeg, lat));
    const double dLatDeg = std::fabs(clampedLat - lat);
    double dl = normalizeLonDeg(lonDeg - b.centerLonDeg());
    const double dLonDeg = std::max(0.0, std::fabs(dl) - 0.5 * spanDeg(id.k));
    const double midLat = 0.5 * (lat + clampedLat);
    const double mPerDeg = kEarthRadiusM * kRadPerDeg;
    const double dx = dLonDeg * std::cos(midLat * kRadPerDeg) * mPerDeg;
    const double dy = dLatDeg * mPerDeg;
    return std::sqrt(dx * dx + dy * dy);
}

// Appends the tiles of level k whose rectangle comes within radiusM of the point (all of them,
// present in a pack or not). Distances use distToTileM's approximation.
inline void tilesIntersectingCircle(int k, double latDeg, double lonDeg, double radiusM,
                                    std::vector<TileId>& out) {
    const double mPerDeg = kEarthRadiusM * kRadPerDeg;
    const double rDeg = radiusM / mPerDeg;
    const double span = spanDeg(k);
    const double lat0 = std::min(90.0, std::max(-90.0, latDeg - rDeg));
    const double lat1 = std::min(90.0, std::max(-90.0, latDeg + rDeg));
    const uint32_t y0 = tileAt(k, lat0, 0).y, y1 = tileAt(k, lat1, 0).y;

    const double worstLat = std::max(std::fabs(lat0), std::fabs(lat1));
    const double cosLat = std::cos(worstLat * kRadPerDeg);
    const uint32_t wide = tilesWide(k);
    int64_t xFirst = 0, xCount = wide;
    if (cosLat > 1e-3 && rDeg / cosLat < 180.0) {
        const double lonSpan = rDeg / cosLat;
        const double fxLo = (normalizeLonDeg(lonDeg) + 180.0 - lonSpan) / span;
        const double fxHi = (normalizeLonDeg(lonDeg) + 180.0 + lonSpan) / span;
        xFirst = static_cast<int64_t>(std::floor(fxLo));
        xCount = std::min<int64_t>(wide, static_cast<int64_t>(std::floor(fxHi)) - xFirst + 1);
    }
    for (uint32_t y = y0; y <= y1; ++y) {
        for (int64_t i = 0; i < xCount; ++i) {
            int64_t xx = (xFirst + i) % static_cast<int64_t>(wide);
            if (xx < 0) xx += wide;
            const TileId id{k, static_cast<uint32_t>(xx), y};
            if (distToTileM(id, latDeg, lonDeg) <= radiusM) out.push_back(id);
        }
    }
}

}  // namespace fsim::terrain

namespace std {
template <>
struct hash<fsim::terrain::TileId> {
    size_t operator()(const fsim::terrain::TileId& id) const noexcept {
        return static_cast<size_t>(fsim::terrain::key(id) * 0x9E3779B97F4A7C15ull);
    }
};
}  // namespace std
