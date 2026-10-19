// Tile scheme math (DATA.md 4.1).
#include <doctest/doctest.h>

#include <set>

#include "world/terrain/TileScheme.h"

using namespace fsim;
using namespace fsim::terrain;

TEST_CASE("terrain: level geometry matches the DATA.md table") {
    CHECK(spanDeg(0) == doctest::Approx(90.0));
    CHECK(spanDeg(5) == doctest::Approx(2.8125));
    CHECK(spanDeg(10) == doctest::Approx(0.087890625));
    CHECK(stepDeg(10) == doctest::Approx(0.000343322753906));
    CHECK(tilesWide(3) == 32u);
    CHECK(tilesHigh(3) == 16u);
    CHECK(tileCountAtLevel(10) == doctest::Approx(8.0 * 1048576.0));  // bitmap = 1 MiB
    CHECK(sampleSpacingM(10) == doctest::Approx(38.2).epsilon(0.01));
    CHECK(sampleSpacingM(6) == doctest::Approx(611.0).epsilon(0.01));
}

TEST_CASE("terrain: bounds, tileAt and shared edges") {
    const TileBounds b0 = bounds(TileId{0, 0, 0});
    CHECK(b0.westDeg == -180.0);
    CHECK(b0.southDeg == -90.0);
    CHECK(b0.eastDeg == -90.0);
    CHECK(b0.northDeg == 0.0);
    const TileBounds b1 = bounds(TileId{0, 3, 1});
    CHECK(b1.westDeg == 90.0);
    CHECK(b1.eastDeg == 180.0);
    CHECK(b1.southDeg == 0.0);
    CHECK(b1.northDeg == 90.0);

    // Mt Hood area at level 10: the tile contains the point and y grows northward.
    const TileId t = tileAt(10, 45.3736, -121.6959);
    const TileBounds b = bounds(t);
    CHECK(b.westDeg <= -121.6959);
    CHECK(b.eastDeg > -121.6959);
    CHECK(b.southDeg <= 45.3736);
    CHECK(b.northDeg > 45.3736);
    CHECK(tileAt(10, b.northDeg + 0.01, -121.6959).y == t.y + 1);
    CHECK(tileAt(10, 45.3736, b.eastDeg + 0.01).x == t.x + 1);

    // A point exactly on a shared edge belongs to the north / east tile.
    const TileId edge = tileAt(5, 45.0, -123.75);
    CHECK(bounds(edge).southDeg == 45.0);
    CHECK(bounds(edge).westDeg == -123.75);

    // Poles and the antimeridian wrap or clamp instead of leaving the grid.
    CHECK(tileAt(5, 90.0, 0.0).y == tilesHigh(5) - 1);
    CHECK(tileAt(5, -90.0, 0.0).y == 0u);
    CHECK(tileAt(5, 0.0, 180.0).x == 0u);
    CHECK(tileAt(5, 0.0, -180.0).x == 0u);
    CHECK(tileAt(5, 0.0, 179.999).x == tilesWide(5) - 1);
    CHECK(tileAt(5, 0.0, 540.0).x == 0u);
}

TEST_CASE("terrain: hierarchy and keys") {
    const TileId t{7, 123, 77};
    const auto kids = children(t);
    std::set<TileId> seen;
    for (const TileId& c : kids) {
        CHECK(c.k == 8);
        CHECK(parent(c) == t);
        CHECK(isAncestorOf(t, c));
        const TileBounds cb = bounds(c), pb = bounds(t);
        CHECK(cb.westDeg >= pb.westDeg);
        CHECK(cb.eastDeg <= pb.eastDeg);
        CHECK(cb.southDeg >= pb.southDeg);
        CHECK(cb.northDeg <= pb.northDeg);
        seen.insert(c);
    }
    CHECK(seen.size() == 4);
    CHECK(ancestorAt(TileId{10, 1000, 700}, 7) == TileId{7, 125, 87});
    CHECK(ancestorAt(t, 7) == t);
    CHECK(!isAncestorOf(TileId{8, 0, 0}, t));
    for (const TileId& id : {TileId{0, 0, 0}, TileId{10, 4095, 2047}, TileId{20, 4194303, 2097151}}) {
        CHECK(isValid(id));
        CHECK(fromKey(key(id)) == id);
    }
    CHECK(!isValid(TileId{3, 32, 0}));
    CHECK(!isValid(TileId{3, 0, 16}));
    CHECK(!isValid(TileId{21, 0, 0}));
    // Bit index per DATA 4.4.
    CHECK(bitIndex(TileId{3, 3, 1}) == 35u);
}

namespace {
std::set<TileId> bruteForce(int k, double lat, double lon, double radiusM) {
    std::set<TileId> out;
    const double rDeg = radiusM / (kEarthRadiusM * kRadPerDeg);
    const TileId lo = tileAt(k, lat - 1.5 * rDeg - spanDeg(k), 0);
    const TileId hi = tileAt(k, lat + 1.5 * rDeg + spanDeg(k), 0);
    for (uint32_t y = lo.y; y <= hi.y; ++y)
        for (uint32_t x = 0; x < tilesWide(k); ++x)
            if (distToTileM(TileId{k, x, y}, lat, lon) <= radiusM) out.insert(TileId{k, x, y});
    return out;
}
}  // namespace

TEST_CASE("terrain: tilesIntersectingCircle matches brute force") {
    struct Case {
        int k;
        double lat, lon, r;
    };
    const Case cases[] = {
        {7, 45.5887, -122.5975, 30000},   // KPDX
        {10, 45.5887, -122.5975, 50000},  // the benchmark circle
        {5, 45.0, -123.75, 100000},       // centred on a tile corner
        {6, 0.0, 179.9, 80000},           // across the antimeridian
        {6, 10.0, -179.95, 80000},        // from the other side
        {7, 89.9, 10.0, 50000},           // near the pole: every column in the top row
        {4, -45.0, 100.0, 1000000},       // large ring
        {8, 45.0, -120.0, 1.0},           // tiny circle on an edge intersection
    };
    for (const Case& c : cases) {
        std::vector<TileId> got;
        tilesIntersectingCircle(c.k, c.lat, c.lon, c.r, got);
        const std::set<TileId> gotSet(got.begin(), got.end());
        CHECK(gotSet.size() == got.size());  // no duplicates
        const std::set<TileId> want = bruteForce(c.k, c.lat, c.lon, c.r);
        CHECK(gotSet == want);
        CHECK(!got.empty());
    }
}

TEST_CASE("terrain: distToTileM is zero inside and grows outside") {
    const TileId t = tileAt(8, 45.5, -122.5);
    const TileBounds b = bounds(t);
    CHECK(distToTileM(t, b.centerLatDeg(), b.centerLonDeg()) == 0.0);
    // 0.01 deg north of the north edge is about 1.11 km.
    CHECK(distToTileM(t, b.northDeg + 0.01, b.centerLonDeg()) ==
          doctest::Approx(1111.95).epsilon(0.01));
    // Across the antimeridian.
    const TileId w{6, 0, tileAt(6, 0, 0).y};
    CHECK(distToTileM(w, 0.0, 179.99) < 2000.0);
}
