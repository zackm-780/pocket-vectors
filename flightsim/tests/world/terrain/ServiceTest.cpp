// TerrainService on synthetic packs: sampling, seams, fallback, async loading, cache cap.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <thread>

#include "world/terrain/SyntheticPack.h"
#include "world/terrain/TerrainService.h"

using namespace fsim;
using namespace fsim::terrain;
using namespace fsim::terrain::testing;

namespace {

double plane(double lat, double lon) {
    return 1000.0 + 3000.0 * (lat - 45.0) + 1500.0 * (lon + 122.0);
}
double smooth(double lat, double lon) {
    return 800.0 + 300.0 * std::sin(1.1 * lat) * std::cos(1.3 * lon);
}

// update + wait for the worker to finish everything it was asked for.
bool pump(TerrainService& s, double lat, double lon, double timeoutS = 20.0) {
    s.update(lat, lon);
    return s.waitForIdle(timeoutS);
}

#define CHECK_NEAR(a, b, tol)                                                                      \
    CHECK(std::fabs(static_cast<double>(a) - static_cast<double>(b)) <= (tol))

double nowS() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

const TileId kP5{5, 20, 48};  // L5 tile: lon -123.75..-120.9375, lat 45..47.8125
const TileId kC6{6, 40, 96};  // its south-west child: lon -123.75..-122.34, lat 45..46.41

}  // namespace

TEST_CASE("terrain: service on a missing pack is inert and never throws") {
    TerrainService svc("/nonexistent/terrain/dir");
    CHECK(!svc.isOpen());
    CHECK(!svc.error().empty());
    svc.update(45, -122);
    svc.prefetch(45, -122, 1000);
    CHECK(svc.waitForIdle(0.1));
    CHECK(svc.elevationM(45, -122) == 0.0f);
    CHECK(svc.landClass(45, -122) == LandClass::Water);
    CHECK(svc.isDetailReady(45, -122));
    CHECK(svc.finestTileAt(45, -122).get() == nullptr);
}

TEST_CASE("terrain: heights are north-up and corner-sampled (orientation)") {
    TempDir dir;
    PackWriter w(dir.path, 5, 5);
    // One spike on grid vertex row 10, column 20 of a zero tile.
    std::vector<int16_t> h(257 * 257, 0);
    h[10 * 257 + 20] = 5000;
    w.writeTileRaw(kP5, h);
    w.finish();

    TerrainService svc(dir.path);
    REQUIRE(svc.isOpen());
    TerrainQuery& q = svc;                  // through the contract
    CHECK(q.elevationM(46, -122) == 0.0f);  // nothing loaded yet: never blocks, returns 0
    REQUIRE(pump(svc, 46, -122));

    const TileBounds b = bounds(kP5);
    const double step = stepDeg(5);
    const double lat10 = b.northDeg - 10 * step;  // row 10 is 10 steps SOUTH of the north edge
    const double lon20 = b.westDeg + 20 * step;   // column 20 is 20 steps EAST of the west edge
    CHECK(q.elevationM(lat10, lon20) == doctest::Approx(5000.0f).epsilon(1e-4));
    CHECK(q.elevationM(lat10, lon20 + step) == doctest::Approx(0.0f).epsilon(1e-3));
    CHECK(q.elevationM(lat10 - step, lon20) == doctest::Approx(0.0f).epsilon(1e-3));
    // Mirrored positions (what a south-up or west-first reader would hit) stay at zero.
    CHECK(q.elevationM(b.southDeg + 10 * step, lon20) == doctest::Approx(0.0f).epsilon(1e-3));
    CHECK(q.elevationM(lat10, b.eastDeg - 20 * step) == doctest::Approx(0.0f).epsilon(1e-3));
    // Bilinear between samples: halfway to the next column / row is half the spike.
    CHECK(q.elevationM(lat10, lon20 + 0.5 * step) == doctest::Approx(2500.0f).epsilon(1e-3));
    CHECK(q.elevationM(lat10 + 0.5 * step, lon20) == doctest::Approx(2500.0f).epsilon(1e-3));
    CHECK(q.elevationM(lat10 + 0.5 * step, lon20 - 0.5 * step) ==
          doctest::Approx(1250.0f).epsilon(1e-3));
}

TEST_CASE("terrain: bilinear reproduces a plane across the whole tile, corners included") {
    TempDir dir;
    PackWriter w(dir.path, 5, 5);
    w.writeTile(kP5, plane);
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(pump(svc, 46, -122));
    const TileBounds b = bounds(kP5);
    const double eps = 1e-9;
    const double lats[] = {b.northDeg - eps, 47.0, 46.123456, 45.5, b.southDeg};
    const double lons[] = {b.westDeg, -123.4, -122.2222, -121.0, b.eastDeg - eps};
    for (const double lat : lats)
        for (const double lon : lons) CHECK_NEAR(svc.elevationM(lat, lon), plane(lat, lon), 0.6);
}

TEST_CASE("terrain: land class is cell-centred, north-up") {
    TempDir dir;
    PackWriter w(dir.path, 5, 5);
    const double step = stepDeg(5);
    // class = 1 + (column index % 3) + 3 * (row index % 3), from the global cell index.
    auto cls = [step](double lat, double lon) -> uint8_t {
        const long ci = static_cast<long>(std::floor((lon + 180.0) / step));
        const long ri = static_cast<long>(std::floor((lat + 90.0) / step));
        return static_cast<uint8_t>(1 + (ci % 3) + 3 * (ri % 3));
    };
    w.writeTile(kP5, plane, cls);
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(pump(svc, 46, -122));

    int checked = 0;
    for (long ci = 5200; ci < 5240; ci += 3) {  // an arbitrary block of cells in the tile
        for (long ri = 12300; ri < 12340; ri += 7) {
            const double lat = -90.0 + (ri + 0.5) * step;
            const double lon = -180.0 + (ci + 0.5) * step;
            const uint8_t want = cls(lat, lon);
            // Inside the cell: centre and +-0.45 cell offsets in every direction.
            for (const double dy : {0.0, -0.45, 0.45})
                for (const double dx : {0.0, -0.45, 0.45})
                    CHECK(static_cast<int>(svc.landClass(lat + dy * step, lon + dx * step)) ==
                          want);
            // Just past the cell edge the neighbouring cell answers.
            CHECK(static_cast<int>(svc.landClass(lat, lon + 0.55 * step)) == cls(lat, lon + step));
            CHECK(static_cast<int>(svc.landClass(lat + 0.55 * step, lon)) == cls(lat + step, lon));
            ++checked;
        }
    }
    CHECK(checked > 20);
}

TEST_CASE("terrain: seams between adjacent tiles of one level are continuous") {
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    const TileId a = tileAt(7, 45.5, -122.6);
    const TileId b{7, a.x + 1, a.y};
    w.writeTile(a, smooth);
    w.writeTile(b, smooth);
    w.finish();
    TerrainService svc(dir.path);
    const TileBounds ba = bounds(a);
    REQUIRE(pump(svc, ba.centerLatDeg(), ba.eastDeg));
    CHECK(svc.answeringLevel(ba.centerLatDeg(), ba.eastDeg - 0.1) == 7);

    const double seamLon = ba.eastDeg;
    const double lat = ba.centerLatDeg() + 0.0123;
    // Edge column of one tile == edge column of the next: both sides of the seam agree.
    CHECK(svc.elevationM(lat, seamLon - 1e-10) ==
          doctest::Approx(svc.elevationM(lat, seamLon + 1e-10)).epsilon(1e-5));
    // Walk across in tenths of a sample spacing: no step bigger than the local slope allows.
    double prev = svc.elevationM(lat, seamLon - 30 * stepDeg(7));
    double worst = 0;
    for (int i = -299; i <= 300; ++i) {
        const double v = svc.elevationM(lat, seamLon + i * 0.1 * stepDeg(7));
        worst = std::max(worst, std::fabs(v - prev));
        prev = v;
    }
    CHECK(worst < 0.6);
    // The same through the north edge.
    const TileId c{7, a.x, a.y + 1};
    TempDir dir2;
    PackWriter w2(dir2.path, 7, 7);
    w2.writeTile(a, smooth);
    w2.writeTile(c, smooth);
    w2.finish();
    TerrainService svc2(dir2.path);
    REQUIRE(pump(svc2, ba.northDeg, ba.centerLonDeg()));
    CHECK(svc2.elevationM(ba.northDeg - 1e-10, ba.centerLonDeg()) ==
          doctest::Approx(svc2.elevationM(ba.northDeg + 1e-10, ba.centerLonDeg())).epsilon(1e-5));
}

TEST_CASE("terrain: seams across levels and ancestor fallback") {
    TempDir dir;
    PackWriter w(dir.path, 5, 6);
    w.writeTile(kP5, smooth);
    w.writeTile(kC6, smooth);  // only the south-west child exists at level 6
    w.finish();
    TerrainService svc(dir.path);
    const double lat = 45.7;
    REQUIRE(pump(svc, lat, -123.0));

    const double seamLon = bounds(kC6).eastDeg;  // -122.34375
    REQUIRE(seamLon == doctest::Approx(-122.34375));
    CHECK(svc.answeringLevel(lat, seamLon - 0.01) == 6);
    CHECK(svc.answeringLevel(lat, seamLon + 0.01) == 5);  // fallback to the finest ancestor
    CHECK(svc.answeringLevel(47.0, -121.5) == 5);

    // The level change is a seam only up to smoothing (area averages): tiny for smooth terrain.
    const double fine = svc.elevationM(lat, seamLon - 1e-7);
    const double coarse = svc.elevationM(lat, seamLon + 1e-7);
    CHECK(std::fabs(fine - coarse) < 1.5);
    CHECK_NEAR(fine, smooth(lat, seamLon), 1.0);
    CHECK_NEAR(coarse, smooth(lat, seamLon), 1.0);
    // Walking across: continuous within the slope.
    double prev = svc.elevationM(lat, seamLon - 0.05), worst = 0;
    for (int i = 1; i <= 200; ++i) {
        const double v = svc.elevationM(lat, seamLon - 0.05 + i * 0.0005);
        worst = std::max(worst, std::fabs(v - prev));
        prev = v;
    }
    CHECK(worst < 1.5);
    // Fallback values are the parent's samples.
    CHECK_NEAR(svc.elevationM(46.5, -121.5), smooth(46.5, -121.5), 1.0);
}

TEST_CASE("terrain: a void sample makes that tile fall back to its parent") {
    TempDir dir;
    PackWriter w(dir.path, 5, 6);
    w.writeTile(kP5, plane);
    std::vector<int16_t> h = PackWriter::sampleHeights(kC6, plane);
    h[100 * 257 + 100] = -32768;
    w.writeTileRaw(kC6, h);
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(pump(svc, 45.7, -123.0));

    const TileBounds b = bounds(kC6);
    const double step = stepDeg(6);
    // Between samples (100..101, 100..101): the cell touches the void.
    const double latVoid = b.northDeg - 100.5 * step, lonVoid = b.westDeg + 100.5 * step;
    CHECK(svc.answeringLevel(latVoid, lonVoid) == 5);
    CHECK_NEAR(svc.elevationM(latVoid, lonVoid), plane(latVoid, lonVoid), 0.6);
    // Two cells away it is the fine tile again.
    CHECK(svc.answeringLevel(b.northDeg - 103.5 * step, b.westDeg + 103.5 * step) == 6);
    CHECK(svc.answeringLevel(b.northDeg - 50.5 * step, b.westDeg + 50.5 * step) == 6);
}

TEST_CASE("terrain: missing tiles mean ocean (0 m, Water) or not-loaded-yet") {
    TempDir dir;
    PackWriter w(dir.path, 5, 5);
    w.writeTile(kP5, [](double, double) { return 321.0; }, [](double, double) { return 3; });
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(svc.isOpen());

    // Far from the pack: nothing to wait for, sea level, Water.
    CHECK(svc.elevationM(0, 0) == 0.0f);
    CHECK(svc.landClass(0, 0) == LandClass::Water);
    CHECK(svc.isDetailReady(0, 0));
    CHECK(svc.answeringLevel(0, 0) == -1);

    // Inside a listed tile that is not loaded yet: sea level, land class Unknown, not ready.
    CHECK(svc.elevationM(46, -122) == 0.0f);
    CHECK(svc.landClass(46, -122) == LandClass::Unknown);
    CHECK(!svc.isDetailReady(46, -122));

    REQUIRE(pump(svc, 46, -122));
    CHECK(svc.elevationM(46, -122) == doctest::Approx(321.0f));
    CHECK(svc.landClass(46, -122) == LandClass::Shrub);
    CHECK(svc.isDetailReady(46, -122));
    // Right outside the covered region (north of the tile): back to the ocean answer.
    CHECK(svc.elevationM(48.5, -122) == 0.0f);
    CHECK(svc.landClass(48.5, -122) == LandClass::Water);
    CHECK(svc.isDetailReady(48.5, -122));
}

TEST_CASE("terrain: an omitted finest tile uses the finest ancestor present") {
    TempDir dir;
    PackWriter w(dir.path, 5, 7);
    // Land everywhere in P5 at 500 m; the L6 child and L7 grandchild hold different values so the
    // answering level is visible in the data.
    w.writeTile(kP5, [](double, double) { return 500.0; });
    w.writeTile(kC6, [](double, double) { return 600.0; });
    const TileId g7 = children(kC6)[0];
    w.writeTile(g7, [](double, double) { return 700.0; });
    w.finish();
    TerrainService svc(dir.path);
    const TileBounds bg = bounds(g7), bc = bounds(kC6);
    REQUIRE(pump(svc, bg.centerLatDeg(), bg.centerLonDeg()));
    CHECK(svc.elevationM(bg.centerLatDeg(), bg.centerLonDeg()) == doctest::Approx(700.0f));
    // A point in the L6 tile but outside its first L7 child: L7 is omitted there -> L6 answers.
    CHECK(svc.elevationM(bc.northDeg - 0.1, bc.eastDeg - 0.1) == doctest::Approx(600.0f));
    CHECK(svc.elevationM(46.8, -121.0) == doctest::Approx(500.0f));
    CHECK(svc.isDetailReady(bc.northDeg - 0.1, bc.eastDeg - 0.1));
}

TEST_CASE("terrain: isDetailReady needs the tiles around the point, not just the point") {
    TempDir dir;
    PackWriter w(dir.path, 6, 6);
    const TileId a{6, 40, 96}, b{6, 41, 96};
    w.writeTile(a, plane);
    w.writeTile(b, plane);
    w.finish();
    TerrainService svc(dir.path);
    const TileBounds ba = bounds(a);
    const double lat = ba.centerLatDeg();
    svc.update(0, 0);  // camera far away so the camera ring does not interfere
    svc.requestTile(a);
    REQUIRE(svc.waitForIdle(20));
    CHECK(svc.isTileResident(a));
    CHECK(!svc.isTileResident(b));
    const double lonFar = ba.westDeg + 0.3;     // ~ 20 km from the shared edge
    const double lonNear = ba.eastDeg - 0.001;  // ~ 80 m from the shared edge
    CHECK(svc.isDetailReady(lat, lonFar));
    CHECK(!svc.isDetailReady(lat, lonNear));  // the 400 m margin reaches into tile b
    svc.requestTile(b);
    REQUIRE(svc.waitForIdle(20));
    CHECK(svc.isDetailReady(lat, lonNear));
}

TEST_CASE("terrain: requests never block and results arrive asynchronously") {
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    const TileId mid = tileAt(7, 45.5, -122.6);
    for (int dy = -1; dy <= 1; ++dy)
        for (int dx = -1; dx <= 1; ++dx) w.writeTile(TileId{7, mid.x + dx, mid.y + dy}, plane);
    w.finish();

    TerrainConfig cfg;
    cfg.debugLoadDelayMs = 100;  // nine tiles take about a second on the worker
    cfg.loadLandClass = false;
    TerrainService svc(dir.path, cfg);
    REQUIRE(svc.isOpen());
    const TileBounds bm = bounds(mid);
    const double lat = bm.centerLatDeg(), lon = bm.centerLonDeg();

    const double t0 = nowS();
    svc.prefetch(lat, lon, 80000);
    svc.update(lat, lon);
    double sum = 0;
    for (int i = 0; i < 2000; ++i) sum += svc.elevationM(lat + i * 1e-5, lon);
    const bool ready = svc.isDetailReady(lat, lon);
    const double elapsedS = nowS() - t0;

    // Structural: the call returned while the worker was still busy with most of the queue.
    const TerrainStats early = svc.stats();
    CHECK(early.residentTiles < 9);
    CHECK((early.queuedTiles > 0 || early.loading));
    CHECK(!ready);
    CHECK(sum == 0.0);      // nothing resident yet: sea level
    CHECK(elapsedS < 0.5);  // generous: far below the ~1 s the loads take
    // Eventually ready, with the camera call pumping results in.
    bool ok = false;
    for (int i = 0; i < 5000 && !ok; ++i) {
        svc.update(lat, lon);
        ok = svc.isDetailReady(lat, lon);
        if (!ok) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    CHECK(ok);
    CHECK_NEAR(svc.elevationM(lat, lon), plane(lat, lon), 0.6);
    CHECK(svc.waitForIdle(20));
    CHECK(svc.stats().residentTiles == 9);
    CHECK(svc.stats().tilesLoaded == 9);
}

TEST_CASE("terrain: destroying the service with loads pending joins promptly") {
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    const TileId mid = tileAt(7, 45.5, -122.6);
    for (int dx = -2; dx <= 2; ++dx) w.writeTile(TileId{7, mid.x + dx, mid.y}, plane);
    w.finish();
    TerrainConfig cfg;
    cfg.debugLoadDelayMs = 150;
    cfg.loadLandClass = false;
    const double t0 = nowS();
    {
        TerrainService svc(dir.path, cfg);
        svc.prefetch(45.5, -122.6, 200000);
        svc.update(45.5, -122.6);
        CHECK(svc.stats().queuedTiles > 0);
    }  // only the tile in flight may finish: well under the 750 ms that all five would take
    CHECK(nowS() - t0 < 0.6);
}

TEST_CASE("terrain: resident tile API for the renderer") {
    TempDir dir;
    PackWriter w(dir.path, 5, 6);
    w.writeTile(kP5, plane);
    w.writeTile(kC6, plane);
    w.finish();
    TerrainService svc(dir.path);
    const uint64_t rev0 = svc.residentRevision();
    std::vector<std::shared_ptr<const TerrainTile>> tiles;
    svc.residentTiles(tiles);
    CHECK(tiles.empty());
    CHECK(svc.packHasTile(kC6));
    CHECK(!svc.packHasTile(TileId{6, 41, 96}));
    CHECK(svc.tile(kC6).get() == nullptr);

    REQUIRE(pump(svc, 45.7, -123.0));
    CHECK(svc.residentRevision() != rev0);
    svc.residentTiles(tiles);
    REQUIRE(tiles.size() == 2);
    auto c6 = svc.tile(kC6);
    REQUIRE(static_cast<bool>(c6));
    CHECK(c6->id == kC6);
    CHECK(c6->heightsM.size() == 257u * 257u);
    CHECK(svc.finestTileAt(45.7, -123.0).get() == c6.get());
    CHECK(svc.finestTileAt(46.5, -121.5)->id == kP5);
    CHECK(svc.finestLevel() == 6);
    CHECK(svc.coarsestLevel() == 5);
    CHECK(svc.ringRadiusM(6) == doctest::Approx(20000.0));
    CHECK(svc.ringRadiusM(5) == doctest::Approx(40000.0));
    // Plane values: min/max from the corners.
    CHECK(c6->maxHeightM > c6->minHeightM);
}

TEST_CASE("terrain: cache cap holds and eviction is LRU (touched tiles survive)") {
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    const TileId first = tileAt(7, 45.5, -122.6);
    std::vector<TileId> strip;
    for (uint32_t i = 0; i < 12; ++i) {
        strip.push_back(TileId{7, first.x + i, first.y});
        w.writeTile(strip.back(), plane);
    }
    w.finish();

    TerrainConfig cfg;
    cfg.cacheMaxBytes = 1u << 20;  // 1 MiB: seven 132 KB tiles fit, an eighth does not
    cfg.loadLandClass = false;
    TerrainService svc(dir.path, cfg);
    svc.update(0, 0);  // camera at sea: the ring wants nothing, only explicit requests count
    for (int i = 0; i < 7; ++i) svc.requestTile(strip[i]);
    REQUIRE(svc.waitForIdle(20));
    CHECK(svc.stats().residentTiles == 7);
    CHECK(svc.stats().residentBytes <= cfg.cacheMaxBytes);
    CHECK(svc.stats().tilesEvicted == 0);

    auto held = svc.tile(strip[1]);  // a renderer holding a tile keeps it alive after eviction
    REQUIRE(static_cast<bool>(held));

    svc.update(0, 0);  // a new frame, so a touch is newer than the load time
    const TileBounds b0 = bounds(strip[0]);
    CHECK(svc.elevationM(b0.centerLatDeg(), b0.centerLonDeg()) != 0.0f);  // touches strip[0]
    svc.requestTile(strip[7]);
    REQUIRE(svc.waitForIdle(20));
    CHECK(svc.stats().residentBytes <= cfg.cacheMaxBytes);
    CHECK(svc.stats().tilesEvicted == 1);
    CHECK(svc.isTileResident(strip[0]));          // recently used: kept
    CHECK(!svc.isTileResident(strip[1]));         // least recently used (ties by key): evicted
    CHECK(svc.isTileResident(strip[7]));          // the newcomer
    CHECK(held->heightsM.size() == 257u * 257u);  // still valid
}

TEST_CASE("terrain: fly-through keeps the cache under its cap and loads ahead") {
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    const TileId first = tileAt(7, 45.5, -124.0);
    const int count = 30;
    for (int i = 0; i < count; ++i) w.writeTile(TileId{7, first.x + i, first.y}, plane);
    w.finish();

    TerrainConfig cfg;
    cfg.cacheMaxBytes = 1u << 20;
    cfg.loadLandClass = false;
    TerrainService svc(dir.path, cfg);
    REQUIRE(svc.isOpen());

    const TileBounds b0 = bounds(first);
    const double lat = b0.centerLatDeg();
    const double startLon = b0.westDeg + 0.05;
    const double endLon = bounds(TileId{7, first.x + count - 1, first.y}).eastDeg - 0.05;
    size_t peak = 0;
    int steps = 0;
    for (double lon = startLon; lon < endLon; lon += 0.05) {  // ~4 km per step
        svc.update(lat, lon);
        REQUIRE(svc.waitForIdle(20));
        peak = std::max(peak, svc.stats().residentBytes);
        CHECK(svc.stats().residentBytes <= cfg.cacheMaxBytes);
        CHECK(svc.isDetailReady(lat, lon));
        CHECK_NEAR(svc.elevationM(lat, lon), plane(lat, lon), 0.6);
        ++steps;
    }
    const TerrainStats s = svc.stats();
    CHECK(steps > 300);
    CHECK(s.tilesLoaded >= static_cast<uint64_t>(count));  // the whole strip streamed through
    CHECK(s.tilesEvicted >= static_cast<uint64_t>(count - 7));
    CHECK(peak <= cfg.cacheMaxBytes);
    CHECK(s.residentTiles <= 7);
}

TEST_CASE("terrain: the camera ring loads finer levels near and coarser levels far") {
    TempDir dir;
    PackWriter w(dir.path, 5, 7);
    // L5 tile, its L6 SW child and that child's four L7 children.
    w.writeTile(kP5, plane);
    w.writeTile(kC6, plane);
    for (const TileId& g : children(kC6)) w.writeTile(g, plane);
    w.finish();
    TerrainConfig cfg;
    cfg.detailRadiusM = 5000;  // L7 within 5 km, L6 within 10 km, L5 within 20 km
    cfg.loadLandClass = false;
    TerrainService svc(dir.path, cfg);
    const TileBounds b7 = bounds(children(kC6)[0]);
    const double lat = b7.southDeg + 0.01, lon = b7.westDeg + 0.01;  // near the SW corner of C6
    REQUIRE(pump(svc, lat, lon));
    CHECK(svc.isTileResident(kP5));
    CHECK(svc.isTileResident(kC6));
    CHECK(svc.isTileResident(children(kC6)[0]));
    // The L7 tiles on the far side of C6 (~50+ km away) are outside the 5 km ring.
    CHECK(!svc.isTileResident(children(kC6)[3]));
    CHECK(svc.answeringLevel(lat, lon) == 7);
}
