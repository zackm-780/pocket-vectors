// Benchmark and spot checks on the REAL terrain pack. Skipped by default (CI has no data):
//   FSIM_TERRAIN_PACK=/path/to/data/terrain ./flightsim_tests --test-case='terrain-bench:*' --no-skip
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include "core/Units.h"
#include "world/terrain/TerrainService.h"

using namespace fsim;
using namespace fsim::terrain;

namespace {

double nowS() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

double rssMb() {
    std::ifstream f("/proc/self/status");
    std::string line;
    while (std::getline(f, line))
        if (line.rfind("VmRSS:", 0) == 0) return std::atof(line.c_str() + 6) / 1024.0;
    return 0;
}

std::string packDir() {
    const char* env = std::getenv("FSIM_TERRAIN_PACK");
    return env ? env : "data/terrain";
}

constexpr double kPdxLat = 45.5887, kPdxLon = -122.5975;  // KPDX reference point, field 31 ft
constexpr double kHoodLat = 45.3735, kHoodLon = -121.6959;
constexpr double kStHelensLat = 46.1914, kStHelensLon = -122.1944;

}  // namespace

TEST_CASE("terrain-bench: cold load of 50 km around KPDX and query throughput" * doctest::skip()) {
    const double rss0 = rssMb();
    TerrainService svc(packDir());
    REQUIRE_MESSAGE(svc.isOpen(), svc.error());
    std::printf("pack: levels %d..%d, L10 tiles in pack %llu\n", svc.coarsestLevel(),
                svc.finestLevel(),
                static_cast<unsigned long long>(svc.pack().tileCount(svc.finestLevel())));

    // ---- cold load ----------------------------------------------------------------------
    const double t0 = nowS();
    svc.prefetch(kPdxLat, kPdxLon, 50000);
    svc.update(kPdxLat, kPdxLon);
    const bool idle = svc.waitForIdle(300);
    const double loadS = nowS() - t0;
    const TerrainStats s = svc.stats();
    std::vector<std::shared_ptr<const TerrainTile>> tiles;
    svc.residentTiles(tiles);
    size_t perLevel[kMaxLevel + 1] = {};
    for (const auto& t : tiles) ++perLevel[t->id.k];
    std::printf("cold load 50 km @ KPDX: %.2f s (idle=%d), %zu tiles, %.1f MB cache, RSS +%.1f MB, "
                "worker %.0f ms total (%.2f ms/tile avg, %.1f max)\n",
                loadS, idle, s.residentTiles, s.residentBytes / 1048576.0, rssMb() - rss0,
                s.loadMsTotal, s.loadMsTotal / std::max<uint64_t>(1, s.tilesLoaded), s.loadMsMax);
    for (int k = 0; k <= kMaxLevel; ++k)
        if (perLevel[k]) std::printf("  L%d: %zu tiles\n", k, perLevel[k]);
    CHECK(idle);
    CHECK(svc.isDetailReady(kPdxLat, kPdxLon));
    CHECK(s.residentBytes <= s.cacheMaxBytes);

    // ---- spot elevations ----------------------------------------------------------------
    const float pdx = svc.elevationM(kPdxLat, kPdxLon);
    std::printf("KPDX field: %.1f m = %.0f ft (published 31 ft), answering level L%d\n", pdx,
                mToFt(pdx), svc.answeringLevel(kPdxLat, kPdxLon));
    svc.prefetch(kHoodLat, kHoodLon, 6000);
    svc.prefetch(kStHelensLat, kStHelensLon, 6000);
    svc.update(kPdxLat, kPdxLon);
    REQUIRE(svc.waitForIdle(60));
    // Highest point within 600 m of the nominal summit (the published point is only approximate).
    float hood = -1e9f;
    double hoodLat = 0, hoodLon = 0;
    for (double dy = -600; dy <= 600; dy += 19.0)
        for (double dx = -600; dx <= 600; dx += 19.0) {
            const double lat = kHoodLat + dy / (kEarthRadiusM * kRadPerDeg);
            const double lon =
                kHoodLon + dx / (kEarthRadiusM * kRadPerDeg * std::cos(kHoodLat * kRadPerDeg));
            const float e = svc.elevationM(lat, lon);
            if (e > hood) {
                hood = e;
                hoodLat = lat;
                hoodLon = lon;
            }
        }
    std::printf("Mt Hood: nominal point %.0f m; max within 600 m = %.0f m = %.0f ft at "
                "%.5f %.5f (published 11,249 ft = 3429 m), L%d\n",
                svc.elevationM(kHoodLat, kHoodLon), hood, mToFt(hood), hoodLat, hoodLon,
                svc.answeringLevel(hoodLat, hoodLon));
    float helens = -1e9f;
    for (double dy = -1500; dy <= 1500; dy += 38.0)
        for (double dx = -1500; dx <= 1500; dx += 38.0) {
            const double lat = kStHelensLat + dy / (kEarthRadiusM * kRadPerDeg);
            const double lon = kStHelensLon + dx / (kEarthRadiusM * kRadPerDeg *
                                                    std::cos(kStHelensLat * kRadPerDeg));
            helens = std::max(helens, svc.elevationM(lat, lon));
        }
    std::printf("Mt St Helens rim max: %.0f m = %.0f ft (published 8,363 ft)\n", helens,
                mToFt(helens));
    CHECK(hood > 3300.0f);
    CHECK(pdx > 0.0f);
    CHECK(pdx < 30.0f);

    // ---- query throughput ---------------------------------------------------------------
    std::mt19937_64 rng(42);
    std::uniform_real_distribution<double> u(-1.0, 1.0);
    const int n = 2000000;
    std::vector<double> lats(n), lons(n);
    for (int i = 0; i < n; ++i) {  // random points within ~15 km of KPDX (all resident at L10)
        lats[i] = kPdxLat + 0.13 * u(rng);
        lons[i] = kPdxLon + 0.19 * u(rng);
    }
    volatile float sink = 0;
    double a = nowS();
    float acc = 0;
    for (int i = 0; i < n; ++i) acc += svc.elevationM(lats[i], lons[i]);
    double b = nowS();
    sink = acc;
    std::printf("elevationM random points in 15 km: %.1f M queries/s (%.0f ns each)\n",
                n / (b - a) / 1e6, (b - a) / n * 1e9);
    // A flight path: consecutive samples 1 m apart (ground contact at 120 Hz is far gentler).
    a = nowS();
    acc = 0;
    for (int i = 0; i < n; ++i)
        acc += svc.elevationM(kPdxLat + i * 1e-5 * 0.5, kPdxLon + i * 1e-5 * 0.3);
    b = nowS();
    sink = acc;
    std::printf("elevationM along a track:          %.1f M queries/s (%.0f ns each)\n",
                n / (b - a) / 1e6, (b - a) / n * 1e9);
    // Land class
    a = nowS();
    int lcAcc = 0;
    for (int i = 0; i < n; ++i) lcAcc += static_cast<int>(svc.landClass(lats[i], lons[i]));
    b = nowS();
    sink = static_cast<float>(lcAcc);
    std::printf("landClass random points:           %.1f M queries/s\n", n / (b - a) / 1e6);
    // Out of coverage (ocean / unbuilt): falls through every level.
    a = nowS();
    acc = 0;
    for (int i = 0; i < n; ++i) acc += svc.elevationM(20.0 + 0.001 * (i % 1000), -150.0);
    b = nowS();
    sink = acc;
    std::printf("elevationM outside the pack:       %.1f M queries/s\n", n / (b - a) / 1e6);
    (void)sink;
    // update() cost when nothing changes
    a = nowS();
    for (int i = 0; i < 100000; ++i) svc.update(kPdxLat, kPdxLon);
    b = nowS();
    std::printf("update() idle: %.0f ns per call\n", (b - a) / 100000 * 1e9);
}

TEST_CASE("terrain-bench: fly-through under the default cache cap" * doctest::skip()) {
    TerrainService svc(packDir());
    REQUIRE_MESSAGE(svc.isOpen(), svc.error());
    // KPDX -> KOTH-ish heading (south-west to the coast) at 250 kt for 190 NM, 10 Hz updates,
    // no waiting: results are integrated as they arrive.
    const double lat0 = kPdxLat, lon0 = kPdxLon;
    const double lat1 = 43.4171, lon1 = -124.2460;
    const double distM = 190.0 * 1852.0;
    const double speedMps = 250.0 * 0.514444;
    const double dt = 0.1;
    size_t peak = 0;
    double worstUpdateMs = 0;
    int notReady = 0, frames = 0;
    for (double d = 0; d < distM; d += speedMps * dt) {
        const double f = d / distM;
        const double lat = lat0 + (lat1 - lat0) * f, lon = lon0 + (lon1 - lon0) * f;
        const double a = nowS();
        svc.update(lat, lon);
        worstUpdateMs = std::max(worstUpdateMs, (nowS() - a) * 1000.0);
        peak = std::max(peak, svc.stats().residentBytes);
        if (!svc.isDetailReady(lat, lon)) ++notReady;
        ++frames;
        std::this_thread::sleep_for(std::chrono::microseconds(2000));  // do not outrun the disk
    }
    const TerrainStats s = svc.stats();
    std::printf("fly-through: %d updates, peak cache %.1f MB (cap %.0f), loaded %llu, evicted "
                "%llu, frames without finest detail %d, worst update() %.2f ms\n",
                frames, peak / 1048576.0, s.cacheMaxBytes / 1048576.0,
                static_cast<unsigned long long>(s.tilesLoaded),
                static_cast<unsigned long long>(s.tilesEvicted), notReady, worstUpdateMs);
    CHECK(peak <= s.cacheMaxBytes);
}
