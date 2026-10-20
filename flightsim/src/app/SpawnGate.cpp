#include "app/SpawnGate.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

namespace fsim {
namespace {

constexpr auto kPollInterval = std::chrono::milliseconds(2);

bool allReady(const TerrainQuery& terrain, const std::vector<SpawnPoint>& points) {
    return std::all_of(points.begin(), points.end(), [&](const SpawnPoint& p) {
        return terrain.isDetailReady(p.latDeg, p.lonDeg);
    });
}

}  // namespace

SpawnGateResult waitForSpawnDetail(TerrainQuery& terrain, const std::vector<SpawnPoint>& points,
                                   float prefetchRadiusM, double timeoutS,
                                   const TerrainPump& pump) {
    SpawnGateResult r;
    if (points.empty()) return r;
    for (const SpawnPoint& p : points) terrain.prefetch(p.latDeg, p.lonDeg, prefetchRadiusM);
    if (pump) pump(points[0].latDeg, points[0].lonDeg);
    r.ready = allReady(terrain, points);
    if (r.ready || !pump) return r;

    const auto t0 = std::chrono::steady_clock::now();
    for (;;) {
        std::this_thread::sleep_for(kPollInterval);
        pump(points[0].latDeg, points[0].lonDeg);
        ++r.polls;
        r.waitedS = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
        if (allReady(terrain, points)) {
            r.ready = true;
            return r;
        }
        if (r.waitedS >= timeoutS) {
            r.ready = false;
            r.timedOut = true;
            return r;
        }
    }
}

}  // namespace fsim
