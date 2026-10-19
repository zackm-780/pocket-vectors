#include "app/SpawnGate.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>

#include "core/Geo.h"
#include "core/Units.h"

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

FlattenArea runwayFlattenArea(const Runway& rwy, float shoulderM, float endMarginM,
                              float blendM) {
    // Along-runway bearing from the pavement start to its far end (the stored heading is rounded;
    // the end points are authoritative).
    double bearingRad = degToRad(rwy.headingTrueDeg);
    if (rwy.startLatDeg != rwy.endLatDeg || rwy.startLonDeg != rwy.endLonDeg)
        bearingRad = initialBearingTrueRad(rwy.startLatDeg, rwy.startLonDeg, rwy.endLatDeg,
                                           rwy.endLonDeg);
    const double halfWidthM = 0.5 * ftToM(rwy.widthFt) + shoulderM;

    double baseLat = 0, baseLon = 0, tipLat = 0, tipLon = 0;
    destinationPoint(rwy.startLatDeg, rwy.startLonDeg, bearingRad + kPi, endMarginM, baseLat,
                     baseLon);
    destinationPoint(rwy.endLatDeg, rwy.endLonDeg, bearingRad, endMarginM, tipLat, tipLon);

    FlattenArea a;
    // Counter-clockwise seen from above: along the right side, back along the left side (the
    // interior is on the left of every edge).
    const double right = bearingRad + 0.5 * kPi, left = bearingRad - 0.5 * kPi;
    destinationPoint(baseLat, baseLon, right, halfWidthM, a.latDeg[0], a.lonDeg[0]);
    destinationPoint(tipLat, tipLon, right, halfWidthM, a.latDeg[1], a.lonDeg[1]);
    destinationPoint(tipLat, tipLon, left, halfWidthM, a.latDeg[2], a.lonDeg[2]);
    destinationPoint(baseLat, baseLon, left, halfWidthM, a.latDeg[3], a.lonDeg[3]);
    a.elevM = static_cast<float>(ftToM(0.5 * (rwy.thrElevFt + rwy.endElevFt)));
    a.blendM = blendM;
    return a;
}

}  // namespace fsim
