#pragma once
// Spawn gating and start-runway flattening (WP F3). Used by Simulation::init so an aircraft never
// appears over terrain that is not loaded yet (it would fall to sea level, or be buried when the
// detail tiles arrive), and so a runway start sits on the runway surface.

#include <functional>
#include <vector>

#include "core/NavTypes.h"
#include "core/TerrainQuery.h"

namespace fsim {

struct SpawnPoint {
    double latDeg = 0, lonDeg = 0;
};

struct SpawnGateResult {
    bool ready = true;      // every point had its finest detail resident when the wait ended
    bool timedOut = false;  // gave up waiting (the caller starts anyway and logs it)
    double waitedS = 0;
    int polls = 0;
};

// Called while waiting so finished tiles get integrated: for TerrainService this is
// TerrainService::update(lat, lon). Main thread only (TerrainService contract).
using TerrainPump = std::function<void(double latDeg, double lonDeg)>;

// Asks the terrain to prefetch around every point, then polls isDetailReady (pumping in between)
// until all are ready or timeoutS passes. Returns immediately when the terrain is already ready
// (FlatTerrain, no pack). Without a pump nothing can change, so it does not wait.
SpawnGateResult waitForSpawnDetail(TerrainQuery& terrain, const std::vector<SpawnPoint>& points,
                                   float prefetchRadiusM, double timeoutS, const TerrainPump& pump);

// Temporary until W5 (airports) registers runway flatten areas for every airport: the quad around
// a runway's pavement (plus shoulders and end margins) forced to the runway elevation (mean of the
// threshold and far-end elevations), blending back to natural terrain over blendM.
FlattenArea runwayFlattenArea(const Runway& runway, float shoulderM = 30.0f,
                              float endMarginM = 60.0f, float blendM = 100.0f);

}  // namespace fsim
