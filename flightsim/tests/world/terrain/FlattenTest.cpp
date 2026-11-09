// Flatten areas: same semantics as core/FlatTerrain, blend by true distance to the quad.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/FlatTerrain.h"
#include "core/Units.h"
#include "world/terrain/FlattenSet.h"
#include "world/terrain/SyntheticPack.h"
#include "world/terrain/TerrainService.h"

using namespace fsim;
using namespace fsim::terrain;
using namespace fsim::terrain::testing;

namespace {

constexpr double kLat0 = 46.0, kLon0 = -122.0;
constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;
const double kMPerDegLon = kMPerDegLat * std::cos(kLat0 * kRadPerDeg);

// 1000 m (east-west) x 500 m (north-south) rectangle centred on kLat0/kLon0, CCW: SW, SE, NE, NW.
FlattenArea rect(float elevM, float blendM, bool clockwise = false) {
    const double dLat = 250.0 / kMPerDegLat, dLon = 500.0 / kMPerDegLon;
    double lat[4] = {kLat0 - dLat, kLat0 - dLat, kLat0 + dLat, kLat0 + dLat};
    double lon[4] = {kLon0 - dLon, kLon0 + dLon, kLon0 + dLon, kLon0 - dLon};
    FlattenArea a;
    for (int i = 0; i < 4; ++i) {
        const int j = clockwise ? (4 - i) % 4 : i;
        a.latDeg[i] = lat[j];
        a.lonDeg[i] = lon[j];
    }
    a.elevM = elevM;
    a.blendM = blendM;
    return a;
}

double smoothstep(double t) {
    return t * t * (3.0 - 2.0 * t);
}

// Point `eastM` east and `northM` north of the area centre.
void at(double eastM, double northM, double& lat, double& lon) {
    lat = kLat0 + northM / kMPerDegLat;
    lon = kLon0 + eastM / kMPerDegLon;
}

}  // namespace

TEST_CASE("terrain: flatten set forces the elevation inside and blends outside") {
    FlattenSet set;
    set.add(rect(250.0f, 100.0f));
    int calls = 0;
    auto natural = [&] {
        ++calls;
        return 100.0f;
    };
    double lat, lon;

    at(0, 0, lat, lon);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(250.0f));
    at(499, 249, lat, lon);  // just inside the NE corner
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(250.0f));
    CHECK(calls == 0);  // natural terrain is not even looked up inside

    // Perpendicular to an edge the blend is smoothstep(d / blend).
    for (const double d : {0.0001, 10.0, 25.0, 50.0, 75.0, 99.0}) {
        at(500.0 + d, 0, lat, lon);  // east of the east edge
        const double want = 250.0 + (100.0 - 250.0) * smoothstep(d / 100.0);
        CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(want).epsilon(0.002));
        at(0, 250.0 + d, lat, lon);  // north of the north edge
        CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(want).epsilon(0.002));
    }
    CHECK(calls == 12);  // once per blended query

    at(500.0 + 100.5, 0, lat, lon);  // past the blend distance: natural terrain
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(100.0f));
    at(5000, 5000, lat, lon);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(100.0f));
}

TEST_CASE("terrain: the blend uses the distance to the quad, so corners are rounded") {
    FlattenSet set;
    set.add(rect(250.0f, 100.0f));
    auto natural = [] { return 100.0f; };
    double lat, lon;
    at(500.0 + 60.0, 250.0 + 60.0, lat, lon);  // diagonally off the NE corner
    const double dTrue = std::hypot(60.0, 60.0);
    const double want = 250.0 + (100.0 - 250.0) * smoothstep(dTrue / 100.0);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(want).epsilon(0.002));
    CHECK(set.distanceM(0, lat, lon) == doctest::Approx(dTrue).epsilon(0.002));
    // Beyond blendM from the corner the terrain is natural, even though the distance to each
    // edge LINE is only 80 m (FlatTerrain still blends there).
    at(500.0 + 80.0, 250.0 + 80.0, lat, lon);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(100.0f));
    FlatTerrain flat(100.0f);
    flat.addFlattenArea(rect(250.0f, 100.0f));
    CHECK(flat.elevationM(lat, lon) > 101.0f);
}

TEST_CASE("terrain: flatten matches FlatTerrain where the distance to an edge line is exact") {
    FlattenSet set;
    FlatTerrain flat(100.0f);
    set.add(rect(250.0f, 100.0f));
    flat.addFlattenArea(rect(250.0f, 100.0f));
    auto natural = [] { return 100.0f; };
    for (const double d : {-200.0, -1.0, 0.5, 20.0, 40.0, 60.0, 90.0, 150.0}) {
        double lat, lon;
        at(500.0 + d, 30.0, lat, lon);  // east of the east edge (negative d = inside)
        CHECK(set.elevationM(lat, lon, natural) ==
              doctest::Approx(flat.elevationM(lat, lon)).epsilon(0.002));
        at(-100.0, -250.0 - d, lat, lon);  // south of the south edge
        CHECK(set.elevationM(lat, lon, natural) ==
              doctest::Approx(flat.elevationM(lat, lon)).epsilon(0.002));
    }
}

TEST_CASE("terrain: flatten accepts clockwise quads and a zero blend") {
    FlattenSet ccw, cw, hard;
    ccw.add(rect(250.0f, 100.0f));
    cw.add(rect(250.0f, 100.0f, true));
    hard.add(rect(250.0f, 0.0f));
    auto natural = [] { return 100.0f; };
    double lat, lon;
    at(0, 0, lat, lon);
    CHECK(cw.elevationM(lat, lon, natural) == doctest::Approx(250.0f));
    at(520, 0, lat, lon);
    CHECK(cw.elevationM(lat, lon, natural) == doctest::Approx(ccw.elevationM(lat, lon, natural)));
    CHECK(hard.elevationM(lat, lon, natural) == doctest::Approx(100.0f));  // no blend
    at(499, 0, lat, lon);
    CHECK(hard.elevationM(lat, lon, natural) == doctest::Approx(250.0f));
}

TEST_CASE("terrain: several flatten areas apply in order, inside wins") {
    FlattenSet set;
    set.add(rect(250.0f, 100.0f));
    FlattenArea other = rect(400.0f, 50.0f);
    for (int i = 0; i < 4; ++i) other.lonDeg[i] += 3000.0 / kMPerDegLon;  // 3 km east
    set.add(other);
    auto natural = [] { return 100.0f; };
    double lat, lon;
    at(3000, 0, lat, lon);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(400.0f));
    at(-500.0 - 50.0, 0, lat, lon);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(175.0f).epsilon(0.002));
    CHECK(set.size() == 2);
    const unsigned rev = set.revision();
    set.clear();
    CHECK(set.empty());
    CHECK(set.revision() != rev);
    CHECK(set.elevationM(lat, lon, natural) == doctest::Approx(100.0f));
}

TEST_CASE("terrain: TerrainService applies flatten areas over the natural surface") {
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    auto slope = [](double lat, double) { return 100.0 + 2000.0 * (lat - 46.0); };  // 22 m/km
    const TileId t = tileAt(7, kLat0, kLon0);
    w.writeTile(t, slope);
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(svc.isOpen());
    svc.update(kLat0, kLon0);
    REQUIRE(svc.waitForIdle(20));

    double lat, lon;
    at(0, 400, lat, lon);
    const float natural = svc.elevationM(lat, lon);
    CHECK(std::fabs(natural - (100.0 + 2000.0 * (lat - 46.0))) < 0.6);  // int16 rounding

    TerrainQuery& q = svc;
    q.addFlattenArea(rect(250.0f, 100.0f));
    at(0, 0, lat, lon);
    CHECK(q.elevationM(lat, lon) == doctest::Approx(250.0f));
    CHECK(svc.naturalElevationM(lat, lon) != doctest::Approx(250.0f));  // unflattened is available
    at(0, 250.0 + 50.0, lat, lon);                                      // halfway through the blend
    const float nat = svc.naturalElevationM(lat, lon);
    CHECK(q.elevationM(lat, lon) == doctest::Approx(250.0f + (nat - 250.0f) * 0.5f).epsilon(0.002));
    at(0, 400, lat, lon);
    CHECK(q.elevationM(lat, lon) == doctest::Approx(natural));
    CHECK(svc.landClass(lat, lon) == LandClass::Unknown);  // no .lc8 in this pack: not Water
    q.clearFlattenAreas();
    at(0, 0, lat, lon);
    CHECK(q.elevationM(lat, lon) == doctest::Approx(svc.naturalElevationM(lat, lon)));
}

// ---- W1b (1.3): graded areas and crossing runways ---------------------------------------------

namespace {

// A runway-like graded area: centerline from (e0, n0) to (e1, n1) in metres around kLat0/kLon0,
// elevations z0 -> z1, pavement half width hw plus a 50 m margin, 100 m beyond the ends.
FlattenArea gradedRunway(double e0, double n0, double e1, double n1, float z0, float z1,
                         double hw = 22.5, float blendM = 150.0f) {
    const double len = std::hypot(e1 - e0, n1 - n0);
    const double ue = (e1 - e0) / len, un = (n1 - n0) / len;  // along
    const double re = un, rn = -ue;                           // right of along
    const double a0 = -100.0, a1 = len + 100.0, l = hw + 50.0;
    const double along[4] = {a0, a0, a1, a1};
    const double lateral[4] = {-l, l, l, -l};  // clockwise; FlattenSet accepts either
    FlattenArea a;
    for (int i = 0; i < 4; ++i)
        at(e0 + ue * along[i] + re * lateral[i], n0 + un * along[i] + rn * lateral[i], a.latDeg[i],
           a.lonDeg[i]);
    a.graded = true;
    at(e0, n0, a.gradeLatDeg[0], a.gradeLonDeg[0]);
    at(e1, n1, a.gradeLatDeg[1], a.gradeLonDeg[1]);
    a.gradeElevM[0] = z0;
    a.gradeElevM[1] = z1;
    a.elevM = 0.5f * (z0 + z1);
    a.blendM = blendM;
    return a;
}

double surfaceAt(const FlattenSet& set, double e, double n, float natural = 95.0f) {
    double lat, lon;
    at(e, n, lat, lon);
    return set.elevationM(lat, lon, [natural] { return natural; });
}

}  // namespace

TEST_CASE("terrain: a graded area follows its grade, clamped beyond the ends") {
    FlattenSet set;
    set.add(gradedRunway(-1500, 0, 1500, 0, 100.0f, 103.0f));
    for (double e = -1500; e <= 1500; e += 100) {
        CAPTURE(e);
        CHECK(surfaceAt(set, e, 0) ==
              doctest::Approx(100.0 + 3.0 * (e + 1500) / 3000).epsilon(1e-5));
        CHECK(surfaceAt(set, e, 60) ==
              doctest::Approx(surfaceAt(set, e, 0)).epsilon(1e-5));  // margin
    }
    CHECK(surfaceAt(set, -1580, 0) == doctest::Approx(100.0f));  // the end pad keeps the end value
    CHECK(surfaceAt(set, 1580, 0) == doctest::Approx(103.0f));
    // The blend goes from the grade to the natural terrain.
    const double d = 75.0;  // half way through the 150 m blend beside the middle
    CHECK(surfaceAt(set, 0, 72.5 + d) ==
          doctest::Approx(101.5 + (95.0 - 101.5) * smoothstep(0.5)).epsilon(1e-4));
    // FlatTerrain follows the grade too.
    FlatTerrain flat(95.0f);
    flat.addFlattenArea(gradedRunway(-1500, 0, 1500, 0, 100.0f, 103.0f));
    double lat, lon;
    at(750, 0, lat, lon);
    CHECK(flat.elevationM(lat, lon) == doctest::Approx(102.25f));
}

// Owner, 2026-11-08 (KPDX 28L across 03/21): "when I hit an intersecting runway I bounced pretty
// dramatically. Runway transitions should always be smooth."
TEST_CASE("terrain: crossing runways at different elevations meet without a step (<= 1:100)") {
    for (const float lowerM : {0.6f, 3.0f}) {  // KPDX-like, and a large difference
        CAPTURE(lowerM);
        FlattenSet set;
        // B (shorter, crossing at 60 degrees) first: insertion order must not matter.
        const double c = std::cos(degToRad(60.0)), s = std::sin(degToRad(60.0));
        set.add(
            gradedRunway(-900 * c, -900 * s, 900 * c, 900 * s, 100.0f - lowerM, 101.0f - lowerM));
        set.add(gradedRunway(-1700, 0, 1700, 0, 100.0f, 100.4f));  // A: longer, wins
        // A's centerline is exactly A's grade, across the crossing too.
        double worstA = 0;
        for (double e = -1700; e <= 1700; e += 5)
            worstA = std::max(worstA,
                              std::fabs(surfaceAt(set, e, 0) - (100.0 + 0.4 * (e + 1700) / 3400)));
        CHECK(worstA < 0.001);
        // Along B's centerline (and 15 m either side) there is no step and no slope over 1:100.
        double worstSlope = 0, worstJump = 0;
        for (const double off : {-15.0, 0.0, 15.0}) {
            double prev = 0;
            for (double t = -1000; t <= 1000; t += 1) {
                const double z = surfaceAt(set, t * c + off * s, t * s - off * c);
                if (t > -1000) {
                    worstSlope = std::max(worstSlope, std::fabs(z - prev));  // per metre
                    worstJump = std::max(worstJump, std::fabs(z - prev));
                }
                prev = z;
            }
        }
        std::printf("[W1b] crossing %.1f m lower: worst slope along B 1:%.0f\n", lowerM,
                    1.0 / worstSlope);
        CHECK(worstSlope <= 0.01);
        // The whole field (both pavements and the area between): a 2 m grid, no gradient steeper
        // than 1:100 on any pavement.
        double worstGrid = 0;
        for (double n = -800; n <= 800; n += 2) {
            for (double e = -1700; e <= 1700; e += 2) {
                const double tB = e * c + n * s, lB = e * s - n * c;  // along and across B
                const bool onA = std::fabs(n) <= 22.5;
                const bool onB = std::fabs(lB) <= 22.5 && std::fabs(tB) <= 900;
                if (!onA && !onB) continue;
                const double z = surfaceAt(set, e, n);
                const double gx = (surfaceAt(set, e + 1, n) - z),
                             gy = (surfaceAt(set, e, n + 1) - z);
                worstGrid = std::max(worstGrid, std::hypot(gx, gy));
            }
        }
        CHECK(worstGrid <= 0.01 + 1e-4);  // grade + transition
    }
}

TEST_CASE("terrain: W1b synthetic pack: a 4 m DEM step under a runway end, graded and blended") {
    // DEM: 100 m, and 104 m east of the last 50 m of a 2,000 m east-west runway. The runway is
    // graded from 100.0 m (west threshold) to 100.5 m (east end), as published.
    TempDir dir;
    PackWriter w(dir.path, 7, 7);
    const double endE = 1000.0;
    auto dem = [&](double, double lon) {
        const double e = (lon - kLon0) * kMPerDegLon;
        return e > endE - 50.0 ? 104.0 : 100.0;
    };
    const TileId t = tileAt(7, kLat0, kLon0);
    w.writeTile(t, dem);
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(svc.isOpen());
    svc.update(kLat0, kLon0);
    REQUIRE(svc.waitForIdle(20));
    svc.addFlattenArea(gradedRunway(-endE, 0, endE, 0, 100.0f, 100.5f));
    double lat, lon;
    // Centerline: within 0.3 m of the graded line over the whole pavement (the DEM step is gone).
    double worstOff = 0;
    for (double e = -endE; e <= endE; e += 10) {
        at(e, 0, lat, lon);
        worstOff = std::max(worstOff, std::fabs(svc.elevationM(lat, lon) -
                                                (100.0 + 0.5 * (e + endE) / (2 * endE))));
    }
    CHECK(worstOff <= 0.3);
    // Beyond the east end (pad, then the blend up to the 104 m DEM) and beside the middle (the
    // blend down to 100 m): no slope steeper than 1:20.
    auto worstSlope = [&](double e0, double n0, double de, double dn, int steps) {
        double worst = 0;
        at(e0, n0, lat, lon);
        double prev = svc.elevationM(lat, lon);
        for (int i = 1; i <= steps; ++i) {
            at(e0 + de * i, n0 + dn * i, lat, lon);
            const double z = svc.elevationM(lat, lon);
            worst = std::max(worst, std::fabs(z - prev) / std::hypot(de, dn));
            prev = z;
        }
        return worst;
    };
    const double endSlope = worstSlope(endE, 0, 5, 0, 80);  // 400 m beyond the end
    const double sideSlope = worstSlope(0, 0, 0, 5, 80);    // 400 m to the north
    std::printf("[W1b] 4 m DEM step: worst off grade %.3f m, end blend 1:%.0f, side 1:%.0f\n",
                worstOff, 1.0 / endSlope, sideSlope > 0 ? 1.0 / sideSlope : 0.0);
    CHECK(endSlope <= 1.0 / 20);
    CHECK(sideSlope <= 1.0 / 20);
}

// W8b: the grid index walks only the areas near a point; the result must equal the full loop.
TEST_CASE("terrain: the grid index gives the same surface as walking every area") {
    FlattenSet set;
    const double c = std::cos(degToRad(60.0)), s = std::sin(degToRad(60.0));
    set.add(gradedRunway(-900 * c, -900 * s, 900 * c, 900 * s, 97.0f, 98.0f));
    set.add(gradedRunway(-1700, 0, 1700, 0, 100.0f, 100.4f));
    set.add(rect(99.0f, 60.0f));
    // A few low-priority (taxiway-like) graded strips across both runways.
    for (int k = 0; k < 40; ++k) {
        const double e0 = -1600 + 80.0 * k, n0 = (k % 2 ? 1 : -1) * (120.0 + 7.0 * k);
        FlattenArea a = gradedRunway(e0, n0, e0 + 60.0, -n0, 99.5f, 100.1f, 11.5, 60.0f);
        a.priority = -1 - (k % 2);
        set.add(a);
    }
    unsigned seed = 12345;
    auto rnd = [&seed](double lo, double hi) {
        seed = seed * 1664525u + 1013904223u;
        return lo + (hi - lo) * static_cast<double>(seed >> 8) / static_cast<double>(1u << 24);
    };
    double worst = 0;
    size_t maxCandidates = 0;
    for (int i = 0; i < 10000; ++i) {
        double lat, lon;
        at(rnd(-2500, 2500), rnd(-2000, 2000), lat, lon);
        const float natural = 95.0f + static_cast<float>(i % 7);
        set.setUseGridForTest(true);
        const float grid = set.elevationM(lat, lon, [natural] { return natural; });
        maxCandidates = std::max(maxCandidates, set.candidateCount(lat, lon));
        set.setUseGridForTest(false);
        const float all = set.elevationM(lat, lon, [natural] { return natural; });
        worst = std::max(worst, static_cast<double>(std::fabs(grid - all)));
    }
    set.setUseGridForTest(true);
    CHECK(worst <= 1e-4);
    CHECK(maxCandidates < set.size());
    // Areas added after a query are picked up (the index rebuilds lazily).
    double lat, lon;
    at(3000, 3000, lat, lon);
    CHECK(set.elevationM(lat, lon, [] { return 50.0f; }) == doctest::Approx(50.0f));
    FlattenArea far = rect(42.0f, 60.0f);
    for (int i = 0; i < 4; ++i) {
        far.latDeg[i] += 3000.0 / kMPerDegLat;
        far.lonDeg[i] += 3000.0 / kMPerDegLon;
    }
    set.add(far);
    CHECK(set.elevationM(lat, lon, [] { return 50.0f; }) == doctest::Approx(42.0f));
    set.clear();
    CHECK(set.elevationM(lat, lon, [] { return 50.0f; }) == doctest::Approx(50.0f));
}

TEST_CASE("terrain: areas below the runways blend at 1:200 and stop at 150 m over pavement") {
    FlattenSet set;
    // A wide strip (core to n = +-110) and a narrow one 3 m higher (core from n = 238.5).
    FlattenArea low = gradedRunway(-500, 0, 500, 0, 100.0f, 100.0f, 60.0, 60.0f);
    low.priority = -1;
    // The narrow one has the longer grade, so it applies after the wide one and blends over it.
    FlattenArea low2 = gradedRunway(-600, 300, 600, 300, 103.0f, 103.0f, 11.5, 60.0f);
    low2.priority = -1;
    set.add(low);
    set.add(low2);
    // 3 m apart, low2's core edge at n = 238.5. A runway would blend over min(600, 200 x 3) =
    // 600 m; these want 300 x 3 = 900 m but stop at 150 m (kMaxLowPaveBlendM).
    CHECK(surfaceAt(set, 0, 0) == doctest::Approx(100.0).epsilon(1e-6));   // 238.5 m away
    CHECK(surfaceAt(set, 0, 50) == doctest::Approx(100.0).epsilon(1e-6));  // 188.5 m: > 150
    const double in = surfaceAt(set, 0, 100);  // on the wide strip, 138.5 m: inside the ring
    const double t = 138.5 / 150.0, sm = t * t * (3.0 - 2.0 * t);
    CHECK(in == doctest::Approx(103.0 + (100.0 - 103.0) * sm).epsilon(1e-5));
    CHECK(FlattenSet::kMaxLowPaveBlendM == 150.0f);
    CHECK(FlattenSet::kLowPaveBlendPerM == 300.0f);  // 1.5 / 300 = 1:200 at the steepest
    CHECK(set.candidateCount(kLat0, kLon0) == 2);
    double lat, lon;
    at(0, -2000, lat, lon);
    CHECK(set.candidateCount(lat, lon) == 0);  // far from both: no list, natural terrain
    CHECK(set.elevationM(lat, lon, [] { return 77.0f; }) == doctest::Approx(77.0f));
}
