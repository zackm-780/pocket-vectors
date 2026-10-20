// Flatten areas: same semantics as core/FlatTerrain, blend by true distance to the quad.
#include <doctest/doctest.h>

#include <cmath>

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
