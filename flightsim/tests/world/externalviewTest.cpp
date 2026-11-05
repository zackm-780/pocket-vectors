// V1 external view without GL: model choice per profile, the built mesh (size, wheels on the
// ground, gear and flaps, winding) and the chase camera pose.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <vector>

#include "core/Geo.h"
#include "core/Units.h"
#include "world/ExternalViewModel.h"

using namespace fsim;
using namespace fsim::world;

namespace {

AircraftProfile profileOf(AircraftCategory cat, int engines, const char* model = "") {
    AircraftProfile p;
    p.category = cat;
    p.engineCount = engines;
    p.externalModel = model;
    return p;
}

struct Mesh {
    std::vector<float> v;
    ExternalModelInfo info;
};
Mesh build(ExternalModelKind kind, float flap, float gear, float gearH = 0, float span = 0,
           int engines = 2) {
    ExternalModelParams p;
    p.kind = kind;
    p.flapNorm = flap;
    p.gearNorm = gear;
    p.gearHeightM = gearH;
    p.spanM = span;
    p.engineCount = engines;
    Mesh m;
    m.info = buildExternalModel(p, m.v);
    return m;
}

const ExternalModelKind kAllKinds[] = {ExternalModelKind::Single, ExternalModelKind::Twin,
                                       ExternalModelKind::Bizjet, ExternalModelKind::Airliner,
                                       ExternalModelKind::Heavy};

}  // namespace

TEST_CASE("externalview: model per profile category, engine count and [view] external_model") {
    CHECK(externalModelFor(profileOf(AircraftCategory::Piston, 1)) == ExternalModelKind::Single);
    CHECK(externalModelFor(profileOf(AircraftCategory::Piston, 2)) == ExternalModelKind::Twin);
    CHECK(externalModelFor(profileOf(AircraftCategory::Turboprop, 2)) == ExternalModelKind::Twin);
    CHECK(externalModelFor(profileOf(AircraftCategory::Jet, 2)) == ExternalModelKind::Airliner);
    CHECK(externalModelFor(profileOf(AircraftCategory::Jet, 3)) == ExternalModelKind::Heavy);
    CHECK(externalModelFor(profileOf(AircraftCategory::Jet, 4)) == ExternalModelKind::Heavy);
    CHECK(externalModelFor(profileOf(AircraftCategory::Jet, 2, "bizjet")) ==
          ExternalModelKind::Bizjet);
    CHECK(externalModelFor(profileOf(AircraftCategory::Jet, 2, "heavy")) ==
          ExternalModelKind::Heavy);
    CHECK(externalModelFor(profileOf(AircraftCategory::Jet, 2, "zeppelin")) ==
          ExternalModelKind::Airliner);  // unknown name: the category rule
    for (ExternalModelKind k : kAllKinds) {
        ExternalModelKind back;
        REQUIRE(parseExternalModel(externalModelName(k), back));
        CHECK(back == k);
    }
}

TEST_CASE("externalview: every model is one small mesh with unit normals and outward winding") {
    for (ExternalModelKind k : kAllKinds) {
        const Mesh m = build(k, 1.0f, 1.0f);
        INFO(externalModelName(k));
        REQUIRE(m.v.size() % (3 * kExternalVertexFloats) == 0);
        const int verts = static_cast<int>(m.v.size() / kExternalVertexFloats);
        CHECK(m.info.triangles * 3 == verts);
        CHECK(m.info.triangles > 150);
        CHECK(m.info.triangles < 1500);  // one draw call, a few hundred triangles
        int inward = 0;
        for (size_t t = 0; t < m.v.size(); t += 3 * kExternalVertexFloats) {
            const float* a = &m.v[t];
            const float* b = a + kExternalVertexFloats;
            const float* c = b + kExternalVertexFloats;
            const float e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]};
            const float e2[3] = {c[0] - a[0], c[1] - a[1], c[2] - a[2]};
            const float cr[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2],
                                 e1[0] * e2[1] - e1[1] * e2[0]};
            const float nl = std::sqrt(a[3] * a[3] + a[4] * a[4] + a[5] * a[5]);
            CHECK(nl == doctest::Approx(1.0f).epsilon(1e-3));
            // Counter-clockwise around the outward normal (back faces are culled).
            if (cr[0] * a[3] + cr[1] * a[4] + cr[2] * a[5] < -1e-7f) ++inward;
        }
        CHECK(inward == 0);
    }
}

TEST_CASE("externalview: the model scales to the wingspan and its wheels touch the ground") {
    const Mesh m737 = build(ExternalModelKind::Airliner, 0, 1, 1.13f, 28.86f);
    CHECK(m737.info.spanM == doctest::Approx(28.86f));
    CHECK(m737.info.lengthM > 25.0f);
    CHECK(m737.info.lengthM < 40.0f);
    CHECK(m737.info.maxZM == doctest::Approx(1.13f).epsilon(0.01));  // lowest point = wheels
    const Mesh md11 = build(ExternalModelKind::Heavy, 0, 1, 5.73f, 51.7f, 3);
    CHECK(md11.info.maxZM == doctest::Approx(5.73f).epsilon(0.01));
    // Gear up: no gear, nothing reaches the ground (the nacelles hang lowest, clear of it).
    const Mesh up = build(ExternalModelKind::Airliner, 0, 0, 1.13f, 28.86f);
    CHECK(up.info.triangles < m737.info.triangles);
    CHECK(up.info.maxZM < 1.13f - 0.25f);
    // Half way: the legs are drawn, part extended.
    const Mesh half = build(ExternalModelKind::Airliner, 0, 0.5f, 1.13f, 28.86f);
    CHECK(half.info.triangles == m737.info.triangles);
    CHECK(half.info.maxZM < m737.info.maxZM - 0.25f);
}

TEST_CASE("externalview: flaps out add the deflected panels; engine count shows") {
    const Mesh clean = build(ExternalModelKind::Airliner, 0, 1);
    const Mesh flaps = build(ExternalModelKind::Airliner, 0.75f, 1);
    CHECK(flaps.info.triangles == clean.info.triangles + 24);  // two flap slabs
    const Mesh two = build(ExternalModelKind::Heavy, 0, 1, 0, 0, 2);
    const Mesh three = build(ExternalModelKind::Heavy, 0, 1, 0, 0, 3);
    const Mesh four = build(ExternalModelKind::Heavy, 0, 1, 0, 0, 4);
    CHECK(three.info.triangles > two.info.triangles);
    CHECK(four.info.triangles > three.info.triangles);
}

TEST_CASE("externalview: chase camera sits behind and above, looks at the CG, level horizon") {
    AircraftState s;
    s.valid = true;
    s.latDeg = 40.0;
    s.lonDeg = -100.0;
    s.altMslFt = 3000;
    s.headingTrueRad = static_cast<float>(degToRad(270.0));
    const float dist = chaseDistanceM(35.0f);
    const ChasePose p = chasePose(s, 270.0f, 0.0f, 0.0f, dist);
    GeoPos cg;
    cg.latDeg = s.latDeg;
    cg.lonDeg = s.lonDeg;
    cg.altMslM = ftToM(s.altMslFt);
    const Ecef c = geodeticToEcef(cg);
    const Vec3d d{c.x - p.eyeEcef.x, c.y - p.eyeEcef.y, c.z - p.eyeEcef.z};
    const double len = std::sqrt(d.x * d.x + d.y * d.y + d.z * d.z);
    CHECK(len == doctest::Approx(dist).epsilon(1e-4));
    CHECK(p.eyeGeo.lonDeg > s.lonDeg);  // heading west: the camera is east, behind the tail
    CHECK(p.eyeGeo.altMslM - cg.altMslM ==
          doctest::Approx(dist * std::sin(degToRad(kChaseBaseElevDeg))).epsilon(1e-3));
    // Forward (-row 2) points at the CG; the right axis (row 0) is horizontal.
    const Vec3d fwd{-p.ecefToEye.m[2][0], -p.ecefToEye.m[2][1], -p.ecefToEye.m[2][2]};
    CHECK((fwd.x * d.x + fwd.y * d.y + fwd.z * d.z) / len == doctest::Approx(1.0).epsilon(1e-6));
    const Vec3d up = nedToEcef(p.eyeGeo.latDeg, p.eyeGeo.lonDeg) * Vec3d{0, 0, -1};
    CHECK(std::fabs(p.ecefToEye.m[0][0] * up.x + p.ecefToEye.m[0][1] * up.y +
                    p.ecefToEye.m[0][2] * up.z) < 1e-6);
    // Orbit yaw 180 looks back at the nose from in front.
    const ChasePose front = chasePose(s, 270.0f, 180.0f, 0.0f, dist);
    CHECK(front.eyeGeo.lonDeg < s.lonDeg);
    // Orbit pitch up lowers the camera under the aircraft.
    const ChasePose low = chasePose(s, 270.0f, 0.0f, 30.0f, dist);
    CHECK(low.eyeGeo.altMslM < cg.altMslM);
}

TEST_CASE("externalview: heading follow eases through north; distance grows with the span") {
    float h = followHeadingDeg(350.0f, 10.0f, 0.6f, 0.1);
    CHECK(h > 350.0f);  // moving clockwise toward 010, not back through 180
    for (int i = 0; i < 200; ++i) h = followHeadingDeg(h, 10.0f, 0.6f, 0.1);
    CHECK(h == doctest::Approx(10.0f).epsilon(1e-3));
    CHECK(chaseDistanceM(11.0f) < chaseDistanceM(35.0f));
    CHECK(chaseDistanceM(35.0f) < chaseDistanceM(60.0f));
    CHECK(chaseDistanceM(5.0f) >= 14.0f);
}

TEST_CASE("externalview: building a model costs well under a millisecond") {
    std::vector<float> v;
    ExternalModelParams p;
    p.kind = ExternalModelKind::Heavy;
    p.engineCount = 4;
    p.flapNorm = 0.5f;
    const auto t0 = std::chrono::steady_clock::now();
    constexpr int kRuns = 50;
    for (int i = 0; i < kRuns; ++i) buildExternalModel(p, v);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
        kRuns;
    std::printf("[V1] heavy model build %.3f ms, %zu vertices\n", ms,
                v.size() / kExternalVertexFloats);
    CHECK(ms < 1.0);
}
