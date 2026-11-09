// W8c: taxiway and apron meshes (world/TaxiwayGeometry), the generic extras at OSM airports, the
// draw range and the scenario start position. Synthetic airports only, no GL.
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "app/Scenario.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "world/AirportGeometry.h"
#include "world/SyntheticAirport.h"
#include "world/TaxiwayGeometry.h"
#include "world/terrain/FlattenSet.h"

using namespace fsim;
using namespace fsim::world;
using fsim::testing::AirportBuilder;
using fsim::testing::curve;
using fsim::testing::line;

namespace {

// The airport's composed surface, as AirportRenderer builds it (flatten areas, then the field).
struct Surface {
    terrain::FlattenSet set;
    TaxiwayField field;
    SurfaceElevFn fn;
    explicit Surface(const Airport& a) : field(a) {
        for (const FlattenArea& f : airportFlattenAreas(a)) set.add(f);
        fn = [this](double lat, double lon) {
            return static_cast<double>(set.elevationM(
                lat, lon, [&] { return static_cast<float>(field.elevM(lat, lon)); }));
        };
    }
};

struct Tri {
    Pt2 a, b, c;
};

// Vertices back to geodetic, triangles back to the local frame.
std::vector<Tri> trianglesOf(const std::vector<SurfaceMesh>& meshes, const LocalFrame& f) {
    std::vector<Tri> out;
    for (const SurfaceMesh& m : meshes) {
        std::vector<Pt2> local;
        for (const SurfaceVertex& v : m.verts) {
            const Ecef p{m.anchor.x + v.pos[0], m.anchor.y + v.pos[1], m.anchor.z + v.pos[2]};
            const GeoPos g = ecefToGeodetic(p);
            local.push_back(f.toLocal(LatLon{g.latDeg, g.lonDeg}));
        }
        for (size_t i = 0; i + 2 < m.indices.size(); i += 3)
            out.push_back({local[m.indices[i]], local[m.indices[i + 1]], local[m.indices[i + 2]]});
    }
    return out;
}

double crossP(const Pt2& a, const Pt2& b, const Pt2& c) {
    return (b.eM - a.eM) * (c.nM - a.nM) - (b.nM - a.nM) * (c.eM - a.eM);
}

bool covered(const std::vector<Tri>& tris, const Pt2& p) {
    for (const Tri& t : tris) {
        const double d1 = crossP(t.a, t.b, p), d2 = crossP(t.b, t.c, p), d3 = crossP(t.c, t.a, p);
        const bool neg = d1 < -1e-9 || d2 < -1e-9 || d3 < -1e-9;
        const bool pos = d1 > 1e-9 || d2 > 1e-9 || d3 > 1e-9;
        if (!(neg && pos)) return true;
    }
    return false;
}

double distToPolyline(const Pt2& p, const std::vector<Pt2>& pts) {
    double best = 1e30;
    for (size_t i = 0; i + 1 < pts.size(); ++i) {
        const Pt2 &a = pts[i], &b = pts[i + 1];
        const double ee = b.eM - a.eM, en = b.nM - a.nM, l2 = ee * ee + en * en;
        double t = l2 > 0 ? ((p.eM - a.eM) * ee + (p.nM - a.nM) * en) / l2 : 0;
        t = std::min(1.0, std::max(0.0, t));
        best = std::min(best, std::hypot(p.eM - a.eM - t * ee, p.nM - a.nM - t * en));
    }
    return best;
}

// Joins: a 90 degree junction of two segments, a 45 degree bend inside one segment and a 20
// degree (mitred) bend; one runway far to the south for the heights.
struct JoinAirport {
    AirportBuilder b{"XJN", 46.0, -122.0, 330.0f, AirportType::Medium};
    std::vector<Pt2> a{{-300, 0}, {0, 0}}, c{{0, 0}, {0, 300}};
    std::vector<Pt2> bend45{{500, 0}, {700, 0}, {841.42, 141.42}};
    std::vector<Pt2> bend20{{500, 400}, {700, 400}, {887.9, 468.4}};
    JoinAirport() {
        b.runway("09", "27", {-1500, -2000}, {1500, -2000}, 100.0, 100.0);
        b.taxiway(a, 20.0f);
        b.taxiway(c, 20.0f);
        b.taxiway(bend45, 20.0f);
        b.taxiway(bend20, 20.0f);
    }
};

// A runway with a 90 and a 30 degree exit ending on its centerline, a taxiway across it and a
// taxilane over an apron.
struct RunwayAirport {
    AirportBuilder b{"XRW", 46.0, -122.0, 330.0f, AirportType::Medium};
    RunwayAirport() {
        b.runway("09", "27", {-1500, 0}, {1500, 0}, 100.0, 100.4);
        b.taxiway(line({-1200, 200}, {1200, 200}, 50), 23.0f);
        b.taxiway(line({-600, 200}, {-600, 0}, 20), 23.0f);
        b.taxiway(curve({400, 200}, {520, 150}, {746, 0}, 10), 23.0f);
        b.taxiway(line({0, 400}, {0, -400}, 20), 23.0f);  // across the runway
        b.taxiway(line({-400, 300}, {-400, 700}, 20), 7.5f, TaxiwayKind::Taxilane);
        b.apron({{-600, 400}, {-200, 400}, {-200, 650}, {-600, 650}});
    }
};

}  // namespace

TEST_CASE("world: taxiway meshes are RTC, 16-bit indexed and sit on the surface plus lift") {
    RunwayAirport ra;
    const Airport& a = ra.b.airport;
    Surface s(a);
    const TaxiwayMeshes m = buildTaxiwayMeshes(a, s.fn);
    REQUIRE(m.pavement.size() == 1);
    REQUIRE(m.markings.size() == 1);
    CHECK(m.pavementTriangles > 50);
    CHECK(m.markingTriangles > 50);
    CHECK(m.radiusM > 700.0);
    for (const auto& [meshes, lift] : {std::pair{&m.pavement, kTaxiwayPavementLiftM},
                                       std::pair{&m.markings, kTaxiwayMarkingLiftM}}) {
        for (const SurfaceMesh& mesh : *meshes) {
            REQUIRE(mesh.indices.size() % 3 == 0);
            for (const uint16_t i : mesh.indices) REQUIRE(i < mesh.verts.size());
            double worst = 0;
            for (const SurfaceVertex& v : mesh.verts) {
                const Ecef p{mesh.anchor.x + v.pos[0], mesh.anchor.y + v.pos[1],
                             mesh.anchor.z + v.pos[2]};
                const GeoPos g = ecefToGeodetic(p);
                worst = std::max(worst, std::fabs(g.altMslM - lift - s.fn(g.latDeg, g.lonDeg)));
                CHECK(std::fabs(v.pos[0]) < 5000.0f);  // offsets from the anchor (RTC)
            }
            CHECK(worst < 0.02);
        }
    }
}

TEST_CASE("world: taxiway pavement has no gap at a 90 degree junction or at bends") {
    JoinAirport ja;
    const Airport& a = ja.b.airport;
    Surface s(a);
    const TaxiwayMeshes m = buildTaxiwayMeshes(a, s.fn);
    const std::vector<Tri> tris = trianglesOf(m.pavement, ja.b.frame());
    const double h = 10.0;
    struct Join {
        const char* name;
        Pt2 at;
        std::vector<std::vector<Pt2>> lines;
    };
    const std::vector<Join> joins = {{"90 junction", {0, 0}, {ja.a, ja.c}},
                                     {"45 bend", {700, 0}, {ja.bend45}},
                                     {"20 bend", {700, 400}, {ja.bend20}}};
    for (const Join& j : joins) {
        int samples = 0, holes = 0;
        for (double e = j.at.eM - 30; e <= j.at.eM + 30; e += 0.5) {
            for (double n = j.at.nM - 30; n <= j.at.nM + 30; n += 0.5) {
                const Pt2 p{e, n};
                double d = 1e30;
                for (const auto& l : j.lines) d = std::min(d, distToPolyline(p, l));
                if (d > h - 0.1) continue;  // not pavement
                ++samples;
                if (!covered(tris, p)) ++holes;
            }
        }
        CAPTURE(j.name);
        CHECK(samples > 1000);
        CHECK(holes == 0);
    }
}

TEST_CASE("world: taxiway edge lines run on through mitred bends") {
    JoinAirport ja;
    const Airport& a = ja.b.airport;
    Surface s(a);
    const TaxiwayMeshes m = buildTaxiwayMeshes(a, s.fn);
    const std::vector<Tri> lines = trianglesOf(m.markings, ja.b.frame());
    // The 20 degree bend at (700, 400): both edge bands (centred 0.225 m inside the 10 m half-width)
    // on both pieces, up to 2.5 m from the joint (the inner band ends at the inner mitre point,
    // 10 x tan 10 deg = 1.8 m before the joint).
    const double off = 10.0 - 0.5 * kEdgeLineWidthM;
    const double c = std::cos(20.0 * kRadPerDeg), sn = std::sin(20.0 * kRadPerDeg);
    int missing = 0, samples = 0;
    for (const double side : {-1.0, 1.0}) {
        for (double t = -15.0; t <= 15.0; t += 0.5) {
            if (std::fabs(t) < 2.5) continue;
            const Pt2 p =
                t < 0 ? Pt2{700.0 + t, 400.0 - side * off}
                      : Pt2{700.0 + c * t + sn * side * off, 400.0 + sn * t - c * side * off};
            ++samples;
            if (!covered(lines, p)) ++missing;
        }
    }
    CHECK(samples > 100);
    CHECK(missing == 0);
}

TEST_CASE("world: taxiways stop at the runway edge, pavement and markings alike") {
    RunwayAirport ra;
    const Airport& a = ra.b.airport;
    Surface s(a);
    const TaxiwayMeshes m = buildTaxiwayMeshes(a, s.fn);
    const LocalFrame& f = ra.b.frame();
    const std::vector<std::array<Pt2, 4>> rects = runwayRectsLocal(a, f);
    REQUIRE(rects.size() == 1);
    const auto& r = rects[0];
    auto deepInside = [&](const Pt2& p) {  // inside the rectangle by more than 5 cm
        for (int e = 0; e < 4; ++e) {
            const Pt2 &q0 = r[static_cast<size_t>(e)], &q1 = r[static_cast<size_t>((e + 1) % 4)];
            const double l = std::hypot(q1.eM - q0.eM, q1.nM - q0.nM);
            if (crossP(q0, q1, p) / l < 0.05) return false;
        }
        return true;
    };
    for (const auto* meshes : {&m.pavement, &m.markings}) {
        int inside = 0;
        for (const Tri& t : trianglesOf(*meshes, f))
            for (const Pt2& p : {t.a, t.b, t.c}) inside += deepInside(p);
        CHECK(inside == 0);
    }
    // The pavement reaches the edge: just outside it, along the exits and the crossing taxiway.
    const std::vector<Tri> pave = trianglesOf(m.pavement, f);
    const double edgeN = 0.5 * ftToM(150.0);
    for (const Pt2 p : {Pt2{-600, edgeN + 0.2}, Pt2{0, edgeN + 0.2}, Pt2{0, -edgeN - 0.2},
                        Pt2{-605, edgeN + 0.2}, Pt2{5, -edgeN - 0.2}}) {
        CAPTURE(p.eM);
        CAPTURE(p.nM);
        CHECK(covered(pave, p));
    }
    CHECK_FALSE(covered(pave, Pt2{0, 0}));  // the runway itself is not taxiway pavement
}

TEST_CASE("world: taxiway edge lines break inside aprons; the centerline runs on") {
    RunwayAirport ra;
    const Airport& a = ra.b.airport;
    Surface s(a);
    const TaxiwayMeshes m = buildTaxiwayMeshes(a, s.fn);
    const LocalFrame& f = ra.b.frame();
    const std::vector<Pt2> lane{{-400, 300}, {-400, 700}};
    int inApron = 0, offCenter = 0;
    for (const Tri& t : trianglesOf(m.markings, f)) {
        for (const Pt2& p : {t.a, t.b, t.c}) {
            if (p.eM < -599 || p.eM > -201 || p.nM < 401 || p.nM > 649) continue;  // apron
            ++inApron;
            if (distToPolyline(p, lane) > 0.3) ++offCenter;
        }
    }
    CHECK(inApron > 0);     // the centerline crosses the apron
    CHECK(offCenter == 0);  // no edge line inside it
}

TEST_CASE("world: KPDX-sized taxiways fit one pavement and one marking mesh") {
    const Airport a = fsim::testing::kpdxSizedAirport();
    Surface s(a);
    const auto t0 = std::chrono::steady_clock::now();
    const TaxiwayMeshes m = buildTaxiwayMeshes(a, s.fn);
    const double ms =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    size_t pv = 0, mv = 0;
    for (const SurfaceMesh& x : m.pavement) pv += x.verts.size();
    for (const SurfaceMesh& x : m.markings) mv += x.verts.size();
    std::printf("[W8c] KPDX-sized: pavement %d triangles (%zu vertices), markings %d triangles "
                "(%zu vertices), %zu + %zu meshes, built in %.1f ms\n",
                m.pavementTriangles, pv, m.markingTriangles, mv, m.pavement.size(),
                m.markings.size(), ms);
    CHECK(m.pavement.size() == 1);
    CHECK(m.markings.size() == 1);
    CHECK(m.pavementTriangles + m.markingTriangles < 40000);
}

TEST_CASE("world: airports without OSM data keep the generic apron; with it, no generic apron") {
    RunwayAirport ra;
    Airport bare = ra.b.airport;
    bare.taxiways.clear();
    bare.aprons.clear();
    Surface s(bare);
    CHECK(buildTaxiwayMeshes(bare, s.fn).empty());
    const AirportExtras generic = buildAirportExtras(bare);
    CHECK_FALSE(generic.flat.empty());
    CHECK_FALSE(generic.solid.empty());
    const AirportExtras osm = buildAirportExtras(ra.b.airport);
    CHECK(osm.flat.empty());
    // XTST (no OSM data) is unchanged too.
    SyntheticNavDb db;
    CHECK_FALSE(buildAirportExtras(db.airport()).flat.empty());
}

TEST_CASE("world: a building stays only clear of the OSM pavement") {
    RunwayAirport ra;
    const Airport& a = ra.b.airport;
    const LocalFrame& f = ra.b.frame();
    auto square = [&](double e, double n, LatLon out[4]) {
        const Pt2 c[4] = {{e, n}, {e + 10, n}, {e + 10, n + 10}, {e, n + 10}};
        for (int i = 0; i < 4; ++i) out[i] = f.toLatLon(c[i]);
    };
    LatLon c[4];
    square(-500, 500, c);  // on the apron
    CHECK_FALSE(clearOfTaxiways(a, c, 4, 30.0));
    square(100, 210, c);  // on the parallel taxiway
    CHECK_FALSE(clearOfTaxiways(a, c, 4, 30.0));
    square(100, 260, c);  // 50 m off it
    CHECK(clearOfTaxiways(a, c, 4, 30.0));
}

TEST_CASE("world: taxiways draw within 5 NM of their bounding circle") {
    const Ecef center{4000000.0, 0.0, 5000000.0};
    const double r = 2500.0;
    const double len = std::sqrt(center.x * center.x + center.z * center.z);
    auto eyeAt = [&](double distM) {
        return Ecef{center.x + center.x / len * distM, 0.0, center.z + center.z / len * distM};
    };
    CHECK(taxiwaysInRange(eyeAt(0.0), center, r));
    CHECK(taxiwaysInRange(eyeAt(r + nmToM(5.0) - 50.0), center, r));
    CHECK_FALSE(taxiwaysInRange(eyeAt(r + nmToM(5.0) + 50.0), center, r));
    CHECK(kTaxiwayRangeM == doctest::Approx(nmToM(5.0)).epsilon(1e-3));
}

TEST_CASE("app: scenario lat_deg / lon_deg / heading_true_deg place a runway or cruise start") {
    SyntheticNavDb db;
    AircraftProfile profile;
    profile.id = "c172p";
    profile.vappKt = 65;
    profile.flapDetents.resize(4);
    Scenario sc;
    std::string err;
    REQUIRE(parseScenario("[scenario]\nairport = XTST\nrunway = 27\nstart = runway\n"
                          "lat_deg = 40.01\nlon_deg = -100.02\nheading_true_deg = 123\n",
                          sc, err));
    StartCondition st;
    REQUIRE_MESSAGE(resolveStart(sc, db, profile, st, err), err);
    CHECK(st.kind == StartCondition::Kind::Runway);
    CHECK(st.latDeg == doctest::Approx(40.01));
    CHECK(st.lonDeg == doctest::Approx(-100.02));
    CHECK(st.headingTrueDeg == doctest::Approx(123.0f));
    REQUIRE(parseScenario("[scenario]\nairport = XTST\nrunway = 27\nstart = cruise\n"
                          "altitude_ft = 3000\nlat_deg = 40.05\nlon_deg = -100.1\n",
                          sc, err));
    REQUIRE_MESSAGE(resolveStart(sc, db, profile, st, err), err);
    CHECK(st.latDeg == doctest::Approx(40.05));
    CHECK(st.altMslFt == doctest::Approx(3000.0));
    // Without the keys nothing changes; a final start ignores them.
    REQUIRE(parseScenario("[scenario]\nairport = XTST\nrunway = 27\nstart = final\n"
                          "distance_nm = 3\nlat_deg = 41\nlon_deg = -101\n",
                          sc, err));
    REQUIRE_MESSAGE(resolveStart(sc, db, profile, st, err), err);
    CHECK(st.latDeg != doctest::Approx(41.0));
}
