// W8b: smooth ground under OSM taxiways and aprons (world/TaxiwayGeometry, world/terrain/FlattenSet).
// Synthetic airports only; the real-data case needs FSIM_REAL_DATA_DIR (desktop, not CI):
//   FSIM_REAL_DATA_DIR=/path/to/data ./flightsim_tests --test-case='*taxiway ground real data*'
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "core/Units.h"
#include "nav/NavDb.h"
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

// Two runways crossing at 60 degrees, the shorter one 3 m lower (as in the W1b crossing test), a
// parallel taxiway that crosses the lower runway, a 90 and a 30 degree exit, and a taxilane into
// a concave apron (an L, 6 points: 4 triangles and 6 edge strips).
struct SmallAirport {
    AirportBuilder b{"XTWY", 46.0, -122.0, 330.0f, AirportType::Medium};
    SmallAirport() {
        const double c = std::cos(degToRad(60.0)), s = std::sin(degToRad(60.0));
        b.runway("09", "27", {-1700, 0}, {1700, 0}, 100.0, 100.4);
        b.runway("04", "22", {-900 * c, -900 * s}, {900 * c, 900 * s}, 97.0, 98.0);
        // Parallel at n = 200, split at its junctions (-750, -600, 0, 400).
        b.taxiway(line({-1500, 200}, {-750, 200}, 30), 23.0f, TaxiwayKind::Taxiway, "A");
        b.taxiway(line({-750, 200}, {-600, 200}, 30), 23.0f, TaxiwayKind::Taxiway, "A");
        b.taxiway(line({-600, 200}, {0, 200}, 30), 23.0f, TaxiwayKind::Taxiway, "A");
        b.taxiway(line({0, 200}, {400, 200}, 30), 23.0f, TaxiwayKind::Taxiway, "A");
        b.taxiway(line({400, 200}, {1500, 200}, 30), 23.0f, TaxiwayKind::Taxiway, "A");
        b.taxiway(line({-600, 200}, {-600, 0}, 20), 23.0f, TaxiwayKind::Taxiway, "A1");
        b.taxiway(curve({400, 200}, {520, 150}, {746, 0}, 10), 23.0f, TaxiwayKind::Taxiway, "A2");
        // A taxilane into a concave apron, clear of both runways' pavement rings.
        b.taxiway(line({-750, 200}, {-750, 530}, 20), 7.5f, TaxiwayKind::Taxilane);
        b.apron({{-950, 500}, {-500, 500}, {-500, 620}, {-690, 620}, {-690, 760}, {-950, 760}});
    }
};

Pt2 lerp(Pt2 a, Pt2 b, double t) {
    return {a.eM + (b.eM - a.eM) * t, a.nM + (b.nM - a.nM) * t};
}

// Worst change per metre along a polyline sampled every metre.
double worstSlopeAlong(const terrain::FlattenSet& set, const LocalFrame& frame,
                       const std::vector<Pt2>& pts, float (*natural)(double, double)) {
    double worst = 0, prev = 0;
    bool first = true;
    for (size_t k = 0; k + 1 < pts.size(); ++k) {
        const double len = std::hypot(pts[k + 1].eM - pts[k].eM, pts[k + 1].nM - pts[k].nM);
        const int n = std::max(1, static_cast<int>(std::lround(len)));
        for (int i = 0; i < n; ++i) {
            const Pt2 p = lerp(pts[k], pts[k + 1], static_cast<double>(i) / n);
            const LatLon ll = frame.toLatLon(p);
            const double z =
                set.elevationM(ll.latDeg, ll.lonDeg, [&] { return natural(p.eM, p.nM); });
            if (!first) worst = std::max(worst, std::fabs(z - prev) * n / len);
            prev = z;
            first = false;
        }
    }
    return worst;
}

float stepUnderTaxiway(double, double nM) {
    return nM > 150.0 ? 99.0f : 95.0f;  // a 4 m DEM step under the parallel taxiway
}

terrain::FlattenSet setOf(const std::vector<FlattenArea>& areas) {
    terrain::FlattenSet set;
    for (const FlattenArea& a : areas) set.add(a);
    return set;
}

}  // namespace

TEST_CASE("world: the taxiway field is the runway surface on a runway, the weighted mean between") {
    SmallAirport sa;
    const Airport& a = sa.b.airport;
    const TaxiwayField field(a);
    terrain::FlattenSet runways;
    for (const Runway& r : a.runways) {
        const RunwayInfo info = makeRunwayInfo(a, r);
        if (info.primary) runways.add(runwayFlattenArea(info));
    }
    // On the runways (cores), the composed runway surface exactly.
    for (const Pt2 p : {Pt2{-1200, 0}, Pt2{300, 30}, Pt2{0, 0}, Pt2{-200, -346}}) {
        const LatLon ll = sa.b.frame().toLatLon(p);
        CHECK(field.elevM(ll.latDeg, ll.lonDeg) ==
              doctest::Approx(runways.elevationM(ll.latDeg, ll.lonDeg, [] {
                  return 0.0f;
              })).epsilon(1e-6));
    }
    // Everywhere: between the lowest and the highest runway elevation.
    for (double e = -2000; e <= 2000; e += 97)
        for (double n = -1200; n <= 1200; n += 89) {
            const LatLon ll = sa.b.frame().toLatLon({e, n});
            const double z = field.elevM(ll.latDeg, ll.lonDeg);
            CHECK(z >= 97.0 - 1e-3);
            CHECK(z <= 100.4 + 1e-3);
        }
}

TEST_CASE("world: taxiways meet runways without a step and no slope over 1:100 (W8b)") {
    SmallAirport sa;
    const Airport& a = sa.b.airport;
    const std::vector<FlattenArea> areas = airportFlattenAreas(a);
    const terrain::FlattenSet set = setOf(areas);
    const LocalFrame& f = sa.b.frame();
    int taxi = 0, apron = 0;
    for (const FlattenArea& ar : areas) {
        taxi += ar.priority == kTaxiwayPriority;
        apron += ar.priority == kApronPriority;
    }
    CHECK(taxi >= 8);
    CHECK(apron == 4 + 6);

    // Along every taxiway centerline, and on onto the runway centerline at the exits.
    struct Path {
        const char* name;
        std::vector<Pt2> pts;
    };
    const std::vector<Path> paths = {
        {"parallel", {{-1500, 200}, {1500, 200}}},
        {"90 exit onto 09/27", {{-600, 200}, {-600, 0}, {-400, 0}}},
        {"30 exit onto 09/27", curve({400, 200}, {520, 150}, {746, 0}, 10)},
        {"taxilane into the apron", {{-750, 200}, {-750, 530}, {-750, 700}}},
        {"09/27 centerline", {{-1700, 0}, {1700, 0}}},
        {"04/22 centerline", {{-450, -779.4}, {450, 779.4}}},
    };
    for (const Path& p : paths) {
        const double worst = worstSlopeAlong(set, f, p.pts, stepUnderTaxiway);
        std::printf("[W8b] %s: worst slope 1:%.0f\n", p.name, worst > 0 ? 1.0 / worst : 0.0);
        CAPTURE(p.name);
        CHECK(worst <= 0.01 + 1e-4);
    }
    // The runways keep exactly the surface they have without taxiways.
    const terrain::FlattenSet runwaysOnly =
        setOf(std::vector<FlattenArea>(areas.begin(), areas.begin() + 2));
    double worstRunway = 0;
    for (double e = -1700; e <= 1700; e += 5) {
        const LatLon ll = f.toLatLon({e, 0});
        worstRunway = std::max(
            worstRunway, static_cast<double>(std::fabs(set.elevationM(ll.latDeg, ll.lonDeg, [] {
                return 95.0f;
            }) - runwaysOnly.elevationM(ll.latDeg, ll.lonDeg, [] { return 95.0f; }))));
    }
    CHECK(worstRunway < 0.001);
    // The apron is flat (within 0.01 m) away from the taxilane (its 43.75 m core and 60 m blend)
    // and the runways' pavement rings.
    float apronElev = 0;
    for (const FlattenArea& ar : areas)
        if (ar.priority == kApronPriority) apronElev = ar.elevM;
    int samples = 0;
    for (const Pt2 p : {Pt2{-930, 740}, Pt2{-930, 520}, Pt2{-520, 520}, Pt2{-600, 600}}) {
        const LatLon ll = f.toLatLon(p);
        CHECK(set.elevationM(ll.latDeg, ll.lonDeg, [] { return 99.0f; }) ==
              doctest::Approx(apronElev).epsilon(1e-4));  // 0.01 m at 100 m
        ++samples;
    }
    CHECK(samples == 4);
}

TEST_CASE("world: airports without OSM data register the same flatten areas as before") {
    SmallAirport sa;
    Airport a = sa.b.airport;
    a.taxiways.clear();
    a.aprons.clear();
    const std::vector<FlattenArea> areas = airportFlattenAreas(a);
    REQUIRE(areas.size() == 2);
    for (const FlattenArea& ar : areas) CHECK(ar.priority == 0);
    CHECK(taxiwayFlattenAreas(a).empty());
}

TEST_CASE("world: polylines simplify, rings triangulate (convex, concave, failing)") {
    // A straight line with noise under the tolerance keeps its ends only.
    std::vector<Pt2> pts;
    for (int i = 0; i <= 20; ++i) pts.push_back({i * 10.0, (i % 2) ? 1.0 : -1.0});
    CHECK(simplifyPolyline(pts, 5.0) == std::vector<int>{0, 20});
    pts.clear();  // an L: the corner stays, the points along its legs go
    for (int i = 0; i <= 10; ++i) pts.push_back({i * 10.0, 0.0});
    for (int i = 1; i <= 10; ++i) pts.push_back({100.0, i * 10.0});
    CHECK(simplifyPolyline(pts, 5.0) == std::vector<int>{0, 10, 20});

    std::vector<std::array<int, 3>> tris;
    const std::vector<Pt2> square = {{0, 0}, {100, 0}, {100, 100}, {0, 100}};
    REQUIRE(triangulatePolygon(square, tris));
    CHECK(tris.size() == 2);
    // Concave L (clockwise input), 6 points: 4 triangles covering its area.
    const std::vector<Pt2> ell = {{0, 0}, {0, 200}, {100, 200}, {100, 100}, {200, 100}, {200, 0}};
    REQUIRE(triangulatePolygon(ell, tris));
    CHECK(tris.size() == 4);
    double area = 0;
    for (const auto& t : tris) {
        const Pt2 &p = ell[static_cast<size_t>(t[0])], &q = ell[static_cast<size_t>(t[1])],
                  &r = ell[static_cast<size_t>(t[2])];
        const double a2 = (q.eM - p.eM) * (r.nM - p.nM) - (q.nM - p.nM) * (r.eM - p.eM);
        CHECK(a2 > 0);  // counter-clockwise
        area += 0.5 * a2;
    }
    CHECK(area == doctest::Approx(30000.0));
    // A bow tie (self-intersecting) fails cleanly.
    const std::vector<Pt2> bow = {{0, 0}, {100, 100}, {100, 0}, {0, 100}};
    CHECK_FALSE(triangulatePolygon(bow, tris));
    CHECK(tris.empty());
    // A ring with a collinear point and a repeated point still covers its area.
    const std::vector<Pt2> messy = {{0, 0}, {50, 0}, {100, 0}, {100, 0}, {100, 80}, {0, 80}};
    REQUIRE(triangulatePolygon(messy, tris));
    double messyArea = 0;
    for (const auto& t : tris) {
        const Pt2 &p = messy[static_cast<size_t>(t[0])], &q = messy[static_cast<size_t>(t[1])],
                  &r = messy[static_cast<size_t>(t[2])];
        messyArea += 0.5 * ((q.eM - p.eM) * (r.nM - p.nM) - (q.nM - p.nM) * (r.eM - p.eM));
    }
    CHECK(messyArea == doctest::Approx(8000.0));
    // Ring simplification keeps at least a triangle and drops near-collinear points.
    std::vector<Pt2> ring;
    for (int i = 0; i < 40; ++i) {
        const double ang = i * 2 * kPi / 40;
        ring.push_back({100 * std::cos(ang), 100 * std::sin(ang)});
    }
    const std::vector<int> kept = simplifyRing(ring, 3.0);
    CHECK(kept.size() >= 8);
    CHECK(kept.size() < 40);
}

TEST_CASE("world: KPDX-sized synthetic field: the grid equals the full loop, few candidates") {
    const Airport a = fsim::testing::kpdxSizedAirport();
    const auto t0 = std::chrono::steady_clock::now();
    const std::vector<FlattenArea> areas = airportFlattenAreas(a);
    const double buildMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
    size_t taxi = 0, apron = 0, ringPts = 0;
    for (const FlattenArea& ar : areas) {
        taxi += ar.priority == kTaxiwayPriority;
        apron += ar.priority == kApronPriority;
    }
    for (const Apron& ap : a.aprons) ringPts += ap.points.size();
    std::printf("[W8b] KPDX-sized: %zu segments, %zu taxiway areas, %zu apron areas (%zu aprons, "
                "%zu ring points), %zu areas in all, built in %.1f ms\n",
                a.taxiways.size(), taxi, apron, a.aprons.size(), ringPts, areas.size(), buildMs);
    CHECK(a.taxiways.size() >= 450);
    CHECK(taxi >= 650);
    CHECK(a.aprons.size() == 15);
    CHECK(ringPts >= 250);

    terrain::FlattenSet set = setOf(areas);
    const LocalFrame frame(a.latDeg, a.lonDeg);
    unsigned seed = 777;
    auto rnd = [&seed](double lo, double hi) {
        seed = seed * 1664525u + 1013904223u;
        return lo + (hi - lo) * static_cast<double>(seed >> 8) / static_cast<double>(1u << 24);
    };
    double worst = 0;
    for (int i = 0; i < 10000; ++i) {
        const LatLon ll = frame.toLatLon({rnd(-2500, 2500), rnd(-1800, 1800)});
        set.setUseGridForTest(true);
        const float grid = set.elevationM(ll.latDeg, ll.lonDeg, [] { return 2.0f; });
        set.setUseGridForTest(false);
        const float all = set.elevationM(ll.latDeg, ll.lonDeg, [] { return 2.0f; });
        worst = std::max(worst, static_cast<double>(std::fabs(grid - all)));
    }
    set.setUseGridForTest(true);
    CHECK(worst <= 1e-4);
    // Mean candidates over a 50 m lattice across the field: under 5 % of the areas.
    double sum = 0;
    size_t n = 0, most = 0;
    for (double e = -1800; e <= 1800; e += 50)
        for (double nn = -1200; nn <= 1300; nn += 50) {
            const LatLon ll = frame.toLatLon({e, nn});
            const size_t c = set.candidateCount(ll.latDeg, ll.lonDeg);
            sum += static_cast<double>(c);
            most = std::max(most, c);
            ++n;
        }
    const double mean = sum / static_cast<double>(n);
    std::printf("[W8b] candidates per lookup: mean %.1f (%.1f %% of %zu), most %zu\n", mean,
                100.0 * mean / static_cast<double>(set.size()), set.size(), most);
    CHECK(mean < 0.05 * static_cast<double>(set.size()));
}

TEST_CASE("world-bench: KPDX-sized flatten lookups, grid against the full loop" * doctest::skip()) {
    const Airport a = fsim::testing::kpdxSizedAirport();
    const auto b0 = std::chrono::steady_clock::now();
    terrain::FlattenSet set;
    int builds = 0;
    for (; builds < 20; ++builds) {
        set.clear();
        for (const FlattenArea& ar : airportFlattenAreas(a)) set.add(ar);
    }
    const double buildMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - b0).count() /
        builds;
    const LocalFrame frame(a.latDeg, a.lonDeg);
    std::vector<LatLon> pts;
    unsigned seed = 99;
    for (int i = 0; i < 4096; ++i) {
        seed = seed * 1664525u + 1013904223u;
        const double e = -1900 + 3800.0 * static_cast<double>(seed >> 8) / (1u << 24);
        seed = seed * 1664525u + 1013904223u;
        const double n = -1100 + 2400.0 * static_cast<double>(seed >> 8) / (1u << 24);
        pts.push_back(frame.toLatLon({e, n}));
    }
    set.elevationM(pts[0].latDeg, pts[0].lonDeg, [] { return 0.0f; });  // build the index
    for (const bool grid : {true, false}) {
        set.setUseGridForTest(grid);
        const int lookups = grid ? 1000000 : 100000;
        double sink = 0;
        const auto t0 = std::chrono::steady_clock::now();
        for (int i = 0; i < lookups; ++i) {
            const LatLon& p = pts[static_cast<size_t>(i) & 4095];
            sink += set.elevationM(p.latDeg, p.lonDeg, [] { return 3.0f; });
        }
        const double us =
            std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0)
                .count() /
            lookups;
        std::printf("[W8b] %s: %.3f us per lookup over %zu areas (checksum %.0f)\n",
                    grid ? "grid" : "full loop", us, set.size(), sink);
    }
    std::printf("[W8b] airportFlattenAreas + add: %.2f ms per build\n", buildMs);
}

TEST_CASE("world: taxiway ground real data: KPDX and KOTH profiles along every taxiway") {
    const char* env = std::getenv("FSIM_REAL_DATA_DIR");
    if (!env || !*env) {
        MESSAGE("FSIM_REAL_DATA_DIR not set; skipped");
        return;
    }
    CsvNavDatabase db;
    std::string err;
    REQUIRE_MESSAGE(db.load(std::string(env) + "/nav", err), err);
    for (const char* ident : {"KPDX", "KOTH"}) {
        const Airport* a = db.findAirport(ident);
        REQUIRE(a != nullptr);
        if (a->taxiways.empty()) {
            MESSAGE(ident << ": no taxiways in this pack; skipped");
            continue;
        }
        const auto t0 = std::chrono::steady_clock::now();
        const std::vector<FlattenArea> areas = airportFlattenAreas(*a);
        const double ms =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0)
                .count();
        const terrain::FlattenSet set = setOf(areas);
        const LocalFrame frame(a->latDeg, a->lonDeg);
        size_t taxi = 0, apron = 0;
        for (const FlattenArea& ar : areas) {
            taxi += ar.priority == kTaxiwayPriority;
            apron += ar.priority == kApronPriority;
        }
        double worstSlope = 0, worstStep = 0;
        std::string where;
        for (const Taxiway& t : a->taxiways) {
            std::vector<Pt2> pts;
            for (const LatLon& p : t.points) pts.push_back(frame.toLocal(p));
            double prev = 0;
            bool first = true;
            for (size_t k = 0; k + 1 < pts.size(); ++k) {
                const double len = std::hypot(pts[k + 1].eM - pts[k].eM, pts[k + 1].nM - pts[k].nM);
                const int n = std::max(1, static_cast<int>(std::lround(len)));
                for (int i = 0; i <= n; ++i) {
                    const LatLon ll = frame.toLatLon(lerp(pts[k], pts[k + 1], double(i) / n));
                    const double z = set.elevationM(ll.latDeg, ll.lonDeg, [] { return 0.0f; });
                    if (!first) {
                        const double step = std::fabs(z - prev);
                        worstStep = std::max(worstStep, step);
                        if (step * n / len > worstSlope) {
                            worstSlope = step * n / len;
                            where = t.ref.empty() ? std::to_string(t.osmWayId) : t.ref;
                        }
                    }
                    prev = z;
                    first = false;
                }
            }
        }
        std::printf(
            "[W8b] %s real data: %zu segments, %zu aprons -> %zu flatten areas (%zu taxiway, "
            "%zu apron) in %.1f ms; worst slope 1:%.0f (taxiway %s), worst 1 m step %.3f m\n",
            ident, a->taxiways.size(), a->aprons.size(), areas.size(), taxi, apron, ms,
            worstSlope > 0 ? 1.0 / worstSlope : 0.0, where.c_str(), worstStep);
        CHECK(worstSlope <= 0.02);  // regression guard; the PR reports the real number
    }
}
