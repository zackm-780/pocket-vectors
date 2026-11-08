// W1b (1.3): the runway surface at KPDX on the real pdx_oth pack. Manual checks: they need the
// real nav and terrain packs and only run when FSIM_REAL_DATA_DIR points at a data root
// (<root>/nav, <root>/terrain); otherwise they return immediately (tests never need real data).
//
//   FSIM_REAL_DATA_DIR=/path/to/data ./flightsim_tests --test-case='*runway surface real data*'
//
// 1. Profiles every 10 m along each KPDX centerline (300 m beyond both ends) and across the middle
//    of each runway: the worst step between neighbouring samples on the pavement, the steepest
//    slope in the blend, and the surface against the straight grade between the two ends.
// 2. A fast roll down 28L across 03/21 (owner, 2026-11-08: "when I hit an intersecting runway I
//    bounced pretty dramatically"): the vertical load factor on the roll.
#include <doctest/doctest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>

#include "app/Simulation.h"
#include "core/Geo.h"
#include "core/Paths.h"
#include "core/Units.h"
#include "nav/NavDb.h"
#include "sim/groundSupport.h"
#include "world/AirportGeometry.h"
#include "world/terrain/FlattenSet.h"
#include "world/terrain/TerrainService.h"

using namespace fsim;
using namespace fsim::test;

namespace {

struct RealKpdx {
    Paths paths = Paths::resolve({});
    CsvNavDatabase nav;
    std::unique_ptr<terrain::TerrainService> svc;
    const Airport* kpdx = nullptr;
    std::string error;

    bool open() {
        const char* env = std::getenv("FSIM_REAL_DATA_DIR");
        if (!env || !*env) return false;
        if (!nav.load(Paths::join(env, "nav"), error)) return false;
        svc = std::make_unique<terrain::TerrainService>(Paths::join(env, "terrain"));
        if (!svc->isOpen()) return false;
        kpdx = nav.findAirport("KPDX");
        if (!kpdx) return false;
        world::registerAirportFlatten(*kpdx, *svc);  // as the AirportRenderer does
        // Load the finest detail over the whole field.
        for (const Runway& r : kpdx->runways) {
            svc->prefetch(r.thrLatDeg, r.thrLonDeg, 3000.0f);
            svc->prefetch(r.endLatDeg, r.endLonDeg, 3000.0f);
        }
        for (int i = 0; i < 4000; ++i) {
            bool ready = true;
            for (const Runway& r : kpdx->runways)
                ready = ready && svc->isDetailReady(r.thrLatDeg, r.thrLonDeg) &&
                        svc->isDetailReady(r.endLatDeg, r.endLonDeg);
            if (ready) break;
            svc->update(kpdx->latDeg, kpdx->lonDeg);
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return true;
    }
};

struct Profile {
    double worstStepM = 0, worstStepAtM = 0;          // on the pavement, between 10 m samples
    double worstBlendSlope = 0;                       // beyond the ends (rise over run)
    double worstOffGradeM = 0, worstOffGradeAtM = 0;  // pavement vs the straight grade
    double worstCrossSlope = 0;                       // across the middle, out to 300 m
};

// The slopes the flattening adds: the surface slope minus the natural slope over the same 10 m,
// inside the area and its blend (margins 50 m across and 100 m beyond the ends, blend 150 m).
// The natural relief beyond (levees, ditches along the Columbia) is not the runway's doing.
Profile profileRunway(RealKpdx& w, const Runway& r) {
    Profile p;
    const LatLon s{r.startLatDeg, r.startLonDeg}, e{r.endLatDeg, r.endLonDeg};
    const double lenM = greatCircleDistM(s.latDeg, s.lonDeg, e.latDeg, e.lonDeg);
    const double brg = initialBearingTrueRad(s.latDeg, s.lonDeg, e.latDeg, e.lonDeg);
    const double z0 = ftToM(r.thrElevFt), z1 = ftToM(r.endElevFt);
    const double reachEndM = 100.0 + 150.0, reachSideM = ftToM(r.widthFt) * 0.5 + 50.0 + 150.0;
    double prev = 0, prevNat = 0;
    bool havePrev = false;
    for (double a = -reachEndM; a <= lenM + reachEndM; a += 10) {
        double lat = 0, lon = 0;
        destinationPoint(s.latDeg, s.lonDeg, a >= 0 ? brg : brg + kPi, std::fabs(a), lat, lon);
        const double z = w.svc->elevationM(lat, lon);
        const double nat = w.svc->naturalElevationM(lat, lon);
        const bool onPavement = a >= 0 && a <= lenM;
        if (havePrev) {
            const double step = std::fabs(z - prev);
            if (onPavement && step > p.worstStepM) {
                p.worstStepM = step;
                p.worstStepAtM = a;
            }
            if (!onPavement)
                p.worstBlendSlope =
                    std::max(p.worstBlendSlope, std::fabs((z - prev) - (nat - prevNat)) / 10.0);
        }
        if (onPavement) {
            const double grade = z0 + (z1 - z0) * a / lenM;
            if (std::fabs(z - grade) > p.worstOffGradeM) {
                p.worstOffGradeM = std::fabs(z - grade);
                p.worstOffGradeAtM = a;
            }
        }
        prev = z;
        prevNat = nat;
        havePrev = true;
    }
    double mLat = 0, mLon = 0;
    destinationPoint(s.latDeg, s.lonDeg, brg, lenM * 0.5, mLat, mLon);
    for (int side : {-1, 1}) {
        double last = w.svc->elevationM(mLat, mLon), lastNat = w.svc->naturalElevationM(mLat, mLon);
        for (double d = 10; d <= reachSideM; d += 10) {
            double lat = 0, lon = 0;
            destinationPoint(mLat, mLon, brg + side * kPi * 0.5, d, lat, lon);
            const double z = w.svc->elevationM(lat, lon), nat = w.svc->naturalElevationM(lat, lon);
            p.worstCrossSlope =
                std::max(p.worstCrossSlope, std::fabs((z - last) - (nat - lastNat)) / 10.0);
            last = z;
            lastNat = nat;
        }
    }
    return p;
}

}  // namespace

TEST_CASE("terrain: runway surface real data: KPDX profiles along and across every runway") {
    RealKpdx w;
    if (!w.open()) return;
    double worstStep = 0, worstBlend = 0, worstOffGrade = 0, worstCross = 0;
    for (const Runway& r : w.kpdx->runways) {
        if (r.ident > r.oppositeIdent) continue;  // one direction per pavement
        const Profile p = profileRunway(w, r);
        const double natThr = w.svc->naturalElevationM(r.thrLatDeg, r.thrLonDeg);
        const double natEnd = w.svc->naturalElevationM(r.endLatDeg, r.endLonDeg);
        std::printf(
            "[W1b] KPDX %s/%s: ends %.1f / %.1f ft (DEM %.1f / %.1f ft); worst step on the "
            "pavement %.3f m at %.0f m; off grade %.3f m at %.0f m; the blend adds 1:%.0f along, "
            "1:%.0f across\n",
            r.ident.c_str(), r.oppositeIdent.c_str(), r.thrElevFt, r.endElevFt, mToFt(natThr),
            mToFt(natEnd), p.worstStepM, p.worstStepAtM, p.worstOffGradeM, p.worstOffGradeAtM,
            p.worstBlendSlope > 0 ? 1.0 / p.worstBlendSlope : 0.0,
            p.worstCrossSlope > 0 ? 1.0 / p.worstCrossSlope : 0.0);
        worstStep = std::max(worstStep, p.worstStepM);
        worstBlend = std::max(worstBlend, p.worstBlendSlope);
        worstOffGrade = std::max(worstOffGrade, p.worstOffGradeM);
        worstCross = std::max(worstCross, p.worstCrossSlope);
    }
    CHECK(worstStep <= 0.10);       // 1:100 over a 10 m sample
    CHECK(worstBlend <= 1.0 / 20);  // W1b: the blend adds no slope steeper than 1:20
    CHECK(worstCross <= 1.0 / 20);
    std::printf("[W1b] KPDX worst: step %.3f m / 10 m, blend adds 1:%.0f, across 1:%.0f\n",
                worstStep, worstBlend > 0 ? 1.0 / worstBlend : 0.0,
                worstCross > 0 ? 1.0 / worstCross : 0.0);
}

TEST_CASE("sim: ground real data: KPDX 28L fast roll across 03/21 without a bump") {
    RealKpdx w;
    if (!w.open()) return;
    Simulation sim(w.paths, w.nav, *w.svc);
    sim.setTerrainPump([&](double la, double lo) { w.svc->update(la, lo); });
    Scenario sc;
    std::string err;
    REQUIRE_MESSAGE(
        parseScenario("[scenario]\nairport = KPDX\nrunway = 28L\nstart = runway\n", sc, err), err);
    REQUIRE_MESSAGE(sim.init(sc, "", err), err);
    const Runway* rwy = findRunway(*w.kpdx, "28L");
    const Runway* x = findRunway(*w.kpdx, "03");
    REQUIRE(rwy);
    REQUIRE(x);
    const LatLon thr{rwy->thrLatDeg, rwy->thrLonDeg}, end{rwy->endLatDeg, rwy->endLonDeg};
    const LatLon xs{x->startLatDeg, x->startLonDeg}, xe{x->endLatDeg, x->endLonDeg};
    for (long i = 0; i < 2 * kSimHz; ++i) sim.stepOnce();  // settle on the gear
    sim.pilot().parkingBrake = false;
    // Roll at 45 kt with the nose held down, through the 03/21 intersection (about 2 km in).
    double worstNz = 0, worstNzAlongM = 0, crossAlongM = -1;
    double integ = 0;
    for (long i = 0; i < 150 * kSimHz && !sim.aircraft().crashed; ++i) {
        const AircraftState& s = sim.aircraft();
        const double err = 45.0 - s.iasKt;
        integ = std::clamp(integ + 0.002 * err / kSimHz, -0.3, 0.3);
        sim.pilot().throttleNorm[0] =
            static_cast<float>(std::clamp(0.5 + 0.05 * err + integ, 0.0, 1.0));
        sim.pilot().rudderNorm = headingHoldRudder(s, rwy->headingTrueDeg);
        sim.pilot().steeringNorm = sim.pilot().rudderNorm;
        sim.pilot().aileronNorm = bankHoldAileron(s);
        sim.pilot().elevatorNorm = -0.3f;
        sim.stepOnce();
        const AircraftState& t = sim.aircraft();
        const double along = alongTrackM({t.latDeg, t.lonDeg}, thr, end);
        if (crossAlongM < 0 && std::fabs(crossTrackM({t.latDeg, t.lonDeg}, xs, xe)) < 1.0)
            crossAlongM = along;
        if (t.simTimeS > 8.0 && t.wowAny && std::fabs(t.nzG - 1.0) > worstNz) {
            worstNz = std::fabs(t.nzG - 1.0);
            worstNzAlongM = along;
        }
        if (crossAlongM > 0 && along > crossAlongM + 300.0) break;
    }
    std::printf("[W1b] KPDX 28L roll: crossed 03/21 at %.0f m; worst |nz - 1| %.3f g at %.0f m\n",
                crossAlongM, worstNz, worstNzAlongM);
    CHECK(crossAlongM > 0);
    CHECK_FALSE(sim.aircraft().crashed);
    CHECK(worstNz < 0.15);
}

// Every airport in the pdx_oth box (needs only the nav data): compose its runway areas as the
// terrain does and walk every centerline every 5 m. Crossing or overlapping runways must meet
// without a step: where their published elevations agree within 3 m, nothing adds more than 1:100
// to a runway's own grade anywhere on its pavement. Where the data disagrees by more (KONP 02/20
// is about 5 m off 16/34 only 200 m from its threshold) no surface can stay under 1:100; the
// transition is still smooth, and must stay under 1:40.
TEST_CASE("terrain: runway surface real data: every pdx_oth airport, no step on any runway") {
    const char* env = std::getenv("FSIM_REAL_DATA_DIR");
    if (!env || !*env) return;
    CsvNavDatabase nav;
    std::string err;
    REQUIRE_MESSAGE(nav.load(Paths::join(env, "nav"), err), err);
    int airports = 0, withOverlap = 0;
    double worst = 0, worstDisagree = 0;
    std::string worstWhere;
    for (const Airport* a : nav.airportsInBox(43.0, -124.7, 46.3, -121.4)) {
        const std::vector<FlattenArea> areas = world::airportFlattenAreas(*a);
        if (areas.empty()) continue;
        ++airports;
        terrain::FlattenSet set;
        for (const FlattenArea& f : areas) set.add(f);
        bool overlap = false;
        for (const Runway& r : a->runways) {
            const world::RunwayInfo info = world::makeRunwayInfo(*a, r);
            if (!info.primary) continue;
            const world::RunwayFrame frame(info);
            double prev = 0, prevGrade = 0, added = 0, offM = 0;
            for (double s = -info.displacedM; s <= info.usableM(); s += 5) {
                const LatLon p = frame.latLon(s, 0);
                const double z = set.elevationM(p.latDeg, p.lonDeg, [] { return -100.0f; });
                const double grade = world::runwayGradeElevM(info, p.latDeg, p.lonDeg);
                offM = std::max(offM, std::fabs(z - grade));  // another runway reaches it
                // The slope added to the runway's own published grade (some strips are 1:50).
                if (s > -info.displacedM)
                    added = std::max(added, std::fabs((z - prev) - (grade - prevGrade)) / 5.0);
                prev = z;
                prevGrade = grade;
            }
            if (offM > 0.01) overlap = true;
            if (offM > 3.0) {
                std::printf("[W1b] %s %s meets a runway %.1f m off its grade: worst 1:%.0f\n",
                            a->ident.c_str(), r.ident.c_str(), offM, 1.0 / added);
                worstDisagree = std::max(worstDisagree, added);
            } else if (added > worst) {
                worst = added;
                worstWhere = a->ident + " " + r.ident;
            }
        }
        if (overlap) ++withOverlap;
    }
    std::printf(
        "[W1b] pdx_oth: %d airports, %d with runways that meet another; worst 1:%.0f (%s)\n",
        airports, withOverlap, worst > 0 ? 1.0 / worst : 0.0, worstWhere.c_str());
    CHECK(airports > 50);
    CHECK(withOverlap > 0);
    CHECK(worst <= 0.01);
    CHECK(worstDisagree <= 1.0 / 40);
}
