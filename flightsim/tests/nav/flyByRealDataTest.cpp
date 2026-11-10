// N10b: the fly-by arcs on the real KPDX-KOTH plan with its I05 approach. Needs the real nav pack
// and only runs when FSIM_REAL_DATA_DIR points at a data root (<root>/nav); otherwise it returns at
// once (tests never need real data):
//
//   FSIM_REAL_DATA_DIR=/path/to/flightsim/data ./flightsim_tests --test-case='*fly-by real data*'
//
// It prints one line per interior fix: the leg types into and out of it, the turn, whether it is
// flown with an arc, and the arc's radius and anticipation at 111 kt with the c172p's bank.
#include <doctest/doctest.h>

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "core/Geo.h"
#include "core/Paths.h"
#include "core/Units.h"
#include "nav/FlightPlan.h"
#include "nav/FlyBy.h"
#include "nav/NavDb.h"
#include "nav/Navigator.h"
#include "nav/Procedures.h"

using namespace fsim;

namespace {

const char* legTypeText(LegType t) {
    static const char* const names[] = {"IF", "TF", "CF", "DF", "CA", "VA",
                                        "FA", "HM", "HA", "HF", "RF"};
    return names[static_cast<int>(t)];
}

double bearingDeg(const LatLon& a, const LatLon& b) {
    return radToDeg(initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg));
}

// One plan: KPDX-KOTH with `approachId` (I05 is synthesized from the runway, I05-Z is the CIFP
// procedure). Prints the table; checks every turn that should have an arc has one.
void diagnose(const CsvNavDatabase& db, const std::string& navDir, const char* approachId) {
    const Paths paths = Paths::resolve({});
    FmsFile fms;
    PlanBuildResult r = loadPlanFromFmsFile(paths.asset("assets/plans/KPDX-KOTH.fms"), db, &fms);
    REQUIRE(r.ok);
    r.plan.approachId = approachId;
    r.plan.approachStartLeg = -1;
    r.plan.arrRunway = "05";
    fms.app = approachId;
    applyFmsProcedures(r.plan, fms, navDir);
    Navigator nav(db);
    nav.setRealApproachProvider(makeCifpApproachProvider(navDir));
    nav.setPlan(std::move(r.plan));
    const FlightPlan& plan = nav.plan();
    REQUIRE(plan.approachStartLeg > 0);

    const double radiusNm = flyByRadiusNm(111.0, flyByBankDeg(20.0));  // c172p: limit 20 deg
    std::printf("FLYBY %s: radius %.2f NM at 111 kt, bank %.0f deg; approach from leg %d\n",
                approachId, radiusNm, flyByBankDeg(20.0), plan.approachStartLeg);
    std::printf("FLYBY  i ident    in out  turn_deg arc  anticip_nm  lat lon\n");
    const int n = static_cast<int>(plan.legs.size());
    int arcs = 0, missing = 0;
    for (int i = 1; i + 1 < n; ++i) {
        const Leg& prev = plan.legs[static_cast<size_t>(i - 1)];
        const Leg& cur = plan.legs[static_cast<size_t>(i)];
        if (sameFixLeg(plan, i)) {  // the approach's IF on the last enroute fix: no turn of its own
            std::printf("FLYBY %2d %-8s %-2s (same fix as leg %d)\n", i, cur.to.ident.c_str(),
                        legTypeText(cur.type), i - 1);
            continue;
        }
        const Leg& next = plan.legs[static_cast<size_t>(flyByOutboundLeg(plan, i))];
        const LatLon p{prev.to.latDeg, prev.to.lonDeg}, x{cur.to.latDeg, cur.to.lonDeg},
            q{next.to.latDeg, next.to.lonDeg};
        const double turnDeg = wrap180(bearingDeg(x, q) - bearingDeg(p, x));
        const FlyByTurn t = planTurnAt(plan, i, radiusNm);
        std::printf("FLYBY %2d %-8s %-2s %-2s %8.1f %-4s %6.2f  %.4f %.4f%s\n", i,
                    cur.to.ident.c_str(), legTypeText(cur.type), legTypeText(next.type), turnDeg,
                    t.valid ? "yes" : "no", t.valid ? t.anticipationNm : 0.0, cur.to.latDeg,
                    cur.to.lonDeg, cur.missedApproach ? "  (missed)" : "");
        if (t.valid) ++arcs;
        // A turn the AP can fly as an arc: 1-135 degrees, not fly-over, on one side of the missed
        // approach, between legs that run straight from their previous fix.
        auto straight = [&](const Leg& l, const LatLon& from) {
            if (l.type == LegType::TF || l.type == LegType::DF) return true;
            if (l.type == LegType::IF) return true;  // mid-plan: flown as a TF
            if (l.type != LegType::CF) return false;
            const LatLon to{l.to.latDeg, l.to.lonDeg};
            const double off = wrap180(l.courseTrueDeg - bearingDeg(from, to));
            const double lenNm =
                mToNm(greatCircleDistM(from.latDeg, from.lonDeg, to.latDeg, to.lonDeg));
            return std::fabs(off) <= kCfStraightMaxDeg &&
                   lenNm * std::fabs(std::sin(degToRad(off))) <= kCfStraightMaxOffsetNm;
        };
        const bool shouldArc = std::fabs(turnDeg) >= 1.0 && std::fabs(turnDeg) <= kMaxFlyByDeg &&
                               !cur.flyOver && cur.missedApproach == next.missedApproach &&
                               straight(cur, p) && straight(next, x);
        if (shouldArc && !t.valid) {
            ++missing;
            INFO("no arc at " << std::string(cur.to.ident) << " ("
                              << std::string(legTypeText(cur.type)) << " -> "
                              << std::string(legTypeText(next.type)) << ", " << turnDeg << " deg)");
            CHECK(t.valid);
        }
    }
    std::printf("FLYBY %s: arcs %d, missing %d\n", approachId, arcs, missing);
    // The fix before the approach's first leg (enroute -> the approach) turns on an arc.
    const int entry = plan.approachStartLeg - 1;
    if (entry >= 1 && entry + 1 < n) CHECK(planTurnAt(plan, entry, radiusNm).valid);
}

}  // namespace

TEST_CASE("nav: fly-by real data: every fly-by turn of KPDX-KOTH has an arc (I05, I05-Z)") {
    const char* env = std::getenv("FSIM_REAL_DATA_DIR");
    if (!env || !*env) return;
    const std::string navDir = Paths::join(env, "nav");
    CsvNavDatabase db;
    std::string error;
    REQUIRE_MESSAGE(db.load(navDir, error), error);
    diagnose(db, navDir, "I05");    // what assets/plans/KPDX-KOTH.fms loads (synthesized)
    diagnose(db, navDir, "I05-Z");  // the CIFP procedure
}
