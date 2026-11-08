#pragma once
// N10: fly-by turns between GPS legs. The turn at a fly-by fix is an arc tangent to both legs: it
// starts on the inbound leg R tan(|turn| / 2) before the fix (T1), ends as far after it on the
// outbound leg (T2), and has radius R = V^2 / (g tan(bank)). The navigator guides along it (XTK and
// DTK relative to the arc) and the map, the mini nav and the route overlay draw it.
//
// Pure geometry in a local flat frame around the fix (east/north metres): turns are a few NM, so the
// flat-earth error is negligible. No allocation.

#include "core/Geo.h"
#include "core/NavTypes.h"

namespace fsim {

struct FlightPlan;

constexpr double kMaxFlyByDeg = 135.0;  // larger track changes are flown over (sequence, intercept)
constexpr double kFlyByMaxBankDeg = 25.0;
constexpr double kFlyByBankMarginDeg = 5.0;  // left to the AP for wind
constexpr double kFlyByMinGsKt = 40.0;
constexpr double kFlyByRollInLeadS = 2.5;  // half the roll-in: the roll starts before the arc

// Turn radius (NM) at a ground speed (kt, at least kFlyByMinGsKt) and bank (deg).
double flyByRadiusNm(double gsKt, double bankDeg);
// The fly-by bank for an aircraft whose bank limit is bankLimitDeg: min(25, limit - 5), >= 5.
double flyByBankDeg(double bankLimitDeg);
// Bank (deg, unsigned) that flies a circle of radiusNm at gsKt.
double flyByBankForRadiusDeg(double gsKt, double radiusNm);

struct FlyByTurn {
    bool valid = false;
    LatLon fix, start, end, center;  // start = T1 on the inbound leg, end = T2 on the outbound leg
    double radiusNm = 0;
    double turnDeg = 0;  // signed track change, + = right (clockwise)
    double inCourseTrueDeg = 0, outCourseTrueDeg = 0;
    double anticipationNm = 0;  // T1 to the fix, and the fix to T2
};

// The turn at `fix` from the leg from -> fix onto the leg fix -> to, radius radiusNm. Invalid when
// the track changes by less than 1 degree or more than kMaxFlyByDeg. When the arc does not fit in
// half of the shorter leg, the radius shrinks until it does.
FlyByTurn flyByTurn(const LatLon& from, const LatLon& fix, const LatLon& to, double radiusNm);

struct ArcGuide {
    double xtkNm = 0;       // + = right of the arc (NavState sign)
    double dtkTrueDeg = 0;  // the arc's tangent at the aircraft's bearing from the centre
    double progress01 = 0;  // swept angle from T1 over |turnDeg| (< 0 before T1)
    bool done = false;      // past T2
};
ArcGuide arcGuide(const FlyByTurn& turn, const LatLon& pos);

// Points along the arc from T1 to T2, about every 10 degrees, both ends included. Returns the count
// (at most maxPoints, at least 2 for a valid turn).
int arcPoints(const FlyByTurn& turn, LatLon* out, int maxPoints);

// ---- plans --------------------------------------------------------------------------------------
// True when the fix at the end of legs[fixLeg] is flown by with an arc: an interior fix, not
// fly-over, the inbound leg TF, IF or DF and the outbound leg TF, both on the same side of the
// missed approach.
bool flyByEligible(const FlightPlan& plan, int fixLeg);
// The turn at the end of legs[fixLeg] as drawn (from legs[fixLeg - 1].to); invalid when not
// eligible or radiusNm <= 0.
FlyByTurn planTurnAt(const FlightPlan& plan, int fixLeg, double radiusNm);

// What a drawer draws for legs[i] (i >= 1): the straight line from `start` to `end` (T2 of the turn
// at the previous fix, T1 of the turn at this one, or the fixes), and `turnIn`, the turn into this
// leg (valid when there is one).
struct LegSpan {
    LatLon start, end;
    FlyByTurn turnIn;
};
LegSpan legSpan(const FlightPlan& plan, int i, double radiusNm);

}  // namespace fsim
