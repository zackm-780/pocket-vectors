#include "nav/FlyBy.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"
#include "nav/FlightPlan.h"

namespace fsim {
namespace {

constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

// Local east/north metres around an origin.
struct Frame {
    double lat0 = 0, lon0 = 0, mPerDegLon = 0;
    explicit Frame(const LatLon& o)
        : lat0(o.latDeg), lon0(o.lonDeg), mPerDegLon(kMPerDegLat * std::cos(degToRad(o.latDeg))) {}
    void toLocal(const LatLon& p, double& e, double& n) const {
        e = wrap180(p.lonDeg - lon0) * mPerDegLon;
        n = (p.latDeg - lat0) * kMPerDegLat;
    }
    LatLon toLatLon(double e, double n) const {
        return {lat0 + n / kMPerDegLat, lon0 + e / mPerDegLon};
    }
};

double courseDeg(double e, double n) {
    double c = radToDeg(std::atan2(e, n));
    if (c < 0) c += 360.0;
    return c;
}

constexpr double kSameFixNm = 0.05;  // N10b: a leg this short repeats the fix before it

double bearingTrueDeg(const LatLon& a, const Waypoint& b) {
    return wrap360(radToDeg(initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg)));
}

// N10b: a leg flown as the straight line from `from` (its previous fix) to its fix: TF, an IF in
// mid-plan (flown as a TF), DF, or a CF whose course is that line within kCfStraightMaxDeg.
bool straightLeg(const Leg& l, const LatLon& from) {
    switch (l.type) {
    case LegType::TF:
    case LegType::IF:
    case LegType::DF: return true;
    case LegType::CF:
        return std::fabs(wrap180(l.courseTrueDeg - bearingTrueDeg(from, l.to))) <=
               kCfStraightMaxDeg;
    default: return false;
    }
}

}  // namespace

double flyByRadiusNm(double gsKt, double bankDeg) {
    const double v = ktToMps(std::max(gsKt, kFlyByMinGsKt));
    const double tanBank = std::tan(degToRad(std::max(bankDeg, 1.0)));
    return mToNm(v * v / (kStdGravityMps2 * tanBank));
}

double flyByBankDeg(double bankLimitDeg) {
    return std::max(5.0, std::min(kFlyByMaxBankDeg, bankLimitDeg - kFlyByBankMarginDeg));
}

double flyByBankForRadiusDeg(double gsKt, double radiusNm) {
    if (radiusNm <= 0) return 0;
    const double v = ktToMps(std::max(gsKt, 0.0));
    return radToDeg(std::atan(v * v / (kStdGravityMps2 * nmToM(radiusNm))));
}

FlyByTurn flyByTurn(const LatLon& from, const LatLon& fix, const LatLon& to, double radiusNm) {
    FlyByTurn t;
    if (!(radiusNm > 0)) return t;
    const Frame f(fix);
    double ae, an, be, bn;
    f.toLocal(from, ae, an);
    f.toLocal(to, be, bn);
    const double lenIn = std::hypot(ae, an), lenOut = std::hypot(be, bn);
    if (lenIn < 1.0 || lenOut < 1.0) return t;
    const double u1e = -ae / lenIn, u1n = -an / lenIn;  // inbound direction of travel
    const double u2e = be / lenOut, u2n = bn / lenOut;  // outbound
    t.inCourseTrueDeg = courseDeg(u1e, u1n);
    t.outCourseTrueDeg = courseDeg(u2e, u2n);
    t.turnDeg = wrap180(t.outCourseTrueDeg - t.inCourseTrueDeg);
    const double absTurn = std::fabs(t.turnDeg);
    if (absTurn < 1.0 || absTurn > kMaxFlyByDeg) return t;
    const double half = std::tan(degToRad(absTurn) * 0.5);
    double rM = nmToM(radiusNm);
    double aM = rM * half;
    const double fitM = 0.5 * std::min(lenIn, lenOut);
    if (aM > fitM) {  // the arc must fit in half of the shorter leg
        aM = fitM;
        rM = aM / half;
    }
    const double s = t.turnDeg > 0 ? 1.0 : -1.0;
    const double t1e = -u1e * aM, t1n = -u1n * aM;
    const double t2e = u2e * aM, t2n = u2n * aM;
    // Right of the inbound direction is (n, -e); the centre is on the turn side.
    const double ce = t1e + s * rM * u1n, cn = t1n - s * rM * u1e;
    t.fix = fix;
    t.start = f.toLatLon(t1e, t1n);
    t.end = f.toLatLon(t2e, t2n);
    t.center = f.toLatLon(ce, cn);
    t.radiusNm = mToNm(rM);
    t.anticipationNm = mToNm(aM);
    t.valid = true;
    return t;
}

ArcGuide arcGuide(const FlyByTurn& t, const LatLon& pos) {
    ArcGuide g;
    if (!t.valid) return g;
    const Frame f(t.center);
    double pe, pn, se, sn;
    f.toLocal(pos, pe, pn);
    f.toLocal(t.start, se, sn);
    const double s = t.turnDeg > 0 ? 1.0 : -1.0;
    const double d = std::hypot(pe, pn);
    const double rM = nmToM(t.radiusNm);
    g.xtkNm = mToNm(s * (rM - d));  // right turn: inside the circle is right of the path
    const double theta = courseDeg(pe, pn), theta1 = courseDeg(se, sn);
    double dtk = theta + s * 90.0;
    if (dtk < 0) dtk += 360.0;
    if (dtk >= 360.0) dtk -= 360.0;
    g.dtkTrueDeg = dtk;
    const double swept = s * wrap180(theta - theta1);
    g.progress01 = swept / std::fabs(t.turnDeg);
    g.done = g.progress01 >= 1.0;
    return g;
}

int arcPoints(const FlyByTurn& t, LatLon* out, int maxPoints) {
    if (!t.valid || maxPoints < 2) return 0;
    const Frame f(t.center);
    double se, sn;
    f.toLocal(t.start, se, sn);
    const double rM = nmToM(t.radiusNm);
    const double theta1 = std::atan2(se, sn);
    const int segs = std::min(
        maxPoints - 1, std::max(1, static_cast<int>(std::ceil(std::fabs(t.turnDeg) / 10.0))));
    const double sweepRad = degToRad(t.turnDeg);  // + = clockwise = increasing bearing
    for (int k = 0; k <= segs; ++k) {
        const double b = theta1 + sweepRad * k / segs;
        out[k] = f.toLatLon(rM * std::sin(b), rM * std::cos(b));
    }
    out[0] = t.start;
    out[segs] = t.end;
    return segs + 1;
}

bool sameFixLeg(const FlightPlan& plan, int legIndex) {
    const int n = static_cast<int>(plan.legs.size());
    if (legIndex < 1 || legIndex >= n) return false;
    const Leg& l = plan.legs[static_cast<size_t>(legIndex)];
    const Leg& prev = plan.legs[static_cast<size_t>(legIndex - 1)];
    if (l.type != LegType::IF && l.type != LegType::TF) return false;
    const double dM = greatCircleDistM(l.to.latDeg, l.to.lonDeg, prev.to.latDeg, prev.to.lonDeg);
    return mToNm(dM) < kSameFixNm;
}

int flyByOutboundLeg(const FlightPlan& plan, int fixLeg) {
    const int n = static_cast<int>(plan.legs.size());
    int o = fixLeg + 1;
    if (sameFixLeg(plan, o) && o + 1 < n) ++o;
    return o;
}

bool flyByEligible(const FlightPlan& plan, int fixLeg) {
    const int n = static_cast<int>(plan.legs.size());
    if (fixLeg < 1 || fixLeg + 1 >= n) return false;
    if (sameFixLeg(plan, fixLeg)) return false;  // the turn is at the fix before, looking through
    const int o = flyByOutboundLeg(plan, fixLeg);
    const Leg& prev = plan.legs[static_cast<size_t>(fixLeg - 1)];
    const Leg& cur = plan.legs[static_cast<size_t>(fixLeg)];
    const Leg& next = plan.legs[static_cast<size_t>(o)];
    if (cur.flyOver || cur.missedApproach != next.missedApproach) return false;
    if (o != fixLeg + 1) {  // the same-fix leg looked through
        const Leg& dup = plan.legs[static_cast<size_t>(fixLeg + 1)];
        if (dup.flyOver || dup.missedApproach != cur.missedApproach) return false;
    }
    return straightLeg(cur, {prev.to.latDeg, prev.to.lonDeg}) &&
           straightLeg(next, {cur.to.latDeg, cur.to.lonDeg});
}

FlyByTurn planTurnAt(const FlightPlan& plan, int fixLeg, double radiusNm) {
    if (!(radiusNm > 0) || !flyByEligible(plan, fixLeg)) return {};
    const Leg& prev = plan.legs[static_cast<size_t>(fixLeg - 1)];
    const Leg& cur = plan.legs[static_cast<size_t>(fixLeg)];
    const Leg& next = plan.legs[static_cast<size_t>(flyByOutboundLeg(plan, fixLeg))];
    return flyByTurn({prev.to.latDeg, prev.to.lonDeg}, {cur.to.latDeg, cur.to.lonDeg},
                     {next.to.latDeg, next.to.lonDeg}, radiusNm);
}

LegSpan legSpan(const FlightPlan& plan, int i, double radiusNm) {
    LegSpan s;
    const int n = static_cast<int>(plan.legs.size());
    if (i < 1 || i >= n) return s;
    const Leg& prev = plan.legs[static_cast<size_t>(i - 1)];
    const Leg& leg = plan.legs[static_cast<size_t>(i)];
    s.start = {prev.to.latDeg, prev.to.lonDeg};
    s.end = {leg.to.latDeg, leg.to.lonDeg};
    if (sameFixLeg(plan, i)) return s;  // N10b: nothing to draw; the turn looks through it
    // N10b: the turn into this leg is at the fix before, or the one before a same-fix leg.
    const int inFix = sameFixLeg(plan, i - 1) ? i - 2 : i - 1;
    if (inFix >= 1 && flyByOutboundLeg(plan, inFix) == i)
        s.turnIn = planTurnAt(plan, inFix, radiusNm);
    if (s.turnIn.valid) s.start = s.turnIn.end;
    const FlyByTurn out = planTurnAt(plan, i, radiusNm);
    if (out.valid) s.end = out.start;
    return s;
}

}  // namespace fsim
