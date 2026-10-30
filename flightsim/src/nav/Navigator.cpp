#include "nav/Navigator.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "core/Log.h"
#include "core/SyntheticNavDb.h"  // findRunway
#include "core/Units.h"
#include "nav/FlightPlan.h"
#include "nav/Procedures.h"

namespace fsim {
namespace {

constexpr double kMinGsForRatesKt = 1.0;
constexpr double kAnticipationMinGsKt = 40.0;  // below this the anticipation uses 40 kt
constexpr double kMaxTurnAnticipationDeg = 120.0;
constexpr double kHoldDirectRadiusNm = 3.0;  // beyond this a hold is flown as direct-to the fix
constexpr double kVirtualLineNm = 300.0;     // length of the line used for CF / CA / VA legs
constexpr double kDepartureRadiusNm = 5.0;
constexpr float kDepartureMaxAglFt = 2000.0f;
constexpr float kLocFinalCourseToleranceDeg = 30.0f;
constexpr float kVnavMinAboveFt = 100.0f;
constexpr double kLatePassNm = 0.5;  // past the MAP this far, still airborne above kMapMinAglFt
constexpr float kMapMinAglFt = 50.0f;

double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

double distNm(double lat1, double lon1, double lat2, double lon2) {
    return mToNm(greatCircleDistM(lat1, lon1, lat2, lon2));
}

LatLon pointFrom(const LatLon& p, double bearingRad, double distanceNm) {
    LatLon out;
    destinationPoint(p.latDeg, p.lonDeg, bearingRad, nmToM(distanceNm), out.latDeg, out.lonDeg);
    return out;
}

bool isCourseLeg(LegType t) {
    return t == LegType::CF || t == LegType::CA || t == LegType::VA || t == LegType::FA;
}
bool isAltLeg(LegType t) {
    return t == LegType::CA || t == LegType::VA || t == LegType::FA;
}
bool isHoldLeg(LegType t) {
    return t == LegType::HM || t == LegType::HA || t == LegType::HF;
}

void mixBits(uint64_t& h, uint64_t v) {
    h ^= v;
    h *= 1099511628211ull;
}
void mixDouble(uint64_t& h, double d) {
    uint64_t bits;
    std::memcpy(&bits, &d, sizeof bits);
    mixBits(h, bits);
}

void copyIdent(char (&dst)[8], const std::string& src) {
    std::strncpy(dst, src.c_str(), sizeof dst - 1);
    dst[sizeof dst - 1] = '\0';
}

const char* kindName(ApproachKind k) {
    switch (k) {
    case ApproachKind::Ils: return "ILS";
    case ApproachKind::Loc: return "LOC";
    case ApproachKind::Rnav: return "RNAV";
    case ApproachKind::Vor: return "VOR";
    case ApproachKind::Ndb: return "NDB";
    case ApproachKind::Visual: return "VIS";
    default: return "";
    }
}

}  // namespace

Navigator::Navigator(const NavDatabase& db) : db_(db) {}

// ---- configuration -----------------------------------------------------------------------------

void Navigator::setAircraftCategory(AircraftCategory category) {
    bankDeg_ = category == AircraftCategory::Jet ? 25.0f : 20.0f;
}

void Navigator::setRealApproachProvider(RealApproachProvider provider) {
    realProvider_ = std::move(provider);
}

NavSourceSelect nextNavSourceSelect(NavSource shown, bool locAvailable, bool vorTuned) {
    switch (shown) {
    case NavSource::Loc: return vorTuned ? NavSourceSelect::Vor : NavSourceSelect::Gps;
    case NavSource::Vor: return NavSourceSelect::Auto;
    case NavSource::Gps:
    case NavSource::None: break;
    }
    if (locAvailable) return NavSourceSelect::Loc;
    if (vorTuned) return NavSourceSelect::Vor;
    return NavSourceSelect::Auto;
}

void Navigator::setSourceSelect(NavSourceSelect select) {
    select_ = select;
    locLatched_ = false;
}

void Navigator::tuneVor(const Navaid& navaid, float obsMagDeg) {
    vorTuned_ = true;
    vorIdent_ = navaid.ident;
    vorPos_ = LatLon{navaid.latDeg, navaid.lonDeg};
    vorMagVarDeg_ =
        navaid.magVarDeg != 0.0f ? navaid.magVarDeg : db_.magVarDeg(navaid.latDeg, navaid.lonDeg);
    vorObsMagDeg_ = obsMagDeg;
}

void Navigator::resetLatches() {
    locLatched_ = false;
    entryPending_ = true;
}

// ---- plan ----------------------------------------------------------------------------------------

void Navigator::setPlan(FlightPlan plan) {
    plan_ = std::move(plan);
    approach_.reset();
    cache_.valid = false;
    resetLatches();
    const int n = static_cast<int>(plan_.legs.size());
    if (n > 1 && plan_.activeLeg < 1) plan_.activeLeg = 1;
    if (n == 1 || plan_.activeLeg >= n) plan_.activeLeg = n - 1;
    if (n == 0) plan_.activeLeg = -1;
    trackedLeg_ = -1;
    if (!plan_.approachId.empty()) {
        if (plan_.approachStartLeg < 0) {
            if (!loadApproachById(plan_.approachId))
                LOG_WARN("navigator: approach %s not available for %s %s", plan_.approachId.c_str(),
                         plan_.destIdent.c_str(), plan_.arrRunway.c_str());
        } else {
            // Legs were inserted by someone else (N3/N4): keep them, fetch the metadata.
            std::string rwy =
                plan_.arrRunway.empty() ? runwayFromApproachId(plan_.approachId) : plan_.arrRunway;
            approach_ = findApproach(db_, plan_.destIdent, rwy, plan_.approachId, realProvider_);
        }
    }
}

void Navigator::clearPlan() {
    plan_ = FlightPlan{};
    approach_.reset();
    cache_.valid = false;
    resetLatches();
    trackedLeg_ = -1;
    state_ = NavState{};
}

// Real procedures: a transition may join the final segment later than its first fix (N4), so the
// final legs before the join are not flown. Synthesized approaches have no such transitions.
static std::vector<Leg> planLegsFor(const Approach& approach, int transitionIndex) {
    if (!approach.synthesized && transitionIndex >= 0 &&
        transitionIndex < static_cast<int>(approach.transitions.size()))
        return assembleApproachLegs(
            approach, approach.transitions[static_cast<size_t>(transitionIndex)].name);
    return approachPlanLegs(approach, transitionIndex);
}

// ---- approach -------------------------------------------------------------------------------------

bool Navigator::loadApproach(const Approach& approach, int transitionIndex) {
    if (plan_.legs.empty() || approach.finalLegs.empty()) return false;
    if (transitionIndex < 0) {
        LatLon prev = lastPos_;
        const int enrouteEnd = enrouteEndIndex(plan_);
        if (enrouteEnd >= 1)
            prev = LatLon{plan_.legs[static_cast<size_t>(enrouteEnd - 1)].to.latDeg,
                          plan_.legs[static_cast<size_t>(enrouteEnd - 1)].to.lonDeg};
        transitionIndex = selectTransition(approach, prev);
    }
    std::string error;
    std::vector<Leg> legs = planLegsFor(approach, transitionIndex);
    if (!insertApproach(plan_, approach.id, approach.kind, approach.runwayIdent, std::move(legs),
                        &error)) {
        LOG_WARN("navigator: cannot insert approach %s: %s", approach.id.c_str(), error.c_str());
        return false;
    }
    approach_ = approach;
    if (approach_->minimumsFt <= 0 && approach_->kind != ApproachKind::Visual) {
        // Real approach without minimums: default as synthesized (NAVIGATION 3.1).
        float thrFt = 0;
        if (const Airport* a = db_.findAirport(approach_->airportIdent))
            if (const Runway* r = findRunway(*a, approach_->runwayIdent)) thrFt = r->thrElevFt;
        approach_->minimumsFt = defaultMinimumsFt(approach_->kind, thrFt, &approach_->minimumsIsDa);
    }
    cache_.valid = false;
    resetLatches();
    return true;
}

bool Navigator::loadApproachById(std::string_view approachId) {
    std::string rwy = !plan_.arrRunway.empty() ? plan_.arrRunway : runwayFromApproachId(approachId);
    if (rwy.empty()) return false;
    auto a = findApproach(db_, plan_.destIdent, rwy, approachId, realProvider_);
    if (!a) return false;
    return loadApproach(*a);
}

void Navigator::removeApproach() {
    fsim::removeApproach(plan_, db_);
    approach_.reset();
    cache_.valid = false;
    resetLatches();
}

bool Navigator::startApproachOnly(const Approach& approach, const LatLon& present,
                                  int transitionIndex) {
    FlightPlan plan;
    plan.originIdent = plan.destIdent = approach.airportIdent;
    plan.arrRunway = approach.runwayIdent;
    Leg ppos;
    ppos.type = LegType::IF;
    ppos.to = Waypoint{"PPOS", WaypointKind::Synthetic, present.latDeg, present.lonDeg};
    plan.legs.push_back(ppos);
    plan.activeLeg = 0;
    plan_ = std::move(plan);
    approach_.reset();
    lastPos_ = present;
    haveLastPos_ = true;
    if (!loadApproach(approach, transitionIndex)) return false;
    plan_.activeLeg = plan_.approachStartLeg;
    trackedLeg_ = -1;
    return true;
}

// ---- guidance commands --------------------------------------------------------------------------

bool Navigator::directToLeg(int legIndex) {
    if (!haveLastPos_) return false;
    if (!fsim::directToLeg(plan_, legIndex, lastPos_)) return false;
    cache_.valid = false;
    resetLatches();
    return true;
}

bool Navigator::directToIdent(std::string_view ident) {
    if (!haveLastPos_) return false;
    if (!fsim::directToIdent(plan_, ident, db_, lastPos_)) return false;
    cache_.valid = false;
    resetLatches();
    return true;
}

bool Navigator::reenterApproach() {
    refreshCache();
    if (cache_.firstFinalIdx < 0) return false;
    return directToLeg(cache_.firstFinalIdx);
}

bool Navigator::activateLeg(int legIndex) {
    if (legIndex < 0 || legIndex >= static_cast<int>(plan_.legs.size())) return false;
    plan_.activeLeg = legIndex;
    resetLatches();
    return true;
}

void Navigator::activateNearestLeg(const AircraftState& ac) {
    refreshCache();
    const int n = static_cast<int>(plan_.legs.size());
    if (n < 2) return;
    lastPos_ = LatLon{ac.latDeg, ac.lonDeg};
    haveLastPos_ = true;
    entryPos_ = lastPos_;
    int best = 1;
    double bestMetric = 1e30;
    for (int i = 1; i < n; ++i) {
        const Leg& l = plan_.legs[static_cast<size_t>(i)];
        if (l.missedApproach || isAltLeg(l.type) || isHoldLeg(l.type)) continue;
        const Guide g = computeGuide(ac, i);
        double metric;
        if (g.remNm < 0)
            metric = std::hypot(g.xtkNm, g.remNm);
        else if (g.remNm > g.legLenNm)
            metric = std::hypot(g.xtkNm, g.remNm - g.legLenNm);
        else
            metric = std::fabs(g.xtkNm);
        const double trackRad = ac.gsKt > 30.0f ? ac.trackTrueRad : ac.headingTrueRad;
        if (ac.gsKt > 30.0f && std::fabs(wrapPi(trackRad - g.dtkRad)) > degToRad(100.0))
            metric += 50.0;
        if (metric < bestMetric) {
            bestMetric = metric;
            best = i;
        }
    }
    plan_.activeLeg = best;
    resetLatches();
}

// ---- caches ---------------------------------------------------------------------------------------

uint64_t Navigator::planSignature() const {
    uint64_t h = 1469598103934665603ull;
    mixBits(h, plan_.legs.size());
    mixBits(h, static_cast<uint64_t>(plan_.approachStartLeg + 1));
    mixBits(h, static_cast<uint64_t>(plan_.approachKind));
    mixBits(h, approach_ ? static_cast<uint64_t>(approach_->kind) : 99u);
    for (const Leg& l : plan_.legs) {
        mixBits(h, static_cast<uint64_t>(l.type) | (l.missedApproach ? 0x100u : 0u) |
                       (l.flyOver ? 0x200u : 0u));
        mixDouble(h, l.to.latDeg);
        mixDouble(h, l.to.lonDeg);
        mixDouble(h, l.courseTrueDeg);
        mixDouble(h, l.alt.altFt);
        mixBits(h, static_cast<uint64_t>(l.alt.kind));
    }
    return h;
}

void Navigator::refreshCache() {
    const uint64_t sig = planSignature();
    if (cache_.valid && sig == cache_.signature) return;

    Cache c;
    c.signature = sig;
    c.valid = true;
    const int n = static_cast<int>(plan_.legs.size());
    c.legLenNm.assign(static_cast<size_t>(n), 0.0f);
    c.afterNm.assign(static_cast<size_t>(n), 0.0f);
    c.afterAllNm.assign(static_cast<size_t>(n), 0.0f);
    c.toMapNm.assign(static_cast<size_t>(n), -1.0f);
    for (int i = 0; i < n; ++i)
        c.legLenNm[static_cast<size_t>(i)] = static_cast<float>(legDistNm(plan_, i));
    for (int i = n - 2; i >= 0; --i) {
        const size_t u = static_cast<size_t>(i);
        const Leg& next = plan_.legs[u + 1];
        c.afterAllNm[u] = c.afterAllNm[u + 1] + c.legLenNm[u + 1];
        c.afterNm[u] = c.afterNm[u + 1] + (next.missedApproach ? 0.0f : c.legLenNm[u + 1]);
    }
    for (int i = 0; i < n; ++i)
        if (plan_.legs[static_cast<size_t>(i)].missedApproach) {
            c.firstMissedIdx = i;
            break;
        }

    const int appStart = plan_.approachStartLeg;
    if (appStart >= 0 && appStart < n) {
        for (int i = n - 1; i >= appStart; --i)
            if (!plan_.legs[static_cast<size_t>(i)].missedApproach) {
                c.mapIdx = i;
                break;
            }
    }
    if (c.mapIdx >= 0) {
        c.firstFinalIdx = appStart;
        for (int i = c.mapIdx; i >= appStart; --i)
            if (plan_.legs[static_cast<size_t>(i)].type == LegType::IF) {
                c.firstFinalIdx = i;
                break;
            }
        c.toMapNm[static_cast<size_t>(c.mapIdx)] = 0.0f;
        for (int i = c.mapIdx - 1; i >= appStart; --i)
            c.toMapNm[static_cast<size_t>(i)] =
                c.toMapNm[static_cast<size_t>(i + 1)] + c.legLenNm[static_cast<size_t>(i + 1)];
        // FAF: ident "FAF" if present, else the fix with the largest distance to the MAP <= 8 NM.
        for (int i = appStart; i < c.mapIdx; ++i)
            if (plan_.legs[static_cast<size_t>(i)].to.ident == "FAF") {
                c.fafIdx = i;
                break;
            }
        // Real procedures publish the FAF: the last plan leg ending at that fix.
        if (c.fafIdx < 0 && approach_ && approach_->fafFinalLeg >= 0 &&
            approach_->fafFinalLeg < static_cast<int>(approach_->finalLegs.size())) {
            const std::string& fafIdent =
                approach_->finalLegs[static_cast<size_t>(approach_->fafFinalLeg)].to.ident;
            for (int i = c.mapIdx - 1; i >= appStart; --i)
                if (plan_.legs[static_cast<size_t>(i)].to.ident == fafIdent) {
                    c.fafIdx = i;
                    break;
                }
        }
        if (c.fafIdx < 0) {
            float bestNm = 0.5f;
            for (int i = c.firstFinalIdx; i < c.mapIdx; ++i) {
                const float d = c.toMapNm[static_cast<size_t>(i)];
                if (d <= 8.0f && d > bestNm) {
                    bestNm = d;
                    c.fafIdx = i;
                }
            }
        }
        if (auto dtk = legDtkTrueDeg(plan_, c.mapIdx)) {
            c.finalCourseTrueDeg = *dtk;
            c.haveFinalCourse = true;
        }
    }

    // Endpoints and the arrival runway.
    if (const Airport* o = db_.findAirport(plan_.originIdent)) {
        c.origin = LatLon{o->latDeg, o->lonDeg};
        c.haveOrigin = true;
    } else if (n > 0 && plan_.legs[0].to.kind != WaypointKind::Synthetic) {
        c.origin = LatLon{plan_.legs[0].to.latDeg, plan_.legs[0].to.lonDeg};
        c.haveOrigin = true;
    }
    c.destAirport = db_.findAirport(plan_.destIdent);
    if (c.destAirport) {
        c.dest = LatLon{c.destAirport->latDeg, c.destAirport->lonDeg};
        c.haveDest = true;
    } else {
        for (int i = n - 1; i >= 0; --i)
            if (!plan_.legs[static_cast<size_t>(i)].missedApproach) {
                c.dest = LatLon{plan_.legs[static_cast<size_t>(i)].to.latDeg,
                                plan_.legs[static_cast<size_t>(i)].to.lonDeg};
                c.haveDest = true;
                break;
            }
    }
    if (c.destAirport) {
        std::string rwy = plan_.arrRunway;
        if (rwy.empty() && approach_) rwy = approach_->runwayIdent;
        if (rwy.empty()) rwy = runwayFromApproachId(plan_.approachId);
        if (!rwy.empty()) c.arrRunway = findRunway(*c.destAirport, rwy);
    }
    const ApproachKind kind = approach_ ? approach_->kind : plan_.approachKind;
    if ((kind == ApproachKind::Ils || kind == ApproachKind::Loc) && c.arrRunway &&
        c.arrRunway->ils && c.mapIdx >= 0)
        c.ils = &*c.arrRunway->ils;

    cache_ = std::move(c);
}

bool Navigator::prevFix(int legIndex, LatLon& out) const {
    for (int j = legIndex - 1; j >= 0; --j) {
        const Leg& p = plan_.legs[static_cast<size_t>(j)];
        if (p.type == LegType::CA || p.type == LegType::VA) continue;
        out = LatLon{p.to.latDeg, p.to.lonDeg};
        return true;
    }
    return false;
}

// ---- leg geometry -------------------------------------------------------------------------------

Navigator::Guide Navigator::computeGuide(const AircraftState& ac, int legIndex) const {
    const Leg& l = plan_.legs[static_cast<size_t>(legIndex)];
    const LatLon pos{ac.latDeg, ac.lonDeg};
    const LatLon to{l.to.latDeg, l.to.lonDeg};
    Guide g;
    g.distToWptNm = distNm(pos.latDeg, pos.lonDeg, to.latDeg, to.lonDeg);
    g.brgToWptRad = initialBearingTrueRad(pos.latDeg, pos.lonDeg, to.latDeg, to.lonDeg);
    g.altTerminated = isAltLeg(l.type);
    g.isHold = isHoldLeg(l.type);

    LatLon start = entryPos_, end = to;
    bool fixedCourse = false;
    double courseRad = 0;
    switch (l.type) {
    case LegType::TF:
    case LegType::IF:
    case LegType::RF:
        if (prevFix(legIndex, start)) {
            if (distNm(start.latDeg, start.lonDeg, to.latDeg, to.lonDeg) < 1e-4) start = entryPos_;
        } else {
            start = entryPos_;
        }
        break;
    case LegType::DF: start = entryPos_; break;
    case LegType::CF:
        fixedCourse = true;
        courseRad = degToRad(l.courseTrueDeg);
        start = pointFrom(to, courseRad + kPi, kVirtualLineNm);
        break;
    case LegType::CA:
    case LegType::VA:
        fixedCourse = true;
        courseRad = degToRad(l.courseTrueDeg);
        if (!prevFix(legIndex, start)) start = entryPos_;
        end = pointFrom(start, courseRad, kVirtualLineNm);
        break;
    case LegType::FA:
        fixedCourse = true;
        courseRad = degToRad(l.courseTrueDeg);
        start = to;
        end = pointFrom(to, courseRad, kVirtualLineNm);
        break;
    case LegType::HM:
    case LegType::HA:
    case LegType::HF:
        if (g.distToWptNm > kHoldDirectRadiusNm) {
            start = entryPos_;  // direct to the fix
        } else {
            fixedCourse = true;  // inbound course through the fix
            courseRad = degToRad(l.courseTrueDeg);
            start = pointFrom(to, courseRad + kPi, kVirtualLineNm);
        }
        break;
    }

    g.legLenNm = distNm(start.latDeg, start.lonDeg, end.latDeg, end.lonDeg);
    if (g.legLenNm < 1e-4) {
        g.dtkRad = g.brgToWptRad;
        g.xtkNm = 0;
        g.remNm = g.distToWptNm;
    } else {
        g.xtkNm = mToNm(crossTrackM(pos, start, end));
        const double alongNm = mToNm(alongTrackM(pos, start, end));
        g.remNm = g.legLenNm - alongNm;
        if (fixedCourse) {
            g.dtkRad = courseRad;
        } else {
            // Local desired track: bearing from the point of the great circle abeam us to the end.
            const double brg0 =
                initialBearingTrueRad(start.latDeg, start.lonDeg, end.latDeg, end.lonDeg);
            if (g.remNm > 0.01) {
                const LatLon foot = pointFrom(start, brg0, alongNm);
                g.dtkRad = initialBearingTrueRad(foot.latDeg, foot.lonDeg, end.latDeg, end.lonDeg);
            } else {
                g.dtkRad = brg0;
            }
        }
    }
    if (l.type == LegType::CA || l.type == LegType::VA) {
        g.distToWptNm = 0;
        g.brgToWptRad = g.dtkRad;
    }
    g.dtkRad = wrap2Pi(g.dtkRad);
    return g;
}

double Navigator::courseOutRad(int legIndex) const {
    const Leg& cur = plan_.legs[static_cast<size_t>(legIndex)];
    const Leg& next = plan_.legs[static_cast<size_t>(legIndex + 1)];
    if (isCourseLeg(next.type) || isHoldLeg(next.type)) return degToRad(next.courseTrueDeg);
    return initialBearingTrueRad(cur.to.latDeg, cur.to.lonDeg, next.to.latDeg, next.to.lonDeg);
}

bool Navigator::shouldSequence(const AircraftState& ac, const Guide& g, int legIndex) const {
    const int n = static_cast<int>(plan_.legs.size());
    if (legIndex < 0 || legIndex >= n - 1) return false;
    const Leg& cur = plan_.legs[static_cast<size_t>(legIndex)];
    const Leg& next = plan_.legs[static_cast<size_t>(legIndex + 1)];
    if (isHoldLeg(cur.type)) return false;
    if (g.altTerminated)
        return cur.alt.kind != AltConstraintKind::None &&
               ac.altMslFt >= static_cast<double>(cur.alt.altFt);

    // The MAP sequences into the missed approach only when passed airborne (NAVIGATION 3.4).
    // Deviation: the spec says AGL > 50 ft, but a normal 3 degree approach crosses the MAP at
    // TCH = 50 ft AGL, which would flip every landing to MISSED. We use AGL > kMissedAglFt (100),
    // or climbing, or still airborne above 50 ft AGL more than 0.5 NM past the MAP.
    if (!cur.missedApproach && next.missedApproach) {
        if (g.remNm > 0 || ac.wowAny) return false;
        if (ac.vsFpm > NavConstants::kMissedClimbFpm) return true;
        if (ac.altAglFt > NavConstants::kMissedAglFt) return true;
        return -g.remNm > kLatePassNm && ac.altAglFt > kMapMinAglFt;
    }

    const bool flyOver = cur.flyOver || isCourseLeg(next.type) || isHoldLeg(next.type);
    if (flyOver) return g.remNm <= 0;

    const double dtkDeg = radToDeg(g.dtkRad);
    const double dPsiDeg = std::min(std::fabs(wrap180(radToDeg(courseOutRad(legIndex)) - dtkDeg)),
                                    kMaxTurnAnticipationDeg);
    const double vMps = ktToMps(std::max(static_cast<double>(ac.gsKt), kAnticipationMinGsKt));
    const double bankRad = degToRad(std::max(static_cast<double>(bankDeg_), 5.0));
    const double radiusM = vMps * vMps / (kStdGravityMps2 * std::tan(bankRad));
    double anticipationNm = mToNm(radiusM * std::tan(degToRad(dPsiDeg) * 0.5));
    anticipationNm = std::min(anticipationNm, std::max(0.5 * g.legLenNm, 0.1));
    return g.remNm <= anticipationNm;
}

void Navigator::captureEntry(const AircraftState& ac) {
    entryPos_ = LatLon{ac.latDeg, ac.lonDeg};
}

// ---- CDI scale, LOC / GS --------------------------------------------------------------------------

float Navigator::targetScaleNm(const Guide& g, NavPhase phase) const {
    switch (phase) {
    case NavPhase::Enroute: return NavConstants::kScaleEnrNm;
    case NavPhase::Approach: {
        const int a = plan_.activeLeg;
        bool apr;
        if (cache_.fafIdx >= 0)
            apr = a > cache_.fafIdx ||
                  (a == cache_.fafIdx && g.distToWptNm <= NavConstants::kAprBeforeFafNm);
        else if (a >= 0 && a < static_cast<int>(cache_.toMapNm.size()) &&
                 cache_.toMapNm[static_cast<size_t>(a)] >= 0)
            apr = g.distToWptNm + cache_.toMapNm[static_cast<size_t>(a)] <= 7.0 /* FAF 5 NM + 2 */;
        else
            apr = false;
        return apr ? NavConstants::kScaleAprNm : NavConstants::kScaleTermNm;
    }
    default: return NavConstants::kScaleTermNm;
    }
}

Navigator::Angular Navigator::computeLoc(const AircraftState& ac) const {
    Angular r;
    const Ils& ils = *cache_.ils;
    const double courseDeg = ils.locCourseTrueDeg;
    const double backDeg = courseDeg + 180.0;
    const double halfWidthDeg = 0.5 * (ils.locWidthDeg > 0 ? ils.locWidthDeg : 5.0);

    r.distToAntennaNm =
        static_cast<float>(distNm(ac.latDeg, ac.lonDeg, ils.locLatDeg, ils.locLonDeg));
    const double brgDeg =
        radToDeg(initialBearingTrueRad(ils.locLatDeg, ils.locLonDeg, ac.latDeg, ac.lonDeg));
    const double offDeg = wrap180(brgDeg - backDeg);  // + = aircraft LEFT of the course
    r.offRightDeg = static_cast<float>(-offDeg);
    r.locValid = r.distToAntennaNm <= NavConstants::kLocMaxDistNm &&
                 std::fabs(offDeg) <= NavConstants::kLocMaxOffDeg;
    r.cdi = static_cast<float>(
        clampd(-r.offRightDeg / halfWidthDeg, -NavConstants::kCdiClamp, NavConstants::kCdiClamp));
    r.xtkNm = static_cast<float>(r.distToAntennaNm * std::sin(degToRad(r.offRightDeg)));
    r.toFlag = std::fabs(offDeg) < 90.0;

    if (ils.hasGs && r.locValid) {
        const double gsDistNm = distNm(ac.latDeg, ac.lonDeg, ils.gsLatDeg, ils.gsLonDeg);
        const double gsBrgDeg =
            radToDeg(initialBearingTrueRad(ils.gsLatDeg, ils.gsLonDeg, ac.latDeg, ac.lonDeg));
        // Cone: measured from the point of the centerline abeam the GS antenna, so a real
        // antenna's lateral offset (hundreds of feet) does not invalidate the signal close in.
        const double alongNm = gsDistNm * std::cos(degToRad(wrap180(gsBrgDeg - backDeg)));
        const double lateralNm = r.distToAntennaNm * std::sin(degToRad(r.offRightDeg));
        const double gsOffDeg =
            alongNm > 1e-6 ? radToDeg(std::atan2(std::fabs(lateralNm), alongNm)) : 180.0;
        const double heightFt = ac.altMslFt - static_cast<double>(ils.gsElevFt);
        const double distFt = nmToFt(gsDistNm);
        const double elevDeg = radToDeg(std::atan2(heightFt, distFt));
        r.gsValid = gsDistNm <= NavConstants::kGsMaxDistNm &&
                    std::fabs(gsOffDeg) <= NavConstants::kGsMaxOffDeg && heightFt > 0 &&
                    distFt > 500.0;
        const double angleDeg = ils.gsAngleDeg > 0 ? ils.gsAngleDeg : 3.0;
        r.gsDefl = static_cast<float>(clampd((angleDeg - elevDeg) / NavConstants::kGsFullScaleDeg,
                                             -NavConstants::kCdiClamp, NavConstants::kCdiClamp));
    }
    return r;
}

// ---- VNAV ----------------------------------------------------------------------------------------

void Navigator::computeVnav(const AircraftState& ac, const Guide& g, NavPhase phase,
                            NavState& s) const {
    const int n = static_cast<int>(plan_.legs.size());
    const int a = plan_.activeLeg;
    if (phase == NavPhase::Missed || a < 0 || a >= n) return;

    double distToTargetNm = g.distToWptNm;
    double targetFt = 0;
    bool found = false;
    for (int i = a; i < n; ++i) {
        const Leg& l = plan_.legs[static_cast<size_t>(i)];
        if (l.missedApproach) break;
        if (i > a) distToTargetNm += cache_.legLenNm[static_cast<size_t>(i)];
        if (isAltLeg(l.type) || isHoldLeg(l.type)) continue;
        if (l.alt.kind == AltConstraintKind::At || l.alt.kind == AltConstraintKind::AtOrBelow) {
            targetFt = l.alt.altFt;
            found = true;
            break;
        }
        if (l.alt.kind == AltConstraintKind::Between) {
            targetFt = std::max(l.alt.altFt, l.alt.alt2Ft);
            found = true;
            break;
        }
    }
    if (!found) {
        // Destination threshold + 50 ft.
        if (cache_.arrRunway)
            targetFt = cache_.arrRunway->thrElevFt;
        else if (cache_.destAirport)
            targetFt = cache_.destAirport->elevFt;
        else
            return;
        targetFt += 50.0;
        distToTargetNm = g.distToWptNm + cache_.afterNm[static_cast<size_t>(a)];
    }
    s.targetAltFt = static_cast<float>(targetFt);
    const double altFt = ac.altMslFt;
    if (altFt <= targetFt + kVnavMinAboveFt) return;

    const double tanPath = std::tan(degToRad(NavConstants::kVnavPathDeg));
    const double pathAltFt = targetFt + nmToFt(distToTargetNm) * tanPath;
    const double devFt = pathAltFt - altFt;  // + = path above the aircraft
    const double todNm = distToTargetNm - ftToNm(altFt - targetFt) / tanPath;
    s.vnavValid = true;
    s.vnavDevFt = static_cast<float>(devFt);
    s.todDistNm = static_cast<float>(std::max(0.0, todNm));
    const double fpmPerKt = kFtPerNm / 60.0;  // 1 kt = 101.27 fpm
    s.requiredVsFpm = devFt <= 0.0
                          ? static_cast<float>(-static_cast<double>(ac.gsKt) * fpmPerKt * tanPath)
                          : 0.0f;
}

// ---- VOR (optional source) ---------------------------------------------------------------------

void Navigator::updateVor(const AircraftState& ac, NavState& s) const {
    const double brgToRad =
        initialBearingTrueRad(ac.latDeg, ac.lonDeg, vorPos_.latDeg, vorPos_.lonDeg);
    const double obsTrueDeg = wrap360(static_cast<double>(vorObsMagDeg_) + vorMagVarDeg_);
    const double brgToDeg = radToDeg(brgToRad);
    const double relDeg = wrap180(brgToDeg - obsTrueDeg);
    const bool to = std::fabs(relDeg) < 90.0;
    double devDeg;  // course-is-right positive
    if (to) {
        devDeg = relDeg;
    } else {
        const double radialDeg = wrap180(brgToDeg + 180.0 - obsTrueDeg);  // + = right of course
        devDeg = -radialDeg;
    }
    s.source = NavSource::Vor;
    s.toFlag = to;
    s.cdiDeflection = static_cast<float>(clampd(devDeg / NavConstants::kVorFullScaleDeg,
                                                -NavConstants::kCdiClamp, NavConstants::kCdiClamp));
    s.cdiFullScaleNm = 0;
    s.desiredTrackTrueDeg = static_cast<float>(obsTrueDeg);
    s.distToWptNm =
        static_cast<float>(distNm(ac.latDeg, ac.lonDeg, vorPos_.latDeg, vorPos_.lonDeg));
    s.crossTrackNm = static_cast<float>(-s.distToWptNm * std::sin(degToRad(devDeg)));
    s.bearingToWptTrueDeg = static_cast<float>(brgToDeg);
    s.wptLatDeg = vorPos_.latDeg;
    s.wptLonDeg = vorPos_.lonDeg;
    copyIdent(s.activeWptIdent, vorIdent_);
    s.eteToWptS =
        ac.gsKt > kMinGsForRatesKt ? static_cast<float>(eteS(s.distToWptNm, ac.gsKt)) : 0.0f;
}

void Navigator::fillIdent(char (&dst)[8], const Leg& leg) const {
    if (leg.type == LegType::CA || leg.type == LegType::VA || leg.type == LegType::FA)
        std::snprintf(dst, sizeof dst, "%dFT", static_cast<int>(std::lround(leg.alt.altFt)));
    else
        copyIdent(dst, leg.to.ident);
}

// ---- update ------------------------------------------------------------------------------------

const NavState& Navigator::update(const AircraftState& ac, double dtS) {
    if (!ac.valid) return state_;
    lastPos_ = LatLon{ac.latDeg, ac.lonDeg};
    haveLastPos_ = true;
    refreshCache();

    NavState s;
    s.magVarDeg = db_.magVarDeg(ac.latDeg, ac.lonDeg);
    const int n = static_cast<int>(plan_.legs.size());
    if (n == 0) {
        if (vorTuned_ && select_ != NavSourceSelect::Gps) updateVor(ac, s);
        state_ = s;
        return state_;
    }
    if (plan_.activeLeg < 0) plan_.activeLeg = n > 1 ? 1 : 0;
    if (plan_.activeLeg >= n) plan_.activeLeg = n - 1;
    if (entryPending_ || trackedLeg_ != plan_.activeLeg) {
        captureEntry(ac);
        trackedLeg_ = plan_.activeLeg;
        entryPending_ = false;
    }

    // Sequencing: at most a few legs per update (closely spaced fixes).
    Guide g = computeGuide(ac, plan_.activeLeg);
    for (int guard = 0; guard < 4 && shouldSequence(ac, g, plan_.activeLeg); ++guard) {
        ++plan_.activeLeg;
        captureEntry(ac);
        trackedLeg_ = plan_.activeLeg;
        g = computeGuide(ac, plan_.activeLeg);
    }
    const int a = plan_.activeLeg;
    const Leg& leg = plan_.legs[static_cast<size_t>(a)];
    const size_t ua = static_cast<size_t>(a);

    // Phase.
    const bool missed = leg.missedApproach;
    const bool inApproach = !missed && plan_.approachStartLeg >= 0 && a >= plan_.approachStartLeg;
    const double distOriginNm =
        cache_.haveOrigin ? distNm(ac.latDeg, ac.lonDeg, cache_.origin.latDeg, cache_.origin.lonDeg)
                          : 1e9;
    const double distDestNm =
        cache_.haveDest ? distNm(ac.latDeg, ac.lonDeg, cache_.dest.latDeg, cache_.dest.lonDeg)
                        : 1e9;
    NavPhase phase;
    if (missed)
        phase = NavPhase::Missed;
    else if (inApproach)
        phase = NavPhase::Approach;
    else if (a <= 1 && distOriginNm < kDepartureRadiusNm &&
             (ac.wowAny || ac.altAglFt < kDepartureMaxAglFt))
        phase = NavPhase::Departure;
    else if (std::min(distOriginNm, distDestNm) <= NavConstants::kTerminalRadiusNm)
        phase = NavPhase::Terminal;
    else
        phase = NavPhase::Enroute;
    s.phase = phase;

    // CDI scale with a linear 1 minute ramp.
    const float scaleTarget = targetScaleNm(g, phase);
    if (!scaleInit_) {
        scaleNm_ = scaleFromNm_ = scaleToNm_ = scaleTarget;
        scaleInit_ = true;
    } else {
        if (scaleTarget != scaleToNm_) {
            scaleFromNm_ = scaleNm_;
            scaleToNm_ = scaleTarget;
            rampS_ = 0;
        }
        rampS_ += static_cast<float>(std::max(dtS, 0.0));
        const float t = std::min(rampS_ / NavConstants::kScaleRampS, 1.0f);
        scaleNm_ = scaleFromNm_ + (scaleToNm_ - scaleFromNm_) * t;
    }

    // Final-course facts and LOC.
    const bool approachIls = cache_.ils != nullptr && phase != NavPhase::Missed;
    const bool onFinal = inApproach && cache_.haveFinalCourse &&
                         std::fabs(wrap180(radToDeg(g.dtkRad) - cache_.finalCourseTrueDeg)) <
                             kLocFinalCourseToleranceDeg;
    Angular loc;
    if (approachIls) loc = computeLoc(ac);
    const double trackRad = ac.gsKt > kMinGsForRatesKt ? ac.trackTrueRad : ac.headingTrueRad;
    bool useLoc = false;
    switch (select_) {
    case NavSourceSelect::Auto:
        if (!approachIls) {
            locLatched_ = false;
        } else if (locLatched_) {
            if (!loc.locValid) locLatched_ = false;
        } else if (onFinal && loc.locValid && std::fabs(loc.cdi) <= 1.0f &&
                   std::fabs(wrap180(radToDeg(trackRad) - cache_.ils->locCourseTrueDeg)) <
                       NavConstants::kLocCaptureInterceptDeg) {
            locLatched_ = true;
        }
        useLoc = locLatched_;
        break;
    case NavSourceSelect::Loc: useLoc = approachIls; break;
    case NavSourceSelect::Gps:
    case NavSourceSelect::Vor: break;
    }
    const bool useVor = select_ == NavSourceSelect::Vor && vorTuned_;

    // Waypoint-related outputs (always from the plan).
    fillIdent(s.activeWptIdent, leg);
    s.wptLatDeg = leg.to.latDeg;
    s.wptLonDeg = leg.to.lonDeg;
    if (a + 1 < n) {
        const Leg& next = plan_.legs[ua + 1];
        if (!next.missedApproach || missed) fillIdent(s.nextWptIdent, next);
    }
    s.activeLegIndex = a;
    s.distToWptNm = static_cast<float>(g.distToWptNm);
    s.bearingToWptTrueDeg = static_cast<float>(radToDeg(g.brgToWptRad));
    const double distAfterNm =
        phase == NavPhase::Missed ? cache_.afterAllNm[ua] : cache_.afterNm[ua];
    s.distToDestNm = static_cast<float>(g.distToWptNm + distAfterNm);
    if (ac.gsKt > kMinGsForRatesKt) {
        s.eteToWptS = static_cast<float>(eteS(s.distToWptNm, ac.gsKt));
        s.eteToDestS = static_cast<float>(eteS(s.distToDestNm, ac.gsKt));
    }
    s.onFinal = onFinal;

    // Approach information.
    const ApproachKind kind = approach_ ? approach_->kind : plan_.approachKind;
    if (kind != ApproachKind::None && plan_.approachStartLeg >= 0) {
        const char* rwy =
            cache_.arrRunway ? cache_.arrRunway->ident.c_str() : plan_.arrRunway.c_str();
        std::snprintf(s.approachName, sizeof s.approachName, "%s %s", kindName(kind), rwy);
        if (approach_) {
            s.minimumsFt = approach_->minimumsFt;
            s.minimumsIsDa = approach_->minimumsIsDa;
        }
    }
    if (cache_.arrRunway)
        s.distToThresholdNm = static_cast<float>(
            distNm(ac.latDeg, ac.lonDeg, cache_.arrRunway->thrLatDeg, cache_.arrRunway->thrLonDeg));
    s.locValid = approachIls && loc.locValid;

    // Lateral guidance.
    if (useVor) {
        updateVor(ac, s);
    } else if (useLoc) {
        s.source = NavSource::Loc;
        s.desiredTrackTrueDeg = static_cast<float>(wrap360(cache_.ils->locCourseTrueDeg));
        s.crossTrackNm = loc.xtkNm;
        s.cdiFullScaleNm = 0;
        s.toFlag = loc.toFlag;
        if (loc.locValid) {
            s.cdiDeflection = loc.cdi;
            s.gsValid = loc.gsValid;
            s.gsDeflection = loc.gsValid ? loc.gsDefl : 0.0f;
        }
    } else {
        s.source = NavSource::Gps;
        s.desiredTrackTrueDeg = static_cast<float>(radToDeg(g.dtkRad));
        s.crossTrackNm = static_cast<float>(g.xtkNm);
        s.cdiFullScaleNm = scaleNm_;
        s.cdiDeflection = static_cast<float>(
            clampd(-g.xtkNm / scaleNm_, -NavConstants::kCdiClamp, NavConstants::kCdiClamp));
        s.toFlag = g.altTerminated || g.remNm >= 0.0;
    }

    computeVnav(ac, g, phase, s);
    state_ = s;
    return state_;
}

}  // namespace fsim
