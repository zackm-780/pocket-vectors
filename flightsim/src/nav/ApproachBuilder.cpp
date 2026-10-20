#include "nav/ApproachBuilder.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <limits>

#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/Fms.h"

namespace fsim {
namespace {

constexpr double kIfDistNm = 10.0, kFafDistNm = 5.0, kVisFafDistNm = 3.0, kIafAbeamNm = 4.0;
constexpr float kIfAboveThrFt = 3200.0f, kFafAboveThrFt = 1600.0f, kVisFafAboveThrFt = 1000.0f;
constexpr float kMapAboveThrFt = 50.0f, kMissedCaAboveThrFt = 1500.0f;
constexpr double kMissedCaLegNm = 5.0;  // where the CA leg's nominal fix is drawn

float ceilTo(float value, float step) {
    return std::ceil(value / step) * step;
}

std::string upper(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

Waypoint synthWpt(const char* ident, double latDeg, double lonDeg) {
    Waypoint w;
    w.ident = ident;
    w.kind = WaypointKind::Synthetic;
    w.latDeg = latDeg;
    w.lonDeg = lonDeg;
    return w;
}

// Point `distNm` from (lat, lon) toward true bearing `bearingTrueDeg`.
LatLon pointAt(double latDeg, double lonDeg, double bearingTrueDeg, double distNm) {
    LatLon p;
    destinationPoint(latDeg, lonDeg, degToRad(bearingTrueDeg), nmToM(distNm), p.latDeg, p.lonDeg);
    return p;
}

AltConstraint altAt(float altFt) {
    AltConstraint a;
    a.kind = AltConstraintKind::At;
    a.altFt = altFt;
    return a;
}

Leg makeLeg(LegType type, const Waypoint& to, const AltConstraint& alt, bool missed = false) {
    Leg l;
    l.type = type;
    l.to = to;
    l.alt = alt;
    l.missedApproach = missed;
    return l;
}

std::string kindId(ApproachKind kind) {
    switch (kind) {
    case ApproachKind::Ils: return "I";
    case ApproachKind::Loc: return "L";
    case ApproachKind::Rnav: return "R";
    case ApproachKind::Vor: return "V";
    case ApproachKind::Ndb: return "N";
    default: return "";
    }
}

// Shared builder for ILS / LOC / RNAV / visual.
Approach build(const Airport& airport, const Runway& rwy, ApproachKind kind) {
    Approach a;
    a.airportIdent = airport.ident;
    a.runwayIdent = rwy.ident;
    a.kind = kind;
    a.synthesized = true;
    const bool visual = kind == ApproachKind::Visual;
    a.id = visual ? "VIS" + rwy.ident : kindId(kind) + rwy.ident;
    a.minimumsFt = defaultMinimumsFt(kind, rwy.thrElevFt, &a.minimumsIsDa);

    const bool useLoc = (kind == ApproachKind::Ils || kind == ApproachKind::Loc) && rwy.ils;
    const double courseTrueDeg = useLoc ? rwy.ils->locCourseTrueDeg : rwy.headingTrueDeg;
    const double backDeg = courseTrueDeg + 180.0;
    const float thrFt = rwy.thrElevFt;

    const double fafNm = visual ? kVisFafDistNm : kFafDistNm;
    const float fafAltFt = thrFt + (visual ? kVisFafAboveThrFt : kFafAboveThrFt);
    const float ifAltFt = ceilTo(thrFt + kIfAboveThrFt, 100.0f);

    const LatLon ifPt = pointAt(rwy.thrLatDeg, rwy.thrLonDeg, backDeg, kIfDistNm);
    const LatLon fafPt = pointAt(rwy.thrLatDeg, rwy.thrLonDeg, backDeg, fafNm);
    const Waypoint ifWpt = synthWpt("IF", ifPt.latDeg, ifPt.lonDeg);
    const Waypoint fafWpt = synthWpt("FAF", fafPt.latDeg, fafPt.lonDeg);

    Waypoint mapWpt;
    mapWpt.ident = dbRunwayToFms(rwy.ident);  // "RW27"
    mapWpt.kind = WaypointKind::Runway;
    mapWpt.latDeg = rwy.thrLatDeg;
    mapWpt.lonDeg = rwy.thrLonDeg;

    if (!visual) {
        a.finalLegs.push_back(makeLeg(LegType::IF, ifWpt, altAt(ifAltFt)));
        a.finalLegs.push_back(makeLeg(LegType::TF, fafWpt, altAt(fafAltFt)));
    } else {
        a.finalLegs.push_back(makeLeg(LegType::IF, fafWpt, altAt(fafAltFt)));
    }
    Leg map = makeLeg(LegType::TF, mapWpt, altAt(thrFt + kMapAboveThrFt));
    map.flyOver = true;
    a.finalLegs.push_back(map);

    // Missed approach: CA on the final course, DF to the IF, hold at the IF.
    const LatLon caPt = pointAt(rwy.thrLatDeg, rwy.thrLonDeg, courseTrueDeg, kMissedCaLegNm);
    AltConstraint caAlt;
    caAlt.kind = AltConstraintKind::AtOrAbove;
    caAlt.altFt = thrFt + kMissedCaAboveThrFt;
    Leg ca = makeLeg(LegType::CA, synthWpt("CA", caPt.latDeg, caPt.lonDeg), caAlt, true);
    ca.courseTrueDeg = static_cast<float>(wrap360(courseTrueDeg));
    a.missedLegs.push_back(ca);
    if (!visual) {
        a.missedLegs.push_back(makeLeg(LegType::DF, ifWpt, altAt(ifAltFt), true));
        Leg hm = makeLeg(LegType::HM, ifWpt, altAt(ifAltFt), true);
        hm.courseTrueDeg = static_cast<float>(wrap360(courseTrueDeg));  // inbound course
        hm.arcRight = true;                                             // right turns
        a.missedLegs.push_back(hm);
    }

    // Transitions: vectors, and (non-visual) an IAF 4 NM abeam the IF on either side.
    a.transitions.push_back(ApproachTransition{});
    if (!visual) {
        const char* names[2] = {"IAF-L", "IAF-R"};
        const double sideDeg[2] = {courseTrueDeg - 90.0, courseTrueDeg + 90.0};
        for (int i = 0; i < 2; ++i) {
            const LatLon iaf = pointAt(ifPt.latDeg, ifPt.lonDeg, sideDeg[i], kIafAbeamNm);
            ApproachTransition t;
            t.name = names[i];
            t.legs.push_back(
                makeLeg(LegType::TF, synthWpt("IAF", iaf.latDeg, iaf.lonDeg), altAt(ifAltFt)));
            a.transitions.push_back(std::move(t));
        }
    }
    return a;
}

}  // namespace

float defaultMinimumsFt(ApproachKind kind, float thrElevFt, bool* isDa) {
    if (isDa) *isDa = kind == ApproachKind::Ils;
    switch (kind) {
    case ApproachKind::Ils: return thrElevFt + 200.0f;
    case ApproachKind::Loc:
    case ApproachKind::Rnav: return ceilTo(thrElevFt + 400.0f, 10.0f);
    case ApproachKind::Vor:
    case ApproachKind::Ndb: return ceilTo(thrElevFt + 500.0f, 10.0f);
    default: return 0.0f;
    }
}

Approach buildIlsApproach(const Airport& airport, const Runway& runway) {
    if (!runway.ils) {
        Approach none;
        none.airportIdent = airport.ident;
        none.runwayIdent = runway.ident;
        return none;
    }
    return build(airport, runway, runway.ils->hasGs ? ApproachKind::Ils : ApproachKind::Loc);
}

Approach buildRnavApproach(const Airport& airport, const Runway& runway) {
    return build(airport, runway, ApproachKind::Rnav);
}

Approach buildVisualApproach(const Airport& airport, const Runway& runway) {
    return build(airport, runway, ApproachKind::Visual);
}

std::vector<Approach> availableApproaches(const Airport& airport, const Runway& runway,
                                          const RealApproachProvider& realProvider) {
    std::vector<Approach> out;
    if (realProvider) out = realProvider(airport, runway);
    auto have = [&](const std::string& id) {
        for (const Approach& a : out)
            if (upper(a.id) == upper(id)) return true;
        return false;
    };
    auto add = [&](Approach a) {
        if (a.kind != ApproachKind::None && !have(a.id)) out.push_back(std::move(a));
    };
    if (runway.ils) add(buildIlsApproach(airport, runway));
    add(buildRnavApproach(airport, runway));
    add(buildVisualApproach(airport, runway));
    return out;
}

std::vector<Approach> availableApproaches(const NavDatabase& db, std::string_view airportIdent,
                                          std::string_view runwayIdent,
                                          const RealApproachProvider& realProvider) {
    const Airport* airport = db.findAirport(airportIdent);
    if (!airport) return {};
    const Runway* runway = findRunway(*airport, runwayIdent);
    if (!runway) return {};
    return availableApproaches(*airport, *runway, realProvider);
}

std::optional<Approach> findApproach(const NavDatabase& db, std::string_view airportIdent,
                                     std::string_view runwayIdent, std::string_view approachId,
                                     const RealApproachProvider& realProvider) {
    const std::string want = upper(approachId);
    for (Approach& a : availableApproaches(db, airportIdent, runwayIdent, realProvider))
        if (upper(a.id) == want) return std::move(a);
    return std::nullopt;
}

std::string runwayFromApproachId(std::string_view approachId) {
    std::string id = upper(approachId);
    size_t pos = 0;
    if (id.compare(0, 3, "VIS") == 0)
        pos = 3;
    else if (!id.empty() && std::isalpha(static_cast<unsigned char>(id[0])))
        pos = 1;
    size_t i = pos;
    while (i < id.size() && std::isdigit(static_cast<unsigned char>(id[i]))) ++i;
    if (i == pos) return {};
    std::string rwy = id.substr(pos, i - pos);
    if (rwy.size() == 1) rwy = "0" + rwy;
    if (i < id.size() && (id[i] == 'L' || id[i] == 'R' || id[i] == 'C')) rwy += id[i];
    return rwy;
}

float inboundCourseTrueDeg(const Approach& a) {
    if (a.finalLegs.size() >= 2) {
        const Leg& first = a.finalLegs.front();
        const Leg& last = a.finalLegs.back();
        // Course arriving at the last fix (the runway heading at a threshold MAP): the reverse of
        // the bearing from there back to the first fix, which avoids meridian-convergence error.
        return static_cast<float>(
            wrap360(180.0 + radToDeg(initialBearingTrueRad(last.to.latDeg, last.to.lonDeg,
                                                           first.to.latDeg, first.to.lonDeg))));
    }
    for (const Leg& l : a.finalLegs)
        if (l.type == LegType::CF || l.type == LegType::CA) return l.courseTrueDeg;
    return 0.0f;
}

int selectTransition(const Approach& a, const LatLon& prev) {
    if (a.transitions.size() <= 1 || a.finalLegs.size() < 2) return 0;
    const Waypoint& target = a.finalLegs.front().to;
    const double toTargetDeg =
        radToDeg(initialBearingTrueRad(prev.latDeg, prev.lonDeg, target.latDeg, target.lonDeg));
    const double diffDeg = std::fabs(wrap180(toTargetDeg - inboundCourseTrueDeg(a)));
    if (diffDeg <= 90.0) return 0;
    int best = 0;
    double bestM = std::numeric_limits<double>::max();
    for (size_t i = 1; i < a.transitions.size(); ++i) {
        if (a.transitions[i].legs.empty()) continue;
        const Waypoint& w = a.transitions[i].legs.front().to;
        const double d = greatCircleDistM(prev.latDeg, prev.lonDeg, w.latDeg, w.lonDeg);
        if (d < bestM) {
            bestM = d;
            best = static_cast<int>(i);
        }
    }
    return best;
}

std::vector<Leg> approachPlanLegs(const Approach& a, int transitionIndex) {
    std::vector<Leg> legs;
    if (transitionIndex >= 0 && transitionIndex < static_cast<int>(a.transitions.size())) {
        for (Leg l : a.transitions[static_cast<size_t>(transitionIndex)].legs) {
            l.missedApproach = false;
            legs.push_back(std::move(l));
        }
    }
    for (Leg l : a.finalLegs) {
        l.missedApproach = false;
        legs.push_back(std::move(l));
    }
    for (Leg l : a.missedLegs) {
        l.missedApproach = true;
        legs.push_back(std::move(l));
    }
    return legs;
}

}  // namespace fsim
