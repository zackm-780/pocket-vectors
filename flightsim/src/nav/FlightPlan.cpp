#include "nav/FlightPlan.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <sstream>

#include "core/Log.h"
#include "core/SyntheticNavDb.h"  // findRunway
#include "core/Units.h"

namespace fsim {
namespace {

double distNmBetween(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    return mToNm(greatCircleDistM(lat1Deg, lon1Deg, lat2Deg, lon2Deg));
}

bool isOriginKind(WaypointKind k) {
    return k == WaypointKind::Airport || k == WaypointKind::Runway || k == WaypointKind::Synthetic;
}

bool hasCoords(const FmsEntry& e) {
    return !(e.latDeg == 0.0 && e.lonDeg == 0.0);
}

Waypoint airportWaypoint(const Airport& a) {
    Waypoint wp;
    wp.ident = a.ident;
    wp.kind = WaypointKind::Airport;
    wp.latDeg = a.latDeg;
    wp.lonDeg = a.lonDeg;
    return wp;
}

Waypoint navaidWaypoint(const Navaid& n) {
    Waypoint wp;
    wp.ident = n.ident;
    wp.kind = (n.type == NavaidType::Ndb || n.type == NavaidType::NdbDme) ? WaypointKind::Ndb
                                                                          : WaypointKind::Vor;
    wp.latDeg = n.latDeg;
    wp.lonDeg = n.lonDeg;
    return wp;
}

bool isVorLike(NavaidType t) {
    return t == NavaidType::Vor || t == NavaidType::VorDme || t == NavaidType::Vortac ||
           t == NavaidType::Tacan;
}

// Runway as a waypoint ("RW28L"), at the pavement start (departure) or the landing threshold.
Waypoint runwayWaypoint(const Runway& r, bool threshold) {
    Waypoint wp;
    wp.ident = dbRunwayToFms(r.ident);
    wp.kind = WaypointKind::Runway;
    wp.latDeg = threshold ? r.thrLatDeg : r.startLatDeg;
    wp.lonDeg = threshold ? r.thrLonDeg : r.startLonDeg;
    return wp;
}

// Origin or destination waypoint: runway if it resolves, else the airport.
bool endpointWaypoint(const NavDatabase& db, const std::string& airportIdent,
                      const std::string& runwayDb, bool threshold, Waypoint& out,
                      std::vector<std::string>* warnings) {
    const Airport* a = db.findAirport(airportIdent);
    if (!a) return false;
    if (!runwayDb.empty()) {
        if (const Runway* r = findRunway(*a, runwayDb)) {
            out = runwayWaypoint(*r, threshold);
            return true;
        }
        if (warnings)
            warnings->push_back("runway " + runwayDb + " not found at " + airportIdent +
                                "; using the airport position");
    }
    out = airportWaypoint(*a);
    return true;
}

Leg makeLeg(LegType type, const Waypoint& wp) {
    Leg l;
    l.type = type;
    l.to = wp;
    return l;
}

// Position the leg starts from: the `to` of the nearest previous leg that has a fix.
bool legStart(const FlightPlan& plan, int index, double& latDeg, double& lonDeg) {
    for (int j = index - 1; j >= 0; --j) {
        const Leg& p = plan.legs[static_cast<size_t>(j)];
        if (p.type == LegType::CA || p.type == LegType::VA) continue;
        latDeg = p.to.latDeg;
        lonDeg = p.to.lonDeg;
        return true;
    }
    return false;
}

double arcLengthNm(const Leg& leg, double fromLatDeg, double fromLonDeg) {
    double b0 =
        initialBearingTrueRad(leg.arcCenterLatDeg, leg.arcCenterLonDeg, fromLatDeg, fromLonDeg);
    double b1 = initialBearingTrueRad(leg.arcCenterLatDeg, leg.arcCenterLonDeg, leg.to.latDeg,
                                      leg.to.lonDeg);
    double sweepRad = leg.arcRight ? wrap2Pi(b1 - b0) : wrap2Pi(b0 - b1);
    return static_cast<double>(leg.arcRadiusNm) * sweepRad;
}

// Index of the destination leg of a plan without an approach, or -1.
int destLegIndex(const FlightPlan& plan) {
    if (plan.approachStartLeg >= 0 || plan.legs.size() < 2) return -1;
    const Leg& last = plan.legs.back();
    if (last.to.kind == WaypointKind::Airport || last.to.kind == WaypointKind::Runway)
        return static_cast<int>(plan.legs.size()) - 1;
    return -1;
}

std::string upperStr(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

struct Candidate {
    Waypoint wp;
};

}  // namespace

// ---- ident resolution ------------------------------------------------------------------------

std::vector<IdentMatch> resolveIdent(const NavDatabase& db, std::string_view ident,
                                     double refLatDeg, double refLonDeg) {
    std::vector<IdentMatch> out;
    std::string id = upperStr(ident);
    auto add = [&](const Waypoint& wp, std::string desc) {
        IdentMatch m;
        m.wp = wp;
        m.distNm = distNmBetween(refLatDeg, refLonDeg, wp.latDeg, wp.lonDeg);
        m.description = std::move(desc);
        out.push_back(std::move(m));
    };
    if (const Airport* a = db.findAirport(id)) add(airportWaypoint(*a), "Airport " + a->name);
    for (const Navaid* n : db.findNavaids(id)) {
        add(navaidWaypoint(*n), std::string(isVorLike(n->type) ? "VOR " : "NDB ") + n->name);
    }
    for (const Waypoint& f : db.findFixes(id)) add(f, "Fix");
    std::stable_sort(out.begin(), out.end(),
                     [](const IdentMatch& a, const IdentMatch& b) { return a.distNm < b.distNm; });
    return out;
}

ApproachKind approachKindFromId(std::string_view id) {
    if (id.empty()) return ApproachKind::None;
    switch (std::toupper(static_cast<unsigned char>(id[0]))) {
    case 'I': return ApproachKind::Ils;
    case 'L':
    case 'B':
    case 'X':
    case 'U': return ApproachKind::Loc;
    case 'R':
    case 'H':
    case 'P':
    case 'J':
    case 'G': return ApproachKind::Rnav;
    case 'V':
    case 'S':
    case 'D':
    case 'T': return ApproachKind::Vor;
    case 'N':
    case 'Q': return ApproachKind::Ndb;
    default: return ApproachKind::None;
    }
}

// ---- .fms -> plan ----------------------------------------------------------------------------

namespace {

// Resolves one file entry to a waypoint. Returns false (with an error) if it cannot be placed.
bool resolveEntry(const FmsEntry& e, const NavDatabase& db, double prevLatDeg, double prevLonDeg,
                  Waypoint& out, std::vector<std::string>& warnings,
                  std::vector<std::string>& errors) {
    constexpr double kFileWinsNm = 25.0;  // single DB match this far from the file position
    constexpr double kSnapNm = 2.0;       // ambiguous: snap to a candidate this close to the file
    const std::string where = "line " + std::to_string(e.line) + ": ";
    const bool fileOk = hasCoords(e);

    if (e.type == FmsEntryType::LatLon) {
        double lat = e.latDeg, lon = e.lonDeg;
        if (!fileOk && !decodeLatLonIdent(e.ident, lat, lon)) {
            errors.push_back(where + "lat/lon waypoint '" + e.ident + "' has no usable position");
            return false;
        }
        out = Waypoint{e.ident, WaypointKind::LatLon, lat, lon};
        return true;
    }

    std::vector<Waypoint> cands;
    WaypointKind kind = WaypointKind::Fix;
    if (e.type == FmsEntryType::Airport) {
        kind = WaypointKind::Airport;
        if (const Airport* a = db.findAirport(e.ident)) cands.push_back(airportWaypoint(*a));
    } else if (e.type == FmsEntryType::Vor || e.type == FmsEntryType::Ndb) {
        const bool wantNdb = e.type == FmsEntryType::Ndb;
        kind = wantNdb ? WaypointKind::Ndb : WaypointKind::Vor;
        for (const Navaid* n : db.findNavaids(e.ident)) {
            bool isNdb = n->type == NavaidType::Ndb || n->type == NavaidType::NdbDme;
            if (isNdb == wantNdb) cands.push_back(navaidWaypoint(*n));
        }
    } else {
        kind = WaypointKind::Fix;
        cands = db.findFixes(e.ident);
    }

    auto fileWaypoint = [&]() { return Waypoint{e.ident, kind, e.latDeg, e.lonDeg}; };
    if (cands.empty()) {
        if (!fileOk) {
            errors.push_back(where + "unknown " + e.ident + " and the file gives no coordinates");
            return false;
        }
        warnings.push_back(where + e.ident +
                           " not in the navigation database; using the file's "
                           "coordinates");
        out = fileWaypoint();
        return true;
    }
    if (cands.size() == 1) {
        out = cands[0];
        out.kind = kind == WaypointKind::Fix ? cands[0].kind : kind;
        if (fileOk && distNmBetween(e.latDeg, e.lonDeg, out.latDeg, out.lonDeg) > kFileWinsNm) {
            warnings.push_back(where + e.ident +
                               " in the database is far from the file's "
                               "position; using the file's coordinates");
            out = fileWaypoint();
        }
        return true;
    }
    // Ambiguous ident.
    size_t best = 0;
    double bestNm = 1e18;
    const double refLat = fileOk ? e.latDeg : prevLatDeg;
    const double refLon = fileOk ? e.lonDeg : prevLonDeg;
    for (size_t i = 0; i < cands.size(); ++i) {
        double d = distNmBetween(refLat, refLon, cands[i].latDeg, cands[i].lonDeg);
        if (d < bestNm) {
            bestNm = d;
            best = i;
        }
    }
    if (fileOk && bestNm > kSnapNm) {
        warnings.push_back(where + e.ident + " is ambiguous and matches no candidate within " +
                           "2 NM; using the file's coordinates");
        out = fileWaypoint();
        return true;
    }
    out = cands[best];
    out.kind = kind == WaypointKind::Fix ? cands[best].kind : kind;
    char buf[160];
    std::snprintf(buf, sizeof buf,
                  "%s%s is ambiguous (%zu matches); using the one %.0f NM from the %s",
                  where.c_str(), e.ident.c_str(), cands.size(), bestNm,
                  fileOk ? "file position" : "previous waypoint");
    warnings.push_back(buf);
    return true;
}

}  // namespace

PlanBuildResult buildPlanFromFms(const FmsFile& file, const NavDatabase& db) {
    PlanBuildResult res;
    FlightPlan& plan = res.plan;
    if (file.entries.size() < 2) {
        res.errors.push_back("a plan needs at least an origin and a destination waypoint");
        return res;
    }
    const FmsEntry& first = file.entries.front();
    const FmsEntry& last = file.entries.back();
    plan.originIdent =
        !file.adep.empty() ? file.adep : (!file.dep.empty() ? file.dep : first.ident);
    plan.destIdent = !file.ades.empty() ? file.ades : (!file.des.empty() ? file.des : last.ident);
    plan.depRunway = fmsRunwayToDb(file.deprwy);
    plan.arrRunway = fmsRunwayToDb(file.desrwy);
    if (!file.adep.empty() && first.ident != file.adep)
        res.warnings.push_back("line " + std::to_string(first.line) + ": first waypoint " +
                               first.ident + " differs from ADEP " + file.adep);
    if (!file.ades.empty() && last.ident != file.ades)
        res.warnings.push_back("line " + std::to_string(last.line) + ": last waypoint " +
                               last.ident + " differs from ADES " + file.ades);
    plan.approachId = file.app;
    plan.approachKind = approachKindFromId(file.app);
    if (!file.app.empty() && plan.arrRunway.empty() && plan.approachKind != ApproachKind::None)
        res.warnings.push_back("APP " + file.app + " given without DESRWY");

    double prevLat = first.latDeg, prevLon = first.lonDeg;
    float maxAltFt = 0;
    for (size_t i = 0; i < file.entries.size(); ++i) {
        const FmsEntry& e = file.entries[i];
        Waypoint wp;
        if (!resolveEntry(e, db, prevLat, prevLon, wp, res.warnings, res.errors)) continue;
        prevLat = wp.latDeg;
        prevLon = wp.lonDeg;
        const bool isFirst = i == 0, isLast = i + 1 == file.entries.size();
        if (isFirst || isLast) {
            const std::string& rwy = isFirst ? plan.depRunway : plan.arrRunway;
            if (!rwy.empty() && wp.kind == WaypointKind::Airport) {
                if (const Airport* a = db.findAirport(wp.ident)) {
                    if (const Runway* r = findRunway(*a, rwy))
                        wp = runwayWaypoint(*r, !isFirst);
                    else
                        res.warnings.push_back("runway " + rwy + " not found at " + wp.ident +
                                               "; using the airport position");
                }
            }
            plan.legs.push_back(makeLeg(isFirst ? LegType::IF : LegType::TF, wp));
            continue;
        }
        Leg leg = makeLeg(LegType::TF, wp);
        if (e.altFt > 0 && e.type != FmsEntryType::Airport) {
            leg.alt.kind = AltConstraintKind::At;
            leg.alt.altFt = e.altFt;
            maxAltFt = std::max(maxAltFt, e.altFt);
        }
        plan.legs.push_back(std::move(leg));
    }
    plan.cruiseAltFt = maxAltFt;
    res.ok = res.errors.empty();
    if (!res.ok) plan.legs.clear();
    return res;
}

PlanBuildResult loadPlanFromFmsText(std::string_view text, const NavDatabase& db,
                                    FmsFile* fileOut) {
    FmsParseResult parsed = parseFms(text);
    PlanBuildResult res;
    if (!parsed.ok) {
        res.errors = parsed.errors;
        res.warnings = parsed.warnings;
        return res;
    }
    res = buildPlanFromFms(parsed.file, db);
    res.warnings.insert(res.warnings.begin(), parsed.warnings.begin(), parsed.warnings.end());
    if (fileOut) *fileOut = parsed.file;
    return res;
}

PlanBuildResult loadPlanFromFmsFile(const std::string& path, const NavDatabase& db,
                                    FmsFile* fileOut) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        PlanBuildResult res;
        res.errors.push_back("cannot open " + path);
        return res;
    }
    std::stringstream ss;
    ss << in.rdbuf();
    PlanBuildResult res = loadPlanFromFmsText(ss.str(), db, fileOut);
    for (auto& e : res.errors) e = path + ": " + e;
    return res;
}

// ---- plan -> .fms ----------------------------------------------------------------------------

FmsFile planToFms(const FlightPlan& plan, const NavDatabase& db, const FmsFile* tmpl) {
    FmsFile f;
    if (tmpl) {
        f.cycle = tmpl->cycle;
        f.sid = tmpl->sid;
        f.sidTrans = tmpl->sidTrans;
        f.star = tmpl->star;
        f.starTrans = tmpl->starTrans;
        f.appTrans = tmpl->appTrans;
    }
    f.adep = plan.originIdent;
    f.ades = plan.destIdent;
    f.deprwy = plan.depRunway.empty() ? "" : dbRunwayToFms(plan.depRunway);
    f.desrwy = plan.arrRunway.empty() ? "" : dbRunwayToFms(plan.arrRunway);
    f.app = plan.approachId;

    auto airportEntry = [&](const std::string& ident, const char* via,
                            const Waypoint* fallback) -> bool {
        FmsEntry e;
        e.type = FmsEntryType::Airport;
        e.ident = ident;
        e.via = via;
        if (const Airport* a = db.findAirport(ident)) {
            e.latDeg = a->latDeg;
            e.lonDeg = a->lonDeg;
            e.altFt = a->elevFt;
        } else if (fallback) {
            e.latDeg = fallback->latDeg;
            e.lonDeg = fallback->lonDeg;
        } else {
            return false;
        }
        f.entries.push_back(std::move(e));
        return true;
    };

    const int endIdx = enrouteEndIndex(plan);
    int start = 0;
    if (!plan.legs.empty() && isOriginKind(plan.legs[0].to.kind)) {
        if (!plan.originIdent.empty()) airportEntry(plan.originIdent, "ADEP", &plan.legs[0].to);
        start = 1;
    }
    for (int i = start; i < endIdx; ++i) {
        const Leg& l = plan.legs[static_cast<size_t>(i)];
        FmsEntry e;
        e.ident = l.to.ident;
        e.latDeg = l.to.latDeg;
        e.lonDeg = l.to.lonDeg;
        e.altFt = l.alt.kind == AltConstraintKind::None ? 0.0f : l.alt.altFt;
        switch (l.to.kind) {
        case WaypointKind::Airport:
            e.type = FmsEntryType::Airport;
            if (const Airport* a = db.findAirport(l.to.ident)) e.altFt = a->elevFt;
            break;
        case WaypointKind::Vor: e.type = FmsEntryType::Vor; break;
        case WaypointKind::Ndb: e.type = FmsEntryType::Ndb; break;
        case WaypointKind::Fix: e.type = FmsEntryType::Fix; break;
        case WaypointKind::LatLon:
            e.type = FmsEntryType::LatLon;
            if (e.ident.empty()) e.ident = encodeLatLonIdent(e.latDeg, e.lonDeg);
            break;
        case WaypointKind::Runway:
        case WaypointKind::Synthetic: continue;  // not representable in the file
        }
        f.entries.push_back(std::move(e));
    }
    if (!plan.destIdent.empty()) {
        const Waypoint* fb = plan.legs.empty() ? nullptr : &plan.legs.back().to;
        if (plan.approachStartLeg >= 0) fb = nullptr;
        airportEntry(plan.destIdent, "ADES", fb);
    }
    return f;
}

std::string writePlanFmsText(const FlightPlan& plan, const NavDatabase& db,
                             const FmsFile* headerTemplate) {
    return writeFms(planToFms(plan, db, headerTemplate));
}

bool writePlanFmsFile(const std::string& path, const FlightPlan& plan, const NavDatabase& db,
                      const FmsFile* headerTemplate, std::string* error) {
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    if (!out) {
        if (error) *error = "cannot write " + path;
        return false;
    }
    out << writePlanFmsText(plan, db, headerTemplate);
    if (!out) {
        if (error) *error = "write failed for " + path;
        return false;
    }
    return true;
}

// ---- quick plan ------------------------------------------------------------------------------

QuickPlanResult buildQuickPlan(const QuickPlanRequest& req, const NavDatabase& db) {
    QuickPlanResult res;
    FlightPlan& plan = res.plan;
    const Airport* dest = db.findAirport(req.destIdent);
    if (!dest) {
        res.error = "unknown destination airport '" + req.destIdent + "'";
        return res;
    }
    Waypoint originWp;
    if (!req.originIdent.empty()) {
        if (!endpointWaypoint(db, req.originIdent, req.originRunway, false, originWp,
                              &res.warnings)) {
            res.error = "unknown origin airport '" + req.originIdent + "'";
            return res;
        }
        plan.originIdent = req.originIdent;
        if (originWp.kind == WaypointKind::Runway) plan.depRunway = req.originRunway;
    } else if (req.presentPos) {
        originWp = Waypoint{"PPOS", WaypointKind::Synthetic, req.presentPos->latDeg,
                            req.presentPos->lonDeg};
    } else {
        res.error = "no origin airport and no present position";
        return res;
    }
    Waypoint destWp;
    endpointWaypoint(db, req.destIdent, req.destRunway, true, destWp, &res.warnings);
    plan.destIdent = req.destIdent;
    if (destWp.kind == WaypointKind::Runway) plan.arrRunway = req.destRunway;
    plan.cruiseAltFt = req.cruiseAltFt;

    plan.legs.push_back(makeLeg(LegType::IF, originWp));

    if (req.viaNavaids) {
        if (!req.navaidsNear) {
            res.warnings.push_back(
                "via-navaids mode needs a navaid search provider; flying direct");
        } else {
            const double totalNm =
                distNmBetween(originWp.latDeg, originWp.lonDeg, destWp.latDeg, destWp.lonDeg);
            const LatLon a{originWp.latDeg, originWp.lonDeg}, b{destWp.latDeg, destWp.lonDeg};
            // Gather VORs near the great circle by sampling it.
            std::vector<const Navaid*> found;
            const double stepNm = 60.0;
            const int samples = std::max(1, static_cast<int>(std::ceil(totalNm / stepNm)));
            for (int s = 0; s <= samples; ++s) {
                double la, lo;
                destinationPoint(a.latDeg, a.lonDeg,
                                 initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg),
                                 nmToM(totalNm * s / samples), la, lo);
                for (const Navaid* n : req.navaidsNear(
                         la, lo,
                         static_cast<float>(totalNm / samples * 0.5 + req.viaMaxOffsetNm + 1)))
                    if (n && isVorLike(n->type) &&
                        std::find(found.begin(), found.end(), n) == found.end())
                        found.push_back(n);
            }
            struct Along {
                const Navaid* n;
                double alongNm;
            };
            std::vector<Along> good;
            for (const Navaid* n : found) {
                LatLon p{n->latDeg, n->lonDeg};
                double xt = std::fabs(mToNm(crossTrackM(p, a, b)));
                double at = mToNm(alongTrackM(p, a, b));
                if (xt <= req.viaMaxOffsetNm && at >= 10.0 && at <= totalNm - 10.0)
                    good.push_back({n, at});
            }
            double curNm = 0;
            while (totalNm - curNm > req.viaMaxLegNm) {
                const Along* pick = nullptr;
                for (const Along& g : good)
                    if (g.alongNm > curNm + 20.0 && g.alongNm - curNm <= req.viaMaxLegNm &&
                        (!pick || g.alongNm > pick->alongNm))
                        pick = &g;
                if (!pick) {
                    res.warnings.push_back("no VOR within range for a leg; skipping ahead");
                    curNm += req.viaMaxLegNm;
                    continue;
                }
                Leg l = makeLeg(LegType::TF, navaidWaypoint(*pick->n));
                if (req.cruiseAltFt > 0) {
                    l.alt.kind = AltConstraintKind::At;
                    l.alt.altFt = req.cruiseAltFt;
                }
                plan.legs.push_back(std::move(l));
                curNm = pick->alongNm;
            }
        }
    }
    plan.legs.push_back(makeLeg(LegType::TF, destWp));
    res.ok = true;
    return res;
}

// ---- editing ---------------------------------------------------------------------------------

int enrouteEndIndex(const FlightPlan& plan) {
    if (plan.approachStartLeg >= 0)
        return std::min(plan.approachStartLeg, static_cast<int>(plan.legs.size()));
    int d = destLegIndex(plan);
    return d >= 0 ? d : static_cast<int>(plan.legs.size());
}

bool insertWaypoint(FlightPlan& plan, int index, const Waypoint& wp, const AltConstraint& alt) {
    const int minIdx = plan.legs.empty() ? 0 : 1;
    if (index < minIdx || index > enrouteEndIndex(plan)) return false;
    Leg l = makeLeg(LegType::TF, wp);
    l.alt = alt;
    plan.legs.insert(plan.legs.begin() + index, std::move(l));
    if (plan.activeLeg >= index) ++plan.activeLeg;
    if (plan.approachStartLeg >= index) ++plan.approachStartLeg;
    return true;
}

bool deleteLeg(FlightPlan& plan, int index) {
    if (index < 1 || index >= enrouteEndIndex(plan)) return false;
    plan.legs.erase(plan.legs.begin() + index);
    if (plan.activeLeg > index) --plan.activeLeg;
    if (plan.approachStartLeg > index) --plan.approachStartLeg;
    if (plan.activeLeg >= static_cast<int>(plan.legs.size()))
        plan.activeLeg = static_cast<int>(plan.legs.size()) - 1;
    return true;
}

bool setAltConstraint(FlightPlan& plan, int index, const AltConstraint& alt) {
    if (index < 0 || index >= static_cast<int>(plan.legs.size())) return false;
    plan.legs[static_cast<size_t>(index)].alt = alt;
    return true;
}

namespace {
Leg pposLeg(const LatLon& present) {
    return makeLeg(LegType::IF,
                   Waypoint{"PPOS", WaypointKind::Synthetic, present.latDeg, present.lonDeg});
}
}  // namespace

bool directToLeg(FlightPlan& plan, int legIndex, const LatLon& present) {
    if (legIndex < 1 || legIndex >= static_cast<int>(plan.legs.size())) return false;
    const Leg& target = plan.legs[static_cast<size_t>(legIndex)];
    if (target.type == LegType::CA || target.type == LegType::VA) return false;
    std::vector<Leg> legs;
    legs.push_back(pposLeg(present));
    Leg df = target;
    df.type = LegType::DF;
    legs.push_back(std::move(df));
    legs.insert(legs.end(), plan.legs.begin() + legIndex + 1, plan.legs.end());
    if (plan.approachStartLeg >= 0)
        plan.approachStartLeg =
            plan.approachStartLeg > legIndex ? plan.approachStartLeg - legIndex + 1 : 1;
    plan.legs = std::move(legs);
    plan.activeLeg = 1;
    return true;
}

void directToWaypoint(FlightPlan& plan, const Waypoint& wp, const LatLon& present) {
    const int tail = enrouteEndIndex(plan);
    std::vector<Leg> legs;
    legs.push_back(pposLeg(present));
    legs.push_back(makeLeg(LegType::DF, wp));
    if (tail < static_cast<int>(plan.legs.size()))
        legs.insert(legs.end(), plan.legs.begin() + tail, plan.legs.end());
    if (plan.approachStartLeg >= 0) plan.approachStartLeg = 2;
    plan.legs = std::move(legs);
    plan.activeLeg = 1;
}

bool directToIdent(FlightPlan& plan, std::string_view ident, const NavDatabase& db,
                   const LatLon& present) {
    const std::string id = upperStr(ident);
    for (int i = 1; i < static_cast<int>(plan.legs.size()); ++i)
        if (plan.legs[static_cast<size_t>(i)].to.ident == id) return directToLeg(plan, i, present);
    auto m = resolveIdent(db, id, present.latDeg, present.lonDeg);
    if (m.empty()) return false;
    directToWaypoint(plan, m.front().wp, present);
    return true;
}

// ---- approach insertion ----------------------------------------------------------------------

bool insertApproach(FlightPlan& plan, std::string_view approachId, ApproachKind kind,
                    std::string_view arrRunway, std::vector<Leg> approachLegs, std::string* error) {
    auto fail = [&](const char* msg) {
        if (error) *error = msg;
        return false;
    };
    if (approachLegs.empty()) return fail("approach has no legs");
    bool seenMissed = false;
    for (const Leg& l : approachLegs) {
        if (l.missedApproach)
            seenMissed = true;
        else if (seenMissed)
            return fail("missed-approach legs must come last");
    }
    if (approachLegs.front().missedApproach) return fail("approach has no non-missed legs");

    const int keep = enrouteEndIndex(plan);
    const bool activeInTail = plan.activeLeg >= keep;
    plan.legs.resize(static_cast<size_t>(keep));
    plan.approachStartLeg = keep;
    plan.legs.insert(plan.legs.end(), std::make_move_iterator(approachLegs.begin()),
                     std::make_move_iterator(approachLegs.end()));
    plan.approachId = std::string(approachId);
    plan.approachKind = kind;
    if (!arrRunway.empty()) plan.arrRunway = std::string(arrRunway);
    if (activeInTail) plan.activeLeg = keep;
    return true;
}

void removeApproach(FlightPlan& plan, const NavDatabase& db) {
    if (plan.approachStartLeg < 0) return;
    const int keep = std::min(plan.approachStartLeg, static_cast<int>(plan.legs.size()));
    plan.legs.resize(static_cast<size_t>(keep));
    plan.approachStartLeg = -1;
    plan.approachId.clear();
    plan.approachKind = ApproachKind::None;
    Waypoint wp;
    if (endpointWaypoint(db, plan.destIdent, plan.arrRunway, true, wp, nullptr))
        plan.legs.push_back(makeLeg(LegType::TF, wp));
    if (plan.activeLeg >= static_cast<int>(plan.legs.size()))
        plan.activeLeg = static_cast<int>(plan.legs.size()) - 1;
}

// ---- distance, time, per-leg data -----------------------------------------------------------

double legDistNm(const FlightPlan& plan, int index) {
    if (index <= 0 || index >= static_cast<int>(plan.legs.size())) return 0;
    const Leg& l = plan.legs[static_cast<size_t>(index)];
    if (l.type == LegType::CA || l.type == LegType::VA) return 0;
    double lat, lon;
    if (!legStart(plan, index, lat, lon)) return 0;
    if (l.type == LegType::RF && l.arcRadiusNm > 0) return arcLengthNm(l, lat, lon);
    return distNmBetween(lat, lon, l.to.latDeg, l.to.lonDeg);
}

std::optional<float> legDtkTrueDeg(const FlightPlan& plan, int index) {
    if (index < 0 || index >= static_cast<int>(plan.legs.size())) return std::nullopt;
    const Leg& l = plan.legs[static_cast<size_t>(index)];
    if (l.type == LegType::CF || l.type == LegType::CA || l.type == LegType::VA ||
        l.type == LegType::FA)
        return static_cast<float>(wrap360(l.courseTrueDeg));
    double lat, lon;
    if (!legStart(plan, index, lat, lon)) return std::nullopt;
    if (lat == l.to.latDeg && lon == l.to.lonDeg) return std::nullopt;
    return static_cast<float>(radToDeg(initialBearingTrueRad(lat, lon, l.to.latDeg, l.to.lonDeg)));
}

std::optional<float> legDtkMagDeg(const FlightPlan& plan, int index, const MagVarFn& magVar) {
    auto t = legDtkTrueDeg(plan, index);
    if (!t) return std::nullopt;
    double lat, lon;
    if (!legStart(plan, index, lat, lon)) {
        lat = plan.legs[static_cast<size_t>(index)].to.latDeg;
        lon = plan.legs[static_cast<size_t>(index)].to.lonDeg;
    }
    const double var = magVar ? magVar(lat, lon) : 0.0;
    return static_cast<float>(wrap360(static_cast<double>(*t) - var));
}

double planDistNm(const FlightPlan& plan, int fromLeg, bool includeMissed) {
    double sum = 0;
    for (int i = std::max(fromLeg, 1); i < static_cast<int>(plan.legs.size()); ++i) {
        if (plan.legs[static_cast<size_t>(i)].missedApproach && !includeMissed) continue;
        sum += legDistNm(plan, i);
    }
    return sum;
}

double distRemainingNm(const FlightPlan& plan, const LatLon& present, bool includeMissed) {
    if (plan.legs.empty()) return 0;
    int a = std::max(plan.activeLeg, 0);
    if (a >= static_cast<int>(plan.legs.size())) return 0;
    const Leg& l = plan.legs[static_cast<size_t>(a)];
    double sum = distNmBetween(present.latDeg, present.lonDeg, l.to.latDeg, l.to.lonDeg);
    return sum + planDistNm(plan, a + 1, includeMissed);
}

double eteS(double distNm, double gsKt) {
    return gsKt > 0 ? distNm / gsKt * 3600.0 : 0.0;
}

std::vector<LegSummary> planSummary(const FlightPlan& plan, const MagVarFn& magVar) {
    std::vector<LegSummary> rows;
    for (int i = 0; i < static_cast<int>(plan.legs.size()); ++i) {
        const Leg& l = plan.legs[static_cast<size_t>(i)];
        LegSummary s;
        s.index = i;
        s.ident = l.to.ident;
        s.type = l.type;
        s.dtkTrueDeg = legDtkTrueDeg(plan, i);
        if (magVar) s.dtkMagDeg = legDtkMagDeg(plan, i, magVar);
        s.distNm = legDistNm(plan, i);
        s.alt = l.alt;
        s.missedApproach = l.missedApproach;
        rows.push_back(std::move(s));
    }
    return rows;
}

}  // namespace fsim
