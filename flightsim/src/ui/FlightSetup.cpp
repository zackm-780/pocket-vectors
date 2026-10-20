#include "ui/FlightSetup.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>

#include "app/Settings.h"  // writeFileAtomic
#include "core/Geo.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "nav/FlightPlan.h"
#include "sim/ProfileLoader.h"

namespace fsim {
namespace {

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

bool endsWithFms(const std::string& name) {
    if (name.size() < 4) return false;
    return upper(name.substr(name.size() - 4)) == ".FMS";
}

int runwayNumber(const std::string& ident) {
    return std::atoi(ident.c_str());
}
int runwaySide(const std::string& ident) {
    if (ident.empty()) return 0;
    switch (std::toupper(static_cast<unsigned char>(ident.back()))) {
    case 'L': return 0;
    case 'C': return 1;
    case 'R': return 2;
    default: return 1;
    }
}

bool hasRunways(const Airport* a) {
    return a && !a->runways.empty();
}

double distFrom(const Airport& a, double latDeg, double lonDeg) {
    return mToNm(greatCircleDistM(latDeg, lonDeg, a.latDeg, a.lonDeg));
}

const char* kindName(ApproachKind k) {
    switch (k) {
    case ApproachKind::Ils: return "ILS";
    case ApproachKind::Loc: return "LOC";
    case ApproachKind::Rnav: return "RNAV (GPS)";
    case ApproachKind::Vor: return "VOR";
    case ApproachKind::Ndb: return "NDB";
    case ApproachKind::Visual: return "VISUAL";
    case ApproachKind::None: break;
    }
    return "APPROACH";
}

}  // namespace

const char* startTypeLabel(StartType t) {
    switch (t) {
    case StartType::Runway: return "ON THE RUNWAY";
    case StartType::Final2Nm: return "2 NM FINAL";
    case StartType::Final6Nm: return "6 NM FINAL";
    case StartType::Final10Nm: return "10 NM FINAL";
    case StartType::InAir: return "IN THE AIR";
    }
    return "";
}

int stepInAirAltitude(int currentFt, int steps, float fieldElevFt) {
    const int floorFt = std::max(
        kInAirAltMinFt,
        static_cast<int>(std::ceil((fieldElevFt + 1000.0f) / kInAirAltStepFt)) * kInAirAltStepFt);
    int v = currentFt;
    if (v == 0 && steps > 0) {  // AUTO -> a sensible first value
        v = std::max(floorFt, 3000);
        --steps;
    } else if (v == 0) {
        return 0;
    }
    v += steps * kInAirAltStepFt;
    if (v < floorFt) return 0;  // below the minimum = back to AUTO
    return std::min(v, kInAirAltMaxFt);
}

// ---- discovery -------------------------------------------------------------------------------

const char* categoryLabel(AircraftCategory c) {
    switch (c) {
    case AircraftCategory::Piston: return "PISTON";
    case AircraftCategory::Turboprop: return "TURBOPROP";
    case AircraftCategory::Jet: return "JET";
    }
    return "";
}

std::vector<AircraftEntry> discoverAircraft(const std::string& aircraftDir) {
    std::vector<AircraftEntry> out;
    std::error_code ec;
    for (const auto& de : std::filesystem::directory_iterator(aircraftDir, ec)) {
        if (!de.is_regular_file(ec) || de.path().extension() != ".ini") continue;
        AircraftProfile p;
        std::string err;
        if (!loadAircraftProfile(de.path().string(), p, err)) {
            LOG_WARN("aircraft list: skipping %s: %s", de.path().string().c_str(), err.c_str());
            continue;
        }
        AircraftEntry e;
        e.id = p.id.empty() ? de.path().stem().string() : p.id;
        e.displayName = p.displayName.empty() ? e.id : p.displayName;
        e.category = p.category;
        e.engineCount = p.engineCount;
        e.cruiseAltFt = p.cruiseAltFt;
        e.cruiseKt = p.cruiseKt;
        out.push_back(std::move(e));
    }
    std::sort(out.begin(), out.end(), [](const AircraftEntry& a, const AircraftEntry& b) {
        const std::string an = upper(a.displayName), bn = upper(b.displayName);
        return an != bn ? an < bn : a.id < b.id;
    });
    return out;
}

std::vector<PlanFileEntry> discoverPlanFiles(const std::vector<std::string>& dirs) {
    std::map<std::string, std::string> byName;  // sorted by name
    for (const std::string& dir : dirs) {
        std::error_code ec;
        for (const auto& de : std::filesystem::directory_iterator(dir, ec)) {
            if (!de.is_regular_file(ec)) continue;
            const std::string name = de.path().filename().string();
            if (!endsWithFms(name)) continue;
            byName.emplace(name, de.path().string());  // emplace keeps the earlier directory
        }
    }
    std::vector<PlanFileEntry> out;
    for (auto& kv : byName) out.push_back({kv.first, kv.second});
    return out;
}

// ---- airports and runways ----------------------------------------------------------------------

std::vector<AirportHit> searchAirportsOrdered(const NavDatabase& db, const std::string& query,
                                              int maxCount, double refLatDeg, double refLonDeg) {
    std::vector<AirportHit> out;
    const std::string q = upper(query);
    if (q.empty() || maxCount < 1) return out;
    auto add = [&](const Airport* a) {
        if (!hasRunways(a)) return;
        for (const AirportHit& h : out)
            if (h.airport == a) return;
        out.push_back({a, distFrom(*a, refLatDeg, refLonDeg)});
    };
    add(db.findAirport(q));                           // exact ICAO ident first
    if (q.size() == 3) add(db.findAirport("K" + q));  // US FAA code
    for (const Airport* a : db.searchAirports(q, maxCount * 2 + 4)) add(a);
    if (static_cast<int>(out.size()) > maxCount) out.resize(static_cast<size_t>(maxCount));
    return out;
}

std::vector<AirportHit> nearestAirportHits(const NavDatabase& db, double latDeg, double lonDeg,
                                           int maxCount, float maxDistNm) {
    std::vector<AirportHit> out;
    for (const Airport* a : db.nearestAirports(latDeg, lonDeg, maxCount * 3 + 6, maxDistNm))
        if (hasRunways(a)) out.push_back({a, distFrom(*a, latDeg, lonDeg)});
    std::stable_sort(out.begin(), out.end(),
                     [](const AirportHit& a, const AirportHit& b) { return a.distNm < b.distNm; });
    if (static_cast<int>(out.size()) > maxCount) out.resize(static_cast<size_t>(maxCount));
    return out;
}

std::vector<const Runway*> sortedRunways(const Airport& airport) {
    std::vector<const Runway*> out;
    for (const Runway& r : airport.runways) out.push_back(&r);
    std::stable_sort(out.begin(), out.end(), [](const Runway* a, const Runway* b) {
        const int na = runwayNumber(a->ident), nb = runwayNumber(b->ident);
        if (na != nb) return na < nb;
        return runwaySide(a->ident) < runwaySide(b->ident);
    });
    return out;
}

const Runway* defaultRunway(const Airport& airport) {
    const Runway* best = nullptr;
    for (const Runway* r : sortedRunways(airport)) {
        if (!best) {
            best = r;
            continue;
        }
        const bool betterHard = r->hard && !best->hard;
        const bool sameHard = r->hard == best->hard;
        if (betterHard || (sameHard && r->lengthFt > best->lengthFt + 1.0f)) best = r;
    }
    return best;
}

float runwayHeadingMagDeg(const Runway& runway, float magVarDeg) {
    return static_cast<float>(wrap360(runway.headingTrueDeg - magVarDeg));
}

std::string runwayDescription(const Runway& r, float magVarDeg) {
    char buf[96];
    int hdg = static_cast<int>(std::lround(runwayHeadingMagDeg(r, magVarDeg)));
    if (hdg == 0) hdg = 360;
    std::snprintf(buf, sizeof buf, "%-3s %03dM %dX%d%s%s", r.ident.c_str(), hdg,
                  static_cast<int>(std::lround(r.lengthFt)),
                  static_cast<int>(std::lround(r.widthFt)), r.hard ? "" : " SOFT",
                  r.ils ? " ILS" : "");
    return buf;
}

// ---- approaches ----------------------------------------------------------------------------------

std::string approachTitle(const Approach& a, const ApproachInfo* info) {
    if (info && !info->title.empty()) return info->title;
    return std::string(kindName(a.kind)) + " RWY " + a.runwayIdent;
}

std::vector<ApproachChoice> approachChoices(const NavDatabase& db, const std::string& airportIdent,
                                            const std::string& runwayIdent,
                                            const std::string& navDir,
                                            const RealApproachProvider& provider) {
    std::vector<ApproachChoice> out;
    std::shared_ptr<const ProcedureSet> set;
    if (!navDir.empty()) set = loadProcedureSet(navDir, airportIdent);
    for (const Approach& a : availableApproaches(db, airportIdent, runwayIdent, provider)) {
        ApproachChoice c;
        c.id = a.id;
        const ApproachInfo* info = set ? findApproachInfo(*set, a.id) : nullptr;
        c.title = approachTitle(a, info);
        c.kind = a.kind;
        c.synthesized = a.synthesized;
        c.minimumsFt = a.minimumsFt;
        c.minimumsIsDa = a.minimumsIsDa;
        if (c.minimumsFt <= 0 && a.kind != ApproachKind::Visual) {
            float thrFt = 0;
            if (const Airport* ap = db.findAirport(airportIdent))
                if (const Runway* r = findRunway(*ap, a.runwayIdent)) thrFt = r->thrElevFt;
            c.minimumsFt = defaultMinimumsFt(a.kind, thrFt, &c.minimumsIsDa);
        }
        out.push_back(std::move(c));
    }
    return out;
}

// ---- FlightSetup -----------------------------------------------------------------------------------

bool FlightSetup::setAirport(const NavDatabase& db, const std::string& ident) {
    const Airport* a = db.findAirport(ident);
    if (!a || a->runways.empty()) return false;
    airportIdent = a->ident;
    const Runway* r = defaultRunway(*a);
    runwayIdent = r ? r->ident : "";
    if (planKind == PlanKind::Quick) clearPlan();
    if (!hasPlan()) approachId.clear();
    return true;
}

void FlightSetup::setRunway(const std::string& runwayIdentDb) {
    if (runwayIdent == runwayIdentDb) return;
    runwayIdent = runwayIdentDb;
    if (planKind == PlanKind::Quick) clearPlan();  // the origin runway is part of the plan
    if (!hasPlan()) approachId.clear();
}

bool FlightSetup::loadPlanFile(const NavDatabase& db, const std::string& path,
                               const std::string& name, std::string& error) {
    FmsFile file;
    PlanBuildResult r = loadPlanFromFmsFile(path, db, &file);
    if (!r.ok) {
        error = name + ": " + (r.errors.empty() ? "unreadable plan" : r.errors.front());
        return false;
    }
    planKind = PlanKind::File;
    planName = name;
    planFile = std::move(file);
    plan = std::move(r.plan);
    approachId = plan.approachId;
    if (const Airport* origin = db.findAirport(plan.originIdent)) {
        if (!origin->runways.empty()) {
            airportIdent = origin->ident;
            const Runway* rwy =
                plan.depRunway.empty() ? nullptr : findRunway(*origin, plan.depRunway);
            if (!rwy) rwy = defaultRunway(*origin);
            runwayIdent = rwy ? rwy->ident : "";
        }
    }
    return true;
}

bool FlightSetup::makeQuickPlan(const NavDatabase& db, const std::string& destIdent,
                                const std::string& destRunway, float cruiseAltFt,
                                std::string& error) {
    QuickPlanRequest req;
    req.originIdent = airportIdent;
    req.originRunway = runwayIdent;
    req.destIdent = destIdent;
    req.destRunway = destRunway;
    req.cruiseAltFt = cruiseAltFt;
    req.navaidsNear = [&db](double lat, double lon, float radiusNm) {
        return db.navaidsNear(lat, lon, radiusNm);
    };
    QuickPlanResult q = buildQuickPlan(req, db);
    if (!q.ok) {
        error = q.error.empty() ? "cannot build the plan" : q.error;
        return false;
    }
    planKind = PlanKind::Quick;
    planName = "QUICK " + airportIdent + "-" + destIdent;
    plan = std::move(q.plan);
    planFile = planToFms(plan, db);
    approachId.clear();
    return true;
}

void FlightSetup::clearPlan() {
    planKind = PlanKind::None;
    planName.clear();
    planFile = FmsFile{};
    plan = FlightPlan{};
    approachId.clear();
}

bool FlightSetup::setPlanDestRunway(const NavDatabase& db, const std::string& runwayIdentDb) {
    if (!hasPlan()) return false;
    planFile.desrwy = dbRunwayToFms(runwayIdentDb);
    planFile.app.clear();
    PlanBuildResult r = buildPlanFromFms(planFile, db);
    if (!r.ok) return false;
    plan = std::move(r.plan);
    plan.arrRunway = runwayIdentDb;
    approachId.clear();
    return true;
}

void FlightSetup::setApproach(const std::string& id) {
    approachId = id;
    if (hasPlan()) {
        planFile.app = id;
        plan.approachId = id;
    }
}

std::string FlightSetup::routeText() const {
    std::string s;
    const int end = enrouteEndIndex(plan);
    for (int i = 0; i < end && i < static_cast<int>(plan.legs.size()); ++i) {
        if (!s.empty()) s += ' ';
        const bool origin = i == 0 && !plan.originIdent.empty();
        s += origin ? plan.originIdent : plan.legs[static_cast<size_t>(i)].to.ident;
    }
    if (plan.approachStartLeg < 0 && end >= static_cast<int>(plan.legs.size())) return s;
    if (!plan.destIdent.empty()) {
        if (!s.empty()) s += ' ';
        s += plan.destIdent;
    }
    return s;
}

double FlightSetup::planDistNm() const {
    return hasPlan() ? fsim::planDistNm(plan) : 0.0;
}

Scenario FlightSetup::buildScenario(const std::string& planFilePath) const {
    Scenario s;
    s.aircraftId = aircraftId;
    s.airportIdent = airportIdent;
    s.runwayIdent = runwayIdent;
    switch (startType) {
    case StartType::Runway: s.startKind = "runway"; break;
    case StartType::Final2Nm:
        s.startKind = "final";
        s.distanceNm = 2;
        break;
    case StartType::Final6Nm:
        s.startKind = "final";
        s.distanceNm = 6;
        break;
    case StartType::Final10Nm:
        s.startKind = "final";
        s.distanceNm = 10;
        break;
    case StartType::InAir:
        s.startKind = "cruise";
        s.altitudeFt = inAirAltFt;
        break;
    }
    if (hasPlan()) s.planPath = planFilePath;
    s.approachId = approachId;
    return s;
}

bool FlightSetup::writePlanFile(const std::string& path, std::string& error) const {
    if (!hasPlan()) {
        error = "no flight plan";
        return false;
    }
    FmsFile f = planFile;
    f.app = approachId;
    if (!writeFileAtomic(path, writeFms(f))) {
        error = "cannot write " + path;
        return false;
    }
    return true;
}

}  // namespace fsim
