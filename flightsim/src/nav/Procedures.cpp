#include "nav/Procedures.h"

#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>

#include "core/Log.h"

namespace fsim {
namespace {

constexpr int kProcFormatVersion = 1;

std::vector<std::string_view> splitWs(std::string_view s) {
    std::vector<std::string_view> out;
    size_t i = 0;
    while (i < s.size()) {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        size_t j = i;
        while (j < s.size() && !std::isspace(static_cast<unsigned char>(s[j]))) ++j;
        if (j > i) out.push_back(s.substr(i, j - i));
        i = j;
    }
    return out;
}

// The text after the first `n` whitespace separated tokens, trimmed.
std::string_view restAfterTokens(std::string_view s, size_t n) {
    size_t i = 0;
    for (size_t k = 0; k < n; ++k) {
        while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
        while (i < s.size() && !std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    }
    while (i < s.size() && std::isspace(static_cast<unsigned char>(s[i]))) ++i;
    s.remove_prefix(i);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

bool parseDouble(std::string_view tok, double& out) {
    std::string t(tok);
    char* end = nullptr;
    errno = 0;
    out = std::strtod(t.c_str(), &end);
    return end != t.c_str() && *end == '\0' && errno == 0 && std::isfinite(out);
}

std::string upper(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

std::string lineErr(int line, const std::string& msg) {
    return "line " + std::to_string(line) + ": " + msg;
}

bool legTypeFromString(std::string_view s, LegType& out) {
    static const struct {
        const char* name;
        LegType type;
    } kTypes[] = {{"IF", LegType::IF}, {"TF", LegType::TF}, {"CF", LegType::CF},
                  {"DF", LegType::DF}, {"CA", LegType::CA}, {"VA", LegType::VA},
                  {"FA", LegType::FA}, {"HM", LegType::HM}, {"HA", LegType::HA},
                  {"HF", LegType::HF}, {"RF", LegType::RF}};
    for (const auto& t : kTypes)
        if (s == t.name) {
            out = t.type;
            return true;
        }
    return false;
}

ApproachKind kindFromString(std::string_view s) {
    if (s == "ILS") return ApproachKind::Ils;
    if (s == "LOC") return ApproachKind::Loc;
    if (s == "RNAV") return ApproachKind::Rnav;
    if (s == "VOR") return ApproachKind::Vor;
    if (s == "NDB") return ApproachKind::Ndb;
    if (s == "VISUAL") return ApproachKind::Visual;
    return ApproachKind::None;
}

bool isHold(LegType t) {
    return t == LegType::HM || t == LegType::HA || t == LegType::HF;
}

// One LEG line. Returns false with `error` set (without the line number).
bool parseLeg(const std::vector<std::string_view>& t, Leg& leg, ProcLegExtra& extra,
              std::string& error) {
    if (t.size() < 3) {
        error = "LEG needs a type and an ident";
        return false;
    }
    if (!legTypeFromString(t[1], leg.type)) {
        error = "unknown leg type " + std::string(t[1]);
        return false;
    }
    size_t i = 3;
    if (t[2] != "-") {
        double lat = 0, lon = 0;
        if (t.size() < 5 || !parseDouble(t[3], lat) || !parseDouble(t[4], lon)) {
            error = "LEG " + std::string(t[2]) + " needs numeric lat and lon";
            return false;
        }
        if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) {
            error = "coordinates out of range";
            return false;
        }
        leg.to.ident = std::string(t[2]);
        leg.to.latDeg = lat;
        leg.to.lonDeg = lon;
        i = 5;
    }
    bool turnSet = false, turnRight = true;
    auto need = [&](size_t n) { return i + n < t.size(); };  // t[i + n] exists
    while (i < t.size()) {
        const std::string_view k = t[i];
        double v = 0;
        if (k == "CRS") {
            if (!need(1) || !parseDouble(t[i + 1], v)) {
                error = "CRS needs a number";
                return false;
            }
            leg.courseTrueDeg = static_cast<float>(v);
            i += 2;
        } else if (k == "ALT") {
            if (!need(2)) {
                error = "ALT needs a kind and a value";
                return false;
            }
            const std::string_view kind = t[i + 1];
            if (kind == "AT")
                leg.alt.kind = AltConstraintKind::At;
            else if (kind == "ATORABOVE")
                leg.alt.kind = AltConstraintKind::AtOrAbove;
            else if (kind == "ATORBELOW")
                leg.alt.kind = AltConstraintKind::AtOrBelow;
            else if (kind == "BETWEEN")
                leg.alt.kind = AltConstraintKind::Between;
            else {
                error = "unknown ALT kind " + std::string(kind);
                return false;
            }
            if (!parseDouble(t[i + 2], v)) {
                error = "ALT value is not a number";
                return false;
            }
            leg.alt.altFt = static_cast<float>(v);
            i += 3;
            if (leg.alt.kind == AltConstraintKind::Between) {
                if (!need(0) || !parseDouble(t[i], v)) {
                    error = "ALT BETWEEN needs two values";
                    return false;
                }
                leg.alt.alt2Ft = static_cast<float>(v);
                ++i;
            }
        } else if (k == "SPD") {
            if (!need(1) || !parseDouble(t[i + 1], v)) {
                error = "SPD needs a number";
                return false;
            }
            leg.speedLimitKt = static_cast<float>(v);
            i += 2;
        } else if (k == "FLYOVER") {
            leg.flyOver = true;
            ++i;
        } else if (k == "ARC") {
            double clat = 0, clon = 0, rad = 0;
            if (!need(4) || !parseDouble(t[i + 1], clat) || !parseDouble(t[i + 2], clon) ||
                !parseDouble(t[i + 3], rad) || (t[i + 4] != "L" && t[i + 4] != "R")) {
                error = "ARC needs centre lat, lon, radius NM and L|R";
                return false;
            }
            leg.arcCenterLatDeg = clat;
            leg.arcCenterLonDeg = clon;
            leg.arcRadiusNm = static_cast<float>(rad);
            leg.arcRight = t[i + 4] == "R";
            turnSet = true;
            turnRight = leg.arcRight;
            i += 5;
        } else if (k == "TURN") {
            if (!need(1) || (t[i + 1] != "L" && t[i + 1] != "R")) {
                error = "TURN needs L or R";
                return false;
            }
            turnSet = true;
            turnRight = t[i + 1] == "R";
            i += 2;
        } else if (k == "HOLD") {
            if (!need(2) || (t[i + 1] != "T" && t[i + 1] != "D") || !parseDouble(t[i + 2], v)) {
                error = "HOLD needs T|D and a number";
                return false;
            }
            if (t[i + 1] == "T")
                extra.holdLegTimeS = static_cast<float>(v * 60.0);
            else
                extra.holdLegDistNm = static_cast<float>(v);
            i += 3;
        } else if (k == "VANG") {
            if (!need(1) || !parseDouble(t[i + 1], v)) {
                error = "VANG needs a number";
                return false;
            }
            extra.vertAngleDeg = static_cast<float>(v);
            i += 2;
        } else if (k == "WK") {
            if (!need(1)) {
                error = "WK needs a kind";
                return false;
            }
            const std::string_view w = t[i + 1];
            if (w == "VOR")
                leg.to.kind = WaypointKind::Vor;
            else if (w == "NDB")
                leg.to.kind = WaypointKind::Ndb;
            else if (w == "RWY")
                leg.to.kind = WaypointKind::Runway;
            else if (w == "APT")
                leg.to.kind = WaypointKind::Airport;
            else if (w == "SYN")
                leg.to.kind = WaypointKind::Synthetic;
            else
                leg.to.kind = WaypointKind::Fix;
            i += 2;
        } else if (k == "IAF") {
            extra.iaf = true;
            ++i;
        } else if (k == "FAF") {
            extra.faf = true;
            ++i;
        } else if (k == "MAP") {
            extra.map = true;
            ++i;
        } else if (k == "ORIG") {
            if (!need(1)) {
                error = "ORIG needs a type";
                return false;
            }
            extra.approximatedFrom = std::string(t[i + 1]);
            i += 2;
        } else if (k.find('=') != std::string_view::npos) {
            ++i;  // reserved KEY=value options of later format versions
        } else {
            error = "unknown option " + std::string(k);
            return false;
        }
    }
    if (turnSet && (isHold(leg.type) || leg.type == LegType::RF || leg.type == LegType::DF))
        leg.arcRight = turnRight;
    return true;
}

}  // namespace

bool parseProcedures(std::string_view text, ProcedureSet& out, std::string& error) {
    out = ProcedureSet();
    error.clear();
    enum class Seg { None, Transition, Final, Missed };
    Approach* cur = nullptr;
    ApproachInfo* curInfo = nullptr;
    Seg seg = Seg::None;
    bool sawFormat = false;
    int lineNo = 0;
    size_t pos = 0;
    while (pos <= text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string_view::npos) eol = text.size();
        std::string_view line = text.substr(pos, eol - pos);
        pos = eol + 1;
        ++lineNo;
        const auto t = splitWs(line);
        if (t.empty() || t[0][0] == '#') continue;
        if (!sawFormat) {
            double ver = 0;
            if (t[0] != "FORMAT" || t.size() < 3) {
                error =
                    lineErr(lineNo, "the first directive must be FORMAT flightsim-proc <version>");
                return false;
            }
            if (t[1] != "flightsim-proc" || !parseDouble(t[2], ver) ||
                static_cast<int>(ver) != kProcFormatVersion) {
                error = "unsupported procedures format '" + std::string(t[1]) + " " +
                        std::string(t[2]) + "' (this build reads flightsim-proc 1)";
                return false;
            }
            sawFormat = true;
            continue;
        }
        const std::string_view key = t[0];
        if (key == "CYCLE") {
            if (t.size() > 1) out.cycle = std::string(t[1]);
        } else if (key == "AIRPORT") {
            if (t.size() > 1) out.airportIdent = std::string(t[1]);
        } else if (key == "APPROACH") {
            if (t.size() < 4) {
                error = lineErr(lineNo, "APPROACH needs id, kind and runway");
                return false;
            }
            out.approaches.emplace_back();
            out.info.emplace_back();
            cur = &out.approaches.back();
            curInfo = &out.info.back();
            cur->id = std::string(t[1]);
            cur->airportIdent = out.airportIdent;
            cur->kind = kindFromString(t[2]);
            cur->runwayIdent = t[3] == "-" ? std::string() : std::string(t[3]);
            cur->synthesized = false;
            curInfo->title = std::string(restAfterTokens(line, 4));
            seg = Seg::None;
        } else if (cur == nullptr) {
            if (key == "LOS" || key == "TRANSITION" || key == "FINAL" || key == "MISSED" ||
                key == "LEG" || key == "END") {
                error = lineErr(lineNo, std::string(key) + " outside an approach");
                return false;
            }
            // unknown directive between approaches: ignored (forward compatibility)
        } else if (key == "LOS") {
            for (size_t i = 1; i < t.size(); ++i) curInfo->levelsOfService.emplace_back(t[i]);
        } else if (key == "TRANSITION") {
            ApproachTransition tr;
            tr.name = t.size() > 1 ? std::string(t[1]) : std::string();
            cur->transitions.push_back(std::move(tr));
            curInfo->transitionExtras.emplace_back();
            seg = Seg::Transition;
        } else if (key == "FINAL") {
            seg = Seg::Final;
        } else if (key == "MISSED") {
            seg = Seg::Missed;
        } else if (key == "END") {
            cur = nullptr;
            curInfo = nullptr;
            seg = Seg::None;
        } else if (key == "LEG") {
            if (seg == Seg::None) {
                error = lineErr(lineNo, "LEG before TRANSITION, FINAL or MISSED");
                return false;
            }
            Leg leg;
            ProcLegExtra extra;
            std::string e;
            if (!parseLeg(t, leg, extra, e)) {
                error = lineErr(lineNo, e);
                return false;
            }
            switch (seg) {
            case Seg::Transition:
                cur->transitions.back().legs.push_back(std::move(leg));
                curInfo->transitionExtras.back().push_back(std::move(extra));
                break;
            case Seg::Final:
                cur->finalLegs.push_back(std::move(leg));
                curInfo->finalExtras.push_back(std::move(extra));
                break;
            default:
                leg.missedApproach = true;
                cur->missedLegs.push_back(std::move(leg));
                curInfo->missedExtras.push_back(std::move(extra));
                break;
            }
        }
        // any other directive inside an approach is ignored
    }
    if (!sawFormat) {
        error = "empty procedures file";
        return false;
    }
    // sort by id (stable keeps the order of equal ids), then derive FAF / MAP indexes
    std::vector<size_t> order(out.approaches.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) {
        return out.approaches[a].id < out.approaches[b].id;
    });
    ProcedureSet sorted;
    sorted.airportIdent = out.airportIdent;
    sorted.cycle = out.cycle;
    for (size_t i : order) {
        sorted.approaches.push_back(std::move(out.approaches[i]));
        sorted.info.push_back(std::move(out.info[i]));
    }
    for (size_t i = 0; i < sorted.approaches.size(); ++i) {
        ApproachInfo& info = sorted.info[i];
        for (size_t k = 0; k < info.finalExtras.size(); ++k) {
            if (info.finalExtras[k].faf && info.fafFinalLeg < 0)
                info.fafFinalLeg = static_cast<int>(k);
            if (info.finalExtras[k].map && info.mapFinalLeg < 0)
                info.mapFinalLeg = static_cast<int>(k);
        }
        Approach& ap = sorted.approaches[i];
        ap.fafFinalLeg = info.fafFinalLeg;
        ap.helicopterOnly = upper(info.title).find("COPTER") != std::string::npos;
        if (ap.finalLegs.empty()) sorted.warnings.push_back(ap.id + ": no final legs");
        // A transition that ends with a direct leg to the first fix of the final segment would
        // repeat that fix there (TF OXVAK, then IF OXVAK): drop the transition's copy, the IF
        // of the final segment carries the constraint. Holds and CF legs stay.
        if (!ap.finalLegs.empty() && ap.finalLegs.front().type == LegType::IF) {
            for (size_t t = 0; t < ap.transitions.size(); ++t) {
                auto& legs = ap.transitions[t].legs;
                if (legs.size() > 1 &&
                    (legs.back().type == LegType::TF || legs.back().type == LegType::DF) &&
                    legs.back().to.ident == ap.finalLegs.front().to.ident) {
                    if (ap.finalLegs.front().alt.kind == AltConstraintKind::None)
                        ap.finalLegs.front().alt = legs.back().alt;
                    legs.pop_back();
                    if (t < info.transitionExtras.size() && !info.transitionExtras[t].empty())
                        info.transitionExtras[t].pop_back();
                }
            }
        }
        // Convention shared with N5's ApproachBuilder: transitions[0] is "vectors" (empty name, no
        // legs, straight to the final segment); the IAF routes follow.
        ap.transitions.insert(ap.transitions.begin(), ApproachTransition{});
        info.transitionExtras.insert(info.transitionExtras.begin(), std::vector<ProcLegExtra>());
    }
    out = std::move(sorted);
    return true;
}

// ---- loading with cache --------------------------------------------------------------------

namespace {

std::mutex gCacheMutex;
std::map<std::string, std::shared_ptr<const ProcedureSet>>& cache() {
    static std::map<std::string, std::shared_ptr<const ProcedureSet>> c;
    return c;
}

bool readFile(const std::string& path, std::string& out) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return false;
    std::ostringstream ss;
    ss << f.rdbuf();
    out = ss.str();
    return true;
}

std::string stripTrailingSlash(std::string s) {
    while (s.size() > 1 && (s.back() == '/' || s.back() == '\\')) s.pop_back();
    return s;
}

}  // namespace

std::shared_ptr<const ProcedureSet> loadProcedureSet(const std::string& navDir,
                                                     std::string_view icao) {
    const std::string ident = upper(icao);
    if (ident.empty() || ident.find('/') != std::string::npos ||
        ident.find('\\') != std::string::npos || ident.find("..") != std::string::npos)
        return nullptr;
    const std::string dir = stripTrailingSlash(navDir);
    const std::string key = dir + "|" + ident;
    {
        std::lock_guard<std::mutex> lock(gCacheMutex);
        auto it = cache().find(key);
        if (it != cache().end()) return it->second;
    }
    std::shared_ptr<const ProcedureSet> result;
    std::string text;
    const std::string rel = "/procedures/" + ident + ".proc";
    if (readFile(dir + rel, text) || readFile(dir + "/nav" + rel, text)) {
        auto set = std::make_shared<ProcedureSet>();
        std::string error;
        if (parseProcedures(text, *set, error)) {
            for (const auto& w : set->warnings)
                LOG_WARN("procedures %s: %s", ident.c_str(), w.c_str());
            LOG_DEBUG("procedures %s: %zu approaches (cycle %s)", ident.c_str(),
                      set->approaches.size(), set->cycle.c_str());
            result = std::move(set);
        } else {
            LOG_WARN("procedures %s: %s", ident.c_str(), error.c_str());
        }
    }
    std::lock_guard<std::mutex> lock(gCacheMutex);
    cache()[key] = result;
    return result;
}

void clearProcedureCache() {
    std::lock_guard<std::mutex> lock(gCacheMutex);
    cache().clear();
}

std::vector<Approach> loadApproaches(const std::string& dataDir, std::string_view icao) {
    auto set = loadProcedureSet(dataDir, icao);
    if (!set) return {};
    return set->approaches;
}

CifpApproachProvider makeCifpApproachProvider(const std::string& navDir) {
    return [navDir](const Airport& airport, const Runway& runway) {
        std::vector<Approach> out;
        auto set = loadProcedureSet(navDir, airport.ident);
        if (!set) return out;
        for (const Approach* a : approachesForRunway(set->approaches, runway.ident))
            out.push_back(*a);
        return out;
    };
}

// ---- queries --------------------------------------------------------------------------------

const Approach* findApproach(const std::vector<Approach>& approaches, std::string_view id) {
    const std::string want = upper(id);
    if (want.empty()) return nullptr;
    for (const Approach& a : approaches)
        if (upper(a.id) == want) return &a;
    // short form: "I05" finds "I05-Y"; "I16R" finds "I16RZ" but "R28" does not find "R28L".
    // Several real ids can share the short form: prefer a fixed-wing procedure (not a COPTER
    // chart), then the last suffix letter (FAA "Z" is usually the primary one).
    const Approach* best = nullptr;
    for (const Approach& a : approaches) {
        const std::string have = upper(a.id);
        if (have.size() <= want.size() || have.compare(0, want.size(), want) != 0) continue;
        const std::string rest = have.substr(want.size());
        const bool dashed =
            rest.size() == 2 && rest[0] == '-' && std::isalpha(static_cast<unsigned char>(rest[1]));
        const bool plain = rest.size() == 1 && std::isalpha(static_cast<unsigned char>(rest[0])) &&
                           rest[0] != 'L' && rest[0] != 'R' && rest[0] != 'C';
        if (!dashed && !plain) continue;
        if (!best || (best->helicopterOnly && !a.helicopterOnly) ||
            (best->helicopterOnly == a.helicopterOnly && upper(a.id) > upper(best->id)))
            best = &a;
    }
    return best;
}

std::vector<const Approach*> approachesForRunway(const std::vector<Approach>& approaches,
                                                 std::string_view runwayIdent) {
    std::vector<const Approach*> out;
    const std::string rwy = upper(runwayIdent);
    for (const Approach& a : approaches)
        if (!rwy.empty() && upper(a.runwayIdent) == rwy) out.push_back(&a);
    std::sort(out.begin(), out.end(),
              [](const Approach* x, const Approach* y) { return x->id < y->id; });
    return out;
}

int transitionJoinLeg(const Approach& approach, size_t transitionIndex) {
    if (transitionIndex >= approach.transitions.size() ||
        approach.transitions[transitionIndex].legs.empty() || approach.finalLegs.empty())
        return 0;
    const std::string& last = approach.transitions[transitionIndex].legs.back().to.ident;
    if (last.empty()) return 0;
    for (size_t k = 0; k < approach.finalLegs.size(); ++k)
        if (approach.finalLegs[k].to.ident == last) return static_cast<int>(k);
    return 0;
}

std::vector<Leg> assembleApproachLegs(const Approach& approach, std::string_view transitionName,
                                      bool* transitionFound) {
    std::vector<Leg> legs;
    size_t trIndex = 0;
    const ApproachTransition* tr = nullptr;
    if (!transitionName.empty()) {
        const std::string want = upper(transitionName);
        for (size_t i = 0; i < approach.transitions.size(); ++i)
            if (upper(approach.transitions[i].name) == want) {
                tr = &approach.transitions[i];
                trIndex = i;
                break;
            }
    }
    if (transitionFound) *transitionFound = tr != nullptr || transitionName.empty();
    if (tr) legs = tr->legs;
    // The transition may join the final segment at its first fix or later (a procedure turn at the
    // FAF, a VOR approach whose IAF is the FAF): the final legs before the join are not flown.
    size_t firstFinal = 0;
    if (tr && !tr->legs.empty() && !approach.finalLegs.empty()) {
        firstFinal = static_cast<size_t>(transitionJoinLeg(approach, trIndex));
        const Leg& last = tr->legs.back();
        const Leg& join = approach.finalLegs[firstFinal];
        const bool direct =
            join.type == LegType::IF || join.type == LegType::TF || join.type == LegType::DF;
        if (join.to.ident == last.to.ident && direct) {
            if (legs.back().alt.kind == AltConstraintKind::None) legs.back().alt = join.alt;
            ++firstFinal;  // the transition already ends at this fix
        }
    }
    legs.insert(legs.end(), approach.finalLegs.begin() + static_cast<long>(firstFinal),
                approach.finalLegs.end());
    for (Leg l : approach.missedLegs) {
        l.missedApproach = true;
        legs.push_back(std::move(l));
    }
    return legs;
}

const ApproachInfo* findApproachInfo(const ProcedureSet& set, std::string_view approachId) {
    const std::string want = upper(approachId);
    for (size_t i = 0; i < set.approaches.size(); ++i)
        if (upper(set.approaches[i].id) == want) return &set.info[i];
    return nullptr;
}

}  // namespace fsim
