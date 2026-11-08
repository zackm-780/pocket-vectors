#include "app/FlightLog.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <fstream>
#include <sstream>

#include "app/Scenario.h"
#include "app/Settings.h"  // writeFileAtomic
#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/Paths.h"
#include "core/Units.h"

namespace fsim {
namespace {

constexpr int kColumns = 19;
constexpr double kArrivalRadiusNm = 2.0;

// Commas and line breaks would break the row: they become spaces.
std::string clean(const std::string& s) {
    std::string out = s.empty() ? "-" : s;
    for (char& c : out)
        if (c == ',' || c == '\n' || c == '\r') c = ' ';
    return out;
}

std::string upper(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::vector<std::string> splitRow(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : line) {
        if (c == ',') {
            out.push_back(cur);
            cur.clear();
        } else if (c != '\r' && c != '\n') {
            cur += c;
        }
    }
    out.push_back(cur);
    return out;
}

bool toDouble(const std::string& s, double& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return end && *end == '\0' && std::isfinite(out);
}

std::string fmt(const char* f, double v) {
    char b[32];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

// The plan file's name without its folder ("-" without a plan).
std::string planName(const std::string& path) {
    if (path.empty()) return "-";
    const size_t slash = path.find_last_of("/\\");
    return slash == std::string::npos ? path : path.substr(slash + 1);
}

}  // namespace

// ---- recorder -------------------------------------------------------------------------------------

void FlightRecorder::begin(const Scenario& sc, const std::string& aircraftId,
                           const std::string& startUtc, const AircraftState& s) {
    entry_ = FlightLogEntry{};
    entry_.startUtc = startUtc;
    entry_.aircraftId = aircraftId;
    entry_.depAirport = sc.airportIdent;
    entry_.depRunway = sc.runwayIdent;
    entry_.startKind = sc.startKind;
    entry_.plan = planName(sc.planPath);
    entry_.approach = sc.approachId.empty() ? "-" : sc.approachId;
    entry_.fuelStartLbs = entry_.fuelEndLbs = s.fuelTotalLbs;
    entry_.maxAltFt = s.altMslFt;
    active_ = true;
    last_ = s;
    haveLast_ = true;
    airborne_ = everAirborne_ = s.valid && !s.wowAny;  // a final or cruise start is airborne
    airborneSinceS_ = 0;
    lastLiftoffS_ = -1e9;
    airSinkFpm_ = std::max(0.0, -static_cast<double>(s.vsFpm));
    tdLatDeg_ = tdLonDeg_ = 0;
    sinceCheckpointS_ = 0;
    checkpointDue_ = false;
}

void FlightRecorder::update(const AircraftState& s, double dtS, bool paused) {
    if (!active_ || !s.valid) return;
    if (!paused && dtS > 0) {
        entry_.flightS += dtS;
        if (!s.wowAny) entry_.airborneS += dtS;
        entry_.distNm += std::max(0.0f, s.gsKt) * dtS / 3600.0;
        entry_.maxAltFt = std::max(entry_.maxAltFt, s.altMslFt);
        sinceCheckpointS_ += dtS;
        if (sinceCheckpointS_ >= kCheckpointS) checkpointDue_ = true;
    }
    entry_.fuelEndLbs = s.fuelTotalLbs;
    const bool wasOnGround = haveLast_ ? last_.wowAny : s.wowAny;
    if (!paused) {
        if (wasOnGround && !s.wowAny) {  // liftoff
            airborne_ = everAirborne_ = true;
            lastLiftoffS_ = entry_.flightS;
        } else if (!wasOnGround && s.wowAny) {  // touchdown
            airborne_ = false;
            // A touchdown less than kBounceS after a liftoff is a bounce (or a hop on the takeoff
            // roll): the same landing, not a new one.
            if (entry_.flightS - lastLiftoffS_ >= kBounceS) {
                ++entry_.landings;
                entry_.touchdownFpm = airSinkFpm_;
                tdLatDeg_ = s.latDeg;
                tdLonDeg_ = s.lonDeg;
                checkpointDue_ = true;
            }
        }
        if (!s.wowAny) airSinkFpm_ = std::max(0.0, -static_cast<double>(s.vsFpm));
    }
    last_ = s;
    haveLast_ = true;
}

FlightLogEntry FlightRecorder::end(FlightEnd why) {
    FlightLogEntry e = entry_;
    if (haveLast_) e.fuelEndLbs = last_.fuelTotalLbs;
    if (why == FlightEnd::Crash || (haveLast_ && last_.crashed)) {
        e.outcome = "CRASHED";
        if (haveLast_ && last_.crashReason[0]) e.outcome += ": " + upper(clean(last_.crashReason));
    } else if (haveLast_ && !last_.wowAny) {
        e.outcome = "IN FLIGHT";
    } else if (e.landings > 0) {
        e.outcome = "LANDED";
    } else {
        e.outcome = everAirborne_ ? "IN FLIGHT" : "ON GROUND";
    }
    active_ = false;
    checkpointDue_ = false;
    return e;
}

// ---- helpers --------------------------------------------------------------------------------------

std::string formatUtcNow() {
    const std::time_t now = std::time(nullptr);
    std::tm tm{};
#if defined(_WIN32)
    gmtime_s(&tm, &now);
#else
    gmtime_r(&now, &tm);
#endif
    char b[32];
    std::strftime(b, sizeof b, "%Y-%m-%dT%H:%M:%SZ", &tm);
    return b;
}

void fillArrival(FlightLogEntry& e, double tdLatDeg, double tdLonDeg, const NavDatabase& nav) {
    e.arrAirport = e.arrRunway = "-";
    if (e.landings <= 0) return;
    const std::vector<const Airport*> near =
        nav.nearestAirports(tdLatDeg, tdLonDeg, 4, static_cast<float>(kArrivalRadiusNm));
    double best = 1e30;
    for (const Airport* a : near) {
        for (const Runway& r : a->runways) {
            // Distance from the touchdown to the runway's centerline segment (clamped).
            const LatLon p{tdLatDeg, tdLonDeg}, s{r.thrLatDeg, r.thrLonDeg},
                f{r.endLatDeg, r.endLonDeg};
            const double len = greatCircleDistM(s.latDeg, s.lonDeg, f.latDeg, f.lonDeg);
            const double along = std::clamp(alongTrackM(p, s, f), 0.0, len);
            const double d = std::hypot(crossTrackM(p, s, f), alongTrackM(p, s, f) - along);
            // The landing direction: the one whose threshold the touchdown is past, nearest.
            const double score = d + 1e-3 * along;
            if (score < best) {
                best = score;
                e.arrAirport = a->ident;
                e.arrRunway = r.ident;
            }
        }
        if (a->runways.empty() && e.arrAirport == "-") e.arrAirport = a->ident;
    }
}

// ---- CSV ------------------------------------------------------------------------------------------

const char* flightLogHeader() {
    return "start_utc,aircraft,dep_airport,dep_runway,start,plan,approach,arr_airport,"
           "arr_runway,flight_s,airborne_s,dist_nm,max_alt_ft,landings,touchdown_fpm,"
           "fuel_start_lbs,fuel_end_lbs,fuel_used_lbs,outcome";
}

std::string flightLogRow(const FlightLogEntry& e) {
    std::string r;
    for (const std::string* s : {&e.startUtc, &e.aircraftId, &e.depAirport, &e.depRunway,
                                 &e.startKind, &e.plan, &e.approach, &e.arrAirport, &e.arrRunway})
        r += clean(*s) + ",";
    r += fmt("%.0f", e.flightS) + "," + fmt("%.0f", e.airborneS) + "," + fmt("%.1f", e.distNm) +
         "," + fmt("%.0f", e.maxAltFt) + "," + std::to_string(e.landings) + "," +
         fmt("%.0f", e.touchdownFpm) + "," + fmt("%.0f", e.fuelStartLbs) + "," +
         fmt("%.0f", e.fuelEndLbs) + "," + fmt("%.0f", e.fuelUsedLbs()) + "," + clean(e.outcome);
    return r;
}

bool parseFlightLogRow(const std::string& line, FlightLogEntry& out) {
    const std::vector<std::string> f = splitRow(line);
    if (static_cast<int>(f.size()) != kColumns || f[0] == "start_utc") return false;
    FlightLogEntry e;
    e.startUtc = f[0];
    e.aircraftId = f[1];
    e.depAirport = f[2];
    e.depRunway = f[3];
    e.startKind = f[4];
    e.plan = f[5];
    e.approach = f[6];
    e.arrAirport = f[7];
    e.arrRunway = f[8];
    double landings = 0;
    if (!toDouble(f[9], e.flightS) || !toDouble(f[10], e.airborneS) || !toDouble(f[11], e.distNm) ||
        !toDouble(f[12], e.maxAltFt) || !toDouble(f[13], landings) ||
        !toDouble(f[14], e.touchdownFpm) || !toDouble(f[15], e.fuelStartLbs) ||
        !toDouble(f[16], e.fuelEndLbs))
        return false;
    e.landings = static_cast<int>(landings);
    e.outcome = f[18];
    if (e.startUtc.size() < 16 || e.outcome.empty() || e.flightS < 0) return false;
    out = std::move(e);
    return true;
}

bool appendFlightLog(const std::string& path, const FlightLogEntry& e) {
    bool needHeader = true;
    if (std::FILE* in = std::fopen(path.c_str(), "rb")) {
        needHeader = std::fgetc(in) == EOF;
        std::fclose(in);
    }
    std::FILE* f = std::fopen(path.c_str(), "ab");
    if (!f) return false;
    bool ok = true;
    if (needHeader) ok = std::fprintf(f, "%s\n", flightLogHeader()) > 0;
    ok = std::fprintf(f, "%s\n", flightLogRow(e).c_str()) > 0 && ok;
    ok = std::fclose(f) == 0 && ok;
    return ok;
}

std::vector<FlightLogEntry> readFlightLog(const std::string& path, size_t maxRows) {
    std::ifstream in(path, std::ios::binary);
    std::deque<FlightLogEntry> keep;
    std::string line;
    while (in && std::getline(in, line)) {
        FlightLogEntry e;
        if (!parseFlightLogRow(line, e)) continue;  // header, damaged or foreign lines
        keep.push_back(std::move(e));
        if (keep.size() > maxRows) keep.pop_front();
    }
    return std::vector<FlightLogEntry>(keep.rbegin(), keep.rend());
}

bool writeOpenFlight(const std::string& path, const FlightLogEntry& e) {
    return writeFileAtomic(path, std::string(flightLogHeader()) + "\n" + flightLogRow(e) + "\n");
}

bool recoverOpenFlight(const std::string& openPath, const std::string& logPath) {
    if (!fileExists(openPath)) return false;
    const std::vector<FlightLogEntry> open = readFlightLog(openPath, 1);
    bool appended = false;
    if (!open.empty()) {
        FlightLogEntry e = open.front();
        e.outcome = "GAME CLOSED";
        appended = appendFlightLog(logPath, e);
    }
    if (appended || open.empty()) std::remove(openPath.c_str());  // a damaged file is dropped
    return appended;
}

// ---- screen text ----------------------------------------------------------------------------------

std::string formatHoursMinutes(double seconds) {
    const long minutes = static_cast<long>(std::max(0.0, seconds) / 60.0 + 0.5);
    char b[24];
    std::snprintf(b, sizeof b, "%ld:%02ld", minutes / 60, minutes % 60);
    return b;
}

std::string flightLogListLine(const FlightLogEntry& e) {
    // "2026-11-07T14:02:00Z" -> "11-07 14:02"
    std::string when = e.startUtc.size() >= 16
                           ? e.startUtc.substr(5, 5) + " " + e.startUtc.substr(11, 5)
                           : e.startUtc;
    std::string route =
        e.depAirport + (e.depRunway.empty() || e.depRunway == "-" ? "" : " " + e.depRunway);
    if (e.arrAirport != "-")
        route += " > " + e.arrAirport + (e.arrRunway == "-" ? "" : " " + e.arrRunway);
    std::string out = when + "  " + upper(e.aircraftId) + "  " + route + "  " +
                      formatHoursMinutes(e.flightS) + "  " + e.outcome;
    return out;
}

}  // namespace fsim
