#include "nav/NavDb.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <cstring>

#include "core/Csv.h"
#include "core/Geo.h"
#include "core/Ini.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "core/Units.h"
#include "nav/NavDbIls.h"

namespace fsim {
namespace {

constexpr int kGridLatCells = 180;
constexpr int kGridLonCells = 360;

std::string toUpper(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

// Numeric parse of a CSV field; `ok` tells whether the field held a number (empty = unknown).
double num(std::string_view s, bool& ok) {
    ok = false;
    if (s.empty() || s.size() > 31) return 0;
    char buf[32];
    std::memcpy(buf, s.data(), s.size());
    buf[s.size()] = '\0';
    char* end = nullptr;
    const double v = std::strtod(buf, &end);
    if (end == buf) return 0;
    ok = true;
    return v;
}
double num(std::string_view s, double def = 0) {
    bool ok;
    const double v = num(s, ok);
    return ok ? v : def;
}

AirportType parseAirportType(std::string_view s) {
    if (s == "large") return AirportType::Large;
    if (s == "medium") return AirportType::Medium;
    if (s == "small") return AirportType::Small;
    if (s == "seaplane") return AirportType::Seaplane;
    return AirportType::Unknown;
}

bool parseNavaidType(std::string_view s, NavaidType& out) {
    static const struct {
        const char* name;
        NavaidType type;
    } kTypes[] = {{"VOR", NavaidType::Vor},       {"VORDME", NavaidType::VorDme},
                  {"VORTAC", NavaidType::Vortac}, {"TACAN", NavaidType::Tacan},
                  {"NDB", NavaidType::Ndb},       {"NDBDME", NavaidType::NdbDme},
                  {"DME", NavaidType::Dme}};
    for (const auto& t : kTypes) {
        if (s == t.name) {
            out = t.type;
            return true;
        }
    }
    return false;
}

int latCell(double latDeg) {
    return std::clamp(static_cast<int>(std::floor(latDeg)) + 90, 0, kGridLatCells - 1);
}
int lonCellWrapped(int lonFloorDeg) {
    int c = (lonFloorDeg + 180) % kGridLonCells;
    if (c < 0) c += kGridLonCells;
    return c;
}
int lonCell(double lonDeg) {
    return lonCellWrapped(static_cast<int>(std::floor(lonDeg)));
}

bool startsWith(const std::string& s, const std::string& prefix) {
    return s.size() >= prefix.size() && s.compare(0, prefix.size(), prefix) == 0;
}

// True when `prefix` starts the string or any word in it (words split on non-alphanumerics).
bool wordPrefix(const std::string& s, const std::string& prefix) {
    for (size_t i = 0; i < s.size(); ++i) {
        if (i > 0 && std::isalnum(static_cast<unsigned char>(s[i - 1]))) continue;
        if (s.compare(i, prefix.size(), prefix) == 0) return true;
    }
    return false;
}

int typeRank(AirportType t) {
    switch (t) {
    case AirportType::Large: return 0;
    case AirportType::Medium: return 1;
    case AirportType::Small: return 2;
    case AirportType::Seaplane: return 3;
    default: return 4;
    }
}

std::chrono::steady_clock::time_point now() {
    return std::chrono::steady_clock::now();
}

}  // namespace

CsvNavDatabase::CsvNavDatabase() {
    reset();
}

void CsvNavDatabase::reset() {
    airports_.clear();
    extra_.clear();
    byIdent_.clear();
    navaids_.clear();
    fixes_.clear();
    airways_.clear();
    magvar_ = MagVar();
    stats_ = NavDbStats();
    // XTST is always present (ARCHITECTURE section 9).
    SyntheticNavDb synthetic;
    addAirport(synthetic.airport(), "", "");
    buildIndexes();
}

void CsvNavDatabase::addAirport(Airport airport, std::string iata, std::string municipality) {
    AirportExtra ex;
    ex.identUpper = toUpper(airport.ident);
    ex.iataUpper = toUpper(iata);
    ex.nameLower = toLower(airport.name);
    ex.muniLower = toLower(municipality);
    byIdent_.emplace(ex.identUpper, static_cast<int>(airports_.size()));
    airports_.push_back(std::move(airport));
    extra_.push_back(std::move(ex));
}

void CsvNavDatabase::buildIndexes() {
    grid_.assign(static_cast<size_t>(kGridLatCells) * kGridLonCells, {});
    for (size_t i = 0; i < airports_.size(); ++i) {
        const Airport& a = airports_[i];
        grid_[static_cast<size_t>(latCell(a.latDeg)) * kGridLonCells +
              static_cast<size_t>(lonCell(a.lonDeg))]
            .push_back(static_cast<int>(i));
    }
}

// ------------------------------------------------------------------------------------------------
// Loading

bool CsvNavDatabase::loadAirports(const std::string& path, std::string& error) {
    CsvReader csv;
    if (!csv.open(path)) {
        error = "cannot read " + path;
        return false;
    }
    const int cIdent = csv.column("ident"), cType = csv.column("type"), cName = csv.column("name"),
              cLat = csv.column("lat_deg"), cLon = csv.column("lon_deg"),
              cElev = csv.column("elev_ft"), cIso = csv.column("iso_country"),
              cMuni = csv.column("municipality"), cIata = csv.column("iata");
    if (cIdent < 0 || cLat < 0 || cLon < 0) {
        error = path + ": missing required columns (ident, lat_deg, lon_deg)";
        return false;
    }
    while (csv.next()) {
        bool latOk, lonOk;
        const double lat = num(csv.field(cLat), latOk), lon = num(csv.field(cLon), lonOk);
        const std::string ident = toUpper(csv.field(cIdent));
        if (ident.empty() || !latOk || !lonOk || byIdent_.count(ident)) {
            ++stats_.skippedRows;  // includes a data row that would shadow XTST
            continue;
        }
        Airport a;
        a.ident = ident;
        a.name = std::string(csv.field(cName));
        a.iso = std::string(csv.field(cIso));
        a.latDeg = lat;
        a.lonDeg = lon;
        a.elevFt = static_cast<float>(num(csv.field(cElev)));
        a.type = parseAirportType(csv.field(cType));
        addAirport(std::move(a), std::string(csv.field(cIata)), std::string(csv.field(cMuni)));
    }
    return true;
}

bool CsvNavDatabase::loadRunways(const std::string& path, std::string& error) {
    CsvReader csv;
    if (!csv.open(path)) {
        error = "cannot read " + path;
        return false;
    }
    const int cApt = csv.column("airport_ident"), cIdent = csv.column("rwy_ident"),
              cOpp = csv.column("opposite_ident"), cThrLat = csv.column("thr_lat_deg"),
              cThrLon = csv.column("thr_lon_deg"), cThrElev = csv.column("thr_elev_ft"),
              cStartLat = csv.column("start_lat_deg"), cStartLon = csv.column("start_lon_deg"),
              cEndLat = csv.column("end_lat_deg"), cEndLon = csv.column("end_lon_deg"),
              cEndElev = csv.column("end_elev_ft"), cHdg = csv.column("heading_true_deg"),
              cLen = csv.column("length_ft"), cWid = csv.column("width_ft"),
              cDisp = csv.column("displaced_ft"), cHard = csv.column("hard"),
              cLit = csv.column("lighted"), cApprox = csv.column("approx");
    if (cApt < 0 || cIdent < 0 || cThrLat < 0 || cThrLon < 0 || cEndLat < 0 || cEndLon < 0 ||
        cHdg < 0) {
        error = path + ": missing required columns";
        return false;
    }
    while (csv.next()) {
        const auto it = byIdent_.find(toUpper(csv.field(cApt)));
        bool thrLatOk, thrLonOk, endLatOk, endLonOk, hdgOk;
        Runway r;
        r.thrLatDeg = num(csv.field(cThrLat), thrLatOk);
        r.thrLonDeg = num(csv.field(cThrLon), thrLonOk);
        r.endLatDeg = num(csv.field(cEndLat), endLatOk);
        r.endLonDeg = num(csv.field(cEndLon), endLonOk);
        r.headingTrueDeg = static_cast<float>(num(csv.field(cHdg), hdgOk));
        if (it == byIdent_.end() || csv.field(cIdent).empty() || !thrLatOk || !thrLonOk ||
            !endLatOk || !endLonOk || !hdgOk) {
            ++stats_.skippedRows;
            continue;
        }
        Airport& apt = airports_[static_cast<size_t>(it->second)];
        r.airportIdent = apt.ident;
        r.ident = std::string(csv.field(cIdent));
        r.oppositeIdent = std::string(csv.field(cOpp));
        r.thrElevFt = static_cast<float>(num(csv.field(cThrElev), apt.elevFt));
        r.startLatDeg = num(csv.field(cStartLat), r.thrLatDeg);
        r.startLonDeg = num(csv.field(cStartLon), r.thrLonDeg);
        r.endElevFt = static_cast<float>(num(csv.field(cEndElev), r.thrElevFt));
        r.lengthFt = static_cast<float>(num(csv.field(cLen)));
        r.widthFt = static_cast<float>(num(csv.field(cWid)));
        r.displacedFt = static_cast<float>(num(csv.field(cDisp)));
        r.hard = num(csv.field(cHard)) != 0;
        r.lighted = num(csv.field(cLit)) != 0;
        r.approxGeometry = num(csv.field(cApprox)) != 0;
        apt.runways.push_back(std::move(r));
        ++stats_.runways;
    }
    return true;
}

void CsvNavDatabase::attachIls(const std::string& path) {
    CsvReader csv;
    if (!csv.open(path)) {
        LOG_INFO("nav: no ils.csv (%s); all ILS will be synthesized", path.c_str());
        return;
    }
    const int cApt = csv.column("airport_ident"), cRwy = csv.column("rwy_ident"),
              cIdent = csv.column("loc_ident"), cFreq = csv.column("freq_mhz"),
              cLocLat = csv.column("loc_lat_deg"), cLocLon = csv.column("loc_lon_deg"),
              cCourse = csv.column("loc_course_true_deg"), cWidth = csv.column("loc_width_deg"),
              cHasGs = csv.column("has_gs"), cGsLat = csv.column("gs_lat_deg"),
              cGsLon = csv.column("gs_lon_deg"), cGsElev = csv.column("gs_elev_ft"),
              cGsAngle = csv.column("gs_angle_deg"), cTch = csv.column("tch_ft");
    if (cApt < 0 || cRwy < 0 || cLocLat < 0 || cLocLon < 0) {
        LOG_WARN("nav: ils.csv has no usable columns; ignored");
        return;
    }
    while (csv.next()) {
        const auto it = byIdent_.find(toUpper(csv.field(cApt)));
        Runway* rwy = nullptr;
        if (it != byIdent_.end()) {
            for (Runway& r : airports_[static_cast<size_t>(it->second)].runways)
                if (r.ident == csv.field(cRwy)) rwy = &r;
        }
        bool latOk, lonOk;
        Ils ils;
        ils.locLatDeg = num(csv.field(cLocLat), latOk);
        ils.locLonDeg = num(csv.field(cLocLon), lonOk);
        if (!rwy || !latOk || !lonOk) {
            ++stats_.skippedRows;
            continue;
        }
        const std::string_view ident = csv.field(cIdent);
        std::memcpy(ils.ident, ident.data(), std::min(ident.size(), sizeof ils.ident - 1));
        ils.freqMhz = static_cast<float>(num(csv.field(cFreq)));
        ils.locCourseTrueDeg =
            static_cast<float>(num(csv.field(cCourse), static_cast<double>(rwy->headingTrueDeg)));
        // Empty width (NASR has none): derive it from the geometry like a synthesized ILS.
        bool widthOk;
        const double width = num(csv.field(cWidth), widthOk);
        ils.locWidthDeg = widthOk ? static_cast<float>(width)
                                  : locWidthDegForGeometry(ils.locLatDeg, ils.locLonDeg,
                                                           rwy->thrLatDeg, rwy->thrLonDeg);
        bool gsLatOk, gsLonOk;
        ils.gsLatDeg = num(csv.field(cGsLat), gsLatOk);
        ils.gsLonDeg = num(csv.field(cGsLon), gsLonOk);
        // has_gs empty is read as "yes if the glideslope position is present".
        bool hasGsOk;
        const double hasGs = num(csv.field(cHasGs), hasGsOk);
        ils.hasGs = (hasGsOk ? hasGs != 0 : true) && gsLatOk && gsLonOk;
        if (!ils.hasGs) ils.gsLatDeg = ils.gsLonDeg = 0;
        ils.gsElevFt = static_cast<float>(num(csv.field(cGsElev), rwy->thrElevFt));
        ils.gsAngleDeg = static_cast<float>(num(csv.field(cGsAngle), kDefaultGsAngleDeg));
        ils.tchFt = static_cast<float>(num(csv.field(cTch), kDefaultTchFt));
        ils.synthesized = false;
        rwy->ils = ils;
        ++stats_.realIls;
    }
}

void CsvNavDatabase::synthesizeMissingIls() {
    for (Airport& apt : airports_) {
        for (Runway& r : apt.runways) {
            if (r.ils || !runwayQualifiesForSynthIls(r)) continue;
            r.ils = synthesizeIls(r);
            ++stats_.synthIls;
        }
    }
}

void CsvNavDatabase::loadNavaids(const std::string& path) {
    CsvReader csv;
    if (!csv.open(path)) {
        LOG_WARN("nav: no navaids.csv (%s)", path.c_str());
        return;
    }
    const int cIdent = csv.column("ident"), cType = csv.column("type"), cName = csv.column("name"),
              cFreq = csv.column("freq_khz"), cLat = csv.column("lat_deg"),
              cLon = csv.column("lon_deg"), cElev = csv.column("elev_ft"),
              cVar = csv.column("magvar_deg"), cRange = csv.column("range_nm");
    if (cIdent < 0 || cLat < 0 || cLon < 0) {
        LOG_WARN("nav: navaids.csv has no usable columns; ignored");
        return;
    }
    while (csv.next()) {
        Navaid n;
        bool latOk, lonOk;
        n.latDeg = num(csv.field(cLat), latOk);
        n.lonDeg = num(csv.field(cLon), lonOk);
        if (csv.field(cIdent).empty() || !latOk || !lonOk ||
            !parseNavaidType(csv.field(cType), n.type)) {
            ++stats_.skippedRows;
            continue;
        }
        n.ident = toUpper(csv.field(cIdent));
        n.name = std::string(csv.field(cName));
        n.freqKhz = static_cast<float>(num(csv.field(cFreq)));
        n.elevFt = static_cast<float>(num(csv.field(cElev)));
        bool varOk;
        const double var = num(csv.field(cVar), varOk);
        n.magVarDeg = varOk ? static_cast<float>(var) : magVarDeg(n.latDeg, n.lonDeg);
        n.rangeNm = static_cast<float>(num(csv.field(cRange)));
        navaids_.push_back(std::move(n));
    }
    std::stable_sort(navaids_.begin(), navaids_.end(),
                     [](const Navaid& a, const Navaid& b) { return a.ident < b.ident; });
    stats_.navaids = navaids_.size();
}

void CsvNavDatabase::loadFixes(const std::string& path) {
    CsvReader csv;
    if (!csv.open(path)) {
        LOG_WARN("nav: no fixes.csv (%s)", path.c_str());
        return;
    }
    const int cIdent = csv.column("ident"), cLat = csv.column("lat_deg"),
              cLon = csv.column("lon_deg");
    if (cIdent < 0 || cLat < 0 || cLon < 0) {
        LOG_WARN("nav: fixes.csv has no usable columns; ignored");
        return;
    }
    while (csv.next()) {
        Waypoint w;
        bool latOk, lonOk;
        w.latDeg = num(csv.field(cLat), latOk);
        w.lonDeg = num(csv.field(cLon), lonOk);
        if (csv.field(cIdent).empty() || !latOk || !lonOk) {
            ++stats_.skippedRows;
            continue;
        }
        w.ident = toUpper(csv.field(cIdent));
        w.kind = WaypointKind::Fix;
        fixes_.push_back(std::move(w));
    }
    std::stable_sort(fixes_.begin(), fixes_.end(),
                     [](const Waypoint& a, const Waypoint& b) { return a.ident < b.ident; });
    stats_.fixes = fixes_.size();
}

bool CsvNavDatabase::load(const std::string& navDir, std::string& error) {
    const auto t0 = now();
    reset();
    navDir_ = navDir;
    const std::string dir = navDir.empty() ? "." : navDir;
    bool ok = loadAirports(Paths::join(dir, "airports.csv"), error) &&
              loadRunways(Paths::join(dir, "runways.csv"), error);
    if (!ok) {
        reset();
        return false;
    }
    // XTST keeps its own hand-built ILS, so the runway counters below only cover data rows.
    std::string magError;
    if (!magvar_.loadFile(Paths::join(dir, "magvar.bin"), magError))
        LOG_WARN("nav: %s; magnetic variation will read 0", magError.c_str());
    attachIls(Paths::join(dir, "ils.csv"));
    synthesizeMissingIls();
    loadNavaids(Paths::join(dir, "navaids.csv"));
    loadFixes(Paths::join(dir, "fixes.csv"));
    buildIndexes();
    stats_.airports = airports_.size();
    stats_.loadMs = std::chrono::duration<double, std::milli>(now() - t0).count();
    LOG_INFO("nav: loaded %zu airports, %zu runways, %zu navaids, %zu fixes, %zu real + %zu "
             "synthesized ILS (%zu rows skipped) in %.0f ms",
             stats_.airports, stats_.runways, stats_.navaids, stats_.fixes, stats_.realIls,
             stats_.synthIls, stats_.skippedRows, stats_.loadMs);
    return true;
}

// ------------------------------------------------------------------------------------------------
// Queries

const Airport* CsvNavDatabase::findAirport(std::string_view ident) const {
    const auto it = byIdent_.find(toUpper(ident));
    return it == byIdent_.end() ? nullptr : &airports_[static_cast<size_t>(it->second)];
}

std::vector<const Airport*> CsvNavDatabase::nearestAirports(double latDeg, double lonDeg,
                                                            int maxCount, float maxDistNm) const {
    if (maxCount < 1 || maxDistNm < 0) return {};
    const double maxDistM = nmToM(maxDistNm);
    // Conservative lat/lon box around the circle, then exact great-circle filtering.
    const double dLatDeg = radToDeg(maxDistM / kEarthRadiusM);
    const double south = latDeg - dLatDeg, north = latDeg + dLatDeg;
    const double edgeLat = std::min(89.9, std::max(std::fabs(south), std::fabs(north)));
    const double dLonDeg = dLatDeg / std::cos(degToRad(edgeLat));
    const int latLo = latCell(std::max(south, -90.0)), latHi = latCell(std::min(north, 90.0));
    const bool allLon = north >= 90.0 || south <= -90.0 || dLonDeg >= 180.0;
    const int lonLoFloor = static_cast<int>(std::floor(lonDeg - dLonDeg));
    const int lonHiFloor = static_cast<int>(std::floor(lonDeg + dLonDeg));
    const int lonCount = allLon ? kGridLonCells : lonHiFloor - lonLoFloor + 1;

    std::vector<std::pair<double, int>> hits;
    for (int la = latLo; la <= latHi; ++la) {
        for (int k = 0; k < lonCount; ++k) {
            const int lo = lonCellWrapped(allLon ? k - 180 : lonLoFloor + k);
            for (int idx :
                 grid_[static_cast<size_t>(la) * kGridLonCells + static_cast<size_t>(lo)]) {
                const Airport& a = airports_[static_cast<size_t>(idx)];
                const double d = greatCircleDistM(latDeg, lonDeg, a.latDeg, a.lonDeg);
                if (d <= maxDistM) hits.emplace_back(d, idx);
            }
        }
    }
    const size_t n = std::min(hits.size(), static_cast<size_t>(maxCount));
    std::partial_sort(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(n), hits.end(),
                      [this](const auto& a, const auto& b) {
                          if (a.first != b.first) return a.first < b.first;
                          return airports_[static_cast<size_t>(a.second)].ident <
                                 airports_[static_cast<size_t>(b.second)].ident;
                      });
    std::vector<const Airport*> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) out.push_back(&airports_[static_cast<size_t>(hits[i].second)]);
    return out;
}

std::vector<const Airport*> CsvNavDatabase::airportsInBox(double south, double west, double north,
                                                          double east) const {
    std::vector<const Airport*> out;
    if (south > north) return out;
    const bool wraps = west > east;  // box crosses the antimeridian
    auto inLon = [&](double lon) {
        return wraps ? (lon >= west || lon <= east) : (lon >= west && lon <= east);
    };
    const int latLo = latCell(south), latHi = latCell(north);
    const int lonLoFloor = static_cast<int>(std::floor(west));
    const double span = wraps ? (east + 360.0 - west) : (east - west);
    const int lonCount = std::min(kGridLonCells, static_cast<int>(std::floor(span)) + 2);
    for (int la = latLo; la <= latHi; ++la) {
        for (int k = 0; k < lonCount; ++k) {
            const int lo = lonCellWrapped(lonLoFloor + k);
            for (int idx :
                 grid_[static_cast<size_t>(la) * kGridLonCells + static_cast<size_t>(lo)]) {
                const Airport& a = airports_[static_cast<size_t>(idx)];
                if (a.latDeg >= south && a.latDeg <= north && inLon(a.lonDeg)) out.push_back(&a);
            }
        }
    }
    std::sort(out.begin(), out.end(),
              [](const Airport* a, const Airport* b) { return a->ident < b->ident; });
    return out;
}

std::vector<const Airport*> CsvNavDatabase::searchAirports(std::string_view text,
                                                           int maxCount) const {
    const std::string q = std::string(trim(text));
    if (q.empty() || maxCount < 1) return {};
    const std::string qUpper = toUpper(q), qLower = toLower(q);

    struct Hit {
        int tier;
        int idx;
    };
    std::vector<Hit> hits;
    for (size_t i = 0; i < airports_.size(); ++i) {
        const AirportExtra& e = extra_[i];
        int tier = -1;
        if (e.identUpper == qUpper)
            tier = 0;
        else if (!e.iataUpper.empty() && e.iataUpper == qUpper)
            tier = 1;
        else if (startsWith(e.identUpper, qUpper))
            tier = 2;
        else if (!e.iataUpper.empty() && startsWith(e.iataUpper, qUpper))
            tier = 3;
        else if (startsWith(e.muniLower, qLower))
            tier = 4;
        else if (startsWith(e.nameLower, qLower))
            tier = 5;
        else if (wordPrefix(e.nameLower, qLower) || wordPrefix(e.muniLower, qLower))
            tier = 6;
        if (tier >= 0) hits.push_back({tier, static_cast<int>(i)});
    }
    auto less = [this](const Hit& a, const Hit& b) {
        if (a.tier != b.tier) return a.tier < b.tier;
        const Airport& pa = airports_[static_cast<size_t>(a.idx)];
        const Airport& pb = airports_[static_cast<size_t>(b.idx)];
        if (typeRank(pa.type) != typeRank(pb.type)) return typeRank(pa.type) < typeRank(pb.type);
        return pa.ident < pb.ident;
    };
    const size_t n = std::min(hits.size(), static_cast<size_t>(maxCount));
    std::partial_sort(hits.begin(), hits.begin() + static_cast<std::ptrdiff_t>(n), hits.end(),
                      less);
    std::vector<const Airport*> out;
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) out.push_back(&airports_[static_cast<size_t>(hits[i].idx)]);
    return out;
}

std::vector<const Navaid*> CsvNavDatabase::findNavaids(std::string_view ident) const {
    const std::string key = toUpper(ident);
    auto first =
        std::lower_bound(navaids_.begin(), navaids_.end(), key,
                         [](const Navaid& n, const std::string& k) { return n.ident < k; });
    std::vector<const Navaid*> out;
    for (; first != navaids_.end() && first->ident == key; ++first) out.push_back(&*first);
    return out;
}

std::vector<Waypoint> CsvNavDatabase::findFixes(std::string_view ident) const {
    const std::string key = toUpper(ident);
    auto first =
        std::lower_bound(fixes_.begin(), fixes_.end(), key,
                         [](const Waypoint& w, const std::string& k) { return w.ident < k; });
    std::vector<Waypoint> out;
    for (; first != fixes_.end() && first->ident == key; ++first) out.push_back(*first);
    return out;
}

float CsvNavDatabase::magVarDeg(double latDeg, double lonDeg) const {
    return magvar_.declinationDeg(latDeg, lonDeg);
}

const std::vector<AirwayPoint>& CsvNavDatabase::airway(std::string_view designator) const {
    static const std::vector<AirwayPoint> kNone;
    std::call_once(airwaysOnce_, [this] {
        CsvReader csv;
        if (!csv.open(Paths::join(navDir_.empty() ? "." : navDir_, "airways.csv"))) return;
        const int cAw = csv.column("airway"), cSeq = csv.column("seq"),
                  cFix = csv.column("fix_ident"), cLat = csv.column("fix_lat_deg"),
                  cLon = csv.column("fix_lon_deg"), cMea = csv.column("mea_ft");
        if (cAw < 0 || cSeq < 0 || cLat < 0 || cLon < 0) return;
        while (csv.next()) {
            bool latOk, lonOk, seqOk;
            AirwayPoint p;
            p.latDeg = num(csv.field(cLat), latOk);
            p.lonDeg = num(csv.field(cLon), lonOk);
            p.seq = static_cast<int>(num(csv.field(cSeq), seqOk));
            if (!latOk || !lonOk || !seqOk || csv.field(cAw).empty()) continue;
            p.fixIdent = toUpper(csv.field(cFix));
            p.meaFt = static_cast<float>(num(csv.field(cMea)));
            airways_[toUpper(csv.field(cAw))].push_back(std::move(p));
        }
        for (auto& [name, pts] : airways_)
            std::stable_sort(
                pts.begin(), pts.end(),
                [](const AirwayPoint& a, const AirwayPoint& b) { return a.seq < b.seq; });
    });
    const auto it = airways_.find(toUpper(designator));
    return it == airways_.end() ? kNone : it->second;
}

}  // namespace fsim
