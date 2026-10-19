#include "nav/Fms.h"

#include <cctype>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "core/Log.h"

namespace fsim {
namespace {

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

bool parseDouble(std::string_view tok, double& out) {
    std::string t(tok);
    char* end = nullptr;
    errno = 0;
    out = std::strtod(t.c_str(), &end);
    return end != t.c_str() && *end == '\0' && errno == 0 && std::isfinite(out);
}

bool parseInt(std::string_view tok, int& out) {
    std::string t(tok);
    char* end = nullptr;
    errno = 0;
    long v = std::strtol(t.c_str(), &end, 10);
    if (end == t.c_str() || *end != '\0' || errno != 0) return false;
    out = static_cast<int>(v);
    return true;
}

std::string upper(std::string_view s) {
    std::string r(s);
    for (char& c : r) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return r;
}

std::string err(int line, const std::string& msg) {
    return "line " + std::to_string(line) + ": " + msg;
}

std::string fmt(const char* f, double v) {
    char buf[64];
    std::snprintf(buf, sizeof buf, f, v);
    return buf;
}

}  // namespace

bool decodeLatLonIdent(std::string_view ident, double& outLatDeg, double& outLonDeg) {
    if (ident.empty() || (ident[0] != '+' && ident[0] != '-')) return false;
    size_t split = std::string_view::npos;
    bool skip = false;  // separator char is not part of the second number
    for (size_t i = 1; i < ident.size(); ++i) {
        if (ident[i] == '_' || ident[i] == ',') {
            split = i;
            skip = true;
            break;
        }
        if (ident[i] == '+' || ident[i] == '-') {
            split = i;
            break;
        }
    }
    if (split == std::string_view::npos) return false;
    double lat, lon;
    if (!parseDouble(ident.substr(0, split), lat) ||
        !parseDouble(ident.substr(skip ? split + 1 : split), lon))
        return false;
    if (std::fabs(lat) > 90.0 || std::fabs(lon) > 180.0) return false;
    outLatDeg = lat;
    outLonDeg = lon;
    return true;
}

std::string encodeLatLonIdent(double latDeg, double lonDeg) {
    char buf[64];
    std::snprintf(buf, sizeof buf, "%+.3f_%+.3f", latDeg, lonDeg);
    return buf;
}

std::string fmsRunwayToDb(std::string_view r) {
    if (r.size() > 2 && (r[0] == 'R' || r[0] == 'r') && (r[1] == 'W' || r[1] == 'w'))
        r.remove_prefix(2);
    return upper(r);
}

std::string dbRunwayToFms(std::string_view r) {
    return "RW" + std::string(r);
}

FmsParseResult parseFms(std::string_view text) {
    FmsParseResult res;
    FmsFile& f = res.file;
    if (text.size() >= 3 && static_cast<unsigned char>(text[0]) == 0xEF &&
        static_cast<unsigned char>(text[1]) == 0xBB && static_cast<unsigned char>(text[2]) == 0xBF)
        text.remove_prefix(3);

    int lineNo = 0;
    int headerLines = 0;  // 0: expect I/A, 1: expect version, 2: body
    size_t pos = 0;
    while (pos <= text.size()) {
        size_t nl = text.find('\n', pos);
        std::string_view raw =
            text.substr(pos, nl == std::string_view::npos ? text.size() - pos : nl - pos);
        pos = nl == std::string_view::npos ? text.size() + 1 : nl + 1;
        ++lineNo;
        auto tok = splitWs(raw);  // also strips \r
        if (tok.empty()) continue;

        if (headerLines == 0) {
            if (tok.size() != 1 || (tok[0] != "I" && tok[0] != "A")) {
                res.errors.push_back(err(lineNo, "first line must be 'I' or 'A', found '" +
                                                     std::string(raw.substr(0, 40)) +
                                                     "' (is this an X-Plane .fms file?)"));
                return res;
            }
            headerLines = 1;
            continue;
        }
        if (headerLines == 1) {
            int v = 0;
            if (tok.size() < 2 || upper(tok[1]) != "VERSION" || !parseInt(tok[0], v)) {
                res.errors.push_back(err(lineNo, "second line must be '1100 Version'"));
                return res;
            }
            if (v != 1100) {
                res.errors.push_back(err(lineNo, "unsupported .fms version " + std::to_string(v) +
                                                     " (only 1100 is supported)"));
                return res;
            }
            f.version = v;
            headerLines = 2;
            continue;
        }

        const bool isNum = std::isdigit(static_cast<unsigned char>(tok[0][0])) != 0;
        if (!isNum) {
            std::string key = upper(tok[0]);
            if (key != "NUMENR" && key != "CYCLE" && key != "ADEP" && key != "DEP" &&
                key != "DEPRWY" && key != "SID" && key != "SIDTRANS" && key != "ADES" &&
                key != "DES" && key != "DESRWY" && key != "STAR" && key != "STARTRANS" &&
                key != "APP" && key != "APPTRANS") {
                LOG_INFO("fms: line %d: ignoring unknown key '%s'", lineNo, key.c_str());
                res.warnings.push_back(err(lineNo, "ignored unknown key '" + key + "'"));
                continue;
            }
            if (tok.size() < 2) {
                res.errors.push_back(err(lineNo, key + " needs a value"));
                continue;
            }
            std::string val(tok[1]);
            if (key == "NUMENR") {
                int n = 0;
                if (!parseInt(tok[1], n) || n < 0)
                    res.errors.push_back(err(lineNo, "NUMENR needs a non-negative integer"));
                else
                    f.numEnr = n;
            } else if (key == "CYCLE") {
                f.cycle = val;
            } else if (key == "ADEP") {
                f.adep = upper(val);
            } else if (key == "DEP") {
                f.dep = upper(val);
            } else if (key == "DEPRWY") {
                f.deprwy = upper(val);
            } else if (key == "SID") {
                f.sid = val;
            } else if (key == "SIDTRANS") {
                f.sidTrans = val;
            } else if (key == "ADES") {
                f.ades = upper(val);
            } else if (key == "DES") {
                f.des = upper(val);
            } else if (key == "DESRWY") {
                f.desrwy = upper(val);
            } else if (key == "STAR") {
                f.star = val;
            } else if (key == "STARTRANS") {
                f.starTrans = val;
            } else if (key == "APP") {
                f.app = upper(val);
            } else if (key == "APPTRANS") {
                f.appTrans = val;
            }
            continue;
        }

        // Entry: type ident via altitude lat lon
        FmsEntry e;
        e.line = lineNo;
        int type = 0;
        if (!parseInt(tok[0], type) ||
            (type != 1 && type != 2 && type != 3 && type != 11 && type != 28)) {
            res.errors.push_back(err(lineNo, "unknown waypoint type '" + std::string(tok[0]) +
                                                 "' (expected 1, 2, 3, 11 or 28)"));
            continue;
        }
        if (tok.size() != 6) {
            res.errors.push_back(
                err(lineNo, "waypoint needs 6 fields (type ident via altitude lat lon), found " +
                                std::to_string(tok.size())));
            continue;
        }
        e.type = static_cast<FmsEntryType>(type);
        e.ident = upper(tok[1]);
        e.via = std::string(tok[2]);
        double alt, lat, lon;
        if (!parseDouble(tok[3], alt)) {
            res.errors.push_back(err(lineNo, "bad altitude '" + std::string(tok[3]) + "'"));
            continue;
        }
        if (!parseDouble(tok[4], lat) || std::fabs(lat) > 90.0) {
            res.errors.push_back(
                err(lineNo, "bad latitude '" + std::string(tok[4]) + "' (need -90..90)"));
            continue;
        }
        if (!parseDouble(tok[5], lon) || std::fabs(lon) > 180.0) {
            res.errors.push_back(
                err(lineNo, "bad longitude '" + std::string(tok[5]) + "' (need -180..180)"));
            continue;
        }
        e.altFt = static_cast<float>(alt);
        e.latDeg = lat;
        e.lonDeg = lon;
        f.entries.push_back(std::move(e));
    }

    if (res.errors.empty()) {
        if (headerLines == 0) {
            res.errors.push_back("file is empty");
        } else if (headerLines == 1) {
            res.errors.push_back("missing '1100 Version' line");
        } else if (f.entries.size() < 2) {
            res.errors.push_back(
                "file has fewer than two waypoints (need at least origin and destination)");
        } else {
            if (f.numEnr >= 0 && f.numEnr != static_cast<int>(f.entries.size()))
                res.warnings.push_back("NUMENR says " + std::to_string(f.numEnr) +
                                       " but the file has " + std::to_string(f.entries.size()) +
                                       " waypoints; using the latter");
            if (f.adep.empty() && f.dep.empty())
                res.warnings.push_back("no ADEP/DEP line; origin taken from the first waypoint");
            if (f.ades.empty() && f.des.empty())
                res.warnings.push_back(
                    "no ADES/DES line; destination taken from the last waypoint");
        }
    }
    res.ok = res.errors.empty();
    return res;
}

std::string writeFms(const FmsFile& f) {
    std::string out = "I\n1100 Version\n";
    out += "CYCLE " + (f.cycle.empty() ? std::string("0000") : f.cycle) + "\n";
    auto kv = [&](const char* k, const std::string& v) {
        if (!v.empty()) out += std::string(k) + " " + v + "\n";
    };
    kv("ADEP", f.adep);
    kv("DEP", f.dep);
    kv("DEPRWY", f.deprwy);
    kv("SID", f.sid);
    kv("SIDTRANS", f.sidTrans);
    kv("ADES", f.ades);
    kv("DES", f.des);
    kv("DESRWY", f.desrwy);
    kv("STAR", f.star);
    kv("STARTRANS", f.starTrans);
    kv("APP", f.app);
    kv("APPTRANS", f.appTrans);
    out += "NUMENR " + std::to_string(f.entries.size()) + "\n";
    for (const FmsEntry& e : f.entries) {
        out += std::to_string(static_cast<int>(e.type)) + " " + e.ident + " " +
               (e.via.empty() ? "DRCT" : e.via) + " " + fmt("%.6f", e.altFt) + " " +
               fmt("%.6f", e.latDeg) + " " + fmt("%.6f", e.lonDeg) + "\n";
    }
    return out;
}

}  // namespace fsim
