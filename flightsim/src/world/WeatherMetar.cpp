#include "world/WeatherMetar.h"

#include <cctype>
#include <cstdlib>
#include <sstream>

#include "core/Baro.h"
#include "core/Units.h"

namespace fsim::world {

namespace {

constexpr double kMPerStatuteMile = 1609.344;
constexpr float kMpsToKt = static_cast<float>(kKtPerMps);
constexpr float kKmhToKt = 1.0f / 1.852f;

bool allDigits(const std::string& s, size_t from, size_t count) {
    if (from + count > s.size()) return false;
    for (size_t i = from; i < from + count; ++i)
        if (!std::isdigit(static_cast<unsigned char>(s[i]))) return false;
    return true;
}
bool endsWith(const std::string& s, const char* suffix) {
    const std::string x(suffix);
    return s.size() >= x.size() && s.compare(s.size() - x.size(), x.size(), x) == 0;
}
int toInt(const std::string& s, size_t from, size_t count) {
    return std::atoi(s.substr(from, count).c_str());
}

bool parseTimeGroup(const std::string& t, Metar& m) {
    if (t.size() != 7 || t[6] != 'Z' || !allDigits(t, 0, 6)) return false;
    m.day = toInt(t, 0, 2);
    m.hour = toInt(t, 2, 2);
    m.minute = toInt(t, 4, 2);
    return true;
}

// dddff[Ggg]KT | dddff[Ggg]MPS | dddff[Ggg]KMH | VRBff[Ggg]KT (speeds may have 3 digits)
bool parseWind(const std::string& t, Metar& m) {
    float unit = 1.0f;
    size_t end = t.size();
    if (endsWith(t, "KT"))
        end -= 2;
    else if (endsWith(t, "MPS"))
        unit = kMpsToKt, end -= 3;
    else if (endsWith(t, "KMH"))
        unit = kKmhToKt, end -= 3;
    else
        return false;
    bool vrb = false;
    if (t.compare(0, 3, "VRB") == 0)
        vrb = true;
    else if (!allDigits(t, 0, 3))
        return false;
    size_t p = 3;
    size_t q = p;
    while (q < end && std::isdigit(static_cast<unsigned char>(t[q]))) ++q;
    if (q - p < 2 || q - p > 3) return false;
    const float speed = static_cast<float>(std::atoi(t.substr(p, q - p).c_str()));
    float gust = 0;
    if (q < end) {
        if (t[q] != 'G') return false;
        size_t r = q + 1;
        size_t g0 = r;
        while (r < end && std::isdigit(static_cast<unsigned char>(t[r]))) ++r;
        if (r != end || r - g0 < 2 || r - g0 > 3) return false;
        gust = static_cast<float>(std::atoi(t.substr(g0, r - g0).c_str()));
    }
    m.hasWind = true;
    m.windVariable = vrb;
    m.windFromTrueDeg = vrb ? 0.0f : static_cast<float>(toInt(t, 0, 3));
    if (!vrb && m.windFromTrueDeg >= 360.0f) m.windFromTrueDeg -= 360.0f;
    m.windKt = speed * unit;
    m.gustKt = gust * unit;
    if (m.gustKt <= m.windKt) m.gustKt = 0;  // a "gust" not above the mean is noise
    return true;
}

// "1/2SM", "M1/4SM", "10SM", "P6SM", "3SM". `whole` is a preceding integer-only token (or -1).
bool parseVisSm(const std::string& t, int whole, Metar& m) {
    if (!endsWith(t, "SM")) return false;
    std::string body = t.substr(0, t.size() - 2);
    bool plus = false;
    if (!body.empty() && body[0] == 'P') plus = true, body.erase(0, 1);
    if (!body.empty() && body[0] == 'M') body.erase(0, 1);
    if (body.empty()) return false;
    double sm = 0;
    const size_t slash = body.find('/');
    if (slash == std::string::npos) {
        for (char c : body)
            if (!std::isdigit(static_cast<unsigned char>(c))) return false;
        sm = std::atof(body.c_str());
    } else {
        const double num = std::atof(body.substr(0, slash).c_str());
        const double den = std::atof(body.substr(slash + 1).c_str());
        if (den <= 0) return false;
        sm = num / den + (whole > 0 ? whole : 0);
    }
    m.hasVisibility = true;
    m.visibilityM = static_cast<float>(sm * kMPerStatuteMile);
    m.visibilityAtLeast = plus;
    return true;
}

bool parseCloud(const std::string& t, Metar& m) {
    struct Prefix {
        const char* text;
        MetarCloud::Cover cover;
    };
    static const Prefix kPrefixes[] = {{"FEW", MetarCloud::Cover::Few},
                                       {"SCT", MetarCloud::Cover::Scattered},
                                       {"BKN", MetarCloud::Cover::Broken},
                                       {"OVC", MetarCloud::Cover::Overcast},
                                       {"VV", MetarCloud::Cover::VerticalVis}};
    for (const Prefix& p : kPrefixes) {
        const size_t n = std::char_traits<char>::length(p.text);
        if (t.compare(0, n, p.text) != 0) continue;
        MetarCloud c;
        c.cover = p.cover;
        if (allDigits(t, n, 3))
            c.baseAglFt = static_cast<float>(toInt(t, n, 3) * 100);
        else if (t.size() >= n + 3 && t.compare(n, 3, "///") == 0)
            c.baseAglFt = -1;  // base not reported
        else
            return false;
        m.clouds.push_back(c);
        if (c.baseAglFt >= 0 &&
            (c.cover == MetarCloud::Cover::Broken || c.cover == MetarCloud::Cover::Overcast ||
             c.cover == MetarCloud::Cover::VerticalVis) &&
            (m.ceilingFt < 0 || c.baseAglFt < m.ceilingFt))
            m.ceilingFt = c.baseAglFt;
        return true;
    }
    return false;
}

bool parseTempValue(const std::string& s, float& out) {
    if (s.empty()) return false;
    const bool neg = s[0] == 'M';
    const std::string digits = neg ? s.substr(1) : s;
    if (digits.empty() || digits.size() > 2) return false;
    for (char c : digits)
        if (!std::isdigit(static_cast<unsigned char>(c))) return false;
    out = static_cast<float>(std::atoi(digits.c_str())) * (neg ? -1.0f : 1.0f);
    return true;
}

// "12/08", "M02/M05", "12/", "12//" (dewpoint missing)
bool parseTemp(const std::string& t, Metar& m) {
    const size_t slash = t.find('/');
    if (slash == std::string::npos || t.size() > 7) return false;
    float temp = 0;
    if (!parseTempValue(t.substr(0, slash), temp)) return false;
    m.hasTemp = true;
    m.tempC = temp;
    std::string dew = t.substr(slash + 1);
    while (!dew.empty() && dew[0] == '/') dew.erase(0, 1);
    float d = 0;
    if (parseTempValue(dew, d)) m.hasDewPoint = true, m.dewPointC = d;
    return true;
}

bool parseAltimeter(const std::string& t, Metar& m) {
    if (t.size() != 5 || !allDigits(t, 1, 4)) return false;
    if (t[0] == 'A') {
        m.altimeterHpa = static_cast<float>(inHgToHpa(toInt(t, 1, 4) / 100.0));
    } else if (t[0] == 'Q') {
        m.altimeterHpa = static_cast<float>(toInt(t, 1, 4));
    } else {
        return false;
    }
    m.hasAltimeter = true;
    return true;
}

}  // namespace

bool parseMetar(const std::string& text, Metar& out) {
    // Find the first line that holds a time group (ddhhmmZ); that is the report.
    std::istringstream lines(text);
    std::string line;
    std::vector<std::string> tokens;
    bool found = false;
    while (std::getline(lines, line) && !found) {
        std::istringstream words(line);
        std::string w;
        tokens.clear();
        while (words >> w) {
            for (char& c : w) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            tokens.push_back(w);
            Metar probe;
            if (parseTimeGroup(w, probe)) found = true;
        }
    }
    if (!found) return false;

    Metar m;
    bool haveTime = false;
    int wholeSm = -1;  // integer token that may precede a fraction ("1 1/2SM")
    for (size_t i = 0; i < tokens.size(); ++i) {
        const std::string& t = tokens[i];
        if (t == "RMK" || t == "TEMPO" || t == "BECMG" || t == "NOSIG") break;
        if (!haveTime) {
            if (parseTimeGroup(t, m)) {
                haveTime = true;
            } else if (m.station.empty() && t.size() == 4 &&
                       std::isalpha(static_cast<unsigned char>(t[0])) && t != "AUTO" &&
                       t != "SPECI" && t != "NIL") {
                bool ok = true;
                for (char c : t)
                    if (!std::isalnum(static_cast<unsigned char>(c))) ok = false;
                if (ok) m.station = t;
            }
            continue;
        }
        if (t == "CAVOK") {
            m.cavok = true;
            m.hasVisibility = true;
            m.visibilityM = 10000.0f;
            m.visibilityAtLeast = true;
            continue;
        }
        if (!m.hasWind && parseWind(t, m)) continue;
        if (!m.hasVisibility) {
            if (allDigits(t, 0, 4) && t.size() == 4) {
                const int v = toInt(t, 0, 4);
                m.hasVisibility = true;
                m.visibilityM = v >= 9999 ? 10000.0f : static_cast<float>(v);
                m.visibilityAtLeast = v >= 9999;
                continue;
            }
            if ((t.size() <= 2 && allDigits(t, 0, t.size())) && i + 1 < tokens.size() &&
                tokens[i + 1].find('/') != std::string::npos && endsWith(tokens[i + 1], "SM")) {
                wholeSm = std::atoi(t.c_str());
                continue;
            }
            if (parseVisSm(t, wholeSm, m)) continue;
        }
        if (parseCloud(t, m)) continue;
        if (!m.hasAltimeter && (t[0] == 'A' || t[0] == 'Q') && parseAltimeter(t, m)) continue;
        if (!m.hasTemp && (t[0] == 'M' || std::isdigit(static_cast<unsigned char>(t[0]))) &&
            parseTemp(t, m))
            continue;
        // Everything else (RVR, weather phenomena, 180V240, COR, AUTO, ...) is skipped.
    }
    if (m.cavok) m.ceilingFt = -1;
    if (!m.valid()) return false;
    out = m;
    return true;
}

}  // namespace fsim::world
