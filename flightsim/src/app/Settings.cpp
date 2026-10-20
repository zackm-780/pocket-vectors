#include "app/Settings.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <string_view>
#include <utility>
#include <vector>

#if !defined(_WIN32)
#include <unistd.h>
#endif

#include "core/Ini.h"
#include "core/Log.h"

namespace fsim {
namespace {

constexpr const char* kSection = "settings";

template <typename E> struct EnumName {
    const char* name;
    E value;
};

constexpr EnumName<HudColor> kHudColors[] = {
    {"green", HudColor::Green}, {"white", HudColor::White}, {"amber", HudColor::Amber}};
constexpr EnumName<HudDeclutter> kDeclutters[] = {
    {"full", HudDeclutter::Full}, {"std", HudDeclutter::Std}, {"min", HudDeclutter::Min}};
constexpr EnumName<PressureUnits> kPressure[] = {{"hpa", PressureUnits::Hpa},
                                                 {"inhg", PressureUnits::InHg}};
constexpr EnumName<GraphicsQuality> kQuality[] = {
    {"low", GraphicsQuality::Low}, {"med", GraphicsQuality::Med}, {"high", GraphicsQuality::High}};
constexpr EnumName<ThrottleMode> kThrottle[] = {{"rate", ThrottleMode::Rate},
                                                {"absolute", ThrottleMode::Absolute}};

template <typename E, size_t N>
bool parseEnum(const EnumName<E> (&table)[N], const std::string& text, E& out) {
    const std::string t = toLower(text);
    for (const auto& e : table) {
        if (t == e.name) {
            out = e.value;
            return true;
        }
    }
    return false;
}

template <typename E, size_t N> const char* enumText(const EnumName<E> (&table)[N], E value) {
    for (const auto& e : table)
        if (e.value == value) return e.name;
    return table[0].name;
}

std::string fmtFloat(float v) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "%.6g", static_cast<double>(v));
    return buf;
}

bool parseFloatStrict(const std::string& s, float& out) {
    char* end = nullptr;
    const float v = std::strtof(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0' || !(v == v)) return false;  // also rejects NaN
    out = v;
    return true;
}

bool parseIntStrict(const std::string& s, int& out) {
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') return false;
    out = static_cast<int>(std::clamp<long>(v, -1000000, 1000000));
    return true;
}

bool parseBoolStrict(const std::string& s, bool& out) {
    const std::string t = toLower(s);
    if (t == "1" || t == "true" || t == "yes" || t == "on") {
        out = true;
        return true;
    }
    if (t == "0" || t == "false" || t == "no" || t == "off") {
        out = false;
        return true;
    }
    return false;
}

}  // namespace

void Settings::clampAll() {
    // NaN-safe: std::clamp of NaN would pass it through, so reset those first.
    auto fix = [](float& v, float lo, float hi, float def) {
        v = (v == v) ? std::clamp(v, lo, hi) : def;
    };
    fix(fovVerticalDeg, 30.0f, 110.0f, 55.0f);
    for (float* s : {&pitchSens, &rollSens, &rudderSens}) fix(*s, 0.25f, 2.0f, 1.0f);
    for (float* e : {&pitchExpo, &rollExpo, &rudderExpo}) fix(*e, 0.0f, 1.0f, 0.0f);
    volumePct = std::clamp(volumePct, 0, 100);
    if (simHz != 60 && simHz != 120) simHz = 120;
}

int Settings::applyText(const std::string& text) {
    Ini ini;
    ini.loadText(text);
    int rejected = 0;
    auto reject = [&rejected](const char* key, const std::string& value) {
        LOG_WARN("settings: ignoring invalid value '%s' for %s", value.c_str(), key);
        ++rejected;
    };
    auto str = [&](const char* key, std::string& dst) {
        if (ini.has(kSection, key)) dst = ini.getString(kSection, key);
    };
    auto num = [&](const char* key, float& dst) {
        if (!ini.has(kSection, key)) return;
        const std::string v = ini.getString(kSection, key);
        float f = 0;
        if (parseFloatStrict(v, f))
            dst = f;
        else
            reject(key, v);
    };
    auto integer = [&](const char* key, int& dst) {
        if (!ini.has(kSection, key)) return;
        const std::string v = ini.getString(kSection, key);
        int i = 0;
        if (parseIntStrict(v, i))
            dst = i;
        else
            reject(key, v);
    };
    auto flag = [&](const char* key, bool& dst) {
        if (!ini.has(kSection, key)) return;
        const std::string v = ini.getString(kSection, key);
        bool b = false;
        if (parseBoolStrict(v, b))
            dst = b;
        else
            reject(key, v);
    };
    auto choice = [&](const char* key, auto& table, auto& dst) {
        if (!ini.has(kSection, key)) return;
        const std::string v = ini.getString(kSection, key);
        if (!parseEnum(table, v, dst)) reject(key, v);
    };

    str("data_dir", dataDir);
    num("fov_vertical_deg", fovVerticalDeg);
    flag("show_fps", showFps);
    choice("hud_color", kHudColors, hudColor);
    choice("hud_declutter_default", kDeclutters, hudDeclutterDefault);
    choice("pressure_units", kPressure, pressureUnits);
    integer("sim_hz", simHz);
    choice("graphics_quality", kQuality, graphicsQuality);
    integer("volume_pct", volumePct);
    flag("assist_auto_rudder", assistAutoRudder);
    flag("assist_auto_trim", assistAutoTrim);
    choice("throttle_mode", kThrottle, throttleMode);
    num("pitch_sensitivity", pitchSens);
    num("roll_sensitivity", rollSens);
    num("rudder_sensitivity", rudderSens);
    num("pitch_expo", pitchExpo);
    num("roll_expo", rollExpo);
    num("rudder_expo", rudderExpo);
    str("last_aircraft", lastAircraft);

    if (ini.has(kSection, "sim_hz") && simHz != 60 && simHz != 120)
        reject("sim_hz", ini.getString(kSection, "sim_hz"));
    clampAll();
    return rejected;
}

bool Settings::load(const std::string& defaultsPath, const std::string& userPath) {
    *this = Settings{};
    std::string text;
    if (readTextFile(defaultsPath, text))
        applyText(text);
    else
        LOG_WARN("settings: defaults file '%s' not readable; using built-in defaults",
                 defaultsPath.c_str());
    if (userPath.empty() || !fileExists(userPath)) {
        LOG_INFO("settings: no user settings at '%s'; using defaults", userPath.c_str());
        return true;
    }
    if (!readTextFile(userPath, text)) {
        LOG_WARN("settings: cannot read '%s'; using defaults", userPath.c_str());
        return false;
    }
    if (text.find('\0') != std::string::npos) {
        LOG_WARN("settings: '%s' is corrupt (binary data); using defaults", userPath.c_str());
        return true;
    }
    const int rejected = applyText(text);
    if (rejected > 0)
        LOG_WARN("settings: %d invalid value(s) in '%s' replaced by defaults", rejected,
                 userPath.c_str());
    return true;
}

StickTuning Settings::stickTuning() const {
    StickTuning t;
    t.pitchSens = pitchSens;
    t.rollSens = rollSens;
    t.rudderSens = rudderSens;
    t.pitchExpo = pitchExpo;
    t.rollExpo = rollExpo;
    t.rudderExpo = rudderExpo;
    t.throttleAbsolute = throttleMode == ThrottleMode::Absolute;
    return t;
}

namespace {

std::vector<std::pair<std::string, std::string>> settingsPairs(const Settings& s) {
    auto b = [](bool v) { return std::string(v ? "true" : "false"); };
    return {
        {"data_dir", s.dataDir},
        {"fov_vertical_deg", fmtFloat(s.fovVerticalDeg)},
        {"show_fps", b(s.showFps)},
        {"hud_color", enumText(kHudColors, s.hudColor)},
        {"hud_declutter_default", enumText(kDeclutters, s.hudDeclutterDefault)},
        {"pressure_units", enumText(kPressure, s.pressureUnits)},
        {"sim_hz", std::to_string(s.simHz)},
        {"graphics_quality", enumText(kQuality, s.graphicsQuality)},
        {"volume_pct", std::to_string(s.volumePct)},
        {"assist_auto_rudder", b(s.assistAutoRudder)},
        {"assist_auto_trim", b(s.assistAutoTrim)},
        {"throttle_mode", enumText(kThrottle, s.throttleMode)},
        {"pitch_sensitivity", fmtFloat(s.pitchSens)},
        {"roll_sensitivity", fmtFloat(s.rollSens)},
        {"rudder_sensitivity", fmtFloat(s.rudderSens)},
        {"pitch_expo", fmtFloat(s.pitchExpo)},
        {"roll_expo", fmtFloat(s.rollExpo)},
        {"rudder_expo", fmtFloat(s.rudderExpo)},
        {"last_aircraft", s.lastAircraft},
    };
}

}  // namespace

std::string Settings::toKeyValueText() const {
    std::string out;
    for (const auto& [k, v] : settingsPairs(*this)) out += k + " = " + v + "\n";
    return out;
}

bool Settings::save(const std::string& userPath) const {
    std::string existing;
    if (fileExists(userPath) && readTextFile(userPath, existing) &&
        existing.find('\0') != std::string::npos)
        existing.clear();  // corrupt: do not carry binary garbage forward

    std::vector<std::pair<std::string, std::string>> pending = settingsPairs(*this);
    std::vector<std::string> lines;
    {
        size_t pos = 0;
        while (pos < existing.size()) {
            size_t nl = existing.find('\n', pos);
            if (nl == std::string::npos) nl = existing.size();
            std::string line = existing.substr(pos, nl - pos);
            if (!line.empty() && line.back() == '\r') line.pop_back();
            lines.push_back(std::move(line));
            pos = nl + 1;
        }
    }

    std::string section;
    bool sawSettings = false;
    size_t insertAt = lines.size();  // where missing keys go: end of the [settings] section
    for (size_t i = 0; i < lines.size(); ++i) {
        const std::string_view t = trim(lines[i]);
        if (t.empty() || t[0] == ';' || t[0] == '#') continue;
        if (t.front() == '[') {
            if (section == kSection && insertAt == lines.size()) insertAt = i;
            const size_t close = t.find(']');
            section = toLower(
                trim(t.substr(1, close == std::string_view::npos ? t.size() - 1 : close - 1)));
            if (section == kSection) sawSettings = true;
            continue;
        }
        if (section != kSection) continue;
        const size_t eq = t.find('=');
        if (eq == std::string_view::npos) continue;
        const std::string key = toLower(trim(t.substr(0, eq)));
        for (auto it = pending.begin(); it != pending.end(); ++it) {
            if (it->first == key) {
                lines[i] = key + " = " + it->second;
                pending.erase(it);
                break;
            }
        }
    }
    if (section == kSection && insertAt == lines.size()) insertAt = lines.size();
    // Don't append after trailing blank lines of the section.
    while (sawSettings && insertAt > 0 && trim(lines[insertAt - 1]).empty()) --insertAt;

    std::vector<std::string> add;
    if (!sawSettings) {
        if (!lines.empty()) add.push_back("");
        add.push_back("[settings]");
        insertAt = lines.size();
    }
    for (const auto& [k, v] : pending) add.push_back(k + " = " + v);
    lines.insert(lines.begin() + static_cast<std::ptrdiff_t>(insertAt), add.begin(), add.end());

    std::string out;
    if (existing.empty())
        out = "; flightsim settings. Comments are whole lines; see assets/settings_default.ini.\n";
    for (const std::string& l : lines) out += l + "\n";
    return writeFileAtomic(userPath, out);
}

bool writeFileAtomic(const std::string& path, const std::string& text) {
    const std::string tmp = path + ".tmp";
    std::FILE* f = std::fopen(tmp.c_str(), "wb");
    if (!f) {
        LOG_ERROR("cannot write '%s'", tmp.c_str());
        return false;
    }
    bool ok = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    ok = std::fflush(f) == 0 && ok;
#if !defined(_WIN32)
    ok = ::fsync(::fileno(f)) == 0 && ok;
#endif
    ok = std::fclose(f) == 0 && ok;
    if (ok) {
#if defined(_WIN32)
        std::remove(path.c_str());  // rename does not replace on Windows
#endif
        ok = std::rename(tmp.c_str(), path.c_str()) == 0;
    }
    if (!ok) {
        std::remove(tmp.c_str());
        LOG_ERROR("cannot write '%s'", path.c_str());
    }
    return ok;
}

PathOverrides effectivePathOverrides(const CliOptions& cli, const Settings& settings) {
    PathOverrides o{cli.assetDir, cli.dataDir, cli.confDir};
    if (o.dataDir.empty()) o.dataDir = settings.dataDir;
    return o;
}

bool effectiveShowFps(const CliOptions& cli, const Settings& settings) {
    return cli.showFps || settings.showFps;
}

std::string scenarioToIni(const Scenario& sc) {
    auto num = [](double v) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%.6g", v);
        return std::string(buf);
    };
    std::string out = "; flightsim last flight (written on exit; loaded by --resume)\n[scenario]\n";
    out += "aircraft = " + sc.aircraftId + "\n";
    out += "airport = " + sc.airportIdent + "\n";
    out += "runway = " + sc.runwayIdent + "\n";
    out += "start = " + sc.startKind + "\n";
    out += "distance_nm = " + num(sc.distanceNm) + "\n";
    out += "altitude_ft = " + num(sc.altitudeFt) + "\n";
    if (!sc.timeUtc.empty()) out += "time_utc = " + sc.timeUtc + "\n";
    if (!sc.planPath.empty()) out += "plan = " + sc.planPath + "\n";          // N5
    if (!sc.approachId.empty()) out += "approach = " + sc.approachId + "\n";  // N5
    out += "\n[wind]\nfrom_true_deg = " + num(sc.windFromTrueDeg) +
           "\nspeed_kt = " + num(sc.windSpeedKt) + "\n";
    if (!sc.apEngage.empty() || sc.hasHdgBug || sc.hasAltBug) {
        out += "\n[autopilot]\n";
        if (!sc.apEngage.empty()) {
            out += "engage =";
            for (const std::string& m : sc.apEngage) out += " " + m;
            out += "\n";
        }
        if (sc.hasHdgBug) out += "hdg_bug_mag_deg = " + num(sc.hdgBugMagDeg) + "\n";
        if (sc.hasAltBug) out += "alt_bug_ft = " + num(sc.altBugFt) + "\n";
    }
    return out;
}

bool writeLastFlight(const std::string& path, const Scenario& scenario) {
    return writeFileAtomic(path, scenarioToIni(scenario));
}

}  // namespace fsim
