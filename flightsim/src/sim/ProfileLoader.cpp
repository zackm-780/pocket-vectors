#include "sim/ProfileLoader.h"

#include <cstdlib>
#include <cstring>

#include "core/AircraftState.h"
#include "core/Ini.h"
#include "core/Paths.h"

namespace fsim {
namespace {

AircraftCategory parseCategory(const std::string& s) {
    const std::string v = toLower(s);
    if (v == "jet") return AircraftCategory::Jet;
    if (v == "turboprop") return AircraftCategory::Turboprop;
    return AircraftCategory::Piston;
}

}  // namespace

bool parseAircraftProfile(const std::string& iniText, AircraftProfile& out, std::string& error) {
    Ini ini;
    ini.loadText(iniText);
    if (!ini.hasSection("aircraft")) {
        error = "missing [aircraft] section";
        return false;
    }

    AircraftProfile p;
    p.id = ini.getString("aircraft", "id");
    p.displayName = ini.getString("aircraft", "display_name", p.id);
    p.jsbsimModel = ini.getString("aircraft", "jsbsim_model", p.id);
    if (p.id.empty() || p.jsbsimModel.empty()) {
        error = "[aircraft] id / jsbsim_model missing";
        return false;
    }
    p.category = parseCategory(ini.getString("aircraft", "category", "piston"));
    p.engineCount = ini.getInt("aircraft", "engines", 1);
    if (p.engineCount < 1 || p.engineCount > kMaxEngines) {
        error = "[aircraft] engines must be 1.." + std::to_string(kMaxEngines);
        return false;
    }
    p.hasRetractableGear = ini.getBool("aircraft", "retractable_gear", false);
    p.hasSpeedbrake = ini.getBool("aircraft", "speedbrake", false);
    p.hasReversers = ini.getBool("aircraft", "reversers", false);
    p.hasAutothrottle = ini.getBool("aircraft", "autothrottle", false);

    // "UP:0, 10:0.333, 20:0.667" -> label:cmdNorm pairs
    for (const std::string& item : splitTrim(ini.getString("flaps", "detents"), ',')) {
        const size_t colon = item.find(':');
        if (item.empty() || colon == std::string::npos) continue;
        FlapDetent d;
        std::strncpy(d.label, item.substr(0, colon).c_str(), sizeof d.label - 1);
        d.cmdNorm = std::strtof(item.c_str() + colon + 1, nullptr);
        p.flapDetents.push_back(d);
    }
    if (p.flapDetents.empty()) {
        FlapDetent up;
        std::strncpy(up.label, "UP", sizeof up.label - 1);
        p.flapDetents.push_back(up);
    }
    p.takeoffFlapDetent = ini.getInt("flaps", "takeoff_detent", 0);
    p.approachFlapDetent = ini.getInt("flaps", "approach_detent", 0);

    p.vs0Kt = ini.getFloat("speeds_kt", "vs0");
    p.vs1Kt = ini.getFloat("speeds_kt", "vs1");
    p.vrKt = ini.getFloat("speeds_kt", "vr");
    p.vrefKt = ini.getFloat("speeds_kt", "vref");
    p.vappKt = ini.getFloat("speeds_kt", "vapp");
    p.vfeKt = ini.getFloat("speeds_kt", "vfe");
    p.vnoKt = ini.getFloat("speeds_kt", "vno");
    p.vneKt = ini.getFloat("speeds_kt", "vne");
    p.vmoKt = ini.getFloat("speeds_kt", "vmo");
    p.mmo = ini.getFloat("speeds_kt", "mmo");
    p.cruiseKt = ini.getFloat("cruise", "ias_kt");
    p.cruiseAltFt = ini.getFloat("cruise", "altitude_ft");
    p.stallWarnAlphaDeg = ini.getFloat("warnings", "stall_warn_alpha_deg");

    if (ini.has("view", "eye_offset_m")) {
        const std::vector<std::string> v = splitTrim(ini.getString("view", "eye_offset_m"), ',');
        if (v.size() == 3) {
            for (size_t i = 0; i < 3; ++i) p.eyeOffsetBodyM[i] = std::strtof(v[i].c_str(), nullptr);
            p.hasEyeOffset = true;
        }
    }
    for (const auto& [key, value] : ini.entries("autopilot"))
        p.autopilot[key] = std::strtof(value.c_str(), nullptr);
    for (const auto& [key, value] : ini.entries("properties")) p.properties[key] = value;

    const int detents = static_cast<int>(p.flapDetents.size());
    if (p.approachFlapDetent < 0 || p.approachFlapDetent >= detents || p.takeoffFlapDetent < 0 ||
        p.takeoffFlapDetent >= detents) {
        error = "flap detent index out of range";
        return false;
    }
    out = std::move(p);
    return true;
}

bool loadAircraftProfile(const std::string& path, AircraftProfile& out, std::string& error) {
    std::string text;
    if (!readTextFile(path, text)) {
        error = "cannot open " + path;
        return false;
    }
    if (!parseAircraftProfile(text, out, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

}  // namespace fsim
