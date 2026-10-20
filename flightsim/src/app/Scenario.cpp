#include "app/Scenario.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>

#include "core/Geo.h"
#include "core/Ini.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"

namespace fsim {
namespace {

struct CommandName {
    const char* name;
    ScriptCommand command;
};
constexpr CommandName kCommandNames[] = {
    {"throttle", ScriptCommand::Throttle},
    {"elevator", ScriptCommand::Elevator},
    {"aileron", ScriptCommand::Aileron},
    {"rudder", ScriptCommand::Rudder},
    {"steering", ScriptCommand::Steering},
    {"brakes", ScriptCommand::Brakes},
    {"parkingbrake", ScriptCommand::ParkingBrake},
    {"flaps", ScriptCommand::Flaps},
    {"gear", ScriptCommand::Gear},
    {"trim", ScriptCommand::Trim},
    {"mixture", ScriptCommand::Mixture},
    {"speedbrake", ScriptCommand::Speedbrake},
};

bool parseDoubleStrict(const std::string& s, double& out) {
    char* end = nullptr;
    out = std::strtod(s.c_str(), &end);
    return end != s.c_str() && *end == '\0';
}

}  // namespace

bool parseScenario(const std::string& iniText, Scenario& out, std::string& error) {
    Ini ini;
    ini.loadText(iniText);
    Scenario s;
    s.aircraftId = ini.getString("scenario", "aircraft", s.aircraftId);
    s.airportIdent = ini.getString("scenario", "airport", s.airportIdent);
    s.runwayIdent = ini.getString("scenario", "runway", s.runwayIdent);
    s.startKind = toLower(ini.getString("scenario", "start", s.startKind));
    s.distanceNm = ini.getDouble("scenario", "distance_nm", s.distanceNm);
    s.altitudeFt = ini.getDouble("scenario", "altitude_ft", s.altitudeFt);
    s.timeUtc = ini.getString("scenario", "time_utc");
    s.planPath = ini.getString("scenario", "plan");        // N5
    s.approachId = ini.getString("scenario", "approach");  // N5
    s.windFromTrueDeg = ini.getFloat("wind", "from_true_deg", 0.f);
    s.windSpeedKt = ini.getFloat("wind", "speed_kt", 0.f);
    {  // X1: weather. [wind] keeps working; [weather] keys override it.
        world::WeatherSetup& w = s.weather;
        if (ini.has("wind", "from_true_deg")) w.windFromTrueDeg = s.windFromTrueDeg;
        if (ini.has("wind", "speed_kt")) w.windKt = s.windSpeedKt;
        auto opt = [&](const char* key, std::optional<float>& dst, float scale = 1.0f) {
            if (ini.has("weather", key)) dst = ini.getFloat("weather", key, 0.f) * scale;
        };
        opt("wind_from_true_deg", w.windFromTrueDeg);
        opt("wind_kt", w.windKt);
        opt("gust_kt", w.gustKt);
        opt("aloft_from_true_deg", w.aloftFromTrueDeg);
        opt("aloft_kt", w.aloftKt);
        opt("visibility_m", w.visibilityM);
        opt("visibility_km", w.visibilityM, 1000.0f);
        opt("qnh_hpa", w.qnhHpa);
        opt("qnh_inhg", w.qnhHpa, static_cast<float>(kHpaPerInHg));
        opt("altimeter_hpa", w.altimeterHpa);
        opt("temp_c", w.tempC);
        opt("turbulence", w.turbulence);
        if (ini.has("weather", "metar")) w.metar = ini.getBool("weather", "metar", true);
        w.seed = static_cast<std::uint32_t>(ini.getInt("weather", "seed", 1));
        w.uniformWind = (w.windKt || w.windFromTrueDeg) && !ini.has("weather", "wind_kt") &&
                        !ini.has("weather", "wind_from_true_deg") && !ini.has("weather", "gust_kt");
        if (w.windFromTrueDeg) s.windFromTrueDeg = *w.windFromTrueDeg;
        if (w.windKt) s.windSpeedKt = *w.windKt;
    }

    if (s.startKind != "runway" && s.startKind != "final" && s.startKind != "cruise") {
        error = "[scenario] start must be runway, final or cruise (got '" + s.startKind + "')";
        return false;
    }
    if (s.distanceNm <= 0) {
        error = "[scenario] distance_nm must be positive";
        return false;
    }

    for (const auto& [key, value] : ini.entries("inputs")) {
        ScriptEvent ev;
        if (!parseDoubleStrict(key, ev.timeS) || ev.timeS < 0) {
            error = "[inputs] bad time '" + key + "'";
            return false;
        }
        const std::vector<std::string> parts = splitTrim(value, ' ');
        std::vector<std::string> words;
        for (const std::string& p : parts)
            if (!p.empty()) words.push_back(p);
        double v = 0;
        if (words.size() != 2 || !parseDoubleStrict(words[1], v)) {
            error =
                "[inputs] expected '<time> = <command> <value>', got '" + key + " = " + value + "'";
            return false;
        }
        bool known = false;
        for (const CommandName& c : kCommandNames) {
            if (toLower(words[0]) == c.name) {
                ev.command = c.command;
                known = true;
            }
        }
        if (!known) {
            error = "[inputs] unknown command '" + words[0] + "'";
            return false;
        }
        ev.value = static_cast<float>(v);
        s.inputs.push_back(ev);
    }
    std::stable_sort(s.inputs.begin(), s.inputs.end(),
                     [](const ScriptEvent& a, const ScriptEvent& b) { return a.timeS < b.timeS; });

    for (const std::string& m : splitTrim(ini.getString("autopilot", "engage"), ' '))
        if (!m.empty()) s.apEngage.push_back(toLower(m));
    s.hasHdgBug = ini.has("autopilot", "hdg_bug_mag_deg");
    s.hdgBugMagDeg = ini.getFloat("autopilot", "hdg_bug_mag_deg");
    s.hasAltBug = ini.has("autopilot", "alt_bug_ft");
    s.altBugFt = ini.getFloat("autopilot", "alt_bug_ft");
    s.hasVsBug = ini.has("autopilot", "vs_bug_fpm");  // N6
    s.vsBugFpm = ini.getFloat("autopilot", "vs_bug_fpm");
    s.hasSpdBug = ini.has("autopilot", "spd_bug_kt");
    s.spdBugKt = ini.getFloat("autopilot", "spd_bug_kt");

    out = std::move(s);
    return true;
}

bool loadScenario(const std::string& path, Scenario& out, std::string& error) {
    std::string text;
    if (!readTextFile(path, text)) {
        error = "cannot open scenario '" + path + "'";
        return false;
    }
    if (!parseScenario(text, out, error)) {
        error = path + ": " + error;
        return false;
    }
    return true;
}

bool resolveStart(const Scenario& sc, const NavDatabase& nav, const AircraftProfile& profile,
                  StartCondition& out, std::string& error) {
    const Airport* airport = nav.findAirport(sc.airportIdent);
    if (!airport) {
        error = "airport '" + sc.airportIdent + "' not found in the navigation database";
        return false;
    }
    const Runway* rwy = findRunway(*airport, sc.runwayIdent);
    if (!rwy) {
        error = "runway '" + sc.runwayIdent + "' not found at " + airport->ident;
        return false;
    }

    StartCondition s;
    s.fieldElevFt = rwy->thrElevFt;
    s.headingTrueDeg = rwy->headingTrueDeg;
    s.windFromTrueDeg = sc.windFromTrueDeg;
    s.windSpeedKt = sc.windSpeedKt;
    const double headingRad = degToRad(rwy->headingTrueDeg);

    // Reference approach speed: profile Vapp, else 1.3 x stall speed, else a generic value.
    double vappKt = profile.vappKt;
    if (vappKt <= 0 && profile.vs0Kt > 0) vappKt = 1.3 * profile.vs0Kt;
    if (vappKt <= 0) vappKt = 70.0;

    if (sc.startKind == "runway") {
        s.kind = StartCondition::Kind::Runway;
        // CG a little past the threshold so the nose gear is on the pavement.
        destinationPoint(rwy->thrLatDeg, rwy->thrLonDeg, headingRad, ftToM(60.0), s.latDeg,
                         s.lonDeg);
        s.flapDetent = profile.takeoffFlapDetent;
        s.gearDown = true;
    } else if (sc.startKind == "final") {
        s.kind = StartCondition::Kind::Final;
        const double tchFt = rwy->ils ? rwy->ils->tchFt : 50.0;
        const double glideDeg = rwy->ils ? rwy->ils->gsAngleDeg : 3.0;
        destinationPoint(rwy->thrLatDeg, rwy->thrLonDeg, headingRad + kPi, nmToM(sc.distanceNm),
                         s.latDeg, s.lonDeg);
        s.altMslFt = rwy->thrElevFt + tchFt + nmToFt(sc.distanceNm) * std::tan(degToRad(glideDeg));
        s.iasKt = vappKt;
        s.flightPathDeg = -glideDeg;
        s.flapDetent = profile.approachFlapDetent;
        s.gearDown = true;
    } else {  // cruise: over the field, heading along the runway
        s.kind = StartCondition::Kind::Cruise;
        s.latDeg = airport->latDeg;
        s.lonDeg = airport->lonDeg;
        s.altMslFt = sc.altitudeFt > 0 ? sc.altitudeFt
                                       : (profile.cruiseAltFt > 0 ? profile.cruiseAltFt : 5000.0);
        s.iasKt = profile.cruiseKt > 0 ? profile.cruiseKt : 1.6 * vappKt;
        s.flightPathDeg = 0;
        s.flapDetent = 0;
        s.gearDown = !profile.hasRetractableGear;
    }
    out = s;
    return true;
}

void ScriptedInputs::setEvents(std::vector<ScriptEvent> events) {
    events_ = std::move(events);
    next_ = 0;
}

void ScriptedInputs::apply(double simTimeS, int engineCount, ControlInputs& c) {
    while (next_ < events_.size() && events_[next_].timeS <= simTimeS) {
        const ScriptEvent& e = events_[next_++];
        switch (e.command) {
        case ScriptCommand::Throttle:
            for (int i = 0; i < engineCount && i < kMaxEngines; ++i) c.throttleNorm[i] = e.value;
            break;
        case ScriptCommand::Elevator: c.elevatorNorm = e.value; break;
        case ScriptCommand::Aileron: c.aileronNorm = e.value; break;
        case ScriptCommand::Rudder: c.rudderNorm = e.value; break;
        case ScriptCommand::Steering: c.steeringNorm = e.value; break;
        case ScriptCommand::Brakes: c.brakeLeftNorm = c.brakeRightNorm = e.value; break;
        case ScriptCommand::ParkingBrake: c.parkingBrake = e.value > 0.5f; break;
        case ScriptCommand::Flaps: c.flapDetent = static_cast<int>(e.value); break;
        case ScriptCommand::Gear: c.gearDown = e.value > 0.5f; break;
        case ScriptCommand::Trim: c.elevTrimNorm = e.value; break;
        case ScriptCommand::Mixture: c.mixtureNorm = e.value; break;
        case ScriptCommand::Speedbrake: c.speedbrakeNorm = e.value; break;
        }
    }
}

}  // namespace fsim
