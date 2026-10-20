#pragma once
// Scenario files (ARCHITECTURE section 9): aircraft, airport/runway, start type, wind, scripted
// control inputs and autopilot settings. Used by the headless runs, tests and screenshots.

#include <string>
#include <vector>

#include "core/AircraftProfile.h"
#include "core/ControlInputs.h"
#include "core/NavDatabase.h"
#include "sim/FlightModel.h"

namespace fsim {

enum class ScriptCommand {
    Throttle,      // 0..1, all engines
    Elevator,      // -1..1, +1 = nose up
    Aileron,       // -1..1, +1 = roll right
    Rudder,        // -1..1, +1 = yaw right
    Steering,      // -1..1 nosewheel
    Brakes,        // 0..1, both wheel brakes
    ParkingBrake,  // 0 / 1
    Flaps,         // profile detent index
    Gear,          // 0 = up, 1 = down
    Trim,          // elevator trim, -1..1
    Mixture,       // 0..1
    Speedbrake     // 0..1
};

struct ScriptEvent {
    double timeS = 0;
    ScriptCommand command = ScriptCommand::Throttle;
    float value = 0;
};

struct Scenario {
    std::string aircraftId = "c172p";
    std::string airportIdent = "XTST";
    std::string runwayIdent = "27";
    std::string startKind = "runway";  // runway | final | cruise
    double distanceNm = 6;             // final only
    double altitudeFt = 0;             // cruise only (0 = profile cruise altitude)
    std::string timeUtc;               // ISO "YYYY-MM-DDTHH:MM:SS" UTC; drives the sun (W4)
    float windFromTrueDeg = 0, windSpeedKt = 0;
    std::vector<ScriptEvent> inputs;  // sorted by time

    // N5: optional navigation setup. `plan` is an .fms file (resolved like --scenario paths),
    // `approach` an approach id ("I27", "R28L", "VIS09") loaded into that plan, or, without a
    // plan, a one-approach plan at the scenario's airport.
    std::string planPath, approachId;

    // [autopilot] section; applied by the autopilot once N6 lands. Kept so scenarios are final.
    std::vector<std::string> apEngage;
    float hdgBugMagDeg = 0, altBugFt = 0;
    bool hasHdgBug = false, hasAltBug = false;
    float vsBugFpm = 0, spdBugKt = 0;  // N6: optional vs_bug_fpm / spd_bug_kt
    bool hasVsBug = false, hasSpdBug = false;
};

bool parseScenario(const std::string& iniText, Scenario& out, std::string& error);
bool loadScenario(const std::string& path, Scenario& out, std::string& error);

// Turns a scenario + nav database + aircraft profile into a concrete start condition.
bool resolveStart(const Scenario& scenario, const NavDatabase& nav, const AircraftProfile& profile,
                  StartCondition& out, std::string& error);

// Applies timeline events to a persistent ControlInputs as simulation time passes.
class ScriptedInputs {
public:
    void setEvents(std::vector<ScriptEvent> events);
    void reset() { next_ = 0; }
    // Applies every event with timeS <= simTimeS that has not been applied yet.
    void apply(double simTimeS, int engineCount, ControlInputs& controls);

private:
    std::vector<ScriptEvent> events_;
    size_t next_ = 0;
};

}  // namespace fsim
