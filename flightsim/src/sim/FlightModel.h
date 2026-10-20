#pragma once
// Flight model wrapper around JSBSim (ARCHITECTURE section 7). Property map, per-model overrides
// and crash rules are documented in src/sim/README.md.
//
// Sign conventions at this boundary are ours (CONVENTIONS section 3): elevator +1 = nose up,
// rudder +1 = yaw right, aileron +1 = roll right. JSBSim uses opposite signs for elevator and
// rudder (verified in tests/sim); the wrapper converts. No other module talks to JSBSim.

#include <memory>
#include <string>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"
#include "core/SimTime.h"
#include "core/TerrainQuery.h"

namespace fsim {

// Fully resolved start condition (the app turns a scenario + nav DB into one of these).
struct StartCondition {
    enum class Kind { Runway, Final, Cruise };
    Kind kind = Kind::Runway;
    double latDeg = 0, lonDeg = 0;  // geodetic start position (runway: on the ground)
    double altMslFt = 0;            // Final/Cruise only; Runway sits on the ground
    double fieldElevFt = 0;         // ground elevation under the start point (flat ground)
    double headingTrueDeg = 0;
    double iasKt = 0;          // Final/Cruise: calibrated airspeed
    double flightPathDeg = 0;  // Final: -3 for a 3 degree glidepath; Cruise: 0
    int flapDetent = 0;        // profile detent index (Final: approach config)
    bool gearDown = true;
    float windFromTrueDeg = 0, windSpeedKt = 0;
};

class FlightModel {
public:
    // jsbsimRoot: directory holding aircraft/, engine/ and systems/ (<build>/jsbsim).
    explicit FlightModel(std::string jsbsimRoot);
    ~FlightModel();
    FlightModel(const FlightModel&) = delete;
    FlightModel& operator=(const FlightModel&) = delete;

    // (Re)loads the aircraft and puts it in the start condition: engines running; for Final and
    // Cruise starts the aircraft is configured, trimmed, and the trim is moved into the trim
    // controls so a centered stick holds the trimmed flight. Returns false (and logs) on failure.
    // Any previous JSBSim instance is destroyed first, so calling this repeatedly does not leak.
    bool start(const AircraftProfile& profile, const StartCondition& start);
    // Starts again with the profile and start condition of the last start() call.
    bool restart();
    bool started() const;

    // One fixed step (dtS should be kSimDtS). Cheap: no allocation, no string work.
    // After a crash the model freezes (step does nothing) until the next start().
    void step(const ControlInputs& in, double dtS = kSimDtS);

    const AircraftState& state() const { return state_; }
    // Controls matching the start condition (trimmed throttle and trims, flaps, gear, brakes).
    // The pilot-control layer should begin from these so nothing jumps at the first step.
    const ControlInputs& startControls() const { return startControls_; }

    // F1 additions (additive) ---------------------------------------------------------------
    // Terrain used for the terrain-slope and water crash rules. Optional (nullptr = none); not
    // owned, must outlive the model. F3 also installs its JSBSim ground callback in this class.
    void setTerrain(const TerrainQuery* terrain);
    // Profile of the running aircraft (valid after a successful start()).
    const AircraftProfile& profile() const;

    // X1 additions (additive) ---------------------------------------------------------------
    // Wind the aircraft flies in, JSBSim convention (where the air moves TO), ft/s. Call before
    // step(); the start-condition wind is switched on by start() as before.
    void setWindNedFps(double northFps, double eastFps, double downFps);
    // Real sea-level pressure and the altimeter setting; fills AircraftState::altBaroOffsetFt.
    void setBaro(double qnhHpa, double altimeterSettingHpa);

    // Direct JSBSim property access for tests and debugging only.
    double getProperty(const std::string& path) const;
    void setProperty(const std::string& path, double value);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    AircraftState state_;
    ControlInputs startControls_;
};

}  // namespace fsim
