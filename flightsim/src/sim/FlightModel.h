#pragma once
// Flight model wrapper around JSBSim (ARCHITECTURE section 7).
//
// WP-00 BOOTSTRAP: this is the minimal version that makes the app run end to end. F1 replaces the
// internals (keeping this interface), F3 adds the terrain ground callback.
//
// Sign conventions at this boundary are ours (CONVENTIONS section 3): elevator +1 = nose up,
// rudder +1 = yaw right, aileron +1 = roll right. JSBSim uses opposite signs for elevator and
// rudder (verified in tests/sim/FlightModelTest.cpp); the wrapper converts.

#include <memory>
#include <string>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"
#include "core/SimTime.h"

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
    bool start(const AircraftProfile& profile, const StartCondition& start);
    bool started() const;

    // One fixed step (dtS should be kSimDtS). Cheap: no allocation, no string work.
    void step(const ControlInputs& in, double dtS = kSimDtS);

    const AircraftState& state() const { return state_; }
    // Controls matching the start condition (trimmed throttle and trims, flaps, gear, brakes).
    // The pilot-control layer should begin from these so nothing jumps at the first step.
    const ControlInputs& startControls() const { return startControls_; }

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
