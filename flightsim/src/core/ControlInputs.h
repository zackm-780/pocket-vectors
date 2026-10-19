#pragma once
// Pilot / autopilot commands. Produced by input/PilotControls, modified by autopilot/, consumed by
// sim/FlightModel. Sign conventions: elevator +1 = nose up (stick back), aileron +1 = roll right,
// rudder +1 = yaw right.

#include "core/AircraftState.h"  // kMaxEngines

namespace fsim {

struct ControlInputs {
    float elevatorNorm = 0, aileronNorm = 0, rudderNorm = 0;  // -1..1
    float throttleNorm[kMaxEngines] = {0, 0, 0, 0};           // 0..1
    bool reverseThrust = false;
    float mixtureNorm = 1;                                            // pistons
    float elevTrimNorm = 0, aileronTrimNorm = 0, rudderTrimNorm = 0;  // absolute, -1..1
    int flapDetent = 0;                                               // index into profile detents
    bool gearDown = true;
    float speedbrakeNorm = 0;  // 0..1
    float brakeLeftNorm = 0, brakeRightNorm = 0;
    bool parkingBrake = false;
    float steeringNorm = 0;  // -1..1, usually follows rudder
};

}  // namespace fsim
