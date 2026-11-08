#pragma once
// 1.3: a scripted pilot who follows the flight director (AP off, FD on), for the FD flights in
// FdFollowTest.cpp and FlyByFlightTest.cpp (N10).
#include <algorithm>

#include "app/Simulation.h"
#include "core/Units.h"

namespace fsim::test {

// A pilot who flies the FPV onto the FD cue. The cue sits (roll command - roll) * 2 px to the side
// of the FPV and (pitch command - pitch) above it, so flying it onto the FPV is a roll and a pitch
// tracking task; this pilot reads both with a human reaction lag and keeps the feet on the floor.
struct FdPilot {
    double lagS = 0.6;
    double rollSeenDeg = 0, pitchSeenDeg = 0, elevInteg = 0;
    bool init = false;
    void step(Simulation& sim) {
        const double dtS = 1.0 / kSimHz;
        const AircraftState& s = sim.aircraft();
        const AutopilotState& a = sim.autopilotState();
        ControlInputs& c = sim.pilot();
        if (!init) {
            rollSeenDeg = a.fdRollCmdDeg;
            pitchSeenDeg = a.fdPitchCmdDeg;
            elevInteg = c.elevatorNorm;
            init = true;
        }
        const double k = std::min(1.0, dtS / lagS);
        rollSeenDeg += (a.fdRollCmdDeg - rollSeenDeg) * k;
        pitchSeenDeg += (a.fdPitchCmdDeg - pitchSeenDeg) * k;
        const double rollDeg = radToDeg(s.rollRad), pitchDeg = radToDeg(s.pitchRad);
        c.aileronNorm = static_cast<float>(
            std::clamp(0.04 * (rollSeenDeg - rollDeg) - 0.02 * radToDeg(s.pRadps), -1.0, 1.0));
        const double pitchErr = pitchSeenDeg - pitchDeg;
        elevInteg = std::clamp(elevInteg + 0.02 * pitchErr * dtS, -0.5, 0.5);
        c.elevatorNorm = static_cast<float>(
            std::clamp(0.08 * pitchErr - 0.05 * radToDeg(s.qRadps) + elevInteg, -1.0, 1.0));
    }
};

}  // namespace fsim::test
