#pragma once
// Pass-through stand-ins for modules built by later work packages, so the loop is complete from
// day one: the real Navigator (N5) and Autopilot (N6) replace these behind the same calls.

#include "core/AircraftState.h"
#include "core/AutopilotState.h"
#include "core/ControlInputs.h"
#include "core/NavState.h"

namespace fsim {

class StubNavigator {
public:
    const NavState& update(const AircraftState&, double /*dtS*/) { return state_; }
    const NavState& state() const { return state_; }

private:
    NavState state_;
};

class StubAutopilot {
public:
    // Pilot controls pass straight through to the flight model.
    ControlInputs update(const ControlInputs& pilot, const AircraftState&, const NavState&,
                         double /*dtS*/) {
        return pilot;
    }
    AutopilotState& state() { return state_; }
    const AutopilotState& state() const { return state_; }

private:
    AutopilotState state_;
};

}  // namespace fsim
