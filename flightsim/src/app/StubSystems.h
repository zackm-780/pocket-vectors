#pragma once
// Pass-through stand-ins for modules built by later work packages, so the loop is complete from
// day one: the real Autopilot (N6) replaces this behind the same calls. (The Navigator stub was
// replaced by nav/Navigator in WP N5.)

#include "core/AircraftState.h"
#include "core/AutopilotState.h"
#include "core/ControlInputs.h"
#include "core/NavState.h"

namespace fsim {

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
