#pragma once
// Test hook (WP H3): canned NavState / AutopilotState for screenshots and manual checks, selected
// with `--hud-fake <name>`. The values are made up around the aircraft's current heading, speed
// and altitude so the symbology lands on the tapes. Not used in normal flights.
//
// Names: gps_enr gps_term gps_apr loc_gs loc_armed hdg_alt vs ap_off minimums fd

#include <cstdint>
#include <string>

#include "core/AircraftState.h"
#include "core/AutopilotState.h"
#include "core/NavState.h"

namespace fsim {

// "gps_enr gps_term ..." for --help and error messages.
const char* hudFakeNames();
// Fills nav / ap (and the extra HUD warning bits) for `name`. `frame` drives the minimums
// scenario (the minimums move past the aircraft altitude at frame 60). Returns false for an
// unknown name (outputs untouched).
bool hudFake(const std::string& name, int frame, const AircraftState& s, float magVarDeg,
             NavState& nav, AutopilotState& ap, std::uint32_t& extraWarnings);

}  // namespace fsim
