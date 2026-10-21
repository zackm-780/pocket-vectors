#pragma once
// Track-up route inset on the HUD (WP H7). Drawn into the open Renderer2D batch after the flight
// HUD. Hidden by the caller in HUD MIN and while the map page is open. No per-frame allocation.

#include "core/AircraftState.h"
#include "core/NavState.h"
#include "hud/MiniNavLogic.h"
#include "hud/Renderer2D.h"

namespace fsim {

class MiniNav {
public:
    // `plan` may be null (the box says NO PLAN). `fixedRangeNm` <= 0 follows the next waypoint.
    void draw(Renderer2D& r, const AircraftState& ac, const NavState& nav, const FlightPlan* plan,
              float fixedRangeNm) const;
};

}  // namespace fsim
