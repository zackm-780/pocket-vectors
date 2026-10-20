#pragma once
// Nav and autopilot HUD overlay (WP H3, HUD.md sections 5-6). Plugs into H2's Hud through
// Hud::setOverlay(); draws into the reserved zones after the flight symbology and before warnings:
//   * FMA row: A/T, lateral, vertical (armed in white), AP / FD / A/T status, nav summary
//   * nav box (right of the lower scales, left of the VS / RA readouts), CDI/LOC and GS/VNAV scales
//   * heading tape marks: heading bug, DTK/CRS bar, bearing-to-waypoint diamond
//   * speed, altitude and minimums bugs on H2's tapes, flight director cue, MINIMUMS callout
// Everything is recorded into the open Renderer2D batch (no extra draw calls) and text goes
// through fixed buffers. The decisions live in NavHudLogic.h (unit-tested without GL).
//
// Per frame, before Hud::draw: navHud.setStates(navState, autopilotState).

#include "core/AutopilotState.h"
#include "core/NavState.h"
#include "hud/Hud.h"
#include "hud/NavHudLogic.h"

namespace fsim {

class NavHud : public HudOverlay {
public:
    void setStates(const NavState& nav, const AutopilotState& ap) {
        nav_ = nav;
        ap_ = ap;
    }
    void draw(const HudFrame& frame) override;
    // New flight: forgets mode-change boxes and the minimums crossing.
    void reset();
    // True once per minimums crossing (hook for the audio alert tone, S1).
    bool consumeMinimumsAlert() { return minimums_.consumeAlert(); }

private:
    void drawFma(const HudFrame& f);
    void drawNavBox(const HudFrame& f);
    void drawScales(const HudFrame& f);
    void drawHeadingMarks(const HudFrame& f);
    void drawBugs(const HudFrame& f);
    void drawFdCue(const HudFrame& f);

    NavState nav_;
    AutopilotState ap_;
    FmaChangeTracker fma_;
    MinimumsTracker minimums_;
};

}  // namespace fsim
