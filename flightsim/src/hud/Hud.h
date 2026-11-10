#pragma once
// Flight HUD (WP H2, HUD.md sections 1-4): conformal horizon and pitch ladder, bank scale,
// boresight, flight path vector, heading tape (magnetic), speed and altitude tapes, both status
// rows, warnings, transient messages and the FULL / STD / MIN declutter levels.
//
// Usage per frame, after the 3D scene and with the camera pose already set from the aircraft:
//     r2d.begin(w, h);
//     hud.draw(r2d, camera, state, controls, profile, messages, options);
//     r2d.end();
// draw() only records geometry into the open Renderer2D batch (one draw call per frame in end()),
// formats text into fixed buffers and never allocates per frame.
//
// H3 hooks: the reserved zones (hudzones:: in HudLogic.h: FMA row, nav box, LOC and GS scales,
// heading tape marks) are left empty by H2. An H3 HudOverlay registered with setOverlay() is
// called from draw() after the flight symbology and before warnings and messages (so warnings
// always stay on top), with everything it needs to place symbols conformally.

#include "core/Actions.h"
#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"
#include "core/MessageQueue.h"
#include "gfx/Camera.h"
#include "hud/HudLogic.h"
#include "hud/HudProjector.h"
#include "hud/Renderer2D.h"
#include "hud/SpeedBugs.h"

namespace fsim {

struct HudFrame {
    Renderer2D& r;
    const gfx::Camera& camera;
    const HudProjector& proj;
    const AircraftState& state;
    const ControlInputs& controls;
    const AircraftProfile& profile;
    const HudOptions& options;
    const HudVisibility& visibility;
    Color color;          // base HUD color (green / white / amber setting)
    float headingMagDeg;  // current magnetic heading, tape center
    double nowS;          // HUD clock (MessageQueue::nowS())
};

// H3 extension point (FMA row, nav box, LOC / GS scales, heading tape marks, FD cue, minimums).
// X3c: also called in the external chase view (frame.options.externalView), where an overlay
// draws only what belongs off the cockpit eye point (NavHud: the FMA row).
class HudOverlay {
public:
    virtual ~HudOverlay() = default;
    virtual void draw(const HudFrame& frame) = 0;
};

class Hud {
public:
    void draw(Renderer2D& r, const gfx::Camera& camera, const AircraftState& state,
              const ControlInputs& controls, const AircraftProfile& profile,
              const MessageQueue& messages, const HudOptions& options);

    // Applies this frame's HUD actions (Action::HudDeclutter cycles FULL -> STD -> MIN) and posts
    // the feedback message ("HUD STD"). Returns true when the level changed.
    static bool applyActions(const ActionState& actions, HudOptions& options,
                             MessageQueue* messages);

    void setOverlay(HudOverlay* overlay) { overlay_ = overlay; }
    // Forgets the speed trend, the warning ages and the speed-bug phase (new flight, restart).
    void reset();

private:
    SpeedTrend trend_;
    WarningTracker warnings_;
    SpeedBugTracker bugs_;  // H9
    HudOverlay* overlay_ = nullptr;
};

}  // namespace fsim
