#pragma once
// Semantic input actions. P2 binds physical inputs (keyboard, gamepad) to these.

namespace fsim {

// BugRate (additive, P2): L1 + right stick Y, rate-based VS / speed bug adjust, +1 = increase.
enum class Axis { Pitch, Roll, Rudder, ThrottleRate, LookYaw, LookPitch, BugRate, Count };

enum class Action {
    TrimNoseUp,
    TrimNoseDown,
    FlapsUp,
    FlapsDown,
    GearToggle,
    Brakes,
    ParkingBrake,
    SpeedbrakeCycle,
    ReverseToggle,
    HudDeclutter,
    MapToggle,
    Pause,
    LookHold,
    LookLeft,
    LookRight,
    LookBack,
    LookDown,
    ApToggle,
    AtToggle,
    FdToggle,
    HdgMode,
    NavMode,
    AprMode,
    AltMode,
    VsMode,
    HdgBugDec,
    HdgBugInc,
    AltBugDec,
    AltBugInc,
    SyncBugs,
    NavSourceCycle,
    DirectTo,
    RudderTrimLeft,
    RudderTrimRight,
    MenuUp,
    MenuDown,
    MenuLeft,
    MenuRight,
    MenuAccept,
    MenuBack,
    Screenshot,
    Quit,  // additive (P2): Select+Start chord or Ctrl+Q
    Count
};

struct ActionState {
    float axis[static_cast<int>(Axis::Count)] = {};   // -1..1 after deadzone/curve
    bool down[static_cast<int>(Action::Count)] = {};  // held
    bool pressed[static_cast<int>(Action::Count)] =
        {};  // edge this frame (auto-repeat where bound)
    // Additive (P2): auto-repeats fired so far in the current hold (0 on the first press).
    // Consumers use it to accelerate bug steps (see input/InputCurves.h acceleratedStep).
    int repeatCount[static_cast<int>(Action::Count)] = {};
};

}  // namespace fsim
