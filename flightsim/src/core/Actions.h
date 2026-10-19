#pragma once
// Semantic input actions. P2 binds physical inputs (keyboard, gamepad) to these.

namespace fsim {

enum class Axis { Pitch, Roll, Rudder, ThrottleRate, LookYaw, LookPitch, Count };

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
    Count
};

struct ActionState {
    float axis[static_cast<int>(Axis::Count)] = {};   // -1..1 after deadzone/curve
    bool down[static_cast<int>(Action::Count)] = {};  // held
    bool pressed[static_cast<int>(Action::Count)] =
        {};  // edge this frame (auto-repeat where bound)
};

}  // namespace fsim
