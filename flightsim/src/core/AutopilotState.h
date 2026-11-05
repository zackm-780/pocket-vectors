#pragma once
// Autopilot / flight director / autothrottle status. Produced by autopilot/ (N6), read by the HUD.

namespace fsim {

enum class LatMode { Off, Roll, Hdg, Nav, Loc };
// Additive (N7): Ga = go-around (TO/GA), pitch on speed capped at the go-around attitude.
enum class VertMode { Off, Pitch, Alt, AltCapture, Vs, Gs, Flc, Ga };
// Additive (N7): Thrust = the A/T holds a fixed throttle (thrustTargetNorm): TOGA in GA, climb or
// idle in FLC.
enum class SpdMode { Off, Speed, Retard, Thrust };

struct AutopilotState {
    bool apEngaged = false, fdOn = false, atEngaged = false;
    LatMode latActive = LatMode::Off, latArmed = LatMode::Off;
    VertMode vertActive = VertMode::Off, vertArmed = VertMode::Off;
    SpdMode spdActive = SpdMode::Off;
    float hdgBugMagDeg = 0, altBugFt = 0, vsBugFpm = 0, spdBugKt = 0;
    float fdPitchCmdDeg = 0, fdRollCmdDeg = 0;
    bool fdValid = false;
    bool disconnectWarning = false;  // flashing AP OFF
    // Additive (N7): throttle the A/T drives to in SpdMode::Thrust (0 = idle, 1 = full).
    float thrustTargetNorm = 0;
};

}  // namespace fsim
