#pragma once
// Autopilot / flight director / autothrottle status. Produced by autopilot/ (N6), read by the HUD.

namespace fsim {

enum class LatMode { Off, Roll, Hdg, Nav, Loc };
enum class VertMode { Off, Pitch, Alt, AltCapture, Vs, Gs, Flc };
enum class SpdMode { Off, Speed, Retard };

struct AutopilotState {
    bool apEngaged = false, fdOn = false, atEngaged = false;
    LatMode latActive = LatMode::Off, latArmed = LatMode::Off;
    VertMode vertActive = VertMode::Off, vertArmed = VertMode::Off;
    SpdMode spdActive = SpdMode::Off;
    float hdgBugMagDeg = 0, altBugFt = 0, vsBugFpm = 0, spdBugKt = 0;
    float fdPitchCmdDeg = 0, fdRollCmdDeg = 0;
    bool fdValid = false;
    bool disconnectWarning = false;  // flashing AP OFF
};

}  // namespace fsim
