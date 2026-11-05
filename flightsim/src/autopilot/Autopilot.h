#pragma once
// Autopilot, flight director and autothrottle (WP N6, NAVIGATION 5, D11).
//
// Generic cascaded controllers driven by AircraftState, NavState and the AutopilotState bugs:
//   lateral outer  ROL / HDG / NAV / LOC  -> bank command (deg, + = right wing down)
//   vertical outer PIT / ALT / ALT* / VS / GS / FLC / GA -> pitch command (deg, + = nose up);
//                  FLC and GA fly pitch on speed through the VS loop (N7)
//   inner          bank -> aileron (+ turn-coordination rudder), pitch -> elevator, with
//                  AP-managed pitch trim (the elevator integrator is offloaded into elevTrimNorm)
//   speed          SPD (PI on IAS -> all throttles), RETARD to idle in the flare, and THR
//                  (fixed throttle: TOGA in GA, climb or idle in FLC)
// With the AP off and the FD on, the same outer loops only produce fdPitchCmdDeg / fdRollCmdDeg.
//
// update() is called once per 120 Hz sim step; handleActions() once per frame. Neither
// allocates. Headings are TRUE internally; the heading bug is magnetic (converted with
// NavState::magVarDeg, + = east).
//
// The autopilot never edits the pilot's ControlInputs; syncPilot() lets the flight loop
// follow the autothrottle (levers move) and the AP trim, so handing
// control back to the pilot is bumpless.

#include <string>
#include <vector>

#include "autopilot/ApConfig.h"
#include "core/Actions.h"
#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/AutopilotState.h"
#include "core/ControlInputs.h"
#include "core/NavState.h"

namespace fsim {

class Autopilot {
public:
    Autopilot() = default;

    // Loads gains from the profile and clears all state (modes, bugs, integrators).
    void configure(const AircraftProfile& profile);
    const ApConfig& config() const { return cfg_; }

    // Final controls for this step. Pass-through when neither AP nor A/T is active.
    ControlInputs update(const ControlInputs& pilot, const AircraftState& ac, const NavState& nav,
                         double dtS);

    // Pilot-facing actions (CONTROLS.md AP layer): engage, modes, bugs. Call once per frame.
    void handleActions(const ActionState& actions, const AircraftState& ac, const NavState& nav,
                       double dtS);

    // Scenario [autopilot] section. Tokens (lower case): ap fd at hdg nav apr alt vs rol pit flc
    // toga (toga turns the FD on, not the AP).
    // A mode token engages the AP when it is not engaged yet.
    void applyScenario(const std::vector<std::string>& engageTokens, const AircraftState& ac,
                       const NavState& nav);

    // Lets the persistent pilot controls follow the autothrottle and the AP-managed trim.
    void syncPilot(ControlInputs& pilot);

    // ---- direct commands (also used by handleActions / scenarios / tests) ------------------
    bool engageAp(const AircraftState& ac, const NavState& nav);
    void disengageAp(bool warn);  // intentional disconnect = warn too (HUD flashing AP OFF)
    bool toggleAp(const AircraftState& ac, const NavState& nav);
    void setFd(bool on, const AircraftState& ac);
    bool setAt(bool on, const AircraftState& ac, const AircraftProfile* profile = nullptr);
    void selectHdg(const AircraftState& ac);      // second press with HDG active -> ROL
    void selectNav(const NavState& nav);          // second press with NAV active/armed -> cancel
    void selectApr(const NavState& nav);          // arms LOC + GS (or NAV on a non-ILS approach)
    void selectAltHold(const AircraftState& ac);  // hold the present altitude
    // The ALT button (L1 + R2): holds the present altitude; pressed again while in ALT / ALT*
    // with the bug more than 100 ft away, climbs or descends toward the bug in VS at the default
    // rate with ALT armed (1.1 device test: there was no way out of ALT on the pad).
    void selectAltButton(const AircraftState& ac);
    void selectVs(const AircraftState& ac);
    void selectRoll(const AircraftState& ac);
    void selectPitch(const AircraftState& ac);
    // N7 TO/GA (airborne only): GA pitch on speed (attitude capped at gaPitchDeg) with the IAS at
    // TO/GA as the target, at least Vref with flaps out or minClimbIasKt clean; A/T to TOGA
    // when the aircraft has one, HDG on the present heading with NAV armed when the plan has an
    // active leg, LOC / GS arms cleared, FD on when neither AP nor FD was. The altitude bug moves
    // to 1,000 ft above (rounded to 100 ft) unless it is already 500 ft or more above. Leaves
    // the AP engaged below minimums. Returns false on the ground.
    bool selectGoAround(const AircraftState& ac, const NavState& nav);
    // N7 FLC: climb or descend to the altitude bug at the speed bug (set to the present IAS when
    // unset), pitch on speed; with the A/T on it holds climb thrust or idle. Within 100 ft of the
    // bug it holds the bug altitude instead; a second press with FLC active selects PIT.
    void selectFlc(const AircraftState& ac);
    void syncBugs(const AircraftState& ac, const NavState& nav);
    void setHdgBugMagDeg(float deg);
    void setAltBugFt(float ft);
    void setVsBugFpm(float fpm);
    void setSpdBugKt(float kt);
    void acknowledgeWarning() { st_.disconnectWarning = false; }
    // Tells the autopilot the aircraft has an autothrottle (normally from configure()).
    bool hasAutothrottle() const { return hasAt_; }

    AutopilotState& state() { return st_; }
    const AutopilotState& state() const { return st_; }

    // Commanded bank / pitch of the outer loops (deg), for tests and traces.
    float bankCmdDeg() const { return bankCmdDeg_; }
    float pitchCmdDeg() const { return pitchCmdDeg_; }
    float apElevTrimNorm() const { return apTrimNorm_; }

private:
    bool modesLive() const { return st_.apEngaged || st_.fdOn; }
    void initModes(const AircraftState& ac);
    void clearModes();
    void initLoops(const ControlInputs& pilot, const AircraftState& ac);
    void checkDisconnects(const ControlInputs& pilot, const AircraftState& ac, const NavState& nav,
                          double dtS);
    void updateModes(const AircraftState& ac, const NavState& nav);
    bool navAvailable(const NavState& nav) const;
    bool captureOk(const AircraftState& ac, const NavState& nav, bool loc) const;
    float lateralBankCmd(const AircraftState& ac, const NavState& nav);
    float turnLawBankDeg(const AircraftState& ac, float errDeg, float tauS) const;
    float verticalPitchCmd(const AircraftState& ac, const NavState& nav, double dtS);
    float vsLoopPitch(const AircraftState& ac, float vsTargetFpm, double dtS, float maxPitchDeg);
    float speedOnPitchVs(const AircraftState& ac, float targetKt) const;
    void updateIasAccel(const AircraftState& ac, double dtS);
    float thrustTargetNorm(const AircraftState& ac) const;
    void armAltitudeCapture(const AircraftState& ac);
    void updateAutothrottle(const ControlInputs& pilot, const AircraftState& ac, double dtS,
                            ControlInputs& out);
    void innerLoops(const ControlInputs& pilot, const AircraftState& ac, double dtS,
                    ControlInputs& out);
    float hdgBugTrueDeg(const NavState& nav) const;
    float currentTrackDeg(const AircraftState& ac) const;

    ApConfig cfg_;
    AutopilotState st_;
    bool hasAt_ = false;
    int engineCount_ = 1;
    int approachFlapDetent_ = 0;

    // modes / guidance state
    float rollHoldDeg_ = 0;
    float pitchHoldDeg_ = 0;
    float holdAltFt_ = 0;     // ALT target
    float captureAltFt_ = 0;  // ALT* target
    float vsBugRawFpm_ = 0, spdBugRaw_ = 0;
    float magVarDeg_ = 0;                     // last NavState value, for bug conversion in handlers
    float bankCmdDeg_ = 0, pitchCmdDeg_ = 0;  // slewed commands

    // vertical PI (pitch command) state
    float vsIntegPitchDeg_ = 0;

    // inner loops
    bool loopsInit_ = false;
    float elevInteg_ = 0, ailInteg_ = 0;
    float apTrimNorm_ = 0, lastSyncTrim_ = 0;
    float stickOverS_ = 0;

    // autothrottle
    bool atInit_ = false;
    float atInteg_ = 0, atThrottle_ = 0, lastSyncThrottle_ = 0, lastIasKt_ = 0, iasAccelKtps_ = 0;
    bool atSynced_ = false;

    float warnTimerS_ = 0;
    bool trimSynced_ = false;
    bool vsFresh_ = true;  // vertical PI needs a bumpless (re)start
    AircraftState lastAc_;
    float hdgStep_ = 0;
    float bugAccumVs_ = 0, bugAccumSpd_ = 0;
    bool retardLatched_ = false;

    // N7: speed-on-pitch (FLC / GA) and fixed thrust
    float gaSpeedKt_ = 0;     // GA speed target: IAS at TO/GA, at least the floor below
    float gaFlapsMinKt_ = 0;  // GA floor with flaps out: Vref (1.3 Vs0 without one)
    bool iasAccelInit_ = false;
    float apLastIasKt_ = 0, apIasAccelKtps_ = 0;  // AP-side low-passed IAS rate (kt/s)
    bool thrustWas_ = false;                      // A/T was in THR on the last step
};

}  // namespace fsim
