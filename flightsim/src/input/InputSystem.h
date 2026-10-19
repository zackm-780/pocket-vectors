#pragma once
// SDL events -> ActionState (ARCHITECTURE section 3, "input.poll()"). Gamepad (SDL
// GameController API, hot-plug aware) and keyboard. Tests inject synthetic SDL_Event structs.
//
// Frame use:
//   while (SDL_PollEvent(&e)) input.handleEvent(e);
//   const ActionState& actions = input.update(dtS);
//
// `pressed` edges accumulated by handleEvent are published by the next update() and live for
// exactly that one frame. Keyboard events are ignored when FLIGHTSIM_PLATFORM=portmaster
// (gptokeyb may inject keys, BUILD_AND_DEPLOY section 6).

#include <SDL.h>

#include "core/Actions.h"
#include "input/ControlsConfig.h"

namespace fsim {

class InputSystem {
public:
    static constexpr int kMaxControllers = 4;

    InputSystem();
    ~InputSystem();
    InputSystem(const InputSystem&) = delete;
    InputSystem& operator=(const InputSystem&) = delete;

    // Replaces the bindings (tests, and loadFiles below). Resets all held state.
    void setConfig(const ControlsConfig& config);
    // Loads assets/controls_default.ini and the optional user override; false if the defaults
    // are unreadable (built-in empty bindings remain: nothing works but nothing crashes).
    bool loadFiles(const std::string& defaultsPath, const std::string& userPath);

    // Initialises the SDL GameController subsystem and opens the pads already attached.
    // Optional: tests never call it.
    void openControllers();

    void handleEvent(const SDL_Event& event);
    // Advances timers (auto-repeat, double-tap, keyboard ramps) and publishes the frame state.
    const ActionState& update(double dtS);
    const ActionState& actions() const { return state_; }

    // SDL_QUIT, the Select+Start chord or Ctrl+Q was seen (sticky).
    bool quitRequested() const { return quit_; }

    void setKeyboardEnabled(bool enabled);
    bool keyboardEnabled() const { return keyboardEnabled_; }
    int controllerCount() const { return controllerCount_; }
    const ControlsConfig& config() const { return cfg_; }

private:
    static constexpr int kMaxHeldActions = 4;
    static constexpr int kMaxHeldAxes = 3;
    static constexpr int kMaxKeyHolds = 24;

    struct PadHold {
        Action actions[kMaxHeldActions] = {};
        int count = 0;
    };
    struct KeyHold {
        int keycode = 0;
        Action actions[kMaxHeldActions] = {};
        int actionCount = 0;
        Axis axes[kMaxHeldAxes] = {};
        float signs[kMaxHeldAxes] = {};
        int axisCount = 0;
    };

    void resetState();
    PadLayer padLayer() const;
    void padPress(PadInput input);
    void padRelease(PadInput input);
    void padAxisMotion(int sdlAxis, int raw);
    void releaseAllPad();
    void keyPress(const SDL_KeyboardEvent& key);
    void keyRelease(const SDL_KeyboardEvent& key);

    void holdAction(Action a);
    void releaseAction(Action a);
    void pulseAction(Action a);

    void addController(int deviceIndex);
    void removeController(int instanceId);

    ControlsConfig cfg_;
    ActionState state_;

    // held-source reference counts and per-frame edges
    int refs_[kActionCount] = {};
    bool pending_[kActionCount] = {};
    float repeatTimerS_[kActionCount] = {};
    int repeatCount_[kActionCount] = {};

    // gamepad
    bool padDown_[kPadInputCount] = {};
    PadHold padHolds_[kPadInputCount];
    float stickRaw_[4] = {};  // LeftX, LeftY, RightX, RightY, -1..1
    SDL_GameController* controllers_[kMaxControllers] = {};
    int controllerIds_[kMaxControllers] = {};
    int controllerCount_ = 0;

    // keyboard
    KeyHold keyHolds_[kMaxKeyHolds];
    int keyHoldCount_ = 0;
    bool lookKeyHeld_ = false;
    float keyAxis_[kAxisCount] = {};
    bool keyboardEnabled_ = true;

    double clockS_ = 0;
    double lastTapS_ = -1e9;
    bool quit_ = false;
};

}  // namespace fsim
