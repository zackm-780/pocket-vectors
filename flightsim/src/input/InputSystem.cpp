#include "input/InputSystem.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <iterator>

#include "core/Log.h"
#include "input/InputCurves.h"

namespace fsim {
namespace {

constexpr int idx(Action a) {
    return static_cast<int>(a);
}
constexpr int idx(Axis a) {
    return static_cast<int>(a);
}

// Rate axes (throttle, bug) are used as -1/0/+1 command levels by keyboard keys; the others ramp
// like a spring-centred stick.
bool isRateAxis(int axis) {
    return axis == idx(Axis::ThrottleRate) || axis == idx(Axis::BugRate);
}

unsigned currentMods(Uint16 sdlMod) {
    unsigned m = 0;
    if (sdlMod & KMOD_SHIFT) m |= kModShift;
    if (sdlMod & KMOD_CTRL) m |= kModCtrl;
    if (sdlMod & KMOD_ALT) m |= kModAlt;
    return m;
}

float approach(float value, float target, float rate, float dtS) {
    const float step = rate * dtS;
    if (value < target) return std::min(value + step, target);
    return std::max(value - step, target);
}

}  // namespace

void clearButtons(ActionState& actions) {
    std::fill(std::begin(actions.down), std::end(actions.down), false);
    std::fill(std::begin(actions.pressed), std::end(actions.pressed), false);
    std::fill(std::begin(actions.repeatCount), std::end(actions.repeatCount), 0);
}

InputSystem::InputSystem() {
    const char* platform = std::getenv("FLIGHTSIM_PLATFORM");
    keyboardEnabled_ = !(platform && std::strcmp(platform, "portmaster") == 0);
}

InputSystem::~InputSystem() {
    for (int i = 0; i < controllerCount_; ++i) SDL_GameControllerClose(controllers_[i]);
}

void InputSystem::resetState() {
    std::memset(refs_, 0, sizeof refs_);
    std::memset(pending_, 0, sizeof pending_);
    std::memset(repeatTimerS_, 0, sizeof repeatTimerS_);
    std::memset(repeatCount_, 0, sizeof repeatCount_);
    std::memset(padDown_, 0, sizeof padDown_);
    std::memset(stickRaw_, 0, sizeof stickRaw_);
    std::memset(keyAxis_, 0, sizeof keyAxis_);
    for (PadHold& h : padHolds_) h.count = 0;
    keyHoldCount_ = 0;
    lookKeyHeld_ = false;
    lastTapS_ = -1e9;
    state_ = ActionState{};
}

void InputSystem::setConfig(const ControlsConfig& config) {
    cfg_ = config;
    resetState();
}

bool InputSystem::loadFiles(const std::string& defaultsPath, const std::string& userPath) {
    ControlsConfig cfg;
    const bool ok = cfg.loadFiles(defaultsPath, userPath);
    setConfig(cfg);
    return ok;
}

void InputSystem::setKeyboardEnabled(bool enabled) {
    keyboardEnabled_ = enabled;
    if (!enabled) {
        for (int i = 0; i < keyHoldCount_; ++i)
            for (int a = 0; a < keyHolds_[i].actionCount; ++a)
                releaseAction(keyHolds_[i].actions[a]);
        keyHoldCount_ = 0;
        lookKeyHeld_ = false;
    }
}

// ---- controllers -----------------------------------------------------------------------------

void InputSystem::openControllers() {
    if (SDL_InitSubSystem(SDL_INIT_GAMECONTROLLER) != 0) {
        LOG_WARN("input: SDL game controller init failed: %s", SDL_GetError());
        return;
    }
    for (int i = 0; i < SDL_NumJoysticks(); ++i) addController(i);
}

void InputSystem::addController(int deviceIndex) {
    if (controllerCount_ >= kMaxControllers || !SDL_IsGameController(deviceIndex)) return;
    SDL_GameController* pad = SDL_GameControllerOpen(deviceIndex);
    if (!pad) {
        LOG_WARN("input: cannot open controller %d: %s", deviceIndex, SDL_GetError());
        return;
    }
    const int id = SDL_JoystickInstanceID(SDL_GameControllerGetJoystick(pad));
    for (int i = 0; i < controllerCount_; ++i) {
        if (controllerIds_[i] == id) return;  // already open (SDL returns the same handle)
    }
    controllers_[controllerCount_] = pad;
    controllerIds_[controllerCount_] = id;
    ++controllerCount_;
    LOG_INFO("input: controller connected: %s", SDL_GameControllerName(pad));
}

void InputSystem::removeController(int instanceId) {
    bool found = false;
    for (int i = 0; i < controllerCount_; ++i) {
        if (controllerIds_[i] != instanceId) continue;
        SDL_GameControllerClose(controllers_[i]);
        for (int j = i; j + 1 < controllerCount_; ++j) {
            controllers_[j] = controllers_[j + 1];
            controllerIds_[j] = controllerIds_[j + 1];
        }
        --controllerCount_;
        found = true;
        break;
    }
    // Everything the unplugged pad held is released, so nothing stays stuck down.
    if (found || controllerCount_ == 0) {
        LOG_INFO("input: controller removed");
        releaseAllPad();
    }
}

void InputSystem::releaseAllPad() {
    for (int i = 0; i < kPadInputCount; ++i) {
        if (padDown_[i]) padRelease(static_cast<PadInput>(i));
    }
    std::memset(stickRaw_, 0, sizeof stickRaw_);
}

// ---- action reference counting ---------------------------------------------------------------

void InputSystem::holdAction(Action a) {
    const int i = idx(a);
    if (refs_[i]++ == 0) {
        repeatTimerS_[i] = cfg_.repeatDelayS;
        repeatCount_[i] = 0;
    }
    pending_[i] = true;
    if (a == cfg_.doubleTapSource) {
        if (clockS_ - lastTapS_ <= cfg_.doubleTapWindowS) {
            lastTapS_ = -1e9;
            pulseAction(cfg_.doubleTapFires);
        } else {
            lastTapS_ = clockS_;
        }
    }
}

void InputSystem::releaseAction(Action a) {
    int& r = refs_[idx(a)];
    if (r > 0) --r;
}

void InputSystem::pulseAction(Action a) {
    pending_[idx(a)] = true;
}

// ---- gamepad ---------------------------------------------------------------------------------

PadLayer InputSystem::padLayer() const {
    if (padDown_[static_cast<int>(PadInput::L2)]) return PadLayer::L2;
    if (padDown_[static_cast<int>(PadInput::L1)]) return PadLayer::L1;
    if (padDown_[static_cast<int>(PadInput::R1)]) return PadLayer::R1;
    return PadLayer::Base;
}

void InputSystem::padPress(PadInput input) {
    const int in = static_cast<int>(input);
    if (padDown_[in]) return;
    const PadLayer layer = padLayer();  // layer in effect BEFORE this press
    padDown_[in] = true;
    PadHold& hold = padHolds_[in];
    hold.count = 0;

    if (input == PadInput::L2) {
        holdAction(Action::LookHold);
        hold.actions[hold.count++] = Action::LookHold;
    } else if (input != PadInput::L1 && input != PadInput::R1) {
        for (const PadButtonBinding& b : cfg_.padButtons[static_cast<int>(layer)]) {
            if (b.input != input || hold.count >= kMaxHeldActions) continue;
            holdAction(b.action);
            hold.actions[hold.count++] = b.action;
        }
    }
    for (const PadChord& c : cfg_.chords) {
        const bool ab = c.a == input && padDown_[static_cast<int>(c.b)];
        const bool ba = c.b == input && padDown_[static_cast<int>(c.a)];
        if (ab || ba) pulseAction(c.action);
    }
}

void InputSystem::padRelease(PadInput input) {
    const int in = static_cast<int>(input);
    if (!padDown_[in]) return;
    padDown_[in] = false;
    PadHold& hold = padHolds_[in];
    for (int i = 0; i < hold.count; ++i) releaseAction(hold.actions[i]);
    hold.count = 0;
}

void InputSystem::padAxisMotion(int sdlAxis, int raw) {
    const float v = input::normalizeSdlAxis(raw);
    switch (sdlAxis) {
    case SDL_CONTROLLER_AXIS_LEFTX: stickRaw_[0] = v; break;
    case SDL_CONTROLLER_AXIS_LEFTY: stickRaw_[1] = v; break;
    case SDL_CONTROLLER_AXIS_RIGHTX: stickRaw_[2] = v; break;
    case SDL_CONTROLLER_AXIS_RIGHTY: stickRaw_[3] = v; break;
    case SDL_CONTROLLER_AXIS_TRIGGERLEFT:
    case SDL_CONTROLLER_AXIS_TRIGGERRIGHT: {
        // Triggers are analog axes in SDL; treat them as buttons with hysteresis.
        const PadInput in =
            sdlAxis == SDL_CONTROLLER_AXIS_TRIGGERLEFT ? PadInput::L2 : PadInput::R2;
        if (v >= cfg_.triggerOnNorm) {
            padPress(in);
        } else if (v <= cfg_.triggerOffNorm) {
            padRelease(in);
        }
        break;
    }
    default: break;
    }
}

// ---- keyboard --------------------------------------------------------------------------------

void InputSystem::keyPress(const SDL_KeyboardEvent& key) {
    if (!keyboardEnabled_ || key.repeat) return;
    const int code = static_cast<int>(key.keysym.sym);
    const unsigned mods = currentMods(key.keysym.mod);
    keyRelease(key);  // a missed release must not leak a stuck action
    if (keyHoldCount_ >= kMaxKeyHolds) return;
    KeyHold hold;
    hold.keycode = code;

    if (cfg_.keyLookModifier != 0 && code == cfg_.keyLookModifier && mods == 0) {
        lookKeyHeld_ = true;
        holdAction(Action::LookHold);
        hold.actions[hold.actionCount++] = Action::LookHold;
        keyHolds_[keyHoldCount_++] = hold;
        return;
    }
    const int layer = lookKeyHeld_ ? 1 : 0;
    for (const KeyBinding& b : cfg_.keys[layer]) {
        if (b.keycode != code || b.mods != mods || hold.actionCount >= kMaxHeldActions) continue;
        holdAction(b.action);
        hold.actions[hold.actionCount++] = b.action;
    }
    for (const KeyAxisBinding& b : cfg_.keyAxes[layer]) {
        if (b.keycode != code || b.mods != mods || hold.axisCount >= kMaxHeldAxes) continue;
        hold.axes[hold.axisCount] = b.axis;
        hold.signs[hold.axisCount] = b.sign;
        ++hold.axisCount;
    }
    if (hold.actionCount > 0 || hold.axisCount > 0) keyHolds_[keyHoldCount_++] = hold;
}

void InputSystem::keyRelease(const SDL_KeyboardEvent& key) {
    const int code = static_cast<int>(key.keysym.sym);
    for (int i = 0; i < keyHoldCount_; ++i) {
        if (keyHolds_[i].keycode != code) continue;
        for (int a = 0; a < keyHolds_[i].actionCount; ++a) releaseAction(keyHolds_[i].actions[a]);
        if (cfg_.keyLookModifier != 0 && code == cfg_.keyLookModifier) lookKeyHeld_ = false;
        keyHolds_[i] = keyHolds_[--keyHoldCount_];
        return;
    }
}

// ---- events and per-frame update -------------------------------------------------------------

void InputSystem::handleEvent(const SDL_Event& e) {
    switch (e.type) {
    case SDL_QUIT: quit_ = true; break;
    case SDL_KEYDOWN: keyPress(e.key); break;
    case SDL_KEYUP:
        if (keyboardEnabled_) keyRelease(e.key);
        break;
    case SDL_CONTROLLERBUTTONDOWN:
    case SDL_CONTROLLERBUTTONUP: {
        if (e.type == SDL_CONTROLLERBUTTONDOWN) {
            probe_.valid = true;
            probe_.axis = false;
            probe_.sdlCode = e.cbutton.button;
            probe_.axisValue = 0;
            ++probe_.serial;
        }
        const PadInput in = cfg_.padInputForSdlButton(e.cbutton.button);
        if (in == PadInput::Count) break;
        if (e.type == SDL_CONTROLLERBUTTONDOWN) {
            padPress(in);
        } else {
            padRelease(in);
        }
        break;
    }
    case SDL_CONTROLLERAXISMOTION:
        if (std::fabs(input::normalizeSdlAxis(e.caxis.value)) >= 0.30f) {
            probe_.valid = true;
            probe_.axis = true;
            probe_.sdlCode = e.caxis.axis;
            probe_.axisValue = input::normalizeSdlAxis(e.caxis.value);
            ++probe_.serial;
        }
        padAxisMotion(e.caxis.axis, e.caxis.value);
        break;
    case SDL_CONTROLLERDEVICEADDED: addController(e.cdevice.which); break;
    case SDL_CONTROLLERDEVICEREMOVED: removeController(e.cdevice.which); break;
    default: break;
    }
}

const ActionState& InputSystem::update(double dtS) {
    const float dt = static_cast<float>(dtS);
    clockS_ += dtS;

    // ---- buttons: held, edges and auto-repeat ----
    for (int i = 0; i < kActionCount; ++i) {
        bool pressed = pending_[i];
        const bool held = refs_[i] > 0;
        if (held && cfg_.repeats[i] && !pressed) {
            repeatTimerS_[i] -= dt;
            if (repeatTimerS_[i] <= 0.0f) {
                pressed = true;
                repeatTimerS_[i] +=
                    input::repeatIntervalS(repeatCount_[i], cfg_.repeatIntervalS,
                                           cfg_.repeatMinIntervalS, cfg_.repeatDecay);
                ++repeatCount_[i];
            }
        }
        state_.pressed[i] = pressed;
        state_.down[i] = held || pressed;
        state_.repeatCount[i] = held ? repeatCount_[i] : 0;
        pending_[i] = false;
    }
    if (state_.pressed[idx(Action::Quit)]) quit_ = true;

    // ---- axes ----
    const PadLayer layer = padLayer();
    float lx = stickRaw_[0], ly = stickRaw_[1];
    input::applyRadialStick(lx, ly, cfg_.leftDeadzone, cfg_.leftExpo);
    float rx =
        input::applyExpo(input::applyDeadzone(stickRaw_[2], cfg_.rightDeadzone), cfg_.rightExpo);
    float ry =
        input::applyExpo(input::applyDeadzone(stickRaw_[3], cfg_.rightDeadzone), cfg_.rightExpo);
    if (layer == PadLayer::Base) input::applyDominance(rx, ry, cfg_.dominanceRatio);
    const float sticks[4] = {lx, ly, rx, ry};

    float padAxis[kAxisCount] = {};
    for (const PadAxisBinding& b : cfg_.padAxes[static_cast<int>(layer)]) {
        const float v = sticks[static_cast<int>(b.source)];
        padAxis[idx(b.axis)] += b.invert ? -v : v;
    }

    float keyTarget[kAxisCount] = {};
    for (int i = 0; i < keyHoldCount_; ++i)
        for (int a = 0; a < keyHolds_[i].axisCount; ++a)
            keyTarget[idx(keyHolds_[i].axes[a])] += keyHolds_[i].signs[a];
    for (int a = 0; a < kAxisCount; ++a) {
        const float target = std::clamp(keyTarget[a], -1.0f, 1.0f);
        if (isRateAxis(a)) {
            keyAxis_[a] = target;
        } else {
            keyAxis_[a] = approach(keyAxis_[a], target,
                                   target == 0.0f ? cfg_.keyReturnPerS : cfg_.keyRampPerS, dt);
        }
        state_.axis[a] = std::clamp(padAxis[a] + keyAxis_[a], -1.0f, 1.0f);
    }
    return state_;
}

}  // namespace fsim
