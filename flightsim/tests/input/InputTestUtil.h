#pragma once
// Helpers to inject synthetic SDL events into an InputSystem (no device needed).

#include <SDL.h>
#include <doctest/doctest.h>

#include <cstdlib>
#include <string>

#include "input/ControlsConfig.h"
#include "input/InputSystem.h"

namespace fsim::testutil {

constexpr double kFrameS = 1.0 / 60.0;

inline ControlsConfig defaultConfig() {
    ControlsConfig cfg;
    REQUIRE(cfg.loadFiles("assets/controls_default.ini", ""));
    return cfg;
}

inline ControlsConfig configFromText(const std::string& defaults, const std::string& user = "") {
    Ini d, u;
    d.loadText(defaults);
    u.loadText(user);
    ControlsConfig cfg;
    cfg.load(d, user.empty() ? nullptr : &u);
    return cfg;
}

// Pad button by the label printed on the device (goes through the label table).
inline void padButton(InputSystem& in, PadInput label, bool down) {
    SDL_Event e{};
    e.type = down ? SDL_CONTROLLERBUTTONDOWN : SDL_CONTROLLERBUTTONUP;
    e.cbutton.button = static_cast<Uint8>(in.config().sdlButtonForLabel[static_cast<int>(label)]);
    e.cbutton.state = down ? SDL_PRESSED : SDL_RELEASED;
    in.handleEvent(e);
}

inline void padAxis(InputSystem& in, SDL_GameControllerAxis axis, float value) {
    SDL_Event e{};
    e.type = SDL_CONTROLLERAXISMOTION;
    e.caxis.axis = static_cast<Uint8>(axis);
    e.caxis.value = static_cast<Sint16>(value * 32767.0f);
    in.handleEvent(e);
}

inline void key(InputSystem& in, SDL_Keycode sym, bool down, Uint16 mod = 0, bool repeat = false) {
    SDL_Event e{};
    e.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    e.key.keysym.sym = sym;
    e.key.keysym.mod = mod;
    e.key.state = down ? SDL_PRESSED : SDL_RELEASED;
    e.key.repeat = repeat ? 1 : 0;
    in.handleEvent(e);
}

inline int idx(Action a) {
    return static_cast<int>(a);
}
inline int idx(Axis a) {
    return static_cast<int>(a);
}

}  // namespace fsim::testutil
