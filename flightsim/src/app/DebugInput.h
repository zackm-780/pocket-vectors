#pragma once
// TEMPORARY keyboard debug controls (WP-00). P2 replaces this with the real input system
// (gamepad, bindings, deadzones). Kept in this one small file pair on purpose.
//
//   arrows        pitch / roll (Down = pull back = nose up, Up = push = nose down)
//   Q / E         rudder left / right
//   PgUp / PgDn   throttle up / down (also + / -)
//   Home / End    elevator trim nose up / nose down
//   [ / ]         flaps up / down one detent
//   G             gear toggle        B  wheel brakes (hold)    Space  parking brake toggle
//   P             pause              Esc  quit

#include "core/ControlInputs.h"

union SDL_Event;

namespace fsim {

class DebugInput {
public:
    // Feeds an SDL event (key edges: gear, flaps, pause, quit).
    void handleEvent(const SDL_Event& event);
    // Applies held keys to the persistent pilot controls. dtS is the wall frame time.
    void update(double dtS, int engineCount, int flapDetentCount, ControlInputs& pilot);

    bool quitRequested() const { return quit_; }
    bool consumePauseToggle();

private:
    bool quit_ = false;
    bool pauseToggle_ = false;
    bool gearToggle_ = false;
    bool parkingToggle_ = false;
    bool brakeKeyPrev_ = false;
    int flapStep_ = 0;
};

}  // namespace fsim
