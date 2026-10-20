#pragma once
// Menu input (WP H5, CONTROLS section 3): the semantic actions P2 produces, reduced to the nine
// buttons a menu cares about. Pure data, so every menu state machine is testable without SDL.
//
//   D-pad (MenuUp/Down/Left/Right, with P2's auto-repeat), A (MenuAccept), B (MenuBack),
//   X and Y (context actions shown in the footer; the same physical buttons that are speedbrake and
//   HUD declutter in flight) and Start (Pause).

#include "core/Actions.h"

namespace fsim {

struct MenuInput {
    bool up = false, down = false, left = false, right = false;
    bool accept = false, back = false;
    bool x = false, y = false, start = false;

    bool any() const { return up || down || left || right || accept || back || x || y || start; }
};

// Edges of this frame (auto-repeats included, they arrive as pressed edges).
inline MenuInput menuInputFromActions(const ActionState& a) {
    auto p = [&](Action act) { return a.pressed[static_cast<int>(act)]; };
    MenuInput m;
    m.up = p(Action::MenuUp);
    m.down = p(Action::MenuDown);
    m.left = p(Action::MenuLeft);
    m.right = p(Action::MenuRight);
    m.accept = p(Action::MenuAccept);
    m.back = p(Action::MenuBack);
    m.x = p(Action::SpeedbrakeCycle);
    m.y = p(Action::HudDeclutter);
    m.start = p(Action::Pause);
    return m;
}

}  // namespace fsim
