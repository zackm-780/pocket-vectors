// Controller enumeration and the interactive input test.
//
// The test shows one prompt at a time ("press L2", "move the right stick", ...) and logs every SDL
// button/axis/hat/key event with its name and raw value. At the end it derives answers to the
// DEVICE.md questions: are L2/R2 analog, do L3/R3 exist, are A/B swapped, is the Menu key visible.
#pragma once

#include "Context.h"

namespace probe {

// Lists and opens every joystick/game controller: name, GUID, mapping, bindings. Keeps them open for
// the input test. Safe to call with no controllers attached.
void reportControllers(Ctx& ctx);

// Runs the interactive test (about 35 s with a responsive human, 60 s worst case). In auto mode the
// waits are shortened; with --selftest-input synthetic events are injected so the logger and the
// analysis can be verified without hardware.
void runInputTest(Ctx& ctx);

void closeControllers();

}  // namespace probe
