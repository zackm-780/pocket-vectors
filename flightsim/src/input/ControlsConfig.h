#pragma once
// Compiled control bindings, parsed from assets/controls_default.ini plus an optional user
// override (conf/controls.ini). See the header comment of controls_default.ini for the format.
// No SDL event handling here; the SDL names (buttons, keys) are only used to translate names.

#include <string>
#include <string_view>
#include <vector>

#include "core/Actions.h"
#include "core/Ini.h"

namespace fsim {

constexpr int kActionCount = static_cast<int>(Action::Count);
constexpr int kAxisCount = static_cast<int>(Axis::Count);

// Physical inputs of the handheld, named by the label PRINTED on the device (CONTROLS.md).
enum class PadInput {
    A,
    B,
    X,
    Y,
    DpadUp,
    DpadDown,
    DpadLeft,
    DpadRight,
    L1,
    R1,
    L2,
    R2,
    Select,
    Start,
    L3,
    R3,
    Count
};
constexpr int kPadInputCount = static_cast<int>(PadInput::Count);

// Shift layers. Priority when several modifiers are held: L2 > L1 > R1.
enum class PadLayer { Base, L1, R1, L2, Count };
constexpr int kPadLayerCount = static_cast<int>(PadLayer::Count);

enum class Stick { LeftX, LeftY, RightX, RightY, Count };

enum KeyMod : unsigned { kModShift = 1, kModCtrl = 2, kModAlt = 4 };

struct PadButtonBinding {
    PadInput input;
    Action action;
};
struct PadAxisBinding {
    Axis axis;
    Stick source;
    bool invert;
};
struct PadChord {
    Action action;
    PadInput a, b;
};
struct KeyBinding {
    int keycode;  // SDL_Keycode
    unsigned mods;
    Action action;
};
struct KeyAxisBinding {
    int keycode;
    unsigned mods;
    Axis axis;
    float sign;
};

struct ControlsConfig {
    // Device-label table: SDL_GameControllerButton for each printed label (-1 = no button).
    int sdlButtonForLabel[kPadInputCount];

    std::vector<PadButtonBinding> padButtons[kPadLayerCount];
    std::vector<PadAxisBinding> padAxes[kPadLayerCount];
    std::vector<PadChord> chords;
    // Keyboard: index 0 = base, 1 = look layer (while keyLookModifier is held).
    std::vector<KeyBinding> keys[2];
    std::vector<KeyAxisBinding> keyAxes[2];
    int keyLookModifier = 0;  // SDL_Keycode, 0 = none

    float leftDeadzone = 0.08f, leftExpo = 0.35f;
    float rightDeadzone = 0.12f, rightExpo = 0.0f;
    float dominanceRatio = 1.5f;
    float triggerOnNorm = 0.5f, triggerOffNorm = 0.3f;
    float keyRampPerS = 2.0f, keyReturnPerS = 4.0f;

    bool repeats[kActionCount] = {};
    float repeatDelayS = 0.4f, repeatIntervalS = 0.15f, repeatMinIntervalS = 0.05f;
    float repeatDecay = 0.85f;

    Action doubleTapSource = Action::Brakes;
    Action doubleTapFires = Action::ParkingBrake;
    float doubleTapWindowS = 0.3f;

    int warningCount = 0;  // unknown names etc. while parsing (each also logged)

    ControlsConfig();

    // Parses `defaults`, then applies `user` (may be null) on top: a key present in the user file
    // REPLACES that action's bindings of that section. Returns false only if `defaults` is empty.
    bool load(const Ini& defaults, const Ini* user);
    // Convenience: file paths (the user file is optional).
    bool loadFiles(const std::string& defaultsPath, const std::string& userPath);

    PadInput padInputForSdlButton(int sdlButton) const;
};

// Name tables (case-insensitive lookups). Return false when unknown.
const char* actionName(Action a);
const char* axisName(Axis a);
bool actionFromName(std::string_view name, Action& out);
bool axisFromName(std::string_view name, Axis& out);
bool padInputFromName(std::string_view name, PadInput& out);

}  // namespace fsim
