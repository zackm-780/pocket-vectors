#include "input/ControlsConfig.h"

#include <SDL.h>

#include <map>

#include "core/Log.h"

namespace fsim {
namespace {

const char* const kActionNames[] = {
    "TrimNoseUp",     "TrimNoseDown", "FlapsUp",         "FlapsDown",       "GearToggle",
    "Brakes",         "ParkingBrake", "SpeedbrakeCycle", "ReverseToggle",   "HudDeclutter",
    "MapToggle",      "Pause",        "LookHold",        "LookLeft",        "LookRight",
    "LookBack",       "LookDown",     "ApToggle",        "AtToggle",        "FdToggle",
    "HdgMode",        "NavMode",      "AprMode",         "AltMode",         "VsMode",
    "HdgBugDec",      "HdgBugInc",    "AltBugDec",       "AltBugInc",       "SyncBugs",
    "NavSourceCycle", "DirectTo",     "RudderTrimLeft",  "RudderTrimRight", "MenuUp",
    "MenuDown",       "MenuLeft",     "MenuRight",       "MenuAccept",      "MenuBack",
    "Screenshot",     "Quit",         "BaroInc",         "BaroDec",         "BaroStd"};
static_assert(sizeof(kActionNames) / sizeof(kActionNames[0]) == kActionCount,
              "kActionNames must list every Action in enum order");

const char* const kAxisNames[] = {"Pitch",   "Roll",      "Rudder", "ThrottleRate",
                                  "LookYaw", "LookPitch", "BugRate"};
static_assert(sizeof(kAxisNames) / sizeof(kAxisNames[0]) == kAxisCount,
              "kAxisNames must list every Axis in enum order");

const char* const kPadNames[] = {"a",         "b",          "x",  "y",  "dpad_up", "dpad_down",
                                 "dpad_left", "dpad_right", "l1", "r1", "l2",      "r2",
                                 "select",    "start",      "l3", "r3"};
static_assert(sizeof(kPadNames) / sizeof(kPadNames[0]) == kPadInputCount,
              "kPadNames must list every PadInput in enum order");

const char* const kStickNames[] = {"leftx", "lefty", "rightx", "righty"};

template <size_t N>
bool lookup(const char* const (&names)[N], std::string_view name, size_t& index) {
    const std::string lower = toLower(name);
    for (size_t i = 0; i < N; ++i) {
        if (lower == toLower(names[i])) {
            index = i;
            return true;
        }
    }
    return false;
}

using Section = std::map<std::string, std::string>;  // lower-case key -> value
using Raw = std::map<std::string, Section>;          // lower-case section -> keys

const char* const kSections[] = {"device_labels", "axes",     "repeat", "double_tap",
                                 "pad.base",      "pad.l1",   "pad.r1", "pad.l2",
                                 "pad.chords",    "key.base", "key.l2", "key.layers"};

void mergeInto(Raw& raw, const Ini& ini) {
    for (const char* sec : kSections) {
        for (const Ini::Entry& e : ini.entries(sec)) raw[sec][e.first] = e.second;
    }
}

// Parses "shift+ctrl+b" into modifier bits and an SDL keycode. Returns false if unknown.
bool parseKeySpec(std::string_view spec, int& keycode, unsigned& mods) {
    mods = 0;
    std::string_view rest = trim(spec);
    while (true) {
        const size_t plus = rest.find('+');
        if (plus == std::string_view::npos || plus == 0 || plus + 1 >= rest.size()) break;
        const std::string mod = toLower(trim(rest.substr(0, plus)));
        if (mod == "shift") {
            mods |= kModShift;
        } else if (mod == "ctrl") {
            mods |= kModCtrl;
        } else if (mod == "alt") {
            mods |= kModAlt;
        } else {
            return false;
        }
        rest = trim(rest.substr(plus + 1));
    }
    const std::string name(rest);
    keycode = static_cast<int>(SDL_GetKeyFromName(name.c_str()));
    return keycode != SDLK_UNKNOWN && !name.empty();
}

}  // namespace

const char* actionName(Action a) {
    const int i = static_cast<int>(a);
    return i >= 0 && i < kActionCount ? kActionNames[i] : "?";
}
const char* axisName(Axis a) {
    const int i = static_cast<int>(a);
    return i >= 0 && i < kAxisCount ? kAxisNames[i] : "?";
}
bool actionFromName(std::string_view name, Action& out) {
    size_t i = 0;
    if (!lookup(kActionNames, name, i)) return false;
    out = static_cast<Action>(i);
    return true;
}
bool axisFromName(std::string_view name, Axis& out) {
    size_t i = 0;
    if (!lookup(kAxisNames, name, i)) return false;
    out = static_cast<Axis>(i);
    return true;
}
bool padInputFromName(std::string_view name, PadInput& out) {
    size_t i = 0;
    if (!lookup(kPadNames, name, i)) return false;
    out = static_cast<PadInput>(i);
    return true;
}

ControlsConfig::ControlsConfig() {
    for (int i = 0; i < kPadInputCount; ++i) {
        sdlButtonForLabel[i] = -1;
        sdlButtonCountForLabel[i] = 0;
        for (int j = 0; j < kMaxSdlButtonsPerLabel; ++j) sdlButtonsForLabel[i][j] = -1;
    }
}

PadInput ControlsConfig::padInputForSdlButton(int sdlButton) const {
    if (sdlButton < 0) return PadInput::Count;
    for (int i = 0; i < kPadInputCount; ++i) {
        for (int j = 0; j < sdlButtonCountForLabel[i]; ++j) {
            if (sdlButtonsForLabel[i][j] == sdlButton) return static_cast<PadInput>(i);
        }
    }
    return PadInput::Count;
}

bool ControlsConfig::loadFiles(const std::string& defaultsPath, const std::string& userPath) {
    Ini defaults, user;
    if (!defaults.loadFile(defaultsPath)) {
        LOG_ERROR("controls: cannot read %s", defaultsPath.c_str());
        return false;
    }
    const bool haveUser = !userPath.empty() && user.loadFile(userPath);
    if (haveUser) LOG_INFO("controls: user overrides from %s", userPath.c_str());
    return load(defaults, haveUser ? &user : nullptr);
}

bool ControlsConfig::load(const Ini& defaults, const Ini* user) {
    Raw raw;
    mergeInto(raw, defaults);
    if (user) mergeInto(raw, *user);
    if (raw.empty()) return false;

    *this = ControlsConfig();
    auto warn = [this](const char* section, const std::string& key, const char* what) {
        LOG_WARN("controls: [%s] %s: %s", section, key.c_str(), what);
        ++warningCount;
    };

    // ---- device labels ----
    for (const auto& [label, sdlName] : raw["device_labels"]) {
        PadInput in;
        if (!padInputFromName(label, in)) {
            warn("device_labels", label, "unknown device label");
            continue;
        }
        // A label may list several SDL buttons ("back, guide"). L2/R2 are triggers (axes);
        // lefttrigger/righttrigger stay in the file as documentation and are not buttons.
        const int idx = static_cast<int>(in);
        int count = 0;
        for (const std::string& token : splitTrim(sdlName, ',')) {
            if (token.empty()) continue;
            const SDL_GameControllerButton b = SDL_GameControllerGetButtonFromString(token.c_str());
            if (b == SDL_CONTROLLER_BUTTON_INVALID) {
                const std::string lower = toLower(token);
                if (lower != "lefttrigger" && lower != "righttrigger") {
                    warn("device_labels", label, "unknown SDL button");
                }
                continue;
            }
            if (count >= kMaxSdlButtonsPerLabel) {
                warn("device_labels", label, "too many SDL buttons");
                break;
            }
            sdlButtonsForLabel[idx][count++] = static_cast<int>(b);
        }
        sdlButtonCountForLabel[idx] = count;
        sdlButtonForLabel[idx] = count > 0 ? sdlButtonsForLabel[idx][0] : -1;
    }

    // ---- scalar settings ----
    auto getF = [&raw](const char* sec, const char* key, float& v) {
        const auto s = raw.find(sec);
        if (s == raw.end()) return;
        const auto k = s->second.find(key);
        if (k == s->second.end()) return;
        try {
            v = std::stof(k->second);
        } catch (...) {
            LOG_WARN("controls: [%s] %s: not a number", sec, key);
        }
    };
    getF("axes", "left_deadzone", leftDeadzone);
    getF("axes", "left_expo", leftExpo);
    getF("axes", "right_deadzone", rightDeadzone);
    getF("axes", "right_expo", rightExpo);
    getF("axes", "dominance_ratio", dominanceRatio);
    getF("axes", "trigger_on", triggerOnNorm);
    getF("axes", "trigger_off", triggerOffNorm);
    getF("axes", "key_ramp_per_s", keyRampPerS);
    getF("axes", "key_return_per_s", keyReturnPerS);
    getF("repeat", "delay_s", repeatDelayS);
    getF("repeat", "interval_s", repeatIntervalS);
    getF("repeat", "min_interval_s", repeatMinIntervalS);
    getF("repeat", "decay", repeatDecay);
    getF("double_tap", "window_s", doubleTapWindowS);

    for (const std::string& name : splitTrim(raw["repeat"]["actions"], ',')) {
        if (name.empty()) continue;
        Action a;
        if (actionFromName(name, a)) {
            repeats[static_cast<int>(a)] = true;
        } else {
            warn("repeat", name, "unknown action");
        }
    }
    {
        Action a;
        const std::string& src = raw["double_tap"]["action"];
        const std::string& fires = raw["double_tap"]["fires"];
        if (!src.empty() && !fires.empty()) {
            if (actionFromName(src, a))
                doubleTapSource = a;
            else
                warn("double_tap", src, "unknown action");
            if (actionFromName(fires, a))
                doubleTapFires = a;
            else
                warn("double_tap", fires, "unknown action");
        }
    }

    // ---- gamepad layers ----
    const char* const padSections[kPadLayerCount] = {"pad.base", "pad.l1", "pad.r1", "pad.l2"};
    for (int layer = 0; layer < kPadLayerCount; ++layer) {
        const char* sec = padSections[layer];
        for (const auto& [key, value] : raw[sec]) {
            if (key.rfind("axis.", 0) == 0) {
                Axis axis;
                if (!axisFromName(key.substr(5), axis)) {
                    warn(sec, key, "unknown axis");
                    continue;
                }
                std::string src = std::string(trim(value));
                if (src.empty()) continue;  // unbound
                const bool invert = src[0] == '-';
                if (invert) src.erase(0, 1);
                size_t si = 0;
                if (!lookup(kStickNames, src, si)) {
                    warn(sec, key, "unknown stick (leftx lefty rightx righty)");
                    continue;
                }
                padAxes[layer].push_back({axis, static_cast<Stick>(si), invert});
                continue;
            }
            Action action;
            if (!actionFromName(key, action)) {
                warn(sec, key, "unknown action");
                continue;
            }
            for (const std::string& in : splitTrim(value, ',')) {
                if (in.empty()) continue;
                PadInput input;
                if (!padInputFromName(in, input)) {
                    warn(sec, in, "unknown gamepad input");
                    continue;
                }
                padButtons[layer].push_back({input, action});
            }
        }
    }
    for (const auto& [key, value] : raw["pad.chords"]) {
        Action action;
        if (!actionFromName(key, action)) {
            warn("pad.chords", key, "unknown action");
            continue;
        }
        for (const std::string& chord : splitTrim(value, ',')) {
            if (chord.empty()) continue;
            const std::vector<std::string> parts = splitTrim(chord, '+');
            PadInput a, b;
            if (parts.size() != 2 || !padInputFromName(parts[0], a) ||
                !padInputFromName(parts[1], b)) {
                warn("pad.chords", chord, "chord must be two gamepad inputs joined by +");
                continue;
            }
            chords.push_back({action, a, b});
        }
    }

    // ---- keyboard ----
    {
        const std::string& look = raw["key.layers"]["look"];
        if (!look.empty()) {
            int code;
            unsigned mods;
            if (parseKeySpec(look, code, mods))
                keyLookModifier = code;
            else
                warn("key.layers", look, "unknown key");
        }
    }
    const char* const keySections[2] = {"key.base", "key.l2"};
    for (int layer = 0; layer < 2; ++layer) {
        const char* sec = keySections[layer];
        for (const auto& [key, value] : raw[sec]) {
            if (key.rfind("axis.", 0) == 0) {
                // axis.<Name>.neg / axis.<Name>.pos
                const std::string rest = key.substr(5);
                const size_t dot = rest.rfind('.');
                Axis axis;
                const std::string side = dot == std::string::npos ? "" : rest.substr(dot + 1);
                if (dot == std::string::npos || (side != "neg" && side != "pos") ||
                    !axisFromName(rest.substr(0, dot), axis)) {
                    warn(sec, key, "expected axis.<Name>.neg or axis.<Name>.pos");
                    continue;
                }
                for (const std::string& spec : splitTrim(value, ',')) {
                    if (spec.empty()) continue;
                    int code;
                    unsigned mods;
                    if (!parseKeySpec(spec, code, mods)) {
                        warn(sec, spec, "unknown key");
                        continue;
                    }
                    keyAxes[layer].push_back({code, mods, axis, side == "pos" ? 1.0f : -1.0f});
                }
                continue;
            }
            Action action;
            if (!actionFromName(key, action)) {
                warn(sec, key, "unknown action");
                continue;
            }
            for (const std::string& spec : splitTrim(value, ',')) {
                if (spec.empty()) continue;
                int code;
                unsigned mods;
                if (!parseKeySpec(spec, code, mods)) {
                    warn(sec, spec, "unknown key");
                    continue;
                }
                keys[layer].push_back({code, mods, action});
            }
        }
    }
    return true;
}

}  // namespace fsim
