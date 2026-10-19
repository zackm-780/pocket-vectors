#include "InputTest.h"

#include <algorithm>
#include <cctype>
#include <climits>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <map>
#include <set>
#include <vector>

#include "Common.h"

namespace probe {

namespace {

// ---------------------------------------------------------------------------------------------
// Open devices
// ---------------------------------------------------------------------------------------------

struct OpenDevices {
    std::vector<SDL_Joystick*> joysticks;
    std::vector<SDL_GameController*> controllers;
};

OpenDevices& devices() {
    static OpenDevices d;
    return d;
}

std::string guidString(SDL_JoystickGUID guid) {
    char buf[64] = {};
    SDL_JoystickGetGUIDString(guid, buf, sizeof(buf));
    return buf;
}

std::string bindingString(const SDL_GameControllerButtonBind& b) {
    switch (b.bindType) {
        case SDL_CONTROLLER_BINDTYPE_BUTTON:
            return strf("joystick button %d", b.value.button);
        case SDL_CONTROLLER_BINDTYPE_AXIS:
            return strf("joystick axis %d", b.value.axis);
        case SDL_CONTROLLER_BINDTYPE_HAT:
            return strf("joystick hat %d mask 0x%x", b.value.hat.hat, b.value.hat.hat_mask);
        default:
            return "none";
    }
}

const char* powerLevelName(SDL_JoystickPowerLevel level) {
    switch (level) {
        case SDL_JOYSTICK_POWER_EMPTY: return "empty";
        case SDL_JOYSTICK_POWER_LOW: return "low";
        case SDL_JOYSTICK_POWER_MEDIUM: return "medium";
        case SDL_JOYSTICK_POWER_FULL: return "full";
        case SDL_JOYSTICK_POWER_WIRED: return "wired";
        case SDL_JOYSTICK_POWER_MAX: return "max";
        default: return "unknown";
    }
}

void openJoystickIndex(Report& report, int index) {
    if (SDL_Joystick* joy = SDL_JoystickOpen(index)) {
        devices().joysticks.push_back(joy);
    } else {
        report.line(strf("  SDL_JoystickOpen(%d) failed: %s", index, SDL_GetError()));
    }
    if (SDL_IsGameController(index) != SDL_FALSE) {
        if (SDL_GameController* gc = SDL_GameControllerOpen(index)) {
            devices().controllers.push_back(gc);
        } else {
            report.line(strf("  SDL_GameControllerOpen(%d) failed: %s", index, SDL_GetError()));
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Event description
// ---------------------------------------------------------------------------------------------

struct EventInfo {
    std::string key;    // aggregation key, e.g. "CTRL button a", "CTRL axis lefttrigger", "JOY axis 2"
    std::string text;   // full log line
    int value = 0;      // axis value / hat mask / 1 = down, 0 = up
    bool isAxis = false;
    bool isPress = false;  // button/key down or hat away from centre
};

std::string hatName(int mask) {
    if (mask == SDL_HAT_CENTERED) {
        return "CENTERED";
    }
    std::string s;
    if ((mask & SDL_HAT_UP) != 0) s += "UP";
    if ((mask & SDL_HAT_DOWN) != 0) s += "DOWN";
    if ((mask & SDL_HAT_LEFT) != 0) s += "LEFT";
    if ((mask & SDL_HAT_RIGHT) != 0) s += "RIGHT";
    return s;
}

bool describeEvent(const SDL_Event& e, EventInfo& out) {
    switch (e.type) {
        case SDL_CONTROLLERBUTTONDOWN:
        case SDL_CONTROLLERBUTTONUP: {
            const char* name = SDL_GameControllerGetStringForButton(static_cast<SDL_GameControllerButton>(e.cbutton.button));
            const bool down = e.type == SDL_CONTROLLERBUTTONDOWN;
            out.key = strf("CTRL button %s", name != nullptr ? name : "?");
            out.value = down ? 1 : 0;
            out.isPress = down;
            out.text = strf("CTRL button %-14s (id %2d) %s", name != nullptr ? name : "?", e.cbutton.button, down ? "DOWN" : "up");
            return true;
        }
        case SDL_CONTROLLERAXISMOTION: {
            const char* name = SDL_GameControllerGetStringForAxis(static_cast<SDL_GameControllerAxis>(e.caxis.axis));
            out.key = strf("CTRL axis %s", name != nullptr ? name : "?");
            out.value = e.caxis.value;
            out.isAxis = true;
            out.text = strf("CTRL axis   %-14s (id %2d) = %6d", name != nullptr ? name : "?", e.caxis.axis, e.caxis.value);
            return true;
        }
        case SDL_JOYBUTTONDOWN:
        case SDL_JOYBUTTONUP: {
            const bool down = e.type == SDL_JOYBUTTONDOWN;
            out.key = strf("JOY button %d", e.jbutton.button);
            out.value = down ? 1 : 0;
            out.isPress = down;
            out.text = strf("JOY  button %-2d                     %s", e.jbutton.button, down ? "DOWN" : "up");
            return true;
        }
        case SDL_JOYAXISMOTION:
            out.key = strf("JOY axis %d", e.jaxis.axis);
            out.value = e.jaxis.value;
            out.isAxis = true;
            out.text = strf("JOY  axis   %-2d                    = %6d", e.jaxis.axis, e.jaxis.value);
            return true;
        case SDL_JOYHATMOTION:
            out.key = strf("JOY hat %d", e.jhat.hat);
            out.value = e.jhat.value;
            out.isPress = e.jhat.value != SDL_HAT_CENTERED;
            out.text = strf("JOY  hat    %-2d                    = %s (0x%02x)", e.jhat.hat, hatName(e.jhat.value).c_str(),
                            e.jhat.value);
            return true;
        case SDL_KEYDOWN:
        case SDL_KEYUP: {
            if (e.key.repeat != 0) {
                return false;
            }
            const bool down = e.type == SDL_KEYDOWN;
            out.key = strf("KEY %s", SDL_GetScancodeName(e.key.keysym.scancode));
            out.value = down ? 1 : 0;
            out.isPress = down;
            out.text = strf("KEY  %-14s scancode=%3d keycode=0x%08x %s", SDL_GetScancodeName(e.key.keysym.scancode),
                            static_cast<int>(e.key.keysym.scancode), static_cast<unsigned>(e.key.keysym.sym),
                            down ? "DOWN" : "up");
            return true;
        }
        default:
            return false;
    }
}

// ---------------------------------------------------------------------------------------------
// Prompts
// ---------------------------------------------------------------------------------------------

struct Prompt {
    const char* id;
    const char* text;
    double timeoutS;  // give up after this long without enough input
    double lingerS;   // keep listening this long after the first significant event
};

const Prompt kPrompts[] = {
    {"A", "Press the button labeled A", 2.5, 0.6},
    {"B", "Press the button labeled B", 2.5, 0.6},
    {"X", "Press the button labeled X", 2.5, 0.6},
    {"Y", "Press the button labeled Y", 2.5, 0.6},
    {"L1", "Press L1 (left shoulder)", 2.5, 0.6},
    {"R1", "Press R1 (right shoulder)", 2.5, 0.6},
    {"L2", "Squeeze L2 slowly, then let go", 3.0, 1.6},
    {"R2", "Squeeze R2 slowly, then let go", 3.0, 1.6},
    {"L3", "Click the LEFT stick straight down (L3)", 2.5, 0.6},
    {"R3", "Click the RIGHT stick straight down (R3)", 2.5, 0.6},
    {"DPAD", "D-PAD: press up, down, left, right", 5.0, 2.5},
    {"SELECT", "Press SELECT (not together with START)", 2.5, 0.6},
    {"START", "Press START (not together with SELECT)", 2.5, 0.6},
    {"MENU", "Press the MENU / function key", 2.5, 0.6},
    {"VOLUP", "Press VOLUME UP", 2.5, 0.6},
    {"VOLDOWN", "Press VOLUME DOWN", 2.5, 0.6},
    {"LSTICK", "Move the LEFT stick in a full circle", 4.0, 2.5},
    {"RSTICK", "Move the RIGHT stick in a full circle", 4.0, 2.5},
    {"FREE", "Free test: mash anything (try L2+R2, both sticks)", 5.0, 5.0},
};
constexpr int kNumPrompts = static_cast<int>(sizeof(kPrompts) / sizeof(kPrompts[0]));
constexpr double kCalibrateS = 1.5;
constexpr int kSignificantAxisDelta = 6000;
constexpr size_t kMaxLoggedEvents = 8000;

struct KeyStat {
    int count = 0;
    int minV = INT_MAX;
    int maxV = INT_MIN;
    bool isAxis = false;
    bool sawPress = false;
    std::set<int> distinct;
};
using StatMap = std::map<std::string, KeyStat>;

struct PromptResult {
    const Prompt* prompt = nullptr;
    StatMap stats;
    bool significant = false;
};

void addStat(StatMap& map, const EventInfo& info) {
    KeyStat& s = map[info.key];
    ++s.count;
    s.minV = std::min(s.minV, info.value);
    s.maxV = std::max(s.maxV, info.value);
    s.isAxis = info.isAxis;
    s.sawPress = s.sawPress || info.isPress;
    if (s.distinct.size() < 64) {
        s.distinct.insert(info.value);
    }
}

// ---------------------------------------------------------------------------------------------
// Synthetic events for --selftest-input
// ---------------------------------------------------------------------------------------------

void pushCtrlButton(SDL_GameControllerButton b, bool down) {
    SDL_Event e;
    SDL_zero(e);
    e.type = down ? SDL_CONTROLLERBUTTONDOWN : SDL_CONTROLLERBUTTONUP;
    e.cbutton.button = static_cast<Uint8>(b);
    e.cbutton.state = down ? SDL_PRESSED : SDL_RELEASED;
    SDL_PushEvent(&e);
}

void pushCtrlAxis(SDL_GameControllerAxis a, int v) {
    SDL_Event e;
    SDL_zero(e);
    e.type = SDL_CONTROLLERAXISMOTION;
    e.caxis.axis = static_cast<Uint8>(a);
    e.caxis.value = static_cast<Sint16>(v);
    SDL_PushEvent(&e);
}

void pushJoyButton(int b, bool down) {
    SDL_Event e;
    SDL_zero(e);
    e.type = down ? SDL_JOYBUTTONDOWN : SDL_JOYBUTTONUP;
    e.jbutton.button = static_cast<Uint8>(b);
    e.jbutton.state = down ? SDL_PRESSED : SDL_RELEASED;
    SDL_PushEvent(&e);
}

void pushJoyAxis(int axis, int v) {
    SDL_Event e;
    SDL_zero(e);
    e.type = SDL_JOYAXISMOTION;
    e.jaxis.axis = static_cast<Uint8>(axis);
    e.jaxis.value = static_cast<Sint16>(v);
    SDL_PushEvent(&e);
}

void pushKey(SDL_Scancode sc, bool down) {
    SDL_Event e;
    SDL_zero(e);
    e.type = down ? SDL_KEYDOWN : SDL_KEYUP;
    e.key.keysym.scancode = sc;
    e.key.keysym.sym = SDL_GetKeyFromScancode(sc);
    e.key.state = down ? SDL_PRESSED : SDL_RELEASED;
    SDL_PushEvent(&e);
}

void pushPair(SDL_GameControllerButton b, int joyButton) {
    pushCtrlButton(b, true);
    pushJoyButton(joyButton, true);
    pushCtrlButton(b, false);
    pushJoyButton(joyButton, false);
}

// Pretends to be a pad whose A/B and X/Y are swapped relative to the printed labels, with an analog L2,
// a digital R2, an L3 but no R3 and no Menu key.
void injectSynthetic(const std::string& id) {
    if (id == "A") pushPair(SDL_CONTROLLER_BUTTON_B, 1);
    else if (id == "B") pushPair(SDL_CONTROLLER_BUTTON_A, 0);
    else if (id == "X") pushPair(SDL_CONTROLLER_BUTTON_Y, 3);
    else if (id == "Y") pushPair(SDL_CONTROLLER_BUTTON_X, 2);
    else if (id == "L1") pushPair(SDL_CONTROLLER_BUTTON_LEFTSHOULDER, 4);
    else if (id == "R1") pushPair(SDL_CONTROLLER_BUTTON_RIGHTSHOULDER, 5);
    else if (id == "L2") {
        for (int v : {0, 3000, 9000, 16000, 24000, 30000, 32767, 20000, 8000, 0}) {
            pushCtrlAxis(SDL_CONTROLLER_AXIS_TRIGGERLEFT, v);
            pushJoyAxis(2, v - 32768);
        }
    } else if (id == "R2") {
        for (int v : {0, 32767, 0}) {
            pushCtrlAxis(SDL_CONTROLLER_AXIS_TRIGGERRIGHT, v);
        }
        pushJoyButton(7, true);
        pushJoyButton(7, false);
    } else if (id == "L3") pushPair(SDL_CONTROLLER_BUTTON_LEFTSTICK, 9);
    else if (id == "DPAD") {
        pushPair(SDL_CONTROLLER_BUTTON_DPAD_UP, 12);
        pushPair(SDL_CONTROLLER_BUTTON_DPAD_DOWN, 13);
        pushPair(SDL_CONTROLLER_BUTTON_DPAD_LEFT, 14);
        pushPair(SDL_CONTROLLER_BUTTON_DPAD_RIGHT, 15);
    } else if (id == "SELECT") pushPair(SDL_CONTROLLER_BUTTON_BACK, 6);
    else if (id == "START") pushPair(SDL_CONTROLLER_BUTTON_START, 11);
    else if (id == "VOLUP") {
        pushKey(SDL_SCANCODE_VOLUMEUP, true);
        pushKey(SDL_SCANCODE_VOLUMEUP, false);
    } else if (id == "VOLDOWN") {
        pushKey(SDL_SCANCODE_VOLUMEDOWN, true);
        pushKey(SDL_SCANCODE_VOLUMEDOWN, false);
    } else if (id == "LSTICK" || id == "RSTICK") {
        const bool left = id == "LSTICK";
        for (int i = 0; i < 24; ++i) {
            const double a = 6.2831853 * i / 24.0;
            pushCtrlAxis(left ? SDL_CONTROLLER_AXIS_LEFTX : SDL_CONTROLLER_AXIS_RIGHTX, static_cast<int>(32000 * std::cos(a)));
            pushCtrlAxis(left ? SDL_CONTROLLER_AXIS_LEFTY : SDL_CONTROLLER_AXIS_RIGHTY, static_cast<int>(32000 * std::sin(a)));
        }
        pushCtrlAxis(left ? SDL_CONTROLLER_AXIS_LEFTX : SDL_CONTROLLER_AXIS_RIGHTX, 0);
        pushCtrlAxis(left ? SDL_CONTROLLER_AXIS_LEFTY : SDL_CONTROLLER_AXIS_RIGHTY, 0);
    }
    // R3, MENU and FREE: nothing is injected on purpose (tests the "no event" verdicts).
}

// ---------------------------------------------------------------------------------------------
// The test
// ---------------------------------------------------------------------------------------------

class InputTester {
public:
    explicit InputTester(Ctx& ctx) : ctx_(ctx), report_(*ctx.report) {}

    void run() {
        const double scale = ctx_.opt.autoMode && ctx_.opt.inputTimeScale >= 1.0 ? 0.15 : ctx_.opt.inputTimeScale;
        scale_ = scale;
        baseTicks_ = SDL_GetTicks();
        report_.section("INPUT TEST");
        report_.line("Prompts follow on the screen. Press what is asked. Do NOT press SELECT and START together:");
        report_.line("that is the exit chord. Every raw event is logged in 'INPUT EVENT LOG'.");
        report_.save();

        calibrate();
        for (int i = 0; i < kNumPrompts && !ctx_.quitRequested; ++i) {
            results_.push_back(runPrompt(kPrompts[i], i));
        }
        summarize();
        writeEventLog();
    }

private:
    double now() const { return nowS() - startS_; }

    void note(const std::string& line) {
        // Keep the raw log out of the on-screen list but visible in the report.
        if (eventLog_.size() < kMaxLoggedEvents) {
            eventLog_.push_back(line);
        } else {
            ++droppedEvents_;
        }
        recent_.push_back(line);
        if (recent_.size() > 12) {
            recent_.erase(recent_.begin());
        }
    }

    void handleDeviceEvent(const SDL_Event& e) {
        if (e.type == SDL_JOYDEVICEADDED) {
            report_.line(strf("device added (joystick index %d)", e.jdevice.which));
            openJoystickIndex(report_, e.jdevice.which);
        } else if (e.type == SDL_JOYDEVICEREMOVED) {
            report_.line(strf("device removed (instance %d)", e.jdevice.which));
        } else if (e.type == SDL_CONTROLLERDEVICEADDED) {
            report_.line(strf("controller added (index %d)", e.cdevice.which));
        }
    }

    // Reads rest values by polling and from events while the human leaves everything alone.
    void calibrate() {
        startS_ = nowS();
        snapshotAxes();
        const double start = nowS();
        const double duration = kCalibrateS * std::max(0.1, scale_);
        while (nowS() - start < duration && !ctx_.quitRequested) {
            SDL_Event e;
            while (SDL_PollEvent(&e) != 0) {
                if (ctx_.handleCommonEvent(e)) {
                    continue;
                }
                handleDeviceEvent(e);
                EventInfo info;
                if (describeEvent(e, info) && info.isAxis) {
                    rest_[info.key] = info.value;
                }
            }
            drawPromptScreen("HANDS OFF", "measuring rest values of sticks and triggers", 0, kNumPrompts,
                             (nowS() - start) / duration);
        }
        snapshotAxes();
        report_.line("-- rest values (hands off) --");
        for (const auto& kv : rest_) {
            report_.line(strf("  %-28s rest = %6d", kv.first.c_str(), kv.second));
        }
        if (rest_.empty()) {
            report_.line("  (no axes found)");
        }
    }

    void snapshotAxes() {
        SDL_JoystickUpdate();
        for (SDL_Joystick* joy : devices().joysticks) {
            const int n = SDL_JoystickNumAxes(joy);
            for (int i = 0; i < n; ++i) {
                rest_[strf("JOY axis %d", i)] = SDL_JoystickGetAxis(joy, i);
            }
        }
        for (SDL_GameController* gc : devices().controllers) {
            for (int a = 0; a < SDL_CONTROLLER_AXIS_MAX; ++a) {
                const char* name = SDL_GameControllerGetStringForAxis(static_cast<SDL_GameControllerAxis>(a));
                if (name != nullptr) {
                    rest_[strf("CTRL axis %s", name)] = SDL_GameControllerGetAxis(gc, static_cast<SDL_GameControllerAxis>(a));
                }
            }
        }
    }

    bool isSignificant(const EventInfo& info) const {
        if (info.isAxis) {
            const auto it = rest_.find(info.key);
            const int rest = it != rest_.end() ? it->second : 0;
            return std::abs(info.value - rest) >= kSignificantAxisDelta;
        }
        return info.isPress;
    }

    PromptResult runPrompt(const Prompt& prompt, int index) {
        PromptResult result;
        result.prompt = &prompt;
        report_.line(strf("[%2d/%d] %s", index + 1, kNumPrompts, prompt.text));
        const double timeout = prompt.timeoutS * scale_;
        const double linger = prompt.lingerS * scale_;
        const double start = nowS();
        double firstSignificantS = -1.0;
        shotTaken_ = false;
        if (ctx_.opt.selfTestInput) {
            injectSynthetic(prompt.id);
        }
        while (!ctx_.quitRequested) {
            const double t = nowS() - start;
            SDL_Event e;
            while (SDL_PollEvent(&e) != 0) {
                if (ctx_.handleCommonEvent(e)) {
                    continue;
                }
                handleDeviceEvent(e);
                EventInfo info;
                if (!describeEvent(e, info)) {
                    continue;
                }
                const double ts = static_cast<double>(e.common.timestamp - baseTicks_) / 1000.0;
                note(strf("[%8.3f] prompt=%-7s %s", ts, prompt.id, info.text.c_str()));
                addStat(result.stats, info);
                addStat(overall_, info);
                if (isSignificant(info) && firstSignificantS < 0.0) {
                    firstSignificantS = nowS() - start;
                    result.significant = true;
                }
            }
            if (t >= timeout) {
                break;
            }
            if (firstSignificantS >= 0.0 && t - firstSignificantS >= linger) {
                break;
            }
            drawPromptScreen(prompt.id, prompt.text, index, kNumPrompts, t / timeout);
        }
        // One-line outcome on screen and in the summary.
        std::string line = strf("   %-8s -> ", prompt.id);
        if (result.stats.empty()) {
            line += "no events";
        }
        bool first = true;
        for (const auto& kv : result.stats) {
            if (!first) {
                line += " | ";
            }
            first = false;
            line += kv.first;
            if (kv.second.isAxis) {
                line += strf(" [%d..%d, %zu values]", kv.second.minV, kv.second.maxV, kv.second.distinct.size());
            }
        }
        report_.line(line);
        return result;
    }

    void drawPromptScreen(const std::string& id, const std::string& text, int index, int total, double progress) {
        if (!ctx_.glOk) {
            SDL_Delay(2);
            return;
        }
        ctx_.beginUiFrame();
        const float s = static_cast<float>(ctx_.uiScale());
        TextRenderer& tr = ctx_.textRenderer;
        tr.text(8 * s, 6 * s, 2 * s, Color{1.0f, 0.85f, 0.2f, 1.0f}, strf("INPUT TEST %d/%d", index + 1, total));
        // The prompt, wrapped at 26 characters of scale 3.
        const int cols = std::max(8, static_cast<int>(static_cast<float>(ctx_.widthPx) / (24.0f * s)) - 1);
        std::string rest = text;
        float y = 40 * s;
        while (!rest.empty()) {
            size_t cut = rest.size() <= static_cast<size_t>(cols) ? rest.size() : rest.rfind(' ', static_cast<size_t>(cols));
            if (cut == std::string::npos || cut == 0) {
                cut = std::min(rest.size(), static_cast<size_t>(cols));
            }
            tr.text(8 * s, y, 3 * s, Color{0.4f, 1.0f, 0.5f, 1.0f}, rest.substr(0, cut));
            y += 28 * s;
            rest = cut < rest.size() ? rest.substr(cut + 1) : std::string();
        }
        const float barW = static_cast<float>(ctx_.widthPx) - 16 * s;
        tr.rect(8 * s, y + 4 * s, barW, 8 * s, Color{0.2f, 0.25f, 0.3f, 1});
        tr.rect(8 * s, y + 4 * s, barW * static_cast<float>(std::min(1.0, std::max(0.0, progress))), 8 * s,
                Color{0.9f, 0.6f, 0.2f, 1});
        // Recent raw events.
        float ey = y + 24 * s;
        tr.text(8 * s, ey, s, Color{0.55f, 0.65f, 0.75f, 1.0f}, "latest raw events:");
        ey += 10 * s;
        const int maxCols = std::max(10, static_cast<int>(static_cast<float>(ctx_.widthPx) / (8.0f * s)) - 2);
        for (const auto& l : recent_) {
            std::string shown = l;
            if (static_cast<int>(shown.size()) > maxCols) {
                shown.resize(static_cast<size_t>(maxCols));
            }
            tr.text(8 * s, ey, s, Color{0.85f, 0.95f, 0.85f, 1.0f}, shown);
            ey += 8 * s;
        }
        tr.text(8 * s, static_cast<float>(ctx_.heightPx) - 12 * s, s, Color{0.55f, 0.65f, 0.75f, 1.0f},
                "id: " + id + "   (SELECT+START together exits)");
        if (!shotTaken_ && progress > 0.2) {
            shotTaken_ = true;
            ctx_.requestShot("input");
        }
        ctx_.endUiFrame();
    }

    // ---- analysis -------------------------------------------------------------------------

    const PromptResult* find(const char* id) const {
        for (const auto& r : results_) {
            if (std::string(r.prompt->id) == id) {
                return &r;
            }
        }
        return nullptr;
    }

    static std::string keysOf(const PromptResult* r, bool pressOnly) {
        if (r == nullptr || r->stats.empty()) {
            return "no event seen";
        }
        std::string out;
        for (const auto& kv : r->stats) {
            if (pressOnly && !kv.second.isAxis && !kv.second.sawPress) {
                continue;
            }
            if (kv.second.isAxis) {
                const int delta = kv.second.maxV - kv.second.minV;
                if (delta < kSignificantAxisDelta) {
                    continue;  // noise
                }
            }
            if (!out.empty()) {
                out += ", ";
            }
            out += kv.first;
        }
        return out.empty() ? "no significant event" : out;
    }

    // Name after "CTRL button " of the first controller button pressed in this prompt, else "".
    static std::string ctrlButtonName(const PromptResult* r) {
        if (r == nullptr) {
            return std::string();
        }
        for (const auto& kv : r->stats) {
            if (kv.first.compare(0, 12, "CTRL button ") == 0 && kv.second.sawPress) {
                return kv.first.substr(12);
            }
        }
        return std::string();
    }
    static std::string joyButtonName(const PromptResult* r) {
        if (r == nullptr) {
            return std::string();
        }
        for (const auto& kv : r->stats) {
            if (kv.first.compare(0, 11, "JOY button ") == 0 && kv.second.sawPress) {
                return kv.first.substr(11);
            }
        }
        return std::string();
    }

    // "analog (N distinct values on <key>)", "digital (...)" or "no event".
    std::string triggerVerdict(const char* id) const {
        const PromptResult* r = find(id);
        if (r == nullptr || r->stats.empty()) {
            return "no event seen";
        }
        size_t bestDistinct = 0;
        std::string bestKey;
        for (const auto& kv : r->stats) {
            if (kv.second.isAxis && kv.second.maxV - kv.second.minV >= kSignificantAxisDelta &&
                kv.second.distinct.size() > bestDistinct) {
                bestDistinct = kv.second.distinct.size();
                bestKey = kv.first;
            }
        }
        if (bestDistinct >= 4) {
            const KeyStat& s = r->stats.at(bestKey);
            return strf("ANALOG (%s, %zu distinct values, range %d..%d)", bestKey.c_str(), bestDistinct, s.minV, s.maxV);
        }
        if (bestDistinct > 0) {
            return strf("DIGITAL (%s only reported %zu distinct values)", bestKey.c_str(), bestDistinct);
        }
        return "DIGITAL (button only: " + keysOf(r, true) + ")";
    }

    void summarize() {
        report_.section("INPUT SUMMARY");
        report_.line("What each physical control produced (CTRL = SDL game controller layer, JOY = raw joystick):");
        for (const auto& r : results_) {
            report_.line(strf("  %-8s %s", r.prompt->id, keysOf(&r, true).c_str()));
        }

        // Face buttons.
        std::string faces;
        int labelMatches = 0;
        int positional = 0;
        for (const char* id : {"A", "B", "X", "Y"}) {
            const PromptResult* r = find(id);
            const std::string c = ctrlButtonName(r);
            const std::string j = joyButtonName(r);
            faces += strf("%s->%s%s%s%s  ", id, c.empty() ? "(none)" : c.c_str(), j.empty() ? "" : " [joy button ",
                          j.c_str(), j.empty() ? "" : "]");
            std::string lower = id;
            lower[0] = static_cast<char>(std::tolower(lower[0]));
            if (c == lower) {
                ++labelMatches;
            }
        }
        const std::string cA = ctrlButtonName(find("A"));
        const std::string cB = ctrlButtonName(find("B"));
        const std::string cX = ctrlButtonName(find("X"));
        const std::string cY = ctrlButtonName(find("Y"));
        positional = (cA == "b" ? 1 : 0) + (cB == "a" ? 1 : 0) + (cX == "y" ? 1 : 0) + (cY == "x" ? 1 : 0);
        std::string faceVerdict;
        if (cA.empty() && cB.empty() && cX.empty() && cY.empty()) {
            faceVerdict = "no controller button events seen for A/B/X/Y";
        } else if (labelMatches == 4) {
            faceVerdict = "SDL names MATCH the printed labels (A=a, B=b, X=x, Y=y)";
        } else if (positional == 4) {
            faceVerdict = "SDL names are POSITIONAL: printed A reports 'b', printed B reports 'a', X reports 'y', Y reports 'x' (A/B and X/Y swapped vs labels)";
        } else {
            faceVerdict = "MIXED, see per-button list";
        }
        report_.fact(rows::kFaceButtons, "pressing printed label -> SDL name: " + faces + "=> " + faceVerdict);

        // Triggers.
        report_.fact(rows::kTriggers, "L2: " + triggerVerdict("L2") + "; R2: " + triggerVerdict("R2"));

        // Stick clicks.
        auto click = [&](const char* id, const char* sdlName) {
            const PromptResult* r = find(id);
            if (r == nullptr || r->stats.empty()) {
                return std::string("NOT seen (no event)");
            }
            const std::string c = ctrlButtonName(r);
            const std::string j = joyButtonName(r);
            std::string s = "present: ";
            s += c.empty() ? std::string("no CTRL button") : "CTRL " + c + (c == sdlName ? " (expected name)" : " (unexpected name)");
            s += j.empty() ? std::string() : ", joy button " + j;
            if (c.empty() && j.empty()) {
                s = "events seen but no button press: " + keysOf(r, false);
            }
            return s;
        };
        report_.fact(rows::kStickClicks, std::string("L3: ") + click("L3", "leftstick") + "; R3: " + click("R3", "rightstick"));

        // Menu key.
        const PromptResult* menu = find("MENU");
        std::string menuText = keysOf(menu, true);
        if (menu == nullptr || menu->stats.empty()) {
            menuText = "NO event during the MENU prompt (key probably handled by the OS or not wired to a port-visible device)";
        } else {
            menuText = "visible to ports as: " + menuText;
        }
        report_.fact(rows::kMenuKey, menuText + "; VOL+: " + keysOf(find("VOLUP"), true) + "; VOL-: " + keysOf(find("VOLDOWN"), true));

        report_.line("-- D-pad --");
        report_.line("  D-pad delivered as: " + keysOf(find("DPAD"), true));

        // Stick ranges over the whole test.
        report_.line("-- axis ranges over the whole test (rest value in brackets) --");
        for (const auto& kv : overall_) {
            if (!kv.second.isAxis) {
                continue;
            }
            const auto it = rest_.find(kv.first);
            report_.line(strf("  %-28s min %6d  max %6d  rest [%6d]  distinct %zu  events %d", kv.first.c_str(),
                              kv.second.minV, kv.second.maxV, it != rest_.end() ? it->second : 0, kv.second.distinct.size(),
                              kv.second.count));
        }
        report_.save();
    }

    void writeEventLog() {
        report_.section("INPUT EVENT LOG (every raw event: seconds since test start)");
        for (const auto& l : eventLog_) {
            report_.quietLine(l);
        }
        if (droppedEvents_ > 0) {
            report_.quietLine(strf("... %zu further events not logged (log cap %zu); statistics above include them",
                                   droppedEvents_, kMaxLoggedEvents));
        }
        report_.save();
    }

    Ctx& ctx_;
    Report& report_;
    double scale_ = 1.0;
    double startS_ = 0.0;
    Uint32 baseTicks_ = 0;
    std::map<std::string, int> rest_;
    StatMap overall_;
    std::vector<PromptResult> results_;
    std::vector<std::string> eventLog_;
    std::vector<std::string> recent_;
    size_t droppedEvents_ = 0;
    bool shotTaken_ = false;
};

}  // namespace

void reportControllers(Ctx& ctx) {
    Report& report = *ctx.report;
    report.section("CONTROLLERS");
    const char* envMapping = std::getenv("SDL_GAMECONTROLLERCONFIG");
    report.kv("SDL_GAMECONTROLLERCONFIG env", envMapping != nullptr ? strf("set (%zu chars)", std::strlen(envMapping)) : "unset");
    if (envMapping != nullptr) {
        std::string m = envMapping;
        for (char& c : m) {
            if (c == '\n') c = '|';
        }
        report.line("  " + m.substr(0, 600));
    }
    const int n = SDL_NumJoysticks();
    report.kv("SDL_NumJoysticks", strf("%d", n));
    if (n == 0) {
        report.line("NO JOYSTICKS/CONTROLLERS DETECTED by SDL (keyboard events are still logged).");
    }
    for (int i = 0; i < n; ++i) {
        const char* jname = SDL_JoystickNameForIndex(i);
        const SDL_JoystickGUID guid = SDL_JoystickGetDeviceGUID(i);
        report.line(strf("-- device %d: \"%s\"", i, jname != nullptr ? jname : "?"));
        report.kv("  GUID", guidString(guid));
        const bool isGc = SDL_IsGameController(i) != SDL_FALSE;
        report.kv("  is game controller (has mapping)", isGc ? "yes" : "NO (raw joystick only; ports need a mapping)");
        openJoystickIndex(report, i);
    }
    for (SDL_Joystick* joy : devices().joysticks) {
        report.line(strf("joystick \"%s\": %d axes, %d buttons, %d hats, %d balls, power %s",
                         SDL_JoystickName(joy) != nullptr ? SDL_JoystickName(joy) : "?", SDL_JoystickNumAxes(joy),
                         SDL_JoystickNumButtons(joy), SDL_JoystickNumHats(joy), SDL_JoystickNumBalls(joy),
                         powerLevelName(SDL_JoystickCurrentPowerLevel(joy))));
    }
    for (SDL_GameController* gc : devices().controllers) {
        const char* name = SDL_GameControllerName(gc);
        report.line(strf("-- game controller \"%s\"", name != nullptr ? name : "?"));
        if (char* mapping = SDL_GameControllerMapping(gc)) {
            report.kv("  mapping string", mapping);
            SDL_free(mapping);
        }
        report.line("  button bindings (SDL name -> physical source):");
        for (int b = 0; b < SDL_CONTROLLER_BUTTON_MAX; ++b) {
            const char* bn = SDL_GameControllerGetStringForButton(static_cast<SDL_GameControllerButton>(b));
            const SDL_GameControllerButtonBind bind = SDL_GameControllerGetBindForButton(gc, static_cast<SDL_GameControllerButton>(b));
            if (bn != nullptr && bind.bindType != SDL_CONTROLLER_BINDTYPE_NONE) {
                report.line(strf("    %-14s <- %s", bn, bindingString(bind).c_str()));
            }
        }
        report.line("  axis bindings:");
        for (int a = 0; a < SDL_CONTROLLER_AXIS_MAX; ++a) {
            const char* an = SDL_GameControllerGetStringForAxis(static_cast<SDL_GameControllerAxis>(a));
            const SDL_GameControllerButtonBind bind = SDL_GameControllerGetBindForAxis(gc, static_cast<SDL_GameControllerAxis>(a));
            if (an != nullptr && bind.bindType != SDL_CONTROLLER_BINDTYPE_NONE) {
                report.line(strf("    %-14s <- %s", an, bindingString(bind).c_str()));
            }
        }
    }
    (void)ctx;
}

void runInputTest(Ctx& ctx) {
    InputTester tester(ctx);
    tester.run();
}

void closeControllers() {
    for (SDL_GameController* gc : devices().controllers) {
        SDL_GameControllerClose(gc);
    }
    for (SDL_Joystick* joy : devices().joysticks) {
        SDL_JoystickClose(joy);
    }
    devices().controllers.clear();
    devices().joysticks.clear();
}

}  // namespace probe
