#include "ui/HelpPages.h"

#include <SDL.h>

#include "BuildInfo.h"

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>

#include "core/Paths.h"

namespace fsim {
namespace {

const char* padLabel(PadInput in) {
    static const char* const names[] = {
        "A",  "B",  "X",  "Y",  "DPAD UP", "DPAD DOWN", "DPAD LEFT", "DPAD RIGHT",
        "L1", "R1", "L2", "R2", "SELECT",  "START",     "L3",        "R3"};
    static_assert(sizeof names / sizeof names[0] == kPadInputCount, "padLabel covers PadInput");
    const int i = static_cast<int>(in);
    return i >= 0 && i < kPadInputCount ? names[i] : "?";
}

const char* stickLabel(Stick s) {
    static const char* const names[] = {"L STICK X", "L STICK Y", "R STICK X", "R STICK Y"};
    const int i = static_cast<int>(s);
    return i >= 0 && i < static_cast<int>(Stick::Count) ? names[i] : "?";
}

void sortLines(std::vector<HelpLine>& lines) {
    std::stable_sort(lines.begin(), lines.end(), [](const HelpLine& a, const HelpLine& b) {
        if (a.sortKey != b.sortKey) return a.sortKey < b.sortKey;
        if (a.action != b.action) return a.action < b.action;
        return a.note < b.note;
    });
}

void addLayer(HelpLayerPage& page, const ControlsConfig& cfg, PadLayer layer) {
    for (const PadButtonBinding& b : cfg.padButtons[static_cast<int>(layer)]) {
        HelpLine line;
        line.sortKey = static_cast<int>(b.input);
        line.input = padLabel(b.input);
        line.action = actionName(b.action);
        page.lines.push_back(std::move(line));
    }
    for (const PadAxisBinding& a : cfg.padAxes[static_cast<int>(layer)]) {
        HelpLine line;
        line.sortKey = 100 + static_cast<int>(a.source);
        line.input = stickLabel(a.source);
        line.action = axisName(a.axis);
        if (a.invert) line.note = "inverted";
        page.lines.push_back(std::move(line));
    }
}

void addActionUses(HelpLayerPage& page, const ControlsConfig& cfg,
                   const std::vector<std::pair<Action, const char*>>& uses) {
    for (const auto& use : uses) {
        bool any = false;
        for (const PadButtonBinding& b : cfg.padButtons[static_cast<int>(PadLayer::Base)]) {
            if (b.action != use.first) continue;
            HelpLine line;
            line.sortKey = static_cast<int>(b.input);
            line.input = padLabel(b.input);
            line.action = actionName(b.action);
            line.note = use.second;
            page.lines.push_back(line);
            any = true;
        }
        if (!any) {
            HelpLine line;
            line.sortKey = 1000;
            line.input = "UNBOUND";
            line.action = actionName(use.first);
            line.note = use.second;
            page.lines.push_back(std::move(line));
        }
    }
}

std::string actionsOnBase(const ControlsConfig& cfg, PadInput in) {
    std::string out;
    auto add = [&](const std::string& name) {
        if (!out.empty()) out += ", ";
        out += name;
    };
    for (const PadButtonBinding& b : cfg.padButtons[static_cast<int>(PadLayer::Base)]) {
        if (b.input == in) add(actionName(b.action));
    }
    for (const PadChord& c : cfg.chords) {
        if (c.a == in || c.b == in) add(std::string("chord ") + actionName(c.action));
    }
    return out.empty() ? "none" : out;
}

std::string sdlButtonList(const ControlsConfig& cfg, PadInput in) {
    std::string out;
    const int i = static_cast<int>(in);
    for (int j = 0; j < cfg.sdlButtonCountForLabel[i]; ++j) {
        const auto button = static_cast<SDL_GameControllerButton>(cfg.sdlButtonsForLabel[i][j]);
        const char* name = SDL_GameControllerGetStringForButton(button);
        if (!name || name[0] == '\0') continue;
        if (!out.empty()) out += ", ";
        out += name;
    }
    return out.empty() ? "-" : out;
}

std::vector<std::string> wrapText(const std::string& text, int maxChars) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream words(text);
    std::string word;
    while (words >> word) {
        if (cur.empty()) {
            cur = word;
        } else if (static_cast<int>(cur.size() + 1 + word.size()) <= maxChars) {
            cur += " ";
            cur += word;
        } else {
            out.push_back(cur);
            cur = word;
        }
    }
    if (!cur.empty()) out.push_back(cur);
    if (out.empty()) out.emplace_back();
    return out;
}

const char* const kHelpOrder[] = {"first-flight", "autopilot", "flight-plan", "ils",
                                  "map",          "fms-plans", "data-packs",  "troubleshooting"};

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

HelpTextPage pageFromFile(const std::string& id, const std::string& path) {
    HelpTextPage page;
    page.id = id;
    const std::string text = readFile(path);
    std::istringstream ss(text);
    std::string line;
    bool title = false;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (!title) {
            page.title = line.empty() ? id : line;
            title = true;
            continue;
        }
        if (line.empty()) {
            page.lines.emplace_back();
            continue;
        }
        const auto wrapped = wrapText(line, 68);
        page.lines.insert(page.lines.end(), wrapped.begin(), wrapped.end());
    }
    if (page.title.empty()) page.title = id;
    return page;
}

// Rewrites [device_labels] keys. Every other line, including comments, is copied through.
bool writeLabelKeys(const std::string& path, const std::map<std::string, std::string>& keys,
                    std::string& error) {
    std::vector<std::string> lines;
    {
        std::ifstream in(path);
        std::string line;
        if (in) {
            while (std::getline(in, line)) {
                if (!line.empty() && line.back() == '\r') line.pop_back();
                lines.push_back(line);
            }
        }
    }
    std::vector<std::string> out;
    bool inSection = false;
    bool found = false;
    std::map<std::string, bool> written;
    auto flushMissing = [&]() {
        for (const auto& kv : keys) {
            if (written[kv.first]) continue;
            out.push_back(kv.first + " = " + kv.second);
            written[kv.first] = true;
        }
    };
    for (const std::string& line : lines) {
        const std::string trimmed = std::string(trim(line));
        if (!trimmed.empty() && trimmed[0] == '[') {
            if (inSection) flushMissing();
            inSection = toLower(trimmed) == "[device_labels]";
            if (inSection) found = true;
            out.push_back(line);
            continue;
        }
        if (inSection && !trimmed.empty() && trimmed[0] != ';' && trimmed[0] != '#') {
            const size_t eq = trimmed.find('=');
            if (eq != std::string::npos) {
                const std::string key = toLower(trim(trimmed.substr(0, eq)));
                const auto it = keys.find(key);
                if (it != keys.end()) {
                    out.push_back(key + " = " + it->second);
                    written[key] = true;
                    continue;
                }
            }
        }
        out.push_back(line);
    }
    if (inSection) flushMissing();
    if (!found) {
        if (!out.empty() && !out.back().empty()) out.emplace_back();
        out.emplace_back("[device_labels]");
        for (const auto& kv : keys) out.push_back(kv.first + " = " + kv.second);
    }
    std::ofstream file(path, std::ios::trunc);
    if (!file) {
        error = "cannot write " + path;
        return false;
    }
    for (const std::string& line : out) file << line << '\n';
    if (!file) {
        error = "cannot write " + path;
        return false;
    }
    return true;
}

}  // namespace

std::string HelpLine::text() const {
    std::string s = input + "  " + action;
    if (!note.empty()) {
        s += "  ";
        s += note;
    }
    return s;
}

std::string HelpLayerPage::plain() const {
    std::string s;
    for (const HelpLine& line : lines) {
        s += line.text();
        s += '\n';
    }
    return s;
}

std::vector<HelpLayerPage> helpControlPages(const ControlsConfig& cfg) {
    std::vector<HelpLayerPage> pages(6);
    pages[0].id = "base";
    pages[0].title = "CONTROLS  BASE";
    pages[1].id = "l1";
    pages[1].title = "CONTROLS  L1 AUTOPILOT";
    pages[2].id = "r1";
    pages[2].title = "CONTROLS  R1 SYSTEMS";
    pages[3].id = "l2";
    pages[3].title = "CONTROLS  L2 LOOK";
    pages[4].id = "map";
    pages[4].title = "CONTROLS  MAP";
    pages[5].id = "menus";
    pages[5].title = "CONTROLS  MENUS";

    addLayer(pages[0], cfg, PadLayer::Base);
    for (const PadChord& c : cfg.chords) {
        HelpLine line;
        line.sortKey = 200;
        line.input = std::string(padLabel(c.a)) + "+" + padLabel(c.b);
        line.action = actionName(c.action);
        pages[0].lines.push_back(std::move(line));
    }
    addLayer(pages[1], cfg, PadLayer::L1);
    addLayer(pages[2], cfg, PadLayer::R1);
    addLayer(pages[3], cfg, PadLayer::L2);
    {
        HelpLine held;
        held.sortKey = static_cast<int>(PadInput::L2);
        held.input = "L2";
        held.action = "LookHold";
        held.note = "while held";
        pages[3].lines.push_back(std::move(held));
    }
    addActionUses(pages[4], cfg,
                  {{Action::MenuUp, "zoom in"},
                   {Action::MenuDown, "zoom out"},
                   {Action::MenuLeft, "previous waypoint"},
                   {Action::MenuRight, "next waypoint"},
                   {Action::MenuAccept, "direct to"},
                   {Action::HudDeclutter, "north or track up"},
                   {Action::MenuBack, "close map"},
                   {Action::MapToggle, "close map"}});
    addActionUses(pages[5], cfg,
                  {{Action::MenuUp, "move"},
                   {Action::MenuDown, "move"},
                   {Action::MenuLeft, "move"},
                   {Action::MenuRight, "move"},
                   {Action::MenuAccept, "select"},
                   {Action::MenuBack, "back"},
                   {Action::Pause, "pause or close"},
                   {Action::SpeedbrakeCycle, "context"},
                   {Action::HudDeclutter, "context"}});
    for (HelpLayerPage& page : pages) sortLines(page.lines);
    return pages;
}

std::vector<HelpTextPage> loadHelpTexts(const std::string& dir) {
    std::vector<HelpTextPage> pages;
    if (dir.empty()) return pages;
    for (const char* id : kHelpOrder) {
        const std::string path = Paths::join(dir, std::string(id) + ".txt");
        if (!fileExists(path)) continue;
        pages.push_back(pageFromFile(id, path));
    }
    return pages;
}

std::vector<std::string> helpAboutLines(const std::string& attributionPath) {
    std::vector<std::string> lines = {
        "POCKET VECTORS",
        "RETRO FLIGHT SIM WITH MODERN DATA",
        "BY ZACK MARTIN",
        "",
        "github.com/zackm-780",
        "github.com/zackm-780/pocket-vectors",
        "buymeacoffee.com/zackm",
        "",
        std::string("VERSION ") + FSIM_VERSION_STRING,
        std::string("COMMIT ") + FSIM_GIT_COMMIT,
        "",
        "JSBSIM  LGPL-2.1",
        "",
    };
    if (attributionPath.empty() || !fileExists(attributionPath)) {
        lines.emplace_back("NO DATA PACK INSTALLED");
        return lines;
    }
    lines.emplace_back("DATA PACK");
    const std::string text = readFile(attributionPath);
    std::istringstream ss(text);
    std::string line;
    while (std::getline(ss, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) {
            lines.emplace_back();
            continue;
        }
        const auto wrapped = wrapText(line, 68);
        lines.insert(lines.end(), wrapped.begin(), wrapped.end());
    }
    return lines;
}

HelpPadReading describeHelpPad(const ControlsConfig& cfg, bool axis, int sdlCode) {
    HelpPadReading reading;
    if (!axis) {
        const auto button = static_cast<SDL_GameControllerButton>(sdlCode);
        const char* name = SDL_GameControllerGetStringForButton(button);
        reading.sdlName = name && name[0] ? name : "button";
        const PadInput in = cfg.padInputForSdlButton(sdlCode);
        if (in == PadInput::Count) {
            reading.label = "UNMAPPED";
            reading.baseAction = "none";
            return reading;
        }
        reading.label = padLabel(in);
        reading.baseAction = actionsOnBase(cfg, in);
        return reading;
    }
    const auto ax = static_cast<SDL_GameControllerAxis>(sdlCode);
    const char* name = SDL_GameControllerGetStringForAxis(ax);
    reading.sdlName = name && name[0] ? name : "axis";
    if (sdlCode == SDL_CONTROLLER_AXIS_TRIGGERLEFT || sdlCode == SDL_CONTROLLER_AXIS_TRIGGERRIGHT) {
        const PadInput in =
            sdlCode == SDL_CONTROLLER_AXIS_TRIGGERLEFT ? PadInput::L2 : PadInput::R2;
        reading.label = padLabel(in);
        reading.baseAction = actionsOnBase(cfg, in);
        return reading;
    }
    Stick stick = Stick::Count;
    switch (sdlCode) {
    case SDL_CONTROLLER_AXIS_LEFTX: stick = Stick::LeftX; break;
    case SDL_CONTROLLER_AXIS_LEFTY: stick = Stick::LeftY; break;
    case SDL_CONTROLLER_AXIS_RIGHTX: stick = Stick::RightX; break;
    case SDL_CONTROLLER_AXIS_RIGHTY: stick = Stick::RightY; break;
    default: break;
    }
    if (stick == Stick::Count) {
        reading.label = "AXIS";
        reading.baseAction = "none";
        return reading;
    }
    reading.label = stickLabel(stick);
    std::string actions;
    for (const PadAxisBinding& a : cfg.padAxes[static_cast<int>(PadLayer::Base)]) {
        if (a.source != stick) continue;
        if (!actions.empty()) actions += ", ";
        actions += axisName(a.axis);
        if (a.invert) actions += " inverted";
    }
    reading.baseAction = actions.empty() ? "none" : actions;
    return reading;
}

std::vector<std::string> helpLabelSummary(const ControlsConfig& cfg) {
    const PadInput labels[] = {PadInput::A, PadInput::B, PadInput::X, PadInput::Y,
                               PadInput::Select};
    std::vector<std::string> lines;
    for (PadInput in : labels) {
        lines.push_back(std::string(padLabel(in)) + " = " + sdlButtonList(cfg, in));
    }
    return lines;
}

bool swapDeviceLabels(const std::string& userPath, const ControlsConfig& effective, bool swapAB,
                      std::string& error) {
    if (userPath.empty()) {
        error = "no controls.ini path";
        return false;
    }
    const PadInput first = swapAB ? PadInput::A : PadInput::X;
    const PadInput second = swapAB ? PadInput::B : PadInput::Y;
    const char* keyA = swapAB ? "a" : "x";
    const char* keyB = swapAB ? "b" : "y";
    std::map<std::string, std::string> keys;
    keys[keyA] = sdlButtonList(effective, second);
    keys[keyB] = sdlButtonList(effective, first);
    if (keys[keyA] == "-" || keys[keyB] == "-") {
        error = "face button has no SDL name";
        return false;
    }
    return writeLabelKeys(userPath, keys, error);
}

}  // namespace fsim
