#pragma once
// Help menu content (WP H6). Bindings come from the live ControlsConfig. How-to pages are plain
// text files. Nothing here draws.

#include <string>
#include <vector>

#include "input/ControlsConfig.h"

namespace fsim {

struct HelpLine {
    int sortKey = 0;
    std::string input;   // printed label, stick, or chord ("A", "L STICK Y", "SELECT+START")
    std::string action;  // actionName() / axisName()
    std::string note;    // optional context ("zoom in", "inverted")
    std::string text() const;
};

struct HelpLayerPage {
    std::string id;  // base, l1, r1, l2, map, menus
    std::string title;
    std::vector<HelpLine> lines;
    std::string plain() const;
};

struct HelpTextPage {
    std::string id;
    std::string title;
    std::vector<std::string> lines;
};

struct HelpPadReading {
    std::string label;       // printed label, stick name, or "UNMAPPED"
    std::string sdlName;     // SDL GameController name
    std::string baseAction;  // base-layer actions that input would fire
};

// Six pages: base, L1, R1, L2, map, menus. Button names are the printed labels from `cfg`.
std::vector<HelpLayerPage> helpControlPages(const ControlsConfig& cfg);

// How-to pages from `dir`, in the documented order. Missing files are skipped.
std::vector<HelpTextPage> loadHelpTexts(const std::string& dir);

// Fixed credits plus data/ATTRIBUTION.txt when `attributionPath` exists.
std::vector<std::string> helpAboutLines(const std::string& attributionPath);

// `axis` false: `sdlCode` is an SDL_GameControllerButton. True: an SDL_GameControllerAxis.
HelpPadReading describeHelpPad(const ControlsConfig& cfg, bool axis, int sdlCode);

// "A = a" lines for the face buttons and Select, from the label table.
std::vector<std::string> helpLabelSummary(const ControlsConfig& cfg);

// Exchanges the SDL buttons of A/B (`swapAB`) or X/Y in `userPath`. Other lines of that file stay.
// The new values come from `effective` (defaults already merged with the current override).
bool swapDeviceLabels(const std::string& userPath, const ControlsConfig& effective, bool swapAB,
                      std::string& error);

}  // namespace fsim
