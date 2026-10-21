// Help menu (WP H6): index, live control pages, button test, how-to text, About.

#include <algorithm>
#include <cstdio>

#include "core/Paths.h"
#include "ui/HelpPages.h"
#include "ui/MenuScreens.h"
#include "ui/UiPainter.h"

namespace fsim {
namespace {

using namespace uilayout;

constexpr int kVisibleLines = 26;
constexpr float kLineH = 13.0f;

enum class HelpSection { Index, Controls, Buttons, Howto, About };

int pageIndex(const std::vector<HelpLayerPage>& pages, const std::string& id) {
    for (int i = 0; i < static_cast<int>(pages.size()); ++i)
        if (pages[static_cast<size_t>(i)].id == id) return i;
    return 0;
}

int textIndex(const std::vector<HelpTextPage>& pages, const std::string& id) {
    for (int i = 0; i < static_cast<int>(pages.size()); ++i)
        if (pages[static_cast<size_t>(i)].id == id) return i;
    return 0;
}

class HelpScreen : public Screen {
public:
    HelpScreen(const MenuSystem& menu, const std::string& spec) : indexNav_(4, 4) {
        load(menu.env());
        applySpec(spec);
    }
    ScreenId id() const override { return ScreenId::Help; }

    void update(const MenuInput& in, MenuSystem& m) override {
        const MenuSystem::PadProbeView& probe = m.padProbe();
        if (probe.valid && probe.serial != seenSerial_) {
            seenSerial_ = probe.serial;
            reading_ = describeHelpPad(cfg_, probe.axis, probe.sdlCode);
            haveReading_ = true;
        }
        if (section_ == HelpSection::Index) {
            indexNav_.handle(in);
            if (in.back) {
                m.pop();
                return;
            }
            if (!in.accept) return;
            section_ = static_cast<HelpSection>(indexNav_.selected() + 1);
            page_ = 0;
            scroll_ = 0;
            swapRow_ = 0;
            return;
        }
        if (in.back) {
            section_ = HelpSection::Index;
            scroll_ = 0;
            status_.clear();
            return;
        }
        if (section_ == HelpSection::Buttons) {
            if (in.up) swapRow_ = 0;
            if (in.down) swapRow_ = 1;
            if (in.left || in.right || in.accept) doSwap(m, swapRow_ == 0);
            return;
        }
        const int n = pageCount();
        if (in.left && n > 1) {
            page_ = (page_ + n - 1) % n;
            scroll_ = 0;
        }
        if (in.right && n > 1) {
            page_ = (page_ + 1) % n;
            scroll_ = 0;
        }
        const int lines = lineCount();
        const int maxScroll = std::max(0, lines - kVisibleLines);
        if (in.up) scroll_ = std::max(0, scroll_ - 1);
        if (in.down) scroll_ = std::min(maxScroll, scroll_ + 1);
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        if (section_ == HelpSection::Index) {
            drawScreenFrame(r, t, "HELP", "POCKET VECTORS", "A SELECT   B BACK");
            std::vector<ListRow> rows(4);
            rows[0].label = "CONTROLS";
            rows[1].label = "BUTTON TEST";
            rows[2].label = "HOW TO FLY";
            rows[3].label = "ABOUT";
            const float x = r.centered(kContentX);
            drawList(r, t, x, kListTop + 20, kContentW, 36, indexNav_, rows);
            return;
        }
        const std::string sub = subtitle();
        const char* footer = section_ == HelpSection::Buttons
                                 ? "LEFT RIGHT SWAP   B BACK"
                                 : "LEFT RIGHT PAGE   UP DOWN SCROLL   B BACK";
        drawScreenFrame(r, t, "HELP", sub, footer);
        const float x = r.centered(kContentX);
        float y = kListTop + 14;
        if (section_ == HelpSection::Buttons) {
            drawButtonTest(r, t, x, y);
            return;
        }
        const std::vector<std::string> lines = bodyLines();
        const int n = static_cast<int>(lines.size());
        for (int i = 0; i < kVisibleLines; ++i) {
            const int idx = scroll_ + i;
            if (idx >= n) break;
            r.text(x, y, lines[static_cast<size_t>(idx)].c_str(), TextSize::S, t.text);
            y += kLineH;
        }
        if (!status_.empty()) r.text(x, kFooterRuleY - 16, status_.c_str(), TextSize::S, t.warn);
    }

private:
    void load(const MenuEnv& env) {
        if (!env.controlsDefaultsPath.empty())
            cfg_.loadFiles(env.controlsDefaultsPath, env.controlsUserPath);
        controls_ = helpControlPages(cfg_);
        howto_ = loadHelpTexts(env.helpDir);
        about_ = helpAboutLines(env.attributionPath);
        userPath_ = env.controlsUserPath;
        defaultsPath_ = env.controlsDefaultsPath;
    }

    void applySpec(const std::string& spec) {
        std::string section, arg;
        const size_t colon = spec.find(':');
        if (colon == std::string::npos) {
            section = spec;
        } else {
            section = spec.substr(0, colon);
            arg = spec.substr(colon + 1);
        }
        for (char& c : section) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
        if (section == "controls" || section == "control") {
            section_ = HelpSection::Controls;
            page_ = pageIndex(controls_, arg);
        } else if (section == "buttons" || section == "button" || section == "test") {
            section_ = HelpSection::Buttons;
        } else if (section == "howto" || section == "how" || section == "guide") {
            section_ = HelpSection::Howto;
            page_ = textIndex(howto_, arg);
        } else if (section == "about") {
            section_ = HelpSection::About;
        } else {
            section_ = HelpSection::Index;
        }
    }

    void doSwap(MenuSystem& m, bool swapAB) {
        std::string error;
        if (!swapDeviceLabels(userPath_, cfg_, swapAB, error)) {
            status_ = error.empty() ? "SWAP FAILED" : error;
            return;
        }
        if (!defaultsPath_.empty()) cfg_.loadFiles(defaultsPath_, userPath_);
        controls_ = helpControlPages(cfg_);
        status_ = swapAB ? "SWAPPED A/B" : "SWAPPED X/Y";
        (void)m;
    }

    int pageCount() const {
        if (section_ == HelpSection::Controls) return static_cast<int>(controls_.size());
        if (section_ == HelpSection::Howto) return std::max(1, static_cast<int>(howto_.size()));
        return 1;
    }

    int lineCount() const { return static_cast<int>(bodyLines().size()); }

    std::string subtitle() const {
        char pageNo[32] = "";
        const int n = pageCount();
        if (n > 1) std::snprintf(pageNo, sizeof pageNo, "   %d/%d", page_ + 1, n);
        if (section_ == HelpSection::Controls && !controls_.empty())
            return controls_[static_cast<size_t>(page_)].title + pageNo;
        if (section_ == HelpSection::Howto) {
            if (howto_.empty()) return std::string("HOW TO FLY") + pageNo;
            return howto_[static_cast<size_t>(page_)].title + pageNo;
        }
        if (section_ == HelpSection::About) return "ABOUT";
        return "BUTTON TEST";
    }

    std::vector<std::string> bodyLines() const {
        if (section_ == HelpSection::Controls && !controls_.empty()) {
            std::vector<std::string> lines;
            for (const HelpLine& line : controls_[static_cast<size_t>(page_)].lines)
                lines.push_back(line.text());
            if (lines.empty()) lines.emplace_back("NO BINDINGS");
            return lines;
        }
        if (section_ == HelpSection::Howto) {
            if (howto_.empty()) return {"NO HELP TEXT"};
            return howto_[static_cast<size_t>(page_)].lines;
        }
        if (section_ == HelpSection::About) return about_;
        return {};
    }

    void drawButtonTest(Renderer2D& r, const UiTheme& t, float x, float y) const {
        r.text(x, y, "PRESS A BUTTON OR MOVE A STICK", TextSize::S, t.dim);
        y += kLineH * 2;
        if (!haveReading_) {
            r.text(x, y, "LAST  NONE", TextSize::S, t.text);
            y += kLineH;
        } else {
            r.text(x, y, ("LAST  " + reading_.label).c_str(), TextSize::M, t.accent);
            y += 20;
            r.text(x, y, ("SDL  " + reading_.sdlName).c_str(), TextSize::S, t.text);
            y += kLineH;
            r.text(x, y, ("BASE  " + reading_.baseAction).c_str(), TextSize::S, t.text);
            y += kLineH;
        }
        y += kLineH;
        for (const std::string& line : helpLabelSummary(cfg_)) {
            r.text(x, y, line.c_str(), TextSize::S, t.text);
            y += kLineH;
        }
        y += kLineH;
        const char* rows[2] = {"SWAP A/B", "SWAP X/Y"};
        for (int i = 0; i < 2; ++i) {
            const bool sel = i == swapRow_;
            if (sel)
                r.fillRect(x, y - 12, kContentW, 20, t.accent.withAlpha(0.25f), FillMode::Back);
            r.text(x + 8, y, rows[i], TextSize::M, sel ? t.accent : t.text);
            y += 28;
        }
        if (!status_.empty()) r.text(x, y + 8, status_.c_str(), TextSize::S, t.warn);
    }

    ControlsConfig cfg_;
    std::vector<HelpLayerPage> controls_;
    std::vector<HelpTextPage> howto_;
    std::vector<std::string> about_;
    std::string userPath_, defaultsPath_, status_;
    HelpSection section_ = HelpSection::Index;
    int page_ = 0;
    int scroll_ = 0;
    int swapRow_ = 0;
    ListNav indexNav_;
    HelpPadReading reading_;
    bool haveReading_ = false;
    unsigned seenSerial_ = 0;
};

}  // namespace

std::unique_ptr<Screen> makeHelpScreen(const MenuSystem& menu, const std::string& spec) {
    return std::make_unique<HelpScreen>(menu, spec);
}

}  // namespace fsim
