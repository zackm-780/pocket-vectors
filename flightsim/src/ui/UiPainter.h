#pragma once
// Drawing helpers for the menus (WP H5) on top of Renderer2D (H1). Layout is designed on the 640x480
// virtual canvas and centered on wider screens (HUD.md section 2). Stroke font sizes: L titles,
// M rows, S hints. The accent color follows the HUD color setting.

#include <string>
#include <vector>

#include "app/Settings.h"
#include "hud/Renderer2D.h"
#include "ui/ListNav.h"
#include "ui/OnScreenKeyboard.h"

namespace fsim {

struct UiTheme {
    Color accent = colors::kHudGreen;
    Color text{0.88f, 0.91f, 0.93f};
    Color dim{0.52f, 0.58f, 0.62f};
    Color warn = colors::kAmber;
    Color panel{0.02f, 0.05f, 0.08f, 0.92f};

    static UiTheme fromHudColor(HudColor c);
};

namespace uilayout {
constexpr float kContentX = 40.0f;  // left edge of the content in the 640 design
constexpr float kContentW = 560.0f;
constexpr float kTitleY = 38.0f;     // baseline of the L title
constexpr float kSubtitleY = 58.0f;  // baseline of the S subtitle
constexpr float kListTop = 74.0f;
constexpr float kRowH = 26.0f;
constexpr int kListRows = 12;        // rows between the header and the footer
constexpr float kFooterRuleY = 440.0f;
constexpr float kFooterY = 462.0f;   // baseline of the hint line
}  // namespace uilayout

struct ListRow {
    std::string label, value;
    bool enabled = true;
    bool adjustable = false;  // draws "< value >" while selected (Left/Right change it)
    bool dimValue = false;
};

// Truncates to at most `maxChars` characters of the monospaced font (adds no ellipsis glyph).
std::string fitText(const std::string& text, int maxChars);

// Darkens the scene behind the menu.
void drawBackdrop(Renderer2D& r, float alpha);
// Full-screen menu frame: backdrop, title, subtitle, rules and the footer hint line.
void drawScreenFrame(Renderer2D& r, const UiTheme& t, const std::string& title,
                     const std::string& subtitle, const std::string& footer);
// Centered panel for the pause menu: dims the flight less and draws a titled box. Returns the
// panel's top-left corner and size through the out parameters.
void drawPanelFrame(Renderer2D& r, const UiTheme& t, float width, float height,
                    const std::string& title, float& outX, float& outY);
// Rows from nav.top(); the selected row is highlighted when `focused`.
void drawList(Renderer2D& r, const UiTheme& t, float x, float y, float w, float rowH,
              const ListNav& nav, const std::vector<ListRow>& rows, bool focused = true);
// Single-line text field with a cursor.
void drawTextField(Renderer2D& r, const UiTheme& t, float x, float y, float w, const char* label,
                   const std::string& text, bool focused);
// Character grid; keys are laid out on OnScreenKeyboard::kColumns unit columns.
void drawKeyboard(Renderer2D& r, const UiTheme& t, float x, float y, const OnScreenKeyboard& kb,
                  bool focused);
// Modal box with a message (lines separated by '\n') and buttons; `selected` is highlighted.
void drawDialog(Renderer2D& r, const UiTheme& t, const std::string& message,
                const std::vector<std::string>& buttons, int selected);

}  // namespace fsim
