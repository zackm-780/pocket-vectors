#include "ui/UiPainter.h"

#include <algorithm>

namespace fsim {

using namespace uilayout;

UiTheme UiTheme::fromHudColor(HudColor c) {
    UiTheme t;
    switch (c) {
    case HudColor::White: t.accent = colors::kWhite; break;
    case HudColor::Amber: t.accent = colors::kAmber; break;
    case HudColor::Green: t.accent = colors::kHudGreen; break;
    }
    return t;
}

std::string fitText(const std::string& text, int maxChars) {
    if (maxChars <= 0) return std::string();
    if (static_cast<int>(text.size()) <= maxChars) return text;
    return text.substr(0, static_cast<size_t>(maxChars));
}

void drawBackdrop(Renderer2D& r, float alpha) {
    r.fillRect(r.leftX(), 0, r.virtualWidth(), r.virtualHeight(), Color(0.0f, 0.02f, 0.04f, alpha),
               FillMode::Back);
}

void drawScreenFrame(Renderer2D& r, const UiTheme& t, const std::string& title,
                     const std::string& subtitle, const std::string& footer) {
    drawBackdrop(r, 0.94f);
    const float x = r.centered(kContentX);
    r.text(x, kTitleY, title.c_str(), TextSize::L, t.accent);
    if (!subtitle.empty()) r.text(x, kSubtitleY, subtitle.c_str(), TextSize::S, t.dim);
    r.line(x, kSubtitleY + 8, x + kContentW, kSubtitleY + 8, t.accent.withAlpha(0.5f));
    r.line(x, kFooterRuleY, x + kContentW, kFooterRuleY, t.accent.withAlpha(0.5f));
    r.text(x, kFooterY, footer.c_str(), TextSize::S, t.text);
}

void drawPanelFrame(Renderer2D& r, const UiTheme& t, float width, float height,
                    const std::string& title, float& outX, float& outY) {
    drawBackdrop(r, 0.55f);
    outX = r.centerX() - width * 0.5f;
    outY = (r.virtualHeight() - height) * 0.5f;
    r.fillRect(outX, outY, width, height, t.panel, FillMode::Back);
    r.strokeRect(outX, outY, width, height, t.accent);
    r.text(r.centerX(), outY + 26, title.c_str(), TextSize::L, t.accent, HAlign::Center);
    r.line(outX + 12, outY + 36, outX + width - 12, outY + 36, t.accent.withAlpha(0.5f));
}

void drawList(Renderer2D& r, const UiTheme& t, float x, float y, float w, float rowH,
              const ListNav& nav, const std::vector<ListRow>& rows, bool focused) {
    const int n = static_cast<int>(rows.size());
    const int charsPerRow = static_cast<int>((w - 20.0f) / StrokeFont::advancePx(TextSize::M));
    for (int i = 0; i < nav.visibleRows(); ++i) {
        const int idx = nav.top() + i;
        if (idx >= n) break;
        const ListRow& row = rows[static_cast<size_t>(idx)];
        const float ry = y + static_cast<float>(i) * rowH;
        const bool sel = focused && idx == nav.selected();
        Color labelColor = row.enabled ? t.text : t.dim.withAlpha(0.7f);
        if (sel) {
            r.fillRect(x, ry, w, rowH - 2, t.accent.withAlpha(0.25f), FillMode::Back);
            r.strokeRect(x, ry, w, rowH - 2, t.accent.withAlpha(0.8f));
            labelColor = t.accent;
        }
        const float base = ry + rowH * 0.5f + 5.0f;
        std::string value = row.value;
        if (sel && row.adjustable && !value.empty()) value = "< " + value + " >";
        const int valueChars = static_cast<int>(value.size());
        const int labelMax = std::max(4, charsPerRow - (valueChars > 0 ? valueChars + 2 : 0));
        r.text(x + 10, base, fitText(row.label, labelMax).c_str(), TextSize::M, labelColor);
        if (!value.empty()) {
            const Color vc = sel ? t.accent : (row.dimValue ? t.dim : t.text);
            r.text(x + w - 10, base, fitText(value, charsPerRow).c_str(), TextSize::M, vc,
                   HAlign::Right);
        }
    }
    // Scroll hints at the right edge.
    const float hx = x + w + 8.0f;
    if (nav.canScrollUp())
        r.fillTriangle({hx, y + 12}, {hx - 6, y + 22}, {hx + 6, y + 22}, t.dim, FillMode::Front);
    if (nav.canScrollDown()) {
        const float by = y + static_cast<float>(nav.visibleRows()) * rowH;
        r.fillTriangle({hx, by - 4}, {hx - 6, by - 14}, {hx + 6, by - 14}, t.dim, FillMode::Front);
    }
}

void drawTextField(Renderer2D& r, const UiTheme& t, float x, float y, float w, const char* label,
                   const std::string& text, bool focused) {
    const float labelW = StrokeFont::measure(label, TextSize::M) + 10;
    r.text(x, y + 20, label, TextSize::M, t.dim);
    const float bx = x + labelW;
    r.fillRect(bx, y, w - labelW, 28, Color(0, 0, 0, 0.5f), FillMode::Back);
    r.strokeRect(bx, y, w - labelW, 28, focused ? t.accent : t.dim);
    const std::string shown = text + (focused ? "_" : "");
    r.text(bx + 8, y + 20, shown.c_str(), TextSize::L, focused ? t.accent : t.text);
}

void drawKeyboard(Renderer2D& r, const UiTheme& t, float x, float y, const OnScreenKeyboard& kb,
                  bool focused) {
    constexpr float kUnitW = 46.0f, kKeyH = 28.0f, kGap = 4.0f;
    for (int row = 0; row < kb.rowCount(); ++row) {
        const auto& keys = kb.row(row);
        for (int k = 0; k < static_cast<int>(keys.size()); ++k) {
            const OnScreenKeyboard::Key& key = keys[static_cast<size_t>(k)];
            const float kx = x + static_cast<float>(key.col) * kUnitW;
            const float ky = y + static_cast<float>(row) * (kKeyH + kGap);
            const float kw = static_cast<float>(key.span) * kUnitW - kGap;
            const bool cur = focused && row == kb.cursorRow() && k == kb.cursorKey();
            if (cur) r.fillRect(kx, ky, kw, kKeyH, t.accent.withAlpha(0.35f), FillMode::Back);
            r.strokeRect(kx, ky, kw, kKeyH, cur ? t.accent : t.dim.withAlpha(0.8f));
            r.text(kx + kw * 0.5f, ky + kKeyH * 0.5f, key.label.c_str(), TextSize::M,
                   cur ? t.accent : t.text, HAlign::Center, VAlign::Middle);
        }
    }
}

void drawDialog(Renderer2D& r, const UiTheme& t, const std::string& message,
                const std::vector<std::string>& buttons, int selected) {
    drawBackdrop(r, 0.55f);
    std::vector<std::string> lines;
    size_t start = 0;
    while (start <= message.size()) {
        const size_t nl = message.find('\n', start);
        lines.push_back(message.substr(start, nl == std::string::npos ? nl : nl - start));
        if (nl == std::string::npos) break;
        start = nl + 1;
    }
    size_t maxChars = 20;
    for (const std::string& l : lines) maxChars = std::max(maxChars, l.size());
    const float w = std::min(580.0f, static_cast<float>(maxChars) * 9.0f + 60.0f);
    const float h = 70.0f + static_cast<float>(lines.size()) * 20.0f + 44.0f;
    const float x = r.centerX() - w * 0.5f, y = (r.virtualHeight() - h) * 0.5f;
    r.fillRect(x, y, w, h, t.panel, FillMode::Back);
    r.strokeRect(x, y, w, h, t.accent);
    float ly = y + 40.0f;
    for (const std::string& l : lines) {
        r.text(r.centerX(), ly, l.c_str(), TextSize::M, t.text, HAlign::Center);
        ly += 20.0f;
    }
    const float bw = 100.0f, gap = 20.0f;
    const float total = static_cast<float>(buttons.size()) * bw + gap * (buttons.size() - 1.0f);
    float bx = r.centerX() - total * 0.5f;
    const float by = y + h - 40.0f;
    for (size_t i = 0; i < buttons.size(); ++i) {
        const bool sel = static_cast<int>(i) == selected;
        if (sel) r.fillRect(bx, by, bw, 26, t.accent.withAlpha(0.3f), FillMode::Back);
        r.strokeRect(bx, by, bw, 26, sel ? t.accent : t.dim);
        r.text(bx + bw * 0.5f, by + 13, buttons[i].c_str(), TextSize::M, sel ? t.accent : t.text,
               HAlign::Center, VAlign::Middle);
        bx += bw + gap;
    }
}

}  // namespace fsim
