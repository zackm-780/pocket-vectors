#include "hud/Renderer2DGlyphSheet.h"

#include <cmath>
#include <cstdio>

namespace fsim {
namespace {

constexpr float kPiF = 3.14159265f;

const char* const kGlyphLines[] = {
    "ABCDEFGHIJKLM",
    "NOPQRSTUVWXYZ",
    "0123456789",
    ".,:/-+" FSIM_SYM_DEGREE "%<>[]()*#'\"=_!?|",
    FSIM_SYM_ARROW_UP FSIM_SYM_ARROW_DOWN FSIM_SYM_ARROW_LEFT FSIM_SYM_ARROW_RIGHT
        FSIM_SYM_TRIANGLE_UP FSIM_SYM_TRIANGLE_DOWN FSIM_SYM_DIAMOND
    " " FSIM_SYM_COPYRIGHT " ~",  // '~' is intentionally unknown: shows the fallback box
};

const char kHudSample[] = "IAS 132  ALT 3200  HDG 271" FSIM_SYM_DEGREE
                          "  FLAP 15  GEAR DN  TRIM " FSIM_SYM_ARROW_UP " +0.12";

void glyphBlock(Renderer2D& r, float x, float& y, TextSize size, Color c) {
    const float cap = StrokeFont::capHeightPx(size);
    for (const char* line : kGlyphLines) {
        y += cap;
        r.text(x, y, line, size, c);
        y += cap * 0.55f;
    }
}

void shapesDemo(Renderer2D& r, float x0, float y0) {
    const Color g = colors::kHudGreen;

    // Line widths 1..4 with square vs butt caps.
    for (int i = 0; i < 4; ++i) {
        r.setLineWidth(1.0f + static_cast<float>(i));
        r.line(x0, y0 + 6.0f + 9.0f * static_cast<float>(i), x0 + 70,
               y0 + 6.0f + 9.0f * static_cast<float>(i), g);
    }
    r.setLineWidth(2.0f);
    r.line(x0 + 80, y0 + 4, x0 + 120, y0 + 4, g, LineCap::Square);
    r.dashedLine(x0 + 80, y0 + 14, x0 + 150, y0 + 14, 6, 4, g);

    // Polyline with joins (acute, right, obtuse).
    const Pt2 zig[] = {{x0 + 160, y0 + 34}, {x0 + 176, y0 + 4},  {x0 + 188, y0 + 34},
                       {x0 + 212, y0 + 34}, {x0 + 226, y0 + 14}, {x0 + 250, y0 + 14}};
    r.polyline(zig, 6, g);

    // Arcs and circles.
    r.arc(x0 + 30, y0 + 90, 26, kPiF, 2.0f * kPiF, g);  // upper half circle
    r.arc(x0 + 30, y0 + 90, 18, kPiF * 1.25f, kPiF * 1.75f, colors::kAmber);
    r.circle(x0 + 90, y0 + 72, 14, g);
    r.fillCircle(x0 + 90, y0 + 72, 4, g, FillMode::FrontOutlined);

    // Rects and triangles (bugs and pointers).
    r.fillRect(x0 + 120, y0 + 58, 40, 24, g.withAlpha(0.35f));
    r.strokeRect(x0 + 120, y0 + 58, 40, 24, g);
    r.fillTriangle({x0 + 180, y0 + 82}, {x0 + 196, y0 + 82}, {x0 + 188, y0 + 62}, g,
                   FillMode::FrontOutlined);
    r.strokeTriangle({x0 + 206, y0 + 62}, {x0 + 222, y0 + 62}, {x0 + 214, y0 + 82}, colors::kAmber);
    r.fillTriangle({x0 + 236, y0 + 82}, {x0 + 252, y0 + 82}, {x0 + 244, y0 + 62}, colors::kRed,
                   FillMode::FrontOutlined);

    // Pitch-ladder style: rotated, dashed lines inside a clip rect.
    const float cx = x0 + 90, cy = y0 + 148;
    r.strokeRect(cx - 80, cy - 40, 160, 80, g.withAlpha(0.5f));
    r.pushClip(cx - 80, cy - 40, 160, 80);
    r.push();
    r.translate(cx, cy);
    r.rotate(-0.30f);
    for (int k = -3; k <= 3; ++k) {
        const float yy = static_cast<float>(k) * 22.0f;
        if (k == 0) {
            r.line(-120, 0, 120, 0, g);
        } else if (k > 0) {
            r.dashedLine(-30, yy, 30, yy, 6, 4, g);
            r.text(-34, yy, "10", TextSize::S, g, HAlign::Right, VAlign::Middle);
        } else {
            r.line(-30, yy, 30, yy, g);
            r.text(34, yy, "10", TextSize::S, g, HAlign::Left, VAlign::Middle);
        }
    }
    r.pop();
    r.popClip();
}

void outlineComparison(Renderer2D& r, float x, float y, float w, float h, Color bg,
                       const char* tag) {
    r.fillRect(x, y, w, h, bg);
    const Color g = colors::kHudGreen;
    float yy = y + 14;
    for (int pass = 0; pass < 2; ++pass) {
        r.setOutline(pass == 0);
        r.text(x + 4, yy, pass == 0 ? "OUTLINE" : "NO OUTLINE", TextSize::S, g);
        r.text(x + 90, yy, tag, TextSize::S, g);
        yy += 14;
        r.text(x + 4, yy, "IAS 132 ALT 3200", TextSize::M, g);
        yy += 18;
        r.text(x + 4, yy, "HDG 271" FSIM_SYM_DEGREE, TextSize::L, g);
        r.line(x + 110, yy - 6, x + w - 6, yy - 6, g);
        yy += 24;
    }
    r.setOutline(true);
}

}  // namespace

void drawGlyphSheet(Renderer2D& r) {
    const Color g = colors::kHudGreen;
    const float left = r.leftX() + 4.0f;

    r.text(left, 14, "GLYPH SHEET H1  " FSIM_SYM_COPYRIGHT " COPERNICUS ESA", TextSize::M, g);
    char dims[64];
    std::snprintf(dims, sizeof dims, "CANVAS %.0fX480", r.virtualWidth());
    r.text(r.rightX() - 4.0f, 14, dims, TextSize::M, g, HAlign::Right);

    float y = 18;
    glyphBlock(r, left, y, TextSize::S, g);
    y += 4;
    glyphBlock(r, left, y, TextSize::M, g);
    y += 4;
    glyphBlock(r, left, y, TextSize::L, g);

    shapesDemo(r, r.rightX() - 262.0f, 26.0f);

    // HUD-like strings at S and M (L only fits a short one).
    float sy = 366.0f;
    r.text(left, sy + 9, kHudSample, TextSize::S, g);
    sy += 16;
    r.text(left, sy + 12, kHudSample, TextSize::M, g);
    sy += 20;
    r.text(left, sy + 16, "IAS 132  ALT 3200", TextSize::L, g);

    // Outline comparison: bright then dark background (bottom right, beside the samples).
    const float panelW = 160.0f, panelH = 126.0f;
    const float px = r.rightX() - 2 * panelW - 8;
    const float py = 230.0f;
    outlineComparison(r, px, py, panelW, panelH, Color(0.92f, 0.95f, 1.0f), "BRIGHT");
    outlineComparison(r, px + panelW + 4, py, panelW, panelH, Color(0.04f, 0.05f, 0.06f), "DARK");
}

}  // namespace fsim
