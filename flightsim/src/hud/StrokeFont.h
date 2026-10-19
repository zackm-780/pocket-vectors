#pragma once
// Hand-authored vector stroke font (PLAN D13, HUD.md section 2). Pure data and metrics, no GL:
// Renderer2D::text() turns the strokes into thick-line quads.
//
// Glyph grid: x 0..4 (cell width), y 0..8 with y = 0 at the cap line and y = 8 on the baseline
// (y grows downward like the HUD canvas). A few glyphs go below the baseline (',').
// One grid unit = capHeight / 8, so the glyph body is 0.5 x capHeight wide and the monospaced
// advance is 0.75 x capHeight.
//
// Text is UTF-8. Lowercase letters draw as uppercase. Supported: A-Z 0-9 space and
//   . , : / - + % < > [ ] ( ) * # ' " = _ ! ? |  plus the symbols below.
// The copyright sign U+00A9 is drawn as the three cells "(C)". Unknown characters draw a box.

#include <cstdint>

namespace fsim {

enum class TextSize { S, M, L };  // cap height 9 / 12 / 16 virtual px
enum class HAlign { Left, Center, Right };
enum class VAlign { Top, Middle, Baseline };

// Named symbols as UTF-8 string literals, for concatenation: "TRIM " FSIM_SYM_ARROW_UP " +0.12".
// clang-format off
#define FSIM_SYM_DEGREE "\xC2\xB0"              // U+00B0
#define FSIM_SYM_COPYRIGHT "\xC2\xA9"           // U+00A9 (drawn as "(C)")
#define FSIM_SYM_ARROW_UP "\xE2\x86\x91"        // U+2191
#define FSIM_SYM_ARROW_DOWN "\xE2\x86\x93"      // U+2193
#define FSIM_SYM_ARROW_LEFT "\xE2\x86\x90"      // U+2190
#define FSIM_SYM_ARROW_RIGHT "\xE2\x86\x92"     // U+2192
#define FSIM_SYM_TRIANGLE_UP "\xE2\x96\xB2"     // U+25B2 filled
#define FSIM_SYM_TRIANGLE_DOWN "\xE2\x96\xBC"   // U+25BC filled
#define FSIM_SYM_DIAMOND "\xE2\x97\x86"         // U+25C6 filled
// clang-format on

// The same symbols as code points.
constexpr std::uint32_t kCpDegree = 0x00B0, kCpCopyright = 0x00A9, kCpArrowUp = 0x2191,
                        kCpArrowDown = 0x2193, kCpArrowLeft = 0x2190, kCpArrowRight = 0x2192,
                        kCpTriangleUp = 0x25B2, kCpTriangleDown = 0x25BC, kCpDiamond = 0x25C6;

constexpr float kGlyphGridWidth = 4.0f;   // grid units
constexpr float kGlyphGridHeight = 8.0f;  // cap height in grid units

struct GlyphPoint {
    float x, y;  // grid units
};

struct GlyphStroke {
    const GlyphPoint* points = nullptr;
    int count = 0;
    bool closed = false;  // join the last point back to the first
    bool filled = false;  // convex polygon, also filled (bugs and pointers)
};

struct Glyph {
    const GlyphStroke* strokes = nullptr;
    int strokeCount = 0;
    bool known = true;  // false for the fallback box
};

class StrokeFont {
public:
    static float capHeightPx(TextSize s);  // 9 / 12 / 16
    static float advancePx(TextSize s) { return capHeightPx(s) * 0.75f; }
    static float unitPx(TextSize s) { return capHeightPx(s) / kGlyphGridHeight; }
    // Default stroke width for text at this size (virtual px).
    static float strokeWidthPx(TextSize s);

    // Number of character cells the text occupies ("(C)" counts 3).
    static int cellCount(const char* utf8);
    // Layout width: cellCount x advance (the trailing inter-glyph gap is included, so strings
    // line up on a fixed grid and right alignment is stable while digits change).
    static float measure(const char* utf8, TextSize s) { return cellCount(utf8) * advancePx(s); }
    // Left edge of the first cell for an anchor point and alignment.
    static float originX(float anchorX, const char* utf8, TextSize s, HAlign h);
    // Top of the cap line for an anchor y and alignment (Baseline: anchor is the baseline).
    static float capTopY(float anchorY, TextSize s, VAlign v);

    // Glyph for a code point. Space has strokeCount 0 and known == true. Unknown code points
    // return the box glyph with known == false. Lowercase maps to uppercase.
    static const Glyph& glyph(std::uint32_t codePoint);
    static bool hasGlyph(std::uint32_t codePoint) { return glyph(codePoint).known; }

    // Reads the next character and writes the code points of the cells it occupies (1, or 3
    // for the copyright sign: '(' 'C' ')'). Returns the cell count, 0 at the end of the string.
    static int nextCells(const char*& p, std::uint32_t cells[3]);

    // Decodes one UTF-8 sequence, advancing `p`. Invalid bytes decode as U+FFFD (box).
    static std::uint32_t decodeUtf8(const char*& p);
};

}  // namespace fsim
