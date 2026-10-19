// Stroke font metrics, alignment, measurement and glyph coverage (no GL context needed).
#include <doctest/doctest.h>

#include <cstring>
#include <initializer_list>
#include <ios>

#include "hud/StrokeFont.h"

using namespace fsim;

TEST_CASE("hud: font sizes and advance") {
    CHECK(StrokeFont::capHeightPx(TextSize::S) == doctest::Approx(9.0f));
    CHECK(StrokeFont::capHeightPx(TextSize::M) == doctest::Approx(12.0f));
    CHECK(StrokeFont::capHeightPx(TextSize::L) == doctest::Approx(16.0f));
    CHECK(StrokeFont::advancePx(TextSize::S) == doctest::Approx(6.75f));
    CHECK(StrokeFont::advancePx(TextSize::M) == doctest::Approx(9.0f));
    CHECK(StrokeFont::advancePx(TextSize::L) == doctest::Approx(12.0f));
    CHECK(StrokeFont::unitPx(TextSize::L) == doctest::Approx(2.0f));
}

TEST_CASE("hud: font measure is monospaced and counts UTF-8 characters") {
    CHECK(StrokeFont::measure("", TextSize::M) == 0.0f);
    CHECK(StrokeFont::measure("IAS 132", TextSize::M) == doctest::Approx(7 * 9.0f));
    CHECK(StrokeFont::measure("WWWWWWW", TextSize::M) ==
          StrokeFont::measure("1111111", TextSize::M));
    // Multi-byte symbols are one cell each.
    CHECK(StrokeFont::cellCount("HDG 271" FSIM_SYM_DEGREE) == 8);
    CHECK(StrokeFont::cellCount("TRIM " FSIM_SYM_ARROW_UP " +0.12") == 12);
    // The copyright sign expands to "(C)".
    CHECK(StrokeFont::cellCount(FSIM_SYM_COPYRIGHT " ESA") == 7);
    CHECK(StrokeFont::measure(FSIM_SYM_COPYRIGHT, TextSize::L) == doctest::Approx(3 * 12.0f));
}

TEST_CASE("hud: font alignment") {
    const char* s = "ALT 3200";  // 8 cells at M = 72 px
    CHECK(StrokeFont::originX(100, s, TextSize::M, HAlign::Left) == doctest::Approx(100));
    CHECK(StrokeFont::originX(100, s, TextSize::M, HAlign::Center) == doctest::Approx(64));
    CHECK(StrokeFont::originX(100, s, TextSize::M, HAlign::Right) == doctest::Approx(28));
    // Vertical: anchor is the top, the middle or the baseline of the cap height (12 px at M).
    CHECK(StrokeFont::capTopY(50, TextSize::M, VAlign::Top) == doctest::Approx(50));
    CHECK(StrokeFont::capTopY(50, TextSize::M, VAlign::Middle) == doctest::Approx(44));
    CHECK(StrokeFont::capTopY(50, TextSize::M, VAlign::Baseline) == doctest::Approx(38));
}

TEST_CASE("hud: UTF-8 decoding") {
    const char* p = "A" FSIM_SYM_DEGREE FSIM_SYM_ARROW_UP FSIM_SYM_DIAMOND;
    CHECK(StrokeFont::decodeUtf8(p) == 'A');
    CHECK(StrokeFont::decodeUtf8(p) == kCpDegree);
    CHECK(StrokeFont::decodeUtf8(p) == kCpArrowUp);
    CHECK(StrokeFont::decodeUtf8(p) == kCpDiamond);
    CHECK(*p == 0);
    const char bad[] = "\xFF"
                       "B";
    const char* q = bad;
    CHECK(StrokeFont::decodeUtf8(q) == 0xFFFD);
    CHECK(StrokeFont::decodeUtf8(q) == 'B');
}

TEST_CASE("hud: every required glyph exists and has strokes") {
    std::uint32_t required[256];
    int n = 0;
    for (char c = 'A'; c <= 'Z'; ++c) required[n++] = static_cast<unsigned char>(c);
    for (char c = '0'; c <= '9'; ++c) required[n++] = static_cast<unsigned char>(c);
    for (const char* p = ".,:/-+%<>[]()*#'\""; *p; ++p)
        required[n++] = static_cast<unsigned char>(*p);
    for (std::uint32_t cp : {kCpDegree, kCpArrowUp, kCpArrowDown, kCpArrowLeft, kCpArrowRight,
                             kCpTriangleUp, kCpTriangleDown, kCpDiamond})
        required[n++] = cp;
    for (int i = 0; i < n; ++i) {
        const Glyph& g = StrokeFont::glyph(required[i]);
        INFO("code point U+" << std::hex << required[i]);
        CHECK(g.known);
        CHECK(g.strokeCount > 0);
        for (int s = 0; s < g.strokeCount; ++s) {
            CHECK(g.strokes[s].count >= 2);
            for (int k = 0; k < g.strokes[s].count; ++k) {
                const GlyphPoint& pt = g.strokes[s].points[k];
                CHECK(pt.x >= -0.5f);
                CHECK(pt.x <= kGlyphGridWidth + 0.5f);
                CHECK(pt.y >= 0.0f);
                CHECK(pt.y <= 10.5f);
            }
        }
    }
}

TEST_CASE("hud: space is known without strokes, lowercase maps to uppercase") {
    const Glyph& sp = StrokeFont::glyph(' ');
    CHECK(sp.known);
    CHECK(sp.strokeCount == 0);
    CHECK(&StrokeFont::glyph('a') == &StrokeFont::glyph('A'));
    // The copyright sign is expanded to "(C)" through nextCells().
    const char* p = FSIM_SYM_COPYRIGHT "X";
    std::uint32_t cells[3];
    CHECK(StrokeFont::nextCells(p, cells) == 3);
    CHECK(cells[0] == '(');
    CHECK(cells[1] == 'C');
    CHECK(cells[2] == ')');
    CHECK(StrokeFont::nextCells(p, cells) == 1);
    CHECK(StrokeFont::nextCells(p, cells) == 0);
    CHECK(StrokeFont::hasGlyph(kCpCopyright));
}

TEST_CASE("hud: unknown characters draw a box") {
    const Glyph& g = StrokeFont::glyph('~');
    CHECK_FALSE(g.known);
    REQUIRE(g.strokeCount == 1);
    CHECK(g.strokes[0].closed);
    CHECK(g.strokes[0].count == 4);
    CHECK_FALSE(StrokeFont::hasGlyph(0x4E2D));  // a CJK character
    CHECK(StrokeFont::glyph(0x4E2D).strokes == g.strokes);
}

TEST_CASE("hud: zero and letter O are distinguishable") {
    const Glyph& zero = StrokeFont::glyph('0');
    const Glyph& oh = StrokeFont::glyph('O');
    bool same = zero.strokeCount == oh.strokeCount;
    if (same) {
        for (int s = 0; s < zero.strokeCount && same; ++s) {
            same = zero.strokes[s].count == oh.strokes[s].count;
            for (int k = 0; same && k < zero.strokes[s].count; ++k)
                same = zero.strokes[s].points[k].x == oh.strokes[s].points[k].x &&
                       zero.strokes[s].points[k].y == oh.strokes[s].points[k].y;
        }
    }
    CHECK_FALSE(same);
    // Zero carries an extra slash stroke.
    CHECK(zero.strokeCount > oh.strokeCount);
}

TEST_CASE("hud: filled symbols are closed filled polygons") {
    for (std::uint32_t cp : {kCpTriangleUp, kCpTriangleDown, kCpDiamond}) {
        const Glyph& g = StrokeFont::glyph(cp);
        REQUIRE(g.strokeCount == 1);
        CHECK(g.strokes[0].filled);
        CHECK(g.strokes[0].closed);
    }
}
