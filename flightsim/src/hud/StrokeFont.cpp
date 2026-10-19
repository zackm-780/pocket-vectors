#include "hud/StrokeFont.h"

#include <cstdlib>
#include <cstring>
#include <utility>
#include <vector>

namespace fsim {
namespace {

// Glyph source. Strokes are separated by ';', points by spaces as "x,y" (grid units, see the
// header). A trailing token "z" closes the stroke, "f" closes and fills it.
struct Source {
    std::uint32_t cp;
    const char* spec;
};

// clang-format off
const Source kSources[] = {
    {' ', ""},
    {'A', "0,8 0,2 2,0 4,2 4,8;0,5 4,5"},
    {'B', "0,8 0,0 3,0 4,1 4,3 3,4 0,4;3,4 4,5 4,7 3,8 0,8"},
    {'C', "4,1 3,0 1,0 0,1 0,7 1,8 3,8 4,7"},
    {'D', "0,0 0,8 2,8 4,6 4,2 2,0 z"},
    {'E', "4,0 0,0 0,8 4,8;0,4 3,4"},
    {'F', "4,0 0,0 0,8;0,4 3,4"},
    {'G', "4,1 3,0 1,0 0,1 0,7 1,8 3,8 4,7 4,4 2,4"},
    {'H', "0,0 0,8;4,0 4,8;0,4 4,4"},
    {'I', "1,0 3,0;2,0 2,8;1,8 3,8"},
    {'J', "4,0 4,7 3,8 1,8 0,7"},
    {'K', "0,0 0,8;4,0 0,4.5 4,8"},
    {'L', "0,0 0,8 4,8"},
    {'M', "0,8 0,0 2,4 4,0 4,8"},
    {'N', "0,8 0,0 4,8 4,0"},
    {'O', "1,0 3,0 4,1 4,7 3,8 1,8 0,7 0,1 z"},
    {'P', "0,8 0,0 3,0 4,1 4,3 3,4 0,4"},
    {'Q', "1,0 3,0 4,1 4,7 3,8 1,8 0,7 0,1 z;2.2,5.5 4.2,8.5"},
    {'R', "0,8 0,0 3,0 4,1 4,3 3,4 0,4;2,4 4,8"},
    {'S', "4,1 3,0 1,0 0,1 0,3 1,4 3,4 4,5 4,7 3,8 1,8 0,7"},
    {'T', "0,0 4,0;2,0 2,8"},
    {'U', "0,0 0,7 1,8 3,8 4,7 4,0"},
    {'V', "0,0 2,8 4,0"},
    {'W', "0,0 1,8 2,4 3,8 4,0"},
    {'X', "0,0 4,8;4,0 0,8"},
    {'Y', "0,0 2,4 4,0;2,4 2,8"},
    {'Z', "0,0 4,0 0,8 4,8"},
    // Digits. Zero is narrower than O (cut corners are larger) and carries a slash.
    {'0', "1,0 3,0 4,1.5 4,6.5 3,8 1,8 0,6.5 0,1.5 z;1,6.2 3,1.8"},
    {'1', "0.8,1.5 2,0 2,8;0.8,8 3.2,8"},
    {'2', "0,1 1,0 3,0 4,1 4,3 0,8 4,8"},
    {'3', "0,1 1,0 3,0 4,1 4,3 3,4 1.5,4;3,4 4,5 4,7 3,8 1,8 0,7"},
    {'4', "3,8 3,0 0,5.5 4,5.5"},
    {'5', "4,0 0,0 0,3.8 3,3.8 4,4.8 4,7 3,8 1,8 0,7"},
    {'6', "4,1 3,0 1,0 0,1 0,7 1,8 3,8 4,7 4,5 3,4 0,4"},
    {'7', "0,0 4,0 1.5,8"},
    {'8', "1,0 3,0 4,1 4,3 3,4 1,4 0,3 0,1 z;1,4 0,5 0,7 1,8 3,8 4,7 4,5 3,4"},
    {'9', "0,7 1,8 3,8 4,7 4,1 3,0 1,0 0,1 0,3 1,4 4,4"},
    // Punctuation.
    {'.', "2,7.6 2,8"},
    {',', "2,7.2 2,8.6 1,10"},
    {':', "2,2 2,2.4;2,5.6 2,6"},
    {'/', "0,8 4,0"},
    {'-', "0.5,4 3.5,4"},
    {'+', "2,1.5 2,6.5;0,4 4,4"},
    {'%', "0,8 4,0;0.4,0 1.6,0 1.6,2.6 0.4,2.6 z;2.4,5.4 3.6,5.4 3.6,8 2.4,8 z"},
    {'<', "3.5,0.5 0.5,4 3.5,7.5"},
    {'>', "0.5,0.5 3.5,4 0.5,7.5"},
    {'[', "3,0 1,0 1,8 3,8"},
    {']', "1,0 3,0 3,8 1,8"},
    {'(', "3,0 1.6,2 1.6,6 3,8"},
    {')', "1,0 2.4,2 2.4,6 1,8"},
    {'*', "2,1 2,5;0.2,2 3.8,4;0.2,4 3.8,2"},
    {'#', "1.4,0 0.6,8;3.4,0 2.6,8;0,2.7 4,2.7;0,5.3 4,5.3"},
    {'\'', "2,0 2,2.5"},
    {'"', "1,0 1,2.5;3,0 3,2.5"},
    {'=', "0,2.8 4,2.8;0,5.2 4,5.2"},
    {'_', "0,8 4,8"},
    {'!', "2,0 2,5.2;2,7.6 2,8"},
    {'?', "0,1 1,0 3,0 4,1 4,3 2,4.5 2,5.5;2,7.6 2,8"},
    {'|', "2,0 2,8"},
    // Symbols.
    {kCpDegree, "1,0 3,0 3.6,0.7 3.6,2.3 3,3 1,3 0.4,2.3 0.4,0.7 z"},
    {kCpArrowUp, "2,8 2,0;0.4,2.2 2,0 3.6,2.2"},
    {kCpArrowDown, "2,0 2,8;0.4,5.8 2,8 3.6,5.8"},
    {kCpArrowLeft, "4,4 0,4;1.8,2.2 0,4 1.8,5.8"},
    {kCpArrowRight, "0,4 4,4;2.2,2.2 4,4 2.2,5.8"},
    {kCpTriangleUp, "0,8 4,8 2,0 f"},
    {kCpTriangleDown, "0,0 4,0 2,8 f"},
    {kCpDiamond, "2,0 4,4 2,8 0,4 f"},
};
// clang-format on

const char kBoxSpec[] = "0,0 4,0 4,8 0,8 z";

struct Table {
    std::vector<GlyphPoint> points;
    std::vector<GlyphStroke> strokes;
    Glyph ascii[128];
    std::vector<std::pair<std::uint32_t, Glyph>> extra;
    Glyph box;

    // Parses one spec, appending to the pools; returns {firstStroke, strokeCount}.
    std::pair<size_t, int> parse(const char* spec) {
        const size_t firstStroke = strokes.size();
        const char* p = spec;
        while (*p) {
            const size_t firstPoint = points.size();
            bool closed = false, filled = false;
            while (*p && *p != ';') {
                if (*p == ' ') {
                    ++p;
                } else if (*p == 'z') {
                    closed = true;
                    ++p;
                } else if (*p == 'f') {
                    closed = filled = true;
                    ++p;
                } else {
                    char* end = nullptr;
                    const float x = std::strtof(p, &end);
                    p = end + 1;  // skip the comma
                    const float y = std::strtof(p, &end);
                    p = end;
                    points.push_back({x, y});
                }
            }
            if (*p == ';') ++p;
            GlyphStroke s;
            s.count = static_cast<int>(points.size() - firstPoint);
            s.closed = closed;
            s.filled = filled;
            // `points` pointer is patched after all parsing (the pool may reallocate); the
            // first-point index is stored in the pointer slot temporarily.
            s.points = reinterpret_cast<const GlyphPoint*>(firstPoint);
            strokes.push_back(s);
        }
        return {firstStroke, static_cast<int>(strokes.size() - firstStroke)};
    }

    Table() {
        points.reserve(1500);
        strokes.reserve(256);
        struct Pending {
            std::uint32_t cp;
            size_t first;
            int count;
        };
        std::vector<Pending> pending;
        for (const Source& src : kSources) {
            const auto r = parse(src.spec);
            pending.push_back({src.cp, r.first, r.second});
        }
        const auto boxR = parse(kBoxSpec);
        for (GlyphStroke& s : strokes)
            s.points = points.data() + reinterpret_cast<std::uintptr_t>(s.points);
        for (const Pending& pd : pending) {
            Glyph g;
            g.strokes = pd.count ? strokes.data() + pd.first : nullptr;
            g.strokeCount = pd.count;
            if (pd.cp < 128)
                ascii[pd.cp] = g;
            else
                extra.emplace_back(pd.cp, g);
        }
        box.strokes = strokes.data() + boxR.first;
        box.strokeCount = boxR.second;
        box.known = false;
        // Unlisted ASCII code points are unknown.
        std::vector<bool> listed(128, false);
        for (const Pending& pd : pending)
            if (pd.cp < 128) listed[pd.cp] = true;
        for (std::uint32_t c = 0; c < 128; ++c)
            if (!listed[c]) ascii[c] = box;
    }
};

const Table& table() {
    static const Table t;
    return t;
}

}  // namespace

float StrokeFont::capHeightPx(TextSize s) {
    switch (s) {
    case TextSize::S: return 9.0f;
    case TextSize::M: return 12.0f;
    case TextSize::L: return 16.0f;
    }
    return 12.0f;
}

float StrokeFont::strokeWidthPx(TextSize s) {
    switch (s) {
    case TextSize::S: return 1.0f;
    case TextSize::M: return 1.5f;
    case TextSize::L: return 2.0f;
    }
    return 2.0f;
}

std::uint32_t StrokeFont::decodeUtf8(const char*& p) {
    const unsigned char c = static_cast<unsigned char>(*p);
    int extra = 0;
    std::uint32_t cp = 0xFFFD;
    if (c < 0x80) {
        ++p;
        return c;
    } else if ((c & 0xE0) == 0xC0) {
        extra = 1;
        cp = c & 0x1F;
    } else if ((c & 0xF0) == 0xE0) {
        extra = 2;
        cp = c & 0x0F;
    } else if ((c & 0xF8) == 0xF0) {
        extra = 3;
        cp = c & 0x07;
    } else {
        ++p;
        return 0xFFFD;
    }
    ++p;
    for (int i = 0; i < extra; ++i) {
        const unsigned char n = static_cast<unsigned char>(*p);
        if ((n & 0xC0) != 0x80) return 0xFFFD;  // truncated: leave p on the offending byte
        cp = (cp << 6) | (n & 0x3F);
        ++p;
    }
    return cp;
}

int StrokeFont::nextCells(const char*& p, std::uint32_t cells[3]) {
    if (!*p) return 0;
    const std::uint32_t cp = decodeUtf8(p);
    if (cp == kCpCopyright) {
        cells[0] = '(';
        cells[1] = 'C';
        cells[2] = ')';
        return 3;
    }
    cells[0] = cp;
    return 1;
}

int StrokeFont::cellCount(const char* utf8) {
    int n = 0;
    const char* p = utf8;
    while (*p) n += decodeUtf8(p) == kCpCopyright ? 3 : 1;
    return n;
}

float StrokeFont::originX(float anchorX, const char* utf8, TextSize s, HAlign h) {
    const float w = measure(utf8, s);
    if (h == HAlign::Center) return anchorX - w * 0.5f;
    if (h == HAlign::Right) return anchorX - w;
    return anchorX;
}

float StrokeFont::capTopY(float anchorY, TextSize s, VAlign v) {
    const float cap = capHeightPx(s);
    if (v == VAlign::Middle) return anchorY - cap * 0.5f;
    if (v == VAlign::Baseline) return anchorY - cap;
    return anchorY;
}

const Glyph& StrokeFont::glyph(std::uint32_t cp) {
    const Table& t = table();
    if (cp >= 'a' && cp <= 'z') cp -= 'a' - 'A';
    if (cp == kCpCopyright) return t.ascii['C'];  // callers expand it via nextCells()
    if (cp < 128) return t.ascii[cp];
    for (const auto& e : t.extra)
        if (e.first == cp) return e.second;
    return t.box;
}

}  // namespace fsim
