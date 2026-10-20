#include "ui/OnScreenKeyboard.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>

namespace fsim {
namespace {

using Key = OnScreenKeyboard::Key;
using KeyKind = OnScreenKeyboard::KeyKind;

void addChars(std::vector<Key>& row, const char* chars, int firstCol) {
    int col = firstCol;
    for (const char* p = chars; *p; ++p, ++col) {
        Key k;
        k.label = std::string(1, *p);
        k.kind = KeyKind::Char;
        k.ch = *p;
        k.col = col;
        row.push_back(std::move(k));
    }
}

void addFunction(std::vector<Key>& row, const char* label, KeyKind kind, int col, int span) {
    Key k;
    k.label = label;
    k.kind = kind;
    k.col = col;
    k.span = span;
    row.push_back(std::move(k));
}

std::vector<std::vector<Key>> buildLetters() {
    std::vector<std::vector<Key>> rows(4);
    addChars(rows[0], "ABCDEFGHI", 0);
    addChars(rows[1], "JKLMNOPQR", 0);
    addChars(rows[2], "STUVWXYZ", 0);
    addFunction(rows[3], "123", KeyKind::ToggleLayer, 0, 2);
    addFunction(rows[3], "SPACE", KeyKind::Space, 2, 3);
    addFunction(rows[3], "DEL", KeyKind::Backspace, 5, 2);
    addFunction(rows[3], "OK", KeyKind::Ok, 7, 2);
    return rows;
}

std::vector<std::vector<Key>> buildDigits() {
    std::vector<std::vector<Key>> rows(3);
    addChars(rows[0], "12345", 2);
    addChars(rows[1], "67890", 2);
    addFunction(rows[2], "ABC", KeyKind::ToggleLayer, 0, 2);
    addFunction(rows[2], "SPACE", KeyKind::Space, 2, 3);
    addFunction(rows[2], "DEL", KeyKind::Backspace, 5, 2);
    addFunction(rows[2], "OK", KeyKind::Ok, 7, 2);
    return rows;
}

const std::vector<std::vector<Key>>& lettersGrid() {
    static const std::vector<std::vector<Key>> g = buildLetters();
    return g;
}
const std::vector<std::vector<Key>>& digitsGrid() {
    static const std::vector<std::vector<Key>> g = buildDigits();
    return g;
}

int centerHalfUnits(const Key& k) { return 2 * k.col + k.span; }

}  // namespace

OnScreenKeyboard::OnScreenKeyboard(size_t maxLength) : maxLength_(maxLength) {
    placeCursor(0, 0);
}

const std::vector<std::vector<Key>>& OnScreenKeyboard::rows() const {
    return layer_ == Layer::Letters ? lettersGrid() : digitsGrid();
}

void OnScreenKeyboard::setText(const std::string& text) {
    text_.clear();
    typeText(text);
}

void OnScreenKeyboard::typeText(const std::string& text) {
    for (char c : text) type(c);
}

void OnScreenKeyboard::placeCursor(int row, int preferredHalfUnits) {
    const auto& g = rows();
    cursorRow_ = std::clamp(row, 0, static_cast<int>(g.size()) - 1);
    const auto& r = g[static_cast<size_t>(cursorRow_)];
    int best = 0, bestDist = 1 << 30;
    for (int i = 0; i < static_cast<int>(r.size()); ++i) {
        const int d = std::abs(centerHalfUnits(r[static_cast<size_t>(i)]) - preferredHalfUnits);
        if (d < bestDist) {
            bestDist = d;
            best = i;
        }
    }
    cursorKey_ = best;
}

void OnScreenKeyboard::moveVertical(int dir) {
    const int n = rowCount();
    const int next = ((cursorRow_ + dir) % n + n) % n;
    placeCursor(next, preferredCol2_);
}

void OnScreenKeyboard::moveHorizontal(int dir) {
    const auto& r = row(cursorRow_);
    const int n = static_cast<int>(r.size());
    cursorKey_ = ((cursorKey_ + dir) % n + n) % n;
    preferredCol2_ = centerHalfUnits(r[static_cast<size_t>(cursorKey_)]);
}

OnScreenKeyboard::Event OnScreenKeyboard::type(char c) {
    if (std::isalpha(static_cast<unsigned char>(c)))
        c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    else if (!std::isdigit(static_cast<unsigned char>(c)) && c != ' ')
        return Event::None;
    if (text_.size() >= maxLength_) return Event::None;
    if (c == ' ' && (text_.empty() || text_.back() == ' ')) return Event::None;  // no leading/double
    text_.push_back(c);
    return Event::Changed;
}

OnScreenKeyboard::Event OnScreenKeyboard::backspace() {
    if (text_.empty()) return Event::None;
    text_.pop_back();
    return Event::Changed;
}

OnScreenKeyboard::Event OnScreenKeyboard::press(const Key& key) {
    switch (key.kind) {
    case KeyKind::Char: return type(key.ch);
    case KeyKind::Space: return type(' ');
    case KeyKind::Backspace: return backspace();
    case KeyKind::Ok: return Event::Done;
    case KeyKind::ToggleLayer: {
        layer_ = layer_ == Layer::Letters ? Layer::Digits : Layer::Letters;
        placeCursor(cursorRow_, preferredCol2_);
        return Event::None;
    }
    }
    return Event::None;
}

OnScreenKeyboard::Event OnScreenKeyboard::handle(const MenuInput& in) {
    if (in.up && !in.down)
        moveVertical(-1);
    else if (in.down && !in.up)
        moveVertical(+1);
    if (in.left && !in.right)
        moveHorizontal(-1);
    else if (in.right && !in.left)
        moveHorizontal(+1);

    Event ev = Event::None;
    if (in.accept) ev = press(cursorKeyRef());
    if (in.x && ev == Event::None) ev = type(' ');
    if (in.y) {
        layer_ = layer_ == Layer::Letters ? Layer::Digits : Layer::Letters;
        placeCursor(cursorRow_, preferredCol2_);
    }
    if (in.back && ev == Event::None) {
        if (text_.empty())
            ev = Event::Cancel;
        else
            ev = backspace();
    }
    if (in.start && ev == Event::None) ev = Event::Done;
    return ev;
}

}  // namespace fsim
