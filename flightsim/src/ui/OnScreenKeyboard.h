#pragma once
// On-screen keyboard (WP H5, CONTROLS section 3): a character grid for ident / name search.
// D-pad moves the cursor, A types the key under it, B deletes (or cancels when the text is empty),
// X types a space, Y switches between the letter and digit layers, Start is "done". The grid also
// has SPACE, DEL, OK and layer keys that A presses. Pure state machine, no drawing.

#include <string>
#include <vector>

#include "ui/MenuInput.h"

namespace fsim {

class OnScreenKeyboard {
public:
    enum class Layer { Letters, Digits };
    enum class KeyKind { Char, Space, Backspace, Ok, ToggleLayer };
    enum class Event { None, Changed, Done, Cancel };

    static constexpr int kColumns = 9;  // grid width in unit columns

    struct Key {
        std::string label;
        KeyKind kind = KeyKind::Char;
        char ch = 0;
        int col = 0, span = 1;  // position in unit columns
    };

    explicit OnScreenKeyboard(size_t maxLength = 24);

    // Feeds one frame of input. Cancel = B pressed with an empty text.
    Event handle(const MenuInput& in);

    const std::string& text() const { return text_; }
    void setText(const std::string& text);
    void clear() { text_.clear(); }
    // Types characters as if pressed (tests, canned screenshots); unknown characters are skipped.
    void typeText(const std::string& text);
    size_t maxLength() const { return maxLength_; }

    Layer layer() const { return layer_; }
    int rowCount() const { return static_cast<int>(rows().size()); }
    const std::vector<Key>& row(int r) const { return rows()[static_cast<size_t>(r)]; }
    int cursorRow() const { return cursorRow_; }
    int cursorKey() const { return cursorKey_; }
    const Key& cursorKeyRef() const { return row(cursorRow_)[static_cast<size_t>(cursorKey_)]; }

private:
    const std::vector<std::vector<Key>>& rows() const;
    void placeCursor(int row, int preferredCol);
    void moveVertical(int dir);
    void moveHorizontal(int dir);
    Event press(const Key& key);
    Event type(char c);
    Event backspace();

    std::string text_;
    size_t maxLength_;
    Layer layer_ = Layer::Letters;
    int cursorRow_ = 0, cursorKey_ = 0;
    int preferredCol2_ = 0;  // cursor column in half units (center of the key), for up/down
};

}  // namespace fsim
