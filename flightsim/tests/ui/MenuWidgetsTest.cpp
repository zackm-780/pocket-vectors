#include <doctest/doctest.h>

#include "ui/ListNav.h"
#include "ui/MenuInput.h"
#include "ui/OnScreenKeyboard.h"

using namespace fsim;

namespace {
MenuInput press(bool MenuInput::*field) {
    MenuInput m;
    m.*field = true;
    return m;
}
}  // namespace

TEST_CASE("ui: menu input maps the semantic actions") {
    ActionState a;
    a.pressed[static_cast<int>(Action::MenuUp)] = true;
    a.pressed[static_cast<int>(Action::MenuAccept)] = true;
    a.pressed[static_cast<int>(Action::SpeedbrakeCycle)] = true;  // X
    a.pressed[static_cast<int>(Action::Pause)] = true;            // Start
    a.down[static_cast<int>(Action::MenuDown)] = true;            // held only: no edge
    const MenuInput m = menuInputFromActions(a);
    CHECK(m.up);
    CHECK(m.accept);
    CHECK(m.x);
    CHECK(m.start);
    CHECK_FALSE(m.down);
    CHECK_FALSE(m.back);
    CHECK_FALSE(m.y);
    CHECK(m.any());
    CHECK_FALSE(menuInputFromActions(ActionState{}).any());
}

TEST_CASE("ui: list navigation wraps and scrolls") {
    ListNav nav(10, 4);
    CHECK(nav.selected() == 0);
    CHECK(nav.top() == 0);
    CHECK(nav.handle(press(&MenuInput::up)));  // wraps to the last row
    CHECK(nav.selected() == 9);
    CHECK(nav.top() == 6);
    CHECK(nav.canScrollUp());
    CHECK_FALSE(nav.canScrollDown());
    CHECK(nav.handle(press(&MenuInput::down)));  // wraps back to the first
    CHECK(nav.selected() == 0);
    CHECK(nav.top() == 0);
    for (int i = 0; i < 5; ++i) nav.handle(press(&MenuInput::down));
    CHECK(nav.selected() == 5);
    CHECK(nav.top() == 2);  // the window follows the selection
    nav.select(100);
    CHECK(nav.selected() == 9);
    nav.setCount(3);  // list shrank
    CHECK(nav.selected() == 2);
    CHECK(nav.top() == 0);
}

TEST_CASE("ui: list navigation skips disabled rows and handles empty lists") {
    ListNav nav(5, 5);
    nav.setDisabled({1, 0, 1, 0, 0});  // rows 0 and 2 disabled
    CHECK(nav.selected() == 1);        // moved off the disabled first row
    nav.move(+1);
    CHECK(nav.selected() == 3);
    nav.move(-1);
    CHECK(nav.selected() == 1);
    nav.move(-1);  // wraps past the disabled row 0 to row 4
    CHECK(nav.selected() == 4);

    ListNav empty(0, 5);
    CHECK_FALSE(empty.handle(press(&MenuInput::down)));
    CHECK(empty.empty());
    CHECK(empty.selected() == 0);

    ListNav both(4, 4);  // up and down together: no movement
    MenuInput m;
    m.up = m.down = true;
    CHECK_FALSE(both.handle(m));
}

TEST_CASE("ui: keyboard types letters, digits, space and deletes") {
    OnScreenKeyboard kb;
    CHECK(kb.text().empty());
    CHECK(kb.cursorKeyRef().label == "A");
    // A types 'A'; right + A types 'B'.
    CHECK(kb.handle(press(&MenuInput::accept)) == OnScreenKeyboard::Event::Changed);
    kb.handle(press(&MenuInput::right));
    CHECK(kb.handle(press(&MenuInput::accept)) == OnScreenKeyboard::Event::Changed);
    CHECK(kb.text() == "AB");
    CHECK(kb.handle(press(&MenuInput::x)) == OnScreenKeyboard::Event::Changed);  // X = space
    CHECK(kb.text() == "AB ");
    CHECK(kb.handle(press(&MenuInput::x)) == OnScreenKeyboard::Event::None);  // no double space
    CHECK(kb.handle(press(&MenuInput::back)) == OnScreenKeyboard::Event::Changed);  // B = delete
    CHECK(kb.text() == "AB");

    kb.handle(press(&MenuInput::y));  // Y = digit layer
    CHECK(kb.layer() == OnScreenKeyboard::Layer::Digits);
    kb.handle(press(&MenuInput::accept));
    CHECK(kb.text().size() == 3);
    CHECK(std::isdigit(static_cast<unsigned char>(kb.text().back())));
    kb.handle(press(&MenuInput::y));
    CHECK(kb.layer() == OnScreenKeyboard::Layer::Letters);
}

TEST_CASE("ui: keyboard cancel, done and limits") {
    OnScreenKeyboard kb(3);
    CHECK(kb.handle(press(&MenuInput::back)) == OnScreenKeyboard::Event::Cancel);  // empty text
    kb.typeText("otha");  // lower case is upper-cased, capped at 3 characters
    CHECK(kb.text() == "OTH");
    CHECK(kb.handle(press(&MenuInput::accept)) == OnScreenKeyboard::Event::None);  // full
    CHECK(kb.handle(press(&MenuInput::start)) == OnScreenKeyboard::Event::Done);
    kb.setText("a-b!c");  // unsupported characters are skipped
    CHECK(kb.text() == "ABC");
    kb.clear();
    CHECK(kb.text().empty());
}

TEST_CASE("ui: keyboard cursor moves vertically keeping its column and wraps") {
    OnScreenKeyboard kb;
    // Go to the function row (row 3) with Up from the top: wraps.
    kb.handle(press(&MenuInput::up));
    CHECK(kb.cursorRow() == 3);
    CHECK(kb.cursorKeyRef().label == "123");  // column 0 maps to the widest left key
    kb.handle(press(&MenuInput::right));
    CHECK(kb.cursorKeyRef().label == "SPACE");
    kb.handle(press(&MenuInput::right));
    CHECK(kb.cursorKeyRef().label == "DEL");
    kb.handle(press(&MenuInput::right));
    CHECK(kb.cursorKeyRef().label == "OK");
    kb.handle(press(&MenuInput::right));  // wraps in the row
    CHECK(kb.cursorKeyRef().label == "123");

    // OK key finishes; the layer key toggles.
    kb.handle(press(&MenuInput::left));
    CHECK(kb.cursorKeyRef().label == "OK");
    CHECK(kb.handle(press(&MenuInput::accept)) == OnScreenKeyboard::Event::Done);
    kb.handle(press(&MenuInput::right));  // 123
    kb.handle(press(&MenuInput::accept));
    CHECK(kb.layer() == OnScreenKeyboard::Layer::Digits);
    CHECK(kb.cursorKeyRef().label == "ABC");

    // Up from the function row, then back down: the preferred column survives a short row.
    OnScreenKeyboard k2;
    for (int i = 0; i < 8; ++i) k2.handle(press(&MenuInput::right));  // column 8 ("I")
    CHECK(k2.cursorKeyRef().label == "I");
    k2.handle(press(&MenuInput::down));
    k2.handle(press(&MenuInput::down));  // row 2 has only 8 keys: lands on the nearest, "Z"
    CHECK(k2.cursorKeyRef().label == "Z");
    k2.handle(press(&MenuInput::up));
    CHECK(k2.cursorKeyRef().label == "R");  // back in the 9-key row, preferred column kept near 8
}
