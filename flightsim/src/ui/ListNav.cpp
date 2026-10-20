#include "ui/ListNav.h"

#include <algorithm>

namespace fsim {

void ListNav::clampWindow() {
    if (count_ <= 0) {
        selected_ = top_ = 0;
        return;
    }
    selected_ = std::clamp(selected_, 0, count_ - 1);
    const int rows = std::max(1, visible_);
    if (selected_ < top_) top_ = selected_;
    if (selected_ >= top_ + rows) top_ = selected_ - rows + 1;
    top_ = std::clamp(top_, 0, std::max(0, count_ - rows));
}

void ListNav::setCount(int count) {
    count_ = std::max(0, count);
    clampWindow();
}

void ListNav::setVisibleRows(int rows) {
    visible_ = std::max(1, rows);
    clampWindow();
}

void ListNav::setDisabled(std::vector<char> disabled) {
    disabled_ = std::move(disabled);
    if (count_ > 0 && isDisabled(selected_)) {  // land on a selectable row
        for (int i = 0; i < count_; ++i)
            if (!isDisabled(i)) {
                selected_ = i;
                break;
            }
        clampWindow();
    }
}

void ListNav::select(int index) {
    selected_ = index;
    clampWindow();
}

bool ListNav::move(int dir) {
    if (count_ <= 0 || dir == 0) return false;
    const int before = selected_;
    int i = selected_;
    for (int n = 0; n < count_; ++n) {
        i = ((i + dir) % count_ + count_) % count_;
        if (!isDisabled(i)) {
            selected_ = i;
            break;
        }
    }
    clampWindow();
    return selected_ != before;
}

bool ListNav::handle(const MenuInput& in) {
    if (in.up && !in.down) return move(-1);
    if (in.down && !in.up) return move(+1);
    return false;
}

}  // namespace fsim
