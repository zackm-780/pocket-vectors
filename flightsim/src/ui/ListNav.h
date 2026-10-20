#pragma once
// Selection and scrolling of a vertical list (WP H5). D-pad up/down move the selection with wrap
// around, skipping disabled rows; the visible window follows the selection.

#include <vector>

#include "ui/MenuInput.h"

namespace fsim {

class ListNav {
public:
    ListNav() = default;
    ListNav(int count, int visibleRows) : count_(count), visible_(visibleRows) {}

    // Replaces the number of rows; keeps the selection when still valid, else clamps it.
    void setCount(int count);
    void setVisibleRows(int rows);
    // Rows that cannot be selected (headers, unavailable items). Cleared by an empty vector.
    void setDisabled(std::vector<char> disabled);

    // Up / down (wrapping). Returns true when the selection moved.
    bool handle(const MenuInput& in);
    // Moves by `dir` (+1 / -1), wrapping and skipping disabled rows.
    bool move(int dir);
    // Sets the selection (clamped); scrolls to show it.
    void select(int index);

    int count() const { return count_; }
    int selected() const { return selected_; }
    int top() const { return top_; }
    int visibleRows() const { return visible_; }
    bool empty() const { return count_ <= 0; }
    bool canScrollUp() const { return top_ > 0; }
    bool canScrollDown() const { return top_ + visible_ < count_; }
    bool isDisabled(int i) const {
        return i >= 0 && i < static_cast<int>(disabled_.size()) &&
               disabled_[static_cast<size_t>(i)] != 0;
    }

private:
    void clampWindow();

    int count_ = 0, visible_ = 8, selected_ = 0, top_ = 0;
    std::vector<char> disabled_;
};

}  // namespace fsim
