#pragma once
// Minimal INI reader used for settings, aircraft profiles and scenarios.
//
// Format: "[section]" headers, "key = value" lines. A line whose first non-whitespace character is
// ';' or '#' is a comment. There are NO inline comments: everything after '=' is the value, kept
// verbatim as UTF-8 (only surrounding whitespace is trimmed), so values may contain ';', '#' or
// '(c)' signs. Section and key names are case-insensitive; values keep their case. Keys before
// the first header live in the section "". Duplicate keys are kept in order (entries()), lookups
// return the last one.

#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace fsim {

class Ini {
public:
    using Entry = std::pair<std::string, std::string>;  // key (lower-case), value

    // Parse a file / text. Both return false on failure (file missing); parse never throws.
    bool loadFile(const std::string& path);
    void loadText(std::string_view text);

    bool has(std::string_view section, std::string_view key) const;
    bool hasSection(std::string_view section) const;

    std::string getString(std::string_view section, std::string_view key,
                          const std::string& def = "") const;
    int getInt(std::string_view section, std::string_view key, int def = 0) const;
    double getDouble(std::string_view section, std::string_view key, double def = 0) const;
    float getFloat(std::string_view section, std::string_view key, float def = 0) const;
    bool getBool(std::string_view section, std::string_view key, bool def = false) const;

    // All entries of a section, in file order (empty if the section does not exist).
    const std::vector<Entry>& entries(std::string_view section) const;

private:
    struct Section {
        std::string name;  // lower-case
        std::vector<Entry> entries;
    };
    const std::string* find(std::string_view section, std::string_view key) const;
    const Section* findSection(std::string_view section) const;

    std::vector<Section> sections_;
};

// Helpers shared by the parsers.
std::string toLower(std::string_view s);
std::string_view trim(std::string_view s);
// Splits on a delimiter, trimming each part; empty parts are kept.
std::vector<std::string> splitTrim(std::string_view s, char delim);

}  // namespace fsim
