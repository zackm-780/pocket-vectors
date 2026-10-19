#include "core/Ini.h"

#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fsim {

std::string toLower(std::string_view s) {
    std::string out(s);
    for (char& c : out) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return out;
}

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

std::vector<std::string> splitTrim(std::string_view s, char delim) {
    std::vector<std::string> out;
    size_t start = 0;
    while (true) {
        const size_t pos = s.find(delim, start);
        out.emplace_back(trim(s.substr(start, pos == std::string_view::npos ? pos : pos - start)));
        if (pos == std::string_view::npos) break;
        start = pos + 1;
    }
    return out;
}

bool Ini::loadFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    loadText(ss.str());
    return true;
}

void Ini::loadText(std::string_view text) {
    sections_.clear();
    sections_.push_back({"", {}});
    size_t cur = 0;
    size_t pos = 0;
    while (pos <= text.size()) {
        size_t eol = text.find('\n', pos);
        if (eol == std::string_view::npos) eol = text.size();
        std::string_view line = trim(text.substr(pos, eol - pos));
        pos = eol + 1;
        if (line.empty() || line.front() == ';' || line.front() == '#') continue;

        if (line.front() == '[') {
            const size_t close = line.find(']');
            const std::string name = toLower(trim(
                line.substr(1, close == std::string_view::npos ? line.size() - 1 : close - 1)));
            cur = sections_.size();
            for (size_t i = 0; i < sections_.size(); ++i) {
                if (sections_[i].name == name) {
                    cur = i;
                    break;
                }
            }
            if (cur == sections_.size()) sections_.push_back({name, {}});
            continue;
        }

        const size_t eq = line.find('=');
        if (eq == std::string_view::npos) continue;
        const std::string key = toLower(trim(line.substr(0, eq)));
        std::string_view value = line.substr(eq + 1);
        sections_[cur].entries.emplace_back(key, std::string(trim(value)));
    }
}

const Ini::Section* Ini::findSection(std::string_view section) const {
    const std::string name = toLower(section);
    for (const Section& s : sections_)
        if (s.name == name) return &s;
    return nullptr;
}

const std::string* Ini::find(std::string_view section, std::string_view key) const {
    const Section* s = findSection(section);
    if (!s) return nullptr;
    const std::string k = toLower(key);
    for (auto it = s->entries.rbegin(); it != s->entries.rend(); ++it)
        if (it->first == k) return &it->second;
    return nullptr;
}

bool Ini::has(std::string_view section, std::string_view key) const {
    return find(section, key) != nullptr;
}

bool Ini::hasSection(std::string_view section) const {
    return findSection(section) != nullptr;
}

std::string Ini::getString(std::string_view section, std::string_view key,
                           const std::string& def) const {
    const std::string* v = find(section, key);
    return v ? *v : def;
}

double Ini::getDouble(std::string_view section, std::string_view key, double def) const {
    const std::string* v = find(section, key);
    if (!v || v->empty()) return def;
    char* end = nullptr;
    const double d = std::strtod(v->c_str(), &end);
    return end == v->c_str() ? def : d;
}

float Ini::getFloat(std::string_view section, std::string_view key, float def) const {
    return static_cast<float>(getDouble(section, key, static_cast<double>(def)));
}

int Ini::getInt(std::string_view section, std::string_view key, int def) const {
    const std::string* v = find(section, key);
    if (!v || v->empty()) return def;
    char* end = nullptr;
    const long n = std::strtol(v->c_str(), &end, 10);
    return end == v->c_str() ? def : static_cast<int>(n);
}

bool Ini::getBool(std::string_view section, std::string_view key, bool def) const {
    const std::string* v = find(section, key);
    if (!v || v->empty()) return def;
    const std::string s = toLower(*v);
    if (s == "1" || s == "true" || s == "yes" || s == "on") return true;
    if (s == "0" || s == "false" || s == "no" || s == "off") return false;
    return def;
}

const std::vector<Ini::Entry>& Ini::entries(std::string_view section) const {
    static const std::vector<Entry> kEmpty;
    const Section* s = findSection(section);
    return s ? s->entries : kEmpty;
}

}  // namespace fsim
