#include "app/GpuClock.h"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fsim {
namespace {

std::string readSmallFile(const std::string& path) {
    std::ifstream in(path);
    if (!in) return {};
    std::ostringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

std::string trimmed(const std::string& s) {
    size_t a = 0, b = s.size();
    while (a < b && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    while (b > a && std::isspace(static_cast<unsigned char>(s[b - 1]))) --b;
    return s.substr(a, b - a);
}

std::string lower(std::string s) {
    for (char& c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

int hzTextToMhz(const std::string& text) {
    const std::string t = trimmed(text);
    if (t.empty()) return -1;
    char* end = nullptr;
    const double hz = std::strtod(t.c_str(), &end);
    if (end == t.c_str() || hz < 0) return -1;
    return static_cast<int>(hz / 1e6 + 0.5);
}

std::vector<int> frequencyListMhz(const std::string& text) {
    std::vector<int> out;
    std::istringstream ss(text);
    std::string tok;
    while (ss >> tok) {
        const int mhz = hzTextToMhz(tok);
        if (mhz >= 0) out.push_back(mhz);
    }
    return out;
}

int pickGpuEntry(const std::vector<std::string>& names) {
    for (size_t i = 0; i < names.size(); ++i) {
        const std::string n = lower(names[i]);
        if (n.find("gpu") != std::string::npos || n.find("mali") != std::string::npos)
            return static_cast<int>(i);
    }
    return names.size() == 1 ? 0 : -1;
}

bool GpuClock::open(const std::string& root) {
    dir_.clear();
    name_.clear();
    std::error_code ec;
    std::vector<std::string> names;
    for (const auto& de : std::filesystem::directory_iterator(root, ec))
        names.push_back(de.path().filename().string());
    if (ec) return false;
    std::sort(names.begin(), names.end());  // a stable choice whatever the listing order
    const int pick = pickGpuEntry(names);
    if (pick < 0) return false;
    name_ = names[static_cast<size_t>(pick)];
    dir_ = (std::filesystem::path(root) / name_).string();
    return true;
}

int GpuClock::curMhz() const {
    return available() ? hzTextToMhz(readSmallFile(dir_ + "/cur_freq")) : -1;
}

int GpuClock::minMhz() const {
    return available() ? hzTextToMhz(readSmallFile(dir_ + "/min_freq")) : -1;
}

int GpuClock::maxMhz() const {
    return available() ? hzTextToMhz(readSmallFile(dir_ + "/max_freq")) : -1;
}

std::vector<int> GpuClock::availableMhz() const {
    if (!available()) return {};
    return frequencyListMhz(readSmallFile(dir_ + "/available_frequencies"));
}

std::string GpuClock::governor() const {
    return available() ? trimmed(readSmallFile(dir_ + "/governor")) : std::string();
}

std::string GpuClock::windowTag() const {
    if (!available()) return {};
    char buf[48];
    const int cur = curMhz(), mx = maxMhz();
    if (mx > 0)
        std::snprintf(buf, sizeof buf, " | gpu %d/%d MHz", cur, mx);
    else
        std::snprintf(buf, sizeof buf, " | gpu %d MHz", cur);
    return buf;
}

std::string GpuClock::describe() const {
    if (!available()) return {};
    std::string gov = governor();
    if (gov.empty()) gov = "?";
    char buf[160];
    std::snprintf(buf, sizeof buf, "gpu clock: %s %s cur %d MHz (min %d max %d; available",
                  name_.c_str(), gov.c_str(), curMhz(), minMhz(), maxMhz());
    std::string out = buf;
    const std::vector<int> avail = availableMhz();
    if (avail.empty()) out += " ?";
    for (int f : avail) out += " " + std::to_string(f);
    out += ")";
    return out;
}

}  // namespace fsim
