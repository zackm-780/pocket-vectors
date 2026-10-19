#include "Report.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <unistd.h>

#include "Common.h"

namespace probe {

namespace {
constexpr size_t kMaxScreenLines = 200;
constexpr const char* kNotMeasured = "NOT MEASURED (phase skipped, failed or not reached)";

std::string wallClockNow() {
    const std::time_t t = std::time(nullptr);
    char buf[64] = {};
    std::tm tmv{};
    gmtime_r(&t, &tmv);
    std::strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S UTC", &tmv);
    return buf;
}
}  // namespace

Report::Report(std::string path) : path_(std::move(path)), startS_(nowS()), startedAt_(wallClockNow()) {
    const char* const names[] = {rows::kGlibc,       rows::kKernel,      rows::kControlFolder,
                                 rows::kPortsFolder, rows::kSdl,         rows::kGl,
                                 rows::kDepthMsaa,   rows::kMaxTex,      rows::kTriggers,
                                 rows::kStickClicks, rows::kFaceButtons, rows::kMenuKey,
                                 rows::kFreeRam,     rows::kGpu,         rows::kCpu,
                                 rows::kAudio};
    for (const char* name : names) {
        facts_.emplace_back(name, kNotMeasured);
    }
}

void Report::section(const std::string& title) {
    sections_.push_back(Section{title, {}});
    std::printf("\n== %s ==\n", title.c_str());
    std::fflush(stdout);
    screenLines_.push_back("== " + title + " ==");
    while (screenLines_.size() > kMaxScreenLines) {
        screenLines_.pop_front();
    }
    save();
}

void Report::quietLine(const std::string& text) {
    if (sections_.empty()) {
        sections_.push_back(Section{"PREAMBLE", {}});
    }
    sections_.back().lines.push_back(text);
    std::printf("%s\n", text.c_str());
    std::fflush(stdout);
}

void Report::line(const std::string& text) {
    quietLine(text);
    screenLines_.push_back(text);
    while (screenLines_.size() > kMaxScreenLines) {
        screenLines_.pop_front();
    }
}

void Report::kv(const std::string& key, const std::string& value) {
    line(key + ": " + value);
}

void Report::fact(const std::string& row, const std::string& value) {
    for (auto& entry : facts_) {
        if (entry.first == row) {
            entry.second = value;
            return;
        }
    }
    facts_.emplace_back(row, value);
}

std::string Report::factValue(const std::string& row) const {
    for (const auto& entry : facts_) {
        if (entry.first == row) {
            return entry.second;
        }
    }
    return std::string();
}

void Report::setStatus(const std::string& status) {
    status_ = status;
}

std::string Report::render() const {
    std::string out;
    out += "FLIGHT SIM PROBE REPORT\n";
    out += "=======================\n";
    out += strf("generated:   %s (probe ran %.1f s)\n", wallClockNow().c_str(), nowS() - startS_);
    out += strf("started:     %s\n", startedAt_.c_str());
    out += strf("status:      %s\n", status_.c_str());
    out += "\nKEY FACTS (copy into docs/DEVICE.md section 2)\n";
    out += "-----------------------------------------------\n";
    for (const auto& entry : facts_) {
        out += strf("* %s:\n    %s\n", entry.first.c_str(), entry.second.c_str());
    }
    for (const auto& s : sections_) {
        out += "\n== " + s.title + " ==\n";
        for (const auto& l : s.lines) {
            out += l;
            out += '\n';
        }
    }
    return out;
}

bool Report::save() {
    if (path_.empty()) {
        return false;
    }
    const std::string tmp = path_ + ".tmp";
    FILE* f = std::fopen(tmp.c_str(), "wb");
    if (f == nullptr) {
        if (!warnedUnwritable_) {
            std::fprintf(stderr, "probe: cannot write %s: %s\n", tmp.c_str(), std::strerror(errno));
            warnedUnwritable_ = true;
        }
        return false;
    }
    const std::string text = render();
    const bool wrote = std::fwrite(text.data(), 1, text.size(), f) == text.size();
    std::fflush(f);
    fsync(fileno(f));
    std::fclose(f);
    if (!wrote || std::rename(tmp.c_str(), path_.c_str()) != 0) {
        if (!warnedUnwritable_) {
            std::fprintf(stderr, "probe: cannot save %s: %s\n", path_.c_str(), std::strerror(errno));
            warnedUnwritable_ = true;
        }
        return false;
    }
    return true;
}

}  // namespace probe
