#include "core/Log.h"

#include <cstdarg>
#include <cstdio>
#include <mutex>

namespace fsim::log {
namespace {

struct Sink {
    std::mutex mutex;
    LogLevel level = LogLevel::Info;
    std::FILE* file = nullptr;
    bool toStderr = true;
};

Sink& sink() {
    static Sink s;  // the single allowed global (function-local to dodge init-order issues)
    return s;
}

const char* levelTag(LogLevel level) {
    switch (level) {
    case LogLevel::Debug: return "DEBUG";
    case LogLevel::Info: return "INFO ";
    case LogLevel::Warn: return "WARN ";
    case LogLevel::Error: return "ERROR";
    }
    return "?";
}

}  // namespace

void setLevel(LogLevel level) {
    std::lock_guard<std::mutex> lock(sink().mutex);
    sink().level = level;
}

bool openFile(const std::string& path) {
    std::lock_guard<std::mutex> lock(sink().mutex);
    if (sink().file) std::fclose(sink().file);
    sink().file = std::fopen(path.c_str(), "w");
    return sink().file != nullptr;
}

void closeFile() {
    std::lock_guard<std::mutex> lock(sink().mutex);
    if (sink().file) std::fclose(sink().file);
    sink().file = nullptr;
}

void setStderrEnabled(bool enabled) {
    std::lock_guard<std::mutex> lock(sink().mutex);
    sink().toStderr = enabled;
}

void write(LogLevel level, const char* fmt, ...) {
    char msg[1024];
    va_list args;
    va_start(args, fmt);
    std::vsnprintf(msg, sizeof msg, fmt, args);
    va_end(args);

    Sink& s = sink();
    std::lock_guard<std::mutex> lock(s.mutex);
    if (level < s.level) return;
    if (s.toStderr) std::fprintf(stderr, "[%s] %s\n", levelTag(level), msg);
    if (s.file) {
        std::fprintf(s.file, "[%s] %s\n", levelTag(level), msg);
        std::fflush(s.file);
    }
}

}  // namespace fsim::log
