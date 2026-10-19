#pragma once
// Logger: stderr plus an optional log file (conf/log.txt). The one allowed global (CONVENTIONS 1).
// Not for hot paths (it formats strings and takes a lock).

#include <string>

namespace fsim {

enum class LogLevel { Debug, Info, Warn, Error };

namespace log {

// Minimum level written (default Info).
void setLevel(LogLevel level);
// Also append to this file (truncates it first). Returns false if it cannot be opened.
bool openFile(const std::string& path);
void closeFile();
// Suppress stderr output (tests). The file sink, if open, keeps working.
void setStderrEnabled(bool enabled);

#if defined(__GNUC__) || defined(__clang__)
#define FSIM_PRINTF_LIKE(fmtIdx, argIdx) __attribute__((format(printf, fmtIdx, argIdx)))
#else
#define FSIM_PRINTF_LIKE(fmtIdx, argIdx)
#endif
void write(LogLevel level, const char* fmt, ...) FSIM_PRINTF_LIKE(2, 3);

}  // namespace log
}  // namespace fsim

#define LOG_DEBUG(...) ::fsim::log::write(::fsim::LogLevel::Debug, __VA_ARGS__)
#define LOG_INFO(...) ::fsim::log::write(::fsim::LogLevel::Info, __VA_ARGS__)
#define LOG_WARN(...) ::fsim::log::write(::fsim::LogLevel::Warn, __VA_ARGS__)
#define LOG_ERROR(...) ::fsim::log::write(::fsim::LogLevel::Error, __VA_ARGS__)
