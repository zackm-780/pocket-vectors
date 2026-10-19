// System information gathering for the probe: libc, kernel, CPU, RAM, environment, device nodes.
// Everything here is plain POSIX/procfs and works with no SDL or GL.
#pragma once

#include <string>

#include "Report.h"

namespace probe {

struct MemInfo {
    long totalKb = -1;
    long freeKb = -1;
    long availableKb = -1;
    long cmaTotalKb = -1;
    long cmaFreeKb = -1;
};

// Reads a whole text file (up to maxBytes). Returns an empty string if it cannot be read.
std::string readTextFile(const std::string& path, size_t maxBytes = 1 << 16);
// First line of a file without the trailing newline, or empty.
std::string readFirstLine(const std::string& path);

MemInfo readMemInfo();
// Resident set size and peak of this process in KB, or -1.
long readSelfRssKb();
long readSelfPeakRssKb();

// Directory containing the running executable (from /proc/self/exe), or "." if unknown.
std::string executableDir();

// One-line CPU frequency / governor / temperature summary, for bracketing benchmarks.
std::string cpuStateSummary();

// Sections: SYSTEM (glibc, kernel, os), CPU, MEMORY, PORTMASTER / ENVIRONMENT, DEVICE NODES.
// Sets the facts for glibc, kernel, controlfolder, ports folder and free RAM.
void reportSystemInfo(Report& report);
// Lists libraries mapped into the process that matter for graphics/audio/input (call after GL init).
void reportLoadedLibraries(Report& report);

}  // namespace probe
