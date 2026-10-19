#include "SysInfo.h"

#include <dirent.h>
#include <sys/statvfs.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <set>
#include <sstream>
#include <thread>
#include <vector>

#if defined(__GLIBC__)
#include <gnu/libc-version.h>
#endif

#include "Common.h"

extern char** environ;

namespace probe {

namespace {

std::string trim(const std::string& s) {
    size_t b = 0;
    size_t e = s.size();
    while (b < e && (s[b] == ' ' || s[b] == '\t' || s[b] == '\n' || s[b] == '\r')) {
        ++b;
    }
    while (e > b && (s[e - 1] == ' ' || s[e - 1] == '\t' || s[e - 1] == '\n' || s[e - 1] == '\r')) {
        --e;
    }
    return s.substr(b, e - b);
}

std::vector<std::string> splitLines(const std::string& text) {
    std::vector<std::string> lines;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        lines.push_back(line);
    }
    return lines;
}

// Lists the entries of a directory (names only, sorted). Empty if it cannot be opened.
std::vector<std::string> listDir(const std::string& path) {
    std::vector<std::string> names;
    DIR* d = opendir(path.c_str());
    if (d == nullptr) {
        return names;
    }
    while (const dirent* e = readdir(d)) {
        if (std::strcmp(e->d_name, ".") != 0 && std::strcmp(e->d_name, "..") != 0) {
            names.emplace_back(e->d_name);
        }
    }
    closedir(d);
    std::sort(names.begin(), names.end());
    return names;
}

bool pathExists(const std::string& path) {
    return access(path.c_str(), F_OK) == 0;
}

std::string joinNames(const std::vector<std::string>& names, size_t maxCount = 64) {
    std::string out;
    for (size_t i = 0; i < names.size() && i < maxCount; ++i) {
        if (i != 0) {
            out += ' ';
        }
        out += names[i];
    }
    if (names.size() > maxCount) {
        out += strf(" ... (+%zu)", names.size() - maxCount);
    }
    return out;
}

long parseKb(const std::string& meminfo, const char* key) {
    const std::string needle = std::string(key) + ":";
    for (const auto& line : splitLines(meminfo)) {
        if (line.compare(0, needle.size(), needle) == 0) {
            return std::strtol(line.c_str() + needle.size(), nullptr, 10);
        }
    }
    return -1;
}

std::string truncateTo(const std::string& s, size_t n) {
    return s.size() <= n ? s : s.substr(0, n) + strf("... (%zu chars)", s.size());
}

bool startsWithAny(const std::string& s, const std::vector<std::string>& prefixes) {
    for (const auto& p : prefixes) {
        if (s.compare(0, p.size(), p) == 0) {
            return true;
        }
    }
    return false;
}

void reportCpu(Report& report) {
    report.section("CPU");
    const std::string cpuinfo = readTextFile("/proc/cpuinfo");
    int processors = 0;
    bool inFirstBlock = true;
    for (const auto& line : splitLines(cpuinfo)) {
        if (line.compare(0, 9, "processor") == 0) {
            ++processors;
            if (processors > 1) {
                inFirstBlock = false;
            }
        }
        if (inFirstBlock && !line.empty()) {
            report.line("cpuinfo: " + line);
        }
    }
    report.kv("logical CPUs (/proc/cpuinfo)", strf("%d", processors));
    report.kv("hardware_concurrency", strf("%u", std::thread::hardware_concurrency()));
    report.kv("compiled for",
#if defined(__aarch64__)
              "aarch64"
#elif defined(__x86_64__)
              "x86_64"
#else
              "other"
#endif
    );
    for (int cpu = 0; cpu < 8; ++cpu) {
        const std::string base = strf("/sys/devices/system/cpu/cpu%d/cpufreq/", cpu);
        if (!pathExists(base)) {
            continue;
        }
        report.line(strf("cpu%d cpufreq: cur=%s kHz min=%s max=%s governor=%s available=%s", cpu,
                         readFirstLine(base + "scaling_cur_freq").c_str(),
                         readFirstLine(base + "scaling_min_freq").c_str(),
                         readFirstLine(base + "scaling_max_freq").c_str(),
                         readFirstLine(base + "scaling_governor").c_str(),
                         readFirstLine(base + "scaling_available_governors").c_str()));
    }
    report.kv("CPU state now", cpuStateSummary());
}

void reportMemory(Report& report) {
    report.section("MEMORY");
    const std::string meminfo = readTextFile("/proc/meminfo");
    const MemInfo mem = readMemInfo();
    for (const char* key : {"MemTotal", "MemFree", "MemAvailable", "Buffers", "Cached", "SwapTotal",
                            "SwapFree", "CmaTotal", "CmaFree"}) {
        const long kb = parseKb(meminfo, key);
        if (kb >= 0) {
            report.line(strf("%-13s %8ld kB (%.1f MB)", key, kb, static_cast<double>(kb) / 1024.0));
        }
    }
    const long rssKb = readSelfRssKb();
    report.kv("probe process RSS at start", strf("%.1f MB", static_cast<double>(rssKb) / 1024.0));
    report.line("Free RAM here is measured while the PortMaster launcher and gptokeyb are already running,");
    report.line("i.e. what a port really gets.");
    std::string freeRam = strf("MemAvailable %.0f MB of %.0f MB total at probe start (MemFree %.0f MB)",
                               static_cast<double>(mem.availableKb) / 1024.0,
                               static_cast<double>(mem.totalKb) / 1024.0,
                               static_cast<double>(mem.freeKb) / 1024.0);
    if (mem.cmaTotalKb > 0) {
        freeRam += strf("; CMA %.0f MB total, %.0f MB free", static_cast<double>(mem.cmaTotalKb) / 1024.0,
                        static_cast<double>(mem.cmaFreeKb) / 1024.0);
    }
    report.fact(rows::kFreeRam, freeRam);
}

void reportEnvironment(Report& report) {
    report.section("PORTMASTER AND ENVIRONMENT");
    // The launch script exports these (see port "Flight Sim Probe.sh").
    const char* pmDirs[] = {"/opt/system/Tools/PortMaster", "/opt/tools/PortMaster", "/roms/ports/PortMaster"};
    for (const char* dir : pmDirs) {
        report.line(strf("PortMaster candidate %-34s %s", dir, pathExists(dir) ? "EXISTS" : "absent"));
    }
    const char* xdg = std::getenv("XDG_DATA_HOME");
    const std::string xdgPm = std::string(xdg != nullptr ? xdg : "") + "/PortMaster";
    if (xdg != nullptr) {
        report.line(strf("PortMaster candidate %-34s %s", xdgPm.c_str(), pathExists(xdgPm) ? "EXISTS" : "absent"));
    }
    const char* controlFolder = std::getenv("PM_CONTROLFOLDER");
    std::string controlFolderFact;
    if (controlFolder != nullptr && controlFolder[0] != '\0') {
        report.kv("PM_CONTROLFOLDER (launch script)", controlFolder);
        report.kv("  control.txt present", pathExists(std::string(controlFolder) + "/control.txt") ? "yes" : "no");
        report.kv("  gptokeyb present", pathExists(std::string(controlFolder) + "/gptokeyb") ? "yes" : "no");
        report.kv("  version", readFirstLine(std::string(controlFolder) + "/version"));
        controlFolderFact = controlFolder;
    } else {
        controlFolderFact = "unknown (not started through the launch script; PM_CONTROLFOLDER unset)";
    }
    report.fact(rows::kControlFolder, controlFolderFact);

    const std::string exeDir = executableDir();
    char cwd[1024] = {};
    if (getcwd(cwd, sizeof(cwd)) == nullptr) {
        cwd[0] = '?';
    }
    std::string portsFolder = exeDir;
    const size_t slash = portsFolder.find_last_of('/');
    if (slash != std::string::npos && slash > 0) {
        portsFolder = portsFolder.substr(0, slash);
    }
    report.kv("executable directory (port folder)", exeDir);
    report.kv("working directory", cwd);
    report.kv("parent of port folder (ports folder)", portsFolder);
    const char* directory = std::getenv("PM_DIRECTORY");
    report.fact(rows::kPortsFolder,
                strf("%s (port folder %s; PortMaster $directory=%s)", portsFolder.c_str(), exeDir.c_str(),
                     directory != nullptr ? directory : "unset"));
    struct statvfs vfs{};
    if (statvfs(exeDir.c_str(), &vfs) == 0) {
        report.kv("free space in port folder",
                  strf("%.0f MB", static_cast<double>(vfs.f_bavail) * static_cast<double>(vfs.f_frsize) / 1048576.0));
    }
    report.kv("writable port folder", access(exeDir.c_str(), W_OK) == 0 ? "yes" : "NO");

    report.line("-- environment (selected variables) --");
    const std::vector<std::string> prefixes = {
        "SDL_",   "XDG_",  "PM_",    "CFW_",   "DEVICE_", "DISPLAY", "WAYLAND", "LD_",    "LIBGL",
        "MESA",   "MALI",  "EGL",    "ANALOG", "ESUDO",   "param_",  "directory", "controlfolder",
        "GPTOKEYB", "HOME", "USER",  "PATH",   "FLIGHTSIM", "PROBE_", "SHELL", "LANG", "PULSE", "ALSA"};
    std::vector<std::string> lines;
    for (char** e = environ; e != nullptr && *e != nullptr; ++e) {
        const std::string entry = *e;
        const size_t eq = entry.find('=');
        if (eq == std::string::npos) {
            continue;
        }
        const std::string name = entry.substr(0, eq);
        if (!startsWithAny(name, prefixes)) {
            continue;
        }
        std::string value = entry.substr(eq + 1);
        std::replace(value.begin(), value.end(), '\n', '|');
        lines.push_back(name + "=" + truncateTo(value, 240));
    }
    std::sort(lines.begin(), lines.end());
    for (const auto& l : lines) {
        report.line("env " + l);
    }
}

void reportDeviceNodes(Report& report) {
    report.section("DEVICE NODES AND KERNEL INPUT/AUDIO/DISPLAY");
    report.kv("/dev/dri", joinNames(listDir("/dev/dri")));
    report.kv("/dev/input", joinNames(listDir("/dev/input")));
    report.kv("/dev/snd", joinNames(listDir("/dev/snd")));
    std::vector<std::string> misc;
    for (const auto& name : listDir("/dev")) {
        if (name.compare(0, 2, "fb") == 0 || name.compare(0, 4, "mali") == 0 || name.compare(0, 3, "ion") == 0 ||
            name.compare(0, 8, "dma_heap") == 0 || name.compare(0, 5, "cedar") == 0 || name.compare(0, 3, "tty") == 0) {
            if (name.compare(0, 3, "tty") == 0 && name != "tty0" && name != "tty1" && name != "tty") {
                continue;
            }
            misc.push_back(name);
        }
    }
    report.kv("/dev (fb/mali/ion/dma_heap/tty)", joinNames(misc));
    for (const char* fb : {"/sys/class/graphics/fb0"}) {
        if (pathExists(fb)) {
            report.line(strf("%s: name=%s virtual_size=%s bits_per_pixel=%s", fb,
                             readFirstLine(std::string(fb) + "/name").c_str(),
                             readFirstLine(std::string(fb) + "/virtual_size").c_str(),
                             readFirstLine(std::string(fb) + "/bits_per_pixel").c_str()));
        }
    }
    for (const auto& name : listDir("/sys/class/drm")) {
        if (name.find('-') == std::string::npos) {
            continue;
        }
        const std::string base = "/sys/class/drm/" + name + "/";
        const auto modes = splitLines(readTextFile(base + "modes", 4096));
        report.line(strf("drm %s: status=%s first modes=%s", name.c_str(), readFirstLine(base + "status").c_str(),
                         modes.empty() ? "-" : modes[0].c_str()));
    }
    report.line("-- /proc/bus/input/devices (every kernel input device; look for gamepad, keys, volume) --");
    for (const auto& l : splitLines(readTextFile("/proc/bus/input/devices", 16384))) {
        report.line("  " + l);
    }
    report.line("-- /proc/asound/cards --");
    for (const auto& l : splitLines(readTextFile("/proc/asound/cards", 4096))) {
        report.line("  " + l);
    }
    for (const auto& name : listDir("/sys/class/power_supply")) {
        const std::string base = "/sys/class/power_supply/" + name + "/";
        report.line(strf("power_supply %s: type=%s status=%s capacity=%s", name.c_str(),
                         readFirstLine(base + "type").c_str(), readFirstLine(base + "status").c_str(),
                         readFirstLine(base + "capacity").c_str()));
    }
}

}  // namespace

std::string readTextFile(const std::string& path, size_t maxBytes) {
    std::ifstream in(path, std::ios::binary);
    if (!in) {
        return std::string();
    }
    std::string data(maxBytes, '\0');
    in.read(&data[0], static_cast<std::streamsize>(maxBytes));
    data.resize(static_cast<size_t>(in.gcount()));
    return data;
}

std::string readFirstLine(const std::string& path) {
    std::ifstream in(path);
    std::string line;
    if (in && std::getline(in, line)) {
        return trim(line);
    }
    return std::string();
}

MemInfo readMemInfo() {
    const std::string text = readTextFile("/proc/meminfo");
    MemInfo m;
    m.totalKb = parseKb(text, "MemTotal");
    m.freeKb = parseKb(text, "MemFree");
    m.availableKb = parseKb(text, "MemAvailable");
    m.cmaTotalKb = parseKb(text, "CmaTotal");
    m.cmaFreeKb = parseKb(text, "CmaFree");
    return m;
}

long readSelfRssKb() {
    return parseKb(readTextFile("/proc/self/status"), "VmRSS");
}

long readSelfPeakRssKb() {
    return parseKb(readTextFile("/proc/self/status"), "VmHWM");
}

std::string executableDir() {
    char buf[4096] = {};
    const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
    if (n <= 0) {
        return ".";
    }
    std::string path(buf, static_cast<size_t>(n));
    const size_t slash = path.find_last_of('/');
    return slash == std::string::npos ? "." : path.substr(0, slash);
}

std::string cpuStateSummary() {
    std::string out;
    for (int cpu = 0; cpu < 8; ++cpu) {
        const std::string f = readFirstLine(strf("/sys/devices/system/cpu/cpu%d/cpufreq/scaling_cur_freq", cpu));
        if (f.empty()) {
            continue;
        }
        out += strf("cpu%d=%.0fMHz ", cpu, std::atof(f.c_str()) / 1000.0);
    }
    out += "gov=" + readFirstLine("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor") + " ";
    for (const auto& name : listDir("/sys/class/thermal")) {
        if (name.compare(0, 12, "thermal_zone") != 0) {
            continue;
        }
        const std::string base = "/sys/class/thermal/" + name + "/";
        const std::string t = readFirstLine(base + "temp");
        if (!t.empty()) {
            out += strf("%s(%s)=%.1fC ", name.c_str(), readFirstLine(base + "type").c_str(), std::atof(t.c_str()) / 1000.0);
        }
    }
    if (out.empty() || out == "gov= ") {
        return "(no cpufreq/thermal sysfs)";
    }
    return trim(out);
}

void reportSystemInfo(Report& report) {
    report.section("SYSTEM");
#if defined(__GLIBC__)
    const std::string glibc = gnu_get_libc_version();
    report.kv("glibc (gnu_get_libc_version)", glibc);
    report.kv("glibc release", gnu_get_libc_release());
    report.kv("glibc compiled against", strf("%d.%d", __GLIBC__, __GLIBC_MINOR__));
    report.fact(rows::kGlibc, glibc + " (runtime, gnu_get_libc_version)");
#else
    report.kv("libc", "not glibc");
    report.fact(rows::kGlibc, "not glibc");
#endif
    struct utsname u{};
    if (uname(&u) == 0) {
        report.kv("uname", strf("%s %s %s %s", u.sysname, u.release, u.version, u.machine));
        report.fact(rows::kKernel, strf("%s %s (%s)", u.sysname, u.release, u.machine));
    }
    report.kv("/proc/version", readFirstLine("/proc/version"));
    report.kv("device-tree model", readFirstLine("/proc/device-tree/model"));
    report.kv("device-tree compatible", readFirstLine("/proc/device-tree/compatible"));
    for (const auto& l : splitLines(readTextFile("/etc/os-release", 2048))) {
        report.line("os-release: " + l);
    }
    report.kv("uptime / loadavg", readFirstLine("/proc/uptime") + " / " + readFirstLine("/proc/loadavg"));
    report.kv("compiler",
#if defined(__clang__)
              "clang " __clang_version__
#else
              "gcc " __VERSION__
#endif
    );
    report.kv("built", std::string(__DATE__) + " " + __TIME__);
    reportCpu(report);
    reportMemory(report);
    reportEnvironment(report);
    reportDeviceNodes(report);
}

void reportLoadedLibraries(Report& report) {
    report.section("LOADED GRAPHICS / AUDIO / INPUT LIBRARIES (from /proc/self/maps)");
    const std::string maps = readTextFile("/proc/self/maps", 1 << 20);
    std::set<std::string> libs;
    const std::vector<std::string> interesting = {"libmali", "libGLES", "libEGL", "libGL", "libgbm", "libdrm", "libSDL2",
                                                  "libasound", "libpulse", "libwayland", "libX11", "libudev", "libJSBSim",
                                                  "libc.", "libc-", "libm.", "libm-", "libstdc++", "libgcc_s", "ld-linux",
                                                  "libpthread", "libdl", "librt", "libvulkan", "libinput", "libevdev",
                                                  "libmesa", "dri/", "libglapi", "libMali", "libpng", "libz."};
    for (const auto& line : splitLines(maps)) {
        const size_t slash = line.find('/');
        if (slash == std::string::npos) {
            continue;
        }
        const std::string path = line.substr(slash);
        for (const auto& key : interesting) {
            if (path.find(key) != std::string::npos) {
                libs.insert(path);
                break;
            }
        }
    }
    if (libs.empty()) {
        report.line("(none found; /proc/self/maps unreadable?)");
    }
    for (const auto& lib : libs) {
        report.line("  " + lib);
    }
}

}  // namespace probe
