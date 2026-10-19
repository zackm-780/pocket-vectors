#include "core/Paths.h"

#include <sys/stat.h>

#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

#if defined(_WIN32)
#include <direct.h>
#include <windows.h>
#elif defined(__APPLE__)
#include <mach-o/dyld.h>
#include <unistd.h>
#else
#include <unistd.h>
#endif

namespace fsim {

bool fileExists(const std::string& path) {
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

bool readTextFile(const std::string& path, std::string& out) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return false;
    std::ostringstream ss;
    ss << in.rdbuf();
    out = ss.str();
    return true;
}

bool makeDirs(const std::string& path) {
    if (path.empty()) return false;
    std::string cur;
    for (size_t i = 0; i <= path.size(); ++i) {
        if (i == path.size() || path[i] == '/' || path[i] == '\\') {
            if (!cur.empty() && cur != ".") {
#if defined(_WIN32)
                _mkdir(cur.c_str());
#else
                ::mkdir(cur.c_str(), 0755);
#endif
            }
        }
        if (i < path.size()) cur.push_back(path[i]);
    }
    struct stat st;
    return ::stat(path.c_str(), &st) == 0;
}

std::string executableDir() {
    char buf[4096] = {0};
    std::string exe;
#if defined(_WIN32)
    const DWORD n = GetModuleFileNameA(nullptr, buf, sizeof buf);
    exe.assign(buf, n);
#elif defined(__APPLE__)
    uint32_t size = sizeof buf;
    if (_NSGetExecutablePath(buf, &size) == 0) exe = buf;
#else
    const ssize_t n = ::readlink("/proc/self/exe", buf, sizeof buf - 1);
    if (n > 0) exe.assign(buf, static_cast<size_t>(n));
#endif
    const size_t slash = exe.find_last_of("/\\");
    return slash == std::string::npos ? std::string(".") : exe.substr(0, slash);
}

std::string Paths::join(const std::string& base, const std::string& rel) {
    if (base.empty()) return rel;
    if (rel.empty()) return base;
    if (base.back() == '/') return base + rel;
    return base + "/" + rel;
}

Paths Paths::resolve(const PathOverrides& o) {
    Paths p;
    p.assetRoot_ = o.assetDir.empty() ? executableDir() : o.assetDir;
    p.dataRoot_ = o.dataDir.empty() ? join(p.assetRoot_, "data") : o.dataDir;

    if (!o.confDir.empty()) {
        p.confRoot_ = o.confDir;
    } else if (const char* env = std::getenv("FLIGHTSIM_CONF_DIR"); env && *env) {
        p.confRoot_ = env;
    } else if (const char* platform = std::getenv("FLIGHTSIM_PLATFORM");
               platform && std::strcmp(platform, "portmaster") == 0) {
        p.confRoot_ = join(p.assetRoot_, "conf");
    } else if (const char* xdg = std::getenv("XDG_CONFIG_HOME"); xdg && *xdg) {
        p.confRoot_ = join(xdg, "flightsim");
    } else if (const char* home = std::getenv("HOME"); home && *home) {
        p.confRoot_ = join(home, ".config/flightsim");
    } else if (const char* appdata = std::getenv("APPDATA"); appdata && *appdata) {
        p.confRoot_ = join(appdata, "flightsim");
    } else {
        p.confRoot_ = join(p.assetRoot_, "conf");
    }
    return p;
}

std::string Paths::resolveInputFile(const std::string& path) const {
    if (fileExists(path)) return path;
    const std::string underAssets = asset(path);
    if (fileExists(underAssets)) return underAssets;
    return path;
}

}  // namespace fsim
