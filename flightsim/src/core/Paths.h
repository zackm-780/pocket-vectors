#pragma once
// Resolution of the three roots (ARCHITECTURE section 8):
//   asset root : read-only, the executable directory (holds jsbsim/ and assets/)
//   data root  : large user-built data (terrain/, nav/, plans/); default <asset root>/data
//   conf root  : writable settings, controls and logs; desktop ~/.config/flightsim, device <port>/conf

#include <string>

namespace fsim {

struct PathOverrides {
    std::string assetDir;  // --asset-dir (tests, packaging); empty = executable directory
    std::string dataDir;   // --data-dir
    std::string confDir;   // --conf-dir
};

class Paths {
public:
    static Paths resolve(const PathOverrides& overrides = {});

    const std::string& assetRoot() const { return assetRoot_; }
    const std::string& dataRoot() const { return dataRoot_; }
    const std::string& confRoot() const { return confRoot_; }

    // Join helpers (always use '/').
    std::string asset(const std::string& rel) const { return join(assetRoot_, rel); }
    std::string data(const std::string& rel) const { return join(dataRoot_, rel); }
    std::string conf(const std::string& rel) const { return join(confRoot_, rel); }

    // Resolve a user-supplied file: used as given if it exists, otherwise tried under the asset
    // root. Returns the input unchanged if neither exists.
    std::string resolveInputFile(const std::string& path) const;

    static std::string join(const std::string& base, const std::string& rel);

private:
    std::string assetRoot_, dataRoot_, confRoot_;
};

bool fileExists(const std::string& path);
bool readTextFile(const std::string& path, std::string& out);  // false if unreadable
bool makeDirs(const std::string& path);                        // mkdir -p
std::string executableDir();  // directory of the running binary, no trailing slash

}  // namespace fsim
