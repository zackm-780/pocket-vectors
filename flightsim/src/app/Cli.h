#pragma once
// Command line (ARCHITECTURE section 9). All flags work on desktop and device.

#include <string>
#include <vector>

namespace fsim {

struct CliOptions {
    bool headless = false;
    std::string scenarioPath;
    std::string aircraftId;  // overrides the scenario aircraft
    double durationS = 0;    // headless: sim seconds to run (0 = 60)
    std::string tracePath;
    int frames = 0;  // windowed: render n frames then exit (0 = run until quit)
    std::string screenshotPath;
    std::string dataDir, confDir, assetDir;
    int windowWidth = 1280, windowHeight = 960;  // 2x the device resolution
    bool fullscreen = false;
    bool showFps = false;
    bool glyphSheet = false;  // H1: draw the 2D renderer / stroke font glyph sheet over the 3D view
    bool help = false;
};

// Parses argv. On error returns false and fills `error`.
bool parseCli(const std::vector<std::string>& args, CliOptions& out, std::string& error);
std::string cliUsage();

}  // namespace fsim
