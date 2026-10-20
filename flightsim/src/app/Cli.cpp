#include "app/Cli.h"

#include <cstdio>
#include <cstdlib>

namespace fsim {
namespace {

bool parseInt(const std::string& s, int& out) {
    char* end = nullptr;
    const long v = std::strtol(s.c_str(), &end, 10);
    if (end == s.c_str() || *end != '\0') return false;
    out = static_cast<int>(v);
    return true;
}

bool parseDouble(const std::string& s, double& out) {
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == s.c_str() || *end != '\0') return false;
    out = v;
    return true;
}

}  // namespace

std::string cliUsage() {
    return "usage: flightsim [options]\n"
           "  --headless              no window or GL; run sim, nav and autopilot as fast as "
           "possible\n"
           "  --scenario <file.ini>   aircraft, airport/runway, start type, wind, scripted inputs\n"
           "  --aircraft <id>         aircraft id (overrides the scenario), e.g. c172p, 737\n"
           "  --duration <s>          sim seconds to run (headless, default 60)\n"
           "  --trace <file.csv>      write aircraft/nav/autopilot state every 0.1 s\n"
           "  --frames <n>            render n frames, then exit (windowed)\n"
           "  --screenshot <file.png> save a PNG of the last frame (use with --frames)\n"
           "  --window <WxH>          window size (default 1280x960)\n"
           "  --fullscreen            fullscreen at the desktop resolution\n"
           "  --show-fps              show the frame rate in the window title and log\n"
           "  --hud-fake <name>       canned nav/autopilot HUD state for screenshots (see "
           "NavHudFake.h)\n"
           "  --glyph-sheet           draw the HUD glyph sheet (stroke font, 2D renderer demo)\n"
           "  --data-dir <path>       terrain/nav data root (default <exe dir>/data)\n"
           "  --conf-dir <path>       settings/log directory (default ~/.config/flightsim)\n"
           "  --asset-dir <path>      asset root holding jsbsim/ and assets/ (default <exe dir>)\n"
           "  --resume                start from the last flight (<conf dir>/last_flight.ini)\n"
           "  --help                  this text\n";
}

bool parseCli(const std::vector<std::string>& args, CliOptions& out, std::string& error) {
    for (size_t i = 0; i < args.size(); ++i) {
        const std::string& a = args[i];
        auto value = [&](std::string& dst) {
            if (i + 1 >= args.size()) {
                error = a + " needs a value";
                return false;
            }
            dst = args[++i];
            return true;
        };
        std::string v;
        if (a == "--headless")
            out.headless = true;
        else if (a == "--fullscreen")
            out.fullscreen = true;
        else if (a == "--show-fps")
            out.showFps = true;
        else if (a == "--glyph-sheet")  // H1
            out.glyphSheet = true;
        else if (a == "--resume")  // P3
            out.resume = true;
        else if (a == "--help" || a == "-h")
            out.help = true;
        else if (a == "--scenario") {
            if (!value(out.scenarioPath)) return false;
        } else if (a == "--aircraft") {
            if (!value(out.aircraftId)) return false;
        } else if (a == "--trace") {
            if (!value(out.tracePath)) return false;
        } else if (a == "--hud-fake") {  // H3
            if (!value(out.hudFake)) return false;
        } else if (a == "--screenshot") {
            if (!value(out.screenshotPath)) return false;
        } else if (a == "--data-dir") {
            if (!value(out.dataDir)) return false;
        } else if (a == "--conf-dir") {
            if (!value(out.confDir)) return false;
        } else if (a == "--asset-dir") {
            if (!value(out.assetDir)) return false;
        } else if (a == "--duration") {
            if (!value(v)) return false;
            if (!parseDouble(v, out.durationS) || out.durationS < 0) {
                error = "bad --duration '" + v + "'";
                return false;
            }
        } else if (a == "--frames") {
            if (!value(v)) return false;
            if (!parseInt(v, out.frames) || out.frames < 0) {
                error = "bad --frames '" + v + "'";
                return false;
            }
        } else if (a == "--window") {
            if (!value(v)) return false;
            int w = 0, h = 0;
            char sep = 0;
            if (std::sscanf(v.c_str(), "%d%c%d", &w, &sep, &h) != 3 || (sep != 'x' && sep != 'X') ||
                w < 64 || h < 64) {
                error = "bad --window '" + v + "' (expected WxH)";
                return false;
            }
            out.windowWidth = w;
            out.windowHeight = h;
        } else {
            error = "unknown option '" + a + "'";
            return false;
        }
    }
    return true;
}

}  // namespace fsim
