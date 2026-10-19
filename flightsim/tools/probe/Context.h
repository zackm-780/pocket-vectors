// Shared state for all probe phases: options, SDL window + GL context, text renderer, report.
#pragma once

#include <string>

#include "Gl.h"
#include "Report.h"

namespace probe {

struct Options {
    std::string outDir;          // where probe_report.txt goes ("" = next to the binary)
    std::string jsbsimDir;       // JSBSim data root ("" = <exe dir>/jsbsim)
    std::string shotDir;         // save BMP screenshots of the probe UI here ("" = off)
    bool fullscreen = false;     // fullscreen desktop window (device)
    int windowW = 640;
    int windowH = 480;
    bool autoMode = false;       // never wait for a human: short input test, no final hold
    bool selfTestInput = false;  // inject synthetic controller events (desktop verification)
    bool quick = false;          // shorter benchmarks (CI / emulation)
    bool noBeep = false;
    bool cpuOnly = false;        // no SDL video/audio/input at all
    bool skipInput = false;
    bool skipGpu = false;
    bool skipCpu = false;
    bool skipSweep = false;
    bool skipAudio = false;
    double inputTimeScale = 1.0; // multiplies the per-prompt timeouts of the input test
    double holdSeconds = 120.0;  // how long the final summary screen stays up (-1 = until a button press)
    bool escapeQuits = true;     // Escape / window close ends the probe (off under PortMaster)
};

// What a window + context request asks for, and what SDL reports back.
struct GlRequest {
    int depthBits = 24;
    int stencilBits = 0;
    int msaaSamples = 0;
    int redBits = -1;  // -1 = SDL default
    int greenBits = -1;
    int blueBits = -1;
    int alphaBits = -1;
    bool preferEs = true;
};

struct GlObtained {
    int redBits = 0, greenBits = 0, blueBits = 0, alphaBits = 0;
    int depthBits = 0, stencilBits = 0;
    int msaaBuffers = 0, msaaSamples = 0;
    int doubleBuffer = 0;
    int profileMask = 0;
    int major = 0, minor = 0;
    int accelerated = -1;
    bool isEs = false;
};

struct Ctx {
    Options opt;
    Report* report = nullptr;

    SDL_Window* window = nullptr;
    SDL_GLContext glContext = nullptr;
    GlFuncs gl;
    TextRenderer textRenderer;
    bool glOk = false;
    bool videoOk = false;
    GlObtained obtained;
    int widthPx = 640;   // drawable size
    int heightPx = 480;
    bool quitRequested = false;
    std::string phase = "starting";
    int shotCounter = 0;
    std::string pendingShot;  // name of a screenshot to take at the end of the next UI frame

    // Creates the window and GL context for `request`. On failure returns false and fills `error`.
    // Any previous window/context is destroyed first.
    bool openWindow(const GlRequest& request, std::string& error);
    void closeWindow();

    // Handles events common to every phase (quit, Escape). Returns true if the event was consumed.
    bool handleCommonEvent(const SDL_Event& e);
    // Drains pending events through handleCommonEvent.
    void pumpEvents();

    // Clears and starts a UI frame. Pair with endUiFrame(). `scale` is the integer text scale.
    int uiScale() const;
    void beginUiFrame();
    // Draws the standard header, the log tail and the footer, then presents. `progress` in 0..1 or <0.
    void drawStatusScreen(const std::string& footer, double progress);
    void endUiFrame();
    // Asks for the next UI frame to be saved as <shotDir>/NN_<name>.bmp (only with --shot-dir).
    void requestShot(const char* name) {
        if (!opt.shotDir.empty()) {
            pendingShot = name;
        }
    }
    // Saves the current back buffer immediately (only with --shot-dir). Normally use requestShot().
    void maybeScreenshot(const char* name);
};

}  // namespace probe
