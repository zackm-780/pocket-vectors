#pragma once
// Top-level application: wires paths, scenario, simulation and (unless --headless) the window and
// debug renderer, then runs the frame loop of ARCHITECTURE section 3.

#include "app/Cli.h"

namespace fsim {

// Game state machine (stubbed in WP-00): menus (H5) and the map (H4) add states later.
// H5: Menu = a full-screen menu (startup / main menu), Paused = the pause menu over a frozen flight.
enum class AppState { Flying, Paused, Menu };

class App {
public:
    explicit App(CliOptions options) : options_(std::move(options)) {}
    // Runs to completion and returns the process exit code (0 = success).
    int run();

private:
    CliOptions options_;
};

}  // namespace fsim
