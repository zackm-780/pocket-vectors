#pragma once
// Menu system (WP H5): a stack of screens driven by MenuInput, drawn with Renderer2D. The state
// logic is independent of GL: update() consumes one frame of input and returns what the app should
// do ("start this scenario", "resume", "restart", "quit"); draw() only paints the current state.
//
//   Startup (no --scenario):  Main -> New flight (hub) -> Aircraft / Airport / Runway / Start type /
//                             Flight plan -> Approach -> START FLIGHT -> MenuAction::StartFlight
//   In flight (Pause button): Pause menu -> Resume / Flight plan / Restart / New flight / Settings /
//                             Quit to menu / Quit / Help / Mini nav
//
// Screens (internal, see MenuScreens.cpp) are pushed and popped through deferred commands, so a
// screen may request navigation from inside its own update().

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "app/Scenario.h"
#include "app/Settings.h"
#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/NavDatabase.h"
#include "hud/Renderer2D.h"
#include "nav/ApproachBuilder.h"
#include "nav/Navigator.h"
#include "ui/FlightSetup.h"
#include "ui/ListNav.h"
#include "ui/MenuInput.h"
#include "ui/UiPainter.h"

namespace fsim {

enum class ScreenId {
    None,
    Main,
    Setup,
    Aircraft,
    AirportSearch,
    Runway,
    StartType,
    Plan,
    PlanFiles,
    Approach,
    Settings,
    Pause,
    FlightPlan,
    Confirm,
    Help
};
const char* screenName(ScreenId id);

enum class MenuAction {
    None,
    StartFlight,    // MenuResult::scenario: (re)initialise the simulation with it
    ResumeFlight,   // close the menu and fly on
    RestartFlight,  // re-initialise with the current flight's scenario
    Quit            // leave the game
};

struct MenuResult {
    MenuAction action = MenuAction::None;
    Scenario scenario;
    std::string message;           // short HUD message to show (e.g. "DIRECT TO KOTH")
    bool settingsChanged = false;  // settings were edited and saved: re-apply them
};

// Everything the menus need from the app. Directories and files are explicit so tests can point
// them anywhere. All pointers must outlive the MenuSystem.
struct MenuEnv {
    const NavDatabase* nav = nullptr;
    Settings* settings = nullptr;
    std::string settingsPath;           // saved on leaving the settings screen; empty = do not save
    std::string aircraftDir;            // assets/aircraft
    std::vector<std::string> planDirs;  // data/plans, assets/plans (earlier wins)
    std::string lastFlightPath;         // conf/last_flight.ini; empty or missing = no "resume"
    std::string menuPlanPath;           // where a menu-built plan is written (conf/menu_plan.fms)
    std::string navDir;                 // <data>/nav, for real approach procedures
    std::string controlsDefaultsPath;   // assets/controls_default.ini (Help, H6)
    std::string controlsUserPath;       // conf/controls.ini, written by the button-test swaps
    std::string helpDir;                // assets/help
    std::string attributionPath;        // data/ATTRIBUTION.txt, optional
    double refLatDeg = 0, refLonDeg = 0;  // nearest-airport reference when no flight is active
    bool hasRef = false;
};

// What the pause menu may look at and change while a flight exists.
struct FlightContext {
    Navigator* navigator = nullptr;
    const AircraftState* aircraft = nullptr;
    const AircraftProfile* profile = nullptr;
    const Scenario* scenario = nullptr;
};

class MenuSystem;

class Screen {
public:
    virtual ~Screen() = default;
    virtual ScreenId id() const = 0;
    virtual void update(const MenuInput& in, MenuSystem& menu) = 0;
    virtual void draw(Renderer2D& r, const UiTheme& theme, const MenuSystem& menu) const = 0;
    // Overlays (dialogs) are drawn on top of the screen below them instead of replacing it.
    virtual bool isOverlay() const { return false; }
};

class MenuSystem {
public:
    explicit MenuSystem(MenuEnv env);
    ~MenuSystem();
    MenuSystem(const MenuSystem&) = delete;
    MenuSystem& operator=(const MenuSystem&) = delete;

    // ---- opening -------------------------------------------------------------------------
    void openMain();   // startup menu
    void openPause();  // needs setFlight() first
    // Canned screens for screenshots and tests (documented in src/ui/README.md):
    //   main | setup | aircraft | airport[:TEXT] | runway[:ICAO] | start | plan[:FILE] |
    //   approach[:FILE] | settings | pause | flightplan | confirm
    // Returns false (with `error`) for an unknown spec.
    bool openCanned(const std::string& spec, std::string& error);
    void close();
    bool active() const { return !stack_.empty(); }
    ScreenId top() const { return stack_.empty() ? ScreenId::None : stack_.back()->id(); }
    int depth() const { return static_cast<int>(stack_.size()); }
    // True while the bottom screen is the main menu (no flight is shown behind it).
    bool fullScreen() const { return !stack_.empty() && stack_.front()->id() == ScreenId::Main; }

    // The flight the pause menu operates on (pointers must stay valid while it is set).
    void setFlight(const FlightContext& ctx) {
        flight_ = ctx;
        hasFlight_ = true;
    }
    void clearFlight() {
        hasFlight_ = false;
        flight_ = FlightContext{};
    }
    bool hasFlight() const { return hasFlight_; }
    const FlightContext& flight() const { return flight_; }

    // A flight failed to start: show the reason (single OK button).
    void showError(const std::string& message);

    // ---- frame ---------------------------------------------------------------------------
    MenuResult update(const MenuInput& in);
    void draw(Renderer2D& r) const;

    // ---- state shared with the screens -----------------------------------------------------
    const MenuEnv& env() const { return env_; }
    const NavDatabase& nav() const { return *env_.nav; }
    Settings& settings() { return *env_.settings; }
    const Settings& settings() const { return *env_.settings; }
    FlightSetup& setup() { return setup_; }
    const FlightSetup& setup() const { return setup_; }
    const std::vector<AircraftEntry>& aircraft() const { return aircraft_; }
    const AircraftEntry* aircraftEntry(const std::string& id) const;
    const RealApproachProvider& approachProvider() const { return provider_; }
    // Position nearest lists are measured from: the aircraft in flight, else the position of the
    // airport last selected (env refLat/LonDeg), else KPDX, else XTST.
    void referencePosition(double& latDeg, double& lonDeg) const;
    bool lastFlightAvailable() const;

    // Last gamepad event, copied in by the app each frame for the Help button test.
    struct PadProbeView {
        bool valid = false;
        bool axis = false;
        int sdlCode = -1;
        unsigned serial = 0;
    };
    void setPadProbe(const PadProbeView& probe) { probe_ = probe; }
    const PadProbeView& padProbe() const { return probe_; }

    // Deferred navigation (applied after the current update() returns).
    void push(std::unique_ptr<Screen> screen);
    void pop();
    void popTo(ScreenId id);  // keeps `id` on top; clears everything when it is not on the stack
    void replaceTop(std::unique_ptr<Screen> screen);
    void clearStack();
    // Results for the app, returned by the current update().
    void post(MenuResult result);
    // Builds the scenario from setup(), writes the plan and posts StartFlight. Shows a message
    // dialog and returns false on failure.
    bool startFlight();
    // Initialises setup() from a scenario (the pause menu's "New flight").
    void seedSetupFrom(const Scenario& s);
    void confirm(const std::string& message, std::function<void(MenuSystem&)> onYes);

private:
    struct Command {
        enum class Kind { Push, Pop, PopTo, Replace, Clear } kind = Kind::Pop;
        std::unique_ptr<Screen> screen;
        ScreenId target = ScreenId::None;
    };
    void applyCommands();

    MenuEnv env_;
    std::vector<std::unique_ptr<Screen>> stack_;
    std::vector<Command> commands_;
    FlightSetup setup_;
    std::vector<AircraftEntry> aircraft_;
    RealApproachProvider provider_;
    FlightContext flight_;
    bool hasFlight_ = false;
    MenuResult pending_;
    PadProbeView probe_;
};

// Menu spec helpers shared with App/Cli (parsing is trivial; kept here for tests).
std::string lastFlightSummary(const Scenario& s);

}  // namespace fsim
