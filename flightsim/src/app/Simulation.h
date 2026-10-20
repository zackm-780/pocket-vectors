#pragma once
// The headless-capable part of the app: scenario -> start condition -> fixed-step loop of
// pilot controls -> autopilot -> flight model -> nav (ARCHITECTURE section 3). Owns no window or
// GL, so tests and --headless runs use it directly.

#include <functional>
#include <memory>
#include <string>

#include "app/Scenario.h"
#include "app/SpawnGate.h"  // F3
#include "app/Trace.h"
#include "autopilot/Autopilot.h"  // N6
#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"
#include "core/NavDatabase.h"
#include "core/Paths.h"
#include "core/TerrainQuery.h"
#include "nav/Navigator.h"  // N5
#include "sim/FlightModel.h"

namespace fsim {

class Simulation {
public:
    Simulation(const Paths& paths, const NavDatabase& nav, TerrainQuery& terrain);

    // Loads the aircraft profile, resolves the start and starts the flight model.
    // aircraftOverride (from --aircraft) wins over the scenario's aircraft when non-empty.
    bool init(const Scenario& scenario, const std::string& aircraftOverride, std::string& error);

    bool openTrace(const std::string& path) { return trace_.open(path); }

    // F3: spawn gating. init() prefetches terrain around the start (and the landing runway) and
    // waits, up to the timeout, until the finest detail there is resident. The pump integrates
    // finished tiles (TerrainService::update); without one init() does not wait.
    void setTerrainPump(TerrainPump pump) { terrainPump_ = std::move(pump); }
    void setSpawnGateTimeoutS(double timeoutS) { spawnTimeoutS_ = timeoutS; }
    const SpawnGateResult& spawnGate() const { return spawnGate_; }  // outcome of the last init()

    // Advance by wall-clock time using the fixed-step accumulator; returns steps executed.
    int advance(double wallDtS);
    // Exactly one fixed step.
    void stepOnce();

    // Persistent pilot controls: the keyboard/gamepad layer edits these between frames, and
    // scripted scenario events edit them as simulation time passes.
    ControlInputs& pilot() { return pilot_; }
    const ControlInputs& finalControls() const { return final_; }
    void setPaused(bool paused) { paused_ = paused; }
    bool paused() const { return paused_; }

    const AircraftState& aircraft() const { return model_.state(); }
    const NavState& navState() const { return navState_; }
    const AutopilotState& autopilotState() const { return autopilot_.state(); }
    const AircraftProfile& profile() const { return profile_; }
    const StartCondition& startCondition() const { return start_; }
    const Scenario& scenario() const { return scenario_; }
    const NavDatabase& navDb() const { return nav_; }
    TerrainQuery& terrain() { return terrain_; }
    FlightModel& flightModel() { return model_; }
    Navigator& navigator() { return navigator_; }  // N5: plan / approach / source control
    Autopilot& autopilot() { return autopilot_; }  // N6: the app feeds ActionState to it

private:
    bool initNavigation(std::string& error);  // N5

    const Paths& paths_;
    const NavDatabase& nav_;
    TerrainQuery& terrain_;
    FlightModel model_;
    Navigator navigator_;  // N5
    Autopilot autopilot_;  // N6
    AircraftProfile profile_;
    Scenario scenario_;
    StartCondition start_;
    ScriptedInputs script_;
    TraceWriter trace_;
    NavState navState_;
    ControlInputs pilot_, final_;
    double accumulatorS_ = 0;
    double nextTraceS_ = 0;
    bool paused_ = false;
    TerrainPump terrainPump_;      // F3
    double spawnTimeoutS_ = 10.0;  // F3
    SpawnGateResult spawnGate_;    // F3
};

}  // namespace fsim
