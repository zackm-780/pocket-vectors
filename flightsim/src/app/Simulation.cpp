#include "app/Simulation.h"

#include <algorithm>

#include "core/Log.h"
#include "core/SimTime.h"
#include "core/SyntheticNavDb.h"  // findRunway
#include "core/Units.h"
#include "nav/ApproachBuilder.h"  // N5
#include "nav/FlightPlan.h"       // N5
#include "nav/Procedures.h"       // N4
#include "sim/ProfileLoader.h"

namespace fsim {
namespace {
constexpr float kSpawnPrefetchRadiusM = 5000.0f;  // F3
}

Simulation::Simulation(const Paths& paths, const NavDatabase& nav, TerrainQuery& terrain)
    : paths_(paths), nav_(nav), terrain_(terrain), model_(paths.asset("jsbsim")), navigator_(nav) {
    // N4: real FAA CIFP approaches (<data root>/nav/procedures/{ICAO}.proc) are listed before the
    // synthesized ones; airports without a file keep the synthesized approaches.
    navigator_.setRealApproachProvider(makeCifpApproachProvider(paths.data("nav")));
}

bool Simulation::init(const Scenario& scenario, const std::string& aircraftOverride,
                      std::string& error) {
    scenario_ = scenario;
    if (!aircraftOverride.empty()) scenario_.aircraftId = aircraftOverride;

    const std::string profilePath =
        paths_.asset("assets/aircraft/" + scenario_.aircraftId + ".ini");
    if (!loadAircraftProfile(profilePath, profile_, error)) return false;
    if (!resolveStart(scenario_, nav_, profile_, start_, error)) return false;

    // F3: the flight model's ground is the terrain. Make the start sit on it. The runway pads
    // (flatten areas) must already be registered (W5: App does it before init()).
    const Airport* airport = nav_.findAirport(scenario_.airportIdent);
    const Runway* runway = airport ? findRunway(*airport, scenario_.runwayIdent) : nullptr;
    std::vector<SpawnPoint> gatePoints{{start_.latDeg, start_.lonDeg}};
    if (runway && start_.kind != StartCondition::Kind::Runway)
        gatePoints.push_back({runway->thrLatDeg, runway->thrLonDeg});  // where it will land
    spawnGate_ = waitForSpawnDetail(terrain_, gatePoints, kSpawnPrefetchRadiusM, spawnTimeoutS_,
                                    terrainPump_);
    if (spawnGate_.timedOut)
        LOG_WARN("terrain detail not ready after %.1f s; starting anyway (ground may jump)",
                 spawnGate_.waitedS);
    else if (spawnGate_.polls > 0)
        LOG_INFO("terrain detail ready after %.2f s", spawnGate_.waitedS);
    // A runway start sits on the (flattened) surface, not at the nominal field elevation.
    start_.fieldElevFt = mToFt(terrain_.elevationM(start_.latDeg, start_.lonDeg));
    model_.setTerrain(&terrain_);

    if (!model_.start(profile_, start_)) {
        error = "flight model failed to start " + profile_.id + " (see log)";
        return false;
    }

    pilot_ = model_.startControls();
    final_ = pilot_;
    script_.setEvents(scenario_.inputs);
    accumulatorS_ = 0;
    nextTraceS_ = 0;
    navState_ = NavState{};
    if (!initNavigation(error)) return false;  // N5

    // Scenario autopilot settings are recorded for the HUD; the real autopilot (N6) acts on them.
    AutopilotState& ap = autopilot_.state();
    ap = AutopilotState{};
    ap.hdgBugMagDeg = scenario_.hdgBugMagDeg;
    ap.altBugFt = scenario_.altBugFt;
    if (!scenario_.apEngage.empty())
        LOG_WARN("scenario requests autopilot engagement; autopilot arrives with WP N6 (ignored)");

    LOG_INFO("start: %s at %s rwy %s (%s) lat %.5f lon %.5f alt %.0f ft hdg %.0f true",
             profile_.id.c_str(), scenario_.airportIdent.c_str(), scenario_.runwayIdent.c_str(),
             scenario_.startKind.c_str(), start_.latDeg, start_.lonDeg,
             start_.kind == StartCondition::Kind::Runway ? start_.fieldElevFt : start_.altMslFt,
             start_.headingTrueDeg);
    return true;
}

// N5: optional `plan` / `approach` scenario keys. A plan is loaded from an .fms file; an approach
// id is loaded into it (or, without a plan, becomes a one-approach plan at the scenario airport).
// Scenarios that start in the air activate the leg nearest the aircraft.
bool Simulation::initNavigation(std::string& error) {
    navigator_.clearPlan();
    navigator_.setAircraftCategory(profile_.category);
    const bool inAir = start_.kind != StartCondition::Kind::Runway;
    if (!scenario_.planPath.empty()) {
        const std::string path = paths_.resolveInputFile(scenario_.planPath);
        PlanBuildResult r = loadPlanFromFmsFile(path, nav_);
        if (!r.ok) {
            error =
                "scenario plan '" + path + "': " + (r.errors.empty() ? "unreadable" : r.errors[0]);
            return false;
        }
        if (!scenario_.approachId.empty()) {
            r.plan.approachId = scenario_.approachId;
            r.plan.approachStartLeg = -1;
            const std::string rwy = runwayFromApproachId(scenario_.approachId);
            if (!rwy.empty()) r.plan.arrRunway = rwy;
        }
        navigator_.setPlan(std::move(r.plan));
        if (!scenario_.approachId.empty() && navigator_.plan().approachStartLeg < 0) {
            error = "scenario approach '" + scenario_.approachId + "' is not available for " +
                    navigator_.plan().destIdent + " " + navigator_.plan().arrRunway;
            return false;
        }
    } else if (!scenario_.approachId.empty()) {
        std::string rwy = runwayFromApproachId(scenario_.approachId);
        if (rwy.empty()) rwy = scenario_.runwayIdent;
        auto approach = findApproach(nav_, scenario_.airportIdent, rwy, scenario_.approachId,
                                     makeCifpApproachProvider(paths_.data("nav")));  // N4
        if (!approach) {
            error = "scenario approach '" + scenario_.approachId + "' is not available for " +
                    scenario_.airportIdent + " " + rwy;
            return false;
        }
        if (!navigator_.startApproachOnly(*approach, LatLon{start_.latDeg, start_.lonDeg}, 0)) {
            error = "cannot load approach " + scenario_.approachId;
            return false;
        }
    } else {
        return true;
    }
    if (inAir) navigator_.activateNearestLeg(model_.state());
    LOG_INFO("navigation: plan %zu legs, active leg %d, approach %s", navigator_.plan().legs.size(),
             navigator_.plan().activeLeg, navigator_.plan().approachId.c_str());
    return true;
}

void Simulation::stepOnce() {
    const double simTimeS = model_.state().simTimeS;
    script_.apply(simTimeS, profile_.engineCount, pilot_);
    final_ = autopilot_.update(pilot_, model_.state(), navState_, kSimDtS);
    model_.step(final_, kSimDtS);
    const AircraftState& s = model_.state();
    navState_ = navigator_.update(s, kSimDtS);
    if (trace_.isOpen() && s.simTimeS + 1e-9 >= nextTraceS_) {
        trace_.write(s, navState_, autopilot_.state());
        nextTraceS_ += kTraceIntervalS;
    }
}

int Simulation::advance(double wallDtS) {
    if (paused_) return 0;
    accumulatorS_ += std::min(wallDtS, kMaxFrameDtS);
    int steps = 0;
    while (accumulatorS_ >= kSimDtS) {
        stepOnce();
        accumulatorS_ -= kSimDtS;
        ++steps;
    }
    return steps;
}

}  // namespace fsim
