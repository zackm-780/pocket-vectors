#include "app/App.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "app/DebugInput.h"
#include "app/Simulation.h"
#include "core/FlatTerrain.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "core/SimTime.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "gfx/Camera.h"
#include "gfx/DebugRenderer.h"
#include "gfx/Screenshot.h"
#include "nav/NavDb.h"
#include "platform/Window.h"
#include "world/Atmosphere.h"  // W4
#include "world/Sky.h"         // W4

namespace fsim {
namespace {

constexpr double kDefaultHeadlessDurationS = 60.0;
constexpr int kDefaultScreenshotFrames = 60;
constexpr double kFixedFrameDtS = 1.0 / 60.0;  // deterministic frame time when --frames is used
constexpr float kDefaultVFovDeg = 55.0f;

void printSummary(const Simulation& sim, double simulatedS) {
    const AircraftState& s = sim.aircraft();
    std::printf(
        "%s: simulated %.1f s | lat %.5f lon %.5f | alt %.0f ft MSL (%.0f AGL) | IAS %.1f kt | "
        "GS %.1f kt | VS %.0f fpm | hdg %.0f true | pitch %.1f roll %.1f deg | %s%s%s\n",
        sim.profile().id.c_str(), simulatedS, s.latDeg, s.lonDeg, s.altMslFt, s.altAglFt, s.iasKt,
        s.gsKt, s.vsFpm, radToDeg(s.headingTrueRad), radToDeg(s.pitchRad), radToDeg(s.rollRad),
        s.crashed ? "CRASHED (" : "ok", s.crashed ? s.crashReason : "", s.crashed ? ")" : "");
}

int runHeadless(Simulation& sim, const CliOptions& opt) {
    const double durationS = opt.durationS > 0 ? opt.durationS : kDefaultHeadlessDurationS;
    const long steps = static_cast<long>(durationS * kSimHz + 0.5);
    for (long i = 0; i < steps; ++i) sim.stepOnce();
    printSummary(sim, sim.aircraft().simTimeS);
    return 0;
}

int runWindowed(Simulation& sim, const CliOptions& opt) {
    Window window;
    WindowConfig wc;
    wc.width = opt.windowWidth;
    wc.height = opt.windowHeight;
    wc.fullscreen = opt.fullscreen;
    if (!window.create(wc)) return 1;

    gfx::DebugRenderer renderer(window.gl(), window.flavor());
    if (!renderer.init()) return 1;
    const Airport* airport = sim.navDb().findAirport(sim.scenario().airportIdent);
    if (airport)
        renderer.setWorld(*airport, static_cast<float>(ftToM(sim.startCondition().fieldElevFt)));

    // W4: shared atmosphere (time from the scenario, else local solar noon) and the sky pass.
    world::Atmosphere atmosphere;
    if (!atmosphere.setUtcText(sim.scenario().timeUtc)) {
        world::UtcTime noon;  // 21 June, 12:00 local mean solar time at the start longitude
        const double utcH = 12.0 - sim.aircraft().lonDeg / 15.0;
        noon.hour = static_cast<int>(std::floor(std::fmod(utcH + 24.0, 24.0)));
        noon.minute = static_cast<int>((std::fmod(utcH + 24.0, 24.0) - noon.hour) * 60.0);
        atmosphere.setUtc(noon);
    }
    world::Sky sky(window.gl(), window.flavor());
    if (!sky.init()) return 1;
    renderer.setAtmosphere(&atmosphere, &sky);

    gfx::Camera camera;
    camera.setLens(kDefaultVFovDeg);
    DebugInput input;

    int frames = opt.frames;
    if (!opt.screenshotPath.empty() && frames == 0) frames = kDefaultScreenshotFrames;
    const bool interactive = frames == 0;  // --frames runs are scripted and deterministic

    AppState state = AppState::Flying;
    int frame = 0;
    int fpsFrames = 0;
    std::uint64_t lastCounter = SDL_GetPerformanceCounter();
    double fpsClockS = 0;
    const double perfFreq = static_cast<double>(SDL_GetPerformanceFrequency());

    while (true) {
        const std::uint64_t now = SDL_GetPerformanceCounter();
        const double wallDtS = static_cast<double>(now - lastCounter) / perfFreq;
        lastCounter = now;
        const double dtS = interactive ? std::min(wallDtS, kMaxFrameDtS) : kFixedFrameDtS;

        SDL_Event event;
        while (SDL_PollEvent(&event)) input.handleEvent(event);
        if (input.quitRequested()) break;
        if (input.consumePauseToggle()) {
            state = state == AppState::Flying ? AppState::Paused : AppState::Flying;
            sim.setPaused(state == AppState::Paused);
        }

        if (state == AppState::Flying) {
            if (interactive)
                input.update(dtS, sim.profile().engineCount,
                             static_cast<int>(sim.profile().flapDetents.size()), sim.pilot());
            sim.advance(dtS);
        }

        int w = 0, h = 0;
        window.drawableSize(w, h);
        camera.setViewport(w, h);
        camera.setPoseFromAircraft(sim.aircraft());
        atmosphere.setObserver(camera.eyeGeo().latDeg, camera.eyeGeo().lonDeg,  // W4
                               ftToM(sim.aircraft().altAglFt));
        renderer.render(camera, w, h);
        ++frame;

        if (frames > 0 && frame >= frames) {
            if (!opt.screenshotPath.empty() &&
                !gfx::saveScreenshotPng(window.gl(), w, h, opt.screenshotPath))
                return 1;
            printSummary(sim, sim.aircraft().simTimeS);
            break;
        }
        window.swap();

        ++fpsFrames;
        fpsClockS += wallDtS;
        if (fpsClockS >= 1.0) {
            if (opt.showFps) {
                const AircraftState& s = sim.aircraft();
                char title[160];
                std::snprintf(title, sizeof title, "Flight Sim | %.0f fps | %.0f kt | %.0f ft",
                              fpsFrames / fpsClockS, s.iasKt, s.altMslFt);
                window.setTitle(title);
                LOG_INFO("%.1f fps", fpsFrames / fpsClockS);
            }
            fpsFrames = 0;
            fpsClockS = 0;
        }
    }
    return 0;
}

}  // namespace

int App::run() {
    const PathOverrides overrides{options_.assetDir, options_.dataDir, options_.confDir};
    const Paths paths = Paths::resolve(overrides);

    if (!options_.headless) {
        // The log file lives in the writable conf root. Headless runs log to stderr only.
        if (makeDirs(paths.confRoot())) log::openFile(paths.conf("log.txt"));
    }

    Scenario scenario;
    std::string error;
    if (!options_.scenarioPath.empty() &&
        !loadScenario(paths.resolveInputFile(options_.scenarioPath), scenario, error)) {
        LOG_ERROR("%s", error.c_str());
        return 2;
    }

    // Missing data never crashes (ARCHITECTURE section 8): the synthetic world is always there.
    // N2: with a nav pack at <data root>/nav (nav.ini present) use CsvNavDatabase, which merges XTST
    // in; otherwise the synthetic XTST-only database.
    std::unique_ptr<NavDatabase> navDbOwner;
    const std::string navDir = paths.data("nav");
    if (fileExists(Paths::join(navDir, "nav.ini"))) {
        auto csvNav = std::make_unique<CsvNavDatabase>();
        std::string navError;
        if (csvNav->load(navDir, navError))
            navDbOwner = std::move(csvNav);
        else
            LOG_ERROR("nav database: %s; using the synthetic database", navError.c_str());
    }
    if (!navDbOwner) navDbOwner = std::make_unique<SyntheticNavDb>();
    const NavDatabase& navDb = *navDbOwner;
    FlatTerrain terrain;
    if (const Airport* airport = navDb.findAirport(scenario.airportIdent))
        terrain.setElevationM(static_cast<float>(ftToM(airport->elevFt)));

    Simulation sim(paths, navDb, terrain);
    if (!sim.init(scenario, options_.aircraftId, error)) {
        LOG_ERROR("%s", error.c_str());
        return 2;
    }
    if (!options_.tracePath.empty() && !sim.openTrace(options_.tracePath)) {
        LOG_ERROR("cannot write trace file '%s'", options_.tracePath.c_str());
        return 2;
    }

    const int rc = options_.headless ? runHeadless(sim, options_) : runWindowed(sim, options_);
    log::closeFile();
    return rc;
}

}  // namespace fsim
