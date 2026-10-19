#include "app/App.h"

#include <SDL.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <memory>

#include "app/Settings.h"  // P3
#include "app/Simulation.h"
#include "core/FlatTerrain.h"
#include "core/Log.h"
#include "core/MessageQueue.h"  // P2
#include "core/Paths.h"
#include "core/SimTime.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "gfx/Camera.h"
#include "gfx/DebugRenderer.h"
#include "gfx/Screenshot.h"
#include "hud/Renderer2D.h"            // H1
#include "hud/Renderer2DGlyphSheet.h"  // H1
#include "input/InputSystem.h"         // P2
#include "input/PilotControls.h"       // P2
#include "nav/NavDb.h"
#include "platform/Window.h"
#include "world/Atmosphere.h"              // W4
#include "world/Sky.h"                     // W4
#include "world/terrain/TerrainService.h"  // W1

namespace fsim {
namespace {

constexpr double kDefaultHeadlessDurationS = 60.0;
constexpr int kDefaultScreenshotFrames = 60;
constexpr double kFixedFrameDtS = 1.0 / 60.0;  // deterministic frame time when --frames is used

void printSummary(const Simulation& sim, double simulatedS) {
    const AircraftState& s = sim.aircraft();
    std::printf(
        "%s: simulated %.1f s | lat %.5f lon %.5f | alt %.0f ft MSL (%.0f AGL) | IAS %.1f kt | "
        "GS %.1f kt | VS %.0f fpm | hdg %.0f true | pitch %.1f roll %.1f deg | %s%s%s\n",
        sim.profile().id.c_str(), simulatedS, s.latDeg, s.lonDeg, s.altMslFt, s.altAglFt, s.iasKt,
        s.gsKt, s.vsFpm, radToDeg(s.headingTrueRad), radToDeg(s.pitchRad), radToDeg(s.rollRad),
        s.crashed ? "CRASHED (" : "ok", s.crashed ? s.crashReason : "", s.crashed ? ")" : "");
}

int runHeadless(Simulation& sim, const CliOptions& opt, terrain::TerrainService* terrainSvc) {
    const double durationS = opt.durationS > 0 ? opt.durationS : kDefaultHeadlessDurationS;
    const long steps = static_cast<long>(durationS * kSimHz + 0.5);
    for (long i = 0; i < steps; ++i) {
        sim.stepOnce();
        // W1: integrate finished tiles and keep the load ring on the aircraft.
        if (terrainSvc) terrainSvc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
    }
    printSummary(sim, sim.aircraft().simTimeS);
    return 0;
}

int runWindowed(Simulation& sim, const CliOptions& opt, const Paths& paths,
                terrain::TerrainService* terrainSvc, const Settings& settings) {
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
    // H1: 2D HUD renderer (one batched draw call per frame). The glyph sheet is a debug view.
    Renderer2D hud2d(window.gl(), window.flavor());
    if (!hud2d.init()) return 1;

    gfx::Camera camera;
    camera.setLens(settings.fovVerticalDeg);  // P3
    // P2: gamepad/keyboard -> ActionState -> pilot controls; feedback goes to the message queue
    // (the HUD, H2, will draw it later; for now it is logged).
    InputSystem input;
    input.loadFiles(paths.asset("assets/controls_default.ini"), paths.conf("controls.ini"));
    input.openControllers();
    PilotControls pilotControls;
    pilotControls.setStickTuning(settings.stickTuning());  // P3
    const bool showFps = effectiveShowFps(opt, settings);  // P3
    MessageQueue messages;
    unsigned long shownMessages = 0;

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
        const ActionState& actions = input.update(dtS);  // P2
        messages.tick(dtS);
        if (input.quitRequested()) break;
        if (actions.pressed[static_cast<int>(Action::Pause)]) {
            state = state == AppState::Flying ? AppState::Paused : AppState::Flying;
            sim.setPaused(state == AppState::Paused);
        }

        if (state == AppState::Flying) {
            if (interactive)
                pilotControls.update(actions, sim.profile(), sim.aircraft(), dtS, sim.pilot(),
                                     &messages);  // P2
            sim.advance(dtS);
        }
        // W1: once per frame (also while paused, so tiles keep arriving).
        if (terrainSvc) terrainSvc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);

        // P2: log new feedback messages (until the HUD draws them)
        shownMessages = std::max(shownMessages, messages.totalPosted() - messages.size());
        for (; shownMessages < messages.totalPosted(); ++shownMessages)
            LOG_INFO(
                "%s",
                messages.newest(static_cast<int>(messages.totalPosted() - shownMessages - 1)).text);

        int w = 0, h = 0;
        window.drawableSize(w, h);
        camera.setViewport(w, h);
        camera.setPoseFromAircraft(sim.aircraft());
        atmosphere.setObserver(camera.eyeGeo().latDeg, camera.eyeGeo().lonDeg,  // W4
                               ftToM(sim.aircraft().altAglFt));
        renderer.render(camera, w, h);
        if (opt.glyphSheet) {  // H1
            hud2d.begin(w, h);
            drawGlyphSheet(hud2d);
            hud2d.end();
        }
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
            if (showFps) {
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
    // P3: settings first (they live in the conf root), then the final paths (CLI > settings).
    const Paths confPaths = Paths::resolve({options_.assetDir, "", options_.confDir});
    Settings settings;
    settings.load(confPaths.asset("assets/settings_default.ini"), confPaths.conf("settings.ini"));
    const Paths paths = Paths::resolve(effectivePathOverrides(options_, settings));
    const bool interactiveRun = !options_.headless && options_.frames == 0 &&
                                options_.screenshotPath.empty();  // P3: only these save state

    if (!options_.headless) {
        // The log file lives in the writable conf root. Headless runs log to stderr only.
        if (makeDirs(paths.confRoot())) log::openFile(paths.conf("log.txt"));
    }

    Scenario scenario;
    std::string error;
    std::string scenarioPath = options_.scenarioPath;
    if (scenarioPath.empty() && options_.resume) {  // P3
        if (fileExists(paths.conf("last_flight.ini")))
            scenarioPath = paths.conf("last_flight.ini");
        else
            LOG_WARN("--resume: no last flight saved; starting the default flight");
    }
    std::string aircraftId = options_.aircraftId;  // P3: --aircraft > last_aircraft setting
    if (aircraftId.empty() && interactiveRun && scenarioPath.empty())
        aircraftId = settings.lastAircraft;
    if (!scenarioPath.empty() &&
        !loadScenario(paths.resolveInputFile(scenarioPath), scenario, error)) {
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

    // W1: with a terrain pack at <data root>/terrain (pack.ini present) use TerrainService, else
    // keep FlatTerrain. F3: either way it is the flight model's ground (Simulation::init).
    std::unique_ptr<terrain::TerrainService> terrainSvc;
    const std::string terrainDir = paths.data("terrain");
    if (fileExists(Paths::join(terrainDir, "pack.ini"))) {
        auto svc = std::make_unique<terrain::TerrainService>(terrainDir);
        if (svc->isOpen())
            terrainSvc = std::move(svc);
        else
            LOG_ERROR("terrain pack: %s; using flat terrain", svc->error().c_str());
    }
    TerrainQuery& terrainQuery = terrainSvc ? static_cast<TerrainQuery&>(*terrainSvc) : terrain;

    Simulation sim(paths, navDb, terrainQuery);
    // F3: spawn gating needs finished tiles integrated while it waits for the start detail.
    if (terrainSvc)
        sim.setTerrainPump([svc = terrainSvc.get()](double latDeg, double lonDeg) {
            svc->update(latDeg, lonDeg);
        });
    if (!sim.init(scenario, aircraftId, error)) {
        LOG_ERROR("%s", error.c_str());
        return 2;
    }
    if (terrainSvc) {  // W1: start loading detail around the start position
        terrainSvc->prefetch(sim.startCondition().latDeg, sim.startCondition().lonDeg, 15000.0f);
        terrainSvc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
    }
    if (!options_.tracePath.empty() && !sim.openTrace(options_.tracePath)) {
        LOG_ERROR("cannot write trace file '%s'", options_.tracePath.c_str());
        return 2;
    }

    const int rc = options_.headless
                       ? runHeadless(sim, options_, terrainSvc.get())
                       : runWindowed(sim, options_, paths, terrainSvc.get(), settings);
    if (rc == 0 && interactiveRun) {  // P3: clean exit -> remember the flight
        Scenario last = sim.scenario();
        last.aircraftId = sim.profile().id;
        writeLastFlight(paths.conf("last_flight.ini"), last);
        settings.lastAircraft = last.aircraftId;
        settings.save(paths.conf("settings.ini"));
    }
    log::closeFile();
    return rc;
}

}  // namespace fsim
