#include "app/App.h"

#include <SDL.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <memory>

#include "app/Settings.h"  // P3
#include "app/Simulation.h"
#include "audio/AudioControl.h"  // S1
#include "audio/AudioEngine.h"   // S1
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
#include "hud/Hud.h"                   // H2
#include "hud/NavHud.h"                // H3
#include "hud/NavHudFake.h"            // H3
#include "hud/Renderer2D.h"            // H1
#include "hud/Renderer2DGlyphSheet.h"  // H1
#include "input/InputSystem.h"         // P2
#include "input/PilotControls.h"       // P2
#include "nav/NavDb.h"
#include "platform/Window.h"
#include "ui/MapPage.h"                    // H4
#include "ui/MenuSystem.h"                 // H5
#include "world/AirportGeometry.h"         // W5
#include "world/AirportRenderer.h"         // W5
#include "world/Atmosphere.h"              // W4
#include "world/Sky.h"                     // W4
#include "world/TerrainRenderer.h"         // W2
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

// H5: (re)starts a flight from a menu-built scenario: flat-terrain elevation, runway pads, the
// simulation and the terrain prefetch. Returns false with `error` when the simulation refuses.
using BeginFlightFn = std::function<bool(const Scenario&, std::string&)>;

int runWindowed(Simulation& sim, const CliOptions& opt, const Paths& paths,
                terrain::TerrainService* terrainSvc, Settings& settings,
                const BeginFlightFn& beginFlight, bool startInMenu) {
    Window window;
    WindowConfig wc;
    wc.width = opt.windowWidth;
    wc.height = opt.windowHeight;
    wc.fullscreen = opt.fullscreen;
    if (!window.create(wc)) return 1;

    gfx::DebugRenderer renderer(window.gl(), window.flavor());
    if (!renderer.init()) return 1;
    const Airport* airport = sim.navDb().findAirport(sim.scenario().airportIdent);
    if (airport) {
        // W5: AirportRenderer draws the runways; the debug ground grid sits at the lowest runway.
        renderer.setDrawRunways(false);
        renderer.setWorld(*airport, static_cast<float>(world::airportLowestFlatElevM(*airport)));
    }

    // W4: shared atmosphere (time from the scenario, else local solar noon) and the sky pass.
    world::Atmosphere atmosphere;
    auto setSkyTime = [&]() {  // H5: also called when a menu starts another flight
        if (atmosphere.setUtcText(sim.scenario().timeUtc)) return;
        world::UtcTime noon;  // 21 June, 12:00 local mean solar time at the start longitude
        const double utcH = 12.0 - sim.aircraft().lonDeg / 15.0;
        noon.hour = static_cast<int>(std::floor(std::fmod(utcH + 24.0, 24.0)));
        noon.minute = static_cast<int>((std::fmod(utcH + 24.0, 24.0) - noon.hour) * 60.0);
        atmosphere.setUtc(noon);
    };
    setSkyTime();
    world::Sky sky(window.gl(), window.flavor());
    if (!sky.init()) return 1;
    renderer.setAtmosphere(&atmosphere, &sky);
    // W2: with a terrain pack the TerrainRenderer replaces the flat ground grid. The env hooks
    // FSIM_W2_CAMERA / FSIM_W2_VISIBILITY_KM override the view pose and fog for screenshots.
    const world::TerrainDebugOverrides dbg = world::readTerrainDebugEnv();
    if (dbg.visibilityKm > 0) atmosphere.setVisibilityM(dbg.visibilityKm * 1000.0f);
    std::unique_ptr<world::TerrainRenderer> terrainRenderer;
    if (terrainSvc) {
        terrainRenderer =
            std::make_unique<world::TerrainRenderer>(window.gl(), window.flavor(), *terrainSvc);
        if (terrainRenderer->init()) {
            renderer.setTerrain(terrainRenderer.get());
            if (dbg.pixelsPerCell > 0) {
                world::TerrainRenderConfig tc = terrainRenderer->config();
                tc.lod.pixelsPerCell = dbg.pixelsPerCell;
                terrainRenderer->setConfig(tc);
            }
        } else
            terrainRenderer.reset();
    }
    auto viewState = [&]() {  // W2: the aircraft state, or the debug camera pose
        AircraftState s = sim.aircraft();
        if (dbg.hasCamera) {
            s.latDeg = dbg.latDeg, s.lonDeg = dbg.lonDeg, s.altMslFt = dbg.altMslFt;
            s.headingTrueRad = static_cast<float>(degToRad(dbg.headingTrueDeg));
            s.pitchRad = static_cast<float>(degToRad(dbg.pitchDeg));
            s.rollRad = 0;
            s.eyeBodyM[0] = s.eyeBodyM[1] = s.eyeBodyM[2] = 0;
            const float groundFt =
                terrainSvc ? static_cast<float>(mToFt(terrainSvc->elevationM(s.latDeg, s.lonDeg)))
                           : 0;
            s.altAglFt = std::max(0.0f, static_cast<float>(s.altMslFt) - groundFt);
        }
        return s;
    };
    // W5: airports near the camera (runways, markings, lights, PAPI); registers flatten areas.
    world::AirportRenderer airports(window.gl(), window.flavor());
    if (!airports.init()) return 1;
    airports.setNavDatabase(&sim.navDb());
    airports.setTerrain(&sim.terrain());
    airports.setAtmosphere(&atmosphere);
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
    bool showFps = effectiveShowFps(opt, settings);        // P3
    MessageQueue messages;
    Hud hud;                // H2: flight HUD, drawn every frame after the 3D scene
    HudOptions hudOptions;  // H2: declutter level, color, paused flag, magnetic variation
    MapPage mapPage;        // H4: moving map (MapToggle), replaces the HUD while shown
    bool mapShown = opt.mapZoomNm > 0;
    if (mapShown) mapPage.setZoomNm(static_cast<float>(opt.mapZoomNm));
    if (opt.mapTrackUp) mapPage.setOrientation(MapOrientation::TrackUp);
    NavHud navHud;  // H3: FMA, nav box, CDI/LOC/GS, marks, bugs, FD, minimums
    hud.setOverlay(&navHud);
    // H2: P3 settings give the starting HUD color and declutter level.
    switch (settings.hudColor) {
    case HudColor::White: hudOptions.color = HudPalette::White; break;
    case HudColor::Amber: hudOptions.color = HudPalette::Amber; break;
    case HudColor::Green: hudOptions.color = HudPalette::Green; break;
    }
    switch (settings.hudDeclutterDefault) {
    case HudDeclutter::Full: hudOptions.declutter = Declutter::Full; break;
    case HudDeclutter::Std: hudOptions.declutter = Declutter::Std; break;
    case HudDeclutter::Min: hudOptions.declutter = Declutter::Min; break;
    }
    unsigned long shownMessages = 0;
    // H5: menus (main, new flight, pause, settings). The flight context lets the pause menu edit
    // the flight plan; the menu itself never touches the simulation.
    MenuEnv menuEnv;
    menuEnv.nav = &sim.navDb();
    menuEnv.settings = &settings;
    menuEnv.settingsPath = paths.conf("settings.ini");
    menuEnv.aircraftDir = paths.asset("assets/aircraft");
    menuEnv.planDirs = {paths.data("plans"), paths.asset("assets/plans")};
    menuEnv.lastFlightPath = paths.conf("last_flight.ini");
    menuEnv.menuPlanPath = paths.conf("menu_plan.fms");
    menuEnv.navDir = paths.data("nav");
    MenuSystem menu(menuEnv);
    menu.setFlight({&sim.navigator(), &sim.aircraft(), &sim.profile(), &sim.scenario()});
    if (!opt.menuSpec.empty()) {
        std::string menuError;
        if (!menu.openCanned(opt.menuSpec, menuError)) {
            LOG_ERROR("--menu: %s", menuError.c_str());
            return 2;
        }
    } else if (startInMenu) {
        menu.openMain();
    }

    int frames = opt.frames;
    if (!opt.screenshotPath.empty() && frames == 0) frames = kDefaultScreenshotFrames;
    const bool interactive = frames == 0;  // --frames runs are scripted and deterministic

    // S1: procedural audio. No device is never an error (init just returns false); scripted
    // --frames runs stay silent.
    audio::AudioEngine audioEngine;
    audio::AudioControl audioControl;
    audio::AudioParams audioParams;
    std::vector<audio::AudioEvent> audioEvents;
    audioEvents.reserve(16);
    if (interactive) audioEngine.init();

    AppState state = AppState::Flying;
    bool quitRequested = false;
    // H5: applies edited settings to the running game (HUD color, sticks, FOV, FPS display).
    auto applySettings = [&]() {
        switch (settings.hudColor) {
        case HudColor::White: hudOptions.color = HudPalette::White; break;
        case HudColor::Amber: hudOptions.color = HudPalette::Amber; break;
        case HudColor::Green: hudOptions.color = HudPalette::Green; break;
        }
        pilotControls.setStickTuning(settings.stickTuning());
        camera.setLens(settings.fovVerticalDeg);
        showFps = showFps || settings.showFps;
    };
    // H5: a flight (re)started: the ground grid, sky time and HUD follow the new airport.
    auto flightStarted = [&]() {
        if (const Airport* ap = sim.navDb().findAirport(sim.scenario().airportIdent))
            renderer.setWorld(*ap, static_cast<float>(world::airportLowestFlatElevM(*ap)));
        setSkyTime();
        menu.setFlight({&sim.navigator(), &sim.aircraft(), &sim.profile(), &sim.scenario()});
    };
    auto startOrReport = [&](const Scenario& sc) {
        std::string err;
        if (beginFlight(sc, err)) {
            menu.close();
            flightStarted();
        } else {
            LOG_ERROR("%s", err.c_str());
            for (char& c : err) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
            menu.showError(err);
        }
    };
    int frame = 0;
    int fpsFrames = 0;
    std::uint64_t lastCounter = SDL_GetPerformanceCounter();
    double fpsClockS = 0;
    const double perfFreq = static_cast<double>(SDL_GetPerformanceFrequency());

    if (terrainRenderer) {  // W2: load and build the first view before the first frame
        int pw = 0, ph = 0;
        window.drawableSize(pw, ph);
        camera.setViewport(pw, ph);
        for (int pass = 0; pass < 4; ++pass) {
            const AircraftState vs = viewState();
            terrainSvc->update(vs.latDeg, vs.lonDeg);
            terrainSvc->waitForIdle(10.0);
            camera.setPoseFromAircraft(vs);
            if (terrainRenderer->prewarm(camera, 800) == 0) break;
        }
    }

    while (true) {
        const std::uint64_t now = SDL_GetPerformanceCounter();
        const double wallDtS = static_cast<double>(now - lastCounter) / perfFreq;
        lastCounter = now;
        const double dtS = interactive ? std::min(wallDtS, kMaxFrameDtS) : kFixedFrameDtS;

        SDL_Event event;
        while (SDL_PollEvent(&event)) input.handleEvent(event);
        const ActionState& actions = input.update(dtS);  // P2
        messages.tick(dtS);
        // H4: the map takes MapToggle; while shown it consumes the D-pad / A / B / Y actions.
        ActionState flightActions = actions;
        if (menu.active()) {
            // H5: an open menu owns the input; the map page stays as it was
        } else if (actions.pressed[static_cast<int>(Action::MapToggle)] && !mapShown) {
            mapShown = true;
            mapPage.resetSelection();
        } else if (mapShown) {
            const MapPageResult mr = mapPage.update(
                actions, sim.navigator().hasPlan() ? &sim.navigator().plan() : nullptr);
            if (mr.directToLeg >= 1 && sim.navigator().directToLeg(mr.directToLeg))
                messages.post("DIRECT TO");
            if (mr.close) mapShown = false;
            for (Action a : {Action::MenuUp, Action::MenuDown, Action::MenuLeft, Action::MenuRight,
                             Action::MenuAccept, Action::MenuBack, Action::HudDeclutter,
                             Action::TrimNoseUp, Action::TrimNoseDown, Action::FlapsUp,
                             Action::FlapsDown, Action::GearToggle, Action::MapToggle}) {
                flightActions.pressed[static_cast<int>(a)] = false;
                flightActions.down[static_cast<int>(a)] = false;
            }
        }
        if (input.quitRequested()) break;
        // H5: an open menu consumes the input (X/Y/Start are menu buttons there); otherwise the
        // Pause button opens the pause menu.
        if (menu.active()) {
            const MenuResult res = menu.update(menuInputFromActions(actions));
            if (!res.message.empty()) messages.post(res.message.c_str());
            if (res.settingsChanged) applySettings();
            switch (res.action) {
            case MenuAction::StartFlight: startOrReport(res.scenario); break;
            case MenuAction::RestartFlight: startOrReport(Scenario(sim.scenario())); break;
            case MenuAction::ResumeFlight: menu.close(); break;
            case MenuAction::Quit: quitRequested = true; break;
            case MenuAction::None: break;
            }
        } else {
            Hud::applyActions(flightActions, hudOptions, &messages);  // H2: Action::HudDeclutter
            if (actions.pressed[static_cast<int>(Action::Pause)]) {
                mapShown = false;
                menu.openPause();
            }
        }
        if (quitRequested) break;
        state = !menu.active() ? AppState::Flying
                               : (menu.fullScreen() ? AppState::Menu : AppState::Paused);
        sim.setPaused(state != AppState::Flying);

        if (state == AppState::Flying) {
            if (interactive)
                pilotControls.update(flightActions, sim.profile(), sim.aircraft(), dtS, sim.pilot(),
                                     &messages);  // P2
            sim.advance(dtS);
        }
        if (audioEngine.deviceOpen()) {  // S1: sim state -> audio params and one-shot events
            const AircraftState& as = sim.aircraft();
            audio::AudioFrameInput ain;
            ain.state = &as;
            ain.profile = &sim.profile();
            const AutopilotState& apState = sim.autopilotState();
            ain.ap = &apState;
            ain.stallWarning = (activeWarnings(as, sim.finalControls(), sim.profile(), hudOptions) &
                                warningBit(WarningId::Stall)) != 0;
            ain.minimumsAlert = navHud.consumeMinimumsAlert();
            ain.paused = state != AppState::Flying;
            ain.volumePct = settings.volumePct;  // live: H5's settings menu edits it
            ain.dtS = static_cast<float>(dtS);
            audioEvents.clear();
            audioControl.update(ain, audioParams, audioEvents);
            // X2 HOOK: append GPWS callout tones (audio::makeTone) to audioEvents here.
            audioEngine.update(audioParams, audioEvents);
        }
        // W1: once per frame (also while paused, so tiles keep arriving).
        const AircraftState view = viewState();  // W2
        if (terrainSvc) terrainSvc->update(view.latDeg, view.lonDeg);

        // P2: log new feedback messages (until the HUD draws them)
        shownMessages = std::max(shownMessages, messages.totalPosted() - messages.size());
        for (; shownMessages < messages.totalPosted(); ++shownMessages)
            LOG_INFO(
                "%s",
                messages.newest(static_cast<int>(messages.totalPosted() - shownMessages - 1)).text);

        int w = 0, h = 0;
        window.drawableSize(w, h);
        camera.setViewport(w, h);
        camera.setPoseFromAircraft(view);
        atmosphere.setObserver(camera.eyeGeo().latDeg, camera.eyeGeo().lonDeg,  // W4
                               ftToM(view.altAglFt));
        renderer.render(camera, w, h);
        airports.render(camera, w, h, sim.aircraft().simTimeS);  // W5: after the ground
        {  // H2: HUD every frame (H1's --glyph-sheet debug view replaces it when requested)
            hudOptions.paused = false;  // H5: the pause menu says PAUSED itself
            hudOptions.magVarDeg =
                sim.navDb().magVarDeg(sim.aircraft().latDeg, sim.aircraft().lonDeg);
            {  // H3: nav / autopilot state for the overlay (or a canned one with --hud-fake)
                NavState hudNav = sim.navState();
                AutopilotState hudAp = sim.autopilotState();
                std::uint32_t fakeWarn = 0;
                if (!opt.hudFake.empty() && hudFake(opt.hudFake, frame, sim.aircraft(),
                                                    hudOptions.magVarDeg, hudNav, hudAp, fakeWarn))
                    hudOptions.extraWarnings = fakeWarn;
                else
                    hudOptions.extraWarnings =
                        hudAp.disconnectWarning ? warningBit(WarningId::ApOff) : 0;
                navHud.setStates(hudNav, hudAp);
            }
            hud2d.begin(w, h);
            if (opt.glyphSheet)
                drawGlyphSheet(hud2d);  // H1
            else if (mapShown)          // H4
                mapPage.draw(hud2d, sim.aircraft(), sim.navState(),
                             sim.navigator().hasPlan() ? &sim.navigator().plan() : nullptr,
                             sim.navDb(), hudOptions.magVarDeg);
            else if (state != AppState::Menu)
                hud.draw(hud2d, camera, sim.aircraft(), sim.finalControls(), sim.profile(),
                         messages, hudOptions);
            hud2d.end();
            if (menu.active()) {  // H5: own batch, so its panels cover the HUD strokes beneath
                hud2d.begin(w, h);
                menu.draw(hud2d);
                hud2d.end();
            }
        }
        ++frame;

        if (frames > 0 && frame >= frames) {
            if (!opt.screenshotPath.empty() &&
                !gfx::saveScreenshotPng(window.gl(), w, h, opt.screenshotPath))
                return 1;
            printSummary(sim, sim.aircraft().simTimeS);
            if (terrainRenderer) LOG_INFO("%s", terrainRenderer->stats().summary().c_str());  // W2
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
                if (terrainRenderer)
                    LOG_INFO("%s", terrainRenderer->stats().summary().c_str());  // W2
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
    const Airport* startAirport = navDb.findAirport(scenario.airportIdent);
    if (startAirport) terrain.setElevationM(static_cast<float>(ftToM(startAirport->elevFt)));

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
    // W5: the start airport's runway pads exist with or without a window (AirportRenderer keeps
    // them in sync with the active airport set afterwards).
    if (startAirport) world::registerAirportFlatten(*startAirport, terrainQuery);

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

    // H5: no --scenario / --resume on an interactive run (the device launch) shows the main menu.
    const bool startInMenu = interactiveRun && options_.scenarioPath.empty() && !options_.resume;
    bool flightBegun = false;  // a menu-started flight exists (else the default one stays unsaved)
    const BeginFlightFn beginFlight = [&](const Scenario& sc, std::string& err) {
        if (const Airport* ap = navDb.findAirport(sc.airportIdent)) {
            terrain.setElevationM(static_cast<float>(ftToM(ap->elevFt)));
            world::registerAirportFlatten(*ap, terrainQuery);
        }
        if (!sim.init(sc, "", err)) return false;
        flightBegun = true;
        if (terrainSvc) {
            terrainSvc->prefetch(sim.startCondition().latDeg, sim.startCondition().lonDeg,
                                 15000.0f);
            terrainSvc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
        }
        return true;
    };
    const int rc = options_.headless ? runHeadless(sim, options_, terrainSvc.get())
                                     : runWindowed(sim, options_, paths, terrainSvc.get(), settings,
                                                   beginFlight, startInMenu);
    if (rc == 0 && interactiveRun && (!startInMenu || flightBegun)) {  // P3: remember the flight
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
