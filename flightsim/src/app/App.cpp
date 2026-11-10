#include "app/App.h"

#include <SDL.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <functional>
#include <memory>

#include "app/FlightLog.h"      // U2
#include "app/FrameStats.h"     // X3
#include "app/GpuClock.h"       // X3c
#include "app/RenderQuality.h"  // X3
#include "app/Settings.h"       // P3
#include "app/Simulation.h"
#include "audio/AudioControl.h"  // S1
#include "audio/AudioEngine.h"   // S1
#include "audio/Callouts.h"      // S2
#include "audio/VoiceBank.h"     // S2
#include "core/Baro.h"           // X1
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
#include "hud/GpwsMonitor.h"           // X2
#include "hud/Hud.h"                   // H2
#include "hud/HudLogic.h"              // X1
#include "hud/MiniNav.h"               // H7
#include "hud/NavHud.h"                // H3
#include "hud/NavHudFake.h"            // H3
#include "hud/Renderer2D.h"            // H1
#include "hud/Renderer2DGlyphSheet.h"  // H1
#include "input/InputSystem.h"         // P2
#include "input/LookControl.h"         // I2
#include "input/PilotControls.h"       // P2
#include "nav/NavDb.h"
#include "platform/Window.h"
#include "sim/FuelPlan.h"                  // F6
#include "ui/MapPage.h"                    // H4
#include "ui/MenuSystem.h"                 // H5
#include "world/AirportGeometry.h"         // W5
#include "world/AirportRenderer.h"         // W5
#include "world/Atmosphere.h"              // W4
#include "world/Clouds.h"                  // W7
#include "world/ExternalView.h"            // V1
#include "world/Sky.h"                     // W4
#include "world/TerrainRenderer.h"         // W2
#include "world/Weather.h"                 // X1
#include "world/WorldOverlay.h"            // W6
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
    GpwsMonitor gpws;  // X2
    for (long i = 0; i < steps; ++i) {
        sim.stepOnce();
        // X2: GPWS-lite events are logged with their time (no HUD or audio in headless runs).
        gpws.update(sim.aircraft(), sim.profile(), &sim.navState(), sim.terrain(), &sim.navDb());
        for (GpwsEvent ev; gpws.consumeEvent(ev);)
            LOG_INFO("GPWS t=%.1f s: %s (agl %.0f ft)", sim.aircraft().simTimeS, ev.text,
                     sim.aircraft().altAglFt);
        // W1: integrate finished tiles and keep the load ring on the aircraft.
        if (terrainSvc) terrainSvc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
        // I1: [scenario] end_agl_ft: stop on the way down (a coupled-approach demo ends before the
        // AP disconnect hands the landing to nobody).
        const float endAglFt = sim.scenario().endAglFt;
        if (endAglFt > 0 && sim.aircraft().altAglFt <= endAglFt && sim.aircraft().vsFpm < 0 &&
            !sim.aircraft().wowAny) {
            LOG_INFO("scenario end: descended through %.0f ft AGL", endAglFt);
            break;
        }
    }
    printSummary(sim, sim.aircraft().simTimeS);
    return 0;
}

// H5: (re)starts a flight from a menu-built scenario: flat-terrain elevation, runway pads, the
// simulation and the terrain prefetch. Returns false with `error` when the simulation refuses.
using BeginFlightFn = std::function<bool(const Scenario&, std::string&)>;

// U2: called once per frame after the simulation step (the flight log recorder).
using FrameHookFn = std::function<void()>;

int runWindowed(Simulation& sim, const CliOptions& opt, const Paths& paths,
                terrain::TerrainService* terrainSvc, Settings& settings,
                const BeginFlightFn& beginFlight, bool startInMenu, const FrameHookFn& onFrame) {
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
    // X1: weather visibility drives the fog (a flight without any keeps the default).
    auto setWeatherVisibility = [&]() {
        const world::WeatherConditions& wc = sim.weather().conditions();
        atmosphere.setVisibilityM(wc.hasVisibility ? world::fogVisibilityM(wc)
                                                   : world::Atmosphere::kDefaultVisibilityM);
    };
    setWeatherVisibility();
    world::Sky sky(window.gl(), window.flavor());
    if (!sky.init()) return 1;
    renderer.setAtmosphere(&atmosphere, &sky);
    // W2: with a terrain pack the TerrainRenderer replaces the flat ground grid. The env hooks
    // FSIM_W2_CAMERA / FSIM_W2_VISIBILITY_KM override the view pose and fog for screenshots.
    const world::TerrainDebugOverrides dbg = world::readTerrainDebugEnv();
    if (dbg.visibilityKm > 0) atmosphere.setVisibilityM(dbg.visibilityKm * 1000.0f);
    std::unique_ptr<world::TerrainRenderer> terrainRenderer;
    // X3: the graphics_quality preset (the FSIM_W2_PX_PER_CELL hook still wins for the LOD).
    GraphicsQuality appliedQuality = settings.graphicsQuality;
    auto applyGraphicsQuality = [&]() {
        appliedQuality = settings.graphicsQuality;
        if (!terrainRenderer) return;
        world::TerrainRenderConfig tc = terrainRenderer->config();
        applyRenderQuality(renderQualityFor(settings.graphicsQuality), tc);
        if (dbg.pixelsPerCell > 0) tc.lod.pixelsPerCell = dbg.pixelsPerCell;
        terrainRenderer->setConfig(tc);
        LOG_INFO("graphics quality %s: terrain %.1f px/cell, %d uploads/frame, detail %s",
                 graphicsQualityLabel(settings.graphicsQuality), tc.lod.pixelsPerCell,
                 tc.uploadsPerFrame, world::detailNoiseName(tc.detailNoise));
    };
    if (terrainSvc) {
        terrainRenderer =
            std::make_unique<world::TerrainRenderer>(window.gl(), window.flavor(), *terrainSvc);
        if (terrainRenderer->init()) {
            renderer.setTerrain(terrainRenderer.get());
            applyGraphicsQuality();
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
    world::WorldOverlay worldOverlay(window.gl(), window.flavor());  // W6
    if (!worldOverlay.init()) LOG_WARN("world overlay: shader failed; markers stay hidden");
    world::Clouds clouds(window.gl(), window.flavor());  // W7
    if (!clouds.init()) LOG_WARN("clouds: shader failed; no cloud layers");
    // W7: layers from the scenario, the clouds setting or the METAR, above the start airport.
    auto setClouds = [&]() {
        const Airport* ap = sim.navDb().findAirport(sim.scenario().airportIdent);
        const float fieldFt = ap ? static_cast<float>(ap->elevFt) : 0.0f;
        const world::CloudDeck deck = world::resolveClouds(
            sim.scenario().clouds, sim.metar(), settings.clouds, settings.cloudBaseFt, fieldFt);
        clouds.setDeck(deck, ap ? ap->latDeg : sim.aircraft().latDeg,
                       ap ? ap->lonDeg : sim.aircraft().lonDeg);
        if (deck.count > 0)
            LOG_INFO("clouds (%s): %d layer(s), lowest %s %.0f-%.0f ft", deck.source.c_str(),
                     deck.count, world::cloudCoverName(deck.layers[0].cover),
                     deck.layers[0].baseFtMsl, deck.layers[0].topFtMsl);
    };
    setClouds();
    world::ExternalView externalView(window.gl(), window.flavor());  // V1
    if (!externalView.init()) LOG_WARN("external view: shader failed; no aircraft model");
    std::string externalViewProfile;  // profile id the model was built for
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
    // I2: L2 look layer -> camera look angles. FSIM_LOOK_DEG=yaw,pitch holds a look for
    // screenshots (like the FSIM_W2_* hooks).
    LookControl look;
    float lookHoldYawDeg = 0, lookHoldPitchDeg = 0;
    if (const char* env = std::getenv("FSIM_LOOK_DEG"))
        std::sscanf(env, "%f,%f", &lookHoldYawDeg, &lookHoldPitchDeg);
    MessageQueue messages;
    Hud hud;                // H2: flight HUD, drawn every frame after the 3D scene
    HudOptions hudOptions;  // H2: declutter level, color, paused flag, magnetic variation
    GpwsMonitor gpws;       // X2: ground proximity warnings and radio altitude callouts
    MapPage mapPage;        // H4: moving map (MapToggle), replaces the HUD while shown
    bool mapShown = opt.mapZoomNm > 0;
    if (mapShown) mapPage.setZoomNm(static_cast<float>(opt.mapZoomNm));
    if (opt.mapTrackUp) mapPage.setOrientation(MapOrientation::TrackUp);
    NavHud navHud;    // H3: FMA, nav box, CDI/LOC/GS, marks, bugs, FD, minimums
    MiniNav miniNav;  // H7: track-up route inset, off unless settings.hudMiniNav
    hud.setOverlay(&navHud);
    // H2: P3 settings give the starting HUD color and declutter level.
    hudOptions.baroInHg = settings.pressureUnits == PressureUnits::InHg;
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
    menuEnv.flightLogPath = paths.conf("flightlog.csv");  // U2
    menuEnv.menuPlanPath = paths.conf("menu_plan.fms");
    menuEnv.navDir = paths.data("nav");
    menuEnv.controlsDefaultsPath = paths.asset("assets/controls_default.ini");
    menuEnv.controlsUserPath = paths.conf("controls.ini");
    menuEnv.helpDir = paths.asset("assets/help");
    menuEnv.attributionPath = paths.data("ATTRIBUTION.txt");
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
    std::vector<audio::AudioEvent> gpwsToneBuf;  // X2: GPWS tones for S1, reused per frame
    LowFuelMonitor lowFuel;                      // F6
    gpwsToneBuf.reserve(8);
    // S2: GPWS voice clips (missing clips fall back to the tones).
    audio::VoiceBank voiceBank;
    {
        const int n = voiceBank.loadDir(paths.asset("assets/audio/voice"));
        LOG_INFO("voice: %d of %d GPWS clips (%zu KiB)", n, audio::kVoiceClipCount,
                 voiceBank.bytes() / 1024);
    }
    audioEngine.setVoiceBank(&voiceBank);
    audioControl.setVoiceBank(settings.gpwsVoice ? &voiceBank : nullptr);
    if (interactive)
        audioEngine.init(
            audio::audioBufferSamples(settings.audioBufferSamples, isPortMasterPlatform()));

    AppState state = AppState::Flying;
    bool quitRequested = false;
    // H5: applies edited settings to the running game (HUD color, sticks, FOV, FPS display).
    // X3: per-pass frame timing, always collected (a few counter reads per frame). The window goes
    // to the log once a second with show_fps; the session table when the pause menu opens, every
    // 2 minutes, when the quality preset changes and at exit.
    const double perfFreq = static_cast<double>(SDL_GetPerformanceFrequency());
    FrameStats frameStats;
    PerfFrame perf;
    PerfTotals shownPerf;  // the last full second, for the on-screen readout
    std::uint64_t passStart = 0;
    long perfLoggedFrames = 0;
    double perfSessionClockS = 0;  // the table also goes to the log every 2 minutes, because
                                   // Select + Start (gptokeyb) can kill the game before its exit
    const bool perfSync = settings.perfGpuSync;
    if (perfSync) LOG_INFO("perf: perf_gpu_sync on (glFinish after each pass)");
    GpuClock gpuClock;  // X3c: Mali devfreq clock with the timing; absent on most desktops
    if (gpuClock.open()) LOG_INFO("%s", gpuClock.describe().c_str());
    auto passEnd = [&](PerfPass p) {
        if (perfSync) window.gl().Finish();
        const std::uint64_t t = SDL_GetPerformanceCounter();
        perf.setPass(p, static_cast<double>(t - passStart) * 1000.0 / perfFreq);
        passStart = t;
    };
    // The table is labelled with the quality preset (and the sync mode) it was measured with.
    auto logPerfSession = [&](long minNewFrames) {
        if (frameStats.sessionFrames() - perfLoggedFrames < std::max(1L, minNewFrames)) return;
        perfLoggedFrames = frameStats.sessionFrames();
        char label[32];
        std::snprintf(label, sizeof label, "%s%s", graphicsQualityLabel(appliedQuality),
                      perfSync ? ", gpu sync" : "");
        LOG_INFO("%s", frameStats.sessionSummary(label).c_str());
        if (gpuClock.available()) LOG_INFO("%s", gpuClock.describe().c_str());  // X3c
    };
    auto applySettings = [&]() {
        switch (settings.hudColor) {
        case HudColor::White: hudOptions.color = HudPalette::White; break;
        case HudColor::Amber: hudOptions.color = HudPalette::Amber; break;
        case HudColor::Green: hudOptions.color = HudPalette::Green; break;
        }
        hudOptions.baroInHg = settings.pressureUnits == PressureUnits::InHg;
        pilotControls.setStickTuning(settings.stickTuning());
        camera.setLens(settings.fovVerticalDeg);
        showFps = effectiveShowFps(opt, settings);  // X3: SHOW FPS OFF also hides the readout
        if (settings.graphicsQuality != appliedQuality) {  // X3: a new timing table per preset
            logPerfSession(1);
            frameStats.resetSession();
            perfLoggedFrames = 0;
            applyGraphicsQuality();
        }
        setClouds();                                                           // W7
        audioControl.setVoiceBank(settings.gpwsVoice ? &voiceBank : nullptr);  // S2
    };
    // X3: frames left out of the timing after the window opens or a flight starts (shader
    // compiles and the first tile uploads are load time, not the frame rate).
    int perfSkipFrames = 10;
    // H5: a flight (re)started: the ground grid, sky time and HUD follow the new airport.
    auto flightStarted = [&]() {
        perfSkipFrames = 10;   // X3
        look.reset();          // I2
        externalView.reset();  // V1
        lowFuel.reset();       // F6
        if (const Airport* ap = sim.navDb().findAirport(sim.scenario().airportIdent))
            renderer.setWorld(*ap, static_cast<float>(world::airportLowestFlatElevM(*ap)));
        setSkyTime();
        setWeatherVisibility();  // X1
        setClouds();             // W7
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
    std::uint64_t lastCounter = SDL_GetPerformanceCounter();
    double fpsClockS = 0;
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
        passStart = now;  // X3

        SDL_Event event;
        while (SDL_PollEvent(&event)) input.handleEvent(event);
        const ActionState& actions = input.update(dtS);  // P2
        messages.tick(dtS);
        // I2: a frame the menu owns gives the flight no buttons, also when the menu closes in it.
        const bool menuOwnsFrame = menu.active();
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
            const PadProbe& probe = input.padProbe();
            MenuSystem::PadProbeView view;
            view.valid = probe.valid;
            view.axis = probe.axis;
            view.sdlCode = probe.sdlCode;
            view.serial = probe.serial;
            menu.setPadProbe(view);
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
                logPerfSession(300);  // X3: about 10 s new; usually opened before quitting
            }
        }
        if (quitRequested) break;
        state = !menu.active() ? AppState::Flying
                               : (menu.fullScreen() ? AppState::Menu : AppState::Paused);
        sim.setPaused(state != AppState::Flying);

        if (state == AppState::Flying) {
            // I2: the press that closed a menu stays in the menu. On the pad A is MenuAccept and
            // GearToggle, so resuming or starting a flight with A used to move the gear.
            ActionState cmdActions = actions;
            if (menuOwnsFrame) {
                clearButtons(flightActions);
                clearButtons(cmdActions);
            }
            if (interactive)
                pilotControls.update(flightActions, sim.profile(), sim.aircraft(), dtS, sim.pilot(),
                                     &messages);  // P2
            // I2: L2 look layer (frozen while a menu is open, eases back when L2 is released).
            if (interactive) look.update(flightActions, dtS);
            // N6: AP / FD / A/T engage, mode and bug actions.
            if (interactive)
                sim.autopilot().handleActions(cmdActions, sim.aircraft(), sim.navState(), dtS);
            if (interactive &&
                cmdActions.pressed[static_cast<int>(Action::OverlayPreset)]) {  // W6: L3 presets
                world::OverlayToggles layers = settings.overlayToggles();
                const int preset = world::overlayCycle(layers);
                settings.setOverlayToggles(layers);
                if (!menuEnv.settingsPath.empty() && !settings.save(menuEnv.settingsPath))
                    LOG_WARN("settings: cannot write %s", menuEnv.settingsPath.c_str());
                messages.postf("OVERLAYS %s", world::overlayPresetName(preset));
            }
            if (interactive && cmdActions.pressed[static_cast<int>(Action::ViewToggle)]) {  // V1
                settings.externalView = !settings.externalView;
                externalView.reset();
                look.reset();
                if (!menuEnv.settingsPath.empty() && !settings.save(menuEnv.settingsPath))
                    LOG_WARN("settings: cannot write %s", menuEnv.settingsPath.c_str());
                messages.post(settings.externalView ? "VIEW CHASE" : "VIEW COCKPIT");
            }
            if (interactive) {  // X1: altimeter setting (R1 + D-pad up/down, R1 + Select = STD)
                const bool inHg = settings.pressureUnits == PressureUnits::InHg;
                auto pr = [&](Action a) { return cmdActions.pressed[static_cast<int>(a)]; };
                if (pr(Action::BaroStd)) sim.setAltimeterSettingHpa(static_cast<float>(kStdQnhHpa));
                const int steps = (pr(Action::BaroInc) ? 1 : 0) - (pr(Action::BaroDec) ? 1 : 0);
                if (steps != 0) sim.stepAltimeter(steps, inHg);
                if (pr(Action::BaroStd) || steps != 0) {
                    char t[16];
                    formatAltimeterSetting(t, sizeof t, sim.altimeterSettingHpa(), inHg);
                    messages.postf("ALTIMETER %s", t);
                }
            }
            if (interactive) {  // I2: R1 + Y nav source, L1 + Select direct-to
                auto pr = [&](Action a) { return cmdActions.pressed[static_cast<int>(a)]; };
                Navigator& nav = sim.navigator();
                if (pr(Action::NavSourceCycle)) {
                    const NavSource shown = sim.navState().source;
                    const NavSourceSelect next =
                        nextNavSourceSelect(shown, nav.locAvailable(), nav.vorTuned());
                    nav.setSourceSelect(next);
                    const char* text = "NAV SOURCE GPS";
                    if (next == NavSourceSelect::Loc) text = "NAV SOURCE LOC";
                    if (next == NavSourceSelect::Vor) text = "NAV SOURCE VOR";
                    if (shown != NavSource::Loc && shown != NavSource::Vor &&
                        next == NavSourceSelect::Auto)
                        text = "NO LOC OR VOR TUNED";
                    messages.post(text);
                }
                if (pr(Action::DirectTo)) {
                    const int leg = nav.hasPlan() ? nav.plan().activeLeg : -1;
                    if (leg >= 1 && leg < static_cast<int>(nav.plan().legs.size()) &&
                        nav.directToLeg(leg)) {
                        // The plan is now [PPOS, DF target, ...] with the target active.
                        const FlightPlan& plan = nav.plan();
                        messages.postf(
                            "DIRECT TO %s",
                            plan.legs[static_cast<size_t>(plan.activeLeg)].to.ident.c_str());
                    } else {  // no plan (or it is flown out): pick a waypoint from the list
                        mapShown = false;
                        menu.openDirectTo();
                    }
                }
            }
            sim.advance(dtS);
        }
        {  // F6: LOW FUEL at 30 and 15 minutes of endurance
            const AircraftState& as = sim.aircraft();
            const int mins = lowFuel.update(enduranceMin(as.fuelTotalLbs, as.fuelFlowLbsPerHr),
                                            !as.wowAny && as.valid);
            if (mins > 0) messages.postf("LOW FUEL %d MIN", mins);
        }
        if (onFrame) onFrame();  // U2: flight log
        gpwsToneBuf.clear();
        {  // X2: GPWS-lite; text callouts go to the message queue, tones to S1
            gpws.update(sim.aircraft(), sim.profile(), &sim.navState(), sim.terrain(),
                        &sim.navDb());
            for (GpwsEvent ev; gpws.consumeEvent(ev);) {
                if (ev.kind == GpwsEventKind::Callout || ev.kind == GpwsEventKind::TooLowGear ||
                    ev.kind == GpwsEventKind::TooLowFlaps)
                    messages.post(ev.text);
                // S2: spoken when the clip is loaded (and the voice is on), else the X2 tone.
                audio::AlertKind kind = audio::AlertKind::Callout;
                switch (ev.kind) {
                case GpwsEventKind::Callout: kind = audio::AlertKind::Callout; break;
                case GpwsEventKind::SinkRate: kind = audio::AlertKind::SinkRate; break;
                case GpwsEventKind::Terrain: kind = audio::AlertKind::Terrain; break;
                case GpwsEventKind::PullUp: kind = audio::AlertKind::PullUp; break;
                case GpwsEventKind::TooLowGear: kind = audio::AlertKind::TooLowGear; break;
                case GpwsEventKind::TooLowFlaps: kind = audio::AlertKind::TooLowFlaps; break;
                }
                gpwsToneBuf.push_back(
                    audio::alertEvent(kind, ev.valueFt, settings.gpwsVoice ? &voiceBank : nullptr));
            }
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
            if (!ain.paused)  // X2: GPWS tones
                audioEvents.insert(audioEvents.end(), gpwsToneBuf.begin(), gpwsToneBuf.end());
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
        // I2: the look angles (or the FSIM_LOOK_DEG screenshot hook).
        if (lookHoldYawDeg != 0 || lookHoldPitchDeg != 0)
            camera.setPoseFromAircraft(view, static_cast<float>(degToRad(lookHoldYawDeg)),
                                       static_cast<float>(degToRad(lookHoldPitchDeg)));
        else
            camera.setPoseFromAircraft(view, look.yawRad(), look.pitchRad());
        // V1: chase view (not on the map page or behind a full-screen menu).
        const bool chase = settings.externalView && state != AppState::Menu && !mapShown;
        if (sim.profile().id != externalViewProfile) {
            externalView.setAircraft(sim.profile());
            externalViewProfile = sim.profile().id;
        }
        if (chase) {
            const bool held = lookHoldYawDeg != 0 || lookHoldPitchDeg != 0;
            externalView.placeCamera(
                camera, view, held ? static_cast<float>(degToRad(lookHoldYawDeg)) : look.yawRad(),
                held ? static_cast<float>(degToRad(lookHoldPitchDeg)) : look.pitchRad(),
                &sim.terrain(), dtS);
        }
        atmosphere.setObserver(camera.eyeGeo().latDeg, camera.eyeGeo().lonDeg,  // W4
                               ftToM(view.altAglFt));
        passEnd(PerfPass::Update);  // X3
        renderer.render(camera, w, h);
        passEnd(PerfPass::Terrain);
        airports.render(camera, w, h, sim.aircraft().simTimeS);  // W5: after the ground
        passEnd(PerfPass::Airports);
        // W7 clouds (blended, no depth writes) before the chase model, so the aircraft stays in
        // view inside a cloud; a layer surface is always farther away than the model.
        const bool cloudsOn = state != AppState::Menu && !mapShown;
        if (cloudsOn) clouds.render(camera, atmosphere);
        passEnd(PerfPass::Clouds);
        if (chase) externalView.render(camera, view, atmosphere.light());  // V1: one draw
        passEnd(PerfPass::Model);
        {  // W6: route, vertical path, airports, runway outlines (at most 4 draws)
            // H10: route and path in AUTO show only at FULL declutter (a Y press applies next frame).
            const world::OverlayToggles layers = world::resolveOverlay(
                settings.overlayToggles(), hudOptions.declutter == Declutter::Full);
            const bool layersOn = layers.route || layers.vpath || layers.airports || layers.runways;
            const bool showOverlays =
                layersOn && state != AppState::Menu && !mapShown && !opt.glyphSheet;
            if (showOverlays) {
                worldOverlay.render(
                    camera, sim.navigator().hasPlan() ? &sim.navigator().plan() : nullptr,
                    &sim.navDb(), &sim.terrain(), layers, settings.overlayAirportRadiusNm,
                    sim.navState().turnRadiusNm);  // N10: fly-by arcs
            } else {
                worldOverlay.clearFrame();
            }
        }
        passEnd(PerfPass::Overlay);
        int hudDraws = 0;  // X3
        {  // H2: HUD every frame (H1's --glyph-sheet debug view replaces it when requested)
            hudOptions.paused = false;        // H5: the pause menu says PAUSED itself
            hudOptions.externalView = chase;  // V1: status strip and messages only
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
                hudOptions.extraWarnings |= gpws.warnings();  // X2
                navHud.setStates(hudNav, hudAp);
            }
            hud2d.begin(w, h);
            if (opt.glyphSheet)
                drawGlyphSheet(hud2d);  // H1
            else if (mapShown)          // H4
                mapPage.draw(hud2d, sim.aircraft(), sim.navState(),
                             sim.navigator().hasPlan() ? &sim.navigator().plan() : nullptr,
                             sim.navDb(), hudOptions.magVarDeg);
            else if (state != AppState::Menu) {
                hud.draw(hud2d, camera, sim.aircraft(), sim.finalControls(), sim.profile(),
                         messages, hudOptions);
                if (!chase && miniNavShown(settings.hudMiniNav, hudOptions.declutter, false))
                    miniNav.draw(hud2d, sim.aircraft(), sim.navState(),
                                 sim.navigator().hasPlan() ? &sim.navigator().plan() : nullptr,
                                 settings.hudMiniNavRangeNm);
                worldOverlay.drawLabels(hud2d);  // W6
            }
            if (showFps && !opt.glyphSheet && state != AppState::Menu) {  // X3: the device has
                char line[48];  // no window title, so show_fps also puts the last second here
                if (shownPerf.frames > 0)
                    std::snprintf(line, sizeof line, "%.0f FPS %.1f MS", shownPerf.fps(),
                                  shownPerf.meanFrameMs());
                else
                    std::snprintf(line, sizeof line, "-- FPS");
                const Color c = colors::kWhite.withAlpha(0.85f);
                const HudRect& z = hudzones::kFpsReadout;  // X3c: a zone the chase layout avoids
                hud2d.text(hud2d.leftX() + z.x, z.y + 9, line, TextSize::S, c);
                if (shownPerf.frames > 0) {
                    std::snprintf(line, sizeof line, "CPU %.1f MS", shownPerf.meanCpuMs());
                    hud2d.text(hud2d.leftX() + z.x, z.y + 25, line, TextSize::S, c);
                }
            }
            hud2d.end();
            hudDraws = hud2d.stats().drawCalls;  // X3
            if (menu.active()) {  // H5: own batch, so its panels cover the HUD strokes beneath
                hud2d.begin(w, h);
                menu.draw(hud2d);
                hud2d.end();
                hudDraws += hud2d.stats().drawCalls;
            }
        }
        passEnd(PerfPass::Hud);
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
        passEnd(PerfPass::Swap);

        {  // X3: one frame of timing, filed under what it showed
            perf.frameMs = static_cast<double>(passStart - now) * 1000.0 / perfFreq;
            perf.scene = state != AppState::Flying ? PerfScene::Menu
                         : mapShown                ? PerfScene::Map
                         : chase                   ? PerfScene::Chase
                                                   : PerfScene::Cockpit;
            perf.sky = !cloudsOn                          ? PerfSky::Clear
                       : clouds.lastInsideAlpha() > 0.01f ? PerfSky::Inside
                       : clouds.lastDrawCalls() > 0       ? PerfSky::Layer
                                                          : PerfSky::Clear;
            const world::TerrainRenderStats* ts =
                terrainRenderer ? &terrainRenderer->stats() : nullptr;
            perf.terrainTriangles = ts ? ts->triangles : 0;
            perf.drawCalls = (ts ? ts->drawCalls : 0) + airports.stats().drawCalls +  // X3c
                             clouds.lastDrawCalls() + (chase ? externalView.lastDrawCalls() : 0) +
                             worldOverlay.lastDrawCalls() + hudDraws;
            if (perfSkipFrames > 0)
                --perfSkipFrames;
            else
                frameStats.add(perf);
        }
        perfSessionClockS += wallDtS;
        if (perfSessionClockS >= 120.0) {
            perfSessionClockS = 0;
            logPerfSession(300);
        }
        fpsClockS += wallDtS;
        if (fpsClockS >= 1.0) {
            if (showFps) {
                const AircraftState& s = sim.aircraft();
                char title[160];
                std::snprintf(title, sizeof title, "Pocket Vectors | %.0f fps | %.0f kt | %.0f ft",
                              frameStats.window().fps(), s.iasKt, s.altMslFt);
                window.setTitle(title);
                LOG_INFO("%s%s", frameStats.windowLine().c_str(),
                         gpuClock.windowTag().c_str());  // X3, X3c: the GPU clock
                if (terrainRenderer)
                    LOG_INFO("%s", terrainRenderer->stats().summary().c_str());  // W2
            }
            shownPerf = frameStats.window();  // X3: the on-screen readout shows the last second
            frameStats.resetWindow();
            fpsClockS = 0;
        }
    }
    logPerfSession(1);  // X3
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
    // X1: weather options from the settings (re-read for each menu-started flight below)
    auto applyWeatherOptions = [&]() {
        sim.setWeatherOptions(
            {settings.weatherMetar, settings.weatherGusts, settings.weatherTurbulence});
    };
    applyWeatherOptions();
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

    // U2: the flight log (interactive runs only, like last_flight.ini). A flight ends when the
    // next one starts, at QUIT or at a crash; a flight the game did not close (Select + Start)
    // is recovered from the open-flight file at the next start.
    FlightRecorder recorder;
    const std::string logPath = paths.conf("flightlog.csv");
    const std::string openLogPath = paths.conf("flightlog_open.csv");
    double recorderSimS = 0;
    if (interactiveRun && recoverOpenFlight(openLogPath, logPath))
        LOG_INFO("flight log: recorded a flight the game did not close (GAME CLOSED)");
    auto beginLoggedFlight = [&]() {
        if (!interactiveRun) return;
        recorder.begin(sim.scenario(), sim.profile().id, formatUtcNow(), sim.aircraft());
        recorderSimS = sim.aircraft().simTimeS;
    };
    auto endLoggedFlight = [&](FlightEnd why) {
        if (!recorder.active()) return;
        FlightLogEntry e = recorder.end(why);
        fillArrival(e, recorder.touchdownLatDeg(), recorder.touchdownLonDeg(), navDb);
        if (appendFlightLog(logPath, e))
            LOG_INFO("flight log: %s", flightLogRow(e).c_str());
        else
            LOG_WARN("flight log: cannot write %s", logPath.c_str());
        std::remove(openLogPath.c_str());
    };
    const FrameHookFn onFrame = [&]() {
        if (!recorder.active()) return;
        const AircraftState& s = sim.aircraft();
        const double dt = s.simTimeS - recorderSimS;  // sim time: nothing while paused
        recorderSimS = s.simTimeS;
        recorder.update(s, std::max(0.0, dt), dt <= 0.0);
        if (s.crashed) {
            endLoggedFlight(FlightEnd::Crash);
            return;
        }
        if (recorder.checkpointDue()) {  // every 60 s of flight and at each touchdown
            FlightLogEntry e = recorder.entry();
            e.outcome = "IN FLIGHT";
            if (e.landings > 0)
                fillArrival(e, recorder.touchdownLatDeg(), recorder.touchdownLonDeg(), navDb);
            if (!writeOpenFlight(openLogPath, e))
                LOG_WARN("flight log: cannot write %s", openLogPath.c_str());
            recorder.checkpointDone();
        }
    };

    const BeginFlightFn beginFlight = [&](const Scenario& sc, std::string& err) {
        endLoggedFlight(FlightEnd::NewFlight);  // U2: before the simulation forgets it
        if (const Airport* ap = navDb.findAirport(sc.airportIdent)) {
            terrain.setElevationM(static_cast<float>(ftToM(ap->elevFt)));
            world::registerAirportFlatten(*ap, terrainQuery);
        }
        applyWeatherOptions();  // X1
        if (!sim.init(sc, "", err)) return false;
        flightBegun = true;
        beginLoggedFlight();  // U2
        if (terrainSvc) {
            terrainSvc->prefetch(sim.startCondition().latDeg, sim.startCondition().lonDeg,
                                 15000.0f);
            terrainSvc->update(sim.aircraft().latDeg, sim.aircraft().lonDeg);
        }
        return true;
    };
    if (!startInMenu) beginLoggedFlight();  // U2: --scenario / --resume start flying at once
    const int rc = options_.headless ? runHeadless(sim, options_, terrainSvc.get())
                                     : runWindowed(sim, options_, paths, terrainSvc.get(), settings,
                                                   beginFlight, startInMenu, onFrame);
    endLoggedFlight(FlightEnd::Quit);                                  // U2
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
