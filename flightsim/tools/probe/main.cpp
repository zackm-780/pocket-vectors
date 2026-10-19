// Flight Sim device probe.
//
// A small standalone SDL2 + GLES2 program that answers every open question in docs/DEVICE.md section 2
// on the real RG40XXH, and writes probe_report.txt next to the binary. See README.md in this directory.
#include <SDL.h>

#include <algorithm>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <string>
#include <vector>

#include "Common.h"
#include "Context.h"
#include "CpuBench.h"
#include "GpuBench.h"
#include "InputTest.h"
#include "Report.h"
#include "SysInfo.h"

namespace {

using namespace probe;

constexpr const char* kProbeVersion = "1.0";

void printUsage() {
    std::printf(
        "Flight Sim device probe %s\n"
        "usage: flightsim_probe [options]\n"
        "  --out DIR            write probe_report.txt here (default: next to the binary)\n"
        "  --fullscreen         fullscreen window at the desktop resolution (device)\n"
        "  --window WxH         window size when not fullscreen (default 640x480)\n"
        "  --jsbsim-dir DIR     JSBSim data root with aircraft/ engine/ systems/ (default: <binary dir>/jsbsim)\n"
        "  --auto               never wait for a human: very short input test, no final screen\n"
        "  --selftest-input     inject synthetic controller events (verifies the logger without hardware)\n"
        "  --quick              shorter benchmarks (CI, emulation)\n"
        "  --cpu-only           no window, no SDL video/audio/input: system info + CPU benchmark only\n"
        "  --skip LIST          comma list of: input,gpu,cpu,sweep,audio\n"
        "  --input-scale F      multiply the input-test prompt timeouts by F\n"
        "  --hold SECONDS       how long the final screen stays (default 120, -1 = until a button is pressed)\n"
        "  --shot-dir DIR       save BMP screenshots of the probe screens (development)\n"
        "  --no-beep            do not play the short test tone\n"
        "  --help\n",
        kProbeVersion);
}

bool parseArgs(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&](std::string& out) {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "missing value for %s\n", a.c_str());
                return false;
            }
            out = argv[++i];
            return true;
        };
        std::string v;
        if (a == "--help" || a == "-h") {
            printUsage();
            return false;
        } else if (a == "--out") {
            if (!next(opt.outDir)) return false;
        } else if (a == "--fullscreen") {
            opt.fullscreen = true;
        } else if (a == "--window") {
            if (!next(v)) return false;
            if (std::sscanf(v.c_str(), "%dx%d", &opt.windowW, &opt.windowH) != 2) {
                std::fprintf(stderr, "bad --window value '%s' (want WxH)\n", v.c_str());
                return false;
            }
        } else if (a == "--jsbsim-dir") {
            if (!next(opt.jsbsimDir)) return false;
        } else if (a == "--auto") {
            opt.autoMode = true;
            opt.holdSeconds = 0.0;
        } else if (a == "--selftest-input") {
            opt.selfTestInput = true;
        } else if (a == "--quick") {
            opt.quick = true;
        } else if (a == "--cpu-only") {
            opt.cpuOnly = true;
        } else if (a == "--no-beep") {
            opt.noBeep = true;
        } else if (a == "--skip") {
            if (!next(v)) return false;
            opt.skipInput = v.find("input") != std::string::npos;
            opt.skipGpu = v.find("gpu") != std::string::npos;
            opt.skipCpu = v.find("cpu") != std::string::npos;
            opt.skipSweep = v.find("sweep") != std::string::npos;
            opt.skipAudio = v.find("audio") != std::string::npos;
        } else if (a == "--input-scale") {
            if (!next(v)) return false;
            opt.inputTimeScale = std::atof(v.c_str());
        } else if (a == "--hold") {
            if (!next(v)) return false;
            opt.holdSeconds = std::atof(v.c_str());
        } else if (a == "--shot-dir") {
            if (!next(opt.shotDir)) return false;
        } else {
            std::fprintf(stderr, "unknown option %s\n", a.c_str());
            printUsage();
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------------------------
// SDL, display and audio information
// ---------------------------------------------------------------------------------------------

void reportSdl(Ctx& ctx) {
    Report& report = *ctx.report;
    report.section("SDL");
    SDL_version compiled;
    SDL_version linked;
    SDL_VERSION(&compiled);
    SDL_GetVersion(&linked);
    report.kv("SDL compiled against", strf("%d.%d.%d", compiled.major, compiled.minor, compiled.patch));
    report.kv("SDL linked (runtime)", strf("%d.%d.%d", linked.major, linked.minor, linked.patch));
    report.kv("SDL revision", SDL_GetRevision());
    report.kv("platform", SDL_GetPlatform());
    std::string drivers;
    for (int i = 0; i < SDL_GetNumVideoDrivers(); ++i) {
        drivers += std::string(i != 0 ? ", " : "") + SDL_GetVideoDriver(i);
    }
    report.kv("video drivers compiled into SDL", drivers);
    const char* current = SDL_GetCurrentVideoDriver();
    report.kv("video driver in use", current != nullptr ? current : "NONE (video init failed)");
    std::string audio;
    for (int i = 0; i < SDL_GetNumAudioDrivers(); ++i) {
        audio += std::string(i != 0 ? ", " : "") + SDL_GetAudioDriver(i);
    }
    report.kv("audio drivers compiled into SDL", audio);
    report.fact(rows::kSdl, strf("SDL %d.%d.%d (built with %d.%d.%d), video driver '%s' (audio driver: see the Audio row)",
                                 linked.major, linked.minor, linked.patch, compiled.major, compiled.minor, compiled.patch,
                                 current != nullptr ? current : "NONE"));
}

void reportDisplays(Ctx& ctx) {
    Report& report = *ctx.report;
    report.section("DISPLAYS");
    const int n = SDL_GetNumVideoDisplays();
    report.kv("displays", strf("%d", n));
    for (int d = 0; d < n; ++d) {
        const char* name = SDL_GetDisplayName(d);
        SDL_Rect bounds{};
        SDL_GetDisplayBounds(d, &bounds);
        SDL_DisplayMode desktop{};
        SDL_GetDesktopDisplayMode(d, &desktop);
        SDL_DisplayMode current{};
        SDL_GetCurrentDisplayMode(d, &current);
        float ddpi = 0, hdpi = 0, vdpi = 0;
        const int dpiOk = SDL_GetDisplayDPI(d, &ddpi, &hdpi, &vdpi);
        report.line(strf("display %d \"%s\": bounds %dx%d at (%d,%d); desktop mode %dx%d @%d Hz fmt %s; current %dx%d @%d Hz; DPI %s",
                         d, name != nullptr ? name : "?", bounds.w, bounds.h, bounds.x, bounds.y, desktop.w, desktop.h,
                         desktop.refresh_rate, SDL_GetPixelFormatName(desktop.format), current.w, current.h,
                         current.refresh_rate,
                         dpiOk == 0 ? strf("%.0f/%.0f/%.0f", ddpi, hdpi, vdpi).c_str() : "unknown"));
        const int modes = SDL_GetNumDisplayModes(d);
        std::string list;
        for (int m = 0; m < modes && m < 40; ++m) {
            SDL_DisplayMode mode{};
            if (SDL_GetDisplayMode(d, m, &mode) == 0) {
                list += strf("%dx%d@%d ", mode.w, mode.h, mode.refresh_rate);
            }
        }
        report.line(strf("  %d display modes: %s%s", modes, list.c_str(), modes > 40 ? "..." : ""));
    }
}

// Writes a short quiet sine tone into `out`.
void makeTone(Sint16* out, int frames, int channels, int freqHz) {
    for (int i = 0; i < frames; ++i) {
        const double env = std::min({1.0, i / 400.0, (frames - i) / 400.0});
        const Sint16 s = static_cast<Sint16>(2400.0 * env * std::sin(6.2831853 * 440.0 * i / freqHz));
        for (int c = 0; c < channels; ++c) {
            out[i * channels + c] = s;
        }
    }
}

void probeAudio(Ctx& ctx) {
    Report& report = *ctx.report;
    report.section("AUDIO");
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        report.line(strf("SDL audio init FAILED: %s", SDL_GetError()));
        report.fact(rows::kAudio, strf("SDL audio init failed: %s", SDL_GetError()));
        return;
    }
    const char* driver = SDL_GetCurrentAudioDriver();
    report.kv("audio driver in use", driver != nullptr ? driver : "none");
    const int devices = SDL_GetNumAudioDevices(0);
    report.kv("playback devices", strf("%d", devices));
    for (int i = 0; i < devices; ++i) {
        const char* n = SDL_GetAudioDeviceName(i, 0);
        report.line(strf("  device %d: %s", i, n != nullptr ? n : "?"));
    }
    SDL_AudioSpec want;
    SDL_zero(want);
    want.freq = 44100;
    want.format = AUDIO_S16SYS;
    want.channels = 2;
    want.samples = 1024;
    SDL_AudioSpec have;
    SDL_zero(have);
    const SDL_AudioDeviceID dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, SDL_AUDIO_ALLOW_ANY_CHANGE);
    if (dev == 0) {
        report.line(strf("SDL_OpenAudioDevice FAILED: %s", SDL_GetError()));
        report.fact(rows::kAudio, strf("driver %s, open failed: %s", driver != nullptr ? driver : "none", SDL_GetError()));
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return;
    }
    report.line(strf("requested %d Hz S16 2ch %d samples; obtained %d Hz, format 0x%04x (%s), %d ch, %d samples buffer",
                     want.freq, want.samples, have.freq, have.format, SDL_AUDIO_ISFLOAT(have.format) ? "float" : "int",
                     have.channels, have.samples));
    std::string beep = "no tone played";
    if (!ctx.opt.noBeep && have.format == AUDIO_S16SYS && have.channels >= 1 && have.channels <= 8) {
        const int frames = have.freq / 4;
        std::vector<Sint16> tone(static_cast<size_t>(frames) * static_cast<size_t>(have.channels));
        makeTone(tone.data(), frames, have.channels, have.freq);
        if (SDL_QueueAudio(dev, tone.data(), static_cast<Uint32>(tone.size() * sizeof(Sint16))) == 0) {
            SDL_PauseAudioDevice(dev, 0);
            const double start = nowS();
            while (nowS() - start < 0.45) {
                ctx.pumpEvents();
                SDL_Delay(5);
            }
            beep = "played a 0.25 s 440 Hz tone (low volume): did you hear it?";
        } else {
            beep = strf("SDL_QueueAudio failed: %s", SDL_GetError());
        }
    }
    report.line(beep);
    report.fact(rows::kAudio, strf("driver %s: obtained %d Hz, %d ch, format 0x%04x, buffer %d samples (asked 44100 S16 stereo); %s",
                                   driver != nullptr ? driver : "none", have.freq, have.channels, have.format, have.samples,
                                   beep.c_str()));
    SDL_CloseAudioDevice(dev);
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

// ---------------------------------------------------------------------------------------------
// Final screen
// ---------------------------------------------------------------------------------------------

void finalScreen(Ctx& ctx) {
    Report& report = *ctx.report;
    report.setStatus("complete");
    report.save();
    if (!ctx.glOk || ctx.opt.holdSeconds == 0.0) {
        return;
    }
    const double start = nowS();
    const double armedAfterS = 2.0;  // ignore stray presses right after the last prompt
    bool shotRequested = false;
    while (!ctx.quitRequested) {
        SDL_Event e;
        while (SDL_PollEvent(&e) != 0) {
            if (ctx.handleCommonEvent(e)) {
                continue;
            }
            const bool press = e.type == SDL_CONTROLLERBUTTONDOWN || e.type == SDL_JOYBUTTONDOWN ||
                               (e.type == SDL_KEYDOWN && e.key.repeat == 0);
            if (press && nowS() - start > armedAfterS) {
                return;
            }
        }
        if (ctx.opt.holdSeconds > 0.0 && nowS() - start > ctx.opt.holdSeconds) {
            return;
        }
        ctx.beginUiFrame();
        const float s = static_cast<float>(ctx.uiScale());
        TextRenderer& tr = ctx.textRenderer;
        tr.text(8 * s, 6 * s, 2 * s, Color{0.4f, 1.0f, 0.5f, 1.0f}, "PROBE COMPLETE");
        tr.text(8 * s, 26 * s, s, Color{0.85f, 0.95f, 0.85f, 1.0f}, "report: " + report.path());
        const int cols = std::max(10, static_cast<int>(static_cast<float>(ctx.widthPx) / (8.0f * s)) - 2);
        float y = 40 * s;
        for (const auto& fact : report.facts()) {
            std::string head = fact.first;
            if (static_cast<int>(head.size()) > cols) head.resize(static_cast<size_t>(cols));
            tr.text(8 * s, y, s, Color{1.0f, 0.85f, 0.2f, 1.0f}, head);
            y += 8 * s;
            std::string v = fact.second;
            for (int line = 0; line < 2 && !v.empty(); ++line) {
                const std::string part = v.substr(0, static_cast<size_t>(cols - 2));
                tr.text(16 * s, y, s, Color{0.85f, 0.95f, 0.85f, 1.0f}, part);
                y += 8 * s;
                v = v.size() > part.size() ? v.substr(part.size()) : std::string();
            }
        }
        tr.text(8 * s, static_cast<float>(ctx.heightPx) - 12 * s, s, Color{0.55f, 0.65f, 0.75f, 1.0f},
                "Press A/B/Start to exit (or SELECT+START). Send back probe_report.txt");
        if (!shotRequested) {
            shotRequested = true;
            ctx.requestShot("final");
        }
        ctx.endUiFrame();
    }
}

}  // namespace

int main(int argc, char** argv) {
    Options opt;
    if (!parseArgs(argc, argv, opt)) {
        return argc > 1 && (std::strcmp(argv[1], "--help") == 0 || std::strcmp(argv[1], "-h") == 0) ? 0 : 2;
    }
    const char* platform = std::getenv("FLIGHTSIM_PLATFORM");
    const char* probePlatform = std::getenv("PROBE_PLATFORM");
    if ((platform != nullptr && std::strcmp(platform, "portmaster") == 0) ||
        (probePlatform != nullptr && std::strcmp(probePlatform, "portmaster") == 0)) {
        opt.escapeQuits = false;  // gptokeyb may inject key events
    }
    if (opt.outDir.empty()) {
        opt.outDir = executableDir();
    }
    if (opt.autoMode && opt.holdSeconds != 0.0) {
        opt.holdSeconds = 0.0;
    }
    std::setvbuf(stdout, nullptr, _IOLBF, 0);

    Report report(opt.outDir + "/probe_report.txt");
    Ctx ctx;
    ctx.opt = opt;
    ctx.report = &report;

    try {
        report.section("PROBE");
        report.kv("probe version", kProbeVersion);
        std::string cmdline;
        for (int i = 0; i < argc; ++i) {
            cmdline += std::string(i != 0 ? " " : "") + argv[i];
        }
        report.kv("command line", cmdline);
        report.kv("report file", report.path());
        report.kv("platform env", strf("FLIGHTSIM_PLATFORM=%s PROBE_PLATFORM=%s", platform != nullptr ? platform : "(unset)",
                                       probePlatform != nullptr ? probePlatform : "(unset)"));
        report.setStatus("running: system info");
        reportSystemInfo(report);

        if (opt.cpuOnly) {
            report.setStatus("running: cpu benchmark");
            runCpuBenchmark(report, opt);
            report.setStatus("complete (cpu-only run)");
            report.save();
            return 0;
        }

        // ---- SDL ----
        report.setStatus("running: SDL init");
        SDL_SetHint(SDL_HINT_JOYSTICK_ALLOW_BACKGROUND_EVENTS, "1");
        const int videoRc = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
        ctx.videoOk = videoRc == 0;
        if (!ctx.videoOk) {
            report.section("SDL VIDEO INIT FAILED");
            report.line(strf("SDL_Init(VIDEO) failed: %s", SDL_GetError()));
            report.line("continuing without a display: input logging, audio and CPU benchmark still run");
            SDL_Init(SDL_INIT_EVENTS);
        }
        if (SDL_InitSubSystem(SDL_INIT_JOYSTICK | SDL_INIT_GAMECONTROLLER) != 0) {
            report.line(strf("SDL joystick/controller init failed: %s", SDL_GetError()));
        }
        reportSdl(ctx);

        if (ctx.videoOk) {
            std::string error;
            report.setStatus("running: GL context");
            if (ctx.openWindow(GlRequest(), error)) {
                if (!error.empty()) {
                    report.line("note: " + error);
                }
                reportDisplays(ctx);
                reportGlInfo(ctx);
                reportLoadedLibraries(report);
            } else {
                report.section("OPENGL ES");
                report.line("WINDOW / GL CONTEXT CREATION FAILED:");
                report.line(error);
                report.fact(rows::kGl, "GL context creation failed: " + error);
                reportDisplays(ctx);
            }
        }

        ctx.phase = "audio";
        if (!opt.skipAudio) {
            report.setStatus("running: audio");
            probeAudio(ctx);
        } else {
            report.fact(rows::kAudio, "skipped");
        }

        report.setStatus("running: controllers");
        reportControllers(ctx);

        if (!opt.skipInput) {
            ctx.phase = "input test";
            report.setStatus("running: input test");
            runInputTest(ctx);
        } else {
            report.fact(rows::kTriggers, "skipped");
            report.fact(rows::kStickClicks, "skipped");
            report.fact(rows::kFaceButtons, "skipped");
            report.fact(rows::kMenuKey, "skipped");
        }

        if (!opt.skipCpu && !ctx.quitRequested) {
            ctx.phase = "cpu benchmark (JSBSim)";
            report.setStatus("running: cpu benchmark");
            ctx.drawStatusScreen("CPU benchmark running (about 15 s)...", 0.3);
            runCpuBenchmark(report, opt);
        }

        if (!opt.skipGpu && ctx.glOk && !ctx.quitRequested) {
            ctx.phase = "gpu benchmark";
            report.setStatus("running: gpu benchmark");
            ctx.drawStatusScreen("GPU benchmark running (about 25 s)...", 0.5);
            runGpuBenchmark(ctx);
        }

        if (!opt.skipSweep && ctx.videoOk && !ctx.quitRequested) {
            report.setStatus("running: GL config sweep");
            runConfigSweep(ctx);
            // Bring back a normal window for the final screen.
            std::string error;
            if (!ctx.openWindow(GlRequest(), error)) {
                report.line("could not reopen the default window for the final screen: " + error);
            }
        }

        report.section("MEMORY AT END");
        report.kv("probe RSS now", strf("%.1f MB (peak %.1f MB)", static_cast<double>(readSelfRssKb()) / 1024.0,
                                         static_cast<double>(readSelfPeakRssKb()) / 1024.0));
        const MemInfo mem = readMemInfo();
        report.kv("MemAvailable now", strf("%.0f MB", static_cast<double>(mem.availableKb) / 1024.0));
        report.kv("CPU state now", cpuStateSummary());

        finalScreen(ctx);
    } catch (const std::exception& e) {
        report.section("PROBE ERROR");
        report.line(std::string("uncaught exception: ") + e.what());
        report.setStatus("aborted by exception");
        report.save();
    }

    closeControllers();
    ctx.closeWindow();
    SDL_Quit();
    report.save();
    std::printf("\nreport written to %s\n", report.path().c_str());
    return 0;
}
