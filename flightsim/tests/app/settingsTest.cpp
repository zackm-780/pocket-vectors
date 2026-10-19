#include <doctest/doctest.h>

#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "app/Cli.h"
#include "app/Scenario.h"
#include "app/Settings.h"
#include "core/Ini.h"
#include "core/Log.h"
#include "core/Paths.h"
#include "input/PilotControls.h"

using namespace fsim;
namespace fs = std::filesystem;

namespace {

struct TempDir {
    std::string path;
    TempDir() {
        static int counter = 0;
        path = (fs::temp_directory_path() / ("fsim_settings_test_" + std::to_string(::getpid()) +
                                             "_" + std::to_string(counter++)))
                   .string();
        fs::remove_all(path);
        fs::create_directories(path);
    }
    ~TempDir() { fs::remove_all(path); }
    std::string file(const std::string& name) const { return path + "/" + name; }
};

void writeText(const std::string& path, const std::string& text) {
    std::ofstream(path, std::ios::binary) << text;
}

std::string readAll(const std::string& path) {
    std::string t;
    readTextFile(path, t);
    return t;
}

const char* kDefaults = "assets/settings_default.ini";

}  // namespace

TEST_CASE("settings: shipped defaults file equals the built-in defaults") {
    Settings s;
    REQUIRE(s.load(kDefaults, ""));
    const Settings d;
    CHECK(s.toKeyValueText() == d.toKeyValueText());
}

TEST_CASE("settings: missing files give defaults") {
    TempDir t;
    Settings s;
    s.volumePct = 3;  // load() must reset to defaults first
    CHECK(s.load(t.file("nope_default.ini"), t.file("nope.ini")));
    CHECK(s.toKeyValueText() == Settings{}.toKeyValueText());
}

TEST_CASE("settings: round trip of every key") {
    TempDir t;
    Settings s;
    s.dataDir = "/mnt/sd/data \xC3\xA9;#x";  // verbatim UTF-8, ';' and '#' inside the value
    s.fovVerticalDeg = 72.5f;
    s.showFps = true;
    s.hudColor = HudColor::Amber;
    s.hudDeclutterDefault = HudDeclutter::Min;
    s.pressureUnits = PressureUnits::InHg;
    s.simHz = 60;
    s.graphicsQuality = GraphicsQuality::High;
    s.volumePct = 35;
    s.assistAutoRudder = true;
    s.assistAutoTrim = true;
    s.throttleMode = ThrottleMode::Absolute;
    s.pitchSens = 0.5f;
    s.rollSens = 1.5f;
    s.rudderSens = 2.0f;
    s.pitchExpo = 0.25f;
    s.rollExpo = 0.5f;
    s.rudderExpo = 1.0f;
    s.lastAircraft = "737";
    REQUIRE(s.save(t.file("settings.ini")));

    Settings r;
    REQUIRE(r.load(kDefaults, t.file("settings.ini")));
    CHECK(r.toKeyValueText() == s.toKeyValueText());
    CHECK(r.dataDir == s.dataDir);
    CHECK(r.fovVerticalDeg == doctest::Approx(72.5f));
    CHECK(r.hudColor == HudColor::Amber);
    CHECK(r.simHz == 60);
    CHECK(r.throttleMode == ThrottleMode::Absolute);
    CHECK(r.rudderExpo == doctest::Approx(1.0f));
    CHECK(r.lastAircraft == "737");
}

TEST_CASE("settings: out-of-range and invalid values are clamped or rejected") {
    log::setStderrEnabled(false);
    Settings s;
    const int rejected = s.applyText("[settings]\n"
                                     "fov_vertical_deg = 500\n"
                                     "volume_pct = -20\n"
                                     "pitch_sensitivity = 99\n"
                                     "roll_sensitivity = 0\n"
                                     "pitch_expo = 7\n"
                                     "rudder_expo = -1\n"
                                     "sim_hz = 90\n"
                                     "hud_color = purple\n"
                                     "show_fps = maybe\n"
                                     "graphics_quality = HIGH\n");
    log::setStderrEnabled(true);
    CHECK(s.fovVerticalDeg == doctest::Approx(110.0f));
    CHECK(s.volumePct == 0);
    CHECK(s.pitchSens == doctest::Approx(2.0f));
    CHECK(s.rollSens == doctest::Approx(0.25f));
    CHECK(s.pitchExpo == doctest::Approx(1.0f));
    CHECK(s.rudderExpo == doctest::Approx(0.0f));
    CHECK(s.simHz == 120);
    CHECK(s.hudColor == HudColor::Green);  // invalid keeps the default
    CHECK(s.showFps == false);
    CHECK(s.graphicsQuality == GraphicsQuality::High);  // case-insensitive
    CHECK(rejected == 3);                               // sim_hz, hud_color, show_fps
}

TEST_CASE("settings: corrupt files fall back to defaults") {
    TempDir t;
    log::setStderrEnabled(false);
    SUBCASE("binary garbage") {
        writeText(t.file("settings.ini"), std::string("\x01\x02\0\xFF\xFE garbage", 14));
        Settings s;
        CHECK(s.load(kDefaults, t.file("settings.ini")));
        CHECK(s.toKeyValueText() == Settings{}.toKeyValueText());
        // saving over it replaces the garbage with a valid file
        s.volumePct = 10;
        REQUIRE(s.save(t.file("settings.ini")));
        Settings r;
        r.load(kDefaults, t.file("settings.ini"));
        CHECK(r.volumePct == 10);
        CHECK(readAll(t.file("settings.ini")).find('\0') == std::string::npos);
    }
    SUBCASE("nonsense text") {
        writeText(t.file("settings.ini"), "[[[\n=\n=====\nfov_vertical_deg = abc\n[settings\nx\n");
        Settings s;
        CHECK(s.load(kDefaults, t.file("settings.ini")));
        CHECK(s.fovVerticalDeg == doctest::Approx(55.0f));
    }
    SUBCASE("user path is a directory") {
        Settings s;
        s.load(kDefaults, t.path);  // unreadable as a file: defaults, no crash
        CHECK(s.toKeyValueText() == Settings{}.toKeyValueText());
    }
    log::setStderrEnabled(true);
}

TEST_CASE("settings: save preserves unknown keys, other sections and comments") {
    TempDir t;
    writeText(t.file("settings.ini"), "; my comment\n"
                                      "# another\n"
                                      "[future]\n"
                                      "thing = 1\n"
                                      "[settings]\n"
                                      "; about fov\n"
                                      "FOV_Vertical_Deg = 60\n"
                                      "future_key = keep me; really\n"
                                      "volume_pct = 10\n"
                                      "\n"
                                      "[tail]\n"
                                      "z = 9\n");
    Settings s;
    s.load(kDefaults, t.file("settings.ini"));
    CHECK(s.fovVerticalDeg == doctest::Approx(60.0f));
    s.volumePct = 55;
    s.lastAircraft = "c172p";
    REQUIRE(s.save(t.file("settings.ini")));

    const std::string out = readAll(t.file("settings.ini"));
    CHECK(out.find("; my comment\n") != std::string::npos);
    CHECK(out.find("# another\n") != std::string::npos);
    CHECK(out.find("; about fov\n") != std::string::npos);
    CHECK(out.find("future_key = keep me; really\n") != std::string::npos);
    CHECK(out.find("[future]\nthing = 1\n") != std::string::npos);
    CHECK(out.find("[tail]\nz = 9\n") != std::string::npos);
    CHECK(out.find("volume_pct = 55\n") != std::string::npos);
    CHECK(out.find("volume_pct = 10") == std::string::npos);

    Ini ini;
    ini.loadText(out);
    CHECK(ini.getString("settings", "future_key") == "keep me; really");
    CHECK(ini.getString("settings", "last_aircraft") == "c172p");  // added inside [settings]
    CHECK(ini.getString("tail", "z") == "9");
    CHECK(ini.getInt("future", "thing") == 1);
    // idempotent: saving again changes nothing
    REQUIRE(s.save(t.file("settings.ini")));
    CHECK(readAll(t.file("settings.ini")) == out);
}

TEST_CASE("settings: safe write leaves no temp file and replaces atomically") {
    TempDir t;
    const std::string path = t.file("a.ini");
    REQUIRE(writeFileAtomic(path, "one\n"));
    REQUIRE(writeFileAtomic(path, "two\n"));
    CHECK(readAll(path) == "two\n");
    CHECK_FALSE(fileExists(path + ".tmp"));
    // failure (directory missing) reports false and leaves no temp file
    log::setStderrEnabled(false);
    CHECK_FALSE(writeFileAtomic(t.file("missing/dir/a.ini"), "x"));
    log::setStderrEnabled(true);
    CHECK_FALSE(fileExists(t.file("missing/dir/a.ini.tmp")));
    // a failed rename (target is a non-empty directory) cleans up the temp file
    fs::create_directories(t.file("blocked/inner"));
    log::setStderrEnabled(false);
    CHECK_FALSE(writeFileAtomic(t.file("blocked"), "x"));
    log::setStderrEnabled(true);
    CHECK_FALSE(fileExists(t.file("blocked.tmp")));
    // a settings save leaves only settings.ini behind
    Settings s;
    REQUIRE(s.save(t.file("settings.ini")));
    int count = 0;
    for (const auto& e : fs::directory_iterator(t.path))
        if (e.path().filename().string().rfind("settings", 0) == 0) ++count;
    CHECK(count == 1);
}

TEST_CASE("settings: precedence CLI > settings > defaults") {
    Settings s;
    CliOptions cli;
    CHECK(effectivePathOverrides(cli, s).dataDir.empty());  // default: Paths picks <asset>/data
    s.dataDir = "/from/settings";
    CHECK(effectivePathOverrides(cli, s).dataDir == "/from/settings");
    cli.dataDir = "/from/cli";
    cli.confDir = "/conf";
    cli.assetDir = "/assets";
    const PathOverrides o = effectivePathOverrides(cli, s);
    CHECK(o.dataDir == "/from/cli");
    CHECK(o.confDir == "/conf");
    CHECK(o.assetDir == "/assets");
    // through the real resolver
    CHECK(Paths::resolve(o).dataRoot() == "/from/cli");
    cli.dataDir.clear();
    CHECK(Paths::resolve(effectivePathOverrides(cli, s)).dataRoot() == "/from/settings");

    CliOptions c2;
    Settings s2;
    CHECK_FALSE(effectiveShowFps(c2, s2));
    s2.showFps = true;
    CHECK(effectiveShowFps(c2, s2));
    s2.showFps = false;
    c2.showFps = true;
    CHECK(effectiveShowFps(c2, s2));

    std::string err;
    CliOptions r;
    REQUIRE(parseCli({"--resume", "--conf-dir", "/c"}, r, err));
    CHECK(r.resume);
    CHECK(r.confDir == "/c");
}

TEST_CASE("settings: conf dir is created on demand") {
    TempDir t;
    const std::string conf = t.file("a/b/conf");
    REQUIRE(makeDirs(conf));
    Settings s;
    REQUIRE(s.save(Paths::join(conf, "settings.ini")));
    CHECK(fileExists(Paths::join(conf, "settings.ini")));
}

TEST_CASE("settings: last_flight.ini round trips through the scenario loader") {
    TempDir t;
    Scenario sc;
    sc.aircraftId = "737";
    sc.airportIdent = "XTST";
    sc.runwayIdent = "09";
    sc.startKind = "cruise";
    sc.distanceNm = 8.5;
    sc.altitudeFt = 12000;
    sc.timeUtc = "2026-06-21T18:00:00";
    sc.windFromTrueDeg = 270;
    sc.windSpeedKt = 12;
    sc.apEngage = {"hdg", "alt"};
    sc.hasHdgBug = true;
    sc.hdgBugMagDeg = 90;
    sc.hasAltBug = true;
    sc.altBugFt = 5000;
    sc.inputs.push_back({1.0, ScriptCommand::Throttle, 1.0f});  // not saved

    REQUIRE(writeLastFlight(t.file("last_flight.ini"), sc));
    CHECK_FALSE(fileExists(t.file("last_flight.ini.tmp")));
    Scenario r;
    std::string err;
    REQUIRE_MESSAGE(loadScenario(t.file("last_flight.ini"), r, err), err);
    CHECK(r.aircraftId == "737");
    CHECK(r.airportIdent == "XTST");
    CHECK(r.runwayIdent == "09");
    CHECK(r.startKind == "cruise");
    CHECK(r.distanceNm == doctest::Approx(8.5));
    CHECK(r.altitudeFt == doctest::Approx(12000));
    CHECK(r.timeUtc == "2026-06-21T18:00:00");
    CHECK(r.windFromTrueDeg == doctest::Approx(270));
    CHECK(r.windSpeedKt == doctest::Approx(12));
    CHECK(r.apEngage == sc.apEngage);
    CHECK(r.hasHdgBug);
    CHECK(r.hdgBugMagDeg == doctest::Approx(90));
    CHECK(r.hasAltBug);
    CHECK(r.altBugFt == doctest::Approx(5000));
    CHECK(r.inputs.empty());
}

TEST_CASE("settings: default scenario also round trips") {
    Scenario sc, r;
    std::string err;
    REQUIRE(parseScenario(scenarioToIni(sc), r, err));
    CHECK(scenarioToIni(r) == scenarioToIni(sc));
}

TEST_CASE("settings: stick tuning reaches the pilot controls") {
    Settings s;
    s.pitchSens = 0.5f;
    s.rollExpo = 1.0f;
    s.throttleMode = ThrottleMode::Absolute;
    const StickTuning t = s.stickTuning();
    CHECK(t.pitchSens == doctest::Approx(0.5f));
    CHECK(t.rollExpo == doctest::Approx(1.0f));
    CHECK(t.throttleAbsolute);

    PilotControls pc;
    pc.setStickTuning(t);
    AircraftProfile profile;
    profile.engineCount = 1;
    AircraftState state;
    ActionState in;
    in.axis[static_cast<int>(Axis::Pitch)] = 1.0f;
    in.axis[static_cast<int>(Axis::Roll)] = 0.5f;
    in.axis[static_cast<int>(Axis::ThrottleRate)] = 1.0f;
    ControlInputs c;
    pc.update(in, profile, state, 1.0 / 60, c);
    CHECK(c.elevatorNorm == doctest::Approx(0.5f));     // sens
    CHECK(c.aileronNorm == doctest::Approx(0.125f));    // pure cubic expo
    CHECK(c.throttleNorm[0] == doctest::Approx(1.0f));  // absolute: stick up = full
    in.axis[static_cast<int>(Axis::ThrottleRate)] = -1.0f;
    pc.update(in, profile, state, 1.0 / 60, c);
    CHECK(c.throttleNorm[0] == doctest::Approx(0.0f));
}
