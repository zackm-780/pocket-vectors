#pragma once
// P3: user settings (conf/settings.ini over assets/settings_default.ini), safe file writes and the
// "resume last flight" file (conf/last_flight.ini, a scenario-compatible INI).
//
// Precedence everywhere: command line > settings.ini > settings_default.ini > built-in defaults.
// Every value is clamped / validated on load, so a hand-edited file can never produce an invalid
// Settings. Missing or corrupt files fall back to the defaults with a log line.

#include <string>

#include "app/Cli.h"
#include "app/Scenario.h"
#include "core/Paths.h"
#include "input/PilotControls.h"

namespace fsim {

enum class HudColor { Green, White, Amber };
enum class HudDeclutter { Full, Std, Min };
enum class PressureUnits { Hpa, InHg };
enum class GraphicsQuality { Low, Med, High };
enum class ThrottleMode { Rate, Absolute };

struct Settings {
    std::string dataDir;           // empty = <asset root>/data
    float fovVerticalDeg = 55.0f;  // 30..110
    bool showFps = false;
    HudColor hudColor = HudColor::Green;
    HudDeclutter hudDeclutterDefault = HudDeclutter::Std;
    PressureUnits pressureUnits = PressureUnits::Hpa;
    int simHz = 120;  // 120 | 60 (stored; the fixed step is still core/SimTime.h kSimHz)
    GraphicsQuality graphicsQuality = GraphicsQuality::Med;
    int volumePct = 80;  // 0..100
    bool assistAutoRudder = false;
    bool assistAutoTrim = false;
    ThrottleMode throttleMode = ThrottleMode::Rate;
    float pitchSens = 1.0f, rollSens = 1.0f, rudderSens = 1.0f;  // 0.25..2
    float pitchExpo = 0.0f, rollExpo = 0.0f, rudderExpo = 0.0f;  // 0..1
    std::string lastAircraft;

    // Parses settings text onto *this (keys that are absent or invalid keep their current value;
    // invalid values are logged). Returns the number of rejected values.
    int applyText(const std::string& text);
    // Loads `defaultsPath` then `userPath` on top. Missing/unreadable files are logged and
    // skipped. Always leaves a valid Settings. Returns false if the user file existed but was
    // unreadable.
    bool load(const std::string& defaultsPath, const std::string& userPath);
    // Writes the settings to `userPath`, preserving unknown keys, other sections and comment lines
    // of an existing file. Safe write. Returns false on IO failure.
    bool save(const std::string& userPath) const;

    // Clamps every numeric field into its legal range and normalises simHz.
    void clampAll();
    // The input hooks (stick sensitivity/expo, throttle mode).
    StickTuning stickTuning() const;
    // Text form of the whole settings block, "key = value" lines in file order.
    std::string toKeyValueText() const;
};

// Write `text` to `path` through `path + ".tmp"`: write, flush, fsync (POSIX), rename. The temp
// file is removed on failure. Creates no directories.
bool writeFileAtomic(const std::string& path, const std::string& text);

// Path overrides with settings applied under the CLI: CLI --data-dir wins, else settings data_dir.
PathOverrides effectivePathOverrides(const CliOptions& cli, const Settings& settings);
// show_fps: the CLI flag turns it on; the setting alone also turns it on.
bool effectiveShowFps(const CliOptions& cli, const Settings& settings);

// ---- resume last flight -------------------------------------------------------------------
// Serialises the start-defining parts of a scenario (aircraft, airport, runway, start type,
// distance/altitude, time_utc, wind, autopilot bugs). Exact mid-air state restore is out of scope:
// resuming re-creates the same start, not the position the aircraft reached. Scripted [inputs]
// are not saved.
std::string scenarioToIni(const Scenario& scenario);
bool writeLastFlight(const std::string& path, const Scenario& scenario);

}  // namespace fsim
