#pragma once
// X1: weather. Pure logic (no GL, no JSBSim): surface wind, a wind profile that veers and
// strengthens with height, smooth gusts, optional light turbulence, visibility and QNH.
//
// Sources, strongest first: scenario [weather]/[wind] keys (WeatherSetup), a METAR file
// (<data root>/weather/<ICAO>.txt) for whatever the scenario leaves unset, built-in defaults.
// Settings (WeatherOptions) switch the METAR use, the gusts and the turbulence.
//
// Gusts and turbulence are our own deterministic value noise (a few hashes per step) instead of
// JSBSim's Dryden model: no extra JSBSim state or tuning, reproducible from a seed, and cheap on
// the handheld. Output is a wind vector the Simulation hands to FlightModel::setWindNedFps.

#include <cstdint>
#include <optional>
#include <string>

#include "world/WeatherMetar.h"

namespace fsim::world {

// What a scenario (or a test) asks for. Unset fields come from the METAR, then the defaults.
struct WeatherSetup {
    std::optional<float> windFromTrueDeg, windKt, gustKt;
    std::optional<float> aloftFromTrueDeg, aloftKt;  // wind at and above kProfileTopFtAgl
    std::optional<float> visibilityM, qnhHpa, tempC;
    std::optional<float> altimeterHpa;  // pilot's altimeter setting; unset = follows QNH
    std::optional<float> turbulence;    // 0..1
    // Legacy [wind] scenarios: the same wind at every height (no default veer/strengthening).
    bool uniformWind = false;
    std::optional<bool> metar;  // false: ignore METAR files for this scenario
    std::uint32_t seed = 1;
};

// From settings.ini (P3).
struct WeatherOptions {
    bool useMetar = true;
    bool gusts = true;
    float turbulence = 0;  // 0..1, used when the scenario does not set its own
};

struct WeatherConditions {
    float windFromTrueDeg = 0, windKt = 0;
    float gustKt = 0;  // peak gust speed, 0 = none
    float aloftFromTrueDeg = 0, aloftKt = 0;
    bool hasVisibility = false;  // false: leave the atmosphere at its default
    float visibilityM = 25000.0f;
    bool visibilityAtLeast = false;  // METAR "9999 / CAVOK / P6SM": clear
    float qnhHpa = 1013.25f;
    float altimeterSettingHpa = 1013.25f;
    bool hasTemp = false;
    float tempC = 15.0f;
    float turbulence = 0;
    std::uint32_t seed = 1;
    std::string source = "default";  // "default", "scenario", "metar KPDX", "scenario+metar KPDX"
};

// Combines the sources. `metar` may be null. A METAR "VRB" wind uses fallbackWindFromTrueDeg
// (the runway heading: a variable wind is then close to a headwind).
WeatherConditions resolveWeather(const WeatherSetup& setup, const WeatherOptions& options,
                                 const Metar* metar, float fallbackWindFromTrueDeg);

// Visibility the fog should use: "10 km or more" reports clear air (25 km).
float fogVisibilityM(const WeatherConditions& c);

// Temperature deviation from ISA at a given elevation (ISA: 15 C at sea level, -1.98 C / 1000 ft).
float isaDeviationC(float tempC, float elevationFt);

// Reads <dataWeatherDir>/<ident>.txt (and K<ident> for a 3-letter ident). True if parsed.
bool loadMetarFor(const std::string& dataWeatherDir, const std::string& ident, Metar& out);

struct WindNedFps {
    double northFps = 0, eastFps = 0, downFps = 0;  // JSBSim: where the air moves TO
};

class Weather {
public:
    // Wind profile: linear from the surface value at 0 ft AGL to the aloft value here and above.
    static constexpr float kProfileTopFtAgl = 2000.0f;
    // Without an explicit aloft wind: veer (clockwise) by this much and strengthen by this factor.
    static constexpr float kDefaultVeerDeg = 25.0f;
    static constexpr float kDefaultAloftFactor = 1.5f;
    static constexpr double kRampInS = 10.0;  // gusts/turbulence fade in after reset
    static constexpr double kMeanLagS = 4.0;  // time constant when conditions change in flight

    Weather() { setConditions(WeatherConditions{}); }
    // Replaces the conditions. The wind follows with a first-order lag (smooth in flight).
    void setConditions(const WeatherConditions& c);
    const WeatherConditions& conditions() const { return c_; }

    // Mean (gust-free) wind at a height above ground.
    void meanWindAt(double altAglFt, float& fromTrueDeg, float& kt) const;
    // Jump to the mean wind at altAglFt, restart the noise clock and fade gusts in.
    void reset(double altAglFt);
    // Advances the clock and returns the wind (mean + gusts + turbulence) for the next step.
    WindNedFps step(double dtS, double altAglFt);

    // The gust shape in [-1, 1] (positive = gust, negative = lull) at time tS. Exposed for tests.
    float gustShape(double tS) const;
    double timeS() const { return timeS_; }

private:
    WeatherConditions c_;
    double timeS_ = 0;
    double meanN_ = 0, meanE_ = 0;  // lagged mean wind vector, ft/s toward
};

}  // namespace fsim::world
