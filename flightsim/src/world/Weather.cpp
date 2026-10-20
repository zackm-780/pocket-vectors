#include "world/Weather.h"

#include <algorithm>
#include <cmath>

#include "core/Geo.h"
#include "core/Paths.h"
#include "core/Units.h"

namespace fsim::world {

namespace {

constexpr float kMinGustBaseKt = 1.0f;    // gust ratio needs a mean wind to scale
constexpr float kGustDirSwingDeg = 6.0f;  // wind direction swings with the gusts
constexpr float kTurbHorizKt = 2.5f;      // peak horizontal turbulence at turbulence = 1
constexpr float kTurbVertFps = 6.0f;      // peak vertical turbulence at turbulence = 1
constexpr float kShapeBias = 0.095f;      // mean of the raw shape (measured), removed again
constexpr float kLullFactor = 0.35f;      // lulls are shallower than gust peaks

// Deterministic hash -> [0, 1).
float hash01(std::uint32_t seed, std::uint32_t channel, std::int64_t knot) {
    std::uint64_t x = static_cast<std::uint64_t>(knot) * 0x9E3779B97F4A7C15ull;
    x ^= (static_cast<std::uint64_t>(seed) << 32) | channel;
    x ^= x >> 30;
    x *= 0xBF58476D1CE4E5B9ull;
    x ^= x >> 27;
    x *= 0x94D049BB133111EBull;
    x ^= x >> 31;
    return static_cast<float>(static_cast<double>(x >> 40) / 16777216.0);
}

// Smooth value noise in [0, 1]: random knots joined by smoothstep ramps. u is in knot units.
float valueNoise(std::uint32_t seed, std::uint32_t channel, double u) {
    const double f = std::floor(u);
    const float a = hash01(seed, channel, static_cast<std::int64_t>(f));
    const float b = hash01(seed, channel, static_cast<std::int64_t>(f) + 1);
    const float t = static_cast<float>(u - f);
    const float s = t * t * (3.0f - 2.0f * t);
    return a + (b - a) * s;
}

// Zero-mean smooth noise in [-1, 1].
float signedNoise(std::uint32_t seed, std::uint32_t channel, double tS, double periodS) {
    return 2.0f * valueNoise(seed, channel, tS / periodS) - 1.0f;
}

double shortestArcDeg(double fromDeg, double toDeg) {
    double d = std::fmod(toDeg - fromDeg, 360.0);
    if (d > 180.0) d -= 360.0;
    if (d < -180.0) d += 360.0;
    return d;
}

void toNedFps(float fromTrueDeg, double kt, double& n, double& e) {
    const double r = degToRad(fromTrueDeg);
    n = -ktToFps(kt) * std::cos(r);
    e = -ktToFps(kt) * std::sin(r);
}

}  // namespace

WeatherConditions resolveWeather(const WeatherSetup& setup, const WeatherOptions& options,
                                 const Metar* metar, float fallbackWindFromTrueDeg) {
    WeatherConditions c;
    c.seed = setup.seed;
    bool fromScenario = false;
    const Metar* m = (metar && options.useMetar && setup.metar.value_or(true)) ? metar : nullptr;

    // Wind: the scenario wins as a whole (a METAR gust must not be paired with a scenario wind).
    if (setup.windKt || setup.windFromTrueDeg || setup.gustKt) {
        fromScenario = true;
        c.windFromTrueDeg = setup.windFromTrueDeg.value_or(0.0f);
        c.windKt = std::max(0.0f, setup.windKt.value_or(0.0f));
        c.gustKt = setup.gustKt.value_or(0.0f);
    } else if (m && m->hasWind) {
        c.windFromTrueDeg = m->windVariable ? fallbackWindFromTrueDeg : m->windFromTrueDeg;
        c.windKt = m->windKt;
        c.gustKt = m->gustKt;
    }
    if (c.gustKt <= c.windKt) c.gustKt = 0;
    if (!options.gusts) c.gustKt = 0;

    // Aloft: explicit, else veered and stronger than the surface.
    const float veerDeg = setup.uniformWind ? 0.0f : Weather::kDefaultVeerDeg;
    const float aloftFactor = setup.uniformWind ? 1.0f : Weather::kDefaultAloftFactor;
    c.aloftFromTrueDeg =
        setup.aloftFromTrueDeg.value_or(static_cast<float>(wrap360(c.windFromTrueDeg + veerDeg)));
    c.aloftKt = setup.aloftKt.value_or(c.windKt * aloftFactor);

    if (setup.visibilityM) {
        fromScenario = true;
        c.hasVisibility = true;
        c.visibilityM = *setup.visibilityM;
    } else if (m && m->hasVisibility) {
        c.hasVisibility = true;
        c.visibilityM = m->visibilityM;
        c.visibilityAtLeast = m->visibilityAtLeast;
    }

    if (setup.qnhHpa) {
        fromScenario = true;
        c.qnhHpa = *setup.qnhHpa;
    } else if (m && m->hasAltimeter) {
        c.qnhHpa = m->altimeterHpa;
    }
    c.altimeterSettingHpa = setup.altimeterHpa.value_or(c.qnhHpa);

    if (setup.tempC) {
        fromScenario = true;
        c.hasTemp = true;
        c.tempC = *setup.tempC;
    } else if (m && m->hasTemp) {
        c.hasTemp = true;
        c.tempC = m->tempC;
    }

    c.turbulence = std::clamp(setup.turbulence.value_or(options.turbulence), 0.0f, 1.0f);
    if (setup.altimeterHpa || setup.turbulence || setup.aloftKt || setup.aloftFromTrueDeg)
        fromScenario = true;

    const bool anyMetar = m && (m->hasWind || m->hasVisibility || m->hasAltimeter);
    if (fromScenario && anyMetar)
        c.source = "scenario+metar " + m->station;
    else if (anyMetar)
        c.source = "metar " + m->station;
    else if (fromScenario)
        c.source = "scenario";
    return c;
}

float fogVisibilityM(const WeatherConditions& c) {
    return c.visibilityAtLeast ? std::max(c.visibilityM, 25000.0f) : c.visibilityM;
}

float isaDeviationC(float tempC, float elevationFt) {
    return tempC - (15.0f - 1.98f * elevationFt / 1000.0f);
}

bool loadMetarFor(const std::string& dataWeatherDir, const std::string& ident, Metar& out) {
    if (ident.empty()) return false;
    std::string up;
    for (char ch : ident) up += static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    std::vector<std::string> names{up};
    if (up.size() == 3) names.push_back("K" + up);  // US FAA idents (PDX -> KPDX)
    for (const std::string& n : names) {
        std::string text;
        if (readTextFile(dataWeatherDir + "/" + n + ".txt", text) && parseMetar(text, out))
            return true;
    }
    return false;
}

void Weather::setConditions(const WeatherConditions& c) {
    c_ = c;
}

void Weather::meanWindAt(double altAglFt, float& fromTrueDeg, float& kt) const {
    const double f = std::clamp(altAglFt / kProfileTopFtAgl, 0.0, 1.0);
    kt = static_cast<float>(c_.windKt + (c_.aloftKt - c_.windKt) * f);
    fromTrueDeg = static_cast<float>(
        wrap360(c_.windFromTrueDeg + shortestArcDeg(c_.windFromTrueDeg, c_.aloftFromTrueDeg) * f));
}

void Weather::reset(double altAglFt) {
    float from = 0, kt = 0;
    meanWindAt(altAglFt, from, kt);
    double n = 0, e = 0;
    toNedFps(from, kt, n, e);
    meanN_ = n;
    meanE_ = e;
    timeS_ = 0;
}

float Weather::gustShape(double tS) const {
    if (c_.gustKt <= c_.windKt || c_.windKt < kMinGustBaseKt) return 0.0f;
    const float n =
        0.6f * valueNoise(c_.seed, 1, tS / 8.3) + 0.4f * valueNoise(c_.seed, 2, tS / 2.7);
    const float u = std::clamp((n - 0.5f) * 3.2f, -1.0f, 1.0f);
    const float shaped = u > 0 ? u : u * kLullFactor;
    return (shaped - kShapeBias) / (1.0f - kShapeBias);  // re-centers so the mean is the set wind
}

WindNedFps Weather::step(double dtS, double altAglFt) {
    timeS_ += dtS;
    float from = 0, kt = 0;
    meanWindAt(altAglFt, from, kt);
    double tn = 0, te = 0;
    toNedFps(from, kt, tn, te);
    const double a = 1.0 - std::exp(-dtS / kMeanLagS);
    meanN_ += (tn - meanN_) * a;
    meanE_ += (te - meanE_) * a;

    const double ramp = std::clamp(timeS_ / kRampInS, 0.0, 1.0);
    double n = meanN_, e = meanE_, d = 0;

    // Gusts scale the whole vector (so the profile keeps its shape) and swing its direction.
    const float shape = gustShape(timeS_);
    if (shape != 0.0f) {
        const double ratio = (c_.gustKt - c_.windKt) / c_.windKt;
        const double scale = 1.0 + ratio * shape * ramp;
        const double swing = degToRad(kGustDirSwingDeg) * shape * ramp;
        const double cs = std::cos(swing), sn = std::sin(swing);
        const double rn = (n * cs - e * sn) * scale, re = (n * sn + e * cs) * scale;
        n = rn;
        e = re;
    }

    // Light turbulence: low-pass noise, a bit stronger in a stronger wind.
    if (c_.turbulence > 0.0f) {
        const double amp = c_.turbulence * ramp;
        const double horizKt = kTurbHorizKt * (1.0 + 0.04 * kt);
        n += ktToFps(horizKt) * amp *
             (0.65 * signedNoise(c_.seed, 3, timeS_, 2.3) +
              0.35 * signedNoise(c_.seed, 4, timeS_, 0.7));
        e += ktToFps(horizKt) * amp *
             (0.65 * signedNoise(c_.seed, 5, timeS_, 2.3) +
              0.35 * signedNoise(c_.seed, 6, timeS_, 0.7));
        d += kTurbVertFps * amp *
             (0.65 * signedNoise(c_.seed, 7, timeS_, 1.9) +
              0.35 * signedNoise(c_.seed, 8, timeS_, 0.6));
    }
    return {n, e, d};
}

}  // namespace fsim::world
