#pragma once
// W4: shared atmosphere state. One Atmosphere instance owns the fog parameters (visibility ->
// exponential fog density, horizon haze color), the sun direction and the light terms. Every
// renderer (sky, debug ground, terrain, airports) reads the SAME fog from here, either through
// fogGlsl() + FogUniforms or through the plain values in fog().
//
// The math (UTC parsing, solar position, fog density, horizon dip) is free of GL and unit tested.

#include <string>

#include "core/Geo.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"

namespace fsim::world {

struct UtcTime {
    int year = 2026, month = 6, day = 21;
    int hour = 12, minute = 0;
    double second = 0;
};

// Accepts "YYYY-MM-DDTHH:MM[:SS]" (a space instead of 'T' and a trailing 'Z' are fine).
bool parseUtc(const std::string& text, UtcTime& out);
// Julian day number (UT) of a Gregorian date/time.
double julianDay(const UtcTime& t);

struct SolarPosition {
    double declinationRad = 0;
    double equationOfTimeMin = 0;
    double subsolarLatDeg = 0, subsolarLonDeg = 0;  // point where the sun is at the zenith
};
// NOAA "solar calculator" algorithm (Meeus-based, accuracy about 0.01 deg for 1900..2100).
SolarPosition solarPosition(const UtcTime& t);

// Sun elevation above the geometric horizon (no refraction) and its true azimuth, for an
// observer at latDeg/lonDeg.
struct SunAngles {
    double elevationRad = 0;
    double azimuthTrueRad = 0;  // clockwise from true north, [0, 2pi)
};
SunAngles sunAngles(const SolarPosition& sun, double latDeg, double lonDeg);
// Unit vector toward the sun in ECEF axes (independent of the observer).
Vec3d sunDirEcef(const SolarPosition& sun);
// Same direction in the observer's local north-east-down axes.
Vec3d sunDirNed(const SolarPosition& sun, double latDeg, double lonDeg);

// Koschmieder: contrast falls to 2 % over the meteorological visibility, so
// extinction = ln(1/0.02) / visibility = 3.912 / visibility.
float fogDensityPerM(float visibilityM);
// Fraction of the surface color that survives over distM (1 = clear, 0 = fully fogged).
float fogTransmittance(float densityPerM, float distM);
// Geometric horizon dip below eye level for an observer heightM above a smooth sphere.
double horizonDipRad(double heightM);

struct FogParams {
    float colorRgb[3] = {0.72f, 0.80f, 0.90f};  // horizon haze color == fog color
    float densityPerM = 1.5e-4f;
    float visibilityM = 25000.0f;
};

struct LightParams {
    // Terrain shading: albedo * (ambientRgb + diffuseRgb * max(0, dot(normal, sunDir))).
    float ambientRgb[3] = {0.35f, 0.40f, 0.48f};
    float diffuseRgb[3] = {0.80f, 0.76f, 0.68f};
    float sunDirEcef[3] = {0, 0, 1};  // unit vector toward the sun, ECEF axes
    float sunDirNed[3] = {0, 0, -1};  // same, local north-east-down axes at the observer
    // X4: night lighting. Moonlight is a faint blue directional term from the antisolar point
    // (full moon); nightFactor is 0 by day, 1 at night (drives city lights).
    float moonRgb[3] = {0, 0, 0};
    float nightFactor = 0;
};

struct SkyColors {
    float zenithRgb[3] = {0.25f, 0.45f, 0.85f};
    float sunDiscRgb[3] = {1, 0.95f, 0.85f};
    float glowRgb[3] = {1, 0.8f, 0.5f};  // warm aureole around the sun
    float glowStrength = 0.4f;
    float bandSharpness = 5.0f;  // larger = thinner haze band above the horizon
    float horizonDipRad = 0;
};

class Atmosphere {
public:
    Atmosphere();
    static constexpr float kDefaultVisibilityM = 25000.0f;
    static constexpr float kMinVisibilityM = 200.0f;
    static constexpr float kMaxVisibilityM = 200000.0f;

    // Visibility is the meteorological visibility (clamped to 200 m .. 200 km).
    void setVisibilityM(float visibilityM);
    void setUtc(const UtcTime& t);
    // Convenience: parses text (see parseUtc); on failure the previous time is kept.
    bool setUtcText(const std::string& text);
    // Observer position and height above the surface that defines the horizon. Recomputes the sun
    // direction, haze color and light terms; cheap enough to call every frame.
    void setObserver(double latDeg, double lonDeg, double heightAglM);

    const FogParams& fog() const { return fog_; }
    const LightParams& light() const { return light_; }
    const SkyColors& sky() const { return sky_; }
    const SunAngles& sun() const { return sunAngles_; }
    const UtcTime& utc() const { return utc_; }

private:
    void update();

    UtcTime utc_;
    SolarPosition solar_;
    double latDeg_ = 0, lonDeg_ = 0, heightAglM_ = 0;
    float visibilityM_ = kDefaultVisibilityM;
    FogParams fog_;
    LightParams light_;
    SkyColors sky_;
    SunAngles sunAngles_;
};

// GLSL (ES 1.00 style) shared by every shader that fogs surfaces. Declares
//   uniform vec3 uFogColor; uniform float uFogDensity;
//   float fsimFogFactor(float distM);          // 1 clear .. 0 fogged
//   vec3  fsimApplyFog(vec3 rgb, float distM);
// Paste it at the top of a vertex or fragment shader body (the shader preamble comes first).
// Use the same uniform names so FogUniforms works everywhere.
const char* fogGlsl();

// Locations of the shared fog uniforms in one program, plus the upload.
struct FogUniforms {
    gfx::GLint color = -1, density = -1;
    // Looks the uniforms up by name (the program must be linked; missing ones are ignored).
    void locate(const gfx::ShaderProgram& program);
    // The program must be in use.
    void set(const gfx::GlApi& gl, const FogParams& fog) const;
};

}  // namespace fsim::world
