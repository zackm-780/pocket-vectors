#pragma once
// W7: cloud layers without GL. Up to three layers (FEW / SCT / BKN / OVC, base and top) from a
// scenario [clouds] section, a manual setting, or the METAR; a tiling noise map that shapes the
// broken layers; and the per-frame plan (which surface of which layer to draw, in which order,
// and how deep the eye is inside one). Clouds.h draws it.

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

#include "world/WeatherMetar.h"

namespace fsim::world {

enum class CloudCover { Few, Scattered, Broken, Overcast };

constexpr int kMaxCloudLayers = 3;

struct CloudLayerSpec {  // as written: heights above the field
    CloudCover cover = CloudCover::Overcast;
    float baseAglFt = 0;
    float topAglFt = 0;  // 0 = base + the default thickness for the cover
};

// Scenario [clouds]: layer1..layer3 = "<FEW|SCT|BKN|OVC> <base ft AGL> [top ft AGL]", or
// clear = true for none. Unset: the setting / METAR decide.
struct CloudSetup {
    bool set = false;  // the scenario has a [clouds] section that decides
    std::vector<CloudLayerSpec> layers;
};

// settings.ini `clouds`: auto = scenario, then METAR; off = none; a cover = one manual layer at
// `cloud_base_ft` above the field.
enum class CloudMode { Auto, Off, Few, Scattered, Broken, Overcast };

struct CloudLayer {  // resolved: heights MSL
    CloudCover cover = CloudCover::Overcast;
    float baseFtMsl = 0, topFtMsl = 0;
};

struct CloudDeck {
    int count = 0;
    CloudLayer layers[kMaxCloudLayers];  // sorted by base, lowest first
    std::string source = "none";         // "none", "scenario", "setting", "metar KPDX"
};

const char* cloudCoverName(CloudCover c);  // "FEW", "SCT", "BKN", "OVC"
bool parseCloudCover(std::string_view text, CloudCover& out);
// "OVC 1500 3500", "BKN 2500", "SCT015" (METAR style, hundreds of feet) are all accepted.
bool parseCloudLayer(std::string_view text, CloudLayerSpec& out);
float cloudCoverageFraction(CloudCover c);  // FEW 0.2, SCT 0.4, BKN 0.75, OVC 1
float defaultCloudThicknessFt(CloudCover c);

CloudDeck resolveClouds(const CloudSetup& scenario, const Metar* metar, CloudMode mode,
                        float manualBaseAglFt, float fieldElevFt);

// ---- coverage pattern ------------------------------------------------------------------------
// A periodic value-noise map, kNoiseSize x kNoiseSize bytes (0..255), the same on CPU and GPU.
// The layer samples it at two scales: 0.65 * n(p / tile) + 0.35 * n(p / (tile / 3.7)).
constexpr int kCloudNoiseSize = 128;
constexpr float kCloudTileM = 8000.0f;
constexpr float kCloudDetailRatio = 3.7f;
void buildCloudNoise(std::vector<std::uint8_t>& out, std::uint32_t seed = 7);
// Bilinear, wrapping lookup in [0, 1] at texture coordinates (u, v) (1 = one tile).
float sampleCloudNoise(const std::vector<std::uint8_t>& noise, double u, double v);
// The blended two-scale value at world east / north meters from the deck origin.
float cloudNoiseAt(const std::vector<std::uint8_t>& noise, double eastM, double northM);
// Shader threshold and softness: alpha = smoothstep(t - s, t + s, noise). OVC returns t < 0.
void cloudAlphaThreshold(CloudCover c, float& threshold, float& soft);
float cloudAlphaAt(CloudCover c, float noise);

// ---- per-frame plan --------------------------------------------------------------------------
struct CloudDraw {
    int layer = 0;
    bool top = false;  // draw the top surface (eye above), else the base (eye below)
    float surfaceFtMsl = 0;
};
struct CloudPlan {
    int drawCount = 0;
    CloudDraw draws[kMaxCloudLayers];  // back to front (farthest surface first)
    int insideLayer = -1;              // the layer the eye is in, -1 = none
    float insideDepth = 0;             // 0 at the base or top .. 1 at 200 ft or more inside
};
CloudPlan planClouds(const CloudDeck& deck, float eyeFtMsl);

}  // namespace fsim::world
