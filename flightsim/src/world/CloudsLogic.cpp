#include "world/CloudsLogic.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdlib>

namespace fsim::world {
namespace {

std::string_view trim(std::string_view s) {
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.front()))) s.remove_prefix(1);
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.remove_suffix(1);
    return s;
}

bool parseNumber(std::string_view s, float& out) {
    if (s.empty()) return false;
    std::string tmp(s);
    char* end = nullptr;
    const float v = std::strtof(tmp.c_str(), &end);
    if (end == tmp.c_str() || *end != 0) return false;
    out = v;
    return true;
}

std::uint32_t hash2(int x, int y, std::uint32_t seed) {
    std::uint32_t h = seed * 0x9E3779B1u;
    h ^= static_cast<std::uint32_t>(x) * 0x85EBCA77u;
    h = (h ^ (h >> 15)) * 0xC2B2AE3Du;
    h ^= static_cast<std::uint32_t>(y) * 0x27D4EB2Fu;
    h = (h ^ (h >> 13)) * 0x165667B1u;
    return h ^ (h >> 16);
}

// Periodic value noise: `cells` lattice cells across the map, smooth interpolation.
float valueNoise(float x, float y, int cells, std::uint32_t seed) {
    const int x0 = static_cast<int>(std::floor(x)), y0 = static_cast<int>(std::floor(y));
    const float fx = x - static_cast<float>(x0), fy = y - static_cast<float>(y0);
    auto lattice = [&](int ix, int iy) {
        const int wx = ((ix % cells) + cells) % cells, wy = ((iy % cells) + cells) % cells;
        return static_cast<float>(hash2(wx, wy, seed) & 0xFFFFu) / 65535.0f;
    };
    const float sx = fx * fx * (3 - 2 * fx), sy = fy * fy * (3 - 2 * fy);
    const float a = lattice(x0, y0), b = lattice(x0 + 1, y0);
    const float c = lattice(x0, y0 + 1), d = lattice(x0 + 1, y0 + 1);
    return (a + (b - a) * sx) + ((c + (d - c) * sx) - (a + (b - a) * sx)) * sy;
}

// The (1 - coverage) quantile of the blended two-scale noise, so a cover hides about its share.
float coverageThreshold(float coverage) {
    static const std::vector<float> sorted = [] {
        std::vector<std::uint8_t> map;
        buildCloudNoise(map);
        std::vector<float> s;
        s.reserve(64 * 64);
        for (int j = 0; j < 64; ++j)
            for (int i = 0; i < 64; ++i)
                s.push_back(cloudNoiseAt(map, (i + 0.37) * kCloudTileM / 16.0,
                                         (j + 0.61) * kCloudTileM / 16.0));
        std::sort(s.begin(), s.end());
        return s;
    }();
    const float q = std::clamp(1.0f - coverage, 0.0f, 1.0f);
    const size_t idx =
        std::min(sorted.size() - 1, static_cast<size_t>(q * static_cast<float>(sorted.size())));
    return sorted[idx];
}

}  // namespace

const char* cloudCoverName(CloudCover c) {
    switch (c) {
    case CloudCover::Few: return "FEW";
    case CloudCover::Scattered: return "SCT";
    case CloudCover::Broken: return "BKN";
    case CloudCover::Overcast: return "OVC";
    }
    return "OVC";
}

bool parseCloudCover(std::string_view text, CloudCover& out) {
    std::string up(trim(text));
    for (char& ch : up) ch = static_cast<char>(std::toupper(static_cast<unsigned char>(ch)));
    for (CloudCover c :
         {CloudCover::Few, CloudCover::Scattered, CloudCover::Broken, CloudCover::Overcast}) {
        if (up == cloudCoverName(c)) {
            out = c;
            return true;
        }
    }
    return false;
}

bool parseCloudLayer(std::string_view text, CloudLayerSpec& out) {
    text = trim(text);
    if (text.size() < 3) return false;
    CloudLayerSpec spec;
    if (!parseCloudCover(text.substr(0, 3), spec.cover)) return false;
    std::string_view rest = trim(text.substr(3));
    if (rest.empty()) return false;
    // METAR style "BKN015": three digits of hundreds of feet directly after the cover.
    if (!std::isspace(static_cast<unsigned char>(text[3]))) {
        float hundreds = 0;
        if (rest.size() != 3 || !parseNumber(rest, hundreds)) return false;
        spec.baseAglFt = hundreds * 100.0f;
        out = spec;
        return spec.baseAglFt >= 0;
    }
    const size_t sp = rest.find_first_of(" \t-");
    if (!parseNumber(trim(rest.substr(0, sp)), spec.baseAglFt)) return false;
    if (sp != std::string_view::npos) {
        if (!parseNumber(trim(rest.substr(sp + 1)), spec.topAglFt)) return false;
        if (spec.topAglFt <= spec.baseAglFt) return false;
    }
    if (spec.baseAglFt < 0) return false;
    out = spec;
    return true;
}

float cloudCoverageFraction(CloudCover c) {
    switch (c) {
    case CloudCover::Few: return 0.2f;
    case CloudCover::Scattered: return 0.4f;
    case CloudCover::Broken: return 0.75f;
    case CloudCover::Overcast: return 1.0f;
    }
    return 1.0f;
}

float defaultCloudThicknessFt(CloudCover c) {
    switch (c) {
    case CloudCover::Few: return 800.0f;
    case CloudCover::Scattered: return 1200.0f;
    case CloudCover::Broken: return 1800.0f;
    case CloudCover::Overcast: return 2500.0f;
    }
    return 2000.0f;
}

CloudDeck resolveClouds(const CloudSetup& scenario, const Metar* metar, CloudMode mode,
                        float manualBaseAglFt, float fieldElevFt) {
    CloudDeck deck;
    std::vector<CloudLayerSpec> specs;
    switch (mode) {
    case CloudMode::Off: return deck;
    case CloudMode::Few:
    case CloudMode::Scattered:
    case CloudMode::Broken:
    case CloudMode::Overcast: {
        CloudLayerSpec s;
        s.cover =
            static_cast<CloudCover>(static_cast<int>(mode) - static_cast<int>(CloudMode::Few));
        s.baseAglFt = std::max(0.0f, manualBaseAglFt);
        specs.push_back(s);
        deck.source = "setting";
        break;
    }
    case CloudMode::Auto:
        if (scenario.set) {
            specs = scenario.layers;
            deck.source = "scenario";
        } else if (metar) {
            for (const MetarCloud& mc : metar->clouds) {
                CloudLayerSpec s;
                switch (mc.cover) {
                case MetarCloud::Cover::Few: s.cover = CloudCover::Few; break;
                case MetarCloud::Cover::Scattered: s.cover = CloudCover::Scattered; break;
                case MetarCloud::Cover::Broken: s.cover = CloudCover::Broken; break;
                case MetarCloud::Cover::Overcast:
                case MetarCloud::Cover::VerticalVis: s.cover = CloudCover::Overcast; break;
                }
                s.baseAglFt = mc.baseAglFt;
                specs.push_back(s);
            }
            if (!specs.empty()) deck.source = "metar " + metar->station;
        }
        break;
    }
    std::sort(specs.begin(), specs.end(), [](const CloudLayerSpec& a, const CloudLayerSpec& b) {
        return a.baseAglFt < b.baseAglFt;
    });
    for (const CloudLayerSpec& s : specs) {
        if (deck.count >= kMaxCloudLayers) break;
        CloudLayer l;
        l.cover = s.cover;
        l.baseFtMsl = fieldElevFt + s.baseAglFt;
        const float top =
            s.topAglFt > s.baseAglFt ? s.topAglFt : s.baseAglFt + defaultCloudThicknessFt(s.cover);
        l.topFtMsl = fieldElevFt + top;
        // A layer ends where the next one starts (no overlap).
        if (deck.count > 0) {
            CloudLayer& prev = deck.layers[deck.count - 1];
            prev.topFtMsl = std::min(prev.topFtMsl, std::max(prev.baseFtMsl + 100.0f, l.baseFtMsl));
        }
        deck.layers[deck.count++] = l;
    }
    if (deck.count == 0) deck.source = "none";
    return deck;
}

void buildCloudNoise(std::vector<std::uint8_t>& out, std::uint32_t seed) {
    constexpr int n = kCloudNoiseSize;
    std::vector<float> v(static_cast<size_t>(n) * n);
    float lo = 1e9f, hi = -1e9f;
    for (int y = 0; y < n; ++y) {
        for (int x = 0; x < n; ++x) {
            float sum = 0, amp = 0.5f, norm = 0;
            for (int cells : {4, 8, 16, 32}) {
                const float k = static_cast<float>(cells) / n;
                sum += amp * valueNoise(static_cast<float>(x) * k, static_cast<float>(y) * k, cells,
                                        seed + static_cast<std::uint32_t>(cells));
                norm += amp;
                amp *= 0.5f;
            }
            const float f = sum / norm;
            v[static_cast<size_t>(y) * n + x] = f;
            lo = std::min(lo, f);
            hi = std::max(hi, f);
        }
    }
    out.resize(v.size());
    const float scale = hi > lo ? 1.0f / (hi - lo) : 1.0f;
    for (size_t i = 0; i < v.size(); ++i)
        out[i] =
            static_cast<std::uint8_t>(std::lround(std::clamp((v[i] - lo) * scale, 0.f, 1.f) * 255));
}

float sampleCloudNoise(const std::vector<std::uint8_t>& noise, double u, double v) {
    constexpr int n = kCloudNoiseSize;
    if (noise.size() != static_cast<size_t>(n) * n) return 0.5f;
    // GL texel centers sit at (i + 0.5) / n.
    const double x = u * n - 0.5, y = v * n - 0.5;
    const double fx0 = std::floor(x), fy0 = std::floor(y);
    const float fx = static_cast<float>(x - fx0), fy = static_cast<float>(y - fy0);
    auto at = [&](long ix, long iy) {
        const long wx = ((ix % n) + n) % n, wy = ((iy % n) + n) % n;
        return static_cast<float>(noise[static_cast<size_t>(wy * n + wx)]) / 255.0f;
    };
    const long x0 = static_cast<long>(fx0), y0 = static_cast<long>(fy0);
    const float a = at(x0, y0), b = at(x0 + 1, y0), c = at(x0, y0 + 1), d = at(x0 + 1, y0 + 1);
    const float top = a + (b - a) * fx, bot = c + (d - c) * fx;
    return top + (bot - top) * fy;
}

float cloudNoiseAt(const std::vector<std::uint8_t>& noise, double eastM, double northM) {
    const double t1 = kCloudTileM, t2 = kCloudTileM / kCloudDetailRatio;
    return 0.65f * sampleCloudNoise(noise, eastM / t1, northM / t1) +
           0.35f * sampleCloudNoise(noise, eastM / t2, northM / t2);
}

void cloudAlphaThreshold(CloudCover c, float& threshold, float& soft) {
    soft = 0.05f;
    if (c == CloudCover::Overcast) {
        threshold = -1.0f;
        return;
    }
    threshold = coverageThreshold(cloudCoverageFraction(c));
}

float cloudAlphaAt(CloudCover c, float noise) {
    float t, s;
    cloudAlphaThreshold(c, t, s);
    const float x = std::clamp((noise - (t - s)) / (2 * s), 0.0f, 1.0f);
    return x * x * (3 - 2 * x);
}

CloudPlan planClouds(const CloudDeck& deck, float eyeFtMsl) {
    CloudPlan plan;
    std::array<float, kMaxCloudLayers> dist{};
    for (int i = 0; i < deck.count; ++i) {
        const CloudLayer& l = deck.layers[i];
        if (eyeFtMsl >= l.baseFtMsl && eyeFtMsl <= l.topFtMsl) {
            const float depth = std::min(eyeFtMsl - l.baseFtMsl, l.topFtMsl - eyeFtMsl);
            const float d = std::clamp(depth / 200.0f, 0.0f, 1.0f);
            if (plan.insideLayer < 0 || d > plan.insideDepth) {
                plan.insideLayer = i;
                plan.insideDepth = d;
            }
            continue;
        }
        CloudDraw draw;
        draw.layer = i;
        draw.top = eyeFtMsl > l.topFtMsl;
        draw.surfaceFtMsl = draw.top ? l.topFtMsl : l.baseFtMsl;
        dist[static_cast<size_t>(plan.drawCount)] = std::fabs(draw.surfaceFtMsl - eyeFtMsl);
        plan.draws[plan.drawCount++] = draw;
    }
    // Back to front: the farthest surface first (blending).
    for (int i = 1; i < plan.drawCount; ++i)
        for (int j = i; j > 0 && dist[static_cast<size_t>(j)] > dist[static_cast<size_t>(j - 1)];
             --j) {
            std::swap(dist[static_cast<size_t>(j)], dist[static_cast<size_t>(j - 1)]);
            std::swap(plan.draws[j], plan.draws[j - 1]);
        }
    return plan;
}

}  // namespace fsim::world
