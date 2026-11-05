#include "world/Clouds.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "core/Geo.h"
#include "core/Units.h"

namespace fsim::world {
namespace {

// Layer disk: positions are east / north meters from the point under the eye; z is the drop of
// the Earth's surface there. The layer's height above or below the eye comes as a uniform.
constexpr char kLayerVertexBody[] = "attribute vec3 aPos;\n"
                                    "uniform mat4 uMvp;\n"
                                    "uniform float uHeight;\n"
                                    "uniform vec3 uOff1;\n"
                                    "uniform vec3 uOff2;\n"
                                    "uniform float uInvTile1;\n"
                                    "uniform float uInvTile2;\n"
                                    "uniform vec3 uColor;\n"
                                    "varying vec2 vUv1;\n"
                                    "varying vec2 vUv2;\n"
                                    "varying vec3 vColor;\n"
                                    "void main() {\n"
                                    "  vec3 p = vec3(aPos.xy, uHeight + aPos.z);\n"
                                    "  vColor = fsimApplyFog(uColor, length(p));\n"
                                    "  vUv1 = (aPos.xy + uOff1.xy) * uInvTile1;\n"
                                    "  vUv2 = (aPos.xy + uOff2.xy) * uInvTile2;\n"
                                    "  gl_Position = uMvp * vec4(p, 1.0);\n"
                                    "}\n";

constexpr char kLayerFragment[] =
    "uniform sampler2D uNoise;\n"
    "uniform float uThreshold;\n"
    "uniform float uSoft;\n"
    "uniform float uShade;\n"
    "varying vec2 vUv1;\n"
    "varying vec2 vUv2;\n"
    "varying vec3 vColor;\n"
    "void main() {\n"
    "  float n = 0.65 * texture2D(uNoise, vUv1).r + 0.35 * texture2D(uNoise, vUv2).r;\n"
    "  float a = smoothstep(uThreshold - uSoft, uThreshold + uSoft, n);\n"
    "  gl_FragColor = vec4(vColor * (1.0 - uShade + uShade * 1.6 * n), a);\n"
    "}\n";

constexpr char kFadeVertex[] = "attribute vec2 aPos;\n"
                               "void main() { gl_Position = vec4(aPos, 0.0, 1.0); }\n";
constexpr char kFadeFragment[] = "uniform vec4 uColor;\n"
                                 "void main() { gl_FragColor = uColor; }\n";

constexpr int kRings = 16, kSegments = 32;
constexpr float kRadiusM = 50000.0f;

// ENU (east, north, up) -> ECEF at a point: columns are the east, north and up unit vectors.
Mat3d enuToEcef(double latDeg, double lonDeg) {
    const Mat3d ned = nedToEcef(latDeg, lonDeg);
    Mat3d m;
    for (int r = 0; r < 3; ++r) {
        m.m[r][0] = ned.m[r][1];
        m.m[r][1] = ned.m[r][0];
        m.m[r][2] = -ned.m[r][2];
    }
    return m;
}

double wrapPositive(double v, double period) {
    const double w = std::fmod(v, period);
    return w < 0 ? w + period : w;
}

}  // namespace

Clouds::~Clouds() {
    if (vbo_ != 0) gl_.DeleteBuffers(1, &vbo_);
    if (ibo_ != 0) gl_.DeleteBuffers(1, &ibo_);
    if (fadeVbo_ != 0) gl_.DeleteBuffers(1, &fadeVbo_);
    if (texture_ != 0) gl_.DeleteTextures(1, &texture_);
}

bool Clouds::init() {
    const std::string vs = std::string(fogGlsl()) + kLayerVertexBody;
    if (!layerProgram_.build(gl_, flavor_, vs.c_str(), kLayerFragment, {"aPos"})) return false;
    if (!fadeProgram_.build(gl_, flavor_, kFadeVertex, kFadeFragment, {"aPos"})) return false;
    uMvp_ = layerProgram_.uniform("uMvp");
    uHeight_ = layerProgram_.uniform("uHeight");
    uOff1_ = layerProgram_.uniform("uOff1");
    uOff2_ = layerProgram_.uniform("uOff2");
    uInvTile1_ = layerProgram_.uniform("uInvTile1");
    uInvTile2_ = layerProgram_.uniform("uInvTile2");
    uColor_ = layerProgram_.uniform("uColor");
    uThreshold_ = layerProgram_.uniform("uThreshold");
    uSoft_ = layerProgram_.uniform("uSoft");
    uShade_ = layerProgram_.uniform("uShade");
    uNoise_ = layerProgram_.uniform("uNoise");
    fog_.locate(layerProgram_);
    uFadeColor_ = fadeProgram_.uniform("uColor");

    // Disk mesh: rings packed toward the center (r ~ (i / N)^1.7), curvature drop in z.
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(1 + kRings * kSegments) * 3);
    verts.insert(verts.end(), {0, 0, 0});
    for (int i = 1; i <= kRings; ++i) {
        const float r = kRadiusM * std::pow(static_cast<float>(i) / kRings, 1.7f);
        const float drop = -r * r / (2.0f * static_cast<float>(kEarthRadiusM));
        for (int k = 0; k < kSegments; ++k) {
            const float a = 2.0f * static_cast<float>(kPi) * static_cast<float>(k) / kSegments;
            verts.insert(verts.end(), {r * std::cos(a), r * std::sin(a), drop});
        }
    }
    std::vector<std::uint16_t> idx;
    for (int k = 0; k < kSegments; ++k)
        idx.insert(idx.end(), {0, static_cast<std::uint16_t>(1 + k),
                               static_cast<std::uint16_t>(1 + (k + 1) % kSegments)});
    for (int i = 1; i < kRings; ++i) {
        const int in = 1 + (i - 1) * kSegments, out = 1 + i * kSegments;
        for (int k = 0; k < kSegments; ++k) {
            const int k1 = (k + 1) % kSegments;
            idx.insert(idx.end(),
                       {static_cast<std::uint16_t>(in + k), static_cast<std::uint16_t>(out + k),
                        static_cast<std::uint16_t>(out + k1), static_cast<std::uint16_t>(in + k),
                        static_cast<std::uint16_t>(out + k1), static_cast<std::uint16_t>(in + k1)});
        }
    }
    indexCount_ = static_cast<int>(idx.size());
    gl_.GenBuffers(1, &vbo_);
    gl_.GenBuffers(1, &ibo_);
    gl_.GenBuffers(1, &fadeVbo_);
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, vbo_);
    gl_.BufferData(gfx::GL_ARRAY_BUFFER, static_cast<gfx::GLsizeiptr>(verts.size() * sizeof(float)),
                   verts.data(), gfx::GL_STATIC_DRAW);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, ibo_);
    gl_.BufferData(gfx::GL_ELEMENT_ARRAY_BUFFER,
                   static_cast<gfx::GLsizeiptr>(idx.size() * sizeof(std::uint16_t)), idx.data(),
                   gfx::GL_STATIC_DRAW);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, 0);
    const float quad[] = {-1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1};
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, fadeVbo_);
    gl_.BufferData(gfx::GL_ARRAY_BUFFER, sizeof quad, quad, gfx::GL_STATIC_DRAW);

    // Coverage texture with its mip chain (box filtered on the CPU: no glGenerateMipmap on GL 2.1).
    buildCloudNoise(noise_);
    gl_.GenTextures(1, &texture_);
    gl_.BindTexture(gfx::GL_TEXTURE_2D, texture_);
    gl_.PixelStorei(gfx::GL_UNPACK_ALIGNMENT, 1);
    std::vector<std::uint8_t> level = noise_;
    int size = kCloudNoiseSize;
    for (int lvl = 0; size >= 1; ++lvl) {
        gl_.TexImage2D(gfx::GL_TEXTURE_2D, lvl, static_cast<gfx::GLint>(gfx::GL_LUMINANCE), size,
                       size, 0, gfx::GL_LUMINANCE, gfx::GL_UNSIGNED_BYTE, level.data());
        if (size == 1) break;
        const int half = size / 2;
        std::vector<std::uint8_t> next(static_cast<size_t>(half) * half);
        for (int y = 0; y < half; ++y)
            for (int x = 0; x < half; ++x) {
                const auto at = [&](int xx, int yy) {
                    return static_cast<int>(level[static_cast<size_t>(yy) * size + xx]);
                };
                next[static_cast<size_t>(y) * half + x] = static_cast<std::uint8_t>(
                    (at(2 * x, 2 * y) + at(2 * x + 1, 2 * y) + at(2 * x, 2 * y + 1) +
                     at(2 * x + 1, 2 * y + 1) + 2) /
                    4);
            }
        level.swap(next);
        size = half;
    }
    gl_.TexParameteri(gfx::GL_TEXTURE_2D, gfx::GL_TEXTURE_MIN_FILTER,
                      static_cast<gfx::GLint>(gfx::GL_LINEAR_MIPMAP_LINEAR));
    gl_.TexParameteri(gfx::GL_TEXTURE_2D, gfx::GL_TEXTURE_MAG_FILTER,
                      static_cast<gfx::GLint>(gfx::GL_LINEAR));
    gl_.TexParameteri(gfx::GL_TEXTURE_2D, gfx::GL_TEXTURE_WRAP_S,
                      static_cast<gfx::GLint>(gfx::GL_REPEAT));
    gl_.TexParameteri(gfx::GL_TEXTURE_2D, gfx::GL_TEXTURE_WRAP_T,
                      static_cast<gfx::GLint>(gfx::GL_REPEAT));
    gl_.BindTexture(gfx::GL_TEXTURE_2D, 0);
    ready_ = layerProgram_.valid() && fadeProgram_.valid() && vbo_ != 0 && texture_ != 0;
    return ready_;
}

void Clouds::setDeck(const CloudDeck& deck, double originLatDeg, double originLonDeg) {
    deck_ = deck;
    originLatDeg_ = originLatDeg;
    originLonDeg_ = originLonDeg;
}

void Clouds::render(const gfx::Camera& camera, const Atmosphere& atmosphere) {
    lastDrawCalls_ = 0;
    lastInsideAlpha_ = 0;
    if (!ready_ || deck_.count == 0) return;
    const GeoPos& eye = camera.eyeGeo();
    const float eyeFt = static_cast<float>(mToFt(eye.altMslM));
    const CloudPlan plan = planClouds(deck_, eyeFt);

    // Eye position on the coverage pattern (east / north meters from the deck origin).
    const double mPerDeg = kEarthRadiusM * kPi / 180.0;
    const double eastM = (eye.lonDeg - originLonDeg_) * mPerDeg * std::cos(degToRad(originLatDeg_));
    const double northM = (eye.latDeg - originLatDeg_) * mPerDeg;

    // Colors: tops lit by the sun, bases darker the thicker the cover.
    const LightParams& light = atmosphere.light();
    const float sunUp = std::max(0.0f, -light.sunDirNed[2]);
    float topRgb[3];
    for (int i = 0; i < 3; ++i)
        topRgb[i] = std::min(1.0f, 1.05f * (light.ambientRgb[i] + light.diffuseRgb[i] * sunUp) +
                                       light.moonRgb[i]);
    auto baseShade = [](CloudCover c) {
        switch (c) {
        case CloudCover::Few: return 0.85f;
        case CloudCover::Scattered: return 0.8f;
        case CloudCover::Broken: return 0.7f;
        case CloudCover::Overcast: return 0.6f;
        }
        return 0.6f;
    };

    gl_.Enable(gfx::GL_DEPTH_TEST);
    gl_.DepthMask(0);
    gl_.Disable(gfx::GL_CULL_FACE);
    gl_.Enable(gfx::GL_BLEND);
    gl_.BlendFunc(gfx::GL_SRC_ALPHA, gfx::GL_ONE_MINUS_SRC_ALPHA);

    if (plan.drawCount > 0) {
        layerProgram_.use();
        const float zero[3] = {0, 0, 0};
        const gfx::Mat4 mvp =
            camera.projection() * gfx::Mat4::rotationOfTranslated(
                                      camera.ecefToEye() * enuToEcef(eye.latDeg, eye.lonDeg), zero);
        gl_.UniformMatrix4fv(uMvp_, 1, 0, mvp.m);
        const double tile1 = kCloudTileM, tile2 = kCloudTileM / kCloudDetailRatio;
        gl_.Uniform3f(uOff1_, static_cast<float>(wrapPositive(eastM, tile1)),
                      static_cast<float>(wrapPositive(northM, tile1)), 0);
        gl_.Uniform3f(uOff2_, static_cast<float>(wrapPositive(eastM, tile2)),
                      static_cast<float>(wrapPositive(northM, tile2)), 0);
        gl_.Uniform1f(uInvTile1_, static_cast<float>(1.0 / tile1));
        gl_.Uniform1f(uInvTile2_, static_cast<float>(1.0 / tile2));
        fog_.set(gl_, atmosphere.fog());
        gl_.ActiveTexture(gfx::GL_TEXTURE0);
        gl_.BindTexture(gfx::GL_TEXTURE_2D, texture_);
        gl_.Uniform1i(uNoise_, 0);
        gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, vbo_);
        gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, ibo_);
        gl_.EnableVertexAttribArray(0);
        gl_.VertexAttribPointer(0, 3, gfx::GL_FLOAT, 0, 3 * sizeof(float), nullptr);
        for (int d = 0; d < plan.drawCount; ++d) {
            const CloudDraw& draw = plan.draws[d];
            const CloudLayer& l = deck_.layers[draw.layer];
            const float shade = draw.top ? 1.0f : baseShade(l.cover);
            gl_.Uniform3f(uColor_, topRgb[0] * shade, topRgb[1] * shade, topRgb[2] * shade);
            gl_.Uniform1f(uHeight_, static_cast<float>(ftToM(draw.surfaceFtMsl - eyeFt)));
            float threshold = 0, soft = 0;
            cloudAlphaThreshold(l.cover, threshold, soft);
            gl_.Uniform1f(uThreshold_, threshold);
            gl_.Uniform1f(uSoft_, soft);
            gl_.Uniform1f(uShade_, l.cover == CloudCover::Overcast ? 0.12f : 0.2f);
            gl_.DrawElements(gfx::GL_TRIANGLES, indexCount_, gfx::GL_UNSIGNED_SHORT, nullptr);
            ++lastDrawCalls_;
        }
        gl_.DisableVertexAttribArray(0);
        gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, 0);
        gl_.BindTexture(gfx::GL_TEXTURE_2D, 0);
    }

    if (plan.insideLayer >= 0) {  // inside: fade the view to the cloud colour
        const CloudLayer& l = deck_.layers[plan.insideLayer];
        const float local = cloudAlphaAt(l.cover, cloudNoiseAt(noise_, eastM, northM));
        lastInsideAlpha_ = plan.insideDepth * local;
        if (lastInsideAlpha_ > 0.01f) {
            const float shade = 0.5f * (1.0f + baseShade(l.cover));
            gl_.Disable(gfx::GL_DEPTH_TEST);
            fadeProgram_.use();
            gl_.Uniform4f(uFadeColor_, topRgb[0] * shade, topRgb[1] * shade, topRgb[2] * shade,
                          std::min(1.0f, lastInsideAlpha_));
            gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, fadeVbo_);
            gl_.EnableVertexAttribArray(0);
            gl_.VertexAttribPointer(0, 2, gfx::GL_FLOAT, 0, 2 * sizeof(float), nullptr);
            gl_.DrawArrays(gfx::GL_TRIANGLES, 0, 6);
            gl_.DisableVertexAttribArray(0);
            gl_.Enable(gfx::GL_DEPTH_TEST);
            ++lastDrawCalls_;
        }
    }
    gl_.DepthMask(1);
    gl_.Disable(gfx::GL_BLEND);
}

}  // namespace fsim::world
