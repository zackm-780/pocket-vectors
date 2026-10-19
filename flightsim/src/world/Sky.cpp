#include "world/Sky.h"

#include <cmath>

#include "core/Units.h"

namespace fsim::world {
using namespace fsim::gfx;  // GL constants
namespace {

// Visual sun radius (real sun: 0.27 deg). Slightly enlarged so it reads at 640x480.
constexpr double kSunRadiusInnerDeg = 0.38;
constexpr double kSunRadiusOuterDeg = 0.50;

constexpr char kVertexShader[] = "attribute vec2 aPos;\n"
                                 "uniform vec4 uTanHalf;\n"
                                 "varying vec3 vRay;\n"
                                 "void main() {\n"
                                 "  vRay = vec3(aPos * uTanHalf.xy, -1.0);\n"
                                 "  gl_Position = vec4(aPos, 1.0, 1.0);\n"
                                 "}\n";

// The sun disc needs 1 - cos(0.4 deg) ~ 2.4e-5, far below mediump resolution: use highp where the
// fragment stage has it (Mali G31 does) and fall back to a coarse blob elsewhere.
constexpr char kFragmentShader[] =
    "#ifdef GL_FRAGMENT_PRECISION_HIGH\n"
    "#define HP highp\n"
    "#else\n"
    "#define HP mediump\n"
    "#endif\n"
    "varying HP vec3 vRay;\n"
    "uniform HP vec3 uUp;\n"
    "uniform HP vec3 uSun;\n"
    "uniform HP vec4 uDiscCos;\n"
    "uniform vec3 uHaze;\n"
    "uniform vec3 uZenith;\n"
    "uniform vec3 uDisc;\n"
    "uniform vec3 uGlow;\n"
    // x = sin(horizon dip), y = cos(horizon dip), z = haze band sharpness, w = glow strength
    "uniform vec4 uParams;\n"
    "void main() {\n"
    "  HP vec3 d = normalize(vRay);\n"
    "  HP float sinEl = clamp(dot(d, uUp), -1.0, 1.0);\n"
    // sin(elevation above the geometric horizon) = sin(el + dip)
    "  HP float x = sinEl * uParams.y + sqrt(max(1.0 - sinEl * sinEl, 0.0)) * uParams.x;\n"
    "  float above = max(x, 0.0);\n"
    "  vec3 col = mix(uZenith, uHaze, exp(-above * uParams.z));\n"
    "  HP float c = dot(d, uSun);\n"
    "  float g = max(c, 0.0);\n"
    "  g = g * g;\n"
    "  g = g * g;\n"
    "  g = g * g;\n"
    "  float g64 = g * g;\n"
    "  g64 = g64 * g64;\n"
    "  g64 = g64 * g64;\n"
    // glow fades to zero at the horizon so it never breaks the match with the fog color
    "  float skyMask = smoothstep(0.0, 0.07, x);\n"
    "  col += uGlow * uParams.w * (0.5 * g + 1.5 * g64) * skyMask;\n"
    "  float disc = smoothstep(uDiscCos.x, uDiscCos.y, c) * smoothstep(0.0, 0.004, x);\n"
    "  col = mix(col, uDisc, disc);\n"
    "  gl_FragColor = vec4(col, 1.0);\n"
    "}\n";

}  // namespace

Sky::~Sky() {
    if (vbo_) gl_.DeleteBuffers(1, &vbo_);
}

bool Sky::init() {
    if (!program_.build(gl_, flavor_, kVertexShader, kFragmentShader, {"aPos"})) return false;
    uTanHalf_ = program_.uniform("uTanHalf");
    uUp_ = program_.uniform("uUp");
    uSun_ = program_.uniform("uSun");
    uHaze_ = program_.uniform("uHaze");
    uZenith_ = program_.uniform("uZenith");
    uDisc_ = program_.uniform("uDisc");
    uGlow_ = program_.uniform("uGlow");
    uParams_ = program_.uniform("uParams");
    uDiscCos_ = program_.uniform("uDiscCos");
    static const float kTriangle[6] = {-1, -1, 3, -1, -1, 3};  // covers the whole viewport
    gl_.GenBuffers(1, &vbo_);
    gl_.BindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_.BufferData(GL_ARRAY_BUFFER, sizeof kTriangle, kTriangle, GL_STATIC_DRAW);
    return true;
}

void Sky::draw(const gfx::Camera& camera, const Atmosphere& atmosphere) {
    if (!program_.valid()) return;
    const FogParams& fog = atmosphere.fog();
    const SkyColors& sky = atmosphere.sky();
    const LightParams& light = atmosphere.light();

    // Local up (ellipsoid normal) and the sun in eye axes.
    const GeoPos& eye = camera.eyeGeo();
    const double lat = degToRad(eye.latDeg), lon = degToRad(eye.lonDeg);
    const Vec3d upEye = camera.ecefDirToEye(
        {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)});
    const Vec3d sunEye =
        camera.ecefDirToEye({light.sunDirEcef[0], light.sunDirEcef[1], light.sunDirEcef[2]});

    const float tanHalf = std::tan(camera.vFovRad() * 0.5f);
    gl_.Disable(GL_DEPTH_TEST);
    gl_.Disable(GL_CULL_FACE);
    program_.use();
    gl_.Uniform4f(uTanHalf_, tanHalf * camera.aspect(), tanHalf, 0, 0);
    gl_.Uniform3f(uUp_, static_cast<float>(upEye.x), static_cast<float>(upEye.y),
                  static_cast<float>(upEye.z));
    gl_.Uniform3f(uSun_, static_cast<float>(sunEye.x), static_cast<float>(sunEye.y),
                  static_cast<float>(sunEye.z));
    gl_.Uniform3f(uHaze_, fog.colorRgb[0], fog.colorRgb[1], fog.colorRgb[2]);
    gl_.Uniform3f(uZenith_, sky.zenithRgb[0], sky.zenithRgb[1], sky.zenithRgb[2]);
    gl_.Uniform3f(uDisc_, sky.sunDiscRgb[0], sky.sunDiscRgb[1], sky.sunDiscRgb[2]);
    gl_.Uniform3f(uGlow_, sky.glowRgb[0], sky.glowRgb[1], sky.glowRgb[2]);
    gl_.Uniform4f(uParams_, std::sin(sky.horizonDipRad), std::cos(sky.horizonDipRad),
                  sky.bandSharpness, sky.glowStrength);
    gl_.Uniform4f(uDiscCos_, static_cast<float>(std::cos(degToRad(kSunRadiusOuterDeg))),
                  static_cast<float>(std::cos(degToRad(kSunRadiusInnerDeg))), 0, 0);

    gl_.BindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_.EnableVertexAttribArray(0);
    gl_.VertexAttribPointer(0, 2, GL_FLOAT, 0, 2 * sizeof(float), nullptr);
    gl_.DrawArrays(GL_TRIANGLES, 0, 3);
    gl_.DisableVertexAttribArray(0);
    gl_.Enable(GL_DEPTH_TEST);
}

}  // namespace fsim::world
