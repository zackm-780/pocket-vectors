#include "world/ExternalView.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace fsim::world {
namespace {

// Flat-shaded, lit per vertex: color * (ambient + diffuse * max(0, n . sun)), sun in body axes.
constexpr char kVertex[] = "attribute vec3 aPos;\n"
                           "attribute vec3 aNormal;\n"
                           "attribute vec3 aColor;\n"
                           "uniform mat4 uMvp;\n"
                           "uniform vec3 uSun;\n"
                           "uniform vec3 uAmbient;\n"
                           "uniform vec3 uDiffuse;\n"
                           "varying vec3 vColor;\n"
                           "void main() {\n"
                           "  float d = max(dot(aNormal, uSun), 0.0);\n"
                           "  vColor = aColor * (uAmbient + uDiffuse * d);\n"
                           "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
                           "}\n";

constexpr char kFragment[] = "varying vec3 vColor;\n"
                             "void main() {\n"
                             "  gl_FragColor = vec4(vColor, 1.0);\n"
                             "}\n";

constexpr float kFollowTauS = 0.6f;  // heading lag of the chase camera
constexpr float kMinEyeAglM = 2.0f;
constexpr int kAnimSteps = 16;  // flap / gear positions are quantized to 1/16 for rebuilds

}  // namespace

ExternalView::~ExternalView() {
    if (vbo_ != 0) gl_.DeleteBuffers(1, &vbo_);
}

bool ExternalView::init() {
    if (!program_.build(gl_, flavor_, kVertex, kFragment, {"aPos", "aNormal", "aColor"}))
        return false;
    uMvp_ = program_.uniform("uMvp");
    uSun_ = program_.uniform("uSun");
    uAmbient_ = program_.uniform("uAmbient");
    uDiffuse_ = program_.uniform("uDiffuse");
    gl_.GenBuffers(1, &vbo_);
    ready_ = program_.valid() && vbo_ != 0;
    return ready_;
}

void ExternalView::setAircraft(const AircraftProfile& profile) {
    params_ = ExternalModelParams{};
    params_.kind = externalModelFor(profile);
    params_.engineCount = profile.engineCount;
    params_.spanM = profile.wingspanM;
    fixedGear_ = !profile.hasRetractableGear;
    groundCgHeightM_ = profile.gearHeightM;
    learnGearHeight_ = profile.gearHeightM <= 0;
    dirty_ = true;
    followInit_ = false;
}

void ExternalView::placeCamera(gfx::Camera& camera, const AircraftState& s, float orbitYawRad,
                               float orbitPitchRad, const TerrainQuery* terrain, double dtS) {
    const float hdgDeg = static_cast<float>(wrap360(radToDeg(s.headingTrueRad)));
    followHdgDeg_ =
        followInit_ ? followHeadingDeg(followHdgDeg_, hdgDeg, kFollowTauS, dtS) : hdgDeg;
    followInit_ = true;
    const float span = info_.spanM > 0 ? info_.spanM : (params_.spanM > 0 ? params_.spanM : 11.0f);
    ChasePose pose = chasePose(s, followHdgDeg_, static_cast<float>(radToDeg(orbitYawRad)),
                               static_cast<float>(radToDeg(orbitPitchRad)), chaseDistanceM(span));
    if (terrain) {  // keep the eye above the ground (looking up from low behind the tail)
        const double groundM = terrain->elevationM(pose.eyeGeo.latDeg, pose.eyeGeo.lonDeg);
        if (pose.eyeGeo.altMslM < groundM + kMinEyeAglM) {
            pose.eyeGeo.altMslM = groundM + kMinEyeAglM;
            pose.eyeEcef = geodeticToEcef(pose.eyeGeo);
            GeoPos cg;
            cg.latDeg = s.latDeg;
            cg.lonDeg = s.lonDeg;
            cg.altMslM = ftToM(s.altMslFt);
            pose.ecefToEye = lookAtEcef(pose.eyeEcef, geodeticToEcef(cg));
        }
    }
    camera.setPose(pose.eyeEcef, pose.ecefToEye);
}

void ExternalView::rebuildIfNeeded(const AircraftState& s) {
    // Profiles carry the measured height on the wheels; without one, learn it on the ground.
    if (learnGearHeight_ && s.wowAny && s.altAglFt > 0)
        groundCgHeightM_ = static_cast<float>(ftToM(s.altAglFt));
    const int flapStep =
        static_cast<int>(std::lround(std::clamp(s.flapPosNorm, 0.f, 1.f) * kAnimSteps));
    const float gear = fixedGear_ ? 1.0f : std::clamp(s.gearPosNorm, 0.f, 1.f);
    const int gearStep = static_cast<int>(std::lround(gear * kAnimSteps));
    const bool heightChanged = std::fabs(groundCgHeightM_ - builtGearHeightM_) > 0.05f;
    if (!dirty_ && flapStep == builtFlapStep_ && gearStep == builtGearStep_ && !heightChanged)
        return;
    params_.flapNorm = static_cast<float>(flapStep) / kAnimSteps;
    params_.gearNorm = static_cast<float>(gearStep) / kAnimSteps;
    params_.gearHeightM = groundCgHeightM_;
    info_ = buildExternalModel(params_, verts_);
    vertexCount_ = static_cast<int>(verts_.size() / kExternalVertexFloats);
    builtFlapStep_ = flapStep;
    builtGearStep_ = gearStep;
    builtGearHeightM_ = groundCgHeightM_;
    dirty_ = false;
    if (ready_) {
        gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, vbo_);
        gl_.BufferData(gfx::GL_ARRAY_BUFFER,
                       static_cast<gfx::GLsizeiptr>(verts_.size() * sizeof(float)), verts_.data(),
                       gfx::GL_STATIC_DRAW);
    }
}

void ExternalView::render(const gfx::Camera& camera, const AircraftState& s,
                          const LightParams& light) {
    lastDrawCalls_ = 0;
    if (!s.valid) return;
    rebuildIfNeeded(s);
    if (!ready_ || vertexCount_ == 0) return;

    const Mat3d bodyToEcef =
        nedToEcef(s.latDeg, s.lonDeg) * bodyToNed(s.rollRad, s.pitchRad, s.headingTrueRad);
    GeoPos cg;
    cg.latDeg = s.latDeg;
    cg.lonDeg = s.lonDeg;
    cg.altMslM = ftToM(s.altMslFt);
    float t[3];
    camera.anchorOffset(geodeticToEcef(cg), t);  // CG - eye, ECEF axes
    // Body-axes vertices: eye = ecefToEye * (bodyToEcef * v + t) = R' * (v + bodyToEcef^T t).
    const Mat3d ecefToBody = transpose(bodyToEcef);
    const Vec3d tb = ecefToBody * Vec3d{t[0], t[1], t[2]};
    const float tBody[3] = {static_cast<float>(tb.x), static_cast<float>(tb.y),
                            static_cast<float>(tb.z)};
    const gfx::Mat4 mv = gfx::Mat4::rotationOfTranslated(camera.ecefToEye() * bodyToEcef, tBody);
    const gfx::Mat4 mvp = camera.projection() * mv;
    const Vec3d sun =
        ecefToBody * Vec3d{light.sunDirEcef[0], light.sunDirEcef[1], light.sunDirEcef[2]};

    gl_.Enable(gfx::GL_DEPTH_TEST);
    gl_.Enable(gfx::GL_CULL_FACE);  // closed convex pieces, CCW outside (default GL_BACK, GL_CCW)
    gl_.Disable(gfx::GL_BLEND);
    program_.use();
    gl_.UniformMatrix4fv(uMvp_, 1, 0, mvp.m);
    gl_.Uniform3f(uSun_, static_cast<float>(sun.x), static_cast<float>(sun.y),
                  static_cast<float>(sun.z));
    gl_.Uniform3f(uAmbient_, light.ambientRgb[0] + light.moonRgb[0],
                  light.ambientRgb[1] + light.moonRgb[1], light.ambientRgb[2] + light.moonRgb[2]);
    gl_.Uniform3f(uDiffuse_, light.diffuseRgb[0], light.diffuseRgb[1], light.diffuseRgb[2]);
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, vbo_);
    constexpr int kStride = kExternalVertexFloats * sizeof(float);
    for (int i = 0; i < 3; ++i) {
        gl_.EnableVertexAttribArray(static_cast<gfx::GLuint>(i));
        gl_.VertexAttribPointer(
            static_cast<gfx::GLuint>(i), 3, gfx::GL_FLOAT, 0, kStride,
            reinterpret_cast<const void*>(static_cast<size_t>(i) * 3 * sizeof(float)));
    }
    gl_.DrawArrays(gfx::GL_TRIANGLES, 0, vertexCount_);
    ++lastDrawCalls_;
    for (int i = 0; i < 3; ++i) gl_.DisableVertexAttribArray(static_cast<gfx::GLuint>(i));
    gl_.Disable(gfx::GL_CULL_FACE);
}

}  // namespace fsim::world
