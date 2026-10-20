#include "world/AirportRenderer.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "core/Log.h"
#include "core/Units.h"

namespace fsim::world {
using namespace fsim::gfx;  // GL constants

namespace {

// Constants missing from gfx/GlApi.h (identical in ES2 and GL 2.1). Kept local so this file does
// not depend on edits to the shared loader header.
constexpr GLenum kGlOne = 1;
constexpr GLenum kGlProgramPointSize = 0x8642;  // desktop only; always on in ES2
constexpr GLenum kGlPointSprite = 0x8861;       // desktop only; always on in ES2

constexpr float kPointDepthBias = 3e-6f;   // clip-space z bias for light sprites
constexpr float kMaxPointPx = 28.0f;

// Surface pass: per-vertex lighting and fog (fogGlsl() is used in the vertex stage only, so the
// shared uniforms never appear in both stages with different default precisions).
constexpr char kSurfaceVertexBody[] =
    "attribute vec3 aPos;\n"
    "attribute vec3 aNormal;\n"
    "attribute vec3 aColor;\n"
    "uniform mat4 uMvp;\n"
    "uniform mat4 uMv;\n"
    "uniform vec3 uSunEcef;\n"
    "uniform vec3 uAmbient;\n"
    "uniform vec3 uDiffuse;\n"
    "varying vec3 vColor;\n"
    "void main() {\n"
    "  vec4 e = uMv * vec4(aPos, 1.0);\n"
    "  float ndl = max(dot(aNormal, uSunEcef), 0.0);\n"
    "  vec3 lit = aColor * (uAmbient + uDiffuse * ndl);\n"
    "  vColor = fsimApplyFog(lit, length(e.xyz));\n"
    "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
    "}\n";

constexpr char kSurfaceFragmentBody[] = "varying vec3 vColor;\n"
                                        "void main() { gl_FragColor = vec4(vColor, 1.0); }\n";

// Light pass: point sprites. aInfo = (size in metres, facing sign, seq). seq >= 100 is a PAPI
// unit (white or red from uPapiWhite), 0..99 a sequenced flasher (on when it equals uFlashActive).
// Directional lights fade out for viewers behind them.
constexpr char kLightVertexBody[] =
    "attribute vec3 aPos;\n"
    "attribute vec3 aColor;\n"
    "attribute vec3 aInfo;\n"
    "uniform mat4 uMvp;\n"
    "uniform mat4 uMv;\n"
    "uniform vec3 uAxisEcef;\n"
    "uniform float uPointScale;\n"
    "uniform float uMaxPoint;\n"
    "uniform float uFlashActive;\n"
    "uniform vec4 uPapiWhite;\n"
    "uniform float uIntensity;\n"
    "uniform float uDepthBias;\n"
    "varying vec4 vColor;\n"
    "void main() {\n"
    "  vec4 e = uMv * vec4(aPos, 1.0);\n"
    "  float dist = length(e.xyz);\n"
    "  vec3 col = aColor;\n"
    "  float vis = 1.0;\n"
    "  float seq = aInfo.z;\n"
    "  if (seq > 99.5) {\n"
    "    float idx = seq - 100.0;\n"
    "    float w = idx < 0.5 ? uPapiWhite.x : (idx < 1.5 ? uPapiWhite.y :\n"
    "              (idx < 2.5 ? uPapiWhite.z : uPapiWhite.w));\n"
    "    col = mix(vec3(1.0, 0.10, 0.04), vec3(1.0, 0.97, 0.88), w);\n"
    "  } else if (seq > -0.5) {\n"
    "    vis = abs(seq - uFlashActive) < 0.5 ? 1.0 : 0.0;\n"
    "  }\n"
    "  if (aInfo.y != 0.0) {\n"
    "    vec3 f = normalize((uMv * vec4(uAxisEcef, 0.0)).xyz) * aInfo.y;\n"
    "    float c = dot(-e.xyz / dist, f);\n"
    "    vis *= smoothstep(-0.02, 0.30, c);\n"
    "  }\n"
    // Lights are brighter than surfaces: fog them over half the distance.
    "  float fog = fsimFogFactor(dist * 0.5);\n"
    "  vColor = vec4(col, vis * fog * uIntensity);\n"
    "  gl_PointSize = clamp(aInfo.x * uPointScale / dist, 2.0, uMaxPoint);\n"
    "  vec4 p = uMvp * vec4(aPos, 1.0);\n"
    "  p.z -= uDepthBias * p.w;\n"
    "  gl_Position = p;\n"
    "}\n";

constexpr char kLightFragmentBody[] =
    "varying vec4 vColor;\n"
    "void main() {\n"
    "  vec2 d = gl_PointCoord * 2.0 - 1.0;\n"
    "  float r = length(d);\n"
    "  if (r > 1.0) discard;\n"
    "  float a = 1.0 - smoothstep(0.45, 1.0, r);\n"
    "  gl_FragColor = vec4(vColor.rgb, vColor.a * a);\n"
    "}\n";

void toEyeUniform(const Camera& camera, const Ecef& anchor, Mat4& mv, Mat4& mvp) {
    mv = camera.modelView(anchor);
    mvp = camera.projection() * mv;
}

double distToEye(const Camera& camera, const Ecef& p) {
    const Ecef& e = camera.eyeEcef();
    return std::sqrt((p.x - e.x) * (p.x - e.x) + (p.y - e.y) * (p.y - e.y) +
                     (p.z - e.z) * (p.z - e.z));
}

}  // namespace

AirportRenderer::~AirportRenderer() {
    for (auto& a : active_) destroy(*a);
}

bool AirportRenderer::init() {
    const std::string fog = fogGlsl();
    const std::string surfaceVs = fog + kSurfaceVertexBody;
    if (!surfaceProgram_.build(gl_, flavor_, surfaceVs.c_str(), kSurfaceFragmentBody,
                               {"aPos", "aNormal", "aColor"}))
        return false;
    uSMvp_ = surfaceProgram_.uniform("uMvp");
    uSMv_ = surfaceProgram_.uniform("uMv");
    uSSun_ = surfaceProgram_.uniform("uSunEcef");
    uSAmbient_ = surfaceProgram_.uniform("uAmbient");
    uSDiffuse_ = surfaceProgram_.uniform("uDiffuse");
    surfaceFog_.locate(surfaceProgram_);

    const std::string lightVs = fog + kLightVertexBody;
    if (!lightProgram_.build(gl_, flavor_, lightVs.c_str(), kLightFragmentBody,
                             {"aPos", "aColor", "aInfo"}))
        return false;
    uLMvp_ = lightProgram_.uniform("uMvp");
    uLMv_ = lightProgram_.uniform("uMv");
    uLAxis_ = lightProgram_.uniform("uAxisEcef");
    uLScale_ = lightProgram_.uniform("uPointScale");
    uLMaxPoint_ = lightProgram_.uniform("uMaxPoint");
    uLFlash_ = lightProgram_.uniform("uFlashActive");
    uLPapi_ = lightProgram_.uniform("uPapiWhite");
    uLIntensity_ = lightProgram_.uniform("uIntensity");
    uLBias_ = lightProgram_.uniform("uDepthBias");
    lightFog_.locate(lightProgram_);
    return true;
}

void AirportRenderer::setNavDatabase(const NavDatabase* nav) {
    if (nav == nav_) return;
    nav_ = nav;
    haveSelection_ = false;  // reselect with the new database
    for (auto& a : active_) destroy(*a);
    active_.clear();
}

AirportRenderer::GpuSurface AirportRenderer::upload(const SurfaceMesh& mesh) {
    GpuSurface s;
    s.anchor = mesh.anchor;
    s.indexCount = static_cast<int>(mesh.indices.size());
    gl_.GenBuffers(1, &s.vbo);
    gl_.BindBuffer(GL_ARRAY_BUFFER, s.vbo);
    gl_.BufferData(GL_ARRAY_BUFFER,
                   static_cast<GLsizeiptr>(mesh.verts.size() * sizeof(SurfaceVertex)),
                   mesh.verts.data(), GL_STATIC_DRAW);
    gl_.GenBuffers(1, &s.ibo);
    gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, s.ibo);
    gl_.BufferData(GL_ELEMENT_ARRAY_BUFFER,
                   static_cast<GLsizeiptr>(mesh.indices.size() * sizeof(uint16_t)),
                   mesh.indices.data(), GL_STATIC_DRAW);
    return s;
}

void AirportRenderer::destroy(GpuSurface& s) {
    if (s.vbo) gl_.DeleteBuffers(1, &s.vbo);
    if (s.ibo) gl_.DeleteBuffers(1, &s.ibo);
    s = GpuSurface{};
}

void AirportRenderer::destroy(AirportGpu& a) {
    for (RunwayGpu& r : a.runways) {
        for (GpuSurface& s : r.pavement) destroy(s);
        for (GpuSurface& s : r.markings) destroy(s);
        if (r.lights.vbo) gl_.DeleteBuffers(1, &r.lights.vbo);
        r.lights = GpuLights{};
    }
    for (GpuSurface& s : a.apronFlat) destroy(s);
    for (GpuSurface& s : a.buildings) destroy(s);
}

void AirportRenderer::buildAirport(const Airport& airport) {
    auto gpu = std::make_unique<AirportGpu>();
    gpu->airport = &airport;
    GeoPos ref;
    ref.latDeg = airport.latDeg;
    ref.lonDeg = airport.lonDeg;
    ref.altMslM = ftToM(airport.elevFt);
    gpu->center = geodeticToEcef(ref);

    for (const Runway& rwy : airport.runways) {
        RunwayGpu r;
        r.info = makeRunwayInfo(airport, rwy);
        r.frame = RunwayFrame(r.info);
        r.center = runwayCenterEcef(r.info);
        RunwayMeshes meshes = buildRunwayMeshes(r.info);
        for (const SurfaceMesh& m : meshes.pavement) r.pavement.push_back(upload(m));
        for (const SurfaceMesh& m : meshes.markings) r.markings.push_back(upload(m));
        if (!meshes.lights.verts.empty()) {
            r.lights.anchor = meshes.lights.anchor;
            r.lights.count = static_cast<int>(meshes.lights.verts.size());
            r.lights.hasPapi = meshes.lights.hasPapi;
            for (int i = 0; i < 3; ++i) r.lights.axisEcef[i] = meshes.lights.axisEcef[i];
            gl_.GenBuffers(1, &r.lights.vbo);
            gl_.BindBuffer(GL_ARRAY_BUFFER, r.lights.vbo);
            gl_.BufferData(GL_ARRAY_BUFFER,
                           static_cast<GLsizeiptr>(meshes.lights.verts.size() * sizeof(LightVertex)),
                           meshes.lights.verts.data(), GL_STATIC_DRAW);
        }
        gpu->runways.push_back(std::move(r));
    }
    const AirportExtras extras = buildAirportExtras(airport);
    for (const SurfaceMesh& m : extras.flat) gpu->apronFlat.push_back(upload(m));
    for (const SurfaceMesh& m : extras.solid) gpu->buildings.push_back(upload(m));
    active_.push_back(std::move(gpu));
}

void AirportRenderer::selectActive(const Camera& camera) {
    const GeoPos& eye = camera.eyeGeo();
    selLatDeg_ = eye.latDeg;
    selLonDeg_ = eye.lonDeg;
    haveSelection_ = true;
    ++stats_.refreshes;
    if (!nav_) return;

    const std::vector<const Airport*> wanted =
        selectAirportsNear(*nav_, eye.latDeg, eye.lonDeg, kActiveRadiusM, kMaxActiveAirports);
    bool changed = wanted.size() != active_.size();
    if (!changed) {
        for (const Airport* a : wanted) {
            const bool have =
                std::any_of(active_.begin(), active_.end(),
                            [&](const std::unique_ptr<AirportGpu>& g) { return g->airport == a; });
            if (!have) {
                changed = true;
                break;
            }
        }
    }
    if (!changed) return;

    // Drop airports that left the set, then build the new ones.
    std::vector<std::unique_ptr<AirportGpu>> keep;
    for (auto& g : active_) {
        if (std::find(wanted.begin(), wanted.end(), g->airport) != wanted.end())
            keep.push_back(std::move(g));
        else
            destroy(*g);
    }
    active_ = std::move(keep);
    for (const Airport* a : wanted) {
        const bool have =
            std::any_of(active_.begin(), active_.end(),
                        [&](const std::unique_ptr<AirportGpu>& g) { return g->airport == a; });
        if (!have) buildAirport(*a);
    }

    // The flatten interface can only clear everything, so re-register the whole active set.
    if (terrain_) {
        terrain_->clearFlattenAreas();
        for (const auto& g : active_) registerAirportFlatten(*g->airport, *terrain_);
    }
}

void AirportRenderer::update(const Camera& camera) {
    const GeoPos& eye = camera.eyeGeo();
    if (haveSelection_) {
        const double moved = greatCircleDistM(selLatDeg_, selLonDeg_, eye.latDeg, eye.lonDeg);
        if (moved < kRefreshDistanceM) return;
    }
    selectActive(camera);
}

void AirportRenderer::drawSurface(const Camera& camera, const GpuSurface& s) {
    Mat4 mv, mvp;
    toEyeUniform(camera, s.anchor, mv, mvp);
    gl_.UniformMatrix4fv(uSMvp_, 1, 0, mvp.m);
    gl_.UniformMatrix4fv(uSMv_, 1, 0, mv.m);
    gl_.BindBuffer(GL_ARRAY_BUFFER, s.vbo);
    const GLsizei stride = sizeof(SurfaceVertex);
    gl_.VertexAttribPointer(0, 3, GL_FLOAT, 0, stride, reinterpret_cast<const void*>(0));
    gl_.VertexAttribPointer(1, 3, GL_FLOAT, 0, stride, reinterpret_cast<const void*>(12));
    gl_.VertexAttribPointer(2, 3, GL_FLOAT, 0, stride, reinterpret_cast<const void*>(24));
    gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, s.ibo);
    gl_.DrawElements(GL_TRIANGLES, s.indexCount, GL_UNSIGNED_SHORT, nullptr);
}

void AirportRenderer::drawSurfaces(const Camera& camera, const std::vector<GpuSurface>& list) {
    for (const GpuSurface& s : list) drawSurface(camera, s);
}

void AirportRenderer::render(const Camera& camera, int /*viewportW*/, int viewportH,
                             double timeS) {
    stats_.runwaysDrawn = 0;
    stats_.lightsDrawn = 0;
    if (!surfaceProgram_.valid() || !lightProgram_.valid()) return;
    update(camera);
    stats_.activeAirports = static_cast<int>(active_.size());
    if (active_.empty()) return;

    const FogParams fog = atmosphere_ ? atmosphere_->fog() : FogParams{};
    const LightParams light = atmosphere_ ? atmosphere_->light() : LightParams{};

    gl_.Enable(GL_DEPTH_TEST);
    gl_.DepthFunc(GL_LEQUAL);
    gl_.Disable(GL_CULL_FACE);
    gl_.Disable(GL_BLEND);
    gl_.DepthMask(1);

    // ---- surfaces, markings and buildings ----
    surfaceProgram_.use();
    surfaceFog_.set(gl_, fog);
    gl_.Uniform3f(uSSun_, light.sunDirEcef[0], light.sunDirEcef[1], light.sunDirEcef[2]);
    gl_.Uniform3f(uSAmbient_, light.ambientRgb[0], light.ambientRgb[1], light.ambientRgb[2]);
    gl_.Uniform3f(uSDiffuse_, light.diffuseRgb[0], light.diffuseRgb[1], light.diffuseRgb[2]);
    gl_.EnableVertexAttribArray(0);
    gl_.EnableVertexAttribArray(1);
    gl_.EnableVertexAttribArray(2);

    auto airportInRange = [&](const AirportGpu& a) {
        return distToEye(camera, a.center) < kSurfaceRangeM + 8000.0;
    };
    auto runwayInRange = [&](const RunwayGpu& r, double rangeM) {
        return distToEye(camera, r.center) - r.info.lengthM * 0.5 < rangeM;
    };

    gl_.Enable(GL_POLYGON_OFFSET_FILL);
    gl_.PolygonOffset(-1.0f, -1.0f);  // layer 1: apron, taxiway stub and runway pavement
    for (const auto& a : active_) {
        if (!airportInRange(*a)) continue;
        drawSurfaces(camera, a->apronFlat);
        for (const RunwayGpu& r : a->runways)
            if (runwayInRange(r, kSurfaceRangeM)) drawSurfaces(camera, r.pavement);
    }
    gl_.PolygonOffset(-2.0f, -2.0f);  // layer 2: markings on top of the pavement
    for (const auto& a : active_) {
        if (!airportInRange(*a)) continue;
        for (const RunwayGpu& r : a->runways) {
            if (!runwayInRange(r, kSurfaceRangeM)) continue;
            drawSurfaces(camera, r.markings);
            ++stats_.runwaysDrawn;
        }
    }
    gl_.Disable(GL_POLYGON_OFFSET_FILL);
    for (const auto& a : active_)
        if (airportInRange(*a)) drawSurfaces(camera, a->buildings);
    gl_.DisableVertexAttribArray(1);
    gl_.DisableVertexAttribArray(2);

    // ---- lights ----
    lightProgram_.use();
    lightFog_.set(gl_, fog);
    // Sprite size: pixels per metre at 1 m distance. Lights bloom a little at night.
    float dayF = 1.0f;
    if (atmosphere_) {
        const double elevDeg = radToDeg(atmosphere_->sun().elevationRad);
        const double t = std::min(1.0, std::max(0.0, (elevDeg + 6.0) / 16.0));
        dayF = static_cast<float>(t * t * (3.0 - 2.0 * t));
    }
    const float focalPx = static_cast<float>(viewportH) * 0.5f / std::tan(camera.vFovRad() * 0.5f);
    gl_.Uniform1f(uLScale_, focalPx * (1.6f - 0.6f * dayF));
    gl_.Uniform1f(uLMaxPoint_, kMaxPointPx);
    gl_.Uniform1f(uLIntensity_, 1.0f - 0.25f * dayF);
    gl_.Uniform1f(uLBias_, kPointDepthBias);
    gl_.Uniform1f(uLFlash_, static_cast<float>(flasherActiveIndex(timeS)));
    if (flavor_ == GlFlavor::Desktop21) {
        gl_.Enable(kGlProgramPointSize);
        gl_.Enable(kGlPointSprite);
    }
    gl_.Enable(GL_BLEND);
    gl_.BlendFunc(GL_SRC_ALPHA, kGlOne);
    gl_.DepthMask(0);
    gl_.EnableVertexAttribArray(0);
    gl_.EnableVertexAttribArray(1);
    gl_.EnableVertexAttribArray(2);
    for (const auto& a : active_) {
        if (!airportInRange(*a)) continue;
        for (const RunwayGpu& r : a->runways) {
            if (r.lights.count == 0 || !runwayInRange(r, kLightRangeM)) continue;
            Mat4 mv, mvp;
            toEyeUniform(camera, r.lights.anchor, mv, mvp);
            gl_.UniformMatrix4fv(uLMvp_, 1, 0, mvp.m);
            gl_.UniformMatrix4fv(uLMv_, 1, 0, mv.m);
            gl_.Uniform3f(uLAxis_, r.lights.axisEcef[0], r.lights.axisEcef[1],
                          r.lights.axisEcef[2]);
            unsigned mask = 0;
            if (r.lights.hasPapi)
                mask = papiWhiteMask(papiEyeElevationDeg(r.frame, r.info, camera.eyeEcef()));
            gl_.Uniform4f(uLPapi_, (mask & 1u) ? 1.0f : 0.0f, (mask & 2u) ? 1.0f : 0.0f,
                          (mask & 4u) ? 1.0f : 0.0f, (mask & 8u) ? 1.0f : 0.0f);
            gl_.BindBuffer(GL_ARRAY_BUFFER, r.lights.vbo);
            const GLsizei stride = sizeof(LightVertex);
            gl_.VertexAttribPointer(0, 3, GL_FLOAT, 0, stride, reinterpret_cast<const void*>(0));
            gl_.VertexAttribPointer(1, 3, GL_FLOAT, 0, stride, reinterpret_cast<const void*>(12));
            gl_.VertexAttribPointer(2, 3, GL_FLOAT, 0, stride, reinterpret_cast<const void*>(24));
            gl_.DrawArrays(GL_POINTS, 0, r.lights.count);
            stats_.lightsDrawn += r.lights.count;
        }
    }
    gl_.DisableVertexAttribArray(0);
    gl_.DisableVertexAttribArray(1);
    gl_.DisableVertexAttribArray(2);
    gl_.DepthMask(1);
    gl_.Disable(GL_BLEND);
}

}  // namespace fsim::world
