#include "world/TerrainRenderer.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "core/Log.h"
#include "core/Units.h"

namespace fsim::world {
namespace {

using Clock = std::chrono::steady_clock;
double msSince(Clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(Clock::now() - t0).count();
}

// Detail noise: two octaves of 3D value noise on absolute ECEF position, periodic every 256
// lattice cells so the per-node base can be reduced in double precision.
constexpr double kNoiseFreqA = 0.16;   // lattice cells per metre (features of ~6 m)
constexpr double kNoiseFreqB = 0.025;  // ~40 m
constexpr double kNoisePeriod = 256.0;

constexpr char kVertexShader[] =
    "attribute vec3 aPos;\n"
    "attribute vec3 aMorph;\n"
    "attribute vec4 aNormal;\n"
    "attribute vec4 aColor;\n"
    "uniform mat4 uMvp;\n"
    "uniform vec3 uEyeOffset;\n"  // anchor - eye in ECEF axes
    "uniform vec4 uMorph;\n"      // x = morph start distance, y = 1 / morph range
    "uniform vec3 uNoiseA;\n"
    "uniform vec3 uNoiseB;\n"
    "varying vec3 vNormal;\n"
    "varying vec4 vColor;\n"
    "varying float vDist;\n"
    "varying highp vec3 vNA;\n"
    "varying highp vec3 vNB;\n"
    "void main() {\n"
    "  float d0 = length(aPos + uEyeOffset);\n"
    "  float k = clamp((d0 - uMorph.x) * uMorph.y, 0.0, 1.0);\n"
    "  vec3 p = aPos + aMorph * k;\n"
    "  vDist = length(p + uEyeOffset);\n"
    "  vNormal = aNormal.xyz;\n"
    "  vColor = aColor;\n"
    "  vNA = p * 0.16 + uNoiseA;\n"
    "  vNB = p * 0.025 + uNoiseB;\n"
    "  gl_Position = uMvp * vec4(p, 1.0);\n"
    "}\n";

// The fog functions (W4) are pasted after the precision block.
constexpr char kFragmentHead[] = "#ifdef GL_ES\n"
                                 "#ifdef GL_FRAGMENT_PRECISION_HIGH\n"
                                 "precision highp float;\n"
                                 "#endif\n"
                                 "#endif\n";

constexpr char kFragmentBody[] =
    "uniform vec3 uSun;\n"
    "uniform vec3 uAmbient;\n"
    "uniform vec3 uDiffuse;\n"
    "uniform vec3 uWater;\n"
    "uniform vec4 uDetail;\n"  // x = amplitude, y = fade-out start A, z = fade-out end A, w = scale B
    "varying vec3 vNormal;\n"
    "varying vec4 vColor;\n"
    "varying float vDist;\n"
    "varying highp vec3 vNA;\n"
    "varying highp vec3 vNB;\n"
    "float hash3(vec3 p) {\n"
    "  p = fract(p * 0.1031);\n"
    "  p += dot(p, p.yzx + 19.19);\n"
    "  return fract((p.x + p.y) * p.z);\n"
    "}\n"
    "float vnoise(vec3 p) {\n"
    "  vec3 i = floor(p);\n"
    "  vec3 f = fract(p);\n"
    "  f = f * f * (3.0 - 2.0 * f);\n"
    "  vec3 i0 = mod(i, 256.0);\n"
    "  vec3 i1 = mod(i + 1.0, 256.0);\n"
    "  float a = mix(hash3(vec3(i0.x, i0.y, i0.z)), hash3(vec3(i1.x, i0.y, i0.z)), f.x);\n"
    "  float b = mix(hash3(vec3(i0.x, i1.y, i0.z)), hash3(vec3(i1.x, i1.y, i0.z)), f.x);\n"
    "  float c = mix(hash3(vec3(i0.x, i0.y, i1.z)), hash3(vec3(i1.x, i0.y, i1.z)), f.x);\n"
    "  float d = mix(hash3(vec3(i0.x, i1.y, i1.z)), hash3(vec3(i1.x, i1.y, i1.z)), f.x);\n"
    "  return mix(mix(a, b, f.y), mix(c, d, f.y), f.z);\n"
    "}\n"
    "void main() {\n"
    "  vec3 n = normalize(vNormal);\n"
    "  float ndl = max(dot(n, uSun), 0.0);\n"
    "  vec3 albedo = vColor.rgb;\n"
    "  float water = smoothstep(0.4, 0.6, vColor.a);\n"
    "  float fadeA = 1.0 - smoothstep(uDetail.y, uDetail.z, vDist);\n"
    "  float fadeB = 1.0 - smoothstep(uDetail.y * 4.0, uDetail.z * 5.0, vDist);\n"
    "  float t = 0.0;\n"
    "  if (fadeB > 0.01) t += (vnoise(vNB) - 0.5) * fadeB * 0.9;\n"
    "  if (fadeA > 0.01) t += (vnoise(vNA) - 0.5) * fadeA;\n"
    "  albedo *= 1.0 + uDetail.x * t * (1.0 - water);\n"
    "  vec3 land = albedo * (uAmbient + uDiffuse * ndl);\n"
    "  vec3 sea = uWater * (uAmbient + uDiffuse * 0.25);\n"
    "  vec3 rgb = mix(land, sea, water);\n"
    "  gl_FragColor = vec4(fsimApplyFog(rgb, vDist), 1.0);\n"
    "}\n";

constexpr float kWaterRgb[3] = {0.04f, 0.10f, 0.18f};
constexpr double kSeaRadiusM = 500000.0;
constexpr double kSeaRebuildM = 25000.0;
constexpr float kNoMorphStartM = 1e30f;

}  // namespace

std::string TerrainRenderStats::summary() const {
    char buf[320];
    std::snprintf(buf, sizeof buf,
                  "terrain: %d nodes (L%d..L%d) %d tris %d draws | built %d queue %d cache %d "
                  "(%.1f MB) | cull f%d h%d sea%d | select %.2f build %.2f draw %.2f ms",
                  nodesDrawn, minLevel, maxLevel, triangles, drawCalls, meshesBuilt, buildQueue,
                  cachedMeshes, static_cast<double>(gpuBytes) / (1024.0 * 1024.0), culledFrustum,
                  culledHorizon, seaSkipped, selectMs, buildMs, drawMs);
    return buf;
}

TerrainRenderer::TerrainRenderer(const gfx::GlApi& gl, gfx::GlFlavor flavor,
                                 terrain::TerrainService& terrain)
    : gl_(gl), flavor_(flavor), svc_(terrain), selector_(terrain) {}

TerrainRenderer::~TerrainRenderer() {
    std::vector<uint32_t> vbos;
    selector_.clear(vbos);
    deleteBuffers(vbos);
    if (ibo_) gl_.DeleteBuffers(1, &ibo_);
    if (seaVbo_) gl_.DeleteBuffers(1, &seaVbo_);
    if (seaIbo_) gl_.DeleteBuffers(1, &seaIbo_);
}

void TerrainRenderer::deleteBuffers(const std::vector<uint32_t>& vbos) {
    for (uint32_t v : vbos) {
        const gfx::GLuint id = v;
        gl_.DeleteBuffers(1, &id);
        --vboCount_;
    }
}

void TerrainRenderer::setConfig(const TerrainRenderConfig& cfg) {
    cfg_ = cfg;
    selector_.setConfig(cfg.lod);
}

bool TerrainRenderer::init() {
    const std::string fragment = std::string(kFragmentHead) + fogGlsl() + kFragmentBody;
    if (!program_.build(gl_, flavor_, kVertexShader, fragment.c_str(),
                        {"aPos", "aMorph", "aNormal", "aColor"}))
        return false;
    loc_.mvp = program_.uniform("uMvp");
    loc_.eyeOffset = program_.uniform("uEyeOffset");
    loc_.morph = program_.uniform("uMorph");
    loc_.noiseA = program_.uniform("uNoiseA");
    loc_.noiseB = program_.uniform("uNoiseB");
    loc_.sun = program_.uniform("uSun");
    loc_.ambient = program_.uniform("uAmbient");
    loc_.diffuse = program_.uniform("uDiffuse");
    loc_.water = program_.uniform("uWater");
    loc_.detail = program_.uniform("uDetail");
    fogUniforms_.locate(program_);
    const std::vector<uint16_t>& ix = nodeIndices();
    gl_.GenBuffers(1, &ibo_);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, ibo_);
    gl_.BufferData(gfx::GL_ELEMENT_ARRAY_BUFFER, static_cast<gfx::GLsizeiptr>(ix.size() * 2),
                   ix.data(), gfx::GL_STATIC_DRAW);
    ready_ = true;
    return true;
}

ViewParams TerrainRenderer::makeView(const gfx::Camera& camera) const {
    ViewParams v;
    v.eye = camera.eyeEcef();
    v.eyeGeo = camera.eyeGeo();
    v.ecefToEye = camera.ecefToEye();
    v.vFovRad = camera.vFovRad();
    v.aspect = camera.aspect();
    v.nearM = camera.nearM();
    v.farM = camera.farM();
    return v;
}

bool TerrainRenderer::buildOne(const BuildRequest& req) {
    NodeRecord* rec = selector_.find(req.id);
    if (!rec) return false;
    MeshBuildOptions opt;
    opt.coarsestTileLevel = svc_.coarsestLevel();
    opt.flatten = &svc_.flattenSet();
    NodeMesh mesh;
    const TileLookupFn lookup = [this](const terrain::TileId& id) { return svc_.tile(id); };
    if (!buildNodeMesh(req.id, lookup, opt, mesh)) {
        selector_.markBuildFailed(*rec);
        return false;
    }
    if (!rec->vbo) {
        gfx::GLuint id = 0;
        gl_.GenBuffers(1, &id);
        rec->vbo = id;
        ++vboCount_;
    }
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, rec->vbo);
    gl_.BufferData(gfx::GL_ARRAY_BUFFER,
                   static_cast<gfx::GLsizeiptr>(mesh.vertices.size() * sizeof(NodeVertex)),
                   mesh.vertices.data(), gfx::GL_STATIC_DRAW);
    selector_.applyMesh(*rec, mesh, rec->vbo);
    ++stats_.totalMeshBuilds;
    return true;
}

int TerrainRenderer::buildQueued(int maxBuilds) {
    std::sort(builds_.begin(), builds_.end(), [](const BuildRequest& a, const BuildRequest& b) {
        if (a.priority != b.priority) return a.priority < b.priority;
        return a.distM < b.distM;
    });
    int built = 0;
    for (const BuildRequest& r : builds_) {
        if (built >= maxBuilds) break;
        if (buildOne(r)) ++built;
    }
    return built;
}

void TerrainRenderer::collect() {
    garbage_.clear();
    selector_.collectGarbage(garbage_);
    deleteBuffers(garbage_);
}

int TerrainRenderer::prewarm(const gfx::Camera& camera, int maxMeshes) {
    if (!ready_) return 0;
    const ViewParams view = makeView(camera);
    int total = 0;
    for (int pass = 0; pass < 200 && total < maxMeshes; ++pass) {
        selector_.select(view, draw_, builds_);
        if (builds_.empty()) break;
        const int n = buildQueued(maxMeshes - total);
        if (n == 0) break;
        total += n;
    }
    return total;
}

void TerrainRenderer::setNoiseUniforms(const Ecef& a) {
    auto base = [](double v, double f) {
        return static_cast<float>(std::fmod(v * f, kNoisePeriod));
    };
    gl_.Uniform3f(loc_.noiseA, base(a.x, kNoiseFreqA), base(a.y, kNoiseFreqA),
                  base(a.z, kNoiseFreqA));
    gl_.Uniform3f(loc_.noiseB, base(a.x, kNoiseFreqB), base(a.y, kNoiseFreqB),
                  base(a.z, kNoiseFreqB));
}

void TerrainRenderer::drawNode(const gfx::Camera& camera, const gfx::Mat4& proj,
                               const NodeRecord& rec, const DrawNode& dn) {
    const gfx::Mat4 mv = camera.modelView(rec.anchor);
    const gfx::Mat4 mvp = proj * mv;
    float t[3];
    camera.anchorOffset(rec.anchor, t);
    gl_.UniformMatrix4fv(loc_.mvp, 1, 0, mvp.m);
    gl_.Uniform3f(loc_.eyeOffset, t[0], t[1], t[2]);
    gl_.Uniform4f(loc_.morph, dn.morphStartM, dn.morphInvRangeM, 0, 0);
    setNoiseUniforms(rec.anchor);
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, rec.vbo);
    gl_.VertexAttribPointer(0, 3, gfx::GL_FLOAT, 0, sizeof(NodeVertex), nullptr);
    gl_.VertexAttribPointer(1, 3, gfx::GL_FLOAT, 0, sizeof(NodeVertex),
                            reinterpret_cast<const void*>(12));
    gl_.VertexAttribPointer(2, 4, gfx::GL_BYTE, 1, sizeof(NodeVertex),
                            reinterpret_cast<const void*>(24));
    gl_.VertexAttribPointer(3, 4, gfx::GL_UNSIGNED_BYTE, 1, sizeof(NodeVertex),
                            reinterpret_cast<const void*>(28));
    gl_.DrawElements(gfx::GL_TRIANGLES, kNodeIndexCount, gfx::GL_UNSIGNED_SHORT, nullptr);
}

void TerrainRenderer::rebuildSea(const GeoPos& center) {
    seaCenter_ = center;
    seaCenter_.altMslM = 0;
    seaAnchor_ = geodeticToEcef(seaCenter_);
    constexpr int kSegments = 48;
    std::vector<double> radii = {0.0};
    for (double r = 250.0; r < kSeaRadiusM * 1.0001; r *= 1.6)
        radii.push_back(std::min(r, kSeaRadiusM));
    if (radii.back() < kSeaRadiusM) radii.push_back(kSeaRadiusM);

    std::vector<NodeVertex> verts;
    auto addVertex = [&](double latDeg, double lonDeg) {
        GeoPos p;
        p.latDeg = latDeg;
        p.lonDeg = lonDeg;
        p.altMslM = 0;
        const Ecef e = geodeticToEcef(p);
        NodeVertex v{};
        v.pos[0] = static_cast<float>(e.x - seaAnchor_.x);
        v.pos[1] = static_cast<float>(e.y - seaAnchor_.y);
        v.pos[2] = static_cast<float>(e.z - seaAnchor_.z);
        const double la = latDeg * kRadPerDeg, lo = lonDeg * kRadPerDeg;
        v.normal[0] = static_cast<int8_t>(std::lround(std::cos(la) * std::cos(lo) * 127));
        v.normal[1] = static_cast<int8_t>(std::lround(std::cos(la) * std::sin(lo) * 127));
        v.normal[2] = static_cast<int8_t>(std::lround(std::sin(la) * 127));
        for (int i = 0; i < 3; ++i)
            v.color[i] = static_cast<uint8_t>(std::lround(kWaterRgb[i] * 255.0f));
        v.color[3] = 255;
        verts.push_back(v);
    };
    addVertex(seaCenter_.latDeg, seaCenter_.lonDeg);
    for (size_t r = 1; r < radii.size(); ++r) {
        for (int s = 0; s < kSegments; ++s) {
            double lat, lon;
            destinationPoint(seaCenter_.latDeg, seaCenter_.lonDeg, kTwoPi * s / kSegments, radii[r],
                             lat, lon);
            addVertex(lat, lon);
        }
    }
    std::vector<uint16_t> ix;
    auto vi = [&](size_t ring, int seg) {
        return static_cast<uint16_t>(1 + (ring - 1) * kSegments + (seg % kSegments));
    };
    for (int s = 0; s < kSegments; ++s) {
        ix.insert(ix.end(), {0, vi(1, s), vi(1, s + 1)});
    }
    for (size_t r = 1; r + 1 < radii.size(); ++r) {
        for (int s = 0; s < kSegments; ++s) {
            ix.insert(ix.end(), {vi(r, s), vi(r + 1, s), vi(r + 1, s + 1), vi(r, s),
                                 vi(r + 1, s + 1), vi(r, s + 1)});
        }
    }
    if (!seaVbo_) gl_.GenBuffers(1, &seaVbo_);
    if (!seaIbo_) gl_.GenBuffers(1, &seaIbo_);
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, seaVbo_);
    gl_.BufferData(gfx::GL_ARRAY_BUFFER,
                   static_cast<gfx::GLsizeiptr>(verts.size() * sizeof(NodeVertex)), verts.data(),
                   gfx::GL_STATIC_DRAW);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, seaIbo_);
    gl_.BufferData(gfx::GL_ELEMENT_ARRAY_BUFFER, static_cast<gfx::GLsizeiptr>(ix.size() * 2),
                   ix.data(), gfx::GL_STATIC_DRAW);
    seaIndexCount_ = static_cast<int>(ix.size());
    haveSea_ = true;
}

void TerrainRenderer::drawSea(const gfx::Camera& camera, const gfx::Mat4& proj) {
    const GeoPos& eye = camera.eyeGeo();
    if (!haveSea_ || greatCircleDistM(eye.latDeg, eye.lonDeg, seaCenter_.latDeg,
                                      seaCenter_.lonDeg) > kSeaRebuildM)
        rebuildSea(eye);
    const gfx::Mat4 mvp = proj * camera.modelView(seaAnchor_);
    float t[3];
    camera.anchorOffset(seaAnchor_, t);
    gl_.UniformMatrix4fv(loc_.mvp, 1, 0, mvp.m);
    gl_.Uniform3f(loc_.eyeOffset, t[0], t[1], t[2]);
    gl_.Uniform4f(loc_.morph, kNoMorphStartM, 0, 0, 0);
    setNoiseUniforms(seaAnchor_);
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, seaVbo_);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, seaIbo_);
    gl_.VertexAttribPointer(0, 3, gfx::GL_FLOAT, 0, sizeof(NodeVertex), nullptr);
    gl_.VertexAttribPointer(1, 3, gfx::GL_FLOAT, 0, sizeof(NodeVertex),
                            reinterpret_cast<const void*>(12));
    gl_.VertexAttribPointer(2, 4, gfx::GL_BYTE, 1, sizeof(NodeVertex),
                            reinterpret_cast<const void*>(24));
    gl_.VertexAttribPointer(3, 4, gfx::GL_UNSIGNED_BYTE, 1, sizeof(NodeVertex),
                            reinterpret_cast<const void*>(28));
    // Pushed back in depth so terrain at sea level (Water tiles, coasts) always wins, at any range.
    gl_.Enable(gfx::GL_POLYGON_OFFSET_FILL);
    gl_.PolygonOffset(2.0f, 8.0f);
    gl_.DrawElements(gfx::GL_TRIANGLES, seaIndexCount_, gfx::GL_UNSIGNED_SHORT, nullptr);
    gl_.Disable(gfx::GL_POLYGON_OFFSET_FILL);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, ibo_);
    ++stats_.drawCalls;
    stats_.triangles += seaIndexCount_ / 3;
}

void TerrainRenderer::render(const gfx::Camera& camera, const Atmosphere* atmosphere) {
    if (!ready_) return;
    const uint64_t builtBefore = stats_.totalMeshBuilds;
    const uint64_t total = stats_.totalMeshBuilds;
    stats_ = TerrainRenderStats{};
    stats_.totalMeshBuilds = total;

    auto t0 = Clock::now();
    const ViewParams view = makeView(camera);
    selector_.select(view, draw_, builds_);
    stats_.selectMs = msSince(t0);

    t0 = Clock::now();
    if (!builds_.empty()) {
        const int built = buildQueued(cfg_.uploadsPerFrame);
        stats_.buildQueue = static_cast<int>(builds_.size()) - built;
        stats_.buildMs = msSince(t0);
        if (built > 0) {
            const auto t1 = Clock::now();
            selector_.select(view, draw_, builds_);  // newly built nodes may now be drawn
            stats_.selectMs += msSince(t1);
            stats_.buildQueue = static_cast<int>(builds_.size());
        }
    }
    stats_.meshesBuilt = static_cast<int>(stats_.totalMeshBuilds - builtBefore);
    if ((selector_.frame() % 30) == 0) collect();

    t0 = Clock::now();
    FogParams fogDefault;
    LightParams lightDefault;
    const FogParams& fog = atmosphere ? atmosphere->fog() : fogDefault;
    const LightParams& light = atmosphere ? atmosphere->light() : lightDefault;

    gl_.Enable(gfx::GL_DEPTH_TEST);
    gl_.DepthFunc(gfx::GL_LESS);
    gl_.DepthMask(1);
    gl_.Disable(gfx::GL_CULL_FACE);
    gl_.Disable(gfx::GL_BLEND);
    program_.use();
    fogUniforms_.set(gl_, fog);
    gl_.Uniform3f(loc_.sun, light.sunDirEcef[0], light.sunDirEcef[1], light.sunDirEcef[2]);
    gl_.Uniform3f(loc_.ambient, light.ambientRgb[0], light.ambientRgb[1], light.ambientRgb[2]);
    gl_.Uniform3f(loc_.diffuse, light.diffuseRgb[0], light.diffuseRgb[1], light.diffuseRgb[2]);
    gl_.Uniform3f(loc_.water, kWaterRgb[0], kWaterRgb[1], kWaterRgb[2]);
    gl_.Uniform4f(loc_.detail, 0.28f, 150.0f, 1500.0f, 0);
    for (int i = 0; i < 4; ++i) gl_.EnableVertexAttribArray(static_cast<gfx::GLuint>(i));
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, ibo_);

    const gfx::Mat4 proj = camera.projection();
    if (cfg_.seaDisc) drawSea(camera, proj);
    stats_.minLevel = 99;
    for (const DrawNode& dn : draw_) {
        drawNode(camera, proj, *dn.rec, dn);
        ++stats_.nodesDrawn;
        ++stats_.drawCalls;
        stats_.triangles += kNodeTriangleCount;
        stats_.minLevel = std::min(stats_.minLevel, dn.rec->id.k);
        stats_.maxLevel = std::max(stats_.maxLevel, dn.rec->id.k);
    }
    if (stats_.nodesDrawn == 0) stats_.minLevel = 0;
    for (int i = 0; i < 4; ++i) gl_.DisableVertexAttribArray(static_cast<gfx::GLuint>(i));
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, 0);
    gl_.BindBuffer(gfx::GL_ELEMENT_ARRAY_BUFFER, 0);
    stats_.drawMs = msSince(t0);

    const SelectionStats& ss = selector_.stats();
    stats_.visited = ss.visited;
    stats_.culledFrustum = ss.culledFrustum;
    stats_.culledHorizon = ss.culledHorizon;
    stats_.seaSkipped = ss.seaSkipped;
    stats_.cachedMeshes = vboCount_;
    stats_.gpuBytes = static_cast<size_t>(vboCount_) * kNodeVertexCount * sizeof(NodeVertex);
}

TerrainDebugOverrides readTerrainDebugEnv() {
    TerrainDebugOverrides o;
    if (const char* cam = std::getenv("FSIM_W2_CAMERA")) {
        double v[5] = {0, 0, 0, 0, 0};
        const int n = std::sscanf(cam, "%lf,%lf,%lf,%lf,%lf", &v[0], &v[1], &v[2], &v[3], &v[4]);
        if (n >= 4) {
            o.hasCamera = true;
            o.latDeg = v[0], o.lonDeg = v[1], o.altMslFt = v[2], o.headingTrueDeg = v[3];
            o.pitchDeg = n >= 5 ? v[4] : 0;
        }
    }
    if (const char* vis = std::getenv("FSIM_W2_VISIBILITY_KM")) {
        o.visibilityKm = static_cast<float>(std::atof(vis));
    }
    if (const char* px = std::getenv("FSIM_W2_PX_PER_CELL"))
        o.pixelsPerCell = static_cast<float>(std::atof(px));
    return o;
}

}  // namespace fsim::world
