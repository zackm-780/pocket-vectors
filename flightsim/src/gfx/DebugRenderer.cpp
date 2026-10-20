#include "gfx/DebugRenderer.h"

#include <algorithm>
#include <cmath>

#include "core/Log.h"
#include "core/Units.h"
#include "world/Sky.h"  // W4

namespace fsim::gfx {
namespace {

// Ground grid: (2*kHalfCells+1)^2 vertices around the camera, snapped to world cells.
constexpr int kHalfCells = 24;
constexpr int kSideVerts = 2 * kHalfCells + 1;
constexpr double kCellM = 1000.0;
constexpr double kFogDensity = 1.5e-4;  // ~97 % fogged at 24 km, hiding the grid edge
constexpr float kFogColor[3] = {0.72f, 0.80f, 0.90f};
constexpr float kSkyColor[3] = {0.40f, 0.60f, 0.86f};

constexpr float kGroundColor[4] = {0.30f, 0.47f, 0.24f, 1};
constexpr float kGridColor[4] = {0.20f, 0.33f, 0.17f, 1};
constexpr float kRunwayColor[4] = {0.22f, 0.22f, 0.24f, 1};
constexpr float kMarkingColor[4] = {0.92f, 0.92f, 0.90f, 1};

constexpr char kVertexShader[] = "attribute vec3 aPos;\n"
                                 "uniform mat4 uMvp;\n"
                                 "uniform mat4 uMv;\n"
                                 "uniform float uFogDensity;\n"
                                 "varying float vFog;\n"
                                 "void main() {\n"
                                 "  vec4 e = uMv * vec4(aPos, 1.0);\n"
                                 "  vFog = exp(-uFogDensity * length(e.xyz));\n"
                                 "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
                                 "}\n";

constexpr char kFragmentShader[] = "uniform vec4 uColor;\n"
                                   "uniform vec3 uFogColor;\n"
                                   "varying float vFog;\n"
                                   "void main() {\n"
                                   "  gl_FragColor = vec4(mix(uFogColor, uColor.rgb, vFog), 1.0);\n"
                                   "}\n";

constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

}  // namespace

DebugRenderer::~DebugRenderer() {
    destroyMesh(ground_);
    for (Mesh& m : runwaySurfaces_) destroyMesh(m);
    for (Mesh& m : runwayMarkings_) destroyMesh(m);
}

bool DebugRenderer::init() {
    if (!program_.build(gl_, flavor_, kVertexShader, kFragmentShader, {"aPos"})) return false;
    uMvp_ = program_.uniform("uMvp");
    uMv_ = program_.uniform("uMv");
    uColor_ = program_.uniform("uColor");
    fogUniforms_.locate(program_);  // W4
    return true;
}

void DebugRenderer::destroyMesh(Mesh& mesh) {
    if (mesh.vbo) gl_.DeleteBuffers(1, &mesh.vbo);
    if (mesh.ibo) gl_.DeleteBuffers(1, &mesh.ibo);
    if (mesh.lineIbo) gl_.DeleteBuffers(1, &mesh.lineIbo);
    mesh = Mesh{};
}

void DebugRenderer::uploadMesh(Mesh& mesh, const std::vector<float>& vertices,
                               const std::vector<unsigned short>& triIndices,
                               const std::vector<unsigned short>& lineIndices) {
    if (!mesh.vbo) gl_.GenBuffers(1, &mesh.vbo);
    gl_.BindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    gl_.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices.size() * sizeof(float)),
                   vertices.data(), GL_STATIC_DRAW);
    if (!triIndices.empty()) {
        if (!mesh.ibo) gl_.GenBuffers(1, &mesh.ibo);
        gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ibo);
        gl_.BufferData(GL_ELEMENT_ARRAY_BUFFER,
                       static_cast<GLsizeiptr>(triIndices.size() * sizeof(unsigned short)),
                       triIndices.data(), GL_STATIC_DRAW);
    }
    if (!lineIndices.empty()) {
        if (!mesh.lineIbo) gl_.GenBuffers(1, &mesh.lineIbo);
        gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.lineIbo);
        gl_.BufferData(GL_ELEMENT_ARRAY_BUFFER,
                       static_cast<GLsizeiptr>(lineIndices.size() * sizeof(unsigned short)),
                       lineIndices.data(), GL_STATIC_DRAW);
    }
    mesh.indexCount = static_cast<int>(triIndices.size());
    mesh.lineIndexCount = static_cast<int>(lineIndices.size());
}

void DebugRenderer::setWorld(const Airport& airport, float groundElevM) {
    airport_ = airport;
    groundElevM_ = groundElevM;
    haveWorld_ = true;
    groundCellN_ = groundCellE_ = 0x7fffffff;  // force a rebuild
    for (Mesh& m : runwaySurfaces_) destroyMesh(m);
    for (Mesh& m : runwayMarkings_) destroyMesh(m);
    runwaySurfaces_.clear();
    runwayMarkings_.clear();
    for (const Runway& r : airport_.runways) {
        // Both landing directions share one pavement: draw it once.
        if (drawRunways_ && r.ident < r.oppositeIdent) buildRunway(r);  // W5
    }
}

void DebugRenderer::rebuildGround(int cellN, int cellE) {
    groundCellN_ = cellN;
    groundCellE_ = cellE;
    const double lat0 = airport_.latDeg, lon0 = airport_.lonDeg;
    const double mPerDegLon = kMPerDegLat * std::cos(degToRad(lat0));

    auto vertexEcef = [&](int a, int b) {
        const double n = (cellN + a - kHalfCells) * kCellM;
        const double e = (cellE + b - kHalfCells) * kCellM;
        GeoPos p;
        p.latDeg = lat0 + n / kMPerDegLat;
        p.lonDeg = lon0 + e / mPerDegLon;
        p.altMslM = groundElevM_;
        return geodeticToEcef(p);
    };

    ground_.anchor = vertexEcef(kHalfCells, kHalfCells);
    std::vector<float> verts;
    verts.reserve(static_cast<size_t>(kSideVerts) * kSideVerts * 3);
    for (int a = 0; a < kSideVerts; ++a) {
        for (int b = 0; b < kSideVerts; ++b) {
            const Ecef p = vertexEcef(a, b);  // float offsets from the double anchor (RTC)
            verts.push_back(static_cast<float>(p.x - ground_.anchor.x));
            verts.push_back(static_cast<float>(p.y - ground_.anchor.y));
            verts.push_back(static_cast<float>(p.z - ground_.anchor.z));
        }
    }
    std::vector<unsigned short> tris, lines;
    auto idx = [](int a, int b) { return static_cast<unsigned short>(a * kSideVerts + b); };
    for (int a = 0; a < kSideVerts - 1; ++a) {
        for (int b = 0; b < kSideVerts - 1; ++b) {
            tris.insert(tris.end(), {idx(a, b), idx(a, b + 1), idx(a + 1, b), idx(a + 1, b),
                                     idx(a, b + 1), idx(a + 1, b + 1)});
        }
    }
    for (int a = 0; a < kSideVerts; ++a) {
        for (int b = 0; b < kSideVerts; ++b) {
            if (b + 1 < kSideVerts) lines.insert(lines.end(), {idx(a, b), idx(a, b + 1)});
            if (a + 1 < kSideVerts) lines.insert(lines.end(), {idx(a, b), idx(a + 1, b)});
        }
    }
    uploadMesh(ground_, verts, tris, lines);
}

void DebugRenderer::buildRunway(const Runway& rwy) {
    const double headingRad = degToRad(rwy.headingTrueDeg);
    const double lengthM =
        greatCircleDistM(rwy.startLatDeg, rwy.startLonDeg, rwy.endLatDeg, rwy.endLonDeg);
    const double halfWidthM = ftToM(rwy.widthFt) * 0.5;
    const double elevM = ftToM(rwy.thrElevFt);

    // Point at (alongM from the pavement start, lateralM right of the centerline, liftM up).
    auto pointEcef = [&](double alongM, double lateralM, double liftM) {
        double lat, lon;
        destinationPoint(rwy.startLatDeg, rwy.startLonDeg, headingRad, alongM, lat, lon);
        if (lateralM != 0.0)
            destinationPoint(lat, lon, headingRad + (lateralM > 0 ? kPi / 2 : -kPi / 2),
                             std::fabs(lateralM), lat, lon);
        GeoPos p;
        p.latDeg = lat;
        p.lonDeg = lon;
        p.altMslM = elevM + liftM;
        return geodeticToEcef(p);
    };

    Mesh surface, markings;
    GeoPos centerGeo;
    centerGeo.latDeg = airport_.latDeg;
    centerGeo.lonDeg = airport_.lonDeg;
    centerGeo.altMslM = elevM;
    surface.anchor = markings.anchor = geodeticToEcef(centerGeo);

    std::vector<float> sv, mv;
    std::vector<unsigned short> si, mi;
    auto addVertex = [&](std::vector<float>& v, const Mesh& mesh, const Ecef& p) {
        v.push_back(static_cast<float>(p.x - mesh.anchor.x));
        v.push_back(static_cast<float>(p.y - mesh.anchor.y));
        v.push_back(static_cast<float>(p.z - mesh.anchor.z));
    };
    // A quad covering [a0,a1] along and [l0,l1] lateral, split into segments so it follows the
    // curvature of the earth (a long flat quad would sink below the curved ground).
    auto addStrip = [&](std::vector<float>& v, std::vector<unsigned short>& ix, const Mesh& mesh,
                        double a0, double a1, double l0, double l1, double lift) {
        const int segments = std::max(1, static_cast<int>(std::ceil((a1 - a0) / 250.0)));
        const unsigned short base = static_cast<unsigned short>(v.size() / 3);
        for (int k = 0; k <= segments; ++k) {
            const double a = a0 + (a1 - a0) * k / segments;
            addVertex(v, mesh, pointEcef(a, l0, lift));
            addVertex(v, mesh, pointEcef(a, l1, lift));
        }
        for (int k = 0; k < segments; ++k) {
            const unsigned short i0 = static_cast<unsigned short>(base + 2 * k);
            ix.insert(ix.end(),
                      {i0, static_cast<unsigned short>(i0 + 1), static_cast<unsigned short>(i0 + 2),
                       static_cast<unsigned short>(i0 + 2), static_cast<unsigned short>(i0 + 1),
                       static_cast<unsigned short>(i0 + 3)});
        }
    };

    addStrip(sv, si, surface, 0, lengthM, -halfWidthM, halfWidthM, 0.15);

    const double lift = 0.30;
    // Centerline dashes: 30 m long every 50 m, 0.9 m wide.
    for (double a = 300.0; a + 30.0 < lengthM - 300.0; a += 50.0)
        addStrip(mv, mi, markings, a, a + 30.0, -0.45, 0.45, lift);
    // Edge lines.
    addStrip(mv, mi, markings, 0, lengthM, -halfWidthM + 0.6, -halfWidthM + 1.1, lift);
    addStrip(mv, mi, markings, 0, lengthM, halfWidthM - 1.1, halfWidthM - 0.6, lift);
    // Threshold "piano keys" and a bar at each end.
    for (int end = 0; end < 2; ++end) {
        for (int k = 0; k < 4; ++k) {
            const double l = 3.0 + k * 3.6;
            for (int side = -1; side <= 1; side += 2) {
                const double lo = side * l, hi = side * (l + 1.8);
                if (end == 0)
                    addStrip(mv, mi, markings, 6.0, 36.0, std::min(lo, hi), std::max(lo, hi), lift);
                else
                    addStrip(mv, mi, markings, lengthM - 36.0, lengthM - 6.0, std::min(lo, hi),
                             std::max(lo, hi), lift);
            }
        }
    }
    if (sv.size() / 3 > 65535 || mv.size() / 3 > 65535) {
        LOG_WARN("runway %s mesh exceeds 16-bit indices; not drawn", rwy.ident.c_str());
        return;
    }
    uploadMesh(surface, sv, si, {});
    uploadMesh(markings, mv, mi, {});
    runwaySurfaces_.push_back(surface);
    runwayMarkings_.push_back(markings);
}

void DebugRenderer::drawMesh(const Camera& camera, const Mesh& mesh, bool lines,
                             const float rgba[4]) {
    const Mat4 modelView = camera.modelView(mesh.anchor);  // RTC: float(anchor - camera) inside
    const Mat4 mvp = camera.projection() * modelView;
    gl_.UniformMatrix4fv(uMvp_, 1, 0, mvp.m);
    gl_.UniformMatrix4fv(uMv_, 1, 0, modelView.m);
    gl_.Uniform4f(uColor_, rgba[0] * shade_[0], rgba[1] * shade_[1], rgba[2] * shade_[2], rgba[3]);
    gl_.BindBuffer(GL_ARRAY_BUFFER, mesh.vbo);
    gl_.EnableVertexAttribArray(0);
    gl_.VertexAttribPointer(0, 3, GL_FLOAT, 0, 3 * sizeof(float), nullptr);
    if (lines) {
        gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.lineIbo);
        gl_.DrawElements(GL_LINES, mesh.lineIndexCount, GL_UNSIGNED_SHORT, nullptr);
    } else {
        gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, mesh.ibo);
        gl_.DrawElements(GL_TRIANGLES, mesh.indexCount, GL_UNSIGNED_SHORT, nullptr);
    }
}

void DebugRenderer::render(const Camera& camera, int viewportW, int viewportH) {
    gl_.Viewport(0, 0, viewportW, viewportH);
    // W4: below-horizon color is the shared haze color; the sky pass covers the rest.
    const float* clear = atmosphere_ ? atmosphere_->fog().colorRgb : kSkyColor;
    gl_.ClearColor(clear[0], clear[1], clear[2], 1.0f);
    gl_.Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    if (atmosphere_ && sky_) sky_->draw(camera, *atmosphere_);  // W4
    if (!haveWorld_ || !program_.valid()) return;

    // Keep the ground window centered on the camera, snapped to whole cells.
    const GeoPos& eye = camera.eyeGeo();
    const double mPerDegLon = kMPerDegLat * std::cos(degToRad(airport_.latDeg));
    const int cellN =
        static_cast<int>(std::floor((eye.latDeg - airport_.latDeg) * kMPerDegLat / kCellM));
    const int cellE =
        static_cast<int>(std::floor((eye.lonDeg - airport_.lonDeg) * mPerDegLon / kCellM));
    if (cellN != groundCellN_ || cellE != groundCellE_) rebuildGround(cellN, cellE);

    // W5: darken the placeholder ground with the sun (same terms terrain shading will use) so
    // dusk and night screenshots read correctly; full brightness by day.
    if (atmosphere_) {
        const world::LightParams& light = atmosphere_->light();
        const float sunUp = static_cast<float>(std::max(0.0, std::sin(atmosphere_->sun().elevationRad)));
        for (int i = 0; i < 3; ++i)
            shade_[i] = std::min(1.0f, light.ambientRgb[i] + light.diffuseRgb[i] * sunUp);
    }
    gl_.Enable(GL_DEPTH_TEST);
    gl_.DepthFunc(GL_LEQUAL);
    gl_.Disable(GL_CULL_FACE);
    program_.use();
    // W4: fog comes from the shared atmosphere (falls back to the old constants without one).
    world::FogParams fallbackFog;
    std::copy(kFogColor, kFogColor + 3, fallbackFog.colorRgb);
    fallbackFog.densityPerM = static_cast<float>(kFogDensity);
    fogUniforms_.set(gl_, atmosphere_ ? atmosphere_->fog() : fallbackFog);

    // Fill pushed back so the grid lines and the runway (pulled forward) win depth ties.
    gl_.Enable(GL_POLYGON_OFFSET_FILL);
    gl_.PolygonOffset(1.0f, 1.0f);
    drawMesh(camera, ground_, false, kGroundColor);
    gl_.PolygonOffset(-1.0f, -1.0f);
    for (const Mesh& m : runwaySurfaces_) drawMesh(camera, m, false, kRunwayColor);
    gl_.PolygonOffset(-2.0f, -2.0f);
    for (const Mesh& m : runwayMarkings_) drawMesh(camera, m, false, kMarkingColor);
    gl_.Disable(GL_POLYGON_OFFSET_FILL);
    drawMesh(camera, ground_, true, kGridColor);
    gl_.DisableVertexAttribArray(0);
}

}  // namespace fsim::gfx
