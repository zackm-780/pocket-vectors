#include "world/WorldOverlay.h"

#include <algorithm>
#include <cmath>

#include "core/Geo.h"
#include "core/Units.h"

namespace fsim::world {
namespace {

constexpr char kVertex[] = "attribute vec3 aPos;\n"
                           "attribute vec3 aColor;\n"
                           "uniform mat4 uMvp;\n"
                           "varying vec3 vColor;\n"
                           "void main() {\n"
                           "  vColor = aColor;\n"
                           "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
                           "}\n";

constexpr char kFragment[] = "varying vec3 vColor;\n"
                             "void main() {\n"
                             "  gl_FragColor = vec4(vColor, 1.0);\n"
                             "}\n";

OverlayEye eyeFromCamera(const gfx::Camera& camera) {
    OverlayEye eye;
    eye.latDeg = camera.eyeGeo().latDeg;
    eye.lonDeg = camera.eyeGeo().lonDeg;
    eye.altMslM = camera.eyeGeo().altMslM;
    eye.ecef = camera.eyeEcef();
    eye.ecefToEye = camera.ecefToEye();
    eye.vFovRad = camera.vFovRad();
    eye.aspect = camera.aspect();
    return eye;
}

}  // namespace

WorldOverlay::~WorldOverlay() {
    if (vbo_[0] != 0) gl_.DeleteBuffers(4, vbo_);
}

bool WorldOverlay::init() {
    if (!program_.build(gl_, flavor_, kVertex, kFragment, {"aPos", "aColor"})) return false;
    uMvp_ = program_.uniform("uMvp");
    gl_.GenBuffers(4, vbo_);
    ready_ = program_.valid() && vbo_[0] != 0;
    return ready_;
}

void WorldOverlay::clearFrame() {
    labelCount_ = 0;
    lastDrawCalls_ = 0;
}

void WorldOverlay::drawBatch(const std::vector<float>& verts, gfx::GLuint vbo,
                             const gfx::Mat4& mvp) {
    if (verts.size() < 18) return;
    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, vbo);
    gl_.BufferData(gfx::GL_ARRAY_BUFFER, static_cast<gfx::GLsizeiptr>(verts.size() * sizeof(float)),
                   verts.data(), gfx::GL_DYNAMIC_DRAW);
    gl_.EnableVertexAttribArray(0);
    gl_.EnableVertexAttribArray(1);
    gl_.VertexAttribPointer(0, 3, gfx::GL_FLOAT, 0, 6 * sizeof(float), nullptr);
    gl_.VertexAttribPointer(1, 3, gfx::GL_FLOAT, 0, 6 * sizeof(float),
                            reinterpret_cast<const void*>(3 * sizeof(float)));
    gl_.UniformMatrix4fv(uMvp_, 1, 0, mvp.m);
    gl_.DrawArrays(gfx::GL_TRIANGLES, 0, static_cast<gfx::GLsizei>(verts.size() / 6));
    ++lastDrawCalls_;
}

void WorldOverlay::render(const gfx::Camera& camera, const FlightPlan* plan, const NavDatabase* nav,
                          const TerrainQuery* terrain, const OverlayToggles& toggles,
                          float radiusNm) {
    clearFrame();
    route_.clear();
    vpath_.clear();
    airports_.clear();
    runways_.clear();
    if (!ready_) return;
    if (!toggles.route && !toggles.vpath && !toggles.airports && !toggles.runways) return;

    const OverlayEye eye = eyeFromCamera(camera);
    OverlayAirportRef markerRefs[12];
    OverlayAirportRef runwayRefs[12];
    int markerN = 0, runwayN = 0;
    if ((toggles.airports || toggles.runways) && nav && radiusNm > 0.0f) {
        const auto found = nav->nearestAirports(eye.latDeg, eye.lonDeg, 24, radiusNm);
        std::vector<double> dist;
        dist.reserve(found.size());
        for (const Airport* ap : found) {
            dist.push_back(
                ap ? mToNm(greatCircleDistM(eye.latDeg, eye.lonDeg, ap->latDeg, ap->lonDeg))
                   : 1.0e9);
        }
        if (toggles.airports) {
            std::vector<int> idx;
            selectOverlayAirports(dist, radiusNm, 1.0, 12, idx);
            for (int i : idx) {
                if (markerN >= 12 || !found[static_cast<size_t>(i)]) break;
                markerRefs[markerN++] =
                    OverlayAirportRef{found[static_cast<size_t>(i)], dist[static_cast<size_t>(i)]};
            }
        }
        if (toggles.runways) {
            std::vector<int> idx;
            selectOverlayAirports(dist, radiusNm, 0.0, 12, idx);
            for (int i : idx) {
                if (runwayN >= 12 || !found[static_cast<size_t>(i)]) break;
                runwayRefs[runwayN++] =
                    OverlayAirportRef{found[static_cast<size_t>(i)], dist[static_cast<size_t>(i)]};
            }
        }
    }

    buildWorldOverlay(toggles, plan, markerRefs, markerN, runwayRefs, runwayN, terrain, eye, route_,
                      vpath_, airports_, runways_, labels_, 48, labelCount_);

    const gfx::Mat4 mvp = camera.viewProjection(camera.eyeEcef());
    gl_.Enable(gfx::GL_DEPTH_TEST);
    gl_.Disable(gfx::GL_CULL_FACE);
    gl_.Disable(gfx::GL_BLEND);
    program_.use();
    // Runways sit on the pavement; draw them first. The path is last so the gates stay visible.
    gl_.Enable(gfx::GL_POLYGON_OFFSET_FILL);
    gl_.PolygonOffset(-3.0f, -3.0f);
    drawBatch(runways_, vbo_[3], mvp);
    gl_.Disable(gfx::GL_POLYGON_OFFSET_FILL);
    drawBatch(airports_, vbo_[2], mvp);
    drawBatch(route_, vbo_[0], mvp);
    drawBatch(vpath_, vbo_[1], mvp);
    gl_.DisableVertexAttribArray(0);
    gl_.DisableVertexAttribArray(1);
}

void WorldOverlay::drawLabels(Renderer2D& r) const {
    if (labelCount_ <= 0) return;
    r.setOutline(true);
    r.setTextStrokeWidth(0);
    for (int i = 0; i < labelCount_; ++i) {
        const OverlayLabel& lab = labels_[i];
        r.text(lab.x, lab.y, lab.text, TextSize::S, Color(lab.r, lab.g, lab.b), HAlign::Center,
               VAlign::Middle);
    }
}

}  // namespace fsim::world
