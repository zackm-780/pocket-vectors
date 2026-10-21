#pragma once
// W6: unlit world overlays. At most four draw calls (one per layer that has triangles). Labels
// are not drawn here; drawLabels() goes in the HUD pass.

#include <vector>

#include "core/NavDatabase.h"
#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "hud/Renderer2D.h"
#include "world/WorldOverlayLogic.h"

namespace fsim::world {

class WorldOverlay {
public:
    WorldOverlay(const gfx::GlApi& gl, gfx::GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~WorldOverlay();
    WorldOverlay(const WorldOverlay&) = delete;
    WorldOverlay& operator=(const WorldOverlay&) = delete;

    bool init();

    // Rebuilds batches from the plan and the nearest airports, then draws. No-ops when the shader
    // failed to build. `radiusNm` is the airport / runway search radius.
    void render(const gfx::Camera& camera, const FlightPlan* plan, const NavDatabase* nav,
                const TerrainQuery* terrain, const OverlayToggles& toggles, float radiusNm);
    // Drops labels and the draw-call count when the overlay is not drawn this frame.
    void clearFrame();

    void drawLabels(Renderer2D& r) const;
    int lastDrawCalls() const { return lastDrawCalls_; }
    int labelCount() const { return labelCount_; }

private:
    void drawBatch(const std::vector<float>& verts, gfx::GLuint vbo, const gfx::Mat4& mvp);

    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    gfx::ShaderProgram program_;
    gfx::GLint uMvp_ = -1;
    gfx::GLuint vbo_[4] = {};
    bool ready_ = false;

    std::vector<float> route_, vpath_, airports_, runways_;
    OverlayLabel labels_[48];
    int labelCount_ = 0;
    int lastDrawCalls_ = 0;
};

}  // namespace fsim::world
