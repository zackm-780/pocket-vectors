#pragma once
// WP-00 placeholder renderer: sky clear color, a flat ground grid around the camera and the runways
// of one airport. It exists to prove the camera and the RTC/ECEF rendering rule (every mesh has a
// double ECEF anchor and float offsets; see gfx/Camera.h) so W2 (terrain), W4 (sky/fog) and W5
// (airports) inherit working camera code. They replace this class; nothing should depend on it.

#include <vector>

#include "core/Geo.h"
#include "core/NavTypes.h"
#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "world/Atmosphere.h"  // W4

namespace fsim::world {
class Sky;  // W4
}

namespace fsim::gfx {

class DebugRenderer {
public:
    DebugRenderer(const GlApi& gl, GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~DebugRenderer();
    DebugRenderer(const DebugRenderer&) = delete;
    DebugRenderer& operator=(const DebugRenderer&) = delete;

    bool init();
    // W4: shared fog/haze/sun parameters and the sky pass (both optional; owned by the caller).
    void setAtmosphere(const world::Atmosphere* atmosphere, world::Sky* sky) {
        atmosphere_ = atmosphere;
        sky_ = sky;
    }
    // W5: AirportRenderer draws the runways now; call with false before setWorld to keep only the
    // ground grid.
    void setDrawRunways(bool on) { drawRunways_ = on; }
    // The airport's runways are drawn on flat ground at groundElevM; the ground grid is anchored
    // to the airport reference point.
    void setWorld(const Airport& airport, float groundElevM);
    // Draws one frame into the currently bound framebuffer (does not swap).
    void render(const Camera& camera, int viewportWidthPx, int viewportHeightPx);

private:
    struct Mesh {
        GLuint vbo = 0;
        GLuint ibo = 0;      // triangles
        GLuint lineIbo = 0;  // optional grid lines
        int indexCount = 0;
        int lineIndexCount = 0;
        Ecef anchor;  // double-precision ECEF anchor; vertices are float offsets from it
    };

    void destroyMesh(Mesh& mesh);
    void uploadMesh(Mesh& mesh, const std::vector<float>& vertices,
                    const std::vector<unsigned short>& triIndices,
                    const std::vector<unsigned short>& lineIndices);
    void rebuildGround(int cellN, int cellE);
    void buildRunway(const Runway& runway);
    void drawMesh(const Camera& camera, const Mesh& mesh, bool lines, const float rgba[4]);

    const GlApi& gl_;
    GlFlavor flavor_;
    ShaderProgram program_;
    GLint uMvp_ = -1, uMv_ = -1, uColor_ = -1;
    world::FogUniforms fogUniforms_;                 // W4: shared fog uniforms
    const world::Atmosphere* atmosphere_ = nullptr;  // W4
    world::Sky* sky_ = nullptr;                      // W4

    Airport airport_;
    bool haveWorld_ = false;
    bool drawRunways_ = true;  // W5
    float shade_[3] = {1, 1, 1};  // W5: ground brightness from the sun (1 by day)
    float groundElevM_ = 0;
    Mesh ground_;
    int groundCellN_ = 0x7fffffff, groundCellE_ = 0x7fffffff;
    std::vector<Mesh> runwaySurfaces_, runwayMarkings_;
};

}  // namespace fsim::gfx
