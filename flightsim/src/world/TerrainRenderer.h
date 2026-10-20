#pragma once
// W2: terrain renderer. Draws the tile pyramid from a TerrainService as a CDLOD quadtree of
// 16x16-cell patches (scheme and rationale in TerrainRendererMesh.h / TerrainRendererLod.h), in
// camera-relative ECEF (RTC, ARCHITECTURE section 4), with land-class colours, procedural detail,
// sun lighting and the shared W4 fog. Ocean (no tile) is a camera-centred sea disc at height 0.
//
// Call order per frame: terrainService.update(...), clear, sky, terrainRenderer.render(...), then
// airports, HUD. All GL calls are on the main thread. Mesh building happens on the main thread
// too, bounded by uploadsPerFrame (each node mesh is about 357 vertices and builds in well under
// a millisecond); the tile IO itself stays on W1's worker.

#include <cstdint>
#include <string>
#include <vector>

#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "world/Atmosphere.h"
#include "world/TerrainRendererLod.h"
#include "world/terrain/TerrainService.h"

namespace fsim::world {

struct TerrainRenderConfig {
    LodConfig lod;
    int uploadsPerFrame = 4;  // node meshes built and uploaded per frame (GPU upload budget)
    bool seaDisc = true;
};

struct TerrainRenderStats {
    int nodesDrawn = 0, drawCalls = 0;
    int triangles = 0;  // issued this frame, skirts included
    int meshesBuilt = 0;                // this frame
    int buildQueue = 0;                 // nodes still waiting for a mesh (after this frame's budget)
    int cachedMeshes = 0;
    size_t gpuBytes = 0;
    int visited = 0, culledFrustum = 0, culledHorizon = 0, seaSkipped = 0;
    int minLevel = 0, maxLevel = 0;  // node levels drawn
    double selectMs = 0, buildMs = 0, drawMs = 0;  // CPU time on this frame
    uint64_t totalMeshBuilds = 0;
    std::string summary() const;  // one log line
};

class TerrainRenderer {
public:
    TerrainRenderer(const gfx::GlApi& gl, gfx::GlFlavor flavor, terrain::TerrainService& terrain);
    ~TerrainRenderer();
    TerrainRenderer(const TerrainRenderer&) = delete;
    TerrainRenderer& operator=(const TerrainRenderer&) = delete;

    bool init();
    void setConfig(const TerrainRenderConfig& cfg);
    const TerrainRenderConfig& config() const { return cfg_; }

    // Draws sea and terrain into the bound framebuffer (depth test and writes on). The atmosphere
    // supplies fog and light; null uses defaults.
    void render(const gfx::Camera& camera, const Atmosphere* atmosphere);
    // Builds meshes with no per-frame budget until the selection is complete or maxMeshes were
    // built: for loading screens and screenshots, so the first frame is not full of holes.
    int prewarm(const gfx::Camera& camera, int maxMeshes);

    const TerrainRenderStats& stats() const { return stats_; }
    const TerrainSelector& selector() const { return selector_; }

private:
    struct Locations {
        gfx::GLint mvp = -1, eyeOffset = -1, morph = -1, noiseA = -1, noiseB = -1, sun = -1,
                   ambient = -1, diffuse = -1, water = -1, detail = -1;
    };

    ViewParams makeView(const gfx::Camera& camera) const;
    int buildQueued(int maxBuilds);
    bool buildOne(const BuildRequest& req);
    void collect();
    void drawNode(const gfx::Camera& camera, const gfx::Mat4& proj, const NodeRecord& rec,
                  const DrawNode& dn);
    void setNoiseUniforms(const Ecef& anchor);
    void rebuildSea(const GeoPos& center);
    void drawSea(const gfx::Camera& camera, const gfx::Mat4& proj);
    void deleteBuffers(const std::vector<uint32_t>& vbos);

    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    terrain::TerrainService& svc_;
    TerrainRenderConfig cfg_;
    TerrainSelector selector_;
    gfx::ShaderProgram program_;
    Locations loc_;
    FogUniforms fogUniforms_;
    gfx::GLuint ibo_ = 0;
    int vboCount_ = 0;

    gfx::GLuint seaVbo_ = 0, seaIbo_ = 0;
    int seaIndexCount_ = 0;
    Ecef seaAnchor_;
    GeoPos seaCenter_;
    bool haveSea_ = false;

    std::vector<DrawNode> draw_;
    std::vector<BuildRequest> builds_;
    std::vector<uint32_t> garbage_;
    TerrainRenderStats stats_;
    bool ready_ = false;
};

// Developer hooks for screenshots (no CLI flags exist yet): read from the environment.
//   FSIM_W2_CAMERA = "lat,lon,altMslFt,headingTrueDeg[,pitchDeg]"  overrides the view pose
//   FSIM_W2_VISIBILITY_KM = "<km>"                                 overrides fog visibility
struct TerrainDebugOverrides {
    bool hasCamera = false;
    double latDeg = 0, lonDeg = 0, altMslFt = 0, headingTrueDeg = 0, pitchDeg = 0;
    float visibilityKm = 0;
};
TerrainDebugOverrides readTerrainDebugEnv();

}  // namespace fsim::world
