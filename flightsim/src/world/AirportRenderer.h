#pragma once
// W5: draws the airports around the camera (ARCHITECTURE section 6, pass order: after the ground,
// before the HUD): runway surfaces, markings (polygon offset layers), apron and a few buildings,
// then edge/threshold/end lights, approach lights and the PAPI as additive point sprites.
//
// Everything is RTC (ARCHITECTURE section 4): each mesh keeps a double ECEF anchor and float
// offsets; the per-draw translation comes from Camera::modelView(anchor). Geometry is built once
// per airport (AirportGeometry) and cached while the airport stays in the active set. When the
// set changes, the renderer re-registers the runway flatten areas with the TerrainQuery (it
// clears ALL flatten areas first, because the interface can only clear everything).
//
// Depth layers (the terrain must draw with polygon offset >= 0): pavement and apron
// glPolygonOffset(-1,-1), markings (-2,-2), buildings none. Lights are depth-tested with a small
// clip-space bias and never write depth.

#include <memory>
#include <vector>

#include "core/NavDatabase.h"
#include "core/TerrainQuery.h"
#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "world/AirportGeometry.h"
#include "world/Atmosphere.h"

namespace fsim::world {

class AirportRenderer {
public:
    // Airports whose reference point is within this distance of the eye are loaded.
    static constexpr double kActiveRadiusM = 60000.0;
    static constexpr int kMaxActiveAirports = 6;
    // Draw ranges (eye to runway centre minus half its length).
    static constexpr double kSurfaceRangeM = 40000.0;
    static constexpr double kLightRangeM = 30000.0;
    // The active set is re-selected when the eye moved this far since the last selection.
    static constexpr double kRefreshDistanceM = 2000.0;

    struct Stats {
        int activeAirports = 0;
        int runwaysDrawn = 0;
        int lightsDrawn = 0;
        int refreshes = 0;
    };

    AirportRenderer(const gfx::GlApi& gl, gfx::GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~AirportRenderer();
    AirportRenderer(const AirportRenderer&) = delete;
    AirportRenderer& operator=(const AirportRenderer&) = delete;

    bool init();
    void setNavDatabase(const NavDatabase* nav);
    void setTerrain(TerrainQuery* terrain) { terrain_ = terrain; }
    void setAtmosphere(const Atmosphere* atmosphere) { atmosphere_ = atmosphere; }

    // Refreshes the active set when needed (no GL state changes besides buffer uploads).
    void update(const gfx::Camera& camera);
    // Draws into the bound framebuffer; call after the ground, before the HUD. timeS drives the
    // sequenced flashers.
    void render(const gfx::Camera& camera, int viewportWidthPx, int viewportHeightPx,
                double timeS);

    const Stats& stats() const { return stats_; }

private:
    struct GpuSurface {
        gfx::GLuint vbo = 0, ibo = 0;
        int indexCount = 0;
        Ecef anchor;
    };
    struct GpuLights {
        gfx::GLuint vbo = 0;
        int count = 0;
        Ecef anchor;
        float axisEcef[3] = {1, 0, 0};
        bool hasPapi = false;
    };
    struct RunwayGpu {
        RunwayInfo info;
        RunwayFrame frame;
        Ecef center;
        std::vector<GpuSurface> pavement, markings;
        GpuLights lights;
    };
    struct AirportGpu {
        const Airport* airport = nullptr;
        Ecef center;
        std::vector<RunwayGpu> runways;
        std::vector<GpuSurface> apronFlat, buildings;
    };

    void selectActive(const gfx::Camera& camera);
    void buildAirport(const Airport& airport);
    GpuSurface upload(const SurfaceMesh& mesh);
    void destroy(GpuSurface& s);
    void destroy(AirportGpu& a);
    void drawSurface(const gfx::Camera& camera, const GpuSurface& s);
    void drawSurfaces(const gfx::Camera& camera, const std::vector<GpuSurface>& list);

    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    gfx::ShaderProgram surfaceProgram_, lightProgram_;
    gfx::GLint uSMvp_ = -1, uSMv_ = -1, uSSun_ = -1, uSAmbient_ = -1, uSDiffuse_ = -1;
    gfx::GLint uLMvp_ = -1, uLMv_ = -1, uLAxis_ = -1, uLScale_ = -1, uLMaxPoint_ = -1,
               uLFlash_ = -1, uLPapi_ = -1, uLIntensity_ = -1, uLBias_ = -1;
    FogUniforms surfaceFog_, lightFog_;

    const NavDatabase* nav_ = nullptr;
    TerrainQuery* terrain_ = nullptr;
    const Atmosphere* atmosphere_ = nullptr;

    std::vector<std::unique_ptr<AirportGpu>> active_;
    bool haveSelection_ = false;
    double selLatDeg_ = 0, selLonDeg_ = 0;
    Stats stats_;
};

}  // namespace fsim::world
