#pragma once
// V1: external chase view. The camera trails the aircraft (behind and above, level horizon,
// orbit with the look angles) and the aircraft is drawn from a model built in code
// (ExternalViewModel.h): one draw call, depth-tested against the terrain and runways, lit by
// the atmosphere's sun. D5 (first person + HUD) still holds for the cockpit view.

#include <vector>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/TerrainQuery.h"
#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "world/Atmosphere.h"
#include "world/ExternalViewModel.h"

namespace fsim::world {

class ExternalView {
public:
    ExternalView(const gfx::GlApi& gl, gfx::GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~ExternalView();
    ExternalView(const ExternalView&) = delete;
    ExternalView& operator=(const ExternalView&) = delete;

    bool init();
    // Model kind, span and gear type for this aircraft; call for every new flight.
    void setAircraft(const AircraftProfile& profile);
    // The next placeCamera() snaps behind the aircraft instead of easing.
    void reset() { followInit_ = false; }

    // Moves `camera` (already posed by setPoseFromAircraft, which sets the far plane) to the
    // chase position. The orbit angles come from the look layer (yaw + = right, pitch + = up).
    // The eye stays at least 2 m above the terrain.
    void placeCamera(gfx::Camera& camera, const AircraftState& s, float orbitYawRad,
                     float orbitPitchRad, const TerrainQuery* terrain, double dtS);
    // Draws the aircraft (rebuilds the mesh only when flaps or gear moved).
    void render(const gfx::Camera& camera, const AircraftState& s, const LightParams& light);

    int lastDrawCalls() const { return lastDrawCalls_; }
    const ExternalModelInfo& modelInfo() const { return info_; }
    ExternalModelKind kind() const { return params_.kind; }

private:
    void rebuildIfNeeded(const AircraftState& s);

    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    gfx::ShaderProgram program_;
    gfx::GLint uMvp_ = -1, uSun_ = -1, uAmbient_ = -1, uDiffuse_ = -1;
    gfx::GLuint vbo_ = 0;
    bool ready_ = false;

    ExternalModelParams params_;
    bool fixedGear_ = true;
    bool dirty_ = true;
    int builtFlapStep_ = -1, builtGearStep_ = -1;
    float builtGearHeightM_ = 0;
    std::vector<float> verts_;
    ExternalModelInfo info_;
    int vertexCount_ = 0;
    int lastDrawCalls_ = 0;

    float followHdgDeg_ = 0;
    bool followInit_ = false;
    float groundCgHeightM_ = 0;  // reference-point height on the wheels: the gear reaches it
    bool learnGearHeight_ = true;
};

}  // namespace fsim::world
