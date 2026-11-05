#pragma once
// W7: cloud layers. Each layer the eye is not inside is one blended disk (radius 50 km, centered
// under the eye, following the Earth's curvature) at its base (seen from below) or top (seen
// from above), shaped by a tiling noise texture and fogged like the terrain. Inside a layer the
// view fades to the cloud colour (one full-screen quad). At most three draw calls in all.

#include <cstdint>
#include <vector>

#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "world/Atmosphere.h"
#include "world/CloudsLogic.h"

namespace fsim::world {

class Clouds {
public:
    Clouds(const gfx::GlApi& gl, gfx::GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~Clouds();
    Clouds(const Clouds&) = delete;
    Clouds& operator=(const Clouds&) = delete;

    bool init();
    // The layers to draw and the point the coverage pattern is anchored to (the start airport).
    void setDeck(const CloudDeck& deck, double originLatDeg, double originLonDeg);
    const CloudDeck& deck() const { return deck_; }

    // Draws after the opaque world (terrain, airports, the external model): depth test on, depth
    // writes off, alpha blended.
    void render(const gfx::Camera& camera, const Atmosphere& atmosphere);
    int lastDrawCalls() const { return lastDrawCalls_; }
    // The view fade inside a layer this frame (0 = clear), for tests and traces.
    float lastInsideAlpha() const { return lastInsideAlpha_; }

private:
    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    gfx::ShaderProgram layerProgram_, fadeProgram_;
    gfx::GLint uMvp_ = -1, uHeight_ = -1, uOff1_ = -1, uOff2_ = -1, uInvTile1_ = -1,
               uInvTile2_ = -1, uColor_ = -1, uThreshold_ = -1, uSoft_ = -1, uNoise_ = -1,
               uShade_ = -1;
    FogUniforms fog_;
    gfx::GLint uFadeColor_ = -1;
    gfx::GLuint vbo_ = 0, ibo_ = 0, fadeVbo_ = 0, texture_ = 0;
    int indexCount_ = 0;
    bool ready_ = false;

    std::vector<std::uint8_t> noise_;
    CloudDeck deck_;
    double originLatDeg_ = 0, originLonDeg_ = 0;
    int lastDrawCalls_ = 0;
    float lastInsideAlpha_ = 0;
};

}  // namespace fsim::world
