#pragma once
// W4: full-screen sky pass. One draw call (a single oversized triangle). X3c: drawn right after
// the terrain, depth-tested at the far plane, so the GPU skips it wherever terrain already is
// (it used to shade every pixel first and then lose half of them to the terrain).
// The per-pixel view ray is built in the vertex shader from the clip-space corner and the lens
// (tan of the half FOV), so the fragment shader gets an interpolated eye-space ray and needs no
// matrix inverse (GLES2-safe). Local "up" and the sun direction arrive as eye-space uniforms.
//
// Colors come from Atmosphere. At the horizon (and everywhere below it) the sky is exactly the
// fog color, so terrain fading into haze meets the sky without a seam.

#include "gfx/Camera.h"
#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "world/Atmosphere.h"

namespace fsim::world {

class Sky {
public:
    Sky(const gfx::GlApi& gl, gfx::GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~Sky();
    Sky(const Sky&) = delete;
    Sky& operator=(const Sky&) = delete;

    bool init();
    // Draws into the bound framebuffer where the depth buffer still holds the cleared 1.0 (depth
    // test on, GL_LEQUAL, no depth writes; the triangle sits at z = w). Call after the opaque
    // terrain and before anything blended. Leaves the depth test on, GL_LESS, depth writes on.
    void draw(const gfx::Camera& camera, const Atmosphere& atmosphere);

private:
    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    gfx::ShaderProgram program_;
    gfx::GLuint vbo_ = 0;
    gfx::GLint uTanHalf_ = -1, uUp_ = -1, uSun_ = -1, uHaze_ = -1, uZenith_ = -1, uDisc_ = -1,
               uGlow_ = -1, uParams_ = -1, uDiscCos_ = -1;
};

}  // namespace fsim::world
