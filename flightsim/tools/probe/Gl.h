// Minimal GLES2 access for the probe.
//
// All GL entry points are loaded at run time with SDL_GL_GetProcAddress, so the binary does not link
// libGLESv2/libEGL (the device's dynamic dependencies stay limited to libc and libSDL2). The same code
// runs on an OpenGL ES 2.0 context (device, Mesa) and on a desktop GL 2.1 context (fallback).
//
// Function prototypes and constants come from SDL's own bundled GLES2 header, so no system GLES
// headers are needed at build time either.
#pragma once

#define SDL_USE_BUILTIN_OPENGL_DEFINITIONS 1
#include <SDL.h>
#include <SDL_opengles2.h>

#include <string>
#include <vector>

namespace probe {

// X-macro list of every GL function the probe calls.
#define PROBE_GL_FUNCS(X) \
    X(ActiveTexture) X(AttachShader) X(BindAttribLocation) X(BindBuffer) X(BindTexture) \
    X(BlendFunc) X(BufferData) X(BufferSubData) X(Clear) X(ClearColor) X(CompileShader) \
    X(CreateProgram) X(CreateShader) X(CullFace) X(DeleteBuffers) X(DeleteProgram) X(DeleteShader) \
    X(DeleteTextures) X(DepthFunc) X(DepthMask) X(Disable) X(DisableVertexAttribArray) X(DrawArrays) \
    X(DrawElements) X(Enable) X(EnableVertexAttribArray) X(Finish) X(Flush) X(GenBuffers) \
    X(GenFramebuffers) X(BindFramebuffer) X(FramebufferTexture2D) X(FramebufferRenderbuffer) \
    X(CheckFramebufferStatus) X(GenRenderbuffers) X(BindRenderbuffer) X(RenderbufferStorage) \
    X(DeleteFramebuffers) X(DeleteRenderbuffers) \
    X(GenTextures) X(GetAttribLocation) X(GetError) X(GetIntegerv) X(GetProgramInfoLog) \
    X(GetProgramiv) X(GetShaderInfoLog) X(GetShaderiv) X(GetString) X(GetUniformLocation) \
    X(LinkProgram) X(PixelStorei) X(ReadPixels) X(ShaderSource) X(TexImage2D) X(TexParameteri) \
    X(Uniform1f) X(Uniform1i) X(Uniform2f) X(Uniform3f) X(Uniform4f) X(UniformMatrix4fv) X(UseProgram) \
    X(VertexAttribPointer) X(Viewport)

struct GlFuncs {
#define PROBE_GL_DECLARE(name) decltype(&::gl##name) name = nullptr;
    PROBE_GL_FUNCS(PROBE_GL_DECLARE)
#undef PROBE_GL_DECLARE
    // Optional: absent on plain desktop GL 2.1 without ARB_ES2_compatibility.
    decltype(&::glGetShaderPrecisionFormat) GetShaderPrecisionFormat = nullptr;

    // Loads all functions for the current context. Returns false and lists the missing ones in
    // `missing` if any required function could not be resolved.
    bool load(std::string& missing);
};

// Column-major 4x4 matrix helpers (OpenGL convention).
struct Mat4 {
    float m[16];
};
Mat4 mat4Identity();
Mat4 mat4Multiply(const Mat4& a, const Mat4& b);  // a * b
Mat4 mat4Perspective(float fovYRad, float aspect, float nearM, float farM);
Mat4 mat4LookAt(const float eye[3], const float target[3], const float up[3]);

// Builds a program from GLSL 1.00 / 1.10 compatible sources. A precision preamble is prepended to the
// fragment shader (needed on ES, ignored on desktop). `attribs` are bound to locations 0..n-1.
// Returns 0 and fills `log` on failure.
GLuint buildProgram(GlFuncs& gl, const char* vertexSrc, const char* fragmentSrc,
                    const std::vector<const char*>& attribs, std::string& log);

// Tiny 8x8 bitmap-font text and rectangle renderer (one dynamic VBO, one draw call per flush).
struct Color {
    float r, g, b, a;
};

class TextRenderer {
public:
    bool init(GlFuncs* gl, std::string& log);
    void shutdown();
    // Starts a new batch for a framebuffer of the given size in pixels. (0,0) is the top-left corner.
    void begin(int widthPx, int heightPx);
    void text(float xPx, float yPx, float scale, const Color& color, const std::string& s);
    void rect(float xPx, float yPx, float wPx, float hPx, const Color& color);
    // Draws the batch. Leaves GL state in a defined state (blend on, depth off).
    void flush();
    static float glyphWidthPx(float scale) { return 8.0f * scale; }

private:
    struct Vertex {
        float x, y, u, v, r, g, b, a;
    };
    void quad(float x, float y, float w, float h, float u0, float v0, float u1, float v1, const Color& c);

    GlFuncs* gl_ = nullptr;
    GLuint program_ = 0;
    GLuint texture_ = 0;
    GLuint vbo_ = 0;
    GLint uScreen_ = -1;
    GLint uTex_ = -1;
    int widthPx_ = 640;
    int heightPx_ = 480;
    std::vector<Vertex> vertices_;
};

}  // namespace probe
