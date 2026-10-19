#pragma once
// The one OpenGL function loader (ARCHITECTURE section 6). Every GL function we use is fetched
// through SDL_GL_GetProcAddress into the GlApi struct, so the same code runs on a GLES2 context
// (device, Mesa) and on a desktop GL 2.1 context. We deliberately do not include any system GL
// header: types and constants are declared here (values are identical in ES2 and GL 2.1).
//
// Add a function by adding one line to FSIM_GL_FUNCTIONS. GL calls are main-thread only.

#include <cstddef>
#include <cstdint>

#if defined(_WIN32)
#define FSIM_GLCALL __stdcall
#else
#define FSIM_GLCALL
#endif

namespace fsim::gfx {

using GLenum = unsigned int;
using GLbitfield = unsigned int;
using GLuint = unsigned int;
using GLint = int;
using GLsizei = int;
using GLboolean = unsigned char;
using GLfloat = float;
using GLchar = char;
using GLubyte = unsigned char;
using GLsizeiptr = std::ptrdiff_t;

// clang-format off
constexpr GLenum GL_DEPTH_BUFFER_BIT = 0x00000100, GL_COLOR_BUFFER_BIT = 0x00004000;
constexpr GLenum GL_POINTS = 0x0000, GL_LINES = 0x0001, GL_LINE_STRIP = 0x0003,
                 GL_TRIANGLES = 0x0004, GL_TRIANGLE_STRIP = 0x0005, GL_TRIANGLE_FAN = 0x0006;
constexpr GLenum GL_DEPTH_TEST = 0x0B71, GL_CULL_FACE = 0x0B44, GL_BLEND = 0x0BE2,
                 GL_POLYGON_OFFSET_FILL = 0x8037, GL_LEQUAL = 0x0203, GL_LESS = 0x0201;
constexpr GLenum GL_SRC_ALPHA = 0x0302, GL_ONE_MINUS_SRC_ALPHA = 0x0303;
constexpr GLenum GL_BYTE = 0x1400, GL_UNSIGNED_BYTE = 0x1401, GL_UNSIGNED_SHORT = 0x1403,
                 GL_UNSIGNED_INT = 0x1405, GL_FLOAT = 0x1406;
constexpr GLenum GL_RGBA = 0x1908, GL_RGB = 0x1907;
constexpr GLenum GL_ARRAY_BUFFER = 0x8892, GL_ELEMENT_ARRAY_BUFFER = 0x8893,
                 GL_STATIC_DRAW = 0x88E4, GL_DYNAMIC_DRAW = 0x88E8;
constexpr GLenum GL_FRAGMENT_SHADER = 0x8B30, GL_VERTEX_SHADER = 0x8B31,
                 GL_COMPILE_STATUS = 0x8B81, GL_LINK_STATUS = 0x8B82, GL_INFO_LOG_LENGTH = 0x8B84;
constexpr GLenum GL_VENDOR = 0x1F00, GL_RENDERER = 0x1F01, GL_VERSION = 0x1F02,
                 GL_EXTENSIONS = 0x1F03, GL_SHADING_LANGUAGE_VERSION = 0x8B8C;
constexpr GLenum GL_PACK_ALIGNMENT = 0x0D05, GL_NO_ERROR = 0;
constexpr GLenum GL_MAX_TEXTURE_SIZE = 0x0D33;
// clang-format on

// X(returnType, Name, (params)); the GL entry point is "gl" + Name.
// clang-format off
#define FSIM_GL_FUNCTIONS(X)                                                              \
    X(void, Clear, (GLbitfield mask))                                                     \
    X(void, ClearColor, (GLfloat r, GLfloat g, GLfloat b, GLfloat a))                     \
    X(void, Viewport, (GLint x, GLint y, GLsizei w, GLsizei h))                           \
    X(void, Enable, (GLenum cap))                                                         \
    X(void, Disable, (GLenum cap))                                                        \
    X(void, DepthFunc, (GLenum func))                                                     \
    X(void, DepthMask, (GLboolean flag))                                                  \
    X(void, BlendFunc, (GLenum sfactor, GLenum dfactor))                                  \
    X(void, PolygonOffset, (GLfloat factor, GLfloat units))                               \
    X(void, LineWidth, (GLfloat width))                                                   \
    X(void, PixelStorei, (GLenum pname, GLint param))                                     \
    X(void, ReadPixels, (GLint x, GLint y, GLsizei w, GLsizei h, GLenum format, GLenum type, void* data)) \
    X(void, Finish, ())                                                                   \
    X(GLenum, GetError, ())                                                               \
    X(const GLubyte*, GetString, (GLenum name))                                           \
    X(void, GetIntegerv, (GLenum pname, GLint* data))                                     \
    X(void, GenBuffers, (GLsizei n, GLuint* buffers))                                     \
    X(void, DeleteBuffers, (GLsizei n, const GLuint* buffers))                            \
    X(void, BindBuffer, (GLenum target, GLuint buffer))                                   \
    X(void, BufferData, (GLenum target, GLsizeiptr size, const void* data, GLenum usage)) \
    X(GLuint, CreateShader, (GLenum type))                                                \
    X(void, ShaderSource, (GLuint shader, GLsizei count, const GLchar* const* string, const GLint* length)) \
    X(void, CompileShader, (GLuint shader))                                               \
    X(void, GetShaderiv, (GLuint shader, GLenum pname, GLint* params))                    \
    X(void, GetShaderInfoLog, (GLuint shader, GLsizei bufSize, GLsizei* length, GLchar* infoLog)) \
    X(void, DeleteShader, (GLuint shader))                                                \
    X(GLuint, CreateProgram, ())                                                          \
    X(void, AttachShader, (GLuint program, GLuint shader))                                \
    X(void, BindAttribLocation, (GLuint program, GLuint index, const GLchar* name))       \
    X(void, LinkProgram, (GLuint program))                                                \
    X(void, GetProgramiv, (GLuint program, GLenum pname, GLint* params))                  \
    X(void, GetProgramInfoLog, (GLuint program, GLsizei bufSize, GLsizei* length, GLchar* infoLog)) \
    X(void, UseProgram, (GLuint program))                                                 \
    X(void, DeleteProgram, (GLuint program))                                              \
    X(GLint, GetUniformLocation, (GLuint program, const GLchar* name))                    \
    X(void, Uniform1f, (GLint location, GLfloat v0))                                      \
    X(void, Uniform3f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2))              \
    X(void, Uniform4f, (GLint location, GLfloat v0, GLfloat v1, GLfloat v2, GLfloat v3))  \
    X(void, UniformMatrix4fv, (GLint location, GLsizei count, GLboolean transpose, const GLfloat* value)) \
    X(void, EnableVertexAttribArray, (GLuint index))                                      \
    X(void, DisableVertexAttribArray, (GLuint index))                                     \
    X(void, VertexAttribPointer, (GLuint index, GLint size, GLenum type, GLboolean normalized, GLsizei stride, const void* pointer)) \
    X(void, DrawArrays, (GLenum mode, GLint first, GLsizei count))                        \
    X(void, DrawElements, (GLenum mode, GLsizei count, GLenum type, const void* indices))
// clang-format on

struct GlApi {
#define FSIM_GL_DECLARE(ret, name, params) ret(FSIM_GLCALL* name) params = nullptr;
    FSIM_GL_FUNCTIONS(FSIM_GL_DECLARE)
#undef FSIM_GL_DECLARE

    // Loads every function through SDL_GL_GetProcAddress; a GL context must be current.
    // Returns false (and logs the name) if any required function is missing.
    bool load();
};

}  // namespace fsim::gfx
