#pragma once
// Shader compilation with the version preamble from ARCHITECTURE section 6: shader bodies are
// written once in GLSL ES 1.00 style (no precision statement, no #version) and get
//   ES 2.0 : "#version 100" (+ "precision mediump float;" in fragment shaders)
//   GL 2.1 : "#version 120" plus empty lowp/mediump/highp macros.

#include <initializer_list>
#include <string>

#include "gfx/GlApi.h"

namespace fsim::gfx {

enum class GlFlavor { Es2, Desktop21 };

class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram() { destroy(); }
    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;

    // Compiles and links. Attribute i of `attribNames` is bound to location i.
    bool build(const GlApi& gl, GlFlavor flavor, const char* vertexBody, const char* fragmentBody,
               std::initializer_list<const char*> attribNames);
    void destroy();

    void use() const { gl_->UseProgram(program_); }
    GLint uniform(const char* name) const { return gl_->GetUniformLocation(program_, name); }
    bool valid() const { return program_ != 0; }

private:
    GLuint compile(GLenum type, const std::string& source);

    const GlApi* gl_ = nullptr;
    GlFlavor flavor_ = GlFlavor::Es2;
    GLuint program_ = 0;
};

}  // namespace fsim::gfx
