#include "gfx/Shader.h"

#include <vector>

#include "core/Log.h"

namespace fsim::gfx {
namespace {

std::string preamble(GlFlavor flavor, bool fragment) {
    if (flavor == GlFlavor::Es2)
        return fragment ? "#version 100\nprecision mediump float;\n" : "#version 100\n";
    return "#version 120\n#define lowp\n#define mediump\n#define highp\n";
}

}  // namespace

GLuint ShaderProgram::compile(GLenum type, const std::string& body) {
    const bool fragment = type == GL_FRAGMENT_SHADER;
    const std::string source = preamble(flavor_, fragment) + body;
    const GLuint shader = gl_->CreateShader(type);
    const GLchar* text = source.c_str();
    gl_->ShaderSource(shader, 1, &text, nullptr);
    gl_->CompileShader(shader);
    GLint ok = 0;
    gl_->GetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        gl_->GetShaderiv(shader, GL_INFO_LOG_LENGTH, &len);
        std::vector<GLchar> log(static_cast<size_t>(len > 1 ? len : 1));
        gl_->GetShaderInfoLog(shader, static_cast<GLsizei>(log.size()), nullptr, log.data());
        LOG_ERROR("%s shader compile failed: %s", fragment ? "fragment" : "vertex", log.data());
        gl_->DeleteShader(shader);
        return 0;
    }
    return shader;
}

bool ShaderProgram::build(const GlApi& gl, GlFlavor flavor, const char* vertexBody,
                          const char* fragmentBody,
                          std::initializer_list<const char*> attribNames) {
    destroy();
    gl_ = &gl;
    flavor_ = flavor;
    const GLuint vs = compile(GL_VERTEX_SHADER, vertexBody);
    const GLuint fs = compile(GL_FRAGMENT_SHADER, fragmentBody);
    if (!vs || !fs) {
        if (vs) gl.DeleteShader(vs);
        if (fs) gl.DeleteShader(fs);
        return false;
    }
    program_ = gl.CreateProgram();
    gl.AttachShader(program_, vs);
    gl.AttachShader(program_, fs);
    GLuint index = 0;
    for (const char* name : attribNames) gl.BindAttribLocation(program_, index++, name);
    gl.LinkProgram(program_);
    gl.DeleteShader(vs);
    gl.DeleteShader(fs);

    GLint ok = 0;
    gl.GetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        GLint len = 0;
        gl.GetProgramiv(program_, GL_INFO_LOG_LENGTH, &len);
        std::vector<GLchar> log(static_cast<size_t>(len > 1 ? len : 1));
        gl.GetProgramInfoLog(program_, static_cast<GLsizei>(log.size()), nullptr, log.data());
        LOG_ERROR("shader link failed: %s", log.data());
        destroy();
        return false;
    }
    return true;
}

void ShaderProgram::destroy() {
    if (gl_ && program_) gl_->DeleteProgram(program_);
    program_ = 0;
}

}  // namespace fsim::gfx
