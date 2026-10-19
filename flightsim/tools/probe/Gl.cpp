#include "Gl.h"

#include <cmath>
#include <cstring>

#include "Common.h"
#include "font8x8.h"

namespace probe {

bool GlFuncs::load(std::string& missing) {
    missing.clear();
#define PROBE_GL_LOAD(fname)                                                                  \
    fname = reinterpret_cast<decltype(fname)>(SDL_GL_GetProcAddress("gl" #fname));            \
    if (fname == nullptr) {                                                                   \
        missing += " gl" #fname;                                                              \
    }
    PROBE_GL_FUNCS(PROBE_GL_LOAD)
#undef PROBE_GL_LOAD
    GetShaderPrecisionFormat =
        reinterpret_cast<decltype(GetShaderPrecisionFormat)>(SDL_GL_GetProcAddress("glGetShaderPrecisionFormat"));
    return missing.empty();
}

Mat4 mat4Identity() {
    Mat4 r{};
    r.m[0] = r.m[5] = r.m[10] = r.m[15] = 1.0f;
    return r;
}

Mat4 mat4Multiply(const Mat4& a, const Mat4& b) {
    Mat4 r{};
    for (int col = 0; col < 4; ++col) {
        for (int row = 0; row < 4; ++row) {
            float sum = 0.0f;
            for (int k = 0; k < 4; ++k) {
                sum += a.m[k * 4 + row] * b.m[col * 4 + k];
            }
            r.m[col * 4 + row] = sum;
        }
    }
    return r;
}

Mat4 mat4Perspective(float fovYRad, float aspect, float nearM, float farM) {
    Mat4 r{};
    const float f = 1.0f / std::tan(fovYRad * 0.5f);
    r.m[0] = f / aspect;
    r.m[5] = f;
    r.m[10] = (farM + nearM) / (nearM - farM);
    r.m[11] = -1.0f;
    r.m[14] = 2.0f * farM * nearM / (nearM - farM);
    return r;
}

namespace {
void normalize3(float v[3]) {
    const float len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    if (len > 0.0f) {
        v[0] /= len;
        v[1] /= len;
        v[2] /= len;
    }
}
}  // namespace

Mat4 mat4LookAt(const float eye[3], const float target[3], const float up[3]) {
    float f[3] = {target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]};
    normalize3(f);
    float s[3] = {f[1] * up[2] - f[2] * up[1], f[2] * up[0] - f[0] * up[2], f[0] * up[1] - f[1] * up[0]};
    normalize3(s);
    const float u[3] = {s[1] * f[2] - s[2] * f[1], s[2] * f[0] - s[0] * f[2], s[0] * f[1] - s[1] * f[0]};
    Mat4 r = mat4Identity();
    r.m[0] = s[0];
    r.m[4] = s[1];
    r.m[8] = s[2];
    r.m[1] = u[0];
    r.m[5] = u[1];
    r.m[9] = u[2];
    r.m[2] = -f[0];
    r.m[6] = -f[1];
    r.m[10] = -f[2];
    r.m[12] = -(s[0] * eye[0] + s[1] * eye[1] + s[2] * eye[2]);
    r.m[13] = -(u[0] * eye[0] + u[1] * eye[1] + u[2] * eye[2]);
    r.m[14] = (f[0] * eye[0] + f[1] * eye[1] + f[2] * eye[2]);
    return r;
}

namespace {

const char* const kFragmentPreamble =
    "#ifdef GL_ES\n"
    "#ifdef GL_FRAGMENT_PRECISION_HIGH\n"
    "precision highp float;\n"
    "#else\n"
    "precision mediump float;\n"
    "#endif\n"
    "#endif\n";

GLuint compile(GlFuncs& gl, GLenum type, const std::string& src, std::string& log) {
    const GLuint shader = gl.CreateShader(type);
    const char* ptr = src.c_str();
    gl.ShaderSource(shader, 1, &ptr, nullptr);
    gl.CompileShader(shader);
    GLint ok = 0;
    gl.GetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (ok == 0) {
        char buf[2048] = {};
        gl.GetShaderInfoLog(shader, sizeof(buf) - 1, nullptr, buf);
        log += strf("%s shader compile failed: %s\n", type == GL_VERTEX_SHADER ? "vertex" : "fragment", buf);
        gl.DeleteShader(shader);
        return 0;
    }
    return shader;
}

}  // namespace

GLuint buildProgram(GlFuncs& gl, const char* vertexSrc, const char* fragmentSrc,
                    const std::vector<const char*>& attribs, std::string& log) {
    const GLuint vs = compile(gl, GL_VERTEX_SHADER, vertexSrc, log);
    const GLuint fs = compile(gl, GL_FRAGMENT_SHADER, std::string(kFragmentPreamble) + fragmentSrc, log);
    if (vs == 0 || fs == 0) {
        if (vs != 0) {
            gl.DeleteShader(vs);
        }
        if (fs != 0) {
            gl.DeleteShader(fs);
        }
        return 0;
    }
    const GLuint program = gl.CreateProgram();
    gl.AttachShader(program, vs);
    gl.AttachShader(program, fs);
    for (size_t i = 0; i < attribs.size(); ++i) {
        gl.BindAttribLocation(program, static_cast<GLuint>(i), attribs[i]);
    }
    gl.LinkProgram(program);
    gl.DeleteShader(vs);
    gl.DeleteShader(fs);
    GLint ok = 0;
    gl.GetProgramiv(program, GL_LINK_STATUS, &ok);
    if (ok == 0) {
        char buf[2048] = {};
        gl.GetProgramInfoLog(program, sizeof(buf) - 1, nullptr, buf);
        log += strf("program link failed: %s\n", buf);
        gl.DeleteProgram(program);
        return 0;
    }
    return program;
}

// ---------------------------------------------------------------------------------------------
// TextRenderer
// ---------------------------------------------------------------------------------------------

namespace {

constexpr int kAtlasCols = 16;
constexpr int kAtlasRows = 8;
constexpr int kAtlasW = kAtlasCols * 8;
constexpr int kAtlasH = kAtlasRows * 8;
constexpr int kSolidCell = kFontNumChars;  // one extra cell after the glyphs: fully lit

const char* const kTextVs =
    "attribute vec2 aPos;\n"
    "attribute vec2 aUv;\n"
    "attribute vec4 aColor;\n"
    "uniform vec2 uScreen;\n"
    "varying vec2 vUv;\n"
    "varying vec4 vColor;\n"
    "void main() {\n"
    "  vec2 ndc = vec2(aPos.x / uScreen.x * 2.0 - 1.0, 1.0 - aPos.y / uScreen.y * 2.0);\n"
    "  gl_Position = vec4(ndc, 0.0, 1.0);\n"
    "  vUv = aUv;\n"
    "  vColor = aColor;\n"
    "}\n";

const char* const kTextFs =
    "uniform sampler2D uTex;\n"
    "varying vec2 vUv;\n"
    "varying vec4 vColor;\n"
    "void main() {\n"
    "  gl_FragColor = vColor * texture2D(uTex, vUv);\n"
    "}\n";

}  // namespace

bool TextRenderer::init(GlFuncs* gl, std::string& log) {
    gl_ = gl;
    program_ = buildProgram(*gl_, kTextVs, kTextFs, {"aPos", "aUv", "aColor"}, log);
    if (program_ == 0) {
        return false;
    }
    uScreen_ = gl_->GetUniformLocation(program_, "uScreen");
    uTex_ = gl_->GetUniformLocation(program_, "uTex");

    unsigned char pixels[kAtlasW * kAtlasH * 4];
    std::memset(pixels, 0, sizeof(pixels));
    for (int cell = 0; cell <= kSolidCell; ++cell) {
        const int cx = (cell % kAtlasCols) * 8;
        const int cy = (cell / kAtlasCols) * 8;
        for (int row = 0; row < 8; ++row) {
            const unsigned char bits = cell == kSolidCell ? 0xff : kFont8x8[cell * 8 + row];
            for (int col = 0; col < 8; ++col) {
                const bool on = (bits & (0x80 >> col)) != 0;
                unsigned char* px = &pixels[((cy + row) * kAtlasW + cx + col) * 4];
                px[0] = px[1] = px[2] = 255;
                px[3] = on ? 255 : 0;
            }
        }
    }
    gl_->GenTextures(1, &texture_);
    gl_->BindTexture(GL_TEXTURE_2D, texture_);
    gl_->PixelStorei(GL_UNPACK_ALIGNMENT, 1);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    gl_->TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    gl_->TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, kAtlasW, kAtlasH, 0, GL_RGBA, GL_UNSIGNED_BYTE, pixels);
    gl_->GenBuffers(1, &vbo_);
    vertices_.reserve(6 * 6000);
    return true;
}

void TextRenderer::shutdown() {
    if (gl_ == nullptr) {
        return;
    }
    if (vbo_ != 0) {
        gl_->DeleteBuffers(1, &vbo_);
    }
    if (texture_ != 0) {
        gl_->DeleteTextures(1, &texture_);
    }
    if (program_ != 0) {
        gl_->DeleteProgram(program_);
    }
    vbo_ = texture_ = program_ = 0;
}

void TextRenderer::begin(int widthPx, int heightPx) {
    widthPx_ = widthPx;
    heightPx_ = heightPx;
    vertices_.clear();
}

void TextRenderer::quad(float x, float y, float w, float h, float u0, float v0, float u1, float v1, const Color& c) {
    const Vertex a{x, y, u0, v0, c.r, c.g, c.b, c.a};
    const Vertex b{x + w, y, u1, v0, c.r, c.g, c.b, c.a};
    const Vertex d{x + w, y + h, u1, v1, c.r, c.g, c.b, c.a};
    const Vertex e{x, y + h, u0, v1, c.r, c.g, c.b, c.a};
    vertices_.push_back(a);
    vertices_.push_back(b);
    vertices_.push_back(d);
    vertices_.push_back(a);
    vertices_.push_back(d);
    vertices_.push_back(e);
}

void TextRenderer::text(float xPx, float yPx, float scale, const Color& color, const std::string& s) {
    float x = xPx;
    const float size = 8.0f * scale;
    for (const char ch : s) {
        int idx = static_cast<unsigned char>(ch) - kFontFirstChar;
        if (idx < 0 || idx >= kFontNumChars) {
            idx = '?' - kFontFirstChar;
        }
        if (ch != ' ') {
            const float u0 = static_cast<float>((idx % kAtlasCols) * 8) / kAtlasW;
            const float v0 = static_cast<float>((idx / kAtlasCols) * 8) / kAtlasH;
            quad(x, yPx, size, size, u0, v0, u0 + 8.0f / kAtlasW, v0 + 8.0f / kAtlasH, color);
        }
        x += size;
    }
}

void TextRenderer::rect(float xPx, float yPx, float wPx, float hPx, const Color& color) {
    // Sample the centre of the solid cell so filtering never picks up a transparent neighbour.
    const float u = (static_cast<float>((kSolidCell % kAtlasCols) * 8) + 4.0f) / kAtlasW;
    const float v = (static_cast<float>((kSolidCell / kAtlasCols) * 8) + 4.0f) / kAtlasH;
    quad(xPx, yPx, wPx, hPx, u, v, u, v, color);
}

void TextRenderer::flush() {
    if (vertices_.empty() || program_ == 0) {
        return;
    }
    gl_->Disable(GL_DEPTH_TEST);
    gl_->Disable(GL_CULL_FACE);
    gl_->Enable(GL_BLEND);
    gl_->BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    gl_->UseProgram(program_);
    gl_->Uniform2f(uScreen_, static_cast<float>(widthPx_), static_cast<float>(heightPx_));
    gl_->ActiveTexture(GL_TEXTURE0);
    gl_->BindTexture(GL_TEXTURE_2D, texture_);
    gl_->Uniform1i(uTex_, 0);
    gl_->BindBuffer(GL_ARRAY_BUFFER, vbo_);
    gl_->BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(vertices_.size() * sizeof(Vertex)), vertices_.data(),
                    GL_STREAM_DRAW);
    const GLsizei stride = sizeof(Vertex);
    gl_->EnableVertexAttribArray(0);
    gl_->EnableVertexAttribArray(1);
    gl_->EnableVertexAttribArray(2);
    gl_->VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(0));
    gl_->VertexAttribPointer(1, 2, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(8));
    gl_->VertexAttribPointer(2, 4, GL_FLOAT, GL_FALSE, stride, reinterpret_cast<const void*>(16));
    gl_->DrawArrays(GL_TRIANGLES, 0, static_cast<GLsizei>(vertices_.size()));
    gl_->DisableVertexAttribArray(0);
    gl_->DisableVertexAttribArray(1);
    gl_->DisableVertexAttribArray(2);
    gl_->BindBuffer(GL_ARRAY_BUFFER, 0);
    vertices_.clear();
}

}  // namespace probe
