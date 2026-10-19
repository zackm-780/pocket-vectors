#include "GpuBench.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <set>
#include <sstream>
#include <vector>

#include "Common.h"

namespace probe {

namespace {

// ---------------------------------------------------------------------------------------------
// GL information
// ---------------------------------------------------------------------------------------------

std::string glString(Ctx& ctx, GLenum name) {
    const GLubyte* s = ctx.gl.GetString(name);
    return s != nullptr ? reinterpret_cast<const char*>(s) : std::string("(null)");
}

std::set<std::string> extensionSet(const std::string& all) {
    std::set<std::string> out;
    std::istringstream in(all);
    std::string tok;
    while (in >> tok) {
        out.insert(tok);
    }
    return out;
}

// Reads an integer limit; returns false if the enum is not valid on this context (desktop GL 2.1).
bool getInt(Ctx& ctx, GLenum name, GLint& value) {
    while (ctx.gl.GetError() != GL_NO_ERROR) {
    }
    value = 0;
    ctx.gl.GetIntegerv(name, &value);
    return ctx.gl.GetError() == GL_NO_ERROR;
}

// ---------------------------------------------------------------------------------------------
// Benchmark scene
// ---------------------------------------------------------------------------------------------

constexpr float kHalfExtentM = 3000.0f;  // the terrain mesh covers 6 km x 6 km

float terrainHeightM(float x, float z) {
    return 380.0f + 250.0f * std::sin(x * 0.0011f) * std::cos(z * 0.0014f) + 120.0f * std::sin(x * 0.0031f + z * 0.0027f) +
           40.0f * std::sin(x * 0.011f) * std::sin(z * 0.009f);
}

struct Mesh {
    GLuint vbo = 0;
    GLuint ibo16 = 0;
    GLuint ibo32 = 0;
    int gridN = 0;
    int numVerts = 0;
    int numTris = 0;
    bool fitsU16 = false;
};

enum class Variant { kSingleDraw16, kManyDraws16, kSingleDraw32 };

const char* const kTerrainVs =
    "attribute vec3 aPos;\n"
    "attribute vec3 aNormal;\n"
    "uniform mat4 uMvp;\n"
    "uniform vec3 uCamPos;\n"
    "uniform vec3 uSunDir;\n"
    "varying vec3 vColor;\n"
    "varying float vDist;\n"
    "void main() {\n"
    "  gl_Position = uMvp * vec4(aPos, 1.0);\n"
    "  float h = clamp(aPos.y / 900.0, 0.0, 1.0);\n"
    "  vec3 base = mix(vec3(0.18, 0.40, 0.14), vec3(0.60, 0.55, 0.45), h);\n"
    "  float lit = 0.35 + 0.65 * max(dot(normalize(aNormal), uSunDir), 0.0);\n"
    "  vColor = base * lit;\n"
    "  vDist = length(aPos - uCamPos);\n"
    "}\n";

// Per-fragment exponential-squared fog, the cost the real terrain shader will have.
const char* const kTerrainFs =
    "uniform vec3 uFogColor;\n"
    "uniform float uFogDensity;\n"
    "varying vec3 vColor;\n"
    "varying float vDist;\n"
    "void main() {\n"
    "  float k = uFogDensity * vDist;\n"
    "  float f = clamp(exp2(-k * k * 1.442695), 0.0, 1.0);\n"
    "  gl_FragColor = vec4(mix(uFogColor, vColor, f), 1.0);\n"
    "}\n";

const char* const kQuadVs =
    "attribute vec2 aPos;\n"
    "varying vec2 vUv;\n"
    "void main() {\n"
    "  gl_Position = vec4(aPos, 0.0, 1.0);\n"
    "  vUv = aPos * 0.5 + 0.5;\n"
    "}\n";

const char* const kFlatFs =
    "varying vec2 vUv;\n"
    "void main() {\n"
    "  gl_FragColor = vec4(vUv, 0.3, 0.5);\n"
    "}\n";

const char* const kFogFs =
    "uniform vec3 uFogColor;\n"
    "uniform float uFogDensity;\n"
    "varying vec2 vUv;\n"
    "void main() {\n"
    "  float d = length(vUv - vec2(0.5)) * 30000.0;\n"
    "  float k = uFogDensity * d;\n"
    "  float f = clamp(exp2(-k * k * 1.442695), 0.0, 1.0);\n"
    "  gl_FragColor = vec4(mix(uFogColor, vec3(vUv, 0.4), f), 0.5);\n"
    "}\n";

struct Fbo {
    GLuint fbo = 0;
    GLuint color = 0;
    GLuint depth = 0;
    int w = 0;
    int h = 0;
};

class GpuBench {
public:
    explicit GpuBench(Ctx& ctx) : ctx_(ctx), gl_(ctx.gl), report_(*ctx.report) {}

    void run() {
        report_.section("GPU BENCHMARK");
        if (!ctx_.glOk) {
            report_.line("skipped: no GL context");
            return;
        }
        if (!setup()) {
            return;
        }
        const double passS = ctx_.opt.quick ? 0.5 : 1.5;
        const double screenS = ctx_.opt.quick ? 0.5 : 2.0;

        report_.line(strf("render size %d x %d, depth bits %d, offscreen = FBO RGBA8 + 16-bit depth", ctx_.widthPx,
                          ctx_.heightPx, ctx_.obtained.depthBits));
        report_.line("Terrain-like heightfield, per-vertex lighting, per-fragment exp2 fog. 'GPU ms' = draw + glFinish,");
        report_.line("'submit ms' = time until the draw calls returned (CPU/driver cost).");

        struct Tier {
            const char* label;
            int targetTris;
            Variant variant;
        };
        const Tier tiers[] = {
            {"10k tris", 10000, Variant::kSingleDraw16},
            {"40k tris", 40000, Variant::kSingleDraw16},
            {"80k tris", 80000, Variant::kSingleDraw16},
            {"40k tris / 150 draw calls", 40000, Variant::kManyDraws16},
        };
        std::vector<double> fitTris;
        std::vector<double> fitMs;
        double ms40k = -1.0;
        double submit40kMs = -1.0;
        bool stop = false;
        for (const Tier& tier : tiers) {
            if (stop || ctx_.quitRequested) {
                break;
            }
            Mesh mesh = buildMesh(tier.targetTris, false);
            ctx_.phase = std::string("gpu benchmark: ") + tier.label;
            ctx_.drawStatusScreen("GPU benchmark running, please wait...", progress_);
            const OffscreenResult off = runOffscreen(mesh, tier.variant, passS);
            const OnscreenResult on = runOnscreen(mesh, tier.variant, screenS);
            report_.line(strf("%-26s %6d tris %6d verts | GPU ms avg %6.2f p50 %6.2f p95 %6.2f max %6.2f | submit %5.2f | "
                              "on-screen vsync: avg %6.2f ms p95 %6.2f, >20ms %d/%d, >36ms %d/%d",
                              tier.label, mesh.numTris, mesh.numVerts, off.gpu.mean, off.gpu.p50, off.gpu.p95, off.gpu.max,
                              off.submit.mean, on.period.mean, on.period.p95, on.over20, static_cast<int>(on.period.count),
                              on.over36, static_cast<int>(on.period.count)));
            if (off.coveragePct >= 0.0) {
                report_.line(strf("    sanity: %.0f%% of sampled pixels were covered by terrain", off.coveragePct));
            }
            if (tier.variant == Variant::kSingleDraw16) {
                fitTris.push_back(mesh.numTris);
                fitMs.push_back(off.gpu.p50);
            }
            if (tier.targetTris == 40000 && tier.variant == Variant::kSingleDraw16) {
                ms40k = off.gpu.p50;
                submit40kMs = off.submit.mean;
            }
            freeMesh(mesh);
            progress_ += 0.15;
            report_.save();
        }

        if (!stop && !ctx_.quitRequested && extensions_.count("GL_OES_element_index_uint") != 0) {
            Mesh mesh = buildMesh(80000, true);
            ctx_.phase = "gpu benchmark: 80k tris, 32-bit indices";
            ctx_.drawStatusScreen("GPU benchmark running, please wait...", progress_);
            const OffscreenResult off = runOffscreen(mesh, Variant::kSingleDraw32, passS);
            report_.line(strf("%-26s %6d tris %6d verts | GPU ms avg %6.2f p50 %6.2f p95 %6.2f max %6.2f | submit %5.2f",
                              "80k tris, uint32 indices", mesh.numTris, mesh.numVerts, off.gpu.mean, off.gpu.p50,
                              off.gpu.p95, off.gpu.max, off.submit.mean));
            freeMesh(mesh);
        } else if (extensions_.count("GL_OES_element_index_uint") == 0) {
            report_.line("GL_OES_element_index_uint NOT available: meshes need 16-bit indices (max 65535 vertices per draw,");
            report_.line("so a 257x257 terrain tile (66049 vertices) must be split or decimated).");
        }

        // Fill rate.
        ctx_.phase = "gpu benchmark: fill rate";
        ctx_.drawStatusScreen("GPU benchmark running, please wait...", 0.8);
        const double fillOpaque = measureFill(flatProgram_, false, passS);
        const double fillBlend = measureFill(flatProgram_, true, passS);
        const double fillFogBlend = measureFill(fogProgram_, true, passS);
        report_.line(strf("fill rate, trivial shader, opaque:        %8.0f Mpixel/s  (may be optimistic: HW can cull hidden overdraw)", fillOpaque));
        report_.line(strf("fill rate, trivial shader, alpha blended: %8.0f Mpixel/s", fillBlend));
        report_.line(strf("fill rate, fog shader, alpha blended:     %8.0f Mpixel/s  (%.1f full-screen layers per 16.7 ms at %dx%d)",
                          fillFogBlend, fillFogBlend * 1e6 * 0.0167 / (static_cast<double>(ctx_.widthPx) * ctx_.heightPx),
                          ctx_.widthPx, ctx_.heightPx));

        // Fit time = a + b * triangles on the single-draw tiers and solve for the frame budgets.
        std::string fact;
        double tris30 = -1.0;
        double tris60 = -1.0;
        if (fitTris.size() >= 2) {
            double sx = 0, sy = 0, sxx = 0, sxy = 0;
            const double n = static_cast<double>(fitTris.size());
            for (size_t i = 0; i < fitTris.size(); ++i) {
                sx += fitTris[i];
                sy += fitMs[i];
                sxx += fitTris[i] * fitTris[i];
                sxy += fitTris[i] * fitMs[i];
            }
            const double denom = n * sxx - sx * sx;
            const double b = denom != 0.0 ? (n * sxy - sx * sy) / denom : 0.0;  // ms per triangle
            const double a = (sy - b * sx) / n;                                  // fixed cost ms
            report_.line(strf("linear fit: GPU ms = %.2f + %.4f ms per 1000 triangles", a, b * 1000.0));
            if (b > 1e-9) {
                tris30 = (1000.0 / 30.0 - a) / b;
                tris60 = (1000.0 / 60.0 - a) / b;
            }
        }
        const double maxMeasured = fitTris.empty() ? 0.0 : fitTris.back();
        if (tris30 > 0.0) {
            const bool beyond = tris30 > maxMeasured;
            fact = strf("about %.0fk triangles/frame fit in 33.3 ms%s, about %.0fk in 16.7 ms%s (fog shader, %dx%d, GPU time only)",
                        tris30 / 1000.0, beyond ? " (extrapolated)" : "", std::max(0.0, tris60) / 1000.0,
                        tris60 > maxMeasured ? " (extrapolated)" : "", ctx_.widthPx, ctx_.heightPx);
        } else {
            fact = "could not fit (GPU time did not grow with triangle count: fill/vsync bound?)";
        }
        if (ms40k >= 0.0) {
            fact += strf("; 40k tris = %.2f ms GPU (PLAN budget 15 ms: %s), %.2f ms submit", ms40k,
                         ms40k <= 15.0 ? "PASS" : "FAIL", submit40kMs);
        }
        fact += strf("; fill %.0f / %.0f / %.0f Mpix/s (opaque / blended / blended+fog)", fillOpaque, fillBlend, fillFogBlend);
        report_.fact(rows::kGpu, fact);
        report_.line("RESULT: " + fact);
        teardown();
        report_.save();
    }

private:
    struct OffscreenResult {
        Stats gpu;
        Stats submit;
        double coveragePct = -1.0;
    };
    struct OnscreenResult {
        Stats period;
        int over20 = 0;
        int over36 = 0;
    };

    bool setup() {
        extensions_ = extensionSet(glString(ctx_, GL_EXTENSIONS));
        std::string log;
        terrainProgram_ = buildProgram(gl_, kTerrainVs, kTerrainFs, {"aPos", "aNormal"}, log);
        flatProgram_ = buildProgram(gl_, kQuadVs, kFlatFs, {"aPos"}, log);
        fogProgram_ = buildProgram(gl_, kQuadVs, kFogFs, {"aPos"}, log);
        if (terrainProgram_ == 0 || flatProgram_ == 0 || fogProgram_ == 0) {
            report_.line("shader build FAILED, GPU benchmark skipped:");
            report_.line(log);
            return false;
        }
        uMvp_ = gl_.GetUniformLocation(terrainProgram_, "uMvp");
        uCamPos_ = gl_.GetUniformLocation(terrainProgram_, "uCamPos");
        uSunDir_ = gl_.GetUniformLocation(terrainProgram_, "uSunDir");
        uFogColor_ = gl_.GetUniformLocation(terrainProgram_, "uFogColor");
        uFogDensity_ = gl_.GetUniformLocation(terrainProgram_, "uFogDensity");
        uFlatFogColor_ = gl_.GetUniformLocation(fogProgram_, "uFogColor");
        uFlatFogDensity_ = gl_.GetUniformLocation(fogProgram_, "uFogDensity");

        const float quad[] = {-1, -1, 1, -1, 1, 1, -1, -1, 1, 1, -1, 1};
        gl_.GenBuffers(1, &quadVbo_);
        gl_.BindBuffer(GL_ARRAY_BUFFER, quadVbo_);
        gl_.BufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_STATIC_DRAW);

        if (!createFbo(ctx_.widthPx, ctx_.heightPx)) {
            report_.line("FBO creation failed: offscreen GPU timing unavailable, GPU benchmark skipped");
            return false;
        }
        // Vsync is on for the on-screen passes (what the game will use).
        const int swap = SDL_GL_SetSwapInterval(1);
        report_.line(strf("SDL_GL_SetSwapInterval(1) -> %d (%s); now %d", swap, swap == 0 ? "ok" : SDL_GetError(),
                          SDL_GL_GetSwapInterval()));
        return true;
    }

    void teardown() {
        gl_.BindFramebuffer(GL_FRAMEBUFFER, 0);
        if (fbo_.fbo != 0) gl_.DeleteFramebuffers(1, &fbo_.fbo);
        if (fbo_.color != 0) gl_.DeleteTextures(1, &fbo_.color);
        if (fbo_.depth != 0) gl_.DeleteRenderbuffers(1, &fbo_.depth);
        if (quadVbo_ != 0) gl_.DeleteBuffers(1, &quadVbo_);
        if (terrainProgram_ != 0) gl_.DeleteProgram(terrainProgram_);
        if (flatProgram_ != 0) gl_.DeleteProgram(flatProgram_);
        if (fogProgram_ != 0) gl_.DeleteProgram(fogProgram_);
        fbo_ = Fbo();
        quadVbo_ = terrainProgram_ = flatProgram_ = fogProgram_ = 0;
    }

    bool createFbo(int w, int h) {
        fbo_.w = w;
        fbo_.h = h;
        gl_.GenTextures(1, &fbo_.color);
        gl_.BindTexture(GL_TEXTURE_2D, fbo_.color);
        gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        gl_.TexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        gl_.TexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, nullptr);
        gl_.GenRenderbuffers(1, &fbo_.depth);
        gl_.BindRenderbuffer(GL_RENDERBUFFER, fbo_.depth);
        gl_.RenderbufferStorage(GL_RENDERBUFFER, GL_DEPTH_COMPONENT16, w, h);
        gl_.GenFramebuffers(1, &fbo_.fbo);
        gl_.BindFramebuffer(GL_FRAMEBUFFER, fbo_.fbo);
        gl_.FramebufferTexture2D(GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D, fbo_.color, 0);
        gl_.FramebufferRenderbuffer(GL_FRAMEBUFFER, GL_DEPTH_ATTACHMENT, GL_RENDERBUFFER, fbo_.depth);
        const GLenum status = gl_.CheckFramebufferStatus(GL_FRAMEBUFFER);
        gl_.BindFramebuffer(GL_FRAMEBUFFER, 0);
        if (status != GL_FRAMEBUFFER_COMPLETE) {
            report_.line(strf("FBO incomplete: status 0x%04x", status));
            return false;
        }
        return true;
    }

    Mesh buildMesh(int targetTris, bool wantU32) {
        Mesh m;
        m.gridN = std::max(2, static_cast<int>(std::ceil(std::sqrt(targetTris / 2.0))));
        const int n = m.gridN;
        m.numVerts = (n + 1) * (n + 1);
        m.numTris = 2 * n * n;
        m.fitsU16 = m.numVerts <= 65535;

        std::vector<float> verts(static_cast<size_t>(m.numVerts) * 6);
        const float step = 2.0f * kHalfExtentM / static_cast<float>(n);
        for (int j = 0; j <= n; ++j) {
            for (int i = 0; i <= n; ++i) {
                const float x = -kHalfExtentM + static_cast<float>(i) * step;
                const float z = -kHalfExtentM + static_cast<float>(j) * step;
                const float d = step;
                float nx = -(terrainHeightM(x + d, z) - terrainHeightM(x - d, z)) / (2.0f * d);
                float nz = -(terrainHeightM(x, z + d) - terrainHeightM(x, z - d)) / (2.0f * d);
                const float len = std::sqrt(nx * nx + 1.0f + nz * nz);
                nx /= len;
                nz /= len;
                float* v = &verts[static_cast<size_t>(j * (n + 1) + i) * 6];
                v[0] = x;
                v[1] = terrainHeightM(x, z);
                v[2] = z;
                v[3] = nx;
                v[4] = 1.0f / len;
                v[5] = nz;
            }
        }
        std::vector<unsigned int> idx;
        idx.reserve(static_cast<size_t>(m.numTris) * 3);
        for (int j = 0; j < n; ++j) {
            for (int i = 0; i < n; ++i) {
                const unsigned int v00 = static_cast<unsigned int>(j * (n + 1) + i);
                const unsigned int v10 = v00 + 1;
                const unsigned int v01 = v00 + static_cast<unsigned int>(n + 1);
                const unsigned int v11 = v01 + 1;
                // Counter-clockwise when seen from above (+y).
                idx.insert(idx.end(), {v00, v01, v11, v00, v11, v10});
            }
        }
        gl_.GenBuffers(1, &m.vbo);
        gl_.BindBuffer(GL_ARRAY_BUFFER, m.vbo);
        gl_.BufferData(GL_ARRAY_BUFFER, static_cast<GLsizeiptr>(verts.size() * sizeof(float)), verts.data(), GL_STATIC_DRAW);
        if (m.fitsU16) {
            std::vector<unsigned short> idx16(idx.begin(), idx.end());
            gl_.GenBuffers(1, &m.ibo16);
            gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo16);
            gl_.BufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(idx16.size() * sizeof(unsigned short)),
                           idx16.data(), GL_STATIC_DRAW);
        }
        if (wantU32) {
            gl_.GenBuffers(1, &m.ibo32);
            gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, m.ibo32);
            gl_.BufferData(GL_ELEMENT_ARRAY_BUFFER, static_cast<GLsizeiptr>(idx.size() * sizeof(unsigned int)), idx.data(),
                           GL_STATIC_DRAW);
        }
        gl_.BindBuffer(GL_ARRAY_BUFFER, 0);
        gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);
        return m;
    }

    void freeMesh(Mesh& m) {
        if (m.vbo != 0) gl_.DeleteBuffers(1, &m.vbo);
        if (m.ibo16 != 0) gl_.DeleteBuffers(1, &m.ibo16);
        if (m.ibo32 != 0) gl_.DeleteBuffers(1, &m.ibo32);
        m = Mesh();
    }

    void drawTerrain(const Mesh& m, Variant variant, double timeS, int w, int h) {
        gl_.Viewport(0, 0, w, h);
        gl_.ClearColor(0.62f, 0.72f, 0.85f, 1.0f);
        const bool depth = ctx_.obtained.depthBits > 0 || fboBound_;
        if (depth) {
            gl_.Enable(GL_DEPTH_TEST);
            gl_.DepthFunc(GL_LEQUAL);
            gl_.DepthMask(GL_TRUE);
            gl_.Clear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        } else {
            gl_.Disable(GL_DEPTH_TEST);
            gl_.Clear(GL_COLOR_BUFFER_BIT);
        }
        gl_.Enable(GL_CULL_FACE);
        gl_.CullFace(GL_BACK);
        gl_.Disable(GL_BLEND);

        const float yaw = 0.25f * std::sin(static_cast<float>(timeS) * 0.7f);
        const float eye[3] = {900.0f * std::sin(static_cast<float>(timeS) * 0.5f), 2200.0f, -3800.0f};
        const float target[3] = {800.0f * yaw, 250.0f, 700.0f};
        const float up[3] = {0.0f, 1.0f, 0.0f};
        const Mat4 proj = mat4Perspective(0.8727f, static_cast<float>(w) / static_cast<float>(h), 20.0f, 40000.0f);
        const Mat4 mvp = mat4Multiply(proj, mat4LookAt(eye, target, up));

        gl_.UseProgram(terrainProgram_);
        gl_.UniformMatrix4fv(uMvp_, 1, GL_FALSE, mvp.m);
        gl_.Uniform3f(uCamPos_, eye[0], eye[1], eye[2]);
        gl_.Uniform3f(uSunDir_, 0.40f, 0.80f, 0.30f);
        gl_.Uniform3f(uFogColor_, 0.62f, 0.72f, 0.85f);
        gl_.Uniform1f(uFogDensity_, 1.0f / 9000.0f);

        gl_.BindBuffer(GL_ARRAY_BUFFER, m.vbo);
        gl_.EnableVertexAttribArray(0);
        gl_.EnableVertexAttribArray(1);
        gl_.VertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 24, reinterpret_cast<const void*>(0));
        gl_.VertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 24, reinterpret_cast<const void*>(12));
        const bool u32 = variant == Variant::kSingleDraw32;
        gl_.BindBuffer(GL_ELEMENT_ARRAY_BUFFER, u32 ? m.ibo32 : m.ibo16);
        const GLenum type = u32 ? GL_UNSIGNED_INT : GL_UNSIGNED_SHORT;
        const int bytesPerIndex = u32 ? 4 : 2;
        const int totalIndices = m.numTris * 3;
        const int calls = variant == Variant::kManyDraws16 ? 150 : 1;
        int perCall = (totalIndices / calls) / 3 * 3;
        int first = 0;
        for (int c = 0; c < calls; ++c) {
            const int count = c == calls - 1 ? totalIndices - first : perCall;
            gl_.DrawElements(GL_TRIANGLES, count, type, reinterpret_cast<const void*>(static_cast<intptr_t>(first) * bytesPerIndex));
            first += count;
        }
        gl_.DisableVertexAttribArray(0);
        gl_.DisableVertexAttribArray(1);
    }

    double coveragePercent() {
        std::vector<unsigned char> px(static_cast<size_t>(fbo_.w) * static_cast<size_t>(fbo_.h) * 4);
        gl_.PixelStorei(GL_PACK_ALIGNMENT, 1);
        gl_.ReadPixels(0, 0, fbo_.w, fbo_.h, GL_RGBA, GL_UNSIGNED_BYTE, px.data());
        const unsigned char sky[3] = {static_cast<unsigned char>(0.62f * 255 + 0.5f), static_cast<unsigned char>(0.72f * 255 + 0.5f),
                                      static_cast<unsigned char>(0.85f * 255 + 0.5f)};
        int sampled = 0;
        int covered = 0;
        for (int y = 0; y < fbo_.h; y += 4) {
            for (int x = 0; x < fbo_.w; x += 4) {
                const unsigned char* p = &px[(static_cast<size_t>(y) * static_cast<size_t>(fbo_.w) + static_cast<size_t>(x)) * 4];
                ++sampled;
                if (std::abs(p[0] - sky[0]) > 6 || std::abs(p[1] - sky[1]) > 6 || std::abs(p[2] - sky[2]) > 6) {
                    ++covered;
                }
            }
        }
        return sampled > 0 ? 100.0 * covered / sampled : -1.0;
    }

    OffscreenResult runOffscreen(const Mesh& mesh, Variant variant, double durationS) {
        OffscreenResult result;
        std::vector<double> gpuMs;
        std::vector<double> submitMs;
        gl_.BindFramebuffer(GL_FRAMEBUFFER, fbo_.fbo);
        fboBound_ = true;
        const double start = nowS();
        int frame = 0;
        while (!ctx_.quitRequested && (nowS() - start < durationS || frame < 12)) {
            const double t0 = nowS();
            drawTerrain(mesh, variant, t0 - start, fbo_.w, fbo_.h);
            const double t1 = nowS();
            gl_.Finish();
            const double t2 = nowS();
            if (frame == 2) {
                result.coveragePct = coveragePercent();
            }
            if (frame >= 3) {
                gpuMs.push_back((t2 - t0) * 1000.0);
                submitMs.push_back((t1 - t0) * 1000.0);
            }
            ++frame;
            if (frame % 16 == 0) {
                ctx_.pumpEvents();
            }
        }
        gl_.BindFramebuffer(GL_FRAMEBUFFER, 0);
        fboBound_ = false;
        result.gpu = computeStats(gpuMs);
        result.submit = computeStats(submitMs);
        return result;
    }

    OnscreenResult runOnscreen(const Mesh& mesh, Variant variant, double durationS) {
        OnscreenResult result;
        std::vector<double> periodMs;
        const double start = nowS();
        double last = nowS();
        int frame = 0;
        while (!ctx_.quitRequested && (nowS() - start < durationS || frame < 12)) {
            drawTerrain(mesh, variant, nowS() - start, ctx_.widthPx, ctx_.heightPx);
            ctx_.textRenderer.begin(ctx_.widthPx, ctx_.heightPx);
            ctx_.textRenderer.text(4, 4, 1, Color{1, 1, 1, 1}, strf("tris %d  frame %d", mesh.numTris, frame));
            ctx_.textRenderer.flush();
            if (frame == 5) {
                ctx_.requestShot("terrain");
            }
            ctx_.endUiFrame();
            const double now = nowS();
            if (frame >= 5) {
                periodMs.push_back((now - last) * 1000.0);
            }
            last = now;
            ++frame;
            ctx_.pumpEvents();
        }
        result.period = computeStats(periodMs);
        for (double p : periodMs) {
            result.over20 += p > 20.0 ? 1 : 0;
            result.over36 += p > 36.0 ? 1 : 0;
        }
        return result;
    }

    // Median GPU ms per frame drawing `quads` full-screen quads offscreen.
    double fillMs(GLuint program, bool blend, int quads, double durationS) {
        gl_.BindFramebuffer(GL_FRAMEBUFFER, fbo_.fbo);
        gl_.Viewport(0, 0, fbo_.w, fbo_.h);
        gl_.Disable(GL_DEPTH_TEST);
        gl_.Disable(GL_CULL_FACE);
        if (blend) {
            gl_.Enable(GL_BLEND);
            gl_.BlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        } else {
            gl_.Disable(GL_BLEND);
        }
        gl_.UseProgram(program);
        if (program == fogProgram_) {
            gl_.Uniform3f(uFlatFogColor_, 0.62f, 0.72f, 0.85f);
            gl_.Uniform1f(uFlatFogDensity_, 1.0f / 9000.0f);
        }
        gl_.BindBuffer(GL_ARRAY_BUFFER, quadVbo_);
        gl_.EnableVertexAttribArray(0);
        gl_.VertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 8, reinterpret_cast<const void*>(0));
        std::vector<double> samples;
        const double start = nowS();
        int frame = 0;
        while (!ctx_.quitRequested && (nowS() - start < durationS || frame < 8)) {
            const double t0 = nowS();
            gl_.ClearColor(0, 0, 0, 1);
            gl_.Clear(GL_COLOR_BUFFER_BIT);
            for (int q = 0; q < quads; ++q) {
                gl_.DrawArrays(GL_TRIANGLES, 0, 6);
            }
            gl_.Finish();
            const double t1 = nowS();
            if (frame >= 3) {
                samples.push_back((t1 - t0) * 1000.0);
            }
            ++frame;
            if (frame % 16 == 0) {
                ctx_.pumpEvents();
            }
        }
        gl_.DisableVertexAttribArray(0);
        gl_.Disable(GL_BLEND);
        gl_.BindFramebuffer(GL_FRAMEBUFFER, 0);
        return computeStats(samples).p50;
    }

    // Returns Mpixel/s. Grows the layer count until a frame takes at least ~8 ms so timer and
    // submission overhead do not dominate.
    double measureFill(GLuint program, bool blend, double durationS) {
        int quads = 4;
        double ms = fillMs(program, blend, quads, 0.25);
        while (ms < 8.0 && quads < 4096 && !ctx_.quitRequested) {
            quads *= 2;
            ms = fillMs(program, blend, quads, 0.25);
        }
        ms = fillMs(program, blend, quads, durationS);
        if (ms <= 0.0) {
            return 0.0;
        }
        const double pixels = static_cast<double>(fbo_.w) * fbo_.h * quads;
        return pixels / (ms / 1000.0) / 1e6;
    }

    Ctx& ctx_;
    GlFuncs& gl_;
    Report& report_;
    std::set<std::string> extensions_;
    GLuint terrainProgram_ = 0;
    GLuint flatProgram_ = 0;
    GLuint fogProgram_ = 0;
    GLuint quadVbo_ = 0;
    GLint uMvp_ = -1, uCamPos_ = -1, uSunDir_ = -1, uFogColor_ = -1, uFogDensity_ = -1;
    GLint uFlatFogColor_ = -1, uFlatFogDensity_ = -1;
    Fbo fbo_;
    bool fboBound_ = false;
    double progress_ = 0.05;
};

}  // namespace

// ---------------------------------------------------------------------------------------------

void reportGlInfo(Ctx& ctx) {
    Report& report = *ctx.report;
    report.section("OPENGL ES");
    if (!ctx.glOk) {
        report.line("no GL context");
        return;
    }
    const std::string vendor = glString(ctx, GL_VENDOR);
    const std::string renderer = glString(ctx, GL_RENDERER);
    const std::string version = glString(ctx, GL_VERSION);
    const std::string glsl = glString(ctx, GL_SHADING_LANGUAGE_VERSION);
    const std::string extensions = glString(ctx, GL_EXTENSIONS);
    const std::set<std::string> extSet = extensionSet(extensions);
    report.kv("GL_VENDOR", vendor);
    report.kv("GL_RENDERER", renderer);
    report.kv("GL_VERSION", version);
    report.kv("GL_SHADING_LANGUAGE_VERSION", glsl);
    report.kv("context type", ctx.obtained.isEs ? strf("OpenGL ES %d.%d (requested ES 2.0)", ctx.obtained.major, ctx.obtained.minor)
                                                : strf("desktop OpenGL %d.%d compatibility (ES 2.0 context was NOT available)",
                                                       ctx.obtained.major, ctx.obtained.minor));
    report.kv("SDL_GL_ACCELERATED_VISUAL", strf("%d", ctx.obtained.accelerated));

    GLint v = 0;
    const struct {
        const char* name;
        GLenum e;
    } limits[] = {
        {"GL_MAX_TEXTURE_SIZE", GL_MAX_TEXTURE_SIZE},
        {"GL_MAX_CUBE_MAP_TEXTURE_SIZE", GL_MAX_CUBE_MAP_TEXTURE_SIZE},
        {"GL_MAX_RENDERBUFFER_SIZE", GL_MAX_RENDERBUFFER_SIZE},
        {"GL_MAX_VERTEX_ATTRIBS", GL_MAX_VERTEX_ATTRIBS},
        {"GL_MAX_VERTEX_UNIFORM_VECTORS", GL_MAX_VERTEX_UNIFORM_VECTORS},
        {"GL_MAX_VARYING_VECTORS", GL_MAX_VARYING_VECTORS},
        {"GL_MAX_FRAGMENT_UNIFORM_VECTORS", GL_MAX_FRAGMENT_UNIFORM_VECTORS},
        {"GL_MAX_TEXTURE_IMAGE_UNITS", GL_MAX_TEXTURE_IMAGE_UNITS},
        {"GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS", GL_MAX_VERTEX_TEXTURE_IMAGE_UNITS},
        {"GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS", GL_MAX_COMBINED_TEXTURE_IMAGE_UNITS},
        {"GL_SUBPIXEL_BITS", GL_SUBPIXEL_BITS},
    };
    GLint maxTex = 0;
    for (const auto& l : limits) {
        if (getInt(ctx, l.e, v)) {
            report.kv(l.name, strf("%d", v));
            if (l.e == GL_MAX_TEXTURE_SIZE) {
                maxTex = v;
            }
        } else {
            report.kv(l.name, "n/a on this context");
        }
    }
    if (ctx.gl.GetShaderPrecisionFormat != nullptr) {
        const struct {
            const char* name;
            GLenum shader;
            GLenum type;
        } precisions[] = {{"fragment highp float", GL_FRAGMENT_SHADER, GL_HIGH_FLOAT},
                          {"fragment mediump float", GL_FRAGMENT_SHADER, GL_MEDIUM_FLOAT},
                          {"vertex highp float", GL_VERTEX_SHADER, GL_HIGH_FLOAT}};
        for (const auto& p : precisions) {
            GLint range[2] = {0, 0};
            GLint precision = 0;
            while (ctx.gl.GetError() != GL_NO_ERROR) {
            }
            ctx.gl.GetShaderPrecisionFormat(p.shader, p.type, range, &precision);
            if (ctx.gl.GetError() == GL_NO_ERROR) {
                report.kv(std::string("precision ") + p.name,
                          precision == 0 && range[0] == 0 ? "NOT SUPPORTED" : strf("range 2^%d..2^%d, %d mantissa bits", range[0], range[1], precision));
            }
        }
    }
    report.line("-- extensions of interest --");
    for (const char* e : {"GL_OES_element_index_uint", "GL_OES_vertex_array_object", "GL_OES_depth24", "GL_OES_depth_texture",
                          "GL_OES_packed_depth_stencil", "GL_OES_standard_derivatives", "GL_OES_texture_float",
                          "GL_OES_texture_half_float", "GL_OES_rgb8_rgba8", "GL_OES_mapbuffer", "GL_EXT_texture_filter_anisotropic",
                          "GL_EXT_multisampled_render_to_texture", "GL_EXT_discard_framebuffer", "GL_EXT_texture_format_BGRA8888",
                          "GL_KHR_texture_compression_astc_ldr", "GL_OES_compressed_ETC1_RGB8_texture",
                          "GL_EXT_texture_compression_s3tc", "GL_ARM_shader_framebuffer_fetch", "GL_EXT_shader_framebuffer_fetch"}) {
        report.line(strf("  %-44s %s", e, extSet.count(e) != 0 ? "yes" : "no"));
    }
    report.line(strf("-- all %zu extensions --", extSet.size()));
    std::string row;
    for (const auto& e : extSet) {
        if (row.size() + e.size() + 1 > 110) {
            report.line("  " + row);
            row.clear();
        }
        row += (row.empty() ? "" : " ") + e;
    }
    if (!row.empty()) {
        report.line("  " + row);
    }

    report.section("WINDOW AND FRAMEBUFFER OBTAINED (main window)");
    const GlObtained& o = ctx.obtained;
    report.kv("colour bits R/G/B/A", strf("%d/%d/%d/%d", o.redBits, o.greenBits, o.blueBits, o.alphaBits));
    report.kv("depth bits", strf("%d", o.depthBits));
    report.kv("stencil bits", strf("%d", o.stencilBits));
    report.kv("multisample buffers / samples", strf("%d / %d", o.msaaBuffers, o.msaaSamples));
    report.kv("double buffered", strf("%d", o.doubleBuffer));
    int w = 0, h = 0, dw = 0, dh = 0;
    SDL_GetWindowSize(ctx.window, &w, &h);
    SDL_GL_GetDrawableSize(ctx.window, &dw, &dh);
    report.kv("window size", strf("%d x %d", w, h));
    report.kv("drawable size", strf("%d x %d", dw, dh));
    report.kv("window flags", strf("0x%08x", SDL_GetWindowFlags(ctx.window)));
    const int swapNow = SDL_GL_GetSwapInterval();
    report.kv("swap interval (default)", strf("%d", swapNow));
    // Can vsync be switched off? Matters for measuring.
    const int off = SDL_GL_SetSwapInterval(0);
    report.kv("SDL_GL_SetSwapInterval(0) (vsync off)", off == 0 ? "supported" : strf("failed: %s", SDL_GetError()));
    const int on = SDL_GL_SetSwapInterval(1);
    report.kv("SDL_GL_SetSwapInterval(1) (vsync on)", on == 0 ? "supported" : strf("failed: %s", SDL_GetError()));

    report.fact(rows::kGl, strf("%s | %s | %s | %zu extensions (list in report)", version.c_str(), renderer.c_str(), vendor.c_str(),
                                extSet.size()));
    report.fact(rows::kMaxTex, strf("%d", maxTex));
    report.fact(rows::kDepthMsaa,
                strf("main window: depth %d, stencil %d, MSAA samples %d (config sweep result follows once it has run)", o.depthBits,
                     o.stencilBits, o.msaaSamples));
    report.save();
}

void runGpuBenchmark(Ctx& ctx) {
    GpuBench bench(ctx);
    bench.run();
}

void runConfigSweep(Ctx& ctx) {
    Report& report = *ctx.report;
    report.section("GL CONFIG SWEEP (depth / stencil / MSAA / colour depth)");
    report.line("Each request gets a fresh window and context; what SDL reports back is the real result.");
    report.line("If the probe stops here, the last 'trying' line is the config that crashed the driver.");
    struct Entry {
        const char* label;
        GlRequest request;
    };
    auto req = [](int depth, int stencil, int msaa, int r = -1, int g = -1, int b = -1, int a = -1) {
        GlRequest q;
        q.depthBits = depth;
        q.stencilBits = stencil;
        q.msaaSamples = msaa;
        q.redBits = r;
        q.greenBits = g;
        q.blueBits = b;
        q.alphaBits = a;
        return q;
    };
    const Entry entries[] = {
        {"depth24", req(24, 0, 0)},
        {"depth16", req(16, 0, 0)},
        {"depth24+stencil8", req(24, 8, 0)},
        {"depth24 MSAA2x", req(24, 0, 2)},
        {"depth24 MSAA4x", req(24, 0, 4)},
        {"depth16 MSAA4x", req(16, 0, 4)},
        {"RGB565 depth16", req(16, 0, 0, 5, 6, 5, 0)},
        {"RGBA8888 depth24", req(24, 0, 0, 8, 8, 8, 8)},
    };
    std::string summary;
    ctx.closeWindow();
    for (const Entry& e : entries) {
        report.line(strf("trying %-18s (depth %d, stencil %d, MSAA %d)...", e.label, e.request.depthBits,
                         e.request.stencilBits, e.request.msaaSamples));
        report.save();
        std::string error;
        if (ctx.openWindow(e.request, error)) {
            const GlObtained& o = ctx.obtained;
            report.line(strf("   -> got RGBA %d/%d/%d/%d depth %d stencil %d MSAA buffers %d samples %d | %s | %s", o.redBits,
                             o.greenBits, o.blueBits, o.alphaBits, o.depthBits, o.stencilBits, o.msaaBuffers, o.msaaSamples,
                             o.isEs ? "ES" : "desktop GL", glString(ctx, GL_RENDERER).c_str()));
            summary += strf("%s: depth %d/stencil %d/MSAA %d; ", e.label, o.depthBits, o.stencilBits, o.msaaSamples);
        } else {
            report.line("   -> FAILED: " + error);
            summary += strf("%s: not available; ", e.label);
        }
        ctx.closeWindow();
    }
    report.fact(rows::kDepthMsaa, summary);
    report.save();
}

}  // namespace probe
