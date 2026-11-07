// X3c: Renderer2D builds its batch per stroke (shared directions for the outline and colour
// passes, stroke-level clip tests) and uploads it with BufferSubData into rotating buffers. These
// tests pin that the vertices are exactly what per-triangle clipping of the unclipped strokes
// gives, and how the batch reaches GL.
#include <doctest/doctest.h>

#include <cstring>
#include <vector>

#include "hud/Renderer2D.h"

using namespace fsim;
using namespace fsim::gfx;

namespace {

struct UploadLog {
    static inline std::vector<GLuint> bound;  // ARRAY_BUFFER bindings
    static inline std::vector<long long> dataSizes;
    static inline std::vector<bool> dataNull;
    static inline std::vector<long long> subOffsets, subSizes;
    static inline int drawArrays = 0, drawCount = 0;
    static void reset() {
        bound.clear();
        dataSizes.clear();
        dataNull.clear();
        subOffsets.clear();
        subSizes.clear();
        drawArrays = drawCount = 0;
    }
};

struct BatchFakeGl {
    GlApi api;
    explicit BatchFakeGl(bool withSubData) {
        UploadLog::reset();
        api.Viewport = [](GLint, GLint, GLsizei, GLsizei) {};
        api.Enable = [](GLenum) {};
        api.Disable = [](GLenum) {};
        api.BlendFunc = [](GLenum, GLenum) {};
        api.GenBuffers = [](GLsizei n, GLuint* b) {
            for (GLsizei i = 0; i < n; ++i) b[i] = static_cast<GLuint>(11 + i);
        };
        api.DeleteBuffers = [](GLsizei, const GLuint*) {};
        api.BindBuffer = [](GLenum target, GLuint b) {
            if (target == GL_ARRAY_BUFFER) UploadLog::bound.push_back(b);
        };
        api.BufferData = [](GLenum, GLsizeiptr size, const void* data, GLenum) {
            UploadLog::dataSizes.push_back(size);
            UploadLog::dataNull.push_back(data == nullptr);
        };
        if (withSubData)
            api.BufferSubData = [](GLenum, GLintptr offset, GLsizeiptr size, const void*) {
                UploadLog::subOffsets.push_back(offset);
                UploadLog::subSizes.push_back(size);
            };
        api.CreateShader = [](GLenum) -> GLuint { return 1; };
        api.ShaderSource = [](GLuint, GLsizei, const GLchar* const*, const GLint*) {};
        api.CompileShader = [](GLuint) {};
        api.GetShaderiv = [](GLuint, GLenum, GLint* p) { *p = 1; };
        api.GetShaderInfoLog = [](GLuint, GLsizei, GLsizei*, GLchar*) {};
        api.DeleteShader = [](GLuint) {};
        api.CreateProgram = []() -> GLuint { return 3; };
        api.AttachShader = [](GLuint, GLuint) {};
        api.BindAttribLocation = [](GLuint, GLuint, const GLchar*) {};
        api.LinkProgram = [](GLuint) {};
        api.GetProgramiv = [](GLuint, GLenum, GLint* p) { *p = 1; };
        api.GetProgramInfoLog = [](GLuint, GLsizei, GLsizei*, GLchar*) {};
        api.UseProgram = [](GLuint) {};
        api.DeleteProgram = [](GLuint) {};
        api.GetUniformLocation = [](GLuint, const GLchar*) -> GLint { return 0; };
        api.Uniform4f = [](GLint, GLfloat, GLfloat, GLfloat, GLfloat) {};
        api.EnableVertexAttribArray = [](GLuint) {};
        api.DisableVertexAttribArray = [](GLuint) {};
        api.VertexAttribPointer = [](GLuint, GLint, GLenum, GLboolean, GLsizei, const void*) {};
        api.DrawArrays = [](GLenum, GLint, GLsizei n) {
            ++UploadLog::drawArrays;
            UploadLog::drawCount += n;
        };
        api.DrawElements = [](GLenum, GLsizei, GLenum, const void*) {};
    }
};

using V = Renderer2D::Vertex;

// Reference clipping: Sutherland-Hodgman of one triangle against the rect, fanned out again,
// as Renderer2D did per triangle before X3c.
int clipEdgeRef(const Pt2* in, int n, Pt2* out, int edge, float v) {
    int m = 0;
    auto inside = [&](const Pt2& p) {
        switch (edge) {
        case 0: return p.x >= v;
        case 1: return p.x <= v;
        case 2: return p.y >= v;
        default: return p.y <= v;
        }
    };
    for (int i = 0; i < n; ++i) {
        const Pt2& a = in[i];
        const Pt2& b = in[(i + 1) % n];
        const bool ia = inside(a), ib = inside(b);
        if (ia) out[m++] = a;
        if (ia != ib) {
            const float t = edge < 2 ? (v - a.x) / (b.x - a.x) : (v - a.y) / (b.y - a.y);
            out[m++] = {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        }
    }
    return m;
}

std::vector<V> clipBatchRef(const V* in, int count, float x0, float y0, float x1, float y1) {
    std::vector<V> out;
    for (int t = 0; t + 2 < count; t += 3) {
        const V& a = in[t];
        const V& b = in[t + 1];
        const V& c = in[t + 2];
        const float minX = std::min({a.x, b.x, c.x}), maxX = std::max({a.x, b.x, c.x});
        const float minY = std::min({a.y, b.y, c.y}), maxY = std::max({a.y, b.y, c.y});
        if (maxX <= x0 || minX >= x1 || maxY <= y0 || minY >= y1) continue;
        if (minX >= x0 && maxX <= x1 && minY >= y0 && maxY <= y1) {
            out.push_back(a);
            out.push_back(b);
            out.push_back(c);
            continue;
        }
        Pt2 bufA[8] = {{a.x, a.y}, {b.x, b.y}, {c.x, c.y}}, bufB[8];
        int n = 3;
        n = clipEdgeRef(bufA, n, bufB, 0, x0);
        if (n) n = clipEdgeRef(bufB, n, bufA, 1, x1);
        if (n) n = clipEdgeRef(bufA, n, bufB, 2, y0);
        if (n) n = clipEdgeRef(bufB, n, bufA, 3, y1);
        for (int i = 1; i + 1 < n; ++i) {
            for (const Pt2& p : {bufA[0], bufA[i], bufA[i + 1]})
                out.push_back({p.x, p.y, a.r, a.g, a.b, a.a});
        }
    }
    return out;
}

bool sameVertices(const std::vector<V>& a, const V* b, int bCount) {
    if (static_cast<int>(a.size()) != bCount) return false;
    return bCount == 0 || std::memcmp(a.data(), b, sizeof(V) * static_cast<size_t>(bCount)) == 0;
}

// A mix of strokes that fall inside, outside and across the clip rect: lines with both caps,
// polylines with joins, closed shapes, arcs, outlined fills and text (stroked and filled glyphs).
void drawMix(Renderer2D& r) {
    r.setLineWidth(2.0f);
    for (int i = 0; i < 24; ++i) {
        const float x = 20.0f + 25.0f * static_cast<float>(i);
        r.line(x, 40, x + 30, 160 + 7.0f * static_cast<float>(i % 5), colors::kHudGreen,
               i % 2 ? LineCap::Square : LineCap::Butt);
    }
    const Pt2 zig[] = {{90, 300}, {140, 200}, {190, 300}, {240, 210}, {300, 290}, {360, 230}};
    r.polyline(zig, 6, colors::kWhite);
    r.strokeRect(150, 120, 260, 140, colors::kAmber);
    r.circle(320, 240, 120, colors::kHudGreen);
    r.arc(100, 400, 60, -2.0f, 0.5f, colors::kRed);
    r.fillTriangle({300, 90}, {340, 90}, {320, 130}, colors::kHudGreen);
    r.fillCircle(500, 200, 9, colors::kWhite);
    r.fillRect(400, 300, 80, 30, colors::kBlack.withAlpha(0.5f), FillMode::Back);
    r.text(130, 250, "IAS 132 ALT 3200 " FSIM_SYM_TRIANGLE_UP FSIM_SYM_DIAMOND, TextSize::M,
           colors::kHudGreen);
    r.text(420, 180, "HDG 271" FSIM_SYM_DEGREE, TextSize::L, colors::kWhite);
    r.dashedLine(60, 330, 600, 350, 6, 4, colors::kHudGreen);
}

}  // namespace

TEST_CASE("hud: clipped strokes equal the unclipped strokes clipped per triangle") {
    BatchFakeGl gl(true);
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    const float cx = 170, cy = 140, cw = 260, ch = 170;  // cuts through most of the mix

    r.begin(640, 480);
    drawMix(r);
    r.end();
    const std::vector<V> all(r.batchData(), r.batchData() + r.batchVertexCount());
    REQUIRE(all.size() > 3000);

    r.begin(640, 480);
    r.pushClip(cx, cy, cw, ch);
    drawMix(r);
    r.popClip();
    r.end();
    const std::vector<V> clipped(r.batchData(), r.batchData() + r.batchVertexCount());

    // Each layer (backs, outlines, colour) is clipped on its own; the reference clips the merged
    // unclipped batch, which keeps the same order because clipping never reorders triangles.
    const std::vector<V> ref =
        clipBatchRef(all.data(), static_cast<int>(all.size()), cx, cy, cx + cw, cy + ch);
    CHECK(ref.size() < all.size());
    CHECK(ref.size() > 500);
    CHECK(sameVertices(ref, clipped.data(), static_cast<int>(clipped.size())));
}

TEST_CASE("hud: a clip rect around everything changes no vertex, one beside it drops them all") {
    BatchFakeGl gl(true);
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    r.begin(640, 480);
    drawMix(r);
    r.end();
    const std::vector<V> all(r.batchData(), r.batchData() + r.batchVertexCount());

    r.begin(640, 480);
    r.pushClip(-50, -50, 740, 580);
    drawMix(r);
    r.popClip();
    r.end();
    CHECK(sameVertices(all, r.batchData(), r.batchVertexCount()));

    r.begin(640, 480);
    r.pushClip(700, 0, 100, 100);  // off to the right of every shape
    drawMix(r);
    r.popClip();
    r.end();
    CHECK(r.batchVertexCount() == 0);
    CHECK(r.stats().drawCalls == 0);
}

TEST_CASE("hud: the batch goes up in three sub-uploads into 4 rotating buffers") {
    BatchFakeGl gl(true);
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    for (int frame = 0; frame < 6; ++frame) {
        r.begin(640, 480);
        drawMix(r);
        r.end();
    }
    REQUIRE(UploadLog::dataSizes.size() == 6);
    for (bool isNull : UploadLog::dataNull) CHECK(isNull);  // storage only, data comes after
    REQUIRE(UploadLog::subSizes.size() == 18);              // backs, outlines, colour
    long long sum = 0, nextOffset = 0;
    for (size_t i = 0; i < 3; ++i) {
        CHECK(UploadLog::subOffsets[i] == nextOffset);
        nextOffset += UploadLog::subSizes[i];
        sum += UploadLog::subSizes[i];
    }
    CHECK(sum == UploadLog::dataSizes[0]);
    CHECK(sum == static_cast<long long>(r.stats().vertices) * static_cast<long long>(sizeof(V)));
    // Bindings cycle 11, 12, 13, 14, 11, 12: a buffer is reused only 4 batches later.
    REQUIRE(UploadLog::bound.size() == 6);
    for (size_t i = 0; i < 6; ++i) CHECK(UploadLog::bound[i] == 11 + i % 4);
    CHECK(UploadLog::drawArrays == 6);
}

TEST_CASE("hud: without BufferSubData the merged batch is uploaded in one call") {
    BatchFakeGl gl(false);
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    r.begin(640, 480);
    drawMix(r);
    r.end();
    REQUIRE(UploadLog::dataSizes.size() == 1);
    CHECK_FALSE(UploadLog::dataNull[0]);
    CHECK(UploadLog::dataSizes[0] ==
          static_cast<long long>(r.batchVertexCount()) * static_cast<long long>(sizeof(V)));
    CHECK(UploadLog::subSizes.empty());
    CHECK(UploadLog::drawCount == r.batchVertexCount());
}

TEST_CASE("hud: batch layers keep their order (backs, outlines, colour)") {
    BatchFakeGl gl(true);
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    r.begin(640, 480);
    r.line(10, 10, 100, 10, colors::kHudGreen);  // outline + colour
    r.fillRect(0, 0, 50, 50, colors::kRed, FillMode::Back);
    r.end();
    REQUIRE(r.batchVertexCount() == 18);  // 6 back, 6 outline, 6 colour
    const V* v = r.batchData();
    CHECK(v[0].r == 255);  // red back first
    CHECK(v[0].g == 38);
    CHECK(v[6].r == 0);  // black outline next
    CHECK(v[6].g == 0);
    CHECK(v[12].g == 255);  // green stroke last
}
