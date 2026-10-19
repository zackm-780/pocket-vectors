// Renderer2D: virtual canvas transform, clipping, outline ordering and draw-call count.
// GL is faked through GlApi function pointers, so no context is needed.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "hud/Renderer2D.h"

using namespace fsim;
using namespace fsim::gfx;

namespace {

struct FakeGl {
    static inline int drawArrays = 0, drawElements = 0, bufferData = 0, drawVertices = 0;
    static inline long long uploadedBytes = 0;

    GlApi api;
    FakeGl() {
        drawArrays = drawElements = bufferData = drawVertices = 0;
        uploadedBytes = 0;
        api.Viewport = [](GLint, GLint, GLsizei, GLsizei) {};
        api.Enable = [](GLenum) {};
        api.Disable = [](GLenum) {};
        api.BlendFunc = [](GLenum, GLenum) {};
        api.GenBuffers = [](GLsizei n, GLuint* b) {
            for (GLsizei i = 0; i < n; ++i) b[i] = 7;
        };
        api.DeleteBuffers = [](GLsizei, const GLuint*) {};
        api.BindBuffer = [](GLenum, GLuint) {};
        api.BufferData = [](GLenum, GLsizeiptr size, const void*, GLenum) {
            ++bufferData;
            uploadedBytes += size;
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
            ++drawArrays;
            drawVertices += n;
        };
        api.DrawElements = [](GLenum, GLsizei, GLenum, const void*) { ++drawElements; };
    }
};

const GlApi& noGl() {
    static FakeGl gl;
    return gl.api;
}

void bbox(const Renderer2D& r, float& x0, float& y0, float& x1, float& y1) {
    x0 = y0 = 1e9f;
    x1 = y1 = -1e9f;
    for (int i = 0; i < r.batchVertexCount(); ++i) {
        const auto& v = r.batchData()[i];
        x0 = std::min(x0, v.x);
        x1 = std::max(x1, v.x);
        y0 = std::min(y0, v.y);
        y1 = std::max(y1, v.y);
    }
}

}  // namespace

TEST_CASE("hud: virtual canvas transform for 4:3") {
    const HudView a = HudView::fromDrawable(640, 480);
    CHECK(a.scale == doctest::Approx(1.0f));
    CHECK(a.virtualWidth == doctest::Approx(640.0f));
    const HudView b = HudView::fromDrawable(1280, 960);
    CHECK(b.scale == doctest::Approx(2.0f));
    CHECK(b.virtualWidth == doctest::Approx(640.0f));
    float px, py;
    b.toPixel(320, 240, px, py);
    CHECK(px == doctest::Approx(640));
    CHECK(py == doctest::Approx(480));
    b.toPixel(640, 480, px, py);
    CHECK(px == doctest::Approx(1280));
    CHECK(py == doctest::Approx(960));
    float vx, vy;
    b.toVirtual(1280, 960, vx, vy);
    CHECK(vx == doctest::Approx(640));
    CHECK(vy == doctest::Approx(480));
}

TEST_CASE("hud: virtual canvas transform for 16:9 extends the width") {
    const HudView v = HudView::fromDrawable(1280, 720);
    CHECK(v.scale == doctest::Approx(1.5f));
    CHECK(v.virtualWidth == doctest::Approx(1280.0f / 1.5f));
    float px, py;
    v.toPixel(v.virtualWidth, 480, px, py);  // right edge, bottom edge
    CHECK(px == doctest::Approx(1280));
    CHECK(py == doctest::Approx(720));
    const HudView w = HudView::fromDrawable(1920, 1080);
    CHECK(w.virtualWidth == doctest::Approx(480.0f * 16.0f / 9.0f));

    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(1280, 720);
    CHECK(r.virtualWidth() == doctest::Approx(853.333f).epsilon(1e-4));
    CHECK(r.leftX() == 0.0f);
    CHECK(r.rightX() == doctest::Approx(r.virtualWidth()));
    CHECK(r.centerX() == doctest::Approx(r.virtualWidth() * 0.5f));
    // A 640-wide design stays centered: its center maps to the canvas center, its left edge
    // moves right by half the extra width.
    CHECK(r.centered(320.0f) == doctest::Approx(r.centerX()));
    CHECK(r.centered(0.0f) == doctest::Approx((r.virtualWidth() - 640.0f) * 0.5f));
    r.begin(640, 480);
    CHECK(r.centered(17.0f) == doctest::Approx(17.0f));
}

TEST_CASE("hud: a line becomes a quad of the requested width") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setOutline(false);
    r.setLineWidth(4.0f);
    r.line(10, 50, 110, 50, colors::kWhite);
    r.end();
    REQUIRE(r.batchVertexCount() == 6);
    float x0, y0, x1, y1;
    bbox(r, x0, y0, x1, y1);
    CHECK(x0 == doctest::Approx(10));
    CHECK(x1 == doctest::Approx(110));
    CHECK(y0 == doctest::Approx(48));
    CHECK(y1 == doctest::Approx(52));
}

TEST_CASE("hud: transforms translate and rotate") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setOutline(false);
    r.setLineWidth(2.0f);
    r.push();
    r.translate(100, 100);
    r.rotate(3.14159265f * 0.5f);  // +x maps to +y (clockwise on screen)
    r.line(0, 0, 40, 0, colors::kWhite);
    r.pop();
    r.line(0, 0, 40, 0, colors::kWhite);  // after pop: identity again
    r.end();
    float x0, y0, x1, y1;
    bbox(r, x0, y0, x1, y1);
    REQUIRE(r.batchVertexCount() == 12);
    // First line: x in [99,101], y in [100,140].
    float ax0 = 1e9f, ax1 = -1e9f, ay0 = 1e9f, ay1 = -1e9f;
    for (int i = 0; i < 6; ++i) {
        const auto& v = r.batchData()[i];
        ax0 = std::min(ax0, v.x);
        ax1 = std::max(ax1, v.x);
        ay0 = std::min(ay0, v.y);
        ay1 = std::max(ay1, v.y);
    }
    CHECK(ax0 == doctest::Approx(99).epsilon(1e-3));
    CHECK(ax1 == doctest::Approx(101).epsilon(1e-3));
    CHECK(ay0 == doctest::Approx(100).epsilon(1e-3));
    CHECK(ay1 == doctest::Approx(140).epsilon(1e-3));
    // Second line is back at the origin, horizontal.
    float bx1 = -1e9f;
    for (int i = 6; i < 12; ++i) bx1 = std::max(bx1, r.batchData()[i].x);
    CHECK(bx1 == doctest::Approx(40));
}

TEST_CASE("hud: CPU clipping keeps geometry inside the clip rect") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setOutline(false);
    r.pushClip(100, 100, 50, 40);
    r.fillRect(0, 0, 640, 480, colors::kWhite);  // covers the clip: becomes the clip rect
    r.line(0, 120, 640, 120, colors::kWhite);    // crosses the clip horizontally
    r.line(300, 300, 400, 400, colors::kWhite);  // fully outside: dropped
    r.end();
    REQUIRE(r.batchVertexCount() > 0);
    float x0, y0, x1, y1;
    bbox(r, x0, y0, x1, y1);
    CHECK(x0 >= 100 - 1e-4f);
    CHECK(x1 <= 150 + 1e-4f);
    CHECK(y0 >= 100 - 1e-4f);
    CHECK(y1 <= 140 + 1e-4f);
    CHECK(x0 == doctest::Approx(100));
    CHECK(x1 == doctest::Approx(150));

    // Nothing inside: empty batch. Then pop restores unclipped drawing.
    r.begin(640, 480);
    r.setOutline(false);
    r.pushClip(0, 0, 10, 10);
    r.fillRect(50, 50, 20, 20, colors::kWhite);
    CHECK(r.pendingVertexCount() == 0);
    r.popClip();
    r.fillRect(50, 50, 20, 20, colors::kWhite);
    CHECK(r.pendingVertexCount() == 6);
}

TEST_CASE("hud: nested clips intersect") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setOutline(false);
    r.pushClip(0, 0, 100, 100);
    r.pushClip(50, 50, 100, 100);  // intersection: 50..100
    r.fillRect(0, 0, 640, 480, colors::kWhite);
    r.end();
    float x0, y0, x1, y1;
    bbox(r, x0, y0, x1, y1);
    CHECK(x0 == doctest::Approx(50));
    CHECK(x1 == doctest::Approx(100));
    CHECK(y0 == doctest::Approx(50));
    CHECK(y1 == doctest::Approx(100));
}

TEST_CASE("hud: outline vertices come first, dark, and colored strokes on top") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setLineWidth(2.0f);
    r.fillRect(0, 0, 640, 480, Color(0, 0, 1, 1));  // Back fill: before the outlines
    r.line(10, 10, 100, 10, colors::kHudGreen);
    r.text(20, 40, "AB", TextSize::M, colors::kHudGreen);
    const int outlineCount = r.pendingOutlineVertexCount();
    CHECK(outlineCount > 0);
    r.end();
    const int total = r.batchVertexCount();
    REQUIRE(total > outlineCount + 6);
    // [0,6) back fill (blue), then the outline block (black, alpha ~ 0.6), then the strokes.
    CHECK(r.batchData()[0].b == 255);
    for (int i = 6; i < 6 + outlineCount; ++i) {
        const auto& v = r.batchData()[i];
        CHECK(v.r == 0);
        CHECK(v.g == 0);
        CHECK(v.b == 0);
        CHECK(v.a == doctest::Approx(0.6f * 255).epsilon(0.02));
    }
    for (int i = 6 + outlineCount; i < total; ++i) {
        const auto& v = r.batchData()[i];
        CHECK(v.g > 200);  // HUD green
    }
}

TEST_CASE("hud: outline is wider than the stroke by the outline width on each side") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setLineWidth(2.0f);
    r.line(10, 50, 110, 50, colors::kWhite);
    r.end();
    // Outline quad first: y in [50-2, 50+2]; stroke quad: y in [49, 51].
    float oy0 = 1e9f, oy1 = -1e9f, cy0 = 1e9f, cy1 = -1e9f;
    for (int i = 0; i < 6; ++i) {
        oy0 = std::min(oy0, r.batchData()[i].y);
        oy1 = std::max(oy1, r.batchData()[i].y);
    }
    for (int i = 6; i < 12; ++i) {
        cy0 = std::min(cy0, r.batchData()[i].y);
        cy1 = std::max(cy1, r.batchData()[i].y);
    }
    CHECK(oy0 == doctest::Approx(48));
    CHECK(oy1 == doctest::Approx(52));
    CHECK(cy0 == doctest::Approx(49));
    CHECK(cy1 == doctest::Approx(51));
}

TEST_CASE("hud: text lands where the alignment says") {
    Renderer2D r(noGl(), GlFlavor::Es2);
    r.begin(640, 480);
    r.setOutline(false);
    r.setTextStrokeWidth(1.0f);
    // "I" at L: one cell of 12 px; cap height 16. Right/baseline aligned at (200, 100).
    r.text(200, 100, "H", TextSize::L, colors::kWhite, HAlign::Right, VAlign::Baseline);
    r.end();
    float x0, y0, x1, y1;
    bbox(r, x0, y0, x1, y1);
    // Cell spans x 188..200; glyph body (32 px / 4 units = 2 px/unit, 8 px wide) centered in it,
    // stroke half-width 0.5 and square caps extend y by 0.5.
    CHECK(x0 == doctest::Approx(188 + 2 - 0.5f).epsilon(1e-3));
    CHECK(x1 == doctest::Approx(200 - 2 + 0.5f).epsilon(1e-3));
    CHECK(y0 == doctest::Approx(84 - 0.5f).epsilon(1e-3));
    CHECK(y1 == doctest::Approx(100 + 0.5f).epsilon(1e-3));
}

TEST_CASE("hud: whole HUD frame is one draw call") {
    FakeGl gl;
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    for (int frame = 0; frame < 3; ++frame) {
        r.begin(1280, 960);
        for (int i = 0; i < 20; ++i) {
            r.line(10, 10.0f * i, 300, 10.0f * i + 5, colors::kHudGreen);
            r.arc(320, 240, 100.0f + i, 0, 3.0f, colors::kHudGreen);
        }
        r.fillRect(0, 0, 100, 100, Color(0, 0, 0, 0.4f));
        r.fillTriangle({1, 1}, {9, 1}, {5, 9}, colors::kAmber);
        r.circle(100, 100, 20, colors::kWhite);
        r.pushClip(50, 50, 200, 200);
        r.push();
        r.rotate(0.3f);
        r.text(100, 100, "IAS 132  ALT 3200  HDG 271" FSIM_SYM_DEGREE, TextSize::M,
               colors::kHudGreen);
        r.pop();
        r.popClip();
        r.end();
        CHECK(r.stats().drawCalls == 1);
        CHECK(r.stats().vertices == r.batchVertexCount());
        CHECK(r.stats().vertices > 1000);
    }
    CHECK(FakeGl::drawArrays == 3);
    CHECK(FakeGl::drawElements == 0);
    CHECK(FakeGl::bufferData == 3);
    CHECK(r.totalDrawCalls() == 3);
    CHECK(FakeGl::drawVertices > 3000);

    // An empty frame draws nothing.
    r.begin(1280, 960);
    r.end();
    CHECK(r.stats().drawCalls == 0);
    CHECK(FakeGl::drawArrays == 3);
}

TEST_CASE("hud: batch capacity is stable after warm-up") {
    FakeGl gl;
    Renderer2D r(gl.api, GlFlavor::Es2);
    REQUIRE(r.init());
    auto frame = [&] {
        r.begin(640, 480);
        for (int i = 0; i < 50; ++i)
            r.text(10, 12.0f + 12 * i, "IAS 132 ALT 3200", TextSize::M, colors::kHudGreen);
        r.end();
    };
    frame();
    const Renderer2D::Vertex* first = r.batchData();
    for (int i = 0; i < 5; ++i) {
        frame();
        CHECK(r.batchData() == first);  // merged_ never reallocated
    }
}
