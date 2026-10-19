#pragma once
// 2D HUD renderer (WP H1, HUD.md section 2, ARCHITECTURE section 4).
//
// * Virtual canvas: 480 px high, origin top-left, y down. Scale = drawable height / 480. Wider
//   aspect ratios extend the width symmetrically, so virtualWidth() = 480 * aspect (640 at 4:3,
//   853.3 at 16:9) and x runs 0..virtualWidth(). Design 640-wide layouts with centerX() and
//   centered(x640) so they stay centered; anchor left elements at leftX() and right elements at
//   rightX(). Camera::projectToHud() uses the same convention (center = hudWidth()/2).
// * Everything is CPU-built triangles in one vertex array, uploaded once and drawn with ONE
//   DrawArrays call in end(). Draw order inside the batch is: back fills (FillMode::Back), the
//   dark outline pass, then all coloured strokes and front fills, so every coloured stroke sits
//   on top of every outline and every outline sits on top of the backgrounds.
// * Thick lines are quads with butt or square caps and bevel joins with one extra rim vertex
//   (round-ish). Outline = same path 2 x outlineWidth wider, black, alpha 0.6 x colour alpha.
//   Overlaps inside the outline layer (joints, crossings) darken slightly; that is accepted.
// * Clipping is done on the CPU (Sutherland-Hodgman on the triangles), not with glScissor, so
//   the whole HUD stays one batch. The clip rect is axis-aligned in canvas coordinates.
// * Transforms: push()/pop() with translate()/rotate() (rotation in radians, clockwise on
//   screen because y is down). No scaling: line widths stay in virtual px. A clip rect passed to
//   pushClip() is mapped through the current transform as a bounding box.
// * No heap allocation after warm-up: vertex vectors keep their capacity across frames.
// * Angles for arcs: 0 = +x (right), increasing clockwise on screen (x = cx + r cos a,
//   y = cy + r sin a). So -pi/2 is straight up.

#include <cstdint>
#include <vector>

#include "gfx/GlApi.h"
#include "gfx/Shader.h"
#include "hud/StrokeFont.h"

namespace fsim {

constexpr float kHudVirtualHeightPx = 480.0f;
constexpr float kHudDesignWidthPx = 640.0f;

struct Color {
    float r = 1, g = 1, b = 1, a = 1;
    constexpr Color() = default;
    constexpr Color(float r_, float g_, float b_, float a_ = 1) : r(r_), g(g_), b(b_), a(a_) {}
    constexpr Color withAlpha(float alpha) const { return Color(r, g, b, a * alpha); }
};

namespace colors {
constexpr Color kHudGreen{0.20f, 1.00f, 0.30f};
constexpr Color kWhite{1, 1, 1};
constexpr Color kAmber{1.00f, 0.75f, 0.10f};
constexpr Color kRed{1.00f, 0.15f, 0.15f};
constexpr Color kBlack{0, 0, 0};
}  // namespace colors

// Virtual canvas <-> drawable pixels (pure math, unit-tested for 4:3 and 16:9).
struct HudView {
    int drawableW = 640, drawableH = 480;
    float scale = 1.0f;          // pixels per virtual px
    float virtualWidth = 640.0f; // = drawableW / scale

    static HudView fromDrawable(int widthPx, int heightPx);
    // Pixel coordinates use the top-left origin (row 0 = top), like the virtual canvas.
    void toPixel(float xVirtual, float yVirtual, float& xPx, float& yPx) const {
        xPx = xVirtual * scale;
        yPx = yVirtual * scale;
    }
    void toVirtual(float xPx, float yPx, float& xVirtual, float& yVirtual) const {
        xVirtual = xPx / scale;
        yVirtual = yPx / scale;
    }
};

struct Pt2 {
    float x = 0, y = 0;
};

enum class LineCap { Butt, Square };

// Where a filled shape goes in the batch (draw order: Back, outlines, then Front):
//   Back          below every outline and stroke: panels, tape backgrounds, tinted boxes.
//   Front         on top of the outlines, no outline of its own.
//   FrontOutlined on top of the outlines and with its own dark outline (bugs, pointers, FPV dot).
enum class FillMode { Back, Front, FrontOutlined };

struct Renderer2DStats {
    int drawCalls = 0;  // GL draw calls issued by the last end()
    int vertices = 0;   // vertices in the last batch (outline + colour)
    int triangles = 0;
};

class Renderer2D {
public:
    // Packed vertex: position in virtual px, straight-alpha colour.
    struct Vertex {
        float x, y;
        std::uint8_t r, g, b, a;
    };

    Renderer2D(const gfx::GlApi& gl, gfx::GlFlavor flavor) : gl_(gl), flavor_(flavor) {}
    ~Renderer2D();
    Renderer2D(const Renderer2D&) = delete;
    Renderer2D& operator=(const Renderer2D&) = delete;

    // Builds the shader and the vertex buffer. Without a successful init() the renderer still
    // records geometry (for tests) but end() issues no GL calls.
    bool init();

    // ---- frame ----------------------------------------------------------------------------
    // Starts a batch for a drawable of this size and resets all state (transform, clip, line
    // width 2, outline on 1 px / alpha 0.6).
    void begin(int drawableWidthPx, int drawableHeightPx);
    // Uploads the batch and issues exactly one draw call (none if the batch is empty).
    void end();

    // ---- canvas ---------------------------------------------------------------------------
    const HudView& view() const { return view_; }
    float virtualWidth() const { return view_.virtualWidth; }
    float virtualHeight() const { return kHudVirtualHeightPx; }
    float leftX() const { return 0.0f; }
    float rightX() const { return view_.virtualWidth; }
    float centerX() const { return view_.virtualWidth * 0.5f; }
    // Maps an x of a 640-wide design to the canvas keeping it centered.
    float centered(float x640) const { return x640 + (view_.virtualWidth - kHudDesignWidthPx) * 0.5f; }

    // ---- state ----------------------------------------------------------------------------
    void setLineWidth(float widthPx) { lineWidthPx_ = widthPx; }
    float lineWidth() const { return lineWidthPx_; }
    void setOutline(bool enabled) { outlineOn_ = enabled; }
    bool outline() const { return outlineOn_; }
    void setOutlineStyle(float widthPx, float alpha) {
        outlineWidthPx_ = widthPx;
        outlineAlpha_ = alpha;
    }
    // 0 = use StrokeFont::strokeWidthPx(size) for text.
    void setTextStrokeWidth(float widthPx) { textWidthPx_ = widthPx; }

    void push();  // saves the transform (max depth 16)
    void pop();
    void translate(float x, float y);
    void rotate(float rad);  // applied about the current origin; clockwise on screen

    void pushClip(float x, float y, float w, float h);  // intersects with the current clip
    void popClip();

    // ---- primitives (all coordinates in the current transform) ---------------------------------
    void line(float x0, float y0, float x1, float y1, Color c, LineCap cap = LineCap::Butt);
    void dashedLine(float x0, float y0, float x1, float y1, float dashPx, float gapPx, Color c);
    void polyline(const Pt2* pts, int n, Color c, bool closed = false,
                  LineCap cap = LineCap::Butt);
    void arc(float cx, float cy, float r, float a0Rad, float a1Rad, Color c);
    void circle(float cx, float cy, float r, Color c);
    void fillCircle(float cx, float cy, float r, Color c, FillMode mode = FillMode::FrontOutlined);
    void fillRect(float x, float y, float w, float h, Color c, FillMode mode = FillMode::Back);
    void strokeRect(float x, float y, float w, float h, Color c);
    void fillTriangle(Pt2 a, Pt2 b, Pt2 c, Color col, FillMode mode = FillMode::FrontOutlined);
    void strokeTriangle(Pt2 a, Pt2 b, Pt2 c, Color col);
    void fillPolygon(const Pt2* pts, int n, Color c, FillMode mode = FillMode::Back);  // convex

    // Stroke text. (x, y) is the anchor for the given alignment (see StrokeFont).
    void text(float x, float y, const char* utf8, TextSize size, Color c,
              HAlign h = HAlign::Left, VAlign v = VAlign::Baseline);

    // ---- introspection (tests, debugging) ---------------------------------------------------
    const Renderer2DStats& stats() const { return stats_; }
    long long totalDrawCalls() const { return totalDrawCalls_; }
    // The merged batch (outline first), valid after end().
    const Vertex* batchData() const { return merged_.data(); }
    int batchVertexCount() const { return static_cast<int>(merged_.size()); }
    // Vertices recorded since begin(), before end().
    int pendingVertexCount() const {
        return static_cast<int>(backVerts_.size() + outlineVerts_.size() + colorVerts_.size());
    }
    int pendingOutlineVertexCount() const { return static_cast<int>(outlineVerts_.size()); }

private:
    struct Affine {
        float a = 1, b = 0, c = 0, d = 1, tx = 0, ty = 0;  // x' = a x + c y + tx, y' = b x + d y + ty
    };
    struct ClipRect {
        float x0, y0, x1, y1;
    };

    Pt2 xf(float x, float y) const {
        return {xf_.a * x + xf_.c * y + xf_.tx, xf_.b * x + xf_.d * y + xf_.ty};
    }
    void emitTriangle(std::vector<Vertex>& out, Pt2 a, Pt2 b, Pt2 c, const std::uint8_t rgba[4]);
    // Strokes a path given in the CURRENT transform's local coordinates.
    void strokePath(const Pt2* local, int n, bool closed, float widthPx, LineCap cap, Color c,
                    bool outline);
    void emitStroke(std::vector<Vertex>& out, const Pt2* pts, int n, bool closed, float widthPx,
                    LineCap cap, const std::uint8_t rgba[4]);
    void fillScreenPolygon(std::vector<Vertex>& out, const Pt2* pts, int n,
                           const std::uint8_t rgba[4]);
    void fillPath(const Pt2* local, int n, Color c, FillMode mode);
    void outlineColor(Color c, std::uint8_t rgba[4]) const;
    static void packColor(Color c, std::uint8_t rgba[4]);
    void drawGlyphStrokes(const Glyph& g, float originX, float capTopY, float unit, Color c,
                          float widthPx);

    const gfx::GlApi& gl_;
    gfx::GlFlavor flavor_;
    gfx::ShaderProgram program_;
    bool ready_ = false;
    gfx::GLuint vbo_ = 0;
    gfx::GLint uScale_ = -1;

    HudView view_;
    Affine xf_;
    Affine xfStack_[16];
    int xfDepth_ = 0;
    ClipRect clip_{};
    bool clipOn_ = false;
    ClipRect clipStack_[8];
    bool clipStackOn_[8] = {};
    int clipDepth_ = 0;

    float lineWidthPx_ = 2.0f;
    bool outlineOn_ = true;
    float outlineWidthPx_ = 1.0f;
    float outlineAlpha_ = 0.6f;
    float textWidthPx_ = 0.0f;

    std::vector<Vertex> backVerts_, outlineVerts_, colorVerts_, merged_;
    std::vector<Pt2> local_, scratch_, scratch2_;  // reused path buffers
    Renderer2DStats stats_;
    long long totalDrawCalls_ = 0;
};

}  // namespace fsim
