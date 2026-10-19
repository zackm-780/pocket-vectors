#include "hud/Renderer2D.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstring>

#include "core/Log.h"

namespace fsim {
namespace {

constexpr float kPi = 3.14159265358979f;

constexpr char kVertexShader[] =
    "attribute vec2 aPos;\n"
    "attribute vec4 aColor;\n"
    "uniform vec4 uScale;\n"  // x = 2/virtualWidth, y = 2/480
    "varying vec4 vColor;\n"
    "void main() {\n"
    "  vColor = aColor;\n"
    "  gl_Position = vec4(aPos.x * uScale.x - 1.0, 1.0 - aPos.y * uScale.y, 0.0, 1.0);\n"
    "}\n";

constexpr char kFragmentShader[] = "varying vec4 vColor;\n"
                                   "void main() { gl_FragColor = vColor; }\n";

constexpr size_t kInitialVertices = 1u << 15;

using Vertex = Renderer2D::Vertex;

void pushVertex(std::vector<Vertex>& out, Pt2 p, const std::uint8_t rgba[4]) {
    out.push_back({p.x, p.y, rgba[0], rgba[1], rgba[2], rgba[3]});
}

// Clips a convex polygon (max 8 points) against one axis-aligned half plane.
// edge: 0 x>=lo, 1 x<=hi, 2 y>=lo, 3 y<=hi.
int clipEdge(const Pt2* in, int n, Pt2* out, int edge, float v) {
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
            float t;
            if (edge < 2)
                t = (v - a.x) / (b.x - a.x);
            else
                t = (v - a.y) / (b.y - a.y);
            out[m++] = {a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        }
    }
    return m;
}

}  // namespace

HudView HudView::fromDrawable(int widthPx, int heightPx) {
    HudView v;
    v.drawableW = std::max(widthPx, 1);
    v.drawableH = std::max(heightPx, 1);
    v.scale = static_cast<float>(v.drawableH) / kHudVirtualHeightPx;
    v.virtualWidth = static_cast<float>(v.drawableW) / v.scale;
    return v;
}

Renderer2D::~Renderer2D() {
    if (vbo_) gl_.DeleteBuffers(1, &vbo_);
}

bool Renderer2D::init() {
    backVerts_.reserve(kInitialVertices);
    outlineVerts_.reserve(kInitialVertices);
    colorVerts_.reserve(kInitialVertices);
    merged_.reserve(3 * kInitialVertices);
    local_.reserve(256);
    scratch_.reserve(256);
    scratch2_.reserve(256);
    if (!program_.build(gl_, flavor_, kVertexShader, kFragmentShader, {"aPos", "aColor"})) {
        LOG_ERROR("Renderer2D: shader build failed");
        return false;
    }
    uScale_ = program_.uniform("uScale");
    gl_.GenBuffers(1, &vbo_);
    ready_ = vbo_ != 0;
    return ready_;
}

void Renderer2D::begin(int drawableWidthPx, int drawableHeightPx) {
    view_ = HudView::fromDrawable(drawableWidthPx, drawableHeightPx);
    xf_ = Affine{};
    xfDepth_ = 0;
    clipOn_ = false;
    clipDepth_ = 0;
    lineWidthPx_ = 2.0f;
    outlineOn_ = true;
    outlineWidthPx_ = 1.0f;
    outlineAlpha_ = 0.6f;
    textWidthPx_ = 0.0f;
    backVerts_.clear();
    outlineVerts_.clear();
    colorVerts_.clear();
    merged_.clear();
    stats_ = {};
    if (outlineVerts_.capacity() == 0) {  // used without init() (tests)
        backVerts_.reserve(kInitialVertices);
        outlineVerts_.reserve(kInitialVertices);
        colorVerts_.reserve(kInitialVertices);
        merged_.reserve(3 * kInitialVertices);
    }
}

void Renderer2D::end() {
    merged_.clear();
    merged_.insert(merged_.end(), backVerts_.begin(), backVerts_.end());
    merged_.insert(merged_.end(), outlineVerts_.begin(), outlineVerts_.end());
    merged_.insert(merged_.end(), colorVerts_.begin(), colorVerts_.end());
    const int n = static_cast<int>(merged_.size());
    stats_.vertices = n;
    stats_.triangles = n / 3;
    if (!ready_ || n == 0) return;

    gl_.Viewport(0, 0, view_.drawableW, view_.drawableH);
    gl_.Disable(gfx::GL_DEPTH_TEST);
    gl_.Disable(gfx::GL_CULL_FACE);
    gl_.Enable(gfx::GL_BLEND);
    gl_.BlendFunc(gfx::GL_SRC_ALPHA, gfx::GL_ONE_MINUS_SRC_ALPHA);
    program_.use();
    gl_.Uniform4f(uScale_, 2.0f / view_.virtualWidth, 2.0f / kHudVirtualHeightPx, 0.0f, 0.0f);

    gl_.BindBuffer(gfx::GL_ARRAY_BUFFER, vbo_);
    // BufferData with a new size orphans the old storage, so the driver never stalls on it.
    gl_.BufferData(gfx::GL_ARRAY_BUFFER, static_cast<gfx::GLsizeiptr>(n * sizeof(Vertex)),
                   merged_.data(), gfx::GL_DYNAMIC_DRAW);
    gl_.EnableVertexAttribArray(0);
    gl_.EnableVertexAttribArray(1);
    gl_.VertexAttribPointer(0, 2, gfx::GL_FLOAT, 0, sizeof(Vertex), nullptr);
    gl_.VertexAttribPointer(1, 4, gfx::GL_UNSIGNED_BYTE, 1, sizeof(Vertex),
                            reinterpret_cast<const void*>(offsetof(Vertex, r)));
    gl_.DrawArrays(gfx::GL_TRIANGLES, 0, n);
    stats_.drawCalls = 1;
    ++totalDrawCalls_;
    gl_.DisableVertexAttribArray(0);
    gl_.DisableVertexAttribArray(1);
    gl_.Disable(gfx::GL_BLEND);
}

// ---- state ---------------------------------------------------------------------------------

void Renderer2D::push() {
    if (xfDepth_ < 16) xfStack_[xfDepth_++] = xf_;
}

void Renderer2D::pop() {
    if (xfDepth_ > 0) xf_ = xfStack_[--xfDepth_];
}

void Renderer2D::translate(float x, float y) {
    const Pt2 t = xf(x, y);
    xf_.tx = t.x;
    xf_.ty = t.y;
}

void Renderer2D::rotate(float rad) {
    const float c = std::cos(rad), s = std::sin(rad);
    // New basis vectors: (a,b) and (c,d) rotated by `rad` in the local frame.
    const float a = xf_.a * c + xf_.c * s, b = xf_.b * c + xf_.d * s;
    const float cc = -xf_.a * s + xf_.c * c, d = -xf_.b * s + xf_.d * c;
    xf_.a = a;
    xf_.b = b;
    xf_.c = cc;
    xf_.d = d;
}

void Renderer2D::pushClip(float x, float y, float w, float h) {
    const Pt2 p0 = xf(x, y), p1 = xf(x + w, y), p2 = xf(x, y + h), p3 = xf(x + w, y + h);
    ClipRect r{std::min({p0.x, p1.x, p2.x, p3.x}), std::min({p0.y, p1.y, p2.y, p3.y}),
               std::max({p0.x, p1.x, p2.x, p3.x}), std::max({p0.y, p1.y, p2.y, p3.y})};
    if (clipDepth_ < 8) {
        clipStack_[clipDepth_] = clip_;
        clipStackOn_[clipDepth_] = clipOn_;
        ++clipDepth_;
    }
    if (clipOn_) {
        r.x0 = std::max(r.x0, clip_.x0);
        r.y0 = std::max(r.y0, clip_.y0);
        r.x1 = std::max(r.x0, std::min(r.x1, clip_.x1));
        r.y1 = std::max(r.y0, std::min(r.y1, clip_.y1));
    }
    clip_ = r;
    clipOn_ = true;
}

void Renderer2D::popClip() {
    if (clipDepth_ == 0) return;
    --clipDepth_;
    clip_ = clipStack_[clipDepth_];
    clipOn_ = clipStackOn_[clipDepth_];
}

// ---- low level -----------------------------------------------------------------------------

void Renderer2D::packColor(Color c, std::uint8_t rgba[4]) {
    auto q = [](float v) {
        return static_cast<std::uint8_t>(std::clamp(v, 0.0f, 1.0f) * 255.0f + 0.5f);
    };
    rgba[0] = q(c.r);
    rgba[1] = q(c.g);
    rgba[2] = q(c.b);
    rgba[3] = q(c.a);
}

void Renderer2D::outlineColor(Color c, std::uint8_t rgba[4]) const {
    packColor(Color(0, 0, 0, outlineAlpha_ * c.a), rgba);
}

void Renderer2D::emitTriangle(std::vector<Vertex>& out, Pt2 a, Pt2 b, Pt2 c,
                              const std::uint8_t rgba[4]) {
    if (!clipOn_) {
        pushVertex(out, a, rgba);
        pushVertex(out, b, rgba);
        pushVertex(out, c, rgba);
        return;
    }
    const float minX = std::min({a.x, b.x, c.x}), maxX = std::max({a.x, b.x, c.x});
    const float minY = std::min({a.y, b.y, c.y}), maxY = std::max({a.y, b.y, c.y});
    if (maxX <= clip_.x0 || minX >= clip_.x1 || maxY <= clip_.y0 || minY >= clip_.y1) return;
    if (minX >= clip_.x0 && maxX <= clip_.x1 && minY >= clip_.y0 && maxY <= clip_.y1) {
        pushVertex(out, a, rgba);
        pushVertex(out, b, rgba);
        pushVertex(out, c, rgba);
        return;
    }
    Pt2 bufA[8] = {a, b, c}, bufB[8];
    int n = 3;
    n = clipEdge(bufA, n, bufB, 0, clip_.x0);
    if (n) n = clipEdge(bufB, n, bufA, 1, clip_.x1);
    if (n) n = clipEdge(bufA, n, bufB, 2, clip_.y0);
    if (n) n = clipEdge(bufB, n, bufA, 3, clip_.y1);
    for (int i = 1; i + 1 < n; ++i) {
        pushVertex(out, bufA[0], rgba);
        pushVertex(out, bufA[i], rgba);
        pushVertex(out, bufA[i + 1], rgba);
    }
}

void Renderer2D::fillScreenPolygon(std::vector<Vertex>& out, const Pt2* pts, int n,
                                   const std::uint8_t rgba[4]) {
    for (int i = 1; i + 1 < n; ++i) emitTriangle(out, pts[0], pts[i], pts[i + 1], rgba);
}

void Renderer2D::emitStroke(std::vector<Vertex>& out, const Pt2* pts, int n, bool closed,
                            float widthPx, LineCap cap, const std::uint8_t rgba[4]) {
    // Drop coincident neighbours so every segment has a direction.
    scratch2_.clear();
    for (int i = 0; i < n; ++i) {
        if (!scratch2_.empty()) {
            const Pt2& q = scratch2_.back();
            if (std::fabs(pts[i].x - q.x) < 1e-4f && std::fabs(pts[i].y - q.y) < 1e-4f) continue;
        }
        scratch2_.push_back(pts[i]);
    }
    if (closed && scratch2_.size() > 1) {
        const Pt2& f = scratch2_.front();
        const Pt2& l = scratch2_.back();
        if (std::fabs(f.x - l.x) < 1e-4f && std::fabs(f.y - l.y) < 1e-4f) scratch2_.pop_back();
    }
    const int m = static_cast<int>(scratch2_.size());
    const float hw = widthPx * 0.5f;
    if (m == 0) return;
    if (m == 1) {
        if (cap == LineCap::Square) {  // a dot
            const Pt2 p = scratch2_[0];
            emitTriangle(out, {p.x - hw, p.y - hw}, {p.x + hw, p.y - hw}, {p.x + hw, p.y + hw},
                         rgba);
            emitTriangle(out, {p.x - hw, p.y - hw}, {p.x + hw, p.y + hw}, {p.x - hw, p.y + hw},
                         rgba);
        }
        return;
    }
    if (m == 2) closed = false;
    const int segs = closed ? m : m - 1;

    auto dirOf = [&](int i, Pt2& d) {
        const Pt2& p = scratch2_[static_cast<size_t>(i % m)];
        const Pt2& q = scratch2_[static_cast<size_t>((i + 1) % m)];
        const float dx = q.x - p.x, dy = q.y - p.y;
        const float len = std::sqrt(dx * dx + dy * dy);
        d = {dx / len, dy / len};
    };

    for (int i = 0; i < segs; ++i) {
        Pt2 d;
        dirOf(i, d);
        Pt2 p = scratch2_[static_cast<size_t>(i)];
        Pt2 q = scratch2_[static_cast<size_t>((i + 1) % m)];
        if (cap == LineCap::Square && !closed) {
            if (i == 0) p = {p.x - d.x * hw, p.y - d.y * hw};
            if (i == segs - 1) q = {q.x + d.x * hw, q.y + d.y * hw};
        }
        const float nx = -d.y * hw, ny = d.x * hw;
        const Pt2 a{p.x + nx, p.y + ny}, b{p.x - nx, p.y - ny};
        const Pt2 c{q.x + nx, q.y + ny}, e{q.x - nx, q.y - ny};
        emitTriangle(out, a, b, c, rgba);
        emitTriangle(out, c, b, e, rgba);
    }

    // Joins: bevel on the outer side plus one rim vertex, which reads as round at HUD widths.
    const int firstJoin = closed ? 0 : 1;
    const int lastJoin = closed ? m - 1 : m - 2;
    for (int j = firstJoin; j <= lastJoin; ++j) {
        Pt2 d0, d1;
        dirOf((j + m - 1) % m, d0);
        dirOf(j, d1);
        const float cross = d0.x * d1.y - d0.y * d1.x;
        const float dot = d0.x * d1.x + d0.y * d1.y;
        if (std::fabs(cross) < 1e-3f && dot > 0) continue;
        const float s = cross > 0 ? -1.0f : 1.0f;
        const Pt2 p = scratch2_[static_cast<size_t>(j)];
        const Pt2 n0{-d0.y * s, d0.x * s}, n1{-d1.y * s, d1.x * s};
        float mx = n0.x + n1.x, my = n0.y + n1.y;
        const float ml = std::sqrt(mx * mx + my * my);
        if (ml > 1e-3f) {
            mx /= ml;
            my /= ml;
        } else {
            mx = n0.x;
            my = n0.y;
        }
        const Pt2 a{p.x + n0.x * hw, p.y + n0.y * hw}, b{p.x + n1.x * hw, p.y + n1.y * hw};
        const Pt2 r{p.x + mx * hw, p.y + my * hw};
        emitTriangle(out, p, a, r, rgba);
        emitTriangle(out, p, r, b, rgba);
    }
}

void Renderer2D::strokePath(const Pt2* local, int n, bool closed, float widthPx, LineCap cap,
                            Color c, bool outline) {
    if (n < 1 || c.a <= 0.0f) return;
    scratch_.clear();
    for (int i = 0; i < n; ++i) scratch_.push_back(xf(local[i].x, local[i].y));
    std::uint8_t rgba[4];
    if (outline && outlineOn_) {
        outlineColor(c, rgba);
        emitStroke(outlineVerts_, scratch_.data(), n, closed, widthPx + 2.0f * outlineWidthPx_, cap,
                   rgba);
    }
    packColor(c, rgba);
    emitStroke(colorVerts_, scratch_.data(), n, closed, widthPx, cap, rgba);
}

void Renderer2D::fillPath(const Pt2* local, int n, Color c, FillMode mode) {
    if (n < 3 || c.a <= 0.0f) return;
    scratch_.clear();
    for (int i = 0; i < n; ++i) scratch_.push_back(xf(local[i].x, local[i].y));
    std::uint8_t rgba[4];
    if (mode == FillMode::Back) {
        packColor(c, rgba);
        fillScreenPolygon(backVerts_, scratch_.data(), n, rgba);
        return;
    }
    if (mode == FillMode::FrontOutlined && outlineOn_) {
        // Stroke centered on the edge: the inner half is covered by the fill drawn later.
        outlineColor(c, rgba);
        emitStroke(outlineVerts_, scratch_.data(), n, true, 2.0f * outlineWidthPx_, LineCap::Butt,
                   rgba);
    }
    packColor(c, rgba);
    fillScreenPolygon(colorVerts_, scratch_.data(), n, rgba);
}

// ---- primitives ----------------------------------------------------------------------------

void Renderer2D::line(float x0, float y0, float x1, float y1, Color c, LineCap cap) {
    const Pt2 p[2] = {{x0, y0}, {x1, y1}};
    strokePath(p, 2, false, lineWidthPx_, cap, c, true);
}

void Renderer2D::dashedLine(float x0, float y0, float x1, float y1, float dashPx, float gapPx,
                            Color c) {
    const float dx = x1 - x0, dy = y1 - y0;
    const float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-4f || dashPx <= 0) return;
    const float ux = dx / len, uy = dy / len;
    for (float t = 0; t < len; t += dashPx + gapPx) {
        const float e = std::min(t + dashPx, len);
        line(x0 + ux * t, y0 + uy * t, x0 + ux * e, y0 + uy * e, c);
    }
}

void Renderer2D::polyline(const Pt2* pts, int n, Color c, bool closed, LineCap cap) {
    strokePath(pts, n, closed, lineWidthPx_, cap, c, true);
}

void Renderer2D::arc(float cx, float cy, float r, float a0Rad, float a1Rad, Color c) {
    const float sweep = a1Rad - a0Rad;
    // Chord error below ~0.2 px.
    const float step = 2.0f * std::acos(std::max(0.0f, 1.0f - 0.2f / std::max(r, 0.5f)));
    int segs = static_cast<int>(std::ceil(std::fabs(sweep) / std::max(step, 0.05f)));
    segs = std::clamp(segs, 2, 128);
    local_.clear();
    for (int i = 0; i <= segs; ++i) {
        const float a = a0Rad + sweep * static_cast<float>(i) / static_cast<float>(segs);
        local_.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    const bool full = std::fabs(sweep) >= 2.0f * kPi - 1e-4f;
    if (full) local_.pop_back();
    strokePath(local_.data(), static_cast<int>(local_.size()), full, lineWidthPx_, LineCap::Butt, c,
               true);
}

void Renderer2D::circle(float cx, float cy, float r, Color c) {
    arc(cx, cy, r, 0.0f, 2.0f * kPi, c);
}

void Renderer2D::fillCircle(float cx, float cy, float r, Color c, FillMode mode) {
    const float step = 2.0f * std::acos(std::max(0.0f, 1.0f - 0.2f / std::max(r, 0.5f)));
    int segs = static_cast<int>(std::ceil(2.0f * kPi / std::max(step, 0.05f)));
    segs = std::clamp(segs, 8, 128);
    local_.clear();
    for (int i = 0; i < segs; ++i) {
        const float a = 2.0f * kPi * static_cast<float>(i) / static_cast<float>(segs);
        local_.push_back({cx + r * std::cos(a), cy + r * std::sin(a)});
    }
    fillPath(local_.data(), segs, c, mode);
}

void Renderer2D::fillRect(float x, float y, float w, float h, Color c, FillMode mode) {
    const Pt2 p[4] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}};
    fillPath(p, 4, c, mode);
}

void Renderer2D::strokeRect(float x, float y, float w, float h, Color c) {
    const Pt2 p[4] = {{x, y}, {x + w, y}, {x + w, y + h}, {x, y + h}};
    strokePath(p, 4, true, lineWidthPx_, LineCap::Butt, c, true);
}

void Renderer2D::fillTriangle(Pt2 a, Pt2 b, Pt2 c, Color col, FillMode mode) {
    const Pt2 p[3] = {a, b, c};
    fillPath(p, 3, col, mode);
}

void Renderer2D::strokeTriangle(Pt2 a, Pt2 b, Pt2 c, Color col) {
    const Pt2 p[3] = {a, b, c};
    strokePath(p, 3, true, lineWidthPx_, LineCap::Butt, col, true);
}

void Renderer2D::fillPolygon(const Pt2* pts, int n, Color c, FillMode mode) {
    fillPath(pts, n, c, mode);
}

// ---- text ----------------------------------------------------------------------------------

void Renderer2D::drawGlyphStrokes(const Glyph& g, float originX, float capTopY, float unit, Color c,
                                  float widthPx) {
    Pt2 pts[32];
    for (int s = 0; s < g.strokeCount; ++s) {
        const GlyphStroke& st = g.strokes[s];
        const int n = std::min(st.count, 32);
        for (int i = 0; i < n; ++i)
            pts[i] = {originX + st.points[i].x * unit, capTopY + st.points[i].y * unit};
        if (st.filled)
            fillPath(pts, n, c, FillMode::FrontOutlined);
        else
            strokePath(pts, n, st.closed, widthPx, LineCap::Square, c, true);
    }
}

void Renderer2D::text(float x, float y, const char* utf8, TextSize size, Color c, HAlign h,
                      VAlign v) {
    if (!utf8 || c.a <= 0.0f) return;
    const float unit = StrokeFont::unitPx(size);
    const float adv = StrokeFont::advancePx(size);
    const float pad = (adv - kGlyphGridWidth * unit) * 0.5f;  // center the glyph in its cell
    const float top = StrokeFont::capTopY(y, size, v);
    float cellX = StrokeFont::originX(x, utf8, size, h);
    const float width = textWidthPx_ > 0 ? textWidthPx_ : StrokeFont::strokeWidthPx(size);
    const char* p = utf8;
    std::uint32_t cells[3];
    while (const int n = StrokeFont::nextCells(p, cells)) {
        for (int i = 0; i < n; ++i) {
            drawGlyphStrokes(StrokeFont::glyph(cells[i]), cellX + pad, top, unit, c, width);
            cellX += adv;
        }
    }
}

}  // namespace fsim
