#include "hud/Hud.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/Units.h"

namespace fsim {
namespace {

constexpr float kCenterY = hudzones::kCenterY;
constexpr float kTapeMarginPx = 16.0f;  // tape outer edge distance from the screen edge
constexpr float kTapeWidthPx = 80.0f;
constexpr float kBankArcRadiusPx = 150.0f;
constexpr float kRowPanelTopPx = 430.0f;
constexpr float kRow1BaselinePx = 447.0f;
constexpr float kRow2BaselinePx = 469.0f;
constexpr float kWarningBaselinePx = 300.0f;
constexpr float kMessageBaselinePx = 348.0f;
constexpr float kMessageStepPx = 13.0f;
constexpr double kBlinkHz = 2.0;
constexpr float kPi_f = static_cast<float>(kPi);

struct Ctx {
    Renderer2D& r;
    const gfx::Camera& cam;
    const HudProjector& proj;
    const AircraftState& s;
    const ControlInputs& c;
    const AircraftProfile& p;
    const HudOptions& opt;
    const HudVisibility& vis;
    Color base;
    Color dim;
    float hdgMagDeg;
    double nowS;
};

Color roleColor(const Ctx& k, Role role) {
    switch (role) {
    case Role::Normal: return k.base;
    case Role::White: return colors::kWhite;
    case Role::Amber: return colors::kAmber;
    case Role::Red: return colors::kRed;
    case Role::Dim: return k.base.withAlpha(0.55f);
    }
    return k.base;
}

Color severityColor(Severity sev) {
    switch (sev) {
    case Severity::Red: return colors::kRed;
    case Severity::Amber: return colors::kAmber;
    case Severity::White: break;
    }
    return colors::kWhite;
}

bool blinkOn(double nowS) {
    const double phase = nowS * kBlinkHz;
    return phase - std::floor(phase) < 0.5;
}

Pt2 norm(Pt2 v) {
    const float len = std::hypot(v.x, v.y);
    return len > 1e-6f ? Pt2{v.x / len, v.y / len} : Pt2{1, 0};
}

// ---------------------------------------------------------------------------------------------
// Attitude: horizon, pitch ladder, bank scale, boresight, FPV, large radio altitude
// ---------------------------------------------------------------------------------------------

// Screen direction of the horizon (unit vector pointing right) and a point on it. False when the
// horizon cannot be projected (looking straight up or down).
bool projectHorizon(const Ctx& k, Pt2& pointOnHorizon, Pt2& dirRight) {
    static constexpr double kOffsetsDeg[] = {45.0, 20.0, 8.0};
    const double psi = k.s.headingTrueRad;
    for (double offDeg : kOffsetsDeg) {
        const double off = degToRad(offDeg);
        Pt2 a, b;
        if (k.proj.projectAzEl(psi - off, 0, a.x, a.y) &&
            k.proj.projectAzEl(psi + off, 0, b.x, b.y)) {
            pointOnHorizon = a;
            dirRight = norm({b.x - a.x, b.y - a.y});
            return true;
        }
    }
    return false;
}

void drawHorizon(const Ctx& k, const Pt2& pt, const Pt2& dir) {
    Renderer2D& r = k.r;
    r.pushClip(r.centered(100), 64, 440, 352);
    constexpr float kLen = 1200.0f;
    r.setLineWidth(2.0f);
    r.line(pt.x - dir.x * kLen, pt.y - dir.y * kLen, pt.x + dir.x * kLen, pt.y + dir.y * kLen,
           k.base);
    r.popClip();
}

void drawLadder(const Ctx& k, float horizonAngleRad) {
    Renderer2D& r = k.r;
    const double psi = k.s.headingTrueRad;
    const double pitchDeg = radToDeg(k.s.pitchRad);
    const HudRect& z = hudzones::kLadderClip;
    r.pushClip(r.centered(z.x), z.y, z.w, z.h);
    r.setLineWidth(1.5f);
    const int firstDeg = static_cast<int>(std::floor((pitchDeg - 40.0) / 5.0)) * 5;
    for (int p = firstDeg; p <= pitchDeg + 40.0; p += 5) {
        if (p == 0 || p <= -85 || p >= 85) continue;
        const double el = degToRad(p);
        Pt2 c;
        if (!k.proj.projectAzEl(psi, el, c.x, c.y)) continue;
        if (c.x < z.x - 150 || c.x > r.virtualWidth() + 150 || c.y < z.y - 150 ||
            c.y > z.y + z.h + 150)
            continue;
        // Tangent along the line (horizon direction at this elevation) and the "up" direction.
        const double daz = degToRad(2.0) / std::max(0.2, std::cos(el));
        Pt2 l, rr, dn, up;
        if (!k.proj.projectAzEl(psi - daz, el, l.x, l.y) ||
            !k.proj.projectAzEl(psi + daz, el, rr.x, rr.y) ||
            !k.proj.projectAzEl(psi, el - degToRad(1.0), dn.x, dn.y) ||
            !k.proj.projectAzEl(psi, el + degToRad(1.0), up.x, up.y))
            continue;
        const Pt2 d = norm({rr.x - l.x, rr.y - l.y});
        const Pt2 u = norm({up.x - dn.x, up.y - dn.y});
        const bool major = p % 10 == 0;
        const float gap = 16.0f, len = major ? 44.0f : 22.0f;
        const float tickDir = p > 0 ? -1.0f : 1.0f;  // end ticks point toward the horizon
        for (int side = -1; side <= 1; side += 2) {
            const float s0 = side * gap, s1 = side * (gap + len);
            const Pt2 a{c.x + d.x * s0, c.y + d.y * s0}, b{c.x + d.x * s1, c.y + d.y * s1};
            if (p > 0)
                r.line(a.x, a.y, b.x, b.y, k.base);
            else
                r.dashedLine(a.x, a.y, b.x, b.y, 6.0f, 4.0f, k.base);
            r.line(b.x, b.y, b.x + u.x * tickDir * 8.0f, b.y + u.y * tickDir * 8.0f, k.base);
            if (major && ladderLabelVisible(k.vis, p)) {
                char label[16];
                std::snprintf(label, sizeof label, "%d", std::abs(p));
                r.push();
                r.translate(b.x + d.x * side * 5.0f, b.y + d.y * side * 5.0f);
                r.rotate(horizonAngleRad);
                r.text(0, 0, label, TextSize::S, k.base, side < 0 ? HAlign::Right : HAlign::Left,
                       VAlign::Middle);
                r.pop();
            }
        }
    }
    r.popClip();
    r.setLineWidth(2.0f);
}

void drawBankScale(const Ctx& k) {
    Renderer2D& r = k.r;
    const float cx = r.centerX();
    constexpr float kHalfArc = 60.0f * kPi_f / 180.0f;
    r.setLineWidth(1.5f);
    r.arc(cx, kCenterY, kBankArcRadiusPx, -kPi_f * 0.5f - kHalfArc, -kPi_f * 0.5f + kHalfArc,
          k.base);
    static constexpr struct {
        int deg;
        float len;
    } kMarks[] = {{10, 5}, {20, 5}, {30, 9}, {45, 6}, {60, 9}};
    for (const auto& m : kMarks) {
        for (int side = -1; side <= 1; side += 2) {
            const float a = -kPi_f * 0.5f + side * static_cast<float>(degToRad(m.deg));
            r.line(cx + kBankArcRadiusPx * std::cos(a), kCenterY + kBankArcRadiusPx * std::sin(a),
                   cx + (kBankArcRadiusPx + m.len) * std::cos(a),
                   kCenterY + (kBankArcRadiusPx + m.len) * std::sin(a), k.base);
        }
    }
    // Fixed index above the arc (zero bank).
    r.fillTriangle({cx - 5, kCenterY - kBankArcRadiusPx - 10},
                   {cx + 5, kCenterY - kBankArcRadiusPx - 10},
                   {cx, kCenterY - kBankArcRadiusPx - 2}, k.base, FillMode::Front);
    // Sky pointer: up on the screen is rotated by -roll (the world turns counter-clockwise when
    // the aircraft banks right). Slip/skid bar rides under the pointer.
    r.push();
    r.translate(cx, kCenterY);
    r.rotate(-k.s.rollRad);
    r.fillTriangle({0, -(kBankArcRadiusPx - 2)}, {-6, -(kBankArcRadiusPx - 13)},
                   {6, -(kBankArcRadiusPx - 13)}, k.base, FillMode::FrontOutlined);
    const float slipPx = std::max(-14.0f, std::min(14.0f, k.s.betaDeg * 5.0f));
    r.fillRect(slipPx - 7, -(kBankArcRadiusPx - 16), 14, 4, k.base, FillMode::FrontOutlined);
    r.pop();
    r.setLineWidth(2.0f);
}

void drawBoresight(const Ctx& k) {
    Renderer2D& r = k.r;
    const float cx = r.centerX(), cy = kCenterY;
    r.setLineWidth(2.0f);
    r.line(cx - 34, cy, cx - 10, cy, k.base);
    r.line(cx + 10, cy, cx + 34, cy, k.base);
    const Pt2 w[5] = {{cx - 10, cy}, {cx - 5, cy + 7}, {cx, cy}, {cx + 5, cy + 7}, {cx + 10, cy}};
    r.polyline(w, 5, k.base);
}

void drawFpv(const Ctx& k, const HudFpv& fpv, float horizonAngleRad) {
    if (!fpv.valid) return;
    Renderer2D& r = k.r;
    const Color col = fpv.clamped ? k.base.withAlpha(0.45f) : k.base;
    r.push();
    r.translate(fpv.xHud, fpv.yHud);
    r.rotate(horizonAngleRad);
    r.setLineWidth(2.0f);
    r.circle(0, 0, 6.0f, col);
    r.line(-17, 0, -6, 0, col);
    r.line(6, 0, 17, 0, col);
    r.line(0, -6, 0, -13, col);
    r.pop();
}

void drawRadioAltLarge(const Ctx& k, const HudFpv& fpv) {
    if (k.s.wowAny || k.s.altAglFt >= 1000.0f) return;
    char buf[16];
    std::snprintf(buf, sizeof buf, "RA %d", roundRadioAltFt(k.s.altAglFt));
    // Under the FPV, below the warning line (y 300) and above the messages.
    const float y = std::max(320.0f, std::min(326.0f, (fpv.valid ? fpv.yHud : kCenterY) + 36.0f));
    k.r.text(k.r.centerX(), y, buf, TextSize::L, k.base, HAlign::Center, VAlign::Baseline);
}

// ---------------------------------------------------------------------------------------------
// Heading tape (magnetic)
// ---------------------------------------------------------------------------------------------

void drawHeadingTape(const Ctx& k) {
    Renderer2D& r = k.r;
    const HudRect& z = hudzones::kHeadingTape;
    const float x0 = r.centered(z.x), cx = r.centerX(), baseY = z.y + z.h;  // baseline y = 56
    const float hdg = k.hdgMagDeg;
    r.pushClip(x0, z.y - 4, z.w, z.h + 12);
    r.setLineWidth(1.5f);
    r.line(x0, baseY, x0 + z.w, baseY, k.base);
    const int first = static_cast<int>(std::floor((hdg - kHeadingTapeHalfRangeDeg) / 5.0f)) * 5;
    for (int v = first; v <= hdg + kHeadingTapeHalfRangeDeg + 0.01f; v += 5) {
        const float x = cx + (static_cast<float>(v) - hdg) * kHeadingTapePxPerDeg;
        const int wrapped = ((v % 360) + 360) % 360;
        const bool major = wrapped % 10 == 0;
        r.line(x, baseY, x, baseY - (major ? 8.0f : 4.0f), k.base);
        if (major && std::fabs(static_cast<float>(v) - hdg) > 6.0f && x > x0 + 9 &&
            x < x0 + z.w - 9) {
            char label[16];
            std::snprintf(label, sizeof label, "%02d", headingTapeLabel(wrapped));
            r.text(x, baseY - 11, label, TextSize::S, k.base, HAlign::Center, VAlign::Baseline);
        }
    }
    // Ground track tick below the baseline.
    if (k.s.gsKt > 5.0f) {
        const float tr = trackMagDeg(k.s.trackTrueRad, k.opt.magVarDeg);
        const float dx = angleDiffDeg(tr, hdg);
        if (std::fabs(dx) <= kHeadingTapeHalfRangeDeg) {
            const float x = cx + dx * kHeadingTapePxPerDeg;
            r.fillTriangle({x - 3.5f, baseY + 7}, {x + 3.5f, baseY + 7}, {x, baseY + 1}, k.base,
                           FillMode::FrontOutlined);
        }
    }
    r.popClip();
    // Current heading box (3 digits) with a pointer into the tape.
    char box[16];
    formatHeadingBox(box, sizeof box, hdg);
    r.fillRect(cx - 20, z.y - 2, 40, 18, colors::kBlack.withAlpha(0.7f), FillMode::Back);
    r.setLineWidth(1.5f);
    r.strokeRect(cx - 20, z.y - 2, 40, 18, k.base);
    r.text(cx, z.y + 12, box, TextSize::M, k.base, HAlign::Center, VAlign::Baseline);
    r.fillTriangle({cx - 5, z.y + 17}, {cx + 5, z.y + 17}, {cx, z.y + 24}, k.base,
                   FillMode::FrontOutlined);
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Speed tape
// ---------------------------------------------------------------------------------------------

void bandRect(const Ctx& k, float x, float w, float loKt, float hiKt, float centerKt, Color col) {
    if (hiKt <= loKt) return;
    const TapeGeom& g = kSpeedTapeGeom;
    const float yHi = std::max(g.topY, std::min(g.bottomY, g.yOf(hiKt, centerKt)));
    const float yLo = std::max(g.topY, std::min(g.bottomY, g.yOf(loKt, centerKt)));
    if (yLo - yHi < 0.5f) return;
    k.r.fillRect(x, yHi, w, yLo - yHi, col, FillMode::Front);
}

void drawSpeedTape(const Ctx& k, const SpeedTrend& trend) {
    Renderer2D& r = k.r;
    const TapeGeom& g = kSpeedTapeGeom;
    const float x0 = r.leftX() + kTapeMarginPx, x1 = x0 + kTapeWidthPx;
    const float ias = std::max(0.0f, k.s.iasKt);
    r.fillRect(x0, g.topY, kTapeWidthPx, g.bottomY - g.topY, colors::kBlack.withAlpha(0.35f),
               FillMode::Back);
    r.pushClip(x0 - 2, g.topY, kTapeWidthPx + 4, g.bottomY - g.topY);
    // Bands along the inner edge.
    const SpeedBands b = computeSpeedBands(k.p, k.s.flapDetent);
    const float bx = x1 - 5;
    bandRect(k, bx, 5, 0, b.redBelowKt, ias, colors::kRed);
    bandRect(k, bx, 5, b.whiteLowKt, b.whiteHighKt, ias, colors::kWhite);
    bandRect(k, bx, 5, b.yellowLowKt, b.yellowHighKt, ias, colors::kAmber);
    if (b.redAboveKt > 0) bandRect(k, bx, 5, b.redAboveKt, 2000.0f, ias, colors::kRed);
    // Ticks every 10 kt, labels every 20.
    r.setLineWidth(1.5f);
    const TickRange tr = tickRange(g, ias, 10);
    for (int v = std::max(0, tr.first); v <= tr.last; v += 10) {
        const float y = g.yOf(static_cast<float>(v), ias);
        if (std::fabs(y - g.centerY) < 16.0f) continue;  // under the value box
        const bool label = isLabelTick(v, 20);
        r.line(x1 - 7, y, x1 - (label ? 15.0f : 11.0f), y, k.base);
        if (label) {
            char t[16];
            std::snprintf(t, sizeof t, "%d", v);
            r.text(x1 - 19, y, t, TextSize::S, k.base, HAlign::Right, VAlign::Middle);
        }
    }
    r.popClip();
    // Current value box with a pointer.
    char v[16];
    std::snprintf(v, sizeof v, "%d", roundToStep(ias, 1));
    const float bw = 66, bh = 26, bxl = x0 + 2;
    r.fillRect(bxl, g.centerY - bh * 0.5f, bw, bh, colors::kBlack.withAlpha(0.8f), FillMode::Back);
    r.setLineWidth(1.5f);
    r.strokeRect(bxl, g.centerY - bh * 0.5f, bw, bh, k.base);
    r.text(bxl + bw - 6, g.centerY + 8, v, TextSize::L, k.base, HAlign::Right, VAlign::Baseline);
    r.fillTriangle({bxl + bw, g.centerY - 6}, {bxl + bw, g.centerY + 6}, {bxl + bw + 8, g.centerY},
                   k.base, FillMode::FrontOutlined);
    // 10 s trend arrow just inside the tape edge.
    const float trendKt = trend.trend10sKt();
    if (std::fabs(trendKt) >= 2.0f) {
        const float yEnd = std::max(g.topY + 4, std::min(g.bottomY - 4, g.yOf(ias + trendKt, ias)));
        const float ax = x1 + 11;
        const float dir = yEnd < g.centerY ? 1.0f : -1.0f;  // +1 = arrow pointing up
        r.line(ax, g.centerY, ax, yEnd, k.base);
        r.line(ax, yEnd, ax - 4, yEnd + dir * 6, k.base);
        r.line(ax, yEnd, ax + 4, yEnd + dir * 6, k.base);
    }
    // GS and Mach below the tape.
    if (k.vis.speedAltReadouts) {
        char t[16];
        std::snprintf(t, sizeof t, "GS %d", roundToStep(k.s.gsKt, 1));
        r.text(x0, 388, t, TextSize::M, k.base, HAlign::Left, VAlign::Baseline);
        if (k.s.mach > 0.40f) {
            formatMach(t, sizeof t, k.s.mach);
            r.text(x0, 404, t, TextSize::M, k.base, HAlign::Left, VAlign::Baseline);
        }
    }
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Altitude tape
// ---------------------------------------------------------------------------------------------

void drawAltTape(const Ctx& k) {
    Renderer2D& r = k.r;
    const TapeGeom& g = kAltTapeGeom;
    const float x1 = r.rightX() - kTapeMarginPx, x0 = x1 - kTapeWidthPx;
    const float alt = static_cast<float>(k.s.altMslFt + k.s.altBaroOffsetFt);  // X1: indicated
    r.fillRect(x0, g.topY, kTapeWidthPx, g.bottomY - g.topY, colors::kBlack.withAlpha(0.35f),
               FillMode::Back);
    r.pushClip(x0 - 2, g.topY, kTapeWidthPx + 4, g.bottomY - g.topY);
    // Ground band: terrain elevation shading when it is on the tape.
    {
        const float yg = g.yOf(k.s.terrainElevFt + k.s.altBaroOffsetFt, alt);
        if (yg < g.bottomY) {
            const float top = std::max(g.topY, yg);
            r.fillRect(x0, top, kTapeWidthPx, g.bottomY - top, k.base.withAlpha(0.22f),
                       FillMode::Back);
            if (yg >= g.topY && std::fabs(yg - g.centerY) > 16.0f) {
                r.setLineWidth(2.0f);
                r.line(x0, yg, x1, yg, k.base);
            }
        }
    }
    r.setLineWidth(1.5f);
    const TickRange tr = tickRange(g, alt, 100);
    for (int v = tr.first; v <= tr.last; v += 100) {
        const float y = g.yOf(static_cast<float>(v), alt);
        if (std::fabs(y - g.centerY) < 17.0f) continue;
        const bool label = isLabelTick(v, 500);
        r.line(x0 + 7, y, x0 + (label ? 15.0f : 11.0f), y, k.base);
        if (label) {
            char t[12];
            std::snprintf(t, sizeof t, "%d", v);
            r.text(x0 + 19, y, t, TextSize::S, k.base, HAlign::Left, VAlign::Middle);
        }
    }
    r.popClip();
    // Value box: thousands large, last three digits (20 ft steps) medium.
    char th[8], rest[8];
    formatAltBoxThousands(th, sizeof th, alt);
    formatAltBoxRest(rest, sizeof rest, alt);
    const float bw = 70, bh = 28, bxl = x1 - bw - 2;
    r.fillRect(bxl, g.centerY - bh * 0.5f, bw, bh, colors::kBlack.withAlpha(0.8f), FillMode::Back);
    r.setLineWidth(1.5f);
    r.strokeRect(bxl, g.centerY - bh * 0.5f, bw, bh, k.base);
    const float restRight = bxl + bw - 5;
    r.text(restRight, g.centerY + 6, rest, TextSize::M, k.base, HAlign::Right, VAlign::Baseline);
    r.text(restRight - StrokeFont::measure(rest, TextSize::M) - 1, g.centerY + 8, th, TextSize::L,
           k.base, HAlign::Right, VAlign::Baseline);
    r.fillTriangle({bxl, g.centerY - 6}, {bxl, g.centerY + 6}, {bxl - 8, g.centerY}, k.base,
                   FillMode::FrontOutlined);
    // VS and radio altitude below the tape.
    char t[16];
    formatVs(t, sizeof t, k.s.vsFpm);
    r.text(x0, 388, t, TextSize::M, k.base, HAlign::Left, VAlign::Baseline);
    if (k.s.altAglFt < 2500.0f && !k.s.wowAny) {
        std::snprintf(t, sizeof t, "RA %d", roundRadioAltFt(k.s.altAglFt));
        r.text(x0, 404, t, TextSize::M, k.base, HAlign::Left, VAlign::Baseline);
    }
    // X1: altimeter setting, "STD", "29.92" or "1013".
    formatAltimeterSetting(t, sizeof t, k.s.altimeterSettingHpa, k.opt.baroInHg);
    r.text(x0, 420, t, TextSize::M, k.base, HAlign::Left, VAlign::Baseline);
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Status rows
// ---------------------------------------------------------------------------------------------

float drawSeg(const Ctx& k, float x, float baseY, const StatusSeg& seg, TextSize size) {
    Renderer2D& r = k.r;
    const float adv = StrokeFont::advancePx(size), cap = StrokeFont::capHeightPx(size);
    const float width = segCells10(seg) * 0.1f * adv;
    const Color col = roleColor(k, seg.role);
    switch (seg.kind) {
    case StatusSeg::Kind::Text:
        if (!seg.blinks || blinkOn(k.nowS))
            r.text(x, baseY, seg.text, size, col, HAlign::Left, VAlign::Baseline);
        break;
    case StatusSeg::Kind::FlapBar: {
        const float bw = adv * 0.5f, bh = cap * 0.8f;
        r.setLineWidth(1.5f);
        for (int i = 0; i < seg.a; ++i) {
            const float bx = x + i * adv * 0.7f;
            if (i < seg.b)
                r.fillRect(bx, baseY - bh, bw, bh, col, FillMode::Front);
            else
                r.strokeRect(bx, baseY - bh, bw, bh, col);
        }
        r.setLineWidth(2.0f);
        break;
    }
    case StatusSeg::Kind::GearLights: {
        const float rad = cap * 0.26f;
        for (int i = 0; i < seg.a; ++i)
            r.fillCircle(x + adv * (0.4f + 0.8f * i), baseY - cap * 0.45f, rad, col,
                         FillMode::FrontOutlined);
        break;
    }
    case StatusSeg::Kind::TrimScale: {
        const float w = width - adv * 0.4f;
        const float yMid = baseY - cap * 0.4f;
        r.setLineWidth(1.5f);
        r.line(x, yMid, x + w, yMid, col);
        for (int i = 0; i < 5; ++i) {
            const float tx = x + w * i / 4.0f;
            r.line(tx, yMid - cap * 0.25f, tx, yMid + cap * 0.25f, col);
        }
        const float mx = x + w * (0.5f + seg.a / 200.0f);
        r.fillTriangle({mx - 3, baseY - cap - 1}, {mx + 3, baseY - cap - 1},
                       {mx, baseY - cap * 0.7f}, col, FillMode::FrontOutlined);
        r.setLineWidth(2.0f);
        break;
    }
    }
    return width;
}

float drawField(const Ctx& k, float x, float baseY, const StatusField& f, TextSize size) {
    const float adv = StrokeFont::advancePx(size);
    float cursor = x;
    for (int i = 0; i < f.segCount; ++i) {
        if (i > 0) cursor += adv;
        cursor += drawSeg(k, cursor, baseY, f.segs[i], size);
    }
    return cursor - x;
}

void drawStatusRows(const Ctx& k) {
    Renderer2D& r = k.r;
    if (!k.vis.statusRow1 && !k.vis.statusRow2) return;
    const StatusInputs in{&k.s, &k.c, &k.p, k.opt.magVarDeg};
    r.fillRect(r.leftX(), kRowPanelTopPx, r.virtualWidth(), 480 - kRowPanelTopPx,
               colors::kBlack.withAlpha(0.5f), FillMode::Back);
    r.setLineWidth(1.0f);
    r.line(r.leftX(), kRowPanelTopPx, r.rightX(), kRowPanelTopPx, k.base.withAlpha(0.6f));
    r.setLineWidth(2.0f);
    const float margin = 6.0f;
    const float avail = r.virtualWidth() - 2 * margin;

    if (k.vis.statusRow1) {
        StatusField f[kMaxStatusFields];
        const int n = buildStatusRow1(in, f);
        // Natural widths with 1.5 cell gaps; fall back to the small font when it does not fit.
        int cells10 = 0;
        for (int i = 0; i < n; ++i) cells10 += fieldCells10(f[i]) + (i > 0 ? 15 : 0);
        const TextSize size = cells10 * 0.1f * StrokeFont::advancePx(TextSize::M) <= avail
                                  ? TextSize::M
                                  : TextSize::S;
        const float adv = StrokeFont::advancePx(size);
        float x = r.leftX() + margin;
        for (int i = 0; i < n; ++i) {
            if (!f[i].present) continue;
            x += drawField(k, x, kRow1BaselinePx, f[i], size) + adv * 1.5f;
        }
    }
    if (k.vis.statusRow2) {
        StatusField f[kMaxStatusFields];
        const int n = buildStatusRow2(in, f);
        const TextSize size = TextSize::M;
        const float adv = StrokeFont::advancePx(size);
        float x = r.leftX() + margin;
        for (int i = 0; i < n; ++i) {
            const bool hidden = !k.vis.row2FuelAndTime && (i == kFuel || i == kTime);
            if (i == kTime) {
                if (f[i].present && !hidden) {
                    const float w = fieldCells10(f[i]) * 0.1f * adv;
                    drawField(k, r.rightX() - margin - w, kRow2BaselinePx, f[i], size);
                }
                continue;
            }
            if (f[i].present && !hidden) drawField(k, x, kRow2BaselinePx, f[i], size);
            x += (f[i].slotCells + 1.5f) * adv;
        }
    }
}

// ---------------------------------------------------------------------------------------------
// Warnings and messages
// ---------------------------------------------------------------------------------------------

void drawWarnings(const Ctx& k, const WarningTracker& tracker, std::uint32_t mask) {
    WarningId list[4];
    const int n = sortedWarnings(mask, list, 4);
    int shown = 0;
    for (int i = 0; i < n && shown < 2; ++i) {
        if (!tracker.visible(list[i], k.nowS)) {
            ++shown;  // keep the slot while blinking off so the lines do not jump
            continue;
        }
        char text[40];
        formatWarning(text, sizeof text, list[i], k.s);
        const Color col = severityColor(warningSeverity(list[i]));
        if (shown == 0)
            k.r.text(k.r.centerX(), kWarningBaselinePx, text, TextSize::L, col, HAlign::Center,
                     VAlign::Baseline);
        else
            k.r.text(k.r.centerX(), kWarningBaselinePx - 20, text, TextSize::M, col, HAlign::Center,
                     VAlign::Baseline);
        ++shown;
    }
}

void drawMessages(const Ctx& k, const MessageQueue& q) {
    int row = 0;
    for (int i = 0; i < q.size() && row < kMaxShownMessages; ++i) {
        const Message& m = q.newest(i);
        const float a = messageAlpha(q.ageS(m));
        if (a <= 0.0f) continue;
        k.r.text(k.r.centerX(), kMessageBaselinePx + row * kMessageStepPx, m.text, TextSize::M,
                 colors::kWhite.withAlpha(a), HAlign::Center, VAlign::Baseline);
        ++row;
    }
}

}  // namespace

bool Hud::applyActions(const ActionState& actions, HudOptions& options, MessageQueue* messages) {
    if (!actions.pressed[static_cast<int>(Action::HudDeclutter)]) return false;
    options.declutter = nextDeclutter(options.declutter);
    if (messages) messages->postf("HUD %s", declutterName(options.declutter));
    return true;
}

void Hud::reset() {
    trend_.reset();
    warnings_ = WarningTracker{};
}

void Hud::draw(Renderer2D& r, const gfx::Camera& camera, const AircraftState& s,
               const ControlInputs& c, const AircraftProfile& p, const MessageQueue& messages,
               const HudOptions& options) {
    if (!s.valid) return;
    const HudVisibility vis = visibilityFor(options.declutter);
    const HudProjector proj(camera, s.latDeg, s.lonDeg);
    const Color base = hudBaseColor(options.color);
    const double nowS = messages.nowS();
    const float hdgMag = headingMagDeg(s.headingTrueRad, options.magVarDeg);
    const Ctx k{r, camera, proj, s, c, p, options, vis, base, base.withAlpha(0.55f), hdgMag, nowS};

    trend_.update(s.iasKt, s.simTimeS);
    const std::uint32_t mask = activeWarnings(s, c, p, options);
    warnings_.update(mask, nowS);

    r.setLineWidth(2.0f);

    // Attitude (conformal).
    Pt2 hp, hd;
    const bool haveHorizon = projectHorizon(k, hp, hd);
    const float horizonAngle = haveHorizon ? std::atan2(hd.y, hd.x) : -s.rollRad;
    if (haveHorizon) drawHorizon(k, hp, hd);
    drawLadder(k, horizonAngle);
    drawBankScale(k);
    drawBoresight(k);
    const HudRect& lz = hudzones::kLadderClip;
    const HudFpv fpv = computeFpv(proj, s, r.centered(lz.x) + 4, lz.y + 4, lz.w - 8, lz.h - 8);
    drawFpv(k, fpv, horizonAngle);
    drawRadioAltLarge(k, fpv);

    if (vis.headingTape) drawHeadingTape(k);
    drawSpeedTape(k, trend_);
    drawAltTape(k);
    drawStatusRows(k);

    if (overlay_) {
        const HudFrame frame{r, camera, proj, s, c, p, options, vis, base, hdgMag, nowS};
        overlay_->draw(frame);
    }

    drawWarnings(k, warnings_, mask);
    drawMessages(k, messages);
    r.setLineWidth(2.0f);
}

}  // namespace fsim
