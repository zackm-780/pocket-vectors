#include "hud/NavHud.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim {

namespace {

constexpr float kTapeMarginPx = 16.0f;  // same as H2's tapes
constexpr float kTapeWidthPx = 80.0f;
constexpr float kScaleDotPx = 18.0f;  // spacing of the LOC / GS dots
constexpr float kNavBoxW = 176.0f, kNavBoxH = 60.0f, kNavBoxY = 368.0f;

void diamond(Renderer2D& r, float cx, float cy, float hw, float hh, Color c, bool filled) {
    const Pt2 p[4] = {{cx, cy - hh}, {cx + hw, cy}, {cx, cy + hh}, {cx - hw, cy}};
    if (filled)
        r.fillPolygon(p, 4, c, FillMode::FrontOutlined);
    else
        r.polyline(p, 4, c, true);
}

void changeBox(Renderer2D& r, float x, float baselineY, const char* text, TextSize size, Color c) {
    const float w = StrokeFont::measure(text, size);
    r.setLineWidth(1.5f);
    r.strokeRect(x - 3, baselineY - StrokeFont::capHeightPx(size) - 3, w + 5,
                 StrokeFont::capHeightPx(size) + 7, c);
}

// Screen px per degree of pitch at the boresight, from the conformal projection.
float pitchPxPerDeg(const HudProjector& proj, const AircraftState& s) {
    float x0, y0, x1, y1;
    const double az = s.headingTrueRad;
    const double e = 5.0 * kPi / 180.0;
    if (proj.projectAzEl(az, 0, x0, y0) && proj.projectAzEl(az, e, x1, y1)) {
        const float d = std::hypot(x1 - x0, y1 - y0) / 5.0f;
        if (d > 1.0f && d < 30.0f) return d;
    }
    return 6.0f;
}

}  // namespace

void NavHud::reset() {
    fma_.reset();
    minimums_.reset();
}

void NavHud::draw(const HudFrame& f) {
    fma_.update(ap_, f.nowS);
    minimums_.update(nav_, static_cast<float>(f.state.altMslFt + f.state.altBaroOffsetFt),
                     f.nowS);  // X1: indicated;

    if (f.visibility.fma) drawFma(f);
    if (f.visibility.headingTape) drawHeadingMarks(f);
    drawBugs(f);
    if (f.visibility.navBoxLines > 0) {
        drawScales(f);
        drawNavBox(f);
    }
    drawFdCue(f);
    if (minimums_.calloutVisible(f.nowS)) {
        f.r.setLineWidth(2.0f);
        f.r.text(f.r.centerX(), 276, "MINIMUMS", TextSize::L, colors::kAmber, HAlign::Center,
                 VAlign::Baseline);
    }
    f.r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// FMA row
// ---------------------------------------------------------------------------------------------

void NavHud::drawFma(const HudFrame& f) {
    Renderer2D& r = f.r;
    const HudRect& z = hudzones::kFma;
    FmaText t;
    buildFma(ap_, t);
    const Color base = f.color;
    const Color armed = colors::kWhite;
    const float y = 16.0f;
    r.fillRect(r.leftX(), z.y, r.virtualWidth(), z.h, colors::kBlack.withAlpha(0.3f),
               FillMode::Back);
    r.setLineWidth(1.5f);

    const float xSpd = r.centered(6), xLat = r.centered(96), xVert = r.centered(178),
                xStat = r.centered(292);
    if (t.spd[0]) {
        r.text(xSpd, y, t.spd, TextSize::M, base);
        if (fma_.boxVisible(FmaColumn::Spd, f.nowS))
            changeBox(r, xSpd, y, t.spd, TextSize::M, base);
    }
    if (t.lat[0]) {
        r.text(xLat, y, t.lat, TextSize::M, base);
        if (fma_.boxVisible(FmaColumn::Lat, f.nowS))
            changeBox(r, xLat, y, t.lat, TextSize::M, base);
    }
    if (t.latArmed[0]) r.text(xLat + 31, y, t.latArmed, TextSize::S, armed);
    if (t.vert[0]) {
        r.text(xVert, y, t.vert, TextSize::M, base);
        if (fma_.boxVisible(FmaColumn::Vert, f.nowS))
            changeBox(r, xVert, y, t.vert, TextSize::M, base);
    }
    if (t.vertArmed[0])
        r.text(xVert + StrokeFont::measure(t.vert, TextSize::M) + 6, y, t.vertArmed, TextSize::S,
               armed);
    if (t.status[0]) r.text(xStat, y, t.status, TextSize::M, base);

    // Right side: bearing pointer + source, waypoint, distance, ETE.
    char sum[48];
    buildFmaNavSummary(nav_, sum, sizeof sum);
    if (sum[0]) {
        const float xr = r.rightX() - 4;
        r.text(xr, y - 1, sum, TextSize::S, base, HAlign::Right);
        r.text(xr - StrokeFont::measure(sum, TextSize::S) - 2, y - 1, FSIM_SYM_DIAMOND, TextSize::S,
               base, HAlign::Right);
    }
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Nav box
// ---------------------------------------------------------------------------------------------

void NavHud::drawNavBox(const HudFrame& f) {
    Renderer2D& r = f.r;
    NavBoxText t;
    buildNavBox(nav_, f.visibility.navBoxLines, f.options.magVarDeg, t);
    if (t.lines == 0) return;
    // Right edge sits left of H2's VS / RA readouts under the altitude tape.
    const float x = r.rightX() - kTapeMarginPx - kTapeWidthPx - 6 - kNavBoxW;
    r.fillRect(x, kNavBoxY, kNavBoxW, kNavBoxH, colors::kBlack.withAlpha(0.35f), FillMode::Back);
    r.setLineWidth(1.5f);
    for (int i = 0; i < t.lines; ++i)
        r.text(x + 4, kNavBoxY + 15 + 17.0f * i, t.line[i], TextSize::S, f.color);
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// LOC / CDI and GS / VNAV scales
// ---------------------------------------------------------------------------------------------

void NavHud::drawScales(const HudFrame& f) {
    Renderer2D& r = f.r;
    const ScaleState sc = buildScales(nav_, ap_);
    const Color c = f.color;
    r.setLineWidth(1.5f);
    if (sc.locVisible) {
        const float cx = r.centerX(), y = hudzones::kLocScaleY;
        for (int i = -2; i <= 2; ++i) {
            if (i == 0)
                r.line(cx, y - 7, cx, y + 7, c);
            else
                r.circle(cx + i * kScaleDotPx * 1.0f, y, 2.5f, c);
        }
        char label[16];
        const CdiScale cs = cdiScaleFor(nav_);
        if (cs != CdiScale::None)
            std::snprintf(label, sizeof label, "%s %s", navSourceName(nav_.source),
                          cdiScaleName(cs));
        else
            std::snprintf(label, sizeof label, "%s", navSourceName(nav_.source));
        r.text(cx - 2 * kScaleDotPx - 14, y + 3.5f, label, TextSize::S, c, HAlign::Right);
        const float dx = sc.loc.dots * kScaleDotPx;
        r.setLineWidth(2.0f);
        diamond(r, cx + dx, y, 7, 7, c, sc.locFilled);
        if (sc.loc.pegged)
            r.line(cx + dx + (dx > 0 ? 10 : -10), y - 6, cx + dx + (dx > 0 ? 10 : -10), y + 6, c);
        r.setLineWidth(1.5f);
    }
    if (sc.gsVisible) {
        const float x = r.centered(hudzones::kGsScaleX), cy = hudzones::kCenterY;
        for (int i = -2; i <= 2; ++i) {
            if (i == 0)
                r.line(x - 7, cy, x + 7, cy, c);
            else
                r.circle(x, cy - i * kScaleDotPx, 2.5f, c);
        }
        r.text(x, cy - 2 * kScaleDotPx - 12, sc.gsIsVnav ? "VNAV" : "G/S", TextSize::S, c,
               HAlign::Center);
        const float dy = -sc.gs.dots * kScaleDotPx;  // + = path above the aircraft = up
        r.setLineWidth(2.0f);
        diamond(r, x, cy + dy, 7, 7, c, sc.gsFilled);
        if (sc.gs.pegged)
            r.line(x - 6, cy + dy + (dy > 0 ? 10 : -10), x + 6, cy + dy + (dy > 0 ? 10 : -10), c);
        r.setLineWidth(1.5f);
    }
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Heading tape marks
// ---------------------------------------------------------------------------------------------

void NavHud::drawHeadingMarks(const HudFrame& f) {
    Renderer2D& r = f.r;
    const HudRect& z = hudzones::kHeadingTape;
    const float cx = r.centerX(), baseY = z.y + z.h;
    const float centerMag = f.headingMagDeg;
    const float magVar = f.options.magVarDeg;
    const Color c = f.color;
    r.setLineWidth(1.5f);

    auto numberOutside = [&](const TapeMark& m, float row) {
        char b[8];
        std::snprintf(b, sizeof b, "%03d", m.valueMagDeg);
        if (m.dxPx < 0)
            r.text(cx - kTapeHalfWidthPx - 4, row, b, TextSize::S, c, HAlign::Right);
        else
            r.text(cx + kTapeHalfWidthPx + 4, row, b, TextSize::S, c, HAlign::Left);
    };

    // Heading bug (AP target): triangle down at the top and up at the baseline.
    if (ap_.apEngaged || ap_.fdOn || ap_.latActive == LatMode::Hdg) {
        const TapeMark m = headingMark(ap_.hdgBugMagDeg, centerMag);
        const float x = cx + m.dxPx;
        r.fillTriangle({x - 5, z.y}, {x + 5, z.y}, {x, z.y + 7}, c, FillMode::FrontOutlined);
        r.fillTriangle({x - 5, baseY}, {x + 5, baseY}, {x, baseY - 7}, c, FillMode::FrontOutlined);
        if (m.offScale) numberOutside(m, z.y + 9);
    }
    // DTK / CRS hollow bar.
    if (nav_.source != NavSource::None && (nav_.activeWptIdent[0] || nav_.locValid)) {
        const TapeMark m = headingMark(trueToMagDeg(nav_.desiredTrackTrueDeg, magVar), centerMag);
        const float x = cx + m.dxPx;
        r.strokeRect(x - 3, z.y + 9, 6, 20, c);
        if (m.offScale) numberOutside(m, z.y + 22);
    }
    // Bearing to the active waypoint.
    if ((nav_.source == NavSource::Gps || nav_.source == NavSource::Vor) &&
        nav_.activeWptIdent[0]) {
        const TapeMark m = headingMark(trueToMagDeg(nav_.bearingToWptTrueDeg, magVar), centerMag);
        const float x = cx + m.dxPx;
        diamond(r, x, baseY + 12, 5, 5, c, true);
        if (m.offScale) numberOutside(m, baseY + 15);
    }
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Speed, altitude and minimums bugs
// ---------------------------------------------------------------------------------------------

void NavHud::drawBugs(const HudFrame& f) {
    Renderer2D& r = f.r;
    const Color c = f.color;
    const bool apOn = ap_.apEngaged || ap_.fdOn;
    r.setLineWidth(1.5f);

    // Speed bug: pentagon on the inner (right) edge of the speed tape, pointing into it.
    if (ap_.spdBugKt > 0.0f && (apOn || ap_.atEngaged)) {
        const float x1 = r.leftX() + kTapeMarginPx + kTapeWidthPx;
        const BugPos b = tapeBug(kSpeedTapeGeom, ap_.spdBugKt, std::max(0.0f, f.state.iasKt));
        const Pt2 p[5] = {{x1 + 8, b.y - 6},
                          {x1 + 8, b.y + 6},
                          {x1 - 1, b.y + 6},
                          {x1 - 6, b.y},
                          {x1 - 1, b.y - 6}};
        r.fillPolygon(p, 5, c, FillMode::FrontOutlined);
        char t[8];
        std::snprintf(t, sizeof t, "%03d", static_cast<int>(std::lround(ap_.spdBugKt)));
        if (b.offScale)
            r.text(x1 - 4, b.above ? b.y + 14 : b.y - 5, t, TextSize::S, c, HAlign::Right);
        if (f.visibility.fma)
            r.text(r.leftX() + kTapeMarginPx + kTapeWidthPx * 0.5f, 66, t, TextSize::S, c,
                   HAlign::Center);
    }
    // Altitude bug and minimums bug on the left edge of the altitude tape.
    const float x0 = r.rightX() - kTapeMarginPx - kTapeWidthPx;
    const float altFt = static_cast<float>(f.state.altMslFt + f.state.altBaroOffsetFt);  // X1
    if (ap_.altBugFt != 0.0f && (apOn || ap_.latActive != LatMode::Off)) {
        const BugPos b = tapeBug(kAltTapeGeom, ap_.altBugFt, altFt);
        const Pt2 p[5] = {{x0 - 8, b.y - 6},
                          {x0 - 8, b.y + 6},
                          {x0 + 1, b.y + 6},
                          {x0 + 6, b.y},
                          {x0 + 1, b.y - 6}};
        r.fillPolygon(p, 5, c, FillMode::FrontOutlined);
        char t[12];
        std::snprintf(t, sizeof t, "%d", static_cast<int>(std::lround(ap_.altBugFt)));
        if (b.offScale)
            r.text(x0 + 10, b.above ? b.y + 14 : b.y - 5, t, TextSize::S, c, HAlign::Left);
        if (f.visibility.fma)
            r.text(x0 + kTapeWidthPx * 0.5f, 66, t, TextSize::S, c, HAlign::Center);
    }
    if (nav_.minimumsFt > 0.0f) {
        const BugPos b = tapeBug(kAltTapeGeom, nav_.minimumsFt, altFt);
        r.setLineWidth(2.0f);
        r.line(x0, b.y, x0 + 18, b.y, colors::kAmber);
        r.line(x0 + 18, b.y - 4, x0 + 18, b.y + 4, colors::kAmber);
        if (b.offScale)
            r.text(x0 + 22, b.above ? b.y + 12 : b.y - 3, "MIN", TextSize::S, colors::kAmber);
    }
    r.setLineWidth(2.0f);
}

// ---------------------------------------------------------------------------------------------
// Flight director cue
// ---------------------------------------------------------------------------------------------

void NavHud::drawFdCue(const HudFrame& f) {
    if (!(ap_.fdOn && ap_.fdValid)) return;
    Renderer2D& r = f.r;
    const HudRect& lz = hudzones::kLadderClip;
    const HudFpv fpv =
        computeFpv(f.proj, f.state, r.centered(lz.x) + 4, lz.y + 4, lz.w - 8, lz.h - 8);
    float fx = r.centerX(), fy = hudzones::kCenterY;
    if (fpv.valid) {
        fx = fpv.xHud;
        fy = fpv.yHud;
    }
    const float pitchDeg = f.state.pitchRad * 180.0f / kPi,
                rollDeg = f.state.rollRad * 180.0f / kPi;
    const FdOffset o = fdCueOffset(ap_.fdPitchCmdDeg, pitchDeg, ap_.fdRollCmdDeg, rollDeg,
                                   pitchPxPerDeg(f.proj, f.state));
    const float x = fx + o.dx, y = fy + o.dy;
    r.setLineWidth(2.0f);
    diamond(r, x, y, 9, 9, f.color, false);
    r.fillCircle(x, y, 1.8f, f.color, FillMode::Front);
}

}  // namespace fsim
