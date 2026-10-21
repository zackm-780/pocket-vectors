#include "hud/MiniNav.h"

#include <cmath>
#include <cstdio>

#include "core/Units.h"
#include "ui/MapPageLogic.h"

namespace fsim {
namespace {

constexpr Color kFill{0.02f, 0.05f, 0.03f, 0.62f};
constexpr Color kFrame{0.55f, 0.9f, 0.6f, 0.9f};
constexpr Color kMagenta{1.0f, 0.25f, 1.0f};
constexpr Color kLeg{1.0f, 1.0f, 1.0f, 0.92f};
constexpr Color kPast{0.8f, 0.8f, 0.8f, 0.4f};
constexpr Color kRing{0.7f, 0.9f, 0.75f, 0.4f};
constexpr Color kTrack{0.30f, 1.0f, 0.40f};
constexpr Color kOwn{0.95f, 0.95f, 0.30f};
constexpr Color kText{0.90f, 1.0f, 0.92f};

Color legColor(MapLegKind kind) {
    if (kind == MapLegKind::Active) return kMagenta;
    if (kind == MapLegKind::Past) return kPast;
    return kLeg;
}

}  // namespace

void MiniNav::draw(Renderer2D& r, const AircraftState& ac, const NavState& nav,
                   const FlightPlan* plan, float fixedRangeNm) const {
    const MiniNavBox box = kMiniNavBox;
    const MiniNavReadout info =
        miniNavReadout(ac.latDeg, ac.lonDeg, ac.gsKt, &nav, plan, fixedRangeNm);
    r.pushClip(box.x, box.y, box.w, box.h);
    r.fillRect(box.x, box.y, box.w, box.h, kFill, FillMode::Back);
    r.setLineWidth(1.5f);
    r.strokeRect(box.x + 0.5f, box.y + 0.5f, box.w - 1.0f, box.h - 1.0f, kFrame);

    const float mapTop = box.y + 2.0f;
    const float mapH = box.h - 30.0f;
    r.pushClip(box.x + 2.0f, mapTop, box.w - 4.0f, mapH);
    const MapProjection proj =
        miniNavProjection(ac.latDeg, ac.lonDeg, ac.trackTrueRad, info.rangeNm, box);
    r.setLineWidth(1.0f);
    r.circle(proj.cx, proj.cy, proj.ringRadiusPx, kRing);

    if (plan) {
        const int n = static_cast<int>(plan->legs.size());
        for (int i = 1; i < n; ++i) {
            const Leg& leg = plan->legs[static_cast<size_t>(i)];
            const Leg& prev = plan->legs[static_cast<size_t>(i - 1)];
            const MapLegStyle st =
                mapLegStyle(i, plan->activeLeg, plan->approachStartLeg, leg.missedApproach);
            const Color col = legColor(st.kind);
            float x0, y0, x1, y1;
            proj.toScreen(prev.to.latDeg, prev.to.lonDeg, x0, y0);
            proj.toScreen(leg.to.latDeg, leg.to.lonDeg, x1, y1);
            r.setLineWidth(st.kind == MapLegKind::Active ? 2.5f : 1.5f);
            if (leg.type == LegType::RF && leg.arcRadiusNm > 0.0f) {
                float ce, cn, pe, pn, te, tn;
                proj.offsetsNm(leg.arcCenterLatDeg, leg.arcCenterLonDeg, ce, cn);
                proj.offsetsNm(prev.to.latDeg, prev.to.lonDeg, pe, pn);
                proj.offsetsNm(leg.to.latDeg, leg.to.lonDeg, te, tn);
                const float b0 = std::atan2(pe - ce, pn - cn);
                const float b1 = std::atan2(te - ce, tn - cn);
                float d = b1 - b0;
                if (leg.arcRight) {
                    while (d < 0) d += static_cast<float>(fsim::kTwoPi);
                } else {
                    while (d > 0) d -= static_cast<float>(fsim::kTwoPi);
                }
                const float rad = std::hypot(pe - ce, pn - cn);
                Pt2 pts[17];
                constexpr int kSteps = 16;
                for (int k = 0; k <= kSteps; ++k) {
                    const float b = b0 + d * static_cast<float>(k) / kSteps;
                    proj.offsetToScreen(ce + rad * std::sin(b), cn + rad * std::cos(b), pts[k].x,
                                        pts[k].y);
                }
                if (st.dashed) {
                    for (int k = 0; k < kSteps; k += 2)
                        r.line(pts[k].x, pts[k].y, pts[k + 1].x, pts[k + 1].y, col);
                } else {
                    r.polyline(pts, kSteps + 1, col);
                }
            } else if (st.dashed) {
                r.dashedLine(x0, y0, x1, y1, 5, 4, col);
            } else {
                r.line(x0, y0, x1, y1, col);
            }
        }
    }

    // Ground track. A one-minute vector is only a few pixels at the 20 NM cap, so keep it
    // long enough to read, and never past the range ring.
    if (ac.gsKt > 5.0f) {
        const float minutePx = (ac.gsKt / 60.0f) * proj.pxPerNm();
        const float lenPx = std::min(proj.ringRadiusPx * 0.85f, std::max(minutePx, 28.0f));
        r.setLineWidth(1.5f);
        r.line(proj.cx, proj.cy, proj.cx, proj.cy - lenPx, kTrack);
    }

    r.push();
    r.translate(proj.cx, proj.cy);
    r.rotate(ac.headingTrueRad - ac.trackTrueRad);
    r.setLineWidth(2.0f);
    r.line(0, -7, 0, 6, kOwn);
    r.line(-6, 1, 6, 1, kOwn);
    r.line(-2.5f, 5, 2.5f, 5, kOwn);
    r.pop();
    r.popClip();

    char line1[32];
    char line2[48];
    if (!info.hasPlan && info.ident[0] == '\0') {
        std::snprintf(line1, sizeof line1, "NO PLAN");
        std::snprintf(line2, sizeof line2, "RNG %.0f", info.rangeNm);
    } else {
        char ete[16];
        mapFormatEte(info.eteS, ete, sizeof ete);
        const char* side = info.crossTrackNm >= 0 ? "R" : "L";
        std::snprintf(line1, sizeof line1, "%s %.1f NM", info.ident[0] ? info.ident : "WPT",
                      info.distNm);
        std::snprintf(line2, sizeof line2, "%s  %.1f%s  %.0fNM", ete, std::fabs(info.crossTrackNm),
                      side, info.rangeNm);
    }
    const float textY = box.y + box.h - 22.0f;
    r.setLineWidth(1.25f);
    r.text(box.x + 6, textY, line1, TextSize::S, kText);
    r.text(box.x + 6, textY + 11, line2, TextSize::S, kText);
    r.popClip();
}

}  // namespace fsim
