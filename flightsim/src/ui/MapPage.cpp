#include "ui/MapPage.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/Geo.h"
#include "core/Units.h"
#include "nav/FlightPlan.h"

namespace fsim {

namespace {

constexpr float kMapPi = 3.14159265f;
constexpr Color kBg{0.02f, 0.04f, 0.07f};
constexpr Color kRing{0.35f, 0.45f, 0.55f, 0.8f};
constexpr Color kText{0.85f, 0.92f, 0.95f};
constexpr Color kDim{0.55f, 0.62f, 0.68f};
constexpr Color kMagenta{1.0f, 0.25f, 1.0f};
constexpr Color kApproach{0.45f, 0.85f, 1.0f};
constexpr Color kAirport{0.55f, 0.75f, 1.0f};
constexpr Color kVor{0.30f, 0.85f, 0.95f};
constexpr Color kNdb{0.95f, 0.65f, 0.30f};
constexpr Color kSelect{1.0f, 0.85f, 0.2f};
constexpr Color kOwn{0.95f, 0.95f, 0.30f};
constexpr Color kTrack{0.25f, 1.0f, 0.35f};

Color legColor(MapLegKind k) {
    switch (k) {
    case MapLegKind::Active: return kMagenta;
    case MapLegKind::Past: return Color(0.6f, 0.6f, 0.6f, 0.5f);
    case MapLegKind::Approach: return kApproach;
    case MapLegKind::Missed: return Color(0.9f, 0.9f, 0.9f, 0.85f);
    default: return colors::kWhite;
    }
}

Pt2 rot(float x, float y, float a) {
    const float c = std::cos(a), s = std::sin(a);
    return {x * c - y * s, x * s + y * c};
}

}  // namespace

MapPageResult MapPage::update(const ActionState& a, const FlightPlan* plan) {
    MapPageResult res;
    auto pressed = [&](Action act) { return a.pressed[static_cast<int>(act)]; };
    if (pressed(Action::MenuUp)) zoomIndex_ = mapZoomStep(zoomIndex_, -1);
    if (pressed(Action::MenuDown)) zoomIndex_ = mapZoomStep(zoomIndex_, +1);
    if (pressed(Action::HudDeclutter))
        orientation_ = orientation_ == MapOrientation::NorthUp ? MapOrientation::TrackUp
                                                               : MapOrientation::NorthUp;
    if (plan && !plan->legs.empty()) {
        const int n = static_cast<int>(plan->legs.size());
        const int first = std::max(plan->activeLeg, 1);
        int sel = selectedLeg_ < 0 ? first : selectedLeg_;
        if (pressed(Action::MenuRight))
            sel = std::min(sel + 1, std::min(n - 1, first + kMaxRows - 1));
        if (pressed(Action::MenuLeft)) sel = std::max(sel - 1, first);
        if (pressed(Action::MenuLeft) || pressed(Action::MenuRight)) selectedLeg_ = sel;
        if (pressed(Action::MenuAccept) && sel >= 1 && sel < n) {
            res.directToLeg = sel;
            selectedLeg_ = -1;
        }
    } else {
        selectedLeg_ = -1;
    }
    if (pressed(Action::MenuBack) || pressed(Action::MapToggle)) res.close = true;
    return res;
}

void MapPage::refreshCaches(const NavDatabase& db, double latDeg, double lonDeg) {
    const float zoom = zoomNm();
    const double moveNm = mToNm(greatCircleDistM(latDeg, lonDeg, cacheLat_, cacheLon_));
    if (cacheValid_ && zoom == cacheZoomNm_ && moveNm < 0.3 * zoom) return;
    cacheValid_ = true;
    cacheLat_ = latDeg;
    cacheLon_ = lonDeg;
    cacheZoomNm_ = zoom;
    const double rNm = zoom * 1.9;
    const double dLat = rNm / 60.0;
    const double dLon = rNm / (60.0 * std::max(0.2, std::cos(degToRad(latDeg))));
    const MapDeclutter caps = mapDeclutterCaps(zoom);

    airports_ = db.airportsInBox(latDeg - dLat, lonDeg - dLon, latDeg + dLat, lonDeg + dLon);
    airports_.erase(
        std::remove_if(airports_.begin(), airports_.end(),
                       [&](const Airport* ap) { return !mapShowAirport(ap->type, zoom); }),
        airports_.end());
    if (static_cast<int>(airports_.size()) > caps.maxAirports) {
        auto distOf = [&](const Airport* ap) {
            return greatCircleDistM(latDeg, lonDeg, ap->latDeg, ap->lonDeg);
        };
        std::sort(airports_.begin(), airports_.end(), [&](const Airport* x, const Airport* y) {
            const int rx = mapAirportRank(x->type), ry = mapAirportRank(y->type);
            return rx != ry ? rx < ry : distOf(x) < distOf(y);
        });
        airports_.resize(static_cast<size_t>(caps.maxAirports));
    }

    navaids_ = db.navaidsNear(latDeg, lonDeg, static_cast<float>(rNm));  // nearest first
    navaids_.erase(std::remove_if(navaids_.begin(), navaids_.end(),
                                  [&](const Navaid* nv) { return !mapShowNavaid(nv->type, zoom); }),
                   navaids_.end());
    if (static_cast<int>(navaids_.size()) > caps.maxNavaids)
        navaids_.resize(static_cast<size_t>(caps.maxNavaids));
}

bool MapPage::placeLabel(float x, float y, float w, float h) {
    if (x + w < 0 || x > vw_ || y + h < 0 || y > 480) return false;
    for (int i = 0; i < labelCount_; ++i) {
        const LabelRect& l = labels_[i];
        if (x < l.x1 && x + w > l.x0 && y < l.y1 && y + h > l.y0) return false;
    }
    if (labelCount_ < kMaxLabels) labels_[labelCount_++] = {x, y, x + w, y + h};
    return true;
}

void MapPage::drawRings(Renderer2D& r, const MapProjection& p, const AircraftState& ac) {
    const float R = p.ringRadiusPx;
    r.setLineWidth(1.5f);
    r.circle(p.cx, p.cy, R, kRing);
    r.circle(p.cx, p.cy, R * 0.5f, kRing.withAlpha(0.6f));
    char buf[16];
    const float z = p.zoomNm;
    const float s45 = 0.7071f;
    std::snprintf(buf, sizeof buf, "%g", z);
    r.text(p.cx + R * s45 + 3, p.cy + R * s45 - 6, buf, TextSize::S, kDim);
    std::snprintf(buf, sizeof buf, "%g", z * 0.5f);
    r.text(p.cx + R * 0.5f * s45 + 3, p.cy + R * 0.5f * s45 - 6, buf, TextSize::S, kDim);
    // North marker on the outer ring and heading tick.
    const float nA = -p.rotationRad;
    const Pt2 n = rot(0, -(R + 10), nA);
    r.text(p.cx + n.x, p.cy + n.y, "N", TextSize::S, kText, HAlign::Center, VAlign::Middle);
    const float hA = ac.headingTrueRad - p.rotationRad;
    const Pt2 h0 = rot(0, -R, hA), h1 = rot(0, -R + 10, hA);
    r.setLineWidth(2.5f);
    r.line(p.cx + h0.x, p.cy + h0.y, p.cx + h1.x, p.cy + h1.y, kVor);
}

void MapPage::drawRoute(Renderer2D& r, const MapProjection& p, const FlightPlan& plan) {
    const int n = static_cast<int>(plan.legs.size());
    float lastLx = -1000, lastLy = -1000;
    for (int i = 0; i < n; ++i) {
        const Leg& leg = plan.legs[static_cast<size_t>(i)];
        const MapLegStyle st =
            mapLegStyle(i, plan.activeLeg, plan.approachStartLeg, leg.missedApproach);
        const Color col = legColor(st.kind);
        float x1, y1;
        p.toScreen(leg.to.latDeg, leg.to.lonDeg, x1, y1);
        if (i >= 1) {
            const Leg& prev = plan.legs[static_cast<size_t>(i - 1)];
            float x0, y0;
            p.toScreen(prev.to.latDeg, prev.to.lonDeg, x0, y0);
            r.setLineWidth(st.widthPx);
            if (leg.type == LegType::RF && leg.arcRadiusNm > 0) {
                float ce, cn, pe, pn, te, tn;
                p.offsetsNm(leg.arcCenterLatDeg, leg.arcCenterLonDeg, ce, cn);
                p.offsetsNm(prev.to.latDeg, prev.to.lonDeg, pe, pn);
                p.offsetsNm(leg.to.latDeg, leg.to.lonDeg, te, tn);
                const float b0 = std::atan2(pe - ce, pn - cn), b1 = std::atan2(te - ce, tn - cn);
                float d = b1 - b0;
                if (leg.arcRight) {
                    while (d < 0) d += 2 * kMapPi;
                } else {
                    while (d > 0) d -= 2 * kMapPi;
                }
                const float rad = std::hypot(pe - ce, pn - cn);
                pts_.clear();
                const int steps = 16;
                for (int k = 0; k <= steps; ++k) {
                    const float b = b0 + d * static_cast<float>(k) / steps;
                    Pt2 q;
                    p.offsetToScreen(ce + rad * std::sin(b), cn + rad * std::cos(b), q.x, q.y);
                    pts_.push_back(q);
                }
                if (st.dashed) {
                    for (size_t k = 0; k + 1 < pts_.size(); k += 2)
                        r.line(pts_[k].x, pts_[k].y, pts_[k + 1].x, pts_[k + 1].y, col);
                } else {
                    r.polyline(pts_.data(), static_cast<int>(pts_.size()), col);
                }
            } else if (st.dashed) {
                r.dashedLine(x0, y0, x1, y1, 8, 6, col);
            } else {
                r.line(x0, y0, x1, y1, col);
            }
        }
        // Waypoint symbol + label.
        if (x1 < -10 || x1 > vw_ + 10 || y1 < -10 || y1 > 490) continue;
        const bool sel = i == effectiveSel_;
        r.setLineWidth(1.5f);
        const Pt2 tri[3] = {{x1, y1 - 5}, {x1 + 4.5f, y1 + 3.5f}, {x1 - 4.5f, y1 + 3.5f}};
        r.strokeTriangle(tri[0], tri[1], tri[2], sel ? kSelect : col);
        if (sel) r.circle(x1, y1, 10, kSelect);
        if (leg.to.ident.empty() || leg.to.kind == WaypointKind::Synthetic) continue;
        const float lw = StrokeFont::measure(leg.to.ident.c_str(), TextSize::S);
        if (std::hypot(x1 - lastLx, y1 - lastLy) < 14 && !sel) continue;
        if (placeLabel(x1 + 7, y1 - 14, lw, 9)) {
            r.text(x1 + 7, y1 - 14, leg.to.ident.c_str(), TextSize::S, sel ? kSelect : col,
                   HAlign::Left, VAlign::Top);
            lastLx = x1;
            lastLy = y1;
            char alt[24];
            if (mapShowConstraints(p.zoomNm) && mapFormatAlt(leg.alt, alt, sizeof alt) > 0) {
                const float aw = StrokeFont::measure(alt, TextSize::S);
                if (placeLabel(x1 + 7, y1 - 3, aw, 9))
                    r.text(x1 + 7, y1 - 3, alt, TextSize::S, kDim, HAlign::Left, VAlign::Top);
            }
        }
    }
}

void MapPage::drawAirports(Renderer2D& r, const MapProjection& p) {
    const float zoom = p.zoomNm;
    for (const Airport* ap : airports_) {
        float x, y;
        p.toScreen(ap->latDeg, ap->lonDeg, x, y);
        if (x < -30 || x > vw_ + 30 || y < -30 || y > 510) continue;
        if (mapShowRunways(ap->type, zoom)) {
            r.setLineWidth(2.5f);
            for (const Runway& rw : ap->runways) {
                if (rw.ident > rw.oppositeIdent && !rw.oppositeIdent.empty())
                    continue;  // once per pair
                float x0, y0, x1, y1;
                p.toScreen(rw.startLatDeg, rw.startLonDeg, x0, y0);
                p.toScreen(rw.endLatDeg, rw.endLonDeg, x1, y1);
                r.line(x0, y0, x1, y1, kAirport);
            }
        }
        r.setLineWidth(1.5f);
        const float rad = ap->type == AirportType::Large    ? 6.0f
                          : ap->type == AirportType::Medium ? 4.5f
                                                            : 3.0f;
        r.circle(x, y, rad, kAirport);
        if (mapShowAirportLabel(ap->type, zoom)) {
            const float w = StrokeFont::measure(ap->ident.c_str(), TextSize::S);
            if (placeLabel(x + rad + 3, y - 4, w, 9))
                r.text(x + rad + 3, y - 4, ap->ident.c_str(), TextSize::S, kAirport, HAlign::Left,
                       VAlign::Top);
        }
    }
}

void MapPage::drawNavaids(Renderer2D& r, const MapProjection& p) {
    const float zoom = p.zoomNm;
    r.setLineWidth(1.5f);
    for (const Navaid* nv : navaids_) {
        float x, y;
        p.toScreen(nv->latDeg, nv->lonDeg, x, y);
        if (x < -10 || x > vw_ + 10 || y < -10 || y > 490) continue;
        const bool ndb = nv->type == NavaidType::Ndb || nv->type == NavaidType::NdbDme;
        const bool vor = nv->type == NavaidType::Vor || nv->type == NavaidType::VorDme ||
                         nv->type == NavaidType::Vortac;
        if (vor) {
            Pt2 hex[6];
            for (int k = 0; k < 6; ++k) {
                const float a = kMapPi / 3.0f * static_cast<float>(k);
                hex[k] = {x + 5 * std::sin(a), y - 5 * std::cos(a)};
            }
            r.polyline(hex, 6, kVor, true);
            r.fillCircle(x, y, 1.2f, kVor, FillMode::Front);
        } else if (ndb) {
            r.circle(x, y, 4, kNdb);
            r.fillCircle(x, y, 1.2f, kNdb, FillMode::Front);
        } else {
            r.strokeRect(x - 3.5f, y - 3.5f, 7, 7, kVor);
        }
        if (mapShowNavaidLabel(nv->type, zoom)) {
            const float w = StrokeFont::measure(nv->ident.c_str(), TextSize::S);
            if (placeLabel(x + 8, y - 4, w, 9))
                r.text(x + 8, y - 4, nv->ident.c_str(), TextSize::S, ndb ? kNdb : kVor,
                       HAlign::Left, VAlign::Top);
        }
    }
}

void MapPage::drawAircraft(Renderer2D& r, const MapProjection& p, const AircraftState& ac) {
    // One-minute ground track vector.
    const float trackA = ac.trackTrueRad - p.rotationRad;
    const float lenNm = std::min(ac.gsKt / 60.0f, p.zoomNm);
    if (lenNm > 0.02f) {
        const Pt2 e = rot(0, -lenNm * p.pxPerNm(), trackA);
        r.setLineWidth(2);
        r.line(p.cx, p.cy, p.cx + e.x, p.cy + e.y, kTrack);
    }
    // Aircraft symbol, rotated to the heading.
    r.push();
    r.translate(p.cx, p.cy);
    r.rotate(ac.headingTrueRad - p.rotationRad);
    r.setLineWidth(2.5f);
    r.line(0, -11, 0, 11, kOwn);
    r.line(-10, 1, 10, 1, kOwn);
    r.line(-4, 9, 4, 9, kOwn);
    r.pop();
}

void MapPage::drawWind(Renderer2D& r, const MapProjection& p, const AircraftState& ac,
                       float magVar) {
    const float x = 34, y = 78;
    r.text(8, 38, orientation_ == MapOrientation::NorthUp ? "NORTH UP" : "TRACK UP", TextSize::S,
           kText);
    char buf[32];
    std::snprintf(buf, sizeof buf, "RANGE %g NM", p.zoomNm);
    r.text(8, 50, buf, TextSize::S, kText);
    if (ac.windKt < 1) {
        r.text(8, 66, "WIND CALM", TextSize::S, kDim);
        return;
    }
    const float toA = degToRad(ac.windFromTrueDeg + 180.0f) - p.rotationRad;
    r.push();
    r.translate(x, y + 8);
    r.rotate(toA);
    r.setLineWidth(2);
    r.line(0, 12, 0, -12, kText);
    r.line(0, -12, -4, -6, kText);
    r.line(0, -12, 4, -6, kText);
    r.pop();
    const int dirMag = static_cast<int>(std::lround(wrap360(ac.windFromTrueDeg - magVar)));
    std::snprintf(buf, sizeof buf, "%03d/%d", dirMag == 0 ? 360 : dirMag,
                  static_cast<int>(std::lround(ac.windKt)));
    r.text(x, y + 28, buf, TextSize::S, kText, HAlign::Center);
}

void MapPage::drawPanel(Renderer2D& r, const FlightPlan* plan, const AircraftState& ac,
                        float magVar) {
    const float w = 190, x0 = vw_ - w - 4, y0 = 6;
    const bool havePlan = plan && rowCount_ > 0;
    const float h = havePlan ? 6 + 14 + 14 * 5 + 6 + 14 * 3 : 36;
    r.fillRect(x0, y0, w, h, Color(0.0f, 0.0f, 0.0f, 0.78f), FillMode::Front);
    r.setLineWidth(1);
    r.strokeRect(x0, y0, w, h, kRing);
    if (!havePlan) {
        r.text(x0 + 8, y0 + 12, "NO FLIGHT PLAN", TextSize::S, kDim);
        r.text(x0 + 8, y0 + 26, "Y NORTH/TRACK UP", TextSize::S, kDim);
        return;
    }
    char buf[48], alt[24];
    r.text(x0 + 8, y0 + 14, "ROUTE", TextSize::S, kMagenta);
    r.text(x0 + w - 8, y0 + 14, "NM", TextSize::S, kDim, HAlign::Right);
    // Window of 5 rows containing the selection.
    int top = 0;
    int selRow = 0;
    for (int i = 0; i < rowCount_; ++i)
        if (rows_[i].legIndex == effectiveSel_) selRow = i;
    if (selRow >= 5) top = selRow - 4;
    float y = y0 + 28;
    for (int i = top; i < std::min(rowCount_, top + 5); ++i, y += 14) {
        const MapWptInfo& row = rows_[i];
        const bool sel = row.legIndex == effectiveSel_;
        const Color c = sel ? kSelect : (row.legIndex == plan->activeLeg ? kMagenta : kText);
        mapFormatAlt(row.alt, alt, sizeof alt);
        std::snprintf(buf, sizeof buf, "%c%-6.6s%5.1f %s", sel ? '>' : ' ', row.ident, row.distNm,
                      alt);
        r.text(x0 + 6, y, buf, TextSize::S, c);
    }
    // Details of the selected waypoint.
    const MapWptInfo* sel = nullptr;
    for (int i = 0; i < rowCount_; ++i)
        if (rows_[i].legIndex == effectiveSel_) sel = &rows_[i];
    if (!sel) sel = &rows_[0];
    y = y0 + 28 + 14 * 5 + 6;
    r.line(x0 + 4, y - 11, x0 + w - 4, y - 11, kRing);
    r.setLineWidth(2);
    std::snprintf(buf, sizeof buf, "%s %.1f NM", sel->ident, sel->distNm);
    r.text(x0 + 8, y + 2, buf, TextSize::S, kSelect);
    char ete[16];
    mapFormatEte(sel->eteS, ete, sizeof ete);
    const int brg = static_cast<int>(std::lround(sel->bearingMagDeg)) % 360;
    if (sel->hasDtk)
        std::snprintf(buf, sizeof buf, "BRG %03d  DTK %03d", brg,
                      static_cast<int>(std::lround(sel->dtkMagDeg)) % 360);
    else
        std::snprintf(buf, sizeof buf, "BRG %03d  DTK ---", brg);
    r.text(x0 + 8, y + 16, buf, TextSize::S, kText);
    std::snprintf(buf, sizeof buf, "ETE %s", ete);
    r.text(x0 + 8, y + 30, buf, TextSize::S, kText);
    (void)ac;
    (void)magVar;
}

void MapPage::draw(Renderer2D& r, const AircraftState& ac, const NavState& nav,
                   const FlightPlan* plan, const NavDatabase& db, float magVarDeg) {
    (void)nav;
    vw_ = r.virtualWidth();
    labelCount_ = 0;
    const MapProjection p =
        makeMapProjection(ac.latDeg, ac.lonDeg, ac.trackTrueRad, orientation_, zoomNm(), vw_);
    refreshCaches(db, ac.latDeg, ac.lonDeg);

    // Rows and selection.
    rowCount_ = 0;
    effectiveSel_ = -1;
    if (plan && !plan->legs.empty()) {
        rowCount_ =
            mapWaypointRows(*plan, ac.latDeg, ac.lonDeg, ac.gsKt, magVarDeg, rows_, kMaxRows);
        const int first = std::max(plan->activeLeg, 1);
        effectiveSel_ = selectedLeg_ >= 0 ? selectedLeg_ : first;
        if (rowCount_ == 0) effectiveSel_ = -1;
    }

    r.setOutline(false);
    r.fillRect(0, 0, vw_, 480, kBg, FillMode::Back);
    drawRings(r, p, ac);
    if (plan) drawRoute(r, p, *plan);
    drawNavaids(r, p);
    drawAirports(r, p);
    drawAircraft(r, p, ac);
    drawWind(r, p, ac, magVarDeg);
    drawPanel(r, plan, ac, magVarDeg);

    char buf[48];
    std::snprintf(buf, sizeof buf, "TRK %03d  GS %d",
                  static_cast<int>(std::lround(wrap360(radToDeg(ac.trackTrueRad) - magVarDeg))) %
                      360,
                  static_cast<int>(std::lround(ac.gsKt)));
    r.text(8, 24, buf, TextSize::S, kTrack);
    r.text(8, 470, "UP/DN RANGE  L/R WPT  A DIRECT  Y N/TRK  B BACK", TextSize::S, kDim);
    r.setOutline(true);
}

}  // namespace fsim
