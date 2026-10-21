#include "world/WorldOverlayLogic.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "core/Units.h"

namespace fsim::world {
namespace {

constexpr float kViewH = 480.0f;
constexpr double kMaxLegM = 200.0 * 1852.0;
constexpr double kRouteRangeM = 40.0 * 1852.0;

float groundFtAt(const TerrainQuery* terrain, double latDeg, double lonDeg) {
    if (!terrain) return 0.0f;
    return static_cast<float>(mToFt(terrain->elevationM(latDeg, lonDeg)));
}

void relPos(const OverlayEye& eye, double latDeg, double lonDeg, double altMslM, float out[3]) {
    GeoPos g;
    g.latDeg = latDeg;
    g.lonDeg = lonDeg;
    g.altMslM = altMslM;
    const Ecef p = geodeticToEcef(g);
    out[0] = static_cast<float>(p.x - eye.ecef.x);
    out[1] = static_cast<float>(p.y - eye.ecef.y);
    out[2] = static_cast<float>(p.z - eye.ecef.z);
}

void pushVert(std::vector<float>& b, const float p[3], float r, float g, float bl) {
    b.push_back(p[0]);
    b.push_back(p[1]);
    b.push_back(p[2]);
    b.push_back(r);
    b.push_back(g);
    b.push_back(bl);
}

void pushQuad(std::vector<float>& b, const float a[3], const float c[3], const float d[3],
              const float e[3], float r, float g, float bl) {
    pushVert(b, a, r, g, bl);
    pushVert(b, c, r, g, bl);
    pushVert(b, d, r, g, bl);
    pushVert(b, a, r, g, bl);
    pushVert(b, d, r, g, bl);
    pushVert(b, e, r, g, bl);
}

void sidePoint(double latDeg, double lonDeg, double bearingRad, double distM, double& outLat,
               double& outLon) {
    destinationPoint(latDeg, lonDeg, bearingRad, distM, outLat, outLon);
}

// One span of a route or path: a flat ribbon plus a matching vertical fin, tapered to `half0` /
// `half1` meters. Skips spans that run through the cockpit.
void addSpan(std::vector<float>& batch, const OverlayEye& eye, double lat0, double lon0,
             double alt0, double lat1, double lon1, double alt1, float half0, float half1, float r,
             float g, float bl) {
    const double dist = greatCircleDistM(lat0, lon0, lat1, lon1);
    if (dist < 25.0 || dist > kMaxLegM) return;
    const double d0 = greatCircleDistM(eye.latDeg, eye.lonDeg, lat0, lon0);
    const double d1 = greatCircleDistM(eye.latDeg, eye.lonDeg, lat1, lon1);
    if (d0 < 180.0 && d1 < 180.0) return;
    // The caller sizes the ends for the whole leg. A clipped span can start much closer, so cap
    // each end by the width of a few pixels at ITS distance. Otherwise the near end is a wall.
    auto localHalf = [&](float requested, double distM) {
        const float want =
            metersForPixels(static_cast<float>(std::max(1.0, distM)), 3.0f, kViewH, eye.vFovRad);
        return std::min(requested, std::clamp(want, 4.0f, 45.0f));
    };
    half0 = localHalf(half0, d0);
    half1 = localHalf(half1, d1);
    const double bear = initialBearingTrueRad(lat0, lon0, lat1, lon1);
    const double left = bear - fsim::kPi * 0.5;
    const double right = bear + fsim::kPi * 0.5;
    double la0, lo0, la1, lo1, ra0, ro0, ra1, ro1;
    sidePoint(lat0, lon0, left, half0, la0, lo0);
    sidePoint(lat1, lon1, left, half1, la1, lo1);
    sidePoint(lat0, lon0, right, half0, ra0, ro0);
    sidePoint(lat1, lon1, right, half1, ra1, ro1);
    float q0[3], q1[3], q2[3], q3[3];
    relPos(eye, la0, lo0, alt0, q0);
    relPos(eye, la1, lo1, alt1, q1);
    relPos(eye, ra1, ro1, alt1, q2);
    relPos(eye, ra0, ro0, alt0, q3);
    pushQuad(batch, q0, q1, q2, q3, r, g, bl);

    relPos(eye, lat0, lon0, alt0 + half0, q0);
    relPos(eye, lat1, lon1, alt1 + half1, q1);
    relPos(eye, lat1, lon1, alt1 - half1, q2);
    relPos(eye, lat0, lon0, alt0 - half0, q3);
    pushQuad(batch, q0, q1, q2, q3, r, g, bl);
}

// Draws the parts of the segment that stay at least ~300 m from the eye, so a leg the aircraft
// is flying does not become a wall through the cockpit.
void addSegment(std::vector<float>& batch, const OverlayEye& eye, double lat0, double lon0,
                double alt0, double lat1, double lon1, double alt1, float half0, float half1,
                float r, float g, float bl) {
    const double len = greatCircleDistM(lat0, lon0, lat1, lon1);
    if (len < 25.0 || len > kMaxLegM) return;
    const LatLon eyeLl{eye.latDeg, eye.lonDeg};
    const LatLon a{lat0, lon0};
    const LatLon b{lat1, lon1};
    const double along = alongTrackM(eyeLl, a, b);
    const double xtk = std::fabs(crossTrackM(eyeLl, a, b));
    constexpr double kKeepM = 320.0;
    const double bear = initialBearingTrueRad(lat0, lon0, lat1, lon1);
    auto emit = [&](double s0, double s1) {
        if (s1 - s0 < 40.0) return;
        double la0, lo0, la1, lo1;
        destinationPoint(lat0, lon0, bear, s0, la0, lo0);
        destinationPoint(lat0, lon0, bear, s1, la1, lo1);
        const double t0 = s0 / len, t1 = s1 / len;
        const float h0 = half0 + static_cast<float>(t0) * (half1 - half0);
        const float h1 = half0 + static_cast<float>(t1) * (half1 - half0);
        addSpan(batch, eye, la0, lo0, alt0 + (alt1 - alt0) * t0, la1, lo1,
                alt0 + (alt1 - alt0) * t1, h0, h1, r, g, bl);
    };
    if (xtk > 900.0 || along < -200.0 || along > len + 200.0) {
        addSpan(batch, eye, lat0, lon0, alt0, lat1, lon1, alt1, half0, half1, r, g, bl);
        return;
    }
    emit(0.0, along - kKeepM);
    emit(along + kKeepM, len);
}

void addPole(std::vector<float>& batch, const OverlayEye& eye, double latDeg, double lonDeg,
             double baseM, double topM, float halfM, float r, float g, float bl) {
    if (topM < baseM + 40.0) topM = baseM + 80.0;
    for (int axis = 0; axis < 2; ++axis) {
        const double bear = axis == 0 ? 0.0 : fsim::kPi * 0.5;
        double la, loa, lb, lob;
        sidePoint(latDeg, lonDeg, bear, halfM, la, loa);
        sidePoint(latDeg, lonDeg, bear + fsim::kPi, halfM, lb, lob);
        float b0[3], b1[3], t1[3], t0[3];
        relPos(eye, la, loa, baseM, b0);
        relPos(eye, lb, lob, baseM, b1);
        relPos(eye, lb, lob, topM, t1);
        relPos(eye, la, loa, topM, t0);
        pushQuad(batch, b0, b1, t1, t0, r, g, bl);
    }
}

float poleHalfM(float distM, float vFovRad) {
    return std::clamp(metersForPixels(distM, 1.6f, kViewH, vFovRad), 8.0f, 40.0f);
}

float routeHalfM(float distM, float vFovRad) {
    return std::clamp(metersForPixels(distM, 3.0f, kViewH, vFovRad), 6.0f, 70.0f);
}

struct PendingLabel {
    float x = 0, y = 0, w = 0, h = 0;
    char text[40] = {};
    float r = 1, g = 1, b = 1;
};

bool projectLabel(const OverlayEye& eye, double latDeg, double lonDeg, double altMslM, float& x,
                  float& y) {
    GeoPos g;
    g.latDeg = latDeg;
    g.lonDeg = lonDeg;
    g.altMslM = altMslM;
    const Ecef p = geodeticToEcef(g);
    const Vec3d dir = eye.ecefToEye * Vec3d{p.x - eye.ecef.x, p.y - eye.ecef.y, p.z - eye.ecef.z};
    if (dir.z >= -1e-4) return false;
    const float focal = (kViewH * 0.5f) / std::tan(eye.vFovRad * 0.5f);
    const float inv = 1.0f / static_cast<float>(-dir.z);
    const float canvasW = kViewH * eye.aspect;
    x = canvasW * 0.5f + focal * static_cast<float>(dir.x) * inv;
    y = kViewH * 0.5f - focal * static_cast<float>(dir.y) * inv;
    return x > -80.0f && x < canvasW + 80.0f && y > -80.0f && y < kViewH + 80.0f;
}

void queueLabel(std::vector<PendingLabel>& pending, const OverlayEye& eye, double latDeg,
                double lonDeg, double altMslM, const char* text, float r, float g, float bl) {
    if (pending.size() >= 48) return;
    float x = 0, y = 0;
    if (!projectLabel(eye, latDeg, lonDeg, altMslM, x, y)) return;
    PendingLabel lab;
    lab.x = x;
    lab.y = y - 16.0f;
    std::snprintf(lab.text, sizeof lab.text, "%s", text);
    lab.w = 7.0f * static_cast<float>(std::strlen(lab.text));
    lab.h = 14.0f;
    lab.r = r;
    lab.g = g;
    lab.b = bl;
    pending.push_back(lab);
}

void formatNm(char* buf, int n, const std::string& ident, double distNm) {
    char id[16];
    std::snprintf(id, sizeof id, "%.12s", ident.c_str());
    std::snprintf(buf, static_cast<size_t>(n), "%s  %.1f NM", id, distNm);
}

void addGate(std::vector<float>& batch, const OverlayEye& eye, double latDeg, double lonDeg,
             double altM, double bearingRad, float halfW, float halfH, float thick, float r,
             float g, float bl) {
    const double right = bearingRad + fsim::kPi * 0.5;
    auto corner = [&](float side, float up, double& olat, double& olon, double& oalt) {
        const double bear = side >= 0 ? right : right + fsim::kPi;
        sidePoint(latDeg, lonDeg, bear, std::fabs(side), olat, olon);
        oalt = altM + up;
    };
    const float s[4] = {-halfW, halfW, halfW, -halfW};
    const float u[4] = {-halfH, -halfH, halfH, halfH};
    const float si[4] = {-halfW + thick, halfW - thick, halfW - thick, -halfW + thick};
    const float ui[4] = {-halfH + thick, -halfH + thick, halfH - thick, halfH - thick};
    if (thick * 2.0f >= halfW || thick * 2.0f >= halfH) return;
    for (int e = 0; e < 4; ++e) {
        const int n = (e + 1) % 4;
        double la, loa, lb, lob, lia, lioa, lib, liob;
        double aa, ab, aia, aib;
        corner(s[e], u[e], la, loa, aa);
        corner(s[n], u[n], lb, lob, ab);
        corner(si[e], ui[e], lia, lioa, aia);
        corner(si[n], ui[n], lib, liob, aib);
        float p0[3], p1[3], p2[3], p3[3];
        relPos(eye, la, loa, aa, p0);
        relPos(eye, lb, lob, ab, p1);
        relPos(eye, lib, liob, aib, p2);
        relPos(eye, lia, lioa, aia, p3);
        pushQuad(batch, p0, p1, p2, p3, r, g, bl);
    }
}

void addRunwayOutline(std::vector<float>& batch, const OverlayEye& eye, const Runway& rw) {
    if (rw.ident > rw.oppositeIdent && !rw.oppositeIdent.empty()) return;
    const double distEnds =
        greatCircleDistM(rw.startLatDeg, rw.startLonDeg, rw.endLatDeg, rw.endLonDeg);
    if (distEnds < 30.0) return;
    const double midLat = (rw.startLatDeg + rw.endLatDeg) * 0.5;
    const double midLon = (rw.startLonDeg + rw.endLonDeg) * 0.5;
    const double distEye = greatCircleDistM(eye.latDeg, eye.lonDeg, midLat, midLon);
    const float trueHalf = static_cast<float>(ftToM(std::max(0.0f, rw.widthFt))) * 0.5f;
    const float half =
        std::min(250.0f, overlayRunwayHalfWidthM(trueHalf, static_cast<float>(distEye), 5.0f,
                                                 kViewH, eye.vFovRad));
    const double bear =
        initialBearingTrueRad(rw.startLatDeg, rw.startLonDeg, rw.endLatDeg, rw.endLonDeg);
    const double left = bear - fsim::kPi * 0.5;
    const double right = bear + fsim::kPi * 0.5;
    const double alt = ftToM(rw.thrElevFt) + 8.0;
    double cLat[4], cLon[4];
    sidePoint(rw.startLatDeg, rw.startLonDeg, left, half, cLat[0], cLon[0]);
    sidePoint(rw.startLatDeg, rw.startLonDeg, right, half, cLat[1], cLon[1]);
    sidePoint(rw.endLatDeg, rw.endLonDeg, right, half, cLat[2], cLon[2]);
    sidePoint(rw.endLatDeg, rw.endLonDeg, left, half, cLat[3], cLon[3]);
    const float edge = std::clamp(
        metersForPixels(static_cast<float>(distEye), 2.0f, kViewH, eye.vFovRad), 8.0f, 36.0f);
    // Tall enough to read from the cockpit: a few pixels of fin, not a wall up close.
    const float fin = std::clamp(
        metersForPixels(static_cast<float>(distEye), 6.0f, kViewH, eye.vFovRad), 14.0f, 160.0f);
    // Long edges (0-3 and 1-2) get a vertical fin; every edge gets a flat stroke.
    const int edges[4][2] = {{0, 3}, {1, 2}, {0, 1}, {3, 2}};
    const bool vertical[4] = {true, true, false, false};
    const float cr = 0.55f, cg = 1.0f, cb = 1.0f;
    for (int e = 0; e < 4; ++e) {
        const int a = edges[e][0], c = edges[e][1];
        addSegment(batch, eye, cLat[a], cLon[a], alt, cLat[c], cLon[c], alt, edge * 0.5f,
                   edge * 0.5f, cr, cg, cb);
        if (!vertical[e]) continue;
        float p0[3], p1[3], p2[3], p3[3];
        relPos(eye, cLat[a], cLon[a], alt, p0);
        relPos(eye, cLat[c], cLon[c], alt, p1);
        relPos(eye, cLat[c], cLon[c], alt + fin, p2);
        relPos(eye, cLat[a], cLon[a], alt + fin, p3);
        pushQuad(batch, p0, p1, p2, p3, cr, cg, cb);
    }
}

}  // namespace

int overlayMatchingPreset(const OverlayToggles& t) {
    if (!t.route && !t.vpath && !t.airports && !t.runways) return 0;
    if (t.route && !t.vpath && !t.airports && !t.runways) return 1;
    if (t.route && t.vpath && !t.airports && !t.runways) return 2;
    if (!t.route && !t.vpath && t.airports && t.runways) return 3;
    if (t.route && t.vpath && t.airports && t.runways) return 4;
    return -1;
}

const char* overlayPresetName(int preset) {
    switch (preset) {
    case 0: return "OFF";
    case 1: return "ROUTE";
    case 2: return "ROUTE+PATH";
    case 3: return "AIRPORTS";
    case 4: return "ALL";
    default: return "CUSTOM";
    }
}

void overlayApplyPreset(OverlayToggles& t, int preset) {
    t = {};
    switch (preset) {
    case 1: t.route = true; break;
    case 2: t.route = t.vpath = true; break;
    case 3: t.airports = t.runways = true; break;
    case 4: t.route = t.vpath = t.airports = t.runways = true; break;
    default: break;
    }
}

int overlayCycle(OverlayToggles& t) {
    const int cur = overlayMatchingPreset(t);
    const int next = cur < 0 ? 0 : (cur + 1) % kOverlayPresetCount;
    overlayApplyPreset(t, next);
    return next;
}

float overlayMarkerAltFt(const AltConstraint& alt, float cruiseFt, float groundFt) {
    if (alt.kind != AltConstraintKind::None && alt.altFt > 1.0f) return alt.altFt;
    if (cruiseFt > groundFt + 100.0f) return cruiseFt;
    return groundFt + 400.0f;
}

float metersForPixels(float distM, float px, float viewH, float vFovRad) {
    if (!(distM > 1.0f) || viewH < 1.0f || !(vFovRad > 0.01f)) return 0.0f;
    const float span = 2.0f * distM * std::tan(vFovRad * 0.5f);
    return px * span / viewH;
}

float overlayRunwayHalfWidthM(float trueHalfWidthM, float distM, float minPx, float viewH,
                              float vFovRad) {
    return std::max(trueHalfWidthM, metersForPixels(distM, minPx, viewH, vFovRad));
}

void selectOverlayAirports(const std::vector<double>& distNm, double radiusNm, double hideInsideNm,
                           int maxCount, std::vector<int>& outIndices) {
    outIndices.clear();
    if (maxCount <= 0 || !(radiusNm > 0.0)) return;
    std::vector<int> idx;
    idx.reserve(distNm.size());
    for (int i = 0; i < static_cast<int>(distNm.size()); ++i) {
        const double d = distNm[static_cast<size_t>(i)];
        if (d > hideInsideNm && d <= radiusNm) idx.push_back(i);
    }
    std::sort(idx.begin(), idx.end(), [&](int a, int b) {
        return distNm[static_cast<size_t>(a)] < distNm[static_cast<size_t>(b)];
    });
    if (static_cast<int>(idx.size()) > maxCount) idx.resize(static_cast<size_t>(maxCount));
    outIndices.swap(idx);
}

bool placeOverlayLabel(float& x, float& y, float w, float h, const OverlayBox* placed,
                       int placedCount, float canvasW, float canvasH) {
    if (!(w > 1.0f) || !(h > 1.0f)) return false;
    const float stepX = w + 6.0f;
    const float stepY = h + 4.0f;
    const float nudges[][2] = {{0, 0},  {0, -1}, {0, -2},  {0, -3}, {1, 0},
                               {-1, 0}, {1, -1}, {-1, -1}, {0, 1}};
    auto hits = [&](const OverlayBox& box) {
        for (int i = 0; i < placedCount; ++i) {
            const OverlayBox& p = placed[i];
            if (box.x < p.x + p.w && box.x + box.w > p.x && box.y < p.y + p.h &&
                box.y + box.h > p.y)
                return true;
        }
        return false;
    };
    for (const float* n : nudges) {
        const float cx = x + n[0] * stepX;
        const float cy = y + n[1] * stepY;
        OverlayBox box{cx - w * 0.5f, cy - h * 0.5f, w, h};
        if (box.x < 2.0f || box.y < 2.0f || box.x + box.w > canvasW - 2.0f ||
            box.y + box.h > canvasH - 2.0f)
            continue;
        if (hits(box)) continue;
        x = cx;
        y = cy;
        return true;
    }
    return false;
}

void overlayEyeAt(double latDeg, double lonDeg, double altMslM, float headingTrueRad, float vFovDeg,
                  float aspect, OverlayEye& out) {
    out.latDeg = latDeg;
    out.lonDeg = lonDeg;
    out.altMslM = altMslM;
    GeoPos g;
    g.latDeg = latDeg;
    g.lonDeg = lonDeg;
    g.altMslM = altMslM;
    out.ecef = geodeticToEcef(g);
    const Mat3d bodyToEcef = nedToEcef(latDeg, lonDeg) * bodyToNed(0.0, 0.0, headingTrueRad);
    Mat3d bodyToEye;
    bodyToEye.m[0][0] = 0;
    bodyToEye.m[0][1] = 1;
    bodyToEye.m[0][2] = 0;
    bodyToEye.m[1][0] = 0;
    bodyToEye.m[1][1] = 0;
    bodyToEye.m[1][2] = -1;
    bodyToEye.m[2][0] = -1;
    bodyToEye.m[2][1] = 0;
    bodyToEye.m[2][2] = 0;
    out.ecefToEye = bodyToEye * transpose(bodyToEcef);
    out.vFovRad = static_cast<float>(degToRad(vFovDeg));
    out.aspect = aspect > 0.1f ? aspect : 1.0f;
}

int buildWorldOverlay(const OverlayToggles& toggles, const FlightPlan* plan,
                      const OverlayAirportRef* airports, int airportCount,
                      const OverlayAirportRef* runwayAirports, int runwayCount,
                      const TerrainQuery* terrain, const OverlayEye& eye, std::vector<float>& route,
                      std::vector<float>& vpath, std::vector<float>& airportVerts,
                      std::vector<float>& runwayVerts, OverlayLabel* labels, int labelCap,
                      int& labelCount) {
    const std::size_t route0 = route.size();
    const std::size_t vpath0 = vpath.size();
    const std::size_t ap0 = airportVerts.size();
    const std::size_t rw0 = runwayVerts.size();
    labelCount = 0;
    std::vector<PendingLabel> pending;

    if (toggles.route && plan) {
        const float cruise = plan->cruiseAltFt;
        const int n = static_cast<int>(plan->legs.size());
        const int active = (plan->activeLeg >= 0 && plan->activeLeg < n) ? plan->activeLeg : -1;
        auto marker = [&](int i, float& topFt, double& baseM, double& topM, float& distM) {
            const Leg& leg = plan->legs[static_cast<size_t>(i)];
            const float ground = groundFtAt(terrain, leg.to.latDeg, leg.to.lonDeg);
            topFt = overlayMarkerAltFt(leg.alt, cruise, ground);
            baseM = ftToM(ground) + 25.0;
            topM = std::max(ftToM(topFt), baseM + 80.0);
            distM = static_cast<float>(
                greatCircleDistM(eye.latDeg, eye.lonDeg, leg.to.latDeg, leg.to.lonDeg));
        };
        for (int i = 0; i < n; ++i) {
            const Leg& leg = plan->legs[static_cast<size_t>(i)];
            if (leg.to.ident.empty() && leg.to.latDeg == 0.0 && leg.to.lonDeg == 0.0) continue;
            float topFt = 0, distM = 0;
            double baseM = 0, topM = 0;
            marker(i, topFt, baseM, topM, distM);
            const bool activeEnd = i == active || (i > 0 && i - 1 == active);
            if (distM > kRouteRangeM && !activeEnd) continue;
            const bool mag = i == active;
            const float r = mag ? 1.0f : (leg.missedApproach ? 0.55f : 0.92f);
            const float g = mag ? 0.25f : (leg.missedApproach ? 0.55f : 0.92f);
            const float b = mag ? 1.0f : (leg.missedApproach ? 0.55f : 0.95f);
            if (distM > 400.0f)
                addPole(route, eye, leg.to.latDeg, leg.to.lonDeg, baseM, topM,
                        poleHalfM(distM, eye.vFovRad), r, g, b);
            if (distM > 700.0f && !leg.to.ident.empty()) {
                char text[40];
                formatNm(text, 40, leg.to.ident, mToNm(distM));
                queueLabel(pending, eye, leg.to.latDeg, leg.to.lonDeg, topM, text, r, g, b);
            }
            if (i == 0) continue;
            const Leg& prev = plan->legs[static_cast<size_t>(i - 1)];
            float pTop = 0, pDist = 0;
            double pBase = 0, pTopM = 0;
            marker(i - 1, pTop, pBase, pTopM, pDist);
            const bool legMag = (i == active);
            const float lr = legMag ? 1.0f : 0.9f;
            const float lg = legMag ? 0.25f : 0.9f;
            const float lb = legMag ? 1.0f : 0.95f;
            addSegment(route, eye, prev.to.latDeg, prev.to.lonDeg, ftToM(pTop), leg.to.latDeg,
                       leg.to.lonDeg, ftToM(topFt), routeHalfM(pDist, eye.vFovRad),
                       routeHalfM(distM, eye.vFovRad), lr, lg, lb);
        }
    }

    if (toggles.vpath && plan && !plan->legs.empty()) {
        const int n = static_cast<int>(plan->legs.size());
        const int active = plan->activeLeg;
        if (active >= 1 && active < n) {
            const Leg& leg = plan->legs[static_cast<size_t>(active)];
            const Leg& prev = plan->legs[static_cast<size_t>(active - 1)];
            if (!leg.missedApproach) {
                const double dist =
                    greatCircleDistM(prev.to.latDeg, prev.to.lonDeg, leg.to.latDeg, leg.to.lonDeg);
                if (dist >= 50.0 && dist < kMaxLegM) {
                    const float g1 = groundFtAt(terrain, leg.to.latDeg, leg.to.lonDeg);
                    const bool approach =
                        plan->approachStartLeg >= 0 && active >= plan->approachStartLeg;
                    float endFt = g1 + 50.0f;
                    if (leg.alt.kind != AltConstraintKind::None && leg.alt.altFt > 1.0f)
                        endFt = leg.alt.altFt;
                    else if (!approach)
                        endFt = overlayMarkerAltFt(leg.alt, plan->cruiseAltFt, g1);
                    float startFt = endFt;
                    if (approach) {
                        if (prev.alt.kind != AltConstraintKind::None && prev.alt.altFt > 1.0f)
                            startFt = prev.alt.altFt;
                        else
                            startFt = endFt + static_cast<float>(mToFt(dist)) *
                                                  static_cast<float>(std::tan(degToRad(3.0)));
                        if (startFt < endFt) startFt = endFt;
                    } else {
                        startFt = endFt;
                    }
                    const double bear = initialBearingTrueRad(prev.to.latDeg, prev.to.lonDeg,
                                                              leg.to.latDeg, leg.to.lonDeg);
                    constexpr int kSamples = 5;
                    double lats[kSamples], lons[kSamples], alts[kSamples];
                    for (int s = 0; s < kSamples; ++s) {
                        const double t = static_cast<double>(s) / (kSamples - 1);
                        destinationPoint(prev.to.latDeg, prev.to.lonDeg, bear, dist * t, lats[s],
                                         lons[s]);
                        alts[s] = ftToM(startFt + static_cast<float>(t) * (endFt - startFt));
                    }
                    const float cr = 0.15f, cg = 0.90f, cb = 1.0f;
                    for (int s = 0; s < kSamples - 1; ++s) {
                        const float d0 = static_cast<float>(
                            greatCircleDistM(eye.latDeg, eye.lonDeg, lats[s], lons[s]));
                        const float d1 = static_cast<float>(
                            greatCircleDistM(eye.latDeg, eye.lonDeg, lats[s + 1], lons[s + 1]));
                        addSegment(vpath, eye, lats[s], lons[s], alts[s], lats[s + 1], lons[s + 1],
                                   alts[s + 1], routeHalfM(d0, eye.vFovRad) * 0.65f,
                                   routeHalfM(d1, eye.vFovRad) * 0.65f, cr, cg, cb);
                    }
                    const float gateT[] = {0.25f, 0.50f, 0.75f};
                    for (float t : gateT) {
                        double lat = 0, lon = 0;
                        destinationPoint(prev.to.latDeg, prev.to.lonDeg, bear,
                                         dist * static_cast<double>(t), lat, lon);
                        const double altM = ftToM(startFt + t * (endFt - startFt));
                        const float d =
                            static_cast<float>(greatCircleDistM(eye.latDeg, eye.lonDeg, lat, lon));
                        if (d < 500.0f) continue;
                        const float halfW = std::clamp(
                            metersForPixels(d, 8.0f, kViewH, eye.vFovRad), 36.0f, 140.0f);
                        const float thick =
                            std::clamp(metersForPixels(d, 1.2f, kViewH, eye.vFovRad), 5.0f, 16.0f);
                        const float halfH = std::max(halfW * 0.45f, thick * 3.0f);
                        addGate(vpath, eye, lat, lon, altM, bear, halfW, halfH, thick, cr, cg, cb);
                    }
                }
            }
        }
    }

    if (toggles.airports && airports && airportCount > 0) {
        const int n = std::min(airportCount, 12);
        for (int i = 0; i < n; ++i) {
            const Airport* ap = airports[i].airport;
            if (!ap || ap->ident.empty()) continue;
            const double distM = greatCircleDistM(eye.latDeg, eye.lonDeg, ap->latDeg, ap->lonDeg);
            const double base = ftToM(ap->elevFt) + 20.0;
            const double top = base + ftToM(500.0);
            const float half = poleHalfM(static_cast<float>(distM), eye.vFovRad);
            addPole(airportVerts, eye, ap->latDeg, ap->lonDeg, base, top, half, 1.0f, 0.75f, 0.15f);
            char text[40];
            formatNm(text, 40, ap->ident, airports[i].distNm);
            queueLabel(pending, eye, ap->latDeg, ap->lonDeg, top, text, 1.0f, 0.78f, 0.2f);
        }
    }

    if (toggles.runways && runwayAirports && runwayCount > 0) {
        const int n = std::min(runwayCount, 12);
        for (int i = 0; i < n; ++i) {
            const Airport* ap = runwayAirports[i].airport;
            if (!ap) continue;
            for (const Runway& rw : ap->runways) addRunwayOutline(runwayVerts, eye, rw);
        }
    }

    if (labels && labelCap > 0) {
        const float canvasW = kViewH * eye.aspect;
        OverlayBox placed[48];
        int placedN = 0;
        for (PendingLabel& lab : pending) {
            if (labelCount >= labelCap) break;
            float x = lab.x, y = lab.y;
            if (!placeOverlayLabel(x, y, lab.w, lab.h, placed, placedN, canvasW, kViewH)) continue;
            OverlayLabel& out = labels[labelCount++];
            out.x = x;
            out.y = y;
            std::snprintf(out.text, sizeof out.text, "%s", lab.text);
            out.r = lab.r;
            out.g = lab.g;
            out.b = lab.b;
            if (placedN < 48)
                placed[placedN++] = OverlayBox{x - lab.w * 0.5f, y - lab.h * 0.5f, lab.w, lab.h};
        }
    }

    int batches = 0;
    if (route.size() > route0) ++batches;
    if (vpath.size() > vpath0) ++batches;
    if (airportVerts.size() > ap0) ++batches;
    if (runwayVerts.size() > rw0) ++batches;
    return batches;
}

}  // namespace fsim::world
