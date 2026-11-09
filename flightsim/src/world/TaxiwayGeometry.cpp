#include "world/TaxiwayGeometry.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <unordered_map>

#include "core/Log.h"
#include "core/Units.h"

namespace fsim::world {
namespace {

constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

double cross(const Pt2& a, const Pt2& b, const Pt2& c) {  // (b - a) x (c - a)
    return (b.eM - a.eM) * (c.nM - a.nM) - (b.nM - a.nM) * (c.eM - a.eM);
}

double dist2(const Pt2& a, const Pt2& b) {
    const double de = b.eM - a.eM, dn = b.nM - a.nM;
    return de * de + dn * dn;
}

double dist2ToSegment(const Pt2& p, const Pt2& a, const Pt2& b) {
    const double ee = b.eM - a.eM, en = b.nM - a.nM;
    const double len2 = ee * ee + en * en;
    double t = len2 > 1e-12 ? ((p.eM - a.eM) * ee + (p.nM - a.nM) * en) / len2 : 0.0;
    t = std::min(1.0, std::max(0.0, t));
    const double de = p.eM - (a.eM + t * ee), dn = p.nM - (a.nM + t * en);
    return de * de + dn * dn;
}

// Douglas-Peucker over pts[i0..i1] (inclusive), marking the points to keep.
void douglasPeucker(const std::vector<Pt2>& pts, int i0, int i1, double tol2,
                    std::vector<char>& keep) {
    std::vector<std::pair<int, int>> stack{{i0, i1}};
    while (!stack.empty()) {
        const auto [a, b] = stack.back();
        stack.pop_back();
        double worst = -1;
        int at = -1;
        for (int i = a + 1; i < b; ++i) {
            const double d2 =
                dist2ToSegment(pts[static_cast<size_t>(i)], pts[static_cast<size_t>(a)],
                               pts[static_cast<size_t>(b)]);
            if (d2 > worst) {
                worst = d2;
                at = i;
            }
        }
        if (at >= 0 && worst > tol2) {
            keep[static_cast<size_t>(at)] = 1;
            stack.emplace_back(a, at);
            stack.emplace_back(at, b);
        }
    }
}

// Inside or on the edge of the counter-clockwise triangle abc.
bool inTriangle(const Pt2& p, const Pt2& a, const Pt2& b, const Pt2& c) {
    constexpr double kEps = -1e-9;
    return cross(a, b, p) >= kEps && cross(b, c, p) >= kEps && cross(c, a, p) >= kEps;
}

// A rectangle around the segment p0 -> p1: halfWidthM to each side, extendM past both ends.
FlattenArea rectArea(const LocalFrame& frame, const Pt2& p0, const Pt2& p1, double halfWidthM,
                     double extendM) {
    const double len = std::sqrt(dist2(p0, p1));
    const double ue = (p1.eM - p0.eM) / len, un = (p1.nM - p0.nM) / len;
    const double re = un, rn = -ue;  // right of the direction
    const Pt2 e0{p0.eM - ue * extendM, p0.nM - un * extendM};
    const Pt2 e1{p1.eM + ue * extendM, p1.nM + un * extendM};
    const Pt2 corner[4] = {{e0.eM - re * halfWidthM, e0.nM - rn * halfWidthM},
                           {e0.eM + re * halfWidthM, e0.nM + rn * halfWidthM},
                           {e1.eM + re * halfWidthM, e1.nM + rn * halfWidthM},
                           {e1.eM - re * halfWidthM, e1.nM - rn * halfWidthM}};
    FlattenArea area;
    for (int i = 0; i < 4; ++i) {
        const LatLon ll = frame.toLatLon(corner[i]);
        area.latDeg[i] = ll.latDeg;
        area.lonDeg[i] = ll.lonDeg;
    }
    return area;
}

}  // namespace

// ---- local frame -------------------------------------------------------------------------------

LocalFrame::LocalFrame(double lat0Deg, double lon0Deg)
    : lat0Deg_(lat0Deg), lon0Deg_(lon0Deg),
      mPerDegLon_(std::max(1.0, kMPerDegLat * std::cos(lat0Deg * kRadPerDeg))) {}

Pt2 LocalFrame::toLocal(const LatLon& p) const {
    double dLon = p.lonDeg - lon0Deg_;
    if (dLon > 180.0) dLon -= 360.0;
    if (dLon < -180.0) dLon += 360.0;
    return {dLon * mPerDegLon_, (p.latDeg - lat0Deg_) * kMPerDegLat};
}

LatLon LocalFrame::toLatLon(const Pt2& p) const {
    LatLon ll;
    ll.latDeg = lat0Deg_ + p.nM / kMPerDegLat;
    ll.lonDeg = lon0Deg_ + p.eM / mPerDegLon_;
    if (ll.lonDeg >= 180.0) ll.lonDeg -= 360.0;
    if (ll.lonDeg < -180.0) ll.lonDeg += 360.0;
    return ll;
}

// ---- polylines ---------------------------------------------------------------------------------

std::vector<int> simplifyPolyline(const std::vector<Pt2>& pts, double toleranceM) {
    const int n = static_cast<int>(pts.size());
    std::vector<int> out;
    if (n <= 2) {
        for (int i = 0; i < n; ++i) out.push_back(i);
        return out;
    }
    std::vector<char> keep(pts.size(), 0);
    keep.front() = keep.back() = 1;
    douglasPeucker(pts, 0, n - 1, toleranceM * toleranceM, keep);
    for (int i = 0; i < n; ++i)
        if (keep[static_cast<size_t>(i)]) out.push_back(i);
    return out;
}

std::vector<int> simplifyRing(const std::vector<Pt2>& ring, double toleranceM) {
    const int n = static_cast<int>(ring.size());
    std::vector<int> out;
    if (n <= 3) {
        for (int i = 0; i < n; ++i) out.push_back(i);
        return out;
    }
    // Split at the point farthest from point 0 and simplify both chains.
    int far = 1;
    for (int i = 2; i < n; ++i)
        if (dist2(ring[0], ring[static_cast<size_t>(i)]) >
            dist2(ring[0], ring[static_cast<size_t>(far)]))
            far = i;
    std::vector<Pt2> chain(ring.begin(), ring.end());
    chain.push_back(ring[0]);  // closes the second chain far .. n-1, 0
    std::vector<char> keep(chain.size(), 0);
    keep[0] = keep[static_cast<size_t>(far)] = 1;
    const double tol2 = toleranceM * toleranceM;
    douglasPeucker(chain, 0, far, tol2, keep);
    douglasPeucker(chain, far, n, tol2, keep);
    for (int i = 0; i < n; ++i)
        if (keep[static_cast<size_t>(i)]) out.push_back(i);
    if (out.size() < 3) {  // keep the point farthest from the line 0 - far
        int best = -1;
        double bestD2 = -1;
        for (int i = 0; i < n; ++i) {
            if (i == 0 || i == far) continue;
            const double d2 = dist2ToSegment(ring[static_cast<size_t>(i)], ring[0],
                                             ring[static_cast<size_t>(far)]);
            if (d2 > bestD2) {
                bestD2 = d2;
                best = i;
            }
        }
        out.push_back(best);
        std::sort(out.begin(), out.end());
    }
    return out;
}

double ringArea(const std::vector<Pt2>& ring) {
    double a2 = 0;
    for (size_t i = 0; i < ring.size(); ++i) {
        const Pt2& p = ring[i];
        const Pt2& q = ring[(i + 1) % ring.size()];
        a2 += p.eM * q.nM - q.eM * p.nM;
    }
    return 0.5 * a2;
}

bool triangulatePolygon(const std::vector<Pt2>& ring, std::vector<std::array<int, 3>>& tris) {
    tris.clear();
    std::vector<int> idx;
    for (int i = 0; i < static_cast<int>(ring.size()); ++i) {
        if (idx.empty() ||
            dist2(ring[static_cast<size_t>(i)], ring[static_cast<size_t>(idx.back())]) > 1e-12)
            idx.push_back(i);
    }
    while (idx.size() > 1 && dist2(ring[static_cast<size_t>(idx.front())],
                                   ring[static_cast<size_t>(idx.back())]) <= 1e-12)
        idx.pop_back();
    if (idx.size() < 3) return false;
    double area2 = 0;
    for (size_t k = 0; k < idx.size(); ++k) {
        const Pt2& p = ring[static_cast<size_t>(idx[k])];
        const Pt2& q = ring[static_cast<size_t>(idx[(k + 1) % idx.size()])];
        area2 += p.eM * q.nM - q.eM * p.nM;
    }
    if (std::fabs(area2) < 1e-6) return false;
    if (area2 < 0) std::reverse(idx.begin(), idx.end());
    auto at = [&](int i) -> const Pt2& { return ring[static_cast<size_t>(i)]; };

    size_t k = 0;
    size_t sinceClip = 0;  // candidates tried since the last clip; a full lap without one fails
    while (idx.size() > 3) {
        const size_t m = idx.size();
        k %= m;
        const int a = idx[(k + m - 1) % m], b = idx[k], c = idx[(k + 1) % m];
        const double cr = cross(at(a), at(b), at(c));
        bool clip = false, emit = false;
        if (std::fabs(cr) < 1e-6) {
            clip = true;  // collinear (or a spike): drop the middle point without a triangle
        } else if (cr > 0) {
            clip = emit = true;
            for (size_t j = 0; j < m && clip; ++j) {
                const int p = idx[j];
                if (p == a || p == b || p == c) continue;
                if (inTriangle(at(p), at(a), at(b), at(c))) clip = emit = false;
            }
        }
        if (clip) {
            if (emit) tris.push_back({a, b, c});
            idx.erase(idx.begin() + static_cast<std::ptrdiff_t>(k));
            if (k > 0) --k;  // the previous point may have become an ear
            sinceClip = 0;
        } else {
            ++k;
            if (++sinceClip > m) {
                tris.clear();
                return false;
            }
        }
    }
    if (cross(at(idx[0]), at(idx[1]), at(idx[2])) > 1e-6) tris.push_back({idx[0], idx[1], idx[2]});
    return !tris.empty();
}

// ---- heights -----------------------------------------------------------------------------------

TaxiwayField::TaxiwayField(const Airport& airport) : airportElevM_(ftToM(airport.elevFt)) {
    for (const Runway& rwy : airport.runways) {
        const RunwayInfo info = makeRunwayInfo(airport, rwy);
        if (!info.primary) continue;
        runways_.push_back(info);
        runwaySet_.add(runwayFlattenArea(info));
        wideSet_.add(runwayFlattenArea(info, 50.0, 100.0, kFieldBlendM));
    }
}

double TaxiwayField::naturalM(double latDeg, double lonDeg) const {
    if (runways_.empty()) return airportElevM_;
    const LocalFrame frame(latDeg, lonDeg);
    const Pt2 here{0, 0};
    double sumW = 0, sumZ = 0;
    for (const RunwayInfo& r : runways_) {
        const double d2 =
            dist2ToSegment(here, frame.toLocal(r.grade[0]), frame.toLocal(r.grade[1]));
        const double w = 1.0 / (d2 + kFieldWeightM * kFieldWeightM);
        sumW += w;
        sumZ += w * runwayGradeElevM(r, latDeg, lonDeg);
    }
    return sumZ / sumW;
}

double TaxiwayField::elevM(double latDeg, double lonDeg) const {
    const auto natural = [&] { return static_cast<float>(naturalM(latDeg, lonDeg)); };
    for (size_t i = 0; i < runwaySet_.size(); ++i)
        if (runwaySet_.distanceM(i, latDeg, lonDeg) <= 0)
            return runwaySet_.elevationM(latDeg, lonDeg, natural);
    return wideSet_.elevationM(latDeg, lonDeg, natural);
}

// ---- flatten areas -----------------------------------------------------------------------------

std::vector<FlattenArea> taxiwayFlattenAreas(const Airport& airport) {
    std::vector<FlattenArea> out;
    if (airport.taxiways.empty() && airport.aprons.empty()) return out;
    const TaxiwayField field(airport);
    const LocalFrame frame(airport.latDeg, airport.lonDeg);
    std::vector<Pt2> pts;

    for (const Taxiway& t : airport.taxiways) {
        pts.clear();
        for (const LatLon& p : t.points) pts.push_back(frame.toLocal(p));
        const std::vector<int> keep = simplifyPolyline(pts, kTaxiwaySimplifyM);
        const double reachM = std::max(0.5 * static_cast<double>(t.widthM), 1.0) + kTaxiwayMarginM;
        for (size_t k = 0; k + 1 < keep.size(); ++k) {
            const size_t i0 = static_cast<size_t>(keep[k]), i1 = static_cast<size_t>(keep[k + 1]);
            if (dist2(pts[i0], pts[i1]) < kMinTaxiwayPieceM * kMinTaxiwayPieceM) continue;
            FlattenArea area = rectArea(frame, pts[i0], pts[i1], reachM, reachM);
            area.graded = true;
            for (int e = 0; e < 2; ++e) {
                const LatLon& p = t.points[e == 0 ? i0 : i1];
                area.gradeLatDeg[e] = p.latDeg;
                area.gradeLonDeg[e] = p.lonDeg;
                area.gradeElevM[e] = static_cast<float>(field.elevM(p.latDeg, p.lonDeg));
            }
            area.elevM = 0.5f * (area.gradeElevM[0] + area.gradeElevM[1]);
            area.blendM = kTaxiwayBlendM;
            area.priority = kTaxiwayPriority;
            out.push_back(area);
        }
    }

    std::vector<Pt2> ring;
    std::vector<std::array<int, 3>> tris;
    for (const Apron& apron : airport.aprons) {
        pts.clear();
        for (const LatLon& p : apron.points) pts.push_back(frame.toLocal(p));
        const std::vector<int> keep = simplifyRing(pts, kApronSimplifyM);
        ring.clear();
        double sumZ = 0;
        for (const int i : keep) {
            ring.push_back(pts[static_cast<size_t>(i)]);
            const LatLon& p = apron.points[static_cast<size_t>(i)];
            sumZ += field.elevM(p.latDeg, p.lonDeg);
        }
        if (!triangulatePolygon(ring, tris)) {
            LOG_WARN("airport %s: apron %lld is not a simple polygon; no flat ground for it",
                     airport.ident.c_str(), apron.osmWayId);
            continue;
        }
        const float elevM = static_cast<float>(sumZ / static_cast<double>(keep.size()));
        auto flat = [&](FlattenArea area) {
            area.elevM = elevM;
            area.blendM = kTaxiwayBlendM;
            area.priority = kApronPriority;
            out.push_back(area);
        };
        for (const auto& tri : tris) {
            FlattenArea area;
            for (int c = 0; c < 4; ++c) {  // a triangle: the 4th corner repeats the 3rd
                const LatLon ll = frame.toLatLon(ring[static_cast<size_t>(tri[std::min(c, 2)])]);
                area.latDeg[c] = ll.latDeg;
                area.lonDeg[c] = ll.lonDeg;
            }
            flat(area);
        }
        for (size_t i = 0; i < ring.size(); ++i) {
            const Pt2& q0 = ring[i];
            const Pt2& q1 = ring[(i + 1) % ring.size()];
            if (dist2(q0, q1) < 1.0) continue;
            flat(rectArea(frame, q0, q1, kApronMarginM, kApronMarginM));
        }
    }
    return out;
}

// ---- meshes (W8c) ------------------------------------------------------------------------------

namespace {

constexpr float kTaxiRgb[3] = {0.33f, 0.33f, 0.35f};      // paved: the generic apron's colour
constexpr float kTaxiSoftRgb[3] = {0.30f, 0.35f, 0.22f};  // grass and dirt
constexpr float kTaxiLineRgb[3] = {0.85f, 0.68f, 0.12f};  // yellow markings

Pt2 add(const Pt2& a, const Pt2& b, double k = 1.0) {
    return {a.eM + b.eM * k, a.nM + b.nM * k};
}
Pt2 sub(const Pt2& a, const Pt2& b) {
    return {a.eM - b.eM, a.nM - b.nM};
}
Pt2 lerp(const Pt2& a, const Pt2& b, double t) {
    return {a.eM + (b.eM - a.eM) * t, a.nM + (b.nM - a.nM) * t};
}
double len(const Pt2& v) {
    return std::hypot(v.eM, v.nM);
}

struct Box2 {
    double minE = 1e300, minN = 1e300, maxE = -1e300, maxN = -1e300;
    void grow(const Pt2& p) {
        minE = std::min(minE, p.eM);
        maxE = std::max(maxE, p.eM);
        minN = std::min(minN, p.nM);
        maxN = std::max(maxN, p.nM);
    }
    bool overlaps(const Box2& o) const {
        return minE <= o.maxE && o.minE <= maxE && minN <= o.maxN && o.minN <= maxN;
    }
};

template <class It> Box2 boxOf(It first, It last) {
    Box2 b;
    for (; first != last; ++first) b.grow(*first);
    return b;
}

// Sutherland-Hodgman against the line a -> b: keeps the left side (cross >= 0) or the right side.
ConvexPoly clipHalf(const ConvexPoly& poly, const Pt2& a, const Pt2& b, bool keepLeft) {
    ConvexPoly out;
    const size_t n = poly.size();
    for (size_t i = 0; i < n; ++i) {
        const Pt2& p = poly[i];
        const Pt2& q = poly[(i + 1) % n];
        const double sp = cross(a, b, p), sq = cross(a, b, q);
        const bool inP = keepLeft ? sp >= 0 : sp <= 0;
        if (inP) out.push_back(p);
        // A vertex on the line is kept (or dropped) once; only a strict crossing adds a point.
        if ((sp < 0 && sq > 0) || (sp > 0 && sq < 0)) out.push_back(lerp(p, q, sp / (sp - sq)));
    }
    return out;
}

// Turns 2D geometry into surface-following ECEF triangles, cutting out the runway rectangles.
class Emitter {
public:
    Emitter(const LocalFrame& frame, const SurfaceElevFn& surface,
            const std::vector<std::array<Pt2, 4>>& rects, SurfaceMeshBuilder& builder, double liftM)
        : frame_(frame), surface_(surface), builder_(builder), liftM_(liftM) {
        for (const auto& r : rects) {
            rects_.push_back(r);
            rectBoxes_.push_back(boxOf(r.begin(), r.end()));
        }
    }

    Ecef ecef(const Pt2& p) const {
        const LatLon ll = frame_.toLatLon(p);
        GeoPos g;
        g.latDeg = ll.latDeg;
        g.lonDeg = ll.lonDeg;
        g.altMslM = surface_(ll.latDeg, ll.lonDeg) + liftM_;
        return geodeticToEcef(g);
    }

    // A convex polygon, minus the runway rectangles.
    void convex(const ConvexPoly& poly, const float rgb[3]) {
        pieces_.assign(1, poly);
        const Box2 box = boxOf(poly.begin(), poly.end());
        for (size_t r = 0; r < rects_.size(); ++r) {
            if (!box.overlaps(rectBoxes_[r])) continue;
            next_.clear();
            for (const ConvexPoly& piece : pieces_) subtractQuad(piece, rects_[r], next_);
            pieces_.swap(next_);
        }
        for (const ConvexPoly& piece : pieces_) {
            if (piece.size() < 3 || std::fabs(ringArea(piece)) < 1e-4) continue;
            ecefs_.clear();
            for (const Pt2& p : piece) ecefs_.push_back(ecef(p));
            builder_.addConvex(ecefs_.data(), static_cast<int>(ecefs_.size()), rgb);
        }
    }

    // A strip of cross-sections: runs of quads clear of the runways become shared-vertex
    // ribbons; a quad that reaches a runway is cut and drawn on its own.
    void strip(const std::vector<Pt2>& left, const std::vector<Pt2>& right, const float rgb[3]) {
        std::vector<Ecef> runL, runR;
        auto flush = [&] {
            if (runL.size() >= 2)
                builder_.addRibbon(runL.data(), runR.data(), static_cast<int>(runL.size()), rgb);
            runL.clear();
            runR.clear();
        };
        for (size_t i = 0; i + 1 < left.size(); ++i) {
            const ConvexPoly quad = {left[i], right[i], right[i + 1], left[i + 1]};
            const Box2 box = boxOf(quad.begin(), quad.end());
            bool nearRunway = false;
            for (const Box2& rb : rectBoxes_) nearRunway = nearRunway || box.overlaps(rb);
            if (nearRunway) {
                flush();
                convex(quad, rgb);
                continue;
            }
            if (runL.empty()) {
                runL.push_back(ecef(left[i]));
                runR.push_back(ecef(right[i]));
            }
            runL.push_back(ecef(left[i + 1]));
            runR.push_back(ecef(right[i + 1]));
        }
        flush();
    }

private:
    const LocalFrame& frame_;
    const SurfaceElevFn& surface_;
    SurfaceMeshBuilder& builder_;
    double liftM_;
    std::vector<std::array<Pt2, 4>> rects_;
    std::vector<Box2> rectBoxes_;
    std::vector<ConvexPoly> pieces_, next_;
    std::vector<Ecef> ecefs_;
};

struct Strip {
    std::vector<Pt2> left, right;
    std::vector<int> quadPiece;  // the piece each quad (section i -> i + 1) belongs to
};

// Cross-sections along a polyline of half-width h: square ends, quads at most maxQuadM long,
// mitre joins at bends under mitreMaxRad (the mitre at most mitreLimit x h), otherwise a square
// break (a new strip) whose point goes to `sharpBends`.
void polylineStrips(const std::vector<Pt2>& p, double h, double maxQuadM, double mitreMaxRad,
                    double mitreLimit, int pieceBase, std::vector<Strip>& strips,
                    std::vector<Pt2>* sharpBends) {
    const size_t n = p.size();
    if (n < 2) return;
    std::vector<Pt2> u(n - 1), r(n - 1);
    for (size_t i = 0; i + 1 < n; ++i) {
        const Pt2 d = sub(p[i + 1], p[i]);
        const double l = std::max(len(d), 1e-9);
        u[i] = {d.eM / l, d.nM / l};
        r[i] = {u[i].nM, -u[i].eM};  // right of the direction
    }
    Strip cur;
    auto section = [&](const Pt2& c, const Pt2& rr, double hh) {
        cur.left.push_back(add(c, rr, -hh));
        cur.right.push_back(add(c, rr, hh));
    };
    section(p[0], r[0], h);
    for (size_t i = 0; i + 1 < n; ++i) {
        const double l = len(sub(p[i + 1], p[i]));
        const int m = std::max(1, static_cast<int>(std::ceil(l / maxQuadM)));
        for (int k = 1; k < m; ++k) {
            section(add(p[i], u[i], l * k / m), r[i], h);
            cur.quadPiece.push_back(pieceBase + static_cast<int>(i));
        }
        cur.quadPiece.push_back(pieceBase + static_cast<int>(i));
        if (i + 2 == n) {
            section(p[i + 1], r[i], h);
            break;
        }
        const double cosT =
            std::max(-1.0, std::min(1.0, u[i].eM * u[i + 1].eM + u[i].nM * u[i + 1].nM));
        if (std::acos(cosT) < mitreMaxRad) {
            Pt2 rm = add(r[i], r[i + 1]);
            const double rl = len(rm);
            rm = rl > 1e-9 ? Pt2{rm.eM / rl, rm.nM / rl} : r[i];
            const double c = rm.eM * r[i].eM + rm.nM * r[i].nM;
            section(p[i + 1], rm, h / std::max(c, 1.0 / mitreLimit));
        } else {
            section(p[i + 1], r[i], h);
            strips.push_back(std::move(cur));
            cur = Strip{};
            section(p[i + 1], r[i + 1], h);
            if (sharpBends) sharpBends->push_back(p[i + 1]);
        }
    }
    strips.push_back(std::move(cur));
}

// A polygon around a circle of radius r (its edges touch the circle, so it covers the disc).
ConvexPoly fillet(const Pt2& c, double r) {
    ConvexPoly out;
    const double rr = r / std::cos(kPi / kFilletSegments);
    for (int k = 0; k < kFilletSegments; ++k) {
        const double a = (k + 0.5) * 2.0 * kPi / kFilletSegments;
        out.push_back({c.eM + rr * std::cos(a), c.nM + rr * std::sin(a)});
    }
    return out;
}

// Taxiway points in the frame without repeats, simplified for drawing.
std::vector<Pt2> drawnPolyline(const Taxiway& t, const LocalFrame& frame) {
    std::vector<Pt2> raw;
    for (const LatLon& ll : t.points) {
        const Pt2 q = frame.toLocal(ll);
        if (raw.empty() || dist2(raw.back(), q) > 0.0025) raw.push_back(q);
    }
    std::vector<Pt2> out;
    for (const int i : simplifyPolyline(raw, kTaxiwayRenderSimplifyM))
        out.push_back(raw[static_cast<size_t>(i)]);
    return out;
}

// Node key for shared segment ends (OSM shared nodes have identical coordinates).
std::pair<long long, long long> nodeKey(const LatLon& p) {
    return {std::llround(p.latDeg * 1e7), std::llround(p.lonDeg * 1e7)};
}

int countTriangles(const std::vector<SurfaceMesh>& meshes) {
    size_t n = 0;
    for (const SurfaceMesh& m : meshes) n += m.indices.size() / 3;
    return static_cast<int>(n);
}

}  // namespace

void subtractQuad(const ConvexPoly& poly, const std::array<Pt2, 4>& quad,
                  std::vector<ConvexPoly>& out) {
    ConvexPoly rest = poly;
    for (int e = 0; e < 4 && rest.size() >= 3; ++e) {
        const Pt2& a = quad[static_cast<size_t>(e)];
        const Pt2& b = quad[static_cast<size_t>((e + 1) % 4)];
        ConvexPoly outside = clipHalf(rest, a, b, false);
        if (outside.size() >= 3 && std::fabs(ringArea(outside)) > 1e-6)
            out.push_back(std::move(outside));
        rest = clipHalf(rest, a, b, true);
    }
}

std::vector<std::array<Pt2, 4>> runwayRectsLocal(const Airport& airport, const LocalFrame& frame) {
    std::vector<std::array<Pt2, 4>> out;
    for (const Runway& rwy : airport.runways) {
        const RunwayInfo info = makeRunwayInfo(airport, rwy);
        if (!info.primary || !info.hard) continue;
        const RunwayFrame rf(info);
        const double a0 = -info.displacedM, a1 = info.usableM(), l = 0.5 * info.widthM;
        const double along[4] = {a0, a0, a1, a1}, lateral[4] = {-l, l, l, -l};
        std::array<Pt2, 4> q;
        for (int i = 0; i < 4; ++i)
            q[static_cast<size_t>(i)] = frame.toLocal(rf.latLon(along[i], lateral[i]));
        if (ringArea(std::vector<Pt2>(q.begin(), q.end())) < 0) std::swap(q[1], q[3]);
        out.push_back(q);
    }
    return out;
}

bool pointInRing(const std::vector<Pt2>& ring, const Pt2& p) {
    bool in = false;
    for (size_t i = 0, j = ring.size() - 1; i < ring.size(); j = i++) {
        const Pt2& a = ring[i];
        const Pt2& b = ring[j];
        if ((a.nM > p.nM) != (b.nM > p.nM) &&
            p.eM < (b.eM - a.eM) * (p.nM - a.nM) / (b.nM - a.nM) + a.eM)
            in = !in;
    }
    return in;
}

bool pointInQuad(const std::array<Pt2, 4>& q, const Pt2& p) {
    for (int e = 0; e < 4; ++e)
        if (cross(q[static_cast<size_t>(e)], q[static_cast<size_t>((e + 1) % 4)], p) < 0)
            return false;
    return true;
}

bool clearOfTaxiways(const Airport& airport, const LatLon* corners, int n, double marginM) {
    const LocalFrame frame(airport.latDeg, airport.lonDeg);
    std::vector<Pt2> ring;
    for (const Apron& apron : airport.aprons) {
        ring.clear();
        for (const LatLon& ll : apron.points) ring.push_back(frame.toLocal(ll));
        for (int i = 0; i < n; ++i)
            if (ring.size() >= 3 && pointInRing(ring, frame.toLocal(corners[i]))) return false;
    }
    const double m2 = marginM * marginM;
    for (const Taxiway& t : airport.taxiways) {
        for (size_t k = 0; k + 1 < t.points.size(); ++k) {
            const Pt2 a = frame.toLocal(t.points[k]), b = frame.toLocal(t.points[k + 1]);
            for (int i = 0; i < n; ++i)
                if (dist2ToSegment(frame.toLocal(corners[i]), a, b) < m2) return false;
        }
    }
    return true;
}

bool taxiwaysInRange(const Ecef& eye, const Ecef& centerEcef, double radiusM) {
    const double dx = eye.x - centerEcef.x, dy = eye.y - centerEcef.y, dz = eye.z - centerEcef.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz) - radiusM < kTaxiwayRangeM;
}

TaxiwayMeshes buildTaxiwayMeshes(const Airport& airport, const SurfaceElevFn& surface) {
    TaxiwayMeshes out;
    if (airport.taxiways.empty() && airport.aprons.empty()) return out;
    const LocalFrame frame(airport.latDeg, airport.lonDeg);
    GeoPos ref;
    ref.latDeg = airport.latDeg;
    ref.lonDeg = airport.lonDeg;
    ref.altMslM = ftToM(airport.elevFt);
    const Ecef anchor = geodeticToEcef(ref);
    const std::vector<std::array<Pt2, 4>> rects = runwayRectsLocal(airport, frame);
    SurfaceMeshBuilder paveB(anchor), markB(anchor);
    Emitter pave(frame, surface, rects, paveB, kTaxiwayPavementLiftM);
    Emitter mark(frame, surface, rects, markB, kTaxiwayMarkingLiftM);
    Box2 extent;

    // Segment ends shared by two or more segments get a round fillet (radius: the widest).
    std::map<std::pair<long long, long long>, std::pair<int, double>> ends;
    for (const Taxiway& t : airport.taxiways) {
        if (t.points.size() < 2) continue;
        const double h = 0.5 * std::max(static_cast<double>(t.widthM), 3.0);
        for (const LatLon* ll : {&t.points.front(), &t.points.back()}) {
            auto& e = ends[nodeKey(*ll)];
            ++e.first;
            e.second = std::max(e.second, h);
        }
    }

    // Pavement ribbons; the pieces (for the edge-line test) and the fillets.
    struct Piece {
        Pt2 a, b;
        double h;
        int strip = -1;  // pieces joined by mitres share a strip (for the edge-line test)
    };
    std::vector<Piece> pieces;
    std::vector<std::pair<Pt2, double>> discs;
    struct SegmentStrips {
        std::vector<Strip> strips;
        double h;
    };
    std::vector<SegmentStrips> segs;
    int stripCount = 0;
    for (const Taxiway& t : airport.taxiways) {
        const std::vector<Pt2> pts = drawnPolyline(t, frame);
        if (pts.size() < 2) continue;
        const double h = 0.5 * std::max(static_cast<double>(t.widthM), 3.0);
        const float* rgb = t.hard ? kTaxiRgb : kTaxiSoftRgb;
        SegmentStrips seg;
        seg.h = h;
        std::vector<Pt2> bends;
        const int base = static_cast<int>(pieces.size());
        for (size_t i = 0; i + 1 < pts.size(); ++i) pieces.push_back({pts[i], pts[i + 1], h});
        polylineStrips(pts, h, kTaxiwayMaxQuadM, kMitreMaxBendDeg * kRadPerDeg, 2.0, base,
                       seg.strips, &bends);
        for (const Strip& st : seg.strips) {
            for (const int q : st.quadPiece) pieces[static_cast<size_t>(q)].strip = stripCount;
            ++stripCount;
        }
        for (const Strip& s : seg.strips) pave.strip(s.left, s.right, rgb);
        for (const Pt2& b : bends) {
            pave.convex(fillet(b, h), rgb);
            discs.emplace_back(b, h);
        }
        for (const Pt2& q : pts) extent.grow(q);
        segs.push_back(std::move(seg));
    }
    for (const auto& [key, e] : ends) {
        if (e.first < 2) continue;
        LatLon ll;
        ll.latDeg = static_cast<double>(key.first) * 1e-7;
        ll.lonDeg = static_cast<double>(key.second) * 1e-7;
        const Pt2 c = frame.toLocal(ll);
        pave.convex(fillet(c, e.second), kTaxiRgb);
        discs.emplace_back(c, e.second);
    }

    // Aprons: ear-clipped, then cut on a 40 m grid so the triangles follow the surface.
    std::vector<std::vector<Pt2>> apronRings;
    std::vector<Box2> apronBoxes;
    std::vector<std::array<int, 3>> tris;
    for (const Apron& apron : airport.aprons) {
        std::vector<Pt2> raw;
        for (const LatLon& ll : apron.points) raw.push_back(frame.toLocal(ll));
        std::vector<Pt2> ring;
        for (const int i : simplifyRing(raw, kTaxiwayRenderSimplifyM))
            ring.push_back(raw[static_cast<size_t>(i)]);
        if (!triangulatePolygon(ring, tris)) continue;
        const float* rgb = apron.hard ? kTaxiRgb : kTaxiSoftRgb;
        for (const auto& tri : tris) {
            const ConvexPoly t = {ring[static_cast<size_t>(tri[0])],
                                  ring[static_cast<size_t>(tri[1])],
                                  ring[static_cast<size_t>(tri[2])]};
            const Box2 b = boxOf(t.begin(), t.end());
            const long long gx0 = static_cast<long long>(std::floor(b.minE / kApronGridM));
            const long long gx1 = static_cast<long long>(std::floor(b.maxE / kApronGridM));
            const long long gy0 = static_cast<long long>(std::floor(b.minN / kApronGridM));
            const long long gy1 = static_cast<long long>(std::floor(b.maxN / kApronGridM));
            for (long long gy = gy0; gy <= gy1; ++gy) {
                for (long long gx = gx0; gx <= gx1; ++gx) {
                    const double e0 = gx * kApronGridM, n0 = gy * kApronGridM;
                    const Pt2 c[4] = {{e0, n0},
                                      {e0 + kApronGridM, n0},
                                      {e0 + kApronGridM, n0 + kApronGridM},
                                      {e0, n0 + kApronGridM}};
                    ConvexPoly piece = t;
                    for (int k = 0; k < 4 && piece.size() >= 3; ++k)
                        piece = clipHalf(piece, c[k], c[(k + 1) % 4], true);
                    if (piece.size() >= 3 && std::fabs(ringArea(piece)) > 0.01)
                        pave.convex(piece, rgb);
                }
            }
        }
        for (const Pt2& q : ring) extent.grow(q);
        apronBoxes.push_back(boxOf(ring.begin(), ring.end()));
        apronRings.push_back(std::move(ring));
    }

    // Centerlines: thin ribbons with mitres up to 120 degree bends.
    for (const Taxiway& t : airport.taxiways) {
        const std::vector<Pt2> pts = drawnPolyline(t, frame);
        if (pts.size() < 2) continue;
        std::vector<Strip> strips;
        polylineStrips(pts, kCenterlineHalfWidthM, kTaxiwayMaxQuadM, 120.0 * kRadPerDeg, 2.0, 0,
                       strips, nullptr);
        for (const Strip& s : strips) mark.strip(s.left, s.right, kTaxiLineRgb);
    }

    // Edge lines along both ribbon edges, dropped where they lie inside other pavement (another
    // piece, a fillet, an apron) or a runway. Pieces are indexed on a 50 m grid for the test.
    constexpr double kCell = 50.0;
    std::unordered_map<long long, std::vector<int>> grid;
    auto cellKey = [](long long x, long long y) { return (x << 32) ^ (y & 0xffffffffLL); };
    for (int i = 0; i < static_cast<int>(pieces.size()); ++i) {
        const Piece& pc = pieces[static_cast<size_t>(i)];
        const long long x0 =
            static_cast<long long>(std::floor((std::min(pc.a.eM, pc.b.eM) - pc.h) / kCell));
        const long long x1 =
            static_cast<long long>(std::floor((std::max(pc.a.eM, pc.b.eM) + pc.h) / kCell));
        const long long y0 =
            static_cast<long long>(std::floor((std::min(pc.a.nM, pc.b.nM) - pc.h) / kCell));
        const long long y1 =
            static_cast<long long>(std::floor((std::max(pc.a.nM, pc.b.nM) + pc.h) / kCell));
        for (long long y = y0; y <= y1; ++y)
            for (long long x = x0; x <= x1; ++x) grid[cellKey(x, y)].push_back(i);
    }
    auto covered = [&](const Pt2& e, const Pt2& inner, int ownPiece) {
        for (const auto& r : rects)
            if (pointInQuad(r, e) || pointInQuad(r, inner)) return true;
        const auto it = grid.find(cellKey(static_cast<long long>(std::floor(e.eM / kCell)),
                                          static_cast<long long>(std::floor(e.nM / kCell))));
        if (it != grid.end()) {
            const int ownStrip = pieces[static_cast<size_t>(ownPiece)].strip;
            for (const int i : it->second) {
                const Piece& pc = pieces[static_cast<size_t>(i)];
                // The piece itself and its mitred neighbours: their edges meet the sample's edge
                // at the mitre, so they never cover it.
                if (i == ownPiece || (pc.strip == ownStrip && std::abs(i - ownPiece) <= 1))
                    continue;
                const double lim = pc.h - 0.05;
                if (lim > 0 && dist2ToSegment(e, pc.a, pc.b) < lim * lim) return true;
            }
        }
        for (const auto& [c, r] : discs)
            if (dist2(e, c) < (r - 0.05) * (r - 0.05)) return true;
        for (size_t a = 0; a < apronRings.size(); ++a) {
            const Box2& b = apronBoxes[a];
            if (e.eM < b.minE || e.eM > b.maxE || e.nM < b.minN || e.nM > b.maxN) continue;
            if (pointInRing(apronRings[a], e)) return true;
        }
        return false;
    };
    // Coverage is tested every 5 m, but a run keeps only its ends and the section points (at most
    // 40 m apart, where the edge may turn), so straight edges cost two triangles per section.
    std::vector<Pt2> runE, runI, keptE, keptI;
    std::vector<char> runCorner;
    for (const SegmentStrips& seg : segs) {
        const double frac = kEdgeLineWidthM / (2.0 * seg.h), nudge = 0.1 / (2.0 * seg.h);
        for (const Strip& s : seg.strips) {
            for (int side = 0; side < 2; ++side) {
                const std::vector<Pt2>& edge = side == 0 ? s.left : s.right;
                const std::vector<Pt2>& other = side == 0 ? s.right : s.left;
                runE.clear();
                runI.clear();
                runCorner.clear();
                auto flush = [&] {
                    if (runE.size() >= 2) {
                        keptE.clear();
                        keptI.clear();
                        for (size_t q = 0; q < runE.size(); ++q) {
                            if (q == 0 || q + 1 == runE.size() || runCorner[q]) {
                                keptE.push_back(runE[q]);
                                keptI.push_back(runI[q]);
                            }
                        }
                        mark.strip(keptE, keptI, kTaxiLineRgb);
                    }
                    runE.clear();
                    runI.clear();
                    runCorner.clear();
                };
                for (size_t k = 0; k + 1 < edge.size(); ++k) {
                    const double l = len(sub(edge[k + 1], edge[k]));
                    const int m = std::max(1, static_cast<int>(std::ceil(l / kEdgeSampleM)));
                    for (int j = (k == 0 ? 0 : 1); j <= m; ++j) {
                        const double tt = static_cast<double>(j) / m;
                        const Pt2 e = lerp(edge[k], edge[k + 1], tt);
                        const Pt2 o = lerp(other[k], other[k + 1], tt);
                        const Pt2 in = lerp(e, o, frac);
                        if (covered(lerp(e, o, nudge), in, s.quadPiece[k])) {
                            flush();
                        } else {
                            runE.push_back(e);
                            runI.push_back(in);
                            runCorner.push_back(j == 0 || j == m);
                        }
                    }
                }
                flush();
            }
        }
    }

    out.pavement = paveB.finish();
    out.markings = markB.finish();
    out.pavementTriangles = countTriangles(out.pavement);
    out.markingTriangles = countTriangles(out.markings);
    if (extent.minE <= extent.maxE) {
        const Pt2 c{0.5 * (extent.minE + extent.maxE), 0.5 * (extent.minN + extent.maxN)};
        const LatLon ll = frame.toLatLon(c);
        GeoPos g;
        g.latDeg = ll.latDeg;
        g.lonDeg = ll.lonDeg;
        g.altMslM = ftToM(airport.elevFt);
        out.centerEcef = geodeticToEcef(g);
        out.radiusM = 0.5 * std::hypot(extent.maxE - extent.minE, extent.maxN - extent.minN) + 50.0;
    }
    return out;
}

}  // namespace fsim::world
