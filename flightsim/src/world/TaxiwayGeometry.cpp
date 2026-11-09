#include "world/TaxiwayGeometry.h"

#include <algorithm>
#include <cmath>

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

}  // namespace fsim::world
