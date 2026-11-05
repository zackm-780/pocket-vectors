#include "world/ExternalViewModel.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <utility>

#include "core/Units.h"

namespace fsim::world {
namespace {

struct V3 {
    float x = 0, y = 0, z = 0;
};
V3 operator+(V3 a, V3 b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}
V3 operator-(V3 a, V3 b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
V3 operator*(V3 a, float k) {
    return {a.x * k, a.y * k, a.z * k};
}
V3 cross(V3 a, V3 b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
float dot(V3 a, V3 b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
V3 normalized(V3 a) {
    const float l = std::sqrt(dot(a, a));
    return l > 1e-9f ? a * (1.0f / l) : V3{0, 0, -1};
}

struct Rgb {
    float r, g, b;
};
constexpr Rgb kWhite{0.90f, 0.91f, 0.93f};
constexpr Rgb kWingGrey{0.74f, 0.76f, 0.79f};
constexpr Rgb kDark{0.46f, 0.47f, 0.50f};
constexpr Rgb kTire{0.07f, 0.07f, 0.08f};
constexpr Rgb kWindow{0.12f, 0.17f, 0.24f};
constexpr Rgb kStrut{0.55f, 0.56f, 0.58f};

enum class Engines { NoseProp, WingProps, UnderWing, RearFuselage };

// Template dimensions, meters, body axes (x fwd, y right, z down), origin at the CG.
struct Spec {
    // fuselage cross-sections: x, center z, half width, half height
    struct Ring {
        float x, cz, hw, hh;
    };
    Ring rings[8];
    int ringCount;
    int windowSegment;  // loft segment whose upper faces are the cockpit glass
    float spanM, rootChordM, tipChordM, sweepDeg, dihedralDeg, wingLeX, wingZ, wingThickM;
    float hTailSpanM, hTailChordM, hTailLeX, hTailZ, hTailSweepDeg;
    float finHeightM, finRootChordM, finTipChordM, finLeX, finBaseZ, finSweepDeg;
    bool tTail;
    Engines engines;
    float engineY[2];  // inner, outer engine y (UnderWing / WingProps / RearFuselage)
    float nacelleLenM, nacelleRadM, nacelleAheadM, nacelleZ;  // ahead of the local wing LE
    float gearHeightM, mainGearX, mainGearY, noseGearX, wheelRadM;
    float flapMaxDeg;
    Rgb accent;
};

const Spec& specFor(ExternalModelKind kind) {
    // clang-format off
    // Field order per row: rings (x, cz, hw, hh) / ring count / glass segment / wing: span,
    // root chord, tip chord, sweep, dihedral, root LE x, z, thickness / h-tail: span, chord, LE x,
    // z, sweep / fin: height, root chord, tip chord, LE x, base z, sweep / T-tail / engines /
    // engine y (inner, outer) / nacelle: length, radius, ahead of the local LE, z / gear: height,
    // main x, main y, nose x, wheel radius / max flap deg / accent color.
    // Cessna 172 class: high wing, nose prop, fixed gear.
    static const Spec kSingle{
        {{2.65f, 0.05f, 0.12f, 0.12f}, {2.35f, 0.0f, 0.45f, 0.45f}, {1.30f, -0.1f, 0.55f, 0.68f},
         {0.10f, -0.1f, 0.55f, 0.70f}, {-1.40f, -0.15f, 0.42f, 0.52f},
         {-3.80f, -0.25f, 0.20f, 0.28f}, {-5.60f, -0.35f, 0.08f, 0.14f}},
        7, 1,
        11.0f, 1.55f, 1.15f, 0.0f, 1.7f, 0.55f, -0.80f, 0.16f,
        3.4f, 1.1f, -4.6f, -0.25f, 5.0f,
        1.45f, 1.6f, 0.8f, -4.2f, -0.35f, 35.0f,
        false, Engines::NoseProp, {0, 0},
        0, 0, 0, 0,
        1.25f, -0.45f, 1.25f, 1.95f, 0.22f,
        30.0f, {0.75f, 0.12f, 0.12f}};
    // Cessna 310 class: low wing, two wing nacelles with props, retractable gear.
    static const Spec kTwin{
        {{3.55f, 0.05f, 0.12f, 0.12f}, {3.10f, 0.0f, 0.45f, 0.45f}, {1.80f, -0.05f, 0.62f, 0.70f},
         {-0.40f, -0.05f, 0.62f, 0.70f}, {-2.20f, -0.10f, 0.45f, 0.55f},
         {-4.60f, -0.20f, 0.22f, 0.30f}, {-6.10f, -0.30f, 0.09f, 0.14f}},
        7, 1,
        11.3f, 2.0f, 1.2f, 2.0f, 5.0f, 0.75f, 0.45f, 0.24f,
        5.0f, 1.3f, -5.2f, -0.30f, 8.0f,
        1.8f, 1.9f, 1.0f, -5.0f, -0.40f, 38.0f,
        false, Engines::WingProps, {2.0f, 0},
        2.7f, 0.42f, 1.0f, 0.25f,
        1.35f, -0.40f, 2.0f, 2.75f, 0.28f,
        35.0f, {0.10f, 0.30f, 0.60f}};
    // Global 5000 class: swept low wing, T-tail, two rear-fuselage engines.
    static const Spec kBizjet{
        {{14.2f, 0.25f, 0.15f, 0.15f}, {12.8f, 0.10f, 0.95f, 1.00f}, {10.4f, 0.0f, 1.35f, 1.40f},
         {-4.0f, 0.0f, 1.35f, 1.40f}, {-9.0f, -0.20f, 1.05f, 1.15f},
         {-13.4f, -0.55f, 0.45f, 0.55f}, {-15.3f, -0.80f, 0.12f, 0.15f}},
        7, 1,
        28.7f, 5.6f, 1.5f, 31.0f, 3.0f, 3.0f, 1.0f, 0.55f,
        9.6f, 2.6f, -14.8f, 0.0f, 33.0f,
        4.6f, 4.8f, 3.4f, -11.0f, -1.0f, 42.0f,
        true, Engines::RearFuselage, {2.25f, 0},
        5.2f, 0.78f, 0.0f, -0.95f,
        2.05f, -1.2f, 2.6f, 11.6f, 0.45f,
        40.0f, {0.12f, 0.20f, 0.35f}};
    // 737 / A320 class: swept low wing, two engines under the wing.
    static const Spec kAirliner{
        {{19.4f, 0.45f, 0.15f, 0.15f}, {17.6f, 0.25f, 1.30f, 1.30f}, {14.5f, 0.0f, 1.90f, 2.00f},
         {-8.0f, 0.0f, 1.90f, 2.00f}, {-14.0f, -0.30f, 1.45f, 1.60f},
         {-18.8f, -0.90f, 0.60f, 0.70f}, {-20.4f, -1.20f, 0.18f, 0.20f}},
        7, 1,
        35.8f, 7.4f, 1.6f, 25.0f, 6.0f, 4.2f, 1.35f, 0.65f,
        14.4f, 4.0f, -15.6f, -0.60f, 30.0f,
        7.2f, 6.2f, 2.2f, -13.6f, -1.5f, 35.0f,
        false, Engines::UnderWing, {4.95f, 0},
        4.6f, 1.05f, 2.8f, 2.15f,
        3.55f, -1.4f, 2.9f, 15.2f, 0.55f,
        40.0f, {0.10f, 0.22f, 0.52f}};
    // 787 / 747 / MD-11 class: large swept wing, two (or four) engines under the wing.
    static const Spec kHeavy{
        {{29.2f, 0.7f, 0.2f, 0.2f}, {26.8f, 0.4f, 2.0f, 2.0f}, {22.5f, 0.0f, 2.95f, 3.05f},
         {-12.0f, 0.0f, 2.95f, 3.05f}, {-21.0f, -0.45f, 2.20f, 2.40f},
         {-28.2f, -1.40f, 0.85f, 0.95f}, {-30.6f, -1.80f, 0.25f, 0.28f}},
        7, 1,
        60.0f, 11.5f, 2.6f, 32.0f, 6.0f, 6.0f, 2.1f, 0.9f,
        21.0f, 6.2f, -24.0f, -0.9f, 34.0f,
        10.5f, 9.5f, 3.6f, -21.5f, -2.3f, 40.0f,
        false, Engines::UnderWing, {9.8f, 21.0f},
        6.6f, 1.7f, 3.4f, 3.3f,
        5.3f, -2.4f, 5.4f, 23.6f, 0.65f,
        38.0f, {0.12f, 0.20f, 0.45f}};
    // clang-format on
    switch (kind) {
    case ExternalModelKind::Single: return kSingle;
    case ExternalModelKind::Twin: return kTwin;
    case ExternalModelKind::Bizjet: return kBizjet;
    case ExternalModelKind::Airliner: return kAirliner;
    case ExternalModelKind::Heavy: return kHeavy;
    }
    return kSingle;
}

class Builder {
public:
    Builder(std::vector<float>& out, float scale) : out_(out), scale_(scale) {}

    // One triangle facing away from `center` (convex pieces): the normal points out and the
    // winding is counter-clockwise seen from outside, so back faces can be culled.
    void tri(V3 a, V3 b, V3 c, V3 center, Rgb col) {
        V3 n = normalized(cross(b - a, c - a));
        const V3 mid = (a + b + c) * (1.0f / 3.0f);
        if (dot(n, mid - center) < 0) {
            n = n * -1.0f;
            std::swap(b, c);
        }
        for (const V3& p : {a, b, c}) {
            out_.insert(out_.end(), {p.x * scale_, p.y * scale_, p.z * scale_, n.x, n.y, n.z, col.r,
                                     col.g, col.b});
        }
        ++triangles_;
    }
    void quad(V3 a, V3 b, V3 c, V3 d, V3 center, Rgb col) {
        tri(a, b, c, center, col);
        tri(a, c, d, center, col);
    }
    // Convex hexahedron: p[0..3] one face, p[4..7] the opposite face, same order.
    void hexa(const V3 (&p)[8], Rgb col) {
        V3 c{};
        for (const V3& v : p) c = c + v;
        c = c * 0.125f;
        quad(p[0], p[1], p[2], p[3], c, col);
        quad(p[4], p[5], p[6], p[7], c, col);
        for (int i = 0; i < 4; ++i) {
            const int j = (i + 1) % 4;
            quad(p[i], p[j], p[j + 4], p[i + 4], c, col);
        }
    }
    void box(V3 center, V3 half, Rgb col) {
        const V3 h = half;
        const V3 p[8] = {center + V3{-h.x, -h.y, -h.z}, center + V3{h.x, -h.y, -h.z},
                         center + V3{h.x, h.y, -h.z},   center + V3{-h.x, h.y, -h.z},
                         center + V3{-h.x, -h.y, h.z},  center + V3{h.x, -h.y, h.z},
                         center + V3{h.x, h.y, h.z},    center + V3{-h.x, h.y, h.z}};
        hexa(p, col);
    }
    // Octagonal loft along x through cross-sections (cy = lateral center). Segment
    // `glassSegment` gets glass on its upper faces.
    void loft(const Spec::Ring* rings, int n, float cy, Rgb col, int glassSegment, Rgb glass) {
        constexpr int kSides = 8;
        auto at = [&](const Spec::Ring& r, int k) {
            const float a =
                (static_cast<float>(k) + 0.5f) * 2.0f * static_cast<float>(kPi) / kSides;
            return V3{r.x, cy + r.hw * std::cos(a), r.cz + r.hh * std::sin(a)};
        };
        for (int s = 0; s + 1 < n; ++s) {
            const Spec::Ring& r0 = rings[s];
            const Spec::Ring& r1 = rings[s + 1];
            const V3 axis{(r0.x + r1.x) * 0.5f, cy, (r0.cz + r1.cz) * 0.5f};
            for (int k = 0; k < kSides; ++k) {
                const float a =
                    (static_cast<float>(k) + 1.0f) * 2.0f * static_cast<float>(kPi) / kSides;
                const bool upper = std::sin(a) < -0.1f;  // z up is negative
                const bool side = std::fabs(std::cos(a)) > 0.5f;
                const Rgb c = (s == glassSegment && upper && side) ? glass : col;
                quad(at(r0, k), at(r0, k + 1), at(r1, k + 1), at(r1, k), axis, c);
            }
        }
        // End caps (nose and tail cones close at small rings).
        for (int e = 0; e < 2; ++e) {
            const Spec::Ring& r = rings[e == 0 ? 0 : n - 1];
            const V3 c{r.x, cy, r.cz};
            const V3 inside{r.x + (e == 0 ? -1.0f : 1.0f), cy, r.cz};
            for (int k = 0; k < kSides; ++k) tri(c, at(r, k), at(r, k + 1), inside, col);
        }
    }
    int triangles() const { return triangles_; }

private:
    std::vector<float>& out_;
    float scale_;
    int triangles_ = 0;
};

// Leading edge x of a swept wing at |y| from the root at y0.
float leAt(float rootLeX, float sweepDeg, float y0, float y) {
    return rootLeX - (std::fabs(y) - y0) * std::tan(static_cast<float>(degToRad(sweepDeg)));
}

void addWing(Builder& b, const Spec& s, float side, float flapNorm) {
    const float y0 = s.rings[2].hw * 0.8f;  // root inside the fuselage
    const float semi = s.spanM * 0.5f;
    const float tanDih = std::tan(static_cast<float>(degToRad(s.dihedralDeg)));
    const float t = s.wingThickM * 0.5f;
    auto zAt = [&](float y) { return s.wingZ - (std::fabs(y) - y0) * tanDih; };
    auto chordAt = [&](float y) {
        const float f = (std::fabs(y) - y0) / std::max(0.1f, semi - y0);
        return s.rootChordM + (s.tipChordM - s.rootChordM) * f;
    };
    const float yr = side * y0, yt = side * semi;
    const float leR = s.wingLeX, leT = leAt(s.wingLeX, s.sweepDeg, y0, semi);
    const float zr = zAt(yr), zt = zAt(yt);
    const float tipT = t * 0.5f;
    const V3 p[8] = {{leR, yr, zr - t},
                     {leR - s.rootChordM, yr, zr - t * 0.3f},
                     {leT - s.tipChordM, yt, zt - tipT * 0.3f},
                     {leT, yt, zt - tipT},
                     {leR, yr, zr + t},
                     {leR - s.rootChordM, yr, zr + t * 0.3f},
                     {leT - s.tipChordM, yt, zt + tipT * 0.3f},
                     {leT, yt, zt + tipT}};
    b.hexa(p, kWingGrey);

    // Flaps: inboard 65 % of the semi-span, 28 % chord, hinged at the trailing edge.
    const float delta =
        static_cast<float>(degToRad(std::clamp(flapNorm, 0.0f, 1.0f) * s.flapMaxDeg));
    if (delta < static_cast<float>(degToRad(0.5))) return;
    const float yIn = side * (s.rings[2].hw + 0.1f), yOut = side * (0.65f * semi);
    const float cd = std::cos(delta), sd = std::sin(delta);
    auto hinge = [&](float y) {
        return V3{leAt(s.wingLeX, s.sweepDeg, y0, y) - chordAt(y), y, zAt(y) + t * 0.3f};
    };
    auto trailing = [&](float y) {
        const float c = 0.28f * chordAt(y);
        return hinge(y) + V3{-c * cd, 0, c * sd};
    };
    const float ft = t * 0.35f;
    const V3 hi = hinge(yIn), ho = hinge(yOut), ti = trailing(yIn), to = trailing(yOut);
    const V3 f[8] = {hi + V3{0, 0, -ft},       ti + V3{0, 0, -ft * 0.4f}, to + V3{0, 0, -ft * 0.4f},
                     ho + V3{0, 0, -ft},       hi + V3{0, 0, ft},         ti + V3{0, 0, ft * 0.4f},
                     to + V3{0, 0, ft * 0.4f}, ho + V3{0, 0, ft}};
    b.hexa(f, kWingGrey);
}

void addHTail(Builder& b, const Spec& s, float side, float z, float leX) {
    const float semi = s.hTailSpanM * 0.5f;
    const float tip = s.hTailChordM * 0.55f;
    const float t = std::max(0.05f, s.hTailChordM * 0.06f);
    const float leT = leAt(leX, s.hTailSweepDeg, 0, semi);
    const V3 p[8] = {{leX, 0, z - t},
                     {leX - s.hTailChordM, 0, z - t},
                     {leT - tip, side * semi, z - t},
                     {leT, side * semi, z - t},
                     {leX, 0, z + t},
                     {leX - s.hTailChordM, 0, z + t},
                     {leT - tip, side * semi, z + t},
                     {leT, side * semi, z + t}};
    b.hexa(p, kWingGrey);
}

void addFin(Builder& b, const Spec& s) {
    const float h = s.finHeightM;
    const float t = std::max(0.06f, s.finRootChordM * 0.05f);
    const float leTop = s.finLeX - h * std::tan(static_cast<float>(degToRad(s.finSweepDeg)));
    const float zb = s.finBaseZ, zt = s.finBaseZ - h;
    const V3 p[8] = {{s.finLeX, -t, zb},
                     {s.finLeX - s.finRootChordM, -t, zb},
                     {leTop - s.finTipChordM, -t, zt},
                     {leTop, -t, zt},
                     {s.finLeX, t, zb},
                     {s.finLeX - s.finRootChordM, t, zb},
                     {leTop - s.finTipChordM, t, zt},
                     {leTop, t, zt}};
    b.hexa(p, s.accent);
}

void addNacelle(Builder& b, float frontX, float y, float z, float len, float rad, Rgb col) {
    const Spec::Ring r[4] = {{frontX, z, rad * 0.92f, rad * 0.92f},
                             {frontX - len * 0.15f, z, rad, rad},
                             {frontX - len * 0.75f, z, rad * 0.9f, rad * 0.9f},
                             {frontX - len, z, rad * 0.55f, rad * 0.55f}};
    b.loft(r, 4, y, col, -1, col);
}

void addProp(Builder& b, float x, float y, float z, float radius) {
    b.box({x, y, z}, {0.12f, 0.12f, 0.12f}, kDark);  // spinner
    b.box({x - 0.05f, y, z}, {0.02f, radius, 0.07f}, kTire);
    b.box({x - 0.05f, y, z}, {0.02f, 0.07f, radius}, kTire);
}

void addGearLeg(Builder& b, float x, float y, float topZ, float groundZ, float wheelR,
                float gearNorm, bool twinWheels) {
    const float len = (groundZ - wheelR - topZ) * gearNorm;
    if (len <= 0.01f) return;
    const float wz = topZ + len;  // wheel center
    const float sw = std::max(0.04f, wheelR * 0.18f);
    b.box({x, y, topZ + len * 0.5f}, {sw, sw, len * 0.5f}, kStrut);
    const float ww = wheelR * 0.45f;
    if (twinWheels) {
        b.box({x, y - ww * 1.2f, wz}, {wheelR, ww * 0.5f, wheelR}, kTire);
        b.box({x, y + ww * 1.2f, wz}, {wheelR, ww * 0.5f, wheelR}, kTire);
    } else {
        b.box({x, y, wz}, {wheelR, ww * 0.5f, wheelR}, kTire);
    }
}

}  // namespace

ExternalModelKind externalModelFor(const AircraftProfile& p) {
    ExternalModelKind k;
    if (!p.externalModel.empty() && parseExternalModel(p.externalModel.c_str(), k)) return k;
    switch (p.category) {
    case AircraftCategory::Piston:
        return p.engineCount > 1 ? ExternalModelKind::Twin : ExternalModelKind::Single;
    case AircraftCategory::Turboprop: return ExternalModelKind::Twin;
    case AircraftCategory::Jet:
        return p.engineCount >= 3 ? ExternalModelKind::Heavy : ExternalModelKind::Airliner;
    }
    return ExternalModelKind::Single;
}

const char* externalModelName(ExternalModelKind kind) {
    switch (kind) {
    case ExternalModelKind::Single: return "single";
    case ExternalModelKind::Twin: return "twin";
    case ExternalModelKind::Bizjet: return "bizjet";
    case ExternalModelKind::Airliner: return "airliner";
    case ExternalModelKind::Heavy: return "heavy";
    }
    return "single";
}

bool parseExternalModel(const char* name, ExternalModelKind& out) {
    for (ExternalModelKind k :
         {ExternalModelKind::Single, ExternalModelKind::Twin, ExternalModelKind::Bizjet,
          ExternalModelKind::Airliner, ExternalModelKind::Heavy}) {
        if (std::strcmp(name, externalModelName(k)) == 0) {
            out = k;
            return true;
        }
    }
    return false;
}

ExternalModelInfo buildExternalModel(const ExternalModelParams& p, std::vector<float>& out) {
    out.clear();
    const Spec& s = specFor(p.kind);
    const float scale = p.spanM > 0 ? p.spanM / s.spanM : 1.0f;
    Builder b(out, scale);

    b.loft(s.rings, s.ringCount, 0, kWhite, s.windowSegment, kWindow);
    addWing(b, s, 1, p.flapNorm);
    addWing(b, s, -1, p.flapNorm);
    addFin(b, s);
    const float tailZ = s.tTail ? s.finBaseZ - s.finHeightM : s.hTailZ;
    const float tailLeX =
        s.tTail ? s.finLeX - s.finHeightM * std::tan(static_cast<float>(degToRad(s.finSweepDeg)))
                : s.hTailLeX;
    addHTail(b, s, 1, tailZ, tailLeX);
    addHTail(b, s, -1, tailZ, tailLeX);

    const float y0 = s.rings[2].hw * 0.8f;
    switch (s.engines) {
    case Engines::NoseProp: addProp(b, s.rings[0].x + 0.1f, 0, s.rings[0].cz, 0.95f); break;
    case Engines::WingProps:
        for (float side : {-1.0f, 1.0f}) {
            const float y = side * s.engineY[0];
            const float front = leAt(s.wingLeX, s.sweepDeg, y0, y) + s.nacelleAheadM;
            addNacelle(b, front, y, s.nacelleZ, s.nacelleLenM, s.nacelleRadM, kWhite);
            addProp(b, front + 0.1f, y, s.nacelleZ, 0.95f);
        }
        break;
    case Engines::UnderWing: {
        const bool four = p.engineCount >= 4 && s.engineY[1] > 0;
        for (float side : {-1.0f, 1.0f}) {
            for (int i = 0; i < (four ? 2 : 1); ++i) {
                const float y = side * s.engineY[i];
                const float front = leAt(s.wingLeX, s.sweepDeg, y0, y) + s.nacelleAheadM;
                addNacelle(b, front, y, s.nacelleZ, s.nacelleLenM, s.nacelleRadM, kDark);
            }
        }
        if (p.engineCount == 3)  // tail engine (MD-11 / DC-10 layout) at the fin root
            addNacelle(b, s.finLeX + s.nacelleLenM * 0.3f, 0, s.finBaseZ - s.nacelleRadM * 0.6f,
                       s.nacelleLenM, s.nacelleRadM * 0.9f, kDark);
        break;
    }
    case Engines::RearFuselage:
        for (float side : {-1.0f, 1.0f})
            addNacelle(b, s.finLeX + s.nacelleLenM * 0.6f, side * s.engineY[0], s.nacelleZ,
                       s.nacelleLenM, s.nacelleRadM, kDark);
        break;
    }

    // Gear: the main legs hang from the wing (low wing) or the fuselage floor; the wheels touch
    // the ground at the template's gear height below the origin when fully down (the whole model
    // moves afterwards so that this matches the aircraft's own height on its wheels).
    const float gearH = s.gearHeightM;
    const float gearNorm = std::clamp(p.gearNorm, 0.0f, 1.0f);
    if (gearNorm > 0.02f) {
        const bool twinWheels = p.kind == ExternalModelKind::Airliner ||
                                p.kind == ExternalModelKind::Heavy ||
                                p.kind == ExternalModelKind::Bizjet;
        const float topMain = std::max(s.wingZ, 0.0f) + s.wingThickM * 0.5f;
        const float topNose = s.rings[2].cz + s.rings[2].hh * 0.8f;
        for (float side : {-1.0f, 1.0f})
            addGearLeg(b, s.mainGearX, side * s.mainGearY, std::min(topMain, gearH - 0.3f), gearH,
                       s.wheelRadM, gearNorm, twinWheels);
        if (p.kind == ExternalModelKind::Heavy)  // body gear
            for (float side : {-1.0f, 1.0f})
                addGearLeg(b, s.mainGearX - 3.0f, side * s.mainGearY * 0.45f,
                           std::min(topNose, gearH - 0.3f), gearH, s.wheelRadM, gearNorm, true);
        addGearLeg(b, s.noseGearX, 0, std::min(topNose, gearH - 0.3f), gearH, s.wheelRadM * 0.8f,
                   gearNorm, twinWheels);
    }

    // JSBSim's reference point is not where the template puts the CG (the 737 reports 1.1 m above
    // the runway on its wheels, the MD-11 5.7 m): shift the model so the wheels sit there.
    if (p.gearHeightM > 0) {
        const float dz = p.gearHeightM - s.gearHeightM * scale;
        for (size_t i = 2; i < out.size(); i += kExternalVertexFloats) out[i] += dz;
    }

    ExternalModelInfo info;
    info.spanM = s.spanM * scale;
    info.lengthM = (s.rings[0].x - s.rings[s.ringCount - 1].x) * scale;
    info.triangles = b.triangles();
    info.minXM = 1e9f;
    info.maxXM = -1e9f;
    info.maxZM = -1e9f;
    for (size_t i = 0; i < out.size(); i += kExternalVertexFloats) {
        info.minXM = std::min(info.minXM, out[i]);
        info.maxXM = std::max(info.maxXM, out[i]);
        info.maxZM = std::max(info.maxZM, out[i + 2]);
    }
    return info;
}

Mat3d lookAtEcef(const Ecef& eye, const Ecef& target) {
    const GeoPos g = ecefToGeodetic(eye);
    const Mat3d n2e = nedToEcef(g.latDeg, g.lonDeg);
    const Vec3d up = n2e * Vec3d{0, 0, -1};
    Vec3d f{target.x - eye.x, target.y - eye.y, target.z - eye.z};
    double l = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
    if (l < 1e-6) {
        f = n2e * Vec3d{1, 0, 0};
        l = 1;
    }
    f = {f.x / l, f.y / l, f.z / l};
    Vec3d r{f.y * up.z - f.z * up.y, f.z * up.x - f.x * up.z, f.x * up.y - f.y * up.x};
    double rl = std::sqrt(r.x * r.x + r.y * r.y + r.z * r.z);
    if (rl < 1e-6) {  // looking straight up or down: any right vector will do
        r = n2e * Vec3d{0, 1, 0};
        rl = 1;
    }
    r = {r.x / rl, r.y / rl, r.z / rl};
    const Vec3d u{r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x};
    Mat3d m;
    const double rows[3][3] = {{r.x, r.y, r.z}, {u.x, u.y, u.z}, {-f.x, -f.y, -f.z}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) m.m[i][j] = rows[i][j];
    return m;
}

ChasePose chasePose(const AircraftState& s, float followHdgDeg, float orbitYawDeg,
                    float orbitPitchDeg, float distM) {
    GeoPos cg;
    cg.latDeg = s.latDeg;
    cg.lonDeg = s.lonDeg;
    cg.altMslM = ftToM(s.altMslFt);
    const Ecef target = geodeticToEcef(cg);
    const double yaw = degToRad(followHdgDeg + orbitYawDeg);
    const double pitch =
        degToRad(std::clamp(orbitPitchDeg - kChaseBaseElevDeg, -80.0f, 30.0f));  // view pitch
    // View direction in NED; the eye sits distM back along it.
    const Vec3d dirNed{std::cos(pitch) * std::cos(yaw), std::cos(pitch) * std::sin(yaw),
                       -std::sin(pitch)};
    const Vec3d dirEcef = nedToEcef(s.latDeg, s.lonDeg) * dirNed;
    ChasePose p;
    p.eyeEcef = {target.x - dirEcef.x * distM, target.y - dirEcef.y * distM,
                 target.z - dirEcef.z * distM};
    p.eyeGeo = ecefToGeodetic(p.eyeEcef);
    p.ecefToEye = lookAtEcef(p.eyeEcef, target);
    return p;
}

float chaseDistanceM(float spanM) {
    return std::max(14.0f, 1.2f * spanM + 7.0f);
}

float followHeadingDeg(float currentDeg, float targetDeg, float tauS, double dtS) {
    const float k = tauS > 0 ? std::min(1.0f, static_cast<float>(dtS) / tauS) : 1.0f;
    const float d = static_cast<float>(wrap180(targetDeg - currentDeg));
    return static_cast<float>(wrap360(currentDeg + k * d));
}

}  // namespace fsim::world
