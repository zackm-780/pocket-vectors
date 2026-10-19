#include "world/AirportGeometry.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/Units.h"
#include "hud/StrokeFont.h"

namespace fsim::world {
namespace {

constexpr float kWhite[3] = {1.0f, 0.97f, 0.88f};
constexpr float kYellow[3] = {1.0f, 0.72f, 0.08f};
constexpr float kGreen[3] = {0.10f, 1.0f, 0.30f};
constexpr float kRed[3] = {1.0f, 0.10f, 0.05f};
constexpr float kFlasherRgb[3] = {0.90f, 0.95f, 1.0f};

constexpr float kAsphaltRgb[3] = {0.20f, 0.20f, 0.22f};
constexpr float kGrassStripRgb[3] = {0.40f, 0.49f, 0.26f};
constexpr float kMarkingRgb[3] = {0.92f, 0.92f, 0.88f};
constexpr float kApronRgb[3] = {0.33f, 0.33f, 0.35f};

constexpr double kPavementLiftM = 0.10;
constexpr double kMarkingLiftM = 0.20;

constexpr double kCenterlineWidthM = 0.9;
constexpr double kStripeWidthM = 1.8;

Vec3d sub(const Ecef& a, const Ecef& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}
double dot(const Vec3d& a, const Vec3d& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}
Vec3d normalized(const Vec3d& v) {
    const double len = std::sqrt(dot(v, v));
    return len > 0 ? Vec3d{v.x / len, v.y / len, v.z / len} : Vec3d{1, 0, 0};
}
Vec3d neg(const Vec3d& v) {
    return {-v.x, -v.y, -v.z};
}

const Runway* findOpposite(const Airport& airport, const Runway& r) {
    for (const Runway& o : airport.runways)
        if (o.ident == r.oppositeIdent && &o != &r) return &o;
    return nullptr;
}

Poly makeRect(double a0, double a1, double l0, double l1, MarkingKind kind) {
    Poly p;
    p.n = 4;
    p.alongM[0] = a0;
    p.lateralM[0] = l0;
    p.alongM[1] = a1;
    p.lateralM[1] = l0;
    p.alongM[2] = a1;
    p.lateralM[2] = l1;
    p.alongM[3] = a0;
    p.lateralM[3] = l1;
    p.kind = kind;
    return p;
}

// A thick line from (a0,l0) to (a1,l1), ends extended by half the width so joints close.
bool makeStrokeQuad(double a0, double l0, double a1, double l1, double widthM, MarkingKind kind,
                    Poly& out) {
    const double da = a1 - a0, dl = l1 - l0;
    const double len = std::hypot(da, dl);
    if (len < 1e-6) return false;
    const double ua = da / len, ul = dl / len;  // direction
    const double na = -ul, nl = ua;             // normal
    const double hw = widthM * 0.5;
    const double sa = a0 - ua * hw, sl = l0 - ul * hw;
    const double ea = a1 + ua * hw, el = l1 + ul * hw;
    out.n = 4;
    out.kind = kind;
    out.alongM[0] = sa + na * hw;
    out.lateralM[0] = sl + nl * hw;
    out.alongM[1] = sa - na * hw;
    out.lateralM[1] = sl - nl * hw;
    out.alongM[2] = ea - na * hw;
    out.lateralM[2] = el - nl * hw;
    out.alongM[3] = ea + na * hw;
    out.lateralM[3] = el + nl * hw;
    return true;
}

// Lays text flat on the runway: glyph x -> lateral (right), glyph y (down) -> toward the threshold,
// so the text reads upright for a pilot looking along the landing direction.
void appendText(std::vector<Poly>& out, const std::string& text, double baselineAlongM,
                double centerLateralM, double unitM, double strokeM) {
    const int n = static_cast<int>(text.size());
    if (n == 0) return;
    const double advance = 6.0 * unitM;  // 4 grid units of glyph + 2 of gap
    const double totalW = (4.0 * n + 2.0 * (n - 1)) * unitM;
    const double lat0 = centerLateralM - totalW * 0.5;
    for (int i = 0; i < n; ++i) {
        const Glyph& g = StrokeFont::glyph(static_cast<std::uint32_t>(
            static_cast<unsigned char>(text[static_cast<size_t>(i)])));
        const double x0 = lat0 + i * advance;
        for (int s = 0; s < g.strokeCount; ++s) {
            const GlyphStroke& st = g.strokes[s];
            const int segs = st.closed ? st.count : st.count - 1;
            for (int k = 0; k < segs; ++k) {
                const GlyphPoint& p = st.points[k];
                const GlyphPoint& q = st.points[(k + 1) % st.count];
                Poly poly;
                if (makeStrokeQuad(baselineAlongM + (8.0 - p.y) * unitM, x0 + p.x * unitM,
                                   baselineAlongM + (8.0 - q.y) * unitM, x0 + q.x * unitM, strokeM,
                                   MarkingKind::Designator, poly))
                    out.push_back(poly);
            }
        }
    }
}

}  // namespace

// ---- runway info ------------------------------------------------------------------------------

double runwayFlatElevM(const Airport& airport, const Runway& runway) {
    const Runway* opposite = findOpposite(airport, runway);
    const Runway* primary = &runway;
    if (opposite && opposite->ident < runway.ident) primary = opposite;
    return ftToM(0.5 * (primary->thrElevFt + primary->endElevFt));
}

RunwayInfo makeRunwayInfo(const Airport& airport, const Runway& r) {
    RunwayInfo info;
    info.airportIdent = r.airportIdent;
    info.ident = r.ident;
    info.oppositeIdent = r.oppositeIdent;
    info.thr = {r.thrLatDeg, r.thrLonDeg};
    const double chordM =
        greatCircleDistM(r.startLatDeg, r.startLonDeg, r.endLatDeg, r.endLonDeg);
    const double lengthM = chordM >= 1.0 ? chordM : ftToM(r.lengthFt);
    info.lengthM = lengthM;
    info.headingTrueRad =
        chordM >= 1.0
            ? initialBearingTrueRad(r.startLatDeg, r.startLonDeg, r.endLatDeg, r.endLonDeg)
            : degToRad(r.headingTrueDeg);
    info.displacedM = greatCircleDistM(r.startLatDeg, r.startLonDeg, r.thrLatDeg, r.thrLonDeg);
    if (info.displacedM > lengthM * 0.5) info.displacedM = 0;  // bad data: ignore
    info.widthM = ftToM(r.widthFt > 0 ? r.widthFt : (r.hard ? 75.0 : 100.0));
    info.hard = r.hard;
    info.lighted = r.lighted;
    info.approx = r.approxGeometry;
    info.instrument = r.ils.has_value() && !r.approxGeometry && r.hard;
    const Runway* opposite = findOpposite(airport, r);
    info.oppositeInstrument = opposite && opposite->ils.has_value() && !opposite->approxGeometry &&
                              opposite->hard;
    info.primary = !opposite || r.ident < opposite->ident;
    info.elevM = runwayFlatElevM(airport, r);
    info.papi = r.hard && !r.approxGeometry && mToFt(lengthM) >= kPapiMinRunwayFt;
    info.approachLights = info.instrument;
    return info;
}

// ---- frame ------------------------------------------------------------------------------------

RunwayFrame::RunwayFrame(const RunwayInfo& info)
    : thr_(info.thr), headingTrueRad_(info.headingTrueRad), elevM_(info.elevM) {}

LatLon RunwayFrame::latLon(double alongM, double lateralM) const {
    LatLon p = thr_;
    if (alongM != 0.0)
        destinationPoint(p.latDeg, p.lonDeg, alongM > 0 ? headingTrueRad_ : headingTrueRad_ + kPi,
                         std::fabs(alongM), p.latDeg, p.lonDeg);
    if (lateralM != 0.0)
        destinationPoint(p.latDeg, p.lonDeg,
                         headingTrueRad_ + (lateralM > 0 ? kPi * 0.5 : -kPi * 0.5),
                         std::fabs(lateralM), p.latDeg, p.lonDeg);
    return p;
}

Ecef RunwayFrame::toEcef(double alongM, double lateralM, double upM) const {
    const LatLon p = latLon(alongM, lateralM);
    GeoPos g;
    g.latDeg = p.latDeg;
    g.lonDeg = p.lonDeg;
    g.altMslM = elevM_ + upM;
    return geodeticToEcef(g);
}

Vec3d RunwayFrame::upEcef(double alongM, double lateralM) const {
    const LatLon p = latLon(alongM, lateralM);
    const double lat = degToRad(p.latDeg), lon = degToRad(p.lonDeg);
    return {std::cos(lat) * std::cos(lon), std::cos(lat) * std::sin(lon), std::sin(lat)};
}

Vec3d RunwayFrame::alongDirEcef(double alongM) const {
    return normalized(sub(toEcef(alongM + 5.0, 0, 0), toEcef(alongM - 5.0, 0, 0)));
}

// ---- markings ---------------------------------------------------------------------------------

int RunwayMarkings::count(MarkingKind kind) const {
    int n = 0;
    for (const Poly& p : polys)
        if (p.kind == kind) ++n;
    return n;
}

int thresholdStripeCount(double widthFt) {
    if (widthFt < 70.0) return 4;     // 60 ft and narrower
    if (widthFt < 90.0) return 6;     // 75 ft
    if (widthFt < 125.0) return 8;    // 100 ft
    if (widthFt < 175.0) return 12;   // 150 ft
    return 16;                        // 200 ft
}

void splitDesignator(const std::string& ident, std::string& number, std::string& suffix) {
    size_t i = 0;
    while (i < ident.size() && ident[i] >= '0' && ident[i] <= '9') ++i;
    number = ident.substr(0, i);
    suffix = ident.substr(i);
    // The zero-padded "09" is painted as "09" in the US, keep it as given.
}

const std::vector<TdzStation>& touchdownZoneStations() {
    static const std::vector<TdzStation> kStations = {
        {500.0, 3}, {1500.0, 2}, {2000.0, 2}, {2500.0, 1}, {3000.0, 1}};
    return kStations;
}

RunwayMarkings buildMarkings(const RunwayInfo& r) {
    RunwayMarkings m;
    if (!r.hard) return m;
    const double w = r.widthM;
    const double usable = r.usableM();
    const double halfMid = usable * 0.5;

    // Threshold stripes: n/2 on each side, 1.8 m wide, outermost 1.5 m from the edge, a 1.8 m
    // half-gap at the centerline, evenly spread in between.
    const int total = thresholdStripeCount(mToFt(w));
    const int perSide = total / 2;
    const double innerStart = 1.8;
    const double outerStart = std::max(innerStart, w * 0.5 - 1.5 - kStripeWidthM);
    const double pitch = perSide > 1 ? (outerStart - innerStart) / (perSide - 1) : 0.0;
    const double a0 = kThresholdStripeStartM, a1 = a0 + kThresholdStripeLengthM;
    for (int side = -1; side <= 1; side += 2) {
        for (int i = 0; i < perSide; ++i) {
            const double l0 = innerStart + i * pitch, l1 = l0 + kStripeWidthM;
            m.polys.push_back(makeRect(a0, a1, side > 0 ? l0 : -l1, side > 0 ? l1 : -l0,
                                       MarkingKind::ThresholdStripe));
        }
    }

    // Designator: letter below (toward the threshold), number above.
    double designatorEnd = a1 + kDesignatorGapM;
    if (usable >= 150.0) {
        std::string number, suffix;
        splitDesignator(r.ident, number, suffix);
        const double h = std::min(ftToM(60.0), std::max(6.0, 0.4 * w));
        const double unit = h / 8.0;
        const double stroke = unit * 0.9;
        double base = a1 + kDesignatorGapM;
        if (!suffix.empty()) {
            appendText(m.polys, suffix, base, 0.0, unit, stroke);
            base += h * 1.4;
        }
        appendText(m.polys, number, base, 0.0, unit, stroke);
        designatorEnd = base + h;
    }
    m.designatorEndM = designatorEnd;

    // Centerline: 120 ft on, 80 ft off, from past the designator to the middle of the runway
    // (the opposite direction paints the other half).
    {
        const double on = ftToM(120.0), period = ftToM(200.0);
        for (double a = designatorEnd + kDesignatorGapM; a + on <= halfMid; a += period)
            m.polys.push_back(makeRect(a, a + on, -kCenterlineWidthM * 0.5,
                                       kCenterlineWidthM * 0.5, MarkingKind::Centerline));
    }

    // Displaced threshold: arrows along the centerline.
    if (r.displacedM > 30.0) {
        const double shaft = 18.0, head = 12.0, period = 45.0;
        for (double a = -r.displacedM + 8.0; a + shaft + head <= -8.0; a += period) {
            m.polys.push_back(makeRect(a, a + shaft, -0.9, 0.9, MarkingKind::DisplacedArrow));
            Poly tri;
            tri.n = 3;
            tri.kind = MarkingKind::DisplacedArrow;
            tri.alongM[0] = a + shaft;
            tri.lateralM[0] = -3.0;
            tri.alongM[1] = a + shaft;
            tri.lateralM[1] = 3.0;
            tri.alongM[2] = a + shaft + head;
            tri.lateralM[2] = 0.0;
            m.polys.push_back(tri);
        }
    }

    if (r.instrument) {
        // Lateral position of the inner edge of the touchdown-zone and aiming-point bars.
        const double inner = std::min(9.0, std::max(4.0, 0.15 * w));
        const double tdzLen = ftToM(75.0), barW = ftToM(6.0), barPitch = ftToM(11.0);
        for (const TdzStation& st : touchdownZoneStations()) {
            const double a = ftToM(st.alongFt);
            if (a + tdzLen > halfMid) continue;
            for (int side = -1; side <= 1; side += 2) {
                for (int i = 0; i < st.barsPerSide; ++i) {
                    const double l0 = inner + i * barPitch, l1 = l0 + barW;
                    m.polys.push_back(makeRect(a, a + tdzLen, side > 0 ? l0 : -l1,
                                               side > 0 ? l1 : -l0, MarkingKind::TouchdownZone));
                }
            }
        }
        const double aim = ftToM(kAimingPointFt), aimLen = ftToM(150.0), aimW = ftToM(30.0);
        if (aim + aimLen <= halfMid) {
            for (int side = -1; side <= 1; side += 2) {
                const double l0 = inner, l1 = inner + aimW;
                m.polys.push_back(makeRect(aim, aim + aimLen, side > 0 ? l0 : -l1,
                                           side > 0 ? l1 : -l0, MarkingKind::AimingPoint));
            }
        }
    }

    // Edge lines for the pair, painted by the primary direction when either end is an
    // instrument runway.
    if (r.primary && (r.instrument || r.oppositeInstrument)) {
        const double inner = w * 0.5 - 1.0, outer = w * 0.5 - 0.55;
        const double chunk = 100.0;
        for (double a = -r.displacedM; a < usable - 0.01; a += chunk) {
            const double e = std::min(a + chunk, usable);
            m.polys.push_back(makeRect(a, e, -outer, -inner, MarkingKind::EdgeLine));
            m.polys.push_back(makeRect(a, e, inner, outer, MarkingKind::EdgeLine));
        }
    }
    return m;
}

// ---- lights -----------------------------------------------------------------------------------

int edgeLightIntervals(double lengthM) {
    return std::max(1, static_cast<int>(std::lround(lengthM / ftToM(kEdgeLightSpacingFt))));
}

static LightSpec makeLight(LightKind kind, double along, double lateral, double height,
                           const float rgb[3], float sizeM, int facing = -1, int seq = -1) {
    LightSpec l;
    l.kind = kind;
    l.alongM = along;
    l.lateralM = lateral;
    l.heightM = height;
    std::memcpy(l.rgb, rgb, sizeof l.rgb);
    l.sizeM = sizeM;
    l.facing = facing;
    l.seq = seq;
    return l;
}

PapiPosition papiUnitPosition(const RunwayInfo& r, int unit) {
    PapiPosition p;
    p.alongM = ftToM(kPapiTchFt / std::tan(degToRad(kPapiGlidepathDeg)));
    p.lateralM = -(r.widthM * 0.5 + ftToM(kPapiFirstUnitFromEdgeFt + kPapiUnitSpacingFt * unit));
    return p;
}

std::vector<LightSpec> buildRunwayLights(const RunwayInfo& r) {
    std::vector<LightSpec> out;
    const double w = r.widthM, len = r.lengthM, usable = r.usableM();

    if (r.lighted) {
        // Edge lights: even spacing of about 200 ft from the pavement start to the end, so the
        // last light sits exactly on the end. Viewed from this landing direction they are white,
        // yellow over the last 2,000 ft of instrument runways and red in the displaced area.
        const int n = edgeLightIntervals(len);
        const double step = len / n;
        const double caution = std::min(ftToM(kEdgeCautionZoneFt), len * 0.5);
        for (int k = 0; k <= n; ++k) {
            const double s = k * step;  // from the pavement start of this direction
            LightKind kind = LightKind::Edge;
            const float* rgb = kWhite;
            if (s < r.displacedM - 1.0) {
                kind = LightKind::EdgeDisplaced;
                rgb = kRed;
            } else if (r.instrument && s >= len - caution - 0.01) {
                kind = LightKind::EdgeCaution;
                rgb = kYellow;
            }
            for (int side = -1; side <= 1; side += 2)
                out.push_back(makeLight(kind, s - r.displacedM, side * (w * 0.5 + 1.0), 0.45, rgb,
                                        0.9f));
        }
        // Threshold (green) and end (red) rows across the full width.
        const int rows = std::max(4, static_cast<int>(std::lround(w / 3.0)));
        for (int i = 0; i <= rows; ++i) {
            const double lat = -w * 0.5 + w * i / rows;
            out.push_back(makeLight(LightKind::Threshold, 0.0, lat, 0.30, kGreen, 1.1f));
            out.push_back(makeLight(LightKind::End, usable, lat, 0.30, kRed, 1.1f));
        }
    }

    if (r.approachLights) {
        // Simplified MALSR: a bar of 5 lights every 200 ft out to 2,400 ft, and sequenced
        // flashers (one per bar position) on the outer 1,000 ft.
        for (int i = 1; i <= kApproachBarCount; ++i) {
            const double along = -ftToM(kApproachBarSpacingFt * i);
            for (int k = 0; k < kApproachLightsPerBar; ++k) {
                const double lat = (k - (kApproachLightsPerBar - 1) * 0.5) * kApproachLightSpacingM;
                out.push_back(
                    makeLight(LightKind::Approach, along, lat, 1.2, kWhite, 1.4f, -1, -1));
            }
        }
        for (int i = 0; i < kFlasherCount; ++i) {
            const double along = -ftToM(kApproachLengthFt - kApproachBarSpacingFt * i);
            out.push_back(makeLight(LightKind::Flasher, along, 0.0, 2.2, kFlasherRgb, 3.2f, -1, i));
        }
    }

    if (r.papi) {
        for (int i = 0; i < kPapiUnitCount; ++i) {
            const PapiPosition p = papiUnitPosition(r, i);
            out.push_back(makeLight(LightKind::Papi, p.alongM, p.lateralM, kPapiUnitHeightM, kWhite,
                                    1.8f, -1, 100 + i));
        }
    }
    return out;
}

// ---- PAPI -------------------------------------------------------------------------------------

bool papiUnitIsWhite(int unit, double elevationDeg) {
    if (unit < 0 || unit >= kPapiUnitCount) return false;
    return elevationDeg > kPapiTransitionDeg[unit];
}

unsigned papiWhiteMask(double elevationDeg) {
    unsigned mask = 0;
    for (int i = 0; i < kPapiUnitCount; ++i)
        if (papiUnitIsWhite(i, elevationDeg)) mask |= 1u << i;
    return mask;
}

double papiEyeElevationDeg(const RunwayFrame& frame, const RunwayInfo& r, const Ecef& eye) {
    const PapiPosition p = papiUnitPosition(r, 0);
    const Ecef unit = frame.toEcef(p.alongM, p.lateralM, kPapiUnitHeightM);
    const Vec3d d = sub(eye, unit);
    const double height = dot(d, frame.upEcef(p.alongM, p.lateralM));
    const double before = -dot(d, frame.alongDirEcef(p.alongM));  // distance short of the unit
    return radToDeg(std::atan2(height, before));
}

int flasherActiveIndex(double timeS) {
    const double phase = timeS * kFlasherHz - std::floor(timeS * kFlasherHz);
    if (phase >= 0.75) return -1;  // short dark gap between sweeps
    return std::min(kFlasherCount - 1, static_cast<int>(phase / 0.75 * kFlasherCount));
}

// ---- terrain flattening -----------------------------------------------------------------------

FlattenArea runwayFlattenArea(const RunwayInfo& r, double lateralMarginM, double endMarginM,
                              float blendM) {
    const RunwayFrame frame(r);
    const double a0 = -r.displacedM - endMarginM, a1 = r.usableM() + endMarginM;
    const double l = r.widthM * 0.5 + lateralMarginM;
    // In (along, right) coordinates this order runs clockwise; the along/right -> east/north map
    // is a reflection, so on the ground it is counter-clockwise as FlattenArea requires.
    const double along[4] = {a0, a0, a1, a1};
    const double lateral[4] = {-l, l, l, -l};
    FlattenArea area;
    for (int i = 0; i < 4; ++i) {
        const LatLon p = frame.latLon(along[i], lateral[i]);
        area.latDeg[i] = p.latDeg;
        area.lonDeg[i] = p.lonDeg;
    }
    area.elevM = static_cast<float>(r.elevM);
    area.blendM = blendM;
    return area;
}

void registerAirportFlatten(const Airport& airport, TerrainQuery& terrain) {
    for (const Runway& rwy : airport.runways) {
        const RunwayInfo info = makeRunwayInfo(airport, rwy);
        if (info.primary) terrain.addFlattenArea(runwayFlattenArea(info));
    }
}

double airportLowestFlatElevM(const Airport& airport) {
    double lowest = ftToM(airport.elevFt);
    bool any = false;
    for (const Runway& rwy : airport.runways) {
        const double e = runwayFlatElevM(airport, rwy);
        lowest = any ? std::min(lowest, e) : e;
        any = true;
    }
    return lowest;
}

// ---- airport selection ------------------------------------------------------------------------

std::vector<const Airport*> selectAirportsNear(const NavDatabase& nav, double latDeg,
                                               double lonDeg, double radiusM, int maxCount) {
    const double dLat = radiusM / (kEarthRadiusM * kRadPerDeg);
    const double cosLat = std::max(0.05, std::cos(degToRad(latDeg)));
    const double dLon = std::min(179.0, dLat / cosLat);
    auto wrapLon = [](double lon) { return lon < -180.0 ? lon + 360.0 : lon > 180.0 ? lon - 360.0 : lon; };
    std::vector<const Airport*> found =
        nav.airportsInBox(std::max(-90.0, latDeg - dLat), wrapLon(lonDeg - dLon),
                          std::min(90.0, latDeg + dLat), wrapLon(lonDeg + dLon));
    struct Entry {
        const Airport* airport;
        double distM;
    };
    std::vector<Entry> kept;
    for (const Airport* a : found) {
        if (a->runways.empty()) continue;
        const double d = greatCircleDistM(latDeg, lonDeg, a->latDeg, a->lonDeg);
        if (d <= radiusM) kept.push_back({a, d});
    }
    std::sort(kept.begin(), kept.end(), [](const Entry& x, const Entry& y) {
        return x.distM != y.distM ? x.distM < y.distM : x.airport->ident < y.airport->ident;
    });
    std::vector<const Airport*> out;
    for (const Entry& e : kept) {
        if (static_cast<int>(out.size()) >= maxCount) break;
        out.push_back(e.airport);
    }
    return out;
}

// ---- mesh builders ----------------------------------------------------------------------------

SurfaceMeshBuilder::SurfaceMeshBuilder(const Ecef& anchor, const float*) : anchor_(anchor) {
    const GeoPos g = ecefToGeodetic(anchor);
    const double lat = degToRad(g.latDeg), lon = degToRad(g.lonDeg);
    up_[0] = static_cast<float>(std::cos(lat) * std::cos(lon));
    up_[1] = static_cast<float>(std::cos(lat) * std::sin(lon));
    up_[2] = static_cast<float>(std::sin(lat));
}

SurfaceMesh& SurfaceMeshBuilder::current(int extraVerts) {
    if (meshes_.empty() || meshes_.back().verts.size() + static_cast<size_t>(extraVerts) > 65000) {
        meshes_.emplace_back();
        meshes_.back().anchor = anchor_;
    }
    return meshes_.back();
}

void SurfaceMeshBuilder::pushVertex(const Ecef& p, const float normal[3], const float rgb[3]) {
    SurfaceVertex v;
    v.pos[0] = static_cast<float>(p.x - anchor_.x);
    v.pos[1] = static_cast<float>(p.y - anchor_.y);
    v.pos[2] = static_cast<float>(p.z - anchor_.z);
    std::memcpy(v.normal, normal, sizeof v.normal);
    std::memcpy(v.rgb, rgb, sizeof v.rgb);
    meshes_.back().verts.push_back(v);
}

void SurfaceMeshBuilder::addPoly(const RunwayFrame& frame, const Poly& poly, double liftM,
                                 const float rgb[3]) {
    SurfaceMesh& mesh = current(poly.n);
    const uint16_t base = static_cast<uint16_t>(mesh.verts.size());
    for (int i = 0; i < poly.n; ++i)
        pushVertex(frame.toEcef(poly.alongM[i], poly.lateralM[i], liftM), up_, rgb);
    for (int i = 1; i + 1 < poly.n; ++i) {
        mesh.indices.push_back(base);
        mesh.indices.push_back(static_cast<uint16_t>(base + i));
        mesh.indices.push_back(static_cast<uint16_t>(base + i + 1));
    }
}

void SurfaceMeshBuilder::addStrip(const RunwayFrame& frame, double a0, double a1, double l0,
                                  double l1, double liftM, const float rgb[3],
                                  double maxSegmentM) {
    const int segments = std::max(1, static_cast<int>(std::ceil((a1 - a0) / maxSegmentM)));
    SurfaceMesh& mesh = current(2 * (segments + 1));
    const uint16_t base = static_cast<uint16_t>(mesh.verts.size());
    for (int k = 0; k <= segments; ++k) {
        const double a = a0 + (a1 - a0) * k / segments;
        pushVertex(frame.toEcef(a, l0, liftM), up_, rgb);
        pushVertex(frame.toEcef(a, l1, liftM), up_, rgb);
    }
    for (int k = 0; k < segments; ++k) {
        const uint16_t i0 = static_cast<uint16_t>(base + 2 * k);
        const uint16_t idx[6] = {i0,
                                 static_cast<uint16_t>(i0 + 1),
                                 static_cast<uint16_t>(i0 + 2),
                                 static_cast<uint16_t>(i0 + 2),
                                 static_cast<uint16_t>(i0 + 1),
                                 static_cast<uint16_t>(i0 + 3)};
        mesh.indices.insert(mesh.indices.end(), idx, idx + 6);
    }
}

void SurfaceMeshBuilder::addBox(const RunwayFrame& frame, double a0, double a1, double l0,
                                double l1, double bottomM, double heightM, const float wallRgb[3],
                                const float roofRgb[3]) {
    const double ac = 0.5 * (a0 + a1), lc = 0.5 * (l0 + l1);
    const Vec3d aDir = frame.alongDirEcef(ac);
    const Vec3d up = frame.upEcef(ac, lc);
    // Right direction = up x along (right of the landing direction).
    const Vec3d rDir = normalized({up.y * aDir.z - up.z * aDir.y, up.z * aDir.x - up.x * aDir.z,
                                   up.x * aDir.y - up.y * aDir.x});
    auto f3 = [](const Vec3d& v, float out[3]) {
        out[0] = static_cast<float>(v.x);
        out[1] = static_cast<float>(v.y);
        out[2] = static_cast<float>(v.z);
    };
    float nA[3], nR[3], nU[3], nAneg[3], nRneg[3];
    f3(aDir, nA);
    f3(rDir, nR);
    f3(up, nU);
    f3(neg(aDir), nAneg);
    f3(neg(rDir), nRneg);

    const double top = bottomM + heightM;
    auto corner = [&](double a, double l, double u) { return frame.toEcef(a, l, u); };
    auto quad = [&](const Ecef& p0, const Ecef& p1, const Ecef& p2, const Ecef& p3,
                    const float normal[3], const float rgb[3]) {
        SurfaceMesh& mesh = current(4);
        const uint16_t base = static_cast<uint16_t>(mesh.verts.size());
        pushVertex(p0, normal, rgb);
        pushVertex(p1, normal, rgb);
        pushVertex(p2, normal, rgb);
        pushVertex(p3, normal, rgb);
        const uint16_t idx[6] = {base, static_cast<uint16_t>(base + 1), static_cast<uint16_t>(base + 2),
                                 base, static_cast<uint16_t>(base + 2), static_cast<uint16_t>(base + 3)};
        mesh.indices.insert(mesh.indices.end(), idx, idx + 6);
    };
    quad(corner(a0, l0, top), corner(a1, l0, top), corner(a1, l1, top), corner(a0, l1, top), nU,
         roofRgb);
    quad(corner(a1, l0, bottomM), corner(a1, l1, bottomM), corner(a1, l1, top),
         corner(a1, l0, top), nA, wallRgb);  // far wall
    quad(corner(a0, l0, bottomM), corner(a0, l1, bottomM), corner(a0, l1, top),
         corner(a0, l0, top), nAneg, wallRgb);  // near wall
    quad(corner(a0, l1, bottomM), corner(a1, l1, bottomM), corner(a1, l1, top),
         corner(a0, l1, top), nR, wallRgb);  // right wall
    quad(corner(a0, l0, bottomM), corner(a1, l0, bottomM), corner(a1, l0, top),
         corner(a0, l0, top), nRneg, wallRgb);  // left wall
}

std::vector<SurfaceMesh> SurfaceMeshBuilder::finish() {
    std::vector<SurfaceMesh> out;
    for (SurfaceMesh& m : meshes_)
        if (!m.indices.empty()) out.push_back(std::move(m));
    meshes_.clear();
    return out;
}

Ecef runwayCenterEcef(const RunwayInfo& r) {
    return RunwayFrame(r).toEcef(r.usableM() * 0.5, 0, 0);
}

RunwayMeshes buildRunwayMeshes(const RunwayInfo& r) {
    RunwayMeshes out;
    const RunwayFrame frame(r);
    const Ecef anchor = runwayCenterEcef(r);

    if (r.primary) {
        SurfaceMeshBuilder pave(anchor);
        pave.addStrip(frame, -r.displacedM, r.usableM(), -r.widthM * 0.5, r.widthM * 0.5,
                      kPavementLiftM, r.hard ? kAsphaltRgb : kGrassStripRgb);
        out.pavement = pave.finish();
    }

    out.markingData = buildMarkings(r);
    SurfaceMeshBuilder marks(anchor);
    for (const Poly& p : out.markingData.polys) marks.addPoly(frame, p, kMarkingLiftM, kMarkingRgb);
    out.markings = marks.finish();

    out.lights.anchor = anchor;
    const Vec3d axis = frame.alongDirEcef(r.usableM() * 0.5);
    out.lights.axisEcef[0] = static_cast<float>(axis.x);
    out.lights.axisEcef[1] = static_cast<float>(axis.y);
    out.lights.axisEcef[2] = static_cast<float>(axis.z);
    for (const LightSpec& l : buildRunwayLights(r)) {
        const Ecef p = frame.toEcef(l.alongM, l.lateralM, l.heightM);
        LightVertex v;
        v.pos[0] = static_cast<float>(p.x - anchor.x);
        v.pos[1] = static_cast<float>(p.y - anchor.y);
        v.pos[2] = static_cast<float>(p.z - anchor.z);
        std::memcpy(v.rgb, l.rgb, sizeof v.rgb);
        v.info[0] = l.sizeM;
        v.info[1] = static_cast<float>(l.facing);
        v.info[2] = static_cast<float>(l.seq);
        out.lights.verts.push_back(v);
        if (l.kind == LightKind::Papi) out.lights.hasPapi = true;
    }
    return out;
}

// ---- apron and buildings ----------------------------------------------------------------------

namespace {

struct Pt {
    double x, y;
};

// Runway pavement rectangle in local east/north metres around (lat0, lon0), grown by margins.
void runwayRectEnu(const RunwayInfo& r, double lat0, double lon0, double lateralGrowM,
                   double endGrowM, Pt out[4]) {
    const RunwayFrame frame(r);
    const double mPerDegLat = kEarthRadiusM * kRadPerDeg;
    const double mPerDegLon = mPerDegLat * std::cos(degToRad(lat0));
    const double a0 = -r.displacedM - endGrowM, a1 = r.usableM() + endGrowM;
    const double l = r.widthM * 0.5 + lateralGrowM;
    const double along[4] = {a0, a0, a1, a1};
    const double lateral[4] = {-l, l, l, -l};
    for (int i = 0; i < 4; ++i) {
        const LatLon p = frame.latLon(along[i], lateral[i]);
        out[i] = {(p.lonDeg - lon0) * mPerDegLon, (p.latDeg - lat0) * mPerDegLat};
    }
}

// Separating-axis test for two convex quads.
bool quadsOverlap(const Pt a[4], const Pt b[4]) {
    for (int pass = 0; pass < 2; ++pass) {
        const Pt* q = pass == 0 ? a : b;
        for (int i = 0; i < 4; ++i) {
            const Pt e = {q[(i + 1) % 4].x - q[i].x, q[(i + 1) % 4].y - q[i].y};
            const Pt n = {-e.y, e.x};
            double minA = 1e30, maxA = -1e30, minB = 1e30, maxB = -1e30;
            for (int k = 0; k < 4; ++k) {
                const double da = a[k].x * n.x + a[k].y * n.y;
                const double db = b[k].x * n.x + b[k].y * n.y;
                minA = std::min(minA, da);
                maxA = std::max(maxA, da);
                minB = std::min(minB, db);
                maxB = std::max(maxB, db);
            }
            if (maxA < minB || maxB < minA) return false;
        }
    }
    return true;
}

}  // namespace

AirportExtras buildAirportExtras(const Airport& airport) {
    AirportExtras out;
    // The longest primary hard runway carries the apron.
    const Runway* best = nullptr;
    double bestLen = 0;
    for (const Runway& rwy : airport.runways) {
        const RunwayInfo info = makeRunwayInfo(airport, rwy);
        if (info.primary && info.hard && info.lengthM > bestLen) {
            best = &rwy;
            bestLen = info.lengthM;
        }
    }
    if (!best || bestLen < ftToM(1500.0)) return out;
    const RunwayInfo main = makeRunwayInfo(airport, *best);
    const RunwayFrame frame(main);
    const double w = main.widthM;

    // Apron 120 x 70 m, 105 m from the runway edge (taxiway stub 16 m wide in between), and a
    // building row behind it. Try both sides and a few stations along the runway; keep the first
    // that stays clear of every other runway.
    const double kApronNear = 105.0, kApronDepth = 70.0, kApronLen = 120.0, kRowDepth = 55.0;
    const double stations[] = {0.5, 0.35, 0.65, 0.2, 0.8};
    for (double frac : stations) {
        for (int side = 1; side >= -1; side -= 2) {
            const double ac = main.usableM() * frac - main.displacedM * 0.0;
            const double lNear = w * 0.5, lFar = w * 0.5 + kApronNear + kApronDepth + kRowDepth;
            Pt cand[4];
            const double lo = side > 0 ? lNear : -lFar, hi = side > 0 ? lFar : -lNear;
            const double along[4] = {ac - kApronLen * 0.5 - 15, ac - kApronLen * 0.5 - 15,
                                     ac + kApronLen * 0.5 + 15, ac + kApronLen * 0.5 + 15};
            const double lateral[4] = {lo, hi, hi, lo};
            for (int i = 0; i < 4; ++i) {
                const LatLon p = frame.latLon(along[i], lateral[i]);
                const double mPerDegLat = kEarthRadiusM * kRadPerDeg;
                cand[i] = {(p.lonDeg - airport.lonDeg) * mPerDegLat * std::cos(degToRad(airport.latDeg)),
                           (p.latDeg - airport.latDeg) * mPerDegLat};
            }
            bool clear = true;
            for (const Runway& other : airport.runways) {
                if (other.ident == main.ident || other.ident == main.oppositeIdent) continue;
                const RunwayInfo oi = makeRunwayInfo(airport, other);
                Pt rect[4];
                runwayRectEnu(oi, airport.latDeg, airport.lonDeg, 40.0, 60.0, rect);
                if (quadsOverlap(cand, rect)) {
                    clear = false;
                    break;
                }
            }
            if (!clear) continue;

            const Ecef anchor = frame.toEcef(ac, side * (w * 0.5 + kApronNear), 0);
            SurfaceMeshBuilder flat(anchor), solid(anchor);
            const double sNear = w * 0.5, sApronNear = w * 0.5 + kApronNear;
            const double sApronFar = sApronNear + kApronDepth;
            auto span = [&](double a, double b) { return side > 0 ? std::make_pair(a, b) : std::make_pair(-b, -a); };
            const auto stub = span(sNear, sApronNear);
            flat.addStrip(frame, ac - 8.0, ac + 8.0, stub.first, stub.second, 0.08, kApronRgb, 60.0);
            const auto apron = span(sApronNear, sApronFar);
            flat.addStrip(frame, ac - kApronLen * 0.5, ac + kApronLen * 0.5, apron.first,
                          apron.second, 0.08, kApronRgb, 60.0);
            out.flat = flat.finish();

            // Buildings in the row behind the apron.
            const float wallA[3] = {0.74f, 0.72f, 0.68f}, roofA[3] = {0.40f, 0.18f, 0.16f};
            const float wallB[3] = {0.62f, 0.66f, 0.70f}, roofB[3] = {0.50f, 0.52f, 0.55f};
            const double rowNear = sApronFar + 8.0;
            const bool big = airport.type == AirportType::Large || airport.type == AirportType::Medium;
            if (big) {
                const auto t = span(rowNear, rowNear + 24.0);
                solid.addBox(frame, ac - 45.0, ac + 25.0, t.first, t.second, -2.0, 13.0, wallB, roofB);
                const auto h = span(rowNear, rowNear + 20.0);
                solid.addBox(frame, ac + 40.0, ac + 60.0, h.first, h.second, -2.0, 9.0, wallA, roofA);
            } else {
                const auto h1 = span(rowNear, rowNear + 18.0);
                solid.addBox(frame, ac - 40.0, ac - 16.0, h1.first, h1.second, -2.0, 7.0, wallA, roofA);
                solid.addBox(frame, ac - 8.0, ac + 16.0, h1.first, h1.second, -2.0, 7.0, wallB, roofA);
                solid.addBox(frame, ac + 24.0, ac + 44.0, h1.first, h1.second, -2.0, 6.0, wallA, roofB);
            }
            out.solid = solid.finish();
            return out;
        }
    }
    return out;
}

}  // namespace fsim::world
