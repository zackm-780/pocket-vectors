// W5: airport geometry (markings, lights, PAPI, flatten areas, meshes). No GL context needed.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>

#include "core/FlatTerrain.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "world/AirportGeometry.h"

using namespace fsim;
using namespace fsim::world;

namespace {

struct Xtst {
    SyntheticNavDb db;
    const Airport& airport = db.airport();
    const Runway& rwy27 = *findRunway(airport, "27");
    const Runway& rwy09 = *findRunway(airport, "09");
    RunwayInfo info27 = makeRunwayInfo(airport, rwy27);
};

int countKind(const std::vector<LightSpec>& lights, LightKind kind) {
    return static_cast<int>(std::count_if(lights.begin(), lights.end(),
                                          [&](const LightSpec& l) { return l.kind == kind; }));
}

double distM(const Ecef& a, const Ecef& b) {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
}

// Counts addFlattenArea calls and keeps the areas.
class RecordingTerrain : public TerrainQuery {
public:
    float elevationM(double, double) const override { return 0; }
    LandClass landClass(double, double) const override { return LandClass::Grass; }
    void addFlattenArea(const FlattenArea& a) override { areas.push_back(a); }
    void clearFlattenAreas() override { areas.clear(); }
    void prefetch(double, double, float) override {}
    bool isDetailReady(double, double) const override { return true; }
    std::vector<FlattenArea> areas;
};

}  // namespace

TEST_CASE("world: PAPI colours at 2.4, 2.9, 3.0, 3.1 and 3.6 degrees") {
    // Unit 0 is nearest the runway (transition 3.50), unit 3 is the outermost (2.50).
    auto whites = [](double elevDeg) {
        std::string s;
        for (int u = 0; u < kPapiUnitCount; ++u) s += papiUnitIsWhite(u, elevDeg) ? 'W' : 'R';
        return s;
    };
    CHECK(whites(2.4) == "RRRR");  // low: all red
    CHECK(whites(2.9) == "RRWW");  // slightly low of the 3.0 path: inner red, outer white
    CHECK(whites(3.0) == "RRWW");  // on path: 2 red (inner) + 2 white (outer)
    CHECK(whites(3.1) == "RRWW");
    CHECK(whites(3.6) == "WWWW");  // high: all white
    CHECK(papiWhiteMask(2.4) == 0u);
    CHECK(papiWhiteMask(3.0) == 0b1100u);
    CHECK(papiWhiteMask(3.6) == 0b1111u);
    // Boundaries: strictly above the transition angle is white.
    CHECK(papiUnitIsWhite(0, 3.51));
    CHECK_FALSE(papiUnitIsWhite(0, 3.49));
    CHECK(papiUnitIsWhite(3, 2.51));
    CHECK_FALSE(papiUnitIsWhite(3, 2.49));
    CHECK_FALSE(papiUnitIsWhite(-1, 10.0));
    CHECK_FALSE(papiUnitIsWhite(4, 10.0));
}

TEST_CASE(
    "world: PAPI units sit on the left, 50 ft out, 30 ft apart, TCH/tan(3 deg) down the runway") {
    Xtst x;
    const double expectedAlongM = ftToM(50.0 / std::tan(degToRad(3.0)));
    CHECK(expectedAlongM == doctest::Approx(290.8).epsilon(0.002));  // ~954 ft
    for (int u = 0; u < kPapiUnitCount; ++u) {
        const PapiPosition p = papiUnitPosition(x.info27, u);
        CHECK(p.alongM == doctest::Approx(expectedAlongM));
        CHECK(p.lateralM < 0);  // left of the landing direction
        CHECK(-p.lateralM == doctest::Approx(x.info27.widthM * 0.5 + ftToM(50.0 + 30.0 * u)));
    }
}

TEST_CASE("world: eye elevation angle drives the PAPI pattern") {
    Xtst x;
    const RunwayFrame frame(x.info27);
    const double unitAlong = papiUnitPosition(x.info27, 0).alongM;
    const double distBeforeM = nmToM(2.0);  // eye 2 NM short of the threshold
    auto eyeAt = [&](double elevDeg) {
        // Height above the unit lamp for the wanted elevation angle at that distance.
        const double h = (distBeforeM + unitAlong) * std::tan(degToRad(elevDeg));
        return frame.toEcef(-distBeforeM, 0, kPapiUnitHeightM + h);
    };
    auto maskAt = [&](double elevDeg) {
        return papiWhiteMask(papiEyeElevationDeg(frame, x.info27, eyeAt(elevDeg)));
    };
    CHECK(papiEyeElevationDeg(frame, x.info27, eyeAt(3.0)) == doctest::Approx(3.0).epsilon(0.02));
    CHECK(maskAt(3.0) == 0b1100u);   // on path: outer two white, inner two red
    CHECK(maskAt(3.6) == 0b1111u);   // high
    CHECK(maskAt(2.2) == 0u);        // low
    CHECK(maskAt(2.65) == 0b1000u);  // slightly low: only the outermost unit is white
    CHECK(maskAt(3.35) == 0b1110u);  // slightly high: only the nearest unit is red
}

TEST_CASE("world: runway frame follows the runway") {
    Xtst x;
    const RunwayFrame frame(x.info27);
    const Ecef thr = frame.toEcef(0, 0, 0);
    CHECK(distM(thr, geodeticToEcef({x.rwy27.thrLatDeg, x.rwy27.thrLonDeg, x.info27.elevM})) < 0.5);
    CHECK(x.info27.usableM() == doctest::Approx(ftToM(10000.0)).epsilon(0.001));
    CHECK(distM(frame.toEcef(x.info27.usableM(), 0, 0),
                geodeticToEcef({x.rwy27.endLatDeg, x.rwy27.endLonDeg, x.info27.elevM})) < 2.0);
    // Right of the landing direction (west-facing runway: right is north).
    const GeoPos right = ecefToGeodetic(frame.toEcef(0, 10.0, 0));
    CHECK(right.latDeg > x.rwy27.thrLatDeg);
    // Pair bookkeeping: 09 owns the pavement of XTST (smaller ident).
    CHECK(x.info27.oppositeIdent == "09");
    CHECK_FALSE(x.info27.primary);
    CHECK(makeRunwayInfo(x.airport, x.rwy09).primary);
    CHECK(x.info27.instrument);
    CHECK(x.info27.papi);
    CHECK(x.info27.approachLights);
}

TEST_CASE("world: light placement on XTST 27 (counts and positions)") {
    Xtst x;
    const std::vector<LightSpec> lights = buildRunwayLights(x.info27);
    const double w = x.info27.widthM;

    // Edge lights: 200 ft spacing from the pavement start to the end, both sides.
    CHECK(edgeLightIntervals(x.info27.lengthM) == 50);
    CHECK(countKind(lights, LightKind::Edge) + countKind(lights, LightKind::EdgeCaution) == 2 * 51);
    // Instrument runway: the last 2,000 ft (11 light stations per side) are yellow.
    CHECK(countKind(lights, LightKind::EdgeCaution) == 2 * 11);
    CHECK(countKind(lights, LightKind::EdgeDisplaced) == 0);
    double minCautionAlong = 1e9, maxEdgeAlong = -1e9;
    for (const LightSpec& l : lights) {
        if (l.kind == LightKind::EdgeCaution) minCautionAlong = std::min(minCautionAlong, l.alongM);
        if (l.kind == LightKind::Edge) maxEdgeAlong = std::max(maxEdgeAlong, l.alongM);
        if (l.kind == LightKind::Edge || l.kind == LightKind::EdgeCaution) {
            CHECK(std::fabs(l.lateralM) == doctest::Approx(w * 0.5 + 1.0));
            // On the 200 ft grid.
            const double k = l.alongM / ftToM(200.0);
            CHECK(std::fabs(k - std::round(k)) < 1e-3);
        }
    }
    CHECK(minCautionAlong == doctest::Approx(ftToM(8000.0)));
    CHECK(maxEdgeAlong == doctest::Approx(ftToM(7800.0)));

    // Threshold (green) at along 0 and end (red) at the far end, spread across the width.
    CHECK(countKind(lights, LightKind::Threshold) == 16);
    CHECK(countKind(lights, LightKind::End) == 16);
    for (const LightSpec& l : lights) {
        if (l.kind == LightKind::Threshold) {
            CHECK(l.alongM == doctest::Approx(0.0));
            CHECK(l.rgb[1] > l.rgb[0]);  // green
            CHECK(std::fabs(l.lateralM) <= w * 0.5 + 1e-6);
        }
        if (l.kind == LightKind::End) {
            CHECK(l.alongM == doctest::Approx(x.info27.usableM()));
            CHECK(l.rgb[0] > l.rgb[1]);  // red
        }
    }

    // MALSR: 12 bars every 200 ft out to 2,400 ft, 5 lights each; 6 sequenced flashers.
    CHECK(countKind(lights, LightKind::Approach) == kApproachBarCount * kApproachLightsPerBar);
    CHECK(countKind(lights, LightKind::Flasher) == kFlasherCount);
    std::vector<double> barAlong;
    for (const LightSpec& l : lights) {
        if (l.kind != LightKind::Approach) continue;
        CHECK(l.alongM < 0);
        if (std::find_if(barAlong.begin(), barAlong.end(), [&](double a) {
                return std::fabs(a - l.alongM) < 1e-6;
            }) == barAlong.end())
            barAlong.push_back(l.alongM);
    }
    CHECK(static_cast<int>(barAlong.size()) == 12);
    std::sort(barAlong.begin(), barAlong.end());
    CHECK(barAlong.front() == doctest::Approx(-ftToM(2400.0)));
    CHECK(barAlong.back() == doctest::Approx(-ftToM(200.0)));
    for (size_t i = 1; i < barAlong.size(); ++i)
        CHECK(barAlong[i] - barAlong[i - 1] == doctest::Approx(ftToM(200.0)));
    std::vector<int> seqs;
    double outerFlasher = 0, innerFlasher = -1e9;
    for (const LightSpec& l : lights) {
        if (l.kind != LightKind::Flasher) continue;
        seqs.push_back(l.seq);
        outerFlasher = std::min(outerFlasher, l.alongM);
        innerFlasher = std::max(innerFlasher, l.alongM);
        if (l.seq == 0) CHECK(l.alongM == doctest::Approx(-ftToM(2400.0)));  // fires first
    }
    std::sort(seqs.begin(), seqs.end());
    CHECK(seqs == std::vector<int>{0, 1, 2, 3, 4, 5});
    CHECK(outerFlasher == doctest::Approx(-ftToM(2400.0)));
    CHECK(innerFlasher == doctest::Approx(-ftToM(1400.0)));  // the outer 1,000 ft only

    // PAPI: four units flagged 100..103.
    CHECK(countKind(lights, LightKind::Papi) == 4);
}

TEST_CASE("world: approx, non-ILS, short, grass and unlit runways") {
    Xtst x;
    SUBCASE("approx runways get no PAPI and no approach lights") {
        Runway r = x.rwy27;
        r.approxGeometry = true;  // DATA 5.3
        const RunwayInfo info = makeRunwayInfo(x.airport, r);
        const auto lights = buildRunwayLights(info);
        CHECK_FALSE(info.papi);
        CHECK_FALSE(info.approachLights);
        CHECK(countKind(lights, LightKind::Papi) == 0);
        CHECK(countKind(lights, LightKind::Approach) == 0);
        CHECK(countKind(lights, LightKind::Flasher) == 0);
        // Still drawn and usable: edge lights and markings exist, but no precision markings.
        CHECK(countKind(lights, LightKind::Edge) > 0);
        const RunwayMarkings m = buildMarkings(info);
        CHECK(m.count(MarkingKind::Designator) > 0);
        CHECK(m.count(MarkingKind::AimingPoint) == 0);
        CHECK(m.count(MarkingKind::TouchdownZone) == 0);
    }
    SUBCASE("no ILS: PAPI but no approach lights, edge lights stay white") {
        Runway r = x.rwy27;
        r.ils.reset();
        const RunwayInfo info = makeRunwayInfo(x.airport, r);
        const auto lights = buildRunwayLights(info);
        CHECK(countKind(lights, LightKind::Papi) == 4);
        CHECK(countKind(lights, LightKind::Approach) == 0);
        CHECK(countKind(lights, LightKind::EdgeCaution) == 0);
    }
    SUBCASE("hard runways under 3,000 ft get no PAPI") {
        Runway r = x.rwy27;
        r.ils.reset();
        // Shorten the pavement to 2,500 ft: end 2,500 ft from the start.
        destinationPoint(r.startLatDeg, r.startLonDeg, degToRad(270.0), ftToM(2500.0), r.endLatDeg,
                         r.endLonDeg);
        const RunwayInfo info = makeRunwayInfo(x.airport, r);
        CHECK(info.lengthM == doctest::Approx(ftToM(2500.0)).epsilon(0.001));
        CHECK_FALSE(info.papi);
        CHECK(countKind(buildRunwayLights(info), LightKind::Papi) == 0);
    }
    SUBCASE("grass runway: no PAPI, no approach lights, no markings") {
        Runway r = x.rwy27;
        r.hard = false;
        r.ils.reset();
        const RunwayInfo info = makeRunwayInfo(x.airport, r);
        CHECK_FALSE(info.papi);
        CHECK(buildMarkings(info).polys.empty());
        CHECK(countKind(buildRunwayLights(info), LightKind::Papi) == 0);
    }
    SUBCASE("unlit runway: only PAPI and approach lights") {
        Runway r = x.rwy27;
        r.lighted = false;
        const auto lights = buildRunwayLights(makeRunwayInfo(x.airport, r));
        CHECK(countKind(lights, LightKind::Edge) == 0);
        CHECK(countKind(lights, LightKind::Threshold) == 0);
        CHECK(countKind(lights, LightKind::End) == 0);
        CHECK(countKind(lights, LightKind::Papi) == 4);
    }
    SUBCASE("displaced threshold: red edge lights in the displaced area, arrows painted") {
        Runway r = x.rwy27;
        // Move the threshold 1,000 ft down the runway.
        destinationPoint(r.startLatDeg, r.startLonDeg, degToRad(270.0), ftToM(1000.0), r.thrLatDeg,
                         r.thrLonDeg);
        const RunwayInfo info = makeRunwayInfo(x.airport, r);
        CHECK(info.displacedM == doctest::Approx(ftToM(1000.0)).epsilon(0.001));
        const auto lights = buildRunwayLights(info);
        CHECK(countKind(lights, LightKind::EdgeDisplaced) == 2 * 5);  // stations 0..800 ft
        for (const LightSpec& l : lights)
            if (l.kind == LightKind::EdgeDisplaced) {
                CHECK(l.alongM < 0);
                CHECK(l.rgb[0] > l.rgb[1]);
            }
        const RunwayMarkings m = buildMarkings(info);
        CHECK(m.count(MarkingKind::DisplacedArrow) > 0);
        for (const Poly& p : m.polys) {
            if (p.kind == MarkingKind::DisplacedArrow)
                for (int i = 0; i < p.n; ++i) CHECK(p.alongM[i] < 0);
            if (p.kind == MarkingKind::ThresholdStripe)
                for (int i = 0; i < p.n; ++i) CHECK(p.alongM[i] > 0);
        }
    }
}

TEST_CASE("world: threshold stripe counts by runway width") {
    CHECK(thresholdStripeCount(60) == 4);
    CHECK(thresholdStripeCount(75) == 6);
    CHECK(thresholdStripeCount(100) == 8);
    CHECK(thresholdStripeCount(150) == 12);
    CHECK(thresholdStripeCount(200) == 16);

    Xtst x;  // 150 ft wide
    const RunwayMarkings m = buildMarkings(x.info27);
    CHECK(m.count(MarkingKind::ThresholdStripe) == 12);
    // Six stripes on each side of the centerline, none crossing it, all inside the runway.
    int left = 0, right = 0;
    for (const Poly& p : m.polys) {
        if (p.kind != MarkingKind::ThresholdStripe) continue;
        const double lo = *std::min_element(p.lateralM, p.lateralM + 4);
        const double hi = *std::max_element(p.lateralM, p.lateralM + 4);
        CHECK(lo * hi > 0);
        CHECK(std::max(std::fabs(lo), std::fabs(hi)) < x.info27.widthM * 0.5);
        (hi < 0 ? left : right) += 1;
    }
    CHECK(left == 6);
    CHECK(right == 6);

    // Narrow runway: 75 ft wide gives 6 stripes.
    Runway narrow = x.rwy27;
    narrow.widthFt = 75;
    CHECK(buildMarkings(makeRunwayInfo(x.airport, narrow)).count(MarkingKind::ThresholdStripe) ==
          6);
}

TEST_CASE("world: touchdown zone, aiming point and centerline geometry") {
    Xtst x;
    const RunwayMarkings m = buildMarkings(x.info27);

    // Aiming-point bars: two, starting 1,000 ft from the threshold.
    CHECK(m.count(MarkingKind::AimingPoint) == 2);
    // Touchdown-zone bars: 3 + 2 + 2 + 1 + 1 per side at 500, 1500, 2000, 2500, 3000 ft.
    CHECK(m.count(MarkingKind::TouchdownZone) == 2 * (3 + 2 + 2 + 1 + 1));
    std::vector<double> starts;
    for (const Poly& p : m.polys) {
        const double a = *std::min_element(p.alongM, p.alongM + 4);
        if (p.kind == MarkingKind::AimingPoint) CHECK(a == doctest::Approx(ftToM(1000.0)));
        if (p.kind == MarkingKind::TouchdownZone &&
            std::none_of(starts.begin(), starts.end(),
                         [&](double s) { return std::fabs(s - a) < 1e-6; }))
            starts.push_back(a);
    }
    std::sort(starts.begin(), starts.end());
    REQUIRE(starts.size() == touchdownZoneStations().size());
    CHECK(starts[0] == doctest::Approx(ftToM(500.0)));
    CHECK(starts[1] == doctest::Approx(ftToM(1500.0)));
    CHECK(starts[2] == doctest::Approx(ftToM(2000.0)));
    CHECK(starts[3] == doctest::Approx(ftToM(2500.0)));
    CHECK(starts[4] == doctest::Approx(ftToM(3000.0)));
    // Stations are 500 ft apart apart from the aiming point at 1,000 ft.
    CHECK(touchdownZoneStations().front().alongFt == 500.0);
    CHECK(touchdownZoneStations().back().alongFt == 3000.0);

    // Centerline: 120 ft on, 80 ft off, from past the designator to the middle of the runway.
    std::vector<std::pair<double, double>> dashes;
    for (const Poly& p : m.polys)
        if (p.kind == MarkingKind::Centerline)
            dashes.push_back({*std::min_element(p.alongM, p.alongM + 4),
                              *std::max_element(p.alongM, p.alongM + 4)});
    std::sort(dashes.begin(), dashes.end());
    REQUIRE(dashes.size() > 10);
    for (size_t i = 0; i < dashes.size(); ++i) {
        CHECK(dashes[i].second - dashes[i].first == doctest::Approx(ftToM(120.0)));
        if (i > 0) CHECK(dashes[i].first - dashes[i - 1].first == doctest::Approx(ftToM(200.0)));
        CHECK(dashes[i].first >= m.designatorEndM);
        CHECK(dashes[i].second <= x.info27.usableM() * 0.5 + 1e-6);
    }

    // Edge lines are painted by the primary direction only (09), as one set for the pair.
    CHECK(m.count(MarkingKind::EdgeLine) == 0);  // 27 is not primary
    const RunwayMarkings m09 = buildMarkings(makeRunwayInfo(x.airport, x.rwy09));
    CHECK(m09.count(MarkingKind::EdgeLine) > 0);
}

TEST_CASE("world: visual runways get designator, centerline and threshold only") {
    Xtst x;
    Runway r = x.rwy27;
    r.ils.reset();
    const RunwayMarkings m = buildMarkings(makeRunwayInfo(x.airport, r));
    CHECK(m.count(MarkingKind::ThresholdStripe) == 12);
    CHECK(m.count(MarkingKind::Designator) > 0);
    CHECK(m.count(MarkingKind::Centerline) > 0);
    CHECK(m.count(MarkingKind::TouchdownZone) == 0);
    CHECK(m.count(MarkingKind::AimingPoint) == 0);
    CHECK(m.count(MarkingKind::EdgeLine) == 0);
}

TEST_CASE("world: designator is built from stroke-font strokes and stays on the runway") {
    Xtst x;
    std::string number, suffix;
    splitDesignator("28L", number, suffix);
    CHECK(number == "28");
    CHECK(suffix == "L");
    splitDesignator("09", number, suffix);
    CHECK(number == "09");
    CHECK(suffix.empty());

    const RunwayMarkings plain = buildMarkings(x.info27);  // "27"
    Runway left = x.rwy27;
    left.ident = "27L";
    const RunwayMarkings withLetter = buildMarkings(makeRunwayInfo(x.airport, left));
    CHECK(withLetter.count(MarkingKind::Designator) > plain.count(MarkingKind::Designator));
    CHECK(withLetter.designatorEndM > plain.designatorEndM);  // the letter takes room
    for (const Poly& p : withLetter.polys) {
        if (p.kind != MarkingKind::Designator) continue;
        for (int i = 0; i < p.n; ++i) {
            CHECK(std::fabs(p.lateralM[i]) < x.info27.widthM * 0.5);
            CHECK(p.alongM[i] > kThresholdStripeStartM + kThresholdStripeLengthM);  // past stripes
            CHECK(p.alongM[i] < withLetter.designatorEndM + 2.0);
        }
    }
}

TEST_CASE("world: flatten area quad from runway geometry") {
    Xtst x;
    const RunwayInfo info = makeRunwayInfo(x.airport, x.rwy09);
    const FlattenArea a = runwayFlattenArea(info, 50.0, 100.0, 60.0f);
    CHECK(a.elevM == doctest::Approx(static_cast<float>(ftToM(2000.0))));
    CHECK(a.blendM == 60.0f);

    // Local east/north metres around XTST: counter-clockwise (positive signed area) and about
    // (length + 200 m) x (width + 100 m).
    const double mPerDegLat = kEarthRadiusM * kRadPerDeg;
    const double mPerDegLon = mPerDegLat * std::cos(degToRad(SyntheticNavDb::kLatDeg));
    double xs[4], ys[4];
    for (int i = 0; i < 4; ++i) {
        xs[i] = (a.lonDeg[i] - SyntheticNavDb::kLonDeg) * mPerDegLon;
        ys[i] = (a.latDeg[i] - SyntheticNavDb::kLatDeg) * mPerDegLat;
    }
    double area2 = 0;
    for (int i = 0; i < 4; ++i) area2 += xs[i] * ys[(i + 1) % 4] - xs[(i + 1) % 4] * ys[i];
    CHECK(area2 > 0);
    const double expected = (info.lengthM + 200.0) * (info.widthM + 100.0);
    CHECK(area2 * 0.5 == doctest::Approx(expected).epsilon(0.01));
    // The runway centerline ends are inside; a point 200 m off the side is outside.
    FlatTerrain terrain(100.0f);
    terrain.addFlattenArea(a);
    CHECK(terrain.elevationM(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg) ==
          doctest::Approx(a.elevM));
    CHECK(terrain.elevationM(info.thr.latDeg, info.thr.lonDeg) == doctest::Approx(a.elevM));
    CHECK(terrain.elevationM(SyntheticNavDb::kLatDeg + 200.0 / mPerDegLat,
                             SyntheticNavDb::kLonDeg) == doctest::Approx(100.0f));
}

TEST_CASE("world: airports register one flatten area per runway pair at the mean elevation") {
    Xtst x;
    RecordingTerrain terrain;
    registerAirportFlatten(x.airport, terrain);
    CHECK(terrain.areas.size() == 1);  // 09/27 is one pavement

    // Elevation: mean of the two ends of the primary direction.
    Airport sloped = x.airport;
    for (Runway& r : sloped.runways) {
        const bool is09 = r.ident == "09";
        r.thrElevFt = is09 ? 100.0f : 200.0f;
        r.endElevFt = is09 ? 200.0f : 100.0f;
    }
    terrain.areas.clear();
    registerAirportFlatten(sloped, terrain);
    REQUIRE(terrain.areas.size() == 1);
    CHECK(terrain.areas[0].elevM == doctest::Approx(static_cast<float>(ftToM(150.0))));
    CHECK(runwayFlatElevM(sloped, *findRunway(sloped, "27")) ==
          doctest::Approx(runwayFlatElevM(sloped, *findRunway(sloped, "09"))));
    CHECK(airportLowestFlatElevM(sloped) == doctest::Approx(ftToM(150.0)));
}

TEST_CASE("world: runway meshes are RTC, 16-bit indexed and consistent with the light list") {
    Xtst x;
    const RunwayInfo info09 = makeRunwayInfo(x.airport, x.rwy09);
    const RunwayMeshes m09 = buildRunwayMeshes(info09);
    const RunwayMeshes m27 = buildRunwayMeshes(x.info27);

    CHECK_FALSE(m09.pavement.empty());
    CHECK(m27.pavement.empty());  // the secondary direction does not repeat the pavement
    auto checkMesh = [](const SurfaceMesh& mesh) {
        CHECK(mesh.indices.size() % 3 == 0);
        CHECK(mesh.verts.size() <= 65535);
        for (uint16_t i : mesh.indices) CHECK(i < mesh.verts.size());
        for (const SurfaceVertex& v : mesh.verts) {
            const double len = std::sqrt(double(v.pos[0]) * v.pos[0] + double(v.pos[1]) * v.pos[1] +
                                         double(v.pos[2]) * v.pos[2]);
            CHECK(len < 6000.0);  // float offsets are small: nothing absolute in floats
            const double nlen =
                std::sqrt(double(v.normal[0]) * v.normal[0] + double(v.normal[1]) * v.normal[1] +
                          double(v.normal[2]) * v.normal[2]);
            CHECK(nlen == doctest::Approx(1.0).epsilon(0.01));
        }
    };
    for (const SurfaceMesh& m : m09.pavement) checkMesh(m);
    for (const SurfaceMesh& m : m09.markings) checkMesh(m);
    for (const SurfaceMesh& m : m27.markings) checkMesh(m);
    CHECK_FALSE(m27.markings.empty());

    // Anchor is a real ECEF position near the runway centre.
    const Ecef center = runwayCenterEcef(info09);
    CHECK(distM(m09.pavement[0].anchor, center) < 1.0);
    // Pavement spans the runway: farthest vertex from the anchor is half the length away.
    double far = 0;
    for (const SurfaceVertex& v : m09.pavement[0].verts)
        far = std::max(far, std::sqrt(double(v.pos[0]) * v.pos[0] + double(v.pos[1]) * v.pos[1] +
                                      double(v.pos[2]) * v.pos[2]));
    CHECK(far == doctest::Approx(info09.lengthM * 0.5).epsilon(0.02));

    // Lights.
    CHECK(m27.lights.verts.size() == buildRunwayLights(x.info27).size());
    CHECK(m27.lights.hasPapi);
    double axisLen = 0;
    for (float c : m27.lights.axisEcef) axisLen += double(c) * c;
    CHECK(std::sqrt(axisLen) == doctest::Approx(1.0).epsilon(0.001));
}

TEST_CASE("world: very long meshes split into 16-bit sized pieces") {
    Xtst x;
    const RunwayFrame frame(x.info27);
    SurfaceMeshBuilder b(frame.toEcef(0, 0, 0));
    const float rgb[3] = {1, 1, 1};
    for (int i = 0; i < 20000; ++i) {
        Poly p;
        p.n = 4;
        for (int k = 0; k < 4; ++k) {
            p.alongM[k] = i * 0.01 + (k == 1 || k == 2 ? 0.5 : 0.0);
            p.lateralM[k] = k >= 2 ? 1.0 : 0.0;
        }
        b.addPoly(frame, p, 0.2, rgb);
    }
    const std::vector<SurfaceMesh> meshes = b.finish();
    CHECK(meshes.size() >= 2);
    size_t total = 0;
    for (const SurfaceMesh& m : meshes) {
        CHECK(m.verts.size() <= 65000);
        total += m.indices.size() / 3;
    }
    CHECK(total == 40000);
}

TEST_CASE("world: sequenced flashers sweep at 2 Hz") {
    // One sweep takes 0.5 s; the six flashers fire in order 0..5, then a short dark gap.
    CHECK(flasherActiveIndex(0.0) == 0);
    int last = -1;
    for (double t = 0.0; t < 0.375; t += 0.005) {
        const int idx = flasherActiveIndex(t);
        CHECK(idx >= last);
        last = idx;
    }
    CHECK(last == kFlasherCount - 1);
    CHECK(flasherActiveIndex(0.45) == -1);
    CHECK(flasherActiveIndex(0.5) == flasherActiveIndex(0.0));
    CHECK(flasherActiveIndex(10.0 + 0.1) == flasherActiveIndex(0.1));
}

TEST_CASE("world: airports near a point are selected through the nav database") {
    SyntheticNavDb db;
    CHECK(selectAirportsNear(db, 40.0, -100.0, 10000.0, 6).size() == 1);
    CHECK(selectAirportsNear(db, 40.2, -100.0, 50000.0, 6).size() == 1);  // 22 km away
    CHECK(selectAirportsNear(db, 41.0, -100.0, 50000.0, 6).empty());      // 111 km away
    CHECK(selectAirportsNear(db, 40.0, -100.0, 50000.0, 0).empty());
    CHECK(selectAirportsNear(db, 40.0, -179.9, 50000.0, 6).empty());  // box wrap is safe
}

TEST_CASE("world: apron and buildings sit beside a hard runway") {
    SyntheticNavDb db;
    const AirportExtras extras = buildAirportExtras(db.airport());
    CHECK_FALSE(extras.flat.empty());
    CHECK_FALSE(extras.solid.empty());
    // Everything stays clear of the pavement: the nearest vertex is outside the runway edge.
    const RunwayInfo info = makeRunwayInfo(db.airport(), db.airport().runways[0]);
    const double halfW = info.widthM * 0.5;
    const RunwayFrame frame(info);
    const Ecef c = runwayCenterEcef(info);
    // Distance from the runway centreline (through the anchor, along the runway) >= half width.
    const Vec3d axis = frame.alongDirEcef(info.usableM() * 0.5);
    for (const SurfaceMesh& m : extras.solid) {
        for (const SurfaceVertex& v : m.verts) {
            const double dx = m.anchor.x + v.pos[0] - c.x, dy = m.anchor.y + v.pos[1] - c.y,
                         dz = m.anchor.z + v.pos[2] - c.z;
            const double along = dx * axis.x + dy * axis.y + dz * axis.z;
            const double off =
                std::sqrt(std::max(0.0, dx * dx + dy * dy + dz * dz - along * along));
            CHECK(off > halfW + 50.0);
        }
    }
    // An airport with only a grass strip gets none.
    Airport grass = db.airport();
    for (Runway& r : grass.runways) {
        r.hard = false;
        r.ils.reset();
    }
    CHECK(buildAirportExtras(grass).empty());
}
