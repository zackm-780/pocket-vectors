// Synthesized ILS geometry, real ILS attachment and MagVar interpolation.
#include <doctest/doctest.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

#include "core/Geo.h"
#include "core/Units.h"
#include "nav/MagVar.h"
#include "nav/NavDb.h"
#include "nav/NavDbIls.h"

using namespace fsim;

namespace {

std::string fixtureDir() {
    std::string f = __FILE__;  // .../tests/nav/navdb_ils_magvar_test.cpp
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    return f + "/fixtures/nav";
}

const CsvNavDatabase& db() {
    static CsvNavDatabase d;
    static bool loaded = false;
    if (!loaded) {
        std::string err;
        REQUIRE_MESSAGE(d.load(fixtureDir(), err), err);
        loaded = true;
    }
    return d;
}

const Runway& rwy(const char* apt, const char* id) {
    const Airport* a = db().findAirport(apt);
    REQUIRE(a != nullptr);
    for (const Runway& r : a->runways)
        if (r.ident == id) return r;
    FAIL(apt << " " << id << " not found");
    return a->runways[0];
}

// A level 5,000 ft runway on heading `headingDeg`, threshold at (45, -100).
Runway makeRunway(double headingDeg, double lengthFt) {
    Runway r;
    r.ident = "09";
    r.thrLatDeg = 45.0;
    r.thrLonDeg = -100.0;
    destinationPoint(r.thrLatDeg, r.thrLonDeg, degToRad(headingDeg), ftToM(lengthFt), r.endLatDeg,
                     r.endLonDeg);
    r.startLatDeg = r.thrLatDeg;
    r.startLonDeg = r.thrLonDeg;
    r.thrElevFt = r.endElevFt = 1234.0f;
    r.headingTrueDeg = static_cast<float>(headingDeg);
    r.lengthFt = static_cast<float>(lengthFt);
    r.widthFt = 100;
    r.hard = true;
    return r;
}

std::vector<uint8_t> magvarBytes(int latMin, int latMax, int lonMin, int lonMax, int step,
                                 const std::vector<int16_t>& values) {
    std::vector<uint8_t> b = {'M', 'A', 'G', 'V', 'A', 'R', '0', '1'};
    const float epoch = 2026.0f;
    uint8_t e[4];
    std::memcpy(e, &epoch, 4);
    b.insert(b.end(), e, e + 4);
    for (int v : {latMin, latMax, lonMin, lonMax, step, 0}) {
        b.push_back(static_cast<uint8_t>(v & 0xff));
        b.push_back(static_cast<uint8_t>((v >> 8) & 0xff));
    }
    for (int16_t v : values) {
        b.push_back(static_cast<uint8_t>(v & 0xff));
        b.push_back(static_cast<uint8_t>((static_cast<uint16_t>(v) >> 8) & 0xff));
    }
    return b;
}

}  // namespace

TEST_CASE("nav: synthesized ILS geometry") {
    const Runway r = makeRunway(90.0, 5000.0);
    const Ils ils = synthesizeIls(r);
    CHECK(ils.synthesized);
    CHECK(ils.hasGs);
    CHECK(std::string(ils.ident) == "I09");
    CHECK(ils.locCourseTrueDeg == doctest::Approx(90.0));
    CHECK(ils.gsAngleDeg == doctest::Approx(3.0));
    CHECK(ils.tchFt == doctest::Approx(50.0));
    CHECK(ils.gsElevFt == doctest::Approx(1234.0));

    // LOC antenna: 1,000 ft beyond the far end, on the extended centerline.
    CHECK(greatCircleDistM(r.endLatDeg, r.endLonDeg, ils.locLatDeg, ils.locLonDeg) ==
          doctest::Approx(ftToM(1000.0)).epsilon(1e-4));
    const LatLon thr{r.thrLatDeg, r.thrLonDeg}, end{r.endLatDeg, r.endLonDeg};
    const LatLon loc{ils.locLatDeg, ils.locLonDeg};
    CHECK(std::fabs(crossTrackM(loc, thr, end)) < 0.5);
    CHECK(alongTrackM(loc, thr, end) == doctest::Approx(ftToM(6000.0)).epsilon(1e-4));

    // LOC width: +-350 ft at the threshold. Antenna is 6,000 ft from the threshold.
    const double halfDeg = radToDeg(std::atan(350.0 / 6000.0));  // 3.34 -> clamped to 3.0
    CHECK(halfDeg > 3.0);
    CHECK(ils.locWidthDeg == doctest::Approx(6.0));

    // GS antenna: TCH / tan(3 deg) = 954 ft past the threshold, on the centerline.
    const LatLon gs{ils.gsLatDeg, ils.gsLonDeg};
    const double gsDistFt = 50.0 / std::tan(degToRad(3.0));
    CHECK(gsDistFt == doctest::Approx(954.1).epsilon(0.001));
    CHECK(alongTrackM(gs, thr, end) == doctest::Approx(ftToM(gsDistFt)).epsilon(1e-3));
    CHECK(std::fabs(crossTrackM(gs, thr, end)) < 0.5);
}

TEST_CASE("nav: localizer width clamps between 1.5 and 3.0 degrees half-width") {
    // Antenna D ft from the threshold on the extended centerline of a north-south line.
    auto widthAt = [](double distFt) {
        double lat, lon;
        destinationPoint(45.0, -100.0, 0.0, ftToM(distFt), lat, lon);
        return locWidthDegForGeometry(lat, lon, 45.0, -100.0);
    };
    CHECK(widthAt(3500.0) == doctest::Approx(6.0));  // 5.7 deg raw -> 3.0 max
    CHECK(widthAt(7000.0) ==
          doctest::Approx(2.0 * radToDeg(std::atan(350.0 / 7000.0))).epsilon(0.002));
    CHECK(widthAt(7000.0) == doctest::Approx(5.72).epsilon(0.01));
    CHECK(widthAt(20000.0) == doctest::Approx(3.0));  // 1.0 deg raw -> 1.5 min
    CHECK(widthAt(13000.0) ==
          doctest::Approx(2.0 * radToDeg(std::atan(350.0 / 13000.0))).epsilon(0.002));
    // Degenerate geometry (antenna on the threshold) falls back to the maximum.
    CHECK(locWidthDegForGeometry(45.0, -100.0, 45.0, -100.0) == doctest::Approx(6.0));
}

TEST_CASE("nav: synthesized ILS qualifies only hard, exact runways of 3,000 ft or more") {
    Runway r = makeRunway(90.0, 3000.0);
    CHECK(runwayQualifiesForSynthIls(r));
    r.lengthFt = 2999.0f;
    CHECK_FALSE(runwayQualifiesForSynthIls(r));
    r.lengthFt = 5000.0f;
    r.hard = false;
    CHECK_FALSE(runwayQualifiesForSynthIls(r));
    r.hard = true;
    r.approxGeometry = true;
    CHECK_FALSE(runwayQualifiesForSynthIls(r));
}

TEST_CASE("nav: real ILS rows win, others are synthesized, approx runways get none") {
    // Real: KPDX 28L (JMJ, 110.5), width empty in the file so derived, TCH empty -> 50.
    const Runway& r28l = rwy("KPDX", "28L");
    REQUIRE(r28l.ils.has_value());
    CHECK_FALSE(r28l.ils->synthesized);
    CHECK(std::string(r28l.ils->ident) == "JMJ");
    CHECK(r28l.ils->freqMhz == doctest::Approx(110.5));
    CHECK(r28l.ils->hasGs);
    CHECK(r28l.ils->tchFt == doctest::Approx(50.0));
    CHECK(r28l.ils->gsAngleDeg == doctest::Approx(3.0));
    CHECK(r28l.ils->locCourseTrueDeg == doctest::Approx(299.03));
    CHECK(r28l.ils->gsElevFt == doctest::Approx(19.9));
    const float expectedWidth = locWidthDegForGeometry(r28l.ils->locLatDeg, r28l.ils->locLonDeg,
                                                       r28l.thrLatDeg, r28l.thrLonDeg);
    CHECK(r28l.ils->locWidthDeg == doctest::Approx(expectedWidth));
    CHECK(r28l.ils->locWidthDeg >= 3.0f);
    CHECK(r28l.ils->locWidthDeg <= 6.0f);
    // Real GS position comes from the file, not from synthesis (about 1,000 ft past the threshold
    // but not exactly 954 ft).
    CHECK(r28l.ils->gsLatDeg == doctest::Approx(45.581287));

    // Real ILS with a non-default glideslope angle.
    const Runway& sfo = rwy("KSFO", "28L");
    REQUIRE(sfo.ils.has_value());
    CHECK(sfo.ils->gsAngleDeg == doctest::Approx(2.85));
    CHECK_FALSE(sfo.ils->synthesized);

    // Localizer-only row (KPDX 21, LOC GPO): attached as real, no glideslope, not synthesized.
    const Runway& r21 = rwy("KPDX", "21");
    REQUIRE(r21.ils.has_value());
    CHECK_FALSE(r21.ils->synthesized);
    CHECK_FALSE(r21.ils->hasGs);
    CHECK(std::string(r21.ils->ident) == "GPO");

    // No real ILS: hard, exact, long enough -> synthesized (KOTH 23 and KPDX 03).
    for (const Runway* r :
         {&rwy("KOTH", "23"), &rwy("KPDX", "03"), &rwy("KOTH", "13"), &rwy("KTTD", "07")}) {
        REQUIRE_MESSAGE(r->ils.has_value(), r->airportIdent << " " << r->ident);
        CHECK(r->ils->synthesized);
        CHECK(r->ils->hasGs);
    }
    // ...and the geometry matches the spec (KOTH 23: far end is the KOTH 05 threshold).
    const Runway& r23 = rwy("KOTH", "23");
    CHECK(greatCircleDistM(r23.endLatDeg, r23.endLonDeg, r23.ils->locLatDeg, r23.ils->locLonDeg) ==
          doctest::Approx(ftToM(1000.0)).epsilon(1e-3));
    const LatLon thr{r23.thrLatDeg, r23.thrLonDeg}, end{r23.endLatDeg, r23.endLonDeg};
    CHECK(alongTrackM({r23.ils->gsLatDeg, r23.ils->gsLonDeg}, thr, end) ==
          doctest::Approx(ftToM(50.0 / std::tan(degToRad(3.0)))).epsilon(2e-3));

    // Real ILS on the other end of a runway does not stop the reciprocal from being synthesized.
    CHECK(rwy("KOTH", "05").ils.has_value());
    CHECK_FALSE(rwy("KOTH", "05").ils->synthesized);

    // No ILS: short hard runway (2,751 ft), approx hard runway (4,100 ft), grass runway.
    CHECK_FALSE(rwy("04W", "06").ils.has_value());
    const Runway& approx = rwy("04WN", "11");
    CHECK(approx.hard);
    CHECK(approx.approxGeometry);
    CHECK(approx.lengthFt >= 3000.0f);
    CHECK_FALSE(approx.ils.has_value());
    CHECK_FALSE(rwy("00AL", "01").ils.has_value());

    CHECK(db().stats().realIls == 26);
    CHECK(db().stats().synthIls > 5);
}

TEST_CASE("nav: MagVar bilinear interpolation and wrap") {
    // 10 degree grid, lat -90..90 (19 rows), lon -180..180 (37 cols), value = row*10 + col in
    // centidegrees so every node is distinct.
    std::vector<int16_t> v;
    for (int row = 0; row < 19; ++row)
        for (int col = 0; col < 37; ++col) v.push_back(static_cast<int16_t>(row * 100 + col * 10));
    const auto bytes = magvarBytes(-90, 90, -180, 180, 10, v);
    MagVar mv;
    std::string err;
    REQUIRE_MESSAGE(mv.loadBytes(bytes.data(), bytes.size(), err), err);
    CHECK(mv.epochYear() == doctest::Approx(2026.0));
    CHECK(mv.rows() == 19);
    CHECK(mv.cols() == 37);

    // Nodes are exact: lat 0 = row 9, lon 0 = col 18.
    CHECK(mv.declinationDeg(0.0, 0.0) == doctest::Approx((9 * 100 + 18 * 10) / 100.0));
    CHECK(mv.declinationDeg(-90.0, -180.0) == doctest::Approx(0.0));
    // 180 is the same meridian as -180, so it reads column 0.
    CHECK(mv.declinationDeg(90.0, 180.0) == doctest::Approx(18.0));
    CHECK(mv.declinationDeg(90.0, 179.999) == doctest::Approx(21.6).epsilon(0.001));
    // Midpoints are the average of the neighbours; quarter points weight 1:3 (value is linear).
    CHECK(mv.declinationDeg(5.0, 0.0) == doctest::Approx((9 * 100 + 18 * 10 + 50) / 100.0));
    CHECK(mv.declinationDeg(0.0, 2.5) == doctest::Approx((9 * 100 + 18 * 10 + 2.5) / 100.0));
    CHECK(mv.declinationDeg(2.5, 7.5) == doctest::Approx((9 * 100 + 18 * 10 + 25 + 7.5) / 100.0));
    // Longitude wraps (190 == -170), latitude clamps.
    CHECK(mv.declinationDeg(0.0, 190.0) == doctest::Approx(mv.declinationDeg(0.0, -170.0)));
    CHECK(mv.declinationDeg(0.0, -541.0) == doctest::Approx(mv.declinationDeg(0.0, 179.0)));
    CHECK(mv.declinationDeg(100.0, 0.0) == doctest::Approx(mv.declinationDeg(90.0, 0.0)));
}

TEST_CASE("nav: MagVar negative values, little-endian parse and bad files") {
    // 2x2 grid over lat 0..10, lon 0..10: east positive, west negative (centidegrees).
    const auto bytes = magvarBytes(0, 10, 0, 10, 10, {-1250, 300, 100, 1650});
    MagVar mv;
    std::string err;
    REQUIRE_MESSAGE(mv.loadBytes(bytes.data(), bytes.size(), err), err);
    CHECK(mv.declinationDeg(0.0, 0.0) == doctest::Approx(-12.5));
    CHECK(mv.declinationDeg(10.0, 10.0) == doctest::Approx(16.5));
    CHECK(mv.declinationDeg(5.0, 5.0) == doctest::Approx((-12.5 + 3.0 + 1.0 + 16.5) / 4.0));
    // Regional grid: longitude clamps instead of wrapping.
    CHECK(mv.declinationDeg(0.0, 50.0) == doctest::Approx(3.0));

    MagVar bad;
    CHECK_FALSE(bad.loadBytes(bytes.data(), 10, err));
    CHECK_FALSE(bad.valid());
    CHECK(bad.declinationDeg(1.0, 1.0) == 0.0f);
    auto wrongMagic = bytes;
    wrongMagic[7] = '9';
    CHECK_FALSE(bad.loadBytes(wrongMagic.data(), wrongMagic.size(), err));
    auto truncated = bytes;
    truncated.pop_back();
    CHECK_FALSE(bad.loadBytes(truncated.data(), truncated.size(), err));
    CHECK_FALSE(bad.loadFile("/nonexistent/magvar.bin", err));
}
