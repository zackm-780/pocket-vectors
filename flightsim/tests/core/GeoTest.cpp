#include <doctest/doctest.h>

#include <cmath>

#include "core/Geo.h"
#include "core/Units.h"

using namespace fsim;

TEST_CASE("units: conversions match known values") {
    CHECK(ftToM(1000.0) == doctest::Approx(304.8));
    CHECK(mToFt(304.8) == doctest::Approx(1000.0));
    CHECK(nmToM(1.0) == doctest::Approx(1852.0));
    CHECK(ktToMps(1.0) == doctest::Approx(0.514444).epsilon(1e-5));
    CHECK(mpsToKt(ktToMps(120.0)) == doctest::Approx(120.0));
    CHECK(mpsToFpm(1.0) == doctest::Approx(196.8504).epsilon(1e-5));
    CHECK(fpmToMps(mpsToFpm(2.5)) == doctest::Approx(2.5));
    CHECK(nmToFt(1.0) == doctest::Approx(6076.1155).epsilon(1e-6));
    CHECK(fpsToKt(ktToFps(77.0)) == doctest::Approx(77.0));
    CHECK(lbsToKg(1.0) == doctest::Approx(0.45359237));
    CHECK(inHgToHpa(29.92) == doctest::Approx(1013.21).epsilon(1e-4));
    CHECK(radToDeg(degToRad(33.0)) == doctest::Approx(33.0));
}

TEST_CASE("geo: geodetic to ECEF known values") {
    Ecef e = geodeticToEcef({0, 0, 0});
    CHECK(e.x == doctest::Approx(kWgs84AM));
    CHECK(e.y == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(e.z == doctest::Approx(0.0).epsilon(1e-9));

    e = geodeticToEcef({90, 0, 0});  // north pole: semi-minor axis
    CHECK(e.z == doctest::Approx(kWgs84BM).epsilon(1e-9));

    e = geodeticToEcef({0, 90, 1000});
    CHECK(e.y == doctest::Approx(kWgs84AM + 1000.0));

    // Mount Everest-ish check against an independent computation (lat 27.9881, lon 86.925, 8848 m):
    // radius from the center must exceed the equatorial radius at the surface only by altitude
    // minus the polar flattening; here just sanity-check the magnitude.
    e = geodeticToEcef({27.9881, 86.925, 8848.0});
    const double r = std::sqrt(e.x * e.x + e.y * e.y + e.z * e.z);
    CHECK(r > 6.37e6);
    CHECK(r < 6.395e6);
}

TEST_CASE("geo: ECEF round trip") {
    const GeoPos cases[] = {{40, -100, 609.6}, {-33.9, 151.2, 3.0}, {0, 0, 0},
                            {89.5, 10, 12000}, {-89.0, -170, 0},    {51.5, -0.1, 400000}};
    for (const GeoPos& p : cases) {
        const GeoPos q = ecefToGeodetic(geodeticToEcef(p));
        CHECK(q.latDeg == doctest::Approx(p.latDeg).epsilon(1e-9));
        CHECK(q.lonDeg == doctest::Approx(p.lonDeg).epsilon(1e-9));
        CHECK(q.altMslM == doctest::Approx(p.altMslM).epsilon(1e-6));
    }
}

TEST_CASE("geo: great circle distance and bearing known values") {
    // One degree of latitude on the mean sphere.
    CHECK(greatCircleDistM(0, 0, 1, 0) == doctest::Approx(111194.93).epsilon(1e-5));
    // A quarter of the equator.
    CHECK(greatCircleDistM(0, 0, 0, 90) == doctest::Approx(kPi / 2 * kEarthRadiusM).epsilon(1e-9));
    // LAX to JFK is about 3,974 km.
    CHECK(greatCircleDistM(33.9425, -118.4081, 40.6398, -73.7789) / 1000.0 ==
          doctest::Approx(3974.0).epsilon(0.003));
    CHECK(greatCircleDistM(10, 20, 10, 20) == doctest::Approx(0.0));

    CHECK(radToDeg(initialBearingTrueRad(0, 0, 0, 1)) == doctest::Approx(90.0));
    CHECK(radToDeg(initialBearingTrueRad(0, 0, 1, 0)) == doctest::Approx(0.0).epsilon(1e-9));
    CHECK(radToDeg(initialBearingTrueRad(0, 0, -1, 0)) == doctest::Approx(180.0));
    CHECK(radToDeg(initialBearingTrueRad(0, 0, 0, -1)) == doctest::Approx(270.0));
    // LAX to JFK initial bearing is about 65.9 degrees true.
    CHECK(radToDeg(initialBearingTrueRad(33.9425, -118.4081, 40.6398, -73.7789)) ==
          doctest::Approx(65.9).epsilon(0.005));
}

TEST_CASE("geo: destination point round trips with distance and bearing") {
    double lat = 0, lon = 0;
    destinationPoint(0, 0, 0, 111194.93, lat, lon);
    CHECK(lat == doctest::Approx(1.0).epsilon(1e-6));
    CHECK(lon == doctest::Approx(0.0).epsilon(1e-9));

    const double startLat = 40.0, startLon = -100.0, bearing = degToRad(123.0), dist = 87654.0;
    destinationPoint(startLat, startLon, bearing, dist, lat, lon);
    CHECK(greatCircleDistM(startLat, startLon, lat, lon) == doctest::Approx(dist).epsilon(1e-9));
    CHECK(initialBearingTrueRad(startLat, startLon, lat, lon) ==
          doctest::Approx(bearing).epsilon(1e-9));
}

TEST_CASE("geo: cross-track is positive to the right, along-track toward the end") {
    const LatLon start{0, 0}, end{0, 1};  // eastbound leg
    // South of an eastbound track is on the right.
    const double xtkRight = crossTrackM({-0.01, 0.5}, start, end);
    const double xtkLeft = crossTrackM({0.01, 0.5}, start, end);
    CHECK(xtkRight == doctest::Approx(1111.95).epsilon(1e-3));
    CHECK(xtkLeft == doctest::Approx(-1111.95).epsilon(1e-3));
    CHECK(crossTrackM({0, 0.5}, start, end) == doctest::Approx(0.0).epsilon(1e-9));

    CHECK(alongTrackM({-0.01, 0.5}, start, end) == doctest::Approx(0.5 * 111194.93).epsilon(1e-3));
    CHECK(alongTrackM({0, -0.25}, start, end) < 0.0);  // behind the start
}

TEST_CASE("geo: NED to ECEF rotation is orthonormal with the right axes") {
    Mat3d r = nedToEcef(0, 0);  // at lat 0 lon 0: north = +z, east = +y, down = -x
    CHECK(r.m[2][0] == doctest::Approx(1.0));
    CHECK(r.m[1][1] == doctest::Approx(1.0));
    CHECK(r.m[0][2] == doctest::Approx(-1.0));

    r = nedToEcef(40, -100);
    const Mat3d should_be_identity = transpose(r) * r;
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            CHECK(should_be_identity.m[i][j] == doctest::Approx(i == j ? 1.0 : 0.0).epsilon(1e-12));
    const double det = r.m[0][0] * (r.m[1][1] * r.m[2][2] - r.m[1][2] * r.m[2][1]) -
                       r.m[0][1] * (r.m[1][0] * r.m[2][2] - r.m[1][2] * r.m[2][0]) +
                       r.m[0][2] * (r.m[1][0] * r.m[2][1] - r.m[1][1] * r.m[2][0]);
    CHECK(det == doctest::Approx(1.0));
}

TEST_CASE("geo: body to NED follows JSBSim yaw-pitch-roll") {
    // Heading east, level: body x (nose) points east.
    Vec3d nose = bodyToNed(0, 0, degToRad(90)) * Vec3d{1, 0, 0};
    CHECK(nose.x == doctest::Approx(0.0).epsilon(1e-12));
    CHECK(nose.y == doctest::Approx(1.0));
    CHECK(nose.z == doctest::Approx(0.0).epsilon(1e-12));

    // Pitch up 10 degrees heading north: nose rises (NED z is down, so negative).
    nose = bodyToNed(0, degToRad(10), 0) * Vec3d{1, 0, 0};
    CHECK(nose.x == doctest::Approx(std::cos(degToRad(10))));
    CHECK(nose.z == doctest::Approx(-std::sin(degToRad(10))));

    // Roll right 30 degrees: the right wing (body y) goes down (positive NED z).
    const Vec3d wing = bodyToNed(degToRad(30), 0, 0) * Vec3d{0, 1, 0};
    CHECK(wing.z == doctest::Approx(std::sin(degToRad(30))));
}

TEST_CASE("geo: angle wrapping") {
    CHECK(wrapPi(3 * kPi / 2) == doctest::Approx(-kPi / 2));
    CHECK(wrapPi(-3 * kPi / 2) == doctest::Approx(kPi / 2));
    CHECK(wrapPi(kPi) == doctest::Approx(kPi));
    CHECK(wrap2Pi(-kPi / 2) == doctest::Approx(3 * kPi / 2));
    CHECK(wrap2Pi(5 * kTwoPi + 0.25) == doctest::Approx(0.25));
    CHECK(wrap360(-10.0) == doctest::Approx(350.0));
    CHECK(wrap360(725.0) == doctest::Approx(5.0));
    CHECK(wrap180(190.0) == doctest::Approx(-170.0));
    CHECK(wrap180(-190.0) == doctest::Approx(170.0));
    CHECK(wrap180(180.0) == doctest::Approx(180.0));
}
