// HUD conformality: pitch ladder lines, the horizon and the flight path vector must land where
// the 3D scene's own projection (gfx::Camera::projectToHud) puts the same world direction.
#include <doctest/doctest.h>

#include <cmath>

#include "core/AircraftState.h"
#include "core/Units.h"
#include "gfx/Camera.h"
#include "hud/HudProjector.h"

using namespace fsim;

namespace {

constexpr double kLatDeg = 40.0, kLonDeg = -100.0;

AircraftState makeState(double headingDeg, double pitchDeg, double rollDeg) {
    AircraftState s;
    s.valid = true;
    s.latDeg = kLatDeg;
    s.lonDeg = kLonDeg;
    s.altMslFt = 3000;
    s.altAglFt = 1000;
    s.headingTrueRad = static_cast<float>(degToRad(headingDeg));
    s.pitchRad = static_cast<float>(degToRad(pitchDeg));
    s.rollRad = static_cast<float>(degToRad(rollDeg));
    return s;
}

gfx::Camera makeCamera(const AircraftState& s, float vFovDeg = 55.0f) {
    gfx::Camera cam;
    cam.setLens(vFovDeg);
    cam.setViewport(640, 480);
    cam.setPoseFromAircraft(s);
    return cam;
}

float focalPx(float vFovDeg) {
    return 240.0f / std::tan(static_cast<float>(degToRad(vFovDeg)) * 0.5f);
}

}  // namespace

TEST_CASE("hud: boresight direction projects to the screen center") {
    const AircraftState s = makeState(90, 5, 0);
    const gfx::Camera cam = makeCamera(s);
    const HudProjector proj(cam, kLatDeg, kLonDeg);
    float x = 0, y = 0;
    REQUIRE(proj.projectAzEl(s.headingTrueRad, s.pitchRad, x, y));
    CHECK(x == doctest::Approx(320.0f).epsilon(1e-4));
    CHECK(y == doctest::Approx(240.0f).epsilon(1e-4));
}

TEST_CASE("hud: pitch ladder lines sit at the camera's own projection of the same elevation") {
    const float f = focalPx(55.0f);
    for (double pitchDeg : {-10.0, 0.0, 5.0, 20.0}) {
        const AircraftState s = makeState(90, pitchDeg, 0);
        const gfx::Camera cam = makeCamera(s);
        const HudProjector proj(cam, kLatDeg, kLonDeg);
        for (int lineDeg = -30; lineDeg <= 30; lineDeg += 5) {
            float x = 0, y = 0;
            REQUIRE(proj.projectAzEl(s.headingTrueRad, degToRad(lineDeg), x, y));
            // Independent expectation: pure pitch rotation about the lateral axis.
            const float expectedY = 240.0f - f * std::tan(static_cast<float>(degToRad(lineDeg - pitchDeg)));
            CHECK(x == doctest::Approx(320.0f).epsilon(1e-3));
            CHECK(y == doctest::Approx(expectedY).epsilon(1e-3));
            // And the same point through the camera API directly (NED direction -> eye -> HUD).
            const double el = degToRad(lineDeg);
            float eye[3], x2 = 0, y2 = 0;
            proj.nedToEye(std::cos(el) * std::cos(s.headingTrueRad),
                          std::cos(el) * std::sin(s.headingTrueRad), -std::sin(el), eye);
            REQUIRE(cam.projectToHud(eye, x2, y2));
            CHECK(x2 == doctest::Approx(x));
            CHECK(y2 == doctest::Approx(y));
        }
    }
}

TEST_CASE("hud: ladder lines and the horizon rotate with bank") {
    const double rollDeg = 20.0;
    const AircraftState s = makeState(90, 0, rollDeg);
    const gfx::Camera cam = makeCamera(s);
    const HudProjector proj(cam, kLatDeg, kLonDeg);
    const float f = focalPx(55.0f);
    const float roll = static_cast<float>(degToRad(rollDeg));

    // The 10 degree line's center is f*tan(10) away from the boresight, toward "up" rotated
    // counter-clockwise by the roll (right bank turns the world counter-clockwise on screen).
    float x = 0, y = 0;
    REQUIRE(proj.projectAzEl(s.headingTrueRad, degToRad(10), x, y));
    const float d = f * std::tan(static_cast<float>(degToRad(10)));
    CHECK(x == doctest::Approx(320.0f - d * std::sin(roll)).epsilon(1e-3));
    CHECK(y == doctest::Approx(240.0f - d * std::cos(roll)).epsilon(1e-3));

    // The horizon is a straight line at angle -roll through the boresight when pitch = 0.
    float ax, ay, bx, by;
    REQUIRE(proj.projectAzEl(s.headingTrueRad - degToRad(30), 0, ax, ay));
    REQUIRE(proj.projectAzEl(s.headingTrueRad + degToRad(30), 0, bx, by));
    const float angle = std::atan2(by - ay, bx - ax);
    CHECK(angle == doctest::Approx(-roll).epsilon(1e-3));
    // Right wing down: the right end of the horizon is higher on screen (smaller y).
    CHECK(by < ay);
}

TEST_CASE("hud: look direction changes are followed by the projection") {
    // Looking 20 degrees right (camera yaw): a point straight ahead moves to the left.
    const AircraftState s = makeState(90, 0, 0);
    gfx::Camera cam;
    cam.setLens(55.0f);
    cam.setViewport(640, 480);
    cam.setPoseFromAircraft(s, static_cast<float>(degToRad(20)), 0);
    const HudProjector proj(cam, kLatDeg, kLonDeg);
    float x = 0, y = 0;
    REQUIRE(proj.projectAzEl(s.headingTrueRad, 0, x, y));
    CHECK(x == doctest::Approx(320.0f - focalPx(55.0f) * std::tan(static_cast<float>(degToRad(20)))).epsilon(1e-3));
    CHECK(y == doctest::Approx(240.0f).epsilon(1e-3));
    // Looking back: the heading direction is behind the eye.
    cam.setPoseFromAircraft(s, static_cast<float>(degToRad(180)), 0);
    CHECK_FALSE(proj.projectAzEl(s.headingTrueRad, 0, x, y));
}

TEST_CASE("hud: flight path vector follows the velocity vector") {
    const float f = focalPx(55.0f);
    AircraftState s = makeState(90, 3, 0);
    const gfx::Camera cam = makeCamera(s);
    const HudProjector proj(cam, kLatDeg, kLonDeg);
    const float rx = 120, ry = 70, rw = 400, rh = 330;

    // Flying exactly along the nose: FPV on the boresight.
    const double pitch = s.pitchRad;
    s.velNorthMps = 0;
    s.velEastMps = static_cast<float>(60.0 * std::cos(pitch));
    s.velDownMps = static_cast<float>(-60.0 * std::sin(pitch));
    HudFpv fpv = computeFpv(proj, s, rx, ry, rw, rh);
    REQUIRE(fpv.valid);
    CHECK_FALSE(fpv.clamped);
    CHECK(fpv.xHud == doctest::Approx(320.0f).epsilon(1e-3));
    CHECK(fpv.yHud == doctest::Approx(240.0f).epsilon(1e-3));

    // Level flight with the nose 3 degrees up: the FPV sits 3 degrees below the boresight.
    s.velEastMps = 60;
    s.velDownMps = 0;
    fpv = computeFpv(proj, s, rx, ry, rw, rh);
    CHECK(fpv.xHud == doctest::Approx(320.0f).epsilon(1e-3));
    CHECK(fpv.yHud == doctest::Approx(240.0f + f * std::tan(static_cast<float>(degToRad(3)))).epsilon(1e-3));

    // Track 10 degrees to the right of the heading (crab): the FPV is right of the boresight.
    AircraftState level = makeState(90, 0, 0);
    const gfx::Camera cam2 = makeCamera(level);
    const HudProjector proj2(cam2, kLatDeg, kLonDeg);
    const double trackRad = degToRad(100);
    level.velNorthMps = static_cast<float>(60 * std::cos(trackRad));
    level.velEastMps = static_cast<float>(60 * std::sin(trackRad));
    fpv = computeFpv(proj2, level, rx, ry, rw, rh);
    CHECK(fpv.xHud == doctest::Approx(320.0f + f * std::tan(static_cast<float>(degToRad(10)))).epsilon(2e-3));
    CHECK(fpv.yHud == doctest::Approx(240.0f).epsilon(1e-3));

    // Descending at 5 m/s with 50 m/s ground speed: -5.7 degrees, below the nose by that much.
    level.velNorthMps = 0;
    level.velEastMps = 50;
    level.velDownMps = 5;
    fpv = computeFpv(proj2, level, rx, ry, rw, rh);
    CHECK(fpv.yHud == doctest::Approx(240.0f + f * 0.1f).epsilon(1e-3));
}

TEST_CASE("hud: flight path vector is clamped at the clip edge, and hidden when stationary") {
    AircraftState s = makeState(90, 0, 0);
    const gfx::Camera cam = makeCamera(s);
    const HudProjector proj(cam, kLatDeg, kLonDeg);
    const float rx = 120, ry = 70, rw = 400, rh = 330;
    // Steep dive: way outside the ladder rect.
    s.velEastMps = 30;
    s.velDownMps = 120;
    HudFpv fpv = computeFpv(proj, s, rx, ry, rw, rh);
    REQUIRE(fpv.valid);
    CHECK(fpv.clamped);
    CHECK(fpv.yHud == doctest::Approx(ry + rh));
    CHECK(fpv.xHud >= rx);
    CHECK(fpv.xHud <= rx + rw);
    // Moving backwards (behind the eye): still parked on the edge, never NaN.
    s.velEastMps = -80;
    s.velDownMps = 0;
    fpv = computeFpv(proj, s, rx, ry, rw, rh);
    CHECK(fpv.clamped);
    CHECK(std::isfinite(fpv.xHud));
    CHECK(std::isfinite(fpv.yHud));
    // Parked: no velocity, no vector.
    s.velEastMps = s.velNorthMps = s.velDownMps = 0.5f;
    CHECK_FALSE(computeFpv(proj, s, rx, ry, rw, rh).valid);
}
