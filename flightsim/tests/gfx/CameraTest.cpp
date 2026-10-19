// Camera and RTC rendering rules (no GL context needed).
#include <doctest/doctest.h>

#include <cmath>
#include <initializer_list>

#include "core/AircraftState.h"
#include "core/Geo.h"
#include "gfx/Camera.h"
#include "gfx/Mat4.h"

using namespace fsim;
using namespace fsim::gfx;

namespace {

AircraftState level(double latDeg, double lonDeg, double altMslFt, double headingDeg) {
    AircraftState s;
    s.valid = true;
    s.latDeg = latDeg;
    s.lonDeg = lonDeg;
    s.altMslFt = altMslFt;
    s.altAglFt = static_cast<float>(altMslFt);
    s.headingTrueRad = static_cast<float>(degToRad(headingDeg));
    return s;
}

// Direction of a local NED unit vector in eye axes for the given camera.
Vec3d nedToEye(const Camera& cam, double latDeg, double lonDeg, const Vec3d& ned) {
    return cam.ecefToEye() * (nedToEcef(latDeg, lonDeg) * ned);
}

}  // namespace

TEST_CASE("camera: looking along the heading, forward is -z, right is +x, up is +y") {
    for (double heading : {0.0, 90.0, 225.0}) {
        Camera cam;
        AircraftState s = level(40, -100, 2000, heading);
        cam.setPoseFromAircraft(s);
        const double h = degToRad(heading);
        // The direction the nose points, as a NED vector.
        const Vec3d fwd = nedToEye(cam, 40, -100, {std::cos(h), std::sin(h), 0});
        CHECK(std::fabs(fwd.x) < 1e-6);
        CHECK(std::fabs(fwd.y) < 1e-6);
        CHECK(fwd.z == doctest::Approx(-1.0));
        // 90 degrees clockwise from the nose (seen from above) is to the pilot's right.
        const Vec3d right = nedToEye(cam, 40, -100, {-std::sin(h), std::cos(h), 0});
        CHECK(right.x == doctest::Approx(1.0));
        // NED down is eye -y.
        const Vec3d down = nedToEye(cam, 40, -100, {0, 0, 1});
        CHECK(down.y == doctest::Approx(-1.0));
    }
}

TEST_CASE("camera: pitch up tilts the view up, roll right drops the right wing") {
    Camera cam;
    AircraftState s = level(40, -100, 2000, 0);
    s.pitchRad = static_cast<float>(degToRad(10));
    cam.setPoseFromAircraft(s);
    // Horizontal north (the horizon ahead) appears below the view center when the nose is up.
    const Vec3d north = nedToEye(cam, 40, -100, {1, 0, 0});
    CHECK(north.y < 0);
    CHECK(north.y == doctest::Approx(-std::sin(degToRad(10))));

    s = level(40, -100, 2000, 0);
    s.rollRad = static_cast<float>(degToRad(30));
    cam.setPoseFromAircraft(s);
    // Rolling right 30 degrees: the world "up" direction (NED -z) leans toward the pilot's left.
    const Vec3d up = nedToEye(cam, 40, -100, {0, 0, -1});
    CHECK(up.x < 0);
    CHECK(up.x == doctest::Approx(-std::sin(degToRad(30))));
}

TEST_CASE("camera: look yaw and pitch rotate relative to the aircraft") {
    Camera cam;
    AircraftState s = level(40, -100, 2000, 0);
    cam.setPoseFromAircraft(s, static_cast<float>(degToRad(90)), 0);  // look right
    const Vec3d east = nedToEye(cam, 40, -100, {0, 1, 0});
    CHECK(east.z == doctest::Approx(-1.0));
    cam.setPoseFromAircraft(s, 0, static_cast<float>(degToRad(30)));  // look up
    const Vec3d north = nedToEye(cam, 40, -100, {1, 0, 0});
    CHECK(north.y == doctest::Approx(-std::sin(degToRad(30))));
}

TEST_CASE("camera: eye point offset moves the camera along the body axes") {
    Camera a, b;
    AircraftState s = level(40, -100, 2000, 90);  // heading east
    a.setPoseFromAircraft(s);
    s.eyeBodyM[0] = 2.0f;   // 2 m forward of the CG
    s.eyeBodyM[2] = -1.0f;  // 1 m up (body z is down)
    b.setPoseFromAircraft(s);
    // 2 m east, 1 m up.
    const GeoPos ga = a.eyeGeo(), gb = b.eyeGeo();
    CHECK(gb.altMslM - ga.altMslM == doctest::Approx(1.0).epsilon(1e-3));
    const double dEastM = (gb.lonDeg - ga.lonDeg) * kRadPerDeg * kWgs84AM * std::cos(degToRad(40));
    CHECK(dEastM == doctest::Approx(2.0).epsilon(0.01));
    CHECK(gb.latDeg == doctest::Approx(ga.latDeg).epsilon(1e-9));
}

TEST_CASE("camera: far plane follows the horizon distance within the 40..400 km limits") {
    Camera cam;
    cam.setPoseFromAircraft(level(40, -100, 100, 0));
    CHECK(cam.farM() == doctest::Approx(40000.0f));
    cam.setPoseFromAircraft(level(40, -100, 20000, 0));  // FL200: horizon ~ 280 km
    CHECK(cam.farM() > 290000.0f);
    CHECK(cam.farM() < 310000.0f);
    cam.setPoseFromAircraft(level(40, -100, 600000, 0));
    CHECK(cam.farM() == doctest::Approx(400000.0f));
}

TEST_CASE("rtc: anchor offsets keep millimeter precision far from the earth center") {
    Camera cam;
    cam.setPoseFromAircraft(level(40, -100, 2000, 0));
    const Ecef eye = cam.eyeEcef();
    // An anchor 1234.5678 m away along ECEF x. Offsets are computed in double, then narrowed.
    const Ecef anchor{eye.x + 1234.5678, eye.y - 0.25, eye.z + 4.0};
    float t[3];
    cam.anchorOffset(anchor, t);
    CHECK(t[0] == doctest::Approx(1234.5678f).epsilon(1e-7));
    CHECK(t[1] == doctest::Approx(-0.25f));
    CHECK(t[2] == doctest::Approx(4.0f));
    // The naive float approach loses this: ECEF coordinates are ~6e6 m, where float spacing is 0.5 m.
    const float naive = static_cast<float>(anchor.y) - static_cast<float>(eye.y);
    CHECK(std::fabs(naive - (-0.25f)) > 0.01f);  // documents why RTC is required
}

TEST_CASE("rtc: model-view places a vertex at rotation * (offset + anchorOffset)") {
    Camera cam;
    cam.setPoseFromAircraft(level(40, -100, 2000, 0));
    // Anchor 500 m north of the eye (NED north expressed in ECEF).
    const Mat3d ned = nedToEcef(40, -100);
    const Vec3d northEcef = ned * Vec3d{1, 0, 0};
    const Ecef eye = cam.eyeEcef();
    const Ecef anchor{eye.x + 500 * northEcef.x, eye.y + 500 * northEcef.y,
                      eye.z + 500 * northEcef.z};
    const Mat4 mv = cam.modelView(anchor);
    // A vertex at the anchor itself lies 500 m straight ahead: eye-space (0, 0, -500).
    const float x = mv.at(0, 3), y = mv.at(1, 3), z = mv.at(2, 3);
    CHECK(x == doctest::Approx(0.0f).epsilon(1e-3));
    CHECK(y == doctest::Approx(0.0f).epsilon(1e-3));
    CHECK(z == doctest::Approx(-500.0f).epsilon(1e-5));
}

TEST_CASE("camera: HUD projection shares the 3D lens") {
    Camera cam;
    cam.setLens(55.0f);
    cam.setViewport(1280, 960);
    CHECK(cam.hudWidth() == doctest::Approx(640.0f));  // 4:3 -> 640x480 virtual pixels

    float x = 0, y = 0;
    const float ahead[3] = {0, 0, -1};
    REQUIRE(cam.projectToHud(ahead, x, y));
    CHECK(x == doctest::Approx(320.0f));
    CHECK(y == doctest::Approx(240.0f));

    // A direction at exactly the top edge of the vertical FOV lands on y = 0.
    const float halfFov = degToRad(27.5);
    const float top[3] = {0, std::sin(static_cast<float>(halfFov)),
                          -std::cos(static_cast<float>(halfFov))};
    REQUIRE(cam.projectToHud(top, x, y));
    CHECK(y == doctest::Approx(0.0f).epsilon(1e-4));

    // Wider window: the virtual width extends.
    cam.setViewport(1920, 960);
    CHECK(cam.hudWidth() == doctest::Approx(960.0f));
    const float behind[3] = {0, 0, 1};
    CHECK_FALSE(cam.projectToHud(behind, x, y));

    // Projection matrix and HUD projection agree: the top-edge direction maps to NDC y = +1.
    const Mat4 p = cam.projection();
    const float clipY = p.at(1, 1) * top[1];
    const float clipW = -top[2];
    CHECK(clipY / clipW == doctest::Approx(1.0f).epsilon(1e-5));
}
