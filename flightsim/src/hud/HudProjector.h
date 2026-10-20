#pragma once
// Conformal projection for the flight HUD (HUD.md section 1, ARCHITECTURE section 6): world
// directions given in the local NED frame (or as true azimuth + elevation) go through the SAME
// camera rotation and lens as the 3D scene (gfx::Camera::projectToHud), so the horizon, pitch
// ladder and flight path vector line up with the rendered world, including roll, eye offset
// direction and look angles.

#include "core/AircraftState.h"
#include "core/Geo.h"
#include "gfx/Camera.h"

namespace fsim {

class HudProjector {
public:
    // The camera pose must already be set for this frame (setPoseFromAircraft).
    HudProjector(const gfx::Camera& camera, double latDeg, double lonDeg)
        : camera_(camera), nedToEcef_(nedToEcef(latDeg, lonDeg)) {}

    // Direction in NED axes (north, east, down) -> eye axes (x right, y up, -z forward).
    void nedToEye(double north, double east, double down, float outEye[3]) const;
    // False when the direction is behind the eye (outputs unchanged).
    bool projectNed(double north, double east, double down, float& xHud, float& yHud) const;
    // Direction by true azimuth (clockwise from north) and elevation above the horizon.
    bool projectAzEl(double azTrueRad, double elRad, float& xHud, float& yHud) const;

private:
    const gfx::Camera& camera_;
    Mat3d nedToEcef_;
};

struct HudFpv {
    bool valid = false;    // velocity large enough to define a direction
    bool clamped = false;  // pushed to the clip edge (draw dimmed)
    float xHud = 0, yHud = 0;
};
// Flight path vector from the inertial velocity (velNorth/East/DownMps), clamped into the rect.
HudFpv computeFpv(const HudProjector& proj, const AircraftState& s, float rectX, float rectY,
                  float rectW, float rectH);

}  // namespace fsim
