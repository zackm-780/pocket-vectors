#include "hud/HudProjector.h"

#include <algorithm>
#include <cmath>

namespace fsim {
namespace {
constexpr float kMinFpvSpeedMps = 2.0f;  // below this the velocity vector is meaningless
}

void HudProjector::nedToEye(double north, double east, double down, float outEye[3]) const {
    const Vec3d ecef = nedToEcef_ * Vec3d{north, east, down};
    const Vec3d eye = camera_.ecefDirToEye(ecef);
    outEye[0] = static_cast<float>(eye.x);
    outEye[1] = static_cast<float>(eye.y);
    outEye[2] = static_cast<float>(eye.z);
}

bool HudProjector::projectNed(double north, double east, double down, float& xHud,
                              float& yHud) const {
    float d[3];
    nedToEye(north, east, down, d);
    return camera_.projectToHud(d, xHud, yHud);
}

bool HudProjector::projectAzEl(double azTrueRad, double elRad, float& xHud, float& yHud) const {
    const double c = std::cos(elRad);
    return projectNed(c * std::cos(azTrueRad), c * std::sin(azTrueRad), -std::sin(elRad), xHud,
                      yHud);
}

HudFpv computeFpv(const HudProjector& proj, const AircraftState& s, float rx, float ry, float rw,
                  float rh) {
    HudFpv f;
    const double n = s.velNorthMps, e = s.velEastMps, d = s.velDownMps;
    if (std::sqrt(n * n + e * e + d * d) < kMinFpvSpeedMps) return f;
    f.valid = true;
    float x = 0, y = 0;
    if (!proj.projectNed(n, e, d, x, y)) {
        // Behind the eye (looking away): park it on the edge in the direction of the eye-plane
        // components, so the pilot still sees which way it is.
        float dir[3];
        proj.nedToEye(n, e, d, dir);
        const float len = std::max(1e-6f, std::hypot(dir[0], dir[1]));
        x = rx + rw * 0.5f + dir[0] / len * rw;
        y = ry + rh * 0.5f - dir[1] / len * rh;
    }
    const float cx = std::min(std::max(x, rx), rx + rw);
    const float cy = std::min(std::max(y, ry), ry + rh);
    f.clamped = cx != x || cy != y;
    f.xHud = cx;
    f.yHud = cy;
    return f;
}

}  // namespace fsim
