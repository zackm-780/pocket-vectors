#include "gfx/Camera.h"

#include <algorithm>
#include <cmath>

namespace fsim::gfx {
namespace {

// Body axes (x forward, y right, z down) -> eye axes (x right, y up, -z forward).
Mat3d bodyToEyeAxes() {
    Mat3d r;
    r.m[0][0] = 0;
    r.m[0][1] = 1;
    r.m[0][2] = 0;  // eye x = body y
    r.m[1][0] = 0;
    r.m[1][1] = 0;
    r.m[1][2] = -1;  // eye y = -body z
    r.m[2][0] = -1;
    r.m[2][1] = 0;
    r.m[2][2] = 0;  // eye z = -body x
    return r;
}

// Rotation of look-direction axes: yaw about body z (down), pitch about body y.
Mat3d lookRotation(float yawRad, float pitchRad) {
    const double cy = std::cos(yawRad), sy = std::sin(yawRad);
    const double cp = std::cos(pitchRad), sp = std::sin(pitchRad);  // +angle about y = nose up
    Mat3d rz, ry;
    rz.m[0][0] = cy;
    rz.m[0][1] = -sy;
    rz.m[1][0] = sy;
    rz.m[1][1] = cy;
    ry.m[0][0] = cp;
    ry.m[0][2] = sp;
    ry.m[2][0] = -sp;
    ry.m[2][2] = cp;
    return rz * ry;
}

}  // namespace

void Camera::setLens(float vFovDeg, float nearM) {
    vFovRad_ = vFovDeg * static_cast<float>(kRadPerDeg);
    nearM_ = nearM;
}

void Camera::setViewport(int widthPx, int heightPx) {
    aspect_ = heightPx > 0 ? static_cast<float>(widthPx) / static_cast<float>(heightPx) : 1.0f;
}

void Camera::setPose(const Ecef& eyeEcef, const Mat3d& ecefToEye) {
    eyeEcef_ = eyeEcef;
    eyeGeo_ = ecefToGeodetic(eyeEcef);
    ecefToEye_ = ecefToEye;
}

void Camera::setPoseFromAircraft(const AircraftState& s, float lookYawRad, float lookPitchRad) {
    const Mat3d bodyToEcef =
        nedToEcef(s.latDeg, s.lonDeg) * bodyToNed(s.rollRad, s.pitchRad, s.headingTrueRad);

    // Eye point: CG position plus the (body-axes) eye offset rotated into ECEF.
    GeoPos cg;
    cg.latDeg = s.latDeg;
    cg.lonDeg = s.lonDeg;
    cg.altMslM = ftToM(s.altMslFt);
    const Ecef cgEcef = geodeticToEcef(cg);
    const Vec3d eyeOffsetEcef = bodyToEcef * Vec3d{s.eyeBodyM[0], s.eyeBodyM[1], s.eyeBodyM[2]};
    eyeEcef_ = {cgEcef.x + eyeOffsetEcef.x, cgEcef.y + eyeOffsetEcef.y, cgEcef.z + eyeOffsetEcef.z};
    eyeGeo_ = ecefToGeodetic(eyeEcef_);

    const Mat3d lookedBodyToEcef = bodyToEcef * lookRotation(lookYawRad, lookPitchRad);
    ecefToEye_ = bodyToEyeAxes() * transpose(lookedBodyToEcef);

    // Far plane: max(40 km, horizon distance + 20 km), capped at 400 km (ARCHITECTURE section 6).
    const double hM = std::max(0.0, static_cast<double>(ftToM(s.altAglFt)));
    const double horizonM = std::sqrt(2.0 * kEarthRadiusM * hM + hM * hM);
    farM_ = static_cast<float>(std::min(400000.0, std::max(40000.0, horizonM + 20000.0)));
}

Mat4 Camera::projection() const {
    return Mat4::perspective(vFovRad_, aspect_, nearM_, farM_);
}

void Camera::anchorOffset(const Ecef& anchor, float out[3]) const {
    out[0] = static_cast<float>(anchor.x - eyeEcef_.x);
    out[1] = static_cast<float>(anchor.y - eyeEcef_.y);
    out[2] = static_cast<float>(anchor.z - eyeEcef_.z);
}

Mat4 Camera::modelView(const Ecef& anchor) const {
    float t[3];
    anchorOffset(anchor, t);
    return Mat4::rotationOfTranslated(ecefToEye_, t);
}

bool Camera::projectToHud(const float dirEye[3], float& xHud, float& yHud) const {
    if (dirEye[2] >= -1e-6f) return false;
    const float focalPx = (kHudVirtualHeight * 0.5f) / std::tan(vFovRad_ * 0.5f);
    const float invDepth = 1.0f / -dirEye[2];
    xHud = hudWidth() * 0.5f + focalPx * dirEye[0] * invDepth;
    yHud = kHudVirtualHeight * 0.5f - focalPx * dirEye[1] * invDepth;
    return true;
}

}  // namespace fsim::gfx
