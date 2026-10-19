#pragma once
// First-person camera at the pilot eye point, plus the RTC (relative-to-center) helpers every
// renderer uses (ARCHITECTURE section 4, decision D7):
//   * each mesh keeps a double-precision ECEF anchor and float vertex offsets;
//   * per draw, translation = float(anchorEcef - cameraEcef) and
//     modelView = cameraRotation * translate(translation);
//   * absolute ECEF coordinates never go into floats.
// The camera rotation maps ECEF axes to eye axes (x right, y up, -z forward). It is built from
// the NED attitude (JSBSim Euler angles), the lat/lon NED->ECEF rotation and the body->eye swap.

#include "core/AircraftState.h"
#include "core/Geo.h"
#include "gfx/Mat4.h"

namespace fsim::gfx {

constexpr float kHudVirtualHeight = 480.0f;  // virtual HUD pixels; width = 480 * aspect

class Camera {
public:
    // Vertical field of view in degrees (default 55), near/far in meters. far is overridden by
    // setPoseFromAircraft according to the horizon distance.
    void setLens(float vFovDeg, float nearM = 0.5f);
    void setViewport(int widthPx, int heightPx);

    // Eye point = aircraft CG + EYEPOINT offset rotated by the attitude. Look angles are relative
    // to the aircraft (yaw + = right, pitch + = up).
    void setPoseFromAircraft(const AircraftState& s, float lookYawRad = 0, float lookPitchRad = 0);
    // Direct pose (tests): eye position in ECEF and the ECEF->eye rotation.
    void setPose(const Ecef& eyeEcef, const Mat3d& ecefToEye);

    const Ecef& eyeEcef() const { return eyeEcef_; }
    const GeoPos& eyeGeo() const { return eyeGeo_; }
    const Mat3d& ecefToEye() const { return ecefToEye_; }
    float vFovRad() const { return vFovRad_; }
    float aspect() const { return aspect_; }
    float farM() const { return farM_; }
    float nearM() const { return nearM_; }
    Mat4 projection() const;

    // Camera-relative offset of an ECEF anchor, computed in double and narrowed to float.
    void anchorOffset(const Ecef& anchor, float out[3]) const;
    // Model-view for a mesh anchored at `anchor` (vertex offsets are in ECEF axes).
    Mat4 modelView(const Ecef& anchor) const;
    Mat4 viewProjection(const Ecef& anchor) const { return projection() * modelView(anchor); }

    // A direction in ECEF axes expressed in eye axes.
    Vec3d ecefDirToEye(const Vec3d& dirEcef) const { return ecefToEye_ * dirEcef; }

    // HUD conformality: project a direction (eye axes) with the same lens as the 3D scene to HUD
    // virtual pixels (origin top-left, y down; width = 480 * aspect). False if behind the eye.
    bool projectToHud(const float dirEye[3], float& xHud, float& yHud) const;
    float hudWidth() const { return kHudVirtualHeight * aspect_; }

private:
    Ecef eyeEcef_;
    GeoPos eyeGeo_;
    Mat3d ecefToEye_;
    float vFovRad_ = 55.0f * static_cast<float>(kRadPerDeg);
    float aspect_ = 4.0f / 3.0f;
    float nearM_ = 0.5f;
    float farM_ = 40000.0f;
};

}  // namespace fsim::gfx
