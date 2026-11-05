#pragma once
// V1: the external (chase) view without GL. Builds the aircraft model in code from a handful of
// primitives per category (fuselage loft, wing and tail slabs, nacelles, props, gear, flaps), so
// no model files or licences are involved, and computes the chase camera pose. ExternalView.h
// draws the mesh and moves the camera.
//
// Model space is body axes, meters, origin at the CG: x forward, y right, z down. Vertices are
// 9 floats: position (3), outward normal (3), color (3); triangles wind counter-clockwise seen
// from outside (back faces are culled). One mesh = one draw call.

#include <vector>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/Geo.h"

namespace fsim::world {

enum class ExternalModelKind { Single, Twin, Bizjet, Airliner, Heavy };

// Profile [view] external_model (single, twin, bizjet, airliner, heavy) when set; otherwise
// piston single -> Single, piston twin and turboprop -> Twin, jet with 3+ engines -> Heavy,
// other jets -> Airliner.
ExternalModelKind externalModelFor(const AircraftProfile& profile);
const char* externalModelName(ExternalModelKind kind);
bool parseExternalModel(const char* name, ExternalModelKind& out);

struct ExternalModelParams {
    ExternalModelKind kind = ExternalModelKind::Single;
    int engineCount = 1;
    float flapNorm = 0;  // 0 = up .. 1 = full (drawn as a deflected trailing-edge panel)
    float gearNorm = 1;  // 0 = up .. 1 = down; fixed gear passes 1
    // Height of the aircraft's reference point above the ground on its wheels (gear down); the
    // model moves so its wheels touch there. 0 = the template's own CG height.
    float gearHeightM = 0;
    float spanM = 0;  // scale the template to this wingspan; 0 = template span
};

// Size facts of a built model (meters), used for the camera distance and tests.
struct ExternalModelInfo {
    float spanM = 0, lengthM = 0;
    float minXM = 0, maxXM = 0, maxZM = 0;  // fore/aft extent; lowest point (z down, wheels)
    int triangles = 0;
};

constexpr int kExternalVertexFloats = 9;

// Appends the triangles of the model to `out` (cleared first).
ExternalModelInfo buildExternalModel(const ExternalModelParams& p, std::vector<float>& out);

// Chase camera: behind and above the CG at distM, looking at the CG. followHdgDeg is the
// (smoothed) heading it trails; orbit yaw / pitch turn the view around the aircraft (yaw + =
// look to the right of the heading, pitch + = look up). The horizon stays level.
struct ChasePose {
    Ecef eyeEcef;
    GeoPos eyeGeo;
    Mat3d ecefToEye;  // rows: right, up, -forward (gfx::Camera convention)
};
constexpr float kChaseBaseElevDeg = 10.0f;  // the camera looks down this much by default
ChasePose chasePose(const AircraftState& s, float followHdgDeg, float orbitYawDeg,
                    float orbitPitchDeg, float distM);
// Camera rotation looking from `eye` at `target` with a level horizon.
Mat3d lookAtEcef(const Ecef& eye, const Ecef& target);
// Distance behind the aircraft for a model of this span.
float chaseDistanceM(float spanM);

// First-order follow of the heading (degrees, wraps through north): time constant tauS.
float followHeadingDeg(float currentDeg, float targetDeg, float tauS, double dtS);

}  // namespace fsim::world
