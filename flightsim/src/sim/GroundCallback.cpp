#include "sim/GroundCallback.h"

#include <algorithm>
#include <cmath>

#include <math/FGColumnVector3.h>
#include <math/FGLocation.h>

#include "core/Units.h"

namespace fsim {
namespace {

constexpr double kMinCosLat = 0.01;  // keeps the longitude step finite near the poles

double finiteOr(double v, double fallback) {
    return std::isfinite(v) ? v : fallback;
}

}  // namespace

TerrainGroundCallback::TerrainGroundCallback(const TerrainQuery* terrain, double semiMajorFt,
                                             double semiMinorFt, const GroundCallbackConfig& config)
    : terrain_(terrain), semiMajorFt_(semiMajorFt), semiMinorFt_(semiMinorFt), config_(config) {
    config_.gradientSpacingM = std::max(config_.gradientSpacingM, 1.0f);
    config_.maxSlopeDeg = std::clamp(config_.maxSlopeDeg, 1.0f, 89.0f);
}

void TerrainGroundCallback::invalidate() const {
    for (MemoEntry& e : memo_) e.valid = false;
}

TerrainGroundCallback::Surface TerrainGroundCallback::computeSurface(double latRad,
                                                                     double lonRad) const {
    Surface s;
    const double latDeg = radToDeg(latRad), lonDeg = radToDeg(lonRad);
    const double spacingM = config_.gradientSpacingM;
    // Spherical step sizes are plenty for a 20 m cross.
    const double dLatDeg = radToDeg(spacingM / kEarthRadiusM);
    const double dLonDeg = dLatDeg / std::max(std::cos(latRad), kMinCosLat);
    s.elevM = finiteOr(terrain_->elevationM(latDeg, lonDeg), 0.0);
    const double north = finiteOr(terrain_->elevationM(latDeg + dLatDeg, lonDeg), s.elevM);
    const double south = finiteOr(terrain_->elevationM(latDeg - dLatDeg, lonDeg), s.elevM);
    const double east = finiteOr(terrain_->elevationM(latDeg, lonDeg + dLonDeg), s.elevM);
    const double west = finiteOr(terrain_->elevationM(latDeg, lonDeg - dLonDeg), s.elevM);
    lookups_ += 5;
    s.slopeNorth = (north - south) / (2.0 * spacingM);
    s.slopeEast = (east - west) / (2.0 * spacingM);
    const double slope = std::hypot(s.slopeNorth, s.slopeEast);
    const double maxSlope = std::tan(degToRad(config_.maxSlopeDeg));
    if (slope > maxSlope) {
        s.slopeNorth *= maxSlope / slope;
        s.slopeEast *= maxSlope / slope;
    }
    return s;
}

TerrainGroundCallback::Surface TerrainGroundCallback::surfaceAt(double latRad,
                                                                double lonRad) const {
    return computeSurface(latRad, lonRad);
}

double TerrainGroundCallback::GetAGLevel(double, const JSBSim::FGLocation& location,
                                         JSBSim::FGLocation& contact,
                                         JSBSim::FGColumnVector3& normal,
                                         JSBSim::FGColumnVector3& v,
                                         JSBSim::FGColumnVector3& w) const {
    ++queries_;
    // The ground does not move.
    v.InitMatrix();
    w.InitMatrix();

    JSBSim::FGLocation l = location;
    l.SetEllipse(semiMajorFt_, semiMinorFt_);
    const double latRad = l.GetGeodLatitudeRad();
    const double lonRad = l.GetLongitude();
    const double altFt = l.GetGeodAltitude();

    // Same point already answered during this step (the CG is asked for several times)?
    const Surface* hit = nullptr;
    for (const MemoEntry& e : memo_) {
        if (e.valid && e.latRad == latRad && e.lonRad == lonRad && e.timeS == this->time) {
            hit = &e.surface;
            break;
        }
    }
    Surface surface;
    if (hit) {
        surface = *hit;
        ++memoHits_;
    } else {
        surface = computeSurface(latRad, lonRad);
        MemoEntry& slot = memo_[memoNext_];
        memoNext_ = (memoNext_ + 1) % kMemoSize;
        slot.valid = true;
        slot.latRad = latRad;
        slot.lonRad = lonRad;
        slot.timeS = this->time;
        slot.surface = surface;
    }

    // D16: terrain meters MSL are JSBSim geodetic feet.
    const double elevFt = mToFt(surface.elevM);

    contact.SetEllipse(semiMajorFt_, semiMinorFt_);
    contact.SetPositionGeodetic(lonRad, latRad, elevFt);

    // Upward normal of the surface z = f(north, east): (-df/dn, -df/de, up 1), written in the
    // local north-east-down frame (down = -1) and rotated to ECEF with the location's geodetic
    // frame (the ellipse is set, so Tl2ec is the geodetic one).
    const double invLen =
        1.0 / std::sqrt(1.0 + surface.slopeNorth * surface.slopeNorth +
                        surface.slopeEast * surface.slopeEast);
    const JSBSim::FGColumnVector3 normalLocal(-surface.slopeNorth * invLen,
                                              -surface.slopeEast * invLen, -invLen);
    normal = l.GetTl2ec() * normalLocal;

    return altFt - elevFt;
}

}  // namespace fsim
