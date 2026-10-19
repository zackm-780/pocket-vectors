#pragma once
// Geodesy helpers: WGS84 geodetic <-> ECEF, great-circle math on the mean-radius sphere, NED/ECEF
// rotations and angle wrapping. Headings and bearings are TRUE and in radians here.

#include "core/Units.h"

namespace fsim {

struct GeoPos {
    double latDeg = 0, lonDeg = 0;
    double altMslM = 0;
};
struct LatLon {
    double latDeg = 0, lonDeg = 0;
};
struct Ecef {
    double x = 0, y = 0, z = 0;
};
struct Vec3d {
    double x = 0, y = 0, z = 0;
};
// Row-major 3x3 rotation matrix: out = m * in.
struct Mat3d {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
};

Ecef geodeticToEcef(const GeoPos& pos);
GeoPos ecefToGeodetic(const Ecef& ecef);

// Great-circle (haversine) distance, meters, on the sphere of radius kEarthRadiusM.
double greatCircleDistM(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
// Initial true bearing from point 1 to point 2, radians in [0, 2pi).
double initialBearingTrueRad(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
// Point reached by travelling distM along a great circle with the given initial true bearing.
void destinationPoint(double latDeg, double lonDeg, double bearingTrueRad, double distM,
                      double& outLatDeg, double& outLonDeg);
// Signed cross-track distance of a point from the great-circle leg start->end: + = right of track.
double crossTrackM(const LatLon& point, const LatLon& legStart, const LatLon& legEnd);
// Along-track distance of the point's projection from legStart, + = toward legEnd.
double alongTrackM(const LatLon& point, const LatLon& legStart, const LatLon& legEnd);

// Rotation taking NED-frame vectors to ECEF (columns are the N, E, D unit vectors in ECEF).
Mat3d nedToEcef(double latDeg, double lonDeg);
// Rotation taking body-frame vectors (x fwd, y right, z down) to NED for JSBSim-style Euler angles
// (yaw-pitch-roll: heading, then pitch, then roll).
Mat3d bodyToNed(double rollRad, double pitchRad, double headingTrueRad);

Mat3d operator*(const Mat3d& a, const Mat3d& b);
Vec3d operator*(const Mat3d& a, const Vec3d& v);
Mat3d transpose(const Mat3d& a);

// Wrap an angle in radians to (-pi, pi].
double wrapPi(double rad);
// Wrap an angle in radians to [0, 2pi).
double wrap2Pi(double rad);
// Wrap an angle in degrees to [0, 360).
double wrap360(double deg);
// Wrap an angle in degrees to (-180, 180].
double wrap180(double deg);

}  // namespace fsim
