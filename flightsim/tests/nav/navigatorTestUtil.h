#pragma once
// Helpers for the navigator tests: aircraft states at geometric positions and a kinematic
// point-mass "perfect pilot" that flies NavState guidance (no JSBSim).
#include <algorithm>
#include <cmath>
#include <string>

#include "core/AircraftState.h"
#include "core/Geo.h"
#include "core/NavState.h"
#include "core/Units.h"

namespace fsim::test {

inline AircraftState makeAircraft(double latDeg, double lonDeg, double altMslFt,
                                  double headingTrueDeg, double gsKt = 100.0,
                                  double terrainElevFt = 0.0) {
    AircraftState a;
    a.valid = true;
    a.latDeg = latDeg;
    a.lonDeg = lonDeg;
    a.altMslFt = altMslFt;
    a.terrainElevFt = static_cast<float>(terrainElevFt);
    a.altAglFt = static_cast<float>(altMslFt - terrainElevFt);
    a.headingTrueRad = static_cast<float>(degToRad(headingTrueDeg));
    a.trackTrueRad = a.headingTrueRad;
    a.gsKt = static_cast<float>(gsKt);
    a.iasKt = a.gsKt;
    a.wowAny = false;
    return a;
}

inline LatLon pointAtNm(double latDeg, double lonDeg, double bearingTrueDeg, double distNm) {
    LatLon p;
    destinationPoint(latDeg, lonDeg, degToRad(bearingTrueDeg), nmToM(distNm), p.latDeg, p.lonDeg);
    return p;
}

// Point `alongNm` from `origin` toward bearingDeg, then `rightNm` to the right of that bearing.
inline LatLon offsetPoint(const LatLon& origin, double bearingDeg, double alongNm, double rightNm) {
    const LatLon p = pointAtNm(origin.latDeg, origin.lonDeg, bearingDeg, alongNm);
    return pointAtNm(p.latDeg, p.lonDeg, bearingDeg + 90.0, rightNm);
}

// Kinematic follower: constant ground speed, bank-limited heading changes, vertical speed from
// NavState (glideslope when valid, else hold the next constraint altitude).
struct Follower {
    AircraftState ac;
    double gsKt = 100.0;
    double bankDeg = 20.0;
    double terrainElevFt = 0;
    double vsLimitFpm = 1000;

    void start(double latDeg, double lonDeg, double altMslFt, double headingTrueDeg, double gs,
               double terrainFt) {
        gsKt = gs;
        terrainElevFt = terrainFt;
        ac = makeAircraft(latDeg, lonDeg, altMslFt, headingTrueDeg, gs, terrainFt);
    }

    double headingDeg() const { return radToDeg(ac.headingTrueRad); }

    void step(const NavState& nav, double dtS) {
        // Lateral: intercept with up to 30 degrees, LOC with a gentler gain.
        double cmdDeg = headingDeg();
        if (nav.source == NavSource::Loc) {
            const double corr = std::clamp(12.0 * static_cast<double>(nav.cdiDeflection), -25.0, 25.0);
            cmdDeg = static_cast<double>(nav.desiredTrackTrueDeg) + corr;
        } else if (nav.source == NavSource::Gps) {
            const double corr = std::clamp(40.0 * static_cast<double>(nav.crossTrackNm), -30.0, 30.0);
            cmdDeg = static_cast<double>(nav.desiredTrackTrueDeg) - corr;
        }
        const double omegaMax =
            radToDeg(kStdGravityMps2 * std::tan(degToRad(bankDeg)) / ktToMps(gsKt));
        const double err = wrap180(cmdDeg - headingDeg());
        const double rate = std::clamp(0.8 * err, -omegaMax, omegaMax);
        const double newHdg = headingDeg() + rate * dtS;

        // Vertical.
        double vsFpm = 0;
        const double pathVs = -gsKt * (kFtPerNm / 60.0) * std::tan(degToRad(3.0));
        if (nav.source == NavSource::Loc && nav.gsValid && std::fabs(nav.gsDeflection) < 1.0f) {
            vsFpm = pathVs + 600.0 * static_cast<double>(nav.gsDeflection);
        } else if (nav.vnavValid) {
            // Advisory VNAV: before the TOD (path above the aircraft) hold, after it follow the path.
            if (nav.vnavDevFt <= 100.0f)
                vsFpm = pathVs + std::clamp(static_cast<double>(nav.vnavDevFt), -300.0, 300.0);
        } else if (nav.targetAltFt > 0) {
            vsFpm = std::clamp(0.5 * (static_cast<double>(nav.targetAltFt) - ac.altMslFt), -vsLimitFpm,
                               vsLimitFpm);
        }
        vsFpm = std::clamp(vsFpm, -1500.0, 1500.0);

        // Integrate.
        ac.headingTrueRad = static_cast<float>(degToRad(wrap360(newHdg)));
        ac.trackTrueRad = ac.headingTrueRad;
        double lat, lon;
        destinationPoint(ac.latDeg, ac.lonDeg, ac.trackTrueRad, ktToMps(gsKt) * dtS, lat, lon);
        ac.latDeg = lat;
        ac.lonDeg = lon;
        ac.altMslFt += vsFpm * dtS / 60.0;
        ac.vsFpm = static_cast<float>(vsFpm);
        ac.altAglFt = static_cast<float>(ac.altMslFt - terrainElevFt);
        ac.wowAny = ac.altAglFt <= 0.0f;
        ac.simTimeS += dtS;
    }
};

}  // namespace fsim::test
