#pragma once
// Terrain ground contact for JSBSim (WP F3): a JSBSim::FGGroundCallback answered from the shared
// TerrainQuery contract (ARCHITECTURE sections 5 and 7).
//
// What JSBSim asks and what we answer (v1.3.3, FGInertial::GetContactPoint):
//   - AGL: the vertical (geodetic) height of the queried point above the ground directly below it,
//     in feet. Negative = the point is underground. FGLGear turns this into strut compression and
//     weight on wheels; FGPropagate publishes it as position/h-agl-ft.
//   - contact point: the ground point below the query, in ECEF (geodetic altitude = surface
//     elevation). position/terrain-elevation-asl-ft reads its altitude.
//   - surface normal in ECEF, pointing up and away from the ground. It tilts the gear force and the
//     friction plane on a slope. It comes from the terrain gradient (central differences).
//   - surface velocity and angular velocity: zero (the ground does not move).
//
// Datum (PLAN D16): the terrain is in meters MSL and JSBSim's geodetic altitude in feet is treated
// as MSL, so ground height in feet = elevationM x kFtPerM with no geoid correction.
//
// Flatten areas (runway pads) are already inside TerrainQuery::elevationM, so the callback needs
// no knowledge of them: a flattened runway gives an exactly flat plane (zero gradient) and a normal
// that points straight up.
//
// Threading and cost: TerrainService queries are main-thread only and JSBSim runs on the main
// thread, so the callback is called from the same thread as TerrainQuery. JSBSim calls it about
// 10 times per step at 120 Hz (every contact point, the CG for AGL and terrain-elevation, and the
// terrain-velocity refresh). Each distinct location costs five elevation lookups (the centre and a
// +-spacing cross for the gradient); a four-entry memo keyed on the exact location and the
// simulation time answers the repeated CG queries within one step for free. No allocation.
//
// Ownership: JSBSim's FGInertial owns the callback once installed (SetGroundCallback). The
// TerrainQuery is not owned and must outlive the callback.

#include <cstdint>

#include <JSBSim_API.h>
#include <input_output/FGGroundCallback.h>

#include "core/TerrainQuery.h"

namespace fsim {

struct GroundCallbackConfig {
    // Half-width of the central-difference cross used for the surface gradient. Terrain packs are
    // 1 m quantised with 30 to 40 m sample spacing at the finest level, so a spacing near one cell
    // keeps the normal from flickering on gentle slopes while still following real relief.
    float gradientSpacingM = 20.0f;
    // Slopes are limited to this angle when building the normal (a cliff or a tile seam must not
    // give a horizontal normal that blows up the contact solver).
    float maxSlopeDeg = 70.0f;
};

class TerrainGroundCallback : public JSBSim::FGGroundCallback {
public:
    // semiMajorFt / semiMinorFt: the planet (FGInertial::GetSemimajor / GetSemiminor).
    TerrainGroundCallback(const TerrainQuery* terrain, double semiMajorFt, double semiMinorFt,
                          const GroundCallbackConfig& config = {});

    double GetAGLevel(double t, const JSBSim::FGLocation& location, JSBSim::FGLocation& contact,
                      JSBSim::FGColumnVector3& normal, JSBSim::FGColumnVector3& v,
                      JSBSim::FGColumnVector3& w) const override;

    // JSBSim writes the initial-condition terrain elevation here. The terrain is the truth, so the
    // value is ignored.
    void SetTerrainElevation(double) override {}
    void SetEllipse(double semimajor, double semiminor) override {
        semiMajorFt_ = semimajor;
        semiMinorFt_ = semiminor;
    }

    // Swap the terrain (nullptr is not allowed: install JSBSim's default callback instead).
    void setTerrain(const TerrainQuery* terrain) {
        terrain_ = terrain;
        invalidate();
    }
    // Drop the memo (call when the terrain or its flatten areas changed without time advancing).
    void invalidate() const;

    const GroundCallbackConfig& config() const { return config_; }

    // Surface elevation (meters MSL) and slope (rise over run, + = rising) at a point, exactly as
    // the callback sees it. Public for tests and tools.
    struct Surface {
        double elevM = 0;
        double slopeNorth = 0;  // d(elev)/d(north), dimensionless
        double slopeEast = 0;   // d(elev)/d(east), dimensionless
    };
    Surface surfaceAt(double latRad, double lonRad) const;

    // Diagnostics: GetAGLevel calls, terrain elevation lookups, memo hits (since construction).
    uint64_t queryCount() const { return queries_; }
    uint64_t elevationLookupCount() const { return lookups_; }
    uint64_t memoHitCount() const { return memoHits_; }

private:
    struct MemoEntry {
        bool valid = false;
        double latRad = 0, lonRad = 0, timeS = 0;
        Surface surface;
    };
    static constexpr int kMemoSize = 4;

    Surface computeSurface(double latRad, double lonRad) const;

    const TerrainQuery* terrain_;
    double semiMajorFt_, semiMinorFt_;
    GroundCallbackConfig config_;
    mutable MemoEntry memo_[kMemoSize];
    mutable int memoNext_ = 0;
    mutable uint64_t queries_ = 0, lookups_ = 0, memoHits_ = 0;
};

}  // namespace fsim
