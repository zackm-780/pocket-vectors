#pragma once
// Navigator (WP N5, NAVIGATION 2.3, 3.2, 3.4, 4): holds the active FlightPlan and, every update,
// turns AircraftState + plan + selected nav source into NavState (what to fly, deviations,
// distances). It owns leg sequencing:
//   - fly-by turn anticipation d = R tan(dPsi/2), R = V^2 / (g tan bank), bank 25 deg (jet) /
//     20 deg (others), dPsi capped at 120 deg; fly-over legs sequence when abeam;
//   - the MAP leg sequences into the missed approach only when passed airborne (3.4);
//   - holds (HM/HA/HF) never sequence; guidance is direct to the fix, then the inbound course
//     (the racetrack itself is not flown by the navigator in v1);
//   - CA / VA / FA legs sequence when the aircraft climbs through the leg altitude.
// Sources: GPS (linear, CDI scale ENR 2.0 / TERM 1.0 / APR 0.3 NM with a 1 minute linear ramp),
// LOC (angular, from the runway's Ils antenna geometry, with the GS) and VOR (optional).
// Auto selection switches GPS -> LOC on the final course, see NavSourceSelect.
//
// All signs follow CONVENTIONS 3: xtk + = aircraft right of course, CDI + = course is to the
// right, GS deflection + = glidepath above the aircraft. Headings are TRUE internally.
// update() does not allocate in steady state (caches are rebuilt only when the plan changes).

#include <cstdint>
#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/Geo.h"
#include "core/NavDatabase.h"
#include "core/NavState.h"
#include "core/NavTypes.h"
#include "nav/ApproachBuilder.h"

namespace fsim {

enum class NavSourceSelect { Auto, Gps, Loc, Vor };

// Additive (I2), R1 + Y (CONTROLS.md): GPS -> LOC -> VOR -> GPS from the source the HUD shows
// now (NavState::source), skipping sources that are not available. Leaving LOC for GPS forces
// GPS, so the automatic GPS -> LOC switch does not take the needle straight back; GPS reached
// from VOR, or with nothing else available, is Auto again.
NavSourceSelect nextNavSourceSelect(NavSource shown, bool locAvailable, bool vorTuned);

// Limits the navigator applies (exposed for tests and the HUD).
struct NavConstants {
    static constexpr float kCdiClamp = 1.25f;   // display clamp, NAVIGATION 3.2
    static constexpr float kScaleEnrNm = 2.0f;  // CDI full scale, NAVIGATION 4
    static constexpr float kScaleTermNm = 1.0f;
    static constexpr float kScaleAprNm = 0.3f;
    static constexpr float kScaleRampS = 60.0f;        // linear ramp between scales
    static constexpr float kTerminalRadiusNm = 30.0f;  // TERM scaling within this of origin / dest
    static constexpr float kAprBeforeFafNm = 2.0f;     // APR scaling starts this far before FAF
    static constexpr float kLocMaxDistNm = 18.0f;      // LOC validity cone
    static constexpr float kLocMaxOffDeg = 35.0f;
    static constexpr float kGsMaxDistNm = 10.0f;  // GS validity cone
    static constexpr float kGsMaxOffDeg = 8.0f;
    static constexpr float kGsFullScaleDeg = 0.7f;  // 1 dot = 0.35 deg
    static constexpr float kLocCaptureInterceptDeg = 45.0f;
    static constexpr float kVorFullScaleDeg = 10.0f;
    static constexpr float kVnavPathDeg = 3.0f;
    static constexpr float kMissedAglFt = 100.0f;  // MAP passed airborne (see Navigator.cpp)
    static constexpr float kMissedClimbFpm = 300.0f;
};

class Navigator {
public:
    explicit Navigator(const NavDatabase& db);

    // ---- configuration ------------------------------------------------------------------
    void setAircraftCategory(AircraftCategory category);  // jet: 25 deg bank, else 20 deg
    void setBankLimitDeg(float bankDeg) { bankDeg_ = bankDeg; }
    void setRealApproachProvider(RealApproachProvider provider);  // N4 plugs in here
    void setSourceSelect(NavSourceSelect select);
    NavSourceSelect sourceSelect() const { return select_; }
    // Optional VOR source: tune a navaid and select an OBS course (magnetic).
    void tuneVor(const Navaid& navaid, float obsMagDeg);
    void setVorObs(float obsMagDeg) { vorObsMagDeg_ = obsMagDeg; }
    void clearVor() { vorTuned_ = false; }
    // Additive (I2): sources R1 + Y can select. LOC needs an ILS or LOC approach in the plan
    // (valid after the first update following a plan change).
    bool locAvailable() const { return cache_.valid && cache_.ils != nullptr; }
    bool vorTuned() const { return vorTuned_; }

    // ---- plan -----------------------------------------------------------------------------
    // Takes the plan. A plan that names an approach (approachId) but has no approach legs gets
    // the approach loaded through loadApproachById(). activeLeg < 1 becomes 1 (leg 0 is origin).
    void setPlan(FlightPlan plan);
    void clearPlan();
    bool hasPlan() const { return !plan_.legs.empty(); }
    const FlightPlan& plan() const { return plan_; }
    // Mutable access for editors (insert/delete waypoints...). Caches rebuild automatically.
    FlightPlan& editPlan() { return plan_; }

    // ---- approach -------------------------------------------------------------------------
    // Replaces the plan tail with the approach legs. transitionIndex < 0 picks one with
    // selectTransition() from the last enroute waypoint (or the present position).
    bool loadApproach(const Approach& approach, int transitionIndex = -1);
    // Looks the approach up for plan.destIdent / plan.arrRunway (or the runway in the id).
    bool loadApproachById(std::string_view approachId);
    void removeApproach();
    // Start from nothing but an approach: plan = [PPOS] + approach legs, active leg = the first.
    bool startApproachOnly(const Approach& approach, const LatLon& present,
                           int transitionIndex = 0);
    const Approach* activeApproach() const { return approach_ ? &*approach_ : nullptr; }

    // ---- guidance commands -----------------------------------------------------------------
    bool directToLeg(int legIndex);              // direct-to legs[legIndex].to (N3)
    bool directToIdent(std::string_view ident);  // plan waypoint first, else nearest DB match
    bool reenterApproach();                      // direct-to the IF (NAVIGATION 3.4)
    bool activateLeg(int legIndex);              // makes a leg active, no direct-to
    // Picks the leg whose segment is closest ahead of the aircraft (scenario starts in flight).
    void activateNearestLeg(const AircraftState& aircraft);

    // ---- output ----------------------------------------------------------------------------
    const NavState& update(const AircraftState& aircraft, double dtS);
    const NavState& state() const { return state_; }

    // Derived facts about the loaded plan (valid after the first update or a plan change).
    int mapLegIndex() const { return cache_.mapIdx; }
    int firstFinalLegIndex() const { return cache_.firstFinalIdx; }
    int fafLegIndex() const { return cache_.fafIdx; }
    float finalCourseTrueDeg() const { return cache_.finalCourseTrueDeg; }
    const Runway* arrivalRunway() const { return cache_.arrRunway; }

private:
    struct Cache {
        uint64_t signature = 0;
        bool valid = false;
        std::vector<float> legLenNm;    // per leg, from legDistNm
        std::vector<float> afterNm;     // sum of legLen after leg i (missed legs excluded)
        std::vector<float> afterAllNm;  // same, missed legs included
        std::vector<float> toMapNm;     // distance from leg i's fix to the MAP, -1 if none
        int mapIdx = -1, firstMissedIdx = -1, firstFinalIdx = -1, fafIdx = -1;
        float finalCourseTrueDeg = 0;
        bool haveFinalCourse = false;
        LatLon origin, dest;
        bool haveOrigin = false, haveDest = false;
        const Airport* destAirport = nullptr;
        const Runway* arrRunway = nullptr;
        const Ils* ils = nullptr;  // arrival runway's ILS when the approach is ILS / LOC
    };
    struct Guide {  // geometry of the active leg relative to the aircraft
        double dtkRad = 0, xtkNm = 0, distToWptNm = 0, brgToWptRad = 0;
        double remNm = 0;  // along-track distance still to go to the leg end (< 0 = passed)
        double legLenNm = 0;
        bool altTerminated = false;  // CA / VA / FA
        bool isHold = false;
    };
    struct Angular {  // LOC / GS evaluation
        bool locValid = false, gsValid = false;
        float offRightDeg = 0;  // aircraft right of the LOC course = +
        float cdi = 0, gsDefl = 0, xtkNm = 0;
        bool toFlag = true;
        float distToAntennaNm = 0;
    };

    void refreshCache();
    uint64_t planSignature() const;
    bool prevFix(int legIndex, LatLon& out) const;
    Guide computeGuide(const AircraftState& ac, int legIndex) const;
    bool shouldSequence(const AircraftState& ac, const Guide& g, int legIndex) const;
    double courseOutRad(int legIndex) const;
    void captureEntry(const AircraftState& ac);
    float targetScaleNm(const Guide& g, NavPhase phase) const;
    Angular computeLoc(const AircraftState& ac) const;
    void computeVnav(const AircraftState& ac, const Guide& g, NavPhase phase, NavState& s) const;
    void fillIdent(char (&dst)[8], const Leg& leg) const;
    void updateVor(const AircraftState& ac, NavState& s) const;
    void resetLatches();

    const NavDatabase& db_;
    RealApproachProvider realProvider_;
    FlightPlan plan_;
    std::optional<Approach> approach_;
    Cache cache_;
    NavState state_;

    NavSourceSelect select_ = NavSourceSelect::Auto;
    float bankDeg_ = 20.0f;
    LatLon lastPos_;
    bool haveLastPos_ = false;
    LatLon entryPos_;
    bool entryPending_ = true;
    int trackedLeg_ = -1;
    bool locLatched_ = false;

    float scaleNm_ = 0, scaleFromNm_ = 0, scaleToNm_ = 0, rampS_ = 0;
    bool scaleInit_ = false;

    bool vorTuned_ = false;
    std::string vorIdent_;
    LatLon vorPos_;
    float vorMagVarDeg_ = 0, vorObsMagDeg_ = 0;
};

}  // namespace fsim
