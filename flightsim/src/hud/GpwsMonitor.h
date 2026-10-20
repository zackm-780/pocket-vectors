#pragma once
// X2: GPWS-lite. Pure logic (no GL, no audio): ground proximity warnings and radio altitude
// callouts from the aircraft state, a terrain query, the nav state and the profile. The HUD shows
// the warnings through HudOptions::extraWarnings (warningBit PullUp / Terrain / SinkRate); events
// are queued for the future audio WP (S1) and posted as short text messages by the app.
//
// Modes (all simplified from Honeywell Mode 1, 2, 4):
//  - Mode 1, sink rate vs radio altitude (envelopes in sinkCautionFpm / sinkWarningFpm).
//  - Mode 2-lite, terrain look-ahead: the projected path (ground track, smoothed vertical speed)
//    is sampled kLookaheadStepS apart out to kLookaheadCautionS and compared with the terrain plus
//    a clearance floor that grows with ground speed (lookaheadCautionFloorFt / WarningFloorFt).
//    TERRAIN when the floor is penetrated within kLookaheadCautionS, PULL UP within
//    kLookaheadWarningS.
//  - Mode 4-lite: TOO LOW GEAR / TOO LOW FLAPS messages (the GEAR bit stays with H2).
//  - Radio altitude callouts on descent: 2500 1000 500 100 50 40 30 20 10, once per approach.
// H3's MINIMUMS callout is not duplicated here.

#include <cstdint>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/NavDatabase.h"
#include "core/NavState.h"
#include "core/TerrainQuery.h"

namespace fsim {

enum class GpwsEventKind { SinkRate, PullUp, Terrain, TooLowGear, TooLowFlaps, Callout };

struct GpwsEvent {
    GpwsEventKind kind = GpwsEventKind::Callout;
    int valueFt = 0;     // Callout: the radio altitude called, ft
    char text[24] = {};  // "SINK RATE", "PULL UP", "TERRAIN", "TOO LOW GEAR", "2500", ...
};

constexpr float kGpwsMinRaFt = 30.0f;         // no warnings below this (landing roll, flare)
constexpr float kGpwsSinkMaxRaFt = 2450.0f;   // Mode 1 applies below this radio altitude
constexpr float kLookaheadStepS = 2.5f;       // sample spacing along the projected path
constexpr float kLookaheadCautionS = 40.0f;   // TERRAIN horizon
constexpr float kLookaheadWarningS = 20.0f;   // PULL UP horizon
constexpr int kLookaheadMaxSamples = 16;      // = kLookaheadCautionS / kLookaheadStepS
constexpr float kLookaheadPeriodS = 0.2f;     // look-ahead evaluated at 5 Hz
constexpr float kGpwsHoldOffS = 3.0f;         // a cleared warning stays on this long (flicker)
constexpr float kStableGsDeflection = 0.65f;  // 1.3 dots (full scale 1.0 = 2 dots)
constexpr float kAirportInhibitNm = 1.0f;     // no terrain look-ahead this close to an airport
constexpr float kAirportFullFloorNm = 5.0f;   // clearance floor is full at this distance

// Envelope helpers (exposed for tests and tuning). Sink rate in ft/min (positive = descending).
float sinkCautionFpm(float raFt);
float sinkWarningFpm(float raFt);
float lookaheadCautionFloorFt(float gsKt);
float lookaheadWarningFloorFt(float gsKt);

class GpwsMonitor {
public:
    void reset();
    // Call once per frame or sim step. nav and navDb may be null. The airport proximity probe
    // (navDb->nearestAirports) runs at most once per second.
    void update(const AircraftState& s, const AircraftProfile& profile, const NavState* nav,
                const TerrainQuery& terrain, const NavDatabase* navDb = nullptr);

    // OR into HudOptions::extraWarnings. Only the highest of PullUp > Terrain > SinkRate is set.
    std::uint32_t warnings() const { return mask_; }
    // Pops the oldest queued event (alerts when a level starts, callouts, too-low messages).
    // False when empty. Hook for the audio WP (S1) and the app's message posting.
    bool consumeEvent(GpwsEvent& out);

    // True while the look-ahead is suppressed by a stabilized approach / airport proximity.
    bool terrainSuppressed() const { return terrainSuppressed_; }

private:
    struct Latch {  // persistence before raising, hold-off after the condition clears
        float onS = 0, offS = 0;
        bool active = false;
        bool step(bool cond, float dtS, float persistS, float holdS);
        void clear() { onS = offS = 0, active = false; }
    };
    void push(GpwsEventKind kind, const char* text, int valueFt = 0);
    void lookahead(const AircraftState& s, const TerrainQuery& terrain, float floorScale,
                   bool& caution, bool& warning) const;
    void callouts(const AircraftState& s);
    void probeAirport(const AircraftState& s, const NavDatabase* navDb);

    double clockS_ = 0, lastSimS_ = -1, lastLookS_ = -1e9, lastProbeS_ = -1e9;
    float vsSmoothFpm_ = 0, airportNm_ = -1;
    bool haveVs_ = false, terrainSuppressed_ = false;
    bool lookCaution_ = false, lookWarning_ = false;
    Latch sinkCaution_, sinkWarning_, terrCaution_, terrWarning_;
    float prevRaFt_ = -1;
    bool calloutFired_[9] = {};
    bool gearMsgFired_ = false, flapsMsgFired_ = false;
    int prevLevel_ = 0;  // 0 none, 1 SinkRate, 2 Terrain, 3 PullUp (for event escalation)
    std::uint32_t mask_ = 0;
    GpwsEvent queue_[16];
    int qHead_ = 0, qSize_ = 0;
};

}  // namespace fsim
