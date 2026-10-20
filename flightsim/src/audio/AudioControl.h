#pragma once
// S1: turns sim state into AudioParams plus one-shot events (edge detection lives here, on the
// game thread, so the synthesizer stays a pure function of its inputs).
#include <vector>

#include "audio/AudioTypes.h"
#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/AutopilotState.h"

namespace fsim::audio {

struct AudioFrameInput {
    const AircraftState* state = nullptr;
    const AircraftProfile* profile = nullptr;
    const AutopilotState* ap = nullptr;
    bool stallWarning = false;   // the HUD STALL condition (activeWarnings & WarningId::Stall)
    bool minimumsAlert = false;  // NavHud::consumeMinimumsAlert() result for this frame
    bool paused = false;         // menu open / sim paused
    int volumePct = 80;          // Settings::volumePct, 0..100
    float dtS = 1.0f / 60.0f;
};

class AudioControl {
public:
    // Fills `params` and appends this frame's one-shot events to `events` (not cleared first).
    void update(const AudioFrameInput& in, AudioParams& params, std::vector<AudioEvent>& events);
    void reset();

    // Linear gain for a volume percentage (quadratic: perceived loudness is roughly linear).
    static float volumeGain(int volumePct);

    // Tone parameters of the built-in alerts (exposed for tests).
    static AudioEvent apDisconnectTone();
    static AudioEvent minimumsTone();

    // X2 HOOK: GPWS callouts. When X2's alert/callout queue lands, App.cpp should drain it each
    // frame and push makeTone(...) events (or add dedicated voices here) into the same `events`
    // vector. Nothing in S1 depends on X2.

private:
    bool primed_ = false;
    bool prevWow_ = false, prevApWarn_ = false;
    float prevGear_ = 0, prevFlap_ = 0, prevSimTimeS_ = 0, airSinkMps_ = 0;
    float gearHoldS_ = 0, flapHoldS_ = 0;
};

}  // namespace fsim::audio
