#include "audio/AudioControl.h"

#include <algorithm>
#include <cmath>

namespace fsim::audio {
namespace {
constexpr float kMotorHoldS = 0.3f;  // keeps the motor sound through frame-to-frame jitter
constexpr float kMotorMinDelta = 1e-4f;
}  // namespace

float AudioControl::volumeGain(int volumePct) {
    const float v = static_cast<float>(std::clamp(volumePct, 0, 100)) / 100.0f;
    return v * v;
}

AudioEvent AudioControl::apDisconnectTone() {
    return makeTone(1000.0f, 0.2f, 5, 0.05f, 0.45f, 700.0f);
}
AudioEvent AudioControl::minimumsTone() {
    return makeTone(880.0f, 0.14f, 3, 0.10f, 0.45f);
}

void AudioControl::reset() {
    primed_ = false;
}

void AudioControl::update(const AudioFrameInput& in, AudioParams& params,
                          std::vector<AudioEvent>& events) {
    params = AudioParams();
    params.paused = in.paused;
    params.masterGain01 = volumeGain(in.volumePct);
    if (!in.state || !in.state->valid) {
        primed_ = false;
        return;
    }
    const AircraftState& s = *in.state;
    const AircraftProfile* prof = in.profile;

    // A restart or aircraft change rewinds sim time: re-prime so nothing fires spuriously.
    if (primed_ && s.simTimeS < prevSimTimeS_ - 1e-6) primed_ = false;
    const bool apWarn = in.ap && in.ap->disconnectWarning;
    if (!primed_) {
        primed_ = true;
        prevWow_ = s.wowAny;
        prevApWarn_ = apWarn;
        prevGear_ = s.gearPosNorm;
        prevFlap_ = s.flapPosNorm;
        gearHoldS_ = flapHoldS_ = 0;
        airSinkMps_ = 0;
    }
    prevSimTimeS_ = static_cast<float>(s.simTimeS);

    // ---- engines
    AircraftCategory cat = prof ? prof->category : AircraftCategory::Piston;
    params.engineKind = cat == AircraftCategory::Jet         ? EngineKind::Jet
                        : cat == AircraftCategory::Turboprop ? EngineKind::Turboprop
                                                             : EngineKind::Piston;
    int n = s.engineCount > 0 ? s.engineCount : (prof ? prof->engineCount : 1);
    n = std::clamp(n, 0, kMaxEngineVoices);
    params.engineCount = n;
    params.cylinders = 4;  // no cylinder count in the profile; 4 sounds right for a C172
    for (int i = 0; i < n; ++i) {
        params.engRpm[i] = s.engines[i].rpm;
        params.engN1Pct[i] = s.engines[i].n1Pct;
        params.engThrottleNorm[i] = s.engines[i].throttleCmdNorm;
        params.engRunning[i] = s.engines[i].running;
    }
    params.iasKt = s.iasKt;

    // ---- gear and flap motors: sound while the surface position is changing
    const float dt = std::max(in.dtS, 1e-4f);
    if (std::fabs(s.gearPosNorm - prevGear_) > kMotorMinDelta) gearHoldS_ = kMotorHoldS;
    if (std::fabs(s.flapPosNorm - prevFlap_) > kMotorMinDelta) flapHoldS_ = kMotorHoldS;
    prevGear_ = s.gearPosNorm;
    prevFlap_ = s.flapPosNorm;
    params.gearMoving = gearHoldS_ > 0;
    params.flapsMoving = flapHoldS_ > 0;
    gearHoldS_ = std::max(0.0f, gearHoldS_ - dt);
    flapHoldS_ = std::max(0.0f, flapHoldS_ - dt);

    params.stallHorn = in.stallWarning;

    if (in.paused) {  // frozen sim: no edges, but keep the baselines current
        prevWow_ = s.wowAny;
        prevApWarn_ = apWarn;
        return;
    }

    // ---- touchdown: weight-on-wheels rising edge, strength from the sink rate just before it
    if (s.wowAny && !prevWow_) {
        AudioEvent e;
        e.type = AudioEventType::Touchdown;
        const float sinkMps = std::max(airSinkMps_, 0.0f);
        e.intensity01 = std::clamp(0.2f + sinkMps / 3.0f, 0.2f, 1.0f);
        e.chirp01 = std::clamp((s.gsKt - 30.0f) / 40.0f, 0.0f, 1.0f);
        events.push_back(e);
    }
    if (!s.wowAny) airSinkMps_ = s.velDownMps;
    prevWow_ = s.wowAny;

    // ---- tones
    if (apWarn && !prevApWarn_) events.push_back(apDisconnectTone());
    prevApWarn_ = apWarn;
    if (in.minimumsAlert) events.push_back(minimumsTone());
}

}  // namespace fsim::audio
