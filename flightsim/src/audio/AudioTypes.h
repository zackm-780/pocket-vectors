#pragma once
// S1: data handed from the game thread to the audio synthesizer. Plain structs, no SDL.
#include <cstdint>

namespace fsim::audio {

constexpr int kSampleRateHz = 22050;
constexpr int kMaxEngineVoices = 4;

enum class EngineKind : std::uint8_t { None, Piston, Turboprop, Jet };

// Continuous state, refreshed once per frame. The synthesizer smooths everything it reads.
struct AudioParams {
    bool paused = false;        // true: all sim voices fade out (UI clicks still play)
    float masterGain01 = 1.0f;  // linear gain 0..1 (AudioControl maps Settings::volumePct)
    EngineKind engineKind = EngineKind::None;
    int engineCount = 0;  // 0..kMaxEngineVoices
    int cylinders = 4;    // pistons: firing frequency = rpm / 60 * cylinders / 2
    float engRpm[kMaxEngineVoices] = {};
    float engN1Pct[kMaxEngineVoices] = {};
    float engThrottleNorm[kMaxEngineVoices] = {};
    bool engRunning[kMaxEngineVoices] = {};
    float iasKt = 0;
    bool gearMoving = false, flapsMoving = false;
    bool stallHorn = false;  // same condition as the HUD STALL warning
};

enum class AudioEventType : std::uint8_t { Touchdown, Tone, UiClick };

// One-shot event. Which fields matter depends on `type`.
struct AudioEvent {
    AudioEventType type = AudioEventType::Tone;
    // Touchdown
    float intensity01 = 1.0f;  // thump strength (from sink rate)
    float chirp01 = 1.0f;      // tyre chirp strength (0 when rolling slowly)
    // Tone: `repeats` beeps of `durS` separated by `gapS`; odd beeps use freqAltHz when > 0.
    float freqHz = 1000.0f, freqAltHz = 0.0f;
    float durS = 0.2f, gapS = 0.1f;
    int repeats = 1;
    float gain01 = 0.5f;
};

// Generic alert tone API (also what X2's GPWS callouts will use).
inline AudioEvent makeTone(float freqHz, float durS, int repeats = 1, float gapS = 0.1f,
                           float gain01 = 0.5f, float freqAltHz = 0.0f) {
    AudioEvent e;
    e.type = AudioEventType::Tone;
    e.freqHz = freqHz;
    e.freqAltHz = freqAltHz;
    e.durS = durS;
    e.gapS = gapS;
    e.repeats = repeats;
    e.gain01 = gain01;
    return e;
}

}  // namespace fsim::audio
