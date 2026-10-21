#pragma once
// S1: procedural audio synthesizer. Pure CPU code with no SDL dependency, so tests and the WAV
// tool can render offline. Allocation-free after construction.
//
// Threading: one producer (the game thread) calls setParams()/post(); one consumer (the audio
// callback, or a test) calls render*(). setParams uses a mutex the consumer only try_locks, so
// the callback never blocks; events travel through a lock-free single-producer ring.
#include <atomic>
#include <cstdint>
#include <mutex>

#include "audio/AudioTypes.h"

namespace fsim::audio {

class AudioSynth {
public:
    AudioSynth();

    void setParams(const AudioParams& p);
    // Jump smoothed values straight to the current params (tests, first frame).
    void snapToParams();
    // Queue a one-shot event. Returns false if the ring is full (event dropped).
    bool post(const AudioEvent& e);

    // Render n mono samples. Output is finite and within [-1, 1].
    void render(float* out, int n);
    void renderS16(std::int16_t* out, int n);

private:
    struct ToneSlot {
        bool active = false;
        AudioEvent ev;
        float tS = 0;
        float phase = 0;
    };
    struct TouchdownVoice {
        bool active = false;
        float tS = 0, inten = 0, chirp = 0, thumpPhase = 0, lp = 0;
    };

    void drainEvents();
    void startEvent(const AudioEvent& e);
    void updateChunk();
    float nextNoise();

    static constexpr int kChunk = 64;
    static constexpr int kRing = 64;
    static constexpr int kToneSlots = 4;
    static constexpr int kTable = 1024;

    // parameters shared with the audio thread
    std::mutex paramMutex_;
    AudioParams shared_;
    bool sharedDirty_ = false;
    AudioParams target_;
    bool snapRequested_ = false;

    // event ring
    AudioEvent ring_[kRing];
    std::atomic<unsigned> ringHead_{0}, ringTail_{0};

    // tables
    float sinTab_[kTable + 1];
    float pistonTab_[kTable + 1];

    // smoothed per-chunk state
    float master_ = 0, simGain_ = 0;
    float engGain_[kMaxEngineVoices] = {}, engFreq_[kMaxEngineVoices] = {},
          engRoar_[kMaxEngineVoices] = {}, engLump_[kMaxEngineVoices] = {},
          engBlade_[kMaxEngineVoices] = {}, engBladeHz_[kMaxEngineVoices] = {};
    float windAmp_ = 0, windCoef_ = 0.02f, gust_ = 1.0f, gustState_ = 0;
    float gearGain_ = 0, flapGain_ = 0, hornGain_ = 0;

    // per-sample oscillator and filter state
    float enginePhase_[kMaxEngineVoices] = {}, lumpPhase_[kMaxEngineVoices] = {},
          bladePhase_[kMaxEngineVoices] = {}, roarLp_[kMaxEngineVoices] = {};
    float windLp_ = 0, gearPhase_ = 0, flapPhase_ = 0, hornPhase_ = 0, gearAm_ = 0, flapAm_ = 0;
    float gearNoiseLp_ = 0;
    ToneSlot tones_[kToneSlots];
    TouchdownVoice td_[2];
    int tdNext_ = 0;
    float clickT_ = 1e9f;
    std::uint32_t rng_ = 0x12345678u;
};

// Minimal 16-bit mono WAV writer (offline renders). Returns false on IO failure.
bool writeWavMono16(const char* path, const std::int16_t* samples, int count, int rateHz);

}  // namespace fsim::audio
