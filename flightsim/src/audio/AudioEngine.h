#pragma once
// S1: SDL2 audio device wrapper around AudioSynth. A missing audio device is never an error:
// init() logs and returns false, and every other call becomes a no-op (null sink).
#include <memory>
#include <vector>

#include "audio/AudioSynth.h"

namespace fsim::audio {

class AudioEngine {
public:
    AudioEngine();
    ~AudioEngine();
    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    // Opens the default device (22050 Hz, mono, S16). `bufferSamples` is the buffer asked for
    // (audioBufferSamples(); 512 is about 23 ms). The driver may pick another size; the log line
    // says what it gave, and bufferSamplesObtained() returns it. Returns true when a device is
    // playing. SDL_AUDIODRIVER=dummy works.
    bool init(int bufferSamples = kDefaultBufferSamples);
    void shutdown();
    bool deviceOpen() const { return device_ != 0; }

    // Once per frame from the game thread.
    void update(const AudioParams& params, const std::vector<AudioEvent>& events);
    void postEvent(const AudioEvent& e);  // e.g. UiClick, GPWS tones (X2 hook)
    // S2: the clips Voice events play (nullptr = none). Keep the bank alive while it is set.
    void setVoiceBank(const VoiceBank* bank);

    // Underruns cannot be observed through the callback API; this counts callbacks, so a device
    // test can confirm the stream is alive.
    unsigned long callbackCount() const;
    int bufferSamplesObtained() const { return bufferObtained_; }  // 0 while no device is open

    static constexpr int kDefaultBufferSamples = 512;
    static constexpr int kPortMasterBufferSamples = 1024;  // S2 part A: 512 underran on the device

    struct Impl;  // public only so the SDL callback in the .cpp can name it

private:
    std::unique_ptr<Impl> impl_;
    unsigned device_ = 0;
    bool first_ = true;
    int bufferObtained_ = 0;
    const VoiceBank* bank_ = nullptr;
};

// Buffer size to ask for. `settingSamples` is settings.ini audio_buffer_samples: 0 = automatic
// (1024 under PortMaster, where 512 underran on the RG40XXH, else 512); any other value is rounded
// to a power of two in 256..4096.
int audioBufferSamples(int settingSamples, bool portMaster);

}  // namespace fsim::audio
