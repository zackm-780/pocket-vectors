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

    // Opens the default device (22050 Hz, mono, S16, 512-sample buffer, about 23 ms).
    // Returns true when a device is playing. SDL_AUDIODRIVER=dummy works.
    bool init();
    void shutdown();
    bool deviceOpen() const { return device_ != 0; }

    // Once per frame from the game thread.
    void update(const AudioParams& params, const std::vector<AudioEvent>& events);
    void postEvent(const AudioEvent& e);  // e.g. UiClick, GPWS tones (X2 hook)

    // Underruns cannot be observed through the callback API; this counts callbacks, so a device
    // test can confirm the stream is alive.
    unsigned long callbackCount() const;

    struct Impl;  // public only so the SDL callback in the .cpp can name it

private:
    std::unique_ptr<Impl> impl_;
    unsigned device_ = 0;
    bool first_ = true;
};

}  // namespace fsim::audio
