#include "audio/AudioEngine.h"

#include <SDL.h>

#include <atomic>

#include "core/Log.h"

namespace fsim::audio {

struct AudioEngine::Impl {
    AudioSynth synth;
    std::atomic<unsigned long> callbacks{0};
};

namespace {
void sdlCallback(void* user, Uint8* stream, int len) {
    auto* impl = static_cast<AudioEngine::Impl*>(user);
    impl->synth.renderS16(reinterpret_cast<std::int16_t*>(stream),
                          len / static_cast<int>(sizeof(std::int16_t)));
    impl->callbacks.fetch_add(1, std::memory_order_relaxed);
}
}  // namespace

AudioEngine::AudioEngine() = default;
AudioEngine::~AudioEngine() {
    shutdown();
}

bool AudioEngine::init() {
    if (device_ != 0) return true;
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        LOG_WARN("audio: SDL audio init failed (%s); running silent", SDL_GetError());
        return false;
    }
    impl_ = std::make_unique<Impl>();
    SDL_AudioSpec want;
    SDL_zero(want);
    want.freq = kSampleRateHz;
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = 512;
    want.callback = sdlCallback;
    want.userdata = impl_.get();
    SDL_AudioSpec have;
    SDL_zero(have);
    const SDL_AudioDeviceID dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
    if (dev == 0) {
        LOG_WARN("audio: no audio device (%s); running silent", SDL_GetError());
        impl_.reset();
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return false;
    }
    device_ = dev;
    first_ = true;
    LOG_INFO("audio: driver %s, %d Hz, %d ch, %d samples", SDL_GetCurrentAudioDriver(), have.freq,
             static_cast<int>(have.channels), static_cast<int>(have.samples));
    SDL_PauseAudioDevice(dev, 0);
    return true;
}

void AudioEngine::shutdown() {
    if (device_ == 0) return;
    SDL_CloseAudioDevice(device_);  // waits for the callback thread to finish
    device_ = 0;
    impl_.reset();
    SDL_QuitSubSystem(SDL_INIT_AUDIO);
}

void AudioEngine::update(const AudioParams& params, const std::vector<AudioEvent>& events) {
    if (device_ == 0) return;
    impl_->synth.setParams(params);
    if (first_) {
        first_ = false;
        impl_->synth.snapToParams();  // note: the callback thread may race; harmless smoothing
    }
    for (const AudioEvent& e : events) impl_->synth.post(e);
}

void AudioEngine::postEvent(const AudioEvent& e) {
    if (device_ != 0) impl_->synth.post(e);
}

unsigned long AudioEngine::callbackCount() const {
    return impl_ ? impl_->callbacks.load(std::memory_order_relaxed) : 0;
}

}  // namespace fsim::audio
