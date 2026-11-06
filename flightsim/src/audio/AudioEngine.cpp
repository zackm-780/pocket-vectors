#include "audio/AudioEngine.h"

#include <SDL.h>

#include <algorithm>
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

int audioBufferSamples(int settingSamples, bool portMaster) {
    if (settingSamples <= 0)
        return portMaster ? AudioEngine::kPortMasterBufferSamples
                          : AudioEngine::kDefaultBufferSamples;
    int n = 256;
    while (n < 4096 && n * 3 / 2 < settingSamples) n *= 2;  // nearest power of two, rounding up
    return n;
}

bool AudioEngine::init(int bufferSamples) {
    if (device_ != 0) return true;
    if (SDL_InitSubSystem(SDL_INIT_AUDIO) != 0) {
        LOG_WARN("audio: SDL audio init failed (%s); running silent", SDL_GetError());
        return false;
    }
    impl_ = std::make_unique<Impl>();
    impl_->synth.setVoiceBank(bank_);  // S2
    SDL_AudioSpec want;
    SDL_zero(want);
    want.freq = kSampleRateHz;
    want.format = AUDIO_S16SYS;
    want.channels = 1;
    want.samples = static_cast<Uint16>(std::clamp(bufferSamples, 64, 8192));
    want.callback = sdlCallback;
    want.userdata = impl_.get();
    SDL_AudioSpec have;
    SDL_zero(have);
    // Let the driver report the buffer it really uses (ALSA rounds the period to what the
    // hardware takes); without this SDL re-blocks to `want.samples` and hides that size. The
    // callback handles any length.
    int allowed = 0;
#ifdef SDL_AUDIO_ALLOW_SAMPLES_CHANGE
    allowed = SDL_AUDIO_ALLOW_SAMPLES_CHANGE;
#endif
    const SDL_AudioDeviceID dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, allowed);
    if (dev == 0) {
        LOG_WARN("audio: no audio device (%s); running silent", SDL_GetError());
        impl_.reset();
        SDL_QuitSubSystem(SDL_INIT_AUDIO);
        return false;
    }
    device_ = dev;
    first_ = true;
    bufferObtained_ = static_cast<int>(have.samples);
    const double bufferMs = have.freq > 0 ? 1000.0 * have.samples / have.freq : 0.0;
    LOG_INFO("audio: driver %s, %d Hz, %d ch, buffer %d samples (%.0f ms; asked %d)",
             SDL_GetCurrentAudioDriver(), have.freq, static_cast<int>(have.channels),
             bufferObtained_, bufferMs, static_cast<int>(want.samples));
    SDL_PauseAudioDevice(dev, 0);
    return true;
}

void AudioEngine::shutdown() {
    if (device_ == 0) return;
    SDL_CloseAudioDevice(device_);  // waits for the callback thread to finish
    device_ = 0;
    bufferObtained_ = 0;
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

void AudioEngine::setVoiceBank(const VoiceBank* bank) {
    bank_ = bank;
    if (impl_) impl_->synth.setVoiceBank(bank);
}

}  // namespace fsim::audio
