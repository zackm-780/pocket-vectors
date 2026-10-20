#include "audio/AudioSynth.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fsim::audio {
namespace {

constexpr float kTwoPi = 6.28318530718f;
constexpr float kDt = 1.0f / static_cast<float>(kSampleRateHz);

inline float approach(float cur, float tgt, float k) {
    const float d = tgt - cur;
    if (std::fabs(d) < 1e-5f) return tgt;  // snap: also keeps decaying gains out of denormals
    return cur + d * k;
}
inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}
inline float fracf(float v) {
    return v - std::floor(v);
}
inline float finiteOr(float v, float fallback) {
    return std::isfinite(v) ? v : fallback;
}

// Amplitude compensation so one-pole low-passed uniform noise has roughly constant RMS.
inline float noiseNorm(float c) {
    return 0.6f * std::sqrt((2.0f - c) / c);
}

template <int N> inline float lut(const float (&tab)[N], float phase01) {
    const float x = phase01 * static_cast<float>(N - 1);
    const int i = static_cast<int>(x);
    const float f = x - static_cast<float>(i);
    return tab[i] + (tab[i + 1] - tab[i]) * f;
}

inline float roarCoef(float whineHz) {
    return 0.08f + 0.35f * clampf(whineHz / 4400.0f, 0.0f, 1.0f);
}

}  // namespace

AudioSynth::AudioSynth() {
    for (int i = 0; i <= kTable; ++i) {
        const float ph = static_cast<float>(i) / static_cast<float>(kTable);
        sinTab_[i] = std::sin(kTwoPi * ph);
    }
    // One firing pulse per cycle: a decaying harmonic series, normalized to peak 1. The
    // fundamental stays the strongest partial so the pitch tracks RPM audibly and measurably.
    float peak = 0;
    for (int i = 0; i <= kTable; ++i) {
        const float ph = static_cast<float>(i) / static_cast<float>(kTable);
        float v = 0;
        for (int k = 1; k <= 10; ++k)
            v += std::sin(kTwoPi * static_cast<float>(k) * ph + 0.5f * static_cast<float>(k - 1)) /
                 std::pow(static_cast<float>(k), 1.1f);
        pistonTab_[i] = v;
        peak = std::max(peak, std::fabs(v));
    }
    for (float& v : pistonTab_) v /= peak;
    pistonTab_[kTable] = pistonTab_[0];
    sinTab_[kTable] = sinTab_[0];
}

void AudioSynth::setParams(const AudioParams& p) {
    std::lock_guard<std::mutex> lk(paramMutex_);
    shared_ = p;
    sharedDirty_ = true;
}

void AudioSynth::snapToParams() {
    snapRequested_ = true;
    updateChunk();
}

bool AudioSynth::post(const AudioEvent& e) {
    const unsigned head = ringHead_.load(std::memory_order_relaxed);
    const unsigned tail = ringTail_.load(std::memory_order_acquire);
    if (head - tail >= static_cast<unsigned>(kRing)) return false;
    ring_[head % kRing] = e;
    ringHead_.store(head + 1, std::memory_order_release);
    return true;
}

float AudioSynth::nextNoise() {
    rng_ ^= rng_ << 13;
    rng_ ^= rng_ >> 17;
    rng_ ^= rng_ << 5;
    return static_cast<float>(static_cast<std::int32_t>(rng_)) * (1.0f / 2147483648.0f);
}

void AudioSynth::startEvent(const AudioEvent& e) {
    switch (e.type) {
    case AudioEventType::Touchdown: {
        TouchdownVoice& v = td_[tdNext_];
        tdNext_ = (tdNext_ + 1) % 2;
        v = TouchdownVoice();
        v.active = true;
        v.inten = clampf(finiteOr(e.intensity01, 0.5f), 0.0f, 1.0f);
        v.chirp = clampf(finiteOr(e.chirp01, 0.0f), 0.0f, 1.0f);
        break;
    }
    case AudioEventType::Tone: {
        if (e.repeats <= 0 || !(e.durS > 0.0f) || !(e.freqHz > 0.0f)) break;
        int slot = 0;
        bool found = false;
        for (int i = 0; i < kToneSlots; ++i) {
            if (!tones_[i].active) {
                slot = i;
                found = true;
                break;
            }
            if (tones_[i].tS > tones_[slot].tS) slot = i;  // otherwise steal the oldest
        }
        (void)found;
        ToneSlot& t = tones_[slot];
        t = ToneSlot();
        t.active = true;
        t.ev = e;
        t.ev.freqHz = clampf(e.freqHz, 50.0f, 8000.0f);
        t.ev.freqAltHz = e.freqAltHz > 0 ? clampf(e.freqAltHz, 50.0f, 8000.0f) : 0.0f;
        t.ev.gain01 = clampf(finiteOr(e.gain01, 0.5f), 0.0f, 1.0f);
        t.ev.gapS = std::max(0.0f, finiteOr(e.gapS, 0.0f));
        t.ev.durS = clampf(e.durS, 0.005f, 10.0f);
        t.ev.repeats = std::min(e.repeats, 64);
        break;
    }
    case AudioEventType::UiClick: clickT_ = 0; break;
    }
}

void AudioSynth::drainEvents() {
    unsigned tail = ringTail_.load(std::memory_order_relaxed);
    const unsigned head = ringHead_.load(std::memory_order_acquire);
    while (tail != head) {
        startEvent(ring_[tail % kRing]);
        ++tail;
    }
    ringTail_.store(tail, std::memory_order_release);
}

// Runs once per kChunk samples: pulls new params (try_lock only) and moves the smoothed values.
void AudioSynth::updateChunk() {
    if (paramMutex_.try_lock()) {
        if (sharedDirty_) {
            target_ = shared_;
            sharedDirty_ = false;
        }
        paramMutex_.unlock();
    }
    const AudioParams& p = target_;
    const bool snap = snapRequested_;
    snapRequested_ = false;
    const float kSlow = snap ? 1.0f : 0.04f;  // ~70 ms time constant
    const float kFast = snap ? 1.0f : 0.25f;  // ~10 ms
    const float kPitch = snap ? 1.0f : 0.12f;

    master_ = approach(master_, clampf(finiteOr(p.masterGain01, 1.0f), 0.0f, 1.0f), kFast);
    simGain_ = approach(simGain_, p.paused ? 0.0f : 1.0f, kFast * 0.5f);

    const int n = std::clamp(p.engineCount, 0, kMaxEngineVoices);
    // Several engines share the mix: n equal voices are scaled so the total stays sane.
    const float share = n > 0 ? 1.0f / std::sqrt(static_cast<float>(n)) : 0.0f;
    for (int i = 0; i < kMaxEngineVoices; ++i) {
        float gain = 0, roar = 0, blade = 0, lump = 0;
        float freq = engFreq_[i];
        float bladeHz = engBladeHz_[i];
        if (i < n && p.engineKind != EngineKind::None && p.engRunning[i]) {
            const float thr = clampf(finiteOr(p.engThrottleNorm[i], 0.0f), 0.0f, 1.0f);
            const float detune = 1.0f + 0.004f * static_cast<float>(i);
            if (p.engineKind == EngineKind::Piston) {
                const float rpm = clampf(finiteOr(p.engRpm[i], 0.0f), 0.0f, 6000.0f);
                const int cyl = std::clamp(p.cylinders, 1, 12);
                freq = rpm / 60.0f * static_cast<float>(cyl) * 0.5f * detune;
                gain = share * 0.30f * (0.45f + 0.55f * thr) * clampf(rpm / 700.0f, 0.0f, 1.0f);
                lump = 0.15f;
            } else {
                const float n1 = clampf(finiteOr(p.engN1Pct[i], 0.0f), 0.0f, 110.0f) / 100.0f;
                freq = (600.0f + 3800.0f * n1) * detune;
                gain = share * 0.05f * n1 * n1;
                roar = share * 0.38f * std::pow(n1, 1.5f);
                if (p.engineKind == EngineKind::Turboprop) {
                    const float rpm = clampf(finiteOr(p.engRpm[i], 0.0f), 0.0f, 3000.0f);
                    bladeHz = rpm / 60.0f * 3.0f;  // three blades
                    blade = share * 0.10f * clampf(rpm / 1000.0f, 0.0f, 1.0f);
                }
            }
        }
        engGain_[i] = approach(engGain_[i], gain, kSlow);
        engRoar_[i] = approach(engRoar_[i], roar, kSlow);
        engBlade_[i] = approach(engBlade_[i], blade, kSlow);
        engLump_[i] = lump;
        engFreq_[i] = (snap || engFreq_[i] <= 0.0f) ? freq : approach(engFreq_[i], freq, kPitch);
        engBladeHz_[i] =
            (snap || engBladeHz_[i] <= 0.0f) ? bladeHz : approach(engBladeHz_[i], bladeHz, kPitch);
    }

    const float ias = clampf(finiteOr(p.iasKt, 0.0f), 0.0f, 600.0f);
    const float x = ias / 250.0f;
    windAmp_ = approach(windAmp_, 0.5f * std::pow(std::min(x, 1.6f), 1.6f), kSlow);
    windCoef_ = approach(windCoef_, 0.02f + 0.33f * std::min(x, 1.0f), kSlow);
    gustState_ += 0.02f * (nextNoise() - gustState_);
    gust_ = clampf(1.0f + 3.0f * gustState_, 0.6f, 1.4f);

    gearGain_ = approach(gearGain_, p.gearMoving ? 1.0f : 0.0f, kFast);
    flapGain_ = approach(flapGain_, p.flapsMoving ? 1.0f : 0.0f, kFast);
    hornGain_ = approach(hornGain_, p.stallHorn ? 1.0f : 0.0f, kFast * 1.2f);
}

void AudioSynth::render(float* out, int n) {
    int done = 0;
    while (done < n) {
        drainEvents();
        updateChunk();
        const int len = std::min(kChunk, n - done);
        const AudioParams& p = target_;
        const int engines = std::clamp(p.engineCount, 0, kMaxEngineVoices);
        const bool piston = p.engineKind == EngineKind::Piston;
        const bool turboprop = p.engineKind == EngineKind::Turboprop;
        const float windNorm = noiseNorm(windCoef_);
        const float firingToRev = 2.0f / static_cast<float>(std::clamp(p.cylinders, 1, 12));
        float roarC[kMaxEngineVoices], roarN[kMaxEngineVoices];
        for (int e = 0; e < kMaxEngineVoices; ++e) {
            roarC[e] = roarCoef(engFreq_[e]);
            roarN[e] = noiseNorm(roarC[e]);
        }

        for (int s = 0; s < len; ++s) {
            const float noise = nextNoise();
            float mix = 0;

            // ---- engines
            for (int e = 0; e < engines; ++e) {
                if (engGain_[e] <= 0.0f && engRoar_[e] <= 0.0f && engBlade_[e] <= 0.0f) continue;
                enginePhase_[e] = fracf(enginePhase_[e] + engFreq_[e] * kDt);
                if (piston) {
                    lumpPhase_[e] = fracf(lumpPhase_[e] + engFreq_[e] * firingToRev * kDt);
                    const float lump = 1.0f + engLump_[e] * lut(sinTab_, lumpPhase_[e]);
                    mix += engGain_[e] * lump * lut(pistonTab_, enginePhase_[e]);
                } else {
                    const float whine = lut(sinTab_, enginePhase_[e]) +
                                        0.4f * lut(sinTab_, fracf(2.0f * enginePhase_[e]));
                    roarLp_[e] += roarC[e] * (noise - roarLp_[e]);
                    mix += engGain_[e] * whine + engRoar_[e] * roarN[e] * roarLp_[e];
                    if (turboprop) {
                        bladePhase_[e] = fracf(bladePhase_[e] + engBladeHz_[e] * kDt);
                        mix += engBlade_[e] * (lut(sinTab_, bladePhase_[e]) +
                                               0.5f * lut(sinTab_, fracf(2.0f * bladePhase_[e])));
                    }
                }
            }

            // ---- wind
            windLp_ += windCoef_ * (noise - windLp_);
            mix += windAmp_ * gust_ * windNorm * windLp_;

            // ---- gear and flap motors
            if (gearGain_ > 0.0f) {
                gearPhase_ = fracf(gearPhase_ + 95.0f * kDt);
                gearAm_ = fracf(gearAm_ + 23.0f * kDt);
                gearNoiseLp_ += 0.2f * (noise - gearNoiseLp_);
                const float am = 0.75f + 0.25f * lut(sinTab_, gearAm_);
                mix += gearGain_ * am *
                       (0.10f * lut(sinTab_, gearPhase_) +
                        0.05f * lut(sinTab_, fracf(2.0f * gearPhase_)) + 0.12f * gearNoiseLp_);
            }
            if (flapGain_ > 0.0f) {
                flapPhase_ = fracf(flapPhase_ + 160.0f * kDt);
                flapAm_ = fracf(flapAm_ + 41.0f * kDt);
                const float am = 0.7f + 0.3f * lut(sinTab_, flapAm_);
                mix += flapGain_ * am *
                       (0.07f * lut(sinTab_, flapPhase_) +
                        0.04f * lut(sinTab_, fracf(2.0f * flapPhase_)) + 0.04f * noise);
            }

            // ---- stall horn (continuous reed tone, odd harmonics)
            if (hornGain_ > 0.0f) {
                hornPhase_ = fracf(hornPhase_ + 460.0f * kDt);
                mix += hornGain_ * 0.22f *
                       (lut(sinTab_, hornPhase_) + 0.45f * lut(sinTab_, fracf(3.0f * hornPhase_)) +
                        0.2f * lut(sinTab_, fracf(5.0f * hornPhase_)));
            }

            mix *= simGain_;

            // ---- touchdown thump and tyre chirp
            for (TouchdownVoice& v : td_) {
                if (!v.active) continue;
                v.thumpPhase = fracf(v.thumpPhase + 58.0f * kDt);
                v.lp += 0.12f * (noise - v.lp);
                const float thump = lut(sinTab_, v.thumpPhase) * std::exp(-v.tS / 0.11f) * 0.8f +
                                    v.lp * 2.0f * std::exp(-v.tS / 0.05f);
                const float chirp = (noise - v.lp) * std::exp(-v.tS / 0.07f) * 0.30f * v.chirp;
                mix += v.inten * (thump + chirp);
                v.tS += kDt;
                if (v.tS > 1.5f) v.active = false;
            }

            // ---- alert tones
            for (ToneSlot& t : tones_) {
                if (!t.active) continue;
                const float cycle = t.ev.durS + t.ev.gapS;
                const int idx = static_cast<int>(t.tS / cycle);
                if (idx >= t.ev.repeats) {
                    t.active = false;
                    continue;
                }
                const float inBeep = t.tS - static_cast<float>(idx) * cycle;
                if (inBeep < t.ev.durS) {
                    const float ramp = std::min(0.005f, t.ev.durS * 0.25f);
                    const float env = std::min({1.0f, inBeep / ramp, (t.ev.durS - inBeep) / ramp});
                    const float f =
                        (idx % 2 == 1 && t.ev.freqAltHz > 0) ? t.ev.freqAltHz : t.ev.freqHz;
                    t.phase = fracf(t.phase + f * kDt);
                    mix += t.ev.gain01 * env * lut(sinTab_, t.phase);
                }
                t.tS += kDt;
            }

            // ---- UI click (ignores the pause fade; still follows master volume)
            float click = 0;
            if (clickT_ < 0.03f) {
                click = 0.25f * std::exp(-clickT_ / 0.006f) * noise;
                clickT_ += kDt;
            }

            // ---- master volume and a smooth limiter: clamp, then 1.5x - 0.5x^3 (peak exactly 1)
            float y = clampf((mix + click) * master_, -1.0f, 1.0f);
            y = 1.5f * y - 0.5f * y * y * y;
            out[done + s] = y;
        }
        done += len;
    }
}

void AudioSynth::renderS16(std::int16_t* out, int n) {
    constexpr int kBlock = 256;
    float buf[kBlock];
    int done = 0;
    while (done < n) {
        const int len = std::min(kBlock, n - done);
        render(buf, len);
        for (int i = 0; i < len; ++i)
            out[done + i] =
                static_cast<std::int16_t>(std::lrintf(clampf(buf[i], -1.0f, 1.0f) * 32767.0f));
        done += len;
    }
}

bool writeWavMono16(const char* path, const std::int16_t* samples, int count, int rateHz) {
    std::FILE* f = std::fopen(path, "wb");
    if (!f) return false;
    auto u32 = [&](std::uint32_t v) {
        const unsigned char b[4] = {
            static_cast<unsigned char>(v), static_cast<unsigned char>(v >> 8),
            static_cast<unsigned char>(v >> 16), static_cast<unsigned char>(v >> 24)};
        std::fwrite(b, 1, 4, f);
    };
    auto u16 = [&](std::uint16_t v) {
        const unsigned char b[2] = {static_cast<unsigned char>(v),
                                    static_cast<unsigned char>(v >> 8)};
        std::fwrite(b, 1, 2, f);
    };
    const std::uint32_t dataBytes = static_cast<std::uint32_t>(count) * 2u;
    std::fwrite("RIFF", 1, 4, f);
    u32(36u + dataBytes);
    std::fwrite("WAVEfmt ", 1, 8, f);
    u32(16);
    u16(1);
    u16(1);
    u32(static_cast<std::uint32_t>(rateHz));
    u32(static_cast<std::uint32_t>(rateHz) * 2u);
    u16(2);
    u16(16);
    std::fwrite("data", 1, 4, f);
    u32(dataBytes);
    for (int i = 0; i < count; ++i) u16(static_cast<std::uint16_t>(samples[i]));
    return std::fclose(f) == 0;
}

}  // namespace fsim::audio
