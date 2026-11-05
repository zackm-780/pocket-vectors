#include "audio/Callouts.h"

#include "audio/AudioControl.h"

namespace fsim::audio {
namespace {

AudioEvent toneFor(AlertKind kind) {
    switch (kind) {  // freq, duration, repeats, gap, gain, alternate freq (the X2 tones)
    case AlertKind::Callout: return makeTone(1000, 0.08f);
    case AlertKind::SinkRate:
    case AlertKind::Terrain: return makeTone(700, 0.2f, 2, 0.12f, 0.6f);
    case AlertKind::PullUp: return makeTone(800, 0.25f, 3, 0.05f, 0.7f, 1200);
    case AlertKind::TooLowGear:
    case AlertKind::TooLowFlaps: return makeTone(600, 0.15f, 2, 0.1f, 0.6f);
    case AlertKind::Minimums: break;
    }
    return AudioControl::minimumsTone();
}

bool clipFor(AlertKind kind, int valueFt, VoiceClip& out) {
    switch (kind) {
    case AlertKind::Callout: return voiceClipForFeet(valueFt, out);
    case AlertKind::SinkRate: out = VoiceClip::SinkRate; return true;
    case AlertKind::PullUp: out = VoiceClip::PullUp; return true;
    case AlertKind::Terrain: out = VoiceClip::Terrain; return true;
    case AlertKind::TooLowGear: out = VoiceClip::TooLowGear; return true;
    case AlertKind::TooLowFlaps: out = VoiceClip::TooLowFlaps; return true;
    case AlertKind::Minimums: out = VoiceClip::Minimums; return true;
    }
    return false;
}

}  // namespace

AudioEvent alertEvent(AlertKind kind, int valueFt, const VoiceBank* bank) {
    VoiceClip clip;
    if (bank && clipFor(kind, valueFt, clip) && bank->has(clip)) {
        AudioEvent e;
        e.type = AudioEventType::Voice;
        e.clip = static_cast<std::uint8_t>(clip);
        e.priority = static_cast<std::uint8_t>(voiceClipPriority(clip));
        e.gain01 = 0.8f;
        return e;
    }
    return toneFor(kind);
}

}  // namespace fsim::audio
