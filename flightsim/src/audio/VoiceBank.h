#pragma once
// S2: GPWS voice clips, loaded once at start from assets/audio/voice/<name>.wav (16-bit mono PCM
// at kSampleRateHz, made by tools/audio/make_voice_clips.py). Read-only after loading, so the
// audio thread can play from it without locks. A missing clip is not an error: the alert falls
// back to its X2 tone (audio/Callouts.h).
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fsim::audio {

enum class VoiceClip : std::uint8_t {
    Minimums,
    Ft2500,
    Ft1000,
    Ft500,
    Ft100,
    Ft50,
    Ft40,
    Ft30,
    Ft20,
    Ft10,
    SinkRate,
    PullUp,
    Terrain,
    TooLowGear,
    TooLowFlaps,
    Count
};
constexpr int kVoiceClipCount = static_cast<int>(VoiceClip::Count);

const char* voiceClipName(VoiceClip c);  // file stem: "minimums", "2500", "sink_rate", ...
// The callout clip for a radio altitude GpwsMonitor calls (2500 ... 10); false for others.
bool voiceClipForFeet(int ft, VoiceClip& out);
// Higher interrupts lower: PULL UP 5, TERRAIN 4, SINK RATE 3, TOO LOW and MINIMUMS 2, heights 1.
int voiceClipPriority(VoiceClip c);

// Parses a RIFF/WAVE file: PCM, 16-bit, mono, kSampleRateHz. Anything else is an error.
bool parseWavMono16(const std::vector<std::uint8_t>& bytes, std::vector<std::int16_t>& out,
                    std::string& error);

class VoiceBank {
public:
    // Loads every clip found in `dir` (<name>.wav); returns how many loaded. Bad files are
    // logged and skipped.
    int loadDir(const std::string& dir);
    bool has(VoiceClip c) const { return !clip(c).empty(); }
    const std::vector<std::int16_t>& clip(VoiceClip c) const {
        return clips_[static_cast<int>(c) % kVoiceClipCount];
    }
    void set(VoiceClip c, std::vector<std::int16_t> samples) {
        clips_[static_cast<int>(c)] = std::move(samples);
    }
    std::size_t bytes() const;

private:
    std::vector<std::int16_t> clips_[kVoiceClipCount];
};

}  // namespace fsim::audio
