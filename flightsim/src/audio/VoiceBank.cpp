#include "audio/VoiceBank.h"

#include <fstream>
#include <iterator>

#include "audio/AudioTypes.h"
#include "core/Log.h"

namespace fsim::audio {
namespace {

std::uint32_t le32(const std::vector<std::uint8_t>& b, std::size_t at) {
    return static_cast<std::uint32_t>(b[at]) | (static_cast<std::uint32_t>(b[at + 1]) << 8) |
           (static_cast<std::uint32_t>(b[at + 2]) << 16) |
           (static_cast<std::uint32_t>(b[at + 3]) << 24);
}
std::uint16_t le16(const std::vector<std::uint8_t>& b, std::size_t at) {
    return static_cast<std::uint16_t>(b[at] | (b[at + 1] << 8));
}
bool tagIs(const std::vector<std::uint8_t>& b, std::size_t at, const char* tag) {
    for (int i = 0; i < 4; ++i)
        if (b[at + static_cast<std::size_t>(i)] != static_cast<std::uint8_t>(tag[i])) return false;
    return true;
}

}  // namespace

const char* voiceClipName(VoiceClip c) {
    switch (c) {
    case VoiceClip::Minimums: return "minimums";
    case VoiceClip::Ft2500: return "2500";
    case VoiceClip::Ft1000: return "1000";
    case VoiceClip::Ft500: return "500";
    case VoiceClip::Ft100: return "100";
    case VoiceClip::Ft50: return "50";
    case VoiceClip::Ft40: return "40";
    case VoiceClip::Ft30: return "30";
    case VoiceClip::Ft20: return "20";
    case VoiceClip::Ft10: return "10";
    case VoiceClip::SinkRate: return "sink_rate";
    case VoiceClip::PullUp: return "pull_up";
    case VoiceClip::Terrain: return "terrain";
    case VoiceClip::TooLowGear: return "too_low_gear";
    case VoiceClip::TooLowFlaps: return "too_low_flaps";
    case VoiceClip::Count: break;
    }
    return "";
}

bool voiceClipForFeet(int ft, VoiceClip& out) {
    struct Entry {
        int ft;
        VoiceClip clip;
    };
    static const Entry kEntries[] = {
        {2500, VoiceClip::Ft2500}, {1000, VoiceClip::Ft1000}, {500, VoiceClip::Ft500},
        {100, VoiceClip::Ft100},   {50, VoiceClip::Ft50},     {40, VoiceClip::Ft40},
        {30, VoiceClip::Ft30},     {20, VoiceClip::Ft20},     {10, VoiceClip::Ft10}};
    for (const Entry& e : kEntries)
        if (e.ft == ft) {
            out = e.clip;
            return true;
        }
    return false;
}

int voiceClipPriority(VoiceClip c) {
    switch (c) {
    case VoiceClip::PullUp: return 5;
    case VoiceClip::Terrain: return 4;
    case VoiceClip::SinkRate: return 3;
    case VoiceClip::TooLowGear:
    case VoiceClip::TooLowFlaps:
    case VoiceClip::Minimums: return 2;
    default: return 1;
    }
}

bool parseWavMono16(const std::vector<std::uint8_t>& b, std::vector<std::int16_t>& out,
                    std::string& error) {
    out.clear();
    if (b.size() < 12 || !tagIs(b, 0, "RIFF") || !tagIs(b, 8, "WAVE")) {
        error = "not a RIFF/WAVE file";
        return false;
    }
    bool haveFmt = false;
    std::size_t at = 12;
    while (at + 8 <= b.size()) {
        const std::uint32_t size = le32(b, at + 4);
        const std::size_t body = at + 8;
        if (body + size > b.size()) {
            error = "chunk runs past the end of the file";
            return false;
        }
        if (tagIs(b, at, "fmt ")) {
            if (size < 16) {
                error = "short fmt chunk";
                return false;
            }
            const std::uint16_t format = le16(b, body), channels = le16(b, body + 2);
            const std::uint32_t rate = le32(b, body + 4);
            const std::uint16_t bits = le16(b, body + 14);
            if (format != 1 || channels != 1 || bits != 16 ||
                rate != static_cast<std::uint32_t>(kSampleRateHz)) {
                error = "need PCM, mono, 16-bit, 22050 Hz";
                return false;
            }
            haveFmt = true;
        } else if (tagIs(b, at, "data")) {
            if (!haveFmt) {
                error = "data before fmt";
                return false;
            }
            out.resize(size / 2);
            for (std::size_t i = 0; i < out.size(); ++i)
                out[i] = static_cast<std::int16_t>(le16(b, body + 2 * i));
            return true;
        }
        at = body + size + (size & 1u);  // chunks are word aligned
    }
    error = "no data chunk";
    return false;
}

int VoiceBank::loadDir(const std::string& dir) {
    int loaded = 0;
    for (int i = 0; i < kVoiceClipCount; ++i) {
        const VoiceClip c = static_cast<VoiceClip>(i);
        const std::string path = dir + "/" + voiceClipName(c) + ".wav";
        std::ifstream in(path, std::ios::binary);
        if (!in) continue;
        const std::vector<std::uint8_t> bytes((std::istreambuf_iterator<char>(in)),
                                              std::istreambuf_iterator<char>());
        std::vector<std::int16_t> samples;
        std::string error;
        if (!parseWavMono16(bytes, samples, error)) {
            LOG_WARN("voice: %s: %s", path.c_str(), error.c_str());
            continue;
        }
        clips_[i] = std::move(samples);
        ++loaded;
    }
    return loaded;
}

std::size_t VoiceBank::bytes() const {
    std::size_t n = 0;
    for (const auto& c : clips_) n += c.size() * sizeof(std::int16_t);
    return n;
}

}  // namespace fsim::audio
