// S2: GPWS voice callouts. WAV loading, the shipped clips, the alert -> sound mapping with the
// tone fallback, the synth's voice channel (priority, queue) and a scripted approach whose audio
// event log names every clip.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "audio/AudioSynth.h"
#include "audio/Callouts.h"
#include "audio/VoiceBank.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "hud/GpwsMonitor.h"

using namespace fsim;
using namespace fsim::audio;

namespace {

std::vector<std::uint8_t> makeWav(int channels, int bits, int rate, int samples) {
    std::vector<std::uint8_t> b;
    auto s = [&](const char* t) { b.insert(b.end(), t, t + 4); };
    auto u32 = [&](std::uint32_t v) {
        for (int i = 0; i < 4; ++i) b.push_back(static_cast<std::uint8_t>(v >> (8 * i)));
    };
    auto u16 = [&](std::uint16_t v) {
        b.push_back(static_cast<std::uint8_t>(v));
        b.push_back(static_cast<std::uint8_t>(v >> 8));
    };
    const std::uint32_t data = static_cast<std::uint32_t>(samples * channels * bits / 8);
    s("RIFF");
    u32(36 + data);
    s("WAVE");
    s("fmt ");
    u32(16);
    u16(1);
    u16(static_cast<std::uint16_t>(channels));
    u32(static_cast<std::uint32_t>(rate));
    u32(static_cast<std::uint32_t>(rate * channels * bits / 8));
    u16(static_cast<std::uint16_t>(channels * bits / 8));
    u16(static_cast<std::uint16_t>(bits));
    s("data");
    u32(data);
    for (int i = 0; i < samples * channels; ++i) {
        if (bits == 16)
            u16(static_cast<std::uint16_t>(static_cast<std::int16_t>(1000 * ((i % 7) - 3))));
        else
            b.push_back(static_cast<std::uint8_t>(128 + i % 7));
    }
    return b;
}

const VoiceBank& shippedBank() {
    static VoiceBank bank;
    static const int loaded = bank.loadDir("assets/audio/voice");  // tests run in the build dir
    (void)loaded;
    return bank;
}

// A synthetic bank: clip i is i+1 hundredths of a second of a 440 Hz tone.
VoiceBank toyBank() {
    VoiceBank b;
    for (int i = 0; i < kVoiceClipCount; ++i) {
        std::vector<std::int16_t> v(static_cast<size_t>(kSampleRateHz / 100 * (i + 1)));
        for (size_t k = 0; k < v.size(); ++k)
            v[k] = static_cast<std::int16_t>(
                8000.0 * std::sin(6.283185307 * 440.0 * static_cast<double>(k) / kSampleRateHz));
        b.set(static_cast<VoiceClip>(i), std::move(v));
    }
    return b;
}

float renderRms(AudioSynth& syn, float sec) {
    std::vector<float> v(static_cast<size_t>(sec * kSampleRateHz));
    syn.render(v.data(), static_cast<int>(v.size()));
    double a = 0;
    for (float x : v) a += static_cast<double>(x) * x;
    return static_cast<float>(std::sqrt(a / static_cast<double>(v.size())));
}

// No engine, wind or other voices: only the events posted.
void quiet(AudioSynth& syn) {
    AudioParams p;
    syn.setParams(p);
    syn.snapToParams();
}

}  // namespace

TEST_CASE("voice: WAV parsing accepts 16-bit mono 22050 Hz only") {
    std::vector<std::int16_t> out;
    std::string err;
    REQUIRE(parseWavMono16(makeWav(1, 16, kSampleRateHz, 100), out, err));
    CHECK(out.size() == 100);
    CHECK(out[0] == -3000);
    CHECK_FALSE(parseWavMono16(makeWav(2, 16, kSampleRateHz, 100), out, err));
    CHECK_FALSE(parseWavMono16(makeWav(1, 8, kSampleRateHz, 100), out, err));
    CHECK_FALSE(parseWavMono16(makeWav(1, 16, 16000, 100), out, err));
    std::vector<std::uint8_t> cut = makeWav(1, 16, kSampleRateHz, 100);
    cut.resize(cut.size() - 50);
    CHECK_FALSE(parseWavMono16(cut, out, err));
    CHECK_FALSE(parseWavMono16({'R', 'I', 'F', 'F'}, out, err));
}

TEST_CASE("voice: every shipped clip loads, is short and the set stays under 2 MB") {
    const VoiceBank& bank = shippedBank();
    for (int i = 0; i < kVoiceClipCount; ++i) {
        const VoiceClip c = static_cast<VoiceClip>(i);
        INFO(voiceClipName(c));
        REQUIRE(bank.has(c));
        const double sec = static_cast<double>(bank.clip(c).size()) / kSampleRateHz;
        CHECK(sec > 0.3);
        CHECK(sec < 2.0);
    }
    MESSAGE("voice clips: " << bank.bytes() / 1024 << " KiB");
    CHECK(bank.bytes() < 2u * 1024u * 1024u);
}

TEST_CASE("voice: alerts are spoken with a clip, the X2 tone without one") {
    const VoiceBank& bank = shippedBank();
    AudioEvent e = alertEvent(AlertKind::Callout, 500, &bank);
    CHECK(e.type == AudioEventType::Voice);
    CHECK(e.clip == static_cast<std::uint8_t>(VoiceClip::Ft500));
    e = alertEvent(AlertKind::PullUp, 0, &bank);
    CHECK(e.type == AudioEventType::Voice);
    CHECK(e.priority > alertEvent(AlertKind::SinkRate, 0, &bank).priority);
    CHECK(alertEvent(AlertKind::SinkRate, 0, &bank).priority >
          alertEvent(AlertKind::Callout, 100, &bank).priority);
    CHECK(alertEvent(AlertKind::Minimums, 0, &bank).type == AudioEventType::Voice);
    // No bank (voice off) or no clip: the tones as before.
    CHECK(alertEvent(AlertKind::Callout, 500, nullptr).type == AudioEventType::Tone);
    CHECK(alertEvent(AlertKind::Callout, 777, &bank).type == AudioEventType::Tone);
    VoiceBank partial;
    partial.set(VoiceClip::Terrain, std::vector<std::int16_t>(100, 1));
    CHECK(alertEvent(AlertKind::Terrain, 0, &partial).type == AudioEventType::Voice);
    const AudioEvent pullUpTone = alertEvent(AlertKind::PullUp, 0, &partial);
    CHECK(pullUpTone.type == AudioEventType::Tone);
    CHECK(pullUpTone.freqAltHz == doctest::Approx(1200.f));  // the X2 warble
}

TEST_CASE("voice: the synth plays a clip once, a warning interrupts a callout, equals queue") {
    const VoiceBank bank = toyBank();
    AudioSynth syn;
    quiet(syn);
    syn.setVoiceBank(&bank);
    CHECK(renderRms(syn, 0.05f) < 1e-4f);
    // One callout: audible while it plays, silent after.
    syn.post(alertEvent(AlertKind::Callout, 500, &bank));
    CHECK(renderRms(syn, 0.02f) > 0.05f);
    CHECK(syn.voiceClipPlaying() == static_cast<int>(VoiceClip::Ft500));
    renderRms(syn, 0.2f);
    CHECK(syn.voiceClipPlaying() == -1);
    CHECK(renderRms(syn, 0.05f) < 1e-4f);
    // PULL UP cuts into a callout at once.
    syn.post(alertEvent(AlertKind::Callout, 2500, &bank));
    renderRms(syn, 0.005f);
    REQUIRE(syn.voiceClipPlaying() == static_cast<int>(VoiceClip::Ft2500));
    syn.post(alertEvent(AlertKind::PullUp, 0, &bank));
    renderRms(syn, 0.005f);
    CHECK(syn.voiceClipPlaying() == static_cast<int>(VoiceClip::PullUp));
    renderRms(syn, 0.5f);
    // Two callouts in one frame: the second waits for the first.
    syn.post(alertEvent(AlertKind::Callout, 50, &bank));
    syn.post(alertEvent(AlertKind::Callout, 40, &bank));
    renderRms(syn, 0.005f);
    CHECK(syn.voiceClipPlaying() == static_cast<int>(VoiceClip::Ft50));
    renderRms(syn, 0.06f);  // Ft50 is 0.06 s in the toy bank
    CHECK(syn.voiceClipPlaying() == static_cast<int>(VoiceClip::Ft40));
}

// The S2 acceptance: a scripted approach through GpwsMonitor; the audio event log is the list of
// clips the alerts would speak, and each of them plays on the synth.
TEST_CASE("voice: scripted approach logs a clip for every GPWS event") {
    struct Flat : TerrainQuery {
        float elevationM(double, double) const override {
            return static_cast<float>(SyntheticNavDb::kElevFt * kMPerFt);
        }
        LandClass landClass(double, double) const override { return LandClass::Grass; }
        void addFlattenArea(const FlattenArea&) override {}
        void clearFlattenAreas() override {}
        void prefetch(double, double, float) override {}
        bool isDetailReady(double, double) const override { return true; }
    } flat;
    SyntheticNavDb db;
    const VoiceBank& bank = shippedBank();
    auto flyDown = [&](double startAglFt, float vsFpm, bool gearUp, double stopAglFt) {
        AircraftState s;
        AircraftProfile profile;
        profile.vappKt = 65;
        profile.hasRetractableGear = gearUp;
        GpwsMonitor mon;
        s.valid = true;
        s.latDeg = 40.0 - 0.5 / 60.0;
        s.lonDeg = -100.0;
        s.altMslFt = SyntheticNavDb::kElevFt + startAglFt;
        s.gsKt = s.iasKt = 80;
        s.vsFpm = vsFpm;
        s.gearRetractable = gearUp;
        s.gearPosNorm = gearUp ? 0.f : 1.f;
        std::vector<std::string> log;
        for (int i = 0; i < 20000; ++i) {
            s.simTimeS += 0.1;
            s.latDeg += ktToMps(s.gsKt) * 0.1 / (kEarthRadiusM * kRadPerDeg);
            s.altMslFt += s.vsFpm / 60.0 * 0.1;
            s.terrainElevFt = static_cast<float>(SyntheticNavDb::kElevFt);
            s.altAglFt = static_cast<float>(s.altMslFt - s.terrainElevFt);
            if (s.altAglFt < stopAglFt) break;
            mon.update(s, profile, nullptr, flat, &db);
            for (GpwsEvent ev; mon.consumeEvent(ev);) {
                AlertKind k = AlertKind::Callout;
                if (ev.kind == GpwsEventKind::SinkRate) k = AlertKind::SinkRate;
                if (ev.kind == GpwsEventKind::PullUp) k = AlertKind::PullUp;
                if (ev.kind == GpwsEventKind::Terrain) k = AlertKind::Terrain;
                if (ev.kind == GpwsEventKind::TooLowGear) k = AlertKind::TooLowGear;
                if (ev.kind == GpwsEventKind::TooLowFlaps) k = AlertKind::TooLowFlaps;
                const AudioEvent ae = alertEvent(k, ev.valueFt, &bank);
                REQUIRE(ae.type == AudioEventType::Voice);  // every event has its clip
                log.emplace_back(voiceClipName(static_cast<VoiceClip>(ae.clip)));
            }
        }
        return log;
    };
    // A normal approach: the height callouts, in order.
    std::vector<std::string> log = flyDown(3000, -600, false, 5);
    std::string joined;
    for (const std::string& c : log) joined += c + " ";
    MESSAGE("approach audio log: " << joined);
    CHECK(log ==
          std::vector<std::string>{"2500", "1000", "500", "100", "50", "40", "30", "20", "10"});
    // 2,500 fpm at 1,000 ft: SINK RATE; 3,500 fpm: PULL UP (the X2 envelopes).
    log = flyDown(1000, -2500, false, 400);
    joined.clear();
    for (const std::string& c : log) joined += c + " ";
    MESSAGE("sink audio log: " << joined);
    CHECK(std::find(log.begin(), log.end(), "sink_rate") != log.end());
    log = flyDown(1000, -3500, false, 400);
    CHECK(std::find(log.begin(), log.end(), "pull_up") != log.end());
    // Gear up on the way down: TOO LOW, GEAR.
    log = flyDown(900, -500, true, 200);
    CHECK(std::find(log.begin(), log.end(), "too_low_gear") != log.end());

    // Each clip of the approach log plays on the synth, one after the other.
    AudioSynth syn;
    quiet(syn);
    syn.setVoiceBank(&bank);
    for (const char* name : {"2500", "500", "pull_up", "minimums"}) {
        VoiceClip c = VoiceClip::Count;
        for (int i = 0; i < kVoiceClipCount; ++i)
            if (std::strcmp(voiceClipName(static_cast<VoiceClip>(i)), name) == 0)
                c = static_cast<VoiceClip>(i);
        REQUIRE(c != VoiceClip::Count);
        AudioEvent e;
        e.type = AudioEventType::Voice;
        e.clip = static_cast<std::uint8_t>(c);
        e.priority = static_cast<std::uint8_t>(voiceClipPriority(c));
        e.gain01 = 0.8f;
        syn.post(e);
        CHECK(renderRms(syn, 0.3f) > 0.02f);
        CHECK(syn.voiceClipPlaying() <= static_cast<int>(c));
        renderRms(syn, 2.0f);
        CHECK(syn.voiceClipPlaying() == -1);
    }
}
