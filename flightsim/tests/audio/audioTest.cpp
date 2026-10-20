// S1 procedural audio: offline renders, no audio device needed.
#include <SDL.h>
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include "audio/AudioControl.h"
#include "audio/AudioEngine.h"
#include "audio/AudioSynth.h"

using namespace fsim;
using namespace fsim::audio;

namespace {

std::vector<float> renderSec(AudioSynth& syn, float sec) {
    std::vector<float> v(static_cast<size_t>(sec * kSampleRateHz));
    syn.render(v.data(), static_cast<int>(v.size()));
    return v;
}

float rms(const std::vector<float>& v, size_t from = 0, size_t to = 0) {
    if (to == 0 || to > v.size()) to = v.size();
    double a = 0;
    for (size_t i = from; i < to; ++i) a += static_cast<double>(v[i]) * v[i];
    return static_cast<float>(std::sqrt(a / static_cast<double>(to - from)));
}

float peakAbs(const std::vector<float>& v) {
    float p = 0;
    for (float x : v) p = std::fmax(p, std::fabs(x));
    return p;
}

// Frequency of the strongest Hann-windowed DFT bin in [loHz, hiHz], 1 Hz resolution.
float dftPeakHz(const std::vector<float>& v, float loHz, float hiHz) {
    const size_t n = v.size();
    float best = 0, bestMag = -1;
    for (float f = loHz; f <= hiHz; f += 1.0f) {
        double re = 0, im = 0;
        const double w = 6.283185307179586 * f / kSampleRateHz;
        for (size_t i = 0; i < n; ++i) {
            const double hann = 0.5 - 0.5 * std::cos(6.283185307179586 * static_cast<double>(i) /
                                                     static_cast<double>(n - 1));
            re += v[i] * hann * std::cos(w * static_cast<double>(i));
            im += v[i] * hann * std::sin(w * static_cast<double>(i));
        }
        const double mag = re * re + im * im;
        if (mag > bestMag) {
            bestMag = static_cast<float>(mag);
            best = f;
        }
    }
    return best;
}

AudioParams pistonParams(float rpm) {
    AudioParams p;
    p.engineKind = EngineKind::Piston;
    p.engineCount = 1;
    p.engRunning[0] = true;
    p.engRpm[0] = rpm;
    p.engThrottleNorm[0] = 0.7f;
    return p;
}

AudioParams jetParams(float n1) {
    AudioParams p;
    p.engineKind = EngineKind::Jet;
    p.engineCount = 1;
    p.engRunning[0] = true;
    p.engN1Pct[0] = n1;
    p.engThrottleNorm[0] = 0.7f;
    return p;
}

AudioParams everything() {
    AudioParams p;
    p.engineKind = EngineKind::Turboprop;
    p.engineCount = 4;
    p.cylinders = 6;
    for (int i = 0; i < 4; ++i) {
        p.engRunning[i] = true;
        p.engRpm[i] = 2700;
        p.engN1Pct[i] = 100;
        p.engThrottleNorm[i] = 1;
    }
    p.iasKt = 350;
    p.gearMoving = p.flapsMoving = p.stallHorn = true;
    return p;
}

}  // namespace

TEST_CASE("audio: piston fundamental tracks RPM x cylinders / 2") {
    for (float rpm : {1200.0f, 2000.0f, 2600.0f}) {
        AudioSynth syn;
        syn.setParams(pistonParams(rpm));
        syn.snapToParams();
        const auto v = renderSec(syn, 1.0f);
        const float expectHz = rpm / 60.0f * 4 / 2;
        const float got = dftPeakHz(v, 20, 400);
        CHECK(std::fabs(got - expectHz) <= 2.0f);
    }
}

TEST_CASE("audio: turbine whine tracks N1 and roar grows") {
    float prevHz = 0, prevRms = 0;
    for (float n1 : {40.0f, 65.0f, 95.0f}) {
        AudioSynth syn;
        syn.setParams(jetParams(n1));
        syn.snapToParams();
        const auto v = renderSec(syn, 1.0f);
        const float expectHz = 600.0f + 3800.0f * n1 / 100.0f;
        const float got = dftPeakHz(v, 500, 5000);
        CHECK(std::fabs(got - expectHz) <= 3.0f);
        CHECK(got > prevHz);
        CHECK(rms(v) > prevRms);
        prevHz = got;
        prevRms = rms(v);
    }
}

TEST_CASE("audio: engine pitch glides when RPM changes") {
    AudioSynth syn;
    syn.setParams(pistonParams(1200));
    syn.snapToParams();
    renderSec(syn, 0.3f);
    syn.setParams(pistonParams(2400));
    renderSec(syn, 1.5f);  // settle
    const auto v = renderSec(syn, 1.0f);
    CHECK(std::fabs(dftPeakHz(v, 20, 400) - 80.0f) <= 2.0f);
}

TEST_CASE("audio: wind loudness rises with IAS") {
    float prev = -1;
    for (float ias : {0.0f, 40.0f, 90.0f, 150.0f, 250.0f}) {
        AudioSynth syn;
        AudioParams p;
        p.iasKt = ias;
        syn.setParams(p);
        syn.snapToParams();
        const float r = rms(renderSec(syn, 1.0f));
        CHECK(r >= prev);
        if (ias > 0) CHECK(r > prev);
        prev = r;
    }
    CHECK(prev > 0.05f);
}

TEST_CASE("audio: stall horn follows the condition") {
    AudioSynth syn;
    AudioParams p;
    syn.setParams(p);
    syn.snapToParams();
    const float off1 = rms(renderSec(syn, 0.3f));
    p.stallHorn = true;
    syn.setParams(p);
    renderSec(syn, 0.1f);
    const auto onBuf = renderSec(syn, 0.5f);
    CHECK(rms(onBuf) > 0.1f);
    CHECK(std::fabs(dftPeakHz(onBuf, 300, 700) - 460.0f) <= 3.0f);
    p.stallHorn = false;
    syn.setParams(p);
    renderSec(syn, 0.2f);
    const float off2 = rms(renderSec(syn, 0.3f));
    CHECK(off1 < 0.001f);
    CHECK(off2 < 0.001f);
}

TEST_CASE("audio: a tone event plays exactly once") {
    AudioSynth syn;
    syn.setParams(AudioParams());
    syn.snapToParams();
    syn.post(AudioControl::minimumsTone());  // 3 beeps of 0.14 s every 0.24 s = 0.62 s
    const auto v = renderSec(syn, 1.5f);
    const size_t sr = kSampleRateHz;
    int beeps = 0;
    bool inBeep = false;
    for (size_t i = 0; i + 100 < v.size(); i += 100) {
        const bool loud = rms(v, i, i + 100) > 0.05f;
        if (loud && !inBeep) ++beeps;
        inBeep = loud;
    }
    CHECK(beeps == 3);
    CHECK(rms(v, sr * 3 / 4, v.size()) < 0.001f);  // silent after the sequence ends
}

TEST_CASE("audio: control fires touchdown, AP disconnect and minimums once per edge") {
    AircraftState s;
    s.valid = true;
    s.engineCount = 1;
    s.wowAny = false;
    s.velDownMps = 1.5f;
    s.gsKt = 60;
    AircraftProfile prof;
    AutopilotState ap;
    AudioControl ctl;
    AudioParams params;
    std::vector<AudioEvent> ev;
    auto step = [&](bool minimums = false) {
        ev.clear();
        s.simTimeS += 1.0 / 60.0;
        AudioFrameInput in;
        in.state = &s;
        in.profile = &prof;
        in.ap = &ap;
        in.minimumsAlert = minimums;
        ctl.update(in, params, ev);
    };
    int touchdowns = 0, tones = 0;
    auto count = [&] {
        for (const AudioEvent& e : ev) {
            if (e.type == AudioEventType::Touchdown) ++touchdowns;
            if (e.type == AudioEventType::Tone) ++tones;
        }
    };
    for (int i = 0; i < 10; ++i) {
        step();
        count();
    }
    CHECK(touchdowns == 0);
    s.wowAny = true;  // touchdown
    for (int i = 0; i < 10; ++i) {
        step();
        count();
    }
    CHECK(touchdowns == 1);
    CHECK(tones == 0);
    s.wowAny = false;  // lift off: no event
    step();
    count();
    CHECK(touchdowns == 1);
    ap.disconnectWarning = true;
    for (int i = 0; i < 30; ++i) {
        step();
        count();
    }
    CHECK(tones == 1);  // one tone for the whole flashing warning
    ap.disconnectWarning = false;
    step();
    ap.disconnectWarning = true;
    step();
    count();
    CHECK(tones == 2);
    step(true);
    count();
    step(false);
    count();
    CHECK(tones == 3);
}

TEST_CASE("audio: starting on the ground fires no touchdown; touchdown strength follows sink") {
    AircraftState s;
    s.valid = true;
    s.wowAny = true;
    AircraftProfile prof;
    AudioControl ctl;
    AudioParams params;
    std::vector<AudioEvent> ev;
    AudioFrameInput in;
    in.state = &s;
    in.profile = &prof;
    ctl.update(in, params, ev);
    CHECK(ev.empty());
    auto landing = [&](float sinkMps) {
        s.wowAny = false;
        s.velDownMps = sinkMps;
        s.simTimeS += 0.1;
        ev.clear();
        ctl.update(in, params, ev);
        s.wowAny = true;
        s.simTimeS += 0.1;
        ctl.update(in, params, ev);
        REQUIRE(ev.size() == 1);
        return ev[0].intensity01;
    };
    CHECK(landing(0.3f) < landing(2.5f));
}

TEST_CASE("audio: gear and flap motors sound only while moving; stall flag passes through") {
    AircraftState s;
    s.valid = true;
    AircraftProfile prof;
    AudioControl ctl;
    AudioParams params;
    std::vector<AudioEvent> ev;
    AudioFrameInput in;
    in.state = &s;
    in.profile = &prof;
    ctl.update(in, params, ev);
    CHECK_FALSE(params.gearMoving);
    s.gearPosNorm = 0.9f;
    s.simTimeS += 0.02;
    ctl.update(in, params, ev);
    CHECK(params.gearMoving);
    CHECK_FALSE(params.flapsMoving);
    for (int i = 0; i < 60; ++i) {
        s.simTimeS += 0.02;
        ctl.update(in, params, ev);
    }
    CHECK_FALSE(params.gearMoving);
    in.stallWarning = true;
    ctl.update(in, params, ev);
    CHECK(params.stallHorn);
}

TEST_CASE("audio: volume gain and mute while paused") {
    CHECK(AudioControl::volumeGain(0) == 0.0f);
    CHECK(AudioControl::volumeGain(100) == doctest::Approx(1.0f));
    CHECK(AudioControl::volumeGain(50) < AudioControl::volumeGain(80));
    CHECK(AudioControl::volumeGain(500) == doctest::Approx(1.0f));

    AudioSynth syn;
    AudioParams p = everything();
    syn.setParams(p);
    syn.snapToParams();
    CHECK(rms(renderSec(syn, 0.3f)) > 0.05f);
    p.paused = true;
    syn.setParams(p);
    renderSec(syn, 0.5f);
    CHECK(rms(renderSec(syn, 0.2f)) < 0.001f);
    syn.post(AudioEvent{AudioEventType::UiClick});
    CHECK(rms(renderSec(syn, 0.05f)) > 0.001f);  // UI click still plays
    p.paused = false;
    p.masterGain01 = 0.0f;
    syn.setParams(p);
    renderSec(syn, 0.3f);
    CHECK(rms(renderSec(syn, 0.2f)) < 0.001f);
}

TEST_CASE("audio: no clipping, NaN or denormals with every voice at full volume") {
    AudioSynth syn;
    AudioParams p = everything();
    syn.setParams(p);
    syn.snapToParams();
    syn.post(makeTone(1000, 0.3f, 4, 0.05f, 1.0f, 700));
    AudioEvent td;
    td.type = AudioEventType::Touchdown;
    syn.post(td);
    syn.post(td);
    const auto v = renderSec(syn, 3.0f);
    CHECK(peakAbs(v) <= 1.0f);
    for (float x : v) {
        REQUIRE(std::isfinite(x));
        REQUIRE(std::fpclassify(x) != FP_SUBNORMAL);
    }
    // garbage in must not blow up either
    AudioParams bad = everything();
    bad.iasKt = NAN;
    bad.engRpm[0] = INFINITY;
    bad.engN1Pct[1] = -1e30f;
    bad.masterGain01 = NAN;
    syn.setParams(bad);
    syn.post(makeTone(NAN, NAN, 1000000, NAN, NAN));
    const auto w = renderSec(syn, 1.0f);
    CHECK(peakAbs(w) <= 1.0f);
    for (float x : w) REQUIRE(std::isfinite(x));
    // decay to silence: gains reach exactly zero rather than denormals
    AudioParams quiet;
    syn.setParams(quiet);
    renderSec(syn, 5.0f);
    for (float x : renderSec(syn, 0.2f)) REQUIRE(std::fpclassify(x) != FP_SUBNORMAL);
}

TEST_CASE("audio: rendering 1 s with every voice active is cheap") {
    AudioSynth syn;
    syn.setParams(everything());
    syn.snapToParams();
    std::vector<float> buf(kSampleRateHz);
    double best = 1e9;
    for (int rep = 0; rep < 5; ++rep) {
        syn.post(makeTone(1000, 0.3f, 3, 0.05f, 0.5f, 700));
        AudioEvent td;
        td.type = AudioEventType::Touchdown;
        syn.post(td);
        const auto t0 = std::chrono::steady_clock::now();
        syn.render(buf.data(), static_cast<int>(buf.size()));
        const auto t1 = std::chrono::steady_clock::now();
        best = std::fmin(best, std::chrono::duration<double, std::milli>(t1 - t0).count());
    }
    std::printf("[audio] 1 s all voices render: %.2f ms (best of 5)\n", best);
    CHECK(best < 10.0);
}

TEST_CASE("audio: engine runs without a device (null sink) and with the dummy driver") {
    setenv("SDL_AUDIODRIVER", "dummy",
           0);  // CI has no sound card; the dummy driver still runs a callback
    AudioEngine eng;
    AudioParams p = everything();
    std::vector<AudioEvent> ev{makeTone(800, 0.1f)};
    if (!eng.init()) {  // no device: everything must still be a safe no-op
        CHECK_FALSE(eng.deviceOpen());
    }
    eng.update(p, ev);
    eng.postEvent(AudioEvent{AudioEventType::UiClick});
    if (eng.deviceOpen()) {
        for (int i = 0; i < 100 && eng.callbackCount() == 0; ++i) SDL_Delay(10);
        CHECK(eng.callbackCount() > 0);
    }
    eng.shutdown();
    eng.shutdown();
    CHECK_FALSE(eng.deviceOpen());
}

TEST_CASE("audio: takeoff sequence WAV (only when FSIM_AUDIO_WAV is set)") {
    const char* path = std::getenv("FSIM_AUDIO_WAV");
    if (!path) return;
    AudioSynth syn;
    AudioControl ctl;
    AircraftState s;
    s.valid = true;
    s.engineCount = 1;
    s.engines[0].running = true;
    s.wowAny = true;
    s.gearRetractable = true;
    AircraftProfile prof;
    AutopilotState ap;
    std::vector<AudioEvent> ev;
    AudioParams params;
    std::vector<std::int16_t> pcm;
    const int framesPerS = 60;
    const int perFrame = kSampleRateHz / framesPerS;  // 367 (the rest is rounded off; fine)
    std::vector<float> buf(perFrame);
    const float totalS = 40;
    for (int f = 0; f < static_cast<int>(totalS * framesPerS); ++f) {
        const float t = static_cast<float>(f) / framesPerS;
        s.simTimeS = t;
        // 0-4 idle, 4-14 takeoff roll, 14 rotate, 15 airborne + gear up (15-23), flaps retract
        // 20-24, climb, 28-31 stall horn, 33 AP disconnect, 35 minimums, 38 touchdown
        const float thr = t < 4 ? 0.1f : (t < 36 ? 1.0f : 0.2f);
        s.engines[0].throttleCmdNorm = thr;
        s.engines[0].rpm = 700 + 1900 * thr;
        s.iasKt = t < 4 ? 0
                        : (t < 14 ? (t - 4) * 7.5f
                                  : (t < 28 ? 75 + (t - 14) * 1.5f : (t < 36 ? 70 : 60)));
        s.gsKt = s.iasKt;
        s.wowAny = t < 14.5f || t >= 38;
        s.velDownMps = t >= 36 && t < 38 ? 1.2f : -3.0f;
        s.gearPosNorm = t < 15 ? 1.0f : (t < 23 ? 1.0f - (t - 15) / 8.0f : 0.0f);
        if (t >= 36 && t < 36.5f) s.gearPosNorm = 0.0f;
        s.flapPosNorm = t < 20 ? 0.3f : (t < 24 ? 0.3f * (1 - (t - 20) / 4.0f) : 0.0f);
        ap.disconnectWarning = t >= 33 && t < 36;
        AudioFrameInput in;
        in.state = &s;
        in.profile = &prof;
        in.ap = &ap;
        in.stallWarning = t >= 28 && t < 31;
        in.minimumsAlert = f == static_cast<int>(35 * framesPerS);
        in.volumePct = 100;
        ev.clear();
        ctl.update(in, params, ev);
        syn.setParams(params);
        for (const AudioEvent& e : ev) syn.post(e);
        if (f == 0) syn.snapToParams();
        syn.render(buf.data(), perFrame);
        for (float x : buf) pcm.push_back(static_cast<std::int16_t>(std::lrintf(x * 32767.0f)));
    }
    CHECK(writeWavMono16(path, pcm.data(), static_cast<int>(pcm.size()), kSampleRateHz));
}
