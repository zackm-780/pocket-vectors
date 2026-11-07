#pragma once
// X3: frame timing for the device performance pass. The app times each pass of a frame on the
// CPU and adds one PerfFrame per frame. FrameStats keeps a short window (the once-a-second
// show_fps log line and the on-screen readout) and session totals per scene (cockpit, chase,
// map, menu) and sky (clear, a cloud layer drawn, inside a layer), with a frame-time histogram
// for the 95th percentile. Adding a frame never allocates.
//
// A pass's CPU time is when its GL calls were issued, not when the GPU ran them. With vsync the
// swap absorbs the GPU work still queued and the wait for the next refresh, so "cpu" (frame minus
// swap) above about 16 ms means the CPU limits the frame rate. With perf_gpu_sync the app calls
// glFinish after each pass, so each pass also holds its GPU time. On a tiler like the Mali that
// splits the frame into several render passes and the total grows: compare passes with each
// other, not with an unsynced frame.

#include <string>

namespace fsim {

// "update" is everything before the 3D view: input, menus, the flight model, GPWS, audio and the
// terrain tile requests. "terrain" is the clear, the sky and the terrain.
enum class PerfPass { Update, Terrain, Airports, Clouds, Model, Overlay, Hud, Swap };
constexpr int kPerfPassCount = 8;
const char* perfPassName(PerfPass p);

enum class PerfScene { Cockpit, Chase, Map, Menu };
constexpr int kPerfSceneCount = 4;
enum class PerfSky { Clear, Layer, Inside };
constexpr int kPerfSkyCount = 3;
const char* perfSceneName(PerfScene s);  // "cockpit", "chase", "map", "menu"
const char* perfSkyName(PerfSky s);      // "", "clouds", "in cloud"

struct PerfFrame {
    double frameMs = 0;  // wall time of the whole frame
    double passMs[kPerfPassCount] = {};
    PerfScene scene = PerfScene::Cockpit;
    PerfSky sky = PerfSky::Clear;
    int terrainTriangles = 0;
    int drawCalls = 0;  // terrain, airports (X3c), clouds, the model, overlays and the HUD
    void setPass(PerfPass p, double ms) { passMs[static_cast<int>(p)] = ms; }
};

constexpr double kPerfBinMs = 0.5;
constexpr int kPerfBins = 200;  // 0..100 ms; longer frames go to the last bin

struct PerfTotals {
    long frames = 0;
    double frameMsSum = 0, worstFrameMs = 0;
    double passMsSum[kPerfPassCount] = {};
    double trianglesSum = 0, drawCallsSum = 0;
    int histogram[kPerfBins] = {};

    void add(const PerfFrame& f);
    double meanFrameMs() const { return frames > 0 ? frameMsSum / frames : 0; }
    double fps() const { return frameMsSum > 0 ? 1000.0 * frames / frameMsSum : 0; }
    double meanPassMs(PerfPass p) const {
        return frames > 0 ? passMsSum[static_cast<int>(p)] / frames : 0;
    }
    double meanCpuMs() const { return meanFrameMs() - meanPassMs(PerfPass::Swap); }
    // Frame time that `fraction` (0..1) of the frames stay at or under: the upper edge of its
    // histogram bin, and never more than the worst frame.
    double percentileMs(double fraction) const;
};

class FrameStats {
public:
    void add(const PerfFrame& f);

    const PerfTotals& window() const { return window_; }
    void resetWindow() { window_ = PerfTotals{}; }
    const PerfTotals& session(PerfScene scene, PerfSky sky) const {
        return session_[static_cast<int>(scene)][static_cast<int>(sky)];
    }
    long sessionFrames() const { return sessionFrames_; }
    // Starts a new session table (the window keeps running), e.g. when the quality preset changes.
    void resetSession();
    PerfScene lastScene() const { return lastScene_; }
    PerfSky lastSky() const { return lastSky_; }

    // The show_fps log line for the window, e.g.
    // "29.9 fps | frame 33.4 ms max 41.0 cpu 14.1 | update 2.1 terrain 6.3 ... | cockpit clouds".
    std::string windowLine() const;
    // The session table: a header line ("perf session (<label>): ..."; no parentheses without a
    // label), then one line per scene and sky that had frames.
    std::string sessionSummary(const char* label = nullptr) const;

private:
    PerfTotals window_;
    PerfTotals session_[kPerfSceneCount][kPerfSkyCount];
    long sessionFrames_ = 0;
    PerfScene lastScene_ = PerfScene::Cockpit;
    PerfSky lastSky_ = PerfSky::Clear;
};

}  // namespace fsim
