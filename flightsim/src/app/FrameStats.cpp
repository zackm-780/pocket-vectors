#include "app/FrameStats.h"

#include <algorithm>
#include <cstdio>

namespace fsim {

const char* perfPassName(PerfPass p) {
    switch (p) {
    case PerfPass::Update: return "update";
    case PerfPass::Terrain: return "terrain";
    case PerfPass::Airports: return "airports";
    case PerfPass::Clouds: return "clouds";
    case PerfPass::Model: return "model";
    case PerfPass::Overlay: return "overlay";
    case PerfPass::Hud: return "hud";
    case PerfPass::Swap: return "swap";
    }
    return "?";
}

const char* perfSceneName(PerfScene s) {
    switch (s) {
    case PerfScene::Cockpit: return "cockpit";
    case PerfScene::Chase: return "chase";
    case PerfScene::Map: return "map";
    case PerfScene::Menu: return "menu";
    }
    return "?";
}

const char* perfSkyName(PerfSky s) {
    switch (s) {
    case PerfSky::Clear: return "";
    case PerfSky::Layer: return "clouds";
    case PerfSky::Inside: return "in cloud";
    }
    return "?";
}

void PerfTotals::add(const PerfFrame& f) {
    const double ms = std::max(0.0, f.frameMs);
    ++frames;
    frameMsSum += ms;
    worstFrameMs = std::max(worstFrameMs, ms);
    for (int i = 0; i < kPerfPassCount; ++i) passMsSum[i] += std::max(0.0, f.passMs[i]);
    trianglesSum += f.terrainTriangles;
    drawCallsSum += f.drawCalls;
    const int bin = std::min(kPerfBins - 1, static_cast<int>(ms / kPerfBinMs));
    ++histogram[bin];
}

double PerfTotals::percentileMs(double fraction) const {
    if (frames <= 0) return 0;
    const double want = std::clamp(fraction, 0.0, 1.0) * static_cast<double>(frames);
    long seen = 0;
    for (int i = 0; i < kPerfBins; ++i) {
        seen += histogram[i];
        if (static_cast<double>(seen) >= want && seen > 0)
            return std::min(worstFrameMs, (i + 1) * kPerfBinMs);
    }
    return worstFrameMs;
}

void FrameStats::add(const PerfFrame& f) {
    window_.add(f);
    session_[static_cast<int>(f.scene)][static_cast<int>(f.sky)].add(f);
    ++sessionFrames_;
    lastScene_ = f.scene;
    lastSky_ = f.sky;
}

void FrameStats::resetSession() {
    for (auto& row : session_)
        for (PerfTotals& t : row) t = PerfTotals{};
    sessionFrames_ = 0;
}

std::string FrameStats::windowLine() const {
    char buf[320];
    int n = std::snprintf(buf, sizeof buf, "%.1f fps | frame %.1f ms max %.1f cpu %.1f |",
                          window_.fps(), window_.meanFrameMs(), window_.worstFrameMs,
                          window_.meanCpuMs());
    for (int i = 0; i < kPerfPassCount && n > 0 && n < static_cast<int>(sizeof buf); ++i)
        n += std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), " %s %.1f",
                           perfPassName(static_cast<PerfPass>(i)),
                           window_.meanPassMs(static_cast<PerfPass>(i)));
    if (n > 0 && n < static_cast<int>(sizeof buf)) {
        const char* sky = perfSkyName(lastSky_);
        std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), " | %s%s%s",
                      perfSceneName(lastScene_), sky[0] ? " " : "", sky);
    }
    return buf;
}

std::string FrameStats::sessionSummary(const char* label) const {
    std::string out;
    char buf[320];
    const bool named = label && label[0];
    std::snprintf(buf, sizeof buf,
                  "perf session%s%s%s: %ld frames (ms per frame; cpu = frame - swap; p95 in %.1f "
                  "ms steps)\n",
                  named ? " (" : "", named ? label : "", named ? ")" : "", sessionFrames_,
                  kPerfBinMs);
    out += buf;
    int n = std::snprintf(buf, sizeof buf, "  %-17s %6s %5s %5s %5s %6s %5s |", "view", "frames",
                          "fps", "frame", "p95", "worst", "cpu");
    for (int i = 0; i < kPerfPassCount; ++i)
        n += std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), " %7s",
                           perfPassName(static_cast<PerfPass>(i)));
    std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), " | %6s %5s\n", "tris", "draws");
    out += buf;
    for (int s = 0; s < kPerfSceneCount; ++s) {
        for (int k = 0; k < kPerfSkyCount; ++k) {
            const PerfTotals& t = session_[s][k];
            if (t.frames == 0) continue;
            char view[32];
            const char* sky = perfSkyName(static_cast<PerfSky>(k));
            std::snprintf(view, sizeof view, "%s%s%s", perfSceneName(static_cast<PerfScene>(s)),
                          sky[0] ? " " : "", sky);
            n = std::snprintf(buf, sizeof buf, "  %-17s %6ld %5.1f %5.1f %5.1f %6.1f %5.1f |", view,
                              t.frames, t.fps(), t.meanFrameMs(), t.percentileMs(0.95),
                              t.worstFrameMs, t.meanCpuMs());
            for (int i = 0; i < kPerfPassCount; ++i)
                n += std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), " %7.2f",
                                   t.meanPassMs(static_cast<PerfPass>(i)));
            std::snprintf(buf + n, sizeof buf - static_cast<size_t>(n), " | %6.0f %5.0f\n",
                          t.trianglesSum / static_cast<double>(t.frames),
                          t.drawCallsSum / static_cast<double>(t.frames));
            out += buf;
        }
    }
    if (!out.empty() && out.back() == '\n') out.pop_back();
    return out;
}

}  // namespace fsim
