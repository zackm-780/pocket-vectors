// X3: frame timing totals, percentiles, buckets by view and the log lines.
#include <doctest/doctest.h>

#include <string>

#include "app/FrameStats.h"

using namespace fsim;

namespace {

PerfFrame frame(double ms, PerfScene scene = PerfScene::Cockpit, PerfSky sky = PerfSky::Clear) {
    PerfFrame f;
    f.frameMs = ms;
    f.setPass(PerfPass::Update, 2.0);
    f.setPass(PerfPass::Terrain, 5.0);
    f.setPass(PerfPass::Swap, ms - 7.0);
    f.scene = scene;
    f.sky = sky;
    f.terrainTriangles = 20000;
    f.drawCalls = 90;
    return f;
}

}  // namespace

TEST_CASE("frame stats: means, fps and cpu time") {
    PerfTotals t;
    for (int i = 0; i < 30; ++i) t.add(frame(33.3));
    CHECK(t.frames == 30);
    CHECK(t.meanFrameMs() == doctest::Approx(33.3));
    CHECK(t.fps() == doctest::Approx(30.03).epsilon(0.001));
    CHECK(t.meanPassMs(PerfPass::Terrain) == doctest::Approx(5.0));
    CHECK(t.meanCpuMs() == doctest::Approx(7.0));  // frame - swap
    CHECK(t.worstFrameMs == doctest::Approx(33.3));
}

TEST_CASE("frame stats: empty totals are zero") {
    PerfTotals t;
    CHECK(t.fps() == 0);
    CHECK(t.meanFrameMs() == 0);
    CHECK(t.percentileMs(0.95) == 0);
}

TEST_CASE("frame stats: p95 comes from the histogram and never passes the worst frame") {
    PerfTotals t;
    for (int i = 0; i < 95; ++i) t.add(frame(16.6));
    for (int i = 0; i < 5; ++i) t.add(frame(40.2));
    CHECK(t.percentileMs(0.95) == doctest::Approx(17.0));  // 16.6 falls in the 16.5..17.0 bin
    CHECK(t.percentileMs(0.96) == doctest::Approx(40.2));  // bin edge 40.5 clamps to the worst
    CHECK(t.percentileMs(0.5) == doctest::Approx(17.0));

    PerfTotals slow;  // frames beyond the histogram land in the last bin
    slow.add(frame(250.0));
    CHECK(slow.percentileMs(0.95) == doctest::Approx(100.0));
    CHECK(slow.worstFrameMs == doctest::Approx(250.0));
}

TEST_CASE("frame stats: the session splits frames by scene and sky") {
    FrameStats s;
    for (int i = 0; i < 10; ++i) s.add(frame(16.7));
    for (int i = 0; i < 4; ++i) s.add(frame(33.4, PerfScene::Chase));
    for (int i = 0; i < 3; ++i) s.add(frame(20.0, PerfScene::Cockpit, PerfSky::Inside));
    CHECK(s.sessionFrames() == 17);
    CHECK(s.session(PerfScene::Cockpit, PerfSky::Clear).frames == 10);
    CHECK(s.session(PerfScene::Chase, PerfSky::Clear).frames == 4);
    CHECK(s.session(PerfScene::Cockpit, PerfSky::Inside).frames == 3);
    CHECK(s.session(PerfScene::Map, PerfSky::Clear).frames == 0);
    CHECK(s.window().frames == 17);
    CHECK(s.lastSky() == PerfSky::Inside);

    s.resetWindow();
    CHECK(s.window().frames == 0);
    CHECK(s.sessionFrames() == 17);  // the session keeps counting
}

TEST_CASE("frame stats: the log lines name every pass and only the views that had frames") {
    FrameStats s;
    for (int i = 0; i < 30; ++i) s.add(frame(33.3, PerfScene::Cockpit, PerfSky::Layer));
    const std::string line = s.windowLine();
    CHECK(line.find("30.0 fps") == 0);
    CHECK(line.find("terrain 5.0") != std::string::npos);
    CHECK(line.find("swap 26.3") != std::string::npos);
    CHECK(line.find("| cockpit clouds") != std::string::npos);

    for (int i = 0; i < 10; ++i) s.add(frame(16.7, PerfScene::Chase));
    const std::string table = s.sessionSummary();
    CHECK(table.find("perf session: 40 frames") == 0);
    CHECK(table.find("cockpit clouds") != std::string::npos);
    CHECK(table.find("chase") != std::string::npos);
    CHECK(table.find("map") == std::string::npos);
    CHECK(table.find("menu") == std::string::npos);
    CHECK(table.find("20000") != std::string::npos);  // mean terrain triangles
    CHECK(table.back() != '\n');
    CHECK(s.sessionSummary("LOW").find("perf session (LOW): 40 frames") == 0);

    s.resetSession();  // a new table, e.g. after the quality preset changed
    CHECK(s.sessionFrames() == 0);
    CHECK(s.session(PerfScene::Chase, PerfSky::Clear).frames == 0);
    CHECK(s.window().frames == 40);  // the window keeps running
}
