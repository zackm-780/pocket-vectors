// X3c: flight data in the external chase view (HUD.md section 7a). The readout's layout keeps
// clear of everything else the chase view draws at 640 x 480 and 16:9, its values follow the
// cockpit's rules, and the FMA row shows only with the AP or FD on.
#include <doctest/doctest.h>

#include <cstring>
#include <string>
#include <vector>

#include "core/MessageQueue.h"
#include "core/Units.h"
#include "gfx/Camera.h"
#include "hud/Hud.h"
#include "hud/NavHud.h"

using namespace fsim;

namespace {

bool overlaps(const HudRect& a, const HudRect& b) {
    if (a.w <= 0 || a.h <= 0 || b.w <= 0 || b.h <= 0) return false;
    return a.x < b.x + b.w && b.x < a.x + a.w && a.y < b.y + b.h && b.y < a.y + a.h;
}

HudRect grown(const HudRect& r, float px) {
    return {r.x - px, r.y - px, r.w + 2 * px, r.h + 2 * px};
}

// A centred text line as the HUD draws warnings and messages, `cells` characters wide.
HudRect centeredLine(float virtualWidth, float baselineY, int cells, TextSize size) {
    const float w = static_cast<float>(cells) * StrokeFont::advancePx(size);
    const float cap = StrokeFont::capHeightPx(size);
    return {virtualWidth * 0.5f - w * 0.5f - 2, baselineY - cap - 2, w + 4, cap + 4};
}

// Everything else the chase view draws, at its widest.
std::vector<std::pair<std::string, HudRect>> otherElements(float vw) {
    std::vector<std::pair<std::string, HudRect>> out;
    out.push_back({"status panel", {0, hudzones::kStatusPanelTopPx, vw, 480 - 430}});
    out.push_back({"FMA row", {0, hudzones::kFma.y, vw, hudzones::kFma.h}});
    out.push_back({"SHOW FPS", hudzones::kFpsReadout});
    // formatWarning writes into 40 chars: up to 39 cells, L for the first, M for the second.
    out.push_back({"warning 1", centeredLine(vw, hudzones::kWarningBaselinePx, 39, TextSize::L)});
    out.push_back(
        {"warning 2", centeredLine(vw, hudzones::kWarningBaselinePx - 20, 39, TextSize::M)});
    // Message::text holds 31 chars; up to kMaxShownMessages lines.
    for (int i = 0; i < kMaxShownMessages; ++i)
        out.push_back({"message " + std::to_string(i),
                       centeredLine(vw, hudzones::kMessageBaselinePx + hudzones::kMessageStepPx * i,
                                    static_cast<int>(sizeof(Message::text)) - 1, TextSize::M)});
    return out;
}

// The readout at its widest: Mach shown, five-digit VS, inHg setting, three-digit GS.
ChaseReadout widestReadout(float vw) {
    AircraftState s;
    s.valid = true;
    s.iasKt = 345;
    s.gsKt = 512;
    s.mach = 0.82f;
    s.altMslFt = 38980;
    s.vsFpm = -4350;
    s.altimeterSettingHpa = 1001.0f;
    HudOptions opt;
    opt.baroInHg = true;
    return buildChaseReadout(s, opt, visibilityFor(Declutter::Full), vw);
}

std::vector<std::pair<std::string, HudRect>> readoutRects(const ChaseReadout& c) {
    std::vector<std::pair<std::string, HudRect>> out;
    out.push_back({"speed box", grown(c.speedBox, 1.5f)});  // stroke and outline
    out.push_back({"altitude box", grown(c.altBox, 1.5f)});
    out.push_back({"heading box", grown(c.headingBox, 1.5f)});
    out.push_back({"GS", chaseLineRect(c.gs)});
    out.push_back({"Mach", chaseLineRect(c.mach)});
    out.push_back({"VS", chaseLineRect(c.vs)});
    out.push_back({"altimeter", chaseLineRect(c.baro)});
    return out;
}

struct ChaseScene {
    gfx::GlApi noGl;  // the renderer records geometry without init(); no GL calls
    Renderer2D r2d{noGl, gfx::GlFlavor::Es2};
    gfx::Camera cam;
    AircraftState s;
    ControlInputs c;
    AircraftProfile p;
    MessageQueue msgs;
    HudOptions opt;
    Hud hud;

    ChaseScene() {
        s.valid = true;
        s.latDeg = 45.5;
        s.lonDeg = -122.6;
        s.altMslFt = 3000;
        s.altAglFt = 2000;
        s.headingTrueRad = 1.5f;
        s.iasKt = 110;
        s.gsKt = 112;
        s.velEastMps = 55;
        s.engineCount = 1;
        s.engines[0].running = true;
        p.engineCount = 1;
        opt.externalView = true;
        cam.setLens(55.0f);
        cam.setViewport(640, 480);
        cam.setPoseFromAircraft(s);
    }
    int vertices() {
        r2d.begin(640, 480);
        hud.draw(r2d, cam, s, c, p, msgs, opt);
        const int n = r2d.pendingVertexCount();
        r2d.end();
        return n;
    }
};

}  // namespace

TEST_CASE("hud chase: the readout stays clear of the strip, warnings, messages, FMA and FPS") {
    for (float vw : {640.0f, 480.0f * 16.0f / 9.0f}) {
        CAPTURE(vw);
        const ChaseReadout c = widestReadout(vw);
        const auto mine = readoutRects(c);
        const auto others = otherElements(vw);
        for (const auto& item : mine) {
            const std::string& name = item.first;
            const HudRect& rect = item.second;
            CAPTURE(name);
            CHECK(rect.w > 0);  // every item is shown in this case
            CHECK(rect.x >= 0);
            CHECK(rect.x + rect.w <= vw);
            for (const auto& other : others) {
                const std::string& otherName = other.first;
                CAPTURE(otherName);
                CHECK_FALSE(overlaps(rect, other.second));
            }
        }
        for (size_t i = 0; i < mine.size(); ++i)
            for (size_t j = i + 1; j < mine.size(); ++j) {
                CAPTURE(mine[i].first);
                CAPTURE(mine[j].first);
                CHECK_FALSE(overlaps(mine[i].second, mine[j].second));
            }
    }
}

TEST_CASE("hud chase: the boxes are the cockpit tapes' boxes, in the cockpit's places") {
    const ChaseReadout c = widestReadout(640);
    CHECK(c.speedBox.x == doctest::Approx(18));
    CHECK(c.speedBox.y + c.speedBox.h * 0.5f == doctest::Approx(hudzones::kCenterY));
    CHECK(c.altBox.x + c.altBox.w == doctest::Approx(640 - 18));
    CHECK(c.altBox.y + c.altBox.h * 0.5f == doctest::Approx(hudzones::kCenterY));
    CHECK(c.headingBox.x + c.headingBox.w * 0.5f == doctest::Approx(320));
    CHECK(c.gs.x == doctest::Approx(16));
    CHECK(c.gs.baselineY == doctest::Approx(388));
    CHECK(c.vs.x == doctest::Approx(640 - 96));
    CHECK(c.baro.baselineY == doctest::Approx(420));
    // 16:9: the right column follows the right edge.
    const ChaseReadout w = widestReadout(853.33f);
    CHECK(w.altBox.x + w.altBox.w == doctest::Approx(853.33f - 18).epsilon(1e-4));
    CHECK(w.vs.x == doctest::Approx(853.33f - 96).epsilon(1e-4));
}

TEST_CASE("hud chase: values follow the cockpit's rules") {
    AircraftState s;
    s.valid = true;
    HudOptions opt;
    const HudVisibility full = visibilityFor(Declutter::Full);

    s.iasKt = 65.4f;
    CHECK(std::string(buildChaseReadout(s, opt, full, 640).speed) == "65");
    s.iasKt = 65.5f;
    CHECK(std::string(buildChaseReadout(s, opt, full, 640).speed) == "66");
    s.iasKt = -3;  // tail wind on the ground
    CHECK(std::string(buildChaseReadout(s, opt, full, 640).speed) == "0");

    // Indicated altitude (with the baro offset), 20 ft steps, thousands split off.
    s.altMslFt = 3950;
    s.altBaroOffsetFt = 30;
    ChaseReadout c = buildChaseReadout(s, opt, full, 640);
    CHECK(std::string(c.altThousands) == "3");
    CHECK(std::string(c.altRest) == "980");
    s.altMslFt = 985;
    s.altBaroOffsetFt = 0;
    c = buildChaseReadout(s, opt, full, 640);
    CHECK(std::string(c.altThousands).empty());
    CHECK(std::string(c.altRest) == "980");

    // Magnetic heading: true 100, variation 15 E.
    s.headingTrueRad = static_cast<float>(degToRad(100.0));
    opt.magVarDeg = 15;
    CHECK(std::string(buildChaseReadout(s, opt, full, 640).heading) == "085");

    s.vsFpm = -340;
    s.gsKt = 69.4f;
    s.mach = 0.30f;
    s.altimeterSettingHpa = 1013.25f;
    c = buildChaseReadout(s, opt, full, 640);
    CHECK(std::string(c.vs.text) == "VS -350");
    CHECK(std::string(c.gs.text) == "GS 69");
    CHECK(std::string(c.mach.text).empty());  // only above M.40
    CHECK(std::string(c.baro.text) == "STD");
    s.mach = 0.78f;
    s.altimeterSettingHpa = 1000.0f;
    c = buildChaseReadout(s, opt, full, 640);
    CHECK(std::string(c.mach.text) == "M.78");
    CHECK(std::string(c.baro.text) == "1000");  // hPa
    opt.baroInHg = true;
    CHECK(std::string(buildChaseReadout(s, opt, full, 640).baro.text) == "29.53");
}

TEST_CASE("hud chase: MIN declutter hides GS, Mach and the heading box, never speed or altitude") {
    AircraftState s;
    s.valid = true;
    s.iasKt = 250;
    s.gsKt = 300;
    s.mach = 0.5f;
    s.vsFpm = 1000;
    const ChaseReadout c = buildChaseReadout(s, HudOptions{}, visibilityFor(Declutter::Min), 640);
    CHECK_FALSE(c.headingShown);
    CHECK(std::string(c.gs.text).empty());
    CHECK(std::string(c.mach.text).empty());
    CHECK(chaseLineRect(c.gs).w == 0);
    CHECK(std::string(c.speed) == "250");
    CHECK(std::string(c.altRest) == "000");
    CHECK(std::string(c.vs.text) == "VS +1000");
    CHECK(std::string(c.baro.text) == "STD");
    const ChaseReadout s2 = buildChaseReadout(s, HudOptions{}, visibilityFor(Declutter::Std), 640);
    CHECK(s2.headingShown);
    CHECK(std::string(s2.gs.text) == "GS 300");
}

TEST_CASE("hud chase: the readout draws and the FMA row needs the AP or the FD") {
    ChaseScene sc;
    sc.opt.externalView = false;
    const int cockpit = sc.vertices();
    sc.opt.externalView = true;
    const int chase = sc.vertices();
    CHECK(chase > 0);
    CHECK(chase < cockpit / 2);  // no tapes, ladder, bank scale or FPV
    CHECK(chase < 12000);        // X3c budget (7.2k before the readout)

    NavHud nav;
    sc.hud.setOverlay(&nav);
    AutopilotState ap;
    NavState ns;
    nav.setStates(ns, ap);
    CHECK(sc.vertices() == chase);  // AP and FD off: no FMA
    ap.apEngaged = true;
    ap.latActive = LatMode::Hdg;
    nav.setStates(ns, ap);
    const int withFma = sc.vertices();
    CHECK(withFma > chase);
    CHECK(withFma < 16000);
    ap.apEngaged = false;
    ap.fdOn = true;
    nav.setStates(ns, ap);
    CHECK(sc.vertices() > chase);
    sc.opt.declutter = Declutter::Min;  // MIN hides the FMA (and the status rows, heading box)
    const int minWithFd = sc.vertices();
    sc.hud.setOverlay(nullptr);
    CHECK(sc.vertices() == minWithFd);
}
