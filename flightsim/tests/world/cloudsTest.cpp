// W7 cloud layers without GL: parsing, resolution from scenario / setting / METAR, the coverage
// pattern and the per-frame draw plan.
#include <doctest/doctest.h>

#include <cmath>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "app/Scenario.h"
#include "app/Settings.h"
#include "core/Paths.h"
#include "world/CloudsLogic.h"
#include "world/WeatherMetar.h"

using namespace fsim;
using namespace fsim::world;

namespace {

std::string readFixture(const std::string& rel) {
    for (const char* base : {"tests/fixtures/", "../tests/fixtures/", "../../tests/fixtures/"}) {
        std::ifstream f(std::string(base) + rel);
        if (f) {
            std::stringstream ss;
            ss << f.rdbuf();
            return ss.str();
        }
    }
    return {};
}

CloudSetup scenarioLayers(std::initializer_list<const char*> texts) {
    CloudSetup s;
    for (const char* t : texts) {
        CloudLayerSpec l;
        REQUIRE(parseCloudLayer(t, l));
        s.layers.push_back(l);
    }
    s.set = true;
    return s;
}

}  // namespace

TEST_CASE("clouds: layer text parses in the scenario and METAR styles") {
    CloudLayerSpec l;
    REQUIRE(parseCloudLayer("OVC 1500 3500", l));
    CHECK(l.cover == CloudCover::Overcast);
    CHECK(l.baseAglFt == doctest::Approx(1500.f));
    CHECK(l.topAglFt == doctest::Approx(3500.f));
    REQUIRE(parseCloudLayer("bkn 2500", l));
    CHECK(l.cover == CloudCover::Broken);
    CHECK(l.topAglFt == doctest::Approx(0.f));  // default thickness later
    REQUIRE(parseCloudLayer("SCT015", l));
    CHECK(l.cover == CloudCover::Scattered);
    CHECK(l.baseAglFt == doctest::Approx(1500.f));
    REQUIRE(parseCloudLayer(" FEW 800-1400 ", l));
    CHECK(l.topAglFt == doctest::Approx(1400.f));
    for (const char* bad : {"XXX 1000", "OVC", "OVC 3000 2000", "BKN15", "OVC abc", ""})
        CHECK_FALSE(parseCloudLayer(bad, l));
}

TEST_CASE("clouds: a METAR fixture gives FEW, BKN and OVC layers above the field") {
    const std::string text = readFixture("weather/XTST_bkn.txt");
    REQUIRE_FALSE(text.empty());
    Metar m;
    REQUIRE(parseMetar(text, m));
    const CloudDeck d = resolveClouds(CloudSetup{}, &m, CloudMode::Auto, 2500, 2000);
    CHECK(d.source == "metar XTST");
    REQUIRE(d.count == 3);
    CHECK(d.layers[0].cover == CloudCover::Few);
    CHECK(d.layers[0].baseFtMsl == doctest::Approx(2800.f));
    CHECK(d.layers[1].cover == CloudCover::Broken);
    CHECK(d.layers[1].baseFtMsl == doctest::Approx(3500.f));
    CHECK(d.layers[0].topFtMsl <= d.layers[1].baseFtMsl);    // no overlap
    CHECK(d.layers[1].topFtMsl == doctest::Approx(5300.f));  // 1,800 ft default BKN thickness
    CHECK(d.layers[2].cover == CloudCover::Overcast);
    CHECK(d.layers[2].baseFtMsl == doctest::Approx(6000.f));
}

TEST_CASE("clouds: the scenario wins in AUTO, a manual setting wins over both, OFF is clear") {
    Metar m;
    REQUIRE(parseMetar("XTST 051756Z 28008KT 9999 BKN015 12/08 A2992", m));
    const CloudSetup sc = scenarioLayers({"OVC 1500 3500"});
    CloudDeck d = resolveClouds(sc, &m, CloudMode::Auto, 2500, 2000);
    CHECK(d.source == "scenario");
    REQUIRE(d.count == 1);
    CHECK(d.layers[0].cover == CloudCover::Overcast);
    CHECK(d.layers[0].baseFtMsl == doctest::Approx(3500.f));
    CHECK(d.layers[0].topFtMsl == doctest::Approx(5500.f));
    d = resolveClouds(sc, &m, CloudMode::Scattered, 4000, 2000);
    CHECK(d.source == "setting");
    REQUIRE(d.count == 1);
    CHECK(d.layers[0].cover == CloudCover::Scattered);
    CHECK(d.layers[0].baseFtMsl == doctest::Approx(6000.f));
    d = resolveClouds(sc, &m, CloudMode::Off, 2500, 2000);
    CHECK(d.count == 0);
    CloudSetup clear;
    clear.set = true;  // [clouds] clear = true
    CHECK(resolveClouds(clear, &m, CloudMode::Auto, 2500, 2000).count == 0);
    // VV (vertical visibility) becomes an overcast at that height; more than 3 layers are cut.
    REQUIRE(parseMetar("XTST 051756Z 00000KT 0400 FG VV002 08/08 A2992", m));
    d = resolveClouds(CloudSetup{}, &m, CloudMode::Auto, 2500, 0);
    REQUIRE(d.count == 1);
    CHECK(d.layers[0].cover == CloudCover::Overcast);
    CHECK(d.layers[0].baseFtMsl == doctest::Approx(200.f));
    d = resolveClouds(scenarioLayers({"FEW 500", "SCT 1500", "BKN 3000", "OVC 8000"}), nullptr,
                      CloudMode::Auto, 0, 0);
    CHECK(d.count == kMaxCloudLayers);
}

TEST_CASE("clouds: the noise map tiles and each cover hides about its share of the sky") {
    std::vector<std::uint8_t> map;
    buildCloudNoise(map);
    REQUIRE(map.size() == static_cast<size_t>(kCloudNoiseSize * kCloudNoiseSize));
    int lo = 255, hi = 0;
    for (std::uint8_t v : map) {
        lo = std::min<int>(lo, v);
        hi = std::max<int>(hi, v);
    }
    CHECK(lo == 0);
    CHECK(hi == 255);
    CHECK(sampleCloudNoise(map, 0.3, 0.7) == doctest::Approx(sampleCloudNoise(map, 1.3, -0.3)));
    for (CloudCover c :
         {CloudCover::Few, CloudCover::Scattered, CloudCover::Broken, CloudCover::Overcast}) {
        int covered = 0, n = 0;
        for (int j = 0; j < 80; ++j)
            for (int i = 0; i < 80; ++i, ++n)
                if (cloudAlphaAt(c, cloudNoiseAt(map, i * 431.0 + 17, j * 389.0 + 5)) > 0.5f)
                    ++covered;
        const float frac = static_cast<float>(covered) / static_cast<float>(n);
        INFO(cloudCoverName(c), " covers ", frac);
        CHECK(std::fabs(frac - cloudCoverageFraction(c)) < 0.08f);
    }
}

TEST_CASE("clouds: the plan draws bases from below, tops from above, fades inside") {
    CloudDeck d = resolveClouds(scenarioLayers({"BKN 1500 3000", "OVC 6000 8000"}), nullptr,
                                CloudMode::Auto, 0, 2000);
    // Below both: two bases, the farther (upper) one first.
    CloudPlan p = planClouds(d, 2500);
    REQUIRE(p.drawCount == 2);
    CHECK(p.insideLayer == -1);
    CHECK_FALSE(p.draws[0].top);
    CHECK(p.draws[0].layer == 1);
    CHECK(p.draws[1].layer == 0);
    // Between: the lower layer's top and the upper layer's base.
    p = planClouds(d, 6000);
    REQUIRE(p.drawCount == 2);
    CHECK(p.draws[0].layer == 1);  // base at 8,000 is 2,000 ft away, the top at 5,000 1,000 ft
    CHECK_FALSE(p.draws[0].top);
    CHECK(p.draws[1].top);
    // Inside the lower layer, 100 ft above its base: half way into the fade.
    p = planClouds(d, 3600);
    CHECK(p.insideLayer == 0);
    CHECK(p.insideDepth == doctest::Approx(0.5f));
    REQUIRE(p.drawCount == 1);
    CHECK(p.draws[0].layer == 1);
    // Above everything: tops only, and never more than three draws.
    p = planClouds(d, 20000);
    CHECK(p.drawCount == 2);
    CHECK(p.draws[0].top);
    CHECK(p.draws[1].top);
}

TEST_CASE("clouds: scenario [clouds] section and the settings keys") {
    Scenario s;
    std::string err;
    REQUIRE_MESSAGE(parseScenario("[scenario]\naircraft = c172p\n[clouds]\nlayer1 = OVC 1500 3500\n"
                                  "layer2 = SCT050\n",
                                  s, err),
                    err);
    CHECK(s.clouds.set);
    REQUIRE(s.clouds.layers.size() == 2);
    CHECK(s.clouds.layers[1].baseAglFt == doctest::Approx(5000.f));
    // last_flight.ini keeps the layers.
    Scenario back;
    REQUIRE_MESSAGE(parseScenario(scenarioToIni(s), back, err), err);
    REQUIRE(back.clouds.layers.size() == 2);
    CHECK(back.clouds.layers[0].topAglFt == doctest::Approx(3500.f));
    CHECK(back.clouds.layers[1].cover == CloudCover::Scattered);
    REQUIRE(parseScenario("[scenario]\naircraft = c172p\n[clouds]\nclear = true\n", s, err));
    CHECK(s.clouds.set);
    CHECK(s.clouds.layers.empty());
    CHECK_FALSE(parseScenario("[scenario]\naircraft = c172p\n[clouds]\nlayer1 = OVX 15\n", s, err));
    CHECK(err.find("[clouds]") != std::string::npos);

    Settings st;
    st.applyText("[settings]\nclouds = bkn\ncloud_base_ft = 99999\n");
    st.clampAll();
    CHECK(st.clouds == CloudMode::Broken);
    CHECK(st.cloudBaseFt == doctest::Approx(12000.f));
    CHECK(st.toKeyValueText().find("clouds = bkn") != std::string::npos);
}
