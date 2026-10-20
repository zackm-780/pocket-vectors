// X4: night factor, city-light mask, minimum ambient and the Atmosphere night terms (no GL).
#include <doctest/doctest.h>

#include "core/TerrainQuery.h"
#include "world/Atmosphere.h"
#include "world/NightLights.h"

using namespace fsim;
using namespace fsim::world;

TEST_CASE("night factor follows sun elevation") {
    CHECK(nightFactor(30.0f) == doctest::Approx(0.0f));
    CHECK(nightFactor(0.0f) == doctest::Approx(0.0f));
    CHECK(nightFactor(-2.0f) == doctest::Approx(0.0f));
    CHECK(nightFactor(-4.0f) == doctest::Approx(0.5f));
    CHECK(nightFactor(-6.0f) == doctest::Approx(1.0f));
    CHECK(nightFactor(-40.0f) == doctest::Approx(1.0f));
    float prev = -1.0f;
    for (float e = 10.0f; e >= -10.0f; e -= 0.5f) {
        const float n = nightFactor(e);
        CHECK(n >= prev);  // monotonic as the sun sinks
        prev = n;
    }
}

TEST_CASE("moonlight needs night and a moon above the horizon") {
    CHECK(moonStrength(20.0f) == doctest::Approx(0.0f));
    CHECK(moonStrength(-3.0f) < 0.1f);
    CHECK(moonStrength(-30.0f) == doctest::Approx(1.0f));
    CHECK(twilightGlow(30.0f) == doctest::Approx(0.0f));
    CHECK(twilightGlow(-4.0f) > 0.9f);
    CHECK(twilightGlow(-30.0f) == doctest::Approx(0.0f));
}

TEST_CASE("city light mask comes from the urban land class only") {
    CHECK(isCityLightClass(static_cast<uint8_t>(LandClass::Urban)));
    CHECK_FALSE(isCityLightClass(static_cast<uint8_t>(LandClass::Tree)));
    CHECK_FALSE(isCityLightClass(static_cast<uint8_t>(LandClass::Water)));
    CHECK_FALSE(isCityLightClass(static_cast<uint8_t>(LandClass::Unknown)));
    CHECK(urbanMaskByte(0, 9) == 0);
    CHECK(urbanMaskByte(9, 9) == 127);
    CHECK(urbanMaskByte(3, 9) == 42);
    CHECK(urbanMaskByte(5, 0) == 0);
    CHECK(urbanMaskByte(20, 9) == 127);  // clamped
}

TEST_CASE("atmosphere keeps a minimum ambient at night and is unchanged by day") {
    Atmosphere atm;
    atm.setObserver(45.5, -122.6, 500.0);
    UtcTime t;
    t.year = 2026;
    t.month = 6;
    t.day = 22;
    t.hour = 20;  // afternoon at Portland
    atm.setUtc(t);
    CHECK(atm.light().nightFactor == doctest::Approx(0.0f));
    CHECK(atm.light().moonRgb[2] == doctest::Approx(0.0f));
    CHECK(atm.light().ambientRgb[0] > 0.3f);

    t.hour = 8;  // 01:00 local: deep night
    atm.setUtc(t);
    CHECK(atm.light().nightFactor == doctest::Approx(1.0f));
    for (int i = 0; i < 3; ++i) CHECK(atm.light().ambientRgb[i] >= kMinAmbientRgb[i]);
    CHECK(atm.light().diffuseRgb[0] == doctest::Approx(0.0f));
    CHECK(atm.light().moonRgb[2] > 0.0f);
    // Fog follows the dark sky.
    CHECK(atm.fog().colorRgb[2] < 0.1f);
}
