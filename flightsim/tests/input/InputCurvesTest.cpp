#include <doctest/doctest.h>

#include <cmath>

#include "input/InputCurves.h"

using namespace fsim::input;

TEST_CASE("input: sdl axis normalisation") {
    CHECK(normalizeSdlAxis(0) == doctest::Approx(0.0f));
    CHECK(normalizeSdlAxis(32767) == doctest::Approx(1.0f));
    CHECK(normalizeSdlAxis(-32768) == doctest::Approx(-1.0f));  // clamped
}

TEST_CASE("input: expo keeps endpoints and softens the centre") {
    CHECK(applyExpo(1.0f, 0.35f) == doctest::Approx(1.0f));
    CHECK(applyExpo(-1.0f, 0.35f) == doctest::Approx(-1.0f));
    CHECK(applyExpo(0.0f, 0.35f) == doctest::Approx(0.0f));
    CHECK(applyExpo(0.5f, 0.0f) == doctest::Approx(0.5f));
    // 0.65*0.5 + 0.35*0.125 = 0.36875
    CHECK(applyExpo(0.5f, 0.35f) == doctest::Approx(0.36875f));
    CHECK(applyExpo(-0.5f, 0.35f) == doctest::Approx(-0.36875f));
}

TEST_CASE("input: single axis deadzone is continuous") {
    CHECK(applyDeadzone(0.11f, 0.12f) == 0.0f);
    CHECK(applyDeadzone(-0.12f, 0.12f) == 0.0f);
    CHECK(applyDeadzone(0.121f, 0.12f) == doctest::Approx(0.0011f).epsilon(0.2));
    CHECK(applyDeadzone(1.0f, 0.12f) == doctest::Approx(1.0f));
    CHECK(applyDeadzone(-1.0f, 0.12f) == doctest::Approx(-1.0f));
    CHECK(applyDeadzone(0.56f, 0.12f) == doctest::Approx(0.5f));
}

TEST_CASE("input: radial deadzone on the left stick") {
    float x = 0.05f, y = 0.05f;  // radius 0.0707 < 0.08
    applyRadialStick(x, y, 0.08f, 0.35f);
    CHECK(x == 0.0f);
    CHECK(y == 0.0f);

    x = 0.07f;
    y = 0.07f;  // radius 0.099 > 0.08: alive although each axis alone is below the deadzone
    applyRadialStick(x, y, 0.08f, 0.35f);
    CHECK(x > 0.0f);
    CHECK(x == doctest::Approx(y));

    x = 1.0f;
    y = 0.0f;
    applyRadialStick(x, y, 0.08f, 0.35f);
    CHECK(x == doctest::Approx(1.0f));

    x = 0.0f;
    y = -1.0f;
    applyRadialStick(x, y, 0.08f, 0.35f);
    CHECK(y == doctest::Approx(-1.0f));

    x = 1.0f;
    y = 1.0f;  // corner exceeds the unit circle: clamped
    applyRadialStick(x, y, 0.08f, 0.35f);
    CHECK(std::sqrt(x * x + y * y) == doctest::Approx(1.0f).epsilon(0.01));
}

TEST_CASE("input: axis dominance filter") {
    float x = 0.2f, y = 0.9f;  // mostly vertical: no rudder
    applyDominance(x, y, 1.5f);
    CHECK(x == 0.0f);
    CHECK(y == doctest::Approx(0.9f));

    x = 0.9f;
    y = 0.2f;  // mostly horizontal: no throttle change
    applyDominance(x, y, 1.5f);
    CHECK(x == doctest::Approx(0.9f));
    CHECK(y == 0.0f);

    x = 0.6f;
    y = 0.7f;  // diagonal: both survive
    applyDominance(x, y, 1.5f);
    CHECK(x == doctest::Approx(0.6f));
    CHECK(y == doctest::Approx(0.7f));
}

TEST_CASE("input: repeat interval shrinks to a floor") {
    CHECK(repeatIntervalS(0, 0.15f, 0.05f, 0.85f) == doctest::Approx(0.15f));
    CHECK(repeatIntervalS(1, 0.15f, 0.05f, 0.85f) == doctest::Approx(0.1275f));
    CHECK(repeatIntervalS(50, 0.15f, 0.05f, 0.85f) == doctest::Approx(0.05f));
}

TEST_CASE("input: accelerated bug step") {
    CHECK(acceleratedStep(0, 1.0f, 10.0f) == doctest::Approx(1.0f));
    CHECK(acceleratedStep(5, 1.0f, 10.0f) == doctest::Approx(1.0f));
    CHECK(acceleratedStep(15, 1.0f, 10.0f) == doctest::Approx(5.5f));
    CHECK(acceleratedStep(25, 1.0f, 10.0f) == doctest::Approx(10.0f));
    CHECK(acceleratedStep(100, 1.0f, 10.0f) == doctest::Approx(10.0f));
}
