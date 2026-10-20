#include "world/NightLights.h"

#include <algorithm>
#include <cmath>

#include "core/TerrainQuery.h"

namespace fsim::world {
namespace {
float smooth(float e0, float e1, float x) {
    const float t = std::min(1.0f, std::max(0.0f, (x - e0) / (e1 - e0)));
    return t * t * (3.0f - 2.0f * t);
}
}  // namespace

float nightFactor(float sunElevDeg) {
    return smooth(-2.0f, -6.0f, sunElevDeg);
}

float moonStrength(float sunElevDeg) {
    return nightFactor(sunElevDeg) * smooth(0.0f, 20.0f, -sunElevDeg);
}

uint8_t urbanMaskByte(int urbanCells, int totalCells) {
    if (totalCells <= 0 || urbanCells <= 0) return 0;
    const float f = std::min(1.0f, static_cast<float>(urbanCells) / static_cast<float>(totalCells));
    return static_cast<uint8_t>(std::lround(f * 127.0f));
}

bool isCityLightClass(uint8_t landClass) {
    return landClass == static_cast<uint8_t>(LandClass::Urban);
}

float twilightGlow(float sunElevDeg) {
    return smooth(-14.0f, -4.0f, sunElevDeg) * (1.0f - smooth(-3.0f, 3.0f, sunElevDeg));
}

}  // namespace fsim::world
