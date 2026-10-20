#pragma once
// X4: night lighting helpers. Pure functions (no GL), unit tested. The terrain shader uses the
// night factor and the urban mask to draw city-light speckle; Atmosphere uses the moonlight and
// twilight terms.

#include <cstdint>

namespace fsim::world {

// 0 in daylight, 1 in full night. Fades in as the sun drops below about -4 deg (0 at -2 deg,
// 1 at -6 deg), which is when city lights start to read against the sky.
float nightFactor(float sunElevDeg);

// Faint moonlight: the moon is modelled as a full moon opposite the sun, so its elevation is
// -sunElev. Returns a 0..1 strength (0 while the moon is below the horizon or in daylight).
float moonStrength(float sunElevDeg);

// Never-below ambient light (per channel) so ridgelines and sea stay faintly visible at night.
constexpr float kMinAmbientRgb[3] = {0.050f, 0.062f, 0.115f};

// Share of a vertex footprint that is built-up, as stored in the terrain vertex (0..127).
uint8_t urbanMaskByte(int urbanCells, int totalCells);
// True for the land class that emits city light (LandClass::Urban).
bool isCityLightClass(uint8_t landClass);

// Twilight horizon glow strength: 0 by day and in deep night, peaks near sun elevation -4 deg.
float twilightGlow(float sunElevDeg);

}  // namespace fsim::world
