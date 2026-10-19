#pragma once
// Synthesized ILS geometry (NAVIGATION.md section 3.2) and helpers shared with the CSV loader.

#include "core/NavTypes.h"

namespace fsim {

// Distances and angles of the synthesized ILS (NAVIGATION section 3.2).
constexpr double kSynthLocBeyondEndFt = 1000.0;  // LOC antenna beyond the far runway end
constexpr double kLocFullScaleFt = 350.0;        // +-350 ft at the threshold = full scale
constexpr float kLocHalfWidthMinDeg = 1.5f;
constexpr float kLocHalfWidthMaxDeg = 3.0f;
constexpr float kDefaultGsAngleDeg = 3.0f;
constexpr float kDefaultTchFt = 50.0f;
constexpr float kSynthMinRunwayLengthFt = 3000.0f;

// Total localizer course width (2 x half-width) such that full-scale deflection is +-350 ft at
// the threshold, clamped to a 1.5..3.0 degree half-width. Used for synthesized ILS and for real
// ILS rows whose width is empty (FAA NASR has no width).
float locWidthDegForGeometry(double locLatDeg, double locLonDeg, double thrLatDeg,
                             double thrLonDeg);

// True when a runway qualifies for a synthesized ILS: hard surface, exact geometry (not
// `approx`), length >= 3,000 ft.
bool runwayQualifiesForSynthIls(const Runway& rwy);

// Builds the synthesized ILS of one runway direction (ident "I<runway>", freqMhz = 0 = untuned).
// The caller decides whether the runway qualifies.
Ils synthesizeIls(const Runway& rwy);

}  // namespace fsim
