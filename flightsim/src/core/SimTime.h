#pragma once
// Fixed simulation step (ARCHITECTURE section 3): the flight model always runs at 120 Hz,
// independent of the render frame rate.

namespace fsim {

constexpr int kSimHz = 120;
constexpr double kSimDtS = 1.0 / kSimHz;
// Wall-clock frame times longer than this are clamped (debugger pauses, window drags).
constexpr double kMaxFrameDtS = 0.1;

}  // namespace fsim
