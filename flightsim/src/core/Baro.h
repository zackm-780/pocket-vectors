#pragma once
// X1: barometric altimetry on the ISA lapse (additive core helper, no state).
//
// "True" altitude is JSBSim's geometric MSL altitude. An altimeter set to `settingHpa` reads the
// altitude at which an ISA atmosphere referenced to that setting has the local static pressure.
// The local static pressure is what the real sea-level pressure (QNH) puts at the true altitude.
// With the setting equal to QNH the indicated altitude equals the true altitude (temperature
// effects are ignored). A wrong setting shifts the reading by about 27 ft per hPa
// (about 910 ft per inHg near sea level).

#include <cmath>

#include "core/Units.h"

namespace fsim {

constexpr double kStdQnhHpa = 1013.25;
constexpr double kIsaBaroScaleFt = 145366.45;  // ISA pressure formula height scale
constexpr double kIsaBaroExponent = 0.190284;

// Static pressure at altFt in an ISA atmosphere whose sea-level pressure is seaLevelHpa.
inline double staticPressureHpa(double altFt, double seaLevelHpa) {
    return seaLevelHpa * std::pow(1.0 - altFt / kIsaBaroScaleFt, 1.0 / kIsaBaroExponent);
}
// Altitude at which that atmosphere has pressureHpa.
inline double pressureAltitudeFt(double pressureHpa, double seaLevelHpa) {
    return kIsaBaroScaleFt * (1.0 - std::pow(pressureHpa / seaLevelHpa, kIsaBaroExponent));
}
// What an altimeter set to settingHpa shows at trueAltFt when the real sea-level pressure is qnhHpa.
inline double indicatedAltFt(double trueAltFt, double qnhHpa, double settingHpa) {
    return pressureAltitudeFt(staticPressureHpa(trueAltFt, qnhHpa), settingHpa);
}

}  // namespace fsim
