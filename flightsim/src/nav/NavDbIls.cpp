#include "nav/NavDbIls.h"

#include <algorithm>
#include <cmath>
#include <cstring>

#include "core/Geo.h"
#include "core/Units.h"

namespace fsim {

float locWidthDegForGeometry(double locLatDeg, double locLonDeg, double thrLatDeg,
                             double thrLonDeg) {
    const double distM = greatCircleDistM(locLatDeg, locLonDeg, thrLatDeg, thrLonDeg);
    float halfDeg = kLocHalfWidthMaxDeg;
    if (distM > 1.0) {
        halfDeg = static_cast<float>(radToDeg(std::atan(ftToM(kLocFullScaleFt) / distM)));
    }
    halfDeg = std::clamp(halfDeg, kLocHalfWidthMinDeg, kLocHalfWidthMaxDeg);
    return 2.0f * halfDeg;
}

bool runwayQualifiesForSynthIls(const Runway& rwy) {
    return rwy.hard && !rwy.approxGeometry && rwy.lengthFt >= kSynthMinRunwayLengthFt;
}

Ils synthesizeIls(const Runway& rwy) {
    Ils ils;
    const std::string name = "I" + rwy.ident;
    std::strncpy(ils.ident, name.c_str(), sizeof ils.ident - 1);
    const double headingRad = degToRad(rwy.headingTrueDeg);
    destinationPoint(rwy.endLatDeg, rwy.endLonDeg, headingRad, ftToM(kSynthLocBeyondEndFt),
                     ils.locLatDeg, ils.locLonDeg);
    ils.locCourseTrueDeg = rwy.headingTrueDeg;
    ils.locWidthDeg =
        locWidthDegForGeometry(ils.locLatDeg, ils.locLonDeg, rwy.thrLatDeg, rwy.thrLonDeg);
    ils.gsAngleDeg = kDefaultGsAngleDeg;
    ils.tchFt = kDefaultTchFt;
    const double gsDistFt = ils.tchFt / std::tan(degToRad(ils.gsAngleDeg));
    destinationPoint(rwy.thrLatDeg, rwy.thrLonDeg, headingRad, ftToM(gsDistFt), ils.gsLatDeg,
                     ils.gsLonDeg);
    ils.gsElevFt = rwy.thrElevFt;
    ils.hasGs = true;
    ils.synthesized = true;
    return ils;
}

}  // namespace fsim
