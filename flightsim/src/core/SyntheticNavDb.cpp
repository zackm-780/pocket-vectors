#include "core/SyntheticNavDb.h"

#include <cmath>
#include <cstring>

#include "core/Geo.h"
#include "core/Ini.h"

namespace fsim {
namespace {

bool equalsNoCase(std::string_view a, std::string_view b) {
    return toLower(a) == toLower(b);
}

// Builds one landing direction. headingTrueDeg is the landing heading; the threshold lies
// lengthFt/2 behind the airport reference point along the opposite heading.
Runway makeRunway(const char* ident, const char* oppositeIdent, double headingTrueDeg,
                  const char* ilsIdent, float ilsFreqMhz) {
    const double lengthM = ftToM(SyntheticNavDb::kRunwayLengthFt);
    const double headingRad = degToRad(headingTrueDeg);
    Runway r;
    r.airportIdent = SyntheticNavDb::kIdent;
    r.ident = ident;
    r.oppositeIdent = oppositeIdent;
    destinationPoint(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg, headingRad + kPi,
                     lengthM * 0.5, r.thrLatDeg, r.thrLonDeg);
    destinationPoint(SyntheticNavDb::kLatDeg, SyntheticNavDb::kLonDeg, headingRad, lengthM * 0.5,
                     r.endLatDeg, r.endLonDeg);
    r.startLatDeg = r.thrLatDeg;
    r.startLonDeg = r.thrLonDeg;
    r.thrElevFt = r.endElevFt = SyntheticNavDb::kElevFt;
    r.headingTrueDeg = static_cast<float>(headingTrueDeg);
    r.lengthFt = SyntheticNavDb::kRunwayLengthFt;
    r.widthFt = SyntheticNavDb::kRunwayWidthFt;
    r.hard = true;
    r.lighted = true;

    Ils ils;
    std::strncpy(ils.ident, ilsIdent, sizeof ils.ident - 1);
    ils.freqMhz = ilsFreqMhz;
    // Localizer 1,000 ft beyond the far end; glideslope 1,000 ft past the threshold, on centerline.
    destinationPoint(r.endLatDeg, r.endLonDeg, headingRad, ftToM(1000.0), ils.locLatDeg,
                     ils.locLonDeg);
    destinationPoint(r.thrLatDeg, r.thrLonDeg, headingRad, ftToM(1000.0), ils.gsLatDeg,
                     ils.gsLonDeg);
    ils.locCourseTrueDeg = static_cast<float>(headingTrueDeg);
    ils.gsElevFt = SyntheticNavDb::kElevFt;
    ils.synthesized = true;
    r.ils = ils;
    return r;
}

}  // namespace

SyntheticNavDb::SyntheticNavDb() {
    airport_.ident = kIdent;
    airport_.name = "Synthetic Test Airport";
    airport_.iso = "ZZ";
    airport_.latDeg = kLatDeg;
    airport_.lonDeg = kLonDeg;
    airport_.elevFt = kElevFt;
    airport_.type = AirportType::Large;
    airport_.runways.push_back(makeRunway("09", "27", 90.0, "IXTA", 110.30f));
    airport_.runways.push_back(makeRunway("27", "09", 270.0, "IXTB", 110.70f));
}

const Airport* SyntheticNavDb::findAirport(std::string_view ident) const {
    return equalsNoCase(ident, kIdent) ? &airport_ : nullptr;
}

std::vector<const Airport*> SyntheticNavDb::nearestAirports(double latDeg, double lonDeg,
                                                            int maxCount, float maxDistNm) const {
    const double distNm = mToNm(greatCircleDistM(latDeg, lonDeg, kLatDeg, kLonDeg));
    if (maxCount < 1 || distNm > maxDistNm) return {};
    return {&airport_};
}

std::vector<const Airport*> SyntheticNavDb::searchAirports(std::string_view text,
                                                           int maxCount) const {
    if (maxCount < 1) return {};
    const std::string needle = toLower(text);
    if (needle.empty() || toLower(airport_.ident).find(needle) != std::string::npos ||
        toLower(airport_.name).find(needle) != std::string::npos)
        return {&airport_};
    return {};
}

std::vector<const Airport*> SyntheticNavDb::airportsInBox(double south, double west, double north,
                                                          double east) const {
    if (kLatDeg >= south && kLatDeg <= north && kLonDeg >= west && kLonDeg <= east)
        return {&airport_};
    return {};
}

const Runway* findRunway(const Airport& airport, std::string_view runwayIdent) {
    for (const Runway& r : airport.runways)
        if (equalsNoCase(r.ident, runwayIdent)) return &r;
    return nullptr;
}

}  // namespace fsim
