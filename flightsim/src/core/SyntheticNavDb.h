#pragma once
// A NavDatabase that contains only the synthetic test airport XTST (ARCHITECTURE section 9):
// 40.0000 N, 100.0000 W, 2,000 ft, runway 09/27, 10,000 x 150 ft, synthesized ILS on both ends.
// Always available so tests, screenshots and the no-data fallback have somewhere to fly.

#include "core/NavDatabase.h"

namespace fsim {

class SyntheticNavDb : public NavDatabase {
public:
    static constexpr const char* kIdent = "XTST";
    static constexpr double kLatDeg = 40.0;
    static constexpr double kLonDeg = -100.0;
    static constexpr float kElevFt = 2000.0f;
    static constexpr float kRunwayLengthFt = 10000.0f;
    static constexpr float kRunwayWidthFt = 150.0f;

    SyntheticNavDb();

    const Airport* findAirport(std::string_view ident) const override;
    std::vector<const Airport*> nearestAirports(double latDeg, double lonDeg, int maxCount,
                                                float maxDistNm) const override;
    std::vector<const Airport*> searchAirports(std::string_view text, int maxCount) const override;
    std::vector<const Navaid*> findNavaids(std::string_view) const override { return {}; }
    std::vector<Waypoint> findFixes(std::string_view) const override { return {}; }
    std::vector<const Airport*> airportsInBox(double south, double west, double north,
                                              double east) const override;
    float magVarDeg(double, double) const override { return 0.0f; }

    const Airport& airport() const { return airport_; }

private:
    Airport airport_;
};

// Finds a runway (landing direction, e.g. "27") of an airport; nullptr if absent.
const Runway* findRunway(const Airport& airport, std::string_view runwayIdent);

}  // namespace fsim
