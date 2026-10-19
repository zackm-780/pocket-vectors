#pragma once
// Navigation database contract. Implemented by nav/ (N2); core/SyntheticNavDb is the always-present
// fallback that only knows airport XTST.

#include <string_view>
#include <vector>

#include "core/NavTypes.h"

namespace fsim {

class NavDatabase {
public:
    virtual ~NavDatabase() = default;
    virtual const Airport* findAirport(std::string_view ident) const = 0;
    virtual std::vector<const Airport*> nearestAirports(double latDeg, double lonDeg, int maxCount,
                                                        float maxDistNm) const = 0;
    virtual std::vector<const Airport*> searchAirports(std::string_view text,
                                                       int maxCount) const = 0;
    virtual std::vector<const Navaid*> findNavaids(std::string_view ident) const = 0;  // ambiguous
    virtual std::vector<Waypoint> findFixes(std::string_view ident) const = 0;         // ambiguous
    virtual std::vector<const Airport*> airportsInBox(double south, double west, double north,
                                                      double east) const = 0;
    virtual float magVarDeg(double latDeg, double lonDeg) const = 0;  // + = east
    // Navaids within radiusNm of a point, nearest first (additive; used by quick-plan "via navaids").
    virtual std::vector<const Navaid*> navaidsNear(double /*latDeg*/, double /*lonDeg*/,
                                                   float /*radiusNm*/) const {
        return {};
    }
};

}  // namespace fsim
