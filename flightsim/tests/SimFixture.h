#pragma once
// Shared helpers for headless flight tests. Tests run with the build directory as the working
// directory, which is also where jsbsim/ and assets/ are assembled (the asset root).

#include <cmath>
#include <string>

#include "app/Scenario.h"
#include "app/Simulation.h"
#include "core/FlatTerrain.h"
#include "core/Geo.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"

namespace fsim::test {

struct SimFixture {
    Paths paths = Paths::resolve({});
    SyntheticNavDb nav;
    FlatTerrain terrain{static_cast<float>(ftToM(SyntheticNavDb::kElevFt))};
    Simulation sim{paths, nav, terrain};
    std::string error;

    // Initializes from scenario text (ini syntax) or a scenario file under assets/scenarios/.
    bool initText(const std::string& iniText, const std::string& aircraftOverride = "") {
        Scenario s;
        if (!parseScenario(iniText, s, error)) return false;
        return sim.init(s, aircraftOverride, error);
    }
    bool initFile(const std::string& name, const std::string& aircraftOverride = "") {
        Scenario s;
        if (!loadScenario(paths.asset("assets/scenarios/" + name), s, error)) return false;
        return sim.init(s, aircraftOverride, error);
    }

    // Runs the fixed-step loop for simTimeS simulated seconds.
    void run(double simTimeS) {
        const long steps = static_cast<long>(simTimeS * kSimHz + 0.5);
        for (long i = 0; i < steps; ++i) sim.stepOnce();
    }
};

// Distance (ft) along the extended centerline of XTST runway 27 and the glidepath altitude there.
// pathAltFt is the altitude of the 3 degree path (50 ft over the threshold) above that point.
struct PathPoint {
    double distToThresholdNm;
    double pathAltFt;
};
inline PathPoint glidepathAt(const SyntheticNavDb& nav, const AircraftState& s) {
    const Runway* r27 = findRunway(*nav.findAirport("XTST"), "27");
    const double distM = greatCircleDistM(s.latDeg, s.lonDeg, r27->thrLatDeg, r27->thrLonDeg);
    return {mToNm(distM), r27->thrElevFt + 50.0 + mToFt(distM) * std::tan(degToRad(3.0))};
}

}  // namespace fsim::test
