#pragma once
// Test harness for driving FlightModel directly (no Simulation, no autopilot stub), plus a few
// small pilot-in-the-loop helpers (heading hold, wings level, pitch hold) for scripted flights.

#include <algorithm>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <string>

#include "app/Scenario.h"
#include "core/Geo.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "sim/FlightModel.h"
#include "sim/ProfileLoader.h"

namespace fsim::test {

struct Flight {
    Paths paths = Paths::resolve({});
    SyntheticNavDb nav;
    FlightModel model{paths.asset("jsbsim")};
    AircraftProfile profile;
    StartCondition start;
    ControlInputs ctl;
    std::string error;

    bool loadProfile(const std::string& aircraft) {
        return loadAircraftProfile(paths.asset("assets/aircraft/" + aircraft + ".ini"), profile,
                                   error);
    }
    // extra: more scenario lines, e.g. "distance_nm = 0.5\n" (they land in [scenario]).
    bool resolve(const std::string& aircraft, const std::string& startKind,
                 const std::string& extra = "", const std::string& sections = "") {
        if (!loadProfile(aircraft)) return false;
        Scenario sc;
        const std::string text = "[scenario]\naircraft = " + aircraft + "\nstart = " + startKind +
                                 "\n" + extra + sections;
        return parseScenario(text, sc, error) && resolveStart(sc, nav, profile, start, error);
    }
    bool begin() {
        if (!model.start(profile, start)) {
            error = "flight model failed to start";
            return false;
        }
        ctl = model.startControls();
        return true;
    }
    bool begin(const std::string& aircraft, const std::string& startKind,
               const std::string& extra = "", const std::string& sections = "") {
        return resolve(aircraft, startKind, extra, sections) && begin();
    }

    const AircraftState& s() const { return model.state(); }
    void step() { model.step(ctl); }
    void run(double seconds) {
        const long n = static_cast<long>(seconds * kSimHz + 0.5);
        for (long i = 0; i < n; ++i) step();
    }
};

// ---- simple pilot models ---------------------------------------------------------------------

// Rudder (and nose-wheel steering) to hold a true heading, using yaw rate as damping.
inline float headingHoldRudder(const AircraftState& s, double headingTrueDeg, double kp = 0.05,
                               double kd = 0.04) {
    const double err = radToDeg(wrapPi(degToRad(headingTrueDeg) - s.headingTrueRad));
    return static_cast<float>(std::clamp(kp * err - kd * radToDeg(s.rRadps), -1.0, 1.0));
}

// Aileron to hold wings level (or a bank angle), using roll rate as damping.
inline float bankHoldAileron(const AircraftState& s, double bankDeg = 0, double kp = 0.05,
                             double kd = 0.02) {
    return static_cast<float>(
        std::clamp(kp * (bankDeg - radToDeg(s.rollRad)) - kd * radToDeg(s.pRadps), -1.0, 1.0));
}

// Elevator to hold a pitch attitude.
inline float pitchHoldElevator(const AircraftState& s, double pitchDeg, double kp = 0.04,
                               double kd = 0.03) {
    return static_cast<float>(
        std::clamp(kp * (pitchDeg - radToDeg(s.pitchRad)) - kd * radToDeg(s.qRadps), -1.0, 1.0));
}

// Resident memory of this process in bytes (from /proc/self/statm), 0 if unavailable.
inline size_t residentBytes() {
    FILE* f = std::fopen("/proc/self/statm", "r");
    if (!f) return 0;
    unsigned long sizePages = 0, residentPages = 0;
    const int n = std::fscanf(f, "%lu %lu", &sizePages, &residentPages);
    std::fclose(f);
    return n == 2 ? static_cast<size_t>(residentPages) * static_cast<size_t>(sysconf(_SC_PAGESIZE))
                  : 0;
}

}  // namespace fsim::test
