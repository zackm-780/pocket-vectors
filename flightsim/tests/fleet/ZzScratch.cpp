// scratch, not committed
#include <doctest/doctest.h>

#include <cstdio>
#include <cstdlib>

#include "FleetPhases.h"
#include "FleetSpecsExt.h"

using namespace fsim;
using namespace fsim::test;
using namespace fsim::test::fleet;

TEST_CASE("zz scratch stall trace" * doctest::skip()) {
    const char* id = std::getenv("ZZ_ID") ? std::getenv("ZZ_ID") : "A320";
    const double startKt = std::getenv("ZZ_KT") ? std::atof(std::getenv("ZZ_KT")) : 220;
    const int det = std::getenv("ZZ_FLAP") ? std::atoi(std::getenv("ZZ_FLAP")) : 0;
    Flight f;
    REQUIRE(f.resolve(id, "cruise"));
    f.start.iasKt = startKt;
    f.start.altMslFt = 9000.0;
    f.start.flapDetent = det;
    f.start.gearDown = det > 0;
    REQUIRE(f.begin());
    setAllThrottles(f, 0.0);
    double pitchCmd = radToDeg(f.s().pitchRad);
    for (long i = 0; i < 400 * kSimHz && !f.s().crashed; ++i) {
        const AircraftState& s = f.s();
        pitchCmd += 0.15 / kSimHz;
        f.ctl.elevatorNorm = pitchHoldElevator(s, pitchCmd, 0.1, 0.1); if (std::getenv("ZZ_TRIM")) f.ctl.elevTrimNorm = static_cast<float>(f.ctl.elevTrimNorm + 0.3 * f.ctl.elevatorNorm / kSimHz);
        f.ctl.aileronNorm = bankHoldAileron(s);
        f.ctl.rudderNorm = headingHoldRudder(s, 270.0, 0.02, 0.02);
        f.step();
        if (i % (5 * kSimHz) == 0)
            std::printf("t %.0f ias %.1f alpha %.1f pitch %.1f cmd %.1f nz %.2f vs %.0f elev %.2f\n",
                        s.simTimeS, s.iasKt, s.alphaDeg, radToDeg(s.pitchRad), pitchCmd, s.nzG,
                        s.vsFpm, f.ctl.elevatorNorm);
    }
}
