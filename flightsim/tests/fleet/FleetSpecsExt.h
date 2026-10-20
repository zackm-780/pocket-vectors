#pragma once
// Per-aircraft numbers for the extended fleet flyability tests (WP F4). Same meaning as the core
// table in FleetSpecs.h; the phases and the test pilot are shared (FleetPhases.h, FleetPilot.h).

#include "FleetSpecs.h"

namespace fsim::test::fleet {

// clang-format off
inline const FleetSpec kFleetExt[] = {
    {"A320", 8.4, 8.4,
     162, 10, 0.5, 0.1, 0.05, 0.04, 155, 180, 8500,
     175, 225, 9, 0.2, 3, 15, 173, 190, 3, 600,
     0.06, 0.05, 0.001, 0.002, 0.01, 0.002, 3.0, -6, 8,
     55, 5.0, 1.0, 8, 4.5, 0.004, 30, 0.5, 1.5, 0.6, 50, 0, 2.5, 0.8, 3.0, 7000, 600},
    {"787-8", 13.8, 14.0,
     155, 8, 0.5, 0.1, 0.05, 0.04, 150, 175, 8000,
     185, 235, 9, 0.2, 3, 12, 179, 200, 3, 800,
     0.06, 0.05, 0.002, 0.003, 0.01, 0.002, 3.0, -6, 7,
     50, 5.5, 1.0, 7, 5.0, 0.004, 35, 0.5, 1.5, 0.6, 50, 0, 2.5, 0.8, 3.0, 9000, 600},
};
// clang-format on

}  // namespace fsim::test::fleet
