// CPU benchmark: JSBSim c172p and 737 stepped at 120 Hz, plus a memory copy test.
#pragma once

#include "Context.h"

namespace probe {

// Sets the CPU fact. Needs no SDL at all, so it also runs under qemu-user for smoke tests.
void runCpuBenchmark(Report& report, const Options& options);

}  // namespace probe
