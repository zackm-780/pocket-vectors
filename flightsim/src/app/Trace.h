#pragma once
// CSV trace of AircraftState + NavState + AutopilotState (--trace). Column names carry units.
// Debug tool: it does file IO, so it is written at 10 Hz only and never enabled in normal play.

#include <string>

#include "core/AircraftState.h"
#include "core/AutopilotState.h"
#include "core/ControlInputs.h"
#include "core/Csv.h"
#include "core/NavState.h"

namespace fsim {

constexpr double kTraceIntervalS = 0.1;

class TraceWriter {
public:
    bool open(const std::string& path);
    bool isOpen() const { return csv_.isOpen(); }
    void write(const AircraftState& ac, const NavState& nav, const AutopilotState& ap,
               const ControlInputs& finalCtl);
    void close() { csv_.close(); }

private:
    CsvWriter csv_;
};

}  // namespace fsim
