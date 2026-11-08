#pragma once
// U2: flight log. A flight starts when NEW FLIGHT, RESTART or RESUME puts the aircraft in the world
// and ends at the next of: another flight starts, the game quits, the aircraft crashes. Only
// interactive runs log (the same rule as last_flight.ini: no headless, --frames or bench runs).
//
//   FlightRecorder  pure: accumulates one flight from AircraftState, once per frame (no IO, no
//                   allocation after begin()).
//   FlightLogEntry  one row of conf/flightlog.csv (appended when a flight ends, never per frame).
//   open flight     conf/flightlog_open.csv holds the current entry, rewritten every 60 s of flight
//                   and at each touchdown, deleted when the row is appended. Select + Start can
//                   kill the game without a clean exit; at the next start a file left behind is
//                   appended with the outcome GAME CLOSED.

#include <string>
#include <vector>

#include "core/AircraftState.h"

namespace fsim {

class NavDatabase;
struct Scenario;

struct FlightLogEntry {
    std::string startUtc;  // wall clock, "YYYY-MM-DDTHH:MM:SSZ"
    std::string aircraftId;
    std::string depAirport, depRunway, startKind;
    std::string plan = "-", approach = "-";
    std::string arrAirport = "-", arrRunway = "-";
    double flightS = 0, airborneS = 0;  // sim seconds, not paused
    double distNm = 0, maxAltFt = 0;
    int landings = 0;
    double touchdownFpm = 0;  // sink rate at the last touchdown (positive down), 0 = none
    double fuelStartLbs = 0, fuelEndLbs = 0;
    std::string outcome;  // LANDED, ON GROUND, CRASHED: <reason>, IN FLIGHT, GAME CLOSED
    double fuelUsedLbs() const { return fuelStartLbs - fuelEndLbs; }
};

// Why a flight ends (the outcome also depends on the last state).
enum class FlightEnd { NewFlight, Quit, Crash };

class FlightRecorder {
public:
    // Starts a flight. startUtc is the wall clock (formatUtcNow()).
    void begin(const Scenario& scenario, const std::string& aircraftId, const std::string& startUtc,
               const AircraftState& s);
    // Once per frame. Paused time counts for nothing.
    void update(const AircraftState& s, double dtS, bool paused);
    // Ends the flight: fills the outcome from the last state. The arrival is looked up separately
    // (fillArrival), at most once per flight.
    FlightLogEntry end(FlightEnd why);
    bool active() const { return active_; }
    const FlightLogEntry& entry() const { return entry_; }
    // True when the open-flight file is due (every 60 s of flight and at each touchdown); the
    // caller writes it and calls checkpointDone().
    bool checkpointDue() const { return active_ && checkpointDue_; }
    void checkpointDone() {
        checkpointDue_ = false;
        sinceCheckpointS_ = 0;
    }
    // Last touchdown position (for the arrival), valid when entry().landings > 0.
    double touchdownLatDeg() const { return tdLatDeg_; }
    double touchdownLonDeg() const { return tdLonDeg_; }

    static constexpr double kCheckpointS = 60.0;
    static constexpr double kBounceS = 3.0;  // a touchdown within 3 s of liftoff is one landing

private:
    bool active_ = false;
    FlightLogEntry entry_;
    AircraftState last_;
    bool haveLast_ = false;
    bool airborne_ = false;
    bool everAirborne_ = false;
    double airborneSinceS_ = 0;  // flight time at the last liftoff
    double lastLiftoffS_ = -1e9;
    double airSinkFpm_ = 0;  // sink rate while airborne, for the touchdown
    double tdLatDeg_ = 0, tdLonDeg_ = 0;
    double sinceCheckpointS_ = 0;
    bool checkpointDue_ = false;
};

// "YYYY-MM-DDTHH:MM:SSZ" for the current wall clock (UTC).
std::string formatUtcNow();
// Arrival: the runway nearest the last touchdown, within 2 NM of an airport ("-" otherwise).
void fillArrival(FlightLogEntry& e, double tdLatDeg, double tdLonDeg, const NavDatabase& nav);

// ---- conf/flightlog.csv ---------------------------------------------------------------------------
const char* flightLogHeader();
std::string flightLogRow(const FlightLogEntry& e);  // no newline
bool parseFlightLogRow(const std::string& line, FlightLogEntry& out);
// Appends one row (writes the header first when the file is new or empty).
bool appendFlightLog(const std::string& path, const FlightLogEntry& e);
// Up to maxRows newest rows, newest first. Bad lines are skipped; a missing file is empty.
std::vector<FlightLogEntry> readFlightLog(const std::string& path, size_t maxRows = 200);
// The open-flight file: one row, written to <path>.tmp and renamed.
bool writeOpenFlight(const std::string& path, const FlightLogEntry& e);
// At startup: an open-flight file left behind is appended with the outcome GAME CLOSED and
// deleted. Returns true when a flight was recovered.
bool recoverOpenFlight(const std::string& openPath, const std::string& logPath);

// ---- the FLIGHT LOG screen ----------------------------------------------------------------------
// "11-07 14:02  C172P  KPDX 28L > KOTH 04  1:12  LANDED"
std::string flightLogListLine(const FlightLogEntry& e);
// "H:MM" for seconds.
std::string formatHoursMinutes(double seconds);

}  // namespace fsim
