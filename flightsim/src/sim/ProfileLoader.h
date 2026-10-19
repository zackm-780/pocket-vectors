#pragma once
// Loads assets/aircraft/<id>.ini into an AircraftProfile. WP-00 bootstrap: F1/F2 own the real
// profile format and will extend this (keys not understood here are ignored).

#include <string>

#include "core/AircraftProfile.h"

namespace fsim {

// Parse profile text/file. On failure returns false and fills `error`.
bool loadAircraftProfile(const std::string& path, AircraftProfile& out, std::string& error);
bool parseAircraftProfile(const std::string& iniText, AircraftProfile& out, std::string& error);

}  // namespace fsim
