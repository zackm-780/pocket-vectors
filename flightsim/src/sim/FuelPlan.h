#pragma once
// F6: fuel load and planning, no JSBSim. The start load (setting / scenario `fuel`), the fuel an
// AUTO load carries for a trip, endurance and fuel at the destination from the present flow, and
// the LOW FUEL message thresholds.
#include <string>
#include <string_view>

namespace fsim {

struct FuelSetting {
    enum class Kind { Default, Full, Fraction, Auto, Lbs };
    Kind kind = Kind::Default;  // Default: the JSBSim model's own load (as before F6)
    float value = 0;            // Fraction: 0.05..1, Lbs: pounds
};
// "default", "full", "auto", "50%", "3000" (pounds). False for anything else.
bool parseFuelSetting(std::string_view text, FuelSetting& out);
std::string fuelSettingText(const FuelSetting& s);  // inverse of parseFuelSetting

constexpr float kFuelReserveMin = 45.0f;   // AUTO: reserve at cruise flow
constexpr float kFuelContingency = 0.10f;  // AUTO: + 10 % of the trip fuel (taxi, climb, winds)

// AUTO: trip at the cruise flow and speed, plus contingency and reserve, pounds. 0 when the trip
// or the cruise data is unknown.
float plannedFuelLbs(double tripNm, float cruiseKt, float cruiseFlowLbsHr,
                     float reserveMin = kFuelReserveMin);
// Minutes of flight left at `flowLbsHr` (a large number when the flow is ~0).
float enduranceMin(float fuelLbs, float flowLbsHr);
// Fuel left on arrival after `distNm` at `gsKt` and the present flow (may be negative).
float fuelAtDestinationLbs(float fuelLbs, float flowLbsHr, double distNm, float gsKt);
// "2:05" for minutes (h:mm), "-" for an unknown (>= 100 h) endurance.
void formatEndurance(char* out, int n, float minutes);

// LOW FUEL: fires once when the endurance first drops below 30 minutes, once more below 15.
class LowFuelMonitor {
public:
    // Returns 30 or 15 when that threshold was just crossed, else 0. Ignored on the ground.
    int update(float enduranceMinutes, bool airborne);
    void reset() { fired30_ = fired15_ = false; }

private:
    bool fired30_ = false, fired15_ = false;
};

}  // namespace fsim
