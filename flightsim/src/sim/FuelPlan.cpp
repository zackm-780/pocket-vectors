#include "sim/FuelPlan.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fsim {

bool parseFuelSetting(std::string_view text, FuelSetting& out) {
    std::string t;
    for (char c : text)
        if (!std::isspace(static_cast<unsigned char>(c)))
            t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    FuelSetting s;
    if (t.empty() || t == "default") {
        s.kind = FuelSetting::Kind::Default;
    } else if (t == "full") {
        s.kind = FuelSetting::Kind::Full;
    } else if (t == "auto") {
        s.kind = FuelSetting::Kind::Auto;
    } else {
        const bool pct = t.back() == '%';
        if (pct) t.pop_back();
        char* end = nullptr;
        const float v = std::strtof(t.c_str(), &end);
        if (end == t.c_str() || *end != 0 || !(v > 0)) return false;
        if (pct) {
            if (v > 100) return false;
            s.kind = FuelSetting::Kind::Fraction;
            s.value = std::max(0.05f, v / 100.0f);
        } else {
            s.kind = FuelSetting::Kind::Lbs;
            s.value = v;
        }
    }
    out = s;
    return true;
}

std::string fuelSettingText(const FuelSetting& s) {
    char b[24];
    switch (s.kind) {
    case FuelSetting::Kind::Default: return "default";
    case FuelSetting::Kind::Full: return "full";
    case FuelSetting::Kind::Auto: return "auto";
    case FuelSetting::Kind::Fraction:
        std::snprintf(b, sizeof b, "%.0f%%", s.value * 100.0f);
        return b;
    case FuelSetting::Kind::Lbs: std::snprintf(b, sizeof b, "%.0f", s.value); return b;
    }
    return "default";
}

float plannedFuelLbs(double tripNm, float cruiseKt, float cruiseFlowLbsHr, float reserveMin) {
    if (!(tripNm > 0) || !(cruiseKt > 0) || !(cruiseFlowLbsHr > 0)) return 0;
    const float tripLbs = static_cast<float>(tripNm / cruiseKt) * cruiseFlowLbsHr;
    return tripLbs * (1.0f + kFuelContingency) + cruiseFlowLbsHr * reserveMin / 60.0f;
}

float enduranceMin(float fuelLbs, float flowLbsHr) {
    if (!(flowLbsHr > 1.0f)) return 6000.0f;
    return std::max(0.0f, fuelLbs) / flowLbsHr * 60.0f;
}

float fuelAtDestinationLbs(float fuelLbs, float flowLbsHr, double distNm, float gsKt) {
    if (!(gsKt > 30.0f)) return fuelLbs;
    return fuelLbs - flowLbsHr * static_cast<float>(distNm / gsKt);
}

void formatEndurance(char* out, int n, float minutes) {
    if (!(minutes < 6000.0f)) {
        std::snprintf(out, static_cast<size_t>(n), "-");
        return;
    }
    const int m = static_cast<int>(std::lround(std::max(0.0f, minutes)));
    std::snprintf(out, static_cast<size_t>(n), "%d:%02d", m / 60, m % 60);
}

int LowFuelMonitor::update(float enduranceMinutes, bool airborne) {
    if (!airborne) return 0;
    if (!fired15_ && enduranceMinutes < 15.0f) {
        fired15_ = fired30_ = true;
        return 15;
    }
    if (!fired30_ && enduranceMinutes < 30.0f) {
        fired30_ = true;
        return 30;
    }
    return 0;
}

}  // namespace fsim
