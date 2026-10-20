#pragma once
// Internal factory functions for the individual menu screens (WP H5). Only MenuSystem.cpp and the
// screens' own files include this.

#include <functional>
#include <memory>
#include <string>

#include "ui/MenuSystem.h"

namespace fsim {

enum class SearchMode { StartAirport, PlanDestination, DirectTo };
enum class RunwayMode { Start, QuickPlanDest, PlanDest };
enum class ApproachMode { Setup, Flight };

std::unique_ptr<Screen> makeMainScreen(MenuSystem& menu);
std::unique_ptr<Screen> makeSetupScreen();
std::unique_ptr<Screen> makeAircraftScreen(const MenuSystem& menu);
std::unique_ptr<Screen> makeSearchScreen(SearchMode mode, const MenuSystem& menu,
                                         const std::string& initialText = "");
std::unique_ptr<Screen> makeRunwayScreen(RunwayMode mode, const std::string& airportIdent,
                                         const MenuSystem& menu);
std::unique_ptr<Screen> makeStartTypeScreen(const MenuSystem& menu);
std::unique_ptr<Screen> makePlanScreen();
std::unique_ptr<Screen> makePlanFilesScreen(const MenuSystem& menu);
std::unique_ptr<Screen> makeApproachScreen(ApproachMode mode, const MenuSystem& menu);
std::unique_ptr<Screen> makeSettingsScreen();
std::unique_ptr<Screen> makePauseScreen();
std::unique_ptr<Screen> makeFlightPlanScreen(const MenuSystem& menu);
// `onYes` empty = information box with a single OK button.
std::unique_ptr<Screen> makeConfirmScreen(const std::string& message,
                                          std::function<void(MenuSystem&)> onYes);

// "5.2 NM" / "143 NM".
std::string formatDistNm(double nm);

}  // namespace fsim
