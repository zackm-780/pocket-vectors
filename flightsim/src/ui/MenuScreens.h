#pragma once
// Internal factory functions for the individual menu screens (WP H5). Only MenuSystem.cpp and the
// screens' own files include this.

#include <functional>
#include <memory>
#include <string>

#include "nav/PlanEdit.h"
#include "ui/MenuSystem.h"

namespace fsim {

// N11b: NewDestination is the in-flight NEW DESTINATION search (rule 5).
enum class SearchMode { StartAirport, PlanDestination, DirectTo, NewDestination };
enum class RunwayMode { Start, QuickPlanDest, PlanDest };
// N11b: Offer is the in-flight list offered after a Direct-To another airport or a new
// destination (makeApproachOfferScreen).
enum class ApproachMode { Setup, Flight, Offer };

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
// N11b: the approaches of the flight's (new) destination, offered after a Direct-To another airport
// or a NEW DESTINATION. The first row is NOT NOW; `pending` (the edit's messages) is posted when
// the pilot leaves with NOT NOW or B, and its message is the screen's subtitle.
std::unique_ptr<Screen> makeApproachOfferScreen(const MenuSystem& menu, MenuResult pending);
// N11b: the in-flight DEPARTURE RUNWAY list (on the ground): rule 6 through PlanEdit.
std::unique_ptr<Screen> makeDepartureRunwayScreen(const MenuSystem& menu);
// N11b: the KEEP ROUTE / CLEAR ROUTE prompt for a new destination, a two-row screen (CLEAR
// selected, B pops it and nothing changes). `onPick(menu, keepRoute)` decides where to go (replaceTop, popTo).
std::unique_ptr<Screen> makeRouteChoiceScreen(const std::string& airportIdent,
                                              std::function<void(MenuSystem&, bool)> onPick);
// N8: SIDs (sid = true) or STARs for the plan's departure / arrival runway, then the transition.
std::unique_ptr<Screen> makeProcedureScreen(bool sid, ApproachMode mode, const MenuSystem& menu);
std::unique_ptr<Screen> makeSettingsScreen();
// U2: detailIndex >= 0 opens that flight's details (0 = newest; --menu flightlog:N).
std::unique_ptr<Screen> makeFlightLogScreen(const MenuSystem& menu, int detailIndex = -1);
std::unique_ptr<Screen> makePauseScreen();
std::unique_ptr<Screen> makeFlightPlanScreen(const MenuSystem& menu);
// `onYes` empty = information box with a single OK button.
std::unique_ptr<Screen> makeConfirmScreen(const std::string& message,
                                          std::function<void(MenuSystem&)> onYes);
// `spec` is the text after "help:" (empty, "controls[:page]", "buttons", "howto[:id]", "about").
std::unique_ptr<Screen> makeHelpScreen(const MenuSystem& menu, const std::string& spec);

// "5.2 NM" / "143 NM".
std::string formatDistNm(double nm);

// ---- N11b: plan edits in flight (NAVIGATION 2.4) -------------------------------------------------

// Where the flight's aircraft is (PlanEditContext of the aircraft; an empty one without a flight).
PlanEditContext flightEditContext(const MenuSystem& menu);
double flightSimTimeS(const MenuSystem& menu);
// Applies `edit` to the flight's navigator (Navigator::applyEdit at the sim time, which keeps the
// plan before a destructive edit for the undo) and fills `res` with the HUD lines: the edit's
// message, and kUndoHint under it when it is destructive. False (res.message = the reason) when
// the edit failed or there is no flight.
bool applyFlightEdit(const MenuSystem& menu, const PlanEditResult& edit, MenuResult& res);
// Direct-To `target` (rules 1-4) from the top screen: the flight resumes with the rule's message,
// or after Direct-To another airport (rule 4) its approaches are offered (replaceTop).
void directToInFlight(MenuSystem& menu, const Waypoint& target);
// The new destination with the KEEP / CLEAR choice (rule 5), then the approach offer. Without a
// route to keep, CLEAR applies at once.
void newDestinationInFlight(MenuSystem& menu, const std::string& airportIdent);
// The undo row / button: restores the plan kept by the last destructive edit while the window is
// open ("ROUTE REPLACED UNDONE"), else "NOTHING TO UNDO".
std::string undoPlanEdit(Navigator& nav, double simTimeS);

}  // namespace fsim
