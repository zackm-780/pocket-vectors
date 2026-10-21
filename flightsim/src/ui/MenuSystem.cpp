#include "ui/MenuSystem.h"

#include <algorithm>
#include <cctype>
#include <cstdio>

#include "core/Log.h"
#include "core/Paths.h"
#include "core/SyntheticNavDb.h"
#include "ui/MenuScreens.h"

namespace fsim {

const char* screenName(ScreenId id) {
    switch (id) {
    case ScreenId::None: return "none";
    case ScreenId::Main: return "main";
    case ScreenId::Setup: return "setup";
    case ScreenId::Aircraft: return "aircraft";
    case ScreenId::AirportSearch: return "airport";
    case ScreenId::Runway: return "runway";
    case ScreenId::StartType: return "start";
    case ScreenId::Plan: return "plan";
    case ScreenId::PlanFiles: return "planfiles";
    case ScreenId::Approach: return "approach";
    case ScreenId::Settings: return "settings";
    case ScreenId::Pause: return "pause";
    case ScreenId::FlightPlan: return "flightplan";
    case ScreenId::Confirm: return "confirm";
    case ScreenId::Help: return "help";
    }
    return "?";
}

std::string lastFlightSummary(const Scenario& s) {
    std::string a = s.aircraftId;
    for (char& c : a) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    std::string start;
    if (s.startKind == "final") {
        char b[32];
        std::snprintf(b, sizeof b, "%g NM FINAL", s.distanceNm);
        start = b;
    } else if (s.startKind == "cruise") {
        start = "IN AIR";
    } else {
        start = "RUNWAY";
    }
    return a + "  " + s.airportIdent + " " + s.runwayIdent + "  " + start;
}

MenuSystem::MenuSystem(MenuEnv env) : env_(std::move(env)) {
    aircraft_ = discoverAircraft(env_.aircraftDir);
    if (!env_.navDir.empty()) provider_ = makeCifpApproachProvider(env_.navDir);
    // Default selections: last flight if there is one, else KPDX, else the synthetic field.
    FlightSetup& s = setup_;
    const std::string wanted = env_.settings ? env_.settings->lastAircraft : std::string();
    if (aircraftEntry(wanted))
        s.aircraftId = wanted;
    else if (aircraftEntry("c172p"))
        s.aircraftId = "c172p";
    else if (!aircraft_.empty())
        s.aircraftId = aircraft_.front().id;
    bool seeded = false;
    if (lastFlightAvailable()) {
        Scenario last;
        std::string err;
        if (loadScenario(env_.lastFlightPath, last, err)) {
            const Airport* a = env_.nav->findAirport(last.airportIdent);
            if (a && findRunway(*a, last.runwayIdent)) {
                seedSetupFrom(last);
                seeded = true;
            }
        }
    }
    if (!seeded && !s.setAirport(*env_.nav, "KPDX"))
        s.setAirport(*env_.nav, SyntheticNavDb::kIdent);
    if (!env_.hasRef)
        if (const Airport* a = env_.nav->findAirport(s.airportIdent)) {
            env_.refLatDeg = a->latDeg;
            env_.refLonDeg = a->lonDeg;
            env_.hasRef = true;
        }
}

MenuSystem::~MenuSystem() = default;

const AircraftEntry* MenuSystem::aircraftEntry(const std::string& id) const {
    for (const AircraftEntry& e : aircraft_)
        if (e.id == id) return &e;
    return nullptr;
}

bool MenuSystem::lastFlightAvailable() const {
    return !env_.lastFlightPath.empty() && fileExists(env_.lastFlightPath);
}

void MenuSystem::referencePosition(double& latDeg, double& lonDeg) const {
    if (hasFlight_ && flight_.aircraft && flight_.aircraft->valid && !fullScreen()) {
        latDeg = flight_.aircraft->latDeg;
        lonDeg = flight_.aircraft->lonDeg;
    } else if (env_.hasRef) {
        latDeg = env_.refLatDeg;
        lonDeg = env_.refLonDeg;
    } else if (const Airport* a = env_.nav->findAirport("KPDX")) {
        latDeg = a->latDeg;
        lonDeg = a->lonDeg;
    } else {
        latDeg = SyntheticNavDb::kLatDeg;
        lonDeg = SyntheticNavDb::kLonDeg;
    }
}

void MenuSystem::seedSetupFrom(const Scenario& s) {
    FlightSetup& f = setup_;
    if (aircraftEntry(s.aircraftId)) f.aircraftId = s.aircraftId;
    if (const Airport* a = env_.nav->findAirport(s.airportIdent)) {
        f.clearPlan();
        f.airportIdent = a->ident;
        f.runwayIdent = findRunway(*a, s.runwayIdent) ? s.runwayIdent : "";
        if (f.runwayIdent.empty())
            if (const Runway* r = defaultRunway(*a)) f.runwayIdent = r->ident;
    }
    if (s.startKind == "final")
        f.startType = s.distanceNm <= 3   ? StartType::Final2Nm
                      : s.distanceNm <= 8 ? StartType::Final6Nm
                                          : StartType::Final10Nm;
    else if (s.startKind == "cruise")
        f.startType = StartType::InAir;
    else
        f.startType = StartType::Runway;
    f.inAirAltFt = static_cast<int>(s.altitudeFt);
    if (!s.planPath.empty()) {
        std::string err;
        const std::string name = [&] {
            const size_t slash = s.planPath.find_last_of("/\\");
            return slash == std::string::npos ? s.planPath : s.planPath.substr(slash + 1);
        }();
        if (!f.loadPlanFile(*env_.nav, s.planPath, name, err)) LOG_WARN("menu: %s", err.c_str());
    }
    if (!s.approachId.empty()) f.setApproach(s.approachId);
}

// ---- stack ----------------------------------------------------------------------------------

void MenuSystem::push(std::unique_ptr<Screen> screen) {
    Command c;
    c.kind = Command::Kind::Push;
    c.screen = std::move(screen);
    commands_.push_back(std::move(c));
}
void MenuSystem::pop() {
    Command c;
    c.kind = Command::Kind::Pop;
    commands_.push_back(std::move(c));
}
void MenuSystem::popTo(ScreenId id) {
    Command c;
    c.kind = Command::Kind::PopTo;
    c.target = id;
    commands_.push_back(std::move(c));
}
void MenuSystem::replaceTop(std::unique_ptr<Screen> screen) {
    Command c;
    c.kind = Command::Kind::Replace;
    c.screen = std::move(screen);
    commands_.push_back(std::move(c));
}
void MenuSystem::clearStack() {
    Command c;
    c.kind = Command::Kind::Clear;
    commands_.push_back(std::move(c));
}

void MenuSystem::applyCommands() {
    std::vector<Command> cmds;
    cmds.swap(commands_);
    for (Command& c : cmds) {
        switch (c.kind) {
        case Command::Kind::Push: stack_.push_back(std::move(c.screen)); break;
        case Command::Kind::Pop:
            if (!stack_.empty()) stack_.pop_back();
            break;
        case Command::Kind::PopTo:
            while (!stack_.empty() && stack_.back()->id() != c.target) stack_.pop_back();
            break;
        case Command::Kind::Replace:
            if (!stack_.empty()) stack_.pop_back();
            stack_.push_back(std::move(c.screen));
            break;
        case Command::Kind::Clear: stack_.clear(); break;
        }
    }
}

void MenuSystem::post(MenuResult result) {
    const bool settingsChanged = pending_.settingsChanged || result.settingsChanged;
    pending_ = std::move(result);
    pending_.settingsChanged = settingsChanged;
}

void MenuSystem::confirm(const std::string& message, std::function<void(MenuSystem&)> onYes) {
    push(makeConfirmScreen(message, std::move(onYes)));
}

void MenuSystem::showError(const std::string& message) {
    // Applied right away when called between frames (the app reports a failed start).
    stack_.push_back(makeConfirmScreen(message, nullptr));
}

bool MenuSystem::startFlight() {
    const Airport* a = env_.nav->findAirport(setup_.airportIdent);
    if (!a || !findRunway(*a, setup_.runwayIdent)) {
        push(makeConfirmScreen("CHOOSE AN AIRPORT AND RUNWAY FIRST", nullptr));
        return false;
    }
    std::string planPath;
    if (setup_.hasPlan()) {
        std::string err;
        planPath = env_.menuPlanPath;
        if (planPath.empty() || !setup_.writePlanFile(planPath, err)) {
            push(makeConfirmScreen("CANNOT SAVE THE FLIGHT PLAN\n" + err, nullptr));
            return false;
        }
    }
    MenuResult r;
    r.action = MenuAction::StartFlight;
    r.scenario = setup_.buildScenario(planPath);
    post(std::move(r));
    return true;
}

// ---- opening --------------------------------------------------------------------------------

void MenuSystem::close() {
    stack_.clear();
    commands_.clear();
}

void MenuSystem::openMain() {
    close();
    stack_.push_back(makeMainScreen(*this));
}

void MenuSystem::openPause() {
    close();
    if (hasFlight_ && flight_.scenario) seedSetupFrom(*flight_.scenario);
    stack_.push_back(makePauseScreen());
}

MenuResult MenuSystem::update(const MenuInput& in) {
    pending_ = MenuResult{};
    if (!stack_.empty()) stack_.back()->update(in, *this);
    applyCommands();
    MenuResult out = std::move(pending_);
    pending_ = MenuResult{};
    return out;
}

void MenuSystem::draw(Renderer2D& r) const {
    if (stack_.empty()) return;
    size_t first = stack_.size() - 1;
    while (first > 0 && stack_[first]->isOverlay()) --first;
    const UiTheme theme =
        UiTheme::fromHudColor(env_.settings ? env_.settings->hudColor : HudColor::Green);
    for (size_t i = first; i < stack_.size(); ++i) stack_[i]->draw(r, theme, *this);
}

bool MenuSystem::openCanned(const std::string& spec, std::string& error) {
    std::string name = spec, arg;
    const size_t colon = spec.find(':');
    if (colon != std::string::npos) {
        name = spec.substr(0, colon);
        arg = spec.substr(colon + 1);
    }
    for (char& c : name) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    close();
    std::string err;
    auto planFile = [&](const std::string& wanted) -> bool {
        const std::string want = wanted.empty() ? "KPDX-KOTH" : wanted;
        for (const PlanFileEntry& e : discoverPlanFiles(env_.planDirs)) {
            std::string stem = e.name.substr(0, e.name.size() - 4);
            if (e.name == want || stem == want)
                return setup_.loadPlanFile(nav(), e.path, e.name, err);
        }
        err = "no plan file '" + want + "'";
        return false;
    };

    if (name == "main") {
        openMain();
    } else if (name == "setup") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSetupScreen());
    } else if (name == "aircraft") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSetupScreen());
        stack_.push_back(makeAircraftScreen(*this));
    } else if (name == "airport") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSetupScreen());
        stack_.push_back(makeSearchScreen(SearchMode::StartAirport, *this, arg));
    } else if (name == "runway") {
        if (!arg.empty() && !setup_.setAirport(nav(), arg)) {
            error = "unknown airport '" + arg + "'";
            return false;
        }
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSetupScreen());
        stack_.push_back(makeRunwayScreen(RunwayMode::Start, setup_.airportIdent, *this));
    } else if (name == "start") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSetupScreen());
        stack_.push_back(makeStartTypeScreen(*this));
    } else if (name == "plan" || name == "approach") {
        if (!planFile(arg)) {
            error = err;
            return false;
        }
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSetupScreen());
        stack_.push_back(makePlanScreen());
        if (name == "approach") stack_.push_back(makeApproachScreen(ApproachMode::Setup, *this));
    } else if (name == "settings") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeSettingsScreen());
    } else if (name == "pause" || name == "flightplan") {
        if (!hasFlight_) {
            error = "'" + name + "' needs a running flight";
            return false;
        }
        openPause();
        if (name == "flightplan") stack_.push_back(makeFlightPlanScreen(*this));
    } else if (name == "confirm") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeConfirmScreen("QUIT THE GAME?", [](MenuSystem& m) {
            MenuResult r;
            r.action = MenuAction::Quit;
            m.post(std::move(r));
        }));
    } else if (name == "help") {
        stack_.push_back(makeMainScreen(*this));
        stack_.push_back(makeHelpScreen(*this, arg));
    } else {
        error = "unknown menu screen '" + name +
                "' (main, setup, aircraft, airport[:TEXT], runway[:ICAO], start, plan[:FILE], "
                "approach[:FILE], settings, pause, flightplan, confirm, help[:PAGE])";
        return false;
    }
    return true;
}

}  // namespace fsim
