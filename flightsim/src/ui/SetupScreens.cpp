// Main menu and the "New flight" screens (WP H5): hub, aircraft, airport search with the on-screen
// keyboard, runway, start type, flight plan, plan files and approach selection.

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

#include "core/Log.h"
#include "core/SyntheticNavDb.h"
#include "nav/FlightPlan.h"
#include "ui/MenuScreens.h"

namespace fsim {

using namespace uilayout;

std::string formatDistNm(double nm) {
    char b[32];
    if (nm < 0)
        return "";
    else if (nm < 10)
        std::snprintf(b, sizeof b, "%.1f NM", nm);
    else
        std::snprintf(b, sizeof b, "%.0f NM", nm);
    return b;
}

namespace {

std::string upperStr(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

const char* kFooterList = "UP DOWN MOVE   A SELECT   B BACK";

// ---- confirmation / message dialog ------------------------------------------------------------

class ConfirmScreen : public Screen {
public:
    ConfirmScreen(std::string message, std::function<void(MenuSystem&)> onYes)
        : message_(std::move(message)), onYes_(std::move(onYes)) {}
    ScreenId id() const override { return ScreenId::Confirm; }
    bool isOverlay() const override { return true; }

    void update(const MenuInput& in, MenuSystem& m) override {
        if (!onYes_) {  // information box
            if (in.accept || in.back || in.start) m.pop();
            return;
        }
        if (in.left || in.right) yes_ = !yes_;
        if (in.back || in.start) {
            m.pop();
        } else if (in.accept) {
            m.pop();
            if (yes_) onYes_(m);
        }
    }
    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        if (!onYes_)
            drawDialog(r, t, message_, {"OK"}, 0);
        else
            drawDialog(r, t, message_, {"YES", "NO"}, yes_ ? 0 : 1);
    }

private:
    std::string message_;
    std::function<void(MenuSystem&)> onYes_;
    bool yes_ = false;  // default answer is NO
};

// ---- main menu -----------------------------------------------------------------------------------

class MainScreen : public Screen {
public:
    explicit MainScreen(MenuSystem& m) : nav_(4, 4) {
        std::string err;
        haveLast_ = m.lastFlightAvailable() && loadScenario(m.env().lastFlightPath, last_, err);
        if (!haveLast_) nav_.setDisabled({1, 0, 0, 0});
    }
    ScreenId id() const override { return ScreenId::Main; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (!in.accept) return;
        switch (nav_.selected()) {
        case 0: {
            MenuResult res;
            res.action = MenuAction::StartFlight;
            res.scenario = last_;
            m.post(std::move(res));
            break;
        }
        case 1: m.push(makeSetupScreen()); break;
        case 2: m.push(makeSettingsScreen()); break;
        case 3:
            m.confirm("QUIT THE GAME?", [](MenuSystem& mm) {
                MenuResult res;
                res.action = MenuAction::Quit;
                mm.post(std::move(res));
            });
            break;
        }
    }
    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        drawBackdrop(r, 0.88f);
        const float cx = r.centerX();
        r.text(cx, 118, "FLIGHT SIM", TextSize::L, t.accent, HAlign::Center);
        r.text(cx, 140, "FIRST PERSON FLIGHT SIMULATOR", TextSize::S, t.dim, HAlign::Center);
        r.line(cx - 150, 152, cx + 150, 152, t.accent.withAlpha(0.6f));
        std::vector<ListRow> rows(4);
        rows[0].label = "RESUME LAST FLIGHT";
        rows[1].label = "NEW FLIGHT";
        rows[2].label = "SETTINGS";
        rows[3].label = "QUIT";
        rows[0].enabled = haveLast_;
        drawList(r, t, cx - 160, 172, 320, 30, nav_, rows);
        if (haveLast_ && nav_.selected() == 0)
            r.text(cx, 320, lastFlightSummary(last_).c_str(), TextSize::S, t.dim, HAlign::Center);
        r.text(cx, 462, "A SELECT    SELECT+START QUIT", TextSize::S, t.text, HAlign::Center);
    }

private:
    ListNav nav_;
    Scenario last_;
    bool haveLast_ = false;
};

// ---- new flight hub --------------------------------------------------------------------------------

class SetupScreen : public Screen {
public:
    SetupScreen() : nav_(7, kListRows) {}
    ScreenId id() const override { return ScreenId::Setup; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        FlightSetup& s = m.setup();
        if (in.back) {
            m.pop();
            return;
        }
        if (in.start) {
            m.startFlight();
            return;
        }
        if (nav_.selected() == 3 && (in.left || in.right)) {  // quick start type change
            const int n = static_cast<int>(s.startType) + (in.right ? 1 : -1);
            s.startType =
                static_cast<StartType>((n % kStartTypeCount + kStartTypeCount) % kStartTypeCount);
        }
        if (!in.accept) return;
        switch (nav_.selected()) {
        case 0: m.push(makeAircraftScreen(m)); break;
        case 1: m.push(makeSearchScreen(SearchMode::StartAirport, m)); break;
        case 2: m.push(makeRunwayScreen(RunwayMode::Start, s.airportIdent, m)); break;
        case 3: m.push(makeStartTypeScreen(m)); break;
        case 4: m.push(makePlanScreen()); break;
        case 5: m.push(makeApproachScreen(ApproachMode::Setup, m)); break;
        case 6: m.startFlight(); break;
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "NEW FLIGHT", "CHOOSE YOUR AIRCRAFT AND WHERE TO START",
                        "A SELECT   B BACK   LEFT RIGHT START TYPE   START FLY");
        const FlightSetup& s = m.setup();
        const Airport* a = m.nav().findAirport(s.airportIdent);
        std::vector<ListRow> rows(7);
        const AircraftEntry* ac = m.aircraftEntry(s.aircraftId);
        rows[0] = {"AIRCRAFT", ac ? ac->displayName : s.aircraftId};
        rows[1] = {"AIRPORT", a ? a->ident + "  " + a->name : s.airportIdent};
        rows[2].label = "RUNWAY";
        rows[2].value = s.runwayIdent.empty() ? "NONE" : s.runwayIdent;
        if (a)
            if (const Runway* rw = findRunway(*a, s.runwayIdent))
                rows[2].value = runwayDescription(*rw, m.nav().magVarDeg(a->latDeg, a->lonDeg));
        rows[3] = {"START", startLabel(s)};
        rows[3].adjustable = true;
        rows[4] = {"FLIGHT PLAN", s.hasPlan() ? s.planName : "NONE"};
        rows[4].dimValue = !s.hasPlan();
        rows[5].label = "APPROACH";
        rows[5].value = s.approachId.empty() ? "NONE" : s.approachId;
        rows[5].dimValue = s.approachId.empty();
        rows[6].label = ">> START FLIGHT";
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, 34, nav_, rows);
        if (s.hasPlan())
            r.text(x + 10, 360, ("ROUTE  " + s.routeText()).c_str(), TextSize::S, t.dim);
    }

private:
    static std::string startLabel(const FlightSetup& s) {
        std::string v = startTypeLabel(s.startType);
        if (s.startType == StartType::InAir) {
            char b[32];
            if (s.inAirAltFt > 0)
                std::snprintf(b, sizeof b, " AT %d FT", s.inAirAltFt);
            else
                std::snprintf(b, sizeof b, " AT CRUISE ALT");
            v += b;
        }
        return v;
    }
    ListNav nav_;
};

// ---- aircraft ----------------------------------------------------------------------------------------

class AircraftScreen : public Screen {
public:
    explicit AircraftScreen(const MenuSystem& m)
        : nav_(static_cast<int>(m.aircraft().size()), kListRows) {
        for (int i = 0; i < nav_.count(); ++i)
            if (m.aircraft()[static_cast<size_t>(i)].id == m.setup().aircraftId) nav_.select(i);
    }
    ScreenId id() const override { return ScreenId::Aircraft; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (in.back) m.pop();
        if (in.accept && !nav_.empty()) {
            m.setup().aircraftId = m.aircraft()[static_cast<size_t>(nav_.selected())].id;
            m.pop();
        }
    }
    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "AIRCRAFT", "FROM assets/aircraft", kFooterList);
        std::vector<ListRow> rows;
        for (const AircraftEntry& e : m.aircraft()) {
            ListRow row;
            row.label = upperStr(e.displayName);
            row.value = std::string(categoryLabel(e.category)) + " " +
                        std::to_string(e.engineCount) + (e.engineCount > 1 ? " ENG" : " ENG");
            rows.push_back(std::move(row));
        }
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, kRowH, nav_, rows);
        if (!nav_.empty()) {
            const AircraftEntry& e = m.aircraft()[static_cast<size_t>(nav_.selected())];
            char b[96];
            std::snprintf(b, sizeof b, "ID %s   CRUISE %.0f KT AT %.0f FT", upperStr(e.id).c_str(),
                          e.cruiseKt, e.cruiseAltFt);
            r.text(x + 10, 424, b, TextSize::S, t.dim);
        } else {
            r.text(x + 10, 100, "NO AIRCRAFT PROFILES FOUND", TextSize::M, t.warn);
        }
    }

private:
    ListNav nav_;
};

// ---- start type ------------------------------------------------------------------------------------------

class StartTypeScreen : public Screen {
public:
    explicit StartTypeScreen(const MenuSystem& m) : nav_(kStartTypeCount, kListRows) {
        nav_.select(static_cast<int>(m.setup().startType));
    }
    ScreenId id() const override { return ScreenId::StartType; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        FlightSetup& s = m.setup();
        if (nav_.selected() == static_cast<int>(StartType::InAir) && (in.left || in.right)) {
            float fieldFt = 0;
            if (const Airport* a = m.nav().findAirport(s.airportIdent)) fieldFt = a->elevFt;
            s.inAirAltFt = stepInAirAltitude(s.inAirAltFt, in.right ? 1 : -1, fieldFt);
        }
        if (in.back) m.pop();
        if (in.accept) {
            s.startType = static_cast<StartType>(nav_.selected());
            m.pop();
        }
    }
    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "START", "WHERE THE FLIGHT BEGINS",
                        "A SELECT   B BACK   LEFT RIGHT ALTITUDE");
        std::vector<ListRow> rows;
        for (int i = 0; i < kStartTypeCount; ++i) {
            ListRow row;
            row.label = startTypeLabel(static_cast<StartType>(i));
            if (i == static_cast<int>(StartType::InAir)) {
                char b[32];
                if (m.setup().inAirAltFt > 0)
                    std::snprintf(b, sizeof b, "%d FT", m.setup().inAirAltFt);
                else
                    std::snprintf(b, sizeof b, "CRUISE ALT");
                row.value = b;
                row.adjustable = true;
            }
            if (static_cast<int>(m.setup().startType) == i && row.value.empty())
                row.value = "CURRENT";
            rows.push_back(std::move(row));
        }
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, 30, nav_, rows);
        const char* hint = "";
        switch (static_cast<StartType>(nav_.selected())) {
        case StartType::Runway:
            hint = "ON THE THRESHOLD, ENGINES RUNNING, READY FOR TAKEOFF";
            break;
        case StartType::Final2Nm:
        case StartType::Final6Nm:
        case StartType::Final10Nm: hint = "ON THE 3 DEGREE PATH, GEAR DOWN, APPROACH FLAPS"; break;
        case StartType::InAir: hint = "OVER THE FIELD, LEFT RIGHT CHANGES THE ALTITUDE"; break;
        }
        r.text(x + 10, 250, hint, TextSize::S, t.dim);
    }

private:
    ListNav nav_;
};

// ---- airport search --------------------------------------------------------------------------------------

struct SearchHit {
    std::string ident, detail;
    double distNm = -1;
    Waypoint wp;
};

class SearchScreen : public Screen {
public:
    SearchScreen(SearchMode mode, const MenuSystem& m, const std::string& initial)
        : mode_(mode), nav_(0, 6) {
        kb_.typeText(initial);
        refresh(m);
    }
    ScreenId id() const override { return ScreenId::AirportSearch; }

    void update(const MenuInput& in, MenuSystem& m) override {
        if (focus_ == Focus::Keyboard) {
            if (in.down && kb_.cursorRow() == kb_.rowCount() - 1 && !hits_.empty()) {
                focus_ = Focus::Results;
                return;
            }
            switch (kb_.handle(in)) {
            case OnScreenKeyboard::Event::Changed: refresh(m); break;
            case OnScreenKeyboard::Event::Done:
                if (!hits_.empty()) focus_ = Focus::Results;
                break;
            case OnScreenKeyboard::Event::Cancel: m.pop(); break;
            default: break;
            }
        } else {
            if (in.up && nav_.selected() == 0) {
                focus_ = Focus::Keyboard;
                return;
            }
            nav_.handle(in);
            if (in.back) focus_ = Focus::Keyboard;
            if ((in.accept || in.start) && !hits_.empty())
                choose(hits_[static_cast<size_t>(nav_.selected())], m);
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        const char* title = mode_ == SearchMode::StartAirport      ? "AIRPORT"
                            : mode_ == SearchMode::PlanDestination ? "DESTINATION"
                                                                   : "DIRECT TO";
        const char* sub = mode_ == SearchMode::DirectTo ? "TYPE A WAYPOINT, NAVAID OR AIRPORT IDENT"
                                                        : "TYPE AN IDENT, IATA CODE OR NAME";
        drawScreenFrame(r, t, title, sub,
                        focus_ == Focus::Keyboard
                            ? "A TYPE  B DELETE  X SPACE  Y 123/ABC  START DONE"
                            : "UP DOWN MOVE   A SELECT   B KEYBOARD");
        const float x = r.centered(kContentX);
        drawTextField(r, t, x, 72, kContentW, "SEARCH", kb_.text(), focus_ == Focus::Keyboard);
        const float kx = x + (kContentW - OnScreenKeyboard::kColumns * 46.0f) * 0.5f;
        drawKeyboard(r, t, kx, 108, kb_, focus_ == Focus::Keyboard);

        char head[48];
        if (kb_.text().empty())
            std::snprintf(head, sizeof head, "NEAREST");
        else
            std::snprintf(head, sizeof head, "MATCHES (%d)", static_cast<int>(hits_.size()));
        r.text(x, 260, head, TextSize::S, t.dim);
        std::vector<ListRow> rows;
        for (const SearchHit& h : hits_) {
            ListRow row;
            row.label = h.ident;
            while (row.label.size() < 6) row.label += ' ';
            row.label += h.detail;
            row.value = formatDistNm(h.distNm);
            rows.push_back(std::move(row));
        }
        if (rows.empty()) {
            r.text(x + 10, 296, kb_.text().empty() ? "NO AIRPORTS IN RANGE" : "NO MATCHES",
                   TextSize::M, t.warn);
        }
        drawList(r, t, x, 268, kContentW, 24, nav_, rows, focus_ == Focus::Results);
        (void)m;
    }

private:
    enum class Focus { Keyboard, Results };

    void refresh(const MenuSystem& m) {
        hits_.clear();
        double lat = 0, lon = 0;
        m.referencePosition(lat, lon);
        const NavDatabase& db = m.nav();
        const std::string q = kb_.text();
        if (mode_ == SearchMode::DirectTo && !q.empty()) {
            for (const IdentMatch& im : resolveIdent(db, q, lat, lon)) {
                SearchHit h;
                h.ident = im.wp.ident;
                h.detail = upperStr(im.description);
                h.distNm = im.distNm;
                h.wp = im.wp;
                hits_.push_back(std::move(h));
                if (hits_.size() >= 12) break;
            }
        } else {
            const std::vector<AirportHit> found = q.empty()
                                                      ? nearestAirportHits(db, lat, lon, 12, 400.0f)
                                                      : searchAirportsOrdered(db, q, 12, lat, lon);
            for (const AirportHit& ah : found) {
                SearchHit h;
                h.ident = ah.airport->ident;
                h.detail = upperStr(ah.airport->name);
                h.distNm = ah.distNm;
                h.wp = Waypoint{ah.airport->ident, WaypointKind::Airport, ah.airport->latDeg,
                                ah.airport->lonDeg};
                hits_.push_back(std::move(h));
            }
        }
        nav_.setCount(static_cast<int>(hits_.size()));
        nav_.select(0);
        if (hits_.empty()) focus_ = Focus::Keyboard;
    }

    void choose(const SearchHit& h, MenuSystem& m) {
        switch (mode_) {
        case SearchMode::StartAirport:
            if (!m.setup().setAirport(m.nav(), h.ident)) {
                m.push(makeConfirmScreen(h.ident + " HAS NO RUNWAYS", nullptr));
                return;
            }
            m.replaceTop(makeRunwayScreen(RunwayMode::Start, h.ident, m));
            break;
        case SearchMode::PlanDestination:
            m.replaceTop(makeRunwayScreen(RunwayMode::QuickPlanDest, h.ident, m));
            break;
        case SearchMode::DirectTo: directTo(h, m); break;
        }
    }

    static void directTo(const SearchHit& h, MenuSystem& m) {
        const FlightContext& f = m.flight();
        if (!f.navigator || !f.aircraft) return;
        Navigator& nav = *f.navigator;
        const LatLon present{f.aircraft->latDeg, f.aircraft->lonDeg};
        bool ok = false;
        bool inPlan = false;
        for (size_t i = 1; i < nav.plan().legs.size(); ++i)
            if (nav.plan().legs[i].to.ident == h.ident) inPlan = true;
        if (inPlan) {
            ok = nav.directToIdent(h.ident);  // keeps the rest of the plan
        } else {
            FlightPlan plan = nav.plan();
            directToWaypoint(plan, h.wp, present);
            nav.setPlan(std::move(plan));
            ok = true;
        }
        MenuResult res;
        res.action = MenuAction::ResumeFlight;
        res.message = ok ? "DIRECT TO " + h.ident : "DIRECT TO FAILED";
        m.post(std::move(res));
    }

    SearchMode mode_;
    OnScreenKeyboard kb_;
    Focus focus_ = Focus::Keyboard;
    std::vector<SearchHit> hits_;
    ListNav nav_;
};

// ---- runway -----------------------------------------------------------------------------------------------

class RunwayScreen : public Screen {
public:
    RunwayScreen(RunwayMode mode, std::string airportIdent, const MenuSystem& m)
        : mode_(mode), airportIdent_(std::move(airportIdent)), nav_(0, kListRows) {
        init(m);
    }
    ScreenId id() const override { return ScreenId::Runway; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (!in.accept || runways_.empty()) return;
        const std::string rwy = runways_[static_cast<size_t>(nav_.selected())]->ident;
        FlightSetup& s = m.setup();
        switch (mode_) {
        case RunwayMode::Start:
            s.setRunway(rwy);
            m.pop();
            break;
        case RunwayMode::PlanDest:
            s.setPlanDestRunway(m.nav(), rwy);
            m.pop();
            break;
        case RunwayMode::QuickPlanDest: {
            const AircraftEntry* ac = m.aircraftEntry(s.aircraftId);
            std::string err;
            if (s.makeQuickPlan(m.nav(), airportIdent_, rwy, ac ? ac->cruiseAltFt : 0.0f, err))
                m.popTo(ScreenId::Plan);
            else
                m.push(makeConfirmScreen(upperStr(err), nullptr));
            break;
        }
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        const Airport* a = m.nav().findAirport(airportIdent_);
        char sub[96] = "";
        if (a)
            std::snprintf(sub, sizeof sub, "%s  %s  ELEV %.0f FT", a->ident.c_str(),
                          upperStr(a->name).c_str(), a->elevFt);
        drawScreenFrame(r, t, mode_ == RunwayMode::Start ? "RUNWAY" : "DESTINATION RUNWAY", sub,
                        kFooterList);
        std::vector<ListRow> rows;
        if (a) {
            const float mv = m.nav().magVarDeg(a->latDeg, a->lonDeg);
            for (const Runway* rw : sortedRunways(*a)) {
                ListRow row;
                row.label = runwayDescription(*rw, mv);
                rows.push_back(std::move(row));
            }
        }
        drawList(r, t, r.centered(kContentX), kListTop, kContentW, kRowH, nav_, rows);
        (void)m;
    }

private:
    void init(const MenuSystem& m) {
        const Airport* a = m.nav().findAirport(airportIdent_);
        if (!a) return;
        runways_ = sortedRunways(*a);
        nav_.setCount(static_cast<int>(runways_.size()));
        std::string wanted;
        if (mode_ == RunwayMode::Start && m.setup().airportIdent == airportIdent_)
            wanted = m.setup().runwayIdent;
        else if (mode_ == RunwayMode::PlanDest)
            wanted = m.setup().plan.arrRunway;
        else if (const Runway* d = defaultRunway(*a))
            wanted = d->ident;
        for (int i = 0; i < nav_.count(); ++i)
            if (runways_[static_cast<size_t>(i)]->ident == wanted) nav_.select(i);
    }

    RunwayMode mode_;
    std::string airportIdent_;
    std::vector<const Runway*> runways_;
    ListNav nav_;
};

}  // namespace

std::unique_ptr<Screen> makeMainScreen(MenuSystem& menu) {
    return std::make_unique<MainScreen>(menu);
}
std::unique_ptr<Screen> makeSetupScreen() {
    return std::make_unique<SetupScreen>();
}
std::unique_ptr<Screen> makeAircraftScreen(const MenuSystem& menu) {
    return std::make_unique<AircraftScreen>(menu);
}
std::unique_ptr<Screen> makeSearchScreen(SearchMode mode, const MenuSystem& menu,
                                         const std::string& initialText) {
    return std::make_unique<SearchScreen>(mode, menu, initialText);
}
std::unique_ptr<Screen> makeRunwayScreen(RunwayMode mode, const std::string& airportIdent,
                                         const MenuSystem& menu) {
    return std::make_unique<RunwayScreen>(mode, airportIdent, menu);
}
std::unique_ptr<Screen> makeStartTypeScreen(const MenuSystem& menu) {
    return std::make_unique<StartTypeScreen>(menu);
}
std::unique_ptr<Screen> makeConfirmScreen(const std::string& message,
                                          std::function<void(MenuSystem&)> onYes) {
    return std::make_unique<ConfirmScreen>(message, std::move(onYes));
}

}  // namespace fsim
