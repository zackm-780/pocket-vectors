// Flight plan setup, plan file list and approach selection screens (WP H5). The approach screen is
// used both before the flight (selections) and in flight (Navigator).

#include <algorithm>
#include <cctype>
#include <cstdio>

#include "nav/FlightPlan.h"
#include "ui/MenuScreens.h"

namespace fsim {

using namespace uilayout;

namespace {

std::string upperStr(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

const char* kFooterList = "UP DOWN MOVE   A SELECT   B BACK";

// ---- flight plan (setup) ---------------------------------------------------------------------------------

class PlanScreen : public Screen {
public:
    PlanScreen() : nav_(5, kListRows) {}
    ScreenId id() const override { return ScreenId::Plan; }

    void update(const MenuInput& in, MenuSystem& m) override {
        FlightSetup& s = m.setup();
        const char off = static_cast<char>(!s.hasPlan());
        nav_.setDisabled({0, 0, off, 0, off});
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (nav_.selected() == 2 && s.hasPlan() && (in.left || in.right))
            cycleRunway(m, in.right ? 1 : -1);
        if (!in.accept) return;
        switch (nav_.selected()) {
        case 0: m.push(makePlanFilesScreen(m)); break;
        case 1: m.push(makeSearchScreen(SearchMode::PlanDestination, m)); break;
        case 2: m.push(makeRunwayScreen(RunwayMode::PlanDest, s.destAirportIdent(), m)); break;
        case 3: m.push(makeApproachScreen(ApproachMode::Setup, m)); break;
        case 4: s.clearPlan(); break;
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "FLIGHT PLAN", "LOAD AN FMS FILE OR MAKE A DIRECT PLAN",
                        "A SELECT   B BACK   LEFT RIGHT DEST RUNWAY");
        const FlightSetup& s = m.setup();
        std::vector<ListRow> rows(5);
        rows[0] = {"PLAN FILE", s.planKind == PlanKind::File ? s.planName : "NONE"};
        rows[0].dimValue = s.planKind != PlanKind::File;
        rows[1] = {"QUICK PLAN FROM " + s.airportIdent + " " + s.runwayIdent + " TO",
                   s.planKind == PlanKind::Quick ? s.plan.destIdent : "CHOOSE"};
        rows[1].dimValue = s.planKind != PlanKind::Quick;
        rows[2] = {"DESTINATION RUNWAY",
                   s.hasPlan() ? s.plan.destIdent + " " + s.plan.arrRunway : "-"};
        rows[2].adjustable = s.hasPlan();
        rows[2].enabled = s.hasPlan();
        rows[3] = {"APPROACH", s.approachId.empty() ? "NONE" : s.approachId};
        rows[3].dimValue = s.approachId.empty();
        rows[4].label = "CLEAR PLAN";
        rows[4].enabled = s.hasPlan();
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, 30, nav_, rows);

        float y = 250;
        r.line(x, y - 14, x + kContentW, y - 14, t.dim.withAlpha(0.5f));
        if (!s.hasPlan()) {
            r.text(x + 10, y + 4, "NO FLIGHT PLAN: THE FLIGHT STARTS WITHOUT ONE", TextSize::S,
                   t.dim);
            return;
        }
        r.text(x + 10, y, ("PLAN  " + s.planName).c_str(), TextSize::S, t.text);
        // Route, wrapped to the content width.
        std::string route = "ROUTE  " + s.routeText();
        const size_t maxChars = 62;
        y += 16;
        while (!route.empty() && y < 330) {
            size_t cut = std::min(route.size(), maxChars);
            if (route.size() > maxChars) {
                const size_t sp = route.substr(0, maxChars).find_last_of(' ');
                if (sp != std::string::npos) cut = sp;
            }
            r.text(x + 10, y, route.substr(0, cut).c_str(), TextSize::S, t.dim);
            route.erase(0, std::min(route.size(), cut + 1));
            y += 14;
        }
        const AircraftEntry* ac = m.aircraftEntry(s.aircraftId);
        char b[96];
        const double dist = s.planDistNm();
        if (ac && ac->cruiseKt > 0) {
            const int minutes = static_cast<int>(eteS(dist, ac->cruiseKt) / 60.0 + 0.5);
            std::snprintf(b, sizeof b, "DISTANCE %.0f NM   ETE %d:%02d AT %.0f KT", dist,
                          minutes / 60, minutes % 60, ac->cruiseKt);
        } else {
            std::snprintf(b, sizeof b, "DISTANCE %.0f NM", dist);
        }
        r.text(x + 10, y + 6, b, TextSize::S, t.text);
        if (!s.approachId.empty())
            r.text(x + 10, y + 22,
                   ("APPROACH  " + s.approachId + " AT " + s.plan.destIdent).c_str(), TextSize::S,
                   t.accent);
    }

private:
    static void cycleRunway(MenuSystem& m, int dir) {
        FlightSetup& s = m.setup();
        const Airport* a = m.nav().findAirport(s.plan.destIdent);
        if (!a) return;
        const auto rwys = sortedRunways(*a);
        if (rwys.empty()) return;
        int idx = 0;
        for (int i = 0; i < static_cast<int>(rwys.size()); ++i)
            if (rwys[static_cast<size_t>(i)]->ident == s.plan.arrRunway) idx = i;
        const int n = static_cast<int>(rwys.size());
        idx = ((idx + dir) % n + n) % n;
        s.setPlanDestRunway(m.nav(), rwys[static_cast<size_t>(idx)]->ident);
    }
    ListNav nav_;
};

class PlanFilesScreen : public Screen {
public:
    explicit PlanFilesScreen(const MenuSystem& m) : files_(discoverPlanFiles(m.env().planDirs)) {
        nav_ = ListNav(static_cast<int>(files_.size()) + 1, kListRows);
        if (m.setup().planKind == PlanKind::File)
            for (size_t i = 0; i < files_.size(); ++i)
                if (files_[i].name == m.setup().planName) nav_.select(static_cast<int>(i) + 1);
    }
    ScreenId id() const override { return ScreenId::PlanFiles; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (!in.accept) return;
        if (nav_.selected() == 0) {
            m.setup().clearPlan();
            m.pop();
            return;
        }
        const PlanFileEntry& f = files_[static_cast<size_t>(nav_.selected()) - 1];
        std::string err;
        if (m.setup().loadPlanFile(m.nav(), f.path, f.name, err))
            m.pop();
        else
            m.push(makeConfirmScreen(upperStr(err), nullptr));
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        drawScreenFrame(r, t, "PLAN FILES", "X-PLANE FMS FROM data/plans AND assets/plans",
                        kFooterList);
        std::vector<ListRow> rows;
        rows.push_back({"NO FLIGHT PLAN", ""});
        for (const PlanFileEntry& f : files_) rows.push_back({upperStr(f.name), ""});
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, kRowH, nav_, rows);
        if (files_.empty()) r.text(x + 10, 130, "NO .FMS FILES FOUND", TextSize::M, t.warn);
    }

private:
    std::vector<PlanFileEntry> files_;
    ListNav nav_;
};

// ---- approach selection (setup and in flight) -----------------------------------------------------------------

class ApproachScreen : public Screen {
public:
    ApproachScreen(ApproachMode mode, const MenuSystem& m) : mode_(mode), nav_(1, 11) {
        std::string rwy;
        std::string current;
        if (mode_ == ApproachMode::Setup) {
            airport_ = m.setup().destAirportIdent();
            rwy = m.setup().destRunwayIdent();
            current = m.setup().approachId;
        } else {
            const FlightContext& f = m.flight();
            if (f.navigator && f.navigator->hasPlan() && !f.navigator->plan().destIdent.empty()) {
                airport_ = f.navigator->plan().destIdent;
                rwy = f.navigator->plan().arrRunway;
                current = f.navigator->plan().approachId;
            } else if (f.scenario) {
                airport_ = f.scenario->airportIdent;
                rwy = f.scenario->runwayIdent;
                if (f.navigator) current = f.navigator->plan().approachId;
            }
        }
        if (const Airport* a = m.nav().findAirport(airport_)) {
            for (const Runway* r : sortedRunways(*a)) runways_.push_back(r->ident);
            if (rwy.empty())
                if (const Runway* d = defaultRunway(*a)) rwy = d->ident;
        }
        for (size_t i = 0; i < runways_.size(); ++i)
            if (runways_[i] == rwy) rwyIdx_ = static_cast<int>(i);
        rebuild(m);
        for (size_t i = 0; i < choices_.size(); ++i)
            if (!current.empty() && choices_[i].id == current) nav_.select(static_cast<int>(i) + 1);
    }
    ScreenId id() const override { return ScreenId::Approach; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if ((in.left || in.right) && runways_.size() > 1) {
            const int n = static_cast<int>(runways_.size());
            rwyIdx_ = ((rwyIdx_ + (in.right ? 1 : -1)) % n + n) % n;
            rebuild(m);
        }
        if (in.back) {
            m.pop();
            return;
        }
        if (!in.accept) return;
        const int sel = nav_.selected();
        if (mode_ == ApproachMode::Setup)
            acceptSetup(m, sel);
        else
            acceptFlight(m, sel);
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "APPROACH", "SELECT THE PROCEDURE TO FLY",
                        "A SELECT   B BACK   LEFT RIGHT RUNWAY");
        const float x = r.centered(kContentX);
        const Airport* a = m.nav().findAirport(airport_);
        std::string name = a ? upperStr(a->name) : std::string();
        const std::string suffix = " AIRPORT";
        if (name.size() > suffix.size() &&
            name.compare(name.size() - suffix.size(), suffix.size(), suffix) == 0)
            name.resize(name.size() - suffix.size());
        r.text(x, 86, fitText("AIRPORT " + airport_ + "  " + name, 48).c_str(), TextSize::S,
               t.text);
        std::string rw = "RUNWAY ";
        rw += runways_.empty() ? "-" : "< " + runways_[static_cast<size_t>(rwyIdx_)] + " >";
        r.text(x + kContentW, 86, rw.c_str(), TextSize::M, t.accent, HAlign::Right);

        std::vector<ListRow> rows;
        rows.push_back({mode_ == ApproachMode::Setup ? "NO APPROACH" : "REMOVE APPROACH", ""});
        for (const ApproachChoice& c : choices_) {
            ListRow row;
            row.label = c.id;
            while (row.label.size() < 8) row.label += ' ';
            row.label += upperStr(c.title);
            if (c.minimumsFt > 0) {
                char b[32];
                std::snprintf(b, sizeof b, "%s %.0f FT", c.minimumsIsDa ? "DA" : "MDA",
                              c.minimumsFt);
                row.value = b;
            }
            rows.push_back(std::move(row));
        }
        drawList(r, t, x, 100, kContentW, kRowH, nav_, rows);
        if (choices_.empty())
            r.text(x + 10, 160, "NO APPROACHES FOR THIS RUNWAY", TextSize::M, t.warn);
    }

private:
    void rebuild(const MenuSystem& m) {
        choices_.clear();
        if (!runways_.empty())
            choices_ = approachChoices(m.nav(), airport_, runways_[static_cast<size_t>(rwyIdx_)],
                                       m.env().navDir, m.approachProvider());
        nav_ = ListNav(static_cast<int>(choices_.size()) + 1, 11);
        nav_.select(choices_.empty() ? 0 : 1);
    }

    void acceptSetup(MenuSystem& m, int sel) {
        FlightSetup& s = m.setup();
        if (sel == 0) {
            s.setApproach("");
            m.pop();
            return;
        }
        const std::string& rwy = runways_[static_cast<size_t>(rwyIdx_)];
        if (s.hasPlan()) {
            if (s.plan.arrRunway != rwy) s.setPlanDestRunway(m.nav(), rwy);
        } else {
            s.setRunway(rwy);
        }
        s.setApproach(choices_[static_cast<size_t>(sel) - 1].id);
        m.pop();
    }

    void acceptFlight(MenuSystem& m, int sel) {
        const FlightContext& f = m.flight();
        if (!f.navigator || !f.aircraft) return;
        MenuResult res;
        res.action = MenuAction::ResumeFlight;
        if (sel == 0) {
            f.navigator->removeApproach();
            res.message = "APPROACH REMOVED";
            m.post(std::move(res));
            return;
        }
        const ApproachChoice& c = choices_[static_cast<size_t>(sel) - 1];
        const std::string& rwy = runways_[static_cast<size_t>(rwyIdx_)];
        auto approach = findApproach(m.nav(), airport_, rwy, c.id, m.approachProvider());
        bool ok = false;
        if (approach) {
            if (f.navigator->hasPlan())
                ok = f.navigator->loadApproach(*approach);
            else
                ok = f.navigator->startApproachOnly(
                    *approach, LatLon{f.aircraft->latDeg, f.aircraft->lonDeg}, 0);
        }
        if (!ok) {
            m.push(makeConfirmScreen("CANNOT LOAD APPROACH " + c.id, nullptr));
            return;
        }
        res.message = "APPROACH " + c.id;
        m.post(std::move(res));
    }

    ApproachMode mode_;
    std::string airport_;
    std::vector<std::string> runways_;
    int rwyIdx_ = 0;
    std::vector<ApproachChoice> choices_;
    ListNav nav_;
};

}  // namespace

std::unique_ptr<Screen> makePlanScreen() {
    return std::make_unique<PlanScreen>();
}
std::unique_ptr<Screen> makePlanFilesScreen(const MenuSystem& menu) {
    return std::make_unique<PlanFilesScreen>(menu);
}
std::unique_ptr<Screen> makeApproachScreen(ApproachMode mode, const MenuSystem& menu) {
    return std::make_unique<ApproachScreen>(mode, menu);
}

}  // namespace fsim
