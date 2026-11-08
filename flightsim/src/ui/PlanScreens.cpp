// Flight plan setup, plan file list and approach selection screens (WP H5). The approach screen is
// used both before the flight (selections) and in flight (Navigator).

#include <algorithm>
#include <cctype>
#include <cstdio>

#include <functional>
#include <utility>

#include "hud/StrokeFont.h"
#include "nav/FlightPlan.h"
#include "nav/Procedures.h"
#include "ui/MenuScreens.h"
#include "ui/UiPainter.h"

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
    static constexpr float kPlanRowH = 30.0f;

public:
    PlanScreen() : nav_(7, kListRows) {}
    ScreenId id() const override { return ScreenId::Plan; }

    void update(const MenuInput& in, MenuSystem& m) override {
        FlightSetup& s = m.setup();
        const char off = static_cast<char>(!s.hasPlan());
        nav_.setDisabled({0, 0, off, off, off, 0, off});
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (nav_.selected() == 4 && s.hasPlan() && (in.left || in.right))
            cycleRunway(m, in.right ? 1 : -1);
        if (!in.accept) return;
        switch (nav_.selected()) {
        case 0: m.push(makePlanFilesScreen(m)); break;
        case 1: m.push(makeSearchScreen(SearchMode::PlanDestination, m)); break;
        case 2: m.push(makeProcedureScreen(true, ApproachMode::Setup, m)); break;   // N8
        case 3: m.push(makeProcedureScreen(false, ApproachMode::Setup, m)); break;  // N8
        case 4: m.push(makeRunwayScreen(RunwayMode::PlanDest, s.destAirportIdent(), m)); break;
        case 5: m.push(makeApproachScreen(ApproachMode::Setup, m)); break;
        case 6: s.clearPlan(); break;
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "FLIGHT PLAN", "LOAD AN FMS FILE OR MAKE A DIRECT PLAN",
                        "A SELECT   B BACK   LEFT RIGHT DEST RUNWAY");
        const FlightSetup& s = m.setup();
        std::vector<ListRow> rows(7);
        rows[0] = {"PLAN FILE", s.planKind == PlanKind::File ? s.planName : "NONE"};
        rows[0].dimValue = s.planKind != PlanKind::File;
        rows[1] = {"QUICK PLAN FROM " + s.airportIdent + " " + s.runwayIdent + " TO",
                   s.planKind == PlanKind::Quick ? s.plan.destIdent : "CHOOSE"};
        rows[1].dimValue = s.planKind != PlanKind::Quick;
        rows[2] = {"DEPARTURE (SID)",
                   s.hasPlan() ? FlightSetup::procedureText(s.plan.sidId, s.plan.sidTransition)
                               : "-"};
        rows[2].enabled = s.hasPlan();
        rows[2].dimValue = s.plan.sidId.empty();
        rows[3] = {"ARRIVAL (STAR)",
                   s.hasPlan() ? FlightSetup::procedureText(s.plan.starId, s.plan.starTransition)
                               : "-"};
        rows[3].enabled = s.hasPlan();
        rows[3].dimValue = s.plan.starId.empty();
        rows[4] = {"DESTINATION RUNWAY",
                   s.hasPlan() ? s.plan.destIdent + " " + s.plan.arrRunway : "-"};
        rows[4].adjustable = s.hasPlan();
        rows[4].enabled = s.hasPlan();
        std::string app = s.approachId.empty() ? "NONE" : s.approachId;
        if (!s.approachId.empty())
            app += s.approachTransition.empty() ? " AUTO" : "." + s.approachTransition;
        rows[5] = {"APPROACH", app};
        rows[5].dimValue = s.approachId.empty();
        rows[6].label = "CLEAR PLAN";
        rows[6].enabled = s.hasPlan();
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, kPlanRowH, nav_, rows);

        // 1.2.3: the summary sits below the list, wherever the list ends (N8 added the SID and
        // STAR rows, and the fixed y of the divider and the summary put them over APPROACH and
        // CLEAR PLAN).
        const float listBottom = kListTop + static_cast<float>(rows.size()) * kPlanRowH;
        const float divY = listBottom + 4;
        r.line(x, divY, x + kContentW, divY, t.dim.withAlpha(0.5f));
        float y = divY + 18;
        if (!s.hasPlan()) {
            r.text(x + 10, y, "NO FLIGHT PLAN: THE FLIGHT STARTS WITHOUT ONE", TextSize::S, t.dim);
            return;
        }
        r.text(x + 10, y, ("PLAN  " + s.planName).c_str(), TextSize::S, t.text);
        // Route, wrapped to the content width, above the distance and approach lines.
        std::string route = "ROUTE  " + s.routeText();
        const size_t maxChars =
            static_cast<size_t>((kContentW - 20.0f) / StrokeFont::advancePx(TextSize::S));
        const float routeBottom = kFooterRuleY - 40;  // room for DISTANCE and APPROACH
        y += 16;
        while (!route.empty()) {
            const bool lastLine = y + 14 > routeBottom;  // no room below for another line
            size_t cut = std::min(route.size(), maxChars);
            if (route.size() > maxChars) {
                const size_t sp = route.substr(0, maxChars).find_last_of(' ');
                if (sp != std::string::npos) cut = sp;
            }
            std::string line = route.substr(0, cut);
            route.erase(0, std::min(route.size(), cut + 1));
            if (lastLine && !route.empty()) {
                line = fitText(line + " " + route, static_cast<int>(maxChars));
                route.clear();
            }
            r.text(x + 10, y, line.c_str(), TextSize::S, t.dim);
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

// ---- N8: generic pick list (procedures, transitions) ----------------------------------------------------------

class PickScreen : public Screen {
public:
    using OnPick = std::function<void(MenuSystem&, int)>;
    PickScreen(std::string title, std::string subtitle, std::vector<std::string> items,
               int selected, OnPick onPick)
        : title_(std::move(title)), subtitle_(std::move(subtitle)), items_(std::move(items)),
          onPick_(std::move(onPick)), nav_(static_cast<int>(items_.size()), kListRows) {
        if (selected >= 0 && selected < static_cast<int>(items_.size())) nav_.select(selected);
    }
    ScreenId id() const override { return ScreenId::Procedure; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (in.accept && !items_.empty()) onPick_(m, nav_.selected());
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        drawScreenFrame(r, t, title_.c_str(), subtitle_.c_str(), kFooterList);
        std::vector<ListRow> rows;
        for (const std::string& s : items_) rows.push_back({s, ""});
        drawList(r, t, r.centered(kContentX), kListTop, kContentW, kRowH, nav_, rows);
    }

private:
    std::string title_, subtitle_;
    std::vector<std::string> items_;
    OnPick onPick_;
    ListNav nav_;
};

// ---- N8: SID / STAR selection (setup and in flight) -------------------------------------------------------------

// Where the procedure goes: the setup's plan file, or the navigator's plan in flight.
struct ProcTarget {
    std::string airport, runway, currentId, currentTransition;
};

ProcTarget procTarget(bool sid, ApproachMode mode, const MenuSystem& m) {
    ProcTarget t;
    if (mode == ApproachMode::Setup) {
        const FlightSetup& s = m.setup();
        t.airport = sid ? s.airportIdent : s.plan.destIdent;
        t.runway = sid ? s.runwayIdent : s.plan.arrRunway;
        t.currentId = sid ? s.plan.sidId : s.plan.starId;
        t.currentTransition = sid ? s.plan.sidTransition : s.plan.starTransition;
    } else if (m.flight().navigator && m.flight().navigator->hasPlan()) {
        const FlightPlan& p = m.flight().navigator->plan();
        t.airport = sid ? p.originIdent : p.destIdent;
        t.runway = sid ? p.depRunway : p.arrRunway;
        t.currentId = sid ? p.sidId : p.starId;
        t.currentTransition = sid ? p.sidTransition : p.starTransition;
    }
    return t;
}

// Applies (or with an empty id removes) the procedure; posts the in-flight result.
void applyProcedure(MenuSystem& m, bool sid, ApproachMode mode, const std::string& id,
                    const std::string& transition) {
    const char* what = sid ? "DEPARTURE " : "ARRIVAL ";
    if (mode == ApproachMode::Setup) {
        if (sid)
            m.setup().setSid(id, transition);
        else
            m.setup().setStar(id, transition);
        return;
    }
    Navigator* nav = m.flight().navigator;
    if (!nav || !nav->hasPlan()) return;
    FlightPlan p = nav->plan();
    MenuResult res;
    res.action = MenuAction::ResumeFlight;
    if (id.empty()) {
        if (sid)
            removeSid(p);
        else
            removeStar(p);
        res.message = std::string(what) + "REMOVED";
    } else {
        const ProcTarget t = procTarget(sid, mode, m);
        const auto set = loadProcedureSet(m.env().navDir, t.airport);
        const TerminalProcedure* proc = nullptr;
        if (set) proc = sid ? findSid(*set, id) : findStar(*set, id);
        std::string error;
        bool ok = false;
        if (proc) {
            std::vector<Leg> legs = sid ? assembleSidLegs(*proc, t.runway, transition)
                                        : assembleStarLegs(*proc, transition, t.runway);
            ok = sid ? insertSid(p, id, transition, std::move(legs), &error)
                     : insertStar(p, id, transition, std::move(legs), &error);
        }
        if (!ok) {
            m.push(makeConfirmScreen("CANNOT LOAD " + id, nullptr));
            return;
        }
        res.message = what + FlightSetup::procedureText(id, transition);
    }
    nav->setPlan(std::move(p));
    m.post(std::move(res));
}

std::unique_ptr<Screen> makeTransitionPick(bool sid, ApproachMode mode,
                                           const TerminalProcedure& proc,
                                           const std::string& current) {
    std::vector<std::string> items{"NONE"};
    int selected = 0;
    for (const ProcRoute& r : proc.enrouteTransitions) {
        if (r.name == current) selected = static_cast<int>(items.size());
        items.push_back(r.name);
    }
    const std::string id = proc.id;
    std::vector<std::string> names = items;  // the callback's copy (items is moved below)
    return std::make_unique<PickScreen>(
        id + " TRANSITION", sid ? "WHERE THE DEPARTURE ENDS" : "WHERE THE ARRIVAL STARTS",
        std::move(items), selected, [sid, mode, id, names](MenuSystem& m, int i) {
            applyProcedure(m, sid, mode, id,
                           i == 0 ? std::string() : names[static_cast<size_t>(i)]);
            if (mode == ApproachMode::Setup) {
                m.pop();
                m.pop();
            }
        });
}

std::unique_ptr<Screen> procedureScreen(bool sid, ApproachMode mode, const MenuSystem& m) {
    const ProcTarget t = procTarget(sid, mode, m);
    const auto set = t.airport.empty() ? nullptr : loadProcedureSet(m.env().navDir, t.airport);
    std::vector<std::string> ids;
    if (set)
        for (const TerminalProcedure* p :
             sid ? sidsForRunway(*set, t.runway) : starsForRunway(*set, t.runway))
            ids.push_back(p->id);
    std::vector<std::string> items{"NONE"};
    int selected = 0;
    for (const std::string& id : ids) {
        if (id == t.currentId) selected = static_cast<int>(items.size());
        items.push_back(id);
    }
    const std::string title = sid ? "DEPARTURE (SID)" : "ARRIVAL (STAR)";
    const std::string sub =
        (ids.empty() ? "NO PROCEDURES FOR " : (sid ? "FROM " : "TO ")) + t.airport + " " + t.runway;
    return std::make_unique<PickScreen>(
        title, sub, std::move(items), selected,
        [sid, mode, set, ids, cur = t.currentTransition](MenuSystem& m, int i) {
            if (i == 0) {
                applyProcedure(m, sid, mode, "", "");
                if (mode == ApproachMode::Setup) m.pop();
                return;
            }
            const TerminalProcedure* proc = sid ? findSid(*set, ids[static_cast<size_t>(i) - 1])
                                                : findStar(*set, ids[static_cast<size_t>(i) - 1]);
            if (!proc) return;
            if (!proc->enrouteTransitions.empty()) {
                m.push(makeTransitionPick(sid, mode, *proc, cur));
                return;
            }
            applyProcedure(m, sid, mode, proc->id, "");
            if (mode == ApproachMode::Setup) m.pop();
        });
}

// Approach transition list: AUTO (nearest to the route), VECTORS, then the named transitions.
std::unique_ptr<Screen> makeApproachTransitionPick(const Approach& approach,
                                                   const std::string& current,
                                                   std::function<void(MenuSystem&, int)> onPick) {
    std::vector<std::string> items{"AUTO", "VECTORS"};
    int selected = current == "VECTORS" ? 1 : 0;
    for (size_t i = 1; i < approach.transitions.size(); ++i) {
        if (approach.transitions[i].name == current) selected = static_cast<int>(items.size());
        items.push_back(approach.transitions[i].name);
    }
    return std::make_unique<PickScreen>(approach.id + " TRANSITION",
                                        "AUTO PICKS THE ONE NEAREST TO THE ROUTE", std::move(items),
                                        selected, std::move(onPick));
}

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
        const std::string id = choices_[static_cast<size_t>(sel) - 1].id;
        s.setApproach(id);
        auto approach = findApproach(m.nav(), airport_, rwy, id, m.approachProvider());
        if (approach && approach->transitions.size() > 1) {  // N8: pick the transition next
            m.push(makeApproachTransitionPick(
                *approach, s.approachTransition, [names = *approach](MenuSystem& mm, int i) {
                    std::string name;  // AUTO
                    if (i == 1) name = "VECTORS";
                    if (i >= 2) name = names.transitions[static_cast<size_t>(i) - 1].name;
                    mm.setup().setApproachTransition(name);
                    mm.pop();
                    mm.pop();
                }));
            return;
        }
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
        if (approach && approach->transitions.size() > 1 && f.navigator->hasPlan()) {
            // N8: choose the transition (AUTO = nearest to the route, as before).
            m.push(makeApproachTransitionPick(
                *approach, f.navigator->plan().approachTransition,
                [a = *approach](MenuSystem& mm, int i) {
                    Navigator* nav = mm.flight().navigator;
                    if (!nav) return;
                    const int idx = i == 0 ? -1 : i - 1;  // VECTORS = transitions[0]
                    MenuResult r;
                    r.action = MenuAction::ResumeFlight;
                    if (!nav->loadApproach(a, idx)) {
                        mm.push(makeConfirmScreen("CANNOT LOAD APPROACH " + a.id, nullptr));
                        return;
                    }
                    const std::string& t = nav->plan().approachTransition;
                    r.message = "APPROACH " + a.id + (t.empty() ? "" : " VIA " + t);
                    mm.post(std::move(r));
                }));
            return;
        }
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
std::unique_ptr<Screen> makeProcedureScreen(bool sid, ApproachMode mode, const MenuSystem& menu) {
    return procedureScreen(sid, mode, menu);
}

}  // namespace fsim
