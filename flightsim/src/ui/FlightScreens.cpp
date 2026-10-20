// Settings, pause menu and in-flight flight plan screens (WP H5).

#include <algorithm>
#include <cmath>
#include <cstdio>

#include "core/Log.h"
#include "nav/FlightPlan.h"
#include "ui/MenuScreens.h"

namespace fsim {

using namespace uilayout;

namespace {

// ---- settings ------------------------------------------------------------------------------------------------

struct SettingDef {
    const char* label;
    bool numeric;  // A does not change numbers (Left/Right do)
    std::string (*show)(const Settings&);
    void (*change)(Settings&, int dir);
};

std::string fmt(const char* f, double v) {
    char b[32];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

template <class E> E cycleEnum(E v, int dir, int count) {
    return static_cast<E>(((static_cast<int>(v) + dir) % count + count) % count);
}

const SettingDef kSettingDefs[] = {
    {"HUD COLOR", false,
     [](const Settings& s) -> std::string {
         return s.hudColor == HudColor::Green   ? "GREEN"
                : s.hudColor == HudColor::White ? "WHITE"
                                                : "AMBER";
     },
     [](Settings& s, int d) { s.hudColor = cycleEnum(s.hudColor, d, 3); }},
    {"HUD DECLUTTER AT START", false,
     [](const Settings& s) -> std::string {
         return s.hudDeclutterDefault == HudDeclutter::Full  ? "FULL"
                : s.hudDeclutterDefault == HudDeclutter::Std ? "STD"
                                                             : "MIN";
     },
     [](Settings& s, int d) { s.hudDeclutterDefault = cycleEnum(s.hudDeclutterDefault, d, 3); }},
    {"PRESSURE UNITS", false,
     [](const Settings& s) -> std::string {
         return s.pressureUnits == PressureUnits::Hpa ? "HPA" : "INHG";
     },
     [](Settings& s, int d) { s.pressureUnits = cycleEnum(s.pressureUnits, d, 2); }},
    {"SHOW FPS", false, [](const Settings& s) -> std::string { return s.showFps ? "ON" : "OFF"; },
     [](Settings& s, int) { s.showFps = !s.showFps; }},
    {"FIELD OF VIEW", true, [](const Settings& s) { return fmt("%.0f DEG", s.fovVerticalDeg); },
     [](Settings& s, int d) { s.fovVerticalDeg += 5.0f * static_cast<float>(d); }},
    {"VOLUME", true, [](const Settings& s) { return fmt("%.0f%%", s.volumePct); },
     [](Settings& s, int d) { s.volumePct += 10 * d; }},
    {"THROTTLE MODE", false,
     [](const Settings& s) -> std::string {
         return s.throttleMode == ThrottleMode::Rate ? "RATE" : "ABSOLUTE";
     },
     [](Settings& s, int d) { s.throttleMode = cycleEnum(s.throttleMode, d, 2); }},
    {"AUTO RUDDER", false,
     [](const Settings& s) -> std::string { return s.assistAutoRudder ? "ON" : "OFF"; },
     [](Settings& s, int) { s.assistAutoRudder = !s.assistAutoRudder; }},
    {"AUTO TRIM", false,
     [](const Settings& s) -> std::string { return s.assistAutoTrim ? "ON" : "OFF"; },
     [](Settings& s, int) { s.assistAutoTrim = !s.assistAutoTrim; }},
    {"PITCH SENSITIVITY", true, [](const Settings& s) { return fmt("%.2f", s.pitchSens); },
     [](Settings& s, int d) { s.pitchSens += 0.25f * static_cast<float>(d); }},
    {"ROLL SENSITIVITY", true, [](const Settings& s) { return fmt("%.2f", s.rollSens); },
     [](Settings& s, int d) { s.rollSens += 0.25f * static_cast<float>(d); }},
    {"RUDDER SENSITIVITY", true, [](const Settings& s) { return fmt("%.2f", s.rudderSens); },
     [](Settings& s, int d) { s.rudderSens += 0.25f * static_cast<float>(d); }},
    {"PITCH EXPO", true, [](const Settings& s) { return fmt("%.1f", s.pitchExpo); },
     [](Settings& s, int d) { s.pitchExpo += 0.1f * static_cast<float>(d); }},
    {"ROLL EXPO", true, [](const Settings& s) { return fmt("%.1f", s.rollExpo); },
     [](Settings& s, int d) { s.rollExpo += 0.1f * static_cast<float>(d); }},
    {"RUDDER EXPO", true, [](const Settings& s) { return fmt("%.1f", s.rudderExpo); },
     [](Settings& s, int d) { s.rudderExpo += 0.1f * static_cast<float>(d); }},
    {"WEATHER FROM METAR", false,  // X1
     [](const Settings& s) -> std::string { return s.weatherMetar ? "ON" : "OFF"; },
     [](Settings& s, int) { s.weatherMetar = !s.weatherMetar; }},
    {"TURBULENCE", true, [](const Settings& s) { return fmt("%.1f", s.weatherTurbulence); },
     [](Settings& s, int d) { s.weatherTurbulence += 0.1f * static_cast<float>(d); }},
};
constexpr int kSettingCount = static_cast<int>(sizeof kSettingDefs / sizeof kSettingDefs[0]);

class SettingsScreen : public Screen {
public:
    SettingsScreen() : nav_(kSettingCount, kListRows) {}
    ScreenId id() const override { return ScreenId::Settings; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        Settings& s = m.settings();
        const SettingDef& def = kSettingDefs[nav_.selected()];
        int dir = 0;
        if (in.right) dir = 1;
        if (in.left) dir = -1;
        if (in.accept && !def.numeric) dir = 1;
        if (dir != 0) {
            def.change(s, dir);
            s.clampAll();
            changed_ = true;
        }
        if (in.back || in.start) {
            if (changed_) {
                if (!m.env().settingsPath.empty() && !s.save(m.env().settingsPath))
                    LOG_WARN("settings: cannot write %s", m.env().settingsPath.c_str());
                MenuResult res;
                res.settingsChanged = true;
                m.post(std::move(res));
            }
            m.pop();
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem& m) const override {
        drawScreenFrame(r, t, "SETTINGS", "SAVED WHEN YOU LEAVE THIS SCREEN",
                        "UP DOWN MOVE   LEFT RIGHT CHANGE   B SAVE AND BACK");
        std::vector<ListRow> rows;
        for (const SettingDef& d : kSettingDefs) {
            ListRow row;
            row.label = d.label;
            row.value = d.show(m.settings());
            row.adjustable = true;
            rows.push_back(std::move(row));
        }
        drawList(r, t, r.centered(kContentX), kListTop, kContentW, kRowH, nav_, rows);
    }

private:
    ListNav nav_;
    bool changed_ = false;
};

// ---- pause menu ---------------------------------------------------------------------------------------------------

class PauseScreen : public Screen {
public:
    PauseScreen() : nav_(7, 7) {}
    ScreenId id() const override { return ScreenId::Pause; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (in.back || in.start) {
            resume(m);
            return;
        }
        if (!in.accept) return;
        switch (nav_.selected()) {
        case 0: resume(m); break;
        case 1: m.push(makeFlightPlanScreen(m)); break;
        case 2:
            m.confirm("RESTART THIS FLIGHT?", [](MenuSystem& mm) {
                MenuResult res;
                res.action = MenuAction::RestartFlight;
                mm.post(std::move(res));
            });
            break;
        case 3: m.push(makeSetupScreen()); break;
        case 4: m.push(makeSettingsScreen()); break;
        case 5:
            m.confirm("LEAVE THIS FLIGHT AND GO\nTO THE MAIN MENU?", [](MenuSystem& mm) {
                mm.clearStack();
                mm.push(makeMainScreen(mm));
            });
            break;
        case 6:
            m.confirm("QUIT THE GAME?", [](MenuSystem& mm) {
                MenuResult res;
                res.action = MenuAction::Quit;
                mm.post(std::move(res));
            });
            break;
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        float px = 0, py = 0;
        drawPanelFrame(r, t, 320, 300, "PAUSED", px, py);
        std::vector<ListRow> rows(7);
        rows[0].label = "RESUME";
        rows[1].label = "FLIGHT PLAN";
        rows[2].label = "RESTART FLIGHT";
        rows[3].label = "NEW FLIGHT";
        rows[4].label = "SETTINGS";
        rows[5].label = "QUIT TO MENU";
        rows[6].label = "QUIT";
        drawList(r, t, px + 16, py + 48, 288, 30, nav_, rows);
        r.text(r.centerX(), py + 292 - 8, "A SELECT   START RESUME", TextSize::S, t.dim,
               HAlign::Center);
    }

private:
    static void resume(MenuSystem& m) {
        MenuResult res;
        res.action = MenuAction::ResumeFlight;
        m.post(std::move(res));
    }
    ListNav nav_;
};

// ---- flight plan in flight --------------------------------------------------------------------------------------

const char* legTypeName(LegType t) {
    static const char* const names[] = {"IF", "TF", "CF", "DF", "CA", "VA",
                                        "FA", "HM", "HA", "HF", "RF"};
    return names[static_cast<int>(t)];
}

std::string altText(const AltConstraint& a) {
    char b[32] = "";
    switch (a.kind) {
    case AltConstraintKind::None: break;
    case AltConstraintKind::At: std::snprintf(b, sizeof b, "%.0f", a.altFt); break;
    case AltConstraintKind::AtOrAbove: std::snprintf(b, sizeof b, "%.0f+", a.altFt); break;
    case AltConstraintKind::AtOrBelow: std::snprintf(b, sizeof b, "%.0f-", a.altFt); break;
    case AltConstraintKind::Between:
        std::snprintf(b, sizeof b, "%.0f-%.0f", a.alt2Ft, a.altFt);
        break;
    }
    return b;
}

class FlightPlanScreen : public Screen {
public:
    explicit FlightPlanScreen(const MenuSystem& m) : nav_(kActionRows, kListRows) {
        const FlightContext& f = m.flight();
        if (f.navigator && f.navigator->hasPlan()) {
            plan_ = f.navigator->plan();
            hasApproach_ = f.navigator->activeApproach() != nullptr;
            const NavDatabase& db = m.nav();
            legs_ = planSummary(plan_,
                                [&db](double lat, double lon) { return db.magVarDeg(lat, lon); });
            distNm_ = planDistNm(plan_);
        }
        nav_.setCount(kActionRows + static_cast<int>(legs_.size()));
        std::vector<char> disabled(static_cast<size_t>(nav_.count()), 0);
        disabled[2] = hasApproach_ ? 0 : 1;
        if (!legs_.empty()) disabled[kActionRows] = 1;  // leg 0 is the origin
        nav_.setDisabled(disabled);
        if (plan_.activeLeg >= 1 && plan_.activeLeg < static_cast<int>(legs_.size()))
            nav_.select(kActionRows + plan_.activeLeg);
    }
    ScreenId id() const override { return ScreenId::FlightPlan; }

    void update(const MenuInput& in, MenuSystem& m) override {
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (in.start) {
            MenuResult res;
            res.action = MenuAction::ResumeFlight;
            m.post(std::move(res));
            return;
        }
        if (!in.accept) return;
        const FlightContext& f = m.flight();
        const int sel = nav_.selected();
        if (sel == 0) {
            m.push(makeSearchScreen(SearchMode::DirectTo, m));
        } else if (sel == 1) {
            m.push(makeApproachScreen(ApproachMode::Flight, m));
        } else if (sel == 2) {
            MenuResult res;
            if (f.navigator && f.navigator->reenterApproach()) {
                res.action = MenuAction::ResumeFlight;
                res.message = "DIRECT TO APPROACH";
                m.post(std::move(res));
            } else {
                m.push(makeConfirmScreen("NO APPROACH TO RE-ENTER", nullptr));
            }
        } else if (f.navigator) {
            const int leg = sel - kActionRows;
            if (f.navigator->directToLeg(leg)) {
                MenuResult res;
                res.action = MenuAction::ResumeFlight;
                res.message = "DIRECT TO " + legs_[static_cast<size_t>(leg)].ident;
                m.post(std::move(res));
            }
        }
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        std::string sub = "NO FLIGHT PLAN";
        if (!legs_.empty()) {
            char b[96];
            std::snprintf(b, sizeof b, "%s > %s   %.0f NM%s%s", plan_.originIdent.c_str(),
                          plan_.destIdent.c_str(), distNm_,
                          plan_.approachId.empty() ? "" : "   APP ", plan_.approachId.c_str());
            sub = b;
        }
        drawScreenFrame(r, t, "FLIGHT PLAN", sub, "A SELECT   B BACK   START RESUME");
        std::vector<ListRow> rows(static_cast<size_t>(nav_.count()));
        rows[0].label = "DIRECT TO IDENT...";
        rows[1].label = "CHANGE APPROACH...";
        rows[2].label = "RE-ENTER APPROACH";
        rows[2].enabled = hasApproach_;
        for (size_t i = 0; i < legs_.size(); ++i) {
            const LegSummary& l = legs_[i];
            char b[96];
            char dtk[8] = "";
            if (l.dtkMagDeg) std::snprintf(dtk, sizeof dtk, "%03.0fM", *l.dtkMagDeg);
            char dist[16] = "";
            if (i > 0) std::snprintf(dist, sizeof dist, "%.1fNM", l.distNm);
            std::snprintf(b, sizeof b, "%c%2d %-7s %-2s %4s %7s",
                          static_cast<int>(i) == plan_.activeLeg ? '>' : ' ', l.index,
                          l.ident.c_str(), legTypeName(l.type), dtk, dist);
            ListRow& row = rows[kActionRows + i];
            row.label = b;
            row.value = altText(l.alt);
            row.enabled = i > 0;
            row.dimValue = l.missedApproach;
        }
        const float x = r.centered(kContentX);
        drawList(r, t, x, kListTop, kContentW, kRowH, nav_, rows);
        if (legs_.empty())
            r.text(x + 10, kListTop + 3 * kRowH + 20, "DIRECT TO CREATES A ONE-LEG PLAN",
                   TextSize::S, t.dim);
    }

private:
    static constexpr int kActionRows = 3;
    FlightPlan plan_;
    std::vector<LegSummary> legs_;
    double distNm_ = 0;
    bool hasApproach_ = false;
    ListNav nav_;
};

}  // namespace

std::unique_ptr<Screen> makeSettingsScreen() {
    return std::make_unique<SettingsScreen>();
}
std::unique_ptr<Screen> makePauseScreen() {
    return std::make_unique<PauseScreen>();
}
std::unique_ptr<Screen> makeFlightPlanScreen(const MenuSystem& menu) {
    return std::make_unique<FlightPlanScreen>(menu);
}

}  // namespace fsim
