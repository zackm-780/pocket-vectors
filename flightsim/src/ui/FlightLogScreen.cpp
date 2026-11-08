// U2: the FLIGHT LOG screen (main menu, after NEW FLIGHT). A summary line, then the flights newest
// first; A on a row opens its details, B goes back. The newest 200 rows of conf/flightlog.csv are
// read when the screen opens; nothing is read in flight.
#include <cctype>
#include <cstdio>
#include <string>
#include <vector>

#include "app/FlightLog.h"
#include "ui/MenuScreens.h"
#include "ui/UiPainter.h"

namespace fsim {
namespace {

using namespace uilayout;

std::string upperCopy(std::string s) {
    for (char& c : s) c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

class FlightLogScreen : public Screen {
public:
    FlightLogScreen(const MenuSystem& m, int detail) : nav_(0, kListRows - 1) {
        if (!m.env().flightLogPath.empty()) flights_ = readFlightLog(m.env().flightLogPath, 200);
        nav_.setCount(static_cast<int>(flights_.size()));
        for (const FlightLogEntry& e : flights_) {
            totalS_ += e.flightS;
            landings_ += e.landings;
        }
        if (detail >= 0 && detail < static_cast<int>(flights_.size())) {
            nav_.select(detail);
            detail_ = detail;
        }
    }
    ScreenId id() const override { return ScreenId::FlightLog; }

    void update(const MenuInput& in, MenuSystem& m) override {
        if (detail_ >= 0) {
            if (in.back || in.accept) detail_ = -1;
            return;
        }
        nav_.handle(in);
        if (in.back) {
            m.pop();
            return;
        }
        if (in.accept && !flights_.empty()) detail_ = nav_.selected();
    }

    void draw(Renderer2D& r, const UiTheme& t, const MenuSystem&) const override {
        if (detail_ >= 0 && detail_ < static_cast<int>(flights_.size())) {
            drawDetail(r, t, flights_[static_cast<size_t>(detail_)]);
            return;
        }
        char sub[96];
        std::snprintf(sub, sizeof sub, "%zu FLIGHT%s   %s HOURS   %d LANDING%s", flights_.size(),
                      flights_.size() == 1 ? "" : "S", formatHoursMinutes(totalS_).c_str(),
                      landings_, landings_ == 1 ? "" : "S");
        drawScreenFrame(r, t, "FLIGHT LOG", flights_.empty() ? "NO FLIGHTS YET" : sub,
                        "UP DOWN MOVE   A DETAILS   B BACK");
        const float x = r.centered(kContentX);
        if (flights_.empty()) {
            r.text(x + 10, kListTop + 30, "FLIGHTS ARE LOGGED WHEN THEY END: A NEW FLIGHT,",
                   TextSize::S, t.dim);
            r.text(x + 10, kListTop + 48, "A CRASH OR QUIT. THE LOG IS conf/flightlog.csv.",
                   TextSize::S, t.dim);
            return;
        }
        std::vector<ListRow> rows;
        rows.reserve(flights_.size());
        for (const FlightLogEntry& e : flights_) {
            // The outcome goes in the value column (right aligned), the rest in the label.
            std::string line = flightLogListLine(e);
            const size_t cut = line.rfind("  " + e.outcome);
            if (cut != std::string::npos) line.resize(cut);
            ListRow row{line, e.outcome};
            row.dimValue = e.outcome != "LANDED";
            rows.push_back(std::move(row));
        }
        drawList(r, t, x, kListTop, kContentW, kRowH, nav_, rows);
    }

private:
    static void drawDetail(Renderer2D& r, const UiTheme& t, const FlightLogEntry& e) {
        drawScreenFrame(r, t, "FLIGHT", flightLogListLine(e), "A B BACK");
        const float x = r.centered(kContentX);
        char b[96];
        std::vector<std::pair<const char*, std::string>> lines;
        lines.push_back({"DATE (UTC)", e.startUtc.size() >= 16 ? e.startUtc.substr(0, 10) + " " +
                                                                     e.startUtc.substr(11, 5)
                                                               : e.startUtc});
        lines.push_back({"AIRCRAFT", upperCopy(e.aircraftId)});
        lines.push_back({"FROM", e.depAirport + " " + e.depRunway});
        lines.push_back({"START", upperCopy(e.startKind)});
        lines.push_back({"PLAN", upperCopy(e.plan)});
        lines.push_back({"APPROACH", e.approach});
        lines.push_back(
            {"TO", e.arrAirport == "-" ? std::string("-") : e.arrAirport + " " + e.arrRunway});
        lines.push_back({"FLIGHT TIME", formatHoursMinutes(e.flightS)});
        lines.push_back({"AIRBORNE", formatHoursMinutes(e.airborneS)});
        std::snprintf(b, sizeof b, "%.1f NM", e.distNm);
        lines.push_back({"DISTANCE", b});
        std::snprintf(b, sizeof b, "%.0f FT MSL", e.maxAltFt);
        lines.push_back({"MAX ALTITUDE", b});
        lines.push_back({"LANDINGS", std::to_string(e.landings)});
        if (e.landings > 0)
            std::snprintf(b, sizeof b, "%.0f FPM", e.touchdownFpm);
        else
            std::snprintf(b, sizeof b, "-");
        lines.push_back({"LAST TOUCHDOWN", b});
        std::snprintf(b, sizeof b, "%.0f LB  (%.0f > %.0f)", e.fuelUsedLbs(), e.fuelStartLbs,
                      e.fuelEndLbs);
        lines.push_back({"FUEL USED", b});
        lines.push_back({"OUTCOME", e.outcome});
        float y = kListTop + 16;
        for (const auto& [label, value] : lines) {
            r.text(x + 10, y, label, TextSize::M, t.dim);
            r.text(x + 220, y, value.c_str(), TextSize::M,
                   std::string(label) == "OUTCOME" ? t.accent : t.text);
            y += 23;
        }
    }

    std::vector<FlightLogEntry> flights_;
    ListNav nav_;
    int detail_ = -1;
    double totalS_ = 0;
    int landings_ = 0;
};

}  // namespace

std::unique_ptr<Screen> makeFlightLogScreen(const MenuSystem& menu, int detailIndex) {
    return std::make_unique<FlightLogScreen>(menu, detailIndex);
}

}  // namespace fsim
