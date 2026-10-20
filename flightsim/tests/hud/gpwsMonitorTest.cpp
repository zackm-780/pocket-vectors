// X2: GPWS-lite logic (sink-rate envelopes, terrain look-ahead timing, suppression, Mode 4,
// callout sequence). Synthetic terrain only; no GL, no data files.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <vector>

#include "core/SyntheticNavDb.h"
#include "core/Units.h"
#include "hud/GpwsMonitor.h"
#include "hud/HudLogic.h"

using namespace fsim;

namespace {

constexpr double kLat0 = 40.0, kLon0 = -100.0;
constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

double northM(double latDeg) {
    return (latDeg - kLat0) * kMPerDegLat;
}

// Terrain defined by elevation (m) as a function of meters north of the origin.
class FnTerrain : public TerrainQuery {
public:
    explicit FnTerrain(std::function<float(double)> f) : f_(std::move(f)) {}
    float elevationM(double latDeg, double) const override { return f_(northM(latDeg)); }
    LandClass landClass(double, double) const override { return LandClass::Grass; }
    void addFlattenArea(const FlattenArea&) override {}
    void clearFlattenAreas() override {}
    void prefetch(double, double, float) override {}
    bool isDetailReady(double, double) const override { return true; }

private:
    std::function<float(double)> f_;
};

// Aircraft flying due north with a commanded vertical speed, 10 Hz.
struct Fly {
    AircraftState s;
    AircraftProfile profile;
    GpwsMonitor mon;
    NavState nav;
    bool useNav = false;
    const TerrainQuery* terrain = nullptr;
    const NavDatabase* navDb = nullptr;
    std::vector<GpwsEvent> events;
    std::uint32_t mask = 0;
    double impactS = -1;

    Fly(const TerrainQuery& t, double northStartM, double altFt, float gsKt, float vsFpm)
        : terrain(&t) {
        s.valid = true;
        s.latDeg = kLat0 + northStartM / kMPerDegLat;
        s.lonDeg = kLon0;
        s.altMslFt = altFt;
        s.gsKt = s.iasKt = gsKt;
        s.vsFpm = vsFpm;
        s.trackTrueRad = 0;
        s.gearPosNorm = 1;
        profile.vappKt = 65;
        updateAgl();
    }
    void updateAgl() {
        s.terrainElevFt = static_cast<float>(mToFt(terrain->elevationM(s.latDeg, s.lonDeg)));
        s.altAglFt = static_cast<float>(s.altMslFt - s.terrainElevFt);
    }
    void step(double dtS = 0.1) {
        s.simTimeS += dtS;
        s.latDeg += ktToMps(s.gsKt) * dtS / kMPerDegLat;
        s.altMslFt += s.vsFpm / 60.0 * dtS;
        updateAgl();
        if (s.altAglFt <= 0 && impactS < 0) impactS = s.simTimeS;
        mon.update(s, profile, useNav ? &nav : nullptr, *terrain, navDb);
        mask = mon.warnings();
        for (GpwsEvent e; mon.consumeEvent(e);) events.push_back(e);
    }
    int count(GpwsEventKind k) const {
        int n = 0;
        for (const GpwsEvent& e : events) n += e.kind == k;
        return n;
    }
};

}  // namespace

TEST_CASE("gpws: envelopes are monotonic and warning is above caution") {
    float prevC = 0, prevW = 0;
    for (float ra = 30; ra <= 2450; ra += 10) {
        CHECK(sinkWarningFpm(ra) > sinkCautionFpm(ra));
        CHECK(sinkCautionFpm(ra) >= prevC);
        CHECK(sinkWarningFpm(ra) >= prevW);
        prevC = sinkCautionFpm(ra), prevW = sinkWarningFpm(ra);
    }
    CHECK(sinkCautionFpm(500) > 1000.0f);  // a normal 700 fpm final never alerts
    CHECK(lookaheadCautionFloorFt(250) > lookaheadCautionFloorFt(100));
    CHECK(lookaheadWarningFloorFt(100) < lookaheadCautionFloorFt(100));
}

TEST_CASE("gpws: sink rate caution, then PULL UP, by radio altitude") {
    FnTerrain flat([](double) { return 600.0f; });
    {  // normal 700 fpm descent at 1000 ft AGL: nothing
        Fly f(flat, 0, 600 * kFtPerM + 1000, 90, -700);
        for (int i = 0; i < 100; ++i) f.step();
        CHECK(f.mask == 0);
    }
    {  // 2,500 fpm at 1,000 ft: SINK RATE only
        Fly f(flat, 0, 600 * kFtPerM + 1000, 90, -2500);
        f.step();
        CHECK(f.mask == 0);  // persistence
        for (int i = 0; i < 15; ++i) f.step();
        CHECK(f.mask == warningBit(WarningId::SinkRate));
        CHECK(f.count(GpwsEventKind::SinkRate) == 1);
        CHECK(f.count(GpwsEventKind::PullUp) == 0);
    }
    {  // 3,500 fpm at 1,000 ft: PULL UP after SINK RATE
        Fly f(flat, 0, 600 * kFtPerM + 1000, 90, -3500);
        for (int i = 0; i < 16; ++i) f.step();
        CHECK(f.mask == warningBit(WarningId::PullUp));
        CHECK(f.events.front().kind ==
              GpwsEventKind::PullUp);  // warning persists 0.5 s, caution 1 s
        CHECK(f.count(GpwsEventKind::PullUp) == 1);
    }
    {  // above 2,450 ft RA mode 1 is off (a fast cruise descent)
        Fly f(flat, 0, 600 * kFtPerM + 3000, 200, -4500);
        for (int i = 0; i < 20; ++i) f.step();
        CHECK((f.mask & (warningBit(WarningId::SinkRate) | warningBit(WarningId::PullUp))) == 0);
    }
    {  // on the ground nothing, even with a bogus sink rate
        Fly f(flat, 0, 600 * kFtPerM + 10, 60, -3000);
        f.s.wowAny = true;
        for (int i = 0; i < 20; ++i)
            f.mon.update(f.s, f.profile, nullptr, flat), f.s.simTimeS += 0.1;
        CHECK(f.mon.warnings() == 0);
    }
    {  // clears after the hold-off, not instantly
        Fly f(flat, 0, 600 * kFtPerM + 1000, 90, -2500);
        for (int i = 0; i < 20; ++i) f.step();
        REQUIRE(f.mask != 0);
        f.s.vsFpm = -500;
        for (int i = 0; i < 10; ++i) f.step();  // 1 s
        CHECK(f.mask != 0);
        for (int i = 0; i < 30; ++i) f.step();  // 4 s total
        CHECK(f.mask == 0);
    }
}

TEST_CASE("gpws: terrain look-ahead warns TERRAIN, then PULL UP, before impact") {
    // Wall 3,000 m high starting 12 km north; aircraft level at 8,000 ft MSL (2,438 m), 100 kt.
    FnTerrain ridge([](double n) { return n > 12000 ? 3000.0f : 600.0f; });
    Fly f(ridge, 0, 8000, 100, 0);
    double tTerr = -1, tPull = -1;
    for (int i = 0; i < 4000 && f.impactS < 0; ++i) {
        f.step();
        if (tTerr < 0 && (f.mask & warningBit(WarningId::Terrain))) tTerr = f.s.simTimeS;
        if (tPull < 0 && (f.mask & warningBit(WarningId::PullUp))) tPull = f.s.simTimeS;
    }
    REQUIRE(f.impactS > 0);
    std::printf("[gpws] ridge: TERRAIN at %.1f s, PULL UP at %.1f s, impact %.1f s\n", tTerr, tPull,
                f.impactS);
    CHECK(tTerr > 0);
    CHECK(tPull > tTerr);
    CHECK(f.impactS - tTerr >= 30.0);  // TERRAIN well before impact
    CHECK(f.impactS - tPull >= 15.0);  // and PULL UP leaves time to react
    CHECK(f.impactS - tPull <= 25.0);
    CHECK(f.count(GpwsEventKind::Terrain) == 1);
    CHECK(f.count(GpwsEventKind::PullUp) == 1);
}

TEST_CASE("gpws: a climb that clears the ridge, and terrain behind or below, stay quiet") {
    FnTerrain ridge([](double n) { return n > 12000 ? 3000.0f : 600.0f; });
    {  // climbing at 1,500 fpm from 8,000 ft clears the 9,800 ft wall by the floor in time
        Fly f(ridge, 0, 8000, 100, 1500);
        std::uint32_t seen = 0;
        for (int i = 0; i < 1500 && f.impactS < 0; ++i) f.step(), seen |= f.mask;
        // Starts alerting at first (path 8,000 ft + 1,500 fpm to 9,800 ft in 72 s > 4 NM away)
        // then ends clear: the climb must reach the wall plus the floor, so a TERRAIN is allowed
        // but no PULL UP and no impact.
        CHECK((seen & warningBit(WarningId::PullUp)) == 0);
        CHECK(f.impactS < 0);
    }
    {  // a steady 5 percent upslope far below the aircraft
        FnTerrain slope([](double n) { return static_cast<float>(600 + 0.05 * n); });
        Fly f(slope, 0, 600 * kFtPerM + 3000, 100, 0);
        std::uint32_t seen = 0;
        for (int i = 0; i < 300; ++i) f.step(), seen |= f.mask;
        CHECK(seen == 0);
    }
    {  // level flight toward the same slope with little clearance does warn
        FnTerrain slope([](double n) { return static_cast<float>(600 + 0.05 * n); });
        Fly f(slope, 0, 600 * kFtPerM + 1000, 100, 0);
        std::uint32_t seen = 0;
        for (int i = 0; i < 600 && f.impactS < 0; ++i) f.step(), seen |= f.mask;
        CHECK((seen & warningBit(WarningId::Terrain)) != 0);
    }
}

TEST_CASE("gpws: stabilized ILS suppresses terrain closure, unstabilized does not") {
    // Hill 1.5 km ahead that rises above the path (a real ILS clears it; the glideslope is truth).
    FnTerrain hill([](double n) { return n > 1500 ? 900.0f : 600.0f; });
    auto run = [&](bool onIls, float gsDefl) {
        Fly f(hill, 0, 600 * kFtPerM + 600, 90, -600);
        f.useNav = true;
        f.nav.onFinal = onIls;
        f.nav.gsValid = onIls;
        f.nav.locValid = onIls;
        f.nav.gsDeflection = gsDefl;
        std::uint32_t seen = 0;
        for (int i = 0; i < 300 && f.impactS < 0; ++i) f.step(), seen |= f.mask;
        return seen;
    };
    const std::uint32_t terr = warningBit(WarningId::Terrain) | warningBit(WarningId::PullUp);
    CHECK((run(false, 0) & terr) != 0);
    CHECK((run(true, 0.3f) & terr) == 0);  // 0.6 dots high
    CHECK((run(true, 0.9f) & terr) != 0);  // 1.8 dots: not stabilized
}

TEST_CASE("gpws: normal climb, cruise and a landing at an airport raise no warnings") {
    SyntheticNavDb db;  // XTST at 40 N, 100 W, 2,000 ft
    FnTerrain flat([](double) { return static_cast<float>(SyntheticNavDb::kElevFt * kMPerFt); });
    const double fieldFt = SyntheticNavDb::kElevFt;
    {  // climb out and cruise (no airport inhibit needed past a few NM)
        Fly f(flat, 0, fieldFt + 100, 90, 700);
        f.navDb = &db;
        std::uint32_t seen = 0;
        for (int i = 0; i < 6000; ++i) {
            f.step();
            if (f.s.altAglFt > 3000) f.s.vsFpm = 0;
            seen |= f.mask;
        }
        CHECK(seen == 0);
    }
    {  // visual landing: 3 degree path from 7 NM south down to the flare at the airport
        const double startNorthM = -nmToM(7.0);
        Fly f(flat, startNorthM, fieldFt + 2200, 90, -600);
        f.navDb = &db;
        std::uint32_t seen = 0;
        while (f.s.altAglFt > 20 && f.s.simTimeS < 600) {
            f.step();
            seen |= f.mask;
        }
        CHECK(f.s.altAglFt <= 20);
        CHECK(seen == 0);
        CHECK(f.count(GpwsEventKind::Terrain) == 0);
    }
}

TEST_CASE("gpws: radio altitude callouts once per approach, again after a climb") {
    SyntheticNavDb db;
    FnTerrain flat([](double) { return static_cast<float>(SyntheticNavDb::kElevFt * kMPerFt); });
    const double fieldFt = SyntheticNavDb::kElevFt;
    Fly f(flat, -nmToM(0.5), fieldFt + 3000, 80, -600);
    f.navDb = &db;
    auto callouts = [&]() {
        std::vector<int> v;
        for (const GpwsEvent& e : f.events)
            if (e.kind == GpwsEventKind::Callout) v.push_back(e.valueFt);
        f.events.clear();
        return v;
    };
    while (f.s.altAglFt > 5) f.step();
    const std::vector<int> expect = {2500, 1000, 500, 100, 50, 40, 30, 20, 10};
    CHECK(callouts() == expect);
    f.s.vsFpm = 1500;  // go around, climb back above 2,500 ft, descend again
    while (f.s.altAglFt < 3200) f.step();
    CHECK(callouts().empty());
    f.s.vsFpm = -600;
    while (f.s.altAglFt > 5) f.step();
    CHECK(callouts() == expect);
}

TEST_CASE("gpws: too low gear and too low flaps messages, once") {
    FnTerrain flat([](double) { return 0.0f; });
    {
        Fly f(flat, 0, 450, 80, -500);
        f.s.gearRetractable = true;
        f.s.gearPosNorm = 0;
        f.profile.hasRetractableGear = true;
        for (int i = 0; i < 50; ++i) f.step();
        CHECK(f.count(GpwsEventKind::TooLowGear) == 1);
        CHECK((f.mask & warningBit(WarningId::Gear)) == 0);  // H2 owns the GEAR bit
    }
    {
        Fly f(flat, 0, 220, 80, -500);
        f.profile.approachFlapDetent = 2;
        f.s.flapDetent = 0;
        for (int i = 0; i < 20; ++i) f.step();
        CHECK(f.count(GpwsEventKind::TooLowFlaps) == 1);
        CHECK(f.count(GpwsEventKind::TooLowGear) == 0);
    }
    {  // climbing out with the gear still up is not a warning
        Fly f(flat, 0, 300, 80, 700);
        f.s.gearRetractable = true;
        f.s.gearPosNorm = 0;
        for (int i = 0; i < 20; ++i) f.step();
        CHECK(f.count(GpwsEventKind::TooLowGear) == 0);
    }
}

TEST_CASE("gpws: update cost") {
    FnTerrain ridge([](double n) { return n > 12000 ? 3000.0f : 600.0f; });
    Fly f(ridge, 0, 8000, 100, 0);
    f.s.simTimeS = 0;
    constexpr int kN = 20000;
    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kN; ++i) {
        f.s.simTimeS += 1.0 / 60.0;  // look-ahead (16 samples) runs every 12th call
        f.mon.update(f.s, f.profile, nullptr, ridge);
    }
    const double us =
        std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count() /
        kN;
    std::printf("[gpws] %.2f us per update (60 Hz calls, look-ahead at 5 Hz)\n", us);
    CHECK(us < 100.0);
}
