// NavHud pure logic (WP H3): FMA text per mode combination, change-box timing, CDI / LOC / GS
// positions and the +-1.25 clamp, heading mark wrap at 360, bug positions on the tapes, flight
// director offsets, the minimums trigger, nav box text and the fake states. No GL.
#include <doctest/doctest.h>

#include <cmath>
#include <cstring>

#include "hud/NavHudFake.h"
#include "hud/NavHudLogic.h"

using namespace fsim;

namespace {
std::string fmaStatus(const AutopilotState& ap) {
    FmaText t;
    buildFma(ap, t);
    return t.status;
}
}  // namespace

TEST_CASE("FMA text per mode combination") {
    AutopilotState ap;
    FmaText t;
    buildFma(ap, t);
    CHECK(std::strcmp(t.spd, "") == 0);
    CHECK(std::strcmp(t.lat, "") == 0);
    CHECK(std::strcmp(t.vert, "") == 0);
    CHECK(std::strcmp(t.status, "") == 0);

    ap.apEngaged = ap.fdOn = ap.atEngaged = true;
    ap.spdActive = SpdMode::Speed;
    ap.latActive = LatMode::Hdg;
    ap.vertActive = VertMode::Alt;
    buildFma(ap, t);
    CHECK(std::strcmp(t.spd, "SPD") == 0);
    CHECK(std::strcmp(t.lat, "HDG") == 0);
    CHECK(std::strcmp(t.vert, "ALT") == 0);
    CHECK(std::strcmp(t.status, "AP FD A/T") == 0);

    ap.latActive = LatMode::Loc;
    ap.vertActive = VertMode::Gs;
    ap.spdActive = SpdMode::Retard;
    buildFma(ap, t);
    CHECK(std::strcmp(t.lat, "LOC") == 0);
    CHECK(std::strcmp(t.vert, "G/S") == 0);
    CHECK(std::strcmp(t.spd, "RETARD") == 0);

    ap.latActive = LatMode::Hdg;
    ap.latArmed = LatMode::Loc;
    ap.vertActive = VertMode::Alt;
    ap.vertArmed = VertMode::Gs;
    buildFma(ap, t);
    CHECK(std::strcmp(t.latArmed, "(LOC)") == 0);
    CHECK(std::strcmp(t.vertArmed, "(G/S)") == 0);

    ap.latActive = LatMode::Roll;
    ap.latArmed = LatMode::Nav;
    ap.vertActive = VertMode::Pitch;
    ap.vertArmed = VertMode::Alt;
    buildFma(ap, t);
    CHECK(std::strcmp(t.lat, "ROL") == 0);
    CHECK(std::strcmp(t.latArmed, "(NAV)") == 0);
    CHECK(std::strcmp(t.vert, "PIT") == 0);
    CHECK(std::strcmp(t.vertArmed, "(ALT)") == 0);

    ap.vertActive = VertMode::AltCapture;
    buildFma(ap, t);
    CHECK(std::strcmp(t.vert, "ALT*") == 0);
    ap.vertActive = VertMode::Flc;
    buildFma(ap, t);
    CHECK(std::strcmp(t.vert, "FLC") == 0);
    ap.latActive = LatMode::Nav;
    buildFma(ap, t);
    CHECK(std::strcmp(t.lat, "NAV") == 0);
}

TEST_CASE("FMA vertical speed text and status combinations") {
    char b[16];
    formatVertActive(b, sizeof b, VertMode::Vs, 712);
    CHECK(std::strcmp(b, "VS +700") == 0);
    formatVertActive(b, sizeof b, VertMode::Vs, -730);
    CHECK(std::strcmp(b, "VS -750") == 0);
    formatVertActive(b, sizeof b, VertMode::Vs, 0);
    CHECK(std::strcmp(b, "VS +0") == 0);

    AutopilotState ap;
    CHECK(fmaStatus(ap).empty());
    ap.fdOn = true;
    CHECK(fmaStatus(ap) == "FD");
    ap.apEngaged = true;
    CHECK(fmaStatus(ap) == "AP FD");
    ap.fdOn = false;
    ap.atEngaged = true;
    CHECK(fmaStatus(ap) == "AP A/T");
    ap.apEngaged = false;
    CHECK(fmaStatus(ap) == "A/T");
}

TEST_CASE("FMA change box lasts 5 s and ignores the first sample") {
    FmaChangeTracker t;
    AutopilotState ap;
    ap.latActive = LatMode::Hdg;
    t.update(ap, 100.0);
    CHECK_FALSE(t.boxVisible(FmaColumn::Lat, 100.0));
    ap.latActive = LatMode::Loc;
    t.update(ap, 101.0);
    CHECK(t.boxVisible(FmaColumn::Lat, 101.0));
    CHECK_FALSE(t.boxVisible(FmaColumn::Vert, 101.0));
    CHECK(t.boxVisible(FmaColumn::Lat, 105.9));
    CHECK_FALSE(t.boxVisible(FmaColumn::Lat, 106.1));
    // Armed changes do not box; a vertical change does, independently.
    ap.vertArmed = VertMode::Gs;
    t.update(ap, 110.0);
    CHECK_FALSE(t.boxVisible(FmaColumn::Vert, 110.0));
    ap.vertActive = VertMode::Gs;
    t.update(ap, 111.0);
    CHECK(t.boxVisible(FmaColumn::Vert, 112.0));
    t.reset();
    t.update(ap, 200.0);
    CHECK_FALSE(t.boxVisible(FmaColumn::Vert, 200.0));
}

TEST_CASE("CDI dots, clamp and pegging") {
    CHECK(deviationDots(0).dots == doctest::Approx(0));
    CHECK(deviationDots(1.0f).dots == doctest::Approx(2.0f));
    CHECK_FALSE(deviationDots(1.0f).pegged);
    CHECK(deviationDots(-0.5f).dots == doctest::Approx(-1.0f));
    CHECK(deviationDots(1.25f).dots == doctest::Approx(2.5f));
    CHECK(deviationDots(1.25f).pegged);
    CHECK(deviationDots(9.0f).dots == doctest::Approx(2.5f));
    CHECK(deviationDots(-9.0f).dots == doctest::Approx(-2.5f));
    CHECK(deviationDots(-9.0f).pegged);
    CHECK(deviationDots(std::nanf("")).dots == doctest::Approx(0));
    // VNAV: 250 ft = full scale
    CHECK(vnavDots(250).dots == doctest::Approx(2.0f));
    CHECK(vnavDots(-125).dots == doctest::Approx(-1.0f));
    CHECK(vnavDots(1000).pegged);
}

TEST_CASE("CDI scale names and sources") {
    NavState n;
    CHECK(cdiScaleFor(n) == CdiScale::None);
    n.source = NavSource::Gps;
    n.phase = NavPhase::Enroute;
    CHECK(cdiScaleFor(n) == CdiScale::Enr);
    n.phase = NavPhase::Terminal;
    CHECK(cdiScaleFor(n) == CdiScale::Term);
    n.phase = NavPhase::Approach;
    CHECK(cdiScaleFor(n) == CdiScale::Apr);
    n.cdiFullScaleNm = 2.0f;
    CHECK(cdiScaleFor(n) == CdiScale::Enr);
    n.cdiFullScaleNm = 1.0f;
    CHECK(cdiScaleFor(n) == CdiScale::Term);
    n.cdiFullScaleNm = 0.3f;
    CHECK(cdiScaleFor(n) == CdiScale::Apr);
    n.source = NavSource::Loc;
    CHECK(cdiScaleFor(n) == CdiScale::None);
    CHECK(std::strcmp(cdiScaleName(CdiScale::Term), "TERM") == 0);
    CHECK(std::strcmp(navSourceName(NavSource::Vor), "VOR") == 0);
    CHECK(std::strcmp(navSourceName(NavSource::None), "") == 0);
}

TEST_CASE("LOC / GS scale state: visibility, fill and direction") {
    NavState n;
    AutopilotState ap;
    ScaleState s = buildScales(n, ap);
    CHECK_FALSE(s.locVisible);
    CHECK_FALSE(s.gsVisible);

    n.source = NavSource::Gps;
    strcpy(n.activeWptIdent, "CEPIN");
    n.cdiDeflection = -0.4f;
    s = buildScales(n, ap);
    CHECK(s.locVisible);
    CHECK(s.loc.dots == doctest::Approx(-0.8f));

    n.source = NavSource::Loc;
    n.locValid = true;
    n.gsValid = true;
    n.cdiDeflection = 1.25f;
    n.gsDeflection = 0.5f;  // path above the aircraft -> positive dots (drawn up)
    s = buildScales(n, ap);
    CHECK(s.locFilled);
    CHECK(s.gsFilled);
    CHECK(s.loc.pegged);
    CHECK(s.gs.dots == doctest::Approx(1.0f));
    CHECK_FALSE(s.gsIsVnav);

    // Armed but not captured: hollow. Captured: filled.
    ap.latArmed = LatMode::Loc;
    ap.vertArmed = VertMode::Gs;
    s = buildScales(n, ap);
    CHECK_FALSE(s.locFilled);
    CHECK_FALSE(s.gsFilled);
    ap.latActive = LatMode::Loc;
    ap.vertActive = VertMode::Gs;
    s = buildScales(n, ap);
    CHECK(s.locFilled);
    CHECK(s.gsFilled);

    // Advisory VNAV: hollow on the GS scale when no glideslope.
    n.gsValid = false;
    n.vnavValid = true;
    n.vnavDevFt = -250;
    s = buildScales(n, ap);
    CHECK(s.gsVisible);
    CHECK(s.gsIsVnav);
    CHECK_FALSE(s.gsFilled);
    CHECK(s.gs.dots == doctest::Approx(-2.0f));
}

TEST_CASE("heading marks wrap at 360 and park off-scale") {
    TapeMark m = headingMark(2, 358);  // 4 deg right across north
    CHECK_FALSE(m.offScale);
    CHECK(m.dxPx == doctest::Approx(4 * kHeadingTapePxPerDeg));
    m = headingMark(358, 2);
    CHECK(m.dxPx == doctest::Approx(-4 * kHeadingTapePxPerDeg));
    m = headingMark(0, 0);
    CHECK(m.valueMagDeg == 360);
    m = headingMark(359.6f, 10);
    CHECK(m.valueMagDeg == 360);
    m = headingMark(90, 90);
    CHECK(m.dxPx == doctest::Approx(0));
    CHECK(m.valueMagDeg == 90);
    m = headingMark(95, 350);  // 105 deg right: off scale, parked at the right end
    CHECK(m.offScale);
    CHECK(m.dxPx == doctest::Approx(kTapeHalfWidthPx));
    m = headingMark(270, 10);  // 100 deg left
    CHECK(m.offScale);
    CHECK(m.dxPx == doctest::Approx(-kTapeHalfWidthPx));
    m = headingMark(35, 0);  // exactly at the edge is still on the tape
    CHECK_FALSE(m.offScale);
    CHECK(headingMark(36, 0).offScale);
    CHECK(trueToMagDeg(5, 10) == doctest::Approx(355));  // east variation: mag = true - var
    CHECK(trueToMagDeg(355, -10) == doctest::Approx(5));
}

TEST_CASE("speed and altitude bug positions") {
    BugPos b = tapeBug(kSpeedTapeGeom, 130, 120);  // 10 kt above = 20 px up
    CHECK_FALSE(b.offScale);
    CHECK(b.y == doctest::Approx(240 - 20));
    b = tapeBug(kSpeedTapeGeom, 120, 120);
    CHECK(b.y == doctest::Approx(240));
    b = tapeBug(kSpeedTapeGeom, 300, 120);  // far above: parked at the top
    CHECK(b.offScale);
    CHECK(b.above);
    CHECK(b.y == doctest::Approx(kSpeedTapeGeom.topY));
    b = tapeBug(kSpeedTapeGeom, 20, 120);
    CHECK(b.offScale);
    CHECK_FALSE(b.above);
    CHECK(b.y == doctest::Approx(kSpeedTapeGeom.bottomY));
    b = tapeBug(kAltTapeGeom, 3100, 3000);  // 100 ft = 16 px
    CHECK(b.y == doctest::Approx(240 - 16));
    b = tapeBug(kAltTapeGeom, 5000, 3000);
    CHECK(b.offScale);
    CHECK(b.above);
}

TEST_CASE("flight director cue offset") {
    FdOffset o = fdCueOffset(5, 5, 0, 0, 6);
    CHECK(o.dx == doctest::Approx(0));
    CHECK(o.dy == doctest::Approx(0));
    o = fdCueOffset(8, 5, 0, 0, 6);  // pitch up needed: cue above the FPV
    CHECK(o.dy == doctest::Approx(-18));
    o = fdCueOffset(0, 5, 0, 0, 6);  // pitch down needed: cue below
    CHECK(o.dy == doctest::Approx(30));
    o = fdCueOffset(0, 0, 15, 0, 6);  // roll right needed: cue right
    CHECK(o.dx == doctest::Approx(15 * kFdRollPxPerDeg));
    o = fdCueOffset(0, 0, -100, 0, 6);
    CHECK(o.dx == doctest::Approx(-kFdMaxXPx));
    o = fdCueOffset(90, 0, 0, 0, 6);
    CHECK(o.dy == doctest::Approx(-kFdMaxYPx));
}

TEST_CASE("minimums trigger on a downward crossing only") {
    MinimumsTracker t;
    NavState n;
    n.minimumsFt = 216;
    t.update(n, 400, 1.0);
    CHECK_FALSE(t.calloutVisible(1.0));
    t.update(n, 250, 2.0);
    CHECK_FALSE(t.consumeAlert());
    t.update(n, 215.9f, 3.0);
    CHECK(t.calloutVisible(3.0));
    CHECK(t.calloutVisible(5.9));
    CHECK_FALSE(t.calloutVisible(6.1));
    CHECK(t.consumeAlert());
    CHECK_FALSE(t.consumeAlert());
    t.update(n, 150, 4.0);  // stays below: no second alert
    CHECK_FALSE(t.consumeAlert());
    // Missed approach climb re-arms after 100 ft above, then another descent fires again.
    t.update(n, 300, 5.0);
    t.update(n, 330, 6.0);
    t.update(n, 200, 7.0);
    CHECK(t.consumeAlert());
}

TEST_CASE("minimums: no trigger without minimums or when starting below") {
    MinimumsTracker t;
    NavState n;
    t.update(n, 500, 0.0);
    t.update(n, 100, 1.0);
    CHECK_FALSE(t.consumeAlert());
    n.minimumsFt = 216;
    t.update(n, 100, 2.0);  // first sample is already below: only records
    CHECK_FALSE(t.consumeAlert());
    t.update(n, 90, 3.0);
    CHECK_FALSE(t.consumeAlert());
    t.reset();
    CHECK_FALSE(t.calloutVisible(3.0));
}

TEST_CASE("minimums text DA / MDA") {
    NavState n;
    char b[16];
    formatMinimums(b, sizeof b, n);
    CHECK(b[0] == 0);
    n.minimumsFt = 213.4f;
    formatMinimums(b, sizeof b, n);
    CHECK(std::strcmp(b, "DA 213") == 0);
    n.minimumsIsDa = false;
    n.minimumsFt = 560;
    formatMinimums(b, sizeof b, n);
    CHECK(std::strcmp(b, "MDA 560") == 0);
}

TEST_CASE("ETE format") {
    char b[16];
    formatEte(b, sizeof b, 372);
    CHECK(std::strcmp(b, "6:12") == 0);
    formatEte(b, sizeof b, 3930);
    CHECK(std::strcmp(b, "1:05:30") == 0);
    formatEte(b, sizeof b, 0);
    CHECK(std::strcmp(b, "--:--") == 0);
    formatEte(b, sizeof b, -5);
    CHECK(std::strcmp(b, "--:--") == 0);
}

TEST_CASE("nav box text per source and declutter") {
    NavState n;
    NavBoxText t;
    buildNavBox(n, 2, 0, t);
    CHECK(t.lines == 1);
    CHECK(std::strcmp(t.line[0], "NO NAV") == 0);

    n.source = NavSource::Gps;
    n.phase = NavPhase::Enroute;
    strcpy(n.activeWptIdent, "CEPIN");
    strcpy(n.nextWptIdent, "ARCHI");
    n.distToWptNm = 12.4f;
    n.eteToWptS = 372;
    n.desiredTrackTrueDeg = 291;
    n.crossTrackNm = 0.2f;
    buildNavBox(n, 2, 10, t);  // magnetic = true - 10 east
    CHECK(t.lines == 3);
    CHECK(std::strstr(t.line[0], "CEPIN 12.4NM") != nullptr);
    CHECK(std::strstr(t.line[1], "DTK 281") != nullptr);
    CHECK(std::strstr(t.line[1], "XTK 0.2R") != nullptr);
    CHECK(std::strstr(t.line[1], "ETE 6:12") != nullptr);
    CHECK(std::strstr(t.line[2], "NEXT ARCHI") != nullptr);
    n.crossTrackNm = -0.7f;
    buildNavBox(n, 2, 10, t);
    CHECK(std::strstr(t.line[1], "XTK 0.7L") != nullptr);

    buildNavBox(n, 1, 10, t);  // STD: one line
    CHECK(t.lines == 1);
    buildNavBox(n, 0, 10, t);  // MIN: none
    CHECK(t.lines == 0);

    n.phase = NavPhase::Approach;
    strcpy(n.approachName, "R05");
    n.minimumsFt = 1190;
    n.minimumsIsDa = false;
    buildNavBox(n, 2, 10, t);
    CHECK(std::strstr(t.line[2], "MDA 1190") != nullptr);
    n.phase = NavPhase::Missed;
    buildNavBox(n, 2, 10, t);
    CHECK(std::strcmp(t.line[2], "MISSED APPR") == 0);

    NavState l;
    l.source = NavSource::Loc;
    l.locValid = true;
    strcpy(l.approachName, "ILS 05");
    l.desiredTrackTrueDeg = 55;
    l.distToThresholdNm = 8.4f;
    l.minimumsFt = 216;
    buildNavBox(l, 2, 10, t);
    CHECK(std::strstr(t.line[0], "LOC ILS 05 CRS 045") != nullptr);
    CHECK(std::strstr(t.line[1], "DME 8.4") != nullptr);
    CHECK(std::strstr(t.line[1], "DA 216") != nullptr);
}

TEST_CASE("FMA nav summary") {
    NavState n;
    char b[48];
    buildFmaNavSummary(n, b, sizeof b);
    CHECK(b[0] == 0);
    n.source = NavSource::Gps;
    strcpy(n.activeWptIdent, "CEPIN");
    n.distToWptNm = 12.4f;
    n.eteToWptS = 372;
    buildFmaNavSummary(n, b, sizeof b);
    CHECK(std::strcmp(b, "GPS CEPIN 12.4 NM 6:12") == 0);
}

TEST_CASE("hud fake states exist and unknown names are rejected") {
    AircraftState s;
    s.headingTrueRad = 1.0f;
    s.iasKt = 90;
    s.altMslFt = 3000;
    NavState nav;
    AutopilotState ap;
    std::uint32_t warn = 0;
    const char* names[] = {"gps_enr", "gps_term", "gps_apr", "loc_gs",   "loc_armed",
                           "hdg_alt", "vs",       "ap_off",  "minimums", "fd"};
    for (const char* name : names) {
        INFO(name);
        CHECK(hudFake(name, 0, s, 0.0f, nav, ap, warn));
    }
    CHECK_FALSE(hudFake("nope", 0, s, 0.0f, nav, ap, warn));
    CHECK(hudFake("ap_off", 0, s, 0.0f, nav, ap, warn));
    CHECK(ap.disconnectWarning);
    CHECK(warn == warningBit(WarningId::ApOff));
    // minimums: above the aircraft before frame 60, below it after.
    CHECK(hudFake("minimums", 0, s, 0.0f, nav, ap, warn));
    CHECK(nav.minimumsFt < 3000);
    CHECK(hudFake("minimums", 90, s, 0.0f, nav, ap, warn));
    CHECK(nav.minimumsFt > 3000);
}
