// Procedures loader (WP N4) on tests/fixtures/nav/procedures: KOTH (all approaches) and KPDX (ILS
// 10R/28L), converted from the real FAA CIFP cycle 2610 by tools/nav/cifp.py.
#include <doctest/doctest.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "core/Geo.h"
#include "nav/FlightPlan.h"
#include "nav/Procedures.h"

using namespace fsim;

namespace {

std::string fixtureNavDir() {
    std::string f = __FILE__;  // .../tests/nav/procedures_test.cpp
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    return f + "/fixtures/nav";
}

std::vector<Approach> koth() {
    clearProcedureCache();
    return loadApproaches(fixtureNavDir(), "KOTH");
}

const Approach& get(const std::vector<Approach>& v, const char* id) {
    const Approach* a = findApproach(v, id);
    REQUIRE_MESSAGE(a != nullptr, id);
    return *a;
}

double distNm(const Waypoint& a, const Waypoint& b) {
    return greatCircleDistM(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg) / 1852.0;
}

}  // namespace

TEST_CASE("nav: KOTH procedures load with the published approach inventory") {
    auto v = koth();
    REQUIRE(v.size() == 7);
    std::vector<std::string> ids;
    for (const auto& a : v) ids.push_back(a.id);
    CHECK(ids ==
          std::vector<std::string>{"H05-Z", "I05-Y", "I05-Z", "L05-Y", "L05-Z", "R05-Y", "VOR-B"});
    const Approach& z = get(v, "I05-Z");
    CHECK(z.airportIdent == "KOTH");
    CHECK(z.runwayIdent == "05");
    CHECK(z.kind == ApproachKind::Ils);
    CHECK_FALSE(z.synthesized);
    CHECK(z.minimumsFt == 0);
    CHECK(get(v, "L05-Z").kind == ApproachKind::Loc);
    CHECK(get(v, "R05-Y").kind == ApproachKind::Rnav);
    CHECK(get(v, "H05-Z").kind == ApproachKind::Rnav);
    CHECK(get(v, "VOR-B").kind == ApproachKind::Vor);
    CHECK(get(v, "VOR-B").runwayIdent.empty());  // circling only
}

TEST_CASE("nav: KOTH ILS Z RWY 5 final and missed approach match the FAA chart") {
    auto v = koth();
    const Approach& a = get(v, "I05-Z");
    // chart: MITUE 1600 (min), OXVAK = FAF 1300, GS 3.00 deg, threshold crossing 16 + 51 = 67 ft
    REQUIRE(a.finalLegs.size() == 3);
    CHECK(a.finalLegs[0].to.ident == "MITUE");
    CHECK(a.finalLegs[0].type == LegType::IF);
    CHECK(a.finalLegs[0].alt.kind == AltConstraintKind::AtOrAbove);
    CHECK(a.finalLegs[0].alt.altFt == doctest::Approx(1600));
    CHECK(a.finalLegs[1].to.ident == "OXVAK");
    CHECK(a.finalLegs[1].type == LegType::CF);
    CHECK(a.finalLegs[1].alt.altFt == doctest::Approx(1300));
    const Leg& rw = a.finalLegs[2];
    CHECK(rw.to.ident == "RW05");
    CHECK(rw.to.kind == WaypointKind::Runway);
    CHECK(rw.alt.kind == AltConstraintKind::At);
    CHECK(rw.alt.altFt == doctest::Approx(67));
    CHECK(rw.flyOver);
    CHECK_FALSE(rw.missedApproach);
    CHECK(rw.courseTrueDeg == doctest::Approx(61.0).epsilon(0.001));  // 046 magnetic + 15.0 E
    CHECK(rw.to.latDeg == doctest::Approx(43.4147278).epsilon(1e-6));
    CHECK(rw.to.lonDeg == doctest::Approx(-124.2605972).epsilon(1e-6));

    // missed: climb to 500, direct course to YICBU (OTH R-301, 8 DME) at or above 2600, hold
    REQUIRE(a.missedLegs.size() == 3);
    for (const Leg& m : a.missedLegs) CHECK(m.missedApproach);
    CHECK(a.missedLegs[0].type == LegType::CA);
    CHECK(a.missedLegs[0].alt.kind == AltConstraintKind::AtOrAbove);
    CHECK(a.missedLegs[0].alt.altFt == doctest::Approx(500));
    CHECK(a.missedLegs[0].to.ident.empty());
    CHECK(a.missedLegs[0].courseTrueDeg == doctest::Approx(60.7).epsilon(0.001));
    CHECK(a.missedLegs[1].type == LegType::CF);
    CHECK(a.missedLegs[1].to.ident == "YICBU");
    CHECK(a.missedLegs[1].alt.altFt == doctest::Approx(2600));
    const Leg& hm = a.missedLegs[2];
    CHECK(hm.type == LegType::HM);
    CHECK(hm.to.ident == "YICBU");
    CHECK(hm.courseTrueDeg == doctest::Approx(136.0).epsilon(0.001));  // inbound 121 magnetic
    CHECK(hm.arcRight);
    CHECK(hm.alt.altFt == doctest::Approx(2600));
}

TEST_CASE("nav: RF and DME-arc legs carry centre, radius and direction") {
    auto v = koth();
    const Approach& rnp = get(v, "H05-Z");
    int rfs = 0;
    for (const Leg& l : rnp.transitions[0].legs) {
        if (l.type != LegType::RF) continue;
        ++rfs;
        CHECK(l.arcRadiusNm == doctest::Approx(3.0).epsilon(0.001));
        CHECK(l.arcRight);
        CHECK(
            distNm(l.to, Waypoint{"c", WaypointKind::Fix, l.arcCenterLatDeg, l.arcCenterLonDeg}) ==
            doctest::Approx(l.arcRadiusNm).epsilon(0.03));
    }
    CHECK(rfs == 2);
    // missed approach with a left RF
    const Leg* left = nullptr;
    for (const Leg& m : rnp.missedLegs)
        if (m.type == LegType::RF) left = &m;
    REQUIRE(left != nullptr);
    CHECK_FALSE(left->arcRight);
    CHECK(left->arcRadiusNm == doctest::Approx(4.85).epsilon(0.001));

    // The 16 DME arc of ILS Z (AF in ARINC) arrives as an RF about the OTH VORTAC.
    const Approach& z = get(v, "I05-Z");
    const Leg& arc = z.transitions[0].legs[1];  // LUPCI transition: IF LUPCI, arc to MITUE
    CHECK(z.transitions[0].name == "LUPCI");
    CHECK(arc.type == LegType::RF);
    CHECK(arc.to.ident == "MITUE");
    CHECK(arc.arcRadiusNm == doctest::Approx(16.0).epsilon(0.01));
    CHECK(arc.arcRight);
    CHECK(arc.arcCenterLatDeg == doctest::Approx(43.4155056).epsilon(1e-6));
}

TEST_CASE("nav: holds, kinds and extras") {
    clearProcedureCache();
    auto set = loadProcedureSet(fixtureNavDir(), "KOTH");
    REQUIRE((set != nullptr));
    CHECK(set->cycle == "2610");
    CHECK(set->airportIdent == "KOTH");
    const ApproachInfo* info = findApproachInfo(*set, "I05-Z");
    REQUIRE(info);
    CHECK(info->title == "ILS Z RWY 05");
    CHECK(info->fafFinalLeg == 1);  // OXVAK
    CHECK(info->mapFinalLeg == 2);  // RW05
    CHECK(info->finalExtras[2].map);
    CHECK(info->finalExtras[2].vertAngleDeg == doctest::Approx(-3.0));
    CHECK(info->missedExtras[2].holdLegTimeS == doctest::Approx(60.0));
    // hold in lieu of a procedure turn at MITUE (HF, one minute, right turns, inbound 060.7 true)
    const Approach& z = get(set->approaches, "I05-Z");
    const auto& oth = z.transitions[1];
    CHECK(oth.name == "OTH");
    REQUIRE(oth.legs.size() == 3);
    CHECK(oth.legs[0].to.kind == WaypointKind::Vor);
    CHECK(oth.legs[2].type == LegType::HF);
    CHECK(oth.legs[2].arcRight);
    CHECK(oth.legs[2].courseTrueDeg == doctest::Approx(60.7).epsilon(0.001));
    CHECK(set->info[0].levelsOfService.empty());
    const ApproachInfo* r = findApproachInfo(*set, "R05-Y");
    REQUIRE(r);
    CHECK(r->levelsOfService == std::vector<std::string>{"LPV", "LNAV/VNAV", "LNAV"});
    // the AF leg is flagged as approximated
    bool sawApprox = false;
    for (const auto& ex : set->info[2].transitionExtras[0])
        if (ex.approximatedFrom == "AF") sawApprox = true;
    CHECK(sawApprox);
}

TEST_CASE("nav: KPDX ILS 10R and 28L") {
    clearProcedureCache();
    auto v = loadApproaches(fixtureNavDir(), "kpdx");  // case-insensitive
    REQUIRE(v.size() == 2);
    const Approach& r = get(v, "I10R");
    CHECK(r.runwayIdent == "10R");
    CHECK(r.finalLegs.front().to.ident == "SCAPO");
    CHECK(r.finalLegs.back().to.ident == "RW10R");
    CHECK(r.finalLegs[2].to.ident == "POWLZ");  // FAF
    CHECK(r.finalLegs[2].alt.altFt == doctest::Approx(2000));
    CHECK(r.finalLegs.back().alt.altFt == doctest::Approx(76));
    CHECK(r.missedLegs.front().type == LegType::CA);
    CHECK(r.missedLegs.back().type == LegType::HM);
    CHECK_FALSE(r.missedLegs.back().arcRight);  // left-hand hold at MULES
    const Approach& l = get(v, "I28L");
    CHECK(l.transitions.size() == 3);
    CHECK(l.finalLegs.back().to.ident == "RW28L");
    CHECK(l.finalLegs.back().alt.altFt == doctest::Approx(78));
    CHECK(l.missedLegs[1].to.ident == "UBG");
    CHECK(l.missedLegs[1].to.kind == WaypointKind::Vor);
}

TEST_CASE("nav: findApproach and approachesForRunway") {
    auto v = koth();
    CHECK((findApproach(v, "I05-Z") == &v[2]));
    CHECK((findApproach(v, "i05-z") == &v[2]));
    CHECK((findApproach(v, "I05") == &v[1]));  // first by id: I05-Y
    CHECK(findApproach(v, "R05")->id == "R05-Y");
    CHECK(findApproach(v, "VOR-B")->id == "VOR-B");
    CHECK((findApproach(v, "I23") == nullptr));
    CHECK((findApproach(v, "") == nullptr));
    auto rwy = approachesForRunway(v, "05");
    CHECK(rwy.size() == 6);
    CHECK(rwy.front()->id == "H05-Z");
    CHECK(approachesForRunway(v, "23").empty());
    CHECK(approachesForRunway(v, "").empty());

    // short-form rule: "R28" must not find "R28L"; "I16R" finds "I16RZ"
    std::vector<Approach> w(3);
    w[0].id = "I16RZ";
    w[1].id = "R28L";
    w[2].id = "R28L-Y";
    CHECK((findApproach(w, "I16R") == &w[0]));
    CHECK((findApproach(w, "R28") == nullptr));
    CHECK((findApproach(w, "R28L") == &w[1]));
}

TEST_CASE("nav: loadApproaches caches and tolerates missing files") {
    clearProcedureCache();
    auto a = loadProcedureSet(fixtureNavDir(), "KOTH");
    auto b = loadProcedureSet(fixtureNavDir() + "/", "koth");
    CHECK((a.get() == b.get()));  // same parsed object
    CHECK(loadApproaches(fixtureNavDir(), "KXYZ").empty());
    CHECK(loadApproaches("/nonexistent/dir", "KOTH").empty());
    CHECK(loadApproaches(fixtureNavDir(), "../nav/KOTH").empty());  // no path tricks
    CHECK(loadApproaches(fixtureNavDir(), "").empty());
    // a data root also works (nav/procedures below it)
    std::string root = fixtureNavDir();
    root = root.substr(0, root.find_last_of('/'));  // .../fixtures: has nav/procedures
    CHECK(loadApproaches(root, "KOTH").size() == 7);
}

TEST_CASE("nav: assembleApproachLegs joins transition, final and missed without duplicates") {
    auto v = koth();
    const Approach& rnp = get(v, "H05-Z");
    // transition DEROY ends at OXVAK, where the final segment starts with an IF at OXVAK
    bool found = false;
    auto legs = assembleApproachLegs(rnp, "deroy", &found);
    CHECK(found);
    const size_t trLegs = rnp.transitions[0].legs.size();
    CHECK(rnp.transitions[0].name == "DEROY");
    CHECK(legs.size() == trLegs + rnp.finalLegs.size() - 1 + rnp.missedLegs.size());
    CHECK(legs[trLegs - 1].to.ident == "OXVAK");
    CHECK(legs[trLegs].to.ident == "RW05");
    CHECK_FALSE(legs[trLegs].missedApproach);
    CHECK(legs[trLegs + 1].missedApproach);
    CHECK(legs.back().missedApproach);
    int ifLegs = 0;
    for (const Leg& l : legs) ifLegs += l.type == LegType::IF;
    CHECK(ifLegs == 1);  // only the transition's own IF remains

    // vectors (no transition): the final segment starts with its IF
    auto vec = assembleApproachLegs(rnp, "", &found);
    CHECK(found);
    CHECK(vec.size() == rnp.finalLegs.size() + rnp.missedLegs.size());
    CHECK(vec.front().type == LegType::IF);
    auto unknown = assembleApproachLegs(rnp, "NOPE", &found);
    CHECK_FALSE(found);
    CHECK(unknown.size() == vec.size());

    // ILS Z via the OTH transition ends in a hold at MITUE, which is also the final IF
    auto ils = assembleApproachLegs(get(v, "I05-Z"), "OTH");
    int atMitue = 0;
    for (const Leg& l : ils) atMitue += l.to.ident == "MITUE";
    CHECK(atMitue == 2);  // TF MITUE + HF MITUE, the duplicate IF is gone
}

TEST_CASE("nav: assembled approach goes into a flight plan") {
    auto v = koth();
    const Approach& a = get(v, "I05-Z");
    FlightPlan p;
    p.originIdent = "KPDX";
    p.destIdent = "KOTH";
    p.arrRunway = "05";
    auto mk = [](const char* id, double lat, double lon) {
        Leg l;
        l.type = LegType::TF;
        l.to = Waypoint{id, WaypointKind::Fix, lat, lon};
        return l;
    };
    Leg o = mk("KPDX", 45.5887, -122.5975);
    o.type = LegType::IF;
    Leg dest = mk("RW05", 43.4147, -124.2606);
    dest.to.kind = WaypointKind::Runway;  // N3 recognises the destination leg by its kind
    p.legs = {o, mk("UBG", 45.3532, -122.9781), mk("RARES", 43.6603, -124.0229), dest};
    std::string err;
    auto legs = assembleApproachLegs(a, "RARES");
    REQUIRE(insertApproach(p, a.id, a.kind, a.runwayIdent, legs, &err));
    CHECK(p.approachId == "I05-Z");
    CHECK(p.approachKind == ApproachKind::Ils);
    CHECK(p.arrRunway == "05");
    CHECK(p.approachStartLeg == 3);
    CHECK(p.legs[3].to.ident == "RARES");  // the transition's own IF
    CHECK(p.legs[3].type == LegType::IF);
    CHECK(p.legs.back().missedApproach);
    CHECK(p.legs.back().type == LegType::HM);
    // flown distance excludes the missed approach; CA/VA legs have no length
    const double flown = planDistNm(p);
    const double all = planDistNm(p, 0, true);
    CHECK(all > flown);
    CHECK(flown > 100.0);
}

TEST_CASE("nav: procedure file parser rejects bad input and skips future options") {
    ProcedureSet s;
    std::string err;
    CHECK_FALSE(parseProcedures("", s, err));
    CHECK_FALSE(parseProcedures("AIRPORT KOTH\n", s, err));
    CHECK(err.find("FORMAT") != std::string::npos);
    CHECK_FALSE(parseProcedures("FORMAT flightsim-proc 2\n", s, err));
    CHECK(err.find("unsupported") != std::string::npos);
    CHECK_FALSE(parseProcedures("FORMAT flightsim-nav 1\n", s, err));
    const std::string head =
        "FORMAT flightsim-proc 1\nAIRPORT KXYZ\nAPPROACH I05 ILS 05 ILS RWY 05\nFINAL\n";
    CHECK_FALSE(parseProcedures(head + "LEG XX ABC 1 2\nEND\n", s, err));
    CHECK(err.find("line 5") != std::string::npos);
    CHECK_FALSE(parseProcedures(head + "LEG TF ABC 1\nEND\n", s, err));
    CHECK_FALSE(parseProcedures(head + "LEG TF ABC 95 2\nEND\n", s, err));  // latitude out of range
    CHECK_FALSE(parseProcedures(head + "LEG TF ABC 1 2 ALT SIDEWAYS 100\nEND\n", s, err));
    CHECK_FALSE(parseProcedures(head + "LEG TF ABC 1 2 BOGUS\nEND\n", s, err));
    CHECK_FALSE(parseProcedures("FORMAT flightsim-proc 1\nLEG TF A 1 2\n", s, err));

    REQUIRE(parseProcedures(
        "# c\nFORMAT flightsim-proc 1\nFUTURE x\nCYCLE 2610\nAIRPORT KXYZ\n" +
            std::string("APPROACH I05 ILS 05 ILS RWY 05\nFINAL\n") +
            "LEG TF ABC 1.5 -2.5 ALT BETWEEN 3000 5000 SPD 210 NEWKEY=7 WK NDB\n"
            "LEG DF DEF 1.6 -2.6 TURN L\nMISSED\nLEG CA - CRS 10.5 ALT AT 800\nEND\n",
        s, err));
    REQUIRE(s.approaches.size() == 1);
    const Approach& a = s.approaches[0];
    CHECK(a.finalLegs[0].alt.kind == AltConstraintKind::Between);
    CHECK(a.finalLegs[0].alt.altFt == doctest::Approx(3000));
    CHECK(a.finalLegs[0].alt.alt2Ft == doctest::Approx(5000));
    CHECK(a.finalLegs[0].speedLimitKt == doctest::Approx(210));
    CHECK(a.finalLegs[0].to.kind == WaypointKind::Ndb);
    CHECK_FALSE(a.finalLegs[1].arcRight);
    CHECK(a.missedLegs[0].missedApproach);
    CHECK(a.missedLegs[0].courseTrueDeg == doctest::Approx(10.5));
    CHECK(a.missedLegs[0].to.ident.empty());
}
