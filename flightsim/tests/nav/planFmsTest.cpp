#include <doctest/doctest.h>

#include <fstream>
#include <sstream>

#include "core/Log.h"
#include "core/SyntheticNavDb.h"
#include "nav/FlightPlan.h"
#include "nav/Fms.h"
#include "planFakeDb.h"

using namespace fsim;
using fsim::test::PlanFakeDb;

namespace {

const char* kBasic =
    "I\n1100 Version\nCYCLE 1710\nADEP KAAA\nDEPRWY RW28L\nADES KBBB\nDESRWY RW05\nAPP I05\n"
    "NUMENR 4\n"
    "1 KAAA ADEP 30.000000 45.000000 -122.000000\n"
    "3 CVO DRCT 8000.000000 44.500000 -123.300000\n"
    "28 +44.000_-123.800 DRCT 8000.000000 44.000000 -123.800000\n"
    "1 KBBB ADES 15.000000 43.000000 -124.000000\n";

std::string readFile(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

bool hasMsg(const std::vector<std::string>& v, const std::string& needle) {
    for (const auto& s : v)
        if (s.find(needle) != std::string::npos) return true;
    return false;
}

}  // namespace

TEST_CASE("nav: fms parses a basic v1100 file") {
    auto r = parseFms(kBasic);
    REQUIRE(r.ok);
    CHECK(r.file.cycle == "1710");
    CHECK(r.file.adep == "KAAA");
    CHECK(r.file.deprwy == "RW28L");
    CHECK(r.file.app == "I05");
    REQUIRE(r.file.entries.size() == 4);
    CHECK(r.file.entries[1].type == FmsEntryType::Vor);
    CHECK(r.file.entries[1].altFt == doctest::Approx(8000.0f));
    CHECK(r.file.entries[2].type == FmsEntryType::LatLon);
    CHECK(r.file.entries[3].latDeg == doctest::Approx(43.0));
    CHECK(r.file.entries[1].line == 11);
}

TEST_CASE("nav: fms tolerates CRLF, tabs, extra whitespace, BOM, blank lines, lower case") {
    std::string t = "\xEF\xBB\xBF";
    t += "I\r\n1100 Version\r\n\r\n  cycle   1710  \r\nADEP\tKAAA\r\nADES KBBB\r\nNUMENR 2\r\n";
    t += "1   KAAA\tADEP  30.0   45.0   -122.0   \r\n";
    t += "1 kbbb ADES 15 43.0 -124.0\r\n\r\n";
    auto r = parseFms(t);
    REQUIRE(r.ok);
    CHECK(r.file.cycle == "1710");
    CHECK(r.file.entries.size() == 2);
    CHECK(r.file.entries[1].ident == "KBBB");
    CHECK(r.file.entries[0].lonDeg == doctest::Approx(-122.0));
    CHECK(r.warnings.empty());
}

TEST_CASE("nav: fms parses a SimBrief-style export with SID, STAR and airways") {
    const char* t =
        "I\n1100 Version\nCYCLE 2610\nADEP KAAA\nDEPRWY RW10R\nSID BRDGE4\nSIDTRANS ZZZ\n"
        "ADES KBBB\nDESRWY RW05\nSTAR OILER2\nSTARTRANS CVO\nAPP I05\nAPPTRANS FAF\nNUMENR 5\n"
        "1 KAAA ADEP 30.000000 45.000000 -122.000000\n"
        "11 WPT01 BRDGE4 0.000000 44.200000 -122.800000\n"
        "3 CVO V495 9000.000000 44.500000 -123.300000\n"
        "11 WPT01 V495 9000.000000 44.200000 -122.800000\n"
        "1 KBBB ADES 15.000000 43.000000 -124.000000\n";
    auto r = parseFms(t);
    REQUIRE(r.ok);
    CHECK(r.file.sid == "BRDGE4");
    CHECK(r.file.sidTrans == "ZZZ");
    CHECK(r.file.star == "OILER2");
    CHECK(r.file.starTrans == "CVO");
    CHECK(r.file.appTrans == "FAF");
    CHECK(r.file.entries[2].via == "V495");
    // The text round-trips at the file level, keeping every header line and via name.
    auto r2 = parseFms(writeFms(r.file));
    REQUIRE(r2.ok);
    CHECK(writeFms(r2.file) == writeFms(r.file));
    CHECK(r2.file.sid == "BRDGE4");
    CHECK(r2.file.entries[1].via == "BRDGE4");
    CHECK(r2.file.entries[3].via == "V495");
    // And through a FlightPlan, SID/STAR lines come back from the template.
    PlanFakeDb db;
    FmsFile file;
    auto pr = loadPlanFromFmsText(t, db, &file);
    REQUIRE(pr.ok);
    CHECK(pr.plan.approachId == "I05");
    CHECK(pr.plan.approachKind == ApproachKind::Ils);
    FmsFile out = planToFms(pr.plan, db, &file);
    CHECK(out.sid == "BRDGE4");
    CHECK(out.star == "OILER2");
    CHECK(out.entries.size() == 5);
}

TEST_CASE("nav: fms ignores unknown keys with a warning") {
    std::string t = std::string("I\n1100 Version\nCYCLE 1\nFOO bar\n") +
                    "1 KAAA ADEP 0 45 -122\n1 KBBB ADES 0 43 -124\n";
    auto r = parseFms(t);
    REQUIRE(r.ok);
    CHECK(hasMsg(r.warnings, "line 4"));
    CHECK(hasMsg(r.warnings, "unknown key"));
}

TEST_CASE("nav: fms malformed files give readable errors with line numbers") {
    SUBCASE("not an fms file") {
        auto r = parseFms("hello world\n");
        CHECK_FALSE(r.ok);
        REQUIRE(r.errors.size() == 1);
        CHECK(r.errors[0].find("line 1") == 0);
        CHECK(r.errors[0].find("'I' or 'A'") != std::string::npos);
    }
    SUBCASE("unsupported version") {
        auto r = parseFms("I\n3 version\n");
        CHECK_FALSE(r.ok);
        CHECK(hasMsg(r.errors, "line 2: unsupported .fms version 3"));
    }
    SUBCASE("empty file") {
        auto r = parseFms("");
        CHECK_FALSE(r.ok);
        CHECK(hasMsg(r.errors, "empty"));
    }
    SUBCASE("missing version line") {
        auto r = parseFms("I\n");
        CHECK_FALSE(r.ok);
        CHECK(hasMsg(r.errors, "Version"));
    }
    SUBCASE("all errors in a file are reported") {
        std::string t = "I\n1100 Version\nCYCLE 1710\n"
                        "1 KAAA ADEP 0 45 -122\n"   // line 4 fine
                        "7 FOO DRCT 0 44 -123\n"    // line 5 bad type
                        "3 CVO DRCT abc 44 -123\n"  // line 6 bad altitude
                        "3 CVO DRCT 0 95 -123\n"    // line 7 bad latitude
                        "3 CVO DRCT 0 44 -190\n"    // line 8 bad longitude
                        "3 CVO DRCT 0 44\n"         // line 9 too few fields
                        "NUMENR\n"                  // line 10 missing value
                        "1 KBBB ADES 0 43 -124\n";
        auto r = parseFms(t);
        CHECK_FALSE(r.ok);
        CHECK(r.errors.size() == 6);
        CHECK(hasMsg(r.errors, "line 5: unknown waypoint type '7'"));
        CHECK(hasMsg(r.errors, "line 6: bad altitude 'abc'"));
        CHECK(hasMsg(r.errors, "line 7: bad latitude '95'"));
        CHECK(hasMsg(r.errors, "line 8: bad longitude '-190'"));
        CHECK(hasMsg(r.errors, "line 9: waypoint needs 6 fields"));
        CHECK(hasMsg(r.errors, "line 10: NUMENR needs a value"));
    }
    SUBCASE("too few waypoints") {
        auto r = parseFms("I\n1100 Version\n1 KAAA ADEP 0 45 -122\n");
        CHECK_FALSE(r.ok);
        CHECK(hasMsg(r.errors, "fewer than two waypoints"));
    }
    SUBCASE("load reports parse errors through the plan API") {
        PlanFakeDb db;
        auto r = loadPlanFromFmsText("I\n1100 Version\n1 KAAA ADEP x 45 -122\n", db);
        CHECK_FALSE(r.ok);
        CHECK(hasMsg(r.errors, "line 3: bad altitude 'x'"));
        auto m = loadPlanFromFmsFile("/nonexistent/none.fms", db);
        CHECK_FALSE(m.ok);
        CHECK(hasMsg(m.errors, "cannot open"));
    }
}

TEST_CASE("nav: fms NUMENR mismatch and missing ADEP are warnings only") {
    auto r = parseFms("I\n1100 Version\nNUMENR 9\n1 KAAA ADEP 0 45 -122\n1 KBBB ADES 0 43 -124\n");
    REQUIRE(r.ok);
    CHECK(hasMsg(r.warnings, "NUMENR says 9"));
    CHECK(hasMsg(r.warnings, "no ADEP"));
}

TEST_CASE("nav: fms lat/lon idents encode and decode") {
    CHECK(encodeLatLonIdent(36.0, -121.0) == "+36.000_-121.000");
    CHECK(encodeLatLonIdent(-33.5, 151.25) == "-33.500_+151.250");
    double lat = 0, lon = 0;
    CHECK(decodeLatLonIdent("+36.000_-121.000", lat, lon));
    CHECK(lat == doctest::Approx(36.0));
    CHECK(lon == doctest::Approx(-121.0));
    CHECK(decodeLatLonIdent("-33.5,+151.25", lat, lon));
    CHECK(lat == doctest::Approx(-33.5));
    CHECK(lon == doctest::Approx(151.25));
    CHECK(decodeLatLonIdent("+36.0-121.0", lat, lon));
    CHECK(lon == doctest::Approx(-121.0));
    CHECK_FALSE(decodeLatLonIdent("CVO", lat, lon));
    CHECK_FALSE(decodeLatLonIdent("+95.0_-121.0", lat, lon));
    CHECK(fmsRunwayToDb("RW28L") == "28L");
    CHECK(fmsRunwayToDb("28L") == "28L");
    CHECK(dbRunwayToFms("05") == "RW05");
}

TEST_CASE("nav: plan round trip through FlightPlan keeps the route") {
    PlanFakeDb db;
    auto r = loadPlanFromFmsText(kBasic, db);
    REQUIRE(r.ok);
    const FlightPlan& p = r.plan;
    CHECK(p.originIdent == "KAAA");
    CHECK(p.destIdent == "KBBB");
    CHECK(p.depRunway == "28L");
    CHECK(p.arrRunway == "05");
    CHECK(p.approachId == "I05");
    CHECK(p.cruiseAltFt == doctest::Approx(8000.0f));
    REQUIRE(p.legs.size() == 4);
    CHECK(p.legs[0].type == LegType::IF);
    CHECK(p.legs[0].to.ident == "RW28L");
    CHECK(p.legs[0].to.kind == WaypointKind::Runway);
    CHECK(p.legs[1].to.kind == WaypointKind::Vor);
    CHECK(p.legs[1].alt.kind == AltConstraintKind::At);
    CHECK(p.legs[2].to.kind == WaypointKind::LatLon);
    CHECK(p.legs[3].to.ident == "RW05");

    std::string out = writePlanFmsText(p, db);
    auto r2 = loadPlanFromFmsText(out, db);
    REQUIRE(r2.ok);
    CHECK(writePlanFmsText(r2.plan, db) == out);
    auto f1 = parseFms(out);
    auto f0 = parseFms(kBasic);
    REQUIRE(f1.ok);
    REQUIRE(f1.file.entries.size() == f0.file.entries.size());
    for (size_t i = 0; i < f0.file.entries.size(); ++i) {
        CHECK(f1.file.entries[i].ident == f0.file.entries[i].ident);
        CHECK(f1.file.entries[i].type == f0.file.entries[i].type);
        CHECK(f1.file.entries[i].altFt == doctest::Approx(f0.file.entries[i].altFt));
        CHECK(f1.file.entries[i].latDeg == doctest::Approx(f0.file.entries[i].latDeg));
        CHECK(f1.file.entries[i].lonDeg == doctest::Approx(f0.file.entries[i].lonDeg));
    }
    CHECK(f1.file.deprwy == "RW28L");
    CHECK(f1.file.desrwy == "RW05");
    CHECK(f1.file.app == "I05");
}

TEST_CASE("nav: plan sample files load and round trip") {
    SyntheticNavDb db;  // knows neither airport nor any navaid: the file coordinates are used
    for (const char* name : {"KPDX-KOTH", "KOTH-KPDX"}) {
        std::string path = std::string("assets/plans/") + name + ".fms";
        std::string text = readFile(path);
        REQUIRE_MESSAGE(!text.empty(), path);
        auto r = loadPlanFromFmsFile(path, db);
        INFO(path);
        REQUIRE(r.ok);
        CHECK(r.plan.legs.size() == 6);
        CHECK(r.plan.approachKind == ApproachKind::Ils);
        std::string out = writePlanFmsText(r.plan, db);
        auto again = loadPlanFromFmsText(out, db);
        REQUIRE(again.ok);
        CHECK(planDistNm(again.plan) == doctest::Approx(planDistNm(r.plan)));
        CHECK(again.plan.legs.size() == r.plan.legs.size());
        auto a = parseFms(text), b = parseFms(out);
        REQUIRE(a.ok);
        REQUIRE(b.ok);
        REQUIRE(a.file.entries.size() == b.file.entries.size());
        for (size_t i = 0; i < a.file.entries.size(); ++i) {
            CHECK(a.file.entries[i].ident == b.file.entries[i].ident);
            CHECK(a.file.entries[i].type == b.file.entries[i].type);
        }
    }
    auto kp = loadPlanFromFmsFile("assets/plans/KPDX-KOTH.fms", db);
    REQUIRE(kp.ok);
    CHECK(kp.plan.depRunway == "28L");
    CHECK(kp.plan.arrRunway == "05");
    CHECK(kp.plan.approachId == "I05");
    CHECK(kp.plan.cruiseAltFt == doctest::Approx(8000.0f));
    // Great-circle leg lengths of the sample route, independently computed (haversine, R = 3440.07).
    CHECK(legDistNm(kp.plan, 1) == doctest::Approx(21.35).epsilon(0.002));
    CHECK(legDistNm(kp.plan, 2) == doctest::Approx(52.98).epsilon(0.002));
    CHECK(legDistNm(kp.plan, 3) == doctest::Approx(22.94).epsilon(0.002));
    CHECK(legDistNm(kp.plan, 4) == doctest::Approx(58.93).epsilon(0.002));
    CHECK(legDistNm(kp.plan, 5) == doctest::Approx(3.40).epsilon(0.01));
    auto ko = loadPlanFromFmsFile("assets/plans/KOTH-KPDX.fms", db);
    REQUIRE(ko.ok);
    CHECK(ko.plan.cruiseAltFt == doctest::Approx(9000.0f));
    CHECK(ko.plan.arrRunway == "10R");
}

TEST_CASE("nav: plan XTST sample loads against the synthetic database") {
    SyntheticNavDb db;
    FmsFile file;
    auto r = loadPlanFromFmsFile("assets/plans/XTST-pattern.fms", db, &file);
    REQUIRE(r.ok);
    CHECK(r.warnings.empty());
    CHECK(r.plan.originIdent == "XTST");
    CHECK(r.plan.depRunway == "09");
    CHECK(r.plan.legs.size() == 5);
    CHECK(r.plan.legs[0].to.ident == "RW09");
    CHECK(r.plan.legs[0].to.kind == WaypointKind::Runway);
    CHECK(r.plan.legs[4].to.ident == "RW09");
    CHECK(r.plan.legs[2].to.kind == WaypointKind::LatLon);
    CHECK(r.plan.approachKind == ApproachKind::Ils);
    CHECK(planDistNm(r.plan) > 40.0);
    // Written text keeps type 28 idents in the +lat_+lon form.
    std::string out = writePlanFmsText(r.plan, db, &file);
    CHECK(out.find("28 +40.150_-99.700 DRCT 4000.000000 40.150000 -99.700000") !=
          std::string::npos);
    CHECK(out.find("1 XTST ADEP 2000.000000") != std::string::npos);
}

TEST_CASE("nav: plan resolves ambiguous idents by nearest to the previous waypoint") {
    PlanFakeDb db;
    // Coordinates 0/0: the file gives no position, so the previous waypoint decides.
    std::string t = "I\n1100 Version\nADEP KAAA\nADES KBBB\n"
                    "1 KAAA ADEP 0 45.0 -122.0\n"
                    "3 CVO DRCT 0 0 0\n"
                    "1 KBBB ADES 0 43.0 -124.0\n";
    auto r = loadPlanFromFmsText(t, db);
    REQUIRE(r.ok);
    CHECK(r.plan.legs[1].to.latDeg == doctest::Approx(44.5));  // Corvallis, not Cairo
    CHECK(hasMsg(r.warnings, "CVO is ambiguous"));

    std::string t3 = "I\n1100 Version\n"
                     "3 FAR DRCT 0 40.0 -118.0\n"
                     "3 CVO DRCT 0 0 0\n"
                     "1 KBBB ADES 0 43.0 -124.0\n";
    auto r3 = loadPlanFromFmsText(t3, db);
    REQUIRE(r3.ok);
    // Previous waypoint is FAR (40N 118W); Oregon CVO is much nearer than Cairo.
    CHECK(r3.plan.legs[1].to.lonDeg == doctest::Approx(-123.3));
}

TEST_CASE("nav: plan file coordinates win for ambiguous and unknown idents") {
    PlanFakeDb db;
    SUBCASE("ambiguous: file position near Cairo selects Cairo") {
        std::string t = "I\n1100 Version\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "3 CVO DRCT 0 30.09 31.39\n1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        REQUIRE(r.ok);
        CHECK(r.plan.legs[1].to.latDeg == doctest::Approx(30.09));
    }
    SUBCASE("ambiguous: file position matches no candidate") {
        std::string t = "I\n1100 Version\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "3 CVO DRCT 0 10.0 10.0\n1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        REQUIRE(r.ok);
        CHECK(r.plan.legs[1].to.latDeg == doctest::Approx(10.0));
        CHECK(hasMsg(r.warnings, "using the file's coordinates"));
    }
    SUBCASE("unknown ident uses the file position") {
        std::string t = "I\n1100 Version\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "11 ZZZZZ DRCT 5000 44.1 -123.2\n1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        REQUIRE(r.ok);
        CHECK(r.plan.legs[1].to.latDeg == doctest::Approx(44.1));
        CHECK(r.plan.legs[1].to.kind == WaypointKind::Fix);
        CHECK(hasMsg(r.warnings, "not in the navigation database"));
    }
    SUBCASE("unknown ident without position is an error") {
        std::string t = "I\n1100 Version\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "11 ZZZZZ DRCT 5000 0 0\n1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        CHECK_FALSE(r.ok);
        CHECK(hasMsg(r.errors, "line 4: unknown ZZZZZ"));
    }
    SUBCASE("single DB match far from the file position loses to the file") {
        std::string t = "I\n1100 Version\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "3 MID DRCT 0 20.0 -100.0\n1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        REQUIRE(r.ok);
        CHECK(r.plan.legs[1].to.latDeg == doctest::Approx(20.0));
    }
    SUBCASE("VOR and NDB with the same ident are told apart by entry type") {
        std::string t = "I\n1100 Version\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "2 ABC DRCT 0 44.05 -123.05\n3 ABC DRCT 0 44.0 -123.0\n"
                        "1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        REQUIRE(r.ok);
        CHECK(r.plan.legs[1].to.kind == WaypointKind::Ndb);
        CHECK(r.plan.legs[2].to.kind == WaypointKind::Vor);
        CHECK(r.plan.legs[2].to.latDeg == doctest::Approx(44.0));
    }
    SUBCASE("unknown runway falls back to the airport with a warning") {
        std::string t = "I\n1100 Version\nADEP KAAA\nDEPRWY RW99\n1 KAAA ADEP 0 45.0 -122.0\n"
                        "1 KBBB ADES 0 43.0 -124.0\n";
        auto r = loadPlanFromFmsText(t, db);
        REQUIRE(r.ok);
        CHECK(r.plan.legs[0].to.kind == WaypointKind::Airport);
        CHECK(hasMsg(r.warnings, "runway 99 not found"));
    }
}

TEST_CASE("nav: plan resolveIdent lists matches nearest first") {
    PlanFakeDb db;
    auto m = resolveIdent(db, "cvo", 44.0, -123.0);
    REQUIRE(m.size() == 2);
    CHECK(m[0].wp.latDeg == doctest::Approx(44.5));
    CHECK(m[0].distNm < m[1].distNm);
    CHECK(m[0].description.find("VOR") == 0);
    auto k = resolveIdent(db, "KAAA", 0, 0);
    REQUIRE(k.size() == 1);
    CHECK(k[0].wp.kind == WaypointKind::Airport);
    CHECK(resolveIdent(db, "NOPE", 0, 0).empty());
    CHECK(resolveIdent(db, "WPT01", 44.0, -122.0).front().wp.latDeg == doctest::Approx(44.2));
}

TEST_CASE("nav: plan approach kinds from CIFP ids") {
    CHECK(approachKindFromId("I05") == ApproachKind::Ils);
    CHECK(approachKindFromId("R28L") == ApproachKind::Rnav);
    CHECK(approachKindFromId("L05") == ApproachKind::Loc);
    CHECK(approachKindFromId("V10") == ApproachKind::Vor);
    CHECK(approachKindFromId("N35") == ApproachKind::Ndb);
    CHECK(approachKindFromId("") == ApproachKind::None);
}
