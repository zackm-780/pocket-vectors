#include <doctest/doctest.h>

#include <cstdio>
#include <fstream>
#include <string>

#include "core/Csv.h"
#include "core/Ini.h"

using namespace fsim;

TEST_CASE("ini: sections, typed getters, defaults and case-insensitivity") {
    Ini ini;
    ini.loadText("; leading comment\n"
                 "top = level\n"
                 "[Scenario]\n"
                 "aircraft = c172p\n"
                 "start = final\n"
                 "distance_nm = 6.5\n"
                 "count = 12\n"
                 "flag = yes\n"
                 "off = 0\n"
                 "empty =\n"
                 "\n"
                 "# hash comment\n"
                 "[wind]\n"
                 "from_true_deg = 270\n");
    CHECK(ini.getString("", "top") == "level");
    CHECK(ini.getString("scenario", "aircraft") == "c172p");
    CHECK(ini.getString("SCENARIO", "AIRCRAFT") == "c172p");
    CHECK(ini.getString("scenario", "start") == "final");
    CHECK(ini.getDouble("scenario", "distance_nm") == doctest::Approx(6.5));
    CHECK(ini.getInt("scenario", "count") == 12);
    CHECK(ini.getBool("scenario", "flag") == true);
    CHECK(ini.getBool("scenario", "off", true) == false);
    CHECK(ini.getFloat("wind", "from_true_deg") == doctest::Approx(270.0f));

    CHECK(ini.has("scenario", "empty"));
    CHECK(ini.getString("scenario", "empty") == "");
    CHECK(ini.getInt("scenario", "empty", 7) == 7);
    CHECK_FALSE(ini.has("scenario", "missing"));
    CHECK(ini.getInt("scenario", "missing", 42) == 42);
    CHECK(ini.getString("nosuch", "key", "dflt") == "dflt");
    CHECK(ini.hasSection("wind"));
    CHECK_FALSE(ini.hasSection("nosuch"));
}

TEST_CASE("ini: values are verbatim UTF-8 with no inline comments") {
    Ini ini;
    ini.loadText("[dem]\n"
                 "dem =   \xC2\xA9 DLR e.V. 2010-2014 and \xC2\xA9 Airbus; provided under "
                 "COPERNICUS # terms  \n"
                 "; a comment line\n"
                 "  # an indented comment line\n"
                 "plain = a ;b\n");
    CHECK(ini.getString("dem", "dem") ==
          "\xC2\xA9 DLR e.V. 2010-2014 and \xC2\xA9 Airbus; provided under COPERNICUS # terms");
    CHECK(ini.getString("dem", "plain") == "a ;b");
    CHECK(ini.entries("dem").size() == 2);  // comment lines are not entries
}

TEST_CASE("ini: ordered entries keep numeric keys and duplicates") {
    Ini ini;
    ini.loadText("[inputs]\n"
                 "0.0  = throttle 1.0\n"
                 "12.0 = elevator 0.3\n"
                 "12.0 = rudder -0.1\n"
                 "[other]\nk = v\n"
                 "[inputs]\n"  // a repeated section header continues the same section
                 "20 = flaps 1\n");
    const auto& e = ini.entries("inputs");
    REQUIRE(e.size() == 4);
    CHECK(e[0].first == "0.0");
    CHECK(e[0].second == "throttle 1.0");
    CHECK(e[2].second == "rudder -0.1");
    CHECK(e[3].first == "20");
    CHECK(ini.entries("nosuch").empty());
}

TEST_CASE("ini: loadFile reports a missing file") {
    Ini ini;
    CHECK_FALSE(ini.loadFile("/nonexistent/path/file.ini"));
}

TEST_CASE("ini: splitTrim and trim") {
    const auto parts = splitTrim(" a , b,,c ", ',');
    REQUIRE(parts.size() == 4);
    CHECK(parts[0] == "a");
    CHECK(parts[2] == "");
    CHECK(parts[3] == "c");
    CHECK(trim("  \t x y \n") == "x y");
    CHECK(toLower("AbC") == "abc");
}

TEST_CASE("csv: header-indexed reading with quotes, comments and blank lines") {
    CsvReader csv;
    REQUIRE(csv.openText("ident,name,lat,elev_ft\n"
                         "# a comment line\n"
                         "XTST,\"Test, Airport\",40.0,2000\n"
                         "\n"
                         "KSFO,\"San \"\"Fran\"\" Intl\",37.62,13\r\n"
                         "NODATA,x\n"));
    CHECK(csv.column("lat") == 2);
    CHECK(csv.column("nope") == -1);

    REQUIRE(csv.next());
    CHECK(csv.field("ident") == "XTST");
    CHECK(csv.field("name") == "Test, Airport");
    CHECK(csv.getDouble("lat") == doctest::Approx(40.0));
    CHECK(csv.getInt("elev_ft") == 2000);

    REQUIRE(csv.next());
    CHECK(csv.field("name") == "San \"Fran\" Intl");
    CHECK(csv.getInt("elev_ft") == 13);  // CRLF handled

    REQUIRE(csv.next());  // short row: missing columns read as empty / default
    CHECK(csv.field("ident") == "NODATA");
    CHECK(csv.getDouble("lat", -1.0) == doctest::Approx(-1.0));
    CHECK_FALSE(csv.next());
}

TEST_CASE("csv: writer output reads back") {
    const std::string path = "csv_roundtrip_test.csv";
    {
        CsvWriter w;
        REQUIRE(w.open(path, {"t_s", "label", "n"}));
        w.addDouble(1.25, 2);
        w.addString("a,b \"q\"");
        w.addInt(-7);
        w.endRow();
        w.addDouble(2.5, 1);
        w.addString("plain");
        w.addInt(8);
        w.endRow();
    }
    CsvReader r;
    REQUIRE(r.open(path));
    REQUIRE(r.next());
    CHECK(r.getDouble("t_s") == doctest::Approx(1.25));
    CHECK(r.field("label") == "a,b \"q\"");
    CHECK(r.getInt("n") == -7);
    REQUIRE(r.next());
    CHECK(r.field("label") == "plain");
    CHECK_FALSE(r.next());
    std::remove(path.c_str());
}
