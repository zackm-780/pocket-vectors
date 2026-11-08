// W8a: taxiways.csv and aprons.csv (DATA.md 5.8-5.10) load into Airport::taxiways / aprons. The
// rows in tests/fixtures/nav/{taxiways,aprons}.csv are synthetic (no OSM data), at KPDX and KOTH.
#include <doctest/doctest.h>

#include <filesystem>
#include <string>
#include <unistd.h>

#include "nav/NavDb.h"

using namespace fsim;
namespace fs = std::filesystem;

namespace {

std::string fixtureDir() {
    std::string f = __FILE__;  // .../tests/nav/navdb_taxiways_test.cpp
    f = f.substr(0, f.find_last_of('/'));
    f = f.substr(0, f.find_last_of('/'));
    return f + "/fixtures/nav";
}

const CsvNavDatabase& db() {
    static CsvNavDatabase d;
    static bool loaded = false;
    if (!loaded) {
        std::string err;
        REQUIRE_MESSAGE(d.load(fixtureDir(), err), err);
        loaded = true;
    }
    return d;
}

}  // namespace

TEST_CASE("nav: taxiway segments load with kind, ref, width, surface and points in order") {
    const Airport* kpdx = db().findAirport("KPDX");
    REQUIRE(kpdx != nullptr);
    REQUIRE(kpdx->taxiways.size() == 3);
    const Taxiway& c1 = kpdx->taxiways[0];
    CHECK(c1.osmWayId == 1001);
    CHECK(c1.kind == TaxiwayKind::Taxiway);
    CHECK(c1.ref == "C");
    CHECK(c1.widthM == doctest::Approx(23.0f));
    CHECK(c1.hard);
    REQUIRE(c1.points.size() == 3);
    CHECK(c1.points[0].latDeg == doctest::Approx(45.59));
    CHECK(c1.points[0].lonDeg == doctest::Approx(-122.6));
    CHECK(c1.points[2].latDeg == doctest::Approx(45.591));
    CHECK(c1.points[2].lonDeg == doctest::Approx(-122.5985));
    // The second segment of the same way starts where the first ends.
    const Taxiway& c2 = kpdx->taxiways[1];
    CHECK(c2.osmWayId == 1001);
    REQUIRE(c2.points.size() == 2);
    CHECK(c2.points[0].latDeg == c1.points[2].latDeg);
    CHECK(c2.points[0].lonDeg == c1.points[2].lonDeg);
    // A taxilane without a width gets the taxilane default; concrete is hard.
    const Taxiway& lane = kpdx->taxiways[2];
    CHECK(lane.kind == TaxiwayKind::Taxilane);
    CHECK(lane.ref.empty());
    CHECK(lane.widthM == doctest::Approx(7.5f));
    CHECK(lane.hard);

    const Airport* koth = db().findAirport("KOTH");
    REQUIRE(koth != nullptr);
    REQUIRE(koth->taxiways.size() == 1);  // the five bad KOTH rows are skipped
    CHECK(koth->taxiways[0].widthM == doctest::Approx(15.0f));
    CHECK_FALSE(koth->taxiways[0].hard);  // grass
}

TEST_CASE("nav: aprons load as open rings; bad rows are skipped and counted") {
    const Airport* kpdx = db().findAirport("KPDX");
    REQUIRE(kpdx != nullptr);
    REQUIRE(kpdx->aprons.size() == 1);
    const Apron& a = kpdx->aprons[0];
    CHECK(a.osmWayId == 4001);
    CHECK(a.hard);
    REQUIRE(a.points.size() == 4);
    CHECK(a.points[3].latDeg == doctest::Approx(45.589));
    CHECK(a.points[3].lonDeg == doctest::Approx(-122.596));

    const Airport* koth = db().findAirport("KOTH");
    REQUIRE(koth != nullptr);
    REQUIRE(koth->aprons.size() == 1);  // the two-point ring is skipped
    CHECK_FALSE(koth->aprons[0].hard);

    CHECK(db().stats().taxiways == 4);
    CHECK(db().stats().aprons == 2);
    // Unknown airport, odd point count, one point, three bad numbers ("abc", "43.417000x", "nan")
    // and a two-point ring.
    CHECK(db().stats().skippedRows >= 7);
    // Airports without rows and XTST keep empty lists.
    const Airport* ksfo = db().findAirport("KSFO");
    REQUIRE(ksfo != nullptr);
    CHECK(ksfo->taxiways.empty());
    CHECK(ksfo->aprons.empty());
    CHECK(db().findAirport("XTST")->taxiways.empty());
}

TEST_CASE("nav: a pack without taxiways.csv and aprons.csv loads unchanged") {
    const fs::path dir =
        fs::temp_directory_path() / ("fsim_navdb_taxiways_" + std::to_string(::getpid()));
    fs::remove_all(dir);
    fs::create_directories(dir);
    for (const char* f : {"airports.csv", "runways.csv"})
        fs::copy_file(fs::path(fixtureDir()) / f, dir / f);

    CsvNavDatabase old;
    std::string err;
    REQUIRE_MESSAGE(old.load(dir.string(), err), err);
    CHECK(old.airportCount() == db().airportCount());
    CHECK(old.stats().runways == db().stats().runways);
    CHECK(old.stats().taxiways == 0);
    CHECK(old.stats().aprons == 0);
    for (const char* ident : {"KPDX", "KOTH", "KSFO"}) {
        const Airport* a = old.findAirport(ident);
        REQUIRE(a != nullptr);
        CHECK(a->taxiways.empty());
        CHECK(a->aprons.empty());
        CHECK(a->runways.size() == db().findAirport(ident)->runways.size());
    }
    fs::remove_all(dir);
}
