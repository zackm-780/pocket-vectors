// Terrain pack reader: pack.ini, presence bitmaps, .h16 / .lc8 decoding.
#include <doctest/doctest.h>

#include "world/terrain/SyntheticPack.h"
#include "world/terrain/TerrainPack.h"

using namespace fsim;
using namespace fsim::terrain;
using namespace fsim::terrain::testing;

namespace {

void writeText(const std::string& path, const std::string& text) {
    REQUIRE(writeFile(path, text.data(), text.size()));
}

const char* kGoodIni =
    "[pack]\nformat = flightsim-terrain\nversion = 1\nsamples = 257\nmin_level = 3\n"
    "max_level = 5\n";

}  // namespace

TEST_CASE("terrain: pack.ini is read verbatim (INI rule) and levels are discovered") {
    TempDir dir;
    PackWriter w(dir.path, 5, 6);
    w.writeTile(TileId{5, 20, 48}, [](double, double) { return 10.0; });
    w.writeTile(TileId{6, 40, 96}, [](double, double) { return 10.0; });
    w.finish();

    TerrainPack pack;
    std::string error;
    REQUIRE(pack.open(dir.path, error));
    CHECK(pack.isOpen());
    CHECK(pack.info().format == "flightsim-terrain");
    CHECK(pack.info().versionMajor == 1);
    CHECK(pack.info().samples == 257);
    CHECK(pack.info().minLevel == 5);
    CHECK(pack.info().maxLevel == 6);
    // Values keep ';', '#' and UTF-8; there are no inline comments.
    CHECK(pack.info().demAttribution == "Test DEM; (c) nobody # not a comment");
    CHECK(pack.info().landcoverCredit == "\xC2\xA9 Test credit; with semicolon");
    REQUIRE(pack.info().regions.size() == 1);
    CHECK(pack.info().regions[0] == "test = levels 5-6");
    CHECK(pack.finestLevel() == 6);
    CHECK(pack.coarsestLevel() == 5);
    CHECK(pack.tileCount(5) == 1);
    CHECK(pack.tileCount(6) == 1);
    CHECK(pack.tileCount(7) == 0);
    CHECK(pack.hasTile(TileId{5, 20, 48}));
    CHECK(!pack.hasTile(TileId{5, 21, 48}));
    CHECK(pack.finestLevelAt(45.5, -123.0) == 6);  // inside the L6 tile (and its L5 parent)
    CHECK(pack.finestLevelAt(47.0, -121.5) == 5);   // only the L5 tile covers it
    CHECK(pack.finestLevelAt(10.0, 10.0) == -1);    // no tile covers it
}

TEST_CASE("terrain: presence bitmaps are LSB-first, row-major with y*width + x") {
    TempDir dir;
    writeText(dir.sub("pack.ini"), kGoodIni);
    // Level 3: 32 x 16 tiles = 64 bytes. Bits 0 and 2 of byte 0 -> (0,0) and (2,0).
    // Tile (3,1) in row 1 is bit 32 + 3 = 35 -> byte 4, bit 3. Last tile (31,15) = bit 511.
    std::vector<uint8_t> bits(64, 0);
    bits[0] = 0b00000101;
    bits[4] = 0b00001000;
    bits[63] = 0b10000000;
    REQUIRE(writeFile(dir.sub("index/L3.bit"), bits.data(), bits.size()));

    TerrainPack pack;
    std::string error;
    REQUIRE(pack.open(dir.path, error));
    CHECK(pack.hasTile(TileId{3, 0, 0}));
    CHECK(!pack.hasTile(TileId{3, 1, 0}));
    CHECK(pack.hasTile(TileId{3, 2, 0}));
    CHECK(pack.hasTile(TileId{3, 3, 1}));
    CHECK(!pack.hasTile(TileId{3, 4, 1}));
    CHECK(pack.hasTile(TileId{3, 31, 15}));
    CHECK(!pack.hasTile(TileId{3, 30, 15}));
    CHECK(pack.tileCount(3) == 4);
    // Out of range ids and levels without a bitmap simply have no tiles.
    CHECK(!pack.hasTile(TileId{3, 32, 0}));
    CHECK(!pack.hasTile(TileId{4, 0, 0}));
    CHECK(!pack.hasTile(TileId{-1, 0, 0}));
    CHECK(!pack.levelHasTiles(4));
}

TEST_CASE("terrain: a wrong-sized bitmap is ignored, not trusted") {
    TempDir dir;
    writeText(dir.sub("pack.ini"), kGoodIni);
    std::vector<uint8_t> bad(63, 0xFF);  // one byte short for level 3
    REQUIRE(writeFile(dir.sub("index/L3.bit"), bad.data(), bad.size()));
    std::vector<uint8_t> good(16, 0x01);  // level 2 = 4^2 bytes
    REQUIRE(writeFile(dir.sub("index/L2.bit"), good.data(), good.size()));
    TerrainPack pack;
    std::string error;
    REQUIRE(pack.open(dir.path, error));
    CHECK(!pack.levelHasTiles(3));
    CHECK(pack.levelHasTiles(2));
    CHECK(pack.finestLevel() == 2);
}

TEST_CASE("terrain: pack.ini validation rejects unknown formats and versions") {
    struct Bad {
        const char* ini;
        const char* needle;
    };
    const Bad bad[] = {
        {"[pack]\nformat = other\nversion = 1\nsamples = 257\nmax_level = 3\n", "format"},
        {"[pack]\nformat = flightsim-terrain\nversion = 2\nsamples = 257\nmax_level = 3\n",
         "version"},
        {"[pack]\nformat = flightsim-terrain\nversion = 1\nsamples = 129\nmax_level = 3\n",
         "samples"},
        {"[pack]\nformat = flightsim-terrain\nversion = 1\nsamples = 257\nmin_level = 5\n"
         "max_level = 3\n",
         "level"},
    };
    for (const Bad& b : bad) {
        TempDir dir;
        writeText(dir.sub("pack.ini"), b.ini);
        TerrainPack pack;
        std::string error;
        CHECK(!pack.open(dir.path, error));
        CHECK(!pack.isOpen());
        CHECK(error.find(b.needle) != std::string::npos);
    }
    TempDir empty;
    TerrainPack pack;
    std::string error;
    CHECK(!pack.open(empty.path, error));
    CHECK(error.find("pack.ini") != std::string::npos);
    // A 1.x minor version is accepted.
    TempDir minor;
    writeText(minor.sub("pack.ini"),
              "[pack]\nformat = flightsim-terrain\nversion = 1.2\nsamples = 257\nmax_level = 3\n");
    CHECK(pack.open(minor.path, error));
}

TEST_CASE("terrain: .h16 decoding is little-endian, north-up, and strict about size") {
    TempDir dir;
    std::vector<uint8_t> raw(132098, 0);
    raw[0] = 0x34;
    raw[1] = 0x12;  // 0x1234 = 4660
    raw[2] = 0x00;
    raw[3] = 0x80;  // -32768 (no data)
    raw[4] = 0xFF;
    raw[5] = 0xFF;  // -1
    const size_t last = 257 * 257 - 1;
    raw[2 * last] = 0x10;
    raw[2 * last + 1] = 0x27;  // 10000
    REQUIRE(writeFile(dir.sub("a.h16"), raw.data(), raw.size()));

    std::vector<int16_t> h;
    std::string error;
    REQUIRE(readHeightFile(dir.sub("a.h16"), h, error));
    REQUIRE(h.size() == 257u * 257u);
    CHECK(h[0] == 4660);
    CHECK(h[1] == -32768);
    CHECK(h[2] == -1);
    CHECK(h[last] == 10000);

    raw.pop_back();
    REQUIRE(writeFile(dir.sub("short.h16"), raw.data(), raw.size()));
    CHECK(!readHeightFile(dir.sub("short.h16"), h, error));
    CHECK(error.find("132098") != std::string::npos);
    CHECK(!readHeightFile(dir.sub("missing.h16"), h, error));
    CHECK(error.find("missing") != std::string::npos);
}

TEST_CASE("terrain: .lc8 is optional and strict about size") {
    TempDir dir;
    std::vector<uint8_t> lc(65536, 4);
    REQUIRE(writeFile(dir.sub("a.lc8"), lc.data(), lc.size()));
    std::vector<uint8_t> out;
    std::string error;
    REQUIRE(readLandClassFile(dir.sub("a.lc8"), out, error));
    CHECK(out.size() == 65536u);
    CHECK(out[1234] == 4);

    CHECK(!readLandClassFile(dir.sub("none.lc8"), out, error));
    CHECK(error.empty());  // a missing .lc8 is not an error

    lc.resize(100);
    REQUIRE(writeFile(dir.sub("bad.lc8"), lc.data(), lc.size()));
    CHECK(!readLandClassFile(dir.sub("bad.lc8"), out, error));
    CHECK(!error.empty());
}

TEST_CASE("terrain: loadTile builds tiles with and without land class") {
    TempDir dir;
    PackWriter w(dir.path, 5, 5);
    const TileId withLc{5, 20, 48}, withoutLc{5, 21, 48}, broken{5, 22, 48};
    w.writeTile(withLc, [](double, double) { return 123.0; }, [](double, double) { return 6; });
    w.writeTile(withoutLc, [](double, double) { return -7.0; });
    w.markPresent(broken);  // listed but no file
    w.finish();

    TerrainPack pack;
    std::string error;
    REQUIRE(pack.open(dir.path, error));
    auto a = pack.loadTile(withLc, true, error);
    REQUIRE(static_cast<bool>(a));
    CHECK(a->hasLandClass());
    CHECK(a->heightAtM(100, 100) == 123);
    CHECK(a->landClassCell(10, 10) == 6);
    CHECK(a->minHeightM == 123);
    CHECK(a->maxHeightM == 123);
    CHECK(!a->hasVoids);
    CHECK(a->memoryBytes() > 132098u);

    auto noLcWanted = pack.loadTile(withLc, false, error);
    REQUIRE(static_cast<bool>(noLcWanted));
    CHECK(!noLcWanted->hasLandClass());

    auto b = pack.loadTile(withoutLc, true, error);
    REQUIRE(static_cast<bool>(b));
    CHECK(!b->hasLandClass());
    CHECK(b->heightAtM(0, 0) == -7);
    CHECK(b->landClassAt(48.0, -122.0) == LandClass::Unknown);

    CHECK(pack.loadTile(broken, true, error).get() == nullptr);
    CHECK(!error.empty());
}
