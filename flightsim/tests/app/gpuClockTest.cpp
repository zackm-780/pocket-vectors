// X3c: GpuClock against a fake /sys/class/devfreq tree in a temporary folder.
#include <doctest/doctest.h>
#include <unistd.h>

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <string>

#include "app/GpuClock.h"

using namespace fsim;
namespace fs = std::filesystem;

namespace {

struct FakeDevfreq {
    fs::path root;
    FakeDevfreq() {
        root =
            fs::temp_directory_path() / ("fsim_devfreq_test_" + std::to_string(::getpid()) + "_" +
                                         std::to_string(reinterpret_cast<std::uintptr_t>(this)));
        fs::remove_all(root);
        fs::create_directories(root);
    }
    ~FakeDevfreq() { fs::remove_all(root); }
    void file(const std::string& entry, const std::string& name, const std::string& text) {
        fs::create_directories(root / entry);
        std::ofstream(root / entry / name) << text;
    }
};

}  // namespace

TEST_CASE("app: devfreq Hz text becomes MHz") {
    CHECK(hzTextToMhz("648000000\n") == 648);
    CHECK(hzTextToMhz("  432000000  ") == 432);
    CHECK(hzTextToMhz("299999999") == 300);  // rounded
    CHECK(hzTextToMhz("") == -1);
    CHECK(hzTextToMhz("abc") == -1);
    const std::vector<int> list = frequencyListMhz("216000000 312000000 432000000 648000000\n");
    REQUIRE(list.size() == 4);
    CHECK(list[0] == 216);
    CHECK(list[3] == 648);
    CHECK(frequencyListMhz("").empty());
}

TEST_CASE("app: the GPU devfreq entry is the one named gpu or mali, else the only one") {
    CHECK(pickGpuEntry({"dmc", "1800000.gpu"}) == 1);
    CHECK(pickGpuEntry({"ff9a0000.Mali", "dmc"}) == 0);
    CHECK(pickGpuEntry({"devfreq0"}) == 0);
    CHECK(pickGpuEntry({"dmc", "devfreq1"}) == -1);
    CHECK(pickGpuEntry({}) == -1);
}

TEST_CASE("app: GpuClock reads cur, min, max, the list and the governor") {
    FakeDevfreq t;
    t.file("dmc", "cur_freq", "792000000\n");
    t.file("1800000.gpu", "cur_freq", "432000000\n");
    t.file("1800000.gpu", "min_freq", "216000000\n");
    t.file("1800000.gpu", "max_freq", "648000000\n");
    t.file("1800000.gpu", "available_frequencies", "216000000 312000000 432000000 648000000\n");
    t.file("1800000.gpu", "governor", "simple_ondemand\n");
    GpuClock g;
    REQUIRE(g.open(t.root.string()));
    CHECK(g.entryName() == "1800000.gpu");
    CHECK(g.curMhz() == 432);
    CHECK(g.minMhz() == 216);
    CHECK(g.maxMhz() == 648);
    CHECK(g.availableMhz().size() == 4);
    CHECK(g.governor() == "simple_ondemand");
    CHECK(g.windowTag() == " | gpu 432/648 MHz");
    CHECK(g.describe() == "gpu clock: 1800000.gpu simple_ondemand cur 432 MHz (min 216 max 648; "
                          "available 216 312 432 648)");

    // The clock is read again on each call (the governor moves it).
    t.file("1800000.gpu", "cur_freq", "648000000\n");
    CHECK(g.curMhz() == 648);
    CHECK(g.windowTag() == " | gpu 648/648 MHz");
}

TEST_CASE("app: GpuClock with missing files or no devfreq folder is quiet") {
    FakeDevfreq t;
    t.file("1800000.gpu", "cur_freq", "300000000\n");  // nothing else
    GpuClock g;
    REQUIRE(g.open(t.root.string()));
    CHECK(g.curMhz() == 300);
    CHECK(g.maxMhz() == -1);
    CHECK(g.availableMhz().empty());
    CHECK(g.windowTag() == " | gpu 300 MHz");
    CHECK(g.describe() == "gpu clock: 1800000.gpu ? cur 300 MHz (min -1 max -1; available ?)");

    GpuClock none;
    CHECK_FALSE(none.open((t.root / "no_such_folder").string()));
    CHECK_FALSE(none.available());
    CHECK(none.curMhz() == -1);
    CHECK(none.windowTag().empty());
    CHECK(none.describe().empty());

    FakeDevfreq empty;
    GpuClock e;
    CHECK_FALSE(e.open(empty.root.string()));
    CHECK(e.describe().empty());
}
