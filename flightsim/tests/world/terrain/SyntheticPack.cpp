#include "world/terrain/SyntheticPack.h"

#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>

namespace fsim::terrain::testing {
namespace fs = std::filesystem;

TempDir::TempDir() {
    std::string tmpl = (fs::temp_directory_path() / "fsim_terrain_XXXXXX").string();
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    if (!mkdtemp(buf.data())) std::abort();
    path = buf.data();
}

TempDir::~TempDir() {
    std::error_code ec;
    fs::remove_all(path, ec);
}

bool writeFile(const std::string& path, const void* data, size_t size) {
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::FILE* f = std::fopen(path.c_str(), "wb");
    if (!f) return false;
    const size_t n = size ? std::fwrite(data, 1, size, f) : 0;
    std::fclose(f);
    return n == size;
}

PackWriter::PackWriter(const std::string& dir, int minLevel, int maxLevel)
    : dir_(dir), minLevel_(minLevel), maxLevel_(maxLevel) {
    bitmaps_.resize(static_cast<size_t>(maxLevel) + 1);
    for (int k = 0; k <= maxLevel; ++k)
        bitmaps_[k].assign(static_cast<size_t>(tileCountAtLevel(k)) / 8, 0);  // 4^k bytes
}

double PackWriter::sampleLatDeg(const TileId& id, int row) {
    return -90.0 + (static_cast<double>(id.y + 1) * kTileCells - row) * stepDeg(id.k);
}

double PackWriter::sampleLonDeg(const TileId& id, int col) {
    return -180.0 + (static_cast<double>(id.x) * kTileCells + col) * stepDeg(id.k);
}

void PackWriter::markPresent(const TileId& id) {
    const uint64_t i = bitIndex(id);
    bitmaps_[id.k][i >> 3] |= static_cast<uint8_t>(1u << (i & 7));
}

void PackWriter::writeTileRaw(const TileId& id, const std::vector<int16_t>& heights,
                              const std::vector<uint8_t>* land) {
    std::vector<uint8_t> raw(heights.size() * 2);
    for (size_t i = 0; i < heights.size(); ++i) {
        const uint16_t u = static_cast<uint16_t>(heights[i]);
        raw[2 * i] = static_cast<uint8_t>(u & 0xFF);
        raw[2 * i + 1] = static_cast<uint8_t>(u >> 8);
    }
    const std::string tail =
        std::to_string(id.k) + "/" + std::to_string(id.x) + "/" + std::to_string(id.y);
    writeFile(dir_ + "/h/" + tail + ".h16", raw.data(), raw.size());
    if (land) writeFile(dir_ + "/c/" + tail + ".lc8", land->data(), land->size());
    markPresent(id);
}

std::vector<int16_t> PackWriter::sampleHeights(const TileId& id, const HeightFn& height) {
    std::vector<int16_t> h(static_cast<size_t>(kTileSamples) * kTileSamples);
    for (int r = 0; r < kTileSamples; ++r)
        for (int c = 0; c < kTileSamples; ++c) {
            const double v = height(sampleLatDeg(id, r), sampleLonDeg(id, c));
            h[r * kTileSamples + c] =
                static_cast<int16_t>(std::lround(std::min(32767.0, std::max(-32767.0, v))));
        }
    return h;
}

void PackWriter::writeTile(const TileId& id, const HeightFn& height, const LandFn& land) {
    const std::vector<int16_t> h = sampleHeights(id, height);
    if (!land) {
        writeTileRaw(id, h, nullptr);
        return;
    }
    std::vector<uint8_t> lc(static_cast<size_t>(kTileCells) * kTileCells);
    const double step = stepDeg(id.k);
    for (int r = 0; r < kTileCells; ++r)
        for (int c = 0; c < kTileCells; ++c) {
            const double lat = sampleLatDeg(id, r) - 0.5 * step;
            const double lon = sampleLonDeg(id, c) + 0.5 * step;
            lc[r * kTileCells + c] = land(lat, lon);
        }
    writeTileRaw(id, h, &lc);
}

void PackWriter::writeBox(int k, double southDeg, double westDeg, double northDeg, double eastDeg,
                          const HeightFn& height, const LandFn& land) {
    const TileId a = tileAt(k, southDeg, westDeg), b = tileAt(k, northDeg, eastDeg);
    for (uint32_t y = a.y; y <= b.y; ++y)
        for (uint32_t x = a.x; x <= b.x; ++x) writeTile(TileId{k, x, y}, height, land);
}

void PackWriter::finish() {
    std::string ini = "; synthetic test pack\n"
                      "[pack]\n"
                      "format = flightsim-terrain\n"
                      "version = 1\n"
                      "samples = 257\n"
                      "min_level = " +
                      std::to_string(minLevel_) +
                      "\n"
                      "max_level = " +
                      std::to_string(maxLevel_) +
                      "\n"
                      "created_utc = 2026-10-19T12:00:00Z\n"
                      "[attribution]\n"
                      "dem = Test DEM; (c) nobody # not a comment\n"
                      "landcover = Test landcover\n"
                      "landcover_credit = \xC2\xA9 Test credit; with semicolon\n"
                      "[regions]\n"
                      "test = levels " +
                      std::to_string(minLevel_) + "-" + std::to_string(maxLevel_) + "\n";
    writeFile(dir_ + "/pack.ini", ini.data(), ini.size());
    for (int k = 0; k <= maxLevel_; ++k) {
        const std::vector<uint8_t>& bits = bitmaps_[k];
        if (std::all_of(bits.begin(), bits.end(), [](uint8_t b) { return b == 0; })) continue;
        writeFile(dir_ + "/index/L" + std::to_string(k) + ".bit", bits.data(), bits.size());
    }
}

}  // namespace fsim::terrain::testing
