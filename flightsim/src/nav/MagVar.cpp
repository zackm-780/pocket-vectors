#include "nav/MagVar.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace fsim {
namespace {

constexpr size_t kHeaderBytes = 24;  // "MAGVAR01" + float32 + 6 x int16
constexpr float kCentiPerDeg = 100.0f;

int16_t readI16(const uint8_t* p) {
    return static_cast<int16_t>(static_cast<uint16_t>(p[0] | (p[1] << 8)));
}

}  // namespace

bool MagVar::loadFile(const std::string& path, std::string& error) {
    std::FILE* f = std::fopen(path.c_str(), "rb");
    if (!f) {
        grid_.clear();
        error = "cannot open " + path;
        return false;
    }
    std::vector<uint8_t> bytes;
    uint8_t buf[65536];
    size_t n;
    while ((n = std::fread(buf, 1, sizeof buf, f)) > 0) bytes.insert(bytes.end(), buf, buf + n);
    std::fclose(f);
    return loadBytes(bytes.data(), bytes.size(), error);
}

bool MagVar::loadBytes(const uint8_t* data, size_t size, std::string& error) {
    grid_.clear();
    if (size < kHeaderBytes || std::memcmp(data, "MAGVAR01", 8) != 0) {
        error = "magvar: bad header (expected MAGVAR01)";
        return false;
    }
    float epoch;
    const uint32_t bits = static_cast<uint32_t>(data[8]) | (static_cast<uint32_t>(data[9]) << 8) |
                          (static_cast<uint32_t>(data[10]) << 16) |
                          (static_cast<uint32_t>(data[11]) << 24);
    std::memcpy(&epoch, &bits, sizeof epoch);
    const int latMin = readI16(data + 12), latMax = readI16(data + 14);
    const int lonMin = readI16(data + 16), lonMax = readI16(data + 18);
    const int step = readI16(data + 20);
    if (step < 1 || latMax <= latMin || lonMax <= lonMin || (latMax - latMin) % step != 0 ||
        (lonMax - lonMin) % step != 0) {
        error = "magvar: inconsistent grid header";
        return false;
    }
    const int rows = (latMax - latMin) / step + 1;
    const int cols = (lonMax - lonMin) / step + 1;
    const size_t count = static_cast<size_t>(rows) * static_cast<size_t>(cols);
    if (size != kHeaderBytes + count * 2) {
        error = "magvar: file size does not match the grid in the header";
        return false;
    }
    grid_.resize(count);
    for (size_t i = 0; i < count; ++i) grid_[i] = readI16(data + kHeaderBytes + i * 2);
    epochYear_ = epoch;
    latMinDeg_ = latMin;
    latMaxDeg_ = latMax;
    lonMinDeg_ = lonMin;
    lonMaxDeg_ = lonMax;
    stepDeg_ = step;
    rows_ = rows;
    cols_ = cols;
    wrapLon_ = (lonMax - lonMin) >= 360;
    return true;
}

float MagVar::nodeDeg(int row, int col) const {
    if (row < 0 || col < 0 || row >= rows_ || col >= cols_) return 0.0f;
    return static_cast<float>(grid_[static_cast<size_t>(row) * static_cast<size_t>(cols_) +
                                    static_cast<size_t>(col)]) /
           kCentiPerDeg;
}

float MagVar::declinationDeg(double latDeg, double lonDeg) const {
    if (grid_.empty()) return 0.0f;
    const double lat =
        std::clamp(latDeg, static_cast<double>(latMinDeg_), static_cast<double>(latMaxDeg_));
    double lon = lonDeg;
    if (wrapLon_) {
        const double span = lonMaxDeg_ - lonMinDeg_;
        lon = std::fmod(lon - lonMinDeg_, span);
        if (lon < 0) lon += span;
        lon += lonMinDeg_;
    } else {
        lon = std::clamp(lon, static_cast<double>(lonMinDeg_), static_cast<double>(lonMaxDeg_));
    }
    const double fy = (lat - latMinDeg_) / stepDeg_;
    const double fx = (lon - lonMinDeg_) / stepDeg_;
    const int j0 = std::min(static_cast<int>(std::floor(fy)), rows_ - 2);
    const int i0 = std::min(static_cast<int>(std::floor(fx)), cols_ - 2);
    const double ty = fy - j0, tx = fx - i0;
    const double v00 = nodeDeg(j0, i0), v01 = nodeDeg(j0, i0 + 1);
    const double v10 = nodeDeg(j0 + 1, i0), v11 = nodeDeg(j0 + 1, i0 + 1);
    const double lo = v00 + (v01 - v00) * tx;
    const double hi = v10 + (v11 - v10) * tx;
    return static_cast<float>(lo + (hi - lo) * ty);
}

}  // namespace fsim
