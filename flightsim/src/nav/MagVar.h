#pragma once
// Magnetic declination lookup from `magvar.bin` (DATA.md section 6): a regular lat/lon grid of
// int16 centidegrees, bilinearly interpolated. + = east, as everywhere else in the code base.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fsim {

class MagVar {
public:
    // Both return false and fill `error` when the data is unreadable or malformed. A failed load
    // leaves the object invalid (declinationDeg() then returns 0).
    bool loadFile(const std::string& path, std::string& error);
    bool loadBytes(const uint8_t* data, size_t size, std::string& error);

    bool valid() const { return !grid_.empty(); }
    float epochYear() const { return epochYear_; }

    // Declination in degrees (+ = east) at a position. Longitude wraps around the antimeridian
    // when the grid covers the whole globe; latitude is clamped to the grid.
    float declinationDeg(double latDeg, double lonDeg) const;

    // Grid value in degrees at node (row, col), for tests and tools.
    float nodeDeg(int row, int col) const;
    int rows() const { return rows_; }
    int cols() const { return cols_; }

private:
    float epochYear_ = 0;
    int latMinDeg_ = 0, latMaxDeg_ = 0, lonMinDeg_ = 0, lonMaxDeg_ = 0, stepDeg_ = 1;
    int rows_ = 0, cols_ = 0;
    bool wrapLon_ = false;
    std::vector<int16_t> grid_;  // centidegrees, row-major from latMin, lonMin
};

}  // namespace fsim
