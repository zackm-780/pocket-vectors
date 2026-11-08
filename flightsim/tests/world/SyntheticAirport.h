#pragma once
// W8b/W8c tests: airports with runways, OSM-style taxiway segments and aprons, built in memory from
// east/north metres around a reference point (no data pack, no OSM data).
#include <string>
#include <vector>

#include "core/NavTypes.h"
#include "world/TaxiwayGeometry.h"

namespace fsim::testing {

class AirportBuilder {
public:
    AirportBuilder(const std::string& ident, double latDeg, double lonDeg, float elevFt,
                   AirportType type);
    // A runway pair from p0 (ident0's threshold, elevation z0M) to p1 (ident1's, z1M).
    void runway(const std::string& ident0, const std::string& ident1, world::Pt2 p0, world::Pt2 p1,
                double z0M, double z1M, double widthFt = 150.0, bool hard = true);
    void taxiway(const std::vector<world::Pt2>& pts, float widthM,
                 TaxiwayKind kind = TaxiwayKind::Taxiway, const std::string& ref = "");
    void apron(const std::vector<world::Pt2>& ring);
    const world::LocalFrame& frame() const { return frame_; }
    Airport airport;

private:
    world::LocalFrame frame_;
};

// Points every stepM along a straight line (both ends included).
std::vector<world::Pt2> line(world::Pt2 a, world::Pt2 b, double stepM);
// Quadratic Bezier a -> b with control point c, about stepM between points.
std::vector<world::Pt2> curve(world::Pt2 a, world::Pt2 c, world::Pt2 b, double stepM);

// A KPDX-sized field at 45.59 N, 122.60 W: two parallel runways (3,400 and 3,200 m) and a 2,000 m
// one crossing the southern one, parallel taxiways with S-bends, 90 and 30 degree exits, two
// terminals (taxilane lattices with curved lead-ins) and 15 aprons with rough, some concave,
// edges. Sized like the real KPDX (553 segments; 780 taxiway and 436 apron flatten areas).
Airport kpdxSizedAirport();

}  // namespace fsim::testing
