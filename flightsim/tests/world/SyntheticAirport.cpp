#include "world/SyntheticAirport.h"

#include <cmath>

#include "core/Geo.h"
#include "core/Units.h"

namespace fsim::testing {

using world::Pt2;

AirportBuilder::AirportBuilder(const std::string& ident, double latDeg, double lonDeg, float elevFt,
                               AirportType type)
    : frame_(latDeg, lonDeg) {
    airport.ident = ident;
    airport.name = ident + " synthetic";
    airport.latDeg = latDeg;
    airport.lonDeg = lonDeg;
    airport.elevFt = elevFt;
    airport.type = type;
}

void AirportBuilder::runway(const std::string& ident0, const std::string& ident1, Pt2 p0, Pt2 p1,
                            double z0M, double z1M, double widthFt, bool hard) {
    const LatLon a = frame_.toLatLon(p0), b = frame_.toLatLon(p1);
    const double lenM = std::hypot(p1.eM - p0.eM, p1.nM - p0.nM);
    Runway r;
    r.airportIdent = airport.ident;
    r.ident = ident0;
    r.oppositeIdent = ident1;
    r.thrLatDeg = r.startLatDeg = a.latDeg;
    r.thrLonDeg = r.startLonDeg = a.lonDeg;
    r.endLatDeg = b.latDeg;
    r.endLonDeg = b.lonDeg;
    r.thrElevFt = static_cast<float>(mToFt(z0M));
    r.endElevFt = static_cast<float>(mToFt(z1M));
    r.headingTrueDeg =
        static_cast<float>(radToDeg(initialBearingTrueRad(a.latDeg, a.lonDeg, b.latDeg, b.lonDeg)));
    r.lengthFt = static_cast<float>(mToFt(lenM));
    r.widthFt = static_cast<float>(widthFt);
    r.hard = hard;
    Runway o = r;
    o.ident = ident1;
    o.oppositeIdent = ident0;
    o.thrLatDeg = o.startLatDeg = b.latDeg;
    o.thrLonDeg = o.startLonDeg = b.lonDeg;
    o.endLatDeg = a.latDeg;
    o.endLonDeg = a.lonDeg;
    o.thrElevFt = r.endElevFt;
    o.endElevFt = r.thrElevFt;
    o.headingTrueDeg =
        static_cast<float>(radToDeg(initialBearingTrueRad(b.latDeg, b.lonDeg, a.latDeg, a.lonDeg)));
    airport.runways.push_back(r);
    airport.runways.push_back(o);
}

void AirportBuilder::taxiway(const std::vector<Pt2>& pts, float widthM, TaxiwayKind kind,
                             const std::string& ref) {
    Taxiway t;
    t.osmWayId = static_cast<long long>(airport.taxiways.size()) + 1000;
    t.kind = kind;
    t.ref = ref;
    t.widthM = widthM;
    for (const Pt2& p : pts) t.points.push_back(frame_.toLatLon(p));
    airport.taxiways.push_back(t);
}

void AirportBuilder::apron(const std::vector<Pt2>& ring) {
    Apron a;
    a.osmWayId = static_cast<long long>(airport.aprons.size()) + 9000;
    for (const Pt2& p : ring) a.points.push_back(frame_.toLatLon(p));
    airport.aprons.push_back(a);
}

std::vector<Pt2> line(Pt2 a, Pt2 b, double stepM) {
    const double len = std::hypot(b.eM - a.eM, b.nM - a.nM);
    const int n = std::max(1, static_cast<int>(std::ceil(len / stepM)));
    std::vector<Pt2> out;
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n;
        out.push_back({a.eM + (b.eM - a.eM) * t, a.nM + (b.nM - a.nM) * t});
    }
    return out;
}

std::vector<Pt2> curve(Pt2 a, Pt2 c, Pt2 b, double stepM) {
    const double len = std::hypot(c.eM - a.eM, c.nM - a.nM) + std::hypot(b.eM - c.eM, b.nM - c.nM);
    const int n = std::max(2, static_cast<int>(std::ceil(len / stepM)));
    std::vector<Pt2> out;
    for (int i = 0; i <= n; ++i) {
        const double t = static_cast<double>(i) / n, u = 1.0 - t;
        out.push_back({u * u * a.eM + 2 * u * t * c.eM + t * t * b.eM,
                       u * u * a.nM + 2 * u * t * c.nM + t * t * b.nM});
    }
    return out;
}

namespace {

// A parallel taxiway at north offset nM with gentle S-bends (12 m, 700 m wavelength).
Pt2 onParallel(double eM, double nM) {
    return {eM, nM + 12.0 * std::sin(2.0 * kPi * eM / 700.0)};
}

// The parallel as segments between consecutive junctions (every 150 m), points every 25 m.
void parallel(AirportBuilder& b, double nM, double e0, double e1, const std::string& ref) {
    for (double e = e0; e < e1 - 1.0; e += 150.0) {
        std::vector<Pt2> pts;
        const double end = std::min(e + 150.0, e1);
        for (double x = e; x < end - 1e-6; x += 25.0) pts.push_back(onParallel(x, nM));
        pts.push_back(onParallel(end, nM));
        b.taxiway(pts, 23.0f, TaxiwayKind::Taxiway, ref);
    }
}

// A rounded rectangle (4 points per corner), with an optional notch cut into one corner.
std::vector<Pt2> apronRing(double ce, double cn, double halfE, double halfN, bool notch) {
    std::vector<Pt2> ring;
    const double r = 15.0;
    const double cx[4] = {ce + halfE - r, ce - halfE + r, ce - halfE + r, ce + halfE - r};
    const double cy[4] = {cn + halfN - r, cn + halfN - r, cn - halfN + r, cn - halfN + r};
    for (int k = 0; k < 4; ++k) {
        if (notch && k == 2) {  // concave notch at the south-west corner
            ring.push_back({ce - halfE, cn - halfN + 40.0});
            ring.push_back({ce - halfE + 60.0, cn - halfN + 40.0});
            ring.push_back({ce - halfE + 60.0, cn - halfN});
            continue;
        }
        for (int i = 0; i < 4; ++i) {
            const double a = (k * 90.0 + i * 30.0) * kRadPerDeg;
            ring.push_back({cx[k] + r * std::cos(a), cy[k] + r * std::sin(a)});
        }
    }
    return ring;
}

// A taxilane lattice beside a parallel at nParallel (dir +1 north, -1 south) with `aprons`
// aprons (rounded rectangles with rough edges, every third one concave).
void terminal(AirportBuilder& b, double nParallel, double dir, int aprons) {
    double rows[4];
    for (int r = 0; r < 4; ++r) rows[r] = nParallel + dir * (100.0 + 100.0 * r);
    for (const double nRow : rows)
        for (double e = -750.0; e < 750.0 - 1.0; e += 150.0)
            b.taxiway(line({e, nRow}, {e + 150.0, nRow}, 50.0), 7.5f, TaxiwayKind::Taxilane);
    for (double e = -750.0; e <= 750.0 + 1.0; e += 150.0) {
        for (int r = 0; r < 3; ++r)
            b.taxiway(line({e, rows[r]}, {e, rows[r + 1]}, 50.0), 7.5f, TaxiwayKind::Taxilane);
        if (static_cast<int>(std::lround(e / 150.0)) % 2 != 0)  // the parallel's junctions
            b.taxiway(line(onParallel(e, nParallel), {e, rows[0]}, 25.0), 15.0f);
    }
    for (int r = 0; r < 4; ++r) {
        const double side = dir * ((r % 2 == 0) ? 1.0 : -1.0);
        for (double e = -725.0; e <= 725.0; e += 50.0)
            b.taxiway(
                curve({e, rows[r]}, {e + 20.0, rows[r]}, {e + 24.0, rows[r] + side * 40.0}, 4.0),
                7.5f, TaxiwayKind::Taxilane);
    }
    unsigned seed = 4242u + static_cast<unsigned>(aprons);
    auto jitter = [&seed] {
        seed = seed * 1664525u + 1013904223u;
        return 8.0 * (static_cast<double>(seed >> 8) / static_cast<double>(1u << 24) - 0.5);
    };
    int k = 0;
    for (int r = 0; r < 3 && k < aprons; ++r) {
        for (double ce : {-600.0, -300.0, 0.0, 300.0, 600.0}) {
            if (k >= aprons) break;
            const std::vector<Pt2> ring =
                apronRing(ce, nParallel + dir * (150.0 + 100.0 * r), 140.0, 45.0, (k % 3) == 0);
            // Points every 20 m along the edges, a few metres off the line (OSM-like).
            std::vector<Pt2> rough;
            for (size_t i = 0; i < ring.size(); ++i) {
                const Pt2 p = ring[i], q = ring[(i + 1) % ring.size()];
                rough.push_back(p);
                const double len = std::hypot(q.eM - p.eM, q.nM - p.nM);
                for (double t = 20.0; t < len - 10.0; t += 20.0)
                    rough.push_back({p.eM + (q.eM - p.eM) * t / len + jitter(),
                                     p.nM + (q.nM - p.nM) * t / len + jitter()});
            }
            b.apron(rough);
            ++k;
        }
    }
}

}  // namespace

Airport kpdxSizedAirport() {
    AirportBuilder b("XKPX", 45.59, -122.60, 30.0f, AirportType::Large);
    b.runway("10L", "28R", {-1700, 600}, {1700, 600}, 8.8, 9.0);
    b.runway("10R", "28L", {-1600, -500}, {1600, -500}, 6.7, 7.0);
    b.runway("03", "21", {-1100, -1100}, {300, 300}, 6.7, 8.2);

    // Parallel taxiways: north of 10L/28R, between the runways, south of 10R/28L.
    parallel(b, 780.0, -1650.0, 1650.0, "A");
    parallel(b, 50.0, -1500.0, 1500.0, "B");
    parallel(b, -680.0, -1500.0, 1500.0, "C");

    // Exits every 300 m from the outer parallels to their runways: 90 degree and 30 degree
    // (high-speed) in turn, curved where they leave the parallel.
    int n = 0;
    for (double e = -1500.0; e <= 1500.0; e += 300.0, ++n) {
        for (const auto& [nP, nR] : {std::pair{780.0, 600.0}, std::pair{-680.0, -500.0}}) {
            const Pt2 a = onParallel(e, nP);
            const double dn = nR - nP;
            if (n % 2 == 0) {
                b.taxiway(curve(a, {e + 40.0, nP + 0.6 * dn}, {e + 40.0, nR}, 10.0), 23.0f);
            } else {
                const double run = std::fabs(dn) / std::tan(30.0 * kRadPerDeg);
                b.taxiway(curve(a, {e - 0.3 * run, nP + 0.2 * dn}, {e - run, nR}, 10.0), 23.0f);
            }
        }
    }
    // Crossing connectors from the middle parallel to both runways every 600 m.
    for (double e = -1200.0; e <= 1200.0; e += 600.0) {
        b.taxiway(curve(onParallel(e, 50.0), {e, 300.0}, {e + 30.0, 600.0}, 10.0), 23.0f);
        b.taxiway(
            curve(onParallel(e + 150.0, 50.0), {e + 150.0, -200.0}, {e + 180.0, -500.0}, 10.0),
            23.0f);
    }

    // Terminals north of the north parallel and south of the south one: taxilane lattices
    // (columns every 150 m, rows every 100 m), connectors to the parallel, curved lead-ins into the
    // stands every 50 m, and aprons over the lattice cells.
    terminal(b, 780.0, 1.0, 9);
    terminal(b, -680.0, -1.0, 6);
    return b.airport;
}

}  // namespace fsim::testing
