// W2 terrain renderer, CPU side (no GL): mesh building, LOD selection, culling, flatten pads.
#include <doctest/doctest.h>

#include <chrono>
#include <cmath>
#include <memory>
#include <set>

#include "core/Units.h"
#include "gfx/Camera.h"
#include "world/TerrainRendererLod.h"
#include "world/TerrainRendererMesh.h"
#include "world/terrain/SyntheticPack.h"
#include "world/terrain/TerrainService.h"

using namespace fsim;
using namespace fsim::world;
using namespace fsim::terrain;
using namespace fsim::terrain::testing;

namespace {

constexpr int kCoarsest = 5, kFinest = 8;

double hills(double lat, double lon) {
    return 900.0 + 350.0 * std::sin(lat * 2.0 * kPi / 0.21) * std::cos(lon * 2.0 * kPi / 0.17);
}
uint8_t landFn(double lat, double lon) {
    (void)lon;
    return lat > 45.2 ? static_cast<uint8_t>(LandClass::Tree)
                      : static_cast<uint8_t>(LandClass::Crop);
}

struct Fixture {
    TempDir dir;
    std::unique_ptr<TerrainService> svc;
    Fixture() {
        PackWriter w(dir.path, kCoarsest, kFinest);
        for (int k = kCoarsest; k <= kFinest; ++k)
            w.writeBox(k, 44.9, -122.6, 45.5, -121.9, hills, landFn);
        w.finish();
        TerrainConfig cfg;
        cfg.detailRadiusM = 60000;
        svc = std::make_unique<TerrainService>(dir.path, cfg);
        REQUIRE(svc->isOpen());
        svc->update(45.2, -122.2);
        REQUIRE(svc->waitForIdle(60.0));
    }
};
Fixture& fx() {
    static Fixture f;
    return f;
}

TileLookupFn lookupOf(TerrainService& s) {
    return [&s](const TileId& id) { return s.tile(id); };
}

// Node of level m containing the point.
NodeId nodeAt(int m, double lat, double lon) {
    return tileAt(m, lat, lon);
}

double heightOf(const NodeMesh& m, int v) {
    return ecefToGeodetic(nodeVertexEcef(m, v)).altMslM;
}

ViewParams makeView(double lat, double lon, double altM, double headingDeg, double pitchDeg = 0,
                    float aglM = -1) {
    AircraftState s;
    s.latDeg = lat;
    s.lonDeg = lon;
    s.altMslFt = mToFt(altM);
    s.altAglFt = static_cast<float>(mToFt(aglM >= 0 ? aglM : altM - 900.0));
    s.headingTrueRad = static_cast<float>(degToRad(headingDeg));
    s.pitchRad = static_cast<float>(degToRad(pitchDeg));
    gfx::Camera cam;
    cam.setLens(55.0f);
    cam.setViewport(640, 480);
    cam.setPoseFromAircraft(s);
    ViewParams v;
    v.eye = cam.eyeEcef();
    v.eyeGeo = cam.eyeGeo();
    v.ecefToEye = cam.ecefToEye();
    v.vFovRad = cam.vFovRad();
    v.aspect = cam.aspect();
    v.nearM = cam.nearM();
    v.farM = cam.farM();
    return v;
}

// Runs selection and builds (like TerrainRenderer without GL) until nothing is missing.
int settle(TerrainSelector& sel, TerrainService& svc, const ViewParams& view,
           std::vector<DrawNode>& draw, std::vector<BuildRequest>& builds, int maxPasses = 80) {
    MeshBuildOptions opt;
    opt.coarsestTileLevel = svc.coarsestLevel();
    opt.flatten = &svc.flattenSet();
    int built = 0;
    for (int pass = 0; pass < maxPasses; ++pass) {
        sel.select(view, draw, builds);
        if (builds.empty()) break;
        for (const BuildRequest& b : builds) {
            NodeRecord* rec = sel.find(b.id);
            NodeMesh mesh;
            if (rec && buildNodeMesh(b.id, lookupOf(svc), opt, mesh)) {
                sel.applyMesh(*rec, mesh, 1);
                ++built;
            } else if (rec) {
                sel.markBuildFailed(*rec);
            }
        }
    }
    return built;
}

FlattenArea squareArea(double cLat, double cLon, double halfM, float elevM) {
    const double dLat = halfM / (kEarthRadiusM * kRadPerDeg);
    const double dLon = dLat / std::cos(cLat * kRadPerDeg);
    FlattenArea a;
    const double la[4] = {cLat - dLat, cLat - dLat, cLat + dLat, cLat + dLat};
    const double lo[4] = {cLon - dLon, cLon + dLon, cLon + dLon, cLon - dLon};
    for (int i = 0; i < 4; ++i) a.latDeg[i] = la[i], a.lonDeg[i] = lo[i];
    a.elevM = elevM;
    a.blendM = 60.0f;
    return a;
}

}  // namespace

TEST_CASE("terrainrender: node constants and shared index buffer") {
    const std::vector<uint16_t>& ix = nodeIndices();
    CHECK(static_cast<int>(ix.size()) == kNodeIndexCount);
    CHECK(kNodeVertexCount == kPatchVerts * kPatchVerts + 4 * kPatchVerts);
    CHECK(kNodeTriangleCount == 2 * (kPatchCells * kPatchCells + 4 * kPatchCells));
    for (uint16_t i : ix) CHECK(i < kNodeVertexCount);
    std::set<uint16_t> used(ix.begin(), ix.end());
    CHECK(static_cast<int>(used.size()) == kNodeVertexCount);
    CHECK(kNodeVertexCount < 65536);  // 16-bit indices (ARCHITECTURE section 6)
    // cell size halves with every level and matches the tile sample spacing
    const NodeId a = nodeAt(12, 45.2, -122.2), b = nodeAt(13, 45.2, -122.2);
    CHECK(nodeCellMeanM(a) == doctest::Approx(2.0 * nodeCellMeanM(b)).epsilon(0.002));
    CHECK(nodeCellDeg(12) == doctest::Approx(stepDeg(naturalTileLevel(12))));
}

TEST_CASE("terrainrender: mesh vertices sit on the tile samples; RTC anchor math") {
    Fixture& f = fx();
    const NodeId id = nodeAt(kFinest + kPatchShift, 45.2, -122.2);  // natural level node
    NodeMesh mesh;
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    REQUIRE(buildNodeMesh(id, lookupOf(*f.svc), opt, mesh));
    REQUIRE(static_cast<int>(mesh.vertices.size()) == kNodeVertexCount);
    CHECK(mesh.sourceLevel == kFinest);

    const std::shared_ptr<const TerrainTile> tile = f.svc->tile(ancestorAt(id, kFinest));
    REQUIRE(tile.get() != nullptr);
    const double cellDeg = nodeCellDeg(id.k);
    float maxOffsetM = 0;
    for (int j = 0; j < kPatchVerts; ++j) {
        for (int i = 0; i < kPatchVerts; ++i) {
            const int v = j * kPatchVerts + i;
            const NodeVertex& nv = mesh.vertices[static_cast<size_t>(v)];
            maxOffsetM = std::max(
                {maxOffsetM, std::fabs(nv.pos[0]), std::fabs(nv.pos[1]), std::fabs(nv.pos[2])});
            const GeoPos g = ecefToGeodetic(nodeVertexEcef(mesh, v));
            const double lat = -90.0 + (static_cast<double>(id.y) * kPatchCells + j) * cellDeg;
            const double lon = -180.0 + (static_cast<double>(id.x) * kPatchCells + i) * cellDeg;
            CHECK(std::fabs(g.latDeg - lat) < 1e-8);
            CHECK(std::fabs(g.lonDeg - lon) < 1e-8);
            // heights are the tile sample at that point (step 1 at the natural level)
            float h = 0;
            REQUIRE(tile->sampleHeightM(lat, lon, h));
            CHECK(std::fabs(g.altMslM - h) < 0.01);
        }
    }
    // float offsets stay small (a finest node is ~600 m wide); absolute ECEF is never in floats
    CHECK(maxOffsetM < 2000.0f);
    // anchor = node centre at height 0
    const TileBounds b = bounds(id);
    GeoPos c;
    c.latDeg = b.centerLatDeg();
    c.lonDeg = b.centerLonDeg();
    const Ecef ce = geodeticToEcef(c);
    CHECK(std::fabs(mesh.anchor.x - ce.x) < 1e-6);
    CHECK(std::fabs(mesh.anchor.y - ce.y) < 1e-6);
    CHECK(std::fabs(mesh.anchor.z - ce.z) < 1e-6);
}

TEST_CASE("terrainrender: normals are unit and point away from the earth") {
    Fixture& f = fx();
    NodeMesh mesh;
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    const NodeId id = nodeAt(kFinest + kPatchShift, 45.15, -122.3);
    REQUIRE(buildNodeMesh(id, lookupOf(*f.svc), opt, mesh));
    for (int v = 0; v < kGridVertexCount; ++v) {
        const NodeVertex& nv = mesh.vertices[static_cast<size_t>(v)];
        const double nx = nv.normal[0] / 127.0, ny = nv.normal[1] / 127.0,
                     nz = nv.normal[2] / 127.0;
        CHECK(std::sqrt(nx * nx + ny * ny + nz * nz) == doctest::Approx(1.0).epsilon(0.02));
        const Ecef p = nodeVertexEcef(mesh, v);
        const double r = std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
        CHECK((nx * p.x + ny * p.y + nz * p.z) / r > 0.3);
    }
}

TEST_CASE("terrainrender: neighbouring nodes share edge vertices exactly (also across tiles)") {
    Fixture& f = fx();
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    const int m = kFinest + kPatchShift;
    // x a multiple of 16 is a tile boundary: node x-1 and node x are in different tiles
    NodeId west = nodeAt(m, 45.2, -122.2);
    west.x = (west.x / 16) * 16 - 1;
    NodeId east = west;
    east.x += 1;
    NodeId north = west;
    north.y += 1;
    NodeMesh a, b, c;
    REQUIRE(buildNodeMesh(west, lookupOf(*f.svc), opt, a));
    REQUIRE(buildNodeMesh(east, lookupOf(*f.svc), opt, b));
    REQUIRE(buildNodeMesh(north, lookupOf(*f.svc), opt, c));
    CHECK(ancestorAt(west, kFinest) != ancestorAt(east, kFinest));
    for (int j = 0; j < kPatchVerts; ++j) {
        const Ecef pa = nodeVertexEcef(a, j * kPatchVerts + kPatchCells);
        const Ecef pb = nodeVertexEcef(b, j * kPatchVerts + 0);
        CHECK(std::fabs(pa.x - pb.x) < 1e-3);
        CHECK(std::fabs(pa.y - pb.y) < 1e-3);
        CHECK(std::fabs(pa.z - pb.z) < 1e-3);
        // lighting is continuous across the seam, too
        for (int k = 0; k < 3; ++k)
            CHECK(
                std::abs(a.vertices[static_cast<size_t>(j * kPatchVerts + kPatchCells)].normal[k] -
                         b.vertices[static_cast<size_t>(j * kPatchVerts)].normal[k]) <= 2);
    }
    for (int i = 0; i < kPatchVerts; ++i) {
        const Ecef pa = nodeVertexEcef(a, kPatchCells * kPatchVerts + i);
        const Ecef pc = nodeVertexEcef(c, i);
        CHECK(std::fabs(pa.x - pc.x) < 1e-3);
        CHECK(std::fabs(pa.y - pc.y) < 1e-3);
        CHECK(std::fabs(pa.z - pc.z) < 1e-3);
    }
}

TEST_CASE("terrainrender: skirts hang below the border and cover LOD mismatches") {
    Fixture& f = fx();
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    const int m = kFinest + kPatchShift - 1;  // coarse node (data from tile level kFinest-1)
    NodeId coarse = nodeAt(m, 45.2, -122.2);
    NodeMesh a;
    REQUIRE(buildNodeMesh(coarse, lookupOf(*f.svc), opt, a));
    CHECK(a.skirtDepthM > 3.0f);
    // south skirt vertex s is the border vertex (s, 0) lowered by skirtDepthM
    for (int s = 0; s < kPatchVerts; ++s) {
        const GeoPos top = ecefToGeodetic(nodeVertexEcef(a, s));
        const GeoPos low = ecefToGeodetic(nodeVertexEcef(a, kGridVertexCount + s));
        CHECK(top.altMslM - low.altMslM == doctest::Approx(a.skirtDepthM).epsilon(0.001));
        CHECK(std::fabs(top.latDeg - low.latDeg) < 1e-8);
    }
    // A finer neighbour on the east: its west edge lies along the coarse node's east edge. The
    // vertical gap between the two edges must stay below the coarse skirt depth.
    NodeId fineEast{m + 1, coarse.x * 2 + 2, coarse.y * 2};
    NodeMesh b;
    REQUIRE(buildNodeMesh(fineEast, lookupOf(*f.svc), opt, b));
    CHECK(b.sourceLevel == kFinest);
    CHECK(a.sourceLevel == kFinest - 1);
    double worst = 0;
    for (int j = 0; j < kPatchVerts; ++j) {  // fine west edge, first half of the coarse east edge
        const double hFine = heightOf(b, j * kPatchVerts);
        const double u = j * 0.5;
        const int j0 = static_cast<int>(std::floor(u)), j1 = std::min(kPatchCells, j0 + 1);
        const double hA = heightOf(a, j0 * kPatchVerts + kPatchCells) * (1 - (u - j0)) +
                          heightOf(a, j1 * kPatchVerts + kPatchCells) * (u - j0);
        worst = std::max(worst, std::fabs(hFine - hA));
    }
    CHECK(worst < a.skirtDepthM);
}

TEST_CASE("terrainrender: geomorph target equals the parent's surface") {
    Fixture& f = fx();
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    const int mc = kFinest + kPatchShift;
    NodeId child = nodeAt(mc, 45.17, -122.31);
    const NodeId parent = NodeId{mc - 1, child.x >> 1, child.y >> 1};
    NodeMesh c, p;
    REQUIRE(buildNodeMesh(child, lookupOf(*f.svc), opt, c));
    REQUIRE(buildNodeMesh(parent, lookupOf(*f.svc), opt, p));
    const int ox = static_cast<int>(child.x & 1) * kPatchCells;  // child origin in child cells
    const int oy = static_cast<int>(child.y & 1) * kPatchCells;
    auto ph = [&](int a, int b) {
        return heightOf(p, std::min(kPatchCells, b) * kPatchVerts + std::min(kPatchCells, a));
    };
    double worst = 0;
    for (int j = 0; j < kPatchVerts; ++j) {
        for (int i = 0; i < kPatchVerts; ++i) {
            const int v = j * kPatchVerts + i;
            const NodeVertex& nv = c.vertices[static_cast<size_t>(v)];
            Ecef e = nodeVertexEcef(c, v);
            e.x += nv.morph[0];
            e.y += nv.morph[1];
            e.z += nv.morph[2];
            const double target = ecefToGeodetic(e).altMslM;
            const int pi = (ox + i) / 2, pj = (oy + j) / 2;  // parent cell
            double expect;
            const bool oddI = ((ox + i) & 1) != 0, oddJ = ((oy + j) & 1) != 0;
            if (!oddI && !oddJ) {
                expect = ph(pi, pj);
            } else if (oddI && !oddJ) {
                expect = 0.5 * (ph(pi, pj) + ph(pi + 1, pj));
            } else if (!oddI && oddJ) {
                expect = 0.5 * (ph(pi, pj) + ph(pi, pj + 1));
            } else {  // cell centre: on the parent's diagonal, as drawn from the shared indices
                expect = (((pi + pj) & 1) == 0) ? 0.5 * (ph(pi, pj) + ph(pi + 1, pj + 1))
                                                : 0.5 * (ph(pi + 1, pj) + ph(pi, pj + 1));
            }
            worst = std::max(worst, std::fabs(target - expect));
        }
    }
    // Parent and child read different tile levels (smoothed differently), so allow the data
    // difference; which vertices and which diagonal are used must match.
    CHECK(worst < 6.0);
}

TEST_CASE("terrainrender: flatten areas are baked into vertex heights") {
    Fixture& f = fx();
    const NodeId id = nodeAt(kFinest + kPatchShift, 45.15, -122.3);
    const TileBounds b = bounds(id);
    const double cLat = b.centerLatDeg(), cLon = b.centerLonDeg();
    const FlattenArea a = squareArea(cLat, cLon, 350.0, 123.0f);
    FlattenSet fs;
    fs.add(a);

    MeshBuildOptions plain, padded;
    plain.coarsestTileLevel = padded.coarsestTileLevel = kCoarsest;
    padded.flatten = &fs;
    NodeMesh m0, m1;
    REQUIRE(buildNodeMesh(id, lookupOf(*f.svc), plain, m0));
    REQUIRE(buildNodeMesh(id, lookupOf(*f.svc), padded, m1));
    const double dLat = 350.0 / (kEarthRadiusM * kRadPerDeg);
    const double dLon = dLat / std::cos(cLat * kRadPerDeg);
    const double blendDeg = 100.0 / (kEarthRadiusM * kRadPerDeg);
    int inside = 0, changed = 0, untouched = 0;
    for (int v = 0; v < kGridVertexCount; ++v) {
        const GeoPos g = ecefToGeodetic(nodeVertexEcef(m1, v));
        const double h0 = heightOf(m0, v);
        const bool in = std::fabs(g.latDeg - cLat) <= dLat && std::fabs(g.lonDeg - cLon) <= dLon;
        if (in) {
            ++inside;
            CHECK(std::fabs(g.altMslM - 123.0) < 0.01);
        }
        if (std::fabs(g.altMslM - h0) > 0.01) ++changed;
        if (std::fabs(g.latDeg - cLat) > dLat + blendDeg ||
            std::fabs(g.lonDeg - cLon) > dLon + 2.0 * blendDeg) {
            ++untouched;
            CHECK(std::fabs(g.altMslM - h0) < 0.01);
        }
    }
    CHECK(inside >= 4);
    CHECK(changed >= inside);
    CHECK(untouched > 100);
    CHECK(m1.sourceLevel == m0.sourceLevel);
}

TEST_CASE("terrainrender: land class colours") {
    Fixture& f = fx();
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    NodeMesh crop, tree;
    REQUIRE(
        buildNodeMesh(nodeAt(kFinest + kPatchShift, 45.1, -122.3), lookupOf(*f.svc), opt, crop));
    REQUIRE(
        buildNodeMesh(nodeAt(kFinest + kPatchShift, 45.3, -122.3), lookupOf(*f.svc), opt, tree));
    const NodeVertex& vc = crop.vertices[8 * kPatchVerts + 8];
    const NodeVertex& vt = tree.vertices[8 * kPatchVerts + 8];
    CHECK(vc.color[3] == 0);  // not water
    CHECK(vt.color[3] == 0);
    CHECK(vt.color[1] > vt.color[0]);  // trees are green
    CHECK(vc.color[0] > vt.color[0]);  // crops are lighter
    CHECK(!crop.allSea);
    float rgb[3];
    landClassAlbedo(LandClass::SnowIce, rgb);
    CHECK(rgb[0] > 0.85f);
    fallbackAlbedo(3500.0f, 1.0f, rgb);
    CHECK(rgb[0] > 0.8f);  // snow line without land class data
}

TEST_CASE("terrainrender: frustum, box distance and horizon culling") {
    const NodeId id = nodeAt(10, 45.2, -122.2);
    const NodeBox box = computeNodeBox(id, 0, 1000);
    CHECK(box.half[0] > 100);
    CHECK(box.half[2] >= 450);
    GeoPos p;
    p.latDeg = bounds(id).centerLatDeg();
    p.lonDeg = bounds(id).centerLonDeg();
    p.altMslM = 500;
    CHECK(distanceToBox(box, geodeticToEcef(p)) < 1.0);
    p.altMslM = 3000;
    CHECK(distanceToBox(box, geodeticToEcef(p)) == doctest::Approx(2000.0).epsilon(0.05));

    // Looking at the node from 15 km south: heading north sees it, heading south does not.
    const double midLat = bounds(id).centerLatDeg();
    const double dLat = 15000.0 / (kEarthRadiusM * kRadPerDeg);
    const ViewParams north = makeView(midLat - dLat, bounds(id).centerLonDeg(), 1500, 0);
    const ViewParams south = makeView(midLat - dLat, bounds(id).centerLonDeg(), 1500, 180);
    const Frustum fn =
        makeFrustum(north.ecefToEye, north.vFovRad, north.aspect, north.nearM, 40000);
    const Frustum fs =
        makeFrustum(south.ecefToEye, south.vFovRad, south.aspect, south.nearM, 40000);
    CHECK(frustumIntersects(fn, box, north.eye));
    CHECK(!frustumIntersects(fs, box, south.eye));
    const Frustum shortFar = makeFrustum(north.ecefToEye, north.vFovRad, north.aspect, 0.5f, 10000);
    CHECK(!frustumIntersects(shortFar, box, north.eye));  // beyond the far plane
    const ViewParams side = makeView(midLat - dLat, bounds(id).centerLonDeg(), 1500, 70);
    const Frustum fside = makeFrustum(side.ecefToEye, side.vFovRad, side.aspect, side.nearM, 40000);
    CHECK(!frustumIntersects(fside, box, side.eye));  // outside the horizontal field of view

    // Horizon: a sea-level node ~150 km away is below the horizon of an eye at 2 m, visible from
    // 9 km altitude; the same node with a 4 km mountain on it shows over the horizon.
    GeoPos eyeLow;
    eyeLow.latDeg = 45.0;
    eyeLow.lonDeg = -123.0;
    eyeLow.altMslM = 2;
    const Ecef eyeLowE = geodeticToEcef(eyeLow);
    auto norm = [](const Ecef& e) { return std::sqrt(e.x * e.x + e.y * e.y + e.z * e.z); };
    const double rOcc = norm(eyeLowE) - 152.0;
    const NodeId far = nodeAt(10, 45.0, -121.3);
    const NodeBox farFlat = computeNodeBox(far, 0, 10);
    const NodeBox farTall = computeNodeBox(far, 0, 4000);
    CHECK(!horizonVisible(eyeLowE, rOcc, farFlat));
    CHECK(horizonVisible(eyeLowE, rOcc, farTall));
    GeoPos eyeHigh = eyeLow;
    eyeHigh.altMslM = 9000;
    const Ecef eyeHighE = geodeticToEcef(eyeHigh);
    CHECK(horizonVisible(eyeHighE, norm(eyeHighE) - 9500.0, farFlat));
}

TEST_CASE("terrainrender: split range scales with cell size and view") {
    LodConfig cfg;
    const double r1 = splitRangeM(100.0, 0.96f, cfg);
    CHECK(splitRangeM(200.0, 0.96f, cfg) == doctest::Approx(2.0 * r1));
    LodConfig fine = cfg;
    fine.pixelsPerCell = cfg.pixelsPerCell / 2;
    CHECK(splitRangeM(100.0, 0.96f, fine) == doctest::Approx(2.0 * r1));
    CHECK(splitRangeM(100.0, 0.5f, cfg) > r1);  // narrower lens: more range
    CHECK(foreshortenFactor(1.0, cfg) == doctest::Approx(1.0));
    CHECK(foreshortenFactor(0.0, cfg) == doctest::Approx(std::sqrt(cfg.foreshortenMin)));
    CHECK(foreshortenFactor(0.5, cfg) < 1.0);
}

TEST_CASE("terrainrender: LOD selection gets finer with proximity and never overlaps") {
    Fixture& f = fx();
    TerrainSelector sel(*f.svc);
    std::vector<DrawNode> draw;
    std::vector<BuildRequest> builds;

    const ViewParams low = makeView(45.05, -122.3, 1300, 0, -2, 300);
    CHECK(settle(sel, *f.svc, low, draw, builds) > 0);
    CHECK(builds.empty());
    REQUIRE(!draw.empty());
    int lowMax = 0;
    for (const DrawNode& d : draw) lowMax = std::max(lowMax, d.rec->id.k);
    CHECK(lowMax == kFinest + kPatchShift);
    for (size_t i = 1; i < draw.size(); ++i)
        CHECK(draw[i - 1].distM <= draw[i].distM);           // front to back
    CHECK(draw.front().rec->id.k >= draw.back().rec->id.k);  // nearest finest, farthest coarsest
    for (size_t i = 0; i < draw.size(); ++i)  // never a node together with its ancestor
        for (size_t j = 0; j < draw.size(); ++j)
            if (i != j) CHECK(!isAncestorOf(draw[i].rec->id, draw[j].rec->id));
    CHECK(static_cast<int>(draw.size()) * kNodeTriangleCount < 40000);  // PLAN section 3
    CHECK(draw.size() < 150);

    TerrainSelector sel2(*f.svc);  // same place much higher: coarser nodes only
    const ViewParams high = makeView(45.05, -122.3, 12000, 0, -20, 11000);
    settle(sel2, *f.svc, high, draw, builds);
    REQUIRE(!draw.empty());
    int highMax = 0;
    for (const DrawNode& d : draw) highMax = std::max(highMax, d.rec->id.k);
    CHECK(highMax < lowMax);

    LodConfig fineCfg, coarseCfg;
    fineCfg.pixelsPerCell = 5.0f;
    coarseCfg.pixelsPerCell = 20.0f;
    TerrainSelector selFine(*f.svc, fineCfg), selCoarse(*f.svc, coarseCfg);
    std::vector<DrawNode> drawFine, drawCoarse;
    settle(selFine, *f.svc, low, drawFine, builds);
    settle(selCoarse, *f.svc, low, drawCoarse, builds);
    CHECK(drawFine.size() > drawCoarse.size());
}

TEST_CASE("terrainrender: a node is only split once its children are built") {
    Fixture& f = fx();
    TerrainSelector sel(*f.svc);
    std::vector<DrawNode> draw;
    std::vector<BuildRequest> builds;
    const ViewParams v = makeView(45.05, -122.3, 1300, 0, -2, 300);
    sel.select(v, draw, builds);  // nothing is built: no draw, roots are queued
    CHECK(draw.empty());
    CHECK(!builds.empty());
    for (const BuildRequest& b : builds) CHECK(b.id.k == kCoarsest);
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    for (const BuildRequest& b : builds) {
        NodeMesh mesh;
        REQUIRE(buildNodeMesh(b.id, lookupOf(*f.svc), opt, mesh));
        sel.applyMesh(*sel.find(b.id), mesh, 1);
    }
    sel.select(v, draw, builds);  // roots drawn, children queued: never a hole
    CHECK(!draw.empty());
    for (const DrawNode& d : draw) CHECK(d.rec->id.k == kCoarsest);
    CHECK(!builds.empty());
}

TEST_CASE("terrainrender: flatten revision marks affected meshes for rebuild") {
    Fixture& f = fx();
    TerrainSelector sel(*f.svc);
    std::vector<DrawNode> draw;
    std::vector<BuildRequest> builds;
    const ViewParams v = makeView(45.05, -122.3, 1300, 0, -2, 300);
    settle(sel, *f.svc, v, draw, builds);
    REQUIRE(!draw.empty());
    const NodeId nearest = draw.front().rec->id;
    const TileBounds b = bounds(nearest);
    f.svc->addFlattenArea(squareArea(b.centerLatDeg(), b.centerLonDeg(), 100.0, 77.0f));
    sel.select(v, draw, builds);
    bool nearestRebuilt = false;
    for (const BuildRequest& r : builds)
        nearestRebuilt = nearestRebuilt || (r.id == nearest && r.priority == 1);
    CHECK(nearestRebuilt);
    settle(sel, *f.svc, v, draw, builds);
    CHECK(builds.empty());
    f.svc->clearFlattenAreas();  // removing the pad rebuilds the same meshes
    sel.select(v, draw, builds);
    nearestRebuilt = false;
    for (const BuildRequest& r : builds)
        nearestRebuilt = nearestRebuilt || (r.id == nearest && r.priority == 1);
    CHECK(nearestRebuilt);
    settle(sel, *f.svc, v, draw, builds);
}

TEST_CASE("terrainrender: no resident tiles draws nothing and does not crash") {
    TempDir dir;
    PackWriter w(dir.path, 6, 7);
    w.writeBox(6, 10.0, 10.0, 10.2, 10.2, hills, landFn);  // far away from the camera
    w.finish();
    TerrainService svc(dir.path);
    REQUIRE(svc.isOpen());
    svc.update(45.2, -122.2);
    svc.waitForIdle(5.0);
    TerrainSelector sel(svc);
    std::vector<DrawNode> draw;
    std::vector<BuildRequest> builds;
    sel.select(makeView(45.2, -122.2, 2000, 0), draw, builds);
    CHECK(draw.empty());
    CHECK(builds.empty());
}

TEST_CASE("terrainrender: mesh build cost stays far below the frame budget") {
    Fixture& f = fx();
    MeshBuildOptions opt;
    opt.coarsestTileLevel = kCoarsest;
    const TileLookupFn lookup = lookupOf(*f.svc);
    const int m = kFinest + kPatchShift;
    NodeMesh mesh;
    const auto t0 = std::chrono::steady_clock::now();
    const int n = 200;
    for (int i = 0; i < n; ++i) {
        NodeId id = nodeAt(m, 45.1, -122.3);
        id.x += static_cast<uint32_t>(i % 10);
        id.y += static_cast<uint32_t>(i / 10);
        REQUIRE(buildNodeMesh(id, lookup, opt, mesh));
    }
    const double perMeshMs =
        std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count() /
        n;
    MESSAGE("mesh build: " << perMeshMs << " ms per node (desktop, " << kNodeVertexCount
                           << " vertices)");
    CHECK(perMeshMs < 2.0);
}
