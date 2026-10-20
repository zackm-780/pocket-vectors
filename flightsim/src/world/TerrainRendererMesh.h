#pragma once
// W2: CPU side of the terrain renderer's mesh building (no GL, unit tested).
//
// ---- LOD scheme (documented decision: CDLOD-style quadtree of fixed-size patches) -------------
// The tile pyramid (DATA section 4) is the data hierarchy. The renderer draws *nodes*: a node is a
// geographic square of "node level" m, spanning 90/2^m degrees, id (m, x, y) with the same
// numbering as a tile (terrain::TileId is reused). Every node is a kPatchCells x kPatchCells quad
// grid, so a node of level m has exactly the sample spacing of tile level m - kPatchShift:
//     node level m  <->  natural tile level k = m - kPatchShift   (step 1 in that tile)
// Nodes coarser than the coarsest tile (m - kPatchShift < coarsest) read the coarsest tile at a
// step of 2^(coarsest - k). Nodes whose natural tile is not resident read the finest resident
// ancestor tile (bilinear), so the topology never changes when a tile streams in, only heights.
//
// Cracks between nodes of different levels are hidden by skirts (a strip hanging below each edge).
// Popping is avoided with CDLOD geomorphing: every vertex carries a delta to the surface of the
// parent (half resolution) lattice and the vertex shader blends toward it by distance.
//
// ---- RTC (ARCHITECTURE section 4) -------------------------------------------------------------
// Each mesh has a double ECEF anchor (node centre at height 0) and float vertex offsets in ECEF
// axes. Absolute ECEF is never stored in floats.

#include <cstdint>
#include <functional>
#include <memory>
#include <vector>

#include "core/Geo.h"
#include "core/TerrainQuery.h"
#include "world/terrain/FlattenSet.h"
#include "world/terrain/TerrainTile.h"
#include "world/terrain/TileScheme.h"

namespace fsim::world {

using NodeId = terrain::TileId;  // k = node level m

constexpr int kPatchCells = 16;  // quads per node side (must divide 256, a power of two)
constexpr int kPatchShift = 4;   // log2(256 / kPatchCells)
static_assert((256 >> kPatchShift) == kPatchCells, "kPatchShift must match kPatchCells");
constexpr int kPatchVerts = kPatchCells + 1;
constexpr int kGridVertexCount = kPatchVerts * kPatchVerts;
constexpr int kSkirtVertexCount = 4 * kPatchVerts;
constexpr int kNodeVertexCount = kGridVertexCount + kSkirtVertexCount;
constexpr int kNodeTriangleCount = (kPatchCells * kPatchCells + 4 * kPatchCells) * 2;
constexpr int kNodeIndexCount = kNodeTriangleCount * 3;
static_assert(kNodeVertexCount < 65536, "node meshes use 16-bit indices");

// Tile level whose sample spacing equals the node's cell spacing.
inline int naturalTileLevel(int nodeLevel) {
    return nodeLevel - kPatchShift;
}
// Degrees of latitude/longitude per node cell.
inline double nodeCellDeg(int nodeLevel) {
    return terrain::spanDeg(nodeLevel) / kPatchCells;
}
// Mean metres per cell at the node's centre latitude (average of N-S and E-W spacing).
double nodeCellMeanM(const NodeId& id);

struct NodeVertex {
    float pos[3];      // offset from the mesh anchor, ECEF axes, metres
    float morph[3];    // pos + k * morph = vertex on the parent (half resolution) lattice
    int8_t normal[4];  // ECEF unit normal * 127 (w unused)
    uint8_t color[4];  // rgb albedo, a = water fraction
};
static_assert(sizeof(NodeVertex) == 32, "NodeVertex layout is shared with the vertex shader");

struct NodeMesh {
    NodeId id;
    Ecef anchor;                           // node centre at height 0 (double precision)
    int sourceLevel = -1;                  // tile level the heights came from
    float minHeightM = 0, maxHeightM = 0;  // over the surface vertices (skirts excluded)
    float skirtDepthM = 0;
    bool allSea = false;  // every vertex is water at about 0 m: the sea disc covers it
    std::vector<NodeVertex> vertices;  // kNodeVertexCount: (kPatchVerts^2 grid, then 4 skirts)
};

// Supplies resident tiles (null when not resident / not in the pack).
using TileLookupFn = std::function<std::shared_ptr<const terrain::TerrainTile>(const terrain::TileId&)>;

struct MeshBuildOptions {
    int coarsestTileLevel = 0;  // == root node level; its nodes have no parent, hence no morph
    const terrain::FlattenSet* flatten = nullptr;  // runway pads baked into vertex heights
};

// Builds the node mesh. Returns false when no tile (the natural tile or any ancestor down to the
// coarsest level) is resident.
bool buildNodeMesh(const NodeId& id, const TileLookupFn& lookup, const MeshBuildOptions& options,
                   NodeMesh& out);

// The one shared 16-bit index buffer for every node (grid triangles, then skirt triangles).
const std::vector<uint16_t>& nodeIndices();

// Albedo of a land class (display colour, 0..1). Unknown returns mid grey.
void landClassAlbedo(LandClass c, float rgb[3]);
// Colour from elevation and slope when no land class is available.
void fallbackAlbedo(float heightM, float slopeCos, float rgb[3]);

// Geodetic -> ECEF with the same formula the builder uses (for tests of the anchor math).
Ecef nodeVertexEcef(const NodeMesh& mesh, int vertexIndex);

}  // namespace fsim::world
