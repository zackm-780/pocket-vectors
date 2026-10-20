#pragma once
// W2: node selection, culling and bookkeeping for the terrain renderer (no GL, unit tested).
// See TerrainRendererMesh.h for the node scheme.
//
// Selection (CDLOD): start from the coarsest-level tiles around the camera. A node is split into
// its four children while the distance from the camera to the node's oriented bounding box is
// below the node's split range
//        range(node) = cellMeters * focalPx / pixelsPerCell
// (so one cell projects to about pixelsPerCell pixels on a 480-line screen). A split only happens
// when the children are drawable: their data tile is resident, and their meshes are built;
// otherwise the parent keeps being drawn and the missing pieces are queued. Nothing ever
// disappears while a finer level loads.

#include <cstdint>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/Geo.h"
#include "world/TerrainRendererMesh.h"
#include "world/terrain/TerrainService.h"

namespace fsim::world {

// ---- geometry: boxes, frustum, horizon --------------------------------------------------------
struct NodeBox {  // oriented box around a node's surface (axes are east/north/up at its centre)
    Ecef centre;
    Vec3d axis[3];
    double half[3] = {0, 0, 0};
    double boundRadiusM = 0;  // half diagonal
};

NodeBox computeNodeBox(const NodeId& id, double minHeightM, double maxHeightM);
// Distance from a point to the box (0 inside).
double distanceToBox(const NodeBox& box, const Ecef& p);

struct Frustum {
    double n[6][3];  // inward normals in ECEF axes
    double d[6];
};
Frustum makeFrustum(const Mat3d& ecefToEye, float vFovRad, float aspect, float nearM, float farM);
bool frustumIntersects(const Frustum& f, const NodeBox& box, const Ecef& eye);

// Horizon culling against a sphere of occluderRadiusM (centre = earth centre). True when some part
// of the box can be seen from the eye. Conservative for a sphere at or below the real surface.
bool horizonVisible(const Ecef& eye, double occluderRadiusM, const NodeBox& box);

// ---- LOD ---------------------------------------------------------------------------------------
struct LodConfig {
    float pixelsPerCell = 9.0f;      // target projected size of one cell (virtual 480-line screen)
    float virtualHeightPx = 480.0f;  // LOD is independent of the real window size
    float morphStartFrac = 0.7f;     // geomorph begins at this fraction of the parent's range
    size_t maxRecords = 1600;        // node cache cap (records with or without meshes)
    uint64_t keepFrames = 300;       // unused records older than this are dropped
    int requestRepeatFrames = 45;    // re-request a still missing tile at most this often
};

// Split range in metres for a node with the given mean cell size.
double splitRangeM(double cellMeanM, float vFovRad, const LodConfig& cfg);

struct ViewParams {
    Ecef eye;
    GeoPos eyeGeo;
    Mat3d ecefToEye;
    float vFovRad = 0.96f, aspect = 1.33f, nearM = 0.5f, farM = 40000.0f;
};

struct NodeRecord {
    NodeId id;
    NodeBox box;
    float cellMeanM = 0;
    float minHeightM = 0, maxHeightM = 0;  // exact once a mesh exists, else the tile's range
    int boxSourceLevel = -1;
    int bestSourceLevel = -1;  // finest resident source tile now (cached per residentRevision)
    uint64_t bestSourceRev = ~0ull;
    // mesh state (the GL buffer itself is owned by TerrainRenderer)
    bool meshReady = false;
    bool isSea = false;
    int meshSourceLevel = -1;
    bool flattenDirty = false;
    uint32_t vbo = 0;
    float meshSkirtDepthM = 0;
    Ecef anchor;  // mesh anchor (node centre at height 0), valid when meshReady
    uint64_t lastUseFrame = 0;
    uint64_t queuedFrame = 0;  // frame the node was last put in the build queue
    bool buildFailed = false;
};

struct DrawNode {
    NodeRecord* rec = nullptr;
    float distM = 0;
    float morphStartM = 1e30f, morphInvRangeM = 0;
};

struct BuildRequest {
    NodeId id;
    float distM = 0;
    int priority = 0;  // 0 = no mesh yet (needed), 1 = refresh of a stale mesh
};

struct SelectionStats {
    int visited = 0, drawn = 0, culledFrustum = 0, culledHorizon = 0, noData = 0;
    int seaSkipped = 0, buildQueued = 0, refreshQueued = 0, maxLevelDrawn = 0, minLevelDrawn = 99;
    int roots = 0;
};

class TerrainSelector {
public:
    TerrainSelector(terrain::TerrainService& svc, const LodConfig& cfg = {}) : svc_(svc), cfg_(cfg) {}
    void setConfig(const LodConfig& cfg) { cfg_ = cfg; }
    const LodConfig& config() const { return cfg_; }

    // One selection pass. Fills `draw` (front to back) and `builds` (unsorted, deduplicated).
    void select(const ViewParams& view, std::vector<DrawNode>& draw, std::vector<BuildRequest>& builds);

    // Records a finished build: refreshes the record's box, source level and flags.
    void applyMesh(NodeRecord& rec, const NodeMesh& mesh, uint32_t vbo);
    void markBuildFailed(NodeRecord& rec) { rec.buildFailed = true; }

    NodeRecord* find(const NodeId& id);
    // Drops stale records; returns their VBOs for the renderer to delete.
    void collectGarbage(std::vector<uint32_t>& vbosToDelete);
    // Forget every mesh (e.g. after a GL context change); returns the VBOs.
    void clear(std::vector<uint32_t>& vbosToDelete);

    size_t recordCount() const { return records_.size(); }
    const SelectionStats& stats() const { return stats_; }
    uint64_t frame() const { return frame_; }
    // Mean metres per cell -> split range for this selector's last view.
    double lastFocalPx() const { return focalPx_; }

private:
    NodeRecord* getRecord(const NodeId& id);
    void refreshBest(NodeRecord& rec);
    void visit(const NodeId& id);
    bool tryExpand(NodeRecord& rec, double rangeM);
    bool relevantForReady(const NodeRecord& rec, double distM) const;
    void queueBuild(NodeRecord& rec, double distM, int priority);
    void requestTile(const terrain::TileId& id, double distM, double rangeM);
    void updateFlattenState();

    terrain::TerrainService& svc_;
    LodConfig cfg_;
    std::unordered_map<uint64_t, NodeRecord> records_;
    uint64_t frame_ = 0;
    uint64_t sweptRevision_ = ~0ull;

    // per-pass state
    ViewParams view_;
    Frustum frustum_;
    double occluderRadiusM_ = 6.3e6;
    double focalPx_ = 480;
    Vec3d fwdEcef_;
    std::vector<DrawNode>* draw_ = nullptr;
    std::vector<BuildRequest>* builds_ = nullptr;
    std::vector<terrain::TileId> rootScratch_;
    std::unordered_map<uint64_t, uint64_t> requested_;  // tile key -> frame last requested
    unsigned flattenRev_ = ~0u;
    struct LatLonBox {
        double south, west, north, east;
    };
    std::vector<LatLonBox> flattenBoxes_;
    SelectionStats stats_;
};

}  // namespace fsim::world
