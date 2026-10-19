#pragma once
// TerrainService: the runtime side of the terrain pack (WP W1). It implements core::TerrainQuery
// for the flight model, HUD and nav code, and exposes a tile API for the terrain renderer (W2).
//
// ---- Threading (ARCHITECTURE section 3) --------------------------------------------------------
//  * ALL public methods, including every TerrainQuery method, must be called from ONE thread, the
//    main thread (the one that calls update()). Nothing here is internally synchronised for
//    callers. JSBSim runs on the main thread too, so ground queries are safe.
//  * One worker thread (TileLoader) does file IO and decoding. It never touches GL, JSBSim or any
//    TerrainService state; finished tiles come back through a queue and are integrated into the
//    cache inside update() (or waitForIdle()).
//  * Nothing on the query path blocks: elevationM() uses the finest tile that is resident right
//    now and falls back to ancestors, then to 0 m. Only waitForIdle() blocks, and it exists for
//    tests, tools and a loading screen, never for the frame loop.
//  * The destructor stops and joins the worker.
//
// ---- Data semantics (DATA.md section 4) -------------------------------------------------------
//  * Heights are bilinear over the corner-sampled 257x257 grid of the finest resident tile that
//    covers the point (north-up rows, shared edges). A tile that is listed in the pack but not
//    loaded yet is skipped, so a query is always answered by the finest RESIDENT level.
//  * A tile missing from the pack (bitmap bit clear) means "ocean or not built": the finest
//    ancestor that is present is used, and if no level covers the point the answer is 0 m / Water.
//  * A void sample (-32768) in the four neighbours of a point makes that tile unusable at that
//    point; the next coarser tile answers instead.
//  * Land class is cell-centred (nearest cell) and read from the finest resident tile that has an
//    .lc8. With no resident tile: Water if the pack has no tile there at all, otherwise Unknown
//    (not loaded yet).
//  * Parents and children differ by smoothing (samples are area averages), so heights jump by a
//    small amount where the answering level changes. That is expected and never exceeds the
//    local relief at the coarser spacing.
//
// ---- Loading policy ----------------------------------------------------------------------------
//  update(camLat, camLon) keeps a ring of tiles around the camera: level k (finest = pack max)
//  is wanted within ringRadiusM(k) = detailRadiusM * ringScale^(finest-k) of the camera, so every
//  level has about the same number of tiles. Requests are sorted by distance normalised to the ring
//  (nearest first, coarser first on ties) and the wanted set is cut to fit the cache cap, so the
//  cache never thrashes. prefetch() and requestTile() add higher-priority requests. Tiles that are
//  no longer wanted stay until the LRU cache needs the room (unwanted and least recently used go
//  first; every query touches the tile it used). residentBytes never exceeds cacheMaxBytes after
//  update() returns. A tile evicted from the cache stays valid for any renderer still holding its
//  shared_ptr (that memory is then outside the cache budget).
//
// ---- API for the terrain renderer (W2) ---------------------------------------------------------
//    scheme math         world/terrain/TileScheme.h (TileId, bounds, parent/children,
//                        tileAt, tilesIntersectingCircle, spans and spacings)
//    enumerate tiles     residentTiles(out), residentRevision() (changes on any load/evict),
//                        tile(id), finestTileAt(lat, lon), isTileResident(id)
//    request tiles       requestTile(id, priority01), prefetch(lat, lon, radiusM)
//    LOD inputs          ringRadiusM(k), finestLevel(), coarsestLevel(), packHasTile(id)
//    flatten areas       flattenSet() (+ its revision()) so meshes can bake runway pads
//    tile data           TerrainTile (heightsM 257x257, landClass 256x256, min/max, bounds)
//    diagnostics         stats()

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include "core/TerrainQuery.h"
#include "world/terrain/FlattenSet.h"
#include "world/terrain/TerrainPack.h"
#include "world/terrain/TerrainTile.h"
#include "world/terrain/TileLoader.h"
#include "world/terrain/TileScheme.h"

namespace fsim::terrain {

struct TerrainConfig {
    size_t cacheMaxBytes = 128u << 20;  // PLAN section 3: terrain cache <= 128 MB
    float detailRadiusM = 20000;        // radius of the finest level around the camera
    float ringScale = 2.0f;             // each coarser level reaches this many times farther
    int finestLevel = -1;               // -1 = finest level in the pack
    int coarsestLevel = -1;             // -1 = coarsest level in the pack
    bool loadLandClass = true;
    float rescheduleDistM = 250;  // recompute the wanted set after the camera moved this far
    float readyMarginM = 400;     // isDetailReady also needs the tiles this far around the point
    int debugLoadDelayMs = 0;     // tests only: slow the worker down
};

struct TerrainStats {
    size_t residentTiles = 0;
    size_t residentBytes = 0;
    size_t cacheMaxBytes = 0;
    size_t wantedTiles = 0;  // in the last wanted set (resident or not)
    size_t queuedTiles = 0;  // waiting for the worker
    bool loading = false;    // the worker is decoding a tile right now
    uint64_t tilesLoaded = 0, tilesFailed = 0, tilesEvicted = 0;
    double loadMsTotal = 0, loadMsMax = 0;  // worker time per tile (file read + decode)
};

class TerrainService : public TerrainQuery {
public:
    // Opens <terrainDir>/pack.ini and the presence bitmaps and starts the worker thread. Check
    // isOpen(): when false, error() says why and every query returns 0 m / Unknown.
    explicit TerrainService(const std::string& terrainDir, const TerrainConfig& config = {});
    ~TerrainService() override;
    TerrainService(const TerrainService&) = delete;
    TerrainService& operator=(const TerrainService&) = delete;

    bool isOpen() const { return open_; }
    const std::string& error() const { return error_; }
    const TerrainPack& pack() const { return pack_; }
    const PackInfo& packInfo() const { return pack_.info(); }
    const TerrainConfig& config() const { return cfg_; }
    int finestLevel() const { return finest_; }      // finest level this service uses
    int coarsestLevel() const { return coarsest_; }  // coarsest level this service uses

    // ---- frame hook ---------------------------------------------------------------------------
    // Call once per frame (or per sim step) from the main thread: integrates finished tiles into
    // the cache, enforces the cap, and (when the camera moved or requests changed) reschedules the
    // worker queue. Cheap when nothing changed.
    void update(double cameraLatDeg, double cameraLonDeg);
    // Blocks until the worker queue is empty and every result is integrated, or the timeout
    // passes. Returns true when idle. For tests, tools and loading screens ONLY. Needs update() to
    // have been called once (or prefetch(), which sets the camera if none was set).
    bool waitForIdle(double timeoutS);

    // ---- TerrainQuery (main thread only, never block) -------------------------------------------
    float elevationM(double latDeg, double lonDeg) const override;
    LandClass landClass(double latDeg, double lonDeg) const override;
    void addFlattenArea(const FlattenArea& area) override { flatten_.add(area); }
    void clearFlattenAreas() override { flatten_.clear(); }
    // Schedules every tile intersecting the circle at all levels (finest first only matters to the
    // cap) with priority over the camera ring. One-shot: once loaded, tiles are ordinary LRU
    // entries kept alive by the camera ring and by queries. At most half the cache is used.
    void prefetch(double latDeg, double lonDeg, float radiusM) override;
    // True when, for the point and four points readyMarginM around it, the finest tile the pack
    // offers is resident (or no tile covers the point, in which case there is nothing to wait for).
    bool isDetailReady(double latDeg, double lonDeg) const override;

    // ---- extras for W2 and diagnostics ---------------------------------------------------------
    // Elevation without flatten areas.
    float naturalElevationM(double latDeg, double lonDeg) const;
    // Level whose tile answered (-1 if none is resident).
    int answeringLevel(double latDeg, double lonDeg) const;
    std::shared_ptr<const TerrainTile> finestTileAt(double latDeg, double lonDeg) const;
    std::shared_ptr<const TerrainTile> tile(const TileId& id) const;  // null if not resident
    bool isTileResident(const TileId& id) const { return residentEntry(id) != nullptr; }
    bool packHasTile(const TileId& id) const { return pack_.hasTile(id); }
    // Appends every resident tile (shared, immutable). Re-enumerate when residentRevision() moves.
    void residentTiles(std::vector<std::shared_ptr<const TerrainTile>>& out) const;
    uint64_t residentRevision() const { return revision_; }
    // Asks for one tile. priority01 in [0,1], 0 = most urgent; explicit requests precede
    // prefetch and the camera ring. Ignored if the tile is not in the pack or already resident.
    void requestTile(const TileId& id, float priority01 = 0.0f);
    // Radius around the camera within which level k is wanted.
    double ringRadiusM(int level) const;
    const FlattenSet& flattenSet() const { return flatten_; }
    TerrainStats stats() const;

private:
    struct Entry {
        std::shared_ptr<const TerrainTile> tile;
        mutable uint64_t lastUse = 0;
    };
    struct LevelCache {
        uint64_t generation = ~0ull;
        uint32_t x = 0, y = 0;
        const Entry* entry = nullptr;
    };
    struct Want {
        TileId id;
        float priority;
        int kind;  // 0 explicit, 1 prefetch, 2 camera ring
    };

    const Entry* residentEntry(const TileId& id) const;  // uncached lookup
    const Entry* findEntry(const TileId& id) const;      // per-level one-entry cache
    const Entry* sampleNatural(double latDeg, double lonDeg, float& outM) const;
    bool pointReady(double latDeg, double lonDeg) const;
    void integrateResults();
    void enforceCap();
    void scheduleLoads();

    TerrainConfig cfg_;
    TerrainPack pack_;
    bool open_ = false;
    std::string error_;
    int finest_ = -1, coarsest_ = -1;
    std::unique_ptr<TileLoader> loader_;

    std::unordered_map<uint64_t, Entry> levels_[kMaxLevel + 1];
    size_t levelCount_[kMaxLevel + 1] = {};
    mutable LevelCache cache_[kMaxLevel + 1];
    size_t residentBytes_ = 0;
    uint64_t generation_ = 0;  // bumps on every insert/evict (invalidates cache_)
    uint64_t revision_ = 0;    // same, public
    uint64_t frame_ = 1;  // update() counter, used as the LRU clock

    std::unordered_set<uint64_t> failed_;
    std::unordered_set<uint64_t> wantedKeys_;
    std::unordered_map<uint64_t, float> explicit_;        // key -> priority01
    std::unordered_map<uint64_t, float> prefetchPending_;  // key -> priority (-1..-0.5)
    bool dirty_ = false;
    bool haveCamera_ = false;
    double camLatDeg_ = 0, camLonDeg_ = 0;      // latest update()
    double schedLatDeg_ = 0, schedLonDeg_ = 0;  // camera at the last scheduleLoads()
    std::vector<TileId> scratchTiles_;
    std::vector<Want> scratchWants_;
    std::vector<LoadResult> scratchResults_;
    FlattenSet flatten_;

    TerrainStats stats_;
};

}  // namespace fsim::terrain
