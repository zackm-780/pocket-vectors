#include "world/terrain/TerrainService.h"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "core/Log.h"
#include "core/Units.h"

namespace fsim::terrain {
namespace {

constexpr double kMPerDeg = kEarthRadiusM * kRadPerDeg;
// Rough size of a tile with and without land class, used to budget tiles that are not loaded yet.
constexpr size_t kEstimatedHeightBytes = static_cast<size_t>(kTileSamples) * kTileSamples * 2;
constexpr size_t kEstimatedLandBytes = static_cast<size_t>(kTileCells) * kTileCells;
constexpr size_t kMaxPrefetchPending = 20000;

double distM(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg) {
    const double dLat = lat2Deg - lat1Deg;
    const double dLon = normalizeLonDeg(lon2Deg - lon1Deg);
    const double midLat = 0.5 * (lat1Deg + lat2Deg) * kRadPerDeg;
    return std::hypot(dLat, dLon * std::cos(midLat)) * kMPerDeg;
}

}  // namespace

TerrainService::TerrainService(const std::string& terrainDir, const TerrainConfig& config)
    : cfg_(config) {
    cfg_.cacheMaxBytes = std::max<size_t>(cfg_.cacheMaxBytes, 1u << 20);
    if (!pack_.open(terrainDir, error_)) {
        LOG_WARN("terrain: %s", error_.c_str());
        return;
    }
    if (pack_.finestLevel() < 0) {
        error_ = "terrain pack at " + terrainDir + " lists no tiles";
        LOG_WARN("terrain: %s", error_.c_str());
        return;
    }
    finest_ = pack_.finestLevel();
    coarsest_ = pack_.coarsestLevel();
    if (cfg_.finestLevel >= 0) finest_ = std::min(finest_, cfg_.finestLevel);
    if (cfg_.coarsestLevel >= 0) coarsest_ = std::max(coarsest_, cfg_.coarsestLevel);
    if (coarsest_ > finest_) coarsest_ = finest_;
    open_ = true;
    loader_ = std::make_unique<TileLoader>(pack_, cfg_.debugLoadDelayMs);
    LOG_INFO("terrain: pack %s levels %d..%d (using %d..%d), cache cap %zu MB", terrainDir.c_str(),
             pack_.coarsestLevel(), pack_.finestLevel(), coarsest_, finest_,
             cfg_.cacheMaxBytes >> 20);
}

TerrainService::~TerrainService() {
    loader_.reset();  // joins the worker before the pack it reads goes away
}

// ---- lookup ------------------------------------------------------------------------------------

const TerrainService::Entry* TerrainService::residentEntry(const TileId& id) const {
    if (id.k < 0 || id.k > kMaxLevel) return nullptr;
    const auto& m = levels_[id.k];
    const auto it = m.find(key(id));
    return it == m.end() ? nullptr : &it->second;
}

const TerrainService::Entry* TerrainService::findEntry(const TileId& id) const {
    LevelCache& c = cache_[id.k];
    if (c.generation == generation_ && c.x == id.x && c.y == id.y) return c.entry;
    const Entry* e = residentEntry(id);
    c.generation = generation_;
    c.x = id.x;
    c.y = id.y;
    c.entry = e;
    return e;
}

const TerrainService::Entry* TerrainService::sampleNatural(double latDeg, double lonDeg,
                                                           float& outM) const {
    for (int k = finest_; k >= coarsest_; --k) {
        if (levelCount_[k] == 0) continue;
        const Entry* e = findEntry(tileAt(k, latDeg, lonDeg));
        if (e && e->tile->sampleHeightM(latDeg, lonDeg, outM)) return e;
    }
    return nullptr;
}

float TerrainService::naturalElevationM(double latDeg, double lonDeg) const {
    if (!open_) return 0.0f;
    float h = 0.0f;
    const Entry* e = sampleNatural(latDeg, lonDeg, h);
    if (!e) return 0.0f;
    e->lastUse = frame_;
    return h;
}

float TerrainService::elevationM(double latDeg, double lonDeg) const {
    return flatten_.elevationM(latDeg, lonDeg,
                               [&] { return naturalElevationM(latDeg, lonDeg); });
}

int TerrainService::answeringLevel(double latDeg, double lonDeg) const {
    if (!open_) return -1;
    float h;
    const Entry* e = sampleNatural(latDeg, lonDeg, h);
    return e ? e->tile->id.k : -1;
}

std::shared_ptr<const TerrainTile> TerrainService::finestTileAt(double latDeg,
                                                                double lonDeg) const {
    if (!open_) return nullptr;
    for (int k = finest_; k >= coarsest_; --k) {
        if (levelCount_[k] == 0) continue;
        if (const Entry* e = findEntry(tileAt(k, latDeg, lonDeg))) return e->tile;
    }
    return nullptr;
}

std::shared_ptr<const TerrainTile> TerrainService::tile(const TileId& id) const {
    const Entry* e = residentEntry(id);
    return e ? e->tile : nullptr;
}

void TerrainService::residentTiles(std::vector<std::shared_ptr<const TerrainTile>>& out) const {
    for (const auto& level : levels_)
        for (const auto& kv : level) out.push_back(kv.second.tile);
}

LandClass TerrainService::landClass(double latDeg, double lonDeg) const {
    if (!open_) return LandClass::Water;
    for (int k = finest_; k >= coarsest_; --k) {
        if (levelCount_[k] == 0) continue;
        const Entry* e = findEntry(tileAt(k, latDeg, lonDeg));
        if (e && e->tile->hasLandClass()) {
            e->lastUse = frame_;
            return e->tile->landClassAt(latDeg, lonDeg);
        }
    }
    // Nothing usable resident: Water when the pack has no tile there at all (ocean or not built),
    // Unknown when a tile exists but has not been loaded yet or has no land class.
    return pack_.finestLevelAt(latDeg, lonDeg) < 0 ? LandClass::Water : LandClass::Unknown;
}

bool TerrainService::pointReady(double latDeg, double lonDeg) const {
    for (int k = finest_; k >= coarsest_; --k) {
        const TileId id = tileAt(k, latDeg, lonDeg);
        if (!pack_.hasTile(id)) continue;
        if (!failed_.empty() && failed_.count(key(id))) continue;
        return findEntry(id) != nullptr;
    }
    return true;
}

bool TerrainService::isDetailReady(double latDeg, double lonDeg) const {
    if (!open_) return true;  // nothing will ever load: do not gate on it
    if (!pointReady(latDeg, lonDeg)) return false;
    const double dLat = cfg_.readyMarginM / kMPerDeg;
    const double cosLat = std::max(0.05, std::cos(latDeg * kRadPerDeg));
    const double dLon = dLat / cosLat;
    return pointReady(latDeg + dLat, lonDeg + dLon) && pointReady(latDeg + dLat, lonDeg - dLon) &&
           pointReady(latDeg - dLat, lonDeg + dLon) && pointReady(latDeg - dLat, lonDeg - dLon);
}

// ---- requests ----------------------------------------------------------------------------------

double TerrainService::ringRadiusM(int level) const {
    return cfg_.detailRadiusM * std::pow(static_cast<double>(cfg_.ringScale), finest_ - level);
}

void TerrainService::requestTile(const TileId& id, float priority01) {
    if (!open_ || !isValid(id) || !pack_.hasTile(id) || residentEntry(id)) return;
    if (failed_.count(key(id))) return;
    explicit_[key(id)] = std::min(1.0f, std::max(0.0f, priority01));
    dirty_ = true;
}

void TerrainService::prefetch(double latDeg, double lonDeg, float radiusM) {
    if (!open_) return;
    if (!haveCamera_) {  // so waitForIdle() works before the first update()
        haveCamera_ = true;
        camLatDeg_ = schedLatDeg_ = latDeg;
        camLonDeg_ = schedLonDeg_ = lonDeg;
    }
    const double r = std::max(1.0f, radiusM);
    for (int k = coarsest_; k <= finest_; ++k) {
        scratchTiles_.clear();
        tilesIntersectingCircle(k, latDeg, lonDeg, r, scratchTiles_);
        for (const TileId& id : scratchTiles_) {
            if (!pack_.hasTile(id) || residentEntry(id) || failed_.count(key(id))) continue;
            if (prefetchPending_.size() >= kMaxPrefetchPending) break;
            const double d = distToTileM(id, latDeg, lonDeg);
            const float pri = static_cast<float>(-1.0 + 0.5 * std::min(1.0, d / r) + 1e-4 * k);
            auto it = prefetchPending_.find(key(id));
            if (it == prefetchPending_.end())
                prefetchPending_.emplace(key(id), pri);
            else
                it->second = std::min(it->second, pri);
        }
    }
    dirty_ = true;
}

// ---- frame update ------------------------------------------------------------------------------

void TerrainService::update(double cameraLatDeg, double cameraLonDeg) {
    if (!open_) return;
    ++frame_;
    integrateResults();
    camLatDeg_ = cameraLatDeg;
    camLonDeg_ = cameraLonDeg;
    if (!haveCamera_ ||
        distM(cameraLatDeg, cameraLonDeg, schedLatDeg_, schedLonDeg_) > cfg_.rescheduleDistM) {
        haveCamera_ = true;
        dirty_ = true;
    }
    if (dirty_) scheduleLoads();
}

bool TerrainService::waitForIdle(double timeoutS) {
    if (!open_) return true;
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                              std::chrono::duration<double>(timeoutS));
    for (;;) {
        integrateResults();
        if (dirty_ && haveCamera_) scheduleLoads();
        if (loader_->idle()) return true;
        if (std::chrono::steady_clock::now() >= deadline) return false;
        loader_->waitForChange(0.01);
    }
}

void TerrainService::integrateResults() {
    scratchResults_.clear();
    loader_->drainResults(scratchResults_);
    bool inserted = false;
    for (LoadResult& r : scratchResults_) {
        const uint64_t k = key(r.id);
        stats_.loadMsTotal += r.loadMs;
        stats_.loadMsMax = std::max(stats_.loadMsMax, r.loadMs);
        if (!r.tile) {
            ++stats_.tilesFailed;
            if (failed_.insert(k).second)
                LOG_WARN("terrain: tile %d/%u/%u failed: %s", r.id.k, r.id.x, r.id.y,
                         r.error.c_str());
            continue;
        }
        if (residentEntry(r.id)) continue;
        Entry e;
        e.tile = std::move(r.tile);
        e.lastUse = frame_;
        residentBytes_ += e.tile->memoryBytes();
        ++levelCount_[r.id.k];
        levels_[r.id.k].emplace(k, std::move(e));
        ++stats_.tilesLoaded;
        inserted = true;
    }
    if (inserted) {
        ++generation_;
        ++revision_;
        enforceCap();
    }
}

void TerrainService::enforceCap() {
    if (residentBytes_ <= cfg_.cacheMaxBytes) return;
    struct Candidate {
        bool wanted;
        uint64_t lastUse;
        int level;
        uint64_t key;
    };
    std::vector<Candidate> cands;
    for (int k = 0; k <= kMaxLevel; ++k)
        for (const auto& kv : levels_[k])
            cands.push_back({wantedKeys_.count(kv.first) != 0, kv.second.lastUse, k, kv.first});
    // Evict first: unwanted, then least recently used, then the finer level.
    std::sort(cands.begin(), cands.end(), [](const Candidate& a, const Candidate& b) {
        if (a.wanted != b.wanted) return !a.wanted;
        if (a.lastUse != b.lastUse) return a.lastUse < b.lastUse;
        if (a.level != b.level) return a.level > b.level;
        return a.key < b.key;
    });
    for (const Candidate& c : cands) {
        if (residentBytes_ <= cfg_.cacheMaxBytes) break;
        auto& m = levels_[c.level];
        const auto it = m.find(c.key);
        residentBytes_ -= it->second.tile->memoryBytes();
        m.erase(it);
        --levelCount_[c.level];
        ++stats_.tilesEvicted;
    }
    ++generation_;
    ++revision_;
}

void TerrainService::scheduleLoads() {
    dirty_ = false;
    schedLatDeg_ = camLatDeg_;
    schedLonDeg_ = camLonDeg_;

    scratchWants_.clear();
    std::unordered_map<uint64_t, size_t> index;
    auto add = [&](const TileId& id, float pri, int kind) {
        const uint64_t k = key(id);
        if (!pack_.hasTile(id) || failed_.count(k)) return;
        const auto it = index.find(k);
        if (it != index.end()) {
            Want& w = scratchWants_[it->second];
            if (pri < w.priority) {
                w.priority = pri;
                w.kind = kind;
            }
            return;
        }
        index.emplace(k, scratchWants_.size());
        scratchWants_.push_back({id, pri, kind});
    };

    for (auto it = explicit_.begin(); it != explicit_.end();) {
        const TileId id = fromKey(it->first);
        if (residentEntry(id) || failed_.count(it->first) || !pack_.hasTile(id)) {
            it = explicit_.erase(it);
            continue;
        }
        add(id, -2.0f + it->second, 0);
        ++it;
    }
    for (auto it = prefetchPending_.begin(); it != prefetchPending_.end();) {
        const TileId id = fromKey(it->first);
        if (residentEntry(id) || failed_.count(it->first)) {
            it = prefetchPending_.erase(it);
            continue;
        }
        add(id, it->second, 1);
        ++it;
    }
    for (int k = coarsest_; k <= finest_; ++k) {
        if (!pack_.levelHasTiles(k)) continue;
        const double radius = ringRadiusM(k);
        scratchTiles_.clear();
        tilesIntersectingCircle(k, schedLatDeg_, schedLonDeg_, radius, scratchTiles_);
        for (const TileId& id : scratchTiles_) {
            const double d = distToTileM(id, schedLatDeg_, schedLonDeg_);
            add(id, static_cast<float>(d / radius + 1e-4 * k), 2);
        }
    }
    std::stable_sort(scratchWants_.begin(), scratchWants_.end(),
                     [](const Want& a, const Want& b) { return a.priority < b.priority; });

    // Keep the wanted set within the cache cap so loading never thrashes.
    const size_t budget = cfg_.cacheMaxBytes / 10 * 9;
    const size_t prefetchBudget = cfg_.cacheMaxBytes / 2;
    const size_t estBytes = kEstimatedHeightBytes + (cfg_.loadLandClass ? kEstimatedLandBytes : 0);
    size_t used = 0, prefetchUsed = 0;
    wantedKeys_.clear();
    std::vector<LoadRequest> queue;
    for (const Want& w : scratchWants_) {
        const Entry* e = residentEntry(w.id);
        const size_t bytes = e ? e->tile->memoryBytes() : estBytes;
        if (w.kind == 1 && prefetchUsed + bytes > prefetchBudget) continue;
        if (used + bytes > budget) break;
        used += bytes;
        if (w.kind == 1) prefetchUsed += bytes;
        wantedKeys_.insert(key(w.id));
        if (!e) queue.push_back({w.id, w.priority, cfg_.loadLandClass});
    }
    stats_.wantedTiles = wantedKeys_.size();
    loader_->replaceQueue(std::move(queue));
}

TerrainStats TerrainService::stats() const {
    TerrainStats s = stats_;
    s.cacheMaxBytes = cfg_.cacheMaxBytes;
    s.residentBytes = residentBytes_;
    s.residentTiles = 0;
    for (const size_t n : levelCount_) s.residentTiles += n;
    if (loader_) {
        s.queuedTiles = loader_->queued();
        s.loading = loader_->busy();
    }
    return s;
}

}  // namespace fsim::terrain
