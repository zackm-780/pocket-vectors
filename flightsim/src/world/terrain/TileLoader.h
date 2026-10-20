#pragma once
// The terrain worker thread (ARCHITECTURE section 3): file IO and decoding only. It never touches
// GL, JSBSim or any TerrainService state. The main thread hands it a prioritised request list and
// collects finished tiles from a result queue. One mutex guards the request queue, the in-flight
// marker and the result queue, so "queue empty and nothing in flight" is observed atomically.

#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "world/terrain/TerrainPack.h"

namespace fsim::terrain {

struct LoadRequest {
    TileId id;
    float priority = 0;  // lower = sooner
    bool wantLandClass = true;
};

struct LoadResult {
    TileId id;
    std::shared_ptr<const TerrainTile> tile;  // null on failure
    std::string error;
    double loadMs = 0;
};

class TileLoader {
public:
    // `pack` must outlive the loader. debugDelayMs sleeps after each load (tests of the
    // never-block guarantee).
    TileLoader(const TerrainPack& pack, int debugDelayMs);
    ~TileLoader();  // stops and joins the thread
    TileLoader(const TileLoader&) = delete;
    TileLoader& operator=(const TileLoader&) = delete;

    // Replaces the pending queue (the tile currently being decoded is not interrupted). Requests
    // for a tile that is in flight or whose result has not been collected yet are skipped.
    void replaceQueue(std::vector<LoadRequest> requests);
    // Moves finished results into `out` (appends). Returns the number moved.
    size_t drainResults(std::vector<LoadResult>& out);

    size_t queued() const;
    bool busy() const;  // a tile is being decoded
    // Nothing queued, nothing in flight and no result waiting to be collected.
    bool idle() const;
    // Blocks until the loader may have changed state or the timeout passes (used by
    // TerrainService::waitForIdle, never on the frame path).
    void waitForChange(double timeoutS);

private:
    void run();

    const TerrainPack& pack_;
    const int debugDelayMs_;
    mutable std::mutex mutex_;
    std::condition_variable wake_;     // main -> worker: new work or stop
    std::condition_variable changed_;  // worker -> main: a result was published / queue drained
    std::vector<LoadRequest> queue_;   // sorted by priority ascending
    std::vector<LoadResult> results_;
    bool inFlight_ = false;
    TileId inFlightId_;
    bool stop_ = false;
    std::thread thread_;
};

}  // namespace fsim::terrain
