#include "world/terrain/TileLoader.h"

#include <algorithm>
#include <chrono>

namespace fsim::terrain {

TileLoader::TileLoader(const TerrainPack& pack, int debugDelayMs)
    : pack_(pack), debugDelayMs_(debugDelayMs) {
    thread_ = std::thread([this] { run(); });
}

TileLoader::~TileLoader() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        stop_ = true;
        queue_.clear();
    }
    wake_.notify_all();
    if (thread_.joinable()) thread_.join();
}

void TileLoader::replaceQueue(std::vector<LoadRequest> requests) {
    std::stable_sort(requests.begin(), requests.end(),
                     [](const LoadRequest& a, const LoadRequest& b) { return a.priority < b.priority; });
    {
        std::lock_guard<std::mutex> lock(mutex_);
        auto skip = [this](const LoadRequest& r) {
            if (inFlight_ && inFlightId_ == r.id) return true;
            for (const LoadResult& res : results_)
                if (res.id == r.id) return true;
            return false;
        };
        requests.erase(std::remove_if(requests.begin(), requests.end(), skip), requests.end());
        queue_ = std::move(requests);
    }
    wake_.notify_one();
    changed_.notify_all();
}

size_t TileLoader::drainResults(std::vector<LoadResult>& out) {
    std::lock_guard<std::mutex> lock(mutex_);
    const size_t n = results_.size();
    for (LoadResult& r : results_) out.push_back(std::move(r));
    results_.clear();
    return n;
}

size_t TileLoader::queued() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.size();
}

bool TileLoader::busy() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return inFlight_;
}

bool TileLoader::idle() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return queue_.empty() && !inFlight_ && results_.empty();
}

void TileLoader::waitForChange(double timeoutS) {
    std::unique_lock<std::mutex> lock(mutex_);
    changed_.wait_for(lock, std::chrono::duration<double>(timeoutS));
}

void TileLoader::run() {
    for (;;) {
        LoadRequest req;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            wake_.wait(lock, [this] { return stop_ || !queue_.empty(); });
            if (stop_) return;
            req = queue_.front();
            queue_.erase(queue_.begin());
            inFlight_ = true;
            inFlightId_ = req.id;
        }

        const auto t0 = std::chrono::steady_clock::now();
        LoadResult res;
        res.id = req.id;
        res.tile = pack_.loadTile(req.id, req.wantLandClass, res.error);
        if (debugDelayMs_ > 0)
            std::this_thread::sleep_for(std::chrono::milliseconds(debugDelayMs_));
        res.loadMs =
            std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();

        {
            std::lock_guard<std::mutex> lock(mutex_);
            results_.push_back(std::move(res));
            inFlight_ = false;
        }
        changed_.notify_all();
    }
}

}  // namespace fsim::terrain
