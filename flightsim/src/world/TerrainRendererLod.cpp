#include "world/TerrainRendererLod.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace fsim::world {
namespace {

constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

double dotV(const Vec3d& a, const Vec3d& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3d axisEast(double lonRad) {
    return {-std::sin(lonRad), std::cos(lonRad), 0};
}
Vec3d axisNorth(double latRad, double lonRad) {
    return {-std::sin(latRad) * std::cos(lonRad), -std::sin(latRad) * std::sin(lonRad),
            std::cos(latRad)};
}
Vec3d axisUp(double latRad, double lonRad) {
    return {std::cos(latRad) * std::cos(lonRad), std::cos(latRad) * std::sin(lonRad),
            std::sin(latRad)};
}

}  // namespace

NodeBox computeNodeBox(const NodeId& id, double minHeightM, double maxHeightM) {
    const terrain::TileBounds b = terrain::bounds(id);
    const double cLat = b.centerLatDeg() * kRadPerDeg, cLon = b.centerLonDeg() * kRadPerDeg;
    NodeBox box;
    box.axis[0] = axisEast(cLon);
    box.axis[1] = axisNorth(cLat, cLon);
    box.axis[2] = axisUp(cLat, cLon);

    GeoPos cg;
    cg.latDeg = b.centerLatDeg();
    cg.lonDeg = b.centerLonDeg();
    cg.altMslM = 0;
    const Ecef c0 = geodeticToEcef(cg);

    double lo[3] = {1e30, 1e30, 1e30}, hi[3] = {-1e30, -1e30, -1e30};
    const double lats[3] = {b.southDeg, b.centerLatDeg(), b.northDeg};
    const double lons[3] = {b.westDeg, b.centerLonDeg(), b.eastDeg};
    const double heights[2] = {minHeightM, maxHeightM};
    for (double lat : lats) {
        for (double lon : lons) {
            for (double h : heights) {
                GeoPos p;
                p.latDeg = std::max(-90.0, std::min(90.0, lat));
                p.lonDeg = lon;
                p.altMslM = h;
                const Ecef e = geodeticToEcef(p);
                const Vec3d d{e.x - c0.x, e.y - c0.y, e.z - c0.z};
                for (int k = 0; k < 3; ++k) {
                    const double v = dotV(box.axis[k], d);
                    lo[k] = std::min(lo[k], v);
                    hi[k] = std::max(hi[k], v);
                }
            }
        }
    }
    Vec3d mid{0, 0, 0};
    for (int k = 0; k < 3; ++k) {
        const double m = 0.5 * (lo[k] + hi[k]);
        box.half[k] = 0.5 * (hi[k] - lo[k]);
        mid.x += m * box.axis[k].x;
        mid.y += m * box.axis[k].y;
        mid.z += m * box.axis[k].z;
    }
    box.centre = {c0.x + mid.x, c0.y + mid.y, c0.z + mid.z};
    box.boundRadiusM =
        std::sqrt(box.half[0] * box.half[0] + box.half[1] * box.half[1] + box.half[2] * box.half[2]);
    return box;
}

double distanceToBox(const NodeBox& box, const Ecef& p) {
    const Vec3d d{p.x - box.centre.x, p.y - box.centre.y, p.z - box.centre.z};
    double sum = 0;
    for (int k = 0; k < 3; ++k) {
        const double v = std::fabs(box.axis[k].x * d.x + box.axis[k].y * d.y + box.axis[k].z * d.z) -
                         box.half[k];
        if (v > 0) sum += v * v;
    }
    return std::sqrt(sum);
}

Frustum makeFrustum(const Mat3d& ecefToEye, float vFovRad, float aspect, float nearM, float farM) {
    const double tanV = std::tan(0.5 * vFovRad), tanH = tanV * aspect;
    // Inward normals in eye axes (x right, y up, -z forward) and plane offsets.
    const double ne[6][3] = {{1, 0, -tanH}, {-1, 0, -tanH}, {0, 1, -tanV},
                             {0, -1, -tanV}, {0, 0, -1},    {0, 0, 1}};
    const double off[6] = {0, 0, 0, 0, -static_cast<double>(nearM), static_cast<double>(farM)};
    Frustum f;
    for (int p = 0; p < 6; ++p) {
        double len = std::sqrt(ne[p][0] * ne[p][0] + ne[p][1] * ne[p][1] + ne[p][2] * ne[p][2]);
        for (int j = 0; j < 3; ++j) {
            // n_ecef = R^T n_eye
            f.n[p][j] = (ecefToEye.m[0][j] * ne[p][0] + ecefToEye.m[1][j] * ne[p][1] +
                         ecefToEye.m[2][j] * ne[p][2]) /
                        len;
        }
        f.d[p] = off[p] / len;
    }
    return f;
}

bool frustumIntersects(const Frustum& f, const NodeBox& box, const Ecef& eye) {
    const Vec3d rel{box.centre.x - eye.x, box.centre.y - eye.y, box.centre.z - eye.z};
    for (int p = 0; p < 6; ++p) {
        const double dist = f.n[p][0] * rel.x + f.n[p][1] * rel.y + f.n[p][2] * rel.z + f.d[p];
        double r = 0;
        for (int k = 0; k < 3; ++k)
            r += box.half[k] * std::fabs(f.n[p][0] * box.axis[k].x + f.n[p][1] * box.axis[k].y +
                                         f.n[p][2] * box.axis[k].z);
        if (dist + r < 0) return false;
    }
    return true;
}

bool horizonVisible(const Ecef& eye, double occluderRadiusM, const NodeBox& box) {
    const double eyeR = std::sqrt(eye.x * eye.x + eye.y * eye.y + eye.z * eye.z);
    if (eyeR <= occluderRadiusM) return true;
    const double cR = std::sqrt(box.centre.x * box.centre.x + box.centre.y * box.centre.y +
                                box.centre.z * box.centre.z);
    if (cR <= 1.0) return true;
    const double lateral = std::sqrt(box.half[0] * box.half[0] + box.half[1] * box.half[1]);
    const double topR = std::sqrt((cR + box.half[2]) * (cR + box.half[2]) + lateral * lateral);
    double cosT = (eye.x * box.centre.x + eye.y * box.centre.y + eye.z * box.centre.z) / (eyeR * cR);
    cosT = std::max(-1.0, std::min(1.0, cosT));
    const double theta = std::acos(cosT);
    const double alpha = std::asin(std::min(1.0, (lateral + box.half[2]) / cR));
    const double limit = std::acos(std::min(1.0, occluderRadiusM / eyeR)) +
                         std::acos(std::min(1.0, occluderRadiusM / topR));
    return theta - alpha <= limit;
}

double splitRangeM(double cellMeanM, float vFovRad, const LodConfig& cfg) {
    const double focal = 0.5 * cfg.virtualHeightPx / std::tan(0.5 * vFovRad);
    return cellMeanM * focal / cfg.pixelsPerCell;
}

double foreshortenFactor(double sinGrazing, const LodConfig& cfg) {
    const double s = std::min(1.0, std::max<double>(cfg.foreshortenMin, sinGrazing));
    return std::sqrt(s);
}

// ---- TerrainSelector ----------------------------------------------------------------------------
namespace {

void heightBounds(const NodeId& id, double minH, double maxH, double& lo, double& hi) {
    const double pad = 0.1 * (maxH - minH) + 10.0;
    const double skirt = 0.35 * nodeCellDeg(id.k) * kMPerDegLat + 3.0;
    lo = minH - pad - skirt;
    hi = maxH + pad;
}

}  // namespace

NodeRecord* TerrainSelector::find(const NodeId& id) {
    auto it = records_.find(terrain::key(id));
    return it == records_.end() ? nullptr : &it->second;
}

NodeRecord* TerrainSelector::getRecord(const NodeId& id) {
    if (NodeRecord* r = find(id)) return r;
    const int coarsest = svc_.coarsestLevel();
    const int natural = std::max(naturalTileLevel(id.k), coarsest);
    std::shared_ptr<const terrain::TerrainTile> tile;
    for (int k = natural; k >= coarsest && !tile; --k) tile = svc_.tile(terrain::ancestorAt(id, k));
    if (!tile) return nullptr;
    NodeRecord r;
    r.id = id;
    r.cellMeanM = static_cast<float>(nodeCellMeanM(id));
    r.minHeightM = tile->minHeightM;
    r.maxHeightM = tile->maxHeightM;
    r.boxSourceLevel = tile->id.k;
    double lo, hi;
    heightBounds(id, r.minHeightM, r.maxHeightM, lo, hi);
    r.box = computeNodeBox(id, lo, hi);
    r.lastUseFrame = frame_;
    return &records_.emplace(terrain::key(id), r).first->second;
}

void TerrainSelector::refreshBest(NodeRecord& rec) {
    const uint64_t rev = svc_.residentRevision();
    if (rec.bestSourceRev == rev) return;
    rec.bestSourceRev = rev;
    rec.buildFailed = false;
    const int coarsest = svc_.coarsestLevel();
    rec.bestSourceLevel = -1;
    for (int k = std::max(naturalTileLevel(rec.id.k), coarsest); k >= coarsest; --k) {
        if (svc_.isTileResident(terrain::ancestorAt(rec.id, k))) {
            rec.bestSourceLevel = k;
            break;
        }
    }
    if (!rec.meshReady && rec.bestSourceLevel > rec.boxSourceLevel) {
        if (auto tile = svc_.tile(terrain::ancestorAt(rec.id, rec.bestSourceLevel))) {
            rec.minHeightM = tile->minHeightM;
            rec.maxHeightM = tile->maxHeightM;
            rec.boxSourceLevel = rec.bestSourceLevel;
            double lo, hi;
            heightBounds(rec.id, rec.minHeightM, rec.maxHeightM, lo, hi);
            rec.box = computeNodeBox(rec.id, lo, hi);
        }
    }
}

void TerrainSelector::applyMesh(NodeRecord& rec, const NodeMesh& mesh, uint32_t vbo) {
    rec.vbo = vbo;
    rec.meshReady = true;
    rec.buildFailed = false;
    rec.isSea = mesh.allSea;
    rec.meshSourceLevel = mesh.sourceLevel;
    rec.flattenDirty = false;
    rec.minHeightM = mesh.minHeightM;
    rec.maxHeightM = mesh.maxHeightM;
    rec.meshSkirtDepthM = mesh.skirtDepthM;
    rec.anchor = mesh.anchor;
    rec.boxSourceLevel = mesh.sourceLevel;
    double lo, hi;
    heightBounds(rec.id, rec.minHeightM, rec.maxHeightM, lo, hi);
    rec.box = computeNodeBox(rec.id, lo, hi);
}

void TerrainSelector::queueBuild(NodeRecord& rec, double distM, int priority) {
    if (rec.queuedFrame == frame_ || rec.buildFailed) return;
    rec.queuedFrame = frame_;
    builds_->push_back({rec.id, static_cast<float>(distM), priority});
    if (priority == 0)
        ++stats_.buildQueued;
    else
        ++stats_.refreshQueued;
}

void TerrainSelector::requestTile(const terrain::TileId& id, double distM, double rangeM) {
    uint64_t& last = requested_[terrain::key(id)];
    if (last != 0 && frame_ - last < static_cast<uint64_t>(cfg_.requestRepeatFrames)) return;
    last = frame_;
    svc_.requestTile(id, static_cast<float>(std::min(1.0, distM / std::max(1.0, 2.0 * rangeM))));
}

bool TerrainSelector::relevantForReady(const NodeRecord& rec, double distM) const {
    if (distM > view_.farM) return false;
    const Vec3d rel{rec.box.centre.x - view_.eye.x, rec.box.centre.y - view_.eye.y,
                    rec.box.centre.z - view_.eye.z};
    if (rel.x * fwdEcef_.x + rel.y * fwdEcef_.y + rel.z * fwdEcef_.z < -rec.box.boundRadiusM)
        return false;
    return horizonVisible(view_.eye, occluderRadiusM_, rec.box);
}

void TerrainSelector::updateFlattenState() {
    const terrain::FlattenSet& fs = svc_.flattenSet();
    if (fs.revision() == flattenRev_) return;
    flattenRev_ = fs.revision();
    std::vector<LatLonBox> now;
    for (const FlattenArea& a : fs.areas()) {
        LatLonBox b{1e9, 1e9, -1e9, -1e9};
        for (int i = 0; i < 4; ++i) {
            b.south = std::min(b.south, a.latDeg[i]);
            b.north = std::max(b.north, a.latDeg[i]);
            b.west = std::min(b.west, a.lonDeg[i]);
            b.east = std::max(b.east, a.lonDeg[i]);
        }
        const double dLat = (a.blendM + 30.0) / kMPerDegLat;
        const double dLon = dLat / std::max(0.05, std::cos(0.5 * (b.south + b.north) * kRadPerDeg));
        b.south -= dLat, b.north += dLat, b.west -= dLon, b.east += dLon;
        now.push_back(b);
    }
    std::vector<LatLonBox> both = now;  // pads removed since the last revision must be undone too
    both.insert(both.end(), flattenBoxes_.begin(), flattenBoxes_.end());
    flattenBoxes_ = std::move(now);
    for (auto& [k, rec] : records_) {
        if (!rec.meshReady) continue;
        const terrain::TileBounds nb = terrain::bounds(rec.id);
        for (const LatLonBox& b : both) {
            if (nb.eastDeg >= b.west && nb.westDeg <= b.east && nb.northDeg >= b.south &&
                nb.southDeg <= b.north) {
                rec.flattenDirty = true;
                break;
            }
        }
    }
}

void TerrainSelector::select(const ViewParams& view, std::vector<DrawNode>& draw,
                             std::vector<BuildRequest>& builds) {
    ++frame_;
    stats_ = SelectionStats{};
    view_ = view;
    draw_ = &draw;
    builds_ = &builds;
    draw.clear();
    builds.clear();
    updateFlattenState();
    frustum_ = makeFrustum(view.ecefToEye, view.vFovRad, view.aspect, view.nearM, view.farM);
    GeoPos sub;
    sub.latDeg = view.eyeGeo.latDeg;
    sub.lonDeg = view.eyeGeo.lonDeg;
    const Ecef s = geodeticToEcef(sub);
    occluderRadiusM_ = std::sqrt(s.x * s.x + s.y * s.y + s.z * s.z) - 150.0;
    focalPx_ = 0.5 * cfg_.virtualHeightPx / std::tan(0.5 * view.vFovRad);
    fwdEcef_ = {-view.ecefToEye.m[2][0], -view.ecefToEye.m[2][1], -view.ecefToEye.m[2][2]};

    const int coarsest = svc_.coarsestLevel();
    rootScratch_.clear();
    terrain::tilesIntersectingCircle(coarsest, view.eyeGeo.latDeg, view.eyeGeo.lonDeg,
                                     static_cast<double>(view.farM), rootScratch_);
    for (const terrain::TileId& id : rootScratch_) {
        if (!svc_.packHasTile(id)) continue;
        ++stats_.roots;
        visit(id);
    }
    std::sort(draw.begin(), draw.end(),
              [](const DrawNode& a, const DrawNode& b) { return a.distM < b.distM; });
}

void TerrainSelector::visit(const NodeId& id) {
    NodeRecord* recp = getRecord(id);
    if (!recp) {
        ++stats_.noData;
        const int coarsest = svc_.coarsestLevel();
        if (id.k == coarsest) requestTile(id, 0, 1e5);
        return;
    }
    NodeRecord& rec = *recp;
    rec.lastUseFrame = frame_;
    refreshBest(rec);
    if (!frustumIntersects(frustum_, rec.box, view_.eye)) {
        ++stats_.culledFrustum;
        return;
    }
    if (!horizonVisible(view_.eye, occluderRadiusM_, rec.box)) {
        ++stats_.culledHorizon;
        return;
    }
    ++stats_.visited;
    const double d = distanceToBox(rec.box, view_.eye);
    if (!rec.meshReady) {
        queueBuild(rec, d, 0);
        return;
    }
    if ((rec.bestSourceLevel >= 0 && rec.meshSourceLevel < rec.bestSourceLevel) || rec.flattenDirty)
        queueBuild(rec, d, 1);
    if (rec.isSea) {
        ++stats_.seaSkipped;
        return;
    }
    double range = splitRangeM(rec.cellMeanM, view_.vFovRad, cfg_);
    {
        const Vec3d rel{rec.box.centre.x - view_.eye.x, rec.box.centre.y - view_.eye.y,
                        rec.box.centre.z - view_.eye.z};
        const double dc = std::sqrt(rel.x * rel.x + rel.y * rel.y + rel.z * rel.z);
        if (dc > 2.0 * rec.box.boundRadiusM) {
            const Vec3d& up = rec.box.axis[2];
            range *= foreshortenFactor(std::fabs(rel.x * up.x + rel.y * up.y + rel.z * up.z) / dc, cfg_);
        }
    }
    const int maxLevel = svc_.finestLevel() + kPatchShift;
    if (id.k < maxLevel && d < range && tryExpand(rec, range)) return;

    DrawNode dn;
    dn.rec = &rec;
    dn.distM = static_cast<float>(d);
    if (id.k > svc_.coarsestLevel()) {
        const double parentRange = 2.0 * range;
        dn.morphStartM = static_cast<float>(cfg_.morphStartFrac * parentRange);
        dn.morphInvRangeM = static_cast<float>(1.0 / ((1.0 - cfg_.morphStartFrac) * parentRange));
    }
    draw_->push_back(dn);
    ++stats_.drawn;
    stats_.maxLevelDrawn = std::max(stats_.maxLevelDrawn, id.k);
    stats_.minLevelDrawn = std::min(stats_.minLevelDrawn, id.k);
}

bool TerrainSelector::tryExpand(NodeRecord& rec, double rangeM) {
    const NodeId id = rec.id;
    const auto kids = terrain::children(id);
    const int coarsest = svc_.coarsestLevel();
    const int parentTile = std::max(naturalTileLevel(id.k), coarsest);
    const int childTile = std::max(naturalTileLevel(id.k + 1), coarsest);
    if (childTile > parentTile) {
        const terrain::TileId t = terrain::ancestorAt(kids[0], childTile);
        if (!svc_.isTileResident(t)) {
            // Absent from the pack: no finer data exists, keep this node. Otherwise ask for it.
            if (svc_.packHasTile(t))
                requestTile(t, distanceToBox(rec.box, view_.eye), rangeM);
            return false;
        }
    }
    bool allReady = true;
    for (const NodeId& c : kids) {
        NodeRecord* rc = getRecord(c);
        if (!rc) {
            allReady = false;
            continue;
        }
        rc->lastUseFrame = frame_;
        refreshBest(*rc);
        const double dc = distanceToBox(rc->box, view_.eye);
        if (!relevantForReady(*rc, dc)) continue;
        if (!rc->meshReady) {
            queueBuild(*rc, dc, 0);
            allReady = false;
        }
    }
    if (!allReady) return false;
    for (const NodeId& c : kids) visit(c);
    return true;
}

void TerrainSelector::collectGarbage(std::vector<uint32_t>& vbos) {
    const bool revChanged = svc_.residentRevision() != sweptRevision_;
    sweptRevision_ = svc_.residentRevision();
    for (auto it = records_.begin(); it != records_.end();) {
        NodeRecord& r = it->second;
        const uint64_t age = frame_ - r.lastUseFrame;
        bool drop = age > cfg_.keepFrames;
        if (!drop && revChanged && age > 30) {
            r.bestSourceRev = ~0ull;
            refreshBest(r);
            drop = r.bestSourceLevel < 0;  // its tiles were evicted and nobody is looking
        }
        if (drop) {
            if (r.vbo) vbos.push_back(r.vbo);
            it = records_.erase(it);
        } else {
            ++it;
        }
    }
    if (records_.size() > cfg_.maxRecords) {
        std::vector<std::pair<uint64_t, uint64_t>> byAge;  // (lastUse, key)
        for (const auto& [k, r] : records_)
            if (r.lastUseFrame != frame_) byAge.push_back({r.lastUseFrame, k});
        std::sort(byAge.begin(), byAge.end());
        for (const auto& [age, k] : byAge) {
            if (records_.size() <= cfg_.maxRecords) break;
            auto it = records_.find(k);
            if (it->second.vbo) vbos.push_back(it->second.vbo);
            records_.erase(it);
        }
    }
}

void TerrainSelector::clear(std::vector<uint32_t>& vbos) {
    for (auto& [k, r] : records_)
        if (r.vbo) vbos.push_back(r.vbo);
    records_.clear();
}

}  // namespace fsim::world
