#include "world/TerrainRendererMesh.h"

#include <algorithm>
#include <cmath>

#include "core/Units.h"

namespace fsim::world {
namespace {

using terrain::kNoDataM;
using terrain::kTileCells;
using terrain::kTileSamples;
using terrain::TerrainTile;
using terrain::TileId;

constexpr int kLat = kPatchVerts + 2;  // lattice with a one-sample border for normals and morph
constexpr double kMPerDegLat = kEarthRadiusM * kRadPerDeg;

float clamp01(float v) {
    return v < 0 ? 0 : (v > 1 ? 1 : v);
}
float smoothstep(float a, float b, float x) {
    const float t = clamp01((x - a) / (b - a));
    return t * t * (3.0f - 2.0f * t);
}

// ---- source data access -------------------------------------------------------------------------
// Heights and land class from the source tile and its eight same-level neighbours, in the source
// tile's own sample coordinates (col east, row south, 0..256). Positions past the tile edge are
// read from the neighbour so lighting and colour are continuous across tile seams; a missing
// neighbour clamps to the edge of the source tile.
class SourceGrid {
public:
    SourceGrid(const TileLookupFn& lookup, std::shared_ptr<const TerrainTile> center)
        : lookup_(lookup), center_(std::move(center)) {
        tiles_[4] = center_;
        fetched_[4] = true;
    }
    const TerrainTile& center() const { return *center_; }
    bool hasLandClass() const { return center_->hasLandClass(); }

    float height(double col, double row) {
        int dx = col < 0 ? -1 : (col > kTileCells ? 1 : 0);
        int dy = row < 0 ? 1 : (row > kTileCells ? -1 : 0);
        const TerrainTile* t = (dx || dy) ? tileAt(dx, dy) : center_.get();
        if (!t) {
            t = center_.get();
            col = std::min<double>(kTileCells, std::max(0.0, col));
            row = std::min<double>(kTileCells, std::max(0.0, row));
        } else {
            col -= kTileCells * dx;
            row += kTileCells * dy;
        }
        return bilinear(*t, col, row);
    }

    // Land class of the cell with the given (possibly out of tile) indices.
    bool landCell(int c, int r, uint8_t& cls) {
        const int dx = c < 0 ? -1 : (c >= kTileCells ? 1 : 0);
        const int dy = r < 0 ? 1 : (r >= kTileCells ? -1 : 0);
        const TerrainTile* t = (dx || dy) ? tileAt(dx, dy) : center_.get();
        if (!t || !t->hasLandClass()) {
            t = center_.get();
            if (!t->hasLandClass()) return false;
            c = std::min(kTileCells - 1, std::max(0, c));
            r = std::min(kTileCells - 1, std::max(0, r));
        } else {
            c -= kTileCells * dx;
            r += kTileCells * dy;
        }
        cls = t->landClassCell(r, c);
        return true;
    }

private:
    // dx east +1, dy NORTH +1 (rows run north to south inside a tile).
    const TerrainTile* tileAt(int dx, int dy) {
        const int idx = (dy + 1) * 3 + (dx + 1);
        if (!fetched_[idx]) {
            fetched_[idx] = true;
            const TileId c = center_->id;
            const int64_t yy = static_cast<int64_t>(c.y) + dy;
            if (yy >= 0 && yy < static_cast<int64_t>(terrain::tilesHigh(c.k))) {
                const int64_t wide = terrain::tilesWide(c.k);
                const int64_t xx = ((static_cast<int64_t>(c.x) + dx) % wide + wide) % wide;
                tiles_[idx] =
                    lookup_(TileId{c.k, static_cast<uint32_t>(xx), static_cast<uint32_t>(yy)});
            }
        }
        return tiles_[idx].get();
    }

    static float bilinear(const TerrainTile& t, double col, double row) {
        const int c0 = std::min(kTileCells - 1, std::max(0, static_cast<int>(std::floor(col))));
        const int r0 = std::min(kTileCells - 1, std::max(0, static_cast<int>(std::floor(row))));
        const float fc = static_cast<float>(std::min(1.0, std::max(0.0, col - c0)));
        const float fr = static_cast<float>(std::min(1.0, std::max(0.0, row - r0)));
        const int16_t s[4] = {t.heightAtM(r0, c0), t.heightAtM(r0, c0 + 1), t.heightAtM(r0 + 1, c0),
                              t.heightAtM(r0 + 1, c0 + 1)};
        float v[4];
        float sum = 0;
        int valid = 0;
        for (int i = 0; i < 4; ++i) {
            if (s[i] != kNoDataM) {
                v[i] = s[i];
                sum += v[i];
                ++valid;
            }
        }
        if (valid == 0) return 0.0f;
        if (valid < 4) {  // voids take the mean of the valid corners
            for (int i = 0; i < 4; ++i)
                if (s[i] == kNoDataM) v[i] = sum / valid;
        }
        const float top = v[0] + (v[1] - v[0]) * fc;
        const float bottom = v[2] + (v[3] - v[2]) * fc;
        return top + (bottom - top) * fr;
    }

    const TileLookupFn& lookup_;
    std::shared_ptr<const TerrainTile> center_;
    std::shared_ptr<const TerrainTile> tiles_[9];
    bool fetched_[9] = {};
};

// ---- colours ------------------------------------------------------------------------------------
struct ClassStyle {
    float rgb[3];
    float rockiness;  // how far steep slopes tint the class toward bare rock
};

constexpr ClassStyle kStyles[12] = {
    {{0.45f, 0.45f, 0.40f}, 0.8f},   // Unknown (not used directly)
    {{0.04f, 0.10f, 0.18f}, 0.0f},   // Water
    {{0.10f, 0.24f, 0.09f}, 0.55f},  // Tree
    {{0.36f, 0.38f, 0.20f}, 0.75f},  // Shrub
    {{0.34f, 0.48f, 0.19f}, 0.85f},  // Grass
    {{0.52f, 0.52f, 0.26f}, 0.35f},  // Crop
    {{0.46f, 0.45f, 0.44f}, 0.0f},   // Urban
    {{0.52f, 0.45f, 0.35f}, 1.0f},   // Bare
    {{0.92f, 0.94f, 0.97f}, 0.2f},   // SnowIce
    {{0.22f, 0.36f, 0.28f}, 0.2f},   // Wetland
    {{0.12f, 0.28f, 0.18f}, 0.2f},   // Mangrove
    {{0.38f, 0.44f, 0.32f}, 0.7f},   // Moss
};
constexpr float kRockRgb[3] = {0.44f, 0.41f, 0.37f};

void mix3(const float a[3], const float b[3], float t, float out[3]) {
    for (int i = 0; i < 3; ++i) out[i] = a[i] + (b[i] - a[i]) * t;
}

// Terrain colour ramp when no land class exists: lowland green, forest, alpine rock, snow.
void elevationRamp(float h, float out[3]) {
    struct Key {
        float h;
        float rgb[3];
    };
    static const Key keys[] = {{0, {0.33f, 0.45f, 0.22f}},    {600, {0.22f, 0.36f, 0.16f}},
                               {1400, {0.16f, 0.28f, 0.12f}}, {2000, {0.40f, 0.38f, 0.30f}},
                               {2600, {0.50f, 0.47f, 0.42f}}, {3200, {0.90f, 0.92f, 0.95f}}};
    constexpr int n = sizeof(keys) / sizeof(keys[0]);
    if (h <= keys[0].h) {
        std::copy(keys[0].rgb, keys[0].rgb + 3, out);
        return;
    }
    for (int i = 1; i < n; ++i) {
        if (h <= keys[i].h) {
            mix3(keys[i - 1].rgb, keys[i].rgb, (h - keys[i - 1].h) / (keys[i].h - keys[i - 1].h),
                 out);
            return;
        }
    }
    std::copy(keys[n - 1].rgb, keys[n - 1].rgb + 3, out);
}

}  // namespace

void landClassAlbedo(LandClass c, float rgb[3]) {
    const int i = static_cast<int>(c);
    const ClassStyle& s = kStyles[(i >= 0 && i < 12) ? i : 0];
    std::copy(s.rgb, s.rgb + 3, rgb);
}

void fallbackAlbedo(float heightM, float slopeCos, float rgb[3]) {
    elevationRamp(heightM, rgb);
    const float rock = smoothstep(0.82f, 0.62f, slopeCos);
    mix3(rgb, kRockRgb, rock * 0.8f, rgb);
}

double nodeCellMeanM(const NodeId& id) {
    const terrain::TileBounds b = terrain::bounds(id);
    const double cellDeg = nodeCellDeg(id.k);
    const double ns = cellDeg * kMPerDegLat;
    const double ew = ns * std::cos(b.centerLatDeg() * kRadPerDeg);
    return 0.5 * (ns + ew);
}

const std::vector<uint16_t>& nodeIndices() {
    static const std::vector<uint16_t> indices = [] {
        std::vector<uint16_t> ix;
        ix.reserve(kNodeIndexCount);
        auto g = [](int i, int j) { return static_cast<uint16_t>(j * kPatchVerts + i); };
        for (int j = 0; j < kPatchCells; ++j) {
            for (int i = 0; i < kPatchCells; ++i) {
                // The diagonal alternates with the lattice parity; geomorphing relies on it.
                if (((i + j) & 1) == 0) {
                    ix.insert(ix.end(), {g(i, j), g(i + 1, j), g(i + 1, j + 1), g(i, j),
                                         g(i + 1, j + 1), g(i, j + 1)});
                } else {
                    ix.insert(ix.end(), {g(i, j), g(i + 1, j), g(i, j + 1), g(i + 1, j),
                                         g(i + 1, j + 1), g(i, j + 1)});
                }
            }
        }
        // Skirts: south, north, west, east strips of kPatchVerts vertices each.
        const int base[4] = {kGridVertexCount, kGridVertexCount + kPatchVerts,
                             kGridVertexCount + 2 * kPatchVerts,
                             kGridVertexCount + 3 * kPatchVerts};
        for (int e = 0; e < 4; ++e) {
            for (int s = 0; s < kPatchCells; ++s) {
                uint16_t a, b;  // surface vertices
                switch (e) {
                case 0: a = g(s, 0), b = g(s + 1, 0); break;
                case 1: a = g(s, kPatchCells), b = g(s + 1, kPatchCells); break;
                case 2: a = g(0, s), b = g(0, s + 1); break;
                default: a = g(kPatchCells, s), b = g(kPatchCells, s + 1); break;
                }
                const uint16_t sa = static_cast<uint16_t>(base[e] + s);
                const uint16_t sb = static_cast<uint16_t>(base[e] + s + 1);
                ix.insert(ix.end(), {a, b, sb, a, sb, sa});
            }
        }
        return ix;
    }();
    return indices;
}

Ecef nodeVertexEcef(const NodeMesh& mesh, int v) {
    const NodeVertex& n = mesh.vertices[static_cast<size_t>(v)];
    return {mesh.anchor.x + n.pos[0], mesh.anchor.y + n.pos[1], mesh.anchor.z + n.pos[2]};
}

bool buildNodeMesh(const NodeId& id, const TileLookupFn& lookup, const MeshBuildOptions& opt,
                   NodeMesh& out) {
    const int m = id.k;
    if (m < opt.coarsestTileLevel || !terrain::isValid(id)) return false;

    // 1. Source tile: the natural tile, else the finest resident ancestor.
    const int natural = std::max(naturalTileLevel(m), opt.coarsestTileLevel);
    std::shared_ptr<const TerrainTile> src;
    for (int k = natural; k >= opt.coarsestTileLevel && !src; --k)
        src = lookup(terrain::ancestorAt(id, k));
    if (!src) return false;
    SourceGrid grid(lookup, src);
    const int ks = src->id.k;
    // Node cell size in source samples (1 = natural, 2 = every other sample, 0.5 = oversampled).
    const double scale = std::ldexp(1.0, ks - m + kPatchShift);
    const double xs = src->id.x, ys = src->id.y;
    const bool hasParent = m > opt.coarsestTileLevel;

    const double cellDeg = nodeCellDeg(m);
    const int64_t gx0 = static_cast<int64_t>(id.x) * kPatchCells;
    const int64_t gy0 = static_cast<int64_t>(id.y) * kPatchCells;

    // 2. Lattice rows/columns: trig once per row and column.
    struct RowTrig {
        double sinLat, cosLat, n, latDeg;
    } rows[kLat];
    struct ColTrig {
        double sinLon, cosLon, lonDeg;
    } cols[kLat];
    for (int b = 0; b < kLat; ++b) {
        const double lat = -90.0 + static_cast<double>(gy0 + b - 1) * cellDeg;
        const double lr = lat * kRadPerDeg;
        rows[b].latDeg = lat;
        rows[b].sinLat = std::sin(lr);
        rows[b].cosLat = std::cos(lr);
        rows[b].n = kWgs84AM / std::sqrt(1.0 - kWgs84E2 * rows[b].sinLat * rows[b].sinLat);
        const double lon = -180.0 + static_cast<double>(gx0 + b - 1) * cellDeg;
        const double lo = lon * kRadPerDeg;
        cols[b].lonDeg = lon;
        cols[b].sinLon = std::sin(lo);
        cols[b].cosLon = std::cos(lo);
    }

    // 3. Heights (natural, then flattened) over the bordered lattice.
    float h[kLat][kLat];
    for (int b = 0; b < kLat; ++b) {
        for (int a = 0; a < kLat; ++a) {
            const double col = static_cast<double>(gx0 + a - 1) * scale - xs * kTileCells;
            const double row = (ys + 1.0) * kTileCells - static_cast<double>(gy0 + b - 1) * scale;
            const float natural = grid.height(col, row);
            h[b][a] = (opt.flatten && !opt.flatten->empty())
                          ? opt.flatten->elevationM(rows[b].latDeg, cols[a].lonDeg,
                                                    [natural] { return natural; })
                          : natural;
        }
    }

    // 4. ECEF positions relative to the anchor (node centre, height 0).
    const double cLat = -90.0 + (static_cast<double>(gy0) + kPatchCells * 0.5) * cellDeg;
    const double cLon = -180.0 + (static_cast<double>(gx0) + kPatchCells * 0.5) * cellDeg;
    {
        GeoPos c;
        c.latDeg = cLat;
        c.lonDeg = cLon;
        out.anchor = geodeticToEcef(c);
    }
    struct P3 {
        double x, y, z;
    };
    static_assert(sizeof(P3) == 24, "");
    P3 pos[kLat][kLat];
    for (int b = 0; b < kLat; ++b) {
        for (int a = 0; a < kLat; ++a) {
            const RowTrig& r = rows[b];
            const ColTrig& c = cols[a];
            const double rho = (r.n + h[b][a]) * r.cosLat;
            pos[b][a] = {rho * c.cosLon - out.anchor.x, rho * c.sinLon - out.anchor.y,
                         (r.n * (1.0 - kWgs84E2) + h[b][a]) * r.sinLat - out.anchor.z};
        }
    }

    out.id = id;
    out.sourceLevel = ks;
    out.vertices.assign(kNodeVertexCount, NodeVertex{});
    const double cellMeridianM = cellDeg * kMPerDegLat;
    out.skirtDepthM = static_cast<float>(0.35 * cellMeridianM + 3.0);

    float minH = 1e9f, maxH = -1e9f;
    bool allWater = true;
    const bool landClass = grid.hasLandClass();
    const double window = std::max(1.0, scale);  // colour window in source samples
    const int taps = std::min(4, std::max(2, static_cast<int>(std::ceil(window))));

    for (int j = 0; j < kPatchVerts; ++j) {
        for (int i = 0; i < kPatchVerts; ++i) {
            const int a = i + 1, b = j + 1;
            NodeVertex& v = out.vertices[static_cast<size_t>(j * kPatchVerts + i)];
            v.pos[0] = static_cast<float>(pos[b][a].x);
            v.pos[1] = static_cast<float>(pos[b][a].y);
            v.pos[2] = static_cast<float>(pos[b][a].z);

            // Normal: east x north of the neighbouring lattice points (right-handed, = up).
            const double ex = pos[b][a + 1].x - pos[b][a - 1].x,
                         ey = pos[b][a + 1].y - pos[b][a - 1].y,
                         ez = pos[b][a + 1].z - pos[b][a - 1].z;
            const double nx = pos[b + 1][a].x - pos[b - 1][a].x,
                         ny = pos[b + 1][a].y - pos[b - 1][a].y,
                         nz = pos[b + 1][a].z - pos[b - 1][a].z;
            double cx = ey * nz - ez * ny, cy = ez * nx - ex * nz, cz = ex * ny - ey * nx;
            const double len = std::sqrt(cx * cx + cy * cy + cz * cz);
            if (len > 0) cx /= len, cy /= len, cz /= len;
            v.normal[0] = static_cast<int8_t>(std::lround(cx * 127.0));
            v.normal[1] = static_cast<int8_t>(std::lround(cy * 127.0));
            v.normal[2] = static_cast<int8_t>(std::lround(cz * 127.0));
            v.normal[3] = 0;

            // Local up (ellipsoid normal) for morph, skirts and slope.
            const double ux = rows[b].cosLat * cols[a].cosLon, uy = rows[b].cosLat * cols[a].sinLon,
                         uz = rows[b].sinLat;
            const float slopeCos = static_cast<float>(cx * ux + cy * uy + cz * uz);

            // Geomorph target: the height this vertex has on the parent (half resolution) lattice.
            float dh = 0;
            if (hasParent) {
                const bool oddI = (i & 1) != 0, oddJ = (j & 1) != 0;
                float hc = h[b][a];
                if (oddI && !oddJ) {
                    hc = 0.5f * (h[b][a - 1] + h[b][a + 1]);
                } else if (!oddI && oddJ) {
                    hc = 0.5f * (h[b - 1][a] + h[b + 1][a]);
                } else if (oddI && oddJ) {
                    const int64_t px = (gx0 + i - 1) >> 1, py = (gy0 + j - 1) >> 1;
                    hc = (((px + py) & 1) == 0) ? 0.5f * (h[b - 1][a - 1] + h[b + 1][a + 1])
                                                : 0.5f * (h[b - 1][a + 1] + h[b + 1][a - 1]);
                }
                dh = hc - h[b][a];
            }
            v.morph[0] = static_cast<float>(ux * dh);
            v.morph[1] = static_cast<float>(uy * dh);
            v.morph[2] = static_cast<float>(uz * dh);

            // Colour: average the land class cells under the vertex's footprint.
            const double colS = static_cast<double>(gx0 + i) * scale - xs * kTileCells;
            const double rowS = (ys + 1.0) * kTileCells - static_cast<double>(gy0 + j) * scale;
            float rgb[3] = {0, 0, 0};
            int land = 0, water = 0, total = 0;
            const float hv = h[b][a];
            float fallback[3];
            fallbackAlbedo(hv, slopeCos, fallback);
            if (!landClass) {
                total = 1;
                if (hv <= 0.5f)
                    water = 1;
                else
                    land = 1, std::copy(fallback, fallback + 3, rgb);
            } else {
                for (int ty = 0; ty < taps; ++ty) {
                    for (int tx = 0; tx < taps; ++tx) {
                        const double ox = ((tx + 0.5) / taps - 0.5) * window;
                        const double oy = ((ty + 0.5) / taps - 0.5) * window;
                        uint8_t cls = 0;
                        grid.landCell(static_cast<int>(std::floor(colS + ox)),
                                      static_cast<int>(std::floor(rowS + oy)), cls);
                        ++total;
                        if (cls == static_cast<uint8_t>(LandClass::Water)) {
                            ++water;
                            continue;
                        }
                        float c[3];
                        if (cls == static_cast<uint8_t>(LandClass::Unknown) || cls > 11) {
                            if (hv <= 0.5f) {
                                ++water;
                                continue;
                            }
                            std::copy(fallback, fallback + 3, c);
                        } else {
                            const ClassStyle& st = kStyles[cls];
                            const float rock =
                                smoothstep(0.82f, 0.62f, slopeCos) * st.rockiness * 0.75f;
                            mix3(st.rgb, kRockRgb, rock, c);
                        }
                        for (int q = 0; q < 3; ++q) rgb[q] += c[q];
                        ++land;
                    }
                }
            }
            float waterFrac =
                total > 0 ? static_cast<float>(water) / static_cast<float>(total) : 0.f;
            if (land > 0) {
                for (float& q : rgb) q /= static_cast<float>(land);
            } else {
                std::copy(kStyles[1].rgb, kStyles[1].rgb + 3, rgb);
                waterFrac = 1.0f;
            }
            for (int q = 0; q < 3; ++q)
                v.color[q] = static_cast<uint8_t>(std::lround(clamp01(rgb[q]) * 255.0f));
            v.color[3] = static_cast<uint8_t>(std::lround(clamp01(waterFrac) * 255.0f));
            if (waterFrac < 0.999f) allWater = false;
            minH = std::min(minH, hv);
            maxH = std::max(maxH, hv);
        }
    }
    out.minHeightM = minH;
    out.maxHeightM = maxH;
    out.allSea = allWater && minH >= -1.0f && maxH <= 1.0f;

    // 5. Skirts: copies of the border vertices pushed down along the local up.
    auto skirt = [&](int dst, int srcGrid, int bIdx, int aIdx) {
        NodeVertex v = out.vertices[static_cast<size_t>(srcGrid)];
        const double ux = rows[bIdx].cosLat * cols[aIdx].cosLon,
                     uy = rows[bIdx].cosLat * cols[aIdx].sinLon, uz = rows[bIdx].sinLat;
        v.pos[0] -= static_cast<float>(ux * out.skirtDepthM);
        v.pos[1] -= static_cast<float>(uy * out.skirtDepthM);
        v.pos[2] -= static_cast<float>(uz * out.skirtDepthM);
        out.vertices[static_cast<size_t>(dst)] = v;
    };
    for (int s = 0; s < kPatchVerts; ++s) {
        skirt(kGridVertexCount + s, 0 * kPatchVerts + s, 1, s + 1);  // south
        skirt(kGridVertexCount + kPatchVerts + s, kPatchCells * kPatchVerts + s, kPatchVerts,
              s + 1);                                                              // north
        skirt(kGridVertexCount + 2 * kPatchVerts + s, s * kPatchVerts, s + 1, 1);  // west
        skirt(kGridVertexCount + 3 * kPatchVerts + s, s * kPatchVerts + kPatchCells, s + 1,
              kPatchVerts);  // east
    }
    return true;
}

}  // namespace fsim::world
