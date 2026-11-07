// X3: the graphics_quality preset as terrain render settings.
#include <doctest/doctest.h>

#include <cstdint>
#include <string>
#include <vector>

#include "app/RenderQuality.h"

using namespace fsim;

TEST_CASE("render quality: MED is the stock terrain config") {
    const world::TerrainRenderConfig stock;
    world::TerrainRenderConfig tc;
    applyRenderQuality(renderQualityFor(GraphicsQuality::Med), tc);
    CHECK(tc.lod.pixelsPerCell == doctest::Approx(stock.lod.pixelsPerCell));
    CHECK(tc.uploadsPerFrame == stock.uploadsPerFrame);
    CHECK(tc.detailNoise == stock.detailNoise);
    CHECK(Settings{}.graphicsQuality == GraphicsQuality::Med);  // the default stays MED
}

TEST_CASE("render quality: LOW is coarser and cheaper, HIGH finer") {
    const RenderQuality low = renderQualityFor(GraphicsQuality::Low);
    const RenderQuality med = renderQualityFor(GraphicsQuality::Med);
    const RenderQuality high = renderQualityFor(GraphicsQuality::High);
    CHECK(low.terrainPixelsPerCell > med.terrainPixelsPerCell);
    CHECK(high.terrainPixelsPerCell < med.terrainPixelsPerCell);
    // Triangles scale with 1 / pixelsPerCell^2: LOW about half of MED, HIGH at most 1.6 x (the
    // P1 probe budget is 40k triangles; MED stays well under it).
    const double lowShare = (med.terrainPixelsPerCell / low.terrainPixelsPerCell) *
                            (med.terrainPixelsPerCell / low.terrainPixelsPerCell);
    const double highShare = (med.terrainPixelsPerCell / high.terrainPixelsPerCell) *
                             (med.terrainPixelsPerCell / high.terrainPixelsPerCell);
    CHECK(lowShare == doctest::Approx(0.51).epsilon(0.05));
    CHECK(highShare <= 1.6);
    CHECK(low.terrainNoise == world::DetailNoise::Off);
    CHECK(med.terrainNoise == world::DetailNoise::Texture);      // X3c: one fetch per octave
    CHECK(high.terrainNoise == world::DetailNoise::Procedural);  // the 1.2.0 shader
    CHECK(low.terrainUploadsPerFrame < med.terrainUploadsPerFrame);
    CHECK(high.terrainUploadsPerFrame >= med.terrainUploadsPerFrame);
}

TEST_CASE("render quality: applying keeps the other terrain fields") {
    world::TerrainRenderConfig tc;
    tc.seaDisc = false;
    tc.lod.morphStartFrac = 0.6f;
    applyRenderQuality(renderQualityFor(GraphicsQuality::Low), tc);
    CHECK_FALSE(tc.seaDisc);
    CHECK(tc.lod.morphStartFrac == doctest::Approx(0.6f));
    CHECK(tc.detailNoise == world::DetailNoise::Off);
    CHECK(std::string(graphicsQualityLabel(GraphicsQuality::High)) == "HIGH");
}

TEST_CASE("render quality: the texture noise table is a 3D value-noise lattice") {
    // X3c: MED's detail noise samples a 256 x 256 RGBA table: R is the lattice value, G the
    // value of the next z slice, found at (+37, +17) with wrap-around.
    const std::vector<std::uint8_t> t = world::detailNoiseTable();
    REQUIRE(t.size() == 256u * 256u * 4u);
    auto at = [&](int x, int y, int c) {
        return t[static_cast<size_t>((((y % 256) * 256) + (x % 256)) * 4 + c)];
    };
    for (int y = 0; y < 256; y += 7)
        for (int x = 0; x < 256; x += 5) CHECK(at(x, y, 1) == at(x + 37, y + 17, 0));
    // Uniform-ish values with mean near 0.5 (as the procedural hash), the same table every run.
    double sum = 0;
    int lo = 0, hi = 0;
    for (size_t i = 0; i < t.size(); i += 4) {
        sum += t[i];
        lo += t[i] < 64;
        hi += t[i] >= 192;
    }
    const double mean = sum / (256.0 * 256.0);
    CHECK(mean == doctest::Approx(127.5).epsilon(0.02));
    CHECK(lo == doctest::Approx(16384).epsilon(0.05));
    CHECK(hi == doctest::Approx(16384).epsilon(0.05));
    CHECK(world::detailNoiseTable() == t);
    CHECK(std::string(world::detailNoiseName(world::DetailNoise::Texture)) == "texture");
}
