// X3: the graphics_quality preset as terrain render settings.
#include <doctest/doctest.h>

#include <string>

#include "app/RenderQuality.h"

using namespace fsim;

TEST_CASE("render quality: MED is the pre-X3 terrain config") {
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
    CHECK_FALSE(low.terrainDetailNoise);
    CHECK(high.terrainDetailNoise);
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
    CHECK_FALSE(tc.detailNoise);
    CHECK(std::string(graphicsQualityLabel(GraphicsQuality::High)) == "HIGH");
}
