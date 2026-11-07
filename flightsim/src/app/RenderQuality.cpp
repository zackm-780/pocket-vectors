#include "app/RenderQuality.h"

namespace fsim {

RenderQuality renderQualityFor(GraphicsQuality quality) {
    RenderQuality rq;  // MED: the pre-X3 look
    switch (quality) {
    case GraphicsQuality::Low:
        rq.terrainPixelsPerCell = 14.0f;  // about half the triangles and node draws of MED
        rq.terrainUploadsPerFrame = 2;
        rq.terrainNoise = world::DetailNoise::Off;
        break;
    case GraphicsQuality::Med: break;
    case GraphicsQuality::High:
        rq.terrainPixelsPerCell = 8.0f;  // about 1.5 x the triangles of MED
        rq.terrainUploadsPerFrame = 6;
        rq.terrainNoise = world::DetailNoise::Procedural;  // X3c: the 1.2.0 shader
        break;
    }
    return rq;
}

const char* graphicsQualityLabel(GraphicsQuality quality) {
    switch (quality) {
    case GraphicsQuality::Low: return "LOW";
    case GraphicsQuality::Med: return "MED";
    case GraphicsQuality::High: return "HIGH";
    }
    return "MED";
}

void applyRenderQuality(const RenderQuality& rq, world::TerrainRenderConfig& cfg) {
    cfg.lod.pixelsPerCell = rq.terrainPixelsPerCell;
    cfg.uploadsPerFrame = rq.terrainUploadsPerFrame;
    cfg.detailNoise = rq.terrainNoise;
}

}  // namespace fsim
