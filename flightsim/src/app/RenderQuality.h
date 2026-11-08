#pragma once
// X3: the graphics_quality preset (settings LOW / MED / HIGH) as render settings. MED is what
// every build before X3 drew, with the detail noise from a texture since 1.2.1 (X3c: the same
// kind of noise at one fetch per octave instead of 8 hashes; HIGH keeps the hashed 1.2.0 shader).
// LOW and HIGH start from the P1 probe numbers (DEVICE.md section 2: draw calls cost more than
// triangles; the fill rate is not the limit at 640 x 480) and from the terrain fragment shader,
// whose detail noise is the heaviest per-pixel work in the frame. The values get tuned with the
// device timing (FrameStats, DEVICE.md section 6).
//
// Clouds, the chase model and the airports are not in the preset: each is bounded already (at
// most 3 cloud draws, 1 model draw), and turning them off changes what the pilot sees, not only
// how sharp it is.

#include "app/Settings.h"
#include "world/TerrainRenderer.h"

namespace fsim {

struct RenderQuality {
    // Terrain LOD target in virtual pixels per cell: larger is coarser, with fewer triangles and
    // fewer node draws (both scale about with 1 / pixelsPerCell^2).
    float terrainPixelsPerCell = 10.0f;
    // Node meshes built and uploaded per frame: fewer smooths CPU spikes, more fills holes sooner.
    int terrainUploadsPerFrame = 4;
    world::DetailNoise terrainNoise = world::DetailNoise::Texture;  // detail on near terrain
};

RenderQuality renderQualityFor(GraphicsQuality quality);
const char* graphicsQualityLabel(GraphicsQuality quality);  // "LOW", "MED", "HIGH"
// Writes the preset into a terrain config; the other fields stay as they are.
void applyRenderQuality(const RenderQuality& rq, world::TerrainRenderConfig& cfg);

}  // namespace fsim
