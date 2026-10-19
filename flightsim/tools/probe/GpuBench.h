// GL information, GPU benchmark and the depth/stencil/MSAA configuration sweep.
#pragma once

#include "Context.h"

namespace probe {

// Vendor/renderer/version/extensions, limits and the configuration SDL actually gave us for the main
// window. Sets the GL and max-texture facts. Requires ctx.glOk.
void reportGlInfo(Ctx& ctx);

// Fill rate and terrain-like triangle throughput with a fog shader. Sets the GPU fact.
// Offscreen (FBO) passes measure pure GPU time; on-screen passes measure frame pacing with vsync.
void runGpuBenchmark(Ctx& ctx);

// Tries several depth/stencil/MSAA/colour-depth requests, each in a fresh window + context, and records
// what was obtained. Sets the depth/MSAA fact. Destroys the current window and context.
void runConfigSweep(Ctx& ctx);

}  // namespace probe
