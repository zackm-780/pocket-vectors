#pragma once
// Saves the current framebuffer (glReadPixels) as a PNG via stb_image_write.

#include <string>

#include "gfx/GlApi.h"

namespace fsim::gfx {

// Reads the bound framebuffer (width x height pixels) and writes a PNG. Call after rendering and
// before the buffer swap. Returns false on GL or file errors.
bool saveScreenshotPng(const GlApi& gl, int width, int height, const std::string& path);

}  // namespace fsim::gfx
