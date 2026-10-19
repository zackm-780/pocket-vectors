#include "gfx/Screenshot.h"

#include <cstring>
#include <vector>

#include "core/Log.h"

#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb/stb_image_write.h>

namespace fsim::gfx {

bool saveScreenshotPng(const GlApi& gl, int width, int height, const std::string& path) {
    const size_t stride = static_cast<size_t>(width) * 4;
    std::vector<unsigned char> pixels(stride * static_cast<size_t>(height));
    gl.Finish();
    gl.PixelStorei(GL_PACK_ALIGNMENT, 1);
    gl.ReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    if (gl.GetError() != GL_NO_ERROR) {
        LOG_ERROR("glReadPixels failed for screenshot");
        return false;
    }
    // OpenGL rows run bottom to top; PNG rows run top to bottom.
    std::vector<unsigned char> row(stride);
    for (int y = 0; y < height / 2; ++y) {
        unsigned char* a = pixels.data() + stride * static_cast<size_t>(y);
        unsigned char* b = pixels.data() + stride * static_cast<size_t>(height - 1 - y);
        std::memcpy(row.data(), a, stride);
        std::memcpy(a, b, stride);
        std::memcpy(b, row.data(), stride);
    }
    for (size_t i = 3; i < pixels.size(); i += 4) pixels[i] = 255;  // opaque
    if (!stbi_write_png(path.c_str(), width, height, 4, pixels.data(), static_cast<int>(stride))) {
        LOG_ERROR("could not write screenshot '%s'", path.c_str());
        return false;
    }
    LOG_INFO("screenshot saved: %s (%dx%d)", path.c_str(), width, height);
    return true;
}

}  // namespace fsim::gfx
