#pragma once
// SDL2 window + GL context. Tries a GLES 2.0 context first (the device baseline) and falls back to
// desktop GL 2.1 (ARCHITECTURE section 6); asks for MSAA first and drops it if unavailable.
// This is the only place that initializes SDL video, so --headless never touches it.

#include <string>

#include "gfx/GlApi.h"
#include "gfx/Shader.h"

struct SDL_Window;

namespace fsim {

struct WindowConfig {
    int width = 1280, height = 960;
    bool fullscreen = false;
    bool vsync = true;
    bool preferMsaa = true;
    std::string title = "Flight Sim";
};

class Window {
public:
    Window() = default;
    ~Window();
    Window(const Window&) = delete;
    Window& operator=(const Window&) = delete;

    // Initializes SDL video, creates the window and context, loads GL functions.
    bool create(const WindowConfig& config);
    void swap();
    void drawableSize(int& widthPx, int& heightPx) const;
    void setTitle(const std::string& title);

    const gfx::GlApi& gl() const { return gl_; }
    gfx::GlFlavor flavor() const { return flavor_; }
    int msaaSamples() const { return msaaSamples_; }

private:
    void destroy();

    SDL_Window* window_ = nullptr;
    void* context_ = nullptr;  // SDL_GLContext
    bool sdlStarted_ = false;
    gfx::GlApi gl_;
    gfx::GlFlavor flavor_ = gfx::GlFlavor::Es2;
    int msaaSamples_ = 0;
};

// True when running inside the PortMaster environment (FLIGHTSIM_PLATFORM=portmaster).
bool isPortMasterPlatform();

}  // namespace fsim
