#include "platform/Window.h"

#include <SDL.h>

#include <cstdlib>
#include <cstring>
#include <vector>

#include "core/Log.h"

namespace fsim {
namespace {

struct ContextAttempt {
    bool es;
    int samples;
};

}  // namespace

bool isPortMasterPlatform() {
    const char* p = std::getenv("FLIGHTSIM_PLATFORM");
    return p && std::strcmp(p, "portmaster") == 0;
}

Window::~Window() {
    destroy();
}

void Window::destroy() {
    if (context_) SDL_GL_DeleteContext(static_cast<SDL_GLContext>(context_));
    if (window_) SDL_DestroyWindow(window_);
    context_ = nullptr;
    window_ = nullptr;
    if (sdlStarted_) SDL_Quit();
    sdlStarted_ = false;
}

bool Window::create(const WindowConfig& cfg) {
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS) != 0) {
        LOG_ERROR("SDL_Init failed: %s", SDL_GetError());
        return false;
    }
    sdlStarted_ = true;

    // GLES2 first (device baseline), then desktop GL 2.1; each with MSAA first when requested.
    std::vector<ContextAttempt> attempts;
    for (bool es : {true, false}) {
        if (cfg.preferMsaa) attempts.push_back({es, 4});
        attempts.push_back({es, 0});
    }

    Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_ALLOW_HIGHDPI;
    if (cfg.fullscreen) flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;

    for (const ContextAttempt& a : attempts) {
        SDL_GL_ResetAttributes();
        SDL_GL_SetAttribute(SDL_GL_RED_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, 8);
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, a.samples > 0 ? 1 : 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, a.samples);
        if (a.es) {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
        } else {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
        }

        window_ = SDL_CreateWindow(cfg.title.c_str(), SDL_WINDOWPOS_CENTERED,
                                   SDL_WINDOWPOS_CENTERED, cfg.width, cfg.height, flags);
        if (!window_) {
            LOG_DEBUG("window creation failed (%s, msaa %d): %s", a.es ? "ES2" : "GL2.1", a.samples,
                      SDL_GetError());
            continue;
        }
        context_ = SDL_GL_CreateContext(window_);
        if (!context_) {
            LOG_DEBUG("context creation failed (%s, msaa %d): %s", a.es ? "ES2" : "GL2.1",
                      a.samples, SDL_GetError());
            SDL_DestroyWindow(window_);
            window_ = nullptr;
            continue;
        }
        flavor_ = a.es ? gfx::GlFlavor::Es2 : gfx::GlFlavor::Desktop21;
        msaaSamples_ = a.samples;
        break;
    }
    if (!context_) {
        LOG_ERROR("could not create an OpenGL ES 2.0 or GL 2.1 context");
        destroy();
        return false;
    }

    if (!gl_.load()) {
        destroy();
        return false;
    }
    if (SDL_GL_SetSwapInterval(cfg.vsync ? 1 : 0) != 0)
        LOG_DEBUG("swap interval not supported: %s", SDL_GetError());

    auto str = [&](gfx::GLenum name) {
        const gfx::GLubyte* s = gl_.GetString(name);
        return s ? reinterpret_cast<const char*>(s) : "?";
    };
    LOG_INFO("OpenGL %s | %s | %s | %s | msaa %d", flavor_ == gfx::GlFlavor::Es2 ? "ES2" : "2.1",
             str(gfx::GL_VERSION), str(gfx::GL_RENDERER), str(gfx::GL_VENDOR), msaaSamples_);
    return true;
}

void Window::swap() {
    SDL_GL_SwapWindow(window_);
}

void Window::drawableSize(int& w, int& h) const {
    SDL_GL_GetDrawableSize(window_, &w, &h);
}

void Window::setTitle(const std::string& title) {
    SDL_SetWindowTitle(window_, title.c_str());
}

}  // namespace fsim
