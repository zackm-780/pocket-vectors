#include "Context.h"

#include <algorithm>
#include <cstring>
#include <vector>

#include "Common.h"

namespace probe {

void Ctx::closeWindow() {
    if (glContext != nullptr) {
        if (glOk) {
            textRenderer.shutdown();
        }
        SDL_GL_DeleteContext(glContext);
        glContext = nullptr;
    }
    if (window != nullptr) {
        SDL_DestroyWindow(window);
        window = nullptr;
    }
    glOk = false;
    gl = GlFuncs();
}

bool Ctx::openWindow(const GlRequest& request, std::string& error) {
    closeWindow();
    error.clear();
    for (int attempt = 0; attempt < 2; ++attempt) {
        const bool es = attempt == 0 && request.preferEs;
        if (attempt == 1 && request.preferEs == false) {
            break;
        }
        SDL_GL_ResetAttributes();
        if (request.redBits >= 0) {
            SDL_GL_SetAttribute(SDL_GL_RED_SIZE, request.redBits);
        }
        if (request.greenBits >= 0) {
            SDL_GL_SetAttribute(SDL_GL_GREEN_SIZE, request.greenBits);
        }
        if (request.blueBits >= 0) {
            SDL_GL_SetAttribute(SDL_GL_BLUE_SIZE, request.blueBits);
        }
        if (request.alphaBits >= 0) {
            SDL_GL_SetAttribute(SDL_GL_ALPHA_SIZE, request.alphaBits);
        }
        SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, request.depthBits);
        SDL_GL_SetAttribute(SDL_GL_STENCIL_SIZE, request.stencilBits);
        SDL_GL_SetAttribute(SDL_GL_DOUBLEBUFFER, 1);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLEBUFFERS, request.msaaSamples > 0 ? 1 : 0);
        SDL_GL_SetAttribute(SDL_GL_MULTISAMPLESAMPLES, request.msaaSamples);
        if (es) {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_ES);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 0);
        } else {
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_COMPATIBILITY);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 2);
            SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 1);
        }
        Uint32 flags = SDL_WINDOW_OPENGL | SDL_WINDOW_SHOWN;
        if (opt.fullscreen) {
            flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;
        }
        window = SDL_CreateWindow("Flight Sim Probe", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, opt.windowW,
                                  opt.windowH, flags);
        if (window == nullptr) {
            error += strf("[%s] SDL_CreateWindow failed: %s\n", es ? "ES2" : "GL2.1", SDL_GetError());
            continue;
        }
        glContext = SDL_GL_CreateContext(window);
        if (glContext == nullptr) {
            error += strf("[%s] SDL_GL_CreateContext failed: %s\n", es ? "ES2" : "GL2.1", SDL_GetError());
            SDL_DestroyWindow(window);
            window = nullptr;
            continue;
        }
        SDL_GL_MakeCurrent(window, glContext);
        break;
    }
    if (glContext == nullptr) {
        return false;
    }

    GlObtained o;
    SDL_GL_GetAttribute(SDL_GL_RED_SIZE, &o.redBits);
    SDL_GL_GetAttribute(SDL_GL_GREEN_SIZE, &o.greenBits);
    SDL_GL_GetAttribute(SDL_GL_BLUE_SIZE, &o.blueBits);
    SDL_GL_GetAttribute(SDL_GL_ALPHA_SIZE, &o.alphaBits);
    SDL_GL_GetAttribute(SDL_GL_DEPTH_SIZE, &o.depthBits);
    SDL_GL_GetAttribute(SDL_GL_STENCIL_SIZE, &o.stencilBits);
    SDL_GL_GetAttribute(SDL_GL_MULTISAMPLEBUFFERS, &o.msaaBuffers);
    SDL_GL_GetAttribute(SDL_GL_MULTISAMPLESAMPLES, &o.msaaSamples);
    SDL_GL_GetAttribute(SDL_GL_DOUBLEBUFFER, &o.doubleBuffer);
    SDL_GL_GetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, &o.profileMask);
    SDL_GL_GetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, &o.major);
    SDL_GL_GetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, &o.minor);
    SDL_GL_GetAttribute(SDL_GL_ACCELERATED_VISUAL, &o.accelerated);
    o.isEs = (o.profileMask & SDL_GL_CONTEXT_PROFILE_ES) != 0;
    obtained = o;

    std::string missing;
    if (!gl.load(missing)) {
        error += "GL functions missing:" + missing + "\n";
        glOk = false;
        return false;
    }
    glOk = true;
    std::string log;
    if (!textRenderer.init(&gl, log)) {
        error += "text renderer init failed: " + log;
        glOk = false;
        return false;
    }
    SDL_GL_GetDrawableSize(window, &widthPx, &heightPx);
    if (widthPx <= 0 || heightPx <= 0) {
        SDL_GetWindowSize(window, &widthPx, &heightPx);
    }
    return true;
}

bool Ctx::handleCommonEvent(const SDL_Event& e) {
    if (e.type == SDL_QUIT) {
        quitRequested = true;
        return true;
    }
    if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE && opt.escapeQuits) {
        quitRequested = true;
        return true;
    }
    return false;
}

void Ctx::pumpEvents() {
    SDL_Event e;
    while (SDL_PollEvent(&e) != 0) {
        handleCommonEvent(e);
    }
}

int Ctx::uiScale() const {
    return std::max(1, heightPx / 480);
}

void Ctx::beginUiFrame() {
    gl.Viewport(0, 0, widthPx, heightPx);
    gl.Disable(GL_DEPTH_TEST);
    gl.ClearColor(0.02f, 0.03f, 0.07f, 1.0f);
    gl.Clear(GL_COLOR_BUFFER_BIT);
    textRenderer.begin(widthPx, heightPx);
}

void Ctx::endUiFrame() {
    textRenderer.flush();
    if (!pendingShot.empty()) {
        maybeScreenshot(pendingShot.c_str());
        pendingShot.clear();
    }
    SDL_GL_SwapWindow(window);
}

void Ctx::drawStatusScreen(const std::string& footer, double progress) {
    if (!glOk) {
        return;
    }
    beginUiFrame();
    const float s = static_cast<float>(uiScale());
    const Color title{1.0f, 0.85f, 0.2f, 1.0f};
    const Color text{0.85f, 0.95f, 0.85f, 1.0f};
    const Color dim{0.55f, 0.65f, 0.75f, 1.0f};
    textRenderer.text(8 * s, 6 * s, 2 * s, title, "FLIGHT SIM PROBE");
    textRenderer.text(8 * s, 26 * s, 1 * s, dim, "phase: " + phase);

    const int cols = std::max(10, static_cast<int>(static_cast<float>(widthPx) / (8.0f * s)) - 2);
    const int top = static_cast<int>(40 * s);
    const int bottom = heightPx - static_cast<int>(34 * s);
    const int rows = std::max(1, static_cast<int>(static_cast<float>(bottom - top) / (8.0f * s)));
    const auto& lines = report->screenLines();
    const size_t first = lines.size() > static_cast<size_t>(rows) ? lines.size() - static_cast<size_t>(rows) : 0;
    float y = static_cast<float>(top);
    for (size_t i = first; i < lines.size(); ++i) {
        std::string l = lines[i];
        if (static_cast<int>(l.size()) > cols) {
            l.resize(static_cast<size_t>(cols));
        }
        textRenderer.text(8 * s, y, s, text, l);
        y += 8 * s;
    }
    if (progress >= 0.0) {
        const float barW = static_cast<float>(widthPx) - 16 * s;
        textRenderer.rect(8 * s, static_cast<float>(heightPx) - 26 * s, barW, 6 * s, Color{0.2f, 0.25f, 0.3f, 1});
        textRenderer.rect(8 * s, static_cast<float>(heightPx) - 26 * s, barW * static_cast<float>(std::min(1.0, progress)),
                          6 * s, Color{0.3f, 0.9f, 0.4f, 1});
    }
    std::string f = footer;
    if (static_cast<int>(f.size()) > cols) {
        f.resize(static_cast<size_t>(cols));
    }
    textRenderer.text(8 * s, static_cast<float>(heightPx) - 16 * s, s, dim, f);
    endUiFrame();
}

void Ctx::maybeScreenshot(const char* name) {
    if (opt.shotDir.empty() || !glOk) {
        return;
    }
    // Called after the batch is drawn but before the swap, so flush text first.
    textRenderer.flush();
    std::vector<unsigned char> pixels(static_cast<size_t>(widthPx) * static_cast<size_t>(heightPx) * 4);
    gl.PixelStorei(GL_PACK_ALIGNMENT, 1);
    gl.ReadPixels(0, 0, widthPx, heightPx, GL_RGBA, GL_UNSIGNED_BYTE, pixels.data());
    // GL rows run bottom to top; flip for the BMP.
    std::vector<unsigned char> flipped(pixels.size());
    const size_t rowBytes = static_cast<size_t>(widthPx) * 4;
    for (int y = 0; y < heightPx; ++y) {
        std::memcpy(&flipped[static_cast<size_t>(y) * rowBytes],
                    &pixels[static_cast<size_t>(heightPx - 1 - y) * rowBytes], rowBytes);
    }
    SDL_Surface* surf = SDL_CreateRGBSurfaceFrom(flipped.data(), widthPx, heightPx, 32, widthPx * 4, 0x000000ff,
                                                 0x0000ff00, 0x00ff0000, 0xff000000);
    if (surf != nullptr) {
        const std::string path = strf("%s/%02d_%s.bmp", opt.shotDir.c_str(), shotCounter++, name);
        SDL_SaveBMP(surf, path.c_str());
        SDL_FreeSurface(surf);
    }
}

}  // namespace probe
