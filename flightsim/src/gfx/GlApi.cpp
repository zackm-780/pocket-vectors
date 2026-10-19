#include "gfx/GlApi.h"

#include <SDL.h>

#include "core/Log.h"

namespace fsim::gfx {

bool GlApi::load() {
    bool ok = true;
#define FSIM_GL_LOAD(ret, name, params)                                                            \
    name = reinterpret_cast<ret(FSIM_GLCALL*) params>(SDL_GL_GetProcAddress("gl" #name));          \
    if (!name) {                                                                                   \
        LOG_ERROR("OpenGL function gl" #name " is not available");                                 \
        ok = false;                                                                                \
    }
    FSIM_GL_FUNCTIONS(FSIM_GL_LOAD)
#undef FSIM_GL_LOAD
    return ok;
}

}  // namespace fsim::gfx
