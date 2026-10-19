#pragma once
// FSIM_ASSERT: aborts in debug builds, logs and continues in release (ARCHITECTURE section 10).

#include <cstdlib>

#include "core/Log.h"

#ifdef NDEBUG
#define FSIM_ASSERT(cond)                                                                          \
    do {                                                                                           \
        if (!(cond)) LOG_ERROR("assertion failed: %s (%s:%d)", #cond, __FILE__, __LINE__);         \
    } while (0)
#else
#define FSIM_ASSERT(cond)                                                                          \
    do {                                                                                           \
        if (!(cond)) {                                                                             \
            LOG_ERROR("assertion failed: %s (%s:%d)", #cond, __FILE__, __LINE__);                  \
            std::abort();                                                                          \
        }                                                                                          \
    } while (0)
#endif
