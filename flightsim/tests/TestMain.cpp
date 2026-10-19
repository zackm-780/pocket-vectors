// doctest entry point. Logging to stderr is silenced so test output stays readable.
#define DOCTEST_CONFIG_IMPLEMENT
#include <doctest/doctest.h>

#include "core/Log.h"

int main(int argc, char** argv) {
    fsim::log::setStderrEnabled(false);
    doctest::Context ctx;
    ctx.applyCommandLine(argc, argv);
    return ctx.run();
}
