#include <cstdio>
#include <string>
#include <vector>

#include "app/App.h"
#include "app/Cli.h"

int main(int argc, char** argv) {
    const std::vector<std::string> args(argv + 1, argv + argc);
    fsim::CliOptions options;
    std::string error;
    if (!fsim::parseCli(args, options, error)) {
        std::fprintf(stderr, "flightsim: %s\n\n%s", error.c_str(), fsim::cliUsage().c_str());
        return 2;
    }
    if (options.help) {
        std::fputs(fsim::cliUsage().c_str(), stdout);
        return 0;
    }
    return fsim::App(std::move(options)).run();
}
