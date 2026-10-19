#include "CpuBench.h"

#include <time.h>

#include <cmath>
#include <cstdlib>
#include <cstring>
#include <exception>
#include <vector>

#include "Common.h"
#include "SysInfo.h"

#ifdef PROBE_HAVE_JSBSIM
#include <FGFDMExec.h>
#endif

namespace probe {

namespace {

constexpr double kStepHz = 120.0;
constexpr double kBudgetMsPerFrame30 = 4.0;  // PLAN.md: JSBSim at 120 Hz, <= 4 ms per frame

double processCpuSeconds() {
    timespec ts{};
    clock_gettime(CLOCK_PROCESS_CPUTIME_ID, &ts);
    return static_cast<double>(ts.tv_sec) + static_cast<double>(ts.tv_nsec) * 1e-9;
}

#ifdef PROBE_HAVE_JSBSIM

struct AircraftSetup {
    const char* id;
    double altFt;
    double casKt;
};

struct AircraftResult {
    std::string id;
    bool ok = false;
    std::string error;
    double loadMs = 0.0;
    double trimMs = 0.0;
    long rssDeltaKb = 0;
    Stats stepUs;
    double cpuUsPerStep = 0.0;
    double finalAltFt = 0.0;
    double finalCasKt = 0.0;
    double simTimeS = 0.0;
    double travelledM = 0.0;
    bool finite = true;
    size_t steps = 0;
};

AircraftResult benchAircraft(const std::string& root, const AircraftSetup& setup, int measuredSteps) {
    AircraftResult r;
    r.id = setup.id;
    const long rssBefore = readSelfRssKb();
    try {
        JSBSim::FGFDMExec fdm;
        fdm.SetDebugLevel(0);
        fdm.SetRootDir(SGPath(root));
        fdm.SetAircraftPath(SGPath("aircraft"));
        fdm.SetEnginePath(SGPath("engine"));
        fdm.SetSystemsPath(SGPath("systems"));

        const double loadStart = nowS();
        if (!fdm.LoadModel(setup.id)) {
            r.error = std::string("LoadModel failed (data dir ") + root + ")";
            return r;
        }
        fdm.Setdt(1.0 / kStepHz);
        fdm.SetPropertyValue("ic/lat-geod-deg", 37.62);
        fdm.SetPropertyValue("ic/long-gc-deg", -122.38);
        fdm.SetPropertyValue("ic/terrain-elevation-ft", 0.0);
        fdm.SetPropertyValue("ic/h-sl-ft", setup.altFt);
        fdm.SetPropertyValue("ic/vc-kts", setup.casKt);
        fdm.SetPropertyValue("ic/psi-true-deg", 270.0);
        fdm.SetPropertyValue("ic/gamma-deg", 0.0);
        if (!fdm.RunIC()) {
            r.error = "RunIC failed";
            return r;
        }
        fdm.SetPropertyValue("propulsion/set-running", -1);
        fdm.SetPropertyValue("fcs/mixture-cmd-norm", 0.9);
        fdm.SetPropertyValue("fcs/throttle-cmd-norm", 0.7);
        fdm.SetPropertyValue("fcs/throttle-cmd-norm[0]", 0.7);
        fdm.SetPropertyValue("fcs/throttle-cmd-norm[1]", 0.7);
        r.loadMs = (nowS() - loadStart) * 1000.0;
        r.rssDeltaKb = readSelfRssKb() - rssBefore;

        // A longitudinal trim keeps the aircraft near level flight for the whole run. Failure is not fatal.
        const double trimStart = nowS();
        try {
            fdm.SetPropertyValue("simulation/do_simple_trim", 1);
        } catch (const std::exception&) {
        } catch (...) {
        }
        r.trimMs = (nowS() - trimStart) * 1000.0;

        for (int i = 0; i < 240; ++i) {  // warm-up: caches, branch predictors, governor ramp-up
            fdm.Run();
        }
        std::vector<double> stepUs;
        stepUs.reserve(static_cast<size_t>(measuredSteps));
        const double cpu0 = processCpuSeconds();
        for (int i = 0; i < measuredSteps; ++i) {
            const double t0 = nowS();
            fdm.Run();
            const double t1 = nowS();
            stepUs.push_back((t1 - t0) * 1e6);
        }
        const double cpu1 = processCpuSeconds();
        r.steps = stepUs.size();
        r.stepUs = computeStats(stepUs);
        r.cpuUsPerStep = (cpu1 - cpu0) * 1e6 / static_cast<double>(measuredSteps);
        r.finalAltFt = fdm.GetPropertyValue("position/h-sl-ft");
        r.finalCasKt = fdm.GetPropertyValue("velocities/vc-kts");
        r.simTimeS = fdm.GetPropertyValue("simulation/sim-time-sec");
        r.travelledM = fdm.GetPropertyValue("position/distance-from-start-mag-mt");
        r.finite = std::isfinite(r.finalAltFt) && std::isfinite(r.finalCasKt);
        r.ok = true;
    } catch (const std::exception& e) {
        r.error = std::string("exception: ") + e.what();
    } catch (const std::string& s) {
        r.error = "exception: " + s;
    } catch (...) {
        r.error = "unknown exception";
    }
    return r;
}

#endif  // PROBE_HAVE_JSBSIM

// Copies 32 MB buffers and reports MB/s: a rough indication of the memory bandwidth the terrain
// streaming and framebuffer work will share with the GPU.
void memoryBandwidth(Report& report, bool quick) {
    const size_t bytes = 32u << 20;
    std::vector<unsigned char> src(bytes, 1);
    std::vector<unsigned char> dst(bytes, 0);
    const int rounds = quick ? 2 : 8;
    double best = 1e30;
    for (int i = 0; i < rounds; ++i) {
        const double t0 = nowS();
        std::memcpy(dst.data(), src.data(), bytes);
        const double t1 = nowS();
        src[static_cast<size_t>(i)] = dst[static_cast<size_t>(i) * 4099 % bytes];  // defeat dead-copy elimination
        best = std::min(best, t1 - t0);
    }
    report.kv("memcpy 32 MB (best of rounds)", strf("%.0f MB/s", static_cast<double>(bytes) / 1048576.0 / best));
}

}  // namespace

void runCpuBenchmark(Report& report, const Options& options) {
    report.section("CPU BENCHMARK");
    report.kv("CPU state before", cpuStateSummary());
    memoryBandwidth(report, options.quick);

#ifdef PROBE_HAVE_JSBSIM
    std::string root = options.jsbsimDir.empty() ? executableDir() + "/jsbsim" : options.jsbsimDir;
    report.kv("JSBSim data root", root);
    report.kv("JSBSim version", JSBSim::FGJSBBase::GetVersion());
    const int measured = options.quick ? 600 : 2400;
    report.line(strf("each aircraft: load, RunIC, trim, 240 warm-up steps, then %d measured steps at %.0f Hz (%.0f s of flight)",
                     measured, kStepHz, measured / kStepHz));
    const AircraftSetup setups[] = {{"c172p", 3000.0, 100.0}, {"737", 15000.0, 250.0}};
    std::string fact;
    bool first = true;
    for (const AircraftSetup& setup : setups) {
        report.line(strf("running %s ...", setup.id));
        const AircraftResult r = benchAircraft(root, setup, measured);
        if (!r.ok) {
            report.line(strf("  %s FAILED: %s", r.id.c_str(), r.error.c_str()));
            fact += strf("%s%s: FAILED (%s)", first ? "" : "; ", r.id.c_str(), r.error.c_str());
            first = false;
            continue;
        }
        const double perFrame30Ms = r.stepUs.mean * 4.0 / 1000.0;
        report.line(strf("  %-6s load+IC %.0f ms, trim %.0f ms, RSS +%.1f MB", r.id.c_str(), r.loadMs, r.trimMs,
                         static_cast<double>(r.rssDeltaKb) / 1024.0));
        report.line(strf("  %-6s us/step: mean %.1f  p50 %.1f  p95 %.1f  p99 %.1f  max %.1f  (stddev %.1f, %zu steps)",
                         r.id.c_str(), r.stepUs.mean, r.stepUs.p50, r.stepUs.p95, r.stepUs.p99, r.stepUs.max, r.stepUs.stddev,
                         r.steps));
        report.line(strf("  %-6s process CPU time per step %.1f us; at 30 fps (4 steps/frame) %.2f ms/frame (budget %.1f ms: %s)",
                         r.id.c_str(), r.cpuUsPerStep, perFrame30Ms, kBudgetMsPerFrame30,
                         perFrame30Ms <= kBudgetMsPerFrame30 ? "PASS" : "FAIL"));
        report.line(strf("  %-6s end state: altitude %.0f ft MSL, CAS %.1f kt, sim time %.1f s, flew %.1f km%s", r.id.c_str(),
                         r.finalAltFt, r.finalCasKt, r.simTimeS, r.travelledM / 1000.0,
                         r.finite ? "" : "  <-- NOT FINITE, simulation diverged"));
        fact += strf("%s%s: mean %.0f us/step (p95 %.0f, p99 %.0f, max %.0f) = %.2f ms per 30 fps frame [budget %.1f ms: %s]",
                     first ? "" : "; ", r.id.c_str(), r.stepUs.mean, r.stepUs.p95, r.stepUs.p99, r.stepUs.max, perFrame30Ms,
                     kBudgetMsPerFrame30, perFrame30Ms <= kBudgetMsPerFrame30 ? "PASS" : "FAIL");
        first = false;
    }
    report.fact(rows::kCpu, fact);
    report.line("RESULT: " + fact);
#else
    report.line("JSBSim was not built into this probe (PROBE_WITH_JSBSIM=OFF): flight model benchmark skipped.");
    report.fact(rows::kCpu, "not measured (probe built without JSBSim)");
#endif
    report.kv("CPU state after", cpuStateSummary());
    report.save();
}

}  // namespace probe
