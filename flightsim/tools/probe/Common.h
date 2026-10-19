// Small shared helpers for the device probe: clock, printf-style formatting, simple statistics.
#pragma once

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <string>
#include <vector>

namespace probe {

// Monotonic seconds since an arbitrary epoch.
inline double nowS() {
    using Clock = std::chrono::steady_clock;
    return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
}

// printf into a std::string.
inline std::string strf(const char* format, ...) __attribute__((format(printf, 1, 2)));
inline std::string strf(const char* format, ...) {
    va_list args;
    va_start(args, format);
    va_list copy;
    va_copy(copy, args);
    const int len = std::vsnprintf(nullptr, 0, format, copy);
    va_end(copy);
    std::string out;
    if (len > 0) {
        out.resize(static_cast<size_t>(len) + 1);
        std::vsnprintf(&out[0], out.size(), format, args);
        out.resize(static_cast<size_t>(len));
    }
    va_end(args);
    return out;
}

// Distribution summary of a set of samples (unit is whatever the caller used).
struct Stats {
    size_t count = 0;
    double mean = 0.0;
    double min = 0.0;
    double max = 0.0;
    double p50 = 0.0;
    double p95 = 0.0;
    double p99 = 0.0;
    double stddev = 0.0;
};

inline Stats computeStats(std::vector<double> samples) {
    Stats s;
    if (samples.empty()) {
        return s;
    }
    std::sort(samples.begin(), samples.end());
    s.count = samples.size();
    double sum = 0.0;
    for (double v : samples) {
        sum += v;
    }
    s.mean = sum / static_cast<double>(s.count);
    double var = 0.0;
    for (double v : samples) {
        var += (v - s.mean) * (v - s.mean);
    }
    s.stddev = std::sqrt(var / static_cast<double>(s.count));
    s.min = samples.front();
    s.max = samples.back();
    auto pct = [&](double p) {
        const double idx = p * static_cast<double>(s.count - 1);
        const size_t lo = static_cast<size_t>(idx);
        const size_t hi = std::min(lo + 1, s.count - 1);
        const double frac = idx - static_cast<double>(lo);
        return samples[lo] * (1.0 - frac) + samples[hi] * frac;
    };
    s.p50 = pct(0.50);
    s.p95 = pct(0.95);
    s.p99 = pct(0.99);
    return s;
}

}  // namespace probe
