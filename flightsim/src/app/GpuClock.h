#pragma once
// X3c: the GPU clock from Linux devfreq (/sys/class/devfreq/<entry>/), for the perf log.
//
// On the RG40XXH the Mali-G31 runs under a devfreq governor. Inside an OVC layer the frame rate
// flipped between 30 and 60 fps with the same triangles (DEVICE.md section 3): either the GPU sits
// at the 16.7 ms edge, or the governor moves its clock. Logging the clock with the timing tells
// the two apart. Desktops usually have no GPU devfreq entry: then available() is false, nothing is
// logged and nothing fails.
//
// open() scans the folder once. The readers open small sysfs files, so call them at most once a
// second (the SHOW FPS log line), never per frame.

#include <string>
#include <vector>

namespace fsim {

class GpuClock {
public:
    // Picks the GPU entry under `root`: a name containing "gpu" or "mali" (any case), else the
    // only entry. Returns available().
    bool open(const std::string& root = "/sys/class/devfreq");
    bool available() const { return !dir_.empty(); }
    const std::string& entryName() const { return name_; }

    // Each reads its sysfs file now; -1 / empty when the file is missing or unreadable.
    int curMhz() const;
    int minMhz() const;
    int maxMhz() const;
    std::vector<int> availableMhz() const;
    std::string governor() const;

    // " | gpu 648/648 MHz" (current / max) for the SHOW FPS log line; "" when not available.
    std::string windowTag() const;
    // "gpu clock: <entry> <governor> cur 648 MHz (min 216 max 648; available 216 312 432 648)";
    // "" when not available.
    std::string describe() const;

private:
    std::string dir_, name_;
};

// Pure helpers (unit-tested).
int hzTextToMhz(const std::string& text);  // "648000000\n" -> 648 (rounded); not a number -> -1
std::vector<int> frequencyListMhz(const std::string& text);  // "216000000 312000000" -> {216, 312}
// Index of the GPU entry among devfreq folder names (see open()), -1 if none fits.
int pickGpuEntry(const std::vector<std::string>& names);

}  // namespace fsim
