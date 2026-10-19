// The probe's report: an ordered list of sections with text lines, plus the "key facts" table that maps
// one-to-one onto the rows of docs/DEVICE.md section 2.
//
// The file is rewritten atomically (temp file + rename) after every section, so a crash, a hang or a
// Select+Start kill still leaves the most recent complete report on the SD card.
#pragma once

#include <deque>
#include <string>
#include <utility>
#include <vector>

namespace probe {

class Report {
public:
    explicit Report(std::string path);

    // Starts a new section ("== TITLE ==") and saves the report.
    void section(const std::string& title);
    // Appends a line to the current section. Also echoed to stdout and shown on screen.
    void line(const std::string& text);
    // Same, but only goes to the file and stdout (used for long raw event logs).
    void quietLine(const std::string& text);
    // "key: value" convenience.
    void kv(const std::string& key, const std::string& value);

    // Sets the value of a DEVICE.md section 2 row. Unknown rows are appended.
    void fact(const std::string& row, const std::string& value);

    // Free-form status shown in the report header, e.g. "running: gpu benchmark".
    void setStatus(const std::string& status);

    // Writes the report file. Returns false (and prints a warning) if the directory is not writable.
    bool save();

    const std::string& path() const { return path_; }
    // Most recent lines for the on-screen log, oldest first.
    const std::deque<std::string>& screenLines() const { return screenLines_; }
    const std::vector<std::pair<std::string, std::string>>& facts() const { return facts_; }
    std::string factValue(const std::string& row) const;

    // Full report text (what save() writes).
    std::string render() const;

private:
    struct Section {
        std::string title;
        std::vector<std::string> lines;
    };

    std::string path_;
    std::string status_ = "starting";
    std::vector<Section> sections_;
    std::vector<std::pair<std::string, std::string>> facts_;
    std::deque<std::string> screenLines_;
    double startS_;
    std::string startedAt_;
    bool warnedUnwritable_ = false;
};

// Names of the DEVICE.md section 2 rows. Keep in sync with docs/DEVICE.md.
namespace rows {
inline constexpr const char* kGlibc = "glibc version";
inline constexpr const char* kKernel = "Kernel";
inline constexpr const char* kControlFolder = "PortMaster install path (controlfolder)";
inline constexpr const char* kPortsFolder = "Ports folder path on the SD card";
inline constexpr const char* kSdl = "SDL2 version and video driver";
inline constexpr const char* kGl = "GL_VERSION / GL_RENDERER / extensions";
inline constexpr const char* kDepthMsaa = "Depth bits / MSAA available";
inline constexpr const char* kMaxTex = "Max texture size";
inline constexpr const char* kTriggers = "L2/R2 analog or digital?";
inline constexpr const char* kStickClicks = "L3/R3 (stick clicks) present?";
inline constexpr const char* kFaceButtons = "Face-button names as SDL reports them (A/B swap?)";
inline constexpr const char* kMenuKey = "Menu key usable by ports?";
inline constexpr const char* kFreeRam = "Free RAM at idle in a port";
inline constexpr const char* kGpu = "GPU: triangles/frame at 30 fps (fog shader)";
inline constexpr const char* kCpu = "CPU: JSBSim us/step (c172p, 737)";
inline constexpr const char* kAudio = "Audio device / sample rate";
}  // namespace rows

}  // namespace probe
