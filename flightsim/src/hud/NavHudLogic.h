#pragma once
// Nav and autopilot HUD logic (WP H3, HUD.md sections 5-6): everything that can be decided without
// drawing. FMA text per mode combination and the mode-change box timer, CDI / LOC / GS / VNAV
// scale positions with the +-1.25 clamp, heading tape mark placement (wrap at 360, off-scale
// parking), speed / altitude / minimums bug positions, the nav box text and the minimums trigger.
// NavHud.cpp only turns these results into strokes, so tests/hud/navhudLogicTest.cpp covers the
// rules without GL. All text goes into caller buffers (no heap allocation per frame).
//
// Sign conventions (core contracts): cdiDeflection + = course is RIGHT of the aircraft, so the
// needle diamond moves right; gsDeflection + = glidepath ABOVE the aircraft, so the diamond moves
// up; vnavDevFt + = path ABOVE the aircraft (up).

#include "core/AutopilotState.h"
#include "core/NavState.h"
#include "hud/HudLogic.h"

namespace fsim {

// ---------------------------------------------------------------------------------------------
// FMA
// ---------------------------------------------------------------------------------------------
constexpr double kFmaChangeBoxS = 5.0;  // HUD.md section 5: box around a new mode for 5 s
constexpr double kMinimumsCalloutS = 3.0;

enum class FmaColumn { Spd = 0, Lat, Vert, Count };

struct FmaText {
    char spd[12] = {0};       // "SPD", "RETARD" or ""
    char lat[8] = {0};        // active lateral: "ROL" "HDG" "NAV" "LOC"
    char latArmed[10] = {0};  // "(LOC)" or ""
    char vert[12] = {0};      // "PIT" "ALT" "ALT*" "VS +700" "G/S" "FLC"
    char vertArmed[10] = {0};
    char status[16] = {0};  // "AP FD A/T", "FD", "A/T" or ""
};
const char* latModeName(LatMode m);    // "" for Off
const char* vertModeName(VertMode m);  // "" for Off, VS has no value here
const char* spdModeName(SpdMode m);
void formatVertActive(char* out, int n, VertMode m, float vsBugFpm);  // "VS +700" (50 fpm steps)
void buildFma(const AutopilotState& ap, FmaText& out);

// Mode-change boxes. update() is called every frame with the current states; the first call
// only records them. A change of the active mode in a column starts a kFmaChangeBoxS window.
class FmaChangeTracker {
public:
    void reset() { have_ = false; }
    void update(const AutopilotState& ap, double nowS);
    bool boxVisible(FmaColumn c, double nowS) const;

private:
    bool have_ = false;
    int last_[3] = {0, 0, 0};
    double since_[3] = {-1e9, -1e9, -1e9};
};

// ---------------------------------------------------------------------------------------------
// Deviation scales
// ---------------------------------------------------------------------------------------------
constexpr float kDeviationClamp = 1.25f;  // NavState clamp, shown pegged at 2.5 dots
constexpr float kDotsPerFullScale = 2.0f;
constexpr float kVnavFullScaleFt = 250.0f;  // NAVIGATION section 4

struct DevPos {
    float dots = 0;       // signed, clamped to +-2.5
    bool pegged = false;  // at or beyond the clamp
};
// `deflection` is in full-scale units (+-1 = 2 dots); NaN-safe (NaN -> 0).
DevPos deviationDots(float deflection);
DevPos vnavDots(float vnavDevFt);

enum class CdiScale { None, Enr, Term, Apr };
const char* cdiScaleName(CdiScale s);  // "ENR" "TERM" "APR" ""
// GPS scale from the full-scale distance in use when known (ramps between), else from the phase.
CdiScale cdiScaleFor(const NavState& nav);
const char* navSourceName(NavSource s);  // "GPS" "LOC" "VOR" ""

struct ScaleState {
    bool locVisible = false;
    bool locFilled = false;  // false = hollow (armed, not captured)
    bool gsVisible = false;
    bool gsFilled = false;  // hollow for armed G/S and for advisory VNAV
    bool gsIsVnav = false;
    DevPos loc, gs;
};
ScaleState buildScales(const NavState& nav, const AutopilotState& ap);

// ---------------------------------------------------------------------------------------------
// Heading tape marks (magnetic)
// ---------------------------------------------------------------------------------------------
constexpr float kTapeHalfWidthPx = 150.0f;
struct TapeMark {
    float dxPx = 0;         // offset from the tape center, parked at +-kTapeHalfWidthPx if off
    bool offScale = false;  // |diff| > 35 deg: draw at the tape end and print the value
    int valueMagDeg = 0;    // rounded, 1..360 (0 prints as 360)
};
// Angles in any range; the difference wraps at 360.
TapeMark headingMark(float valueMagDeg, float centerMagDeg);
float trueToMagDeg(float trueDeg, float magVarDeg);  // [0,360)

// ---------------------------------------------------------------------------------------------
// Tape bugs
// ---------------------------------------------------------------------------------------------
struct BugPos {
    float y = 0;            // clamped into the tape
    bool offScale = false;  // beyond the tape: draw at the edge with the value printed
    bool above = false;     // off-scale at the top
};
BugPos tapeBug(const TapeGeom& g, float value, float centerValue);

// ---------------------------------------------------------------------------------------------
// Flight director cue
// ---------------------------------------------------------------------------------------------
struct FdOffset {
    float dx = 0, dy = 0;  // screen px relative to the FPV (y down)
};
constexpr float kFdRollPxPerDeg = 2.0f;
constexpr float kFdMaxXPx = 70.0f, kFdMaxYPx = 70.0f;
// Pitch error (command - actual) moves the cue up when positive; roll error (command - actual)
// moves it right when positive. Flying the FPV onto the cue removes both errors.
FdOffset fdCueOffset(float pitchCmdDeg, float pitchDeg, float rollCmdDeg, float rollDeg,
                     float pxPerDeg);

// ---------------------------------------------------------------------------------------------
// Minimums
// ---------------------------------------------------------------------------------------------
// Fires once when the baro altitude crosses from above to at-or-below the minimums of the loaded
// approach. The first sample only records the side. Re-arms when the aircraft climbs 100 ft above
// the minimums or the minimums disappear.
class MinimumsTracker {
public:
    void reset();
    void update(const NavState& nav, float altMslFt, double nowS);
    bool calloutVisible(double nowS) const;  // "MINIMUMS" amber for kMinimumsCalloutS
    // True once after each crossing (for the audio tone, S1).
    bool consumeAlert();

private:
    bool haveAlt_ = false, below_ = false, alert_ = false;
    double firedS_ = -1e9;
};
void formatMinimums(char* out, int n, const NavState& nav);  // "DA 213" / "MDA 560", "" if none

// ---------------------------------------------------------------------------------------------
// Nav box and FMA right-hand summary
// ---------------------------------------------------------------------------------------------
struct NavBoxText {
    char line[3][40] = {{0}, {0}, {0}};
    int lines = 0;
};
void formatEte(char* out, int n, float eteS);  // "6:12", "1:05:30", "--:--" when invalid
// Line count honours HudVisibility::navBoxLines (FULL: 3 lines, STD: 1, MIN: 0).
void buildNavBox(const NavState& nav, int navBoxLines, float magVarDeg, NavBoxText& out);
// One line for the FMA row: "GPS CEPIN 12.4 NM 6:12" (the diamond bearing pointer is drawn
// separately). Empty when there is no active waypoint.
void buildFmaNavSummary(const NavState& nav, char* out, int n);


}  // namespace fsim
