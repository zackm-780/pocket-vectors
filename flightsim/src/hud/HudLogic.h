#pragma once
// Flight HUD logic (WP H2): everything that can be decided without drawing. Tape scaling and label
// stepping, heading wrap, declutter rules, status-row field formatting, warning priority and flash
// timing, the speed trend filter and speed bands. Hud.cpp only turns these results into strokes,
// so the unit tests (tests/hud/hud*) cover the rules without GL.
//
// Units: every physical quantity carries its unit suffix; headings say True or Mag. All text
// formatting writes into caller buffers (no heap allocation per frame).

#include <cstdint>

#include "core/AircraftProfile.h"
#include "core/AircraftState.h"
#include "core/ControlInputs.h"
#include "hud/Renderer2D.h"

namespace fsim {

// ---------------------------------------------------------------------------------------------
// Options and declutter
// ---------------------------------------------------------------------------------------------

enum class Declutter { Full, Std, Min };
enum class HudPalette { Green, White, Amber };

Declutter nextDeclutter(Declutter d);  // FULL -> STD -> MIN -> FULL
const char* declutterName(Declutter d);
Color hudBaseColor(HudPalette c);

// Bits for HudOptions::extraWarnings: warnings produced outside H2 (X2 ground proximity, H3 AP).
enum class WarningId : int {
    Crash,
    PullUp,
    Stall,
    Terrain,
    SinkRate,
    Overspeed,
    Gear,
    ApOff,
    ParkBrk,
    Paused,
    Count
};
constexpr std::uint32_t warningBit(WarningId id) {
    return 1u << static_cast<int>(id);
}

struct HudOptions {
    Declutter declutter = Declutter::Full;
    HudPalette color = HudPalette::Green;
    bool paused = false;
    bool baroInHg = false;  // altimeter setting shown in inHg instead of hPa (P3 pressure units)
    float magVarDeg = 0;    // + = east; heading and wind are shown magnetic (true - magVar)
    // OR of warningBit() values raised by other modules: PullUp, Terrain, SinkRate (X2), ApOff (H3).
    std::uint32_t extraWarnings = 0;
};

// What each declutter level shows (HUD.md section 1). Attitude, speed, altitude, FPV, warnings and
// messages are always visible.
struct HudVisibility {
    bool headingTape = true;
    bool speedAltReadouts = true;  // GS / Mach under the speed tape
    bool statusRow1 = true, statusRow2 = true;
    bool row2FuelAndTime = true;
    bool fma = true;
    int navBoxLines = 2;  // H3: 2 (FULL), 1 (STD), 0 (MIN)
    // Ladder labels are drawn only for |pitch| > this (FULL: every labelled line, i.e. -1).
    int ladderLabelMinAbsDeg = -1;
};
HudVisibility visibilityFor(Declutter d);
bool ladderLabelVisible(const HudVisibility& v, int pitchDeg);

// ---------------------------------------------------------------------------------------------
// Layout zones (virtual px, 640-wide design coordinates; add Renderer2D::centered() offset for
// centered zones, rightX() - 640 for right anchored ones). H3 draws into the reserved zones.
// ---------------------------------------------------------------------------------------------
struct HudRect {
    float x, y, w, h;
};
namespace hudzones {
constexpr HudRect kFma{0, 0, 640, 22};              // H3: FMA row (centered)
constexpr HudRect kHeadingTape{170, 26, 300, 30};   // H2 draws, H3 adds marks (centered)
constexpr HudRect kNavBox{440, 368, 196, 60};       // H3: nav box (right anchored)
constexpr HudRect kLadderClip{120, 100, 400, 300};  // pitch ladder clip (centered); HUD.md says
    // y 70, trimmed so lines stay inside the bank arc
constexpr float kLocScaleY = 392;  // H3: LOC scale, horizontal, centered
constexpr float kGsScaleX = 500;   // H3: GS scale, vertical (centered x)
constexpr float kCenterY = 240;    // boresight and tape value boxes
}  // namespace hudzones

// ---------------------------------------------------------------------------------------------
// Tapes
// ---------------------------------------------------------------------------------------------
struct TapeGeom {
    float centerY = 240;  // y of the current-value box
    float pxPerUnit = 1;  // virtual px per ft or kt
    float topY = 70, bottomY = 370;
    float yOf(float value, float centerValue) const {
        return centerY - (value - centerValue) * pxPerUnit;
    }
    float valueAt(float y, float centerValue) const {
        return centerValue + (centerY - y) / pxPerUnit;
    }
};
constexpr TapeGeom kSpeedTapeGeom{240, 2.0f, 70, 370};  // 10 kt = 20 px
constexpr TapeGeom kAltTapeGeom{240, 0.16f, 70, 370};   // 100 ft = 16 px

struct TickRange {
    int first = 0,
        last = -1;  // tick values (multiples of step), first <= v <= last; empty if last < first
    bool empty() const { return last < first; }
};
// Tick values (multiples of `step`) whose y lies inside the tape for the given current value.
TickRange tickRange(const TapeGeom& g, float centerValue, int step);
// True for ticks that carry a label (value is a multiple of labelStep; handles negatives).
bool isLabelTick(int value, int labelStep);

// ---------------------------------------------------------------------------------------------
// Heading
// ---------------------------------------------------------------------------------------------
constexpr float kHeadingTapeHalfRangeDeg = 35.0f;  // +-35 deg visible over 300 px
constexpr float kHeadingTapePxPerDeg = 300.0f / 70.0f;

float headingMagDeg(float headingTrueRad, float magVarDeg);  // [0,360)
float trackMagDeg(float trackTrueRad, float magVarDeg);      // alias with the same math
float angleDiffDeg(float aDeg, float bDeg);                  // a - b wrapped to (-180, 180]
// x on the tape for a heading, relative to the tape center (wraps: 359 next to 001).
float headingTapeX(float valueMagDeg, float centerMagDeg, float centerXpx, float pxPerDeg);
int headingTapeLabel(int deg);  // 270 -> 27, 0 -> 36 (deg is a multiple of 10)
void formatHeadingBox(char* out, int n, float headingMagDeg);  // "000".."359", 359.6 -> "000"

// ---------------------------------------------------------------------------------------------
// Speed bands and trend
// ---------------------------------------------------------------------------------------------
struct SpeedBands {
    float redBelowKt = 0;                     // 0..this red (Vs1 clean, Vs0 with flaps)
    float whiteLowKt = 0, whiteHighKt = 0;    // flap operating range (empty if equal)
    float yellowLowKt = 0, yellowHighKt = 0;  // Vno..Vne caution (empty if equal)
    float redAboveKt = 0;                     // Vne / Vmo and above red (0 = none)
};
SpeedBands computeSpeedBands(const AircraftProfile& profile, int flapDetent);

// Low-pass filtered IAS acceleration -> 10 s predicted change (kt).
class SpeedTrend {
public:
    void reset() {
        have_ = false;
        accelKtPerS_ = 0;
    }
    void update(float iasKt, double simTimeS);
    float trend10sKt() const { return accelKtPerS_ * 10.0f; }
    float accelKtPerS() const { return accelKtPerS_; }

private:
    bool have_ = false;
    float lastIasKt_ = 0;
    double lastTimeS_ = 0;
    float accelKtPerS_ = 0;
};

// ---------------------------------------------------------------------------------------------
// Readout formatting
// ---------------------------------------------------------------------------------------------
int roundToStep(float value, int step);             // nearest multiple (half away from zero)
void formatThousands(char* out, int n, int value);  // 9840 -> "9 840"
void formatMach(char* out, int n, float mach);      // 0.78 -> "M.78", 1.02 -> "M1.02"
// X1: "STD" at the standard setting, else "29.92" (inHg) or "1013" (hPa).
void formatAltimeterSetting(char* out, int n, float settingHpa, bool inHg);
void formatVs(char* out, int n, float vsFpm);               // "VS +700" (rounded to 50)
int roundRadioAltFt(float agl);                             // 5 ft below 100, 10 below 500, else 50
void formatAltBoxThousands(char* out, int n, float altFt);  // "35" (thousands, sign kept)
void formatAltBoxRest(char* out, int n, float altFt);       // "000" (20 ft steps)
void formatFlightTime(char* out, int n, double simTimeS);   // "T+00:42:10"

// ---------------------------------------------------------------------------------------------
// Status rows (HUD.md section 4)
// ---------------------------------------------------------------------------------------------
enum class Role { Normal, White, Amber, Red, Dim };

struct StatusSeg {
    enum class Kind { Text, FlapBar, GearLights, TrimScale };
    Kind kind = Kind::Text;
    char text[28] = {0};
    Role role = Role::Normal;
    bool blinks = false;  // hidden half of the 2 Hz blink period (flap label while moving)
    int a = 0, b = 0;     // FlapBar: a = boxes, b = filled; GearLights: a = lit (0..3);
                          // TrimScale: value x100 in a (-100..100)
};
constexpr int kMaxSegs = 7;
struct StatusField {
    StatusSeg segs[kMaxSegs];
    int segCount = 0;
    bool present = false;  // row 2 reserves the slot when absent, row 1 skips it
    int slotCells = 0;     // row 2: minimum width in character cells
};

struct StatusInputs {
    const AircraftState* state = nullptr;
    const ControlInputs* controls = nullptr;
    const AircraftProfile* profile = nullptr;
    float magVarDeg = 0;
};

// Individual fields (also used directly by tests).
void formatThrottle(char* out, int n, const ControlInputs& c, int engineCount);
void formatFlapLabel(char* out, int n, const AircraftProfile& p, int detent);  // "FLAP 15"
bool flapsMoving(const AircraftState& s, const AircraftProfile& p);
bool flapsOverVfe(const AircraftState& s, const AircraftProfile& p);
// Vfe (kt) that applies to the flaps as they are now: per-detent FlapDetent::vfeKt of the highest
// of the selected detent and the detent the surface has reached, else the profile vfeKt; 0 = none.
float flapVfeLimitKt(const AircraftState& s, const AircraftProfile& p);
// "TRIM ^ +0.12" (arrow glyphs): up arrow = nose-up trim.
void formatTrim(char* out, int n, float elevTrimNorm);
// "L0.05" / "R0.05"; empty string when |value| below the display threshold.
void formatSideTrim(char* out, int n, float trimNorm);
void formatWind(char* out, int n, float windFromTrueDeg, float windKt, float magVarDeg);

// Row 2 field order (slots are reserved even when a field is absent so the row does not jump).
enum Row2Field { kSpdBrk = 0, kRev, kBrk, kPark, kFuel, kWind, kTime, kRow2Fields };
constexpr int kMaxStatusFields = 8;

// Builds the fields; returns the field count (row 2 always returns kRow2Fields, absent fields
// have present == false). `out` must hold kMaxStatusFields fields.
int buildStatusRow1(const StatusInputs& in, StatusField* out);
int buildStatusRow2(const StatusInputs& in, StatusField* out);
int segCells10(const StatusSeg& s);      // width in tenths of a character cell
int fieldCells10(const StatusField& f);  // sum of segments + 1 cell between segments

// ---------------------------------------------------------------------------------------------
// Warnings (HUD.md section 3 "Warnings", priority order = enum order)
// ---------------------------------------------------------------------------------------------
enum class Severity { Red, Amber, White };

std::uint32_t activeWarnings(const AircraftState& s, const ControlInputs& c,
                             const AircraftProfile& p, const HudOptions& opt);
// Highest priority first. Returns the count written (<= max).
int sortedWarnings(std::uint32_t mask, WarningId* out, int max);
Severity warningSeverity(WarningId id);
void formatWarning(char* out, int n, WarningId id, const AircraftState& s);
// New warnings flash at 2 Hz for 3 s, then are steady. ageS = seconds since it became active.
constexpr float kWarningFlashDurationS = 3.0f;
constexpr float kWarningFlashHz = 2.0f;
bool warningFlashVisible(double ageS);
bool gearWarningActive(const AircraftState& s, const ControlInputs& c, const AircraftProfile& p);

class WarningTracker {
public:
    // Call once per frame with the active mask and the HUD clock.
    void update(std::uint32_t mask, double nowS);
    double ageS(WarningId id, double nowS) const;
    bool visible(WarningId id, double nowS) const { return warningFlashVisible(ageS(id, nowS)); }

private:
    double since_[static_cast<int>(WarningId::Count)] = {};
    bool active_[static_cast<int>(WarningId::Count)] = {};
};

// ---------------------------------------------------------------------------------------------
// Transient messages
// ---------------------------------------------------------------------------------------------
constexpr double kMessageLifeS = 2.0;
constexpr double kMessageFadeS = 0.5;
constexpr int kMaxShownMessages = 3;
float messageAlpha(double ageS);  // 1 until life - fade, then fading to 0, 0 after life

}  // namespace fsim
