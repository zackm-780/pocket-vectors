# Architecture

## 1. Data flow

```
        ┌───────────┐  ActionState   ┌───────────────┐ ControlInputs (pilot)
SDL ───▶│ input/    │──────────────▶ │ PilotControls │───────────────┐
        └───────────┘                └───────────────┘               ▼
                                                            ┌────────────────┐ ControlInputs (final)
         NavDatabase ◀── data/nav     FlightPlan ──▶ ┌────▶│ autopilot/     │────────────┐
              │                                        │    │ AP / FD / A/T  │            ▼
              ▼                                        │    └────────────────┘   ┌────────────────┐
        ┌───────────┐  NavState                       │      AutopilotState     │ sim/           │
        │ nav/      │─────────────────────────────────┤                          │ FlightModel    │
        │ Navigator │◀──────── AircraftState ─────────┼──────────────────────────│ (JSBSim)       │
        └───────────┘                                  │                          └───────▲────────┘
              │                                        │                                  │ ground
              ▼                                        ▼                                  │ queries
        ┌──────────────────────────────────────────────────────┐   ┌──────────────────────┴───┐
        │ Renderers: Sky → Terrain → Airports → HUD → UI/Map   │◀──│ world/ TerrainService     │◀── data/terrain
        └──────────────────────────────────────────────────────┘   │ (TerrainQuery)            │
                                                                     └───────────────────────────┘
```

All cross-module data passes through the **contract types in `src/core/`** (section 5). Modules
depend on `core/`, never on each other's internals. Example: the HUD reads `AircraftState`,
`NavState` and `AutopilotState` and never includes JSBSim or Navigator headers.

## 2. Source tree and ownership

```
flightsim/
  CMakeLists.txt, cmake/            build, deps (FetchContent), toolchains        WP-00, P1
  src/
    main.cpp                        entry point                                     WP-00
    core/                           SHARED CONTRACTS (additive changes only)        WP-00 (+supervisor)
    app/                            App, game state machine, CLI, scenarios         WP-00, P3
    platform/                       SDL window, GL context, timing, platform detect WP-00, P1
    gfx/                            GL loader, shaders, buffers, textures, camera,   WP-00 base;
                                    RTC helpers, screenshot                          W2/W4/W5/H1 add files
    input/                          InputSystem, bindings, PilotControls            P2
    sim/                            FlightModel (JSBSim), profiles, ground callback F1, F3
    world/terrain/                  tile IO, cache, TerrainService                  W1
    world/                          TerrainRenderer, SkyRenderer, AirportRenderer   W2, W4, W5
    nav/                            NavDatabase, FlightPlan, FMS IO, procedures,    N2, N3, N4, N5
                                    Navigator, ApproachBuilder, MagVar
    autopilot/                      Autopilot, FlightDirector, AutoThrottle         N6
    hud/                            Renderer2D, StrokeFont, Hud, NavHud             H1, H2, H3
    ui/                             menu framework, screens, on-screen keyboard,    H4, H5
                                    MapPage
    audio/                          procedural audio                                S1
  assets/
    aircraft/<id>.ini               aircraft profiles                               F2, F4
    jsbsim-overlay/                 fixes layered over stock JSBSim files           F2, F4
    scenarios/*.ini                 start / test scenarios                          any WP (own files)
  tests/                            doctest unit tests, mirrors src/ layout          each WP its own subdir
  tools/terrain/                    terrain pipeline (Python)                       W3
  tools/nav/                        nav pipeline (Python)                           N1, N4
  tools/probe/                      device probe program                            P1
  port/                             PortMaster launch script and metadata          P1
  docs/                             these documents                                 supervisor; WPs update their sections
```

## 3. Frame loop and threading

```
loop:
  dtS = clock.tick()                                      // wall time, clamped to 0.1 s
  actions = input.poll()                                  // SDL events → ActionState
  stateMachine.update(actions)                            // menus/map consume input when open
  if (state == Flying && !paused):
      pilot = pilotControls.update(actions, profile, aircraftState, dtS)
      accumulatorS += dtS
      while accumulatorS >= kSimDtS (1/120):
          final = autopilot.update(pilot, aircraftState, navState, kSimDtS)
          flightModel.step(final, kSimDtS)                // JSBSim Run()
          aircraftState = flightModel.state()
          accumulatorS -= kSimDtS
      navState = navigator.update(aircraftState, dtS)
  terrain.update(cameraGeo)                               // schedule loads, publish finished tiles
  render: sky → terrain → airports → HUD → UI/map overlays → swap
```

- **Main thread:** input, sim, nav, autopilot and **all GL calls**.
- **One worker thread** (W1): tile file IO and CPU-side mesh building. It never touches GL or JSBSim.
  Finished tiles are published to the main thread through a queue. GPU uploads are limited per
  frame (budget set by W2).
- The sim runs at 120 Hz no matter what the frame rate is. If the device can't keep up, the profile
  may set `sim_hz = 60` (validate ground handling first).
- Pause stops the accumulator. Menus and the map can open while paused.

## 4. Coordinate frames and units

| Frame | Definition | Used for |
|---|---|---|
| **Geodetic** | WGS84 latitude/longitude in degrees (`double`). Altitude in ft or m **MSL** (D16). | Positions everywhere: state, nav DB, plans, terrain queries |
| **NED** | Local north-east-down at the aircraft | JSBSim Euler angles (`rollRad`, `pitchRad`, `headingTrueRad`), velocities |
| **Body** | x forward, y right, z down (JSBSim body axes), origin at CG | Eye offset, rates |
| **ECEF** | WGS84 earth-centered earth-fixed, meters (`double`) | Rendering anchors (RTC) |
| **Camera/eye** | GL convention: x right, y up, −z forward | View matrix |
| **HUD virtual** | 640×480 virtual pixels, origin top-left, y down. Scaled to the drawable; wider aspect ratios extend the width. | All 2D drawing |

**Rendering (D7):** each mesh stores a `double` ECEF anchor and `float` vertex offsets in ECEF axes.
Per draw: `t = float(anchorEcef - cameraEcef)`, and model-view = `cameraRotation * translate(t)`.
The camera rotation maps ECEF axes to eye axes (built from NED attitude plus the lat/lon rotation).
Never put absolute ECEF coordinates in floats.

**Units:** every physical quantity carries a unit suffix (CONVENTIONS.md). Math is done in SI and
radians. Aviation units (ft, kt, NM, fpm, degrees) are fine in state snapshots, data files and display
code. Conversions live in `core/Units.h`. **Headings, courses and bearings are true internally.**
Magnetic is used only for display and user input, converted with `MagVar` (N2, from `data/nav/magvar.bin`).

## 5. Shared contracts (`src/core/`)

WP-00 created these headers. **The headers in `src/core/` are now authoritative**; the sketches
below show names, units and semantics. WP-00 also added `core/SimTime.h` (`kSimHz`, `kSimDtS`,
`kMaxFrameDtS`), `core/Assert.h`, the `LatLon` / `Vec3d` / `Mat3d` types in `Geo.h`
(`crossTrackM` and `alongTrackM` take `LatLon`), `wrap2Pi` / `wrap180`,
`AircraftProfile::eyeOffsetBodyM[3]` + `hasEyeOffset`, and `FlatTerrain` / `SyntheticNavDb`
(with `findRunway()`). Later additive changes: P2 added `Axis::BugRate`, `Action::Quit`,
`ActionState::repeatCount[]` (auto-repeats fired in the current hold, for accelerated bug steps)
and `core/MessageQueue.h` (fixed ring of short HUD feedback messages, ticked once per frame). N2
documented `Ils::locWidthDeg` as the total course width and `freqMhz = 0` as untuned. The
supervisor added `Approach` / `ApproachTransition` to `NavTypes.h` (the one approach type that N4's
real CIFP procedures and N5's synthesized approaches both produce) and
`NavDatabase::navaidsNear()` (default empty). F1 added `FlapDetent::vfeKt` (per-detent flap limit; 0 = use the profile `vfeKt`)
and stores extra profile sections `[crash]`/`[ground]` as `crash.*`/`ground.*` keys in
`AircraftProfile::properties` (documented in `src/sim/README.md`). After WP-00 lands, changes are **additive only** (CLAUDE.md). Every struct
must have safe defaults, so a zero-initialised value means "nothing / invalid".

```cpp
namespace fsim {

constexpr int kMaxEngines = 4;

// ---------- core/Units.h ----------
// kFtPerM, kMPerFt, kMPerNm, kKtPerMps, kMpsPerKt, kFpmPerMps, kDegPerRad, kRadPerDeg,
// kEarthRadiusM (mean, 6371008.8), WGS84 a/f constants. Inline helpers: ftToM(), mToFt(), ...

// ---------- core/Geo.h ----------
struct GeoPos { double latDeg = 0, lonDeg = 0; double altMslM = 0; };
struct Ecef   { double x = 0, y = 0, z = 0; };
Ecef   geodeticToEcef(const GeoPos&);
GeoPos ecefToGeodetic(const Ecef&);
double greatCircleDistM(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
double initialBearingTrueRad(double lat1Deg, double lon1Deg, double lat2Deg, double lon2Deg);
void   destinationPoint(double latDeg, double lonDeg, double bearingTrueRad, double distM,
                        double& outLatDeg, double& outLonDeg);
double crossTrackM(/* point, leg start, leg end */);   // signed: + = right of track
double alongTrackM(/* point, leg start, leg end */);
// NED<->ECEF rotation helpers, wrapPi(), wrap360()

// ---------- core/AircraftState.h ----------   produced by sim/FlightModel, read by everyone
struct EngineState {
  bool  running = false;
  float throttleCmdNorm = 0;     // 0..1 as commanded
  float n1Pct = 0, n2Pct = 0;    // turbines (0 for pistons)
  float rpm = 0;                 // pistons/props (0 for jets)
  float thrustLbf = 0;
  bool  reverserDeployed = false;
};
struct AircraftState {
  bool   valid = false;
  double simTimeS = 0;
  // position
  double latDeg = 0, lonDeg = 0, altMslFt = 0;
  float  altAglFt = 0;              // above terrain/runway surface under the CG
  // Additive (X1): indicated altitude = altMslFt + altBaroOffsetFt (0 while the altimeter is set
  // to the real QNH, the default); altimeterSettingHpa is what the pilot has set. See core/Baro.h.
  float  altBaroOffsetFt = 0, altimeterSettingHpa = 1013.25f;
  float  terrainElevFt = 0;
  // attitude and rates (NED)
  float  rollRad = 0, pitchRad = 0, headingTrueRad = 0;
  float  pRadps = 0, qRadps = 0, rRadps = 0;
  // air data
  float  iasKt = 0, tasKt = 0, mach = 0, alphaDeg = 0, betaDeg = 0, vsFpm = 0, nzG = 1;
  // ground data
  float  gsKt = 0, trackTrueRad = 0;
  float  velNorthMps = 0, velEastMps = 0, velDownMps = 0;
  // wind at aircraft
  float  windFromTrueDeg = 0, windKt = 0;
  // eye point relative to CG, body axes, meters
  float  eyeBodyM[3] = {0, 0, 0};
  // configuration
  float  flapPosNorm = 0;           // actual surface position 0..1
  int    flapDetent = 0;            // currently selected detent index
  float  gearPosNorm = 1;           // 1 = down and locked
  bool   gearRetractable = false;
  bool   wowAny = false, wowAll = false;   // weight on wheels
  float  speedbrakePosNorm = 0;
  float  elevTrimNorm = 0, aileronTrimNorm = 0, rudderTrimNorm = 0;   // -1..1
  float  brakeLeftNorm = 0, brakeRightNorm = 0;
  bool   parkingBrake = false;
  int    engineCount = 0;
  EngineState engines[kMaxEngines];
  float  fuelTotalLbs = 0;
  // warnings
  bool   stallWarning = false, overspeed = false;
  bool   crashed = false;
  char   crashReason[48] = {0};
};

// ---------- core/ControlInputs.h ----------   produced by input/PilotControls, modified by autopilot/, consumed by sim/
struct ControlInputs {
  float elevatorNorm = 0, aileronNorm = 0, rudderNorm = 0;   // -1..1 (elevator +1 = nose up)
  float throttleNorm[kMaxEngines] = {0, 0, 0, 0};            // 0..1
  bool  reverseThrust = false;
  float mixtureNorm = 1;                                     // pistons
  float elevTrimNorm = 0, aileronTrimNorm = 0, rudderTrimNorm = 0; // absolute, -1..1
  int   flapDetent = 0;                                      // index into profile detents
  bool  gearDown = true;
  float speedbrakeNorm = 0;                                  // 0..1
  float brakeLeftNorm = 0, brakeRightNorm = 0;
  bool  parkingBrake = false;
  float steeringNorm = 0;                                    // -1..1, usually follows rudder
};

// ---------- core/AircraftProfile.h ----------   parsed from assets/aircraft/<id>.ini by sim/
struct FlapDetent { float cmdNorm = 0; char label[8] = {0}; };   // "UP", "10", "15"...
enum class AircraftCategory { Piston, Turboprop, Jet };
struct AircraftProfile {
  std::string id, displayName, jsbsimModel;
  AircraftCategory category = AircraftCategory::Piston;
  int engineCount = 1;
  std::vector<FlapDetent> flapDetents;
  bool hasRetractableGear = false, hasSpeedbrake = false, hasReversers = false, hasAutothrottle = false;
  // speeds, kt IAS
  float vs0Kt = 0, vs1Kt = 0, vrKt = 0, vrefKt = 0, vappKt = 0, vfeKt = 0, vnoKt = 0, vneKt = 0;
  float vmoKt = 0, mmo = 0, cruiseKt = 0, cruiseAltFt = 0;
  float stallWarnAlphaDeg = 0;
  // start configurations
  int   approachFlapDetent = 0, takeoffFlapDetent = 0;
  // autopilot gains and limits (N6 defines the keys; generic map keeps core stable)
  std::map<std::string, float> autopilot;
  // JSBSim property overrides for non-standard models (F1 defines the keys)
  std::map<std::string, std::string> properties;
};

// ---------- core/TerrainQuery.h ----------   implemented by world/terrain (W1); used by sim, renderers, HUD, nav
enum class LandClass : uint8_t { Unknown = 0, Water, Tree, Shrub, Grass, Crop, Urban, Bare,
                                 SnowIce, Wetland, Mangrove, Moss };
struct FlattenArea {               // e.g. a runway rectangle plus margin, forced to one elevation
  double latDeg[4], lonDeg[4];     // convex quad, CCW
  float  elevM;
  float  blendM = 60;              // blend distance back to natural terrain
};
class TerrainQuery {
 public:
  virtual ~TerrainQuery() = default;
  // Surface elevation (after flattening), meters MSL. NEVER blocks: uses the best loaded LOD,
  // and returns 0 (sea level) if nothing is loaded.
  virtual float elevationM(double latDeg, double lonDeg) const = 0;
  virtual LandClass landClass(double latDeg, double lonDeg) const = 0;
  virtual void  addFlattenArea(const FlattenArea& area) = 0;
  virtual void  clearFlattenAreas() = 0;
  // Ask for full detail around a point (ground contact, start positions).
  virtual void  prefetch(double latDeg, double lonDeg, float radiusM) = 0;
  // True when the finest available LOD around the point is resident (gate for spawning).
  virtual bool  isDetailReady(double latDeg, double lonDeg) const = 0;
};
// WP-00 ships FlatTerrain (constant elevation, honours FlattenArea) for tests and fallback.

// ---------- core/NavTypes.h ----------
enum class AirportType { Large, Medium, Small, Seaplane, Unknown };
struct Ils {
  char   ident[6] = {0}; float freqMhz = 0;
  double locLatDeg = 0, locLonDeg = 0; float locCourseTrueDeg = 0; float locWidthDeg = 5;
  double gsLatDeg = 0, gsLonDeg = 0;  float gsElevFt = 0; float gsAngleDeg = 3; float tchFt = 50;
  bool   hasGs = true; bool synthesized = false;
};
struct Runway {                       // ONE LANDING DIRECTION (e.g. "28R")
  std::string airportIdent, ident, oppositeIdent;
  double thrLatDeg = 0, thrLonDeg = 0; float thrElevFt = 0;     // landing threshold (displacement applied)
  double startLatDeg = 0, startLonDeg = 0;                      // physical start of pavement
  double endLatDeg = 0, endLonDeg = 0; float endElevFt = 0;     // far end
  float  headingTrueDeg = 0, lengthFt = 0, widthFt = 0, displacedFt = 0;
  bool   hard = true, lighted = false, approxGeometry = false;
  std::optional<Ils> ils;
};
struct Airport {
  std::string ident, name, iso;  double latDeg = 0, lonDeg = 0; float elevFt = 0;
  AirportType type = AirportType::Unknown;
  std::vector<Runway> runways;  // both directions of every runway
};
enum class NavaidType { Vor, VorDme, Vortac, Tacan, Ndb, NdbDme, Dme };
struct Navaid { std::string ident, name; NavaidType type; float freqKhz = 0;
                double latDeg = 0, lonDeg = 0; float elevFt = 0; float magVarDeg = 0; float rangeNm = 0; };
enum class WaypointKind { Airport, Runway, Vor, Ndb, Fix, LatLon, Synthetic };
enum class AltConstraintKind { None, At, AtOrAbove, AtOrBelow, Between };
struct AltConstraint { AltConstraintKind kind = AltConstraintKind::None; float altFt = 0, alt2Ft = 0; };
struct Waypoint { std::string ident; WaypointKind kind = WaypointKind::Fix; double latDeg = 0, lonDeg = 0; };
enum class LegType { IF, TF, CF, DF, CA, VA, FA, HM, HA, HF, RF };   // ARINC 424 subset (NAVIGATION.md)
struct Leg {
  LegType type = LegType::TF;  Waypoint to;
  float courseTrueDeg = 0;                 // CF/CA/VA/FA
  AltConstraint alt;  float speedLimitKt = 0;  bool flyOver = false;
  bool  missedApproach = false;
  double arcCenterLatDeg = 0, arcCenterLonDeg = 0; float arcRadiusNm = 0; bool arcRight = true; // RF
};
enum class ApproachKind { None, Ils, Loc, Rnav, Vor, Ndb, Visual };
struct FlightPlan {
  std::string originIdent, destIdent, depRunway, arrRunway, approachId;
  ApproachKind approachKind = ApproachKind::None;
  float cruiseAltFt = 0;
  std::vector<Leg> legs;
  int activeLeg = -1;                      // index into legs, -1 = none
  int approachStartLeg = -1;               // first approach leg (legs[approachStartLeg..] = approach+missed), -1 = none (N3)
};

// ---------- core/NavDatabase.h ----------   implemented by nav/ (N2); WP-00 ships SyntheticNavDb (XTST)
class NavDatabase {
 public:
  virtual ~NavDatabase() = default;
  virtual const Airport* findAirport(std::string_view ident) const = 0;
  virtual std::vector<const Airport*> nearestAirports(double latDeg, double lonDeg, int maxCount,
                                                      float maxDistNm) const = 0;
  virtual std::vector<const Airport*> searchAirports(std::string_view text, int maxCount) const = 0;
  virtual std::vector<const Navaid*> findNavaids(std::string_view ident) const = 0;  // may be ambiguous
  virtual std::vector<Waypoint> findFixes(std::string_view ident) const = 0;          // may be ambiguous
  virtual std::vector<const Airport*> airportsInBox(double south, double west,
                                                    double north, double east) const = 0;
  virtual float magVarDeg(double latDeg, double lonDeg) const = 0;   // + = east
};

// ---------- core/NavState.h ----------   produced by nav/Navigator (N5), read by HUD, map, autopilot
enum class NavSource { None, Gps, Vor, Loc };
enum class NavPhase { None, Departure, Enroute, Terminal, Approach, Missed };
struct NavState {
  NavSource source = NavSource::None;  NavPhase phase = NavPhase::None;
  char  activeWptIdent[8] = {0};  double wptLatDeg = 0, wptLonDeg = 0;
  char  nextWptIdent[8] = {0};
  float distToWptNm = 0, eteToWptS = 0, distToDestNm = 0, eteToDestS = 0;
  float bearingToWptTrueDeg = 0, desiredTrackTrueDeg = 0, crossTrackNm = 0;   // xtk + = right of course
  float cdiDeflection = 0;      // ±1 = full scale (2 dots), clamped to ±1.25; + = course is RIGHT
  float cdiFullScaleNm = 0;     // GPS scaling in use (0 when angular)
  bool  toFlag = true;
  // vertical
  bool  gsValid = false;  float gsDeflection = 0;   // ±1 full scale, clamped to ±1.25; + = glidepath ABOVE aircraft
  float targetAltFt = 0;  float requiredVsFpm = 0;  // advisory VNAV
  // approach
  bool  locValid = false; char approachName[16] = {0};
  float minimumsFt = 0;   bool minimumsIsDa = true;   // DA (MSL) or MDA (MSL)
  float distToThresholdNm = 0;
  float magVarDeg = 0;    // at aircraft position, + = east
  // Additive (N5): advisory VNAV and bookkeeping, zero / false when not applicable.
  bool vnavValid = false;   // advisory 3 degree path to targetAltFt exists (aircraft above it)
  float vnavDevFt = 0;      // path altitude - aircraft altitude: + = path ABOVE the aircraft
  float todDistNm = 0;      // distance to the top of descent along the route; 0 = at/past it
  int activeLegIndex = -1;  // FlightPlan::legs index of the active leg, -1 = none
  bool onFinal = false;     // active leg lies on the final approach course (non-missed)
};

// ---------- core/AutopilotState.h ----------   produced by autopilot/ (N6), read by HUD
enum class LatMode  { Off, Roll, Hdg, Nav, Loc };
enum class VertMode { Off, Pitch, Alt, AltCapture, Vs, Gs, Flc };
enum class SpdMode  { Off, Speed, Retard };
struct AutopilotState {
  bool apEngaged = false, fdOn = false, atEngaged = false;
  LatMode latActive = LatMode::Off, latArmed = LatMode::Off;
  VertMode vertActive = VertMode::Off, vertArmed = VertMode::Off;
  SpdMode spdActive = SpdMode::Off;
  float hdgBugMagDeg = 0, altBugFt = 0, vsBugFpm = 0, spdBugKt = 0;
  float fdPitchCmdDeg = 0, fdRollCmdDeg = 0; bool fdValid = false;
  bool  disconnectWarning = false;          // flashing AP OFF
};

// ---------- core/Actions.h ----------   semantic input actions (P2 binds physical inputs to these)
enum class Axis { Pitch, Roll, Rudder, ThrottleRate, LookYaw, LookPitch, Count };
enum class Action { TrimNoseUp, TrimNoseDown, FlapsUp, FlapsDown, GearToggle, Brakes, ParkingBrake,
                    SpeedbrakeCycle, ReverseToggle, HudDeclutter, MapToggle, Pause, LookHold,
                    LookLeft, LookRight, LookBack, LookDown,
                    ApToggle, AtToggle, FdToggle, HdgMode, NavMode, AprMode, AltMode, VsMode,
                    HdgBugDec, HdgBugInc, AltBugDec, AltBugInc, SyncBugs, NavSourceCycle, DirectTo,
                    RudderTrimLeft, RudderTrimRight, MenuUp, MenuDown, MenuLeft, MenuRight,
                    MenuAccept, MenuBack, Screenshot, Count };
struct ActionState {
  float axis[(int)Axis::Count] = {};          // -1..1 after deadzone/curve
  bool  down[(int)Action::Count] = {};        // held
  bool  pressed[(int)Action::Count] = {};     // edge this frame (with auto-repeat where bound)
};

// ---------- core/Log.h, core/Ini.h, core/Csv.h, core/Paths.h ----------
// LOG_INFO/WARN/ERROR(fmt, ...) → stderr + conf/log file; Ini (sections, typed getters with defaults);
// Csv (header-indexed rows, streaming); Paths (asset root, data root, conf root resolution).
}
```

## 6. Rendering

- **API:** GLES 2.0 / GLSL ES 1.00 is the baseline. Desktop uses a GL 2.1 context and the same code:
  every GL function is loaded with `SDL_GL_GetProcAddress` (gfx loader), and shaders get a preamble
  (`#version 100` + `precision mediump float;` on ES, `#version 120` + empty precision macros on
  desktop). Optional extensions (MSAA, 32-bit indices, anisotropy) are queried at runtime and
  must have fallbacks. Use 16-bit indices for meshes ≤ 65,535 vertices.
- **Pass order:** sky (full-screen gradient using the inverse view-projection) → terrain → airports
  (runway surface, then markings with polygon offset, then lights as point sprites) → HUD (2D,
  depth off) → UI and map overlays.
- **Projection:** vertical FOV setting (default 55°), near 0.5 m, far = max(40 km, horizon distance
  + 20 km), capped at 400 km. 24-bit depth. If far-range precision is insufficient, W2 may split the
  frustum into near and far passes.
- **Fog/visibility:** exponential fog toward the horizon haze color, shared by sky, terrain and
  airports through a common uniform block (W4 owns the parameters).
- **HUD conformality:** horizon, pitch ladder and flight path vector must use the **same** view and
  projection as the 3D scene. `gfx/Camera` exposes `projectToHud(dirEye) → HUD virtual px`.

## 7. Flight model integration (JSBSim)

- `FGFDMExec` with root = the runtime `jsbsim/` directory, which holds `aircraft/`, `engine/` and
  `systems/` (stock files for our fleet plus `assets/jsbsim-overlay/`).
- Debug output off. The model loads by profile `jsbsim_model`.
- **Inputs** go to standard properties: `fcs/aileron-cmd-norm`, `fcs/elevator-cmd-norm`,
  `fcs/rudder-cmd-norm`, `fcs/throttle-cmd-norm[n]`, `fcs/mixture-cmd-norm[n]`, `fcs/flap-cmd-norm`,
  `gear/gear-cmd-norm`, `fcs/speedbrake-cmd-norm`, `fcs/pitch-trim-cmd-norm`,
  `fcs/roll-trim-cmd-norm`, `fcs/yaw-trim-cmd-norm`, `fcs/left-brake-cmd-norm`,
  `fcs/right-brake-cmd-norm`, `fcs/center-brake-cmd-norm`, `fcs/steer-cmd-norm`.
  Reversers and anything non-standard are mapped per aircraft through profile `[properties]`.
- **Outputs** come from `position/*`, `attitude/*`, `velocities/*`, `aero/*`, `accelerations/*`,
  `fcs/*-pos-norm`, `gear/*`, `propulsion/engine[n]/*`.
  **These names are starting points.** F1 must verify every one against JSBSim v1.3.3 and each
  model, and record the final mapping in `sim/README.md`.
- **Sign conventions (verified by WP-00):** JSBSim `fcs/elevator-cmd-norm` + = **nose down** and
  `fcs/rudder-cmd-norm` + = **yaw left**, the opposite of ours. Aileron + = roll right in both, and
  `fcs/steer-cmd-norm` + = right. `FlightModel` converts at the boundary (elevator, rudder and their
  trims are negated). No other module may talk to JSBSim properties directly.
- **Latitude:** set the IC with `SetGeodLatitudeRadIC`. `SetLatitudeDegIC` is geocentric and puts a
  40° start about 0.19° off.
- **Trim recipe (verified for c172p and 737):** settle flaps and gear with `SetHoldDown(true)`
  before trimming (737 flaps take about 25 s). `DoTrim(tFull)` already writes the elevator trim into
  `fcs/pitch-trim-cmd-norm`, so only the trimmed aileron and rudder are moved into the roll and yaw
  trims. Wind (F1, supersedes the earlier "apply wind after speed and re-trim" recipe, which fails
  for quartering crosswinds): trim in **calm air** at the requested IAS, then switch the wind on with
  `FGWinds::SetWindNED` and shift the body velocity by the same vector (IAS stays exact). Final
  starts crab the IC heading so the ground track stays on the extended centerline.
- **Weight on wheels:** `wowAny` and `wowAll` count wheel (bogey) contact points only. Contact by any
  non-wheel point is a structure strike, which sets `crashed`.
- **Stock-model quirks** (fixed by F2 in `assets/jsbsim-overlay/`): the 737 binds input sockets
  5137 and 5139; the global5000 writes `global5000.csv` to the working directory; the c310's
  constant-speed propeller cannot be trimmed, so it gets a fixed-pitch propeller. (The c310 output
  sockets 1138/1139 are already commented out in v1.3.3.)
- **Ground:** a custom `JSBSim::FGGroundCallback` (F3) answers from `TerrainQuery`, including
  runway flattening, with spawn gating on resident terrain detail.
- **Start conditions** (F1): runway (on the threshold, brakes set), final (N NM out on the 3°
  path, in approach configuration and trimmed), cruise (trimmed at profile cruise). After JSBSim trim,
  move the trimmed elevator command into `elevTrimNorm` so a centered stick holds the trim.
  Engines start with `InitRunning(-1)`.
- **Eye point:** from the model's `EYEPOINT` (structural frame) converted to body axes, falling back
  to the profile `[view] eye_offset_m`.
- **Gear protection:** ignore gear-up while any wheel has weight on it.
- **Crash detection:** vertical speed at touchdown, bank or pitch limits on ground contact, or
  terrain contact with a non-gear point. Result goes to `crashed` and `crashReason`.

## 8. Data and paths

- Read-only **asset root**: the executable directory (desktop: the build or install dir; device: the
  port folder). Contains `jsbsim/`, `assets/`.
- **Data root** (large, user-built): default `<asset root>/data`, overridable with `--data-dir` or
  `data_dir` in settings. Contains `terrain/`, `nav/`, `plans/` (format in [DATA.md](DATA.md)).
- **Conf root** (writable): desktop `~/.config/flightsim/`; device `<port>/conf/`. Holds
  `settings.ini`, `controls.ini`, the log, and the last flight.
- **Missing data never crashes.** No terrain pack means flat ground at 0 ft (or the scenario
  elevation). No nav DB means only the synthetic airport `XTST`.

## 9. Testability hooks (WP-00 provides these; every WP uses them)

| Flag | Meaning |
|---|---|
| `--headless` | No window or GL. Runs sim + nav + autopilot only, as fast as possible |
| `--scenario <file.ini>` | Aircraft, airport/runway, start type, wind, scripted inputs |
| `--duration <s>` | Sim seconds to run (headless) |
| `--trace <file.csv>` | Writes AircraftState + NavState + AutopilotState each 0.1 s |
| `--frames <n>` / `--screenshot <file.png>` | Render n frames, save a PNG, exit (works under Xvfb + Mesa llvmpipe) |
| `--aircraft <id>`, `--data-dir <path>`, `--window WxH`, `--fullscreen`, `--show-fps` | Self-explanatory |

**Synthetic world (always available):** airport `XTST` at 40.0000 N, 100.0000 W, elevation
2,000 ft, runway 09/27, 10,000 × 150 ft, true heading 090/270, ILS on both ends (synthesized), flat
terrain. All tests and screenshots that don't explicitly need real data use XTST.

Scenario file:

```ini
[scenario]
aircraft = c172p
airport = XTST
runway = 27
; start: runway | final | cruise
start = final
; distance_nm: final only. altitude_ft: cruise only (0 = profile cruise altitude)
distance_nm = 6
altitude_ft = 0
time_utc = 2026-06-21T18:00:00
[wind]
from_true_deg = 270
speed_kt = 10
; optional timeline "t_s = command value"
[inputs]
0.0  = throttle 1.0
12.0 = elevator 0.3
; optional
[autopilot]
engage = hdg alt
hdg_bug_mag_deg = 270
alt_bug_ft = 4000
```
INI files have **no inline comments**: a comment is a whole line starting with `;` or `#`
(DATA §4.1). Values are verbatim UTF-8.

## 10. Error handling and logging

- Exceptions are allowed at load time (parsing, JSBSim). Catch them at module boundaries and degrade
  gracefully: fall back, log, and show a message in the UI. Nothing throws per frame.
- Log to stderr and `conf/log.txt` (the PortMaster script also tees stdout to `log.txt`).
- Assert invariants with `FSIM_ASSERT`, which is active in debug builds and logs-and-continues in release.
