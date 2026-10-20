# sim/: JSBSim flight model (WP F1)

`FlightModel` is the only code that talks to JSBSim (v1.3.3). It turns a `StartCondition` plus an
`AircraftProfile` into a running model, converts our `ControlInputs` into JSBSim properties, and
publishes an `AircraftState` snapshot after every step. All property names below were checked
against the v1.3.3 sources (`FGFCS.cpp`, `FGPropagate.cpp`, `FGAuxiliary.cpp`, `FGWinds.cpp`,
`FGLGear.cpp`, `FGThruster.cpp`, `FGTurbine.cpp`, `FGPiston.cpp`, `FGPropeller.cpp`) and against
the stock `c172p` and `737` models.

Files: `FlightModel.*` (wrapper), `CrashRules.*` (pure crash verdicts, unit tested),
`ProfileLoader.*` (INI to `AircraftProfile`).

## Sign conversions (only here, nothing else touches JSBSim)

| Ours | JSBSim | Conversion |
|---|---|---|
| elevator +1 = nose up | `fcs/elevator-cmd-norm` + = nose down | negate |
| rudder +1 = yaw right | `fcs/rudder-cmd-norm` + = yaw left | negate |
| aileron +1 = roll right | same sign | none |
| elevator / rudder trim | `fcs/pitch-trim-cmd-norm`, `fcs/yaw-trim-cmd-norm` | negate |
| aileron trim | `fcs/roll-trim-cmd-norm` | none |
| steering +1 = right | `fcs/steer-cmd-norm` + = right | none (scaled by the speed fade below) |

## Property map

Inputs (written every step; cached nodes, no string work). `{n}` = engine index. The last column
is the `[properties]` key that overrides the path for a model that differs.

| Input | Property | Override key |
|---|---|---|
| elevator, aileron, rudder | `fcs/elevator-cmd-norm`, `fcs/aileron-cmd-norm`, `fcs/rudder-cmd-norm` | none |
| trims | `fcs/pitch-trim-cmd-norm`, `fcs/roll-trim-cmd-norm`, `fcs/yaw-trim-cmd-norm` | `pitch_trim`, `roll_trim`, `yaw_trim` |
| throttle | `fcs/throttle-cmd-norm[{n}]` | `throttle_cmd` |
| mixture | `fcs/mixture-cmd-norm[{n}]` | `mixture_cmd` |
| flaps | `fcs/flap-cmd-norm` (value = detent `cmdNorm`) | `flap_cmd` |
| gear | `gear/gear-cmd-norm` (forced down while any wheel has weight, and on fixed-gear aircraft) | `gear_cmd` |
| speedbrake | `fcs/speedbrake-cmd-norm` (0 when the profile has no speedbrake) | `speedbrake_cmd` |
| ground spoilers | not written unless `spoiler_cmd` is set; then 1 on the ground with speedbrake > 0.5 or reverse deployed, else 0 | `spoiler_cmd` |
| reversers | `propulsion/engine[{n}]/reverser-angle-rad` (only when `reversers = true`) | `reverser_cmd`, `reverser_on`, `reverser_off`, `reverser_travel_s` |
| brakes | `fcs/left-brake-cmd-norm`, `fcs/right-brake-cmd-norm`; `fcs/center-brake-cmd-norm` = max(parking, mean of left and right) | `brake_left`, `brake_right`, `brake_center` |
| parking brake | no JSBSim property: both brakes (and center) at 1 | none |
| steering | `fcs/steer-cmd-norm` | `steer_cmd` |

Outputs (read every step):

| `AircraftState` field | Property or source |
|---|---|
| `latDeg` | `position/lat-geod-deg` (geodetic) |
| `lonDeg` | `position/long-gc-deg` |
| `altMslFt` | `position/h-sl-ft` (D16: treated as MSL) |
| `altAglFt` | `position/h-agl-ft` (CG height above the ground callback surface) |
| `terrainElevFt` | `position/terrain-elevation-asl-ft` |
| roll, pitch, heading | `attitude/phi-rad`, `theta-rad`, `psi-rad` (heading wrapped to 0..2pi, true) |
| p, q, r | `velocities/p-rad_sec`, `q-rad_sec`, `r-rad_sec` |
| `iasKt` | `velocities/vc-kts` (calibrated) |
| `tasKt`, `mach` | `velocities/vtrue-kts`, `velocities/mach` |
| `alphaDeg`, `betaDeg` | `aero/alpha-deg`, `aero/beta-deg` |
| `vsFpm` | `velocities/h-dot-fps` x 60 |
| `nzG` | `accelerations/Nz` |
| `gsKt` | `velocities/vg-fps` |
| `velNorth/East/DownMps` | `velocities/v-north-fps`, `v-east-fps`, `v-down-fps` |
| `trackTrueRad` | atan2(east, north) of the velocity, heading below 1 kt ground speed |
| wind | `atmosphere/wind-north-fps` + `gust-north-fps`, same for east; `windFromTrueDeg` is the opposite direction (JSBSim wind vectors point where the air goes) |
| `flapPosNorm` | `fcs/flap-pos-norm`; `flapDetent` = last commanded detent |
| `gearPosNorm` | `gear/gear-pos-norm` |
| `wowAny`, `wowAll` | `FGLGear::GetWOW()` over **wheel** contacts only (below) |
| `speedbrakePosNorm` | `fcs/speedbrake-pos-norm` |
| trims | the three trim command properties, signs converted back |
| brakes | `fcs/left-brake-cmd-norm`, `fcs/right-brake-cmd-norm` |
| `engines[i].running` | `propulsion/engine[{n}]/set-running` (reads the running state) |
| `engines[i].n1Pct`, `n2Pct` | `propulsion/engine[{n}]/n1`, `/n2` (turbine and turboprop engines only, else 0) |
| `engines[i].rpm` | `propulsion/engine[{n}]/engine-rpm`, else `/propeller-rpm`; always 0 for `category = jet` |
| `engines[i].thrustLbf` | `propulsion/engine[{n}]/thrust-lbs` (negative with reverse) |
| `engines[i].reverserDeployed` | slewed reverser position > 0.5 |
| `fuelTotalLbs` | `propulsion/total-fuel-lbs` |
| `eyeBodyM` | model `EYEPOINT` through `FGMassBalance::StructuralToBody`, else profile `[view] eye_offset_m` |

Engine and gear counts: `engineCount` is `min(profile engines, model engines, kMaxEngines)`.

## Per-model differences found

- **c172p**: single piston, fixed gear, no speedbrake. Contacts: 3 wheels plus structure points
  `NOSE_SKID` (prop hub), `TAIL_SKID`, `LEFT_TIP`, `RIGHT_TIP`. The tail skid touches at about 10
  degrees of pitch on the gear, so rotating hard at 55 kt with full back stick is a tail strike.
  No center brake group. The model needs only a little rudder on the takeoff roll.
- **737**: two `CFM56` turbines with `direct` thrusters, **only three wheel contacts and no
  structure points**: belly, engine and tail contact do not exist in the model, so the gear-up and
  attitude crash rules below are ours. `fcs/flap-cmd-norm` is 0..1 over 0..40 degrees (kinematic
  settings in eighths). Gear takes 5 s, flaps up to 25 s end to end. Has `fcs/speedbrake-*` (flight
  spoilers) and `fcs/spoiler-*` (ground spoilers). `reverser-angle-rad` exists because the thruster
  is `direct`: thrust is multiplied by cos(angle), so pi gives full reverse. The model has a yaw
  damper but no autopilot. At load it binds UDP/TCP input sockets (5137 and 5139) and logs an ERROR;
  F2 removes this through `assets/jsbsim-overlay/`.
- **Other stock models** (read, not flown): `pa28`, `c182` and `A320` declare wing tips, tail and
  nose as `BOGEY` contacts; list those names in `structure_contacts` so they count as structure
  rather than wheels. `c182` and `pa28` main wheels are castering (`max_steer 360`): steer with
  differential brakes. `global5000`, `B747`, `A320` use `direct` thrusters, so reversers work as for
  the 737. `c310` opens output sockets 1138 and 1139 (F2).
- Turboprops reverse through `propulsion/engine[{n}]/reverser` (a bool), not an angle: set
  `reverser_cmd = propulsion/engine[{n}]/reverser`, `reverser_on = 1`, `reverser_off = 0`,
  `reverser_travel_s = 0`.

## Profile INI format (`assets/aircraft/<id>.ini`)

Comments are whole lines starting with `;` or `#`. There are no inline comments (DATA 4.1).

```ini
[aircraft]
id = c172p                 ; file name and key used by --aircraft and scenarios
display_name = Cessna 172P Skyhawk
jsbsim_model = c172p       ; directory name under jsbsim/aircraft (defaults to id)
category = piston          ; piston | turboprop | jet (picks crash limits, rpm vs n1 reporting)
engines = 1                ; 1..4
retractable_gear = false
speedbrake = false         ; false = the speedbrake input is ignored
reversers = false          ; true = reverser logic and the reverser_* keys apply
autothrottle = false       ; for N6, not used by sim/

[flaps]
detents = UP:0.0, 10:0.3333   ; label:fcs/flap-cmd-norm, in lever order, values 0..1
vfe_kt = 0, 110               ; optional, per detent, 0 = use [speeds_kt] vfe
takeoff_detent = 0            ; indices into detents
approach_detent = 2

[speeds_kt]                ; IAS, all optional (0 = unknown)
vs0, vs1, vr, vref, vapp, vfe, vno, vne, vmo, mmo   ; mmo is a Mach number

[cruise]
ias_kt = 105
altitude_ft = 5500

[warnings]
stall_warn_alpha_deg = 11  ; stall warning when alpha exceeds this; clears 1 degree lower

[view]
eye_offset_m = 0.13, 0.0, -0.26   ; x fwd, y right, z down from the CG; only if the model has no EYEPOINT

[autopilot]                ; free-form key = number map for N6

[properties]               ; JSBSim path overrides and F1 options (table above), plus:
structure_contacts = LEFT_TIP, RIGHT_TIP   ; BOGEY contacts to treat as structure (crash on touch)
spoiler_cmd = fcs/spoiler-cmd-norm

[crash]                    ; optional overrides of the category defaults
hard_landing_fpm = 900            ; sink rate at first wheel contact
touchdown_bank_deg = 25           ; |bank| limit during the touchdown window
touchdown_pitch_up_deg = 20       ; tail strike
touchdown_pitch_down_deg = 15     ; nose strike
touchdown_window_s = 1.0          ; how long after first contact the attitude limits apply
belly_agl_ft = 3.3                ; gear-up: CG height at which the belly touches (default 0.7 x gear height)

[ground]
steer_full_below_kt = 15          ; full nose-wheel authority below this ground speed
steer_min_above_kt = 60           ; reduced to steer_min_frac at and above this
steer_min_frac = 0.15
```

`[crash]` and `[ground]` values are stored in `AircraftProfile::properties` as `crash.<key>` and
`ground.<key>` (keeps `core/` unchanged). The `;` annotations above are for this document only:
real files must not put comments after values.

Category defaults (`CrashLimits`): piston 900 fpm / 25 / 20 / 15, turboprop 1000 / 18 / 14 / 10,
jet 1100 / 12 / 12 / 8 (hard landing fpm, bank, pitch up, pitch down; degrees).

## Start conditions

- **Runway**: on the ground at the given point, parking brake on, engines running, `DoTrim(tGround)`
  then 2 s of settling. IAS 0.
- **Final / Cruise**: flaps and gear are driven to their commanded positions with the aircraft
  held (`SetHoldDown`), then `DoTrim(tFull)` (falls back to `tLongitudinal`). The trimmed elevator
  is already in `fcs/pitch-trim-cmd-norm`; trimmed aileron and rudder are moved into the roll and
  yaw trim, and the stick starts centered. `startControls()` returns trimmed throttle, trims,
  flaps, gear. A failed trim restores the controls and starts untrimmed (logged), never NaN.
- **Wind** is applied after the trim, never through the initial condition. JSBSim's IC wind keeps the
  ground velocity and shifts the airspeed, which leaves a crosswind start with a sideslip that
  cannot be trimmed. Instead the model is trimmed in calm air at the requested IAS, then
  `FGWinds::SetWindNED` is set and the body velocity is shifted by the same vector, so IAS, alpha,
  beta and trim are unchanged and only ground speed and track change. This **supersedes** the older
  "apply the IC wind after the speed and re-trim" recipe. For a Final start the IC heading is the
  runway heading plus the wind-correction angle, so the ground track stays on the centerline.
  (The first snapshot after `start()` still shows the pre-wind ground speed; it updates on step 1.)

## Crash rules (`CrashRules.*`, evaluated in `FlightModel::refresh`)

Nothing is judged in the first second after a start. The first matching rule sets `crashed` and
`crashReason` (at most 47 characters) and the model freezes until the next `start()`.

1. **Structure contact**: any non-wheel contact point with weight. Reason from its name: `tip` is
   "wingtip strike", `tail` "tail strike", `nose` "nose strike", `eng`/`pod` "engine strike", `wing`
   "wing strike", `roof`/`cabin`/`belly` "fuselage strike", else "structure strike".
2. **Gear-up landing** ("gear-up landing"): retractable gear not down and locked (position < 0.99)
   with either a structure contact or the CG below the belly height (models without belly points,
   like the 737).
3. **Water** ("water impact"): any contact while `TerrainQuery::landClass` is `Water`.
4. **Touchdown** (rising edge of weight on wheels): "hard landing" when the vertical speed in the last
   step is below `-hard_landing_fpm`; "terrain impact" when the speed relative to the terrain
   slope ahead (60 m along the track) is; then the attitude check.
5. **Attitude during the touchdown window**: |bank| above the limit "wing strike", pitch above the
   limit "tail strike", below minus the limit "nose strike".

Terrain plug-in: call `FlightModel::setTerrain(const TerrainQuery*)`. The slope and water rules
read it each time they are needed (not every step), and F3 installs the JSBSim ground callback
for the real surface (next section).

## Ground contact (WP F3: `GroundCallback.*`)

`setTerrain()` (before or after `start()`) installs `TerrainGroundCallback`, a `JSBSim::FGGroundCallback`
over the `TerrainQuery`; `start()` installs it before the initial condition. With no terrain the
default JSBSim ground stays: flat at `StartCondition::fieldElevFt`. With a terrain set, **the
terrain is the ground**: `fieldElevFt` is only the Runway start's height reference, so the caller
must put it on the terrain (`Simulation::init` does; tests use `placeStartOnTerrain`).

- AGL = vertical (geodetic) height above `elevationM(lat, lon)`; D16: meters MSL = geodetic feet.
- Contact point = the ground point below the query; normal = ECEF up tilted by the surface
  gradient (central differences, +-20 m, slope limited to 70 degrees); surface velocity zero.
  Runway flatten areas are inside `elevationM`, so a pad is exactly flat with an up normal.
- Cost: five elevation lookups per distinct location, a 4-entry memo per sim step answers the
  repeated CG queries. 0.2 us per call on a pack (desktop), +1 us per flight step.
- The ground does not wait for tiles: until a finer tile is resident `elevationM` answers from the
  coarser one and the ground can step by the difference. Starts avoid this with the spawn gate
  (`app/SpawnGate.*`): `Simulation::init` prefetches around the start (and the landing runway),
  pumps `TerrainService::update` through `setTerrainPump` and waits up to 10 s for
  `isDetailReady`, then starts anyway with a warning. It also registers the start runway's pad
  (`runwayFlattenArea`, `// F3: temporary until W5`) so a runway start sits on the runway surface.
- Water rule: any weight on any contact point while `landClass` is `Water` ("water impact");
  closure rule: touchdown vs or the speed against the slope 60 m ahead above the limit
  ("terrain impact"); contact by a non-wheel point is a structure strike as before.

## Inputs with rules

- **Gear**: gear-up is ignored while any wheel has weight; fixed gear is always down.
- **Reversers**: the request (`reverseThrust`) latches only with weight on wheels and all throttles
  at 0.05 or below. It releases when the request drops or the wheels leave the ground. The angle
  slews over `reverser_travel_s` (1.5 s). Reverse power then follows the throttle.
- **Steering fade**: the nose-wheel command is scaled by 1 below `steer_full_below_kt`, falling
  linearly to `steer_min_frac` at `steer_min_above_kt`.
- **Warnings**: stall warning is off with weight on wheels, on above `stall_warn_alpha_deg`, off
  again 1 degree lower. Overspeed is IAS above Vmo (else Vne) or Mach above Mmo.

## Restarts

`start()` destroys the previous `FGFDMExec` first; `restart()` repeats the last start. The test
`sim: 100 restarts do not grow resident memory` reads `/proc/self/statm` and allows 8 MiB.

## Measured model facts (for F2 and the HUD)

At about 107,000 lb (737) and 2,350 lb (c172p), idle power, 1 g, slow deceleration:

| Aircraft | Config | Max alpha (deg) | IAS at stall (kt) |
|---|---|---|---|
| c172p | flaps UP | 14.2 | 51 |
| c172p | flaps 10 / 20 / 30 | 12.9 / 12.3 / 12.1 | 48 / 47 / 46 |
| 737 | flaps 30 | 11.7 | 124 |
| 737 | flaps 40 | 11.2 | 117 |
| 737 | flaps UP, flaps 5 | about 20, about 17.5 (dynamic, deceleration too fast to call) | about 150, about 146 |

The c172p profile uses the POH placard numbers requested for the HUD (Vs0 33, Vs1 44) although the
model stalls about 10 kt above them. The 737 cruise start (FL330, 270 KIAS, Mach 0.76) holds
altitude within 60 ft for 60 s hands-off. The 737 lifts off at about 155 KIAS with flaps 5 and 10
degrees of pitch on a 10,000 ft runway.
