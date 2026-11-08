# Navigation: flight plans, approaches, navigator, autopilot

Owners: N2 (nav DB), N3 (flight plans), N4 (CIFP procedures), N5 (navigator and approach builder),
N6 (autopilot), W5 (lights and PAPI), H3/H4/H5 (display and UI).
All geometry is **true** internally. Magnetic is used only for display and user input (CONVENTIONS §2).

## 1. Concepts
- **Nav database** (N2): airports, runways (per landing direction), navaids, ILS, fixes, magnetic variation.
- **Flight plan** (N3): an ordered list of `Leg`s (ARCHITECTURE §5) from origin runway to
  destination runway, optionally ending in an approach and missed approach.
- **Navigator** (N5): takes the aircraft state, plan and selected nav source, and produces `NavState`
  (what to fly, deviations, distances). It owns waypoint sequencing.
- **Autopilot and flight director** (N6): turns `NavState` and the AP bugs into control commands
  (AP) or guidance cues (FD).

## 2. Flight plans

### 2.1 Files: X-Plane 11 `.fms` (D10)
- Location: `data/plans/*.fms` (plus samples in `assets/plans/`). Import and export.
- Format (plain text, verified against X-Plane's documentation; details and our conventions are in
  `src/nav/README_FMS.md`):
  ```
  I
  1100 Version
  CYCLE 2610
  ADEP KSFO
  DEPRWY RW28R
  ADES KLAX
  DESRWY RW24R
  APP I24R
  NUMENR 5
  1 KSFO ADEP 13.000000 37.618999 -122.375000
  3 SFO DRCT 0.000000 37.619500 -122.373900
  11 PORTE DRCT 0.000000 37.…    -122.…
  28 +36.000_-121.000 DRCT 9000.000000 36.000000 -121.000000
  1 KLAX ADES 125.000000 33.942501 -118.407997
  ```
  Entry types: `1` airport, `2` NDB, `3` VOR, `11` fix, `28` lat/lon. Field 3 (via) = `ADEP`/`ADES`/
  `DRCT`/airway name (airways are flattened to their fixes in v1). Field 4 = altitude ft (0 = none).
  Unknown keys are ignored with a log line. Coordinates in the file win over the DB if the ident is
  ambiguous or missing.
- SimBrief, Little Navmap and similar tools export this format.
- **Conventions (N3):** `depRunway`/`arrRunway` use DB idents (`28L`) and the file uses `RW28L`.
  `APP` uses CIFP-style idents without `RW` (`I05` = ILS 05, `R28L` = RNAV, `V10` = VOR). The
  file carries only the approach *name*; legs come from N4/N5 at load time. `FlightPlan::approachStartLeg`
  marks the first approach leg. Plans are built on a PC and copied to the SD card.

### 2.2 In-game planning (H5 UI, N3 logic)
- **Quick plan:** origin airport and runway (current position by default) → destination airport,
  cruise altitude (default from the profile), approach selection (§3.1). The route is direct (great
  circle), with an optional "via navaids" mode that inserts VORs within 15 NM of the great circle
  every ≤ 120 NM.
- **Edit:** insert a waypoint (ident search with the on-screen keyboard; ambiguous idents are listed
  with distance), delete, set an altitude constraint, **direct-to** any plan waypoint or any ident.
- **Approach selection:** list of available approaches for the destination runway: real (CIFP, US),
  then synthesized ILS / RNAV / visual. Choosing one replaces the plan tail after the last enroute
  waypoint with transition + final + missed legs. **N8:** then its transition: AUTO (the IAF
  nearest the route, as before), VECTORS (straight to the final segment) or a named transition,
  in setup and in flight (pause menu, FLIGHT PLAN, CHANGE APPROACH). The choice is kept in
  `FlightPlan::approachTransition`, the .fms `APPTRANS` line and the scenario's
  `approach_transition` key.
- **SIDs and STARs (N8, US CIFP):** DEPARTURE (SID) and ARRIVAL (STAR) rows on the setup FLIGHT PLAN
  screen and the in-flight one list the procedures that serve the departure / arrival runway,
  then their enroute transitions. A SID goes right after the origin (runway transition + common
  route + enroute transition), a STAR before the approach (enroute transition + common + runway
  transition). A plan waypoint that repeats the SID's last or the STAR's first fix is replaced by
  the procedure's leg. The legs are tagged (`Leg::source`), so a new choice replaces the old one
  and removing it restores the plan. .fms files carry them by name (`SID`, `SIDTRANS`, `STAR`,
  `STARTRANS`); procedure legs are never written as waypoints.
- **Plan summary:** total distance, ETE at profile cruise speed, and a list with DTK, distance,
  altitude constraint.

### 2.3 Sequencing (N5)
- Legs are flown as great-circle segments (TF) unless the type says otherwise.
- **Fly-by** (default): sequence when along-track distance to the waypoint ≤ turn anticipation
  `d = R · tan(Δψ/2)` plus half a roll-in (2.5 s at the ground speed), with
  `R = V² / (g · tan(bank))`, V = ground speed, bank = min(25°, the AP's bank limit − 5°): 15°
  for the light aircraft (limit 20°), 20° for the jets (limit 25°), Δψ = course change.
- **Fly-by arcs (N10):** between a TF, IF or DF leg and a TF leg, with |Δψ| ≤ 135°, the turn is
  an arc tangent to both legs: from T1 (d before the fix) to T2 (d after it), radius R, shrunk
  when it does not fit in half of the shorter leg. From the sequence to T2 the guidance is
  relative to the arc: DTK is its tangent, XTK the distance from it (+ = right), and NavState
  carries `turnActive`, the feed-forward `turnBankDeg` and `legTrackTrueDeg` (the leg's own
  course, which the nav box shows). The turn ends past T2 or more than 1 NM off the arc. Turns
  over 135°, fly-over fixes and course or hold legs keep the plain sequence and intercept (Δψ
  capped at 120° for the anticipation). The map page, the mini nav and the route overlay draw
  the arcs with `NavState::turnRadiusNm`.
- **Fly-over:** sequence when abeam (along-track ≤ 0).
- Missed legs are not sequenced into until the MAP is passed airborne (§3.4).
- Direct-to creates a DF leg from present position.

## 3. Approaches

### 3.1 Available approaches per runway
| Kind | Source | Lateral | Vertical | Minimums |
|---|---|---|---|---|
| ILS | Real ILS data (US NASR) or **synthesized** for any hard, non-`approx` runway ≥ 3,000 ft | LOC (angular) | GS (angular) | DA = threshold + 200 ft |
| RNAV (GPS) | CIFP (US), or synthesized for any runway | GPS (linear, APR scaling) | Advisory 3° VNAV (hollow diamond) | Synthesized: MDA = threshold + 400 ft (rounded up to 10 ft). CIFP: from the procedure if present, otherwise as synthesized. |
| VOR / NDB | CIFP (US) | Course from the procedure (GPS overlay in v1) | none | From the procedure or threshold + 500 ft |
| Visual | synthesized | Extended centerline | 3° advisory | none |

### 3.2 Synthesized ILS geometry (N2 builds `Ils`, N5 computes deviations)
- **LOC antenna:** on the extended centerline, 1,000 ft beyond the far end of the runway. Course =
  runway true heading.
- **LOC half-width:** chosen so full-scale deflection (2 dots) = ±350 ft at the threshold, clamped
  to ≤ 3.0° (total width 6°) and ≥ 1.5°.
- **GS antenna:** abeam the glidepath intercept point, `TCH / tan(gsAngle)` past the threshold
  (TCH = 50 ft, gsAngle = 3.0° → ~954 ft), on the centerline for simplicity, elevation = threshold elevation.
- **GS full scale:** ±0.7° (1 dot = 0.35°).
- **Validity:** LOC within 18 NM and ±35° of the inbound course and in front of the antenna. GS within
  10 NM and ±8° of course, and above the antenna.
- **DME:** distance to the threshold.
- **Representation (N2):** `Ils::locWidthDeg` is the **total** course width (twice the half-width
  above). Synthesized ILS have ident `I<rwy>` and `freqMhz = 0`, meaning untuned. The navigator treats them as
  available without tuning.
- **Signs (CONVENTIONS §3):** `cdiDeflection = −angularOffset / halfWidth` (aircraft right of course →
  negative → "fly left"). `gsDeflection = (gsAngle − elevationAngleFromGsAntenna) / 0.7°` (aircraft
  low → positive → "fly up"). Clamp the display at ±1.25 (pegged).

### 3.3 Synthesized RNAV / visual approach legs (N5 `ApproachBuilder`)
- `IF` on the extended centerline at 10 NM, altitude = threshold + 3,200 ft (rounded up to 100).
- `FAF` at 5 NM, altitude = threshold + 1,600 ft. `MAP` = runway threshold fixed `RWxx` at threshold
  + 50 ft (TCH), fly-over.
- **Entry:** if the course from the previous waypoint to the IF differs from the inbound course by
  more than 90°, insert an `IAF` 4 NM abeam the IF on the arrival side, at the IF altitude (base leg).
- **Missed:** CA on runway heading to threshold + 1,500 ft → DF to IF → HM (hold) at IF, inbound on
  the final course, right turns, 1-minute legs.
- **Visual:** like RNAV, but FAF at 3 NM / threshold + 1,000 ft, no IF entry logic and no missed hold.

### 3.4 Missed approach
- When the MAP is passed airborne (AGL > 50 ft, or climbing), the navigator sequences into the missed
  legs, `NavState.phase = Missed`, and the HUD shows `MISSED APPR`. The AP lateral NAV follows if
  engaged. Vertical stays pilot or AP mode.
- Re-entry: direct-to IF (L1+Select) to fly the approach again.

### 3.5 Real procedures (N4, CIFP, US only)
- Parsed offline into `procedures/{ICAO}.proc` (DATA §7). Runtime legs follow `fsim::LegType`.
- Supported natively: IF, TF, CF, DF, CA, VA, FA, RF, HM/HA/HF. Others (AF, PI, VD, VI, VM, CI, CD,
  FC, FD, FM) are approximated by the nearest supported type, with a log line and a counter in the
  N4 coverage report.
- Altitude constraints: `At`, `AtOrAbove`, `AtOrBelow`, `Between`. Speed limits are shown and not enforced.

### 3.6 Airport lighting and visual aids (W5 renders, data from N2)
- **PAPI** (every hard runway ≥ 3,000 ft, left side): 4 units, first unit 50 ft from the runway edge,
  30 ft spacing, located `TCH / tan(3°)` past the threshold (~954 ft). Transition angles from the
  unit **nearest the runway** outward: 3.50°, 3.17°, 2.83°, 2.50°. A unit is **white** if the eye's
  elevation angle above it exceeds its transition angle, otherwise **red**. On path = 2 white
  (outer) + 2 red (inner).
- **Approach lights:** ILS runways get a simplified MALSR: light bars every 200 ft out to 2,400 ft,
  5 white lights per bar, sequenced flashers on the outer 1,000 ft (2 Hz). Non-ILS lighted runways get
  no approach lights.
- **Runway lights:** edge white every 200 ft (instrument runways: yellow on the last 2,000 ft),
  threshold green, end red, optional centerline.
- **Markings** (FAA AC 150/5340-1 as a simplified reference): threshold stripes (count by width),
  designator number + L/C/R, centerline stripes 120 ft on / 80 ft off, touchdown zone bars every
  500 ft to 3,000 ft, aiming point bars at 1,000 ft. Visual (non-instrument) runways get designator,
  centerline and threshold only.

## 4. Navigator (N5)
- **Inputs:** `AircraftState`, `NavDatabase`, `FlightPlan`, selected source (auto/GPS/LOC/VOR), dt.
- **Output:** `NavState` (ARCHITECTURE §5), updated every frame.
- **Nav sources:**
  - `Gps`: plan-based, linear deviations. CDI full scale: ENR 2.0 NM, TERM 1.0 NM (within 30 NM of
    origin or destination), APR 0.3 NM (from 2 NM before the FAF). Scale changes ramp linearly over 1 minute.
  - `Loc`: angular per §3.2, active when an ILS/LOC approach is loaded and the LOC is valid. **Auto
    switch GPS → LOC** when on the final approach course inbound and the LOC is valid (intercept
    angle < 45°), as GPS navigators do on ILS approaches.
  - `Vor` (v1: optional): direct to a VOR along a course with angular deviation (±10° full scale), radial
    and DME. Plan waypoints that are VORs are still flown as GPS.
- **Phases:** Departure → Enroute → Terminal → Approach → Missed, as in §2.3 and §3.4.
- **Advisory VNAV:** for the next altitude constraint (or destination threshold + 50 ft), compute
  the 3° path. `targetAltFt`, `requiredVsFpm = −gsKt × 101.27 × tan(3°)` (fpm), TOD distance in the
  nav box, hollow VNAV diamond on the GS scale (scale ±250 ft).
- **Minimums:** from the active approach (§3.1). `distToThresholdNm` is used for RA/minimums callouts (X2).

### 4.1 Implementation notes (N5: `src/nav/Navigator*`, `src/nav/ApproachBuilder*`)
- **API:** `Navigator(db)`; `setPlan`, `loadApproach/loadApproachById`, `startApproachOnly`,
  `directToLeg/directToIdent`, `reenterApproach` (direct-to IF), `setSourceSelect(Auto|Gps|Loc|Vor)`,
  `tuneVor`, `update(AircraftState, dtS) -> NavState`. `availableApproaches(db, airport, runway,
  realProvider)` lists real approaches first (N4 plugs in via the provider), then `I<rwy>` (or `L<rwy>`
  without a glideslope), `R<rwy>`, `VIS<rwy>`; a synthesized id that a real approach already has is dropped.
- **Clarifications:** the CDI/GS display clamp (+-1.25) applies to GPS and LOC. Auto GPS -> LOC
  capture additionally requires the needle to be inside full scale (|cdi| <= 1) and the active leg to be on
  the final course (DTK within 30 deg of the MAP leg's); LOC then latches until it is invalid, a missed
  approach starts, or the source is forced. GS is evaluated only with the LOC source; its +-8 deg cone is
  measured from the point of the centerline abeam the GS antenna (real antennas sit hundreds of feet off
  the centerline). Advisory VNAV uses the next At / AtOrBelow / Between constraint (Between: the upper
  altitude) or threshold + 50 ft; `requiredVsFpm` is the path VS once at or past the TOD and 0 before it.
- **Deviation (MAP):** the MAP is the threshold at TCH = 50 ft, so "AGL > 50 ft" would turn every normal
  landing into a missed approach. The missed approach starts when the MAP is passed airborne with
  AGL > 100 ft, or climbing (> 300 fpm), or more than 0.5 NM past it still airborne above 50 ft AGL.
- **Holds** (`HM`) are flown as direct-to the fix, then the inbound course; the racetrack itself is not
  generated in v1. `RF` legs are flown as `TF` to the arc end. Mid-plan `IF` legs are flown as `TF`.
- **Scenario keys:** `plan = <file.fms>` and `approach = <id>` in `[scenario]` (a lone `approach` makes a
  one-approach plan at the scenario airport). In-air starts activate the leg nearest the aircraft.

## 5. Autopilot, flight director, autothrottle (N6, D11)
Generic cascaded controllers driven by `AircraftState`, `NavState` and `AutopilotState` bugs. They
produce the final `ControlInputs` (AP engaged) or FD cues only (AP off, FD on). Gains and limits live
in each profile's `[autopilot]` section, with sane category defaults (piston / turboprop / jet).

| Layer | Modes | Law (sketch) |
|---|---|---|
| Lateral outer | ROL (hold bank, ≤ 6° → wings level), HDG, NAV (GPS), LOC | HDG: bankCmd = k·headingError, clamp ±25° (jet) / ±20° (light). NAV/LOC: intercept guidance = desired track + correction(xtk or angular deviation), max 45° intercept; LOC gains scale with distance to avoid S-turns near the threshold. |
| Lateral inner | n/a | Roll attitude PD → aileron. Turn coordination: rudder from yaw rate / lateral acceleration (or the auto-rudder assist). |
| Vertical outer | PIT, ALT, ALT* (capture), VS, GS, FLC, GA (N7) | ALT capture starts when \|altErr\| < \|VS\|/6 (min 100 ft) and blends to hold. VS: pitch to hit the target VS. GS: pitch to track the glidepath (deviation → target VS = path VS + k·dev). |
| Vertical inner | n/a | Pitch attitude PI → elevator. **AP-managed pitch trim** offloads sustained elevator into `elevTrimNorm` (rate-limited) so disconnects don't jolt. |
| Speed | SPD (A/T), RETARD, THR (N7) | PI on IAS → throttle (all engines), rate-limited. RETARD to idle below 30 ft RA on landing. A/T disconnects at touchdown. THR holds a fixed throttle: TOGA in GA, climb or idle in FLC. |

- **Engage rules:** AP engage with no mode selected = ROL + PIT (hold current). NAV arms if the plan
  has an active leg. APR arms LOC + GS. Captures happen automatically. A mode button (HDG, NAV, APR,
  ALT, VS, FLC) pressed with the AP and the FD both off turns the FD on first (1.2.0), like TO/GA.
- **Disconnects:** pilot stick deflection > 0.5 for > 0.5 s, AP toggle, stall warning, bank > 45° or
  pitch beyond ±25°, and below minimums − 50 ft (no autoland in v1). Each disconnect sets
  `disconnectWarning`.
- **FD:** the same outer loops, outputting `fdPitchCmdDeg` / `fdRollCmdDeg` without driving the surfaces.
- **Acceptance** is in WORK_PACKAGES N6.

### 5.1 Implementation notes (N6: `src/autopilot/`, details in its README)
- **Entry points:** `Autopilot::update` (120 Hz, via `Simulation`), `handleActions` (per frame, `App.cpp`),
  `syncPilot` (levers and trim follow the A/T and the AP). Scenario `[autopilot]`: `engage`, `hdg_bug_mag_deg`,
  `alt_bug_ft`, `vs_bug_fpm`, `spd_bug_kt`.
- **Lateral law:** NAV and LOC steer the **ground track** to `DTK - atan(xtk / (gs * tau))` (look-ahead
  guidance, intercept <= 45 deg NAV / 30 deg LOC), so crosswind needs no integrator. LOC uses the linear XTK,
  which scales the angular beam by distance (no S-turns near the threshold). HDG and the track errors become a
  turn rate (`err / tau`) and then the coordinated bank for that rate, clamped to 20 / 25 deg.
- **Capture:** NAV / LOC capture when closing and `|xtk| <= R (1 - cos(rel)) + 0.1 NM` (R from the bank
  limit); LOC also needs `|cdi| <= 1`. NAV armed over ROL captures at once (1.2.3): ROL has no
  intercept heading, so the NAV law flies its own intercept; over HDG it waits for the closing
  intercept. NAV never captures with weight on wheels. GS needs LOC active and `|gsDeflection| <= 0.5` (from below or within one
  dot). ALT* starts at `max(100 ft, |VS| / 6)` toward the bug and ALT holds within 25 ft.
- **A/T availability:** `hasAutothrottle` profiles, plus any profile with `[autopilot] at_available = 1`
  (c172p and c310 use it for speed hold; deviation from "jets only").
- **Disconnect warning:** cleared by the next AP toggle press (which does not re-engage) or after 5 s.
- **FD cues:** `fdRollCmdDeg` = commanded bank, `fdPitchCmdDeg` = commanded pitch attitude (absolute, deg).
- **FLC and TO/GA (N7):** both fly **pitch on speed** through the VS loop: VS target = VS +
  5.311 · TAS · (TAS / IAS) · (IAS rate + (IAS − target) / `spd_tau_s`), the energy balance
  ḣ + (V / g) V̇ turned into fpm (TAS and IAS in kt, rates in kt/s). FLC targets the speed bug (synced to the
  IAS when unset) and never climbs with the altitude bug below or descends with it above; with the A/T on it
  holds `climb_throttle` (THR CLB) or idle (THR IDLE). Selecting FLC within 100 ft of the bug holds the bug
  instead. TO/GA (airborne only) targets the IAS at the press, at least Vref with flaps out (1.25 Vs1 clean),
  caps the attitude at `ga_pitch_deg` (piston 10, turboprop 12, jet 15), never descends while within 5 kt of
  the target, sets the A/T to TOGA, holds the present heading in HDG with NAV armed on the plan, clears the
  LOC / GS arms and turns the FD on when neither AP nor FD was. The altitude bug goes 1,000 ft up unless it is
  already 500 ft or more above. The minimums disconnect does not apply in GA. FLC and GA end at ALT* (the A/T
  returns to SPD without a jump) or when another vertical mode is selected. The navigator sequences into the
  missed approach when the MAP is passed airborne; NAV then flies it. FMA: `TOGA | GA`, `THR CLB | FLC`,
  `THR IDLE | FLC`.
- `DirectTo` and `NavSourceCycle` are wired in the app (I2). Not implemented: VOR-source specifics (NAV flies
  whatever the navigator reports).

