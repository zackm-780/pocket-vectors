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
- Format (plain text, verify details against X-Plane's documentation, which N3 records in `src/nav/README.md`):
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
- SimBrief, Little Navmap and similar tools export this format. The user builds plans on a PC and
  copies them to the SD card.

### 2.2 In-game planning (H5 UI, N3 logic)
- **Quick plan:** origin airport and runway (current position by default) → destination airport,
  cruise altitude (default from the profile), approach selection (§3.1). The route is direct (great
  circle), with an optional "via navaids" mode that inserts VORs within 15 NM of the great circle
  every ≤ 120 NM.
- **Edit:** insert a waypoint (ident search with the on-screen keyboard; ambiguous idents are listed
  with distance), delete, set an altitude constraint, **direct-to** any plan waypoint or any ident.
- **Approach selection:** list of available approaches for the destination runway: real (CIFP, US),
  then synthesized ILS / RNAV / visual. Choosing one replaces the plan tail after the last enroute
  waypoint with transition + final + missed legs.
- **Plan summary:** total distance, ETE at profile cruise speed, and a list with DTK, distance,
  altitude constraint.

### 2.3 Sequencing (N5)
- Legs are flown as great-circle segments (TF) unless the type says otherwise.
- **Fly-by** (default): sequence when along-track distance to the waypoint ≤ turn anticipation
  `d = R · tan(Δψ/2)` with `R = V² / (g · tan(bank))`, V = ground speed, bank = 25° (jets) or
  20° (light aircraft), Δψ = course change (capped at 120°).
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

## 5. Autopilot, flight director, autothrottle (N6, D11)
Generic cascaded controllers driven by `AircraftState`, `NavState` and `AutopilotState` bugs. They
produce the final `ControlInputs` (AP engaged) or FD cues only (AP off, FD on). Gains and limits live
in each profile's `[autopilot]` section, with sane category defaults (piston / turboprop / jet).

| Layer | Modes | Law (sketch) |
|---|---|---|
| Lateral outer | ROL (hold bank, ≤ 6° → wings level), HDG, NAV (GPS), LOC | HDG: bankCmd = k·headingError, clamp ±25° (jet) / ±20° (light). NAV/LOC: intercept guidance = desired track + correction(xtk or angular deviation), max 45° intercept; LOC gains scale with distance to avoid S-turns near the threshold. |
| Lateral inner | n/a | Roll attitude PD → aileron. Turn coordination: rudder from yaw rate / lateral acceleration (or the auto-rudder assist). |
| Vertical outer | PIT, ALT, ALT* (capture), VS, GS, FLC (optional) | ALT capture starts when \|altErr\| < \|VS\|/6 (min 100 ft) and blends to hold. VS: pitch to hit the target VS. GS: pitch to track the glidepath (deviation → target VS = path VS + k·dev). |
| Vertical inner | n/a | Pitch attitude PI → elevator. **AP-managed pitch trim** offloads sustained elevator into `elevTrimNorm` (rate-limited) so disconnects don't jolt. |
| Speed | SPD (A/T), RETARD | PI on IAS → throttle (all engines), rate-limited. RETARD to idle below 30 ft RA on landing. A/T disconnects at touchdown. |

- **Engage rules:** AP engage with no mode selected = ROL + PIT (hold current). NAV arms if the plan
  has an active leg. APR arms LOC + GS. Captures happen automatically.
- **Disconnects:** pilot stick deflection > 0.5 for > 0.5 s, AP toggle, stall warning, bank > 45° or
  pitch beyond ±25°, and below minimums − 50 ft (no autoland in v1). Each disconnect sets
  `disconnectWarning`.
- **FD:** the same outer loops, outputting `fdPitchCmdDeg` / `fdRollCmdDeg` without driving the surfaces.
- **Acceptance** is in WORK_PACKAGES N6.
