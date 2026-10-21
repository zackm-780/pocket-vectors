# tests/fleet: fleet flyability suite (WP F2 core fleet, WP F4 extended fleet)

Headless tests that fly each core aircraft (`c172p`, `c310`, `global5000`, `737`, `B747`) through
`FlightModel` with a scripted pilot, on runway 27 of the synthetic airport XTST (10,000 ft, 2,000 ft
elevation, no wind, flat ground). Every test is named `sim: fleet <id> <phase>`, so it runs in
ctest's `sim_tests`:

```bash
cd build
./flightsim_tests --test-case="sim: fleet *"            # whole suite, about 12 s
./flightsim_tests --test-case="sim: fleet 737 *"        # one aircraft
./flightsim_tests --test-case="sim: fleet * landing"    # one phase on all aircraft
```

The core fleet (c172p, c310, global5000, 737, B747) is in `FleetTest.cpp`. The extended fleet (A320,
787-8, MD11, c182, pa28) is in `FleetExtTest.cpp` (same phases and pilot) and its autopilot flights are
in `FleetApTest.cpp`. The phase functions are shared in `FleetPhases.h`; the per-aircraft numbers are in
`FleetSpecs.h` (core) and `FleetSpecsExt.h` (extended). The extended suite takes about 13 s in all, about
2.5 s per aircraft with its autopilot flights.

Each test also prints `FLEET <id> <key> <value>` lines with its measurements (the numbers below).

| Phase | What is flown | Pass criteria |
|---|---|---|
| profile | Profile and model consistency | Engine count, gear type, flap detents (UP first, rising commands, labels, Vfe falling), speed order, Vfe above Vapp, speedbrake really moves |
| takeoff and climb | Full power, rotate at Vr, gear up, flaps up on schedule, climb to 3,000 ft AGL | Liftoff inside the band and within Vr-5..Vr+20, heading within 6 degrees, inside the runway, flaps and gear up, no stall warning, no crash |
| cruise | Trimmed `cruise` start at the profile altitude and speed, hands off 60 s | Altitude within 200 ft, bank under 3 degrees, throttle not at the stops, no stall or overspeed |
| configuration changes | Gear up and down, every flap detent up and back, speedbrake in and out, holding altitude and a Vfe-safe speed | Gear 1..15 s, each flap step under 40 s, retraction under 60 s, no crash, altitude within 800 ft |
| approach | `final` start 8 NM out, pilot holds the 3 degree path and Vapp down to 150 ft | Path within 150 ft (75 ft below 500 ft AGL), IAS within 10 kt of Vapp, no stall warning |
| hands-off final | `final` start 6 NM out, no inputs for 60 s | Path within 150 ft |
| landing | `final` start 2.5 to 3 NM out, flare, rollout with brakes, spoilers and reversers where fitted | No crash, touchdown sink under the limit, within 60 ft of the centerline, stops on the runway within the aircraft's limit |
| stall | Idle deceleration at 1 g from a trimmed start, clean, approach flaps and full flaps | Measured stall speeds match the table, profile Vs0/Vs1 within 30% of the model, Vapp at least 1.15 x the approach stall speed, stall warning alpha below the stall alpha |
| overlay | Load, start and trim every aircraft in runway, cruise and final starts with the log file sink open | No JSBSim ERROR or WARN line, no `global5000.csv` written, overlay files installed |

The pilot (`FleetPilot.h`) is the same code for every aircraft; the per-aircraft numbers (rotation
speed, climb speeds, flare height, brake use) are in `FleetSpecs.h`. The tests have no pass bands
tuned against a specific result other than those above.

## Results

Light weights as shipped by each stock model (c172p 2,350 lb, c310 about 4,400 lb, global5000 about
80,000 lb, 737 about 107,000 lb, B747 about 551,000 lb). Runway 10,000 ft, sea-level-ish (2,000 ft
elevation), ISA, calm. All 41 tests pass.

| | c172p | c310 | global5000 | 737 | B747 |
|---|---|---|---|---|---|
| Takeoff flaps | UP | UP | 16 | 5 | 20 |
| Rotation starts (Vr in profile) | 55 (test 52) | 80 | 125 | 145 | 140 |
| Liftoff speed, kt IAS | 57 | 84 | 131 | 157 | 152 |
| Liftoff distance, ft | 790 | 1,940 | 2,970 | 4,070 | 3,450 |
| Time to 3,000 ft AGL, s | 139 | 183 | 79 | 62 | 63 |
| Measured stall, clean, kt / alpha | 49.5 / 15.6 | 74.6 / 15.5 | 150.8 / 14.3 | 152.3 / 13.8 | 156.4 / 14.3 |
| Measured stall, approach flaps, kt / alpha | 45.3 / 14.6 (20) | 68.0 / 15.3 (15) | 95.8 / 14.0 (30) | 124.2 / 11.8 (30) | 112.9 / 7.7 (30) |
| Measured stall, full flaps, kt / alpha | 44.9 / 14.3 | 59.0 / 15.0 | 95.8 / 14.0 | 118.2 / 11.2 | 112.9 / 7.7 |
| Profile Vs1 / Vs0, kt | 44 / 33 | 75 / 63 | 150 / 96 | 152 / 118 | 156 / 113 |
| Profile stall warning alpha, deg | 11 | 12.5 | 12 | 10.5 | 7.0 |
| Approach flaps / Vapp, kt | 20 / 65 | 15 / 95 | 30 / 130 | 30 / 145 | 30 / 140 |
| Hands-off path error in 60 s, ft | 7 | 17 | 27 | 41 | 35 |
| Piloted path error from 8 NM, ft | 1 | 1 | 1 | 1 | 1 |
| Touchdown sink, fpm | 265 | 321 | 302 | 326 | 501 |
| Touchdown speed, kt IAS | 64 | 93 | 125 | 141 | 136 |
| Roll after touchdown, ft | 876 | 879 | 2,021 | 2,043 | 2,604 |
| Landing distance from the threshold, ft | 1,801 | 1,845 | 2,986 | 3,141 | 3,317 |
| Gear up / down, s | fixed | 5.0 / 4.9 | 5.0 / 4.9 | 5.0 / 4.9 | 5.0 / 4.9 |
| Slowest single flap step, s | 1.9 | 1.9 | 2.5 | 3.8 | 2.3 |
| Flaps full to UP, s | 3.9 | 3.9 | 6.9 | 21.6 | 6.9 |
| Cruise altitude hold over 60 s, ft | 0.4 | 1.4 | 21 | 42 | 29 |
| Cruise point | 105 kt, 5,500 ft | 125 kt, 7,500 ft | 260 kt, FL370 | 270 kt, FL330 | 280 kt, FL350 |
| Cruise throttle | 78% | 78% | 87% | 79% | 65% |
| Stopping devices used | brakes | brakes | brakes, spoilers, reversers | same | same |

Stall angles of attack differ by flap setting for the jets (the 747 falls from 14.3 degrees clean to
7.7 with flaps 30), while the profile carries a single `stall_warn_alpha_deg`.

## Exceptions and notes for the maintainer

- **c310 propeller (overlay):** the stock constant-speed propeller cannot be trimmed in JSBSim 1.3.3
  (blade pitch resets to fine pitch at every initial condition and `fcs/advance-cmd-norm` is never
  set), so a trimmed start lost 600 to 1,000 ft in 60 s. `assets/jsbsim-overlay` swaps in a
  fixed-pitch copy of the propeller (24 degrees). The c310 cruise point was moved to the README's
  economy cruise (125 kt IAS, 7,500 ft) because the fixed pitch cannot hold 150 kt trimmed.
- **c310 sockets:** the c310 output sockets that `src/sim/README.md` lists are inside XML comments in
  the v1.3.3 stock file, so there is nothing to remove. The 737 input sockets were real and are
  removed by the overlay; the global5000 model wrote `global5000.csv` to the working directory on every
  run, which the overlay removes.
- **Stall warning alpha is per profile, not per flap:** the 747's stall alpha falls to 7.7 degrees
  in the landing configuration while liftoff alpha with flaps 20 is 4.5, so 7.0 is the compromise.
  Per-detent values would need an additive field in `FlapDetent` (maintainer decision).
- **B747 and 737 are not realistic against real figures:** the stock models stall well above real
  airplanes at these weights (737 flaps 30 stall 124 kt, B747 flaps 30 stall 113 kt, and the B747
  cannot be trimmed above about 150 kt with flaps 30). Profile speeds follow the model, with the
  published flap placards and Vmo/Mmo kept as published.
- **Published figures are not from primary documents** (no AFM or POH is available offline). Each
  profile header says which numbers are published (from the JSBSim `README.c310`, or public flap
  placard lists) and which are measured. Per-detent Vfe values for the global5000 are estimates.
- **c172p** is unchanged from F1: it keeps POH Vs0/Vs1 (33/44), which sit 26% and 11% under what
  the model measures (44.9 and 49.5).

## Extended fleet (WP F4)

Added: **A320, 787-8, MD11, c182, pa28**. Not added: **DHC6** and **J3Cub** (reasons below). Every
phase above passes for the added aircraft (51 test cases with the extended load test, plus 10 autopilot
cases), and the core fleet results are unchanged. Weights as shipped by the stock models: A320
141,000 lb, 787-8 371,000 lb, MD11 398,000 lb, c182 about 2,100 lb, pa28 about 1,950 lb. Same runway and
conditions as the core table.

| | A320 | 787-8 | MD11 | c182 | pa28 |
|---|---|---|---|---|---|
| Takeoff flaps | 3 (25 deg) | 15 | 15 | 10 | UP |
| Rotation starts (Vr in profile) | 166 | 155 | 150 | 58 (profile 55) | 58 (profile 60) |
| Liftoff speed, kt IAS | 170 | 160 | 163 | 63 | 69 |
| Liftoff distance, ft | 5,130 | 5,210 | 3,570 | 1,230 | 2,020 |
| Time to 3,000 ft AGL, s | 68 | 73 | 56 | 176 | 367 |
| Measured stall, clean, kt / alpha | 173.7 / 8.9 (elevator-limited) | 179.0 / 15.5 | 163.4 / 14.1 | 52.6 / 16.0 | 60.8 / 7.9 (elevator-limited) |
| Measured stall, approach flaps, kt / alpha | 150.4 / 6.8 (FULL) | 139.6 / 15.3 (30) | 108.8 / 13.9 (30) | 48.2 / 15.8 (20) | 67.5 / 3.7 (40, elevator-limited) |
| Measured stall, full flaps, kt / alpha | 150.4 / 6.8 | 135.4 / 15.2 | 108.8 / 13.9 | 47.6 / 15.8 | 67.5 / 3.7 |
| Profile Vs1 / Vs0, kt | 174 / 150 | 179 / 136 | 163 / 109 | 54 / 49 (published) | 61 / 54 |
| Profile stall warning alpha, deg | 6.0 | 11 | 10 | 11 | 7 |
| Approach flaps / Vapp, kt | FULL / 173 | 30 / 165 | 30 / 135 | 20 / 70 | 40 / 80 |
| Hands-off path error in 60 s, ft | 62 | 49 | 29 | 10 | 23 |
| Piloted path error from 8 NM, ft | 2 | 10 | 4 | 1 | 1 |
| Touchdown sink, fpm | 505 | 502 | 496 | 286 | 290 |
| Touchdown speed, kt IAS | 168 | 162 | 131 | 69 | 78 |
| Roll after touchdown, ft | 3,660 | 4,050 | 2,190 | 733 | 1,390 |
| Landing distance from the threshold, ft | 4,670 | 5,100 | 2,850 | 1,660 | 2,330 |
| Gear up / down, s | 5.0 / 4.9 | 14.9 / 14.7 | 5.0 / 4.9 | fixed | fixed |
| Slowest single flap step, s | 10.6 | 8.3 | 3.9 | 1.9 | 1.6 |
| Flaps full to UP, s | 22.4 | 29.7 | 6.9 | 3.9 | 3.9 |
| Cruise altitude hold over 60 s, ft | 17 | 13 | 19 | 0.1 | 1.0 |
| Cruise point | 250 kt, FL330 | 250 kt, FL370 | 270 kt, FL330 | 110 kt, 6,500 ft | 100 kt, 5,500 ft |
| Cruise throttle | 83% | 68% | 57% | 81% | 90% |
| Stopping devices used | brakes, speedbrake, reversers | same | same | brakes | brakes |

### Autopilot checks (N6) on the added aircraft

`FleetApTest.cpp` repeats the two N6 acceptance flights with the same pass bands (90 degree heading
change: overshoot under 5 degrees, altitude within 100 ft; ILS from 10 NM with a 10 kt crosswind: within
1 dot of LOC and G/S down to 200 ft AGL, disconnect between 100 and 160 ft AGL). The scenarios are
`assets/scenarios/<id>_ap_hdg90_xtst.ini` and `<id>_ap_ils_xtst.ini` (the test arms APR by hand at 30 s
on the airliners and 45 s on the piston aircraft, like N6's test). The `[autopilot]` keys that differ
from the category defaults are in each profile, with the reason.

| | A320 | 787-8 | MD11 | c182 | pa28 |
|---|---|---|---|---|---|
| 90 degree turn, heading overshoot, deg | 0.0 | 0.0 | 0.0 | 0.0 | 0.1 |
| 90 degree turn, altitude deviation, ft | 46 | 83 | 64 | 12 | 15 |
| ILS: LOC / G/S captured at, s | 56 / 56 | 57 / 57 | 53 / 53 | 56 / 56 | 58 / 60 |
| ILS: largest LOC / G/S deviation, dots | 0.91 / 0.46 | 0.81 / 0.61 | 0.96 / 0.74 | 0.60 / 0.40 | 0.54 / 1.00 |
| ILS: AP disconnect, ft AGL | 150 | 150 | 150 | 150 | 150 |

What the keys fix: the A320's low stall warning alpha made the default angle-of-attack limit (warning
minus 2.5 degrees) cut the pitch demand in a 25 degree bank (it lost 1,290 ft), so `alpha_limit_deg` is
set. The A320 and 787-8 oscillated in speed and path on the ILS (period about 70 s, 0.76 dots at 200 ft)
until the A/T gains were raised; the MD11 overshot its vertical speed to 9 degrees nose down until the VS
loop was softened (and then disconnected on the stall warning); the 787-8 also takes a higher pitch gain.
The pa28 G/S deviation sits on the 1 dot limit (full flaps on a model with little elevator authority).

### Overlay fixes (extended fleet)

Both are in `assets/jsbsim-overlay/engine/`, same recipe as the c310 propeller (the stock files stay
untouched, the overlay replaces them in the runtime layout):

- **c182, `prop_81in2v.xml`:** with the stock 81 inch variable-pitch tables the parked c182 engine dies at
  idle during the ground trim (rpm 0, not running), so the runway start has no engine. The overlay uses
  the one-dimensional fixed-pitch tables of the c172p propeller at 81 inches.
- **pa28, `propC8v.xml`:** the stock 84 inch variable-pitch propeller sits at 11 degrees (JSBSim resets
  the blade pitch to minpitch at every initial condition) and the cruise and final starts cannot be
  trimmed (udot). The overlay is the c172p fixed-pitch propeller (75 inch; the real PA-28-181 has 74).
- A320, 787-8 and MD11 need no overlay: they load, trim and fly as shipped. They use `direct` thrusters,
  so the reversers work through `reverser-angle-rad` like the 737. The A320 has no ground spoiler
  property, so the full speedbrake is the lift dump.

### Not added

- **DHC6 (Twin Otter):** the stock model cannot be trimmed on the ground, at the profile cruise point or
  on the approach (JSBSim reports "udot/wdot doesn't appear to be trimmable" for most flap and speed
  combinations), with the stock propeller, with a fixed-pitch two-dimensional propeller at 15, 22, 24
  and 30 degrees, and with the propeller tables cut to one dimension. The failures follow the flap
  setting and speed, so it is not a small overlay fix. Skipped; the draft profile and overlay were not
  committed.
- **J3Cub:** it loads, and the cruise start trims and flies, but (a) it has no flaps, and the suite (the
  profile test, flap schedule, approach and landing configuration, flap transit) assumes at least two
  flap detents and an approach detent, and (b) it is a tail-dragger with a free tail wheel: the scripted
  pilot (nose-wheel rotation, flare, steering and rollout) is written for tricycle gear, and in a quick
  test with full throttle it reached only 10 kt in 25 s while the heading drifted 7 degrees. A
  tail-dragger pilot is a separate piece of work.

### Notes for the maintainer

- **Elevator authority limits the stall tests on the A320 and pa28:** the idle deceleration cannot raise
  alpha past 8.9 degrees (A320 clean; Cm-alpha is -4 with a weak elevator) or past 7.9 / 3.7 degrees
  (pa28 clean / full flaps), so the "stall" speeds are the lowest speeds at which the elevator still
  holds 1 g, well above the real Vs. The profile speeds follow the model, as for the 737 and B747. The
  A320 Vapp is 173 kt with full flaps (Vfe 185). The pa28 spec sets `stallElevatorLimited`, which drops two
  checks: full-flap speed under clean speed (the flaps pitch the nose down beyond the elevator) and the
  stall warning alpha under the full-flap alpha. A small Cm-alpha or CmDe overlay would give the A320 a
  real stall, but that is a tuning change, not a fix, so it was not made.
- **Stall test settling:** `stallTest` takes a settle time before a load factor drop counts as the stall
  (default 5 s as in F2; the extended suite uses 30 s). The airliners start 1.4 g away from the trim of the
  new configuration after a configuration change at the start (MD11 flaps 30 at 165 kt), and the 5 s
  default ended the test on that transient (116 instead of 109 kt).
- **Pilot additions (all default off, core results unchanged):** `FleetSpec::airRudderKp` (rudder on the
  heading error after liftoff; the c182 and pa28 otherwise reached 8 to 11 degrees of heading error from
  torque yaw), `maxTakeoffHdgErrDeg`, `stallElevatorLimited`, and a reverser re-select when the reverser
  fails to latch (a wheel bounce on the A320 left forward thrust at 60% on the rollout, the reverser only
  latches at idle). `FLEET_TRACE=1` also prints the takeoff and climb.
- **787-8 flaps:** the model has one flap channel, 0 to 35 degrees in 30 s, and the gear takes 15 s (14.9 s
  against the 15 s test limit). Detents UP, 1, 5, 15, 20, 25, 30 and 35 (the real airplane stops at 30;
  the profile test needs the last detent at command 1.0). **MD11 flaps:** the model has 0, 15 and 30
  degrees only. **pa28 flaps:** the trim fails at 25 degrees (udot), so the approach uses 40 degrees.
- **Published speeds** (Vmo, Mmo, flap and gear placards, Vne and Vno) are estimates from public
  summaries, marked as such in each profile header; there is no AFM or POH in the repo.
- **`cmake/RuntimeLayout.cmake`** (outside the owned paths): the extended fleet ids are added to the list
  of stock aircraft copied into the runtime layout (`FSIM_EXTENDED_FLEET`, additive). Device packaging
  (`port/`) was not touched; the five aircraft add about 230 KB of stock files.
- **Device test:** the A320, 787-8 and MD11 are the same kind of JSBSim model as the 737 and B747 (two or
  three turbofans, `direct` thrusters), so the per-frame cost should be similar, but the frame cost and
  load time of the heavier models on the RG40XXH are not measured here.
