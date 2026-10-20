# tests/fleet: core fleet flyability suite (WP F2)

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
| stall | Idle deceleration at 1 g from a trimmed start, clean, approach flaps and full flaps | Measured stall speeds match the table, profile Vs0/Vs1 within 30% of the model, Vapp at least 1.2 x the approach stall speed, stall warning alpha below the stall alpha |
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
| Measured stall, full flaps, kt / alpha | 44.9 / 14.3 | 59.0 / 15.0 | 95.8 / 14.0 | 118.6 / 11.0 | 112.9 / 7.7 |
| Profile Vs1 / Vs0, kt | 44 / 33 | 75 / 63 | 150 / 96 | 152 / 118 | 156 / 113 |
| Profile stall warning alpha, deg | 11 | 12.5 | 12 | 10.5 | 7.0 |
| Approach flaps / Vapp, kt | 20 / 65 | 15 / 95 | 30 / 130 | 30 / 155 | 30 / 140 |
| Hands-off path error in 60 s, ft | 7 | 17 | 27 | 47 | 35 |
| Piloted path error from 8 NM, ft | 1 | 1 | 1 | 1 | 1 |
| Touchdown sink, fpm | 265 | 321 | 302 | 311 | 501 |
| Touchdown speed, kt IAS | 64 | 93 | 125 | 151 | 136 |
| Roll after touchdown, ft | 876 | 879 | 2,021 | 2,311 | 2,604 |
| Landing distance from the threshold, ft | 1,801 | 1,845 | 2,986 | 3,425 | 3,317 |
| Gear up / down, s | fixed | 5.0 / 4.9 | 5.0 / 4.9 | 5.0 / 4.9 | 5.0 / 4.9 |
| Slowest single flap step, s | 1.9 | 1.9 | 2.5 | 3.8 | 2.3 |
| Flaps full to UP, s | 3.9 | 3.9 | 6.9 | 21.6 | 6.9 |
| Cruise altitude hold over 60 s, ft | 0.4 | 1.4 | 21 | 42 | 29 |
| Cruise point | 105 kt, 5,500 ft | 125 kt, 7,500 ft | 260 kt, FL370 | 270 kt, FL330 | 280 kt, FL350 |
| Cruise throttle | 78% | 78% | 87% | 79% | 65% |
| Stopping devices used | brakes | brakes | brakes, spoilers, reversers | same | same |

Stall angles of attack differ by flap setting for the jets (the 747 falls from 14.3 degrees clean to
7.7 with flaps 30), while the profile carries a single `stall_warn_alpha_deg`.

## Exceptions and notes for the supervisor

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
  Per-detent values would need an additive field in `FlapDetent` (supervisor decision).
- **B747 and 737 are not realistic against real figures:** the stock models stall well above real
  airplanes at these weights (737 flaps 30 stall 124 kt, B747 flaps 30 stall 113 kt, and the B747
  cannot be trimmed above about 150 kt with flaps 30). Profile speeds follow the model, with the
  published flap placards and Vmo/Mmo kept as published.
- **Published figures are not from primary documents** (no AFM or POH is available offline). Each
  profile header says which numbers are published (from the JSBSim `README.c310`, or public flap
  placard lists) and which are measured. Per-detent Vfe values for the global5000 are estimates.
- **c172p** is unchanged from F1: it keeps POH Vs0/Vs1 (33/44), which sit 26% and 11% under what
  the model measures (44.9 and 49.5).
