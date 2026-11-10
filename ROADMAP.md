# Roadmap

Pocket Vectors 1.0 runs on the Anbernic RG40XXH with the Pacific Northwest (`pdx_oth`) data pack:
real terrain from Portland (KPDX) to the coast at North Bend (KOTH), worldwide airport data, US
instrument approaches, and night lighting. Hardware measurements are in
[flightsim/docs/DEVICE.md](flightsim/docs/DEVICE.md).

## The plan

Work after 1.2 is on the **work board**: [flightsim/docs/wp/README.md](flightsim/docs/wp/README.md).
Each item is a card that one contributor or one agent can finish in one pull request. Point an
agent at a card (`flightsim/docs/wp/<ID>.md`) and at
[flightsim/docs/wp/BUILDER.md](flightsim/docs/wp/BUILDER.md); it needs nothing else to start.
The board was written before 1.2 took in all of Wave 6, so the cards that 1.2 finished say
`done` in their `Status:` line.

| Release | State | What it brings |
|---|---|---|
| **1.1** | Tagged `v1.1.0`. The release build failed, so there is no GitHub Release yet; the release workflow can be re-run for the tag from the Actions tab | Buttons that match the printed labels, in-game Help, the mini nav inset, world overlays |
| **1.2** | Released as `v1.2.0` (2026-11-08) | The table below |
| **1.2.1** | Released as `v1.2.1` (2026-11-08), device-tested 2026-11-07 (MED 59 fps in the cockpit) | Device tuning (X3c): the HUD about three times faster on the CPU, the GPU clock in the log, MED's terrain detail from a texture and the sky after the terrain (toward 60 fps), and speed, altitude, VS, heading and the FMA in the chase view |
| **1.2.2** | Merged (#14) | The launch script `PocketVectors.sh` with box art, so the Ports menu shows the name, and a one-zip install on the release |
| **1.3** | On `main` (2026-11-08/09: #15–#29). Not tagged yet: there is no `v1.3.0` tag or GitHub Release (the latest is `v1.2.1`); the owner tags it | Taxiways and aprons from OpenStreetMap, on smooth ground that meets the runways without a step, with yellow centerlines and edge lines (W8a–W8c; data © OpenStreetMap contributors, ODbL); a Seattle to San Francisco pack on the release (R3); fly-by turn arcs between GPS legs, flown and drawn (N10); the flight director follows the GPS track (NAV armed over ROL captures at once); smooth runway intersections and runways graded and flush with the terrain (W1b); a flight log (U2); the flight-plan screens without overlapping text; newer README screenshots |
| **1.4** | On `main` (2026-11-09/10: #30–#42 and the release prep); the owner device-checks `main`, then tags `v1.4.0` | Stall warnings by flap setting (F5); V-speed and flap bugs on the speed tape (H9); route and path overlays that follow the declutter level (H10); flight-plan edits like a Garmin or an FMS: Direct-To rules, NEW DESTINATION with KEEP / CLEAR ROUTE, delete and activate legs, DIRECT TO NEXT, a 10 s undo (N11a, N11b); arcs at every fly-by turn (N10b); a PA28 that climbs on a balked landing (F8); gentler pitch, a stepped trim, PITCH DAMPER and AUTO TRIM (F9); armed speed brakes (F10); nose-wheel steering for taxiing (F11); the Seattle to San Francisco pack as the release's pack |
| **Later** | Cards on the board | Pedal steering above 35 kt held to the tiller's lateral limit (a light aircraft can still tip on full stick there; F11 follow-up), a larger minimum fly-by arc if the physical turn looks too small on the map (N10b follow-up), a `--bench` run (rest of X3a), probe prompts (P1b), live METAR at launch (X5), Quick Flight and landing scores (U1), airways in plans (N8), map terrain shading and holds (H8), more region packs (R3); after the taxiways: hold-short bars, taxiway lights, taxi routes on the map, signs |

## Known issues

- Frame rate on the RG40XXH (1.2.1, 2026-11-07, DEVICE.md §3): GRAPHICS QUALITY MED, the default,
  holds 59 fps in the cockpit and 60 inside a cloud, but drops to 30 under an overcast layer and
  while orbiting the chase camera (the GPU, which also stays at its lowest clock).
- X and Y on the RG40XXH match the printed labels in the PortMaster mapping. That still wants a
  confirmation in the game (A and B were checked on the device with the previous, swapped table).

## 1.2

Device-tested on 2026-11-05. The 1.2.0 release pull request adds that session's fixes (route
overlay descent, mode buttons turn the FD on, a 1024-sample audio buffer, `._*` files skipped) and
a data pack with SIDs and STARs on the release; then the owner tags `v1.2.0`.

| ID | What | Notes |
|---|---|---|
| I2 | 1.1 device-test fixes | Gear no longer moves when a menu closes with A; L2 look-around works; R1 + Y and L1 + Select do what CONTROLS.md says; runway outlines clear below 250 ft above the field; L1 + R2 leaves ALT hold toward the altitude bug |
| N7 | Autopilot go-around (TO/GA) and FLC | Missing from the 1.0 autopilot |
| V1 | External chase view | Simple models built in code |
| W7 | Cloud layers | From the scenario, a setting or the METAR |
| N8 | SIDs, STARs and approach transitions | SIDs and STARs need a data pack rebuilt with the new `cifp.py`; older packs still load. The card board calls this N9 |
| S2 | GPWS voice callouts, audio buffer | Synthesized clips (Flite, PLAN D22); the tones stay as the fallback. 1.2.0: a 1024-sample buffer on the handheld (the board's S2 part A) |
| F6 | Fuel load and planning | FUEL row on the new-flight screen, endurance on the HUD, LOW FUEL |
| X3 | Device performance pass | Done in 1.2.1: frame timing and the GPU clock in the log, the on-screen FPS, a LOW / MED / HIGH preset, and the tuning (X3c): MED 59 fps in the cockpit on the device (1.2.0: 30) |

## Later

Live METAR refresh on the device and the rest of the v2 list are in
[flightsim/docs/PLAN.md](flightsim/docs/PLAN.md).
Online ideas (live traffic, multiplayer, ATC) are explored, not planned, in
[flightsim/docs/ONLINE.md](flightsim/docs/ONLINE.md). Photoreal scenery, 3D cockpits and failures
are non-goals.
