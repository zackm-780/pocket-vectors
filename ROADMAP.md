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
| **1.2** | Device-tested on 2026-11-05; the owner tags `v1.2.0` after the release pull request | The table below |
| **1.2.1** | Next | Device tuning (X3c): the HUD's CPU time, the GPU clock in the log, MED toward 60 fps |
| **After 1.2.1** | Cards on the board | A `--bench` run (rest of X3a), probe prompts (P1b), live METAR at launch (X5), stall warnings by flap setting (F5), runways flush with the terrain (W1b), Quick Flight and landing scores (U1), airways in plans (N8), map terrain shading and holds (H8), more region packs (R3) |

## Known issues

- Frame rate on the RG40XXH (2026-11-05, DEVICE.md §3): GRAPHICS QUALITY MED, the default, holds
  30 fps in the cockpit and LOW 60; the chase view dips to 20 fps while orbiting the camera. The
  HUD takes 8 ms of CPU per frame in the cockpit. 1.2.1 (X3c) works on both.
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
| X3 | Device performance pass | Frame timing in the log, the on-screen FPS and a LOW / MED / HIGH graphics preset are in, and measured on the device (MED 30 fps, LOW 60). Tuning is X3c, in 1.2.1 |

## Later

Live METAR refresh on the device and the rest of the v2 list are in
[flightsim/docs/PLAN.md](flightsim/docs/PLAN.md).
Online ideas (live traffic, multiplayer, ATC) are explored, not planned, in
[flightsim/docs/ONLINE.md](flightsim/docs/ONLINE.md). Photoreal scenery, 3D cockpits and failures
are non-goals.
