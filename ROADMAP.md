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
| **1.2** | On `main`, waiting for the owner's device test before the `v1.2.0` tag | The table below |
| **After 1.2** | Cards on the board | Device tuning to a 30 fps floor (X3c), audio without crackle (S2 part A), a `--bench` run (rest of X3a), probe prompts (P1b), live METAR at launch (X5), stall warnings by flap setting (F5), runways flush with the terrain (W1b), Quick Flight and landing scores (U1), airways in plans (N8), map terrain shading and holds (H8), more region packs (R3) |

## Known issues

- The frame rate on the device has not been measured. Turn on Settings → SHOW FPS (it shows at
  the top left) and follow the run in
  [flightsim/docs/DEVICE.md](flightsim/docs/DEVICE.md) §6; the timing tables land in
  `flightsim_launch_log.txt`.
- X and Y on the RG40XXH match the printed labels in the PortMaster mapping. That still wants a
  confirmation in the game (A and B were checked on the device with the previous, swapped table).

## Next: 1.2

On `main`, not tagged yet: it waits for the owner's device test.

| ID | What | Notes |
|---|---|---|
| I2 | 1.1 device-test fixes | Gear no longer moves when a menu closes with A; L2 look-around works; R1 + Y and L1 + Select do what CONTROLS.md says; runway outlines clear below 250 ft above the field; L1 + R2 leaves ALT hold toward the altitude bug |
| N7 | Autopilot go-around (TO/GA) and FLC | Missing from the 1.0 autopilot |
| V1 | External chase view | Simple models built in code |
| W7 | Cloud layers | From the scenario, a setting or the METAR |
| N8 | SIDs, STARs and approach transitions | SIDs and STARs need a data pack rebuilt with the new `cifp.py`; older packs still load. The card board calls this N9 |
| S2 | GPWS voice callouts | Synthesized clips (Flite, PLAN D22); the tones stay as the fallback |
| F6 | Fuel load and planning | FUEL row on the new-flight screen, endurance on the HUD, LOW FUEL |
| X3 | Device performance pass | Frame timing in the log, the on-screen FPS and a LOW / MED / HIGH graphics preset are in. Tuning needs the timing from a device flight (DEVICE.md §6) |

## Later

Live METAR refresh on the device and the rest of the v2 list are in
[flightsim/docs/PLAN.md](flightsim/docs/PLAN.md).
Online ideas (live traffic, multiplayer, ATC) are explored, not planned, in
[flightsim/docs/ONLINE.md](flightsim/docs/ONLINE.md). Photoreal scenery, 3D cockpits and failures
are non-goals.
