# Roadmap

Pocket Vectors 1.0 runs on the Anbernic RG40XXH with the Pacific Northwest (`pdx_oth`) data pack:
real terrain from Portland (KPDX) to the coast at North Bend (KOTH), worldwide airport data, and
US instrument approaches. Open work is tracked as work packages in
[flightsim/docs/WORK_PACKAGES.md](flightsim/docs/WORK_PACKAGES.md). Hardware measurements are in
[flightsim/docs/DEVICE.md](flightsim/docs/DEVICE.md).

## Known issues

- Frame rate on the device has not been measured. Set `show_fps = true` in
  `flightsim/conf/settings.ini` and read `flightsim_launch_log.txt`.
- X and Y on the RG40XXH match the printed labels in the PortMaster mapping. That still wants a
  confirmation in the game (A and B were checked on device with the previous, swapped table).

## Next: 1.2

In review in one pull request, waiting for the owner's device test.

| ID | What | Notes |
|---|---|---|
| I2 | 1.1 device-test fixes | Gear no longer moves when a menu closes with A; L2 look-around works; R1 + Y and L1 + Select do what CONTROLS.md says; runway outlines clear below 250 ft above the field; L1 + R2 leaves ALT hold toward the altitude bug |
| N7 | Autopilot go-around (TO/GA) and FLC | Missing from the 1.0 autopilot |
| V1 | External chase view | Simple models built in code |
| W7 | Cloud layers | From the scenario, a setting or the METAR |
| N8 | SIDs, STARs and approach transitions | SIDs and STARs need a data pack rebuilt with the new `cifp.py`; older packs still load |
| S2 | GPWS voice callouts | Synthesized clips (Flite, PLAN D22); the tones stay as the fallback |
| F6 | Fuel load and planning | FUEL row on the new-flight screen, endurance on the HUD, LOW FUEL |
| X3 | Device performance pass | Frame timing in the log, the on-screen FPS and a LOW / MED / HIGH graphics preset are in. Tuning needs the timing from a device flight (DEVICE.md §6) |

## Later

Live METAR refresh on the device and the rest of the v2 list are in
[flightsim/docs/PLAN.md](flightsim/docs/PLAN.md).
Online ideas (live traffic, multiplayer, ATC) are explored, not planned, in
[flightsim/docs/ONLINE.md](flightsim/docs/ONLINE.md).
