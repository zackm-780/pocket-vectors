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

| ID | What | Notes |
|---|---|---|
| I2 | 1.1 device-test fixes | Gear no longer moves when a menu closes with A; L2 look-around works; R1 + Y and L1 + Select do what CONTROLS.md says; runway outlines clear below 250 ft above the field |
| N7 | Autopilot go-around (TO/GA) and FLC | Missing from the 1.0 autopilot |
| V1 | External chase view | Simple models built in code |
| X3 | Device performance pass | Draw-call budget on the Mali-G31; needs frame-rate numbers from a device flight |

## Later

Cloud layers (W7), SIDs and STARs (N8) and GPWS voice callouts (S2) are specified in
[flightsim/docs/WORK_PACKAGES.md](flightsim/docs/WORK_PACKAGES.md) (Wave 6). Live METAR refresh on
the device and the rest of the v2 list are in [flightsim/docs/PLAN.md](flightsim/docs/PLAN.md).
