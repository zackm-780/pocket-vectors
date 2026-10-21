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

## Next

| ID | What | Notes |
|---|---|---|
| H6 | In-game Help: live control list, button test, how-to pages, About | Credits and data attribution |
| H7 | Small track-up nav inset on the HUD ("fly the pink line") | Off by default |
| W6 | World markers for the route, vertical path, nearby airports, and brighter runways | Each layer toggles on its own |
| R1 | Repeatable release packaging and a fuller install guide | Flight plans, more regions |
| X3 | Device performance pass | Draw-call budget on the Mali-G31 |

## Later

Night lighting, live METAR weather, and GPWS voice callouts are described in
[flightsim/docs/PLAN.md](flightsim/docs/PLAN.md). They are not part of 1.0.
