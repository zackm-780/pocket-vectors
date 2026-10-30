# Roadmap

Pocket Vectors 1.0 runs on the Anbernic RG40XXH with the Pacific Northwest (`pdx_oth`) data pack:
real terrain from Portland (KPDX) to the coast at North Bend (KOTH), worldwide airport data, US
instrument approaches, and night lighting. Hardware measurements are in
[flightsim/docs/DEVICE.md](flightsim/docs/DEVICE.md).

## The plan

Everything after 1.0 is on the **work board**: [flightsim/docs/wp/README.md](flightsim/docs/wp/README.md).
Each item is a card that one contributor or one agent can finish in one pull request. Point an
agent at a card (`flightsim/docs/wp/<ID>.md`) and at
[flightsim/docs/wp/BUILDER.md](flightsim/docs/wp/BUILDER.md); it needs nothing else to start.

| Release | What it brings | Cards |
|---|---|---|
| **1.1** | Buttons that match the printed labels, in-game Help, the mini nav inset, world overlays (all on `main` now), the right version on the About page, release notes that point at the terrain pack, and frame timing for the performance work | R2, P1b, X3a, device session T1 |
| **1.2** | Quality presets and a 30 fps floor on the device, go-around and FLC, voice callouts without audio crackle, live METAR at launch, stall warnings by flap setting, runways flush with the terrain, Quick Flight and landing scores | X3b, X3c, N7, S2, X5, F5, W1b, U1, device session T2 |
| **1.3** | Airways and SIDs/STARs in flight plans, terrain shading and holds on the map, cloud layers, more region packs | N8, N9, H8, W7, R3, device session T3 |

## Known issues

- The frame rate on the device has not been measured. Set `show_fps = true` in
  `flightsim/conf/settings.ini` and read `flightsim_launch_log.txt` (card T1).
- X and Y on the RG40XXH match the printed labels in the PortMaster mapping. That still wants a
  confirmation in the game (A and B were checked on the device with the previous, swapped table).

## Later, not scheduled

An external view with simple models (PLAN §2). Photoreal scenery, 3D cockpits, multiplayer, ATC
and failures are non-goals.
