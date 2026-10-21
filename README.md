# Pocket Vectors

**Retro flight sim with modern data.**

Pocket Vectors is a first-person flight simulator for the Anbernic RG40XXH. You fly from a HUD,
with a real flight model, real terrain, and real airports. The v1.0 release carries the Pacific
Northwest: terrain streamed from Portland (KPDX) to the coast at North Bend (KOTH).

![Cruising over the Columbia near Portland](screenshots/cruise-over-kpdx.png)

## Features

- **JSBSim flight model**, using stock aircraft from the Cessna 172 up through airliners.
- **Worldwide airports** and runways, with markings, lights and PAPI.
- **FAA approaches, fixes and airways**, plus a synthesized ILS for any runway.
- **Terrain streamed for the KPDX to KOTH region** in the release data pack (Copernicus DEM and
  ESA WorldCover).
- Flight plans built in game or loaded from X-Plane `.fms` files, a moving map, autopilot and a
  full HUD.

## Supported device

Tested on the **Anbernic RG40XXH stock OS**: 64-bit userspace and SDL2 (`libSDL2-2.0.so.0`).
PortMaster is optional. Other PortMaster handhelds are untested. If buttons land on the wrong
actions, the mapping lives in `flightsim/conf/controls.ini`
([install notes](flightsim/INSTALL.md)).

## Install

Download **`pocket-vectors-1.0.0-Roms-PORTS.zip`** from the
[v1.0.0 release](https://github.com/zackm-780/pocket-vectors/releases/tag/v1.0.0) and unzip it
into `Roms/PORTS` on the SD card. The top of the zip is `FlightSim.sh` and `flightsim/` (the game
and the `pdx_oth` data). If your unzip tool creates a folder named after the zip, move those two
items into `Roms/PORTS`.

The on-card folder stays `flightsim/`, the script stays `FlightSim.sh`, and the binary stays
`flightsim.aarch64`, so an update keeps `flightsim/conf/` and `flightsim/data/`.

The same release also has a game-only zip and a data-only zip for updates. Full steps:
[flightsim/INSTALL.md](flightsim/INSTALL.md).

![On runway 28L at KPDX](screenshots/ground-kpdx.png)

## Contributing

Contributions are welcome. Read [CONTRIBUTING.md](CONTRIBUTING.md). Every outside contribution
comes in as a fork and a pull request, CI has to pass (including the device workflow), and the
owner reviews and approves it before merge. Also see [CLAUDE.md](CLAUDE.md),
[flightsim/docs/CONVENTIONS.md](flightsim/docs/CONVENTIONS.md) and
[flightsim/docs/WORK_PACKAGES.md](flightsim/docs/WORK_PACKAGES.md). Contributions are accepted
under Apache-2.0.

Source and the on-device port both live under `flightsim/` so the card layout does not change
between versions.

## Licence

The code is licensed under the [Apache License 2.0](LICENSE). Forks and redistributions keep the
copyright notice and the [NOTICE](NOTICE) file. JSBSim (LGPL-2.1) and SDL2 (zlib) keep their own
licences. Data packs carry their sources' terms in `ATTRIBUTION.txt` (Copernicus DEM, ESA
WorldCover, OurAirports, FAA NASR and CIFP, WMM).

## Author and support

Made by **Zack Martin** ([@zackm-780](https://github.com/zackm-780)).
If you enjoy it and want to support the work: [buymeacoffee.com/zackm](https://buymeacoffee.com/zackm).
