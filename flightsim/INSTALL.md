# Install Pocket Vectors on the Anbernic RG40XXH (stock OS)

Pocket Vectors is a first-person flight sim for the RG40XXH stock OS (64-bit, SDL2). PortMaster
is optional; the launch script works with or without it.

The ports folder on the card is `Roms/PORTS` (the stock OS shows it as `/mnt/sdcard/Roms/PORTS`).

## 1. One zip (start here)

Download **`pocket-vectors-1.0.0-Roms-PORTS.zip`** from the GitHub release
[Pocket Vectors 1.0](https://github.com/zackm-780/pocket-vectors/releases/tag/v1.0.0).

That zip is the game and the Pacific Northwest data pack (`pdx_oth`) together. It is about 125 MB.
Unzip it **into** `Roms/PORTS`. The zip has no extra folder around the game: its contents are

- `FlightSim.sh`
- `flightsim/` (the binary, libraries, and `flightsim/data/` with terrain and nav)

so after a correct unzip you have `Roms/PORTS/FlightSim.sh` next to `Roms/PORTS/flightsim/`.

Some unzip tools (Windows "Extract All", macOS Archive Utility) create a folder named after the
zip. If that happens, open that folder and move `FlightSim.sh` and `flightsim/` into `Roms/PORTS`.
Do not put them in a folder under `Roms/PORTS`.

In Ports, launch **Pocket Vectors**. Exit with **Select + Start**.

`flightsim/libs.aarch64/libJSBSim.so.1` must stay a normal file (about 4.3 MB), not a shortcut.
The zip is built that way because FAT/exFAT cards cannot store symlinks.

## 2. Updating later

Two smaller zips are on the same release, for when you do not want to copy everything again:

- **Game only** (`pocket-vectors-1.0.0-game.zip`): unzip into `Roms/PORTS`. It is `FlightSim.sh`
  and `flightsim/` without `data/`. On Windows, copying over the existing folder keeps
  `flightsim/data/` and `flightsim/conf/`. macOS Finder "Replace" deletes the old folder first,
  so copy `data/` back afterwards (or use the combined zip).
- **Data only** (`pocket-vectors-1.0.0-data-pdx_oth.zip`): unzip it and copy the `data/` folder
  inside it to `Roms/PORTS/flightsim/`, so you get `Roms/PORTS/flightsim/data/terrain/` and
  `Roms/PORTS/flightsim/data/nav/`.

To keep the pack somewhere else on the card, set `data_dir` in `flightsim/conf/settings.ini`
(created on first launch).

## 3. Assembling a ports zip

Tagged releases after 1.0 attach a **game-only** zip: `FlightSim.sh` and `flightsim/`, no probe
and no terrain. `flightsim/tools/sdcard/make_sdcard_zip.py` (Python 3.8 or newer, standard
library only) merges that zip with one or more data packs and optional `.fms` files:

```bash
python3 flightsim/tools/sdcard/make_sdcard_zip.py \
  --game pocket-vectors-vX.Y.Z-game.zip \
  --data pdx_oth.zip \
  --plan KPDX-KOTH.fms \
  --out ports.zip
```

`--game` and `--data` accept a zip or a folder. A single wrapper folder around the game is
unwrapped. Repeat `--data` to merge packs (later packs replace earlier files) and `--plan` for
each flight plan. The script refuses a symlink `libJSBSim.so.1`, a missing binary, or a pack
without `data/terrain` and `data/nav/nav.ini`, and it writes `.sh` files with LF line endings.
Unzip `ports.zip` into `Roms/PORTS` the same way as the 1.0 zip. The probe is not copied.

## 4. Flight plans

X-Plane `.fms` version 1100 files belong in `flightsim/data/plans/` (the script's `--plan`
flag puts them there). SimBrief and Little Navmap export this format. List every waypoint in
the file: the game reads an airway name but flies direct between the points that are written.
An approach uses the ARINC name without `RW`, for example `I28L` for the ILS to 28L. Sample
plans in the game: `assets/plans/KPDX-KOTH.fms`, `KOTH-KPDX.fms`, and `XTST-pattern.fms`.

In the game, New flight then Flight plan loads a file or builds a quick plan from the departure
airport to a destination runway (direct, or via navaids along the great circle) and can attach
an approach. Format notes: `src/nav/README_FMS.md`.

## 5. More regions

A tagged release does not include terrain or the nav database. Build a pack and keep its
`ATTRIBUTION.txt`. Copernicus DEM is free for worldwide use with the notice in that file. ESA
WorldCover is CC BY 4.0.

- From the Actions tab, run the **datapack** workflow. Presets: `pdx_oth` (Portland to the
  coast), `bayarea`, `mthood`, `tam_diablo`. Set **bbox** to `west,south,east,north` for a
  custom box (that replaces the preset). The artifact is kept 30 days. Unzip it once and pass
  the folder, or a zip of it, to `--data`.
- Locally: `tools/datapack/build_datapack.sh --out /path/to/pack --preset bayarea`, or
  `--bbox -122.5,37.2,-121.5,38.2`. See `tools/datapack/README.md`.

## 6. If something goes wrong

Read `flightsim_launch_log.txt` next to `FlightSim.sh`. Its `candidate` lines list every folder
the script checked for `flightsim.aarch64`, and `GAMEDIR=` shows the folder it used. The game
also writes `flightsim/conf/log.txt`.

## Buttons

On the RG40XXH the defaults match the printed labels: A is gear, B is brakes (double-tap for the
parking brake), X cycles the speedbrake, Y cycles the HUD, Select opens the map, Start pauses, and
Select+Start quits. The Menu key also acts as Select. L3 cycles world overlays (route, vertical
path, nearby airports, runway outlines). Every layer starts off; the pause menu and Settings turn
them on one at a time. R3 is not used in flight (the click jolts
the rudder).

**Other devices:** if A and B (or X and Y) come out swapped, or Select does nothing, create
`flightsim/conf/controls.ini` (plain text; the game reads it and does not overwrite it). A label
may list several SDL buttons, separated by commas. Example for a layout where the face buttons are
swapped and Select is `back`:

```ini
[device_labels]
a = b
b = a
x = y
y = x
select = back, guide
```

You can also rebind one action, for example a `[pad.base]` section with `GearToggle = a`. See the
comments in `flightsim/assets/controls_default.ini` and `docs/CONTROLS.md`.

Frame rate on the device has not been measured yet. To see it, set `show_fps = true` under
`[video]` in `flightsim/conf/settings.ini` (or use the Settings menu) and look at
`flightsim_launch_log.txt`.

## Linux / SD-card notes

- The card is usually FAT32 or exFAT. The release zips contain only real files (no symlinks) and
  `FlightSim.sh` uses LF line endings. Do not edit that script in Notepad (CRLF breaks
  `#!/bin/bash`).
- FAT often drops the executable bit. The launcher starts the binary via
  `/lib/ld-linux-aarch64.so.1` when needed.
- Needs a **64-bit** userspace with SDL2 (`libSDL2-2.0.so.0`). Some stock OS releases are 32-bit;
  the launch log checks this (`LONG_BIT`, presence of `/lib/ld-linux-aarch64.so.1`).

## Building it yourself

The git repository holds source only. The **device** workflow builds the handheld game, and the
**datapack** workflow builds a `data/` folder from public sources. Those Actions artifacts expire;
the release zips above do not. Testers who want the on-device probe still use the device
workflow's `flightsim-sdcard-PORTS` artifact (`FlightSimProbe.sh` and `flightsimprobe/`). See
`docs/BUILD_AND_DEPLOY.md`.
