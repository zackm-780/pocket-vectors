# Install on Anbernic RG40XXH (stock OS)

One-page steps. Everything comes from two GitHub Actions downloads, and each one unzips
once into folders you copy straight onto the card. The git repository itself has only source:
the game and the probe are compiled for the handheld by the **device** workflow, and the map and
nav data are built from public downloads by the **datapack** workflow. Neither is stored in git.

The ports folder on the card is `Roms/PORTS` (the stock OS shows it as `/mnt/sdcard/Roms/PORTS`).
PortMaster is optional; the launch scripts work with or without it.

## 1. Copy the ports

1. On GitHub open **Actions**, then **device**, then the newest green run on `main` (or the branch
   you are testing). Under **Artifacts** download **`flightsim-sdcard-PORTS`**.
2. Unzip it once. Inside are `FlightSim.sh`, `flightsim/`, `FlightSimProbe.sh` and
   `flightsimprobe/`.
3. Copy those four into `Roms/PORTS` on the SD card, replacing the old ones. Windows merges
   folders when you copy over them, so `flightsim/data/` and `flightsim/conf/` survive an update.
   A Mac's Finder "Replace" deletes the old folder first, so after an update copy `data/` (step 3)
   in again.

(`flightsim-device-zips` in the same run holds the same ports as PortMaster zips, for a
PortMaster catalogue submission. You do not need it.)

## 2. Check the JSBSim library

Confirm `flightsim/libs.aarch64/libJSBSim.so.1` is a **normal file** (not a shortcut/symlink),
about **4.3 MB**. FAT/exFAT cards cannot store symlinks; the port ships this as one real file on
purpose. Same check for `flightsimprobe/libs.aarch64/libJSBSim.so.1`.

## 3. Copy the data pack

1. On GitHub open **Actions**, then **datapack**, then **Run workflow** (defaults: preset
   `pdx_oth`, world layer off). When it finishes, download the artifact
   **`flightsim-data-<preset>-<date>`**.
2. Unzip it once. Inside is one `data/` folder (`terrain/`, `nav/`, `ATTRIBUTION.txt`,
   `README.txt`).
3. Copy `data/` into `Roms/PORTS/flightsim/` on the card, so you get
   `Roms/PORTS/flightsim/data/terrain/` and so on.

To keep the pack somewhere else on the card, set `data_dir` in `flightsim/conf/settings.ini`
(created on first launch).

## 4. Probe first, then fly

1. From Ports, run **Flight Sim Probe**. It writes `flightsimprobe_launch_log.txt` next to
   `FlightSimProbe.sh`. When it finishes, copy **`flightsimprobe/probe_report.txt`** off
   the card (that is the file to keep).
2. Run **Flight Sim**. Exit with **Select + Start**.
3. If something goes wrong, read `flightsim_launch_log.txt` next to `FlightSim.sh`. Its
   `candidate` lines list every folder the script checked for `flightsim.aarch64`, and
   `GAMEDIR=` shows the folder it used.

## Buttons in the wrong place?

The game reads the gamepad through SDL with the button names PortMaster gives your device, and a
small table in `flightsim/assets/controls_default.ini` (`[device_labels]`) says which SDL name
each **printed** label produces. Devices and firmwares differ, so you can override that table
without touching the game: create `flightsim/conf/controls.ini` (a plain text file) and put the
lines you need in it. The game only reads this file, never overwrites it.

**RG40XXH stock OS** (until the defaults are fixed, see WORK_PACKAGES D1):

```ini
[device_labels]
a = a
b = b
x = x
y = y
select = guide
```

**Other devices:** if A and B (or X and Y) are swapped, swap the right-hand sides of those lines.
If Select does nothing, try `select = back` or `select = guide`. The probe (Flight Sim Probe)
lists what every button produces in `flightsimprobe/probe_report.txt` (section CONTROLLERS and
INPUT TEST). You can also rebind actions in the same file, e.g. a `[pad.base]` section with
`GearToggle = a`; see the comments in `controls_default.ini` and `docs/CONTROLS.md`.

## Linux / SD-card notes

- FAT/exFAT often drops the executable bit. The launchers start the binary via
  `/lib/ld-linux-aarch64.so.1` when needed, so that is fine.
- Keep `FlightSim.sh` / `FlightSimProbe.sh` with **LF** line endings. Do not edit them in
  Notepad (CRLF breaks `#!/bin/bash`).
- Needs a **64-bit** userspace with SDL2 (`libSDL2-2.0.so.0`). Some stock OS releases are
  32-bit; the launch log checks this (`LONG_BIT`, presence of
  `/lib/ld-linux-aarch64.so.1`).
