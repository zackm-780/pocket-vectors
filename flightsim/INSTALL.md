# Install on Anbernic RG40XXH (stock OS)

One-page steps for the PortMaster-style port. PortMaster itself is optional; the stock OS
launcher works the same way. Get `flightsim-portmaster.zip` and
`flightsim-probe-portmaster.zip` from the GitHub **device** workflow artifact
(`flightsim-device-zips`), and a data pack from the **datapack** workflow
(`flightsim-data-<preset>-<date>`).

## 1. Unzip the ports

Unzip each zip so the `.sh` launcher sits **next to** its folder in the ports/scripts
directory your launcher uses:

- `FlightSim.sh` next to `flightsim/`
- `FlightSimProbe.sh` next to `flightsimprobe/`

## 2. Check the JSBSim library

After unzip, confirm `flightsim/libs.aarch64/libJSBSim.so.1` is a **normal file** (not a
shortcut/symlink), about **4.3 MB**. FAT/exFAT cards and some unzip tools drop symlink
chains; the port ships this as one real file on purpose. Same check for
`flightsimprobe/libs.aarch64/libJSBSim.so.1` if you installed the probe.

## 3. Install the data pack

Unzip the datapack artifact **into** `flightsim/data/`. The zip’s top level is
`terrain/`, `nav/`, `ATTRIBUTION.txt`, `README.txt` — not a nested `data/` folder.
If the pack was split into multiple part zips (e.g. `*-part1` / `*-part2` / `*-part3`
for upload size limits), unzip all of them into the same `flightsim/data/` folder in
any order; together they form the full pack. Alternatively put the pack elsewhere and
set `data_dir` in `flightsim/conf/settings.ini` (created on first launch).

## 4. Probe first, then fly

1. From Ports, run **Flight Sim Probe**. It writes `flightsimprobe_launch_log.txt` next to
   `FlightSimProbe.sh`. When it finishes, copy **`flightsimprobe/probe_report.txt`** off
   the card (that is the file to keep).
2. Run **Flight Sim**. Exit with **Select + Start**.
3. If something goes wrong, read `flightsim_launch_log.txt` next to `FlightSim.sh`.

## Linux / SD-card notes

- FAT/exFAT often drops the executable bit. The launchers start the binary via
  `/lib/ld-linux-aarch64.so.1` when needed, so that is fine.
- Keep `FlightSim.sh` / `FlightSimProbe.sh` with **LF** line endings. Do not edit them in
  Notepad (CRLF breaks `#!/bin/bash`).
- Needs a **64-bit** userspace with SDL2 (`libSDL2-2.0.so.0`). Some stock OS releases are
  32-bit; the launch log checks this (`LONG_BIT`, presence of
  `/lib/ld-linux-aarch64.so.1`).
