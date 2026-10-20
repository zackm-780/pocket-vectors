# Device: Anbernic RG40XXH (stock OS)

## 1. Known specs (manufacturer)
| Item | Value |
|---|---|
| SoC | Allwinner H700, 4× Cortex-A53 @ 1.5 GHz (aarch64) |
| GPU | Mali-G31 MP2 |
| RAM | 1 GB LPDDR4 (shared with GPU) |
| Display | 4.0" IPS, **640×480** (4:3) |
| Controls | D-pad, A/B/X/Y, L1/R1, L2/R2, Select, Start, Menu/function key, volume ±, power, **two analog sticks** |
| Connectivity | Wi-Fi, Bluetooth, mini-HDMI out, USB-C |
| Storage | microSD (TF) card(s) |
| OS | Anbernic stock Linux (64-bit). Runs PortMaster ports. |

## 2. Unknowns: measure with the P1 probe, then fill in
| Fact | How | Value |
|---|---|---|
| glibc version | `ldd --version` / probe | _tbd_ |
| Kernel | probe | _tbd_ |
| PortMaster install path (`controlfolder`) | probe / launch script log | _tbd_ |
| Ports folder path on the SD card | user | _tbd_ |
| SDL2 version and video driver | probe | _tbd_ |
| GL_VERSION / GL_RENDERER / extensions | probe | _tbd_ |
| Depth bits / MSAA available | probe | _tbd_ |
| Max texture size | probe | _tbd_ |
| L2/R2 analog or digital? | probe input log | _tbd_ |
| L3/R3 (stick clicks) present? | probe input log | _tbd_ |
| Face-button names as SDL reports them (A/B swap?) | probe input log | _tbd_ |
| Menu key usable by ports? | probe input log | _tbd_ |
| Free RAM at idle in a port | probe | _tbd_ |
| GPU: triangles/frame at 30 fps (fog shader) | probe benchmark | _tbd_ |
| CPU: JSBSim µs/step (c172p, 737) | probe benchmark | _tbd_ |
| Audio device / sample rate | probe | _tbd_ |

## 3. Field notes
Add observations from device tests here (date, build, what was seen).

- **2026-10-19, first sideload (stock OS):** both the probe and the game opened and exited
  immediately. Likely cause: the stock OS has no PortMaster, so the PortMaster-template launch
  scripts failed before writing any log. The scripts now run without PortMaster (and under a
  plain `sh`) and always write `<port>_launch_log.txt` next to the script. They were also renamed
  to `FlightSim.sh` / `FlightSimProbe.sh` (no spaces, in case the launcher does not quote paths),
  and the log now records whether the userspace is 64-bit (some stock OS releases are 32-bit,
  which the aarch64 build cannot run on). Awaiting a retest.

## 4. How to run the probe
The probe is the PortMaster port "Flight Sim Probe" (source in `tools/probe/`, full description in
`tools/probe/README.md`). Every row of the table in section 2 has a matching entry in the report's
KEY FACTS block.
1. Get `flightsim-probe-portmaster.zip` from the `device` CI workflow artifact (or `tools/build_device.sh`).
2. Unzip it into your PortMaster ports folder on the SD card (`FlightSimProbe.sh` and `flightsimprobe/`
   side by side).
3. Run "Flight Sim Probe" from the Ports menu. Hands off for the first 1.5 s, then press what each prompt
   asks (never Select and Start together: that exits). About 2 minutes in total.
4. Send back `flightsimprobe/probe_report.txt` and `flightsimprobe/log.txt`.
5. Paste the KEY FACTS block under section 5 and fill in the table in section 2.

## 5. Probe results
_tbd_: paste the KEY FACTS block of `probe_report.txt` here (device firmware, date, probe version).
