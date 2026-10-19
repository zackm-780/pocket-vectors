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
