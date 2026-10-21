# Plan

## 1. Vision

A real flight simulator that fits in your pocket. Pick an aircraft from a Cessna 172 to a Boeing 747,
start on a runway or on final at a real airport, and fly over real terrain. Follow a flight plan and
shoot an ILS approach down to minimums. Everything is flown first person from a heads-up display
that carries all the data a pilot needs. It targets the Anbernic RG40XXH and its two analog sticks.

Visual target: late-90s / early-2000s sim fidelity (think MS Flight Simulator 98). Shaded,
land-cover-colored real terrain with recognizable mountains and coastlines, runways with correct
markings and lights, and PAPIs that work. Flight physics and navigation matter more than looks.

## 2. Scope

### v1 (in scope)
- First-person view with full HUD (see [HUD.md](HUD.md)), look-around, and no cockpit or exterior
  aircraft model. JSBSim ships no visual models.
- JSBSim flight model with stock aircraft (fleet in section 5).
- Real terrain from an offline-built tile pyramid: the whole world at coarse resolution plus
  high-detail regions chosen when the data pack is built. Land-cover coloring and water.
- Real airports and runways worldwide. Runway markings, edge, threshold and approach lights, PAPI.
- Start positions: on a runway, on final at N NM, airborne at cruise.
- Flight plans: build in-game (quick plan, direct-to, edit) or import X-Plane 11 `.fms`.
- Approaches: synthesized ILS and visual approach for any runway, plus real US procedures
  (ILS / RNAV / VOR) from FAA CIFP. Missed approach.
- Navigator (GPS/FMS-style, ILS/LOC), autopilot (HDG/NAV/APR/ALT/VS), flight director,
  autothrottle for jets.
- 2D moving-map page.
- Menus: aircraft select, airport/runway search, start position, flight plan, settings, pause.
- PortMaster package that runs on the stock OS. Desktop builds for development.

### Later (v2+)
Live weather from METAR over Wi-Fi, clouds, night lighting, procedural audio polish,
GPWS voice callouts, SIDs/STARs and airways routing, external view with simple models.

### Non-goals
Photoreal scenery, 3D cockpits, multiplayer, ATC, failures, add-on aircraft authoring.

## 3. Target platform and budgets

Device facts and the values still to be measured are in [DEVICE.md](DEVICE.md). Short version:
Allwinner H700 (4× Cortex-A53 @ 1.5 GHz), Mali-G31 MP2, **1 GB RAM**, **640×480** 4" screen,
two analog sticks, Wi-Fi, 64-bit Linux stock OS with PortMaster.

**Provisional budgets.** These are revised after the P1 probe benchmark runs on the device.

| Item | Budget |
|---|---|
| Frame rate | 30 fps locked minimum (33.3 ms), 60 fps stretch on low settings |
| Flight model | JSBSim at 120 Hz, ≤ 4 ms per frame on device |
| Terrain | ≤ 40k triangles visible, ≤ 15 ms GPU |
| HUD | Single batched draw, ≤ 4k line quads, ≤ 2 ms |
| Draw calls | ≤ 150 per frame |
| Memory | Process RSS ≤ 350 MB; GPU textures and buffers ≤ 128 MB; terrain cache ≤ 128 MB |
| Load time | Menu → flying ≤ 15 s |
| Storage | Unconstrained (user has plenty). Data packs live on the SD card. |

## 4. Decision log

Append new decisions at the bottom. Don't rewrite old ones; supersede them with a new entry.

| # | Decision | Why |
|---|---|---|
| D1 | **Not Pico-8.** | 128×128, 16 colors and ~32 KB carts can't hold terrain data, a flight model or nav logic. |
| D2 | **C++17, SDL2, OpenGL ES 2.0** (desktop GL 2.1 fallback through one function loader and a shader preamble). | Fastest option on an A53/Mali-G31. Same code builds on desktop and device. GLES2 is the safe baseline on Mali drivers. |
| D3 | **JSBSim v1.3.3**, fetched by CMake and built as a **shared library** shipped with the port. | Proven open-source 6-DOF model with stock aircraft. Shared linking keeps LGPL-2.1 compliance simple. |
| D4 | **Delivered as a PortMaster port** on the stock OS. No custom firmware needed. | The stock OS is 64-bit Linux and runs PortMaster ports. |
| D5 | **First person + HUD only**, no cockpit or exterior model in v1. | JSBSim has no visual models. Keeps the GPU and memory budget on terrain and the HUD. |
| D6 | **Stock JSBSim fleet.** Bizjet = Global 5000, heavy = 747 (no A380 in JSBSim). | Use the aircraft JSBSim already ships. |
| D7 | **Rendering in ECEF, camera-relative (RTC).** Every mesh has a double-precision ECEF anchor and float local vertices. | Correct earth curvature and horizon at any altitude, no float jitter, no origin shifting. |
| D8 | **Terrain = geographic quadtree tile pyramid**, 257×257 int16 tiles (format in [DATA.md](DATA.md)). | Simple, streamable, works for the whole world at coarse resolution plus detailed regions. |
| D9 | **Nav data from OurAirports (public domain) plus FAA NASR/CIFP (US public domain)**, preprocessed into compact CSVs. | Free and legal. Real approach procedures are only free for the US. Other regions get synthesized approaches. |
| D10 | **Flight plan file format = X-Plane 11 `.fms`** (import and export). | Plain text, exported by SimBrief, Little Navmap and others. |
| D11 | **Our own autopilot, flight director and autothrottle**, not JSBSim's per-aircraft autopilots. | Uniform behaviour and HUD annunciation across every aircraft. Most stock models have no autopilot. |
| D12 | **Offline data pipeline in Python** (numpy, rasterio, requests) run on a PC. The device only reads prebuilt binary/CSV. | Heavy geoprocessing doesn't belong on the handheld. |
| D13 | **HUD uses a hand-authored stroke (vector) font** drawn as thick lines. | Scales to any size, looks like real HUD symbology, no font files or dependencies. |
| D14 | **Runtime config and data formats are INI and CSV**, plus fixed binary layouts. No JSON library. | Fewer dependencies, easy to hand-edit. |
| D15 | **aarch64 builds target Debian 11 (glibc 2.31)**, with `-static-libstdc++ -static-libgcc`. SDL2 comes from the device or PortMaster. | Older glibc maximises device compatibility. Revisit after the probe reports the device's glibc. |
| D16 | **Treat JSBSim geodetic altitude as MSL** and ignore geoid undulation everywhere. | Terrain (EGM2008 heights) and airport elevations are MSL. Using one consistent datum keeps runways on the ground. |
| D17 | **Dependencies allowed:** SDL2 (zlib), JSBSim (LGPL-2.1, shared), doctest (MIT, tests only), stb_image_write (public domain/MIT, screenshots). | Minimal set. Anything else needs a new decision entry. |
| D18 | **Work is split into packages** with a separate review pass (see [AGENTS.md](AGENTS.md)). | One package per pull request, with review before merge. |
| D19 | **First detailed region = PDX–OTH corridor**, terrain preset `pdx_oth` = bbox lon −124.7..−121.4, lat 43.0..46.3, levels 7–10 (≈ 0.4 GB with land class). It covers KPDX, KOTH, the Oregon coast, the Coast Range, Willamette Valley, Mt Hood and Mt St Helens. **Primary test route: KPDX ⇄ KOTH** (~160 NM via UBG–CVO–EUG–OTH; sample plans in `assets/plans/`). Real-data scenarios, sample plans and acceptance tests use these airports. XTST stays the synthetic test airport for data-free unit tests. | Mountains, coast, valley, ILS and RNAV approaches in one corridor. |
| D20 | **Licence: Apache-2.0** for the project's own code, with a `NOTICE` file that credits Zack Martin. | Forks may modify and redistribute the code if they keep the copyright and the NOTICE. Compatible with JSBSim (LGPL-2.1, linked as a shared library) and SDL2 (zlib). Data packs keep their sources' terms (ATTRIBUTION.txt). `LICENSE` and `NOTICE` ship in `port/licenses/`. |
| D21 | **Name: Pocket Vectors**, subtitle "Retro Flight Sim with Modern Data". Display name only: folders, scripts and binaries stay `flightsim/`, `FlightSim.sh`, `flightsim.aarch64` so updates keep `conf/` and `data/`. | Display name set 2026-10-21. The on-card folder name stays stable across updates. |

## 5. Fleet

All aircraft are stock JSBSim models from `JSBSim-Team/jsbsim` `aircraft/`. Each needs an aircraft
profile (`assets/aircraft/<id>.ini`) and must pass the flyability tests in WP F2.

| Tier | JSBSim id | Aircraft | Type | Why it's in |
|---|---|---|---|---|
| Core | `c172p` | Cessna 172P Skyhawk | single piston, fixed gear | Default trainer. Required from M1. |
| Core | `c310` | Cessna 310 | twin piston, retractable | Gear and twin practice |
| Core | `global5000` | Bombardier Global 5000 | business jet | The "G6"-class bizjet |
| Core | `737` | Boeing 737 | narrow-body airliner | Airliner |
| Core | `B747` | Boeing 747 | wide-body heavy | Stand-in for the A380 |
| Extended | `pa28`, `c182`, `J3Cub`, `DHC6`, `A320`, `787-8`, `MD11` | | | After the core fleet is solid |

## 6. Milestones

Each milestone is done when its exit criteria pass on desktop and, where marked 📱, on the device.

| Milestone | WPs | Exit criteria |
|---|---|---|
| **M0 Hello Device** | WP-00, P1 | Desktop build and CI green. The port zip launches on the RG40XXH and quits with Select+Start 📱. Probe report recorded in DEVICE.md 📱. |
| **M1 First Flight** | F1, P2, H1, H2, W4, W5 (synthetic airport) | Cessna 172 takes off, flies and lands at the synthetic airport XTST over flat ground. HUD shows every status field in HUD.md section 4. Gamepad controls per CONTROLS.md. ≥ 30 fps on device 📱. |
| **M2 Real World** | W1, W2, W3, F3, N1, N2, W5 (real data) | Fly over real terrain in the PDX–OTH corridor (D19) with the world coarse layer around it. Take off from KPDX and land at KOTH on runways that sit flat on the terrain. Mt Hood is recognizable. The aircraft collides with terrain. Device ≥ 30 fps 📱. |
| **M3 Approaches** | N5, N6, H3, H5 | Start on 10 NM final at any runway. Fly a synthesized ILS by hand with HUD deviation scales, or with the AP coupled. Autopilot modes and FMA work. Airport and runway search menu. |
| **M4 Flight Plans** | N3, N4, H4 | Fly **KPDX → KOTH** from an imported `.fms` plan, following it with the HUD, FD and AP (LNAV). Load a real CIFP approach at KOTH, fly it including the missed approach, then fly back to KPDX with a real ILS. Moving map shows the route. |
| **M5 Fleet** | F2, F4 | All core aircraft pass the flyability tests and are selectable. Then the extended fleet. |
| **M6 Polish** | P3, S1, X1–X4 | Settings persistence, audio, manual and METAR weather, terrain warnings, device performance pass, night lighting (stretch). |

## 7. Open questions

| # | Question | Status |
|---|---|---|
| Q1 | Project name? | **Answered (D21):** Pocket Vectors. The on-card folder stays `flightsim`. |
| Q2 | Which region gets high-detail terrain first? | **Answered (D19):** the PDX–OTH corridor, preset `pdx_oth`. |
| Q3 | Is the main route in the US? | **Answered:** Portland (KPDX) ⇄ Southwest Oregon Regional, North Bend (KOTH), both covered by FAA CIFP. |
| Q4 | On-device facts: glibc, SDL video driver, GLES version, whether L2/R2 are analog, whether L3/R3 exist. | Measured by the probe and recorded in DEVICE.md. |
| Q5 | Units: aviation standard (kt, ft, NM, inHg/hPa)? | kt / ft / NM / fpm. Pressure in hPa or inHg is a setting. |
| Q6 | Do printed X and Y match SDL names on the RG40XXH, the same way A and B do? | Likely yes (DEVICE.md §2). Needs one more on-device check (ROADMAP, D1). |
| Q7 | What frame rate does a flight hold on the RG40XXH? | Not measured. `show_fps` in settings writes it to the launch log. |
