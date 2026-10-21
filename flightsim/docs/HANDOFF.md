# Hand-off, 2026-10-21

Start here if you are picking up this project. Read `CLAUDE.md` first (hard rules), then this
page, then the WP you take in [WORK_PACKAGES.md](WORK_PACKAGES.md) ("Wave 5" at the end).

## 1. Where things stand

- **Code:** `main`. All 30 original WPs are merged; only X3 (device performance pass) is open.
- **It runs on the device.** On 2026-10-20 the game and the probe ran on the RG40XXH stock OS from
  the SD card (`/mnt/sdcard/Roms/PORTS`). The game loaded the nav DB and the `pdx_oth` terrain pack,
  opened the menus, and the user flew the Global 5000 at KPDX. Device facts are in
  [DEVICE.md](DEVICE.md) §2 (filled from the probe), the run is in DEVICE.md §3.
- **Reported problems:** A and B are swapped, maybe more buttons. The probe shows why (DEVICE.md
  §2, WP D1). The user can work around it today with `flightsim/conf/controls.ini`
  ([INSTALL.md](../INSTALL.md), "Buttons in the wrong place?").
- **Not measured yet:** frame rate on the device. Ask the user to set `show_fps = true` in
  `flightsim/conf/settings.ini` (or the Settings menu) and send `flightsim_launch_log.txt`.

Fixes made in the 2026-10-21 session (already on `main`):
- The launch scripts used PortMaster's ports folder (`/roms/ports/<port>`, internal storage) instead
  of the folder next to the script on the SD card. They now use the folder that holds the binary,
  next to the script first, and log every candidate. `port/test_launch_scripts.sh` covers it.
- When the SD card drops the execute bit, the scripts start the binary through
  `/lib/ld-linux-aarch64.so.1`; `/proc/self/exe` then names the loader, so the game would not find
  its assets. The scripts now pass `--asset-dir` (game) and `--jsbsim-dir` (probe).
- The device workflow uploads `flightsim-sdcard-PORTS` (both ports unpacked) and the datapack
  workflow uploads a `data/` folder, so each download unzips once.

## 2. What the user asked for next

In their words, then the WP that covers it:

| # | Request | WP |
|---|---|---|
| 1 | Button mapping is off (A/B swapped, maybe others). Also: how does mapping work on other Anbernic devices, can people fix it in an ini? | **D1** (defaults), INSTALL.md section (done), H6 button test |
| 2 | Show the control mapping in game, in a menu | **H6** Controls page |
| 3 | A help menu with docs on how the game works and how to do things, plus an About section: "Zack Martin", GitHub `zackm-780`, Buy Me a Coffee `buymeacoffee.com/zackm` | **H6** |
| 4 | Visual flight path: waypoints with a floating name, vertical guidance optional, everything toggleable | **W6** layers 1–2 |
| 5 | Or a HUD mode with the "pink line" in a corner to hand-fly | **H7** |
| 6 | Runways are hard to see from above; toggle nearby airports with NM distance | **W6** layers 3–4 |
| 7 | README: name, GitHub and coffee link near the bottom; repo public and forkable for contributors | Done in root `README.md` (see §5 about a licence) |
| 8 | Public-facing install instructions: no probe, how to get flight plans, how to add data packs, a script that packages everything into one zip to unzip onto the SD card | **R1** |

Suggested order: D1, then H6 and R1, then H7 and W6 (at most 3 agents in parallel).

## 3. Build, test, deliver

- Desktop: the commands in `CLAUDE.md` (cmake + ctest, headless scenario runs, xvfb screenshots).
  Every UI change needs 640×480 screenshots.
- **Device build:** push a branch; the `device` workflow builds in Debian 11 (GCC 10, `-Werror`)
  and catches warnings the desktop compiler misses, so check it after **every** push. It also runs
  the game and the probe under qemu as a smoke test.
- **What the user copies to the card:**
  - Game: artifact **`flightsim-sdcard-PORTS`** of the device run. Unzip once, copy the four items
    into `Roms/PORTS`. The user's card already has `flightsim/data/` and `flightsim/conf/`:
    copying over the folder on Windows merges and keeps them; on a Mac, Finder "Replace" deletes
    them. When only a few files changed, say which ones.
  - Data: artifact `flightsim-data-pdx_oth-20261021` of
    [datapack run 39525169093](https://github.com/zackm-780/pocket-vectors/actions/runs/39525169093)
    (expires 2026-11-20; rerun the workflow after that). It holds `data/`; copy it into
    `Roms/PORTS/flightsim/`.
  - The user prefers **one zip**. If your environment can send files, assemble it: unzip the ports
    artifact into a folder, unzip the data artifact into its `flightsim/`, zip the folder's
    contents. R1 turns this into `tools/sdcard/make_sdcard_zip.py`.
- Device-only behaviour: mark the PR "needs device test" and give the user a short checklist.

## 4. Device constraints that matter for new code

From the probe (DEVICE.md §2):
- **SDL at runtime is 2.0.12** (headers in the build image are 2.0.14). Calling an SDL function
  added after 2.0.12 makes the binary fail to start on the device. Check the SDL version of every
  new SDL call.
- GLES 2.0 context on Mali-G31, 640×480, depth 24, MSAA 4. **Draw calls are the expensive part**
  (150 calls add about 3 ms); triangles are cheap (40k = 2.5 ms). Batch overlays.
- Blended fill with the fog shader is about 17 full-screen layers per 16.7 ms: keep blended
  overlays small.
- L2/R2 are digital. R3 jolts right stick X (rudder) when clicked; L3 is clean. The Menu key
  reaches the game as SDL `back`; Select arrives as SDL `guide`.
- gptokeyb kills the process on Select+Start (`-sudokill`, exit 137). Anything that must be saved
  (settings, last flight) has to be written before that, not only on a clean exit. Check this
  when touching settings.
- Audio: one ALSA underrun at 512 samples / 22.05 kHz in a flight; 1024 may be safer (S1).

## 5. Open questions for the user

- X and Y: swapped as well? Does L1 work (the probe's L1 prompt got no press)?
- "It kind of works": what else felt wrong (frame rate, stick feel, text size, terrain)?
- Decided 2026-10-21 (PLAN D20, D21): the name is **Pocket Vectors**, subtitle "Retro Flight Sim
  with Modern Data" (main menu, window title, port metadata and READMEs are done). Only the display
  name changed: keep `flightsim/`, `FlightSim.sh` and `flightsim.aarch64`, or users lose `conf/` and
  `data/` on update. The licence is **Apache-2.0** with a `NOTICE` crediting Zack Martin; both
  ship in `port/licenses/`, and the H6 About page should show the NOTICE.

## 6. Process

Roles, models and review rules are in [AGENTS.md](AGENTS.md); code rules in
[CONVENTIONS.md](CONVENTIONS.md). One WP per agent, stay in its owned paths, additive changes to
`src/core/`, units in every name, no generated data in git, update the WP `Status:` line.
