# Work board: after 1.0

This folder is the plan for everything after Pocket Vectors 1.0. Each work package is one **card**
(`<ID>.md`) that one agent or one person can finish in one pull request. A card carries everything
its builder needs: why, what to read, which paths it owns, the steps, the acceptance list and the
checks for the handheld. Waves 0–5 stay in [../WORK_PACKAGES.md](../WORK_PACKAGES.md) as history.

**State on 2026-11-06.** The owner put Wave 6 of [../WORK_PACKAGES.md](../WORK_PACKAGES.md)
(I2, N7, V1, W7, N8, S2, F6, plus X3) into 1.2. That finished N7, N9 (Wave 6 calls it N8), W7, X3b,
part B of S2 and most of X3a here; their `Status:` lines say so. The external chase view (V1) is
in 1.2 too. The 2026-11-05 device session (the DEVICE.md §6 timing run) gave T2's frame numbers:
MED holds 30 fps in the cockpit, LOW 60 (DEVICE.md §3). The PR #11 checklist runs alongside the
1.2.0 release PR (#12), which adds the session's fixes and S2 part A (WORK_PACKAGES I3). The owner ships 1.2 now (**v1.2.0**)
and takes **X3c next as 1.2.1** (HUD CPU, the GPU clock in the log, MED toward 60). **2026-11-08:**
v1.2.0 and v1.2.1 are released. X3c (#13, 1.2.1) is device-tested: MED 59 fps in the cockpit,
HUD 3.2 ms. A small packaging PR (#14, 1.2.2) renames the launch script to `PocketVectors.sh` and
adds box art. **1.3** (2026-11-08, on `main`, the owner tags `v1.3.0` once `main` is stable): the
owner's flight director, plan-screen and runway-intersection bugs, W1b (#15, extended to smooth
crossings), U2 (#16, the flight log) and N10 (#19, fly-by arcs); planned as 1.2.3 and 1.2.4.
**Taxiways** (W8a #23, W8b #24, W8c #25; owner, 2026-11-08) are in **1.3.0** too, with a data
pack that carries them on the release; the owner device-checks `main` and tags it. The phases below are the original plan; take the open cards in that order.

- **Builders** read [BUILDER.md](BUILDER.md) and their card. Nothing else up front.
- **The orchestrator** runs `/wp-orchestrate` ([../../../.claude/skills/wp-orchestrate/SKILL.md](../../../.claude/skills/wp-orchestrate/SKILL.md)).
- **The owner** runs the device sessions (T1, T2, T3) on `main`, answers the decisions below, and
  tags releases once `main` is stable. Pull requests merge once their checks are green (owner,
  2026-11-08).

## Why the work is split this way

An agent pays for its whole context on every turn. The old reading list (CLAUDE.md, PLAN,
ARCHITECTURE, CONVENTIONS, WORK_PACKAGES and the specs) is about 170 KB, roughly 45k tokens,
and every builder carried it through every one of its turns. A card plus the sections it names is
under 10k. The planning (reading code, choosing files and anchors) happens once, when the card is
written, so builders don't have to explore.

| Role | Who | Reads | Never |
|---|---|---|---|
| Planner | The strongest model, high effort, in its own session | Code and specs, to write or refine cards | Builds features |
| Orchestrator | The strongest model, in a short session per round | This board, card `Status:` lines, branch names, PR states, builders' reports | Code, diffs, build logs, specs |
| Builder | A mid-tier coding model, background, own git worktree, no nested agents | BUILDER.md, its card, the sections the card names | Other cards, whole spec docs, Docker/device builds |
| Reviewer (optional, for `Effort: high` cards) | A mid-tier model, read-only | The PR diff and the card's acceptance list | Anything else |
| Owner | You | Reports, `main` on the handheld, releases | |

State lives in the repo (card `Status:` lines, `wp/<ID>-*` branches, PRs), not in a chat. Any
session can be thrown away and a fresh one picks up from the board.

## Phases and gates

| Phase | Cards | Runs in parallel | Gate to leave the phase | Release |
|---|---|---|---|---|
| 1 | T1 (owner), R2, P1b, X3a | P1b and X3a together; T1 any time | T1 done (tests the v1.1.0 release) | **v1.1.0** published from R2; a fix release if T1 finds a bug |
| 2 | X3b, N7, S2, X5, F5, W1b, U1 | At most 3 builders; never two cards that share a hot file (below) | Every phase-2 card merged | none |
| 2b | T2 (owner, done 2026-11-05), then X3c (refined 2026-11-07: parts A–E, one pull request) | | v1.2.0 ships first (owner, 2026-11-06); X3c meets the 30 fps floor, moves MED toward 60 and shows the flight data in the chase view | owner tags **v1.2.0**, then **v1.2.1** for X3c |
| 2c | N10 (owner, 2026-11-08: arcs between GPS legs) | alone: it shares `Autopilot.cpp` and the navigator with the 1.3 fixes | N10 merged | in **v1.3.0** |
| 3 | N8, N9, H8, W7, R3, H9, U2, T3 (owner) | Same rule as phase 2. N9, H8 and W7 are outlines: a planner session refines them first. H9 waits for F5 and X3c | T3 done | owner tags **v1.4.0** (1.3 went out early, 2026-11-08) |
| 4 | W8a, W8b, W8c (owner, 2026-11-08: taxiways; [../TAXIWAYS.md](../TAXIWAYS.md)) | one at a time: each needs the one before | W8c merged and device-checked | in **v1.3.0** (owner, 2026-11-08) |

## Cards

| ID | Title | Phase | Depends | Effort | Who | Hot files |
|---|---|---|---|---|---|---|
| [T1](T1.md) | Device session 1: confirm the post-1.0 build | 1 | none | | owner, then builder writes up | `docs/DEVICE.md` |
| [R2](R2.md) | Version 1.1.0 and release notes | 1 | none | medium | builder | `release.yml`, `CMakeLists.txt` |
| [P1b](P1b.md) | Probe prompts and tool detection in the launch log | 1 | none | medium | builder | `port/PocketVectors.sh` |
| [X3a](X3a.md) | Frame timing breakdown and a bench run | 1 | none | high | builder | `App.cpp`, `Cli.*` |
| [X3b](X3b.md) | Quality presets (wire `graphics_quality`) | 2 | X3a, T1 | high | builder | `App.cpp`, `Settings.*`, `FlightScreens.cpp` |
| [N7](N7.md) | Go-around (TOGA) and FLC | 2 | none | high | builder | `core/AutopilotState.h`, `core/Actions.h`, `controls_default.ini` |
| [S2](S2.md) | Audio: no crackle, voice callouts | 2 | none (part B: OD1) | medium | builder | `App.cpp`, `Settings.*`, `FlightScreens.cpp` |
| [X5](X5.md) | Live METAR at launch | 2 | P1b + one device launch log, OD4 | medium | builder | `port/PocketVectors.sh`, `Settings.*`, `FlightScreens.cpp` |
| [F5](F5.md) | Stall warning by flap setting | 2 | none | medium | builder | `core/AircraftProfile.h` |
| [W1b](W1b.md) | Runways flush with the terrain at KPDX | 2 | none | high | builder | `src/world/terrain/` |
| [U1](U1.md) | Quick flight and landing score | 2 | OD3 | medium | builder | `App.cpp`, `SetupScreens.cpp` |
| [T2](T2.md) | Device session 2: phase-2 build and frame numbers | 2b | phase 2 merged | | owner, then builder writes up | `docs/DEVICE.md` |
| [X3c](X3c.md) | Device tuning (HUD CPU, GPU clock, MED toward 60) and flight data in the chase view | 2b | T2 | high | builder | `Renderer2D.*`, `Hud.cpp`, `HudLogic.*`, `TerrainRenderer.*`, `App.cpp`, `FrameStats.*`, `CMakeLists.txt` |
| [N8](N8.md) | Airways in `.fms` plans | 3 | none | medium | builder | `src/nav/Fms*`, `src/nav/FlightPlan*` |
| [N9](N9.md) | SIDs and STARs (outline) | 3 | N8 | high | planner refines, then builder | `tools/nav/`, `src/nav/Procedures*` |
| [N10](N10.md) | Fly-by turn arcs between GPS legs | 2c | the 1.3 fixes (#17) | high | builder | `src/nav/Navigator*`, `core/NavState.h`, `Autopilot.cpp`, `MapPage*`, `MiniNav*`, `WorldOverlayLogic*` |
| [H8](H8.md) | Map page terrain shading and holds (outline) | 3 | X3c | medium | planner refines, then builder | `src/ui/MapPage*` |
| [W7](W7.md) | Cloud layers (outline) | 3 | X3c, X5 | high | planner refines, then builder | `src/world/Sky*`, `App.cpp` |
| [R3](R3.md) | More region packs on releases | 3 | OD2 | medium | builder | `release.yml`, `README.md` |
| [H9](H9.md) | V-speed and flap bugs on the speed tape | 3 | X3c, F5 | high | builder | `Hud.cpp`, `HudLogic.*`, `core/AircraftProfile.h`, `core/AircraftState.h`, `assets/aircraft/*.ini` |
| [U2](U2.md) | Flight log | 3 | none | medium | builder | `App.cpp`, `SetupScreens.cpp` |
| [T3](T3.md) | Device session 3: phase-3 build | 3 | phase 3 merged | | owner | `docs/DEVICE.md` |
| [W8a](W8a.md) | Taxiways from OpenStreetMap: extraction, pack format, loader | 4 | OD6 | high | builder | `tools/nav/`, `tools/datapack/`, `datapack.yml`, `src/nav/NavDb*`, `core/NavTypes.h` |
| [W8b](W8b.md) | Smooth ground under taxiways and aprons (FlattenSet grid index) | 4 | W8a, W1b | high | builder | `src/world/terrain/`, `src/world/TaxiwayGeometry.*`, `src/world/AirportGeometry.*`, `TerrainRendererLod.cpp` |
| [W8c](W8c.md) | Draw taxiways and aprons | 4 | W8b | high | builder | `src/world/TaxiwayGeometry.*`, `src/world/AirportGeometry.*`, `src/world/AirportRenderer.*`, `src/ui/HelpPages.cpp`, `src/app/Scenario.*` |

**Hot files.** Two cards in flight at the same time must not share an entry in the Hot files
column. Settings rows (`Settings.*`, `assets/settings_default.ini`, `FlightScreens.cpp`) count as
one hot file. Phase 2 can start with X3b, N7 and F5 together, then W1b, then S2, X5 and U1 as
the hot files free up. Added 2026-11-07: H9 shares `core/AircraftProfile.h` with F5 (it waits for
it anyway) and `Hud.cpp` / `HudLogic.*` with X3c; U2 shares `App.cpp` and `SetupScreens.cpp` with
U1, so those two never run together. Added 2026-11-08: N10 shares `MapPage*` with H8, and the navigator and
`core/NavState.h` with any nav card. W8a, W8b and W8c run one at a time; W8b shares
`src/world/terrain/` with W1b (done) and `TerrainRendererLod.cpp` with X3c (done), W8c shares
`AirportGeometry.*` with W8b, and W8a shares `core/NavTypes.h` and `src/nav/NavDb*` with N8 and N9.

**Status values** (first line of each card): `ready`, `owner`, `blocked (<on what>)`,
`outline (planner refines)`, `done (<PR>)`. A card is taken when an `origin/wp/<ID>-*` branch
exists. The builder sets `done` in its own pull request, so the merge marks it done.

## Owner decisions

| # | Decision | Blocks | Default if you say nothing |
|---|---|---|---|
| OD1 | Where the voice callout clips come from: your own recordings, a CC0 clip set, or offline text-to-speech whose output licence allows redistribution | S2 part B | Tones stay; S2 ships the clip player with a tone fallback |
| OD2 | Attach more prebuilt region packs (bayarea, mthood, tam_diablo) to releases | R3 | From 1.2.0 a release attaches a `pdx_oth` pack with SIDs and STARs (from a datapack run); no other regions |
| OD3 | Add Quick Flight and landing scores (a suggestion, not in PLAN.md) | U1 | U1 waits |
| OD4 | Fetch METARs over Wi-Fi at launch | X5 | Off by default, a setting turns it on |
| OD5 | Pin the GPU clock: the 1.2.1 device run read 420 MHz all session (the lowest of 420–648 MHz, `simple_ondemand` governor). The launch script runs as root and could set the devfreq governor to `performance` (or raise `min_freq`) while the game runs, at a battery and heat cost; it would likely lift the 30 fps cases under an overcast layer and in the chase orbit | a launch-script card, not yet written | Leave the governor alone |
| OD6 | Where taxiway geometry comes from: OpenStreetMap (ODbL: attribution, the taxiway layer of the pack shared under ODbL), the X-Plane Scenery Gateway (GPL), or procedural stubs only | W8 | **Answered 2026-11-08: OpenStreetMap** (PLAN.md D23, [../TAXIWAYS.md](../TAXIWAYS.md)) |

## Not planned

- DHC6 and J3Cub: skipped in F4 (the DHC6 model has no trim; the J3Cub is a tail-dragger
  without flaps).
- External view with simple models: done in 1.2 (Wave 6 V1, PLAN D5a).
- Photoreal scenery, 3D cockpits, multiplayer, ATC, failures: PLAN §2 non-goals.
