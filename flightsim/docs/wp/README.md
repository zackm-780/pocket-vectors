# Work board: after 1.0

This folder is the plan for everything after Pocket Vectors 1.0. Each work package is one **card**
(`<ID>.md`) that one agent or one person can finish in one pull request. A card carries everything
its builder needs: why, what to read, which paths it owns, the steps, the acceptance list and the
checks for the handheld. Waves 0–5 stay in [../WORK_PACKAGES.md](../WORK_PACKAGES.md) as history.

- **Builders** read [BUILDER.md](BUILDER.md) and their card. Nothing else up front.
- **The orchestrator** runs `/wp-orchestrate` ([../../../.claude/skills/wp-orchestrate/SKILL.md](../../../.claude/skills/wp-orchestrate/SKILL.md)).
- **The owner** runs the device sessions (T1, T2, T3), answers the decisions below, and approves
  pull requests.

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
| Owner | You | Reports, PRs, the handheld | |

State lives in the repo (card `Status:` lines, `wp/<ID>-*` branches, PRs), not in a chat. Any
session can be thrown away and a fresh one picks up from the board.

## Phases and gates

| Phase | Cards | Runs in parallel | Gate to leave the phase | Release |
|---|---|---|---|---|
| 1 | T1 (owner), R2, P1b, X3a | R2, P1b and X3a together; T1 any time | T1 done, R2 merged | owner tags **v1.1.0** |
| 2 | X3b, N7, S2, X5, F5, W1b, U1 | At most 3 builders; never two cards that share a hot file (below) | Every phase-2 card merged | none |
| 2b | T2 (owner), then X3c | | X3c meets the 30 fps floor | owner tags **v1.2.0** |
| 3 | N8, N9, H8, W7, R3, T3 (owner) | Same rule as phase 2. N9, H8 and W7 are outlines: a planner session refines them first | T3 done | owner tags **v1.3.0** |

## Cards

| ID | Title | Phase | Depends | Effort | Who | Hot files |
|---|---|---|---|---|---|---|
| [T1](T1.md) | Device session 1: confirm the post-1.0 build | 1 | none | | owner, then builder writes up | `docs/DEVICE.md` |
| [R2](R2.md) | Version 1.1.0 and release notes | 1 | none | medium | builder | `release.yml`, `CMakeLists.txt` |
| [P1b](P1b.md) | Probe prompts and tool detection in the launch log | 1 | none | medium | builder | `port/FlightSim.sh` |
| [X3a](X3a.md) | Frame timing breakdown and a bench run | 1 | none | high | builder | `App.cpp`, `Cli.*` |
| [X3b](X3b.md) | Quality presets (wire `graphics_quality`) | 2 | X3a, T1 | high | builder | `App.cpp`, `Settings.*`, `FlightScreens.cpp` |
| [N7](N7.md) | Go-around (TOGA) and FLC | 2 | none | high | builder | `core/AutopilotState.h`, `core/Actions.h`, `controls_default.ini` |
| [S2](S2.md) | Audio: no crackle, voice callouts | 2 | none (part B: OD1) | medium | builder | `App.cpp`, `Settings.*`, `FlightScreens.cpp` |
| [X5](X5.md) | Live METAR at launch | 2 | P1b + one device launch log, OD4 | medium | builder | `port/FlightSim.sh`, `Settings.*`, `FlightScreens.cpp` |
| [F5](F5.md) | Stall warning by flap setting | 2 | none | medium | builder | `core/AircraftProfile.h` |
| [W1b](W1b.md) | Runways flush with the terrain at KPDX | 2 | none | high | builder | `src/world/terrain/` |
| [U1](U1.md) | Quick flight and landing score | 2 | OD3 | medium | builder | `App.cpp`, `SetupScreens.cpp` |
| [T2](T2.md) | Device session 2: phase-2 build and frame numbers | 2b | phase 2 merged | | owner, then builder writes up | `docs/DEVICE.md` |
| [X3c](X3c.md) | Device tuning to a 30 fps floor | 2b | T2 | high | planner refines, then builder | per refined card |
| [N8](N8.md) | Airways in `.fms` plans | 3 | none | medium | builder | `src/nav/Fms*`, `src/nav/FlightPlan*` |
| [N9](N9.md) | SIDs and STARs (outline) | 3 | N8 | high | planner refines, then builder | `tools/nav/`, `src/nav/Procedures*` |
| [H8](H8.md) | Map page terrain shading and holds (outline) | 3 | X3c | medium | planner refines, then builder | `src/ui/MapPage*` |
| [W7](W7.md) | Cloud layers (outline) | 3 | X3c, X5 | high | planner refines, then builder | `src/world/Sky*`, `App.cpp` |
| [R3](R3.md) | More region packs on releases | 3 | OD2 | medium | builder | `release.yml`, `README.md` |
| [T3](T3.md) | Device session 3: phase-3 build | 3 | phase 3 merged | | owner | `docs/DEVICE.md` |

**Hot files.** Two cards in flight at the same time must not share an entry in the Hot files
column. Settings rows (`Settings.*`, `assets/settings_default.ini`, `FlightScreens.cpp`) count as
one hot file. Phase 2 can start with X3b, N7 and F5 together, then W1b, then S2, X5 and U1 as
the hot files free up.

**Status values** (first line of each card): `ready`, `owner`, `blocked (<on what>)`,
`outline (planner refines)`, `done (<PR>)`. A card is taken when an `origin/wp/<ID>-*` branch
exists. The builder sets `done` in its own pull request, so the merge marks it done.

## Owner decisions

| # | Decision | Blocks | Default if you say nothing |
|---|---|---|---|
| OD1 | Where the voice callout clips come from: your own recordings, a CC0 clip set, or offline text-to-speech whose output licence allows redistribution | S2 part B | Tones stay; S2 ships the clip player with a tone fallback |
| OD2 | Attach more prebuilt region packs (bayarea, mthood, tam_diablo) to releases | R3 | Releases link the 1.0 `pdx_oth` pack only |
| OD3 | Add Quick Flight and landing scores (a suggestion, not in PLAN.md) | U1 | U1 waits |
| OD4 | Fetch METARs over Wi-Fi at launch | X5 | Off by default, a setting turns it on |

## Not planned

- DHC6 and J3Cub: skipped in F4 (the DHC6 model has no trim; the J3Cub is a tail-dragger
  without flaps).
- External view with simple models: PLAN §2 "Later", not scheduled.
- Photoreal scenery, 3D cockpits, multiplayer, ATC, failures: PLAN §2 non-goals.
