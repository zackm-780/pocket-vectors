# Pocket Vectors: handheld review, 2026-11-10

A read-only review of `main` at `0e69d77` (75 commits past `v1.2.1`) for the Anbernic RG40XXH:
UX, UI, performance, security and reliability. No code was changed.

**How to read the evidence.** Each finding carries one of three marks:

- **[V]** I read the code at the cited lines myself.
- **[T]** An exploration pass traced it and cited lines; I did not re-read every line.
- **[I]** Inference or estimate. Nothing in the repo measures it.

**What was and was not run.** See [What I ran](#what-i-ran) at the end. Nothing here was measured
on the device, and the data packs (`pdx_oth`, `sea_sfo`) were not available, so every
millisecond figure for the Mali-G31 is an estimate to be confirmed with the perf table.

**Two corrections to the brief.**

- The smallest text is 9 px cap height, which on a 4.0" 640×480 panel (200 ppi, 0.127 mm per
  pixel) is about **1.1 mm**, not 1.4 mm. That is roughly 4.6 pt at the 30–35 cm a handheld is
  held at. It is smaller than the brief assumed.
- There is no `--bench` flag. `Cli.cpp` has no such option, and card X3a still lists it as left
  to do. The performance plan below measures with `show_fps`, the perf table and
  `perf_gpu_sync` instead, and recommends finishing `--bench`.

---

## 1. Executive summary: the top 10

1. **Show a loading screen and stop freezing.** The nav database load, a full JSBSim init of a
   default flight at XTST that the menu then throws away, every shader compile and up to four
   10-second terrain waits all happen before the first `swap()`. Pressing START FLIGHT blocks
   the frame for up to 10 s with no redraw and no event polling. Create the window first, draw
   `LOADING`, skip the throwaway flight, and pump frames during the spawn gate. (R1, R2)
2. **Take the cheap GPU fill cuts before touching the governor.** Draw the sea disc after the
   terrain, skip the sky pass when an overcast layer covers it, draw an opaque overcast layer
   without blending, and stop rendering the 3D world behind full-screen menus. These are small
   changes aimed exactly at the two measured 30 fps cases. (P1–P4)
3. **Add a frame-rate setting (30 or 60) and make MSAA a setting.** With vsync only, a frame
   that costs 17–33 ms flips between 30 and 60, which looks worse than a steady 30. Because 30
   is the bar, a locked 30 is also the largest battery and heat saving available. MSAA 4× is
   requested on every preset and no setting controls it. (P5, P6)
4. **Make the game hard to kill.** There is no top-level `try/catch`, `fdm->Run()` is unguarded
   in the 120 Hz step, there is no NaN check on the flight state, an allocation failure on the
   tile worker terminates the process, and the voice bank is destroyed before the audio device
   closes. (R3–R6)
5. **Fix legibility.** Most flight data is drawn at 9 px (1.1 mm). Add a `TEXT SIZE` setting
   that maps the small size to 12 px, and give the flap bugs a prefix (`F20`) so they stop
   looking like tape numerals. (U1, U2)
6. **Tell the player when something went wrong.** Settings and flight-log write failures, the
   synthetic-world fallback when no data pack loads, the spawn-gate timeout, a missing METAR
   file and a disconnected controller reach only `log.txt`. The settings screen says `SAVED`
   whatever happened. Add one status line to the menu frame and use it. (U3–U6)
7. **Cut the HUD's CPU cost at its source.** All text is a stroke font tessellated twice
   (outline and colour) every frame. Cache a vertex template per glyph and size, and build the
   three tapes once as static strips that scroll with a uniform. (P7)
8. **Make quit, Start and "resume" consistent.** Select+Start quits a flight in progress with no
   confirmation. Start on the new-flight hub starts a flight at once, even when the hub was
   opened from the pause menu where Start means resume. `RESUME LAST FLIGHT` repeats the last
   start, not the last position. (U7–U9)
9. **Harden the file loaders and add sanitizers to CI.** I found no memory-corruption bug: the
   parsers are careful. The gaps are unbounded allocation, a quadratic hang in the INI reader,
   an uncapped runway length that can allocate hundreds of megabytes, and a launch-script
   library path that includes the current directory. CI has no ASan, UBSan or fuzzing, and
   nothing enforces the SDL 2.0.12 ceiling. (S1–S9, T1–T5)
10. **Fix release hygiene, then build N11b.** The board says v1.3.0 is released, the remote has
    no such tag, and the version never reaches `log.txt`. N11b is marked blocked on a card that
    merged. The map page runs the sim with no speed or altitude on screen. (C1–C4, U10)

---

## 2. Findings table

Severity is for the handheld player. Effort: S is under a day, M is a few days, L is a week or
more. Paths are under `flightsim/`.

### Performance

| ID | Sev | Finding | Evidence | User impact | Fix | Effort | Risk |
|---|---|---|---|---|---|---|---|
| P1 | High | The sea disc (500 km radius, the full terrain fragment shader) is drawn before the terrain nodes, so land pixels are shaded twice. Nodes are sorted front to back, but the sea goes first and nothing is in the depth buffer to reject it. | `src/world/TerrainRenderer.cpp:541-550` [V]; shader cost `:113-145` [T] | Wasted fill on every frame over land | Draw the sea after the node loop. The polygon offset already pushes it behind, so the picture is the same. | S | Low: check coastlines for z-fighting |
| P2 | High | Under an overcast layer the sky shader (highp, `normalize`, `sqrt`, `exp`, three smoothsteps) runs for pixels the layer then covers completely. Overcast alpha is always 1 but is still blended. Up to three layers are drawn, each blended. | Order: `src/gfx/DebugRenderer.cpp:268-269`, `src/app/App.cpp:653` [V for App]; alpha `src/world/CloudsLogic.cpp:255-258`; blend `src/world/Clouds.cpp:220-221` [T] | The measured 30–40 fps under overcast | Skip `Sky::draw` when an opaque layer hides it; draw OVC with blending off; drop layers hidden behind a nearer opaque one | S–M | Low: check the horizon join by eye |
| P3 | High | The main menu and every full-screen menu still render terrain and airports at full cost, at vsync rate. Only clouds, model and overlays are skipped. | `src/app/App.cpp:646-653` [V] | Battery and heat while the player reads a menu | When the menu is full-screen, clear and draw only the menu. Cap menus at 30 fps. | S | Low |
| P4 | Med | The terrain fragment shader is all highp, normalises and lights per fragment, and computes fog `exp` per fragment. Night-light code is compiled into the day shader behind a runtime branch. | `src/world/TerrainRenderer.cpp:56-60`, `:113-145`; `src/world/Atmosphere.cpp:37-43` [T] | The base cost of every terrain pixel | Move lighting and the fog factor to the vertex shader; use mediump for colour math; build day and night variants | M | Med: banding in fog, check on device |
| P5 | High | Pacing is vsync only. There is no frame limiter and no 30 fps option. A swap-interval failure is logged at debug level and the loop then runs uncapped. | `src/platform/Window.cpp:108-109` [V]; no `SDL_Delay` in `src/` [T] | Judder between 30 and 60 when a frame costs just over 16.7 ms; no way to trade frame rate for battery | Add `FRAME RATE 30/60`: try swap interval 2, fall back to a sleep-based limiter | S–M | Med: swap interval 2 is untested on the device driver |
| P6 | Med | 4× MSAA is requested first on every preset. No setting, flag or preset controls it. The device fill-rate figures were taken without MSAA. | `src/platform/Window.h:19`, `Window.cpp:53-54` [V]; `docs/DEVICE.md:24,31` [T] | Unknown fill cost on a fill-bound GPU | Add `ANTIALIASING OFF/4X` and an env override; measure both at MED under overcast; pick the default from the numbers | S | Low |
| P7 | Med | HUD: every glyph is a stroke path, tessellated to quads with joins, emitted twice (outline and colour), and CPU-clipped, every frame. Nothing is cached. Estimated 25–45k vertices per frame against a plan budget of 4k line quads. The vertex count is tracked but never logged. | `src/hud/Renderer2D.cpp:640-729`, `:848-887`, `:430-476`; `src/hud/StrokeFont.cpp:17-80` [T] | 3.2 ms against a 2 ms budget | See the performance plan: glyph templates, static tape strips, log the count | M | Low–Med: pixel-compare screenshots |
| P8 | Med | About 13 GL calls per terrain node for about 90 nodes: five uniforms, a buffer bind, four attribute pointers. The device doc says draws cost more than triangles here. | `src/world/TerrainRenderer.cpp:360-379` [T]; `docs/DEVICE.md:31` | CPU time in the driver every frame | Use `OES_vertex_array_object` (listed as available in DEVICE.md) and merge the per-node uniforms | M | Low |
| P9 | Med | Airport meshes, including OSM taxiways, are built on the main thread when the active set changes, with no time cap. It then clears and re-registers every flatten area, which queues terrain rebuilds. | `src/world/AirportRenderer.cpp:196-264`, `:306-320` [T] | A hitch when approaching a large airport such as KPDX | Build the CPU mesh on the worker and upload on the main thread; re-register only the airports that changed | M–L | Med: threading the nav data |
| P10 | Low | The tile worker pops with `queue_.erase(queue_.begin())` on a vector of up to 20,000 entries, under the lock. | `src/world/terrain/TileLoader.cpp:77` [T] | Small main-thread stalls while the worker holds the lock | Use a deque or an index | S | Low |
| P11 | Low | Cloud UVs are mediump varyings reaching about 27 tile units. | `src/world/Clouds.cpp:34-46`; `src/gfx/Shader.cpp:198` [T]; effect [I] | Possible swimming or blocky cloud texture | Make the UV varyings highp or wrap them in the vertex shader | S | Low |
| P12 | Low | The audio callback keeps synthesising while paused. | `src/app/App.cpp:601` [T] | A little CPU in menus | `SDL_PauseAudioDevice` after the fade-out | S | Low |

### UX, UI and accessibility

| ID | Sev | Area | Finding | Evidence | User impact | Fix | Effort | Risk |
|---|---|---|---|---|---|---|---|---|
| U1 | High | A11y | Three text sizes: 9, 12 and 16 px cap height. Tape numerals, the nav box, bug labels, armed modes, the whole map page, help text and every footer use 9 px (1.1 mm). No scale setting exists. Status row 1 silently drops from 12 to 9 px when it overflows. | `src/hud/StrokeFont.cpp:193-198`; `src/hud/Hud.cpp:384,455,353,580`; `src/hud/NavHud.cpp:142` [T] | The primary numbers are hard to read on the target screen | `TEXT SIZE NORMAL/LARGE`, where LARGE maps S to 12 px; lay out the tapes for it | M | Med: layout overlap, needs screenshots |
| U2 | Med | UI | Flap bugs are labelled with the bare detent name (`10`, `20`, `30` on the C172), in the same size and colour as the tape numerals, a few pixels from them. The chase readout prints `5 178`. | `src/hud/Hud.cpp:318-356` [V, and seen in a screenshot]; `src/hud/SpeedBugs.h:37`; `assets/aircraft/c172p.ini:16` [T] | A flap label reads as a speed | Prefix flap bugs with `F` and put the label in a tag shape; V-speed bugs keep their names | S | Low |
| U3 | High | UX | The settings screen always shows `SAVED WHEN YOU LEAVE THIS SCREEN`; a failed save is a log line. The same holds for the pause toggles and the save at exit. | `src/ui/FlightScreens.cpp:182-196` [V] | On a full or read-only card the player's settings vanish with no warning | Return the result, show `NOT SAVED: CARD READ-ONLY OR FULL` on the menu status line | S | Low |
| U4 | High | UX | With no nav or terrain pack, or a pack that fails to load, the game silently uses a synthetic one-airport flat world. The only mention is on the About page. | `src/app/App.cpp:835-860` [V] | A bad install looks like a broken game | A banner on the main menu: `NO DATA PACK: TEST AIRPORT ONLY`, with the reason | S | Low |
| U5 | Med | UX | Low fuel is a 2-second white message. At the default declutter level (STD) fuel and endurance are hidden, and there is no fuel warning. `HUD.md` says 1.5 s; the code says 2.0 s. | `src/hud/HudLogic.cpp:90-110`; `src/app/App.cpp:566`; `src/hud/HudLogic.h:42-53,281` [T] | Fuel exhaustion with one easy-to-miss cue | Show endurance at STD when under 30 min; add a persistent amber `FUEL` caution | S | Low |
| U6 | Med | UX | `WEATHER FROM METAR` defaults on and does nothing without a file. No message says which weather is in use. | `src/world/Weather.cpp:140-152`; `src/app/Simulation.cpp:70-72` [T] | A setting that appears broken | Post `WX: METAR KPDX` or `WX: DEFAULT (NO METAR FILE)` at flight start; say so in the setting's value | S | Low |
| U7 | High | UX | Select+Start quits from anywhere, including mid-flight, with no confirmation, while both menu quits ask. `gptokeyb` runs the same hotkey and can kill the game before it saves. | `src/app/App.cpp:450` [V]; `assets/controls_default.ini:151`; `port/PocketVectors.sh:129-130` [V] | One accidental chord ends a long flight | In flight, require a one-second hold and show `HOLD TO QUIT`; keep the instant quit in menus | S | Low |
| U8 | Med | UX | Start means resume in the pause menu, start-the-flight on the new-flight hub (no confirmation, also when opened from pause), save-and-back in settings, and cancel in a dialog. Footers use SELECT and START as both verbs and button names. | `src/ui/SetupScreens.cpp:128,151,178`; `src/ui/FlightScreens.cpp:182,290` [T] | Replacing a flight in progress by habit | Confirm when a flight is active; write footers as `A OK  B BACK  START FLY` | S | Low |
| U9 | Med | UX | `RESUME LAST FLIGHT` restores the chosen start, not the position. After QUIT TO MENU it still points at the previous session's flight. | `src/app/Settings.h:99-103` [T]; `src/ui/SetupScreens.cpp:79,115` [T] | The label promises something else | Rename to `REPEAT LAST FLIGHT`; re-read the file when the main menu opens | S | Low |
| U10 | High | UX | The map page replaces the HUD and the sim keeps running. Only track and ground speed are shown: no airspeed, altitude, warnings or messages. A on a waypoint is an immediate Direct-To. | `src/app/App.cpp:436-449` [V]; `src/ui/MapPage.cpp:404-441,63-66` [T] | The player can stall or hit terrain while reading the map | A strip with IAS, ALT, VS and the active warning on the map, or an option to pause under it | S–M | Low |
| U11 | Med | A11y | With the HUD colour set to amber or white, cautions (amber) or advisories (white) look identical to normal text. Speed bands, endurance, speed-brake and reverse states are colour-only. | `src/hud/Hud.cpp:40-58,369-372`; `src/hud/HudLogic.cpp:81-88,512-553` [T] | Colour-blind players, and anyone using the amber HUD, lose the caution channel | Prefix cautions with a triangle glyph and warnings with a filled box; hatch the red band | M | Low |
| U12 | Med | UX | No first-run guidance. Help and the button test sit under HELP on the main menu and row 8 of the pause menu; no footer mentions them. | `src/ui/SetupScreens.cpp:99-128` [T] | A new player meets a HUD with no controls hint | On first launch (no `settings.ini`), open a one-page controls card; add `Y HELP` to the main footer | S | Low |
| U13 | Med | UX | The on-screen keyboard has no clear key, B does not auto-repeat, and Start does nothing on `NO MATCHES`, so leaving a dead four-letter query takes about five presses. No recents or favourites. | `src/ui/OnScreenKeyboard.cpp`; `src/ui/SetupScreens.cpp:352-507`; `assets/controls_default.ini:141` [T] | Slow, slightly trapping airport entry | L1 clears, B repeats, Start cancels on no hits; a `RECENT` list above `NEAREST`; grey out letters with no match | M | Low |
| U14 | Low | UI | `fitText` cuts with `substr` on bytes: no ellipsis, and a cut inside a multi-byte glyph draws a box. Dialog text does not wrap and can run off the box. | `src/ui/UiPainter.cpp:454-458,555-590` [T] | Clipped names and unreadable long errors | Ellipsis, code-point-aware cut, wrap dialogs to two or three lines | S | Low |
| U15 | Low | UX | Settings is one flat list of 28 rows, 12 visible, with no paging, grouping, position indicator or reset. Labels such as `EXPO` and `OVERLAY PATH` are unexplained. | `src/ui/FlightScreens.cpp:40-161`; `src/ui/UiPainter.h:398` [T] | Scrolling to find a setting | Group under headers, L1/R1 pages, a one-line description of the selected row | M | Low |
| U16 | Low | UX | A controller disconnect centres the controls and the flight carries on, with a log line only. | `src/platform/InputSystem.cpp:124-149` [T] | A lost flight if a pad drops | Open the pause menu when the last controller is removed | S | Low |

### Reliability

| ID | Sev | Finding | Evidence | User impact | Fix | Effort | Risk |
|---|---|---|---|---|---|---|---|
| R1 | Critical | Before the first presented frame: settings, nav DB load, terrain pack open, JSBSim construction, a full `sim.init` of the default flight (with a spawn gate of up to 10 s, a trim and a 2 s settle), then the window, about 12 shader programs, audio, and up to four `waitForIdle(10.0)` prewarm passes. When the game starts in the menu, that first flight is discarded. | `src/app/App.cpp:801-884` and `:403-414`, first swap `:745` [V]; `src/app/Simulation.cpp:45`; `src/sim/FlightModel.cpp:467` [T] | A black screen at launch with no sign of life; the 15 s menu-to-flying budget has no device number | Window first, draw `LOADING`, skip `sim.init` when `startInMenu`, load the nav DB on a thread and join before the menu | M | Med: `Simulation` assumes an initialised flight |
| R2 | High | START FLIGHT calls `sim.init` inside the frame. The menu freezes for up to 10 s with no redraw and no event polling, so queued presses replay afterwards. On timeout the flight starts on coarse ground and the log says "ground may jump". | `src/app/App.cpp:939-955` [V]; `src/app/SpawnGate.cpp:32-56`; `src/app/Simulation.cpp:47-49` [T] | Looks like a hang; mashed buttons act on the new flight | Draw a `LOADING` frame before the call at minimum; better, run the gate across frames; flush the event queue after | S (frame) / M (async) | Low / Med |
| R3 | High | No `try/catch` in `main` or `App::run`. `FlightModel::step` calls `fdm->Run()` unguarded and ignores its result. | `src/main.cpp` (no try), `src/sim/FlightModel.cpp:793-802` [V] | Any JSBSim exception or `bad_alloc` ends the game with no message and no settings save | Catch at the top, save, show `SIMULATION ERROR` and return to the menu | S | Low |
| R4 | High | No finite check on the flight state. `AircraftState.valid` is set unconditionally. | `src/sim/FlightModel.cpp:627`; `isfinite` only in `GroundCallback.cpp:17` [T] | A NaN from JSBSim reaches the camera, HUD and terrain selection; the flight is unrecoverable | Check position, attitude and speeds in `refresh`; on failure end the flight with a message | S | Low |
| R5 | High | `voiceBank` is declared after `audioEngine`, so it is destroyed first while the audio callback can still read its clips. | `src/app/App.cpp:298-308` [V]; `src/audio/AudioEngine.cpp:27-28` [V] | A possible crash at exit when a voice callout is playing | Call `audioEngine.shutdown()` before leaving, or swap the declarations | S | Low |
| R6 | Med | `snapToParams()` runs on the main thread after the device is unpaused and writes smoothing state the callback also writes. The comment calls it harmless. | `src/audio/AudioEngine.cpp:95-98` [T] | A click at worst; undefined behaviour formally, and TSan will flag it | Set a flag under the existing mutex and let the callback consume it | S | Low |
| R7 | Med | `log.txt` is opened with mode `w`, so each launch erases the last session's log. The version and commit are not written to it. | `src/core/Log.cpp:42`; `src/ui/HelpPages.cpp:334-335` [T] | A crash report is gone once the player relaunches to check | Rotate to `log.prev.txt`; make the version the first line | S | Low |
| R8 | Med | Nothing reports memory use. The 350 MB budget is unverified, and tiles held by the renderer sit outside the 128 MB cache cap. | `src/world/terrain/TerrainService.h:40-41,72` [T] | Risk of the OOM killer on a 1 GB device with `sea_sfo` | Log `VmHWM` from `/proc/self/status` in the perf session line and at exit | S | Low |
| R9 | Low | `appendFlightLog` appends without `fsync`; the atomic writer does not sync the directory. | `src/app/FlightLog.cpp:239-254`; `src/app/Settings.cpp:394-418` [T] | A lost log row on power loss; the 60 s checkpoint already limits it | `fsync` after the append | S | Low |
| R10 | Low | Profile values such as `bankLimitDeg = 0` divide by zero in the autopilot; `left_deadzone = 1` divides by zero in the input curve. | `src/autopilot/Autopilot.cpp:572,621`; `src/platform/InputCurves.h:25,37` [T] | Only with an edited file | Clamp at load | S | Low |

### Security and integrity

| ID | Sev | Finding | Evidence | User impact | Fix | Effort | Risk |
|---|---|---|---|---|---|---|---|
| S1 | Med | `LD_LIBRARY_PATH="$GAMEDIR/libs.${DEVICE_ARCH}:$LD_LIBRARY_PATH"` ends in a colon when the variable was empty. An empty entry means the current directory, and the script has already done `cd "$GAMEDIR"`. | `port/PocketVectors.sh:124` and `:97` [V] | A stray `libSDL2-2.0.so.0` in the game folder is loaded as root ahead of the system copy | `${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}` | S | Low |
| S2 | Med | Files are read whole before any size check: `readWholeFile` resizes to the file size, `MagVar::loadFile` reads everything, CSV lines and `.fms` files have no limit. On the worker thread a failed allocation terminates the process. | `src/world/terrain/TerrainPack.cpp:16-31`; `src/nav/MagVar.cpp:20-33`; `src/core/Csv.cpp:26`; `src/nav/Fms.cpp:239` [T] | A corrupt or hostile pack crashes or exhausts memory | Check the size against the expected or a cap before allocating; cap `.fms` at a few hundred waypoints | S | Low |
| S3 | Med | Runway length is taken from the CSV with no clamp, and edge lights are generated per interval with no cap. A lighted runway with identical ends and a huge `length_ft` allocates hundreds of megabytes when it comes into range. | `src/world/AirportGeometry.cpp:137,371-372,408-421` [T, not run] | Out of memory near that airport | Clamp length, width and displaced threshold at load; cap light counts | S | Low |
| S4 | Low | The INI reader scans all sections linearly for every header, so a file of many distinct sections is quadratic. | `src/core/Ini.cpp:55-65` [V, triggered: see What I ran] | A 1 MB settings or scenario file stalls the launch for tens of seconds | Index sections in a map; cap the file size | S | Low |
| S5 | Low | Nav CSV numbers accept `nan`, `inf` and out-of-range coordinates (the taxiway path is stricter). `Ini::getDouble` accepts them too, so `distance_nm = nan` places a start at NaN. | `src/nav/NavDb.cpp:31-42`; `src/core/Ini.cpp:108-114`; `src/app/Scenario.cpp:130` [T] | Feeds R4 | Reject non-finite values and range-check latitude and longitude | S | Low |
| S6 | Low | The weather path is built from the airport ident with no separator check (procedures do check). Writes follow symlinks. | `src/world/Weather.cpp:148`; `src/nav/Procedures.cpp:~515` [T] | Reads an arbitrary `.txt` as a METAR; no disclosure beyond parsed values. Low on FAT cards. | Reuse the procedures ident check | S | Low |
| S7 | Med | Release zips have no checksum, signature or provenance. The data packs attached to a release come from hard-coded workflow run IDs with no check of the ref that built them. | `.github/workflows/release.yml:54-60,105` [T] | A player cannot verify a download; a pack built from any branch could ship | Publish `SHA256SUMS`, add `actions/attest-build-provenance`, record and check the pack's commit | S | Low |
| S8 | Low | JSBSim is fetched by tag (`v1.3.3`); the Docker base is a tag; actions are pinned by tag. doctest and stb are pinned by hash. Data downloads have no hash check. | `cmake/Dependencies.cmake:11-15,38-45`; `.github/workflows/*.yml` [V for actions] | Supply-chain drift | Pin JSBSim to the commit SHA, the image to a digest, actions to SHAs; add Dependabot for actions | S | Low |
| S9 | Low | No hardening flags, and `check_package.sh` does not check for them or for RPATH. | `CMakeLists.txt:35-37`; `port/check_package.sh` [T] | Less margin if a parser bug appears | `-D_FORTIFY_SOURCE=2 -fstack-protector-strong -fPIE -pie -Wl,-z,relro,-z,now`; assert no RPATH | S | Low: confirm no measurable frame cost |

**What held up.** No unsafe string call exists in `src/` (no `sprintf`, `strcpy`, `gets`, or
`sscanf("%s")`). Ident copies are bounded and terminated (`Navigator.cpp:64-66` [V]). The magvar,
tile and WAV loaders check sizes before indexing. File-derived indices are range-checked or
clamped at use. The `.fms` parser rejects non-finite numbers and range-checks coordinates. The
binary calls no `system`, `popen`, `exec`, socket or `dlopen`, which confirms the no-network
rule by grep. The 7-character ident truncation is silent but safe. [T]

### Code health and process

| ID | Sev | Finding | Evidence | Fix | Effort |
|---|---|---|---|---|---|
| C1 | High | Version drift, and the docs disagree with each other. CMake says 1.3.0; the remote's newest tag is `v1.2.1`; `docs/wp/README.md` says "v1.3.0 is released and device-checked (2026-11-09)"; `ROADMAP.md` says it will be tagged once stable. The changelog has no 1.4 section although F5, H9, H10 and N11a are merged. `INSTALL.md` still names 1.0.0 zips. | `CMakeLists.txt:2`; `git ls-remote --tags origin` [V]; `docs/wp/README.md` [V] | Decide whether `v1.3.0` is tagged at the device-checked commit; bump `main` to `1.4.0-dev`; CI fails a tag that differs from the CMake version; open an `Unreleased` changelog section | S |
| C2 | Med | Nothing enforces the SDL 2.0.12 ceiling. Every call in use today exists in 2.0.12 [T], but the build image carries newer headers, so a newer call would compile and fail only on the device. | no version guard in `src/`, CMake or workflows [T] | A device-workflow step: `nm -D` the binary and compare undefined `SDL_*` symbols with a 2.0.12 allowlist | S |
| C3 | Low | N11b says `blocked (N11a)`; N11a merged as #33 and H10 as #32. T3's blocker is also met. | `docs/wp/N11b.md:1` [T] | Set N11b to `ready` | S |
| C4 | Low | `sim_hz` is parsed, clamped and saved, and used nowhere. `App.h:9` still calls the state machine a stub. The save-settings block is repeated four times. | `src/app/Settings.cpp:123,178,216,290`; `src/app/App.h:9`; `src/app/App.cpp:505,513,965` [T] | Remove `sim_hz` (keep reading the key so old files load); fix the comment; one `saveSettings()` helper that returns the result (also serves U3) | S |
| C5 | Low | `App.cpp` (972 lines) holds a single 680-line `runWindowed` with about 30 locals captured by lambdas. `Navigator.cpp` is 1,016 lines. | `wc -l` [V] | Split `runWindowed` into init, frame update and frame render over a context struct. Do it with R1, which needs the same seams. | M |
| C6 | Low | The X/Y mapping is still unconfirmed on the device (ROADMAP known issues). R3 is correctly unbound. | `assets/controls_default.ini:44-50,100` [T] | One button-test run on the device | S |

---

## 3. Performance plan

**Framing.** The bar is a stable 30 fps. On the 1.2.1 device run the cockpit holds about 59, an
overcast layer gives 30–40 and the chase orbit 30. So the floor is already met, narrowly, and
the real defects are: (a) no margin under overcast and in the orbit, (b) judder when the frame
time sits between one and two refresh intervals, and (c) power spent where it buys nothing.
The plan is ordered by gain per unit of risk.

All GPU figures below are **estimates [I]** for the Mali-G31 at 420 MHz, derived from the
device doc's fill numbers. Each has a measurement so the estimate can be replaced.

| Step | Change | Expected effect | How to measure |
|---|---|---|---|
| 0 | **Instrument first.** Log HUD vertices (`hud2d.stats().vertices`), `VmHWM`, and nodes built per frame in the perf line. Finish `--bench` from X3a: a fixed camera path over fixed scenes (cockpit clear, cockpit OVC, chase orbit, main menu) that prints the perf table and exits. | None by itself. It turns every row below into a number. | The bench is the measurement |
| 1 | P3: no 3D world behind full-screen menus, 30 fps cap in menus | Menu GPU load falls to almost nothing. Battery and heat only. | Perf table, scene `Menu`: terrain and airport passes read 0 ms |
| 2 | P1: sea after terrain | 0.5–2 ms of GPU time over land, nothing over open sea | `perf_gpu_sync = 1`, terrain pass, inland scenario, before and after |
| 3 | P2: skip the sky under an opaque layer, no blending for OVC, cull hidden layers | 2–4 ms under overcast. This is the change most likely to lift the 30–40 fps case to a steady 60, or to give a locked 30 real margin. | Scene `Cockpit`, sky `Layer`, in the perf table |
| 4 | P6: MSAA setting, measured | Unknown. Mali resolves MSAA in the tile, so the cost is usually modest, but the fill figures in DEVICE.md were taken without it. Decide from the number. | Same scenes with `ANTIALIASING` off and on |
| 5 | P5: `FRAME RATE 30/60` | At 30: frame-time variance drops to near zero in every scene that today wanders between 30 and 60, and CPU and GPU work per second halve. | Frame-time histogram in the perf session log; battery drain over a 30-minute flight |
| 6 | P7: HUD glyph templates and static tapes | 3.2 ms to about 1.2–1.8 ms of CPU | Perf table, HUD pass; vertex count from step 0 |
| 7 | P4: terrain shader to per-vertex lighting and fog, mediump colour | 10–25% off the terrain fragment cost | Terrain pass with `perf_gpu_sync` |
| 8 | P8: VAOs and fewer per-node uniforms | 0.3–0.8 ms of CPU in the driver | Terrain pass CPU time without GPU sync |
| 9 | P9: airport mesh build off the main thread | Removes a one-off hitch near large airports | Worst frame time in the session log on a KPDX approach |

### The leads in the brief, answered

**MSAA on device.** It should be a setting, not a constant, and it should not be tied to LOW
alone until measured. LOW already holds 60 in the cockpit, so the place the decision matters is
MED under overcast. My expectation [I] is that 4× costs a few percent of frame time in clear
air and more in the blended cloud and HUD passes. If step 3 lands first, MSAA may be
affordable everywhere. Edge aliasing on thin HUD strokes is not at stake: the HUD strokes carry
their own outline.

**GPU governor (OD5).** I recommend **not** pinning it by default.

- The floor is met today without it. Steps 2 and 3 attack the same two scenes for no power cost.
- A governor that sits at its lowest step while the GPU is the bottleneck suggests its load
  metric is not seeing the work. That is worth one diagnostic before any pinning: log
  `/sys/class/devfreq/*/load` (or `cur_freq` and `trans_stat`) beside the frame time.
- If, after steps 2–5, overcast or the orbit still falls below 30, add an **opt-in**
  `PERFORMANCE MODE` setting that the launch script applies and restores on exit with a `trap`.
  Raise `min_freq` one step before reaching for the `performance` governor.
- Pinning from the launcher also widens what the script does as root. Keep it opt-in for that
  reason as well.

**HUD at 3.2 ms.** The time is in tessellation, not in upload or draw: one draw call, no
per-frame allocation, no `glGetError` [T]. Three cuts, in order:

1. *Glyph templates.* For each (glyph, size) build the outline and colour vertices once. Drawing
   text becomes a copy with an offset. Only the pitch-ladder labels rotate, and they can keep the
   current path. This removes `preparePath`, the per-segment `sqrt` and the join maths for
   nearly all text.
2. *Static tape strips.* The speed, altitude and heading tapes are translations of a fixed
   strip. Build each once into a static buffer and scroll it with a uniform offset under
   `glScissor`. That costs three more draws (the draw budget has room) and removes the CPU
   clipper for the tapes.
3. *Static furniture.* Bank scale, boresight, value-box frames and panels go in the same static
   buffer.

Smaller follow-ups: indexed quads (four vertices, not six), no outline pass on 9 px text if U1
raises that size anyway, `GL_STREAM_DRAW`, and dropping the redundant buffer orphan in front of
the ring of four buffers.

**Overcast.** Fix the order and the blending first (step 3). A lower-resolution offscreen cloud
pass is the next lever and also the general one: rendering the whole 3D scene to a 480×360
target and the HUD at native resolution cuts scene fill by 44% and makes MSAA cheap through
`EXT_multisampled_render_to_texture` (listed in DEVICE.md). That is an L-sized change with real
risk (framebuffer objects are not in `GlApi` today), so hold it until the cheap steps are
measured. Dithered alpha instead of blending is not worth it on a 200 ppi panel under MSAA.

**ES 3.** Not now. SDL 2.0.12 can request an ES 3.0 context, so the ceiling does not forbid it,
but every shader would need porting to `#version 300 es`, the desktop GL 2.1 fallback would
diverge, and the two things ES 3 would buy here are both available as ES 2 extensions on this
driver: vertex array objects (`OES_vertex_array_object`) and multisampled render-to-texture.
`highp` in fragment shaders is already available and already used; the gain is in using
`mediump` more. Instanced terrain would turn about 90 draws into a handful, but it is a rewrite
of the node mesh path and belongs after everything above.

**Moving work off the main thread.** In order of value and safety:

1. The nav database load, in parallel with window creation and shader compiles, joined before
   the menu is built. It is read-only after load. (R1)
2. Airport and taxiway CPU mesh builds, on the existing worker with a main-thread upload. (P9)
3. Terrain node mesh builds, the same way. Today they cost 0.4–0.7 ms each, up to four per
   frame on MED [T]. This is probably what the chase orbit pays for [I]: orbiting sweeps the
   view across nodes with no cached mesh. Confirm with the `built` and `queue` fields before
   doing it.

JSBSim, the navigator and the autopilot should stay where they are: 0.35 ms is not a problem.

**Startup.** See R1 and R2 and the mockup in section 4.

**RSS against 350 MB.** There is no measurement. A code-based estimate [I]: terrain cache up to
128 MB, plus tiles the renderer still holds, plus up to 1,600 node meshes at about 11 KB
(18 MB, in system RAM on Mali), nav data 30–50 MB for a full pack, JSBSim about 20 MB, and the
driver and SDL. That puts a long `sea_sfo` flight at roughly 250–330 MB, close to the budget
with no instrumentation to catch a breach. Step 0's `VmHWM` line settles it in one session.

---

## 4. UX and UI redesign notes

### 4.1 Launch and loading

Today the player sees black from launch to menu, and a frozen menu from START FLIGHT to the
flight. Both get the same screen:

```
+--------------------------------------------------------------+
|                                                              |
|                                                              |
|                       POCKET VECTORS                         |
|                          1.4.0                               |
|                                                              |
|                                                              |
|              LOADING NAVIGATION DATA                         |
|              [##########..............]                      |
|                                                              |
|                                                              |
|                                                              |
|   PACK: SEATTLE TO SAN FRANCISCO                             |
+--------------------------------------------------------------+
```

Stages: `NAVIGATION DATA`, `SHADERS`, `TERRAIN`. For a flight start: `AIRCRAFT`, then
`TERRAIN AT KPDX`. If the spawn gate times out, the flight starts with an amber
`TERRAIN STILL LOADING` message that clears when detail arrives, instead of a log line.

### 4.2 Main menu

Keep the six rows. Add a status line (the missing error channel), the data pack name and a help
hint. The status line is where U3, U4 and flight-log failures appear.

```
+--------------------------------------------------------------+
|  POCKET VECTORS                                      1.4.0   |
|--------------------------------------------------------------|
|                                                              |
|   > REPEAT LAST FLIGHT                                       |
|       C172P  KPDX 28L  RUNWAY                                |
|     NEW FLIGHT                                               |
|     FLIGHT LOG                                               |
|     SETTINGS                                                 |
|     HELP AND CONTROLS                                        |
|     QUIT                                                     |
|                                                              |
|                                                              |
|  ! SETTINGS NOT SAVED: CARD IS READ-ONLY                     |
|--------------------------------------------------------------|
|  A OK     Y HELP                    PACK: PDX TO OTH         |
+--------------------------------------------------------------+
```

The flow depth is fine. A repeat flight is one press and a default new flight is two (A, then
Start on the hub), which is right for a handheld. The hub is the correct pattern: eight rows
with sensible defaults, not a wizard. What it lacks is discoverability of that Start shortcut
for a first-time player, so the hub's last row `>> START FLIGHT` should be the default
selection, not the first row.

**Onboarding.** On first launch (no `settings.ini`), open the controls page once before the
menu, with `A CONTINUE`. That single page does more than the eight help pages a new player will
not find. Owner decision OD3 (quick flight) is the stronger version of this: one row that puts
the player on a three-mile final in the C172.

### 4.3 HUD hierarchy

The attitude, tapes and flight-path vector are right. The problems are at the edges.

**Status rows.** Row 1 carries seven items in 18 px and changes text size with the aircraft.
Row 2 at STD is nearly empty. Rebalance by frequency of use: things the player checks in every
phase on row 1, configuration on row 2, and show a field only when it is not in its normal
state.

```
 now, STD:
 | THR 65% RPM 2300 FLAP 10 [##..] GEAR FIXED TRIM [..|..] |
 | WIND CALM                                               |

 proposed, STD:
 | THR 65%  2300 RPM          FLAP 10 [##..]   TRIM [..|.] |
 | FUEL 2:10        WIND 270/08              ! PARK BRAKE  |
```

- Fixed gear is never shown. Retractable gear shows `GEAR DN`, `GEAR UP`, or the transit arrow.
- Rudder and aileron trim appear only when non-zero (already the case).
- Endurance is on row 2 at every level except MIN, and turns into a caution under 30 minutes.
- Row 1 never changes text size. If a four-engine aircraft overflows, abbreviate the engine
  readouts, not the font.

**Speed bugs.** Tape numerals stay inside the tape. Bugs sit outside it with a tag shape and a
prefix, so a flap bug can never be read as a speed:

```
 now:                     proposed:
   60 -|                    60 -|
       |- UP                    |-<UP]
       |- 10                    |-<F10]
   40 -|- 20                40 -|-<F20]
       |- 30                    |-<F30]
```

**Cautions and warnings without colour.** Position already helps (warnings are centred and
large). Add shape so the amber and white HUD themes, and colour-blind players, keep the channel:

```
   warning:   [ STALL ]        filled box, blinking, large text
   caution:   /!\ GEAR         triangle prefix, steady, medium text
   advisory:      AP OFF       plain
```

On the speed tape, hatch the red band so it differs from amber by pattern as well as hue.

**The map.** Keep the sim running under it (pausing would break its use for navigation), but
never leave the player blind:

```
+--------------------------------------------------------------+
| IAS 104   ALT 3500   VS -200          /!\ TERRAIN      LIVE  |
|--------------------------------------------------------------|
|                          (map)                               |
```

### 4.4 Legibility

- Add `TEXT SIZE NORMAL / LARGE`. LARGE maps the 9 px size to 12 px and the 12 px size to
  14 px. The tapes then label every 20 kt and every 200 ft as now, with the tape 8 px wider.
- Whatever the setting, footers and the nav box should not be 9 px. They are read at a glance,
  and they have room.
- Disabled menu rows are about 3.6:1 contrast at thin stroke widths [I]. Raise the alpha.
- Check the second warning line (baseline 280) against `MINIMUMS` (baseline 276): they appear
  to share a band [I]. One Xvfb screenshot with both forced on settles it.

### 4.5 Error states

One rule: anything that changes what the player gets must be visible where the player is.

| Event | Today | Proposed |
|---|---|---|
| Settings or last-flight save fails | log | Menu status line, and the footer says `NOT SAVED` |
| Flight log cannot be written | log | Status line; the log screen says `LOG COULD NOT BE SAVED` instead of `NO FLIGHTS YET` |
| No data pack, or a pack failed to load | log, About page | Main-menu banner with the reason |
| Spawn gate timed out | log | In-flight amber message until terrain arrives |
| No METAR file | log | `WX: DEFAULT` message at flight start |
| Controller removed | log | Pause menu opens with `CONTROLLER DISCONNECTED` |
| Simulation error (exception or NaN) | process ends | Back to the menu with `SIMULATION ERROR`, flight logged |

### 4.6 Missing pieces, in the order I would build them

1. **N11b** (in-flight plan edits). It is unblocked, the rules and tests exist, and today an
   off-plan Direct-To silently drops enroute legs with no undo.
2. **F11** (nose-wheel steering). Taxiways shipped in 1.3, so taxiing is now a real activity.
3. **U1** (quick flight), if OD3 is a yes. It is the best onboarding lever.
4. **H8** (map terrain shading). Useful in the Cascades, but after the map's live strip.
5. **N8** (airways in `.fms`). Until then, warn at load: `AIRWAYS FLOWN DIRECT`.
6. **X5** (live METAR). Only through the launcher, per `ONLINE.md`.

---

## 5. Security and hardening plan

The threat is modest and specific: a root process that reads files from a removable card. The
code is in better shape than most. No memory-corruption path was found. The work is about
turning "crash or hang on bad data" into "reject and tell the player".

**Phase 1: an afternoon each.**

1. S1: the `LD_LIBRARY_PATH` colon.
2. R3: top-level `try/catch`, plus one around the tile worker loop that marks the tile failed.
3. S2: size checks before allocation in `readWholeFile`, `MagVar::loadFile` and
   `readTextFile`; a line-length cap in the CSV reader; a waypoint cap in `.fms`.
4. S3 and S5: clamp runway dimensions and reject non-finite numbers at load.
5. S4: map-indexed INI sections.
6. S7: `SHA256SUMS` on every release.

**Phase 2: CI.** See section 6. ASan and UBSan over the existing 774 tests will likely surface
the out-of-range `double` to `int` casts in `Procedures.cpp:297`, `NavDb.cpp:714` and
`FlightLog.cpp:234` [T].

**Phase 3: supply chain.** Pin JSBSim to a commit SHA, the Docker base to a digest and the
actions to SHAs. Add build provenance attestation to the release. Record the commit that built
each data pack in its `REGION.txt` and have the release job check it is on `main`. Add
hardening flags (S9) and have `check_package.sh` assert RELRO, PIE and no RPATH.

**Launch script.** Sourcing PortMaster's `control.txt` is how every port works and is not worth
fighting. Two small tightenings: drop `$SCRIPT_DIR/PortMaster` from the search list unless a
device needs it (it is the one candidate on the removable card), and write the launch log with
a fixed name only when `SCRIPT_DIR` resolved, not into `/tmp`.

**CLI paths.** `--scenario`, `--data-dir`, `--conf-dir` and `--trace` need no jail: whoever can
pass arguments already controls the process. The paths worth checking are the ones that come
from *file contents*: the weather ident (S6) and a scenario's `plan=`.

**Licensing.** Nothing here touches the data-pack licences. The ODbL notice for taxiways and
the About-page credits are in place.

---

## 6. Test and CI additions

| ID | Addition | Why | Cost |
|---|---|---|---|
| T1 | A `sanitize` job: `-fsanitize=address,undefined -fno-sanitize-recover`, run `ctest` and the two headless scenarios | Catches the audio shutdown order, the casts and any regression in the loaders | About 5 minutes of CI |
| T2 | Fuzz targets (libFuzzer, 60 s each on pull requests, longer nightly). All are callable from bytes today: `Ini::loadText` + `Settings::applyText` + `parseScenario` (`src/core/Ini.cpp:55`, `src/app/Settings.cpp:126`, `src/app/Scenario.cpp:46`); `parseFms` (`src/nav/Fms.cpp:105`); `parseProcedures` (`src/nav/Procedures.cpp:264`); `MagVar::loadBytes` (`src/nav/MagVar.cpp:35`); `parseMetar` (`src/world/WeatherMetar.cpp:180`); `parseWavMono16` (`src/audio/VoiceBank.cpp:79`) | The files a player or a pack supplies | S |
| T3 | A text-taking overload for the `CsvNavDatabase` loaders, then a seventh fuzz target | The nav CSVs are the largest untrusted input and cannot be fuzzed from bytes today | S–M |
| T4 | Device workflow: compare the binary's undefined `SDL_*` symbols with a 2.0.12 allowlist | Makes the SDL ceiling a check, not a convention | S |
| T5 | Release workflow: fail when the tag differs from the CMake version | Ends the drift in C1 | S |
| T6 | A TSan run of one headless scenario with audio enabled (dummy driver) | Confirms R6 and guards the worker | S |
| T7 | Xvfb screenshots of the states this review could not see: both HUD colour themes with a caution showing, `MINIMUMS` with two warnings, the map page, a failed-save footer | Regression pictures for section 4 | S |
| T8 | `clang-tidy` with a short list (`bugprone-*`, `cert-err34-c`, `performance-*`), warnings only at first | Cheap; finds the unchecked conversions | S |
| T9 | Robustness unit tests: an oversized tile file, a 10,000-section INI, a one-billion-foot runway, a NaN start distance | Pins S2–S5 | S |
| T10 | `permissions: contents: read` at the top of `ci.yml`, `device.yml` and `datapack.yml` | Least privilege for the token | S |

---

## 7. Quick wins (each under an hour)

1. `PocketVectors.sh:124`: `${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}`. (S1)
2. Move `drawSea` below the node loop. (P1)
3. Declare `voiceBank` before `audioEngine`, or call `audioEngine.shutdown()` before return. (R5)
4. `try/catch` in `main` that logs and returns non-zero. (R3)
5. Write the version and commit as the first line of `log.txt`; keep `log.prev.txt`. (R7)
6. Log `VmHWM` at exit. (R8)
7. Log `hud2d.stats().vertices` in the perf line. (P7)
8. Rename the menu row to `REPEAT LAST FLIGHT`. (U9)
9. Prefix flap bug labels with `F`. (U2)
10. Draw one `LOADING` frame and swap before `sim.init` in `startOrReport`. (R2)
11. Make the settings footer reflect the save result. (U3)
12. Post the weather source at flight start. (U6)
13. Add an ellipsis in `fitText`. (U14)
14. Set N11b to `ready`; fix the `App.h:9` comment. (C3, C4)
15. `SHA256SUMS` in the release job. (S7)
16. Skip `renderer.render` and `airports.render` when the menu is full-screen. (P3)
17. `permissions: contents: read` on the three workflows without it. (T10)

---

## 8. Questions for the owner

1. **Is v1.3.0 tagged?** The board says released, the remote has no tag. Should it be tagged at
   the commit you device-checked, with `main` moving to `1.4.0-dev`?
2. **Frame rate.** Would you take a `FRAME RATE 30/60` setting, and which should be the default
   on the device? A locked 30 is steadier in the heavy scenes and roughly halves the work.
3. **OD5.** Agree to leave the governor alone by default and add an opt-in performance mode only
   if overcast or the orbit still dips below 30 after the fill cuts? Can a device session log
   the devfreq load beside the frame time to see why it never leaves 420 MHz?
4. **MSAA.** Is a visible `ANTIALIASING` setting acceptable, with the default chosen from a
   device measurement?
5. **Text size.** A `LARGE` setting, or simply raise the smallest size for everyone? Which HUD
   elements do you find hardest to read on the device today?
6. **Resume.** Rename to `REPEAT LAST FLIGHT`, or do you want a true resume (position, speed,
   fuel, plan progress)? The second is a real feature, with JSBSim state to restore.
7. **Select+Start.** Hold-to-quit in flight? `gptokeyb` also acts on the chord: should the
   launcher stop starting it, now that the game handles the chord itself?
8. **The map.** Keep the sim live under it with an instrument strip, or pause?
9. **Order of work.** Is N11b next? Where do F11, N8, H8, U1 (OD3) and X5 (OD4) sit after it?
10. **RSS budget.** Is 350 MB still right for a 1 GB device with `sea_sfo`, and can a device
    session record the peak?
11. **Release integrity.** Checksums only, or provenance attestation too? Should packs be
    rebuilt per release instead of taken from pinned run IDs?
12. **`sim_hz`.** Remove it?
13. **X/Y mapping.** Can the next device session confirm it in the button test?
14. **Other handhelds.** Is anything beyond the RG40XXH in scope (other H700 devices at 640×480,
    or 720×720 panels)? It decides whether layout stays fixed at 640×480.
15. **Hardening flags.** Acceptable to add PIE, RELRO, stack protector and FORTIFY to the device
    build, subject to no measurable frame cost?
16. **Data packs for review.** Can `pdx_oth` be made available to a review session? Terrain
    cost, RSS, KPDX mesh-build time and nav-load time could not be checked without it.

---

## What I ran

- Built `main` at `0e69d77` with GCC (`-DCMAKE_C_COMPILER=gcc -DCMAKE_CXX_COMPILER=g++`, RelWithDebInfo): 298 targets, clean.
- `ctest`: 3 of 3 passed (57 s).
- Headless flight, `c172_final_xtst.ini`, 60 s: ends on final at 65 kt, `ok`.
- Xvfb screenshot of the same scenario at 1280×960 (Mesa software GL). It shows the flap bugs
  `20` and `30` stacked beside the tape's `40` and `20` numerals (U2), the crowded status row 1
  over a row 2 holding only `WIND CALM`, and pitch-ladder labels cut off where they run under
  the nav box and the status panel.
- S4 triggered: a scenario file of 60,000 empty sections (0.5 MB) takes 3.6 s to load on a
  desktop core, and 120,000 sections (1.1 MB) takes 16.5 s. That is quadratic, and it will be
  several times slower on a Cortex-A53.
- `git ls-remote --tags origin`: `v1.0.0`, `v1.1.0`, `v1.2.0`, `v1.2.1` only.
- Four read-only exploration passes over the render path, the UI and HUD, the parsers and
  packaging, and the app and startup code. I re-read the lines behind every [V] finding.

**Not verified here, and why.**

- Anything on the device: frame times, the governor, swap interval 2, MSAA cost, X/Y mapping.
- Anything needing a data pack: nav-load time, RSS, terrain triangle and draw counts, airport
  mesh-build time, the runway-length allocation in S3.
- S3 was traced in the code and not triggered with a crafted file.
- All figures marked [I], including every millisecond estimate in section 3 and the vertex
  count in P7.
