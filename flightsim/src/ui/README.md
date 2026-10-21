# ui/ menus and flight setup (WP H5)

`MenuSystem` is a stack of screens driven by `MenuInput` (the nine menu buttons derived from P2's
`ActionState`: D-pad, A accept, B back, X, Y, Start). `update()` returns a `MenuResult`
(`StartFlight` + a `Scenario`, `ResumeFlight`, `RestartFlight`, `Quit`, a HUD message, a
settings-changed flag); `draw()` paints with `Renderer2D` in its own batch after the HUD. No GL in
the logic: `ListNav`, `OnScreenKeyboard`, `FlightSetup` and the screens are unit tested
(`tests/ui/`). `MapPage*` belongs to H4.

| File | Content |
|---|---|
| `MenuInput.h`, `ListNav.*` | input reduction, list selection with wrap, disabled rows and scrolling |
| `OnScreenKeyboard.*` | character grid (A to Z, 0 to 9): D-pad moves, A types, B deletes (cancels when empty), X space, Y letters/digits, Start done |
| `FlightSetup.*` | selections to `Scenario`; discovery of aircraft profiles (`assets/aircraft/*.ini`) and `.fms` files (`data/plans`, `assets/plans`); airport search ordering; runway and approach lists |
| `MenuSystem.*` | screen stack, deferred navigation, canned screens (`--menu`), start / restart / quit results |
| `SetupScreens.cpp`, `PlanScreens.cpp`, `FlightScreens.cpp` | main menu, new flight hub, aircraft, airport search, runway, start type, plan, plan files, approach; settings, pause menu, in-flight flight plan |
| `UiPainter.*` | theme (accent follows the HUD color setting), lists, keyboard, dialogs |

## Flow
* No `--scenario` and no `--resume` on an interactive run (the device launch): main menu with Resume
  last flight (when `<conf>/last_flight.ini` exists), New flight, Settings, Help, Quit. `--scenario`
  runs go straight into the flight; headless runs never see a menu.
* New flight hub: aircraft, airport (keyboard search, empty text shows the nearest airports),
  runway (both ends), start (runway, 2 / 6 / 10 NM final, in the air at an altitude), flight plan
  (`.fms` file, or a quick direct plan from the start airport to a destination runway), approach
  (real CIFP first with the procedure title, then synthesized ILS / RNAV / visual). Start (the
  button or the last row) builds the `Scenario`. A plan is written to `<conf>/menu_plan.fms` and
  referenced by `Scenario::planPath` (so Resume last flight keeps it); a plan-less approach goes in
  `Scenario::approachId` (practice approach at the start runway).
* Pause button: Resume, Flight plan (direct-to a leg or an ident, change or remove the approach,
  re-enter the approach), Restart flight, New flight, Settings, Quit to menu, Quit, Help, Mini nav
  (toggles the HUD inset and writes `hud_mini_nav` in `conf/settings.ini`). Restart and quits ask
  for confirmation. Select+Start still quits from anywhere (P2).
* Help (main menu and pause menu): live control list from the loaded bindings (one page per layer,
  plus the map and the menus), a button test that can swap A/B or X/Y in `conf/controls.ini`,
  how-to pages from `assets/help/`, and About (credits, build version, data attribution).
* Settings: HUD color, declutter default, pressure units, show FPS, FOV, volume, throttle mode,
  assists, stick sensitivity and expo. Saved with P3's safe write when the screen is left.

## `--menu <screen[:arg]>` (screenshots and tests)
Opens a screen at startup (works with `--frames` / `--screenshot`, and with `--scenario` for the
screens that sit over a flight): `main`, `setup`, `aircraft`, `airport[:TEXT]` (types TEXT),
`runway[:ICAO]`, `start`, `plan[:FILE]` (loads the plan file, default `KPDX-KOTH`),
`approach[:FILE]`, `settings`, `pause`, `flightplan`, `confirm`,
`help[:controls[:PAGE]|buttons|howto[:ID]|about]`. Example:
`flightsim --data-dir data --scenario assets/scenarios/c172_cruise_kpdx_koth_plan.ini --menu pause --frames 5 --screenshot /tmp/pause.png`.
