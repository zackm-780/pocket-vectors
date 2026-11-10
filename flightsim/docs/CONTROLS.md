# Controls

Owners: P2 (input system and defaults), P3 (persisted remaps), H5 (menu navigation). Physical inputs
are bound to **semantic actions** (`core/Actions.h`). Gameplay code never reads buttons directly.

## 1. RG40XXH physical inputs
D-pad, A, B, X, Y, L1, R1, L2, R2, Select, Start, left stick, right stick, and the Menu key. Volume
and power stay with the OS. This document always means the **label printed on the device**. Bindings
use those labels. SDL names live only in `[device_labels]` in `assets/controls_default.ini`.

Measured on the RG40XXH stock OS (PortMaster gamecontroller mapping, 2026-10-20):

- Printed face labels match SDL GameController names: A = `a`, B = `b`, X = `x`, Y = `y`.
- Select is SDL `guide` (button 6). The Menu key is SDL `back` (button 8).
- L2 and R2 are digital triggers (`lefttrigger` / `righttrigger` jump from 0 to full).
- L3 (left-stick click) is clean. R3 (right-stick click) jolts right stick X by about −0.32, so no
  in-flight layer binds R3 (rudder is on that axis).

The default table is `select = back, guide`. Both the stock-OS Select button and the Menu key
produce Select (map, and Select+Start to quit). Firmwares that report Select as `back` work the
same way.

**Other devices.** If A/B or X/Y come out swapped, or Select does nothing, override `[device_labels]`
in `flightsim/conf/controls.ini`. A label may list several SDL buttons, separated by commas. See
INSTALL.md.

## 2. Flight controls (default)

### Base layer
| Input | Action |
|---|---|
| Left stick | Pitch (Y) and roll (X). Deadzone 0.08 radial; roll expo 0.35 (`[axes] left_expo`), pitch expo 0.6 (`left_pitch_expo`, F9), or the aircraft's own `[controls] pitch_expo` (C172 0.83, PA28 0.6, 737 0.68), so small pitch inputs are gentle while full deflection is still full elevator. |
| Right stick X | Rudder / nosewheel steering. Deadzone 0.12. |
| Right stick Y | **Throttle rate:** push up = increase, pull down = decrease, rate ∝ deflection (full deflection: 0→100% in 2.5 s). Released = hold. An axis-dominance filter suppresses rudder while the stick is mostly vertical. |
| D-pad ↑ / ↓ | Elevator trim nose-down / nose-up (↓ = nose up, like pulling a trim wheel back). F9: a press moves the trim one step (a tap ≈ 30–150 fpm of VS); held past 0.3 s it runs at a steady rate (one second ≈ 150–600 fpm). Step and rate shrink with (Vref / IAS)² above Vref (at least a quarter), so a tap does about the same at any speed; per aircraft `[controls] trim_step_norm` / `trim_rate_norm_per_s`. |
| D-pad ← / → | Flaps up / down one detent |
| A | Gear toggle (ignored with weight on wheels; shows `GEAR LOCKED ON GROUND`) |
| B | **Wheel brakes** (hold). Double-tap B = parking brake toggle. |
| X | Speedbrake/spoilers cycle: retracted → 50% → 100% → retracted (aircraft without them show `NO SPEEDBRAKE`) |
| Y | HUD declutter cycle FULL → STD → MIN |
| R2 | Brakes (hold), the same as B. Lets you keep a thumb on the right stick for steering. |
| Select | Map page toggle |
| Start | Pause menu |
| L3 | Cycle world overlays (off, route, route + path, airports, all). Each layer starts off. |
| Select + Start | **Quit** (PortMaster gptokeyb hotkey) |

### Map page (Select opens it; the sim keeps running)
| Input | Action |
|---|---|
| D-pad ↑ / ↓ | Zoom in / out (2, 5, 10, 20, 40, 80, 160 NM) |
| D-pad ← / → | Previous / next route waypoint (info panel) |
| A | Direct-to the selected waypoint |
| Y | North-up ↔ track-up |
| B or Select | Close the map |

While the map is open, trim, flaps and gear inputs are ignored.

### L1 held: autopilot layer
| Input | Action |
|---|---|
| L1 + A | AP engage / disengage |
| L1 + B | Autothrottle on/off (SPD) |
| L1 + X | HDG mode (pressing again with HDG active → ROL) |
| L1 + Y | NAV ↔ APR (arms LOC/GS when an approach is loaded) |
| L1 + D-pad ← / → | Heading bug −/+ 1° (hold: accelerates to 10°/step) |
| L1 + D-pad ↑ / ↓ | Altitude bug +/− 100 ft (hold: accelerates to 1,000 ft/step) |
| L1 + right stick Y | VS bug (in VS mode) or speed bug (A/T on), rate-based |
| L1 + R2 | ALT hold at the current altitude; pressed again in ALT with the altitude bug more than 100 ft away: VS toward the bug at the default rate (ALT armed) |
| L1 + Select | DIRECT TO NEXT (N11b, NAVIGATION 2.4 rule 8): direct to the active waypoint while it is still ahead, else to the first waypoint ahead (no plan, or none left: opens the direct-to list) |
| L1 + L3 | TO/GA (airborne): go-around pitch on speed, A/T to TOGA, HDG on the present heading with NAV armed on the plan; the AP stays engaged below minimums |
| L1 + Start | FLC: climb or descend to the altitude bug at the speed bug (A/T: climb thrust or idle); again → PIT |

With the AP and the FD both off, a mode button (HDG, NAV / APR, ALT, VS, FLC) turns the FD on
first, the way TO/GA does, so the mode is selected on the FD; L1 + A then hands it to the AP.

### R1 held: systems layer
| Input | Action |
|---|---|
| R1 + D-pad ← / → | Rudder trim left / right |
| R1 + A | Thrust reversers toggle (only on ground with throttle at idle) |
| R1 + B | Flight director on/off |
| R1 + X | Sync bugs (heading bug = current heading, altitude bug = current altitude rounded to 100) |
| R1 + Y | Nav source cycle GPS → LOC (ILS or LOC approach loaded) → VOR (when tuned) → GPS. Leaving LOC stays on GPS until the next press; `NO LOC OR VOR TUNED` when there is nothing to switch to. |
| R1 + D-pad ↑ / ↓ | Altimeter setting +/− 0.01 inHg or 1 hPa (per the pressure-units setting; hold repeats) |
| R1 + Select | Altimeter to standard (STD, 1013.25 hPa) |
| R1 + Start | Undo the last route change (`ROUTE REPLACED`, `PLAN CLEARED`, `SID REMOVED`, a removed STAR or approach) for 10 s of flight (N11b): `ROUTE REPLACED UNDONE`, or `NOTHING TO UNDO` |

### L2 held: look layer
| Input | Action |
|---|---|
| L2 + right stick | Free look (yaw ±150°, pitch −60°..+45°). The stick turns the view (150°/s at full deflection); let go and the view stays there while L2 is held. |
| L2 + D-pad ← / → / ↓ | Snap look left 90° / right 90° / back |
| L2 + D-pad ↑ | Look down 25° over the nose (landing view) |
| Release L2 | The view eases back to straight ahead in about 0.3 s |
| L2 + L3 | Cockpit ↔ external chase view (V1, saved as `external_view`). In the chase view the right stick and the D-pad snaps orbit the camera around the aircraft instead. |

While L2 is held, the right stick does **not** affect throttle or rudder, and the D-pad does not
trim or move the flaps. The look layer is **L2**. R2 is the brakes, so R2 + D-pad is brakes plus
trim or flaps. The HUD stays on screen; the horizon, ladder and flight path marker stay conformal
to the world, so they slide off when you look away.

Buttons pressed in a menu stay in the menu: the A that picks RESUME (or starts a flight) does not
also toggle the gear.

## 3. Menus
FLIGHT PLAN in flight (N11b): on a leg row A goes direct to it, X activates it and Y deletes it
(the footer says so). The KEEP ROUTE / CLEAR ROUTE prompt opens on CLEAR ROUTE; up / down (or left /
right) and A choose, B cancels without a change.
FLIGHT LOG (main menu, U2) uses the standard list keys: D-pad up and down to move, A for a
flight's details, B back. No new buttons.
Help, on the main menu and the pause menu, lists the live bindings, tests a button, and holds the
how-to pages. D-pad navigates, A accepts, B goes back, Start closes the pause menu, X and Y are context actions shown
in the footer. The on-screen keyboard (airport/fix search) uses a character grid (D-pad + A, B =
delete, Start = done, X = space, Y = cycle letters/digits).

## 4. Desktop keyboard (default; ignored on device, see BUILD_AND_DEPLOY §6)
| Key | Action | Key | Action |
|---|---|---|---|
| ↑ ↓ ← → | pitch / roll (keyboard ramp) | Q / E | rudder left / right |
| W / S, PgUp / PgDn | throttle up / down | Home / End | trim nose-down / nose-up |
| [ / ] | flaps up / down | G | gear |
| B (hold) | brakes | Shift+B | parking brake |
| / | speedbrake cycle | R | reversers |
| Tab | declutter | M | map |
| Esc / P | pause menu | F12 | screenshot |
| F1 | AP | F2 | A/T |
| F3 | HDG | F4 | NAV/APR |
| F5 | FD | - / = | heading bug |
| F6 | TO/GA | F7 | FLC |
| C | cockpit / chase view | | |
| 9 / 0 | altitude bug −/+ | V + arrows | look |
| Ctrl+Q | quit | Ctrl+Z | undo the last route change (R1 + Start) |

Gamepads work on desktop with the same mapping as the device.

## 5. Assists (settings, off by default unless noted)
- **Auto-rudder:** coordinates turns in flight, steers on the ground with the roll axis.
- **Auto-trim** (`assist_auto_trim`, AUTO TRIM, F9, not realistic): with the stick centred for 1 s the trim holds the flight-path angle you left, with flight-path-rate damping, so it also calms the phugoid. Off on the ground and while the AP is engaged; a trim press or a stick input takes over.
- **Pitch damper** (`assist_pitch_damper`, PITCH DAMPER, F9, not realistic): adds pitch-rate damping to the elevator while the stick is within ±0.3 of the centre (not on the ground).
- **Stick sensitivity / expo** per axis.
- **Throttle mode:** rate (default) or absolute (right stick Y position = throttle, for players who prefer it).
