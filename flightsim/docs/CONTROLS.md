# Controls

Owners: P2 (input system and defaults), P3 (persisted remaps), H5 (menu navigation). Physical inputs
are bound to **semantic actions** (`core/Actions.h`). Gameplay code never reads buttons directly.

## 1. RG40XXH physical inputs
D-pad, A, B, X, Y, L1, R1, L2, R2, Select, Start, left stick, right stick. The Menu/function key,
volume and power keys are **reserved for the OS**. L3/R3 stick clicks and whether L2/R2 are analog are
**unverified**: design so nothing essential depends on them, and update after the P1 probe.

> Face-button naming: Anbernic prints a Nintendo-style layout (A on the right, B at the bottom). SDL's
> GameController API names buttons by **position** in an Xbox layout (SDL `A` = bottom). PortMaster
> supplies `SDL_GAMECONTROLLERCONFIG` for the device. P2 must confirm from the probe log which SDL
> name each **printed** label produces. This doc always means the **label printed on the device**.

## 2. Flight controls (default)

### Base layer
| Input | Action |
|---|---|
| Left stick | Pitch (Y) and roll (X). Deadzone 0.08 radial, expo 0.35. |
| Right stick X | Rudder / nosewheel steering. Deadzone 0.12. |
| Right stick Y | **Throttle rate:** push up = increase, pull down = decrease, rate ∝ deflection (full deflection: 0→100% in 2.5 s). Released = hold. An axis-dominance filter suppresses rudder while the stick is mostly vertical. |
| D-pad ↑ / ↓ | Elevator trim nose-down / nose-up (hold to keep trimming; ↓ = nose up, like pulling a trim wheel back) |
| D-pad ← / → | Flaps up / down one detent |
| A | Gear toggle (ignored with weight on wheels; shows `GEAR LOCKED ON GROUND`) |
| B | **Wheel brakes** (hold). Double-tap B = parking brake toggle. |
| X | Speedbrake/spoilers cycle: retracted → 50% → 100% → retracted (aircraft without them show `NO SPEEDBRAKE`) |
| Y | HUD declutter cycle FULL → STD → MIN |
| R2 | Brakes (hold), the same as B. Lets you keep a thumb on the right stick for steering. |
| Select | Map page toggle |
| Start | Pause menu |
| Select + Start | **Quit** (PortMaster gptokeyb hotkey) |

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
| L1 + R2 | ALT hold at the current altitude |
| L1 + Select | Direct-to the next waypoint in the plan (or open the direct-to list) |

### R1 held: systems layer
| Input | Action |
|---|---|
| R1 + D-pad ← / → | Rudder trim left / right |
| R1 + A | Thrust reversers toggle (only on ground with throttle at idle) |
| R1 + B | Flight director on/off |
| R1 + X | Sync bugs (heading bug = current heading, altitude bug = current altitude rounded to 100) |
| R1 + Y | Nav source cycle GPS → LOC/VOR (when tuned) → GPS |

### L2 held: look layer
| Input | Action |
|---|---|
| L2 + right stick | Free look (yaw ±150°, pitch −60°..+45°). Released = recenter over 0.3 s. |
| L2 + D-pad ← / → / ↓ | Snap look left 90° / right 90° / back |
| L2 + D-pad ↑ | Look down 25° over the nose (landing view) |

While L2 is held, the right stick does **not** affect throttle or rudder.

## 3. Menus
D-pad navigates, A accepts, B goes back, Start closes the pause menu, X and Y are context actions shown
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
| 9 / 0 | altitude bug −/+ | V + arrows | look |
| Ctrl+Q | quit | | |

Gamepads work on desktop with the same mapping as the device.

## 5. Assists (settings, off by default unless noted)
- **Auto-rudder:** coordinates turns in flight, steers on the ground with the roll axis.
- **Auto-trim:** slowly trims out sustained elevator input. Handy on small sticks.
- **Stick sensitivity / expo** per axis.
- **Throttle mode:** rate (default) or absolute (right stick Y position = throttle, for players who prefer it).
