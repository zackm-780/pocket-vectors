# HUD specification

The game has no cockpit, so **the HUD is the cockpit**. Everything a pilot needs (attitude, speeds,
altitudes, flaps, gear, trim, throttle and engines, brakes, nav, autopilot) must be readable on a
4" 640×480 screen. Owners: H1 (renderer and font), H2 (flight HUD), H3 (nav and AP elements), H4 (map).

## 1. Principles
- **Conformal where it matters:** horizon, pitch ladder and flight path vector line up exactly with
  the 3D world (same camera and projection, ARCHITECTURE §6).
- **Green stroke symbology with a dark outline** (1 px virtual, ~60% alpha black) so it reads against
  bright sky and dark terrain. The color setting offers green, white or amber.
- **Caution and warning colors:** amber = caution, red = warning. These are the only non-HUD colors.
  Flashing at 2 Hz for new warnings for 3 s, then steady.
- **Declutter levels** (Y button / Tab): `FULL` (everything), `STD` (no ladder numbers below ±10°, nav
  box reduced to one line), `MIN` (attitude, speed, altitude, FPV and warnings only).
- **Feedback for every button:** a transient message (1.5 s, bottom center), e.g. `FLAPS 15`,
  `GEAR DOWN`, `PARK BRK SET`, `AP ON`, `HDG 270`.

## 2. Rendering basics (H1)
- Virtual canvas 640×480, origin top-left. Scale = drawable height / 480. Wider screens extend the
  width symmetrically, and elements anchored left or right stay at the edges.
- Line width 2 px virtual (1.5 at MIN declutter optional). Everything is drawn as quads in one
  batch, with the outline pass first.
- Stroke font sizes (cap height): **S = 9 px**, **M = 12 px**, **L = 16 px**. Monospaced advance =
  0.75 × height. Digits must be unambiguous (slashed or dotted zero not required, but 0 and O differ).
- Glyphs: A–Z, 0–9, `. , : / - + ° % < > [ ] ( ) * # ' "`, arrows ↑ ↓ ← → (for trend and trim),
  ▲ ▼ (bugs) and ◆ (bearing pointer).

## 3. Layout (virtual px)

```
 0 ┌──────────────────────────────────────────────────────────────────────────────┐
   │ A/THR SPD │ LOC  (G/S) │ ALT   │ AP1 FD          ◆ GPS  CEPIN   12.4 NM 6:12 │ FMA row (y 4–20)
24 │                 ┌──────────── heading tape 300 px ───────────┐               │
   │                 │ 24  25  26 ▼27▲ 28  29  30   [271]          │               │ (y 26–56)
60 │ ┌──────┐                    bank scale arc                         ┌──────┐  │
   │ │speed │        ── 10 ──                     ── 10 ──             │ alt  │  │
   │ │ tape │                                                           │ tape │  │
   │ │      │              ─────── horizon ───────                      │      │  │
   │ │[ 132]│                    ─W─   ⊖ FPV                            │[3200]│  │ center y = 240
   │ │      │        ── -5 ──                       GS scale ◆ (x≈500)  │      │  │
   │ │      │                                                           │      │  │
   │ └──────┘          LOC scale ◆ (y≈392)                               └──────┘  │
   │ GS 128  M.21                                                      VS +700    │
   │ ↑ trend                     [ warnings y≈300 ]                    RA  450    │
   │                                                     ┌ nav box (x 440–636) ┐  │
   │                                                     │ ILS 28R 111.70 CRS 297│ │ (y 368–428)
   │                                                     │ DME 8.4  DA 213       │ │
430├──────────────────────────────────────────────────────────────────────────────┤
   │ THR 72% │ N1 84.2 84.0      │ FLAP 15 ▮▮▯▯ │ GEAR ●●● DN │ TRIM ↑ +0.12        │ status row 1 (y 434–450)
   │ SPDBRK 50% │ REV │ BRK │ PARK │ FUEL 9 840 LB │ WIND 270/15 │ T+00:42:10      │ status row 2 (y 456–472)
480└──────────────────────────────────────────────────────────────────────────────┘
```

| Zone | Box (x, y, w, h) | Contents |
|---|---|---|
| FMA row | 0, 0, 640, 22 | Autothrottle mode · lateral active (armed in parentheses, smaller) · vertical active (armed) · AP/FD status · nav source + active waypoint + distance + ETE (right-aligned) |
| Heading tape | 170, 26, 300, 30 | ±35° visible, ticks every 5°, labels every 10° (`27` style), current heading box (3 digits, **magnetic**), heading bug ▲▼, DTK/CRS mark, bearing-to-waypoint ◆, ground track tick |
| Bank scale | arc radius 150 around (320,240), top | Ticks 10/20/30/45/60, pointer, slip/skid bar under pointer |
| Pitch ladder | 120..520 × 70..400, clipped | Lines every 5°, labels every 10°. Negative pitch lines dashed. Rotates with bank. Horizon line full width. |
| Boresight / waterline | center (320,240) | `─W─` fixed aircraft reference |
| FPV | conformal | Circle + wings + tail at the velocity vector. Clamped (and dimmed) at the clip edge. |
| Speed tape | 16, 70, 80, 300 | IAS kt, ticks every 10, labels every 20, current box at center, speed bug, bands (red below Vs0/Vs1, white flap band Vs0..Vfe, red above Vne/Vmo), 10 s trend arrow |
| Below speed tape | 16, 374 | `GS 128` (ground speed); `M.78` when mach > 0.40 |
| Altitude tape | 544, 70, 80, 300 | ft MSL, ticks every 100, labels every 500, current box (thousands larger), altitude bug, minimums bug, ground band (terrain elevation) when in range |
| Below altitude tape | 544, 374 | `VS +700` (fpm, rounded to 50) and `RA 450` (radio altitude when AGL < 2,500 ft). Below 1,000 ft AGL, RA also shows as large text under the FPV. The altimeter setting (X1) is the third line: `STD`, `29.92` (inHg) or `1013` (hPa) per the pressure-units setting. |
| LOC scale | horizontal, centered (320, 392), ±2 dots | Diamond = deviation. Shown when NavState.locValid, or GPS/CDI in approach phase. |
| GS scale | vertical, (500, 240), ±2 dots | Diamond = glidepath. Shown when gsValid. Hollow when only advisory VNAV. |
| Nav box | 440, 368, 196, 60 | GPS: `GPS → CEPIN 12.4NM` / `DTK 281 XTK 0.2R ETE 6:12` / `NEXT ARCHI 3000A`. ILS: `ILS 28R 111.70 CRS 297` / `DME 8.4 DA 213`. VOR: radial and DME. |
| Status row 1 | 0, 434, 640, 18 | Throttle and engines, flaps, gear, trim (see §4) |
| Status row 2 | 0, 456, 640, 18 | Speedbrake, reversers, brakes and parking brake, fuel, wind, flight time |
| Warnings | centered, y 300 | Highest priority first: `CRASH: <reason>`, `PULL UP`, `STALL`, `TERRAIN`, `SINK RATE`, `OVERSPEED`, `GEAR` (amber), `AP OFF` (amber flash), `PARK BRK` (amber while throttle > 50% and parking brake set), `PAUSED` |
| Messages | centered, y 330 | Transient feedback (§1) |
| Mini nav | 118, 214, 140, 140 | Track-up route inset (H7), off unless `hud_mini_nav` is on. Aircraft symbol, magenta active leg, other legs white, next-waypoint distance, ETE, cross-track and a range ring. Hidden in MIN declutter and while the map page is open. |

## 4. Status rows: required fields

Every field is visible in **FULL** declutter. In STD, row 2 hides FUEL and flight time. MIN hides both rows.

| Field | Source | Format and rules |
|---|---|---|
| Throttle | `ControlInputs.throttleNorm` (commanded) | `THR 72%`. Multiple engines with split levers: `THR 72/70%`. Reverse shows `THR REV 60%`. |
| Engines | `AircraftState.engines[]` | Jets: `N1 84.2 84.0` (one per engine, up to 4). Pistons: `RPM 2350`. Turboprops: `TQ`/`NP` when available, otherwise `N1`. An engine that isn't running shows amber `OFF`. |
| Flaps | `flapDetent`, `flapPosNorm`, profile labels | `FLAP 15` (selected detent label) plus a detent bar `▮▮▯▯`. Label blinks while the surface is moving. Amber if IAS > Vfe for that detent. |
| Gear | `gearPosNorm`, `gearRetractable`, `wowAny` | Fixed gear: `GEAR FIXED` (dim). Retractable: `GEAR DN ●●●` (green, down and locked), `GEAR ↕` (amber, in transit), `GEAR UP` (white). |
| Trim | `elevTrimNorm` (+ rudder/aileron trim if non-zero) | `TRIM ↑ +0.12` (↑ = nose-up trim) plus a 5-tick mini scale with a takeoff band if the profile defines one. Rudder trim `RTRIM L0.05` only when non-zero. |
| Speedbrake / spoilers | `speedbrakePosNorm` | `SPDBRK 50%` when > 0. Amber if extended with throttle > 50%. Hidden when the aircraft has none. |
| Reversers | `engines[].reverserDeployed` | `REV` (amber in flight, green on ground) |
| Brakes | `brakeLeftNorm/RightNorm`, `parkingBrake` | `BRK` while braking. `PARK` while the parking brake is set. |
| Fuel | `fuelTotalLbs` | `FUEL 9 840 LB` |
| Wind | `windFromTrueDeg`, `windKt` | `WIND 270/15` (direction shown magnetic, as pilots expect) |
| Flight time | sim time since start | `T+00:42:10` |

## 5. FMA (flight mode annunciator), H3
- Columns: A/T (`SPD`, `RETARD`, blank) · lateral (`ROL`, `HDG`, `NAV`, `LOC`, plus armed `(LOC)` in
  white small text) · vertical (`PIT`, `ALT`, `ALT*`, `VS +700`, `G/S`, `FLC`, plus armed `(G/S)`,
  `(ALT)`) · `AP` / `FD` / `A/T`.
- A mode change draws a box around the new mode for 5 s.
- AP disconnect: amber flashing `AP OFF` in the warnings area plus a tone (S1) until acknowledged by
  pressing the AP toggle again or after 5 s.

## 6. Nav symbology, H3
- **CDI/LOC:** horizontal deviation scale with dots at ±1 and ±2. Full scale per NavState.cdiDeflection.
  Shows the source label (`GPS`, `LOC`, `VOR`) at the left end.
- **GS / VNAV:** vertical scale on the right of center. ILS GS = filled diamond. Advisory VNAV = hollow.
- **Heading tape marks:** DTK/CRS as a hollow bar, bearing to the active waypoint as ◆, heading bug
  as ▲▼ (AP target). Off-scale marks park at the tape end with the value printed.
- **Flight director:** a guidance cue (small circle) placed relative to the FPV, so flying the FPV
  onto the cue satisfies both pitch and bank commands. Shown when `fdOn && fdValid`.
- **Minimums:** `DA 213` / `MDA 560` in the nav box, a bug on the altitude tape, and `MINIMUMS` (amber, 3 s)
  plus a tone when crossing.
- **Approach runway outline (optional, nice to have):** conformal dashed outline of the destination
  runway when within 10 NM, which helps in low visibility.

## 7. Map page, H4
Full screen, toggled with Select (or M). Opaque background. Pauses nothing.
- North-up or track-up, range 2/5/10/20/40/80/160 NM (D-pad up/down while the map is open).
- Route legs (magenta active leg, white others, dashed missed approach), waypoint idents and
  constraints, airports (runway lines for big ones), navaids, the aircraft symbol with a 1-minute track
  vector, and range rings.
- Optional terrain shading from coarse terrain tiles (colors by height relative to the aircraft).
- Info panel: next waypoints with DTK, distance, ETE, and altitude constraint. Direct-to with A on
  a selected waypoint.

## 8. Acceptance screenshots (H2/H3 must provide these)
1. c172p on the XTST runway, parking brake set, engine idle.
2. Climbing through 1,500 ft, 25° right bank, flaps 10.
3. 737 cruise at FL350, A/T SPD, HDG, ALT, M.78 visible.
4. 737 on ILS 27 at XTST, 1,000 ft AGL, gear down, flaps 30, LOC/G/S captured, RA large.
5. Stall warning with high alpha (amber/red check).
6. MIN and STD declutter versions of #4.
