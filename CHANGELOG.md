# Changelog

## 1.4.0

The owner's handling, HUD and flight-plan list (2026-11-09), arcs at every fly-by turn
(2026-11-10), and the Seattle to San Francisco pack as the release's data pack.

- **Stall warning by flap setting** (F5). The airliners warned at about 12 degrees of pitch and in
  steeper banks, above their stall speeds. Now each flap detent has its own warning angle of attack,
  set at max(1.05 × Vs, Vs + 6 kt) for that detent (just over FAR 25.207). It interpolates with the
  real flap position, and the HUD and the flight model use the same value. 737, B747, A320, 787-8,
  MD11 and global5000; the light aircraft keep one value. No warning in normal climbs or 30-degree
  turns.
- **V-speed and flap bugs on the speed tape** (H9):
  - on takeoff: V1, VR and V2 (pistons VR only);
  - on approach: REF and APP;
  - at each flap detent: the minimum speed for it (1.3 × its stall speed, never above the next
    detent's Vfe), and in the descent the clean minimum.
  - All of them scale with the square root of the weight. The chase view shows the next bug
    under the speed box.
- **Route and path overlays follow the HUD declutter level** (H10). OVERLAY ROUTE and OVERLAY PATH are
  AUTO (shown at the FULL HUD), ON or OFF; L3 still cycles the presets.
- **Flight-plan edits like a Garmin or an FMS** (N11a, N11b). On the FLIGHT PLAN screen in flight:
  - **Direct-To:** a waypoint in the plan flies on along the plan; a fix off the plan rejoins it at
    the next waypoint ahead (`DIRECT TO X, THEN Y`); the destination keeps its arrival and
    approach; another airport replaces the route and offers its approaches (NOT NOW flies direct).
  - **NEW DESTINATION** asks KEEP ROUTE or CLEAR ROUTE (CLEAR is selected, B cancels).
  - **DEPARTURE RUNWAY** on the ground keeps the SID when it serves the runway, and ACTIVATE
    APPROACH picks the approach back up.
  - **On a leg:** A goes direct to it, X activates it, Y deletes it.
  - **One press:** L1 + Select is DIRECT TO NEXT, after vectors or a heading.
  - **Undo:** R1 + Start (Ctrl+Z) undoes a route change within 10 s. The HUD says what each edit
    did, with R1+START UNDO under a destructive one.
  - The autopilot stays in NAV through every edit. On the setup screens a new start runway
    keeps the plan, and a new destination asks KEEP / CLEAR too.
- **Arcs at every fly-by turn** (N10b, the owner's device feedback). The turn from the route onto an
  approach (an IF in mid-plan), DF legs and straight CF legs now get arcs too. An approach that starts
  on the last enroute fix (the real KOTH I05-Z at OTH) turns on an arc onto its next leg.
- **PA28 balked landing** (F8). The stock model could not climb with full flaps at full power
  (−200 fpm at flaps 40, −320 at 25: the flaps pitched the nose down past the elevator). An overlay
  reduces the flap pitching moment and fixes the 25-degree flap lift. Flaps 40 now climbs about
  +130 fpm, 25 +260 and 10 +410, and the clean airplane is unchanged. The first-flight help gives
  the go-around: full power, flaps 25 at once, then 10.
- **Pitch handling** (F9). Small stick inputs were far too strong: in the C172, 0.2 of stick changed
  the VS by 1,550 fpm in 3 s.
  - **Stick:** the left stick's pitch now has its own expo (`[axes] left_pitch_expo`, and per
    aircraft `[controls] pitch_expo`), so 0.2 is now about 450 fpm. Roll and full deflection are
    unchanged.
  - **Trim:** a trim press is one step (a tap moves the VS by 30–150 fpm); held, it runs at a
    steady rate. Both shrink with speed, so a tap does about the same at any speed.
  - **Aids (settings, off by default):** PITCH DAMPER, and AUTO TRIM (now wired), which holds
    the climb or descent you leave the stick at and calms the phugoid.
- **Armed speed brakes** (F10). X cycles RETRACTED, ARMED, 50 %, FULL. Armed (SPDBRK ARM on the HUD),
  they deploy fully at touchdown with the thrust at idle and stow on a go-around (737 at XTST:
  rollout 2,400 ft against 3,370 ft unarmed).
- **Nose-wheel steering for taxiing** (F11).
  - **The tiller:** below 35 kt the right stick is a tiller up to the aircraft's real limit
    (light aircraft 30 degrees, airliners 60–78). It is held to 0.3 g at the ground speed, on a
    curve that keeps small moves small and at a hydraulic rate. At walking pace a C172 turns on
    about 7 m, an A320 on 20 m and a B747 on 14 m; before, the A320 needed 136 m and the B747
    337 m.
  - **Above 35 kt** the pedal steering is unchanged.
  - **Overlays:** the C310 and the Global 5000 now steer on the ground at all (their JSBSim
    autopilots overwrote the steering), and the C182 and PA28 no longer pivot on castering main
    wheels.
- **The release's data pack is Seattle to San Francisco** (owner, 2026-11-10). From 1.4 the release
  attaches only `pocket-vectors-v1.4.0-data-sea_sfo.zip` (with SIDs, STARs and taxiways at 211
  airports) and the one-zip install `pocket-vectors-v1.4.0-Roms-PORTS.zip`. It covers all of the
  Pacific Northwest pack that 1.2 and 1.3 attached, and that pack keeps working.

## 1.3.0

Taxiways, fly-by turns flown as arcs, fixes from the owner's 2026-11-08 flights, runways that
meet the terrain and each other smoothly, and a flight log. (Planned as 1.2.3 and 1.2.4; shipped
together as 1.3, owner 2026-11-08.)

- **Taxiways and aprons** (W8a, W8b, W8c), so you can taxi to the runway instead of back-taxiing
  on it.
  - **What you see:** real taxiways from OpenStreetMap, drawn as pavement ribbons with round
    fillets where they meet, and the aprons around the terminals. They carry yellow centerlines,
    and edge lines that break at junctions and on the ramps.
  - **Where:** 47 of the 50 airports with a hard runway in the `pdx_oth` pack (2,930 taxiway
    segments, 116 aprons). KPDX has its whole network (553 segments, 15 aprons), and KOTH has its
    taxiways and ramps.
  - **The ground:** taxiways sit on ground as smooth as the runways and meet them without a step
    (no slope steeper than about 1:110 at KPDX). Their heights come from the runways, so loading
    terrain never moves them.
  - **Drawing:** they are drawn out to 5 NM, in two more draws per airport (one at KPDX, where the
    generic apron goes away). Airports without OpenStreetMap taxiways keep the generic apron and
    stub.
  - **Licence:** taxiway data © OpenStreetMap contributors. The pack's `nav/taxiways.csv` and
    `nav/aprons.csv` are shared under the Open Database License (`nav/TAXIWAYS-ODbL.txt`), and
    the About page carries the credit.
  - **Adding them to an existing install:** copy `data/nav/` from the 1.3 data pack over
    `flightsim/data/nav/` on the SD card. Older packs still work, with generic aprons.
  - **More regions:** the `datapack` workflow builds taxiways for any region (on by default).
- **Seattle to San Francisco pack on the release** (owner, 2026-11-09; R3). A new `sea_sfo` preset
  covers the coast and the Cascades from Everett to San Jose: KSEA, KPDX, KEUG, KOTH, KSFO and the
  fields between in one pack (about 1.3 GB, taxiways at 211 airports), so you can fly between them
  without swapping packs. The release attaches it as `pocket-vectors-v1.3.0-data-sea_sfo.zip` and as a
  one-zip install with the game (`pocket-vectors-v1.3.0-Roms-PORTS-sea_sfo.zip`). It holds all of the
  Pacific Northwest pack, so use one or the other. Each pack now carries a `REGION.txt`.

- **GPS turns are arcs** (N10). At each fly-by waypoint the route turns along an arc tangent to
  both legs, flown at a constant bank: 15 degrees in the light aircraft, 20 in the jets (the
  autopilot's bank limit less 5, so it keeps room for wind). The AP and the flight director fly
  it with the bank fed forward and roll out on the next leg without overshoot, instead of
  cutting a sharp corner and intercepting the next leg. The map page, the mini nav and the route
  overlay draw the arcs, and the nav box shows the course you are turning onto. Turns over 135
  degrees, fly-over waypoints and course or hold legs are flown as before.
- **Flight director follows the GPS track.** A mode button pressed with the AP and the FD off
  turns the FD on in ROL with NAV armed, and NAV only captured on a closing intercept. Off course
  and heading away, the FD held the wings level for good, so following it took you away from the
  track (from the KPDX-KOTH cruise start: 26 NM off after 15 minutes). NAV armed over ROL now
  captures at once and flies its own intercept (up to 45 degrees). Over HDG it still waits for
  your intercept heading. NAV no longer captures on the ground, so the FD does not ask for bank
  on the takeoff roll. A pilot following the FD now tracks the plan as closely as the AP does.
- **Smooth runway intersections** (W1b). Each runway was flat at the mean of its two end
  elevations, and where two crossed, one simply ended in a step: 0.6 m where 10R/28L meets 03/21
  at KPDX, enough to wreck a fast roll down 28L. Runways now follow the grade between their ends,
  crossing runways blend into each other with no step (under about 1:130), the runway areas blend
  into the terrain over 150 m instead of 60 m, and the pavement, markings and lights sit exactly
  on the ground the wheels feel. Every airport with crossing runways gets the same treatment.
- **Flight log** (U2): FLIGHT LOG on the main menu lists every flight (when, the aircraft, from
  and to, how long and how it ended: LANDED, CRASHED, IN FLIGHT ...), with the details (distance,
  maximum altitude, landings, the last touchdown in fpm, fuel used) on A. The log is
  `flightsim/conf/flightlog.csv`, so an update keeps it. A flight that ends with Select + Start is
  logged as GAME CLOSED at the next start.
- **Flight plan screens:** the NO FLIGHT PLAN line, the divider and the route summary no longer
  sit behind the APPROACH and CLEAR PLAN rows, and in flight the DIRECT TO hint and the fuel line
  no longer overlap other text.
- README screenshots from the current build.

## 1.2.2

- **Pocket Vectors in the Ports menu:** the launch script is now `PocketVectors.sh`, with box art
  for the stock OS (copied to `Roms/PORTS/Imgs/` the first time the game starts) and a
  `screenshot.png` for PortMaster. The `flightsim/` folder keeps its name, so `conf/` and `data/`
  stay. **After updating, delete the old `FlightSim.sh`**, or the game is listed twice.
- **One-zip install:** the release also carries `pocket-vectors-vX.Y.Z-Roms-PORTS.zip`, the game
  and the Pacific Northwest pack together, to unzip straight into `Roms/PORTS`.
- **1.2.1 measured on the RG40XXH (2026-11-07):** MED holds 59 fps in the cockpit (1.2.0: 30) and
  60 inside a cloud; the HUD takes 3.2 ms of CPU (8.0). Under an overcast layer and while orbiting
  the chase camera it still drops to 30.

## 1.2.1

Device tuning from the 1.2 timing run on the RG40XXH (X3c), and the flight data in the chase view.

- **Chase view:** speed, altitude, vertical speed and heading now show in the chase view, where
  the cockpit HUD has them (the tapes' value boxes, the heading box, GS / Mach, VS and the
  altimeter setting), plus the autopilot modes while the AP or the FD is on. No tapes or pitch
  ladder, so the aircraft stays the focus.
- **HUD CPU time:** the HUD is built about three times faster (the same picture, bit for bit). It
  took 8 ms of CPU per frame in the cockpit on the handheld.
- **GRAPHICS QUALITY MED** draws the terrain's detail noise from a texture: the same kind of
  noise for much less GPU work. HIGH keeps the 1.2.0 shader, LOW still has none.
- The sky is drawn after the terrain, so the GPU no longer shades it where the terrain covers it.
- **Performance log:** the launch log shows the GPU clock (`gpu clock:` lines, and
  `gpu <cur>/<max> MHz` on the SHOW FPS lines), and the draw count includes the airports.

## 1.2.0

- **1.1 device-test fixes.** The gear no longer moves when a menu closes with A. L2 + D-pad
  snaps the view and L2 + right stick looks around. R1 + Y cycles the nav source and L1 + Select
  goes direct to the active waypoint. Runway outlines clear below 250 ft above the field.
  L1 + R2 in ALT hold climbs or descends toward the altitude bug.
- **Autopilot TO/GA and FLC:** L1 + L3 goes around and flies the missed approach; L1 + Start
  climbs or descends on speed to the altitude bug.
- **Chase view:** L2 + L3 shows the aircraft from behind; L2 + right stick orbits.
- **Cloud layers** from the CLOUDS setting, the scenario or the METAR.
- **SIDs, STARs and approach transitions** on the flight-plan screen. They need a data pack built
  with the new `cifp.py`; older packs still load without them.
- **Spoken GPWS callouts** (MINIMUMS, 500 … 10, SINK RATE, PULL UP and more). GPWS VOICE OFF
  keeps the tones.
- **Fuel:** a FUEL row on the new-flight screen (AS LOADED, FULL, 75 %, 50 %, 25 %, AUTO for the
  plan plus reserves), endurance on the HUD, and LOW FUEL warnings.
- **Performance tools:** SHOW FPS shows the frame rate on screen, and the launch log gets timing
  tables. GRAPHICS QUALITY LOW / MED / HIGH sets the terrain detail (MED is the 1.1 look).
- The About page shows the real version (the 1.1.0 build still said 0.1.0).

Fixes from the 1.2 device test (2026-11-05):

- **Route overlay:** the magenta line climbs out of the departure runway and descends to the
  landing runway at 3°, through the published approach altitudes, instead of dropping straight
  down from cruise onto the runway. It clears below 250 ft above the field on the approach, like
  the path and the runway outlines. The runway waypoint has no pole, and the missed approach shows
  once you fly it.
- **Autopilot:** HDG, NAV, APR, ALT, VS and FLC turn the flight director on when the AP and the
  FD are both off, like TO/GA. L1 + R2 used to work only with the AP on.
- **Audio:** a 1024-sample buffer on the handheld against crackle (ALSA underruns);
  `audio_buffer_samples` in `settings.ini` overrides it, and the log shows the size in use.
- **SD cards written on a Mac:** `._*` files no longer show up (or warn) in the aircraft and
  flight-plan lists.
- **Frame rate on the RG40XXH:** MED (the default) holds 30 fps in the cockpit, LOW 60, HIGH 30.
- **Data pack:** the release carries a Pacific Northwest pack with SIDs and STARs
  (`pocket-vectors-v1.2.0-data-pdx_oth.zip`). To add them to an existing install, copy its
  `data/nav/` over `flightsim/data/nav/`.

## 1.1.0

- **Buttons match the printed labels on the RG40XXH.** A = gear, B = brakes, X = speedbrake,
  Y = HUD declutter. Select works whether the firmware reports it as `guide` or `back`, so the
  Menu key also acts as Select. R3 is not bound in flight. If you created
  `flightsim/conf/controls.ini` for the 1.0 workaround, delete it.
- **Help menu** on the main menu and the pause menu: the live control list, a button test with
  Swap A/B and Swap X/Y, how-to pages, and About with the build and the data attribution.
- **Mini nav:** a small track-up route map in a HUD corner. Off by default; turn it on with
  MINI NAV in the pause menu.
- **World overlays:** route poles, vertical-path gates, nearby airports and brighter runway
  outlines. L3 cycles OFF, ROUTE, ROUTE+PATH, AIRPORTS and ALL. All layers start off.
- **Release tooling:** `flightsim/tools/sdcard/make_sdcard_zip.py` combines a game zip, data
  packs and `.fms` plans into one `Roms/PORTS` zip. Tagged releases publish the game only.
- The About page shows the real version (it said 0.1.0 in 1.0).

## 1.0.0

First release. JSBSim flight model with the stock fleet from the Cessna 172 to the 747, a full
HUD, worldwide airports with markings, lights and PAPI, FAA approaches, fixes and airways, a
synthesized ILS for any runway, flight plans built in game or loaded from `.fms`, a moving map,
autopilot, weather, terrain warnings, night lighting and procedural audio. Terrain for the
Portland (KPDX) to North Bend (KOTH) region in the `pdx_oth` data pack.
