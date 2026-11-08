# Changelog

## 1.3.0

Fly-by turns flown as arcs, fixes from the owner's 2026-11-08 flights, runways that meet the
terrain and each other smoothly, and a flight log. (Planned as 1.2.3 and 1.2.4; shipped together
as 1.3, owner 2026-11-08.)

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
