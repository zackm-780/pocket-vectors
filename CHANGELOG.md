# Changelog

## 1.2.0 (not released yet: waiting for the device test)

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
