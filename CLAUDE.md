# Agent guide: Pocket Vectors

This repo is **Pocket Vectors**, a first-person flight simulator for the **Anbernic RG40XXH**
handheld running its **stock OS**, which runs it through **PortMaster**. The source and the
on-device port live in **`flightsim/`** (that folder name, `FlightSim.sh` and `flightsim.aarch64`
stay stable so a card update keeps `conf/` and `data/`). The game has a full HUD, real terrain,
real airports, flight plans and instrument approaches, built on JSBSim.

Several agents build it in parallel, one **work package (WP)** per agent. This file is the entry
point. Read it fully before touching anything.

## Read before you write code

0. `ROADMAP.md`: what is open, and `CONTRIBUTING.md`: how a change is reviewed and merged.
1. `flightsim/docs/PLAN.md`: what we are building, why, decisions, milestones.
2. `flightsim/docs/ARCHITECTURE.md`: modules, frames and units, shared contracts, frame loop.
3. `flightsim/docs/CONVENTIONS.md`: code style, units rule, git and PR workflow, definition of done.
4. Your WP entry in `flightsim/docs/WORK_PACKAGES.md`, plus every spec doc it lists
   (`HUD.md`, `CONTROLS.md`, `NAVIGATION.md`, `DATA.md`, `BUILD_AND_DEPLOY.md`, `DEVICE.md`).
5. `flightsim/docs/AGENTS.md`: how a contributor or an AI coding tool takes one work package.

## Hard rules

- **One WP per agent.** Work only on your WP. Edit only the paths it **owns**. If you need a change
  elsewhere, describe it in the pull request and let a maintainer route it.
- **Shared contracts** (`flightsim/src/core/`) change **additively only** (new fields or functions with
  defaults). Renaming, removing or changing the meaning of anything there needs maintainer approval
  first.
- **Units in names.** Every physical quantity carries a unit suffix (`altMslFt`, `iasKt`,
  `rollRad`, `distNm`). Every heading, course, track or bearing says `True` or `Mag`.
- **No generated data in git**: no terrain tiles, nav databases or downloads. Tests use synthetic
  fixtures (< 1 MB) under `flightsim/tests/fixtures/` or generate data in memory.
- **Don't modify JSBSim** sources or its stock aircraft. Fixes go in
  `flightsim/assets/jsbsim-overlay/`, which is copied over the stock files at install time.
- **No new third-party dependency** without a decision-log entry in `PLAN.md` and a license check.
- **You cannot test on the device.** Prove behaviour on desktop with unit tests, headless scenario
  runs, traces and screenshots, and mark the PR "needs device test" when on-device behaviour matters.
- Never commit secrets. Never push to `main` directly.
- **Contributions follow [CONTRIBUTING.md](CONTRIBUTING.md).** Fork the repo, open a pull request,
  and wait for CI, including the device workflow. The owner reviews and approves before merge.
  Do not merge the pull request yourself.

## Build and test (available once WP-00 has landed)

```bash
cd flightsim
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo
cmake --build build
ctest --test-dir build --output-on-failure
# headless sim run with a CSV trace
./build/flightsim --headless --scenario assets/scenarios/c172_final_xtst.ini --duration 60 --trace /tmp/trace.csv
# render a screenshot without a display (Mesa software GL)
xvfb-run -s "-screen 0 1280x960x24" ./build/flightsim --scenario assets/scenarios/c172_final_xtst.ini --frames 120 --screenshot /tmp/shot.png
```

See `flightsim/docs/BUILD_AND_DEPLOY.md` for device (aarch64) builds and PortMaster packaging.
