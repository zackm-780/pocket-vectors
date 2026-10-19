# Flight Sim (working title)

A first-person flight simulator for the **Anbernic RG40XXH** (stock OS, via PortMaster) that also
builds on desktop Linux, macOS and Windows for development.

- **Flight model:** [JSBSim](https://github.com/JSBSim-Team/jsbsim) with its stock aircraft. The fleet
  runs from the Cessna 172 to the Boeing 747.
- **View:** first person with a full HUD and no cockpit model. The HUD shows attitude, speeds, altitude,
  flaps, gear, trim, throttle and engines, brakes, nav, autopilot modes and warnings.
- **World:** real elevation data (Copernicus DEM), land cover, real airports and runways (OurAirports).
- **Navigation:** flight plans (in-game or imported X-Plane `.fms` files), synthesized ILS approaches
  at any runway, real US instrument approaches (FAA CIFP), autopilot and flight director.

## Docs

| Doc | Purpose |
|---|---|
| [docs/PLAN.md](docs/PLAN.md) | Vision, scope, target budgets, decision log, milestones, open questions |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Modules, frame loop, coordinate frames, shared contracts |
| [docs/WORK_PACKAGES.md](docs/WORK_PACKAGES.md) | Work packages for parallel agents, with dependencies, waves and acceptance criteria |
| [docs/AGENTS.md](docs/AGENTS.md) | Agent roles, models and effort levels, supervision and review workflow |
| [docs/CONVENTIONS.md](docs/CONVENTIONS.md) | Code style, units rule, git/PR workflow, definition of done |
| [docs/HUD.md](docs/HUD.md) | HUD layout and every data element it must show |
| [docs/CONTROLS.md](docs/CONTROLS.md) | RG40XXH and keyboard control mapping |
| [docs/NAVIGATION.md](docs/NAVIGATION.md) | Flight plans, approaches, navigator, autopilot |
| [docs/DATA.md](docs/DATA.md) | Data sources, licenses, offline pipeline, on-disk formats |
| [docs/BUILD_AND_DEPLOY.md](docs/BUILD_AND_DEPLOY.md) | Desktop builds, headless testing, aarch64 build, PortMaster packaging |
| [docs/DEVICE.md](docs/DEVICE.md) | RG40XXH facts: known specs and values still to be measured on the device |
