# Conventions

## 1. Code style
- **C++17.** Format with the repo `.clang-format` (from WP-00): 4-space indent, 100 columns, braces
  attached.
- **Naming:** types `PascalCase`; functions and variables `camelCase`; private members end in `_`
  (`state_`); constants `kName`; macros `FSIM_NAME`; namespace `fsim` (sub-namespaces per module
  allowed, e.g. `fsim::nav`).
- **Files:** `PascalCase.h/.cpp` per class. Headers use `#pragma once`. Keep includes minimal and
  include what you use.
- **No globals** except the logger. Pass dependencies through constructors.
- **Hot paths** (per frame or per sim step): no heap allocation, no file IO, no exceptions, no
  `std::string` building. Reuse buffers.
- **GL calls** only on the main thread, and only in `gfx/`, renderers, `hud/` and `ui/`.
- Prefer plain structs and free functions over deep class hierarchies. Interfaces only where
  `core/` defines them.
- **Python tools:** Python ≥ 3.10, type hints, `argparse` CLIs, `--help` that explains every
  option, deterministic outputs, caches in `tools/**/cache/` (git-ignored), and pinned dependencies in
  `tools/requirements.txt`.

## 2. Units and angles (mandatory)
- Every variable, field or parameter holding a physical quantity carries a unit suffix:
  `M`, `Km`, `Ft`, `Nm` (nautical miles), `Kt`, `Mps`, `Fpm`, `Fps`, `S`, `Ms`, `Deg`, `Rad`,
  `Radps`, `Lbs`, `Lbf`, `Pct`, `Norm` (0..1 or −1..1), `G`, `Hpa`, `InHg`.
  Example: `float altMslFt`, `double distM`, `float iasKt`.
- **Headings, courses, tracks and bearings must say `True` or `Mag`:** `headingTrueRad`,
  `courseMagDeg`. True is used internally. Magnetic is only for display and input.
- Altitudes say their reference: `Msl`, `Agl` or `Pressure` (later).
- Use radians in math and degrees in data files, configs and display. The suffix makes it obvious.
- Conversions only via `core/Units.h` helpers. No magic numbers like `0.3048` inline.

## 3. Sign conventions
- Elevator +1 = nose up (stick back). Aileron +1 = roll right. Rudder +1 = yaw right.
- Pitch + = nose up. Roll + = right wing down. Vertical speed + = climbing.
- CDI deflection + = course is to the right. GS deflection + = glidepath is above the aircraft.
- Cross-track + = aircraft is right of course.

## 4. Testing
- **doctest** for C++ unit tests in `tests/<module>/`. Every WP adds tests for its non-GL logic.
- **Headless scenarios** (`--headless --scenario … --trace …`) for anything that flies. Assert on
  traces in C++ tests (helper provided by WP-00).
- **Visual work:** produce screenshots with `--screenshot` under Xvfb, and attach or link them in
  the report or PR. Store canonical screenshots only if the supervisor asks (they go out of date).
- Tests must not need network access or real data packs. Use synthetic fixtures and XTST.
- A test that is flaky gets fixed. It is never skipped.

## 5. Git and PRs
- One WP per branch and PR. Branch `wp/<WP-ID>-<slug>`, or your session's designated branch.
- Small, focused commits in imperative mood: `nav: add FMS v11 parser`.
- **PR body:** what and why · how it was verified (tests, traces, screenshots) · deviations from
  spec · performance impact · docs updated · `[ ] needs device test`.
- Merge `main` into your branch to stay current. Never rewrite history others depend on.
- Never commit generated data, downloads, build outputs or secrets (see `.gitignore`).

## 6. Definition of done
1. Acceptance criteria in WORK_PACKAGES.md are met and demonstrated.
2. Builds with zero warnings (CI uses `FSIM_WERROR=ON`). All tests pass.
3. Only owned paths changed. Contract changes are additive and mirrored in ARCHITECTURE §5.
4. Docs updated: the WP `Status:` line, plus spec or format sections the WP owns.
5. Supervisor review passed (AGENTS.md §2).

## 7. Licenses
Third-party code and data must be compatible with free redistribution in a PortMaster port. Every
shipped dependency's license goes into `port/licenses/`. Data attribution goes into
`data/*/pack.ini`/`nav.ini` and the in-game About screen.
