# Builder rules: one card, one pull request

You were handed one card, `flightsim/docs/wp/<ID>.md`. For card work, these rules replace the long
reading list in CLAUDE.md. The hard rules in CLAUDE.md still apply.

## 1. Read only what the card names

- Read your card first. Then read only what its **Read** list names, in the ranges it gives.
- For any file over about 300 lines, find the spot with `grep -n` and read a window around it
  (`sed -n 'A,Bp' file`) instead of the whole file.
- Don't open PLAN.md, ARCHITECTURE.md, WORK_PACKAGES.md, the board or other cards unless your card
  points at a section of them.
- Don't explore to "get familiar". If a pointer in the card is stale, find the right spot with
  `grep`, then carry on.

## 2. Stay in scope

- Change only the paths under **Owns**, plus the small edits listed under **Also touches**.
- `src/core/` changes stay additive: new fields with defaults, new enumerators at the end of an
  enum, new functions. Add one line for each to ARCHITECTURE §5 (find the struct with `grep -n`).
- Every physical quantity carries a unit suffix (`altMslFt`, `iasKt`, `distNm`, `durS`), and every
  heading, course, track or bearing says `True` or `Mag`. Conversions go through `core/Units.h`.
- Hot paths (per frame or per sim step): no heap allocation, no file IO, no `std::string` building.
- No new third-party dependency. No generated data in git. Don't modify JSBSim.
- Tests use synthetic fixtures or XTST. Nothing may need the network or a data pack in CI.

## 3. Build and test quietly

Build and test output is long. Send it to files in `build/` and print only the verdict or the
first errors:

```bash
cd flightsim && mkdir -p build
cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=RelWithDebInfo -DFSIM_WERROR=ON \
  > build/_cfg.log 2>&1 || tail -n 30 build/_cfg.log
cmake --build build > build/_build.log 2>&1 && echo BUILD OK \
  || grep -m 25 -E 'error|Error' build/_build.log
ctest --test-dir build -R '<module>' > build/_test.log 2>&1 && tail -n 3 build/_test.log \
  || grep -m 8 -B 2 -A 12 -E 'FAILED|ERROR|failed' build/_test.log
git diff --name-only origin/main... -- '*.h' '*.cpp' \
  | xargs -r clang-format --dry-run --Werror 2>&1 | head -n 20
```

- The first build fetches and compiles JSBSim and takes several minutes. Later builds are
  incremental. Never stream the build to your context.
- Run the full `ctest` once, quietly, before you push.
- Screenshots: `xvfb-run -a -s "-screen 0 640x480x24" ./build/flightsim --scenario <ini> --frames
  120 --screenshot build/<name>.png`. Open an image only when the card asks for a visual check,
  at most three per card.
- Don't run `tools/build_device.sh` or Docker. The device workflow in CI builds aarch64.
- If the same error survives two honest fixes, stop and report it. Don't loop.

## 4. Finish

1. Set your card's first line to `Status: done (needs device test)` when the card has device
   checks, or to `Status: done` when it has none. The merge makes it true.
2. Commit in small imperative commits (`hud: add …`). Right before pushing, merge `origin/main`
   into your branch; if anything came in, rebuild and rerun the tests quietly.
3. Push `wp/<ID>-<slug>`. If `gh auth status` succeeds, open the pull request titled
   `[<ID>] <card title>`. The body has: what changed and why, how it was verified (test names,
   numbers), deviations from the card, and the card's device checks as a `[ ] needs device test`
   list. Otherwise leave the pull request to the orchestrator.
4. Don't wait for CI, and don't merge.

## 5. Report: your final message, at most 10 lines

```
<ID> done | blocked
Branch: wp/<ID>-<slug>   PR: <url | not opened>
Tests: <passed>/<total>; new: <test names>
Acceptance: <one short line per item: met, or not met and why>
Deviations: <none | one line>
Needs the owner: <none | the question or decision>
```

No diffs, no logs, no file lists. The orchestrator reads only this.
