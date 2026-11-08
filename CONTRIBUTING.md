# Contributing to Pocket Vectors

Contributions are welcome. The game is licensed under Apache-2.0.

## Pull requests

Every outside contribution goes through a **fork** and a **pull request** into `main`.

1. Fork [zackm-780/pocket-vectors](https://github.com/zackm-780/pocket-vectors) and create a branch
   for the change.
2. Open a pull request. One change per pull request keeps review possible.
3. CI has to pass. That includes the desktop build and tests, and the **device** workflow
   (Debian 11, GCC 10, `-Werror`, the aarch64 package checks and the qemu smoke test).
4. The owner reviews the pull request and approves it before it is merged. Do not merge it
   yourself.

Read [CLAUDE.md](CLAUDE.md) and [flightsim/docs/CONVENTIONS.md](flightsim/docs/CONVENTIONS.md)
before writing code. Open work is listed in
[flightsim/docs/WORK_PACKAGES.md](flightsim/docs/WORK_PACKAGES.md).

## AI agents

An AI agent working on a contribution follows the same path as a person:

- Fork, branch, and open a pull request. Do not push to `main`.
- Wait until CI is green, including the device workflow.
- Leave the pull request for the owner's review and approval. Do not merge it.

The owner's own agent sessions work differently (owner, 2026-11-08): they branch in this
repository, and once every check is green, including the device workflow, the agent merges its
pull request. The owner device-checks `main` rather than each branch, and tags a release only once
`main` is stable. Agents never tag.

The on-device folder stays `flightsim/`, with `flightsim.aarch64`, so an update does not drop a
player's `conf/` or `data/`. The launch script is `PocketVectors.sh` from 1.2.2 (it was
`FlightSim.sh`), because the Ports menu lists scripts by file name.

## What to put in the pull request

- What changed and why.
- How you verified it: tests, headless scenario runs, and screenshots for anything visible.
- Whether someone still needs to try it on an RG40XXH.
