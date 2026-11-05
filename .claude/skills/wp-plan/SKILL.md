---
name: wp-plan
description: Plan or refine one Pocket Vectors work-board card - an outline card, a card waiting on new device numbers, or a new card from a device-session bug note. Use when the user types /wp-plan <ID or bug note>.
---

# Plan one card

You are the planner. Planning is the one step where reading code is expected, so that builders
don't have to: the card you write must let a builder work without exploring.

1. Read `flightsim/docs/wp/README.md`, `flightsim/docs/wp/BUILDER.md`, the card or bug note you
   were given, and `flightsim/docs/wp/X3a.md` as the format example.
2. Read only the code and spec sections you need to fill the card's parts: **Read** (exact files
   with line ranges or grep anchors), **Owns**, **Also touches**, **Do** (numbered and concrete),
   **Acceptance** (each item testable on desktop), **Device checks**.
3. Size it for one builder and one pull request. If it is larger, split it into `<ID>a`,
   `<ID>b`, … with Depends between them.
4. Check every anchor you cite with `grep` before you write it down.
5. Set the card's first line to `Status: ready` (or `Status: blocked (<what>)`), and update the
   cards table, the phases and the hot files in README.md.
6. Commit on branch `wp/plan-<ID>`, push it, and open a docs-only pull request titled
   `[plan] <ID>: <title>`. Don't build and don't change code.

Report in at most 6 lines: the cards written, their effort, their hot files, and anything the
owner has to decide.
