# Agents: roles, models, workflow

The flight sim is built by several AI agents working in parallel. This doc sets who does what,
which model and effort each role uses, and how work moves from a work package (WP) to `main`.

## 1. Roles

| Role | Model | Effort | Responsibilities |
|---|---|---|---|
| **Supervisor** (lead / planner / reviewer) | **Opus** | **extra high (`xhigh`)** | Owns the plan and these docs. Schedules waves, writes agent briefs, guards the `core/` contracts, reviews every WP against its acceptance criteria, runs builds and tests, sends fixes back to implementers, merges, resolves cross-WP integration, and updates PLAN, ARCHITECTURE and WP statuses. Decides when a WP needs splitting or re-planning. |
| **Implementer** | **Sonnet** | **medium** or **high**, as set per WP in [WORK_PACKAGES.md](WORK_PACKAGES.md) | Implements exactly one WP inside its owned paths. Writes tests. Proves acceptance with tests, traces and screenshots. Reports back with a summary of what changed, what was verified, and open issues. |
| **Reviewer** (optional, for large WPs) | **Opus** | **extra high** | A second Opus pass on high-risk merges (WP-00, W2, N5, N6, F1) when the supervisor wants an independent check: correctness, contract adherence, performance, hidden coupling. |

### How effort was assigned
- **Sonnet / high:** foundations and anything with subtle math, concurrency, rendering, or physics
  tuning, where mistakes are expensive to find later: WP-00, P1, F1, F2, F3, W1, W2, W3, W5, N4,
  N5, N6, H2, X3.
- **Sonnet / medium:** well-specified, self-contained work with clear tests: P2, P3, F4, W4, N1,
  N2, N3, H1, H3, H4, H5, S1, X1, X2, X4.
- The supervisor may raise a WP's effort to high after a failed review round, or escalate a
  single hard sub-problem to an Opus implementer. Escalation is the exception, not the default.

## 2. Workflow

```
Supervisor (Opus xhigh)                     Implementer (Sonnet medium/high)
───────────────────────                     ───────────────────────────────
1. Pick ready WPs (deps merged)
2. Brief each agent (template §3)  ───────▶  3. Read CLAUDE.md + docs + WP entry
                                             4. Implement in owned paths, on its own branch/worktree
                                             5. Build, test, headless runs, screenshots
                                  ◀───────  6. Report: summary, evidence, deviations, open issues
7. Review: diff, contracts, acceptance,
   build + tests yourself, screenshots
8a. Changes needed → send back      ───────▶ (repeat 4–6)
8b. Accept → merge into the integration
    branch, run full build/tests, update
    the WP Status line and any docs
9. Next wave
```

### Branches
- **Cloud sessions** usually have a designated branch. Use it. Otherwise use `wp/<WP-ID>-<slug>`.
- **Subagents inside one supervisor session** each work in an isolated **git worktree** on a local
  branch `wp/<WP-ID>-<slug>`. They commit there and never push. The supervisor reviews, merges into
  the session's integration branch, and pushes.
- PR titles are `[<WP-ID>] <title>`. One WP per PR. `main` only receives reviewed work.

### Review checklist (supervisor)
- [ ] Only owned paths touched. `core/` changes are additive and mirrored in ARCHITECTURE §5.
- [ ] Builds clean with warnings as errors. All tests pass, including other WPs' tests.
- [ ] The **device** build passes too: GCC 10 (Debian 11, aarch64) reports warnings the desktop
  compiler does not (e.g. `maybe-uninitialized`). Check the `device` workflow on every push, or run
  `tools/build_device.sh --chroot <dir>` locally before merging C++ changes.
- [ ] Every acceptance criterion is demonstrated (test names, trace excerpts, screenshots).
- [ ] Unit suffixes and True/Mag naming are respected (CONVENTIONS §2).
- [ ] No per-frame allocations or blocking IO on the main thread in hot paths. Budgets respected.
- [ ] No new dependencies without a decision-log entry. No generated data committed.
- [ ] Docs updated: WP Status line, any format or spec changes, the "needs device test" note.

## 3. Implementer brief template

```
You are a Sonnet implementer agent on the RG40XXH flight sim. Your work package is <WP-ID>: <title>.

1. Read CLAUDE.md, flightsim/docs/PLAN.md, ARCHITECTURE.md, CONVENTIONS.md, AGENTS.md, and the
   <WP-ID> entry in flightsim/docs/WORK_PACKAGES.md, plus every spec doc it lists.
2. Work only in the paths <WP-ID> owns. Commit on branch wp/<WP-ID>-<slug> (or your session's
   designated branch). Do not push unless told to.
3. Meet every acceptance criterion and prove it (test names and output, trace excerpts, screenshot paths).
4. If a src/core/ contract must change in a non-additive way, or a spec is ambiguous or wrong,
   stop and report instead of guessing.
5. Final report: what you built (files), how you verified it, deviations from the spec,
   known issues, suggested follow-ups, and whether it needs a device test.
<extra context from the supervisor: current state, gotchas, related WPs in flight>
```

## 4. Concurrency rules
- WPs in the same wave must not own overlapping paths. WORK_PACKAGES.md is written to guarantee
  this. If two WPs need the same file, the supervisor splits the file or sequences the WPs.
- `core/` is owned by the supervisor after WP-00. Implementers propose additive changes in their
  report, and the supervisor applies or approves them.
- Shared docs: implementers edit only their WP's `Status:` line and the doc sections their WP owns
  (for example, N4 owns DATA §7). Everything else goes through the supervisor.
- The device is a shared, human-only resource. Batch "needs device test" items per milestone and give the
  user one checklist.

## 5. Usage budget
All agents (implementers and the supervisor) share one account usage limit. Every agent step
re-sends that agent's whole conversation, so a session's cost grows with its length times its
number of steps, and running more agents at once does not finish more work per usage window: it
only reaches the limit sooner. Each resume after a limit stop also replays the full transcript.
- **At most 3 implementer agents at a time.** Start the next WP when one is merged.
- **Small WPs.** If a WP will not fit comfortably in one session, the supervisor splits it.
- **Targeted reading.** The brief names the doc sections to read. Read your own WORK_PACKAGES
  entry, not the whole file. Read source files in the ranges you need.
- **Short tool output.** Pipe builds, tests and traces through `tail` or `grep`. Never print whole
  logs, CSV traces or large diffs.
- **Images only to check something.** Open a screenshot only when verifying it, and prefer the
  640×480 version.
- **Effort:** medium by default. High only where the WP's math or physics needs it (see §1).
