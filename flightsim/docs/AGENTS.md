# Guide for contributors and AI coding tools

Pocket Vectors is built as a set of work packages (WPs) listed in
[WORK_PACKAGES.md](WORK_PACKAGES.md). A person or an AI coding tool takes **one** package, stays
inside the paths it owns, and opens a pull request.
[CONTRIBUTING.md](../../CONTRIBUTING.md) is the rule for how that pull request lands: a fork,
green CI including the device workflow, and maintainer review before merge. Do not merge the
pull request and do not push to `main`.

## 1. Before editing

**Work after 1.0 is on the card board** ([wp/README.md](wp/README.md)). A card holds everything
its builder needs. For a card, read [wp/BUILDER.md](wp/BUILDER.md) and the card, and nothing else
up front. The brief for an AI coding tool is then one line: "Build the card
`flightsim/docs/wp/<ID>.md`; follow `flightsim/docs/wp/BUILDER.md`." The rest of this section and
§3 apply to work without a card.

Read [CLAUDE.md](../../CLAUDE.md), [PLAN.md](PLAN.md), [ARCHITECTURE.md](ARCHITECTURE.md),
[CONVENTIONS.md](CONVENTIONS.md), the WP entry, and every spec it names. Current open work is
summarized in [ROADMAP.md](../../ROADMAP.md).

## 2. How a change is made

1. Branch from current `main`. Name it `wp/<WP-ID>-<slug>` or another short feature branch.
2. Implement only the paths that WP owns. Shared contracts in `src/core/` change additively
   (new fields or functions with defaults). Renaming or changing the meaning of a contract
   needs a decision recorded in PLAN.md first.
3. Prove the acceptance criteria with tests, headless traces, and screenshots for anything
   visible.
4. Open a pull request titled `[<WP-ID>] <title>`. One WP per pull request.
5. CI, including the device workflow, must be green. The maintainer reviews and merges.

If two packages need the same file, sequence them or split the file. Do not edit another WP's
paths to "help". Describe the needed change in the pull request.

### Pull request checklist

- [ ] Only owned paths changed. `core/` changes are additive and mirrored in ARCHITECTURE §5.
- [ ] Builds clean with warnings as errors. All tests pass.
- [ ] The device workflow passes (Debian 11, GCC 10, aarch64, `-Werror`). It reports warnings
      the desktop compiler does not.
- [ ] Every acceptance criterion is demonstrated (test names, trace excerpts, screenshots).
- [ ] Unit suffixes and True/Mag naming follow CONVENTIONS §2.
- [ ] No per-frame allocations or blocking IO on the main thread in hot paths.
- [ ] No new dependencies without a decision-log entry. No generated data committed.
- [ ] Docs updated: the WP `Status:` line, and any format or spec section the WP owns.
- [ ] Device-only behaviour is marked "needs device test" with a short checklist.

## 3. Brief for an AI coding tool

```
You are implementing work package <WP-ID>: <title> in Pocket Vectors.

1. Read CLAUDE.md, flightsim/docs/PLAN.md, ARCHITECTURE.md, CONVENTIONS.md, AGENTS.md, and the
   <WP-ID> entry in flightsim/docs/WORK_PACKAGES.md, plus every spec it lists.
2. Work only in the paths <WP-ID> owns. Commit on a feature branch. Push that branch to a fork
   and open a pull request. Do not push to main and do not merge.
3. Meet every acceptance criterion and prove it (tests, traces, screenshots).
4. If a src/core/ contract must change in a non-additive way, or a spec is ambiguous, stop and
   say so in the pull request instead of guessing.
5. In the pull request, list what changed, how it was verified, deviations from the spec, and
   whether it needs a test on the RG40XXH.
```
