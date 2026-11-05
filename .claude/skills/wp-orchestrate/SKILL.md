---
name: wp-orchestrate
description: Run one dispatch round on the Pocket Vectors work board (flightsim/docs/wp/) - find startable cards, start wp-builder subagents, open their pull requests, and tell the owner what needs them. Use when the user types /wp-orchestrate or asks to start, continue or check the work board.
---

# Work board: one dispatch round

You coordinate; builders build. Your context has to stay small for the whole session, so you
never read source code, diffs, build or CI logs, or spec docs. Your inputs are the board, the
first line of each card, branch names, pull request states and the builders' short reports.

## 1. Look (cheap commands only)

```bash
git fetch -q origin
git show origin/main:flightsim/docs/wp/README.md          # once per session
git grep -m1 '^Status:' origin/main -- 'flightsim/docs/wp/*.md' | sed 's#.*/wp/##'
git branch -r --list 'origin/wp/*'
```

Open pull requests: number, title and check state only (`gh pr list --state open --json
number,title,headRefName,statusCheckRollup`, or the GitHub tools with minimal output).

## 2. Decide

A card can start when all of these hold:
- its status is `ready`, or `blocked (…)` and everything it names is now done or answered;
- every card in its Depends column is `done` on `origin/main`;
- any owner decision it waits on (OD1–OD4 on the board) has an answer from the owner, in this
  session or in the card;
- no `origin/wp/<ID>-*` branch exists yet;
- none of its hot files (board column) is shared with a card whose branch is still open;
- fewer than 3 builders are running.

Fill free builder slots from the earliest phase first. Start a later-phase card only when no
earlier-phase card can start.

Cards with status `owner` are the owner's: list them with their link. Cards with status
`outline (planner refines)` need `/wp-plan <ID>` in a fresh planning session: say so.

## 3. Start builders

For each startable card, call the Agent tool:
- `subagent_type`: `wp-builder`
- `description`: `<ID> builder`
- `prompt`: `Card: flightsim/docs/wp/<ID>.md` plus, if the card waits on one, the owner's decision
  in one line
- `effort`: the card's Effort value (`medium` or `high`)
- `run_in_background`: true
- Don't pass `model` unless the user named one in this session. The user's settings choose the
  builder model.

Then end your turn. Each report wakes you.

## 4. When a report arrives

- PR opened: note it.
- `PR: not opened`: open it. Title `[<ID>] <card title>`, base `main`, head the branch. Body: a
  link to the card, the report lines, and the card's Device checks as a `[ ] needs device test`
  list.
- Blocked, or a question for the owner: put it to the user in at most 3 lines.
- For an `Effort: high` card, start `wp-reviewer` with `Branch <b>, card <path>`. If it says
  changes are needed, start `wp-builder` again with `Card: <path>. Branch: <b> (already exists,
  check it out). Fix these review findings: <the finding lines>`.

## 5. CI

Look at each open `wp/` pull request's check state once per round. When one is red, take only the
failing job's name and its first 20 error lines (for example
`gh run view <id> --log-failed | grep -m 20 -E 'error|FAILED|Error'`) and start `wp-builder` with
`Card: <path>. Branch: <b> (already exists). Fix CI: job <name>: <the lines>`. Never paste a
whole log. Never re-run CI more than once for the same commit.

## 6. Report to the owner (at most 8 lines)

Running · In review (with CI state) · Needs you (device sessions, decisions, approvals) · Next up.
When a phase gate on the board is reached, say what the owner does next (device session, tag).

## Never

Read code or diffs, build, run tests, review a PR yourself, edit cards (that is `/wp-plan`),
merge, push to `main`, run more than 3 builders, or poll CI in a loop. Everything you need is in
the repo, so a fresh session for the next round costs less than a long one.
