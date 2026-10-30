Status: ready
# P1b: Probe prompts and tool detection in the launch log

Phase 1 · Effort: medium · Depends: none · Branch: `wp/P1b-probe-tools`

## Why

- The probe's A/B/X/Y/L1 prompts timed out unanswered on the first device run (DEVICE.md §5), so
  X and Y are still unconfirmed. This was D1's optional deliverable 5 and was never done.
- X5 (live METAR) needs to know whether the stock OS has `python3` with TLS, `curl` or `wget`, and
  a network route. The launch log doesn't record any of that today.

## Read

- `tools/probe/README.md`; in the probe source, `grep -n -i 'prompt\|press' tools/probe/*.cpp` and
  read a window around the prompt loop
- `port/FlightSim.sh` lines 1–110 (the launch-log block ends at the `exec >> "$LAUNCHLOG"` line)
- `port/test_launch_scripts.sh` (how the device workflow exercises the script)

## Owns

`tools/probe/`, `port/FlightSim.sh` (launch-log block only), `port/test_launch_scripts.sh`.

## Do

1. Probe: each input prompt waits up to 8 s for its first press and records "missed" only after
   that. It then waits 1 s for the release before the next prompt. Keep the 1.5 s hands-off start.
2. Launch log, inside the existing `{ … } >> "$LAUNCHLOG"` block:
   - `command -v python3 curl wget timeout` and the versions of the ones found
   - `python3 -c 'import ssl; print(ssl.OPENSSL_VERSION)'` when `python3` exists
   - a default route (`ip route show default` or `/proc/net/route`) and, behind `timeout 3`, one
     DNS lookup of `aviationweather.gov` (`getent hosts`). Log the result and never fail on it.
3. Keep the script LF-only, runnable under a plain `sh` until its bash re-exec, and quiet on
   screen.
4. Extend `test_launch_scripts.sh` so a PATH without `python3`, `curl` or `timeout` still launches.

## Acceptance

- [ ] `test_launch_scripts.sh` passes, including the reduced-PATH case
- [ ] The probe runs on desktop (`xvfb-run`) and still writes `probe_report.txt`
- [ ] The device workflow is green (CI)

## Device checks

- [ ] Launch log shows `python3`, `curl` and `wget` lines, the SSL version and the DNS result
- [ ] Probe: every face-button prompt records the press
