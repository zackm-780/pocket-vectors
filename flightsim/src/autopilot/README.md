# autopilot/ (WP N6)

`Autopilot` = autopilot + flight director + autothrottle behind `update(pilot, aircraft, nav, dt)`
(called per 120 Hz sim step), `handleActions(ActionState, aircraft, nav, dt)` (per frame, wired in
`App.cpp`) and `syncPilot(pilot)` (called by `Simulation` after every step so the persistent pilot
controls follow the A/T levers and the AP-managed elevator trim). `ApConfig` holds the gains.

## Modes and arming

| Layer | Mode | Notes |
|---|---|---|
| Lateral | ROL | holds bank (<= 6 deg: wings level); engage default |
| | HDG | bug (magnetic, `NavState::magVarDeg`) via turn-rate law, bank limit 20 (light) / 25 (jet) |
| | NAV | arms when selected; captures when closing and inside `R(1-cos(rel)) + 0.1 NM` (or within 0.1 NM); intercept = `atan(xtk / (gs * navTau))`, <= 45 deg, track based so wind needs no extra gain |
| | LOC | APR arms LOC + GS; LOC captures with the NAV rule plus `|cdi| <= 1` and `source == Loc`; linear XTK law (distance independent), <= 30 deg |
| Vertical | PIT | holds the pitch at engage |
| | VS | `vsBugFpm`, PI on vertical speed -> pitch (gains scale with 1/TAS) |
| | ALT* / ALT | capture starts at `max(100, |VS|/6)` ft toward the bug (from PIT, VS, FLC or GA), ALT at <= 25 ft. The ALT button (`selectAltButton`) holds the present altitude; in ALT / ALT* with the bug more than 100 ft away it starts VS toward the bug at `vs_default_fpm` (ALT armed) |
| | GS | captures when LOC is active and `|gsDeflection| <= 0.5` (1 dot): from below, or within 1 dot; target VS = path VS + height error / 12 s |
| | FLC (N7) | pitch on speed to the speed bug (synced to the IAS when unset), toward the altitude bug only (no climb with the bug below, no descent with it above); within 100 ft of the bug: ALT on the bug; a second press: PIT |
| | GA (N7) | TO/GA, airborne only: pitch on speed to the IAS at the press (at least Vref with flaps out, `min_climb_ias_kt` clean), attitude <= `ga_pitch_deg`, no descent while within 5 kt of that speed; ends at ALT* or another vertical mode |
| Speed | SPD | PI + acceleration term on IAS -> all throttles, slew limited |
| | RETARD | idle below 30 ft AGL, descending, landing configuration; A/T off at touchdown |
| | THR (N7) | fixed throttle `thrustTargetNorm`: TOGA (`ga_throttle`) in GA, `climb_throttle` or idle in FLC; SPD takes over at ALT* without a jump |

**Pitch on speed (FLC, GA).** The VS loop flies `VS + 5.311 * TAS * (TAS / IAS) * (a + (IAS - target)
/ spd_tau_s)`, with `a` the IAS rate low-passed over `spd_accel_tau_s` (the AP keeps its own; the
A/T's only runs with the A/T on). 5.311 fpm per kt per kt/s is the energy balance `dh/dt + (V / g)
dV/dt` in fpm and kt; TAS / IAS turns the IAS rate into a TAS rate. The change is clamped to
`spd_max_delta_vs_fpm` and the result to +/-8,000 fpm; the pitch limits and the AoA cut still apply.

**TO/GA** also sets HDG on the present heading, arms NAV when the plan has an active leg (the
navigator sequences into the missed approach once the MAP is passed airborne, and NAV flies it),
clears the LOC / GS arms, engages the A/T on aircraft that have one, turns the FD on when neither
AP nor FD was, and moves the altitude bug 1,000 ft up (rounded to 100 ft) unless it is already
500 ft or more above. The pitch command slews at `ga_pitch_rate_degps` in GA.

FD (`fdOn`) runs the same outer loops with the AP off: `fdRollCmdDeg` is the commanded bank and
`fdPitchCmdDeg` the commanded pitch attitude (deg, + = right wing down / nose up); `fdValid` is set
when a lateral and a vertical mode are active. Modes are shared between FD and AP. A mode button
(HDG, NAV, APR, ALT, VS, FLC) pressed with the AP and the FD both off turns the FD on first (in
`handleActions`), like TO/GA; the `select*` calls themselves still need the AP or the FD.

## Disconnects (each sets `disconnectWarning`, cleared by the next AP toggle press or after 5 s)
Stick > 0.5 for 0.5 s, AP toggle press, stall warning, |bank| > 45 deg, |pitch| > 25 deg, and
below `minimumsFt - 50` while coupled on an approach (`phase == Approach` and LOC/GS/onFinal), but
not in GA: TO/GA below minimums keeps the AP flying the go-around.
Weight on wheels disconnects silently. The A/T goes off when the pilot moves the throttle lever
and at touchdown.

## Safety nets
Underspeed (no A/T): VS target reduced by 100 fpm per knot below `min_climb_ias_kt` (default
1.25 * vs1). Angle-of-attack: pitch demand cut above `alpha_limit_deg` (default stall warning - 2.5).

## Profile `[autopilot]` keys (all optional; defaults by category in `ApConfig.cpp`)
`bank_limit_deg`, `heading_tau_s`, `nav_tau_s`, `max_intercept_deg`, `loc_max_intercept_deg`,
`rol_wings_level_deg`, `bank_rate_limit_degps`, `roll_kp/kd/ki`, `rudder_beta_gain`,
`rudder_rate_gain`, `pitch_max_deg`, `pitch_min_deg`, `pitch_rate_limit_degps`, `pitch_kp/kd/ki`,
`trim_gain_per_s`, `trim_max_rate_per_s`, `vs_kp_rel`, `vs_ki_rel`, `vs_max_climb_fpm`,
`vs_max_descent_fpm`, `vs_default_fpm`, `alt_capture_gain_fpm_per_ft`, `alt_hold_gain_fpm_per_ft`,
`alt_capture_min_ft`, `alt_hold_within_ft`, `gs_tau_s`, `gs_max_corr_fpm`, `min_climb_ias_kt`,
`alpha_limit_deg`, `at_available` (1 = offer the A/T on aircraft without one), `at_kp`, `at_ki`,
`at_k_accel`, `at_slew_per_s`, `retard_agl_ft`, `retard_slew_per_s`, `stick_disconnect`,
`stick_disconnect_s`, `bank_disconnect_deg`, `pitch_disconnect_deg`, `spd_bug_min_kt`,
`spd_bug_max_kt`; N7: `spd_tau_s` (5), `spd_accel_tau_s` (1), `spd_max_delta_vs_fpm` (2,500 piston,
3,000 turboprop, 4,000 jet), `ga_pitch_deg` (10 / 12 / 15), `ga_pitch_rate_degps` (3),
`climb_throttle` (1.0 piston, 0.95 turboprop and jet), `ga_throttle` (1.0).

## Scenario `[autopilot]`
`engage = ap fd at hdg nav apr alt vs rol pit flc toga` (a mode token engages the AP; `toga` only
turns the FD on), `hdg_bug_mag_deg`,
`alt_bug_ft`, `vs_bug_fpm`, `spd_bug_kt`. Bugs default to the present heading / altitude.
`alt` flies toward the altitude bug (VS with capture) or holds it when within 100 ft.
