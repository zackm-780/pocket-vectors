# X-Plane 11 `.fms` (v1100) and flight plan conventions (WP N3)

Source: X-Plane developer documentation, "Flightplan files (v11 .FMS file format)"
(developer.x-plane.com/article/flightplan-files-v11-fms-file-format/), checked on 2026-10-19.
Code: `Fms.h/.cpp` (text IO), `FlightPlan.h/.cpp` (plan logic). Tests: `tests/nav/plan*`.

## 1. What the format says (and what we rely on)

Plain UTF-8 text, LF or CRLF, spaces or tabs between fields. X-Plane 12 also reads and writes v1100.

```
I                        line 1: "I" (or "A"; equivalent)
1100 Version             line 2: version. 3 is legacy and not supported here (clear error).
CYCLE 1710               mandatory 4-digit AIRAC cycle
ADEP KCUB                origin airport   (or  DEP <point>  for a named nav point, e.g. a VOR)
DEPRWY RW13              optional; omit when no procedure is loaded
SID <name>               optional departure procedure
SIDTRANS <name>          optional, only with SID
ADES KRDU                destination airport (or  DES <point>)
DESRWY RW05L             required if STAR or APP is set
STAR <name>  STARTRANS <name>
APP I05L     APPTRANS <name>   approach name per ARINC 424 section 5.10: I26L ILS, B08R LOC back
                         course, R29 RNAV, V01L VOR, N35 NDB, R35-Y RNAV with multi-indicator,
                         VDM, NDBB, LOCD. First letter = type, then runway, then optional suffix.
NUMENR 9                 number of waypoint lines that follow (the sample count includes both
                         airports, i.e. it equals the number of lines)
1 KCUB ADEP 0.000000 33.970470 -80.995247
3 CTF DRCT 0.000000 34.650497 -80.274918
11 NOMOE V155 0.000000 34.880920 -79.996437
...
1 KRDU ADES 435.000000 35.877640 -78.787476
```

Waypoint line: `type ident via altitude lat lon`
- **type**: 1 airport, 2 NDB, 3 VOR, 11 fix (named waypoint), 28 lat/lon (unnamed).
- **via**: `ADEP` / `ADES` for the origin / destination airport, `DRCT` for a direct leg, or an
  airway name (the leg that ENDS at this waypoint is flown along that airway; every fix on it is
  listed explicitly, so flattening is lossless for flying purposes).
- **altitude**: feet, 0 = none. Airports carry their elevation in the samples.
- **lat lon**: decimal degrees, north and east positive.

Only track-to-fix legs are in the file. Procedure legs (SID/STAR/approach) are NOT expanded; the
file names them and the receiving software loads them.

Not specified by the X-Plane page (so our own convention, see section 3): the ident of type 28
waypoints and what the altitude means for a mid-route fix.

## 2. Tolerance rules implemented by `parseFms`
- A UTF-8 BOM, blank lines, CRLF, tabs and repeated spaces are accepted. Keys and idents are
  case-insensitive on input (stored upper case).
- Unknown header keys are ignored with a log line and a warning (`line N: ignored unknown key`).
- `NUMENR` that disagrees with the number of lines gives a warning; the real count is used.
- Missing `ADEP`/`ADES` give a warning; the first/last waypoint is the origin/destination.
- Errors (the whole file is rejected, all of them are collected, each says `line N:`): first line not
  `I`/`A`, bad or unsupported version, unknown waypoint type, wrong field count, non-numeric or
  out-of-range altitude / latitude / longitude, key without a value, fewer than two waypoints.
- Coordinates in the file win over the nav database when the ident is unknown, when a single DB
  match is more than 25 NM from the file position, and for ambiguous idents when no candidate is
  within 2 NM of the file position. See `buildPlanFromFms`.

## 3. Our conventions
- **Type 28 ident** is written `+DD.DDD_+DDD.DDD` (signed, 3 decimals, underscore), e.g.
  `+36.000_-121.000` as in NAVIGATION 2.1. On input the ident is only used when the line's
  coordinates are 0/0 (decoded from `+lat_+lon`, `+lat,+lon` or `+lat+lon`).
- **Runways**: files say `RW28L`, the nav database and `FlightPlan.depRunway/arrRunway` say `28L`
  (`fmsRunwayToDb` / `dbRunwayToFms`).
- **Approach ident** is the CIFP/ARINC style name used in `APP`, without the `RW`: `I05` = ILS 05,
  `I10R` = ILS 10R, `R28L`, `V10`. It goes into `FlightPlan.approachId`; the kind comes from the
  first letter (`approachKindFromId`). A file with `APP` does not contain the legs: N5/N4 build
  them and call `insertApproach`.
- **Altitude**: a non-zero altitude on a waypoint (not an airport) becomes `AltConstraint{At, alt}`
  and `cruiseAltFt` = the largest such altitude. On export the constraint altitude is written.
- **Airways** are flattened: the via field is read but not kept; export writes `DRCT`.
- **SID/STAR/CYCLE/transitions** are not part of `FlightPlan`; `loadPlanFromFmsText` can return the
  parsed `FmsFile` and `planToFms(plan, db, &file)` copies those lines back.
- **Sample plans** (`assets/plans/`): `KPDX-KOTH.fms` (RW28L, VORs UBG CVO EUG OTH at 8,000 ft since
  the route is southwest-bound, APP I05), `KOTH-KPDX.fms` (RW05, 9,000 ft northeast-bound, APP
  I10R), and `XTST-pattern.fms` (data-free, type 28 waypoints). Coordinates are from the nav pack
  (OurAirports/FAA NASR). MEAs on the route (V495/V481/V121) are 3,500 to 5,000 ft.

## 4. Plan layout produced by this package
See the header comment of `FlightPlan.h`. Origin leg IF (runway start or airport), enroute TF legs,
destination TF leg (runway threshold or airport) which `insertApproach` replaces by the approach
legs and `FlightPlan.approachStartLeg` marks (core, additive field). Direct-to creates
`[PPOS IF][DF target][rest]`.
