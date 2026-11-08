# Taxiways: feasibility note (2026-11-08)

Owner, 2026-11-08: "I'd like to have taxiways... having taxiways makes realism great so you don't
have to just back taxi on runways." This note answers whether the data exists, under which terms,
and what a first version takes. It is a plan, not a build: the cards are in
[wp/W8.md](wp/W8.md) (outline, for a planner to refine).

Today the packs have no taxiway geometry. `runways.csv` comes from OurAirports (runways only), and
`AirportGeometry` draws a generic apron and a short "taxiway stub" beside the longest runway
(`buildAirportExtras`).

## 1. Sources

| Source | What it has | Terms | KPDX | KOTH |
|---|---|---|---|---|
| **OpenStreetMap** `aeroway=taxiway / taxilane / apron / holding_position` | Centerlines (ways) with optional `ref` (name), `width`, `surface`; aprons as polygons; holding positions as nodes or short ways | **ODbL 1.0**: attribution; a distributed derivative database stays ODbL (share-alike) | 145 taxiway ways (32.4 km), 46 with a `ref` (A, A1–A6, B, B1–B8, C, C1–C8, E, E2–E4 …); 85 taxilanes (9.4 km); 15 aprons; 64 holding-position nodes and 36 ways; **no `width` tags** | 19 taxiway ways (5.9 km), **no `ref`, no `width`**; 7 aprons; runways tagged 04/22 and 13/31 (current charts: 05/23) |
| **X-Plane Scenery Gateway** `apt.dat` | Pavement polygons with Bézier edges, painted lines and lights, the taxi routing network with names, signs, hold-short lines | The default X-Plane airport data is published under the **GNU GPL**; the Gateway's own terms for each submission were not confirmed (its site is a script-only app) | 16 submissions, the newest approved 3D (Dec 2022 terminal layout) | 10 submissions; the approved one has the current names (A, B, B1–B3) and 05/23 |
| **FAA** | Airport diagrams (PDF charts, public domain); NASR has runways but no taxiway geometry; the airport GIS / AMDB data is not published openly | Public domain where published | chart only | chart only |

Counts: the OSM API `map` call for each field on 2026-11-08 (KPDX box −122.625, 45.575 to
−122.565, 45.600; KOTH box −124.262, 43.405 to −124.232, 43.428). Gateway: its public API
(`apiv1/airport/<ICAO>`). The Overpass API was not reachable from the planning session; GitHub
Actions can reach it (to be confirmed by the first card).

**Choice: OpenStreetMap** (owner, 2026-11-08, board OD6; PLAN.md D23). It is the only source with
open, machine-readable geometry for the whole region under terms that fit a redistributed pack.
The Gateway is richer per airport (pavement outlines, signs) but its data is GPL, its format is far
more complex (Bézier pavement), and its per-submission terms are unclear. FAA charts are not
geometry.

### What OSM lacks, and how to fill it

- **Widths** are almost never tagged. Default by airport size and taxiway kind: large airports
  23 m (75 ft, ICAO code D/E), medium 15 m (50 ft), small 10.5 m (35 ft); taxilanes 7.5 m. A
  `width` tag wins.
- **Names** are partial (KPDX 46 of 145 ways, KOTH none). Untagged taxiways draw without a
  name; nothing in the first version needs names (they matter for taxi routes, later).
- **Stale runway designators** (KOTH 04/22 in OSM, 05/23 in the nav data): match OSM to the nav
  database by geometry, never by designator. Runway ways from OSM are not used; ours stay.
- **Elevation**: OSM has none. Heights come from the terrain and the runway surface (section 3).

## 2. Licence

ODbL 1.0 (OSM wiki, "Open Data License/Produced Work - Guideline", endorsed by the OSMF board
2014-06-06; OSMF "Licence and Legal FAQ"):

- **The pack's taxiway layer is a derivative database.** Extracted, filtered and re-encoded OSM
  data is still a database. Distributed publicly (the release attaches the pack), it must be
  offered under ODbL. Keep it a separate file set (`data/nav/taxiways.csv`, `aprons.csv`) with its
  own licence note, so the rest of the pack keeps its own terms (a collective database).
- **The game's picture is a produced work.** What is drawn on screen needs only the attribution:
  "© OpenStreetMap contributors" on the About page, in `ATTRIBUTION.txt` and the release notes.
- **No effect on the code.** The game stays Apache-2.0; ODbL covers the data files only.
- Share-alike also means: anyone may take our taxiway files and reuse them under ODbL. That is
  intended.

This needs the PLAN.md decision entry (D23, added with this note) and the release checklist:
`ATTRIBUTION.txt` line, `data/nav/TAXIWAYS-ODbL.txt`, About page line.

## 3. Pipeline

**Extraction** (`tools/nav/osm_taxiways.py`, standard library only like the other nav tools):

1. For each airport in the pack's box with at least one hard runway: one Overpass query on the
   airport's bounding box (runways plus 1.5 km) for `aeroway ~ taxiway|taxilane|apron|
   holding_position`, cached under `tools/nav/cache/osm/` (keyed by airport and date). About 190
   queries for `pdx_oth`; throttle to one a second (Overpass fair use).
2. Keep ways whose nodes are within the box; drop `area=yes` taxiways (draw aprons separately),
   `disused`, `abandoned`, `proposed`, `construction`. Split ways at shared nodes into segments.
3. Width from the tag, else the defaults above; surface from the tag (`asphalt`, `concrete`,
   `grass` …), else the airport's runway surface.
4. Write `data/nav/taxiways.csv` and `data/nav/aprons.csv` (format below) and append the ODbL
   attribution to `ATTRIBUTION.txt`.

**Format** (DATA.md §5, new subsections):

```
# taxiways.csv: one row per OSM way, points as "lat lon" pairs, 6 decimals, space separated
airport_ident,osm_way_id,kind,ref,width_m,surface,points
KPDX,123456789,taxiway,C,23.0,asphalt,"45.588123 -122.601234 45.588201 -122.600987 ..."
# aprons.csv: one row per apron polygon (outer ring, closed)
airport_ident,osm_way_id,surface,points
```

**Size** (measured from the OSM data above): KPDX 245 taxiway, taxilane and apron ways with 2,978
points, about 76 KB of CSV; KOTH 26 ways, 296 points, about 8 KB. The whole `pdx_oth` region
(190 airports, mostly small) is estimated at 0.3–0.5 MB, against a pack of about 400 MB.

**Loading**: `CsvNavDatabase` reads the two files when present (older packs keep working without
them) into an additive `Airport::taxiways` / `Airport::aprons` (`core/NavTypes.h`, additive).

**Rendering** (`AirportGeometry`, `AirportRenderer`):

- Pavement ribbons per segment, joined with round fillets at the nodes (no gaps at bends), in one
  RTC mesh per airport (one draw, like the runways today); aprons as filled polygons
  (ear clipping); both on the composed surface (below).
- Markings: the yellow centerline, and yellow edge lines where a taxiway meets an apron or grass;
  later, hold-short bars at `holding_position` nodes.
- Lights: blue edge lights at night, later (`NightLights`).
- The generic apron and taxiway stub go away at airports that have OSM taxiways.

**Flattening** (the W1b surface, `world/terrain/FlattenSet`):

- Each taxiway segment registers a graded flatten area (`FlattenArea::graded`, a quad around the
  segment plus a margin) with **priority −1**, below the runways (0). The W1b composition then
  makes every runway win where they overlap and blends the taxiway into it without a step or a
  slope over about 1:130; taxiway to taxiway joins the same way.
- Heights: a segment end that lies in a runway area takes the runway surface there; other ends take
  the DEM smoothed over 100 m, clamped to the airport's runway elevations ± 5 m. Aprons are flat at
  the mean of their edge heights (graded aprons are later).
- **Cost**: KPDX would add several hundred areas (one per segment between shared nodes) to its 3
runway areas. `FlattenSet::elevationM` loops
  over every area with a bounding-box test, called per wheel per step and per terrain vertex. It
  needs a uniform grid index (cell 500 m, areas listed per cell) before this lands.

## 4. Device budget

- One or two more draws per airport (taxiway pavement, then markings), like a runway's. KPDX's
  2,978 points make roughly 6,000 pavement triangles plus fillets and markings (an estimate; W8c
  measures it). Taxiways beyond 5 NM are not drawn (they are subpixel).
- Mesh build at airport load, when an airport enters the active set: W8c measures it on the
  desktop and the device check covers the handheld.

## 5. Cards

The first version is too big for one pull request. Proposed split (outline in
[wp/W8.md](wp/W8.md)):

| Card | What | Depends |
|---|---|---|
| W8a | Extraction tool, pack format, DATA.md, ATTRIBUTION, loader into `Airport`, tests on a fixture | OD6 (answered: OSM) |
| W8b | FlattenSet grid index; taxiway and apron flatten areas (priority −1); the composed surface at runway joins; tests | W8a, W1b merged |
| W8c | Taxiway and apron meshes on the surface, centerlines and edge lines; the generic stub removed where OSM data exists; screenshots | W8a, W8b |

Later, not in these cards: hold-short lines and bars, taxiway lights, taxi routes on the map page
(the names, then the network), taxiing ground speed and steering feel, signs.
