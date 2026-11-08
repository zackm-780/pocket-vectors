# OSM fixtures for `test_osm_taxiways.py`

- `koth_aeroway.osm`: the aeroway taxiway, taxilane and apron ways of KOTH (Southwest Oregon
  Regional) with their nodes, cut by `make_osm_fixture.py` from one OpenStreetMap API `map` call
  on 2026-11-08 (`bbox=-124.270,43.400,-124.225,43.432`). **Data © OpenStreetMap contributors,
  available under the Open Database License 1.0**
  (<https://www.openstreetmap.org/copyright>, <https://opendatacommons.org/licenses/odbl/1-0/>).
  Keep it small. The OSM API usage policy allows no bulk reads; the full extract runs through the
  Overpass API in the `datapack` workflow.
- `edge_cases.osm`: synthetic (no OSM data), in Overpass element order (ways before nodes).
- `make_osm_fixture.py`: the cutter.
