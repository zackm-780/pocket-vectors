#!/usr/bin/env python3
"""Cuts an OSM XML file down to its aeroway=taxiway/taxilane/apron ways and their nodes.

Used once to make koth_aeroway.osm from one OSM API `map` call (2026-11-08):
    curl -o koth.osm 'https://api.openstreetmap.org/api/0.6/map?bbox=-124.270,43.400,-124.225,43.432'
    python3 make_osm_fixture.py koth.osm koth_aeroway.osm
The output keeps the OSM API element order (nodes, then ways). Data (c) OpenStreetMap
contributors, ODbL 1.0.
"""
import sys
import xml.etree.ElementTree as ET

KEEP = {"taxiway", "taxilane", "apron"}


def main(src: str, dst: str) -> None:
    root = ET.parse(src).getroot()
    ways = [w for w in root.findall("way")
            if any(t.get("k") == "aeroway" and t.get("v") in KEEP for t in w.findall("tag"))]
    refs = {n.get("ref") for w in ways for n in w.findall("nd")}
    out = ['<?xml version="1.0" encoding="UTF-8"?>',
           '<osm version="0.6" generator="make_osm_fixture.py">',
           "<!-- Data (c) OpenStreetMap contributors, ODbL 1.0, "
           "https://www.openstreetmap.org/copyright -->"]
    for n in root.findall("node"):
        if n.get("id") in refs:
            out.append(f' <node id="{n.get("id")}" lat="{n.get("lat")}" lon="{n.get("lon")}"/>')
    for w in ways:
        out.append(f' <way id="{w.get("id")}">')
        out += [f'  <nd ref="{n.get("ref")}"/>' for n in w.findall("nd")]
        for t in w.findall("tag"):
            v = t.get("v").replace("&", "&amp;").replace('"', "&quot;").replace("<", "&lt;")
            out.append(f'  <tag k="{t.get("k")}" v="{v}"/>')
        out.append(" </way>")
    out.append("</osm>")
    with open(dst, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
