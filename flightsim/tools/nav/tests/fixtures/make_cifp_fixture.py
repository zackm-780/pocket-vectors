#!/usr/bin/env python3
"""Cut a small ARINC 424 fixture out of the real FAA CIFP file (US Government work, public domain).

    python3 make_cifp_fixture.py /path/to/FAACIFP18 cifp_mini.txt

Keeps the cycle header, the approach records of the selected approaches and every waypoint,
navaid, runway, airport and localizer record they reference. The result is committed as
``cifp_mini.txt`` (< 200 KB) and used by test_cifp.py; the pipeline itself never needs this file.
"""
from __future__ import annotations

import sys

# (airport, approach ident or None for all approaches). Chosen so that every path terminator that
# the converter handles shows up at least once.
SELECTION = [
    ("KOTH", None),  # RNAV (RNP) with RF, ILS/LOC with AF arcs, PI, VI, HF, HM, CA, VOR-B (circling)
    ("KPDX", "I10R"), ("KPDX", "I28L"),
    ("KBFI", "I14R"),  # CD
    ("KDEN", "I16L"),  # VD
    ("KRFD", "I07"),  # VM
    ("KDDC", "I14"),  # CI
    ("KCOE", "I06"),  # FA
    ("KLPR", "I07"),  # VA
    ("KLAX", "I25L"),  # VR
    ("PANC", "I07L"),  # FC (Alaska is area CAN)
]


def main(src: str, dst: str) -> int:
    lines = [l.rstrip("\r\n") for l in open(src, encoding="latin-1")]
    want = {(a, i) for a, i in SELECTION}
    keep: list[str] = []
    refs: set[tuple] = set()  # (sec, sub, ident, region, airport)
    apts = set()
    for l in lines:
        if l.startswith("HDR"):
            keep.append(l)
            continue
        if len(l) < 40 or l[4] != "P" or l[12] != "F":
            continue
        apt, ident = l[6:10].strip(), l[13:19].strip()
        if (apt, ident) in want or (apt, None) in want:
            keep.append(l)
            apts.add(apt)
            if l[38] in "01":
                if l[29:34].strip():
                    refs.add((l[36], l[37].strip(), l[29:34].strip(), l[34:36], apt))
                if l[50:54].strip():
                    refs.add((l[78].strip(), l[79].strip(), l[50:54].strip(), l[54:56], apt))
                if l[106:111].strip():
                    refs.add((l[114].strip(), l[115].strip(), l[106:111].strip(), l[112:114], apt))
    extra: list[str] = []
    for l in lines:
        if len(l) < 40 or l[0] not in "ST":
            continue
        sec, apt = l[4], l[6:10].strip()
        if sec == "P" and l[12] == "A" and apt in apts:
            extra.append(l)
        elif sec == "P" and l[12] == "G" and apt in apts:
            extra.append(l)
        elif sec == "P" and l[12] == "I" and apt in apts and l[21] in "01":
            extra.append(l)
        elif sec == "P" and l[12] == "C" and l[21] in "01":
            if any(r[0] == "P" and r[1] == "C" and r[2] == l[13:18].strip() and r[4] == apt for r in refs):
                extra.append(l)
        elif sec == "E" and l[5] == "A" and l[21] in "01":
            if any(r[0] == "E" and r[2] == l[13:18].strip() for r in refs):
                extra.append(l)
        elif sec == "D" and l[5] == " " and l[21] in "01":
            if any(r[0] == "D" and r[1] == "" and r[2] == l[13:17].strip() for r in refs):
                extra.append(l)
        elif sec == "D" and l[5] == "B" and l[21] in "01":
            if any(r[0] == "D" and r[1] == "B" and r[2] == l[13:17].strip() for r in refs):
                extra.append(l)
    # waypoint records first, then approaches (the order in the real file is unimportant)
    hdr = [l for l in keep if l.startswith("HDR")]
    appr = [l for l in keep if not l.startswith("HDR")]
    out = hdr + extra + appr
    with open(dst, "w", encoding="latin-1", newline="\n") as f:
        f.write("\n".join(out) + "\n")
    print(f"{len(out)} lines, {sum(len(x) + 1 for x in out) / 1024:.0f} KiB -> {dst}")
    return 0


if __name__ == "__main__":
    if len(sys.argv) != 3:
        print(__doc__)
        sys.exit(2)
    sys.exit(main(sys.argv[1], sys.argv[2]))
