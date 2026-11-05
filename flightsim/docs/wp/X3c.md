Status: blocked (T2 numbers)
# X3c: Device tuning to a 30 fps floor

Phase 2b · Effort: high · Depends: T2 · Who: a planner session refines this card from the T2
numbers, then a builder takes it

## Goal

On the default preset, on the device: p95 frame time ≤ 33.3 ms at the KPDX runway, in cruise,
on the KOTH final and on a night final. At most 150 draw calls and 40k terrain triangles in the
Portland view. Then update PLAN §3 with the measured budgets, and close X3 in WORK_PACKAGES.md.

## Levers to pick from (the planner chooses from the numbers)

- Draw calls: batch airport surfaces, lights and PAPI per airport into fewer draws; merge the four
  overlay layers into one draw; skip empty passes.
- Terrain: `pixelsPerCell` per preset, skirt depth, horizon culling margin, the far range in
  fog.
- Fill: the sky and fog shader cost (311 Mpix/s blended with fog on the device), MSAA, render
  scale for Low (draw the 3D view at 480×360 and upscale, HUD at full resolution).
- CPU: anything the `perf:` line shows above 2 ms that isn't render.

## Planner instructions

Replace this section with a concrete card in the same format as the others (Read, Owns, Do,
Acceptance, Device checks), sized for one builder. If the work is larger, split it into X3c1,
X3c2 and so on, and add them to the board table.
