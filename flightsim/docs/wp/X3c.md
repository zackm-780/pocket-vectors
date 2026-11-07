Status: outline (planner refines) · next, as 1.2.1 (owner, 2026-11-06): HUD CPU, the GPU clock in the log, MED toward 60 fps. Device numbers: DEVICE.md §3, 2026-11-05
# X3c: Device tuning to a 30 fps floor

Phase 2b · Effort: high · Depends: T2 · Who: a planner session refines this card from the T2
numbers, then a builder takes it

## Device numbers and owner decisions (2026-11-05 / 2026-11-06)

The first timing run (DEVICE.md §3, PLAN §3) on the Mali-G31 at MED: a locked 30 fps in the
cockpit, GPU-bound (swap 22 ms), CPU 11.6 ms of which the **HUD is 8.0 ms**, 44k terrain triangles
in 70 draws. Chase: 30 fps, CPU 4.5 ms (HUD 0.7), dips to 20 fps while orbiting (GPU). Inside OVC
the same 47k triangles flip between 30 and 60 fps (avg 55): the GPU is at the 16.7 ms edge, or
GPU DVFS changes its clock. LOW: a locked 60 fps (p95 17.0 ms), CPU 9.4 ms (HUD 6.6), 37k
triangles. HIGH: 30 fps.

The owner keeps **MED as the default** (LOW looks rough, MED decent) and **keeps HIGH** (other
devices). This card ships as **1.2.1** and covers:

1. **HUD CPU:** 8.0 ms in the cockpit against PLAN §3's 2 ms. Find where it goes (glyph
   batching, per-frame string formatting, tape loops) and cut it.
2. **The GPU clock in the log:** read the Mali devfreq current frequency (look under
   `/sys/class/devfreq/`) with each session table and SHOW FPS line, so the 30 ↔ 60 flip
   inside OVC can be told apart from DVFS.
3. **MED toward 60 fps:** from the levers below, within MED's look.

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
