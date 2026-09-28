# Allocation profiling pass, 2026-09-28

Follow-up to `reports/alloc-profiling2026_09_23/heaptrack.md`. Full e2e
battery was green (97/97) before the pass, so this is a pure profiling
session: one 30 s capture, analysis, two easy fixes, re-capture,
verification.

## The headline: the profile shape changed with the renderer

The 09-23 baseline (Xvfb + llvmpipe) read 2.15M allocations, 68%
temporary, with libgallium software-GL churn dominating. This box has a
DRM render node, so the game ran on the SDL offscreen/EGL path (the same
path the e2e battery uses here) and the software-GL churn is gone:

| | 09-23 baseline (llvmpipe) | 09-28 before (EGL) | 09-28 after (EGL) |
|---|---|---|---|
| allocations (30 s) | 2,150,646 | 158,466 | **136,519** |
| allocs/s | ~69k | 5,171 | **4,456** |
| temporary % | 68% | 4.3% | 4.3% |
| peak heap | 293M | 74.5M | 73.7M |
| "leaked" at exit | 172.8M | 18.1M | 18.0M |

Two consequences for reading heaptrack on this project:

1. **Profiles are renderer-relative.** The 09-23 numbers cannot be
   compared with EGL ones; the "llvmpipe caveat" in the 09-23 report now
   applies in reverse — llvmpipe inflates driver-side allocation counts
   ~13x. Chase our symbols either way, but expect the driver entries to
   dwarf everything under software GL.
2. **The "leak" at exit is driver/asset state, not a bug.** 18.0M =
   SDL texture surfaces (2x 16.78M terrain textures), the 9.16M streamed
   music buffer, and Bullet's 7.27M collision config — all legitimate
   lifetime allocations.

## What the profile showed

Per-frame churn in our code: **nothing**. The 09-23 scratch-buffer fixes
(`projectedArea`, `applyAeroForce`, `collectVehiclesInto`) still hold —
no `src/` frame-loop symbol appears anywhere in the top entries, and all
of the top *temporary* entries are GPU-driver (`libgallium`, the
EGL/radeonsi path) or one-shot third-party boot work (stbtt font
raster, assimp path building, SDL hotplug thread, shader string
handling).

The remaining our-code entries were all boot-time terrain construction:

| entry | before | after |
|---|---|---|
| `Mesh::FromData` (4 unreserved push_back vectors per grid) | 19,088 | 1,744 |
| `buildGridGeom` (unreserved ~15k index fill, worker thread) | 5,312 | gone |
| `btAllocDefault` | 6,318 | 6,318 (Bullet-internal) |
| `SDL_malloc_REAL` / `SDL_strdup_REAL` | 4.2k / 2.0k | unchanged (SDL-internal) |

## The fixes (commit 8a61565)

Both were the same pattern — a known count pushed into an unreserved
`std::vector`, paying ~log2(n) reallocations per terrain grid:

- `src/mesh.cpp` `Mesh::FromData`: `reserve()` the 4 model vectors
  before the fill loops (the caller's arrays are already contiguous;
  `InitMesh` uploads straight from them).
- `src/terragen.h` `buildGridGeom`: `reserve()` `geom.indices` to the
  exact quad count `(edge-1)^2` (skirted) or `(size-1)^2` (plain) times
  6, next to the existing `geom.verts.resize`.

Verification: `make test` all pass; e2e 01/95/96 pass; re-capture
158,466 -> 136,519 (-14%), both entries as above. A subagent quality
pass found no blocker (reserve is semantically inert: same bad_alloc
semantics, smaller peak allocation, zero/null inputs unchanged) and
flagged one pre-existing latent issue, filed as #43 (`InitMesh`
dereferences `model.indices[0]` etc. for empty vectors — unreachable by
any current caller).

## What is left (not "easy")

- The 1.7k residual `Mesh::FromData` allocations are the per-mesh GL
  upload buffers (`new double[]`, `new int[]`, the VAB array) —
  legitimate one-per-mesh lifetime allocations.
- `btAllocDefault` (6.3k) is Bullet's internal allocator; swapping it
  out is an invasive Bullet-config change for no per-frame gain.
- Everything temporary is now driver/third-party. The next meaningful
  allocation win would have to come from reducing *work* (fewer patches
  built, see #28's unbounded-LOD-growth), not from buffer reuse.

## Reproducing

```bash
# capture (EGL path; no Xvfb needed on a box with /dev/dri/renderD*)
SDL_VIDEODRIVER=offscreen DISPLAY= heaptrack -o tmp/heaptrack/osp ./osp \
  --timeout 30 --startship racer,res/ships/racer.json,Kerbin,rot-orbit \
  --time-accel 10 --data-dir tmp/heaptrack/data \
  --sim-press 500,500,R --sim-press 1500,3000,T

heaptrack_print tmp/heaptrack/osp.zst > tmp/heaptrack/report.txt
```

Artifacts from this pass: `tmp/heaptrack/osp.zst` (after),
`report-2026_09_28.txt` (before), `report-2026_09_28-fixed.txt`,
`report-2026_09_28-final.txt`, `run*_stdout.log`.
