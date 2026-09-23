# Allocation profiling with heaptrack

Snapshot of the allocation-profiling workflow as of 2026-09-23, and the
results of the first pass (which found and fixed the `projectedArea`
per-substep churn).

## What it measures

heaptrack intercepts every `malloc`/`free`/`new`/`delete` and records the
backtrace. It answers:

- **how many** allocations happen (total, per second)
- **from where** (exact call stacks, attributed to our functions)
- **temporary allocations** — freed almost immediately after allocating;
  pure churn in a hot loop, the main thing to hunt in a game
- **peak heap** per call site and **what is still live at exit** ("leaks")

Overhead is small (a few %), unlike valgrind's 20-50x — runs play at
near-normal speed.

## Setup (one-time)

```bash
sudo apt-get install -y heaptrack    # 1.5.0 installed here
```

No KDE/GUI needed: analysis is done headless with `heaptrack_print`.

## Running a capture

Wrap the normal game launch in `xvfb-run -a heaptrack -o <prefix>` (the
same Xvfb path the e2e battery uses; `-o` puts the capture in tmp/ per
the scratch-file rule):

```bash
mkdir -p tmp/heaptrack
xvfb-run -a heaptrack -o tmp/heaptrack/osp ./osp \
  --timeout 30 --scenario rot-orbit --ship res/ships/racer.json \
  --time-accel 10 --data-dir tmp/heaptrack/data \
  --sim-press 500,500,R --sim-press 1500,3000,T
```

Any scenario/sim-press flags work — pick whatever exercises the code
under investigation. Prints a summary at exit:

```
heaptrack stats:
        allocations:            2150646
        leaked allocations:     31054
        temporary allocations:  1456240
```

Capture files are `tmp/heaptrack/osp.<pid>.zst` (zstd-compressed).

## Analyzing (headless)

```bash
heaptrack_print tmp/heaptrack/osp.zst > tmp/heaptrack/report.txt
```

The report sections, in order of usefulness:

| Section | What it tells you |
|---|---|
| `MOST CALLS TO ALLOCATION FUNCTIONS` | ranked by allocation COUNT — the churn ranking. Each entry: total count + per-caller-stack breakdown |
| `MOST TEMPORARY ALLOCATIONS` | same, but only alloc/free-immediately pairs — the hot-loop offenders |
| `PEAK MEMORY CONSUMERS` | ranked by BYTES live — memory footprint, not churn |
| summary (last lines) | totals: allocs/s, temporary %, peak heap, leaked bytes |

Tips:

- The binary should have symbols (the release build does). For line
  numbers, build once with `make CXX_OPT="-O2 -g"`.
- Our code appears by function name (`Vehicle::applyAeroForce`,
  `Mesh::Draw`); driver code often appears as bare `0x...` addresses
  with `in /usr/lib/.../libgallium...so`.
- **llvmpipe caveat:** under Xvfb the GL stack is software rendering
  (llvmpipe), and it allocates heavily per draw call. Those gallium
  entries are NOT representative of real GPU hardware — read them as
  "proportional to our draw-call count", and chase the entries that
  land in *our* symbols first.
- To re-run the exact same flight, reuse the same CLI flags; counts are
  deterministic for a fixed scenario, so before/after reports are
  directly comparable.

## First-pass results (2026-09-23)

30 s orbit-burn run (`rot-orbit`, racer, time-accel 10, throttle+burn
via sim-press), release build:

- 2.15M allocations, ~69k/s; 68% temporary; peak heap 293M.
- **#1 (1.80M calls): libgallium/llvmpipe**, entered per frame from
  `Mesh::Draw` via `TerrainBody::Draw/DrawAtmosphere/DrawOcean/DrawClouds`
  and `GeoPatch::Draw` — software-GL artifact, see caveat above.
- **#2 (181,320 calls): `projectedArea()` in src/drag.h** — our own hot
  path. Called every physics substep from `Vehicle::applyAeroForce`
  (once per part + once for the whole ship), and each call allocated two
  fresh `std::vector`s (`p` projected points, `h` hull, the latter grown
  by unreserved push_back). Peak consumption only 21K — pure churn.
- Reported "leaked" at exit: 172.76M — almost certainly SDL/GL driver
  state we never tear down at shutdown; not investigated further.

### The fix (committed separately)

`p`/`h` in `projectedArea` and `shipVerts` in `applyAeroForce` became
`static thread_local` scratch buffers, `clear()`/`resize()`d per call
(capacity is kept, so steady state allocates nothing). Verification:

- `make test` + e2e 03 (thrust-ascent), 04 (orbit-burn): pass.
- Re-run capture: `projectedArea` and `applyAeroForce` no longer appear
  anywhere in the report; total allocations 2.15M -> 2.05M (the remainder
  is the llvmpipe/driver churn).

Reports at the time of writing: `tmp/heaptrack/report.txt` (before),
`tmp/heaptrack/report-fixed.txt` (after).

## Related tools

- `valgrind --tool=callgrind` + `callgrind_annotate --tree=caller`:
  exact deterministic call counts incl. callers, 20-50x slower; good
  cross-check when heaptrack attribution is unclear.
- `valgrind --tool=massif`: heap-over-time snapshots; complements
  heaptrack's peak/consumer view.
