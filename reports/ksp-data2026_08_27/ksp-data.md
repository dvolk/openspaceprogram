# KSP body data pipeline

Snapshot of how the KSP body data flows from the wiki into the game, as of
2026-08-27. Written so the pipeline can be picked up later without
re-deriving it.

## What it does

Turns the KSP wiki's per-body orbital data into the two star-system JSON
files the game loads. The headline output is the **epoch starting
positions**: each body now begins at its real KSP 0-UT position (from the
wiki's mean anomaly, argument of periapsis, ascending node, inclination and
eccentricity) instead of the old "all bodies on one axis" layout.

## Layout (after the 2026-08-27 reorganization)

```
utils/                        scripts + data (NOT loaded by the game)
  gen_ksp_csv.py              fetch the 17 wiki body pages  -> ksp_bodies.csv
  gen_systems.py              catalog + ksp_bodies.csv      -> the JSONs
  ksp_bodies.csv              17 x 47 rich per-body data (tracked)
  ksp_system.csv              old 13-col catalog (gitignored local source)
  gen_parts.py, gen_adapter.py, gen_nose_cap.py,
  rescale_obj.py, check_mesh.py     part-mesh tooling (unrelated to body data)

ksp_system.json               Kerbal system -- game default, at the root
old_system.json               Eerbon system (test world), at the root
```

The two system JSONs stay at the repo **root** because the game loads them
from its CWD (default `--system ksp_system.json`). The scripts and CSVs moved
to `utils/` to declutter the root; each resolves its own paths via `__file__`
so they can be run from anywhere.

## The flow

```
KSP wiki
├─ Kerbol_System/Table page ─► ksp_system.csv   (old catalog; the hand-added
│                                  axial tilt lives here)
└─ 17 individual body pages ─► gen_ksp_csv.py   (pandas.read_html; merges
                                              the axial tilt from ksp_system.csv)
                                              │
                                              ▼
                                        ksp_bodies.csv   (47 columns)
                                              │  gen_systems.py reads ONLY the
                                              │  orbital elements:
                                              │  e, i, w, raan, M, period
   K table (hardcoded in gen_systems.py: mass, radius, g, seed,
   has_sea, power_scaler, surface, axial tilt -- transcribed from
   ksp_system.csv, NOT read from it at runtime)
                                              │
                                              ▼
                                        gen_systems.py
                                              │
                                              ▼
                                   ksp_system.json + old_system.json
                                              │
                                              ▼
                               main.cpp load_system() -> frame tree -> game
```

## I/O of each script

| script         | reads                          | writes                          |
|----------------|--------------------------------|---------------------------------|
| gen_ksp_csv.py | 17 wiki pages, ksp_system.csv  | ksp_bodies.csv                  |
| gen_systems.py | the K table, ksp_bodies.csv    | ksp_system.json, old_system.json|

## Two sources of truth (the wrinkle)

- The **catalog** (mass, radius, g, seed, has_sea, power_scaler, surface
  palette, axial tilt) is **hardcoded in the `K` table** in
  `gen_systems.py`. It was transcribed from `ksp_system.csv` at some point
  and is no longer read from that file. To change a body's mass / seed /
  tilt, edit the K table.
- The **orbital elements** (e, i, w, raan, M, period) come from
  `ksp_bodies.csv` (the wiki). `gen_systems.py` reads only these columns.

So `ksp_system.csv` is now mostly vestigial for generation: it is still read
by `gen_ksp_csv.py` to merge the axial-tilt column into `ksp_bodies.csv`,
and it is the origin of the K-table values. It is gitignored (local source
data "baked into the tracked `ksp_system.json`").

## Epoch positions (what the game does with the elements)

- `gen_systems.py` converts mean anomaly `M -> nu` (Newton on Kepler's
  equation; exact for e = 0) and emits per body: `orb_ang_speed` (2·pi/
  period), `arg_peri` = w, `true_anomaly0` = nu, `orb_incl` = i,
  `lon_asc_node` = raan, `ecc` = e. Circular bodies still emit the angles so
  they start at w + M rather than +X.
- `main.cpp load_system` builds the epoch state with
  `railStateFromElements` (orbit.h) and orients the orbital plane with
  `orient = R_Y(-raan) * R_X(incl)` — which reduces to the old X-tilt when
  raan = 0, so `old_system.json` / Eerbon is untouched.
- In-plane longitude lands at raan + w + nu (exactly, for i = 0), which is
  why the bodies are now spread out (deg, +X toward +Z): Kerbin 180, Eve
  195, Moho 265, Duna 315, Jool 58, Dres 190, Eeloo 130. Each moon clusters
  on its parent. Eden (game-added, not on the wiki) sits 60° ahead of
  Kerbin at 120° — its L4 slot, re-anchored to Kerbin's new position.

## How to run

```
python3 utils/gen_ksp_csv.py     # refresh ksp_bodies.csv (needs network + pandas)
python3 utils/gen_systems.py     # regenerate ksp_system.json + old_system.json
```

There is no strong reason to rerun these often — the data is static and the
JSONs are committed. Rerun `gen_ksp_csv.py` only if the wiki changes or new
columns are wanted; rerun `gen_systems.py` after editing the K table or the
element-handling logic.

## Caveats / open questions

- **Moon element reference frame:** the moon orbital elements (i, raan) are
  applied directly in the parent's frame. KSP's exact reference frame for
  moons may differ slightly — fine at this stage, but worth checking if a
  moon's orbit looks "off".
- **The K-table / CSV split** is the main maintenance trap: a value can live
  in the hardcoded table rather than a CSV. If that is annoying, a cleanup
  would be to make `gen_systems.py` read the catalog from a CSV too.
- **Unused columns:** the extra `ksp_bodies.csv` columns (atmosphere,
  temperature, science, escape velocity, ...) are stored but not yet consumed
  by the game — they are there for when those systems are built.
