# Content layout: where games live

**Question (Denis, 2026-08-23):** should all the JSONs move into `./games/<game>`
so the player loads individual games (elegant), *or* stay in `./res` with `games/`
holding only "game" JSONs that list which res files belong to the game (composable but ugly)?

**Short answer:** go with the per-game directories (your "elegant" option) — but with one
refinement that also buys you the composability, so the two stop being a trade-off. A game is
a folder; the files it doesn't provide fall back to a shared content library; rendering assets
stay shared in `res/`. No manifest, no pointer table.

---

## 1. What the code actually does today (ground truth)

Content splits cleanly into two classes that already live apart:

| Class | Files | Referenced how |
|---|---|---|
| **Game data** (the "game") | `ksp_system.json`, `system.json` (repo root), `res/parts.json`, `res/ships/basic.json`, `res/fleet.json` | CLI flags `--system/--parts/--ship/--fleet` |
| **Rendering assets** | `res/*.obj`, `res/*.png`, `res/*Shader.*`, `res/*.ttf`, `res/skybox.png`, indicator icons | hardcoded `./res/...` in `main.cpp` |

The reference chain inside the data is **already name-based at every hop**:

```
fleet.json     "ship": "res/ships/basic.json"   <- the ONE path-based link
ships/basic    "part": "engine"                 (name)
parts.json     "mesh": "engine.obj"             (bare name)
                       |
                       v   resolved at exactly one site:
             main.cpp:1375-1376   "./res/" + pd.mesh / pd.texture
```

Two consequences fall straight out of this:

1. **The data/asset split is already the de-facto design.** Data-JSONs reference assets by
   bare name; the code knows to look in `res/`. Moving data-JSONs elsewhere does *not* touch
   asset resolution — there is a single `./res/` prefix site, and it stays put.
2. **The only path in the data is `fleet → ship`.** Everything else is names. So "a game is a
   folder" is a small, local change, not a rewrite of the loader.

The two existing games confirm the shape:

| Game | System | Parts | Ships | Fleet |
|---|---|---|---|---|
| KSP (17 bodies, home Kerbin) | `ksp_system.json` | `res/parts.json` (shared) | `res/ships/basic.json` (shared) | `res/fleet.json` (shared) |
| Eerbon (3 bodies, home Eerbon) | `system.json` | same | same | same |

**They differ only in the system file.** Parts/ships/fleet are identical. So right now "a game"
is mostly "a system + the shared tech," which is the strongest argument against a manifest: the
one thing that actually varies per game is already a single self-contained file.

---

## 2. Recommendation: a game is a directory; shared files fall back

```
res/                      # rendering assets ONLY — shared, never duplicated
  engine.obj  engine.png  capsule.obj  ...  partsShader.*  *.ttf  skybox.png  ...

games/
  ksp/
    system.json
  eerbon/
    system.json
  minimal/                # a fully custom game, for when one exists
    system.json
    parts.json
    ships/basic.json
    ships/heavy.json
    fleet.json
  _shared/                # content library for games that don't override it
    parts.json
    ships/basic.json
    fleet.json
```

**Resolution precedence (highest wins), per file:**

```
1. explicit CLI flag        --system / --parts / --ship / --fleet     (dev escape hatch)
2. games/<name>/<file>      the game's own content
3. games/_shared/<file>     the shared content library
4. (legacy) res/<file>      only if we keep the shared data there for now
```

So `./osp --game ksp` resolves to `games/ksp/system.json` + `games/_shared/{parts.json,ships/,fleet.json}` —
exactly today's KSP setup, except the system now lives with the game. A custom game drops all four
into its own dir and they win. `--system /path/to.json` still overrides everything, so the current
power-user / testing workflow is unchanged.

This is your "elegant" option (a game is a folder you can copy) **plus** the composability you wanted
from the manifest option (shared parts/ships/fleet, no duplication) — achieved by *directory
precedence* instead of a pointer file. The two are no longer a trade-off.

### Why this beats the manifest (your Option B)

- The manifest's only job is to list which files a game uses. But those files split into
  **assets** (already shared by living in `res/` — no manifest needed) and **game data**
  (the game's identity — you don't really want two games to be the same game). So the manifest
  mostly points at things that are either already shared or shouldn't be.
- It adds a 4th file type on top of system/parts/ships/fleet, a new indirection layer on every
  load, and makes a game **non-portable** (copy the folder away and it dangles pointers back into
  `res/`). A per-game directory is portable by construction.
- "Composable" is the selling point, but the composition being saved is ~40 lines of JSON
  (`parts.json`), while the cost is a pointer table. Directory fallback gets the same sharing for
  free.

---

## 3. The one genuine decision: per-game vs shared parts/ships

This is the only real fork, and it depends on a question only you can answer:

- **Will your games have different parts catalogs / ships?** If yes → put them in each game's dir
  (self-contained). If two games happen to share a catalog, that's a copy of ~40 lines; fine.
- **Are all your games the same tech with different maps?** (Today: yes.) → keep parts/ships/fleet
  in `_shared/` and let each game carry just its `system.json`. This is the cheapest model and
  matches reality right now.

My default recommendation: **start with games carrying only `system.json`, and parts/ships/fleet in
`_shared/`.** The moment a game needs its own parts or ships, drop those files into that game's dir
and they override — no mechanism change. That gives you the current two games with minimal churn and
a clean path to fully custom games.

---

## 4. CLI design (player-facing surface)

```
./osp --game ksp                     # play the KSP game (system from games/ksp, rest shared)
./osp --game eerbon
./osp                                # default game (see below)
./osp --game ksp --system my.json    # override one file (dev)
```

- `--game <name>` is the new primary flag; it sets the base dir to `games/<name>/`.
- Existing `--system/--parts/--ship/--fleet` become explicit overrides (they already are file
  paths, so they slot into precedence level 1 with no new code shape).
- **Default game:** pick one (e.g. make `ksp` the default, or a `games/default` symlink) so a bare
  `./osp` just works. Recommend defaulting to KSP since it's the fullest.
- Optional: `--list-games` (or `--game ?`) enumerates `games/*/` so the player sees what's loadable.
  Cheap, and it's the "game menu" you'd otherwise need a UI for.

---

## 5. Other ideas (small, optional)

1. **Fleet references ships by name, not path.** Change `fleet.json` `"ship": "res/ships/basic.json"`
   to `"ship": "basic"` and resolve to `<game>/ships/basic.json` (falling back to `_shared/ships/`).
   Removes the last path-based link in the data; the whole chain becomes names.
2. **Ship auto-discovery.** A game's ships = all files in its `ships/` (then `_shared/ships/`). No
   list to maintain; drop a file to add a ship.
3. **Point `gen_systems.py` at the game dirs.** It currently writes `system.json` / `ksp_system.json`
   to the repo root (lines 289-290). It should write `games/eerbon/system.json` and
   `games/ksp/system.json` — the generator already *is* the "these are two games" statement.
4. **Per-game metadata later.** If you want a display name / description / default scenario, add a
   tiny `games/<name>/game.json` *inside* the game dir. Note this is different from Option B: it's
   colocated and self-contained, not an external pointer. Don't add it until you need it.
5. **Keep `res/` pure assets.** End-state: `res/` has zero JSON (all data is under `games/`). Clean
   rule: "res/ = things the renderer loads, games/ = things the player picks."

---

## 6. Migration cost (concrete, small)

Assuming "games carry system.json; parts/ships/fleet → `_shared/`":

1. `mkdir games/{ksp,eerbon,_shared/ships}`.
2. `git mv ksp_system.json games/ksp/system.json`; `git mv system.json games/eerbon/system.json`.
3. `git mv res/parts.json games/_shared/parts.json`;
   `git mv res/ships/basic.json games/_shared/ships/basic.json`;
   `git mv res/fleet.json games/_shared/fleet.json`.
4. `main.cpp`: add `--game`; change the four defaults to the precedence lookup (game dir →
   `_shared` → legacy res); update help text + the two doc comments.
5. `fleet.cpp:35` default ship path; `fleet.h` / `shipdef.h` doc comments.
6. `tests/test_shipload.cpp`, `tests/test_fleet.cpp`: point at the new paths.
7. `gen_systems.py:289-290`: emit into the game dirs.
8. `build_ship` (`main.cpp:1375-1376`) **does not change** — assets still resolve to `res/`.

Everything is a path string; no logic changes except the small precedence helper. Reversible with `git mv` back.

---

## 7. What I'd do first (one reversible step)

Introduce `--game <name>` + the precedence helper, move **just the two system files** into
`games/{ksp,eerbon}/`, and leave parts/ships/fleet where they are (as the fallback). That proves the
whole mechanism end-to-end with the smallest possible diff, keeps both games working, and leaves the
bigger `_shared/` consolidation as a follow-up you can take when (and if) a game needs its own parts.
