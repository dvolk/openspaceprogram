# Science v1 — design review

Date: 2026-09-29
Status: design + staged work plan. Nothing built yet.
Rev 2: §2.4 reverses the Rev 1 recommendation about where experiment data
lives, after a fact-check pass against the source. Changelog in §8.

## 0. What was asked

Add v1 of science:

- A game-level science counter plus a list of recovered experiments, both
  saved and loaded. For now it is only a score.
- Kerbals perform experiments; the result depends on the experiment type,
  the body, the altitude above that body, and the body's biome. The user
  picks a kerbal and clicks a button in the part window.
- Clicking the capsule also runs an experiment, picking a kerbal at random
  (no roles or skills yet).
- v1 experiments are `"<low/high> orbit observation of <biome> on <body>"`,
  and the set should be extensible.
- Experiment data lives on kerbals, saved and loaded, unlimited capacity.
  Later, other parts may hold data, and kerbals may be limited to one.
- Recovering a vessel converts its kerbals' experiments into the score.
  Experiments count once; diminishing returns or a repeat penalty may come
  later.

Four forks were settled before this report was written (§8).

**Verdict, short form:** the proposal is sound and fits the codebase's
existing patterns unusually well — it is `flightlog.h` + the `suit_fuel`
save precedent + a part-window button, and nothing in it needs new
machinery. Three changes make it materially better: split *experiment* from
*situation* (that is the real extensibility axis), store only a subject *id*
and derive the display text and value from it, and pick the crew
deterministically rather than at random. One Rev 1 recommendation was wrong
and is reversed in §2.4.

---

## 1. Current state (facts, with line references)

### 1.1 There is no global game state at all

`struct Game` (`src/game.h:223-861`) borrows its subsystems
(`display, ships, sys, sun, home, args`, game.h:225-232) and owns runtime
state: `time`/`time_accel` (game.h:349-350, mutated only through `setTime`,
game.h:434), `ship` (game.h:457), `kerbal`/`lastShip` (game.h:465-466),
`part_sels` (game.h:471), `flightSummary` (game.h:538-543), `view`
(game.h:546).

Grepping `src/` for `money|science|funds|score` returns nothing. The only
persisted global is `SaveMeta` (`src/save.h:192`), whose single gameplay
field, `exhaust_scale`, lives awkwardly in the borrowed `g.args` and
round-trips at `save.cpp:630` / `save.cpp:664-667`.

**A science counter is therefore the first piece of career state in the
codebase.** That is not a blocker — `exhaust_scale` is a complete, if
awkward, template — but it means there is no existing "career" struct to
hang it on, and the teardown hygiene has to be thought about (§6, R1).

### 1.2 A save is a directory; the pure JSON layer is header-only

`dir/save.json` (global, `SaveMeta`) + `dir/ships/<slug>.json` (one
`SaveShip` per vehicle; slug is `"v" + index` in canonical fleet order,
`save.cpp:44`). All (de)serialization is **inline in `save.h`** so it is
unit-testable with no Game and no Bullet; the Game-coupled capture/restore
lives in `save.cpp`.

Reads are permissive by convention — `if(j.contains(k) && <type check>)`,
absent key keeps the struct default (`save.h:409-470`). **New keys need no
migration and no format bump.** This is the same contract `flog` used
(`save.h:357-358` writes only when started; `save.h:415-416` reads before the
`is_crew` split).

`SaveMeta` keys today (`saveMetaToJson`, save.h:471-483):
`format, saved_at, system, parts, time, time_accel, active_ship,
exhaust_scale, ships[]`.

### 1.3 A kerbal is a one-part `Vehicle`, and crew state has a save precedent

`struct Kerbal : Vehicle` — `src/eva.h:35`, built from `res/ships/kerbal.json`
(one part, `"kerbal"`, `res/data/parts.json:960-972`). Virtuals
`isEva()/isCrewAboard()/capsulePart()` at eva.h:52-54.

Kerbals are saved as `SaveShip` records with `is_crew` set
(`save.h:134`), and the crew-only fields are already there:

| field | decl | capture | restore |
|---|---|---|---|
| `flog` | save.h:143 | save.cpp:194 | save.cpp:460-462 (ships), 521-523 (crew) — **before** `setSoi`, which observes |
| `aboard` / `aboard_part` | save.h:168 / 176 | — | save.cpp:594-596 |
| `suit_fuel` | save.h:181 | save.cpp:212-217 | save.cpp:500-509 |
| `suit_inventory` | save.h:187 | save.cpp:219-223 | save.cpp:511-521 |

`suit_fuel` and `suit_inventory` are exactly the shape of "per-kerbal state
that is not part geometry". **`experiments` is the fifth entry in that
table.**

Ownership and the containment edge: `Vehicle::crew` (`src/vehicle.h:111`,
`std::vector<Vehicle*>`) is the sole owner; `~Vehicle` deletes crew. Aboard,
`Kerbal::aboardPart` (eva.h:70) points at the capsule `Part*` and the capsule's
`Part::contents` holds `k->parts[0]` (`src/part.h:117-135`); the two are kept
in lockstep at spawn (`ships.cpp:275-278`), board (`game.cpp:908`) and load
(`save.cpp:594-596`).

Ready-made accessors: `shipCrew(Vehicle*)` → `std::vector<Kerbal*>`
(game.cpp:783-787), `partCrew(Part*)` (game.cpp:789-802),
`freeKerbals(System&)` (game.cpp:804+). The `static_cast<Kerbal*>` downcast
of a `crew` entry is the established idiom (game.cpp:785, 799, 808).

Naming: `k->name = dedupName(sys, "kerbal")` (`ships.cpp:240`), which appends
`" #2"`, `" #3"`… against the live fleet (`ships.cpp:208-221`), and
round-trips verbatim (`save.cpp:485`). **There is no stable per-kerbal id** —
`Part` has a process-wide `uid` (`part.h:70`), `Vehicle`/`Kerbal` do not. So a
"already recovered" set can never be keyed by kerbal; it must be keyed by the
*subject* (§2.3).

### 1.4 `recoverActive` is the only score-worthy removal path

`Game::recoverActive()` — declared game.h:802, implemented
`src/game.cpp:1564-1643`:

1. guard `ship == nullptr` (1565-1569)
2. re-run SoI detection, then `v->flog.observe(time, …)` (1578-1580)
3. **snapshot `flightSummary.{shipName,log,end_t}` (1581-1583)** ← the harvest
   goes here
4. `dropPartWindowsFor(v)` and for each aboard crew (1587-1588)
5. clear other ships' dock intents (1591-1597)
6. drop `ship/lastShip/kerbal` refs through the `diesWith` lambda (1601-1611)
7. `v->detachSoiList(); delete v;` (1615-1616) — `~Vehicle` frees `v->crew`
8. `syncShipFocus(); enterSpaceCenter(*this); setWinOpen(W_FlightSummary, true);`
   (1620-1622)
9. `printf("[recover] …")` then a `[flight] …` block (1624-1638), `fflush`,
   `toast` (1642)

Step 9's printfs are the e2e anchors (`e2e/cases/101-recover-ship.txt`
EXPECTs `[recover] t=`). UI entry: the hub menu button at
`gameui.cpp:2568-2576`. Headless: `--recover MS` (`cli.h:100-101`, fired
`main.cpp:1190-1196`).

**Crew are deleted inside step 7**, so the harvest must run between step 3 and
step 4 — literally where `flightSummary` is filled.

`Game::remove_ship` (game.cpp:1444) is the *other* removal path. It refuses
crewed ships, produces no summary, and must **not** award science.

Teardown: `clearFlightSummary()` (game.cpp:265-268) is called from
`unloadGame` (game.cpp:436), `newGame` (game.cpp:569) and `load_game`
(save.cpp:765). That is *per-mission* clearing; a career science total must
not be cleared there (§6, R1).

### 1.5 Biome classification exists, is cheap, and has no consumer yet

All of it is in **`src/terragen.h`** (pure header, worker-safe) — not
terrain.h/.cpp. Added by 7df2065:

```cpp
enum class Biome : unsigned char { None, Ocean, Lowland, Midlands, Mountain };  // terragen.h:409-415
inline Biome biomeFromAltitude(double alt, const Surface &s);                   // terragen.h:421-429
inline Biome biomeAt(const glm::vec3 &p, const TerrainParams &t);               // terragen.h:433-437
inline const char *biomeName(Biome);                                            // terragen.h:440-448
```

`biomeFromAltitude` takes metres above sea level: `s.bands → None`;
`s.has_sea && alt <= 0 → Ocean`; `alt >= 0.8*max_height → Mountain`;
`>= 0.5*max_height → Midlands`; else `Lowland` (flat-body guard `mh <= 0 →
Lowland`). `biomeName` returns `"ocean"/"lowlands"/"midlands"/"mountains"/"none"`.

**`biomeAt`'s `p` is a UNIT DIRECTION in the body's ROTATING frame**, not a
lat/lon pair. Lat/lon → direction is the surfmap convention
`dir = (cos lat · sin lon, sin lat, cos lat · cos lon)` (`surfmapDir`,
`src/surfmap.h:27-33`; inverse `surfmapLonLat`, surfmap.h:39-44).

`surfmap.cpp` does **not** use biomes — it uses raw `terrainHeight` for the
Ocean-checkbox sea paint (`surfmap.cpp:110-114`). Today the only consumer is
`tests/test_terrain.cpp:587-647`. **Science is the first real consumer.**

Cost: one `biomeAt` is one full-detail `terrainHeight` FBM sample (9 simplex
octaves by default, terragen.h:134, 338-340) — the identical query the HUD's
Alt readout already runs *every frame*
(`ship->m_parent->GetTerrainHeight(normalize(pos))`, `gameui.cpp:362`;
delegate `terrain.h:220-222`). No mesh generation. Per-tick is fine, and
per-button-click is free.

**Readiness caveat:** classification depends on `surface.max_height`, which is
*measured numerically* by the heavy phase (`BuildRootGeoms`, terrain.h:249-282,
`maxh = std::max(1.0f, (hi - surface.sea_level) * 1.05f)` at terrain.h:281) and
applied by `AttachRoot` (terrain.h:305+, `ready` flag terrain.h:174-177). Until
then `max_height` is its 1.0 default and **everything above 0.8 m reads
"Mountain"**. The active ship's body is always synchronously built, so queries
at the ship's own position are safe; queries against a distant, still-streaming
body are not.

### 1.6 There is no situation enum; here is what is derivable

Nothing in the codebase distinguishes landed / splashed / flying / orbiting.
Available primitives:

- `Vehicle::inTerrainBand()` — vehicle.h:1189, vehicle.cpp:2897-2903:
  `periapsis <= body->radius + 3000`, using COM state in the inertial node
  (`comStateIn`, then `computeOrbitElements`).
- `Vehicle::canRail()` — vehicle.cpp:2905-2911; grounded =
  `inTerrainBand() && frame->isRotFrame()`; a grounded railed ship is
  `onRails && railFrozen` (vehicle.h:383-388).
- EVA contact-based ground truth: `Kerbal::grounded` (eva.h:37), rewritten every
  tick by `evaArmCommands` (eva.cpp:35-100); the contact check is eva.cpp:45-53.
- Orbit: `o.periapsis - m_parent->radius > 0` from `OrbitElements`
  (`computeOrbitElements`, `src/orbit.h:57`; `OrbitElements` orbit.h:22-45,
  `-1` for hyperbolic).
- **Splashed does not exist anywhere.** It would be derived as
  `biome == Ocean` while grounded. Oceans are just terrain below sea level.

Altitude has no single helper; the HUD idiom is `gameui.cpp:361-364`:

```cpp
const double asl = distance - ship->m_parent->radius;
const double agl = distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos));
```

`ShipView` (`game.h:107-140`, filled by `updateShipView`, `render.cpp:108-224`)
carries the per-frame snapshot: `pos/vel`, `surf_pos/surf_vel` (rotating frame),
`orbit_pos/orbit_vel` (inertial), `o` (OrbitElements), `mu`,
`latitude/longitude` (render.cpp:222-223). **It is written by the render path**,
so headless runs cannot rely on it (§6, R3).

Atmospheres *are* modelled: `AtmosphereParams` (`terragen.h:88-101`) with
`enabled`, `thickness` (a **visual shell** radius above `radius + max_height`,
not an edge of space), `sea_level_density` and `scale_height` feeding the drag
model `rho(alt) = sea_level_density * exp(-alt / scale_height)`
(`src/drag.h`). Authored values:

| body | radius | scale_height | ×10 = edge of space |
|---|---|---|---|
| Kerbin | 600 km | 5 500 m | 55 km |
| Eve | 700 km | 7 000 m | 70 km |
| Shay | 550 km | 6 000 m | 60 km |
| Laythe | 500 km | 5 500 m | 55 km |
| Duna | 320 km | 4 000 m | 40 km |
| Jool | 6 000 km | 20 000 m | 200 km |

`scale_height * 10` lands within a few km of the reference game's atmosphere
tops, so it is a usable edge-of-space with no new authored data.

### 1.7 Body, SoI and identity

`Vehicle::m_parent` (`TerrainBody*`, vehicle.h:98) and `Vehicle::frame`
(vehicle.h:99). `setSoi(Frame*, double t)` (vehicle.h:1129-1142,
vehicle.cpp:2821-2851) is the **one** writer of frame / m_parent / ships-list /
`flog.observe` (3ae9c43), and it re-homes aboard crew too (vehicle.cpp:2850).

Bodies are identified by plain name strings (`TerrainBody::name`,
terrain.h:160; `System::find(name)`, `src/system.h:28-33`), which is what saves
reference everywhere (`SavePose.body`). `res/systems/ksp_system.json` has 18
bodies, `"home": "Kerbin"`. Seas: **Kerbin, Shay, Laythe** have `has_sea: true`
(a body-level key, not inside `surface`); the other 15 do not. `Jool` is the
only `bands: true` body → `Biome::None`.

`sea_level` is **never authored** in any system JSON; it is always the
`Surface` default of 0.0 (`terragen.h:138`).

### 1.8 Part window, Flight Summary, and the window recipe

`void drawPartWindows(Game &g)` — `src/gameui.cpp:1742-2010` (decl
gameui.h:19-25). One plain `ImGui::Begin` per `g.part_sels` entry
(`PartSel`, game.h:96-104), opened by RMB → `pickAt` (game.h:846) →
`Game::openPartWindow` (game.cpp:244). Sections: stats (1794-1840), docking
(1841-1877), **crew EVA/Board buttons (1879-1917)**, **inventory Drop/Pick up
(1918-1990)**. The button idiom:

```cpp
ImGui::PushID(item);
ImGui::Text(" %s (%.1f kg)", in, item->effectiveMass());
if(ImGui::SmallButton("Drop")) { g.dropItem(item); }
ImGui::PopID();
```

Aboard crew are enumerated on demand: `std::vector<Kerbal*> aboard =
partCrew(ship->parts[part]);` (gameui.cpp:1886).

Flight Summary — payload `Game::FlightSummary {shipName, log, end_t}`
(game.h:538-543), drawn by `drawFlightSummary(Game&)` (gameui.cpp:2720-2771)
via `drawWin(g, W_FlightSummary, …)`, declared gameui.h:49-53, called from
`spaceCenterDrawUi` (scene.cpp:101-106).

Registering a new window (full recipe): add `W_Science` to the `Win` enum
(`src/uiwins.h:63-77`, before `W_Count`) → add a `kWins[]` entry
(`src/uiwins.cpp`; the FlightSummary model is uiwins.cpp:236-246) → add it to
the scene set (`kSpaceCenterWinIds` uiwins.cpp:310-312, `kFlightWinIds`
uiwins.cpp:293-297) → write `drawScience(Game&)` with
`drawWin(g, W_Science, body)` (template uiwins.h:118-123) → call it from the
scene's drawUi (scene.cpp:33-38 flight, 101-106 space center). Optional key
binding: new `Slot` in keys.h:44-70, default in `KeyBindings::resetDefaults`
(keys.cpp:17-101), entries in the slotName/slotLabel tables, fired in
events.cpp — the `DebugInfo` toggle is the pattern (events.cpp:294-298).

### 1.9 Tests and e2e

`tests/test_flightlog.cpp` (112 lines) is the template for a header-only
test: include the header, local `CHECK` macro (:14-19), brace-scoped sections,
print `test_flightlog: all checks passed` / `N FAILURE(S)`, return 0/1. Build
rule links nothing (Makefile:515-516). Adding a test = a target near :515, the
name in `TESTS = …` (:646-652), a run line in the `test` target (:666-703);
short-name aliases come free from `.PHONY: $(TESTS)` (:660-663).

`tests/test_save.cpp` (684 lines) round-trips the header-only layer with its own
`CHECK` (:29-34): flog (:166-255, including no-key-when-fresh and
malformed-flog), crew SaveShip (:257), `suit_fuel` (:275), `suit_inventory`
(:365), SaveMeta (:444), permissive/wrong-type reads (:469), uid strictness
(:512-540). Registered Makefile:498-499.

e2e: cases are `e2e/cases/*.txt`, auto-discovered (`e2e/run.py:941`) — **adding
one is dropping a file in**. Format (run.py:41-58): `NAME`, `ARGS`, `EXPECT`,
`FORBID`, `CHECK <python expr>`, `LIMIT`, `WRITE`. Model case
`e2e/cases/101-recover-ship.txt`:

```
NAME recover-ship
ARGS --time-accel 1 --timeout 8 --startship racer,res/ships/racer.json,Kerbin,rot-orbit
ARGS --space-center 1000 --recover 2000
EXPECT [scene] flight -> spacecenter (push)
EXPECT [recover] t=
```

Save e2e precedent: `53-save.txt` / `54-save-load.txt` (`--save tmp/e2e_save`,
`--reload`). Run: `make e2e` (Makefile:719-721) or `python3 e2e/run.py recover`.

---

## 2. Evaluation of the proposal

### 2.1 What is right

- **Data on kerbals, converted on recovery.** Correct, and it is the only
  option that survives the codebase's topology: kerbals have no persistent id
  (§1.3), and dock-absorb deletes a ship's identity (issue #49 already notes
  `flog` is lost there). A `Kerbal` object survives boarding, EVA and
  `absorbShip` unchanged.
- **Unlimited capacity, counted once.** Bounded by construction (§2.7), so
  neither saves nor memory can grow without limit.
- **Score-only for now.** Right call. A spendable currency needs something to
  spend it on; inventing a tech tree in the same change would bury the
  feature.
- **Saved and loaded from day one.** The permissive save format (§1.2) makes
  this nearly free, and doing it later is much worse — an unpersisted score
  trains the player to treat it as fake.

### 2.2 Split *experiment* from *situation* — that is the extensibility axis

`"<low/high> orbit observation of <biome> on <body>"` bundles four independent
things:

| axis | v1 values | later |
|---|---|---|
| experiment (the instrument) | `observation` | `surface_sample`, `temp_scan`, `crew_report`, `eva_report` |
| situation (how the vessel is flying) | `space_low`, `space_high` | `landed`, `splashed`, `flying_low` |
| body | from `m_parent` | — |
| biome | from `biomeAt` | — |

If the experiment's *name* encodes "low orbit", then adding a surface sample
means authoring one experiment per situation — combinatorial, and the situation
logic gets duplicated per experiment. Instead:

```cpp
struct ExperimentDef {
    const char *id;         // "observation"
    const char *name;       // "Orbital Observation"
    unsigned situations;    // bitmask: where this instrument works
    double base_value;      // 10.0
};
```

v1 ships one entry valid in `space_low | space_high`. A future `surface_sample`
is one more entry with `landed | splashed` and no new plumbing. This is also
exactly how `PartDef` already works — behavior derived from optional fields
that "combine freely" (shipdef.h:230-234), so it is idiomatic here rather than
an imported abstraction.

### 2.3 Store the subject *id*; derive the name and the value

Both the global recovered set and the kerbal's held data should be plain
strings:

```
observation@Kerbin@space_low@mountain
observation@Kerbin@space_high@
```

with one `encodeSubject`/`decodeSubject` pair, and the display text and score
computed from the decoded subject on demand. Always four `@`-separated fields,
last one empty when there is no biome — so decode is a fixed split with no
special cases.

Three reasons:

1. **Display text must not be the key.** Wording changes ("Mountains" →
   "Mountain Range") would invalidate every save. `biomeName()`
   (terragen.h:440-448) is a *display* function; a separate `biomeToken()`
   returns the stable id token.
2. **Value must not be cached.** Deriving it at recovery means a balance change
   applies retroactively and there is nothing to go stale.
3. **It makes the whole model header-only pure C++**, testable without linking
   `Vehicle` — precisely the `flightlog.h` doctrine ("pure containers + logic —
   no game types — so tests can pin the behavior", flightlog.h:20-22).

A side effect worth having: `recovered_subjects` is a set of ids drawn from a
finite space, so it is bounded (§2.7). An event-log design would not be.

### 2.4 CORRECTION — the data belongs on `Kerbal`, not on `Part`

Rev 1 of this review recommended `Part::experiments`, on the grounds that
Parts are the persistent, uid-stable objects that survive containment
transitions, and that it would directly serve the "later other parts may hold
experiment data" requirement. **That was wrong for v1.** The fact-check found:

- Per-kerbal state already has a home and a save path: `suit_fuel` and
  `suit_inventory` are `SaveShip` crew fields (save.h:181-187) captured at
  save.cpp:212-223 and restored at save.cpp:500-521. `experiments` is the fifth
  row of that table and needs no new plumbing shape.
- `SavePart` (save.h:85-102) carries geometry, stage, mass, hull margin, fuel
  and nested inventory — all part-generic. A `experiments` vector there would
  be dead weight on every part of every ship (~24 bytes × parts × ships) for a
  field only kerbals use.
- The Kerbal object already survives every transition that matters: boarding
  (game.cpp:908), EVA (game.cpp:822), dock-absorb (`absorbShip` moves `crew`).
  Nothing is gained by moving the data down to the Part.

So: `std::vector<std::string> experiments;` on `Kerbal` (`src/eva.h:35`), next
to `aboardPart`. When a science-module *part* eventually needs storage, that is
the moment to add `Part::experiments` + `SavePart::experiments`, and the subject
id format is already the shared currency — so deferring costs nothing.

### 2.5 Pushback accepted: pick the crew deterministically, not at random

Random buys nothing in v1 — there are no roles or skills, and the data pools at
recovery regardless of who holds it — and costs two things: the capsule button
cannot say who will run it, and any e2e assertion about *which* kerbal holds the
data becomes flaky. Settled: **first crew in `partCrew(part)` order**
(game.cpp:789), with the button labelled `Run experiment (kerbal #2)`.

If spreading data across the crew ever matters, a round-robin index on the
capsule part is still deterministic. When roles/skills land, the choice starts
to carry meaning and can be revisited then.

Corollary for v1, stated plainly: **an uncrewed capsule cannot run
experiments** — the button is disabled and says why. Probes and labs are a later
phase.

### 2.6 Biome resolves from low orbit only

Settled: `space_high` yields a body-level observation with an empty biome;
`space_low` resolves the biome under the vessel. Without this the two
situations differ only in a label and there is no reason to prefer either.

Note where the rule lives: it is a property of the **situation**, not of the
experiment, so it belongs in the subject builder (`if situation == space_high
→ biome = None`) and every future experiment inherits it for free. No flag on
`ExperimentDef`.

Also note what the biome *is*: `biomeAt` samples `terrainHeight` at the
sub-vessel point, so the biome is that of **the ground below**, independent of
the vessel's own altitude (§1.5). That is exactly right for "observation of
<biome>" and needs no extra work.

### 2.7 Dedup makes the score bounded — here is the actual ceiling

With `has_sea` on Kerbin/Shay/Laythe only, `Jool` banded, and biome resolved in
low orbit only:

| body class | count | low-orbit subjects | high-orbit | total |
|---|---|---|---|---|
| sea + terrain (Kerbin, Shay, Laythe) | 3 | 4 biomes | 1 | 5 each |
| terrain, no sea | 14 | 3 biomes | 1 | 4 each |
| banded (Jool) | 1 | 0 biomes | 1 | 2 |

**73 subjects × 10.0 = 730 science** is the v1 ceiling for the shipped system.
That is a good number: finite, completionist-visible, and it means
`recovered_subjects` can never exceed 73 short strings no matter how long a
career runs. It also means the score is a *map* — the only way to earn more is
to go somewhere new, which is the point.

### 2.8 Reuse the Flight Summary; do not invent a dialog

`recoverActive` already fills `flightSummary` (game.h:538-543) and already opens
`W_FlightSummary` (game.cpp:1622). Adding `double science` to that struct and
one line to `drawFlightSummary` (gameui.cpp:2720-2771) gives "+30.0 science
recovered" for free, with correct per-mission clearing via
`clearFlightSummary()` (game.cpp:265-268). A separate `W_Science` window then
shows the career total and the recovered subjects grouped by body.

### 2.9 The button must say NEW vs already-recovered

Since duplicates score 0 (§2.7), a second low-orbit pass over the same biome
silently does nothing. Without a hint the button looks broken. The subject is
computable from the vessel's current state *without* running anything, so the
part window can label it:

```
Run experiment (kerbal #2)
  Low orbit observation of the Mountains on Kerbin   [NEW]
```

and grey it to `already recovered` otherwise. This is a requirement, not polish
— it is the only feedback loop v1 has.

---

## 3. Recommended architecture

### 3.1 `src/science.h` — header-only, pure, no game types

```cpp
enum class Situation : unsigned char { SpaceLow, SpaceHigh };   // v1; extended later
inline constexpr unsigned situMask(Situation s) { return 1u << unsigned(s); }
inline const char *situationToken(Situation);   // "space_low" / "space_high"
inline std::optional<Situation> situationFromToken(std::string_view);

struct ExperimentDef { const char *id, *name; unsigned situations; double base_value; };
inline const ExperimentDef *findExperiment(std::string_view id);
inline constexpr ExperimentDef kExperiments[] = {
    { "observation", "Orbital Observation", situMask(SpaceLow)|situMask(SpaceHigh), 10.0 },
};

struct Subject {                    // value type; no allocation
    std::string_view experiment;    // points into kExperiments
    std::string_view body;
    Situation situation = Situation::SpaceLow;
    Biome biome = Biome::None;      // terragen.h:409
};
std::string encodeSubject(const Subject&);                    // 4 '@' fields, last may be empty
std::optional<Subject> decodeSubject(std::string_view);       // nullopt on malformed
std::string subjectName(const Subject&);                      // display text
double subjectValue(const Subject&);                          // findExperiment()->base_value

// --- the low/high split -------------------------------------------------
inline constexpr double kLowOrbitRadiusFrac = 0.25;   // ceiling = radius * this …
inline constexpr double kLowOrbitFloor      = 25000.0;// … but never below this (m)
inline constexpr double kAtmoScaleHeights   = 10.0;   // edge of space = scale_height * this

// Pure classifier. `grounded` and the two thresholds are computed by the
// caller from the body; nullopt = no v1 experiment applies here.
std::optional<Situation> situationFor(bool grounded, double alt_asl,
                                      double space_floor, double low_ceiling);
```

The thresholds need two constants rather than one because body radii in
`ksp_system.json` span three orders of magnitude (Gilly 13 km → Jool 6 000 km).
A bare radius fraction makes Gilly's low-orbit ceiling 3.25 km inside a 126 km
SoI, which is unflyable; the floor fixes it:

| body | radius | `max(r×0.25, 25 km)` | edge of space |
|---|---|---|---|
| Gilly | 13 km | 25 km | 0 (airless) |
| Minmus | 60 km | 25 km | 0 |
| Mun | 200 km | 50 km | 0 |
| Kerbin | 600 km | 150 km | 55 km |
| Jool | 6 000 km | 1 500 km | 200 km |

Both constants are tunables in one place; if the numbers ever feel wrong the
alternative is a per-body `low_orbit_alt` in the system JSON, which needs no
back-compat (QWEN.md).

Display strings (v1):

- `space_low` + biome → `"Low orbit observation of the Mountains on Kerbin"`
- `space_high` → `"High orbit observation of Kerbin"`

### 3.2 Game-side glue (thin; all of it in existing files)

```
Kerbal::experiments            src/eva.h:35        std::vector<std::string>, subject ids
SaveShip::experiments          src/save.h:187      next to suit_inventory
Game::science                  src/game.h:538      double, next to flightSummary
Game::recovered_subjects       src/game.h          std::unordered_set<std::string>
Game::FlightSummary::science   src/game.h:538-543  per-mission award
SaveMeta::science / ::recovered src/save.h:192
```

Two new functions carry all the logic:

- `Subject scienceSubjectFor(Vehicle &v)` — the **sampling** function, run at
  click time. Reads `v.m_parent` (name, `radius`, `params().surface.atmosphere`),
  the vessel's position in the body's rotating frame for `biomeAt`, altitude
  ASL, and the grounded predicate. Returns the subject, or signals "not
  possible here".
- `double Game::harvestScience(Vehicle &v)` — the **conversion** function, run
  from `recoverActive` between game.cpp:1583 and 1587. Walks `shipCrew(v)`,
  and for each subject id not already in `recovered_subjects`, adds
  `subjectValue` and inserts. Returns the mission total for
  `flightSummary.science`.

Sampling at click time (not at recovery) is what makes eccentric orbits
unambiguous: the situation is a fact about the instant the observation was
made.

### 3.3 Grounded predicate — needs empirical confirmation

Intended:

```cpp
bool vesselGrounded(const Vehicle &v) {
    if(v.isEva()) { return static_cast<const Kerbal&>(v).grounded; }   // eva.h:37
    return v.inTerrainBand() && v.frame && v.frame->isRotFrame();      // vehicle.cpp:2897, frame.h:81
}
```

The EVA half is contact-based and certain. The ship half is inferred from the
`canRail()` logic (vehicle.cpp:2905-2911) and must be **confirmed with debug
prints on the pad and in a 100 km Kerbin orbit before the button is wired**
(the project's stated approach to uncertainty). Fallback if the rails predicate
proves unreliable: require `alt_agl` above a small clearance in addition to the
space floor.

---

## 4. Work plan

Steps are deliberately small: each is independently committable, each has a
named verification, and none depends on a later one. Per project rules, run
`make test` and the relevant e2e cases before each commit.

### Phase 0 — the pure model (no behavior change)

**0.1 — `src/science.h` + `tests/test_science.cpp`.**
Everything in §3.1. No game types, no includes beyond `<optional> <string>
<string_view> <vector>` and `terragen.h` for `Biome`.
*Files:* `src/science.h`, `tests/test_science.cpp`, `Makefile` (target near
:515, name in `TESTS` :646-652, run line :666-703).
*Verify:* `make test_science` then `make test`. Cases: encode/decode round-trip
for every situation × biome including the empty-biome form; malformed ids →
`nullopt` (wrong field count, unknown experiment, unknown situation, bad
biome); `situationFor` across the threshold table in §3.1 (grounded → nullopt,
below space floor → nullopt, either side of the ceiling, airless body with
`space_floor == 0`); `subjectValue` for a known and an unknown experiment.

**0.2 — nothing else.** Resist wiring it up in the same commit; the point of
phase 0 is that `make test` proves the model before any game code touches it.

### Phase 1 — data on the kerbal, persisted

**1.1 — `Kerbal::experiments` + save/load.**
Field on `Kerbal` (eva.h:35, next to `aboardPart`), `SaveShip::experiments`
(save.h, next to `suit_inventory` :187) with inline
`saveShipToJson`/`FromJson` entries following the permissive pattern
(save.h:352/409), capture in `saveShipFromVehicle` next to save.cpp:219-223,
restore in `buildKerbalFromSave` next to save.cpp:511-521. Write the key only
when non-empty, exactly like `flog` (save.h:357-358).
*Files:* `src/eva.h`, `src/save.h`, `src/save.cpp`.
*Verify:* `tests/test_save.cpp` — a crew `SaveShip` round-trip carrying three
subject ids (model on the `suit_inventory` case at :365); no-key-when-empty;
malformed entry (non-string element) keeps the default. `make test`.

**1.2 — an e2e save/load round-trip is deferred to phase 3**, because there is
no way to *create* an experiment yet. Note it here so it is not forgotten.

### Phase 2 — career state + the recovery conversion

**2.1 — `Game::science` and `Game::recovered_subjects`.**
Declared next to `flightSummary` (game.h:538). `SaveMeta` gains `science` and
`recovered` (`vector<string>`), with `saveMetaToJson`/`FromJson` entries
(save.h:471/485), written in `save_game` next to save.cpp:625-631 and read in
`load_game` next to save.cpp:656-667.
**Reset both in `newGame` (game.cpp:569) and in `unloadGame` (game.cpp:436).**
Do *not* put them in `clearFlightSummary()` (game.cpp:265-268) — that is
per-mission. See §6 R1.
*Files:* `src/game.h`, `src/game.cpp`, `src/save.h`, `src/save.cpp`.
*Verify:* `tests/test_save.cpp` SaveMeta round-trip with a non-empty recovered
list (extend :444); `make test`.

**2.2 — `Game::harvestScience` + the Flight Summary line.**
Called from `recoverActive` between game.cpp:1583 and 1587 (before
`dropPartWindowsFor`, well before the `delete v` at 1616). Adds
`double science` to `FlightSummary` (game.h:538-543), zeroed by
`clearFlightSummary()`. One line in `drawFlightSummary` (gameui.cpp:2720-2771).
Add a `[science]` printf inside the existing block at game.cpp:1624-1638:
`[science] vessel='X' earned=30.0 total=120.0 subjects=12`.
**Only `recoverActive`.** `remove_ship` (game.cpp:1444) must not award.
*Files:* `src/game.h`, `src/game.cpp`, `src/gameui.cpp`.
*Verify:* unit coverage of the dedup rule is not possible without Game, so
prove it in phase 3's e2e. `make test` must stay green.

### Phase 3 — running experiments (the interaction)

**3.1 — `scienceSubjectFor(Vehicle&)` + the grounded predicate.**
Confirm §3.3's predicate with debug prints first (pad, 100 km orbit, EVA
standing, EVA in free fall). Compute inputs from the **Vehicle**, not from
`g.view` — `ShipView` is written by the render path (render.cpp:108-224) and
headless runs must work (§6 R3). Biome via
`biomeAt(glm::normalize(rot_dir), v.m_parent->params())` (terragen.h:433,
terrain.h:195-201); verify it agrees with `g.view.surf_pos` interactively.
*Files:* `src/game.cpp` (or a small `src/science_game.cpp` if it grows).
*Verify:* debug prints across the four states above; `make test`.

**3.2 — `Game::runExperiment(Part *sel)` and the part-window buttons.**
In `drawPartWindows`: a button in the crew section (gameui.cpp:1879-1917) when
the selected part is a kerbal's suit, and one in the capsule section when
`partCrew(part)` is non-empty — using the first crew member (§2.5) and naming
them on the button. Disabled with a reason when there is no crew, or when
`situationFor` returns `nullopt`. Dedup on insert into `Kerbal::experiments`
(a kerbal holding the same subject twice is indistinguishable from holding it
once in v1, and it keeps saves small).
*Files:* `src/gameui.cpp`, `src/game.h`, `src/game.cpp`.
*Verify:* manual — fly `racer` to a Kerbin orbit, run one, confirm the held id.

**3.3 — headless hook + the e2e case.**
Add `--run-experiment MS` following `--recover MS` exactly (cli.h:100-101,
fired main.cpp:1190-1196): it runs the experiment on the first crew of the
active ship and prints `[experiment] subject='observation@Kerbin@space_low@…'`.
New case `e2e/cases/102-science-orbit.txt` on the `101-recover-ship.txt`
skeleton: start in a Kerbin orbit, `--run-experiment 1500`, `--recover 2000`,
EXPECT `[experiment] subject=` and `[science] vessel=… earned=10.0 total=10.0`.
Then run the experiment twice before recovering and assert `earned=10.0` still
(dedup on insert). Then a second flight recovering the same subject and assert
`earned=0.0` (global dedup). Then a `--save`/`--reload` pair asserting the total
survives (the phase 1.2 debt).
*Files:* `src/cli.h`, `src/cli.cpp`, `src/main.cpp`, `e2e/cases/102-science-orbit.txt`.
*Verify:* `python3 e2e/run.py science` then `make e2e`.

### Phase 4 — the Science window + the NEW hint

**4.1 — `W_Science`.** Full recipe at §1.8: enum (uiwins.h:63-77), `kWins[]`
entry modelled on uiwins.cpp:236-246, scene set (uiwins.cpp:310-312),
`drawScience(Game&)` in gameui.cpp using `drawWin`, declared gameui.h, called
from `spaceCenterDrawUi` (scene.cpp:101-106) and optionally
`flightDrawUi` (scene.cpp:33-38). Content: career total, then recovered
subjects grouped by body, decoded through `decodeSubject` + `subjectName`, with
a per-body `n/5` count so the §2.7 ceiling is visible.
*Verify:* manual + a smoke e2e that opens the window headlessly if one exists
for other windows.

**4.2 — the `[NEW]` / `already recovered` hint** on the phase 3.2 buttons
(§2.9). Pure lookup in `recovered_subjects`; no state change.
*Verify:* manual — run one, recover, run the same one again, confirm the label
flips.

**4.3 — optional key binding** for "run experiment on the selected part"
(§1.8's `DebugInfo` pattern, events.cpp:294-298). Deferred: it needs a
selection, which headless does not have, so it does not help e2e. Nice for
play.

---

## 5. Explicitly *not* in v1

Each is a later phase, and none is needed for the above to be coherent:

- diminishing returns / repeat penalties / a per-subject science cap
- storage limits on kerbals, or a "one experiment per kerbal" rule
- per-body, per-biome or per-situation value multipliers
- kerbal roles, skills, or experience affecting value
- experiment duration, animation, EC cost, or a failure mode
- uncrewed probes, science modules as parts, or labs
- transmitting data over an antenna vs. returning it physically
- a tech tree or anything that *spends* science
- `landed` / `splashed` / `flying_low` situations (and therefore `Splashed`,
  which has no concept anywhere in the codebase yet — §1.6)
- `res/data/science.json` (§8: the table stays in `science.h` for v1)

---

## 6. Risks

**R1 — Career state has no home and no teardown discipline.** Science is the
first global gameplay field (§1.1). `clearFlightSummary()` runs on unload,
new-game *and* load (game.cpp:265-268, 436, 569; save.cpp:765); if science is
cleared there, a load wipes the score, and if it is *not* reset in `newGame`, a
new career inherits the previous one. Both are one-line bugs that no existing
test would catch. Mitigation: reset in `newGame`/`unloadGame`, restore in
`load_game`, and add a `test_save` case plus (if cheap) an e2e
new-game-after-recover check.

**R2 — Biome is wrong on a body whose heavy phase has not landed.** Before
`AttachRoot` (terrain.h:305+), `max_height` is 1.0 and everything above 0.8 m
classifies as Mountain (§1.5). The active ship's body is always synchronously
built, and `scienceSubjectFor` only ever reads `v.m_parent`, so this is
unreachable in v1 — but it is a trap for the next consumer, and it is why the
function must not accept an arbitrary body. Document the constraint at the call
site rather than adding a `ready` guard for a case that cannot happen.

**R3 — `g.view` is render-side.** `ShipView` is filled by `updateShipView`
(render.cpp:108-224). Phase 3's e2e runs headless through `--run-experiment`,
so `scienceSubjectFor` must derive everything from the `Vehicle` and the body.
If it quietly reads `g.view.surf_pos`, the headless path produces a garbage
biome (or a zero direction) and the e2e passes while the game is wrong — or
vice versa. Verify both paths agree interactively.

**R4 — The grounded predicate is inferred, not documented.** §3.3's ship-side
test is assembled from `canRail()`'s internals (vehicle.cpp:2905-2911). If it
misfires, a landed ship offers an "orbit observation" and the whole feature
reads as fake. Debug-print it in four states before wiring any button.

**R5 — Two removal paths.** Science must convert only in `recoverActive`
(game.cpp:1564), never in `remove_ship` (game.cpp:1444). `remove_ship` refuses
crewed ships today, so it cannot silently eat data in v1 — but that guard is
doing load-bearing work for this feature and is not documented as such.

**R6 — Dock-absorb drops ship-identity state.** Issue #49 already records that
`flog` is lost when a ship is absorbed. Experiments on `Kerbal` survive
(`absorbShip` moves `crew`), which is a further argument for §2.4 — but the
absorbed *ship's* flight summary contribution vanishes, so a mission that
docked and recovered as one vessel reports only the survivor's journal.

**R7 — Unknown bodies in old saves.** `decodeSubject` yields a body name
string; if the system JSON changes, that body may no longer exist. v1 should
simply not resolve it (value 0, name falls back to the raw token) rather than
throw — the save layer is permissive everywhere else (§1.2).

---

## 7. Other things noticed (flagged per project convention)

1. **`sea_level` is dead data.** `Surface::sea_level` (terragen.h:138) is never
   authored in any of the six `res/systems/*.json` files, so it is always 0.0.
   `biomeFromAltitude`'s `alt <= 0` Ocean test and the sea paint therefore key
   off `radius`, not `radius + sea_level`. Either author it or note that the
   sea surface *is* the base radius.
2. **Two altitude conventions.** The HUD's ASL is `distance - radius`
   (gameui.cpp:361) while `biomeAt` internally uses
   `terrainHeight - radius - sea_level` (terragen.h:435). Harmless while
   `sea_level == 0` (item 1) and a silent discrepancy the moment it is not.
3. **`AtmosphereParams::thickness` is visual only.** It is the rim-shell radius
   above `radius + max_height` (terragen.h:91, used at terrain.h:336-345), not
   an edge of space — Kerbin's is 15 km against a 5 500 m scale height. Anyone
   looking for "where the atmosphere ends" will find it and be misled.
   `scale_height * 10` is the physically meaningful number (§1.6). A doc
   comment on the field would save the next reader an hour.
4. **`Biome` has no consumer outside tests** (§1.5) — science is the first, so
   its thresholds (`0.5` / `0.8 × max_height`) have never been play-tested.
   Expect to retune them once orbital observations exist.
5. **Kerbals have no persistent identity** (§1.3). Names are
   `dedupName`-generated against the *live* fleet, so "kerbal #2" can be a
   different individual in a later career. Anything that ever needs to key on a
   kerbal across careers will need a uid on `Vehicle`, mirroring
   `Part::uid` (part.h:70).

---

## 8. Decisions taken

| fork | decision | rationale |
|---|---|---|
| capsule crew pick | **deterministic first crew** | the button can name the kerbal; e2e is stable; random buys nothing with no roles/skills |
| biome from high orbit | **low orbit only** | otherwise the two situations differ only by label and neither is worth flying for |
| experiment defs | **hardcoded table in `src/science.h`** | one experiment in v1; a JSON loader is dead weight. Same struct shape a loader would fill, so migrating later is ~30 lines |
| score value | **flat `base_value` per experiment** | nothing to balance a multiplier table against until several experiments exist |

### Rev 2 changelog

- **§2.4 reverses Rev 1.** Experiment data moves from `Part::experiments` to
  `Kerbal::experiments`, after finding the `suit_fuel`/`suit_inventory`
  precedent (save.h:181-187, save.cpp:212-223, 500-521) and confirming that a
  `Kerbal` already survives boarding, EVA and dock-absorb.
- §1.6 gained the atmosphere table: `scale_height * 10` reproduces the
  reference game's atmosphere tops from data that is already authored, which is
  what makes the `space_floor` in §3.1 free.
- §3.1's low-orbit ceiling gained the 25 km floor after checking body radii —
  a bare radius fraction gives Gilly a 3.25 km ceiling inside a 126 km SoI.
- §2.7's subject count is computed from the actual `ksp_system.json`
  (`has_sea` on Kerbin/Shay/Laythe; `Jool` banded): 73 subjects, 730 ceiling.
- §1.5 corrected: biomes live in `terragen.h`, not `terrain.h/.cpp`, and
  `surfmap.cpp` does not use them.
- §7 item 1-3 added after finding `sea_level` is never authored.
