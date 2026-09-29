# Science v1 — design review

Date: 2026-09-29
Status: design + staged work plan. Nothing built yet.
Rev 3: corrected after a full fact-check pass against the source at `7df2065`.
Rev 2 is preserved alongside as `science-design-rev2-superseded.md`. Changelog
in §8. **The Rev 2 grounded predicate (§3.3) was wrong and has been replaced;
the Rev 2 score ceiling was wrong by four subjects.**

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

**Verdict, short form:** the proposal is sound and fits the codebase's existing
patterns unusually well — it is `flightlog.h` + the `suit_fuel` save precedent +
a part-window button, and it needs no new machinery. Three changes make it
materially better: split *experiment* from *situation* (that is the real
extensibility axis), store only a subject *id* and derive the display text and
value from it, and pick the crew deterministically rather than at random.

One Rev 1 recommendation was reversed on the facts (§2.4), and one Rev 2
mechanism was wrong and is replaced (§3.3): the "is this vessel on the ground"
test assembled from `canRail()`'s internals reads *grounded* for the entire
ascent to orbit, because `inTerrainBand()` is an osculating-periapsis test
while the rotating frame reaches 100 km above Kerbin. The replacement is
simpler — one existing primitive, negated — and removes a parameter.

---

## 1. Current state (facts, with line references)

All line numbers verified against the working tree at `7df2065` (clean).

### 1.1 There is no global game state at all

`struct Game` (`src/game.h:223-838`) borrows its subsystems (`display, ships,
sys, sun, home, args`; `GameArgs &args` at game.h:236) and owns runtime state:
`time`/`time_accel` (game.h:349-350, mutated only through `setTime`,
game.h:389), `ship` (game.h:457), `kerbal`/`lastShip` (game.h:465-466),
`part_sels` (game.h:471), `flightSummary` (game.h:538-543), `view`
(game.h:546).

Grepping `src/` for `\b(money|science|funds|score)\b` returns nothing but a
prose hit ("Under**score**d uniform name", `gameui.cpp:537`). The only
persisted global is `SaveMeta` (`src/save.h:192`), whose single gameplay field,
`exhaust_scale`, lives in the borrowed `g.args` and round-trips at
`save.cpp:630` / `save.cpp:664-667`.

**A science counter is therefore the first piece of career state in the
codebase.** Not a blocker — `exhaust_scale` is a complete, if awkward, template
— but there is no existing "career" struct to hang it on, and the teardown
hygiene has to be thought about (§6, R1).

### 1.2 A save is a directory; the pure JSON layer is header-only

`dir/save.json` (global, `SaveMeta`) + `dir/ships/<slug>.json` (one `SaveShip`
per vehicle; slug is `"v" + index` in canonical fleet order, `save.cpp:44`).
All (de)serialization is **inline in `save.h`** so it is unit-testable with no
Game and no Bullet; the Game-coupled capture/restore lives in `save.cpp`.

Reads are permissive by convention — `if(j.contains(k) && <type check>)`,
absent key keeps the struct default (`save.h:409-470`). **New keys need no
migration and no format bump.** This is the same contract `flog` used
(`save.h:359` writes only when started; `save.h:415-416` reads before the
`is_crew` split).

`SaveMeta` keys today (`saveMetaToJson`, save.h:471-483):
`format, saved_at, system, parts, time, time_accel, active_ship,
exhaust_scale, ships[]`.

### 1.3 A kerbal is a one-part `Vehicle`, and crew state has a save precedent

`struct Kerbal : Vehicle` — `src/eva.h:36`, built from `res/ships/kerbal.json`
(one part, `"kerbal"`, `res/data/parts.json:959-976`). Virtuals
`isEva()/isCrewAboard()/capsulePart()` at eva.h:52-54.

Kerbals are saved as `SaveShip` records with `is_crew` set (`save.h:134`), and
the crew-only fields are already there:

| field | decl | capture | restore |
|---|---|---|---|
| `flog` | save.h:143 | save.cpp:186 | save.cpp:414-417 (ships), 506-508 (crew) — **before** `setSoi` (421 / 515 / 595), which observes |
| `aboard` / `aboard_part` | save.h:168 / 176 | — | save.cpp:594-596 |
| `suit_fuel` | save.h:181 | save.cpp:205-208 | save.cpp:486-492 |
| `suit_inventory` | save.h:187 | save.cpp:209-215 | save.cpp:493-505 |

`suit_fuel` and `suit_inventory` are exactly the shape of "per-kerbal state
that is not part geometry". **`experiments` is the fifth row of that table.**

Ownership and the containment edge: `Vehicle::crew` (`src/vehicle.h:111`,
`std::vector<Vehicle*>`) is the sole owner; `~Vehicle` deletes crew
(vehicle.cpp:1506-1513). Aboard, `Kerbal::aboardPart` (eva.h:67) points at the
capsule `Part*` and the capsule's `Part::contents` holds `k->parts[0]`
(`src/part.h:117-135`); the two are kept in lockstep at spawn
(`ships.cpp:275-278`), board (`game.cpp:908`) and load (`save.cpp:594-596`).

Ready-made accessors: `shipCrew(Vehicle*)` → `std::vector<Kerbal*>`
(game.cpp:783-787), `partCrew(Part*)` (game.cpp:789-802),
`freeKerbals(System&)` (game.cpp:804+). The `static_cast<Kerbal*>` downcast of
a `crew` entry is the established idiom (game.cpp:785, 799, 808).

Naming: `k->name = dedupName(sys, "kerbal")` (`ships.cpp:240`), which appends
`" #2"`, `" #3"`… against the live fleet (`ships.cpp:208-221`), and round-trips
verbatim (`save.cpp:463`). **There is no stable per-kerbal id** — `Part` has a
process-wide `uid` (`part.h:72`), `Vehicle`/`Kerbal` do not. So an
"already recovered" set can never be keyed by kerbal; it must be keyed by the
*subject* (§2.3).

**What survives a topology change** (verified, and it is the load-bearing fact
behind §2.4): `absorbShip` *moves* the crew vector
(`vehicle.cpp:2229-2236`, `crew.push_back(B->crew[i])` then `B->crew.clear()`);
`extractSubtreeAsShip` *partitions* it by capsule ownership
(`vehicle.cpp:2399-2415`) — a move, never a copy, so no duplication;
`kerbalEVA` (game.cpp:822) and `kerbalBoard` (game.cpp:908) move the same
`Kerbal*` between `ship->crew` and the body's ship list without reconstructing
it. `~Vehicle` is the only deleter.

### 1.4 `recoverActive` is the score-worthy removal path; `remove_ship` has a hole

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

Step 9's printfs are the e2e anchors. UI entry: the hub menu button at
`gameui.cpp:2568-2576`. Headless: `--recover MS` (`cli.h:100-101`, fired
`main.cpp:1190-1196`).

**Crew are freed in step 7**, so the harvest must run between step 3 and step 4
— literally where `flightSummary` is filled.

`Game::remove_ship` (declared game.h:795, implemented game.cpp:1444) is the
*other* removal path and must not award science. Its guard
(**game.cpp:1460**) is:

```cpp
if(v->isCrewAboard() || !shipCrew(v).empty()) {
```

The comment above it (game.cpp:1451-1457) says it covers "a ship that still
carries crew -- **or a crew member themselves**". **It does not.** For a free
(EVA) kerbal, `isCrewAboard()` → `aboardPart != nullptr` (eva.h:53, 67-68) is
false, and `shipCrew(v)` reads `v->crew`, which a kerbal does not have. And
free kerbals are reachable: the Ship List (`gameui.cpp:1534-1573`) and Tracking
Ship List (`gameui.cpp:3409-3450`) iterate `collectVehicles(sys)` and skip only
`v->isCrewAboard()` (`:1546`, `:3426`), so a free kerbal gets an "x" button
that calls `g.remove_ship(v)` (`:1568`, `:3448`). Filed as **issue #57**. This
matters here because §2.4 puts the experiment data on exactly that object.

Teardown: `clearFlightSummary()` (declared game.h:818, defined
game.cpp:265-268) is called from **`Game::newGame()` (game.cpp:421, call at
436)**, **`Game::unloadGame()` (game.cpp:555, call at 569)** and `load_game`
(save.cpp:765). That is *per-mission* clearing; a career science total must not
be cleared there (§6, R1).

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
`s.has_sea && alt <= 0 → Ocean`; `max_height <= 0 → Lowland` (flat-body guard);
`alt >= 0.8*max_height → Mountain`; `>= 0.5*max_height → Midlands`; else
`Lowland`. `biomeName` returns
`"ocean"/"lowlands"/"midlands"/"mountains"/"none"` — a **display** function.

**The contract at terragen.h:404-406 is explicit and constrains this design:**

> It classifies SOLID bodies only: a banded body is None, and a star has no
> biome at all -- the caller knows the body's type and should not classify it
> (a star's Surface is just noise, see issue #52).

**`biomeAt`'s `p` is a UNIT DIRECTION in the body's ROTATING frame**, not a
lat/lon pair. Lat/lon → direction is the surfmap convention
`dir = (cos lat · sin lon, sin lat, cos lat · cos lon)` (`surfmapDir`,
`src/surfmap.h:27-33`; inverse `surfmapLonLat`, surfmap.h:39-44).

`surfmap.cpp` does **not** use biomes — it uses raw `terrainHeight` for the
Ocean-checkbox sea paint (`surfmap.cpp:110-114`). Today the only consumer is
`tests/test_terrain.cpp:587-647`. **Science is the first real consumer.**

Cost: one `biomeAt` is one full-detail `terrainHeight` sample. That is
`terrainHeightFade` (terragen.h:306-313, 338-340): a continent FBM over
`min(octaves, 6)` octaves **plus** a mountain fold over octaves 2..N — roughly
13 simplex evaluations at the default `octaves = 9`. It is byte-for-byte the
same call the HUD's Alt readout already makes *every frame*
(`ship->m_parent->GetTerrainHeight(normalize(pos))`, `gameui.cpp:362`; delegate
`terrain.h:220-222`). No mesh generation.

**Readiness caveat:** classification depends on `surface.max_height`, which is
*not* authored. It defaults to `1.0f` (`terragen.h:141-144`) and is measured
numerically by the heavy phase (`TerrainBody::BuildRootGeoms`,
terrain.h:249-282, `maxh = std::max(1.0f, (hi - surface.sea_level) * 1.05f)` at
terrain.h:281), applied by `AttachRoot` (**terrain.h:312**, `ready` flag
**terrain.h:173**). Until then **every point above 0.8 m classifies as
Mountain**. See §6 R2 for why this is reachable, not theoretical.

### 1.6 There is no situation enum; here is what is derivable

Nothing in the codebase distinguishes landed / splashed / flying / orbiting.
Available primitives:

- **`Vehicle::inTerrainBand()`** — declared vehicle.h:1183 (doc comment
  1180-1182: *"The COM's osculating orbit dips into the terrain band (periapsis
  within 3 km of the surface): sitting on / skimming the ground rather than
  coasting clear of it"*), implemented vehicle.cpp:2897-2903:
  ```cpp
  Frame *inertial = frame->getNonRotFrame();
  comStateIn(inertial, p, v);
  const OrbitElements el = computeOrbitElements(p, v, inertial->body->mu);
  return el.periapsis <= inertial->body->radius + 3000.0;
  ```
  This is an **osculating-periapsis** test, not a proximity test. A hyperbolic
  trajectory has `periapsis == -1` (`orbit.h:22-45`) and therefore reads *in
  band*.
- `Vehicle::canRail()` — vehicle.h:1189, vehicle.cpp:2905-2911; its doc comment
  gives the codebase's own vocabulary: *"a FLYING ship (periapsis clear of the
  terrain band) coasts on its conic; a GROUNDED one (periapsis inside the band)
  can only freeze in its rotating surface frame. Anything else -- e.g. a
  suborbital descent -- is not rail-eligible."*
- `Game::updateProximity()` already uses `inTerrainBand()` as a coarse
  grounded/flying discriminator: `const bool grounded = a->inTerrainBand();`
  (**game.cpp:1128**), with a comment saying the two regimes just need
  "very different radii". Tolerant there; not tolerant enough for a situation
  classifier (§3.3).
- EVA ground truth is **AGL-based**: `k->grounded = BodyInContact(k->hull) ||
  alt < rest + kGroundBand` (`eva.cpp:48-53`, `kGroundBand = 0.25` m), where
  `alt = |COM| - m_parent->GetTerrainHeight(dir)`. `Kerbal::grounded` at
  eva.h:40, rewritten every tick by `evaArmCommands` (eva.cpp:35-100).
- **Splashed does not exist anywhere.** It would be derived as `biome == Ocean`
  while grounded. Oceans are just terrain below sea level.

Altitude has no single helper; the HUD idiom is `gameui.cpp:361-364`:

```cpp
const double asl = distance - ship->m_parent->radius;
const double agl = distance - ship->m_parent->GetTerrainHeight(glm::normalize(pos));
```

`ShipView` (`game.h:105-136`, filled by `updateShipView`, `render.cpp:108-224`)
carries the per-frame snapshot: `pos/vel`, `surf_pos/surf_vel` (rotating frame),
`orbit_pos/orbit_vel` (inertial), `o` (OrbitElements), `mu`,
`latitude/longitude` (render.cpp:222-223). **It is written by the render path**,
so headless runs cannot rely on it (§6, R3).

**The rotating-frame transform** (§6, R8) — `render.cpp:158-168`:

```cpp
surf_pos = pos;  surf_vel = vel;                       // already rotating
if(ship->frame->isRotFrame() == false and
   ship->frame->hasRotFrame() == true) {
    Frame *rot = ship->frame->getRotFrame();
    surf_pos = ship->frame->GetOrientRelTo(rot) * pos;  // inertial -> rotating
    ...
}
```

Atmospheres *are* modelled: `AtmosphereParams` (`src/terragen.h:88-101`) with
`enabled`, `thickness`, `sea_level_density`, `scale_height`, feeding
`rho(alt) = sea_level_density * exp(-alt / scale_height)` (`src/drag.h`).
Authored values, recomputed from `res/systems/ksp_system.json`:

| body | radius | scale_height | ×10 | rim `thickness` | drag floor altitude¹ |
|---|---|---|---|---|---|
| Kerbin | 600 km | 5 500 m | 55 km | 15 km | 191 km |
| Eve | 700 km | 7 000 m | 70 km | 25 km | 246 km |
| Shay | 550 km | 6 000 m | 60 km | 22 km | 209 km |
| Laythe | 500 km | 5 500 m | 55 km | 12 km | 191 km |
| Duna | 320 km | 4 000 m | 40 km | 8 km | 130 km |
| Jool | 6 000 km | 20 000 m | 200 km | 90 km | 705 km |

¹ where `rho` falls to `kRhoFloor = 1e-15` (`drag.h:45`, applied at
`vehicle.cpp:1617`; the comment at drag.h:35-41 gives "~190 km on a Kerbin-like
air, ~700 km in the thickest atmosphere"). **Drag has no altitude cutoff at all**
— only this density floor — so a ship at 100 km over Kerbin still generates aero
force.

Two consequences, both load-bearing for §3.1:

- `thickness` is a **rendering** parameter (the rim-shell radius above
  `radius + max_height`, consumed only by `BuildAtmosphere`, terrain.h:336-345).
  Kerbin's shell is 15 km thick against a 5.5 km scale height. Filed as
  **issue #56**.
- The `scale_height * 10` column is within ~20% of the reference game's
  atmosphere tops (exact for Jool, +5 km for Laythe, −15 to −20 km for
  Kerbin/Eve/Duna) — close enough to be a usable edge of space, and **not**
  the same number as the drag floor. §3.1 explains why the smaller one must be
  used.

### 1.7 Body, SoI and identity

`Vehicle::m_parent` (`TerrainBody*`, vehicle.h:98) and `Vehicle::frame`
(vehicle.h:99). `setSoi(Frame*, double t)` (vehicle.h:1129-1142,
vehicle.cpp:2821-2849) is the one writer of frame / m_parent / ships-list /
`flog.observe` **for a live vessel's re-home** (3ae9c43), and it re-homes aboard
crew too (vehicle.cpp:2848). Direct pre-placement writes exist at
`save.cpp:467,469` and `save.cpp:584-585` (immediately before the `setSoi` at
595), `ships.cpp:170,172` and `ships.cpp:244,246`, plus the test scaffolds.
`flog.observe` has a second caller: `game.cpp:1580` in `recoverActive`.

Bodies are identified by plain name strings (`TerrainBody::name`,
terrain.h:161; `System::find(name)`, `src/system.h:28-33`), which is what saves
reference everywhere (`SavePose.body`).

`res/systems/ksp_system.json` has 18 bodies, `"home": "Kerbin"`. Seas:
**Kerbin, Shay, Laythe** have `has_sea: true` — a **body-level** key, not inside
`surface` (parsed at `system.cpp:76`); the other 15 do not. `Jool` is the only
`bands: true` body → `Biome::None`. `Kerbol` is the only `type: "star"`.

**The rotating frame is large.** `rf->soi` comes from the authored
`rotating.soi` (`system.cpp:275`) or defaults to `radius + 100e3`
(`system.cpp:295`). Authored in `ksp_system.json`: Kerbin 700 km (radius
600 km), Eve 800 km, Jool 6 100 km, Mun 300 km, Minmus 160 km, Gilly 113 km.
`soiTarget` (vehicle.cpp:2799-2818) moves a ship *into* the rotating frame
whenever `dist < rf->soi - kSoiMargin` (10 km, vehicle.cpp:2797). **So being in
the rotating frame says nothing about being on the ground** — this is what broke
the Rev 2 predicate (§3.3).

**`TerrainBody` does not store its parsed `type`.** `system.cpp:191-200` reads
`"star" | "planet" | "moon"` (`system.h:57`) and uses it only to pick `shader`
and `colour_func`; the string is then discarded. So terragen.h:404-406's "the
caller knows the body's type" is not directly satisfiable. What *is* available:
`System::root` — documented as *"the star (frame-tree root)"*
(**system.h:23**) — and `Game::sun` (game.h:230). **The star test is
`body == sys.root`.** See §7 item 6.

### 1.8 Part window, Flight Summary, and the window recipe

`void drawPartWindows(Game &g)` — `src/gameui.cpp:1742-2001` (decl
gameui.h:24). One plain `ImGui::Begin` per `g.part_sels` entry (`PartSel`,
**game.h:90-97**), opened by RMB → `pickAt` (game.h:846) →
`Game::openPartWindow` (game.cpp:244). Sections: stats (1794-1840), docking
(1841-1877), **crew (1879-1917)**, **inventory (1918-1990)**.

**The crew section is gated on the capsule def, not on a suit:**

```cpp
// gameui.cpp:1879  --- crew (this part is a capsule: holds EVA characters) ---
if(def->crew_capacity > 0) {                                   // gameui.cpp:1884
    std::vector<Kerbal *> aboard = partCrew(ship->parts[part]); // gameui.cpp:1886
```

The `kerbal` part (`res/data/parts.json:959-976`) has **no** `crew_capacity` —
only `"inventory_capacity": 3`. So a kerbal's own suit window renders the stats
block and the inventory block (from gameui.cpp:1923) and **never enters
1879-1917**. Phase 3.2 needs a *new* block for the suit (§4).

The button idiom, verbatim from `gameui.cpp:1939-1945`:

```cpp
ImGui::PushID(item);
ImGui::Text(" %s (%.1f kg)", in,
            item->effectiveMass());
if(ImGui::SmallButton("Drop")) {
    g.dropItem(item);
}
ImGui::PopID();
```

Flight Summary — payload `Game::FlightSummary {shipName, log, end_t}`
(game.h:538-543), drawn by `drawFlightSummary(Game&)` (gameui.cpp:2720-2771)
via `drawWin(g, W_FlightSummary, …)`, declared gameui.h:53, called from
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

`tests/test_flightlog.cpp` (112 lines) is the template for a header-only test:
include the header, local `CHECK` macro (:14-19), brace-scoped sections, print
`test_flightlog: all checks passed` / `N FAILURE(S)`, return 0/1. Build rule
links nothing (Makefile:515-516). Adding a test = a target near :515, the name
in `TESTS = …` (:646-652), a run line in the `test` target (:666-703);
short-name aliases come free from `.PHONY: $(TESTS)` (:660-663).

`tests/test_save.cpp` (683 lines) round-trips the header-only layer with its own
`CHECK` (:29-34): flog (:166-255, including no-key-when-fresh and
malformed-flog), crew SaveShip (:257), `suit_fuel` (:275), `suit_inventory`
(:365), SaveMeta (:444), permissive/wrong-type reads (:469), uid strictness
(:512-540). Registered Makefile:498-499.

e2e: cases are `e2e/cases/*.txt`, auto-discovered by
`sorted(glob(CASES_DIR/*.txt))` (`e2e/run.py:941`) — **adding one is dropping a
file in**, and `select_cases` matches on the `NAME` field or the file stem.
Format (run.py:41-58): `NAME`, `ARGS`, `EXPECT`, `FORBID`, `CHECK <python
expr>`, `LIMIT`, `WRITE`. Model case `e2e/cases/101-recover-ship.txt`, in full:

```
NAME recover-ship
ARGS --time-accel 1 --timeout 8 --startship racer,res/ships/racer.json,Kerbin,rot-orbit
ARGS --space-center 1000 --recover 2000
EXPECT [scene] flight -> spacecenter (push)
EXPECT [recover] t=
EXPECT 'racer' recovered
EXPECT [spacecenter] entered
EXPECT Timeout reached (8.
FORBID GL_
FORBID error:
```

The two `FORBID` lines are what make it a crash assertion; copy them.
**Numbering: `102-recover-esc-multi.txt` and `103-flight-events.txt` already
exist, so the science case is `104-…`.**

Save e2e precedent — note these are two *different* mechanisms:
`53-save.txt` uses `--save tmp/e2e_save`, which fires **only inside the
`--timeout` branch** just before the loop exits (`main.cpp:1097-1121`);
`54-save-load.txt` loads a **committed fixture**, `--load
e2e/fixtures/save_racer`. The mid-run `--reload DIR --reload-at MS` hook
(`cli.h:87-89`) is used by `64-reload.txt`. **A single run cannot
`--save` then `--reload` the same directory** — the save does not exist yet when
the reload fires (§4, phase 3.3). Run: `make e2e` (Makefile:716-718) or
`python3 e2e/run.py recover`.

---

## 2. Evaluation of the proposal

### 2.1 What is right

- **Data on kerbals, converted on recovery.** Correct, and it is the only option
  that survives the codebase's topology: kerbals have no persistent id (§1.3),
  and dock-absorb deletes a ship's identity (issue #49 already notes `flog` is
  lost there). Verified in §1.3: a `Kerbal` object survives boarding, EVA,
  absorb and split unchanged, and is never copied.
- **Unlimited capacity, counted once.** Bounded by construction (§2.7), so
  neither saves nor memory can grow without limit.
- **Score-only for now.** Right call. A spendable currency needs something to
  spend it on; inventing a tech tree in the same change would bury the feature.
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
that "combine freely" (shipdef.h:230-234, verbatim) — so it is idiomatic here
rather than an imported abstraction.

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

1. **Display text must not be the key.** Wording changes would invalidate every
   save. `biomeName()` (terragen.h:440-448) is a *display* function; a separate
   `biomeToken()` returns the stable id token.
2. **Value must not be cached.** Deriving it at recovery means a balance change
   applies retroactively and there is nothing to go stale.
3. **It makes the whole model header-only pure C++**, testable without linking
   `Vehicle` — the `flightlog.h` doctrine, which puts it as: *"Pure containers +
   logic -- no game types -- so tests can pin the enter/leave pairing without
   linking Vehicle"* (flightlog.h:20-21).

A side effect worth having: `recovered_subjects` is a set of ids drawn from a
finite space, so it is bounded (§2.7). An event-log design would not be.

### 2.4 CORRECTION (Rev 2) — the data belongs on `Kerbal`, not on `Part`

Rev 1 recommended `Part::experiments`, on the grounds that Parts are the
persistent, uid-stable objects that survive containment transitions, and that it
would directly serve the "later other parts may hold experiment data"
requirement. **That was wrong for v1.** The fact-check found:

- Per-kerbal state already has a home and a save path: `suit_fuel` and
  `suit_inventory` are `SaveShip` crew fields (save.h:181-187) captured at
  save.cpp:205-215 and restored at save.cpp:486-505. `experiments` is the fifth
  row of that table and needs no new plumbing shape.
- `SavePart` (save.h:85-102) carries geometry, stage, mass, hull margin, fuel
  and nested inventory — all part-generic. An `experiments` vector there would
  be dead weight on every part of every ship for a field only kerbals use.
- §1.3's verification settles it: a `Kerbal` object already survives boarding,
  EVA, `absorbShip` and `extractSubtreeAsShip` as the *same object*, moved and
  never copied. Nothing is gained by moving the data down to the Part.

So: `std::vector<std::string> experiments;` on `Kerbal` (`src/eva.h:36`), next
to `aboardPart` (eva.h:67). When a science-module *part* eventually needs
storage, that is the moment to add `Part::experiments` + `SavePart::experiments`,
and the subject id format is already the shared currency — so deferring costs
nothing.

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
`space_low` resolves the biome under the vessel. Without this the two situations
differ only in a label and there is no reason to prefer either.

Note where the rule lives: it is a property of the **situation**, not of the
experiment, so it belongs in the subject builder (`if situation == space_high →
biome = None`) and every future experiment inherits it for free. No flag on
`ExperimentDef`.

Also note what the biome *is*: `biomeAt` samples `terrainHeight` at the
sub-vessel point, so the biome is that of **the ground below**, independent of
the vessel's own altitude (§1.5). That is exactly right for "observation of
<biome>" and needs no extra work.

### 2.7 Dedup makes the score bounded — here is the actual ceiling

`Kerbol` must be excluded: terragen.h:404-406 says a star has no biome and the
caller must not classify it (§1.5), and Kerbol has `bands: false`, so
`biomeFromAltitude` would happily return Lowland/Midlands/Mountain off pure
noise. The test is `body == sys.root` (§1.7). That leaves 17 bodies:

| body class | count | low-orbit subjects | high-orbit | each |
|---|---|---|---|---|
| sea + terrain (Kerbin, Shay, Laythe) | 3 | 4 biomes | 1 | 5 |
| solid, no sea | 13 | 3 biomes | 1 | 4 |
| banded (Jool) | 1 | 0 biomes | 1 | 2 |

**3×5 + 13×4 + 1×2 = 69 subjects × 10.0 = 690 science** is the v1 ceiling for
the shipped system. Finite, completionist-visible, and `recovered_subjects` can
never exceed 69 short strings however long a career runs. It also means the
score is a *map* — the only way to earn more is to go somewhere new, which is
the point.

Note the per-body denominator is **5 / 4 / 2, not a single number** — the
Science window's progress display must derive it (§3.1's `subjectsForBody`),
not hardcode `/5`.

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
— it is the only feedback loop v1 has. **But see §6 R9**: computing it means one
Kepler solve plus one FBM sample *per frame the window is open*, so it needs
caching.

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
    { "observation", "Orbital Observation",
      situMask(Situation::SpaceLow) | situMask(Situation::SpaceHigh), 10.0 },
};

// CLICK-TIME ONLY -- `body` is a view into TerrainBody::name and `experiment`
// a view into kExperiments. Never stored; encode to std::string immediately.
struct Subject {
    std::string_view experiment;
    std::string_view body;
    Situation situation = Situation::SpaceLow;
    Biome biome = Biome::None;      // terragen.h:409
};
std::string encodeSubject(const Subject&);              // 4 '@' fields, last may be empty
std::optional<Subject> decodeSubject(std::string_view); // nullopt on malformed
std::string subjectName(const Subject&);                // display text
double subjectValue(const Subject&);                    // findExperiment()->base_value

// --- the altitude bands -------------------------------------------------
inline constexpr double kLowOrbitRadiusFrac = 0.25;    // ceiling = radius * this …
inline constexpr double kLowOrbitFloor      = 25000.0; // … but never below this (m)
inline constexpr double kAtmoScaleHeights   = 10.0;    // edge of space = scale_height * this

// Pure classifier: is an orbit observation possible, and which band?
//   in_terrain_band  Vehicle::inTerrainBand() -- periapsis within 3 km of the
//                    surface, i.e. landed / splashed / suborbital / skimming
//   alt_asl          metres above the body's base radius
//   space_floor      scale_height * kAtmoScaleHeights (0 for an airless body)
//   low_ceiling      max(radius * kLowOrbitRadiusFrac, kLowOrbitFloor)
// nullopt = no v1 experiment applies here.
std::optional<Situation> situationFor(bool in_terrain_band, double alt_asl,
                                      double space_floor, double low_ceiling);

// The finite subject space, for the Science window's per-body progress and
// for asserting the career ceiling. 5 / 4 / 2 by body class (§2.7).
int subjectsForBody(bool is_star, bool banded, bool has_sea);
```

**Why `space_floor` is `scale_height * 10` and not the drag floor.** §1.6's
`kRhoFloor` altitudes (Kerbin 191 km) look like the more physical choice, but
they would break the feature: Kerbin's `low_ceiling` is `600 km × 0.25 =
150 km`, so a 191 km floor sits *above* the ceiling and **`space_low` becomes
unreachable on Kerbin** — deleting most of the subject space. `scale_height ×
10` (55 km) sits below the ceiling with room for a 55-150 km band, and it is
also within ~20% of the reference game's atmosphere tops. The consequence is
deliberate and worth stating: between 55 km and 191 km over Kerbin a vessel
counts as "in space" for science while still generating drag. That matches the
reference game's hard atmosphere top.

The two low-orbit constants are needed because body radii span 4.3 orders of
magnitude (Gilly 13 km → Kerbol 261 600 km). A bare radius fraction gives Gilly
a 3.25 km ceiling inside its authored 126.12 km SoI, which is unflyable; the
25 km floor fixes it:

| body | radius | `max(r×0.25, 25 km)` | space floor |
|---|---|---|---|
| Gilly | 13 km | 25 km | 0 (airless) |
| Minmus | 60 km | 25 km | 0 |
| Mun | 200 km | 50 km | 0 |
| Kerbin | 600 km | 150 km | 55 km |
| Jool | 6 000 km | 1 500 km | 200 km |

Both are tunables in one place; if the numbers ever feel wrong the alternative
is a per-body `low_orbit_alt` in the system JSON, which needs no back-compat
(QWEN.md).

Display strings (v1):

- `space_low` + biome → `"Low orbit observation of the Mountains on Kerbin"`
- `space_high` → `"High orbit observation of Kerbin"`

### 3.2 Game-side glue (thin; all of it in existing files)

```
Kerbal::experiments            src/eva.h:36        std::vector<std::string>, subject ids
SaveShip::experiments          src/save.h:187      next to suit_inventory
Game::science                  src/game.h:538      double, next to flightSummary
Game::recovered_subjects       src/game.h          std::unordered_set<std::string>
Game::FlightSummary::science   src/game.h:538-543  per-mission award
SaveMeta::science / ::recovered src/save.h:192
```

Two new functions carry all the logic:

- `std::optional<Subject> scienceSubjectFor(Vehicle &v, const System &sys)` —
  the **sampling** function, run at click time. Reads `v.m_parent` (name,
  `radius`, `params().surface.atmosphere`, `bands`, `has_sea`), refuses
  `m_parent == sys.root` (a star, §1.7/§2.7) and `m_parent == nullptr`,
  computes the rotating-frame direction (§3.4), altitude ASL, and
  `v.inTerrainBand()`. `nullopt` = no experiment possible here.
- `double Game::harvestScience(Vehicle &v)` — the **conversion** function, run
  from `recoverActive` between game.cpp:1583 and 1587. Walks `shipCrew(v)`, and
  for each subject id not already in `recovered_subjects`, adds `subjectValue`
  and inserts. Returns the mission total for `flightSummary.science`.

Sampling at click time (not at recovery) is what makes eccentric orbits
unambiguous: the situation is a fact about the instant the observation was made.

### 3.3 CORRECTION (Rev 3) — the ground test

Rev 2 proposed `grounded = inTerrainBand() && frame->isRotFrame()`, assembled
from `canRail()`'s internals. **That is wrong in the common case, not the exotic
one.** Both terms are true for the entire ascent to orbit:

- `inTerrainBand()` tests *osculating periapsis*, not proximity
  (vehicle.cpp:2897-2903). Periapsis stays inside the body until the
  circularisation burn, so it is true from the pad all the way up.
- The rotating frame is **large**: `rf->soi` is the authored `rotating.soi` or
  `radius + 100e3` (system.cpp:275, 295). Kerbin's is 700 km, so a ship is in
  the rotating frame up to ~90 km ASL; `soiTarget` moves it there
  (vehicle.cpp:2799-2818).

Result: on Kerbin the predicate reads *grounded* for 0-90 km ASL, so the button
stays disabled through the whole climb and only lights up after circularisation.
Worse, a **circular orbit below 3 km over an airless body** — a real, flyable
Mun/Minmus/Gilly orbit, inside their 100 km-deep rotating frames — also reads
grounded. And on the 12 airless bodies `space_floor == 0`, so the ground test is
the *only* thing separating "landed" from "low orbit". The failure direction is
the opposite of what Rev 2's R4 assumed: it *withholds* experiments from
airborne ships rather than offering them to landed ones.

**The fix is simpler than what it replaces.** `inTerrainBand()` used *alone and
negated* is already the right question — its own doc comment (vehicle.h:1180-1182)
describes it as *"sitting on / skimming the ground rather than coasting clear of
it"*. Rev 2's error was conjoining it with a frame test and calling the result
"grounded". So there is no `grounded` parameter at all:

```
if (v.inTerrainBand())                  -> nullopt   // landed, splashed, suborbital, skimming
if (alt_asl < space_floor(body))        -> nullopt   // still inside the atmosphere
return alt_asl < low_ceiling(body) ? SpaceLow : SpaceHigh
```

The atmosphere check is what handles atmospheric bodies (at 0 km ASL,
`alt_asl < 55 km` refuses); `inTerrainBand()` is what handles the 12 airless
ones. For an EVA kerbal the same two tests apply — no need for
`Kerbal::grounded` (eva.h:40), since a kerbal standing on the Mun has its
periapsis deep inside the body. `situationFor` therefore takes
`(bool in_terrain_band, double alt_asl, double space_floor, double low_ceiling)`
and stays pure.

**Known v1 limitation, deliberate:** a *hyperbolic* trajectory has
`periapsis == -1` (`orbit.h:22-45`), so `inTerrainBand()` returns true and a
flyby on an escape trajectory is refused. That is defensible for an experiment
literally named "orbit observation", and it is a one-line relaxation later
(treat `periapsis < 0` as clear of the band). Record it rather than special-case
it now.

### 3.4 The rotating-frame direction (Rev 3 addition)

`biomeAt` needs a unit direction in the body's **rotating** frame
(terragen.h:431-437). The obvious call — `Vehicle::frameS()` (vehicle.h:204,
vehicle.cpp:509) — returns the position in `ship->frame`, which above Kerbin's
~90 km rotating-frame boundary is the **inertial** node. Feeding that to
`biomeAt` computes the biome at the wrong longitude, off by the accumulated
spin. This matters precisely in the `space_low` band (55-150 km over Kerbin, of
which 90-150 km is inertial-frame).

The correct transform is the one `render.cpp:158-168` uses for `surf_pos`:

```cpp
glm::dvec3 dir = pos;                                    // already rotating
if(!ship->frame->isRotFrame() && ship->frame->hasRotFrame()) {
    Frame *rot = ship->frame->getRotFrame();             // frame.h:90
    dir = ship->frame->GetOrientRelTo(rot) * pos;
}
Biome b = biomeAt(glm::normalize((glm::vec3)dir), v.m_parent->params());
```

R3 (§6) still applies: derive this from the `Vehicle`, not from `g.view`, and
verify the two agree interactively.

---

## 4. Work plan

Steps are deliberately small: each is independently **committable**, and none
depends on a later one to *build*. Verification is a different story — phase 2's
behaviour is only *observable* once phase 3 can create an experiment (noted
inline, and it is the one place the plan is not as clean as it looks). Per
project rules, run `make test` and the relevant e2e cases before each commit.

### Phase 0 — the pure model (no behavior change)

**0.1 — `src/science.h` + `tests/test_science.cpp`.**
Everything in §3.1. No game types; includes `<optional> <string> <string_view>
<vector>` plus `terragen.h` for `Biome`. Note that `terragen.h` pulls in
`<glm/glm.hpp>` and `<glm/gtc/noise.hpp>` (terragen.h:71-72) and is ~42 KB of
inline simplex/FBM, so `test_science` will not compile as fast as
`test_flightlog` — glm is header-only, so the "links nothing" rule
(Makefile:515-516) still holds. Accept the compile time knowingly; do not
duplicate a local `Biome` enum to avoid it.
*Files:* `src/science.h`, `tests/test_science.cpp`, `Makefile` (target near
:515, name in `TESTS` :646-652, run line :666-703).
*Verify:* `make test_science` then `make test`. Cases: encode/decode round-trip
for every situation × biome including the empty-biome form; malformed ids →
`nullopt` (wrong field count, unknown experiment, unknown situation, bad biome);
`situationFor` across §3.1's threshold table **and** §3.3's refusal rules (in
terrain band → nullopt; below space floor → nullopt; either side of the ceiling;
airless body with `space_floor == 0`; the `space_floor > low_ceiling`
degenerate case asserting `space_low` is empty, which is the bug §3.1 warns
about); `subjectsForBody` returning 5/4/2/0 and summing to 69 for the shipped
body classes; `subjectValue` for a known and an unknown experiment.

**0.2 — nothing else.** Resist wiring it up in the same commit; the point of
phase 0 is that `make test` proves the model before any game code touches it.

### Phase 1 — data on the kerbal, persisted

**1.1 — `Kerbal::experiments` + save/load.**
Field on `Kerbal` (eva.h:36, next to `aboardPart` at :67),
`SaveShip::experiments` (save.h, next to `suit_inventory` :187) with inline
`saveShipToJson`/`FromJson` entries following the permissive pattern
(save.h:352/409), capture in `saveShipFromVehicle` next to save.cpp:209-215,
restore in `buildKerbalFromSave` next to save.cpp:493-505. Write the key only
when non-empty, exactly like `flog` (save.h:359).
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
**Reset both in `Game::newGame()` (game.cpp:421, alongside the
`clearFlightSummary()` at :436) and in `Game::unloadGame()` (game.cpp:555,
alongside :569).** Do *not* put them in `clearFlightSummary()`
(game.cpp:265-268) — that is per-mission, and it is also called by `load_game`
(save.cpp:765), which would wipe a loaded score. See §6 R1.
*Files:* `src/game.h`, `src/game.cpp`, `src/save.h`, `src/save.cpp`.
*Verify:* `tests/test_save.cpp` SaveMeta round-trip with a non-empty recovered
list (extend :444); `make test`.

**2.2 — `Game::harvestScience` + the Flight Summary line.**
Called from `recoverActive` between game.cpp:1583 and 1587 (before
`dropPartWindowsFor`, well before the `delete v` at 1616 that frees the crew).
Adds `double science` to `FlightSummary` (game.h:538-543), zeroed by
`clearFlightSummary()`. One line in `drawFlightSummary`
(gameui.cpp:2720-2771). Add a `[science]` printf inside the existing block at
game.cpp:1624-1638:
`[science] vessel='X' earned=30.0 total=120.0 subjects=12`.
**Only `recoverActive`.** `remove_ship` (game.cpp:1444) must not award — and
note it currently *deletes* free kerbals and their data (issue #57); until that
is fixed, a kerbal left on EVA is a way to silently destroy unrecovered
experiments.
*Verification gap, stated plainly:* global dedup across two recoveries, and
"remove_ship does not award", are **not observable** until phase 3 can create an
experiment. Phase 2 is committable and `make test`-clean but not e2e-provable on
its own. Two ways to close it: accept the debt and prove both in 3.3, or pull a
tiny `--grant-subject ID` debug hook into phase 2 so it can be e2e'd standalone.
Recommend the former — the hook is test scaffolding that would outlive its use.
*Files:* `src/game.h`, `src/game.cpp`, `src/gameui.cpp`.

### Phase 3 — running experiments (the interaction)

**3.1 — `scienceSubjectFor(Vehicle&, const System&)` + confirm §3.3/§3.4.**
Implement §3.3's two-test rule and §3.4's rotating-frame transform. **Confirm
with debug prints in five states before wiring any button** (the project's
stated approach to uncertainty): on the pad, mid-ascent below 90 km over
Kerbin, a 100 km Kerbin circular orbit, an EVA kerbal standing on the surface,
and an EVA kerbal in free fall. Assert the mid-ascent case reads `nullopt` —
that is the exact state Rev 2's predicate got wrong.
*Files:* `src/game.cpp` (or a small `src/science_game.cpp` if it grows).
*Verify:* the five debug prints; `make test`.

**3.2 — `Game::runExperiment(...)` and the part-window buttons.**
Two **separate** blocks in `drawPartWindows`, because the existing crew section
is capsule-gated (§1.8):
- a **new** block for a kerbal's own suit — gate on `ship->isEva()` (or
  `def->type == "kerbal"`), placed near the inventory block (gameui.cpp:1918+);
- a button inside the **existing** `if(def->crew_capacity > 0)` block
  (gameui.cpp:1884-1917), using `partCrew(part)[0]` (§2.5) and naming that
  kerbal on the button.

Both disabled with a reason when `scienceSubjectFor` returns `nullopt`, and the
capsule one also when `partCrew(part)` is empty. Dedup on insert into
`Kerbal::experiments` (a kerbal holding the same subject twice is
indistinguishable from holding it once in v1, and it keeps saves small).
*Files:* `src/gameui.cpp`, `src/game.h`, `src/game.cpp`.
*Verify:* manual — fly `racer` to a Kerbin orbit, run one, confirm the held id.
Also confirm the button is **disabled during ascent** (the §3.3 regression).

**3.3 — headless hook + the e2e case.**
Add `--run-experiment MS` following `--recover MS` exactly (cli.h:100-101, fired
main.cpp:1190-1196): it runs the experiment on the first crew of the active ship
and prints `[experiment] subject='observation@Kerbin@space_low@…'`.

New case **`e2e/cases/104-science-orbit.txt`** (102 and 103 are taken, §1.9) on
the `101-recover-ship.txt` skeleton, including its `FORBID GL_` /
`FORBID error:` lines: start in a Kerbin orbit, `--run-experiment 1500`,
`--recover 2000`, EXPECT `[experiment] subject=` and
`[science] vessel=… earned=10.0 total=10.0`. Then: run twice before recovering
and assert `earned=10.0` still (dedup on insert); a second flight recovering the
same subject asserting `earned=0.0` (global dedup); and an ascent case asserting
no `[experiment]` line before circularisation (the §3.3 regression, and the one
worth having most).

**The save/load round-trip (phase 1.2's debt) cannot be one run.** `--save`
fires only inside the `--timeout` branch (main.cpp:1097-1121) while `--reload`
fires mid-run (cli.h:87-89), so `--save X --reload X` would reload a directory
that does not exist yet. Three options, in order of preference:
1. **Two chained cases** — `104-science-orbit.txt` saves at timeout, a second
   case loads that directory and asserts the total. Cheap, no new code, but it
   couples two case files and depends on run order (`run.py:941` sorts, so
   naming controls it).
2. **A committed fixture** carrying science (the `54-save-load.txt` /
   `e2e/fixtures/save_racer` pattern) — order-independent, but the fixture must
   be regenerated whenever the subject format changes.
3. **Issue #51's proposed `--resave`** (load, then save on exit) — the clean
   answer, and it is already an open issue. Doing it here would close #51 as a
   side effect.

Recommend 3 if #51 is cheap, else 1.
*Files:* `src/cli.h`, `src/cli.cpp`, `src/main.cpp`,
`e2e/cases/104-science-orbit.txt` (+ possibly `e2e/fixtures/`).
*Verify:* `python3 e2e/run.py science` then `make e2e`.

### Phase 4 — the Science window + the NEW hint

**4.1 — `W_Science`.** Full recipe at §1.8: enum (uiwins.h:63-77), `kWins[]`
entry modelled on uiwins.cpp:236-246, scene set (uiwins.cpp:310-312),
`drawScience(Game&)` in gameui.cpp using `drawWin`, declared gameui.h, called
from `spaceCenterDrawUi` (scene.cpp:101-106) and optionally `flightDrawUi`
(scene.cpp:33-38). Content: career total, then recovered subjects grouped by
body, decoded through `decodeSubject` + `subjectName`, with a **per-body `n /
subjectsForBody(…)`** count (§2.7 — 5/4/2, *not* a hardcoded `/5`) so the 690
ceiling is visible. A subject whose body no longer exists in the loaded system
decodes but does not resolve: show the raw token and count it as 0 (§6 R7).
*Verify:* manual + a smoke e2e if one exists for other windows.

**4.2 — the `[NEW]` / `already recovered` hint** on the phase 3.2 buttons
(§2.9), with the caching §6 R9 requires. Pure lookup in `recovered_subjects`;
no state change.
*Verify:* manual — run one, recover, run the same one again, confirm the label
flips.

**4.3 — optional key binding** for "run experiment on the selected part"
(§1.8's `DebugInfo` pattern, events.cpp:294-298). Deferred: it needs a
selection, which headless does not have, so it does not help e2e. Nice for play.

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
- hyperbolic flybys (§3.3's deliberate limitation)
- `res/data/science.json` (§8: the table stays in `science.h` for v1)

---

## 6. Risks

**R1 — Career state has no home and no teardown discipline.** Science is the
first global gameplay field (§1.1). `clearFlightSummary()` runs from `newGame`
(game.cpp:436), `unloadGame` (:569) *and* `load_game` (save.cpp:765); clearing
science there wipes a loaded score, while *not* resetting it in `newGame` leaks
the previous career into a new one. Both are one-line bugs no existing test
would catch. Mitigation: reset in `newGame`/`unloadGame`, restore in
`load_game`, and add a `test_save` case.

**R2 — Biome is wrong on a body whose heavy phase has not landed, and this is
reachable.** Before `AttachRoot` (terrain.h:312), `max_height` is 1.0 and
everything above 0.8 m classifies as Mountain (§1.5). Rev 2 called this
unreachable because "the active ship's body is always synchronously built".
**It is not:** `postHeavyPhase` caps the sync set at the star plus *two* other
bodies (`main.cpp:139-145`, `if(!dup && isSync.size() < 3)`; the comment at
main.cpp:612 says so), so a fleet spanning three or more bodies leaves at least
one deferred. Nothing gates `GetTerrainHeight`, `params()` or `biomeAt` on
`ready` — only `Draw`/`Update`/`CountPatches` are gated (terrain.h:674, 714,
743). A ship whose `setSoi` re-homes it to a still-queued body would record
`mountain` for every observation. Contrary to Rev 2's advice, **do add the
guard**: `if(!body->ready) return nullopt;` is one line and is cheaper than the
comment explaining why it cannot happen. Filed as **issue #54**.

**R3 — `g.view` is render-side.** `ShipView` is filled by `updateShipView`
(render.cpp:108-224). Phase 3's e2e runs headless through `--run-experiment`,
so `scienceSubjectFor` must derive everything from the `Vehicle`, the body and
the `System`. If it quietly reads `g.view.surf_pos`, the headless path produces a
garbage biome and the e2e passes while the game is wrong — or vice versa. Verify
both paths agree interactively.

**R4 — (Rev 3, rewritten) The situation classifier is the load-bearing
assumption.** Rev 2's risk was "a landed ship offers an orbit observation". The
real failure mode, found by the fact-check, was the opposite: an *airborne* ship
told it is landed, withholding the experiment for the whole ascent (§3.3). The
replacement rule is built from `inTerrainBand()` alone, whose doc comment
(vehicle.h:1180-1182) states the intended meaning — but it still needs the
five-state debug confirmation in phase 3.1 before any button is wired, because
on the 12 airless bodies it is the *only* discriminator between "landed" and
"low orbit".

**R5 — Two removal paths, and one has a hole.** Science converts only in
`recoverActive` (game.cpp:1564), never in `remove_ship` (game.cpp:1444). Rev 2
said `remove_ship` "refuses crewed ships, so it cannot silently eat data in v1"
— **wrong for free kerbals**: the guard at game.cpp:1460 does not fire for a
kerbal on EVA, and the Ship List exposes an "x" button for one
(gameui.cpp:1568, 3448). Since §2.4 puts the data on the `Kerbal`, that button
is a way to destroy unrecovered experiments for zero score. **Issue #57**; fix it
before or alongside phase 2.

**R6 — Dock-absorb drops ship-identity state.** Issue #49 records that `flog` is
lost when a ship is absorbed. Experiments on `Kerbal` survive (§1.3 verified
`absorbShip` moves the crew vector), which is a further argument for §2.4 — but
the absorbed *ship's* journal vanishes, so a mission that docked and recovered as
one vessel reports only the survivor's history in the Flight Summary. Commented
on #49.

**R7 — Unknown bodies in old saves.** `decodeSubject` yields a body name string;
if the system JSON changes, that body may no longer exist. v1 should not resolve
it (value 0, name falls back to the raw token) rather than throw — the save layer
is permissive everywhere else (§1.2).

**R8 — The rotating-frame transform is easy to get wrong.** `frameS()`
(vehicle.h:204) is *not* a rotating-frame position above ~90 km over Kerbin;
`biomeAt` fed from it computes the biome at the wrong longitude (§3.4). The bug
is silent and longitude-dependent — it would look like "the biome is sometimes
wrong", which is miserable to diagnose. Use `render.cpp:158-168`'s transform
verbatim and assert agreement with `g.view.surf_pos` during phase 3.1.

**R9 — The NEW hint costs a Kepler solve and an FBM sample per frame.** §2.9
wants the prospective subject displayed while the part window is open, which
means calling `scienceSubjectFor` every frame: `inTerrainBand()` runs
`comStateIn` + `computeOrbitElements` (a full Kepler solve — issue #33 already
flags these as expensive per tick) and `biomeAt` runs ~13 simplex evaluations.
The HUD already pays the FBM cost per frame (gameui.cpp:362), so that half is
free in practice; the Kepler solve is the new one. Cache the computed subject on
the `PartSel` and refresh it at ~1 Hz or on a frame/SoI change. Do not recompute
it in the draw loop unconditionally — QWEN.md treats performance as a
first-class concern.

---

## 7. Other things noticed (flagged per project convention)

1. **`sea_level` is never authored non-zero, and its presence has a side
   effect.** `Surface::sea_level` (`terragen.h:138`) is absent from
   `ksp_system.json` and `old_system.json`, and appears as `0.0` in the four
   `solar_system*.json` files (`solar_system.json:172` and its three siblings).
   That is not inert: `system.cpp:84-87` sets `s.has_sea = true` as a side
   effect of the key being present, which flips `biomeFromAltitude`'s Ocean
   branch on. So `sea_level` is *either* absent (no ocean) *or* present-and-zero
   (ocean exactly at the base radius, zero depth) — there is no way to author a
   sea level that is not the base radius. Filed as **issue #55**.
2. **Two altitude conventions.** The HUD's ASL is `distance - radius`
   (gameui.cpp:361) while `biomeAt` internally uses
   `terrainHeight - radius - sea_level` (terragen.h:435). Harmless while
   `sea_level == 0` (item 1) and a silent discrepancy the moment it is not.
   Science must pick one and say which; §3.1 uses `distance - radius`.
3. **`AtmosphereParams::thickness` is visual only** (terragen.h:91,
   terrain.h:336-345) — Kerbin's rim shell is 15 km against a 5 500 m scale
   height. Anyone looking for "where the atmosphere ends" will find it and be
   wrong by an order of magnitude. **Issue #56.**
4. **`Biome` has no consumer outside tests** (§1.5) — science is the first, so
   the `0.5` / `0.8 × max_height` thresholds have never been play-tested. Expect
   to retune them once orbital observations exist.
5. **Kerbals have no persistent identity** (§1.3). Names are `dedupName`-generated
   against the *live* fleet, so "kerbal #2" can be a different individual in a
   later career. Anything that ever needs to key on a kerbal across careers will
   need a uid on `Vehicle`, mirroring `Part::uid` (part.h:72).
6. **`TerrainBody` does not store its parsed `type`.** `system.cpp:191-200` reads
   `"star" | "planet" | "moon"` and uses it only to choose `shader` and
   `colour_func`, then discards it — while `terragen.h:404-406` instructs the
   caller that "the caller knows the body's type". No caller can, from the body
   alone. Science works around it with `body == sys.root` (system.h:23), which
   assumes exactly one star. Storing a `BodyType` on `TerrainBody` would be two
   lines and would serve any future consumer (a system map, a biome legend, an
   "observations remaining" count).
7. **`--save` and `--reload` cannot be combined in one run** (§1.9,
   main.cpp:1097-1121 vs cli.h:87-89). That constrains every future
   save-round-trip e2e, not just this one, and it is the same gap **issue #51**
   (`--resave`) proposes to close.

---

## 8. Decisions taken

| fork | decision | rationale |
|---|---|---|
| capsule crew pick | **deterministic first crew** | the button can name the kerbal; e2e is stable; random buys nothing with no roles/skills |
| biome from high orbit | **low orbit only** | otherwise the two situations differ only by label and neither is worth flying for |
| experiment defs | **hardcoded table in `src/science.h`** | one experiment in v1; a JSON loader is dead weight. Same struct shape a loader would fill, so migrating later is ~30 lines |
| score value | **flat `base_value` per experiment** | nothing to balance a multiplier table against until several experiments exist |

### Rev 3 changelog (fact-check pass against `7df2065`)

Design-breaking:

- **§3.3 rewritten.** Rev 2's `grounded = inTerrainBand() && isRotFrame()` is
  wrong: the rotating frame reaches `radius + 100 km` (Kerbin's authored
  `rotating.soi` is 700 km, system.cpp:275/295) while `inTerrainBand()` tests
  osculating periapsis (vehicle.cpp:2897-2903), so the predicate reads *grounded*
  for the entire ascent and for sub-3 km orbits over airless bodies. Replaced by
  `inTerrainBand()` alone (negated) plus the atmosphere floor — one fewer
  parameter than Rev 2, and the failure direction in R4 was inverted.
- **§2.7 corrected: 69 subjects / 690 science, not 73 / 730.** Kerbol must be
  excluded per terragen.h:404-406 ("a star has no biome at all"), and it has
  `bands: false`, so `biomeFromAltitude` would classify off pure noise.
- **§4 phase 4.1:** the per-body denominator is 5/4/2, not a hardcoded `/5`.
- **§1.4 + R5:** `remove_ship`'s guard (game.cpp:1460) does **not** protect a
  free EVA kerbal, contrary to its own comment — and the UI exposes a remove
  button for one. Issue **#57** filed.
- **§4 phase 3.2:** the part window's crew section is gated on
  `def->crew_capacity > 0` (gameui.cpp:1884), which the `kerbal` part does not
  have (parts.json:959-976). The suit button needs a **new** block; Rev 2 put it
  in a section it can never reach.
- **§6 R2:** biome-before-`AttachRoot` is **reachable**, not unreachable —
  `postHeavyPhase` syncs only the star plus two bodies (main.cpp:139-145). A
  `ready` guard is now required, reversing Rev 2's advice.
- **§3.4 added:** the rotating-frame transform. `frameS()` is not a
  rotating-frame position above ~90 km over Kerbin; `biomeAt` fed from it gets
  the longitude wrong. Uses render.cpp:158-168's transform.
- **§3.1 added:** why `space_floor` must be `scale_height * 10` (55 km on
  Kerbin) and not the `kRhoFloor` altitude (191 km) — the latter sits *above*
  Kerbin's 150 km low-orbit ceiling and would make `space_low` unreachable,
  deleting most of the subject space.
- **§4 phase 3.3:** a single-run `--save` + `--reload` is impossible
  (main.cpp:1097-1121 vs cli.h:87-89); three alternatives given. The case number
  is **104**, not 102 (`102-recover-esc-multi.txt` and `103-flight-events.txt`
  exist).
- **§6 R9 added:** the NEW hint costs a Kepler solve + FBM sample per frame;
  needs caching.
- **§3.2:** `scienceSubjectFor` returns `std::optional<Subject>`; Rev 2 declared
  a non-optional return and then said it "signals not possible".

Mechanical: `setTime` game.h:389 (was 434); the whole `save.cpp` column of
§1.3's table (capture 186/205-208/209-215, restore 414-417/486-492/493-505,
name 463 — all were 7-44 lines high); `newGame` game.cpp:421 and `unloadGame`
game.cpp:555 with their `clearFlightSummary()` calls at 436 and 569 (**Rev 2 had
the two functions swapped**); `PartSel` game.h:90-97; `ShipView` game.h:105-136;
`Game` 223-838; `inTerrainBand` vehicle.h:1183; `isRotFrame` frame.h:75;
`AttachRoot` terrain.h:312; `ready` terrain.h:173; `Part::uid` part.h:72;
`Kerbal` eva.h:36, `grounded` eva.h:40, `aboardPart` eva.h:67;
`drawPartWindows` 1742-2001; `make e2e` Makefile:716-718; `test_save.cpp` 683
lines; the `101-recover-ship.txt` quote is now complete (Rev 2 omitted the two
`FORBID` lines that make it a crash assertion); the button idiom and the
`flightlog.h` quotation are now verbatim; `54-save-load.txt` loads a committed
fixture rather than using `--reload`; §1.7's `setSoi` claim narrowed to "one
writer for a live vessel's re-home"; the `money|science|funds|score` grep needs
`\b` (one prose hit at gameui.cpp:537); `terrainHeight` is ~13 simplex
evaluations, not 9; body radii span 4.3 orders of magnitude (Gilly→Kerbol), not
2.7.

Verified unchanged: §1.2, §1.5's biome API signatures/thresholds/enum
order/`biomeName` strings/`biomeAt` rotating-frame contract/"only consumer is
tests/test_terrain.cpp"/cost claim, §1.8's structural claims, §1.9's Makefile
line numbers, §2.4's central premise (crew are moved, never copied, through
absorb/split/EVA/board), the atmosphere table's radii and scale heights, the
18-body / `home: Kerbin` / sea / bands classification, and the `recoverActive`
walkthrough including the harvest window between game.cpp:1583 and 1587.
