# main.cpp refactoring candidates (2026-08-26)

Lunch-sized refactors found in `src/main.cpp` (5201 lines at the time of
writing). All line numbers are as of this snapshot; the project moves fast,
so re-verify before editing. Items are ordered by confidence/effort.

Verification status:
- "verified" = grepped/confirmed the claim (dead code, redundancy) against
  the tree on this date.
- Everything else is a read-only observation from the code itself.

## A. Dead code (pure deletes, zero risk)

1. **`Vehicle::setPosition`** (main.cpp:1209-1214) — verified dead + landmine.
   Never called anywhere in `src/`. Its body calls `SetPosition(Body*, dvec3,
   dvec3)`, which is forward-declared *locally inside the method* and defined
   **nowhere** in the codebase (not in physics.h, not in physics.cpp). It
   compiles only because it's never ODR-used; the first caller would be a
   link error. Delete the method.

2. **`NOISE_FUNC` macro** (main.cpp:2280) — verified dead.
   `#define NOISE_FUNC (((noise3d(sphere_p, 12, 0.60) * 2500)))` — one
   definition, zero uses (its hardcoded 12 octaves / 0.6 / 2500 are the
   legacy defaults, superseded by `Surface`). Delete.

3. **`redraw` flag** (main.cpp:3709, 4486, 4491) — verified vestigial.
   Initialized `false`, set `redraw = true;` at the end of every iteration's
   LOGIC block, never reset. The render section therefore always runs and
   `if(redraw == true)` is a tautology. Vestige of an old event-driven
   design. Delete the flag, keep the block.

4. **`screenshot_count`** (main.cpp:3711, 5154) — verified vestigial.
   Incremented on screenshot, never read. Delete.

5. **Commented-out code blocks** —
   - Dead `ImGui::Text("pos: ...")` / `facing:` / `up:` / `Ground hed:` /
     `Pitch:` / `Heading:` block in the render section (~4650-4660).
   - In `orbitMapWindow`: `raan_p`, `peri_p`, `apo_p` computations marked
     `/* incorrect */` (~5095-5110) whose only consumers are commented-out
     draw calls; the commented `planet[26]` / `AddCircle` lines; the
     commented `horizon_indicator` draw pair. Delete the dead math and
     comments.

6. **Autopilot window** (~5055) — six `ImGui::Button`s (Prograde, Retrograde,
   Radial-in/out, Normal, Anti-normal) with no handlers (return values
   unchecked). Dead UI. Either wire them to the existing `Slew*`/radial
   commands or delete the window and its "DUMB-ASS" checkbox.

## B. Small mechanical refactors

7. **Forward declarations in the wrong place** —
   Already declared in physics.h (which main.cpp includes) → the local
   re-declarations are redundant, verified against physics.h:80-131:
   - `void SetMass(Body*, double)` — main.cpp:1126 (inside
     `consumeResourceMass`)
   - `glm::dmat3 GetOrient(Body*)` — main.cpp:1631 (inside
     `GetOrientRelTo`) and 1652 (inside `moveToFrame`)
   - `void setPosRot(Body*, dvec3, dmat3)` — main.cpp:1651 (inside
     `moveToFrame`)
   - `glm::dvec3 getRelAxis_(Body*, int)` — main.cpp:4567 (render section)
   Delete all five.
   Declared *only* locally (the only declaration that exists) → move into
   physics.h so the header is the single source of truth:
   - `void create_physics(void)` — main.cpp:3152 (defined physics.cpp:133)
   - `void physics_tick(float)` — main.cpp:4351 (defined physics.cpp:184)

8. **Gravitational constant triplicated** — verified:
   `const double G = 6.674e-11;` at main.cpp:494 (load_system), 1230
   (applyGravity), 2224 (spin_log tidal torque). One named constant
   (`constexpr double kG = 6.674e-11;`) in a shared header, or a getter.

9. **JSON color parsing triplicated in `load_system()`** — the same
   `is_array() && size() >= 3` check + three `get<float>()` calls appears
   for `surface.sea_color` (~640), `surface.palette` stops (~655), and
   `surface.atmosphere.color` (~685). Extract
   `static bool readColor3(const nlohmann::json &c, glm::vec3 &out)`.

10. **Frame→inertial state transform copy-pasted 4×** — the sequence
    ```
    v += frame->GetStasisVelocity(p);
    v  = frame->GetOrientRelTo(inertial) * v + frame->GetVelocityRelTo(inertial);
    p  = frame->GetOrientRelTo(inertial) * p + frame->GetPositionRelTo(inertial);
    ```
    appears in `Vehicle::canRail` (~1757), `Vehicle::goOnRails` (~1793),
    the `--orbit-log` block (~4447), and the render section's
    `orbit_pos`/`orbit_vel` (~4625). A `Frame::transformTo(Frame *dst,
    dvec3 &p, dvec3 &v)` helper collapses each site to one call and keeps
    the stasis/velocity convention in one place.

11. **`GeoPatch` kids handled four-ways** — `kids[4]` is touched by
    explicit per-index code in `Draw` (4 recursive calls), `Update` (4
    deletes), `~GeoPatch` (4 deletes), while `Subdivide`/`CountChildren`
    loop. Convert to `std::array<GeoPatch*, 4>` (or keep the C array) and
    range-for everywhere; the destructor and Update's delete block become
    loops.

12. **Reference-to-temporary** (main.cpp:1231) —
    `const double& parent_mass = m_parent->mass;` binds a reference to a
    float→double *temporary* (lifetime-extended). Legal but misleading;
    `const double parent_mass = m_parent->mass;` says the same thing.

13. **`--scenario` name list duplicated** — the CLI
    `->check(CLI::IsMember({...9 names...}))` (~2590) and the
    `kScenarios[]` table (~2625) both enumerate the nine scenario names;
    adding a scenario means editing two places. Build the IsMember list
    from `kScenarios` (and `scenario_by_name`'s "available:" error already
    walks the table).

## C. Bigger but still lunch-sized (pick one)

- **Move the "Planets" debug window out of `TerrainBody::Draw`**
  (main.cpp:366-383, has its own `// TODO move this out maybe?`) and kill
  the global `bool planetsWindow` (main.cpp:47). The window only reads
  `name`, `cam_dist`, and two frame angles — all reachable from the
  ImGui section in main(). The global `ImFont *bigger` (main.cpp:48) is
  similarly movable into main() (only `topHUDWindows` uses it).

- **Group the ~14 window-visibility bools** (main.cpp:3830-3846): the F10
  handler ("TODO should really toggle the UI") sets all of them
  individually (~4328-4341). A `struct UIWindows { bool orbitInfo; ... }`
  with a `clearAll()` would make the F10 path and the checkbox list
  table-driven.

## D. Flagged as odd (design questions, not refactors)

- `getFuelMass(const std::vector /* eh */ <enum ResourceType>& types)`
  (main.cpp:1141) — the "eh" comment is an admission. Also the
  `{ ResourceType::Hydrogen, ResourceType::LOX }` fuel list is hardcoded in
  `getDeltaV`, `getMaxTWR`, and `ApplyThrust`; a named `kFuelTypes` vector
  would centralize it.
- `parts[i]->mass -= amt; /* why does Body have mass at all? */`
  (main.cpp:1124) — open design question: Body carries `mass` and
  `Vehicle` also tracks resources; `SetMass` is called to sync Bullet.
  Worth a real answer eventually.
- `front_indicator->Draw(camera, M_PI /* <- ?? */ + roll)` (~4810) —
  unexplained magic angle baked into the billboard rotation convention.
- `const float camFov = M_PI/3.0; // TODO specify FOV in deg?` (~3755).
- `const double dt = 1.0/50.0; // TODO explain why 50` (~3813).
- `billboardcolor ... // TODO should these be different colors?` (~3600).
- `if(free_cam_pos.size() == 3) { // TODO do we need these guards?`
  (~3770) — CLI11's `->expected(3)` already enforces arity; the guards are
  redundant (a successful parse means size == 3).
- `selftest-rails` drift accumulation: five near-identical
  `drift = std::max(drift, fabs(a-b)/fabs(b))` lines (~3960-3970) — a
  tiny helper or a small table would tidy it.

## Recommendation

A single low-risk "main.cpp: dead code + declaration hygiene" commit
covering A (items 1-6) + B (items 7-9, 12) changes no behavior and is
verifiable by build alone (`make`). C items are good follow-up lunch tasks;
D items are design notes for whenever the relevant systems get touched.
