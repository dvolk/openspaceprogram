# In-game time based on the home planet

**Question (Denis, 2026-08-23):** how could in-game time work if it were based on the
system's home planet?

**Short answer:** it's almost free. The sim already has a monotonically increasing clock
(`time`, seconds), the home planet is already a first-class system field (`home`), and the
home planet already visibly spins on an analytic schedule (`ang = fmod(rot_ang_speed*time, 2π)`).
A calendar is a pure function of those two existing things: `day = 2π / home's spin rate`,
`year = 2π / home's orbital rate`. No new state, no new physics — just derive, format, and show
it. For both current systems a home day is **21,549 s (5 h 59 m)** and a home year is
**9,203,545 s (106.5 real days ≈ 427.1 home days)**.

---

## 1. What exists today (ground truth)

| Thing | Where | What it is |
|---|---|---|
| Sim clock | `main.cpp:2578` | `time += dt * time_accel` — one `double`, seconds, starts at 0 |
| Time accel | `main.cpp:2292, 2384-2399, 1937-1938` | `time_accel ∈ {0,1,10,100,1000}`, keys `,`/`.`, CLI `-t` |
| Home planet | `main.cpp:306, 334, 399, 622-640` | `home` field of the system JSON: "the planet the ship starts on" (default: first non-star) |
| Body spin | `frame.cpp:54-61` | `ang = fmod(rot_ang_speed * time, 2π)` — analytic, cumulative, no snap on accel change |
| Spin data | system JSON `rotating.rot_ang_speed` (rad/s) + `axial_tilt` | read at `main.cpp:568-580`; a star gets a dummy zero-spin frame |
| Sun direction on the ground | `main.cpp:251-255` (`SunlightDir`) | sun→planet vector from the **inertial** frames, expressed in the render frame — so the lit/dark side **already moves** with spin + orbit |
| Clock display | `main.cpp:2995` | `Time: %f` — raw seconds in the debug window. No day, no time-of-day, no year anywhere |

Two things worth noting before the design:

1. **The day/night cycle already runs.** `SunlightDir` is computed from the inertial positions
   of sun and planet, which advance with `time`, and it's expressed in the (rotating) render
   frame. The terminator is already sweeping across the home planet at the spin rate — the game
   is missing the *name* of what the player is watching, not the phenomenon.
2. **Nothing persists.** There is no save state (the only "save" in the tree is
   `SaveScreenshot`), so every launch starts at `t = 0`. An epoch anchored to game start is
   therefore not a compromise — it's the only option the persistence model allows, and it stays
   valid if saves ever appear (persist `time`; the calendar is derived from it).

The raw material for a home-planet clock is already loaded:

```
home->rot_frame->rot_ang_speed   rad/s   (spin  -> defines the day)
home->frame->orb_ang_speed       rad/s   (orbit -> defines the year)
time                             s       (the sim clock)
```

---

## 2. The proposal: a home-planet calendar

Define, at load time, from the two numbers above:

```
D = 2π / home.rot_ang_speed      // one home day, in sim seconds
Y = 2π / home.orb_ang_speed      // one home year, in sim seconds  (0 if home doesn't orbit)
```

Then the calendar at sim time `t` is pure arithmetic:

```
year    = floor(t / Y) + 1                 (or 1, if Y == 0)
day     = floor(t / D) + 1
tod     = t mod D                          -> mapped onto a 24-hour dial:
                                              hh = tod * 24 / D,  etc.
                                              (1 home hour = D/24 sim seconds)
```

Display as **`Yr 1 · Day 3 · 14:22:07  (Eerbon time)`** — KSP-style, where the system clock is
*home time everywhere*, even while orbiting the Moon or sitting on Duna. The ship HUD already
shows "Home distance"; the clock is its companion.

**Concrete numbers for the two current systems** (from `system.json` / `ksp_system.json`,
i.e. `gen_systems.py`):

| | Eerbon | Kerbin (KSP) |
|---|---|---|
| day `D` | **21,549 s = 5 h 59 m 09 s** | **21,549 s** (same) |
| year `Y` | **9,203,545 s = 106.5 real days** | **9,203,544.6 s** |
| days per year | **427.1** (non-integer, fine — real calendars do this too) | 427.1 |
| 1 home hour | 897.9 s of sim time | 897.9 s |
| first moon (Moon / Mun) | 138,984 s = 38.6 h = **6.45 home days**, tidally locked | same |

Both home planets happen to be Kerbin-analogs, so the two games would feel identical on the
clock. A future system with a 10-hour day or a 300-day year gets the right calendar for free —
that's the point of deriving it rather than hardcoding it.

For contrast, the rest of the KSP system (CSV values in `gen_systems.py`) spans a wide range,
and each would get its own correct day if it were ever promoted to `home`:

| body | day | year | days/year |
|---|---|---|---|
| Eeloo | 19,460 s (5.4 h) | 156,992,048 s | 8,067 |
| Moho | 1,210,000 s (14.0 d) | 2,215,754 s | 1.8 |
| Eve | 80,500 s (22.4 h) | 5,657,995 s | 70 |
| Duna | 65,518 s (18.2 h) | 17,315,400 s | 264 |
| Dres | 34,800 s (9.7 h) | 47,893,063 s | 1,376 |
| Jool | 36,000 s (10 h) | 104,661,432 s | 2,907 |

---

## 3. The decisions (and my recommendations)

### 3.1 Sidereal day vs solar day — use the sidereal one (what the sim actually computes)

The quantity the sim advances is the planet's **spin angle**. Defining the clock by
`D = 2π/rot_ang_speed` keeps the clock exactly locked to the visible rotation: every time the
clock wraps at midnight, the home planet has turned through exactly 2π, so a fixed landmark
returns to exactly the same sky direction on every clock-day.

The **solar** day (noon-to-noon) is longer, because the planet also moves along its orbit:
`solar = D / (1 − D/Y)`. For Eerbon/Kerbin that's 21,600 s — a drift of **~50 s per day**,
accumulating to one full day per year (that's what seasons are). Consequences of choosing
sidereal:

- the sun's position in the local sky lags the clock by 50 s/day — invisible at game scales;
- if we ever want the clock to track the *sun* exactly (e.g. for "solar noon" events), it's a
  one-line change: swap `D` for `D/(1−D/Y)`.

Recommendation: **sidereal** — it matches the simulated quantity, keeps the clock and the
rendered rotation perfectly in phase, and the drift is negligible.

### 3.2 The dial — use a 24-hour home dial, not real HH:MM:SS

Two ways to format `tod`:

- **(a) Real clock:** 1 h = 3600 s always. A 21,549-s day then ends at "05:59:09", and the
  clock and the sky decouple (the same local sun position appears at 5:59 one day and 05:59
  another).
- **(b) Home dial (recommended):** 1 h = D/24. The clock always reads 00:00–23:59 within one
  rotation; "12:00" always means "a quarter-turn from midnight". This is the KSP convention and
  what players intuit.

### 3.3 Epoch — game start is Day 1, 00:00:00

No persistence exists, so `t = 0` → `Day 1, 00:00:00` is deterministic, per-launch, and needs
zero stored state. If we later want the launch pad to start at local noon (nicer first
impression), that's just a phase offset in the formatter (`tod = fmod(t + D/2, D)`) — one line,
and the phase can be chosen per-system in JSON if we ever want it.

### 3.4 Year — include it, skip months

`Y = 9,203,545 s` ≈ 427 home days, so the year matters after a few weeks of play and is
meaningful for "how long was this campaign". Months are KSP-absent too and add nothing until
seasons exist — skip them. (If seasons arrive later, they'd be a quadrant of `t mod Y`, and the
`axial_tilt` values are already in both system files: Eerbon 23.4°, Kerbin 23.44°.)

### 3.5 Universality — one clock per system, home-based, everywhere

The clock is home time even while on the Moon. This is the KSP convention (Kerbin time
everywhere) and it keeps inter-body rendezvous talkable ("we meet at Day 12, 09:00"). Per-body
local time (Duna time, Mun time) can be a *secondary* readout later — it's the same formula with
a different body's `D` — but the primary calendar should be the home's.

### 3.6 Pause interaction — falls out for free

The clock is a function of `time`, and `time` only advances when `time_accel != 0`. So a paused
world (`time_accel == 0`) has a frozen clock automatically — consistent with the established
"paused = nothing moves" principle. No special-casing.

---

## 4. What a home day/year costs in real time, per time accel

With `D = 21,549 s`, `Y = 9,203,545 s` (both current systems):

| time accel | one home day takes | one home year takes | feel |
|---|---|---|---|
| 1× | 5.99 real h | 106.5 real days | real-time; day/night barely perceptible |
| 10× | 35.9 real min | 10.65 real days | a day per half-hour — nice for watching the terminator |
| 100× | 3.59 real min | 25.6 real h | day/night visible on every flight |
| 1000× | **21.5 s** | 2.56 real h | days tick by like seconds; the year is a long campaign |

This is a useful side effect: the accel steps that exist for orbital mechanics (`,` / `.`)
double as "how fast does time pass" without any new control.

---

## 5. What this unlocks (why it's worth more than a HUD line)

1. **Naming the existing day/night.** The lighting already cycles (`SunlightDir`); the clock
   tells the player where they are in it ("why is it dark here? it's 23:40").
2. **Campaign feel.** "How long was that flight?" becomes answerable — KSP's flight time is
   one of the things that makes the sim feel like a world with history.
3. **Time-based content** becomes expressible: docking windows, scheduled rendezvous,
   per-day consumables (crew food/O2), events ("on Day N…") — all testable against a clock
   instead of a raw-seconds counter.
4. **Seasons later:** `t mod Y` + `axial_tilt` (already in the data) give a year position the
   sun-angle math can use.
5. **Per-game calendars for free:** a new system file with a 10-hour day / 300-day year just
   works — the clock is derived from the loaded `home` body, nothing per-game to configure.

---

## 6. Implementation sketch (small)

**`struct System`** (next to `home`/`moon`, `main.cpp:301-312`):

```cpp
double day_seconds;   // 2π / home's spin rate;   0 = home doesn't spin (star)
double year_seconds;  // 2π / home's orbital rate; 0 = no orbit
```

**After home resolution** (`main.cpp:622-640`):

```cpp
sys.day_seconds  = (sys.home && sys.home->rot_frame &&
                    sys.home->rot_frame->rot_ang_speed > 0.0)
                 ? 2.0 * M_PI / sys.home->rot_frame->rot_ang_speed : 0.0;
sys.year_seconds = (sys.home && sys.home->frame &&
                    sys.home->frame->orb_ang_speed > 0.0)
                 ? 2.0 * M_PI / sys.home->frame->orb_ang_speed : 0.0;
```

**Clock** (pure function — trivially testable):

```cpp
struct HomeTime { int year, day, hh, mm, ss; };

HomeTime home_clock(double t, double day, double year)
{
    HomeTime h;
    h.day  = (int)std::floor(t / day) + 1;
    h.year = (year > 0.0) ? (int)std::floor(t / year) + 1 : 1;
    double hours = std::fmod(t, day) * 24.0 / day;   // 24-hour home dial
    h.hh = (int)hours;
    double mins = (hours - h.hh) * 60.0;
    h.mm = (int)mins;
    h.ss = (int)std::lround((mins - h.mm) * 60.0);
    if (h.ss >= 60) { h.ss -= 60; h.mm++; }
    if (h.mm >= 60) { h.mm -= 60; h.hh++; }
    return h;
}
```

**HUD** — two one-line additions:

- debug window (`main.cpp:2995`): keep `Time: %f`, add
  `ImGui::Text("Clock: Yr %d  Day %d  %02d:%02d:%02d  (%s time)", ..., sys.home->name.c_str());`
- main HUD, next to the existing readouts (e.g. near "Home distance", `main.cpp:3015`):
  `Day 3  14:22:07  (Eerbon time)`.

**Edge cases:** home is a star or a dummy-spin body → `day_seconds == 0`; guard the HUD to show
raw seconds in that case. Both current systems have a proper home planet, so this is defensive.

**Test** (`tests/test_homeclock.cpp`, same `CHECK`-macro style as the other standalone tests):
- `2π / 2.9157090303706880702966723086e-4 ≈ 21549 s` (Eerbon's actual JSON spin rate);
- `home_clock` rollover: `t = D − ε` → `00:00:00`-ish, `t = D` → Day 2 00:00:00;
- year rollover at `t = Y`; day-427 boundary sanity for the 427.1-day year.

Total footprint: ~30 lines in `main.cpp`, one header-visible pure function, one test file.
No new state, no new JSON fields, no physics, no asset changes.

---

## 7. Open questions (yours to answer)

1. **Dial:** 24-hour home dial (recommended) vs real HH:MM:SS?
2. **Year display:** show `Yr N` always, or only once `N > 1`?
3. **Epoch phase:** Day 1 00:00:00 at launch (recommended, simplest) or start at local noon?
4. **Per-body local time** as a secondary readout (e.g. "Duna local 09:12") — now, or later?
5. **Scope check:** is this just the clock (this report), or the first step toward a save
   system where the clock is what you're actually preserving? (It's derived from `time`, so
   either way there's nothing extra to persist.)
