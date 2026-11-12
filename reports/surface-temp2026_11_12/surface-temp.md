# Everything gets a surface temperature

Plan, 2026_11_12. Directive: **everything gets a surface temp — anything that gets in the
way of that gets deleted**, and where the corpus has no number, we **invent one out loud**
rather than dashing the row. Includes the gas giants. No e2e work in here (user: "forget
e2e") — the cases that quote temperatures get re-pinned by measurement, not new cases.

This is a snapshot of how the project stands and what I propose; the code below is not
written yet. Every number in this report was measured off `res/systems/*.json` with the
generators' own arithmetic (`utils/atmo_units.py:blackbody_temperature`, `semimajor_axis`)
by `tmp/temp_surface_probe.py` → `tmp/surface_probe.log`.

> **Amended same day, before any code was written.** Jool is *not* pair-solved. The KSP wiki's
> min/max column is the one this file already knows is (record low, average) — Kerbin's `max`
> *is* its 288.15 K mean, argued in `utils/gen_systems.py:1275-1278` — and Duna and Laythe sit
> in that same column. Pair-solving Jool while refusing Duna's would be reading one column two
> ways, so the Kerbal airs keep their air-datum anchors and Jool takes the DEFAULT air-mass knob
> like the four solar giants: **151.43 K, f 0.9299, ends 148.7 / 158.8 K**. §3.1, §3.2, §3.4, §7
> and §8 are updated; the pair-solve set is now exactly {Moon, Pluto}, both from RSS pages that
> state a range and no mean.

---

## 1. Where the dash comes from today (measured, not guessed)

| catalogue | non-star bodies | no albedo | no knob (`even_share`) | banded | stated `surface.temperature` | anchored by air datum | anchored by DERIVED | **anchored by nothing** |
|---|---|---|---|---|---|---|---|---|
| `ksp_system.json` | 17 | 0 | **1 (Jool)** | 1 | 1 (Kerbin) | 5 | 11 | 0 |
| `solar_system_full.json` | 239 | **209** | **213** | 4 | 8 | 1 (Pluto) | 29 | **209** |

So **214 of the 256 shipped worlds print a dash for their ground today**, and 209 of them are
one bug: the giant-swarm moons (Jupiter 91, Saturn 76, Uranus 23, Neptune 15, Pluto's 4) carry
no albedo row in the RSS corpus, so the third anchor rung — the black-body the starlight and
this world's albedo imply — cannot fire, and the surface map says *"nothing anchors this
world's ground"*. That is not a gap in the corpus, it is a gap in our nerve.

The other five are the banded worlds (Jupiter, Saturn, Uranus, Neptune, Jool): they have a
stated or air-datum anchor and are refused a knob because three places in the tooling and one
in the game assert that a banded world has no ground.

Two more, quieter, and both wrong in a way a dash hides:

* **Pluto reads 13.4 K at midnight and 53.6 K at noon.** Its anchor is its air datum (38.00 K,
  one layer) and its knob is the airless floor 0.0153 — a 10 µbar air is airless, so the knob
  is right, and the product is a night end 10.6 K **below the 24 K its own sheet states**, in
  an air that cannot hold either number up.
* **The Moon reads 95.0 / 381.2 K** against a stated 95–390 K. Its night end is exact and its
  day end is 8.8 K short — inside the gate, so nobody noticed. Both ends are available; we use
  one of them and throw the other away as a check that nobody reads.

---

## 2. The four things in the way — the deletion list

1. **The banded refusal, spelled three times in the generators.**
   * `utils/gen_systems.py:1200-1205` — `if bands: no_ground.append(name); continue` (Jool, in
     the fitted pass).
   * `utils/gen_systems.py:1411-1414` — `print('  diurnal %-8s dash (a banded giant: the file
     gives this world air at levels and no ground to swing, #738)')` (Jool, in the defaults
     pass).
   * `utils/make_solar_system.py:2583-2586` and `:2641-2642` — the same test in the fitted pass
     and again in the default pass (`Jupiter`, `Saturn`, `Uranus`, `Neptune`).
   Deleted. A banded world's "ground" is the level the file itself calls the surface — the
   1 bar datum, which every one of them already authors (`sea_level_pressure 100000`) — and a
   temperature there is a perfectly good temperature.
2. **`src/surfmap.h:180-182`** — the game's own branch:
   ```cpp
   else if(s.bands) { legend "a banded giant: the file gives this world air at levels, no ground";
                      block "no ground"; }
   ```
   Deleted. The banded world's map draws a ground temperature like any other; the legend keeps
   the *honest* half of that sentence ("this is the 1 bar level, not a sea floor") and loses the
   refusal.
3. **The 209 missing albedos.** Replaced by an invention layer that calls itself an invention
   (§3.3). This is the one that turns 209 dashes into 209 numbers.
4. **The discarded half of a stated pair.** Today a night/day pair is only ever a *test*, and
   when the test has no anchor to be a test against, the pair is printed and dropped. §3.2 uses
   it as the anchor it already contains.

Kept (they are not in the way, they are defence): `kAnchorNone` in `src/groundtemp.h`, the
`!evenShareAuthored` dash in `src/surfmap.h`, and `kEvenShareUnauthored` in `src/authored.h`.
A hand-authored file can still decline; §6 makes sure a *shipped* file cannot do it silently.

---

## 3. The rule, end to end, after the deletions

### 3.1 Anchor (the number the swing lives around) — `src/groundtemp.h`'s ladder, one rung longer

| rank | rung | who it catches |
|---|---|---|
| 1 | stated `surface.temperature` | Mercury, Venus, Earth, Mars, the 4 solar giants, Kerbin |
| **1.5 (new)** | **a stated night/day range, solved for both unknowns** | **Moon, Pluto** |
| 2 | the datum of the air sitting on the ground (#625) | Eve, Shay, Duna, Laythe, **Jool** |
| 3 | DERIVED black-body: starlight + albedo | everything else — all 12 airless KSP bodies, all 239-minus-9 solar bodies |

Rung 1.5 outranks 2 because a source that states a ground's two extremes is talking about the
ground; the air datum is a number we authored for a column. It loses to rung 1 because a stated
*mean* plus a stated pair over-determines the law, and an over-determined law is a test
(#727's whole point) — so Earth, Mars, Kerbin and the solar giants keep their current
behaviour untouched.

**It fires on a range only when the range is a claim about night and day.** RSS labels the
Moon's row "Diurnal temperature range (equator)" and states Pluto's as a surface temperature
range; the KSP wiki's min/max column is a different claim altogether — `gen_systems.py:1275-1278`
established that its `max` is that body's *mean* — so no Kerbal body is pair-solved, Jool
included.

**How it lands in the file (this replaces a 4th C++ rung).** The solving happens at generation
time, so the game needs no new algebra: the generator writes the solved anchor into the key
every consumer already reads — `surface.temperature` — and says what it is in a sibling
`surface.temperature_src`, exactly the way `albedo_src` and `even_share_src` already travel
(`src/system.cpp:668` and `:701` parse those two and refuse a src string that explains an
absent value; the third gets the same invariant). The dossier prints the src string when it is
there instead of "stated ground temperature", which is the #722 rule that a computed figure
announces itself.

### 3.2 The pair algebra (this is the only new maths in the plan)

The law is `T(lat,lon,t) = T0 · [ f + (1-f)·4·cos_inc ]^(1/4)`, so its two ends are

```
T_night^4 = T0^4 · f            T_day^4  = T0^4 · (4 - 3f)
```

Given a stated pair, that is two equations and two unknowns — exact, no fitting, no gate:

```python
r  = (T_day / T_night) ** 4
f  = 4.0 / (3.0 + r)
T0 = T_night / f ** 0.25
```

| body | stated range | → f | → anchor | cross-check |
|---|---|---|---|---|
| Moon | 95 – 390 K | 0.0139 | **276.50 K** | its derived black-body is 270.33 K → **+2.3 %** |
| Pluto | 24 – 38 K | 0.4308 | **29.62 K** | derived 32.23 K → **−8 %** (waiver, see §5.2) |

The Moon's +2.3 % is the interesting one: a number solved from a lander-page's two extremes
lands 2.3 % from a number derived independently from the Sun's temperature and a scraped
albedo. That is the law being right, and it is a much better test than the one we have now
(which only checks the night end, exactly the end the algebra can't get wrong). The cross-check
becomes the guard (§6.2) instead of the anchor.

**Why no Kerbal body goes through here.** The wiki's min/max column would pair-solve all six
airs, and the guard catches four of them for us — read it as evidence about the column:

| body | wiki min/max | → anchor | vs its derived black-body | vs its own air datum |
|---|---|---|---|---|
| Kerbin | 186.95 – 288.15 | 226.67 K | **−11.2 % (refused)** | −21.3 % |
| Duna | 150 – 233 | 182.80 K | **−12.9 % (refused)** | −14.9 % |
| Eve | 160.02 – 420 | 301.57 K | +33.0 % | +3.4 % |
| Laythe | 199 – 277 | 226.85 K | +100.0 % | **−21.3 %** |
| Jool | 120.01 – 225.07 | 168.03 K | +50.8 % | +11.0 % |

A ground that pair-solves out *colder than starlight alone permits* is not a night-side
temperature — Kerbin and Duna say so on their own, and they are the two bodies where #742 had
already caught the column being (record low, average). Laythe is the one the guard misses (its
air is a greenhouse by construction, which is #738's open question), and it is 21 % under the
air we authored for it. So the rule is by the source's wording, not by the arithmetic: RSS
labels the Moon's row "Diurnal temperature range (equator)"; the KSP wiki states one min and
one max with no claim about what they are extremes *of*.

### 3.3 `invent_albedo_value()` — the invention layer, named so it cannot be mistaken for data

One function, shared by both generators (`utils/ground_model.py`, beside the other anchor
helpers), called by each generator right after its scraped-albedo pass:

```python
# INVENTED. The RSS satellite pages for the giant swarms state a radius, a mass and an
# orbit, and stop. A dash there costs 209 worlds their temperature, so we invent an
# albedo and label it, the way KSP_AIR_FOLLOW's value and the air-mass rule's constants
# are invented -- loudly, in the src string, not dressed in an analogy's clothes.
INVENTED_ALBEDO_FAMILY = {          # what this family's surface is made of, judged from
    "Jupiter": 0.05,                #   its stated moons AND from who orbits close in
    "Saturn":  0.50,                #   (the ring-region moons are water ice: the family's
    "Uranus":  0.07,                #    own stated rows run 0.41-0.68)
    "Neptune": 0.07,
    "Pluto":   0.25,                # = Charon, its only stated moon
}
INVENTED_ALBEDO_CAPTURED = 0.05     # anything beyond 100 parent radii is a captured
                                    # irregular: dark rock, whatever its family orbits
INVENTED_ALBEDO_LOOSE  = 0.11       # the Moon, for a body with no family at all

def invent_albedo_value(body, by):
    """[1] A Bond albedo for a body the corpus gives no row. Uses the body's own rail
    around its parent (semimajor_axis off orb_ang_speed) to tell a family's regular
    satellites from its captured ones, and says which rule answered."""
```

The captured/regular split is one comparison — the moon's own semimajor axis in parent
radii, `>= 100` meaning captured — and it sorts the swarm the way the solar system does it:
Metis 1.8 R_J, Pan 2.3 R_S, Cordelia 2.0 R_U, Hippocamp 4.3 R_N are the inner swarm; Himalia
164, Phoebe 222, Sycorax 480, Nereid 224 are captured. Pluto's four sit at 34–52 R and stay
with the family.

Provenance written into the file, same shape as the existing `albedo_src`:

```
albedo_src: "INVENTED: Jupiter regular-family value, no row in the corpus"
albedo_src: "INVENTED: captured irregular (a = 222 parent radii)"
```

Cost of the invention, exactly: T ∝ (1−A)^¼, so any albedo in [0.05, 0.5] moves a derived
anchor by **at most 17 %**, and the anchors land where the real ones do:

| body | invented A | anchor | night | noon |
|---|---|---|---|---|
| Metis | 0.05 | 120.5 K | 42.3 | 169.9 |
| Himalia | 0.05 | 120.5 K | 42.3 | 169.9 |
| Pan | 0.50 | 75.8 K | 26.6 | 106.9 |
| Phoebe | 0.05 | 89.0 K | 31.3 | 125.5 |
| Cordelia | 0.07 | 62.4 K | 21.9 | 88.0 |
| Hippocamp | 0.07 | 49.9 K | 17.5 | 70.3 |
| Styx / Hydra | 0.25 | 41.2 K | 14.5 | 58.1 |

Pan darker-and-colder than Phoebe is the algebra working: the icy moon reflects half the light.

### 3.4 Knob (`even_share`), no change to the ladder — one body more in each tier

Already the right shape (#749): fitted from a stated pair → stated "~0 range" → DEFAULT
air-mass rule `1 − exp(−p/p_ref)` → DEFAULT airless floor 0.0153, with `p_ref` and the floor
**measured from the run's own fits** rather than typed. Deleting the banded skips just moves
five bodies into the tiers; nothing else in it changes.

| body | anchor | knob | night / noon |
|---|---|---|---|
| Jupiter | 165.00 K (stated, 1 bar) | DEFAULT 0.9299 | 162.0 / 173.1 |
| Saturn | 134.00 | DEFAULT 0.9299 | 131.6 / 140.5 |
| Uranus | 76.00 | DEFAULT 0.9299 | 74.6 / 79.7 |
| Neptune | 72.00 | DEFAULT 0.9299 | 70.7 / 75.5 |
| Jool | 151.43 K (air datum) | DEFAULT 0.9299 | 148.7 / 158.8 |
| Venus | 737.00 | 1.0000 **stated ~0 range** | 737.0 / 737.0 |

Two things to read there. The four solar giants' pages state **no diurnal range at all** (only
"Temperature at 1 bar" / "at 0.1 bar", which is a vertical difference, not a day/night one), so
their knob can only ever be the DEFAULT — and the DEFAULT says a 1-bar air meets itself
night-to-day, which for a giant is right. And the rule **saturates**: at Venus's 9.2 MPa it
returns exactly 1.000000, the same number Venus's source states. That is why the src string
matters (`DEFAULT air-mass rule` vs `stated: no range`) and why the two must never be merged
into one field.

Jool now sits in the same tier as its four real analogues, which is also the cheapest thing to
explain in the dossier: *a banded world's surface is its 1 bar level, and a 1-bar air spreads
day and night into each other.* Deleting the banded skip therefore gives Jool an anchor it
already had (its air datum) and a knob from the same rule the solar giants get — one rule, five
worlds, no Kerbal special case. What it does **not** do is touch the air: Jool has no
`air_follow` authored (`KSP_AIR_FOLLOW` is Kerbin and Laythe only), so its profile stays exactly
as authored. If anyone later gives Jool a follow, the 1-bar air would swing with that ground —
print it when it happens, don't let it arrive by accident.

---

## 4. Saying what each number is

The rule in this corpus is that a computed figure announces itself (#722). Invention is a
third degree of separation from a source, so the readouts get one more word, not a new layout:

* `src/gameui.cpp:7090-7115` dossier: when a body carries `surface.temperature_src`, its anchor
  row prints that string rather than "stated ground temperature", and the albedo row already
  prints `albedo_src`, so §3.3's `INVENTED: …` needs no C++ at all.
* `src/system.cpp` gains a third src string beside `albedo_src` (`:668`) and `even_share_src`
  (`:701`), with the same invariant it already enforces for those two: a src string that
  explains an absent value is a loader error, not a decoration.
* `src/surfmap.h` legend: banded worlds keep "the 1 bar level, not a sea floor" and lose
  "no ground"; a body whose albedo was invented says so on the legend, because the anchor under
  it is only as good as that albedo.
* `src/groundtemp.h`'s rung enum is **unchanged** — the pair was solved at generation time, so
  spelling its algebra in C++ would be a fourth copy for nothing (#636).

---

## 5. Guards, so a dash cannot come back quietly

1. **Coverage assert, in both generators.** After the last pass, for every non-star body:
   an anchor rung that is not `None`, and an authored `even_share` — else `assert` with the
   body's name. Today the generators print a verdict and move on; a dash is a legitimate
   outcome there. Under this directive it is a build failure. This is the single change that
   stops the 209 from reappearing next time a page moves.
2. **The one-sided black-body cross-check** (the new use for the derived figure, replacing it
   as the anchor for pair-solved bodies): the pair anchor may sit **above** its derived
   black-body — that gap is internal heat, and it is a real published fact about the giants —
   but never *below*, which would mean the pair cannot be radiative at all. Moon +2.3 % pass,
   **Pluto −8 % fails**, and fails against a corpus that already contradicts itself: the
   `('Pluto','bb')` waiver at `make_solar_system.py:1338-1348` documents that its Bond albedo
   0.72 and its published black-body 37.5 K cannot both come from its own 0.873 W/m² row.
   Extend that waiver to the pair, same argument, and keep printing the number. Applied to the
   Kerbal wiki's min/max column this guard is what refuses Kerbin (−11.2 %) and Duna (−12.9 %)
   — see §3.2 — which is the guard earning its keep on bodies we were never going to take.
3. **A C++ pin** beside `tests/test_scenarios.cpp`'s existing catalogue loops (it already walks
   `ksp_system.json` + the four solar variants for `kWindsAuthored`): every non-star body in
   every shipped file answers `groundAnchorRung(...) != kAnchorNone` and
   `evenShareAuthored(...)`. A hand-authored file that declines now needs a name in a waiver
   list, the way the wind hole does.
4. **Red mutations in `utils/check_atmo_band.py`-style**: delete an `albedo`, delete an
   `even_share`, move a pair outside the cross-check band — each must turn the gate red. The
   coverage assert that never fires is indistinguishable from one that checks nothing.
5. `tests/test_groundtemp.cpp` moves with the law (it is the pin between the header and
   `ground_model.py`): its Moon row goes from `270.33 / 0.015251705 / 95.0 / 381.2` to
   `276.50 / 0.0139 / 95.0 / 390.0` — both ends exact instead of one — and gains Pluto and a
   banded world under the DEFAULT knob (`Jool 151.43 / 0.9299 → 148.7 / 158.8`).

---

## 6. Deferred, with the numbers that justify deferring

* **The parent's own light** (a moon warmed by the world it orbits). Measured, and it is small:
  Metis gets 10.3 % of the Sun's flux back off Jupiter plus 3.3 % of Jupiter's internal heat →
  **+3.2 % in temperature**; Pan **+3.0 %**; Io **+0.3 %**. Io's real surface is much hotter
  than any of that because it is volcanic, and volcanism is not a temperature model — it is a
  heat source, and that's a different ticket. Deferring costs ≤3 % everywhere in either
  catalogue, so this stays out and I'll open the issue rather than implement it.
* **Thermal inertia (#728).** The reason the airless floor exists at all: a bare rock at
  midnight answers ~0 K in this law, and the corpus's Moon row says 95 K. The authored `f`
  damps that empirically. Unchanged.
* **Greenhouse (#738).** The gap between an authored air datum and the black-body its sunlight
  implies (Earth 288 vs 255, Laythe's 53 W/m² question). This plan swings *around* the authored
  number, so it neither closes nor hides that gap. Jool's +51 % pair-vs-black-body line is the
  closest it comes, and it prints rather than explains.
* **Latitudinal and seasonal temperature**, **eclipse**, **obliquity**: the law has one shape
  term (`cos_inc`) and no time constant. Unchanged.
* **Stars**: Kerbol 2838.1 K and the Sun 5769.4 K already state a surface temperature, and the
  swing stays dashed for them — *"no star in range to light this world"* is a fact about the
  swing, not a refusal of the temperature. The surfmap branch stays; the coverage assert skips
  `type == "star"`.

---

## 7. Order of work, one proof per step

1. `invent_albedo_value()` in `utils/ground_model.py` + both generators' calls, with the src
   string and a per-family print (209 per-body lines would bury the rest of the verdict; the
   default pass already suppresses per-moon chatter for bodies with no page). **Proof:** the
   generator prints the family rows and `solar_system_full.json`'s no-albedo count goes
   209 → 0; the JSON-normalised diff touches only `albedo`/`albedo_src`.
2. Scrape Pluto's `Surface temperature: 24 - 38 K` row. The sheet parser reads only a "Diurnal
   temperature range" line today (`make_solar_system.py:1300`), so this range is in the corpus
   and invisible to it. **Proof:** the parse prints both ends; nothing emitted changes yet.
3. `pair_solve()` in `utils/ground_model.py`, the authoring into `surface.temperature` /
   `temperature_src` / `even_share`, and the one-sided cross-check with its Pluto waiver.
   **Proof:** the generator prints Moon `276.50 / 0.0139` and Pluto `29.62 / 0.4308`, each with
   both stated ends exact, and the black-body comparison printed beside each.
4. The C++ half of the src string: `terragen.h`, `src/system.cpp` (with the "explains an
   authored value" invariant the other two src strings already carry), `src/gameui.cpp`.
   **Proof:** loading a mutated file that puts a `temperature_src` on a body with no
   `temperature` is a loader error; the Moon's dossier anchor row reads the SOLVED string
   instead of "stated ground temperature".
5. Delete the banded skips (three in the generators, one in `src/surfmap.h`). **Proof:** the
   verdict lines print `Jupiter 0.9299`, `Saturn 0.9299`, `Uranus 0.9299`, `Neptune 0.9299` and
   Jool's DEFAULT row; `--surfmap-body Jupiter` draws a temperature grid instead of "no ground"
   (visual check, ask me).
6. Coverage assert in both generators + the catalogue walk in `tests/test_scenarios.cpp` + the
   three red mutations. **Proof:** `make test` green, and each mutation red with the body's
   name in the failure.
7. Re-pin by measurement, not by reasoning: the cases that quote ground/air temperatures —
   `science/727-solar-log-dashes-what-a-world-does-not-state` (its dash legs have to move:
   Pluto and the swarm no longer dash), `science/727-solar-log-weighs-the-star-against-the-ground`,
   `science/667`, `science/125-jool-shell-vacuum`, `science/724-air-follows-ground`,
   `flight/631-earth-air-in-flight`, `flight/245-air-below-datum`, `harness/662`. Run these
   once, at the end, together — not per step.
8. Issue for the parent-flux term, and close #724's coverage line.

Estimated file count: 7 code files (2 generators, `ground_model.py`, `terragen.h`,
`system.cpp`, `surfmap.h`, `gameui.cpp`), 2 test files, 5 emitted JSONs, and 8 e2e case files
re-pinned.

---

## 8. Decisions I took that you can veto

1. **The KSP wiki's min/max column is not a night/day range**, so no Kerbal body is pair-solved
   and Jool takes the DEFAULT air-mass knob like its four real analogues — **151.43 K,
   f 0.9299, 148.7 / 158.8 K**. This reverses what the first cut of this report argued (Jool at
   168.03 K from its pair); §3.2 has the evidence, which is that Kerbin's and Duna's pairs
   solve *below* their own black-bodies, so the column cannot be extremes of a day.
2. **The solved anchor goes into `surface.temperature` with a sibling `temperature_src`**,
   rather than into a 4th rung of the anchor ladder. Cost: the ladder's rank no longer
   distinguishes "a source states a mean" from "we solved this from a range" — that honesty
   lives entirely in the src string and in the dossier printing it. The gain is that every
   existing consumer (surfmap, the air-follow coupling, the tick readout) works unchanged and
   the algebra stays spelled twice instead of three times.
3. **Pluto is re-anchored** from 38 K (air datum, printing 13.4/53.6 K) to 29.62 K with knob
   0.4308, printing exactly its stated 24/38 K, with the −8 % black-body deficit waived under
   the argument that already covers its albedo/black-body contradiction. Its row is not labelled
   diurnal — it is RSS's "Surface temperature" range — so this is reading a stated *range*, and
   the waiver is where that gets argued rather than hidden.
4. **A DEFAULT knob of exactly 1.000000 is allowed to print** for an air thick enough to saturate
   the air-mass rule (Venus), as long as the src string distinguishes it from Venus's *stated*
   ~0 range. If you would rather no body ever print a perfectly flat swing, the alternative is a
   `f_max = 0.99` cap on the rule — one more invented number, and every thick air gets a 2 K
   swing nobody stated.
5. **The captured/regular line at 100 parent radii** decides whether Phoebe is icy or rocky. It
   puts Phoebe (222 R) with the dark rock and Pan (2.3 R) with the ice, which is right for both,
   and it is one comparison instead of a 209-name list.
6. **Stars keep a dashed swing** while keeping their stated temperature.
