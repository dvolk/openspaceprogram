# Atmospheres: us vs Orbiter vs Pioneer

Snapshot 2026-11-06. Our side at `8fbd582`. Reference checkouts in `~/references`:
Orbiter `ce328588`, Pioneer `8ea5aa5df`. The input documents are the two agent-written
reports that sit beside this file — `orbiter_ATMOSPHERE_REPORT.md` and
`pioneer_ATMOSPHERE_REPORT.md` (also in `tmp/aero/`); they are kept here unedited as the
raw inputs, **not** as findings: every claim quoted below I re-read in the checkout
myself. Their path citations are a directory level off in places
(`.../Atmosphere/MarsAtm2006.cpp` is really `.../Atmosphere/MarsAtm2006/MarsAtm2006.cpp`);
the line numbers I quote are mine.

## 1. The three models in one table

| | us | Orbiter | Pioneer |
|---|---|---|---|
| Profile | one exponential (isothermal) **or** a `layers` table with lapse rates, integrated hydrostatically (`atmo.h:levelPressure`, `log1p` form) | per-body model plug-in: mode 1 = two-number exponential, mode 4 = piecewise isothermal/lapse layers **compiled into a DLL** (Earth, Mars, Venus) | exactly one lapse layer, closed form, lapse = `g/cp` (dry adiabatic) |
| Altitude variable | geometric height above the sea-level datum | **geopotential** — `alt *= rad/(rad+alt)` at the top of every mode-4 model | geometric |
| `g` in the profile | `body->g` at the datum, constant with height (same in both branches) | a `g0/R` constant baked into each model DLL | `CalcSurfaceGravity()`, constant |
| Sea-level pressure | **derived**: `p0 = ρ0·(R/M)·T0` | **authored** (`AtmPressure0`), and config can override the model's own constants | **derived** from `ρ0`, `T0`, `M` |
| Temperature | derived on an exponential (`T = gHM/R`, atlas says so); authored per level with `layers` | authored per layer; **hard-coded 288.16 K** for all mode-1 bodies (`Planet.cpp:843`) | authored one scalar per body |
| Density | derived: `ρ = p/(Rs T)` | **authored alongside p** (`ρ_base·e/t`) | derived: `p·M/(R·T(h))` |
| Composition | mole-fraction mix → `M`, `γ`, `oxidizer_share` (`atmo.h:resolveMix`, #630) | per-body `R`, `γ` constants | two hardcoded mixes; `atmosOxidizing` is decorative for physics |
| Speed of sound | `sqrt(γ g H)`, or `sqrt(γ Rs T(h))` with layers | `sqrt(γ R T)` with T from the profile | **none** (no Mach effects at all) |
| Where the air stops | authored `height`, else `10 × scale_height` (`kAtmoScaleHeights`) | `atm.altlimit`, config-overridable (Mars: model says 100 km, `Mars.cfg` says 280 km; Jupiter 3200 km, comment `; raised to enable aerobrake capture`) | **derived** from a pressure cutoff: `p = 0.001 atm` |
| Load-time refusals | ~12 (see §2) | none | none |
| Wind | none (air co-rotates, always still) | yes: seeded ±25 m/s per integer km of altitude, Catmull-Rom in altitude, off by default | none |
| Reynolds / viscosity | none | `Re0 = ρV/μ`, `μ = 1.7894e-5` fixed, in-source `// MAKE VARIABLE!` | none (TODO) |
| Heating / crush / scooping | none | none (reentry is a particle stream keyed off density) | all three: drag-G heating, per-ship pressure limit, fuel scoop |
| Sky vs physics | rim mesh at `datum + max_height + thickness` (an independent authored number) | haze/fog/colour params are an independent set (`AtmAttenuationAlt` 100 km vs a 2500 km model) | **isothermal exponential to 10·H = 84 km for the sky, adiabatic to 25 km for physics** — you can see sky where density is 0 |

## 2. What our loader refuses that they ship

The aero/atmosphere invariants we added over the last weeks (`system.cpp`,
`atmo.h:resolveAtmoLevels`, `resolveMix`) are load-time refusals. Each of these
real shipped data errors would be one of them:

| Their data | What it implies | Which of our checks fires |
|---|---|---|
| Orbiter Io: `R = 1.92`, `γ = 0.3333`, `p0 = ρ0 = 1e-4` | γ < 1 is unphysical; scale height 0.56 m | "resolves to a molar mass or gamma no gas in the species table gives" |
| Orbiter Jupiter mode 1: `p0 = 2.014e6`, `ρ0 = 1.3293` | implied `T = p0/(ρ0 R) = 7770 K`, while `GetAtmParam` reports **288.16 K** for every mode-1 body (Saturn 5910 K, Uranus 4400 K, Neptune 5610 K, Titan 83 K) | unrepresentable: our `T` *is* `gHM/R`, one statement |
| Orbiter Mars 2006 surface triple: `p0 = 610 Pa`, `ρ0 = 0.02`, `T = 195 K`, `R = 188.92` | `ρ_gas = p/(RT) = 0.01657` — the authored density is **20.7 % high**. Both chains share one exponent, so `ρ/p` stays pinned at a 161.6 K value while `prm->T` reports 195/180/165 K → Mars Mach numbers biased ~10 % (`sqrt(195/161.6)`) | unrepresentable: we derive ρ from p and T. (Earth's table is consistent to 0.2 %, Venus's to 0.06 % — only Mars is off.) |
| Pioneer Earth: `atmos_density = 1.0` | `p0 = 0.816 atm` by accident — the density field *is* the pressure knob | same failure mode is open to us: see §6, our Venus |
| Pioneer's own profile: `L = g/cp`, one layer, Earth `T0 = 288` | `T` walks through 0 at 29.4 km; their top is 25.1 km where `T = 42.6 K` | "this lapse walks T through zero inside the air" — we would reject Pioneer's model *as authored* |

One place we are demonstrably cleaner: our authored `g` matches `G·mass/r_sea²` on every
body with air (worst case 6e-4 relative, Kerbin 9.810 vs 9.811), and the profile always
reads the live `body->g` (`vehicle.cpp:dragAtm(atm, m_parent->g)`). Orbiter bakes `g0/R`
into each model DLL, so a mod that changes the planet's mass keeps the air's gravity.

A note on our own lapse floor: `resolveAtmoLevels` refuses `lapse <= -g/Rs` (the point
where density grows with height). The convective limit is `-g/cp`, 3.5× stricter, and
Pioneer *uses that one as its default*. `atmo.h` already names it in a comment; we have
no shipped level anywhere near it (Earth's steepest is −6.5 K/km against −9.77 K/km), so
this is a documented slack rather than a bug — but it matters the moment bodies are
*generated* rather than authored.

## 3. Where the air stops (the cliff question)

All three cut to hard zero. What differs is how thin the air already is there.

- **Pioneer** derives its top from `p = 0.001 atm` — a *thinness* rule. On Earth that is
  25.1 km, where `ρ = 8.9e-3 kg/m³`. At orbital speed (7.9 km/s) the last metre of air
  still bills **q = 277 kPa**. A pressure ceiling is not by itself enough.
- **Orbiter** puts its tops very far out and lets a cold isothermal tail collapse the
  density: Mars 280 km → `ρ ≈ 1.3e-15`, Venus 600 km → essentially zero, Earth's 2006
  model at 200 km → `ρ ≈ 1.6e-13`, and Earth's *domain* runs to 2500 km because
  Jacchia-71/Gill and NRLMSISE-00 are there to fill it in. The cliff is invisible because
  the air is genuinely gone; the price is an enormous "am I in atmosphere?" sphere and a
  thermospheric model to make it mean anything.
- **Us**: authored `height` or `10 H`. Kerbin's edge is at 12 Pa of q on a hull at
  circular speed (negligible, and arguably a feature: "periapsis above 70 km is stable
  forever"); Venus's derived edge is at **115 kPa** — a wall. Tables and the full
  argument are in **#682**.

Their `lng=0; lat=0` lines and the never-set `PRM_F/AP` flags are the reason Orbiter's
fancy upper-atmosphere models are decorative: J71G's diurnal/seasonal terms are always
evaluated at 0°N 0°E, and F10.7/Kp are frozen at 140/3 forever. Worth remembering as
"an elaborate model with frozen inputs is a static profile with extra steps".

## 4. What the profile assumes about gravity (new, #683)

`airDensity`/`levelPressure` integrate a column in a **uniform** field (`body->g`), while
`Vehicle::applyGravity` pulls each part with `G·m·M/r²`. Orbiter reconciles the two by
working in geopotential altitude (`alt *= rad/(rad+alt)`); we don't, and on KSP-scale
radii that is not a rounding error: the column our air implies is too deep by
`1 + z/R`.

| body | R | top | edge `ρ` (ours) | edge `ρ` in the field the hull feels | ratio | at 20 km |
|---|---|---|---|---|---|---|
| Kerbin | 600 km | 70 km | 3.6e-6 | 1.4e-5 | 3.8× | +12 % |
| Eve | 700 km | 90 km | 4.4e-6 | 1.9e-5 | 4.3× | +8 % |
| Duna | 320 km | 50 km | 4.5e-7 | 2.4e-6 | 5.4× | +34 % |
| Laythe | 500 km | 50 km | 1.4e-4 | 3.2e-4 | 2.3× | +15 % |
| Earth | 6371 km | 85 km | 6.7e-6 | 6.8e-6 | 1.01× | +0.7 % |

In the regime people actually fly the error is single-digit to tens of percent; it is at
the top of the air that it gets large — which makes #683 and #682 pull in opposite
directions: fixing the gravity makes the air 4× thicker at the top, so the cliff gets
*before* moving any top is decided.

## 5. Features they have that we do not

- **Wind.** Orbiter's is ±25 m/s of seeded noise per kilometre of altitude (identical
  parameters for Titan and Mars, no latitude/season/terrain, off by default) — a warning
  about the cheap version, not a design to copy. Pioneer has none, like us. Our roadmap
  already lists it (1.3, plus gusts and pressure cells). The one thing both references get
  right and we should copy exactly: wind enters as a **velocity of the air**, so
  airspeed → lift, drag, Mach, HUD and control authority all follow through one code path.
  We already have that seam (`Vehicle::airRelativeVel` subtracts co-rotation).
- **Reynolds/viscosity**: Orbiter has a fixed-μ proxy with a "MAKE VARIABLE" TODO. We
  have nothing; the roadmap names it as a known hole. Neither reference is worth copying
  here — a constant μ buys nothing over nothing.
- **Reentry heating, pressure crush, fuel scooping** (Pioneer): all three are player-
  visible *consequences* of an atmosphere, which is exactly what our air currently lacks
  beyond force. Pioneer's heating is a drag-G proxy (the ρv³ version is commented out in
  its own source); its crush limit is a plain per-ship atm limit × shield rating; its
  scoop rate is `ρ·v·area`. All cheap, all gated on state we mostly already have — note
  a scoop would want our `oxidizer_share`, which neither reference can compute.
- **Upper atmosphere / space weather** (Orbiter): Jacchia-71/77 + Gill's bi-polynomial,
  NRLMSISE-00's full `gtd7`. Far past us, and §3 explains why it barely earns its keep in
  Orbiter itself.
- **Scattering sky** (both): Pioneer derives its render scale height from `RT/Mg` and
  precomputes per-planet (k, b, c) out-/in-scattering coefficients once instead of
  ray-integrating per pixel — a good budget trick if our rim shell ever becomes real
  scattering. Orbiter has a full haze/Mie stack in the client, decoupled from physics.

## 6. What we have that neither reference has

- A composition with a *consequence*: `oxidizer_share` gates air-breathing engines
  (#637) and a mix resolves `M` and `γ` (`#630`). Pioneer has a decorative scalar;
  Orbiter has two per-body numbers that can and do contradict the profile.
- A data-authored layer table. Orbiter's is compiled C++ in a DLL (its own developer
  manual has no editable profile anywhere); Pioneer has none.
- `liftSlopeMach()` in the core: Mach-corrected lifting-surface authority. Orbiter leaves
  all transonic/wave-drag behaviour to per-vessel module callbacks; Pioneer has no Mach
  at all.
- The trio lock (`ρ0`, `H`, `g` ⇒ `T`, `p0`, `a`) with the atlas *labelling* it
  "(isothermal)" rather than dressing it up — the honesty device neither reference has.
- The near-body frame shell derived **from** the air top (`bodylimits.h:shellEdge`), so
  the co-rotation law the aero path assumes cannot be violated by data.

## 7. Findings this comparison produced

- **#682** — the air's hard top is a distance rule (`10 H`), so dense-air worlds end in a
  wall: Venus at 160 km still has 4.2 g/m³ and 115 kPa of q there; Laythe's authored top
  is only 9.1 H, so its edge is thicker than the rule would give.
- **#683** — the profile integrates a uniform-`g` column while the hull feels `1/r²`:
  3.8–5.4× too thin at the air top on the KSP-scale bodies, ~10 % at 20 km. Orbiter's
  geopotential conversion is the one-line answer.
- **#684** — our Venus authors `sea_level_density 92`, which is the surface *pressure*
  (92 bar) mistyped as a density: the atlas reads 129 atm against a real 92 atm. `65`
  makes the whole column land within ~1 % of Venus. Orbiter carries the same datum —
  `Venus.cfg:32 AtmDensity0 = 65` beside `AtmPressure0 = 9200e3` (and note that it authors
  both, which is exactly the pair our format refuses). Pioneer's Sol bodies are generated
  procedurally in this checkout, so it has no comparable datum to cite.
- A correction to my own fix idea in #682: a **pressure** ceiling is not a safe thinness
  rule — Pioneer's `0.001 atm` ceiling still leaves 277 kPa of q at the edge. A residual
  *density* target, or simply "many e-folds out", is the right shape.

## 8. Sequencing

Nothing here argues for touching the aero model now. #684 is one JSON number. #683 wants
a decision (adopt geopotential altitude in both profile branches, or keep geometric and
say so) and it moves flight goldens, so it belongs with the aero-law rows. #682 wants a
profile decision for the dense worlds, which is the same conversation as #625. If wind
(roadmap 1.3) ever lands, both references agree on where it goes: into the air's
velocity, not into the force terms.
