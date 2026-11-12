# Surface temperature: what the field does NOT do yet (lag, and four other missing effects)

Snapshot of `src/groundtemp.h` / `utils/ground_model.py` as of `081ac4ab` (2026-11-12). Companion to
`reports/surface-temp2026_11_12/surface-temp.md`, which describes what the model *does*. This one is
about the gap the user pointed at -- "the temp doesn't change immediately with the sun" -- plus the
other effects I found while looking, each with the cost and the data hook it would need.

## 1. What the field is today, exactly

```
T(lat, lon, t) = T0 * [ f + (1 - f) * 4 * cos_inc ] ^ (1/4)
```

`cos_inc` is the floored cosine of the star's elevation at that place, computed at the *instant* the
readout asks (`insolation.h:skyAtPlace`). `f` is the authored even share. Nothing in the chain has a
time constant, and `groundtemp.h` already says so out loud ("a hull at noon feels a noon ground
immediately").

Three consequences worth naming, because they are what a player would notice:

1. **The hot spot sits exactly under the star.** The maximum of the field is at the subsolar point,
   to the pixel. On Earth the hottest ground is at local noon; in reality it is mid-afternoon.
2. **The whole night hemisphere is one isotherm.** Anywhere `cos_inc = 0` answers exactly
   `T0 * f^(1/4)`. Measured for the shipped file: Earth's swing row `283.0 to 301.8 K` and the
   Moon's `95.0 to 390.0 K` are literally `anchor * f^0.25` and
   `anchor * (f + 4(1-f))^0.25`:

   | body | anchor T0 | f | night end (calc) | day end (calc) | atlas prints |
   |---|---|---|---|---|---|
   | Earth | 288.0 | 0.9323 | 283.0 | 301.6 | 283.0 to 301.8 |
   | Moon | 276.50 | 0.01394 | 95.0 | 390.0 | 95.0 to 390.0 |

   So there is no *pre-dawn minimum* -- the coldest moment is anywhere in the night, equally -- and
   no cold-trap credit for a permanently shadowed pole: it reads the same as the anti-solar point.
3. **The terminator is a cliff.** At the day edge (`cos_inc -> 0+`) the gradient along longitude is
   `T0 (1-f) f^(-3/4)` kelvin per radian. Measured on the shipped numbers: **Earth 0.4 K per degree**
   of longitude, **Kerbin 0.4**, **the Moon 117 K per degree**. The air worlds look smooth because
   their `f` is near 1; the airless ones have a wall where the day begins. Real regolith conducts
   across that band and would soften it.

## 2. The user's instinct, in physics terms

A surface with heat capacity `C` obeys `C dT/dt = (1-A) S g(t) - eps sigma T^4`. Linearised about the
mean, the response to the diurnal harmonic has **one** parameter, the thermal time constant
`tau_th = C / (4 eps sigma T0^3)`, and it shows up as **two** effects at once:

- amplitude damping, gain `1 / sqrt(1 + (omega tau_th)^2)`, and
- a **phase lag**, `phi = atan(omega tau_th)`, i.e. the peak sits at local time `12h + tau_lag`
  with `tau_lag = phi / omega`.

The current model has an empirical damping knob (`f`) and *no* phase. That is why it can look
reasonable in a dossier (the day/night ends match the stated pairs) and wrong in time (the peak is
under the star). Both halves of a real inertia come from the same physics, so adding only the phase
is a partial model -- but it is a cheap and honest partial, and it is the half that is missing.

The scale of it, using the shipped rotation rates:

| body | day length | a 1 h lag is | Earth-like ~1.5 h lag is |
|---|---|---|---|
| Earth | 23.93 h | 15.0 deg | 22.6 deg east of the subsolar point |
| Mars | 24.62 h | 14.6 deg | 21.9 deg |
| Kerbin | 5.99 h | 60.1 deg | 90.2 deg |
| Titan | 382.7 h | 0.94 deg | 1.4 deg |
| Venus | 5832.6 h | 0.06 deg | 0.09 deg |

Two things fall out of that table. Authoring the lag **in hours** (the unit a source would state --
"the hottest hour is about 14:00") is fine for slow rotators and enormous for Kerbin: the same 1.5 h
that tilts Earth's hot spot 23 degrees swings Kerbin's a full quarter-turn. So the unit choice is a
real decision, not formatting: hours are what a fact sheet says, a fraction of the day is what the
physics rotates.

And for Titan/Venus the lag is *negligible as an angle* because their days are so long -- for a
tidally-slow world the whole "afternoon" question is already smeared over months. That is a useful
negative: the effect is only interesting on fast rotators, which is exactly where our shipped
playground (Kerbin) lives.

## 3. Do NOT derive the lag from `f`

Tempting, because `f` is already a damping knob and one knob is better than two. It does not work.
Reading `f` as the diurnal gain (`gain = 1 - f`) and inverting the linear response gives

```
phi = arcsin( sqrt(2f - f^2) )
```

For Earth (`f = 0.9323`) that is **86.3 degrees** -- nearly a quarter turn -- where the observed
daily-max lag is roughly 1.5 to 2.5 h (23 to 38 degrees). For the Moon (`f = 0.0139`) it gives 9.5
degrees, where an airless body should have essentially none. The reason is that `f` is not a thermal
inertia in this model: it is a *redistribution* knob (the even share of emission), fitted from a
stated night/day pair. On Earth the small day/night difference is the atmosphere moving heat around,
not the ground storing it; on the Moon the low night end is bare radiative cooling, not diffusion.
Conflating the two invents a phase an order of magnitude wrong.

**So the phase needs its own authored number**, or nothing. Which is also the honest answer about
the corpus: the RSS fact sheets state night and day *extremes*, not times of maximum. There is no
`tau` to read.

## 4. The increment that costs nothing to get wrong: a unit-mass response kernel

Here is the formulation I would implement, because it is inert by default and provably keeps the
anchor theorem the whole design rests on.

Convolve the forcing shape with a **normalised causal kernel** over the rotation phase:

```
shape(h) = f + (1 - f) * 4 * (K * g)(h),    g(h) = max(cos(h - h_ss), 0)
K(s) = exp(-s / tau) / tau for s >= 0, unit mass over the periodic phase
```

- **tau = 0 reproduces today's law bit for bit.** No golden moves unless a body authors a tau.
- **The anchor stays exact.** `T^4` averaged over a full rotation is unchanged by convolving with a
  unit-mass periodic kernel -- the same identity that makes `<cos_inc> = 1/4` exact in
  `tests/test_solar.cpp`. So `T0` keeps meaning "the radiation mean", the dossier's swing rows keep
  their arithmetic, and no corpus fit has to be redone.
- **It buys the phase and the smoothing at once**, which is what one physical parameter should do:
  the hot spot moves east *and* the terminator cliff softens. Item 3 in section 1 dies for free.
- **Cost.** The integrand depends only on (latitude, phase), so a paint evaluates
  `nLat x nPhase` samples (64 x 64 for the current map) and reuses them along each latitude row. The
  two readout paths (`[solar]` underfoot, the rung D air coupling) each need one evaluation.

Call sites the knob has to reach (they all take `cos_inc`, so the lag goes in the helper that
produces it): `src/insolation.h:skyAtPlace`/`skyAtZenith` for the hull paths (`src/tick.cpp:679,765`,
`src/vehicle.cpp:3139`) and `src/surfmap.h:220` for the painter. Sentinel in `authored.h` beside
`kEvenShareUnauthored`, so "no tau" means "no lag" and never a zero-filled guess.

What to author, if anything: Earth at ~1.5 h (23 deg) is an `INVENTED:` analogue reading we would
have to say so about. **Kerbin is the one that matters for play** and its answer is not obvious --
90 degrees on a 6-hour day is a big visible change, and an Earth-like atmosphere on a fast rotator
would carry the hot spot along with the co-rotation rather than leaving it a quarter-turn behind. I
would not author Kerbin's tau without seeing it painted.

Filed as **#794**.

## 5. Other effects the field is missing, ranked

1. **Elevation, for bodies with air.** The ground field ignores terrain height entirely -- the cursor
   line prints `elev -2.2 km` next to a temperature that does not care. A mountain should read colder
   by roughly the air's own dry lapse (`atmosphere.air`'s authored lapse, ~6.5 K/km on Earth), and
   the atlas would suddenly show continents with cold spines instead of flat isotherms. Cheap to
   compute; the catch is the anchor: an elevation term moves the area mean, so either the generator
   folds the mean lapse into `T0` or the readout has to say the anchor is at the geoid. Data hook:
   none needed -- the lapse and the terrain amplitude are both already authored.
2. **Cold traps.** Because the night hemisphere is one isotherm, a permanently shadowed polar crater
   reads the same as the anti-solar point, and the Moon's 95 K floor applies everywhere in night.
   Real PSRs sit far below the diurnal night floor because they never see the sun over a whole orbit.
   A `cos_inc` averaged over the body's *season* rather than the instant would give this for free at
   the poles; it is the same kernel machinery applied to the annual cycle.
3. **Internal heat in the anchor.** Jupiter's dossier prints `level - airless mean +55.1 K` and the
   label says "greenhouse" while the mechanism is internal heat (the giant radiates more than it
   absorbs; #738 already objects to the word). An authored `internal_flux_W_m2` added inside the
   fourth root -- `T0^4 = (1-A) S / 4 sigma + F_int / sigma` -- names the mechanism instead of
   leaving a gap that invites a greenhouse reading. Io and Enceladus would be the small-scale
   version. Data hook: the corpus states emitted-vs-absorbed for the giants; the small moons' heat
   flow is stated in some fact sheets.
4. **Seasonal phase lag.** Earth's hottest weeks lag the solstice by a month; that is the same causal
   kernel over orbital phase instead of rotation phase. It also implies the anchor machinery has to
   integrate over a year, which the sim can do. Speculative, but the formulation in section 4 covers
   it without new ideas -- only a second `tau` in days.
5. **Sublimation thermostats.** A night side capped at the frost point of a volatile the air states
   (CO2 on Mars near 148 K, water where humid) would flatten the coldest end realistically. Needs a
   volatiles/reservoir model; note the coincidence that Mars's air table already steps to 148.0 K at
   its second level.
6. **Ocean versus land inertia.** The file already knows Earth has seas (`surface.sea_level`,
   `ocean`); two different `f` (and taus) over sea and land would make the day/night map read like
   weather. Highest visual payoff, worst provenance: nothing states a per-surface-type knob, and it
   breaks the "one knob per body, provenanced" story that makes the swing rows testable.
7. **Air-side transport.** The rung D coupling is pointwise: air follows the ground *under* the hull.
   A hull downwind of a hot region feels nothing of it. Fixing that needs an advected state field,
   which is the heat-as-a-mechanic question #728 is parked on, not a knob.

## 6. What I would do next, in order

1. Land the kernel with `tau` inert everywhere (`tau = 0` path identical to today), plus a unit test
   pinning `<T^4> = T0^4` under a nonzero `tau` -- the theorem, not the number. Nothing visible moves.
2. Paint Earth and Kerbin with a candidate `tau` and *look at them* (ask the user, per project habit)
   before authoring anything.
3. Fix the elevation gap (item 5.1) first among the other effects: it needs no invented physics, only
   the authored lapse, and it is the one a player sees as wrong on the map today.
4. File the internal-heat row (5.3) against #738's greenhouse wording -- it is the one place the
   dossier currently implies a mechanism the data does not support.
