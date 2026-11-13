# Surface temperature: what the field does NOT do yet (lag, and four other missing effects)

Snapshot of `src/groundtemp.h` / `utils/ground_model.py` as of `081ac4ab` (2026-11-12). Companion to
`reports/surface-temp2026_11_12/surface-temp.md`, which describes what the model *does*. This one is
about the gap the user pointed at -- "the temp doesn't change immediately with the sun" -- plus the
other effects I found while looking, each with the cost and the data hook it would need.

Revised in place 2026-11-13 after an outside review of the lag proposal. The code is unchanged --
this is still a snapshot of `081ac4ab` -- but the first draft's kernel was wrong about its own
normalisation, its anchor theorem, its units and its call sites, and several quoted numbers came
from the wrong table. Fixed where they stand rather than appended as corrections.

## 1. What the field is today, exactly

```
T(lat, lon, t) = T0 * [ f + (1 - f) * 4 * cos_inc ] ^ (1/4)
```

`cos_inc` is the floored cosine of the star's elevation at that place, computed at the *instant* the
readout asks. On the map that is `surfmap.h:237` -- `groundTemperature(anchor, f, cosIncidence(dir,
sun_dir))`, one pixel at a time -- and the hull paths take the cosine from `insolation.h:skyAtZenith`
(`tick.cpp:679`, `vehicle.cpp:3242`). Note that `insolation.h:skyAtPlace` -- the named-place wrapper
this field was supposedly built on -- **has no caller in `src/`**; its own comment says the sweep
works from pixel indices instead, and only `tests/test_solar.cpp` exercises it. `f` is the authored
even share. Nothing in the chain has a time constant, and `groundtemp.h` already says so out loud
("a hull at noon feels a noon ground immediately").

Three consequences worth naming, because they are what a player would notice:

1. **The hot spot sits exactly under the star.** The maximum of the field is at the subsolar point,
   to the pixel. On Earth the hottest ground is at local noon; in reality it is mid-afternoon.
2. **The whole night hemisphere is one isotherm.** Anywhere `cos_inc = 0` answers exactly
   `T0 * f^(1/4)`. Measured for the shipped files:

   | body | anchor T0 | f | night end (calc) | day end (calc) | atlas prints | what the page states |
   |---|---|---|---|---|---|---|
   | Earth | 288.0 | 0.9323 | 283.0 | 301.6 | `283.0 to 301.6 K` | "Diurnal temperature range: 283 K to 293 K (10 to 20 C)" |
   | Moon | 276.50 | 0.01394 | 95.0 | 390.0 | `95.0 to 390.0 K` | a 95-390 K range |
   | Kerbin | 288.15 | 0.9323 | 283.1 | 301.8 | `283.1 to 301.8 K` | invented world, no page |

   (The two printed rows come from `--atlas-dump`; the dossier row is the same arithmetic.) Earth's
   `f` is `(283/288)^4` exactly -- fitted from the **night** end against the stated mean -- so the
   day end is a *prediction*, and it lands **8.6 K above the maximum the page states**. The page is
   quoting 2 m *air*, 10 to 20 C; the model reads it as a ground radiometer. Keep that in view in
   section 4, because it is exactly the gap a lag would be tempted to absorb.

   So there is no *pre-dawn minimum* -- the coldest moment is anywhere in the night, equally -- and
   no cold-trap credit for a permanently shadowed pole: it reads the same as the anti-solar point.
3. **The terminator is a cliff.** At the day edge (`cos_inc -> 0+`) the gradient along longitude is
   `T0 (1-f) f^(-3/4)` kelvin per radian. Measured on the shipped numbers: **Earth 0.36 K per degree**
   of longitude, **Kerbin 0.36**, **the Moon 117 K per degree**. The air worlds look smooth because
   their `f` is near 1; the airless ones have a wall where the day begins. Real regolith conducts
   across that band and would soften it -- but note that the map cannot *show* the wall as it stands:
   the bake defaults to `--surfmap-n 512` (`cli.h:139`), so a pixel is 0.70 deg of longitude, and the
   Moon's 117 K per degree puts **~82 K inside a single sample**. Where the cliff is concerned the
   picture is an alias, not a measurement -- which matters for any claim about how much a smoothing
   kernel would change what the player sees.

## 2. The user's instinct, in physics terms

A surface with heat capacity `C` obeys `C dT/dt = (1-A) S g(t) - eps sigma T^4`. Linearised about the
mean, the response to the diurnal harmonic has **one** parameter, the thermal time constant
`tau_th = C / (4 eps sigma T0^3)`, and it shows up as **two** effects at once:

- amplitude damping, gain `1 / sqrt(1 + (omega tau_th)^2)`, and
- a **phase lag**, `phi = atan(omega tau_th)`, i.e. the peak sits at local time `12h + tau_lag`
  with `tau_lag = phi / omega`.

The current model has an empirical damping knob (`f`) and *no* phase. That is why it can look
reasonable in a dossier (the ends land near the stated pairs -- for Earth, inside the generator's
8% day-end gate, `ground_model.DAY_PREDICTION_GATE`) and wrong in time (the peak is under the star).
Both halves of a real inertia come from the same physics, so adding only the phase is a partial
model -- but it is a cheap and honest partial, and it is the half that is missing.

The scale of it, using the shipped rotation rates -- which are **sidereal**, because that is what
`rotating.rot_ang_speed` holds. A lag is a phase of the *subsolar* cycle, so the number to divide an
authoring hours-figure by is the solar day. Earth and Mars differ by under 1%; Venus differs by a
factor of two, because its 5832.6 h sidereal period is retrograde (`axial_tilt` 177.4 deg) against a
224.7 d orbit, and its subsolar day is 116.75 d:

| body | authored sidereal | subsolar day | a 1 h lag is | Earth-like ~1.5 h lag is |
|---|---|---|---|---|
| Earth | 23.93 h | 24.00 h | 15.0 deg | 22.6 deg east of the subsolar point |
| Mars | 24.62 h | 24.66 h | 14.6 deg | 21.9 deg |
| Kerbin | 5.99 h | 6.00 h | 60.1 deg | 90.2 deg |
| Titan | 382.7 h | 382.7 h (synchronous) | 0.94 deg | 1.4 deg |
| Venus | 5832.6 h | 2802 h (116.75 d) | 0.13 deg | 0.19 deg |

The degrees-per-hour columns use the **subsolar** day, because that is the cycle a lag is a phase of.
Earth, Mars and Kerbin hardly move between the two periods; Titan's are equal because it is
synchronous; Venus's differ by a factor of two, and using the sidereal figure there would halve the
lag anyone meant by "an hour".

Two things fall out of that table. Authoring the lag **in hours** (the unit a source would state --
"the hottest hour is about 14:00") is fine for slow rotators and enormous for Kerbin: the same 1.5 h
that tilts Earth's hot spot 23 degrees swings Kerbin's a full quarter-turn. So the unit choice is a
real decision, not formatting: hours are what a fact sheet says, a fraction of the day is what the
physics rotates. (Section 4 adds that the quarter-turn is not reachable anyway.)

On Titan and Venus the *angle* is negligible -- `phi = atan(omega tau)`, and a slow world simply does
not sweep far during a given `tau`. That is not the same as the effect being small: ten degrees on
Venus is 78 hours of afternoon. "Slow rotator, so ignore it" is only a statement about what the map
looks like, and it is the wrong reason; the right reason to leave those bodies alone is that we would
be inventing the `tau` along with the knob.

One mismatch to keep honest between this section and the next. The equation above is a heat balance in
`T`, and its linearisation gives `tau_th = C / (4 eps sigma T0^3)` -- which with a fixed areal `C`
*changes with temperature*, and that is part of why a lunar night cools faster than one RC time says.
Real regolith is stiffer than the lumped model again: heat diffuses, so the response is not one
exponential but a diffusion wave with skin depth `d = sqrt(2 kappa / omega)` -- damping as
`sqrt(omega)` rather than `1/omega`, about 45 deg of surface phase, with two depths of material
before a 90 deg lag shows up at all. The increment in section 4 filters the `T^4` *shape* this law
already works in, not `T`, so its effective `C` goes as `T^3` -- a different model, and not an
inferior one, but not the one this section derived. Say which is being authored instead of borrowing
`tau_th`'s physics for a filter that does not have it.

## 3. Do NOT derive the lag from `f`

Tempting, because `f` is already a damping knob and one knob is better than two. It does not work.
Reading `f` as the diurnal gain (`gain = 1 - f`) and inverting the linear response gives

```
phi = arcsin( sqrt(2f - f^2) )        (equivalently arccos(1 - f))
```

For Earth (`f = 0.9323`) that is **86.1 degrees** -- nearly a quarter turn -- where the observed
daily-max lag is roughly 1.5 to 2.5 h (23 to 38 degrees). For the Moon (`f = 0.0139`) it gives 9.6
degrees, where an airless body should have essentially none. The reason is that `f` is not a thermal
inertia in this model: it is a *redistribution* knob (the even share of emission), fitted from a
stated night/day pair. On Earth the small day/night difference is the atmosphere moving heat around,
not the ground storing it; on the Moon the low night end is bare radiative cooling, not diffusion.
Conflating the two invents a phase an order of magnitude wrong.

**So the phase needs its own authored number**, or nothing. That is also the honest thing to say
about *our* corpus: the fact sheets state night and day *extremes*, not times of maximum, and no file
in `res/systems/` carries a thermal-inertia row. There is no `tau` to read **from what we already
scrape** -- which is not the same as the number being unobtainable. Published **thermal inertia**
`Gamma` [W m^-2 K^-1 s^-1/2] exists as global maps for the Moon and Mars, and `tau_th` follows from
`Gamma` and the rotation rate. So the data hook for a provenanced `tau` is a new scraped column plus
a conversion -- `utils/ground_model.py` is where that conversion would live, and it is the same
provenance discipline the swing knob already runs (`even_share_src` beside the number, an `INVENTED:`
mark when there is no page). Reading it off `f` is the one path that is certainly wrong.

## 4. The increment: a causal response kernel, and the traps in it

Here is the formulation. It stays inert by default, but the first draft of this section overclaimed
about its own normalisation, its anchor theorem, its units and its call sites. Each of those is the
kind of error that survives review as a silent drift in a readout, so they are written out here.

Convolve the forcing shape with a causal kernel over the rotation phase:

```
shape(h) = f + (1 - f) * 4 * (K * g)(h),    g(h) = max(cos(h - h_ss), 0)
K(s) = exp(-s / tau) / tau for s >= 0
```

What it buys: the hot spot moves east **and** the terminator cliff softens, from one parameter, which
is what one physical parameter should do -- section 1's item 3 shrinks for free. Everything below is
about the parts of that sentence that are easy to get wrong.

- **`tau` is in phase radians, not seconds.** One rotation is `2 pi` of `h`, and that is the only unit
  in which the kernel is dimensionless. To author it in hours -- which is what a fact sheet would
  say, "the hottest hour is about 14:00" -- convert with the body's *subsolar* day:
  `tau_h = tau * P_subsolar / (2 pi)`. Section 2's Venus row is the warning: use the sidereal period
  there and the figure is out by a factor of two.
- **`tau = 0` needs its own branch.** As `tau -> 0` the kernel tends to a delta, but numerically it is
  a spike of width `tau` sampled at whatever step the sweep happens to use, which is not the identity.
  "Bit for bit what we do today" is only true behind an explicit `tau == 0 -> g` shortcut.
- **The kernel as written is not unit mass over a wrapped period.** Its integral over all `s >= 0` is
  1; over one cycle of phase it collects only `1 - e^(-2 pi / tau)` -- `0.9999996` at `tau = 0.42`,
  and **`0.877` at `tau = 3`**. Ship the wrap with that mass divided back out, or skip the
  convolution and solve the ODE the kernel is the Green's function of:

  ```
  tau y' + y = g(h), y periodic    ->    day:   y = (cos h + tau sin h) / (1 + tau^2) + P e^{-h/tau}
                                         night: y = N e^{-h/tau}
  ```

  with `P` and `N` from continuity at the terminators and `y(pi) = y(-pi)`. That is closed form, so a
  readout costs O(1) rather than a quadrature per pixel -- the `nLat x nPhase` sample table the first
  draft proposed is only worth building for a non-exponential kernel. The figures in this section come
  from that closed form, **cross-checked against a direct RK4 relaxation over 60 rotations**: the two
  agree to 0.01 deg in phase and better than 4e-7 in amplitude, the rotation mean lands on `1/pi` to
  3e-6, the terminator jumps are ~1e-10 and the ODE residual ~2e-9 (`tmp/lagcheck4_20261113.py`). The
  first pass at this dropped a `(1 + tau^2)` in the derivative used to locate the peak and was 3 deg
  off at `tau = 3`; the cross-check is what caught it, which is the argument for keeping it.
- **What the anchor theorem really says, and what the test has to be.** A normalised periodic kernel
  preserves the *rotation mean at a fixed place*: `mean(K*g) = mean(g) = 1/pi`. The sphere identity
  `<cos_inc> = 1/4` is a **second and separate** step, and it only follows when latitude means are
  averaged with the `cos(lat)` weight -- the same weighting `groundtemp.h` warns the surface map's
  pixel mean omits (4% low). So the test for this increment is: the **cos(lat)-weighted sphere
  average of the lagged field over a full rotation**, and for a tilted body over a full orbit. Do not
  write it as "the average of `y` equals the average of `g`" and call the anchor pinned; that only
  bounds one latitude. And be aware that with obliquity an *instantaneous* global mean can move while
  the annual one holds -- anchor the claim, and the test, to the annual figure.
- **`g` is the zero-obliquity equator form, and that is the least portable part of the sketch.** At
  latitude `lat` with the star at declination `dec` the daily forcing is `max(a + b cos h, 0)` with
  `a = sin lat sin dec`, `b = cos lat cos dec`. The closed form survives that substitution (`a` rides
  through untouched, `b` scales the cosine term); `cos(h - h_ss)` does not, and it misplaces the
  terminator at every latitude where the star is not overhead. Slope and aspect -- item 5.2 -- belong
  inside `g`, and the floor has to be taken **before** convolving: self-shadowing is a property of the
  forcing, which no filter can invent afterwards.
- **The lag saturates, and amplitude is what it costs.** Measured on the closed form (Earth's shipped
  knobs in the last two columns, where "night end" is the coldest point of the cycle):

  | tau (rad) | peak lag | peak of `K*g` | Earth day end | Earth night end |
  |---|---|---|---|---|
  | 0.10 | 5.7 deg | 0.995 | 301.5 K | 283.0 K |
  | 0.42 | 22.5 deg | 0.924 | 300.3 K | 283.0 K |
  | 1.00 | 40.7 deg | 0.759 | 297.4 K | 283.5 K |
  | 3.00 | 59.8 deg | 0.503 | 292.8 K | 286.2 K |
  | 100 | 71.1 deg | 0.324 | 289.4 K | 289.2 K |

  The lag **saturates at `arccos(1/pi) = 71.4 deg`** -- the phase where the forcing has fallen to its
  own rotation mean, past which the filter can only be heading down -- so a quarter-turn lag is not
  available at any `tau`. That matters because section 2's table wanted 90.2 deg on Kerbin (1.5 h of
  its 5.99 h day): the honest version of "a 1.5 h lag on Kerbin" is "the diurnal cycle flattens
  away". A 60 deg lag -- **one Kerbin hour** -- needs `tau ~ 3.05 rad`, which halves the day-side peak
  forcing (`K*g` 0.500) and lifts the night floor by 3 K. `K*g`'s peak is below 1 for every nonzero
  `tau`, so the day end always drops.
- Free with the phase: the **coldest moment stops being a whole hemisphere.** With `tau` nonzero the
  cycle has one minimum, sitting at sunrise (`min(K*g)` at -90.0 deg for `tau = 0.1`, drifting into
  the morning as the filter slows: -88.7 deg at `tau = 1`, -80.9 deg at `tau = 3`). Section 1's item 2
  complains that every night point reads the same temperature; a lag answers that part of the
  complaint unasked. Note it lands at *sunrise*, while the real pre-dawn minimum is a little before
  it -- that last step belongs to the air, not the ground.
- **"No corpus fit has to be redone" is false, and that is the expensive part.** `f` and `tau` both
  set amplitude, so a body that authors a `tau` either gets refit or its swing rows stop agreeing
  with the night/day pairs that priced them. Earth makes the trap vivid: the page tops out at 293 K,
  our field says 301.6 K, so a lag *looks* like the fix. It would take `tau = 2.83 rad` to print
  293 K at the shipped `f` -- a **59 deg, 3.9 hour** lag, and it would lift the night end from 283.0
  to 286.0 K, breaking the end the fit is pinned to. Earth's ground does not peak four hours after
  noon. The 8.6 K gap is the provenance of the source row (section 1: a station's 2 m air maximum
  against our subsolar point -- `ground_model.py`'s own comment on `DAY_PREDICTION_GATE` says exactly
  this), not a phase effect, and a kernel must not be allowed to absorb it.
- **The readouts have to move with the field.** `gameui.cpp:7436` prints the dossier's swing row as
  `groundTemperature(anchor, f, 0.0)` and `(anchor, f, 1.0)`, and `surfmap.h:208` builds the legend's
  `lo`/`hi` the same way. Those are the two ends of the **un-lagged** cosine. With `tau` nonzero they
  print numbers the field never reaches, so the swing row and the legend have to ask the lagged helper
  for its own extremes. That is also the mechanical reason a lag cannot be a pure add-on: three
  readouts change behaviour even when nothing is authored.
- **Do not lag the geometry.** The first draft named `insolation.h:skyAtPlace` / `skyAtZenith` as the
  place to put the kernel, which is wrong and would have done damage. `skyAtZenith`'s cosine feeds
  the `[solar]` log (`tick.cpp:679`), the calendar's subsolar point and the at-a-glance elevation,
  and `cosIncidence` feeds `solar.h:incidentFlux` -- a hull's **panel flux**, which must stay
  instantaneous. Only one of those consumers wants history: the ground under the hull
  (`vehicle.cpp:3242`'s rung D coupling). So keep `cos_inc` honest and put the lag in a
  ground-temperature helper that only the surface field (`surfmap.h:237`), the underfoot readout
  (`tick.cpp:765`) and rung D's ground term ask. Sentinel in `authored.h` beside
  `kEvenShareUnauthored`, so "no tau" means "no lag" and never a zero-filled guess.
- **Statelessness is a feature with a cost.** A convolved field stays a pure function of (place,
  time): nothing accumulates, so a save/load cannot drift and a paused game cannot desync. The price:
  it has no memory of anything that is not periodic in `h`, so an eclipse, a body dropped into a new
  orbit, or a `tau` changed mid-flight produces no transient. If eclipse response ever matters, that
  is a stateful field -- a different model, not a longer kernel.

What to author, if anything: Earth at ~1.5 h (23 deg) is an `INVENTED:` analogue reading we would have
to say so about, and at that `tau` the day end moves 1.3 K -- below what any readout can defend.
**Kerbin is the one that matters for play**, and its answer is bounded rather than obvious: its 5.99 h
day turns 60 deg per hour, so the usable range is `tau` under about 1 rad before the cycle is
flattened past recognition, and an Earth-like atmosphere on a fast rotator would carry the hot spot
along with the co-rotation rather than leaving a quarter-turn behind. I would not author Kerbin's
`tau` without seeing it painted.

Filed as **#794**.

## 5. Other effects the field is missing, ranked

1. **Elevation, for bodies with air.** The ground field ignores terrain height entirely -- the cursor
   line prints `elev -2.2 km` next to a temperature that does not care. The mechanism worth naming is
   the anchor: `groundAnchor_K` takes the air's **base** temperature (the sea-level datum), so the
   ground on a 5 km ridge swings around the sea-level number and reads tens of kelvin too warm. The
   fix is not a new term -- it is evaluating the authored profile **at the terrain height** and
   swinging around that, which the profile already supports. Two cautions once it does. The authored
   lapse is an *environmental* value (`surface.atmosphere.layers[0].lapse`: Earth `-0.00648878`,
   Kerbin `-0.00638083`, both ~6.4 K/km), not the dry adiabatic `g/cp` ~9.8 K/km, so the readout has
   to say which is meant. And rung D
   (`AtmosphereParams::air_follow`) already pulls the air toward the ground under a hull, so a ground
   anchored at altitude plus a coupled air can count the column twice: fix the order -- ground
   anchored at terrain height, air follows it -- and pin it with a test. Data hook: none needed -- the
   lapse and the terrain amplitude are both already authored. Filed as **#821**, because unlike the
   rest of this list it is wrong *today*: on Kerbin the authored profile puts the air at the tallest
   summit ~27 K under the datum, and the ground there still reads the sea-level swing.
2. **Cold traps.** Because the night hemisphere is one isotherm, a permanently shadowed polar crater
   reads the same as the anti-solar point, and the Moon's 95 K floor applies everywhere in night. The
   shipped numbers for that floor are worth seeing: at Earth's pole the field reads **283.0 K at the
   equinox and 283.0 K at the northern winter solstice** (290.8 K at the summer solstice) -- a polar
   winter sitting exactly on the global night floor, which is a statement about `f`, not about that
   place. Real PSRs sit far below the diurnal floor because they never see the star over an orbit.
   **This is not the kernel's job, and the earlier draft of this section said it was.** A lag is
   linear: inside a permanently shadowed region the *forcing* is identically zero -- `g == 0` at every
   phase -- so any convolution of it is still zero, and the region reads whatever the floor says. The
   missing piece is the forcing: a horizon / self-shadow query against the baked heightfield, so that
   `g` goes to zero where the rim blocks the star. With `g` right, a seasonal average is a sensible
   second step (it damps what little light does arrive) and an annual-mean floor keeps the crater from
   reading 0 K. Cost lands at paint time, when the heightfield is already in memory; nothing at
   readout time.
3. **Internal heat in the anchor.** Jupiter's dossier prints `level - airless mean +55.1 K` and the
   label says "greenhouse" while the mechanism is internal heat (the giant radiates more than it
   absorbs; #738 already objects to the word). An authored `internal_flux_W_m2` added inside the
   fourth root -- `T0^4 = (1-A) S / 4 sigma + F_int / sigma` -- names the mechanism instead of leaving
   a gap that invites a greenhouse reading. Before that formula is trusted, though, notice that **`T0`
   in this corpus is not one quantity.** Earth's anchor 288.0 K is an air datum that already contains
   the unmodelled greenhouse -- the atlas weighs it against `black-body temp 255.1 K` and prints the
   difference as a `greenhouse gap +32.9 K` row -- while the Moon's 276.5 K was *solved* from its
   95-390 K pair (`ground_model.pair_solve`), making it an emission equivalent rather than a measured
   mean. A flux added inside that root therefore means a different thing on every body unless the row
   says which level it prices. And Jupiter's +55.1 K is the difference between a stated level
   temperature and an airless mean -- for a giant that is mostly the drop from the emission level to
   the 1-bar level, a **lapse** -- so on a body that has an authored profile, a `F_int` term and that
   profile both describe the same heat and the giant pays for it twice. The knob is still worth
   having; the row has to name its level. Data hook: the corpus states emitted-vs-absorbed for the
   giants; the small moons' heat flow is stated in some fact sheets.
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
   breaks the "one knob per body, provenanced" story that makes the swing rows testable. Note it
   breaks the anchor exactly the way a `tau` does: with two knobs the radiation mean is the
   area-weighted mix of two shapes, so `T0` stops being the body's mean unless the generator weights
   the two surfaces -- and the area split has to come from the same `ocean` data the map paints with,
   not from a guessed fraction.
7. **Air-side transport.** The rung D coupling is pointwise: air follows the ground *under* the hull.
   A hull downwind of a hot region feels nothing of it. Fixing that needs an advected state field,
   which is the heat-as-a-mechanic question #728 is parked on, not a knob.

## 6. What I would do next, in order

1. Settle the authoring story before writing a line of the kernel: `tau`'s unit (phase radians, with
   the subsolar-day conversion written next to it), where a provenanced value would come from
   (`Gamma` -> `tau_th`, a new scraped column), and the rule for a body that has both `f` and `tau` --
   refit `f`, because two amplitude knobs against one published pair is overdetermined. Writing that
   down is cheap; retrofitting it after three bodies carry a `tau` is not.
2. Land the kernel inert, with an explicit `tau == 0` branch so the identity is real rather than
   asymptotic, and with the test stated correctly: the **cos(lat)-weighted sphere average of the
   lagged field over a full rotation** still equals `T0^4`. Pin the phase against the closed form the
   same way `tmp/lagcheck4_20261113.py` pins it -- closed form against a relaxation -- because the
   first attempt at that arithmetic was quietly 3 deg wrong. Nothing visible moves.
3. In the same change, put the atlas's swing row and the surface map's legend behind the lagged helper
   (`gameui.cpp:7436`, `surfmap.h:208`). They print `cos_inc = 0` and `1` today, so the moment a
   `tau` exists anywhere they print extremes the field cannot reach -- and that is the failure mode
   where the map and its own legend disagree in front of the player.
4. Paint Earth and Kerbin with a candidate `tau` and *look at them* (ask the user, per project habit)
   before authoring anything. Expectation to check against, not hope: at a 23 deg lag Earth's day end
   moves 1.3 K, which no readout will notice; the interesting case is Kerbin, where one hour of the
   dial is 60 deg and that costs half the day-side peak forcing.
5. Fix the elevation gap (item 5.1) first among the other effects -- it is an anchor bug more than a
   missing law, needs no invented physics, and is what a player sees wrong on the map today. Then cold
   traps (5.2), which need a horizon/self-shadow query rather than any kernel; keep them filed
   separately so nobody implements the lag and thinks the poles came with it.
6. File the internal-heat row (5.3) against #738's greenhouse wording, with the level named -- it is
   the one place the dossier currently implies a mechanism the data does not support.
