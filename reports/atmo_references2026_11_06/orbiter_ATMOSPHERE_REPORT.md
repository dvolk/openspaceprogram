# Orbiter atmosphere & atmospheric flight model — reference report

source available in ~/references

Purpose: a compact, exact reference of how Orbiter (main branch, Nov 2026 checkout,
commit `ce328588`) models planetary atmospheres and atmospheric flight, for
comparison against a Pioneer!/KSP-style simulation.

All statements below were read out of the source in this checkout. File:line
references are given so each claim can be checked. Units are SI unless noted.

---

## 1. Scope & provenance

| Area | Files |
|---|---|
| Core atmosphere plumbing | `Src/Orbiter/Planet.{h,cpp}`, `Src/Orbiter/Celbody.cpp` |
| Atmosphere model plugins | `Src/Celbody/Vsop87/{Earth,Mars,Venus}/Atmosphere/*` |
| Vessel flight model | `Src/Orbiter/Vessel.cpp`, `Src/Orbiter/Vesselbase.cpp` |
| Public API | `Orbitersdk/include/CelBodyAPI.h`, `Orbitersdk/include/OrbiterAPI.h` |
| Model selection UI | `Src/Plugin/LaunchpadExtensions/AtmConfig/AtmConfig.cpp` |
| Design docs | `Doc/Orbiter Technical Reference/earth_atm.tex` (2009), `Doc/Orbiter Developer Manual/PLANETS.tex` |

---

## 2. Data flow

```
Config/<Body>.cfg  ─┐
Config/<Body>/Atmosphere.cfg (MODULE_ATM = <dll>) ─┘
        │
        ▼
CELBODY2 owns an ATMOSPHERE object (loaded DLL, or nil)
        │  clbkConstants(ATMCONST*)   → p0, rho0, R, gamma, altlimit, horizonalt, color0
        │  clbkParams(PRM_IN*, PRM_OUT*) → T, p, rho      ← ONLY these three outputs
        ▼
Planet::GetAtmParam(alt, lng, lat, ATMPARAM*)     Planet.cpp:835-879
        │  dispatch on AtmInterface ∈ {0,1,2,3,4}
        ▼
SurfParam::Set()  Vesselbase.cpp:43-99
        │  altitude, ground velocity, wind velocity, airspeed
        │  dynp = ½ρV²,  Mach = V/a,  a = sqrt(γRT)
        ▼
Vessel::UpdateAerodynamicForces()  Vessel.cpp:4099-4229
        │  per-airfoil CL/Cm/CD = f(aoa, M, Re); control surfaces; drag elements; angular damping
        ▼
rigid-body integration
```

Key structural point: **the atmosphere is a model plug-in, not a data set.** A
body either has (a) a compiled model DLL, (b) a two-number exponential profile in
the `.cfg`, or (c) no atmosphere. There is no layer table, profile curve, or
weather config file that a user can edit as data.

---

## 3. `AtmInterface` dispatch table

`Planet::AtmInterface` (`Planet.h:193-198`), set in `Planet.cpp:216-243`:

| Value | Source of `T,p,ρ` | Formula / note |
|---|---|---|
| 0 | none | no atmosphere |
| 1 | `.cfg`: `AtmPressure0`, `AtmDensity0` | `T = 288.16` **hard-coded**; `p = p0·exp(−C·alt)`, `ρ = p·ρ0/p0`, with `C = ρ0·g/p0` (`Planet.cpp:660, 843-846`) → single isothermal scale height derived from the body's own gravity |
| 2 | obsolete `Planet_AtmPrm` module fn | legacy |
| 3 | `CELBODY::clbkAtmParam(alt, …)` | legacy |
| 4 | `CELBODY2` → `ATMOSPHERE::clbkParams` | current; used by Earth, Mars, Venus |

Config values **override model constants** after the module reports them
(`Planet.cpp:245-277`): `AtmAltLimit`, `AtmAttenuationAlt`, `AtmHorizonAlt`,
`AtmGasConstant`, `AtmGamma`, `AtmHaze*`, `AtmColor0`, `AtmFogParam`,
`AtmTintColor`. Example: `MarsAtm2006` reports `altlimit = 100 km`, but
`Mars.cfg:43` sets `AtmAltLimit = 280e3`, so 280 km wins.

`atm.radlimit = size + altlimit` (`Planet.cpp:661`) — the "in atmosphere" test is
purely radial: `rad < planet->AtmRadLimit()` (`Vesselbase.cpp:90`).

---

## 4. Bodies with atmospheres in the stock distribution

| Body | Model | p0 [Pa] | ρ0 [kg/m³] | R [J/kg·K] | γ | cutoff |
|---|---|---|---|---|---|---|
| Earth | `EarthAtmJ71G` (default), also `EarthAtm2006`, `EarthAtmNRLMSISE00` | 101325 | 1.2247 | 286.91 | 1.40 | 2500 km |
| Mars | `MarsAtm2006` | 610 | 0.02 | 188.92 | 1.2941 | 280 km (cfg) |
| Venus | `VenusAtm2006` | 9.2e6 | 65 | 188.92 | 1.2857 | 600 km (cfg) |
| Jupiter | mode 1 | 2.014e6 | 1.3293 | 194.92 | 1.3333 | **3200 km** (`; raised to enable aerobrake capture`) |
| Saturn | mode 1 | 7.54e5 | 0.7293 | 174.92 | 1.3333 | 2900 km |
| Uranus | mode 1 | 4.54e5 | 0.5293 | 194.92 | 1.3333 | 2600 km |
| Neptune | mode 1 | 4.14e5 | 0.5093 | 144.92 | 1.3333 | 1800 km |
| Titan | mode 1 | 1.46e5 | 6.131 | 285.92 | 1.3941 | 600 km |
| Triton | mode 1 | 1.45 | 1e-4 | 297 | 1.40 | 20 km |
| Io | mode 1 | 1e-4 | 1e-4 | **1.92** | **0.3333** | 20 km |

Implicit surface gravities recoverable from the models' `g0/R` constants
(cross-check these against your own gravity): Earth `g0R = 0.0341643` →
9.80 m/s²; Mars `g0R = 0.0197275` → 3.726 m/s²; Venus `g0R = 0.0469506` →
8.869 m/s². Isothermal scale heights implied at the reference level:
Earth ≈ 8.4 km, Mars ≈ 9.9 km, Venus ≈ 16.0 km.

### Consistency check you should steal

For a body with a layered model, Orbiter's numbers pass the ideal-gas sanity
check `T_implied = p0 / (ρ0·R)`: Earth → 288.7 K, Venus → 749 K, Mars → 161 K
(vs. 195 K profile). The mode-1 bodies mostly do not, because their density
scale height `H = p0/(ρ0·g)` is set independently of their `R`, `γ`, and of the
hard-coded 288.16 K that `GetAtmParam` reports:

| Body | `T_implied = p0/(ρ0·R)` | T reported by `GetAtmParam` |
|---|---|---|
| Jupiter | ≈ 7770 K | 288.16 K |
| Saturn | ≈ 5910 K | 288.16 K |
| Uranus | ≈ 4400 K | 288.16 K |
| Neptune | ≈ 5610 K | 288.16 K |
| Titan | ≈ 83 K (real ≈ 94 K) | 288.16 K |
| Triton | ≈ 49 K (real ≈ 38 K) | 288.16 K |
| Io | ≈ 521 K | 288.16 K |

Consequence: on those bodies the drag density and the speed of sound are derived
from mutually inconsistent states, so `M = V/a` is meaningless even where `q` is
usable. Io's `R = 1.92` / `γ = 0.3333` (γ < 1 is unphysical) look like decimal
typos for SO₂ (R ≈ 197), and its `p0 = ρ0 = 1e-4` gives a scale height of
`1/g ≈ 0.56 m`. Also note `Triton.cfg`/`Io.cfg` set `AtmHorizonAlt` (7 km) below
or above their `AtmAltLimit` inconsistently (Io: horizon 30 km > limit 20 km).

---

## 5. Profile mathematics (mode 4 layer models)

All three 2006-era models are piecewise profiles in **geopotential** altitude:

```
h = z · R_body / (R_body + z)          // geometric → geopotential
isothermal layer:  p = p_base · exp( −(g0/R/T_base)·(h − h_base) )
lapse layer:       T = T_base + a·(h − h_base)
                   p = p_base · (T/T_base)^( −g0/(a·R) )
                   ρ = ρ_base · (T/T_base)^( −g0/(a·R) − 1 )      [lapse]
                   ρ = ρ_base · exp( −(g0/R/T_base)·(h−h_base) )  [isothermal]
```

Documented in `Doc/Orbiter Technical Reference/earth_atm.tex:170-190`.

### 5.1 Earth — 2006 legacy model (`EarthAtm2006.cpp:28-90`)

`R_body = 6.37101e6`, `g0/R = 0.0341643`.

| z (geopot.) | T [K] | lapse [K/km] | p at layer base [Pa] | ρ at layer base |
|---|---|---|---|---|
| 0 – 11 km | 288.16 − 6.5z | −6.5 | 1.01325e5 | 1.2250 |
| 11 – 25 km | 216.66 | 0 | 2.26323e4 | 0.36392 |
| 25 – 47 km | 216.66 + 3.0(z−25) | +3.0 | 2488.726 | 0.0400179 |
| 47 – 53 km | 282.66 | 0 | 120.4524 | 0.00148459 |
| 53 – 79 km | 282.66 − 4.5(z−53) | −4.5 | 58.32623 | 0.000718878 |
| 79 – 90 km | 165.66 | 0 | 1.009601 | 2.12319e-5 |
| 90 – 105 km | 165.66 + 4.0(z−90) | +4.0 | 0.1044559 | 2.19671e-6 |
| 105 – 200 km | 225.66 | 0 | 0.007454273 | 1.15082e-7 |

This is a *smoothed approximation* of US76: it merges the 20/32 km discontinuity
into one 25 km breakpoint with +3.0 K/km, and uses 53/79/90/105 km instead of
US76's 51/71/84.8 km.

### 5.2 Earth — US76 layer table inside J71G (`EarthAtmJ71G.cpp:176-253` temperature, `269-353` density)

Used by `EarthAtmJ71G` below 90 km. (`EarthAtmNRLMSISE00` does **not** use this
table — it has its own spline-node lower-atmosphere scheme, see §6.)
`h = z·6369/(z+6369)`, `g0/R = 34.163195` K·km⁻¹, pressure bases relative to 1 atm:

| z [km] | T_base [K] | lapse [K/km] | p_base (atm) |
|---|---|---|---|
| 0 | 288.15 | −6.5 | 1.0 |
| 11 | 216.65 | 0 | 2.233611e-1 |
| 20 | 216.65 | +1.0 | 5.403295e-2 |
| 32 | 228.65 | +2.8 | 8.5666784e-3 |
| 47 | 270.65 | 0 | 1.0945601e-3 |
| 51 | 270.65 | −2.8 | 6.6063531e-4 |
| 71 | 214.65 | −2.0 | 3.9046834e-5 |
| 85 – 91 | 188.0 | — | column-mass integration with a fudged mean-molecular-weight profile (`EarthAtmJ71G.cpp:338-375, 392-403`) |
| > 91 | Jacchia-77 `T(z, T∞)` | — | Gill polynomial fit |

### 5.3 Mars — `MarsAtm2006.cpp:29-77`

`R_body = 3.38992e6`, `g0/R = 0.0197275`, `R = 188.92` (CO₂).

| z [km] | T [K] | lapse [K/km] | p at base [Pa] | ρ at base |
|---|---|---|---|---|
| 0 – 2 | 195 + 2.5z | +2.5 | 610.0 | 0.02 |
| 2 – 4 | 200 | 0 | 499.534 | 1.59687e-2 |
| 4 – 14 | 200 − 2.0(z−4) | −2.0 | 410.1 | 1.31097e-2 |
| 14 – 20 | 180 | 0 | 145.061 | 5.1524e-3 |
| 20 – 30 | 180 − 1.5(z−20) | −1.5 | 75.1564 | 2.66947e-3 |
| 30 – 280 | 165 | 0 | 23.9322 | 9.27323e-4 |

### 5.4 Venus — `VenusAtm2006.cpp:26-70`

`R_body = 6.05184e6`, `g0/R = 0.0469506`, `R = 188.92`.

| z [km] | T [K] | lapse [K/km] | p at base [Pa] | ρ at base |
|---|---|---|---|---|
| 0 – 30 | 750 − 9.0z | −9.0 | 9.2e6 | 65 |
| 30 – 60 | 480 − 8.33(z−30) | −8.33 | 8.967687e5 | 9.89979 |
| 60 – 70 | 230 | 0 | 1.421879e4 | 3.274408e-1 |
| 70 – 90 | 230 − 2.5(z−70) | −2.5 | 1846.393 | 4.252010e-2 |
| 90 – 600 | 180 | 0 | 18.49556 | 5.442431e-4 |

---

## 6. Earth upper atmosphere & space weather

Three selectable models (Launchpad → Extras → *Atmosphere Configuration*, which
only writes `MODULE_ATM` — `AtmConfig.cpp:97-111, 199-241`):

**`EarthAtm2006`** — static, cutoff 200 km. Underestimates ρ/p above ~120 km
(documented as a feature: "simplifies orbit-keeping for LEO objects").

**`EarthAtmJ71G`** — Jacchia-71/77 with Gill's bi-polynomial density fit
(`log10 ρ = ΣΣ c_ij z^i T∞^j`, fit over 90–2500 km and T∞ ∈ [500,1900] K, max
error 1.6–3.8 %, `EarthAtmJ71G.cpp:405-450`). Below 90 km it delegates to US76.
Exospheric temperature (`EarthAtmJ71G.cpp:71-140`):

```
Tc    = 379 + 3.24·F̄10.7                    // = 832.6 K at F̄ = 140
T_L   = Tc · (1 + 0.3·(sin^2.2|θ| + (cos^2.2|η| − sin^2.2|θ|)·cos^3(τ/2)))   // diurnal
ΔT∞   = 14·Kp + 0.02·e^Kp  (z<350 km) → 28·Kp + 0.03·e^Kp (z>350 km), tanh blend
T∞    = T_L + ΔT∞
ρ     = 10^(dens_fit(z,T∞) + d_Gm + d_Sa + d_Sl) + d_He   // He added separately
p     = ρ·(N_A·k_B)/M(z,T∞)·T
```

**`EarthAtmNRLMSISE00`** — the full Picone/Hedin `gtd7` C code vendored in
(`nrlmsise-00.c`, `nrlmsise-00_data.c`). Orbiter feeds it day-of-year,
second-of-day, geodetic lat/long, local apparent solar time
(`input.lst = h + g_long/15`), F10.7 and Ap (`EarthAtmNRLMSISE00.cpp:53-96`),
then takes `rho = d[5]·1e3` (total mass density g/cm³ → kg/m³) and
`p = Σnᵢ·k·T`.

### The real inputs are frozen

`Planet.cpp:859` sets only `PRM_ALT | PRM_LNG | PRM_LAT` on every call. The
flags `PRM_FBR`, `PRM_F`, `PRM_AP` (`CelBodyAPI.h:556-562`) are *never* set, so
the models always fall back to **F10.7 = 140 sfu, Kp/Ap = 3.0**, forever. The
technical reference confirms this: "In Orbiter, solar flux values based on
observations are not taken into account… variations in geomagnetic activity are
ignored" (`earth_atm.tex:33-77`). There is no API to inject real space weather.

Additionally, `EarthAtmJ71G.cpp:68-69` overwrites the geographic inputs:

```cpp
double lng = (prm_in->flag & PRM_LNG ? prm_in->lng : 0.0);   lng=0;
double lat = (prm_in->flag & PRM_LAT ? prm_in->lat : 0.0);   lat=0;
```

so J71G's diurnal and seasonal-latitudinal terms are always evaluated at 0°N 0°E
(the seasonal-latitudinal density term `d_Sl` is identically zero because
`sin(lat)=0`). Unchanged since the SVN→git import.

### Lower-atmosphere time dependence (MSIS path only)

This is the one place in Orbiter where temperature below the turbopause moves
with time and place. `gtd7` builds the 0–72.5 km profile from **fixed node
heights** and splines through them (`nrlmsise-00.c:956-959`):

```
zn3 = { 0, 10, 15, 20, 32.5 } km     // troposphere / lower stratosphere
zn2 = { 32.5, 45, 55, 72.5 } km      // upper stratosphere / mesosphere
zmix = 62.5 km                       // turbopause for the mixing-ratio scheme
```

Each node temperature is `p·pavg / (1 − sw·glob7s(...))`, and `glob7s`
(`nrlmsise-00.c:857-940`) evaluates latitude-dependent Legendre polynomials plus
**symmetrical annual, symmetrical semi-annual, asymmetrical annual, asymmetrical
semi-annual, diurnal and semi-diurnal** terms driven by `input->doy` and
`input->lst` (via `ctloc = cos(hr·lst)`, `nrlmsise-00.c:694`). Orbiter's flags
struct enables all of these (`flags = {0,1,1,…}`, `EarthAtmNRLMSISE00.cpp:46`;
`swc[]` mirrors `switches[]`, `nrlmsise-00.c:88-98`).

So with NRLMSISE-00 selected, a vessel at 0 km in January at 60°N at 02:00 local
time gets a *different* T (and hence a different sound speed and Mach number)
than at 45°N in July at 14:00. Amplitudes are climatological and modest (a few K
near the surface), but the mechanism is real and is absent from both other Earth
models and from every other body.

Cost (1000 evaluations, `earth_atm.tex:214-224`): legacy 5.5e-5 s, J71G
2.9e-4–2.0e-3 s, NRLMSISE-00 3.8e-3–2.7e-2 s. 50 vessels in-atmosphere with
NRLMSISE-00 dropped the frame rate 130 → 114 fps in the 2009 measurement.

---

## 7. Derived flight quantities

| Quantity | Formula | Source |
|---|---|---|
| Airspeed | `V_air = V_ground − V_wind` | `Vesselbase.cpp:66-71` |
| Dynamic pressure | `q = ½ρV_air²` | `Vesselbase.cpp:96` |
| Speed of sound | `a = sqrt(γ·R·T)` — **variable with local T** | `Planet.h:139-141` |
| Mach number | `M = V_air / a` | `Vesselbase.cpp:97, 164` |
| Reynolds proxy | `Re₀ = ρ·V_air/μ`, `μ = 1.7894e-5` fixed | `Vessel.cpp:4113-4114` |
| Ground velocity | `2π(R+h)cosφ / RotT` (planet rotation) | `Planet.cpp:894-899` |

Notes:
- γ and R are **per-body constants** (config-overridable). No composition,
  humidity, or temperature dependence of γ, R, or μ.
- `μ` is annotated in-source: `// viscosity coefficient dummy - MAKE VARIABLE!`
- For `AtmInterface = 1` bodies, T ≡ 288.16 K, so sound speed on Titan or Jupiter
  is wrong by construction.
- **Altitude used for the atmosphere lookup is sphere-relative, not
  terrain-relative**: `alt0 = rad − size` is passed to `GetAtmParam`
  (`Vesselbase.cpp:33, 93`), while terrain elevation is applied to a separate
  `alt`. A vessel sitting on a 5 km plateau still gets sea-level ρ and p.

---

## 8. Atmospheric flight model

`Vessel::UpdateAerodynamicForces()` — preceded in-source by the comment
*"This function encodes the atmospheric flight model / NEEDS EXTENSIVE
OVERHAUL!"* (`Vessel.cpp:4096-4099`).

**Modern path** (`Vessel.cpp:4099-4229`): a lifting-surface model over
developer-declared airfoils.

- Angles: `aoa = atan2(−v_y, v_z)`, sideslip `beta = atan2(−v_x, v_z)` (ship frame, air-velocity vector).
- Per airfoil, orientation ∈ {LIFT_VERTICAL, LIFT_HORIZONTAL, FORCE_AND_MOMENT}:
  the **vessel module** supplies `CL/Cm/CD = f(aoa, M, Re)` (three interface
  generations: `AirfoilCoeffFunc`, `…Ex`, `…Ex2` returning CA/CN/CY + Cl/Cm/Cn —
  `OrbiterAPI.h:1695-1723`, documented `VesselAPI.h:1416-1523`). All
  transonic/wave-drag behaviour is therefore authored per spacecraft, not by the
  sim core.
- Reference area: airfoil `S`, else the vessel's projected cross-section
  `|dẑ|·S_z + |dŷ|·S_y`.
- Moment: `Cm·q·S·c` (chord) about the airfoil's axis.
- Control surfaces: linear in deflection, `ΔC_L = −δ·dCl`, parasitic drag
  `∝ |δ|·area·q` (`Vessel.cpp:4198-4220`).
- User drag elements: `CD = level·factor`, force `CD·q` along the airflow.
- Angular damping: `−q_mod·S·C_d·ω` clamped so it can never reverse the rotation
  within one step; note the *modified* dynamic pressure
  `q_mod = ½ρ(V+30)²` — a floor so damping exists at zero airspeed
  (`Vessel.cpp:4116-4130`).

**Legacy path** (`UpdateAerodynamicForces_OLD`, `Vessel.cpp:4231-4288`): used when
the vessel declares no airfoils. `L = C_L(aoa)·q·S_y`, induced drag
`D_i = L·C_L/e`, parasite drag from projected cross-section × `C_w`, and a fake
weathercocking moment `∝ size·mass·ρ·V·S·sin(Δaoa)` (in-source comment: "the hack
above"). Requires explicit `C_w` definition (`bCWexplicit`, issue #573).

**Not modelled in core:** buoyancy (the only hit for "buoyancy" in the tree is a
cosmetic particle tweak, `OVP/D3D9Client/Particle.cpp:594`), compressibility or
wave-drag corrections, reentry *heating* (reentry is a visual particle stream
keyed off density thresholds, `Vessel.cpp:2344-2352`), Coriolis terms, ground
effect, WIG, rotor/wing flapping, per-component airspeed sampling.

`FlightModel = 0|1` (simple/realistic, `Config.cpp:68, 574`) is merely passed
into the vessel module's constructor (`Vessel.cpp:5951-5964`); it does not change
the equations above.

---

## 9. Wind

`Planet::WindVelocity()` (`Planet.cpp:901-970`) is the entire wind model:

- For each integer kilometre of altitude, seed a PRNG from
  `g_rseed[(alt_km + k) % 100]` and draw two horizontal components in
  **±25 m/s**.
- Catmull-Rom cubic interpolation in altitude between those four samples.
- Optional per-vessel short-term perturbation: amplitude ±5 m/s, correlated over
  `corr_length = 1.0 s`, blended with the previous value (`WindPrm` state,
  `pert_t`, `pert_v`).
- Vertical component is never set (`dim += 2` loop skips `y`).
- In frames 2/3 the planet's rotational ground velocity is added, which is how
  "eastward wind" appears at all.

Characteristics to note when comparing: identical parameters for **every**
atmosphere (Titan and Mars get Earth-magnitude winds); no latitude, longitude,
local time, season, terrain, or surface-friction dependence; no gust model,
turbulence spectrum, shear profile, or jet stream; not deterministic across
session restarts in the perturbation term (`rand()` rather than the seeded
generator).

Off by default: `bAtmWind` default `false` (`Config.cpp:47`), config keyword
`AtmosphericWind` (`Config.cpp:590, 1155`), toggle on Launchpad → Extras →
Physics (`OptionsPages.cpp:750, 760`). Read once into a static
`Planet::bEnableWind` (`Planet.cpp:664`).

Wind *is* fully coupled into flight: it changes `V_air`, hence lift, drag, Mach
number, HUD airspeed, and control-surface authority.

---

## 10. Clouds and the visual atmosphere (decoupled from physics)

- Clouds are a **textured spherical shell** at a fixed `CloudAlt`, rotated at
  `CloudRotPeriod`, with optional cloud shadows and micro-textures between
  `CloudMicrotextureAlt[0..1]` (`Planet.cpp:316-338`). Quad-tree tile layers
  (`MinCloudResolution`…`MaxCloudResolution`). No thickness, no density, no
  effect on aerodynamics or on solar flux.
- Rendering haze/fog is a separate parameter set that does **not** use the
  physical profile: `AtmHazeExtent`, `AtmHazeShift`, `AtmHazeDensity`,
  `AtmHazeColor`, `AtmColor0`, `AtmFogParam = dens_0 dens_ref alt_ref`,
  `AtmFogColor`, `AtmTintColor`, plus `AtmoControls` sliders and Rayleigh/Mie
  scattering in `OVP/D3D9Client/{HazeMgr,VPlanetAtmo}.cpp`.
- Light attenuation on vessels cuts off at `AtmAttenuationAlt`
  (`Planet.cpp:249-251`) — for Earth 100 km (`Earth.cfg:41`) while the physical
  model extends to 2500 km.

Consequence worth avoiding: in Orbiter you can be in a physically dense layer
while the sky/haze rendering disagrees, because the two systems read different
parameters.

---

## 11. Feature matrix

| Feature | Status | Evidence |
|---|---|---|
| Altitude profile of p, ρ, T | yes | §5 |
| Layer tables with lapse rates | yes, hard-coded in C++ | §5 |
| US Standard Atmosphere 1976 fidelity (Earth) | yes, in J71G/MSIS path | §5.2 |
| Jacchia-71/77 + Gill, NRLMSISE-00 | yes, Earth only | §6 |
| Real sea-level values, R, γ per body | yes | §4 |
| Real F10.7 / Kp space weather | **no — frozen at 140 / 3** | §6 |
| Variable speed of sound | yes (`sqrt(γRT)`) | §7 |
| Variable γ, R, μ, composition | no | §7 |
| Buoyancy | no | §8 |
| Reentry heating | no (visual only) | §8 |
| Wind | yes, synthetic noise, off by default | §9 |
| Wind shear / jet stream / gust / turbulence | no | §9 |
| Time-of-day temperature | legacy: never · J71G: thermosphere only, at a fixed 0°N 0°E · **MSIS: yes, incl. troposphere/stratosphere** (climatological, few K) | §6 |
| Seasons in the atmosphere | legacy: never · J71G: semi-annual thermospheric term (`d_Sl` zeroed by `lat=0`) · **MSIS: annual + semi-annual terms down to 0 km** · no surface seasons anywhere | §6 |
| Seasonal insolation | yes, but rendering/lighting only (axial tilt) | lighting |
| Climate, humidity, precipitation, convection | no | — |
| Diurnal surface temperature | no dedicated model (MSIS produces a small z=0 diurnal signal as a side effect) | §6 |
| Dust cycles / aerosol opacity variation | no | — |
| Cloud *physics* (altitude fixed, texture only) | no | §10 |
| Terrain-relative altitude in atmosphere lookup | **no** | §7 |

---

## 12. Quirks and landmines (do not replicate blindly)

1. **`lng=0; lat=0` in J71G** (`EarthAtmJ71G.cpp:68-69`) silently disables
   geographic dependence of the only model that has it.
2. **Space-weather flags never set** by the caller (`Planet.cpp:859`) — the API
   supports real data, the core never provides any.
3. **Mode-1 temperature hard-coded to 288.16 K** (`Planet.cpp:843`) — every gas
   giant, Titan, Triton and Io reports Earth surface temperature regardless of
   their declared `R`/`γ`, which feeds `a = sqrt(γRT)` and hence Mach number,
   while the density scale height comes from `p0/(ρ0·g)` instead. The two are
   never reconciled (§4).
4. **Bad input data survives in the shipped config**: Io has `R = 1.92`,
   `γ = 0.3333` (γ < 1 is unphysical) and `p0 = ρ0 = 1e-4`, giving a 0.56 m
   scale height; `Io.cfg` sets `AtmHorizonAlt = 30e3` above its
   `AtmAltLimit = 20e3`. Worth a validation pass on load.
5. **Atmosphere sampled above the reference sphere, not above terrain**
   (`Vesselbase.cpp:33, 93`) — elevation-independent density.
6. **Cutoff altitude is a config override of a model constant** (`Mars.cfg:43`
   turns a 100 km model into 280 km) — model and domain can disagree.
7. **`μ` fixed at 1.7894e-5** with an explicit "MAKE VARIABLE" TODO
   (`Vessel.cpp:4113`).
8. **`(V+30)²` in the damping term** (`Vessel.cpp:4117`) — a magic constant to
   keep rotational damping alive at zero airspeed.
9. **Legacy weathercocking moment scales with `size·mass·ρ`** (`Vessel.cpp:4246`)
   — dimensionally ad hoc; in-source comment calls it a hack.
10. **MSIS is called at all altitudes** although its own header notes that
    "f107, f107A, and ap effects are neither large nor well established below
    80 km and these parameters should be set to 150., 150., and 4. respectively"
    (`nrlmsise-00.h:114-116`). Orbiter feeds 140/140/3 instead — the J71G
    defaults — and sub-80 km values are extrapolations of a thermospheric model.
11. **Two disconnected "atmosphere" parameter sets** — physical (p0, ρ0, R, γ)
    and visual (haze/fog/color) — with no shared scale height.
12. **Wind is a seeded function of altitude only**, so two vessels at the same
    altitude always see the same base wind (useful for multiplayer determinism,
    but physically empty).

---

## 13. What to borrow, what to replace (for a Pioneer!/KSP-like sim)

**Worth copying:**
- The `p0, ρ0, R, γ, cutoff` + `profile(alt) → T,p,ρ` split. It is small, stable,
  and every derived quantity (a, Mach, q, Re, Reynolds, range, ΣQ) follows.
- Piecewise isothermal/lapse layers in **geopotential** altitude with the two
  analytic forms (§5). Cheap, exact, and matches published standard atmospheres.
- Keeping the model behind an interface so each body picks its own
  (mode 1 exponential for thin/no-air bodies, layered model for Earth-like, high
  model for drag-domain extension).
- Deriving scale height from the body's own gravity (`C = ρ0 g/p0`) instead of
  hand-tuning — self-consistent when you change planet mass/radius.
- `a = sqrt(γRT)` with T from the profile: this alone gives you the meaningful
  "Mach number changes with altitude and temperature" behaviour for free.
- Wind coupled through airspeed rather than as an ad-hoc force term — one code
  path, correct HUD airspeed, correct control authority.

**Replace / do differently:**
- Make the profile a **data asset** (per-body layer table: `h_base, T_base,
  lapse, p_base`), not compiled C++. Orbiter's biggest structural regret here is
  that there is no editable layer table anywhere.
- Sample the atmosphere at **terrain-relative** altitude.
- Reserve the space-weather / insolation inputs as first-class fields with
  observed data (or at least a 11-year solar cycle + storm model) rather than
  constants baked into the model.
- Give each body its own `R`, `γ`, `μ(T)` from composition; make μ a Sutherland
  law function of T. Then assert ideal-gas consistency at load time:
  `p0 ≈ ρ0·R·T_surface` (Orbiter's mode-1 bodies fail this by factors of 3–27,
  see §4), because an inconsistent triple silently breaks Mach, dynamic
  pressure and scale-height agreement.
- Add a **circulation model** instead of seeded noise: even a
  latitude × altitude wind lookup with Hadley/ferrel cells + a diurnal term +
  a correlated gust (OU process) gives qualitatively better flight behaviour
  than ±25 m/s per-kilometre noise.
- Add a surface energy balance (diurnal + seasonal + latitude) if you want
  temperature to mean anything at the low altitudes where people actually fly.
  Only NRLMSISE-00 gives Orbiter any sub-90 km time dependence at all, and it is
  a fitted climatology, not a model — the legacy and J71G profiles are strictly
  time-independent below ~90 km.
- Model buoyancy and reentry heating physically if airships or aerobraking matter.
- Unify the physical and visual atmospheres on one density scale height so the
  sky agrees with the drag.

---

## 14. Config keyword quick reference (`Planet.cpp:216-338`)

```
Module_Atm           <dll name>                 ; default atmosphere module
MODULE_ATM           <dll name>                 ; in Config/<Body>/Atmosphere.cfg, user override
AtmPressure0         [Pa]                       ; only used when no module supplies one
AtmDensity0          [kg/m^3]                   ; idem
AtmGasConstant       [J/kg/K]  (default 286.91) ; overrides module
AtmGamma             (-)        (default 1.4)   ; overrides module
AtmAltLimit          [m]                        ; overrides module cutoff
AtmAttenuationAlt    [m]
AtmHorizonAlt        [m]
AtmHazeExtent/Shift/Density/Color, AtmColor0, AtmFogParam, AtmFogColor, AtmTintColor
CloudAlt, CloudRotPeriod, CloudShadowDepth, BrightenClouds,
CloudMicrotextureAlt, Min/MaxCloudResolution, CloudFormat
```

Global physics switch:

```
AtmosphericWind = TRUE | FALSE      ; default FALSE (Config.cpp:47, 590)
```
