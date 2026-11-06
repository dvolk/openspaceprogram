# Pioneer atmosphere & atmo-flight model — reference report

source available in ../references

Purpose: a precise description of how Pioneer models planetary atmospheres and
atmospheric flight, for comparison when designing a KSP-like game.

Provenance: `master` @ `8ea5aa5df` ("Null audio backend for running headless unit tests").
All line numbers below are from that checkout. Worked-example numbers were
recomputed independently from the code's own formulas and constants
(`src/gameconsts.h`), not read out of a running build.

Primary files:

| Area | File |
| --- | --- |
| Body data model + profile math | `src/galaxy/SystemBody.h`, `src/galaxy/SystemBody.cpp` |
| Physics query surface | `src/Planet.cpp` (`GetAtmosphericState`), `src/DynamicBody.cpp` |
| Flight forces | `src/Ship.cpp`, `src/DynamicBody.cpp` |
| Render-only params | `src/galaxy/AtmosphereParameters.h`, `src/BaseSphere.cpp`, `src/GeoSphere.cpp`, `src/GasGiant.cpp` |
| Shaders | `data/shaders/opengl/rayleigh.glsl`, `rayleigh_fast/accurate`, `geosphere_sky.frag`, `geosphere_terrain.frag`, `lib.glsl` |
| Authored Sol data | `data/systems/00_sol.lua` (JSON twin: `data/systems/custom/00_sol.json`) |
| Procedural generation | `src/galaxy/StarSystemGenerator.cpp` |
| Terrain "climate" | `src/terrain/Terrain.cpp`, `src/terrain/TerrainColor*.cpp` |

---

## 1. TL;DR coverage matrix

| Feature | Pioneer | Notes |
| --- | --- | --- |
| Vertical atmosphere profile | yes (analytic) | one layer, closed form |
| **Layer tables / altitude keyframes** | **no** | nothing tabulated anywhere |
| Lapse rate | yes | dry adiabatic `L = g/cp`, one value |
| Surface pressure authored? | no | derived from density + T + molar mass |
| Composition | 2 hardcoded mixes | "air" and "H/He"; a scalar `atmosOxidizing` exists but does **not** affect `M`/`cp` |
| Speed of sound / Mach effects | **no** | no compressibility, wave drag, or sonic boom |
| Reynolds number / viscosity | **no** | explicit TODO in code |
| Wind | **no** | air co-rotates with the body, always still |
| Coriolis / centrifugal | yes | real, in rotating frames |
| Weather, clouds, storms | **no** | |
| Climate simulation | **no** | static biome pick at generation from global scalars |
| Time-of-day temperature | **no** | one constant temperature per body |
| Seasons (thermal) | **no** | illumination is seasonal/geometry-correct, temperature is not |
| Reentry heating | yes, but a proxy | drag-G based, not skin friction |
| Pressure crush damage | yes | per-ship limit × atmospheric-shield equip |
| Fuel scooping | yes | ρ·v based |
| Lift / stall | crude | AoA curve, no real airfoils |

---

## 2. Data model: scalars only

`SystemBody` stores only these atmosphere-relevant fields
(`src/galaxy/SystemBody.h:121-137`, `:359-362`):

| Field | Unit | Notes |
| --- | --- | --- |
| `m_volatileGas` (Lua/JSON: `atmosDensity`) | **kg/m³ at the reference radius** | `1.225` = Earth (see `EARTH_ATMOSPHERE_SURFACE_DENSITY`, `gameconsts.h:24`) |
| `m_atmosOxidizing` | 0..1 | 0 = reducing (H₂, NH₃), 1 = oxidising (CO₂, O₂). Comment cites Sagan et al. photochemistry (doi 10.1088/0004-637X/784/1/63) |
| `m_averageTemp` | K | single global value |
| `m_radius`, `m_mass` | Earth radii / Earth masses | feed gravity |
| `m_atmosPressure` | atm | **derived** |
| `m_atmosRadius` | m (height above surface) | **derived**: height where p = 0.001 atm |
| `m_atmosColor` | RGBA | render only |

`HasAtmosphere()` is literally `m_volatileGas > fixed(1,100)` i.e. 0.01 kg/m³
(`SystemBody.cpp:373-376`).

### Derived surface pressure (not authored)

`SystemBody::SetAtmFromParameters()` — `src/galaxy/SystemBody.cpp:329-371`:

```cpp
double surfaceDensity = GetAtmSurfaceDensity() / gasMolarMass; // kg/m^3 -> mol/m^3
double surfaceTemperature_T0 = GetAverageTemp();               // K
// P = (n/V) * R * T
m_atmosPressure = PA_2_ATMOS * ((surfaceDensity) * GAS_CONSTANT_R * surfaceTemperature_T0); // atm

const double lapseRate_L = surfaceGravity_g / GetSpecificHeat(GetSuperType()); // deg/m
if (m_atmosPressure < 0.002)
    m_atmosRadius = 0; // no meaningful radius
else {
    double RLdivgM = (GAS_CONSTANT_R * lapseRate_L) / (surfaceGravity_g * gasMolarMass);
    m_atmosRadius = (1.0 - exp(RLdivgM * log(0.001 / m_atmosPressure))) * surfaceTemperature_T0 / lapseRate_L;
}
```

Consequence: authored density silently *is* the pressure knob. Pioneer's Earth is
authored at `atmos_density(f(1,1))` = 1.0 kg/m³ (not 1.225), so its sea-level
pressure works out to **0.816 atm**.

Stars get a fabricated surface pressure (`SystemBody.cpp:335-352`):
`target_pressure = (0.01 / 274.0) * surfaceGravity` — anchored to "the Sun's
surface pressure is ~0.01 atm".

---

## 3. The physical profile: one layer, dry-adiabatic

`src/galaxy/SystemBody.cpp:303-327`:

```cpp
double SystemBody::GetAtmPressure(double altitude) const
{
    const double gasMolarMass = GetMolarMass(GetSuperType());
    const double surfaceGravity_g = CalcSurfaceGravity();
    const double specificHeat = GetSpecificHeat(GetSuperType());
    const double lapseRate_L = surfaceGravity_g / specificHeat;            // deg/m
    const double surfaceTemperature_T0 = GetAverageTemp();                // K

    return m_atmosPressure * pow((1 - lapseRate_L * altitude / surfaceTemperature_T0),
                                 (specificHeat * gasMolarMass / GAS_CONSTANT_R)); // atm
}

double SystemBody::GetAtmDensity(double altitude, double pressure) const
{
    double gasMolarMass = GetMolarMass(GetSuperType());
    double aerialTemp = GetAtmAverageTemp(altitude);
    return (pressure / (PA_2_ATMOS * GAS_CONSTANT_R * aerialTemp)) * gasMolarMass;
}

double SystemBody::GetAtmAverageTemp(double altitude) const
{
    const double lapseRate_L = CalcSurfaceGravity() / GetSpecificHeat(GetSuperType());
    return double(GetAverageTemp()) - lapseRate_L * altitude;
}
```

That is the **US Standard Atmosphere 1976 troposphere equation**

```
p(h) = p0 * (1 - L*h/T0) ^ (g*M / (R*L))
T(h) = T0 - L*h
rho(h) = p(h)*M / (R*T(h))
```

but with `L = g/cp` (the **dry adiabatic** lapse rate) instead of 6.5 K/km, and
with **only that single layer** — no tropopause, no stratospheric inversion, no
mesosphere/thermosphere. Above `m_atmosRadius`, density and pressure are hard
zero: `src/Planet.cpp:79` comments "This model has no atmosphere beyond the
adiabatic limit".

Composition constants — `src/galaxy/SystemBody.cpp:283-301`, both prefixed
`// TODO: a more detailed atmospheric simulation should replace this`:

| Supertype | Molar mass `M` (kg/mol) | `cp` (J/kg·K) | ⇒ lapse `L = g/cp` |
| --- | --- | --- | --- |
| gas giant | `0.0023139903` | `12950.0` | tiny (~2 K/km at 1 g-ish) |
| star | `0.00125` | `1000.5` | — |
| everything else | `0.02897` (`// XXX using earth's molar mass of air...`) | `1000.5` | ~9.8 K/km at 1 g |

Constants: `src/gameconsts.h` — `GAS_CONSTANT_R = 8.3144621`,
`PA_2_ATMOS = 1/101325`, `EARTH_RADIUS = 6378135.0`, `EARTH_MASS = 5.9742e24`,
`G = 6.67428e-11`.

Physics query — `src/DynamicBody.cpp:103-114`: only valid inside a **rotating
planet frame**; otherwise p = ρ = 0.

### Worked profiles (recomputed from the above)

Earth (r=1, m=1, T=288 K, ρ=1.0): `g = 9.802 m/s²`, `L = 9.797 K/km`,
`p0 = 0.8158 atm`, exponent `k = 3.486`, **atmo top = 25.10 km**, T at top = **42.1 K**.

| h (km) | p (atm) | T (K) | ρ (kg/m³) |
| --- | --- | --- | --- |
| 0 | 0.81576 | 288.0 | 1.00000 |
| 1 | 0.72305 | 278.2 | 0.91756 |
| 5 | 0.42591 | 239.0 | 0.62910 |
| 10 | 0.19147 | 190.0 | 0.35572 |
| 15 | 0.06774 | 141.1 | 0.16954 |
| 20 | 0.01531 | 92.1 | 0.05871 |
| 25 | 0.00108 | 43.1 | 0.00889 |

Real 1976 US Standard Atmosphere for comparison: 5 km → 0.540 atm / 255.6 K /
0.736 kg/m³; 20 km → 0.0547 atm / 216.7 K / 0.088 kg/m³. So Pioneer's single
adiabatic layer is ~20 % low at 5 km and ~35 % low at 20 km, and its upper
atmosphere is far colder than reality (no stratosphere to hold temperature up).

Jupiter (r=11, m=317.8, T=165 K, ρ=1.63, H/He): `g = 25.74 m/s²`,
`L = 1.988 K/km`, `p0 = 9.537 atm`, atmo top = 76.5 km.

| h (km) | p (atm) | ρ (kg/m³) |
| --- | --- | --- |
| 0 | 9.537 | 1.630 |
| 5 | 7.624 | 1.387 |
| 10 | 6.005 | 1.167 |
| 20 | 3.531 | 0.795 |

Note: most hulls have `atmospheric_pressure_limit` of 3–10 atm (see §5), so
"Pioneer's gas giants" are lethal within a few kilometres of the cloud deck.

Venus (r=0.95, m=0.815, T=735, ρ=53): `p0 = 110.3 atm` (real ≈ 92 atm), top 80 km.
Mars (r=0.533, m=0.107, T=278, ρ=0.489): `g = 3.69`, `L = 3.69 K/km`, `p0 = 0.385 atm` (real ≈ 0.006 atm).
Io (r=0.286, m=0.015, T=130, ρ=0.0816): `p0 = 0.0301 atm` (real ≈ 1e-11 atm) — inflated so you can land.

---

## 4. Rendering uses a *different* atmosphere model

`SystemBody::CalcAtmosphereParams()` (`src/galaxy/SystemBody.cpp:458-521`) builds
`AtmosphereParameters` for shaders — an **isothermal exponential** model, not the
adiabatic one used by physics:

```cpp
// The equation for pressure is: Pressure at height h = Pressure surface * e^((-Mg/RT)*h)
float atmosScaleHeight = GAS_CONSTANT_R * T / (M * g);          // H = RT/Mg
params.atmosInvScaleHeight = std::max(20.0f, GetRadius() / atmosScaleHeight);
params.atmosRadius = 1.0f + (10.0f * atmosScaleHeight) / GetRadius();   // visual top = 10 H
params.rayleighCoefficients = GetCoefficients(radiusKm, atmosHeightKm, atmosScaleHeight);
params.mieCoefficients     = GetCoefficients(radiusKm, atmosHeightKm, atmosScaleHeight / 6.66);
params.scaleHeight = vector2f(atmosScaleHeight, atmosScaleHeight / 6.66);
```

`GetCoefficients()` / `ComputeDensity()` (`SystemBody.cpp:410-455`) precompute a
(k, b, c) triple per planet — a fit of out-scattering `k*exp(-h*b)` plus an `erf`
in-scattering ramp — so the shaders can avoid per-pixel ray integration of the
vertical density. There is a `// XXX hack to avoid issues with sysgen giving 0
temps` that clamps T to 165 K.

Shaders (`data/shaders/opengl/rayleigh.glsl`): 16-sample raymarch,
`betaR = vec3(3.8e-6, 13.5e-6, 33.1e-6)`, `betaM = vec3(21e-6)`, Mie phase with
`g = 0.76`, cylindrical-shadow segment subtraction, per-planet coefficients for
the sunward leg. `lib.glsl:95 AtmosLengthDensityProduct` does a 6-tap
exponential-fog density integral. `geosphere_sky.frag` adds a hand-tuned sunset
redshift (`sunset = vec4(0.8, pow(p,0.8), pow(p,1.2), 1)`).

**Consequence for Earth:** the flight model ends at 25 km, but the visible sky
shell extends to 10 × 8.43 km = **84.3 km**. You can see atmosphere where the
physics says density is exactly zero.

Two material variants exist: `rayleigh_fast` and `rayleigh_accurate`, selected by
the planet-detail setting (`src/GeoSphere.cpp:573-576`, commit `210859422`).

---

## 5. Atmospheric flight model

### Drag — `src/Ship.cpp:480-536`

```cpp
// TODO: vary drag coefficient based on Reynolds number, specifically by
// atmospheric composition (viscosity) and airspeed (mach number).
const vector3d localVel = GetVelocity() * GetOrient();
const auto rho_2 = density / 2.;
const vector3d lv2(localVel * localVel);
const vector3d fAtmosDrag(
    -sign(localVel.x) * rho_2 * lv2.x * sideCrossSec  * sideDragCoeff,
    -sign(localVel.y) * rho_2 * lv2.y * topCrossSec   * topDragCoeff,
    -sign(localVel.z) * rho_2 * lv2.z * frontCrossSec * frontDragCoeff);
```

i.e. incompressible `½ρv²C_dA`, applied per body axis with **per-axis authored
cross-section and drag coefficient**. Early-out when density ≈ 0.

Per-ship parameters (`data/ships/*.json`, loaded in `src/ShipType.cpp:195`):

| Ship | front/side/top cross-section (m²) | front/side/top Cd | pressure limit (atm) |
| --- | --- | --- | --- |
| venturestar | 120.002 / 280.002 / 1190.002 | 0.06 / 0.21 / 0.95 | 7.2 |
| deneb | 60.002 / 225.002 / 712.002 | 0.08 / 0.25 / 0.89 | 7.2 |
| nerodia | 210.002 / 358.002 / 810.002 | 0.6 / 0.8 / 1.1 | 3.7 |
| xylophis | 7.002 / 17.502 / 32.002 | 0.56 / 0.84 / 0.95 | 3.0 |

(The `.002` tails are the default `DEFAULT_CROSS_SECTION = 0.002` floor.)

### Lift — same function

```cpp
double AoAMultiplier = localVel.NormalizedSafe().y;
if (std::abs(AoAMultiplier) < 0.61) {           // stall gate
    // Generated lift peaks at around 20 degrees here, and falls off fully at 35-ish.
    AoAMultiplier = cos((std::abs(AoAMultiplier) - 0.31) * 5.0) * sign(AoAMultiplier);
    // We scale down the lift contribution because fAtmosDrag handles deflection-based lift.
    fAtmosLift.y = CalcAtmosphericDrag(pow(localVel.z, 2), topCrossSec, shipLiftCoeff) * -AoAMultiplier * 0.2;
}
```

Lift is computed from the *forward* speed only, using the ship's **top**
cross-section as a proxy for wing area, times `shipLiftCoefficient`. There is a
stall threshold at ~35° AoA and a peak near 20°. Comments admit the gaps:
"handle AoA better / more gracefully with an actual angle- and curve-based
implementation", "non-cambered wings". No induced drag, no wing geometry, no
flaps, no tail/trim.

### Weathervane stability — `src/Ship.cpp:538-560`

Torque that points the nose into the relative wind, gated to speeds > 100 m/s,
scaled by the ship's `atmoStability` and by `Pi::game->GetInvTimeAccelRate()`
(time-acceleration compensation). Comment: "TODO: evaluate this function and
properly implement based upon ship cross-section."

### Rotating frames, Coriolis, co-rotating air

`src/DynamicBody.cpp:200-226` applies gravity, atmospheric drag, then

```cpp
// centrifugal and coriolis forces for rotating frames
if (f->IsRotFrame()) {
    vector3d angRot = vector3d(0.0, f->GetAngSpeed(), 0.0);
    m_externalForce -= m_mass * angRot.Cross(angRot.Cross(GetPosition())); // centrifugal
    m_externalForce -= 2 * m_mass * angRot.Cross(GetVelocity());           // coriolis
}
```

Frames: `src/Space.cpp:779-818` creates a non-rotating Hill-scaled frame plus a
rotating frame (`FLAG_ROTATING`) with `angSpeed = 2π / rotationPeriod` and
orientation `RotateX(axialTilt) * RotateY(rotationPhaseAtStart)`. Gas giants get
the same treatment (`SUPERTYPE_GAS_GIANT` branch), and in-space gas giants are
`Planet` objects (the gas-giant look is a `GasGiant` visual attached to
`TerrainBody`, `src/TerrainBody.cpp:33-38`), so the atmo physics applies to them.

Because drag uses the rotating-frame velocity, **the air is implicitly
co-rotating**: your frame-relative velocity is exactly your airspeed. That is
self-consistent, and it means Coriolis is real while wind is definitionally zero.

### Pressure crush — `src/Ship.cpp:1307-1313`

```cpp
int atmo_shield_cap = std::max(m_stats.atmo_shield_cap, 1);
if (pressure > (m_type->atmosphericPressureLimit * atmo_shield_cap)) {
    float damage = float(pressure - m_type->atmosphericPressureLimit);
    DoDamage(damage);
}
```

Equipment scaling: `Ship::GetAtmosphericPressureLimit()` = base × `atmo_shield_cap`
(`Ship.cpp:455-458`); sized shield variants exist (commit `5666baa9f`). The AI
uses the same limit to avoid diving too deep (`src/ShipAICmd.cpp:1302-1306`).

### Reentry / aerodynamic heating — `src/Ship.cpp:1133-1142`

```cpp
double Ship::GetHullTemperature() const
{
    // TODO: fix this to calculate appropriate skin friction and heating.
    //const double dragGs = CalcAtmosphericDrag(GetVelocity().LengthSqr(), GetClipRadius(), dragCoeff) / (GetMass()*9.81);
    //return dragGs / 25.0;
    // TODO: fix this to properly account for heating due to air friction instead of G-force.
    double dragGs = GetAtmosForce().Length() / (GetMass() * 9.81);
    return dragGs / (15.0 * (1.0 + m_stats.atmo_shield_cap + (2.0 * (1.0 - m_wheelState))));
}
```

Heating is a **drag-G proxy** (the physically-motivated version is commented
out), divided by a factor that includes the atmo-shield rating and landing-gear
state — gear down roughly triples heating (commit `3f8449007`: "Halve it for
landing gear down, increase it 3 times for baseline"). Applied as damage when
`> 1.0` (`Ship.cpp:1295-1297`) and drives a hull glow via
`Clamp(GetHullTemperature(), 0, 1)` (`Ship.cpp:1735`).

### Fuel scooping — `src/Ship.cpp:1322-1358`

Rate ∝ `speed * density * hydrogen_density(0.0002) * scoop_area`, requires nose
within ~0.90 dot of prograde and `speed*density > 30`. `IsScoopable()`
(`SystemBody.cpp:378-388`): all gas giants, plus terrestrials with
`gas > 0.01` and `0.3 < atmosOxidizing <= 0.55` (i.e. hydrogen-rich).

### Exhaust plume cutoff

`Ship.cpp:1491-1498` uses `GetAtmRadius()` (floored at 1000 m) as the altitude
cap for thruster-plume rendering.

### Audio

`src/sound/AmbientSounds.cpp:207-236`: airflow volume =
`density * speed * 0.0005`, split into bands across several
`Atmosphere_Flying.ogg` layers — i.e. volume tracks dynamic pressure. Surface
wind/ambient noise is a single per-body sample; no wind speed involved.
Note `src/sound/AlAudioBackend.cpp:58` sets `alSpeedOfSound, 1000.f` — that is
only the OpenAL Doppler propagation constant, *not* an atmospheric model.

---

## 6. Climate, weather, seasons, day/night

**No simulation.** What exists:

- **Static biome selection** at generation (`src/terrain/Terrain.cpp:141-300`):
  branches on global scalars — `life > 0.7 && gas > 0.2 && avgTemp > 240` →
  Earth-like; gas/liquid/ice thresholds → desert / ice / "TFGood" / "TFPoor";
  `ices > 0.8 && avgTemp < 250` → frozen world.
- **Sea level** = `volatileLiquid` clamped 0..1 (`Terrain.cpp:475`).
- **Polar caps, snow on mountains, equatorial desert** are analytic
  latitude/height hacks in the colour generators, e.g.
  `TerrainColorEarthLike.cpp:35-49`:
  `if ((m_icyness*0.5) + fabs(p.y*p.y*p.y*0.38) > 0.6) → ice`, and an
  `equatorial_desert` term proportional to `(1 - p.y²)`.
- **Albedo/ice cover** feed planet appearance; planet descriptions and icons
  threshold `averageTemp` at 250/273/300 K (`SystemBody.cpp:676-952`).

**Time of day:** illumination is genuinely geometric — rotating frame +
`axialTilt` + orbital position give a correct day/night terminator, changing sun
declination (so *lighting* is seasonal), plus real eclipse shadows
(`eclipse.glsl`, `Camera.cpp`). But `GetAtmAverageTemp()` and `averageTemp`
never depend on local time, latitude, or orbital phase.

**Seasons:** no thermal seasons. The generator computes perihelion/aphelion
temperature extremes only to decide habitability
(`StarSystemGenerator.cpp:786-790`), then throws that variance away.

**Wind/weather/clouds:** absent. Grep across `src/` and all `data/**/*.lua`
finds no `wind`/`weather`/`season` concept at all (only hyperspace "clouds").

---

## 7. Procedural generation: what's real physics

`StarSystemGenerator.cpp`:

- Stellar flux: `calcEnergyPerUnitAreaAtDist()` (`:520-534`) — σT⁴ scaled by
  radius and inverse-square distance. Real.
- Equilibrium temperature: `:605-614` —
  `T = (flux*(1-albedo) / (4*(1-greenhouse)*σ))^0.25`. Real formula.
- Greenhouse/volatile feedback (`:745-780`) is stepped fudge:
  `if (T > 195) greenhouse += volatiles/3` (CO₂ sublimation),
  `if (T > 273) greenhouse += volatiles/5` (H₂O liquid),
  `if (T > 373) greenhouse += volatiles/3` (boiling); then gas/liquid/ice split
  by ad-hoc ratios `T/(100+T)`, `T/(50+T)`.
- Mass–radius: `:673-691` uses `pow(mass, 0.3)` / `pow(mass, 0.5)` regimes,
  citing the PHL "Standard mass-radius relation for exoplanets", then a random
  ±20 % radius jitter ("otherwise the surface gravity will be consistently 1.0 g").
- Gas giant cloud-deck density: `rand.NormFixed(1.05, 0.8)` kg/m³, described as
  "real-world density-at-1-bar data … with an approximation factor of 3e"
  (`:727-731`).

---

## 8. Authored Sol data vs reality

From `data/systems/00_sol.lua` (`atmos_density` is kg/m³ at the reference radius):

| Body | Pioneer T (K) | real T | Pioneer ρ | real ρ | verdict |
| --- | --- | --- | --- | --- | --- |
| Mercury | 340 | ~440 | 0 (none) | ~1e-13 | no atmosphere modelled |
| Venus | 735 | 735 | 53 | ~65 | good |
| Earth | 288 | 288 | 1.0 | 1.225 | p₀ = 0.82 atm by accident |
| Mars | **278** | ~210 | **0.489** | 0.020 | in-fiction terraformed; file comment: `-- XXX composition copied from earth until there's a way to indicate terraformed` |
| Io | 130 | ~130 | 0.0816 (`f(100,1225)`) | 1.4e-5 | inflated so landings work |
| Jupiter | 165 | ~165 @1 bar | 1.63 | ~0.16 | 10× dense |
| Titan | 94 | 94 | 12.2 | ~5.4 | 2× dense |
| Triton | 38 | ~38 | 0.0816 (`f(100,1225)`) | ~7e-5 | in-game ≈ 0.009 atm vs real ~1.8 Pa |

Orbital elements, radii and masses are largely real (Titan radius 0.400, mass
0.0225 EU; Jupiter 11 EU / 317.8 EU; Venus 0.95/0.815). Atmosphere densities are
hand-tuned for playability. Note the odd authored form `atmos_density(f(100,1225))`
— a fraction written against Earth's 1.225. Line refs in `data/systems/00_sol.lua`:
Mercury `:27`, Venus `:46`, Earth `:66`, Mars `:152`, Io `:345`, Jupiter `:263`,
Titan `:549`, Triton `:677`.

There is also `data/systems/custom/00_sol.json` (2761 lines), the editor-facing
JSON twin, which carries the same scalar set (`averageTemp`, `atmosDensity`, …
see `SystemBody.cpp:102-109`).

---

## 9. Lua / modding surface

SystemBody attributes (`src/lua/LuaSystemBody.cpp`, typed stubs in
`data/meta/SystemBody.lua:36-80`): `averageTemp`, `atmosDensity` ("0.0 = no
atmosphere, 1.225 = earth atmosphere density, 64 ~= venus"), `atmosOxidizing`,
`surfacePressure`, `volatileLiquid`, `volatileIces`, `volcanicity`, `life`,
`hasAtmosphere`, `isScoopable`.

Body methods: `GetAtmosphericState(forBody)` → `{pressure, density}` in the
querying body's frame (`src/lua/LuaBody.cpp:660-675`, `:791`; works between any
two bodies since commit `29a74ca88`). `Ship:GetHullTemperature()`,
`ShipDef.atmosphericPressureLimit`.

Example consumer: `data/modules/Scout/Scout.lua:371` scales mission difficulty
by `log(sBody.atmosDensity)`.

Editor readout (`src/editor/system/GalaxyEditAPI.cpp:858-866`) exposes surface
pressure, atmosphere height and scoopability as read-only derived values.

---

## 10. Known inconsistencies / gaps (useful as "don't do this" list)

1. **Two models, two extents.** Physics profile (adiabatic, ends 25 km on Earth)
   vs render profile (isothermal exponential, ends 10·H = 84 km). Visible sky
   where physics density is 0.
2. **Pressure is derived, so density is the only knob.** Earth inherits 0.816 atm
   from an authored `1.0` instead of `1.225`.
3. **No upper-atmosphere structure.** With `L = g/cp` and one layer, the top of
   the flight atmosphere is at 42 K on Earth. Any supersonic/high-altitude
   gameplay is out of reach by construction.
4. **Composition scalar exists but is decorative for physics** — `atmosOxidizing`
   drives text, icons and scoopability, never `M` or `cp`. A hydrogen terrestrial
   still uses air's molar mass.
5. **Gas giants' reference density ⇒ ~9.5 atm at the cloud deck**, so "fly in a
   gas giant" is not a flight regime, it's a death timer.
6. **Heating is drag-G, not stagnation heat** — no dependence on ρ·v³ heating
   rate, no ablation, no heat-shield orientation.
7. **Lift uses the ship's top cross-section as wing area** and a `cos()` AoA
   curve; no airfoils, no induced drag, no control-surface authority.
8. **`// XXX hack` temperature clamp to 165 K** in `CalcAtmosphereParams()`
   papers over generator cases that produce T = 0.
9. **`GetAtmPressure()` can produce a negative `pow` base** above the adiabatic
   limit; callers are only safe because `Planet::GetAtmosphericState` clamps on
   `m_atmosphereRadius` first.
10. **Density → 0 discontinuously** at the atmosphere boundary — no smooth
    falloff to zero, so drag switches off abruptly at 25 km.

---

## 11. Takeaways for a KSP-like design

Worth stealing (cheap, high payoff):

- Single closed-form profile with `L = g/cp`: two authored numbers per body
  (`T_surface`, `ρ_surface`) plus radius/mass ⇒ a physically-shaped p/ρ/T curve
  for free. This is the entire reason Pioneer's atmospheres feel consistent
  across procedurally generated worlds.
- Deriving surface pressure by ideal-gas law rather than authoring it, so
  gravity + temperature + density stay mutually consistent.
- Deriving the atmosphere's outer radius from a fixed cutoff pressure rather
  than authoring it — gives a sane, body-specific "edge of space".
- Per-axis cross-section + Cd on the craft: instantly gives ships distinct
  reentry characters with no mesh analysis.
- Rotating-frame Coriolis/centrifugal with co-rotating air: real curved flight
  paths, no wind system required.
- Weathervane torque: passive aerodynamic stability for free, makes prograde
  orientation meaningful.
- Precomputing per-planet scattering coefficients once instead of per-pixel
  (`GetCoefficients`) — a good budget trick for a scattered-atmosphere renderer.

Things you'd want that Pioneer lacks, in ascending cost:

1. **Piecewise layer table** — the single biggest gap. A small per-body array of
   `{baseHeight, baseTemp, lapseRate}` layers (exactly the US Standard Atmosphere
   1976 table shape: troposphere + stratosphere + mesosphere + thermosphere,
   each with its own exponent) fixes the 42 K-aloft problem, gives a real
   tropopause, and lets density taper smoothly instead of snapping to 0. This is
   the axis on which KSP-like reference atmosphere models (and the 1976 standard)
   are most differentiated from Pioneer.
2. **Composition-driven `M`, `cp`, and gas colour/spectra** — `atmosOxidizing`
   shows they already track the intent; wiring it into `M` and `cp` is a small
   change with large variety gains (H₂ worlds ⇒ huge scale heights, tiny lapse
   rates).
3. **Shared profile for physics and rendering** — one source of truth for scale
   height/top-of-atmosphere; the current split causes visible sky over vacuum.
4. **Wind field** — a global zonal jet (function of latitude + rotation rate)
   plus turbulence is enough for airships, SAHPI-style, and adds ~one vector per
   body. Pioneer's structure makes this a small insertion point: subtract a wind
   vector from `localVel` in `CalcAtmosphericForce`.
5. **Diurnal + latitudinal temperature model** — insolation from the already
   correct sun geometry (`cos(zenith)` × atmospheric path length), a thermal
   inertia term, and `T(h, lat, localTime)`; this is also what makes real wind
   and weather possible.
6. **Mach-number-dependent drag and a real speed of sound**
   `a = sqrt(γRT/M)` — you already have `T(h)` and `M`; adding `γ` per
   composition gives compressibility, wave drag near M≈1, and sonic booms.
7. **Stagnation-point heating** `∝ sqrt(ρ/R_nose) · v³` for reentry instead of
   drag-G.
8. **Real lift surfaces** — per-lifting-surface area/airfoil with CL/CD curves
   and stall, so wings matter more than hull cross-section.

---

## Appendix: reproduction script for the profile tables

Every number in §3 was produced from the code's own formulas with this script
(constants copied from `src/gameconsts.h`):

```python
G=6.67428e-11; ER=6378135.0; EM=5.9742e24
R=8.3144621; PA2ATM=1.0/101325.0

def profile(name, r_eu, m_eu, T0, rho0, M, cp, p_cut=0.001):
    r, m = r_eu*ER, m_eu*EM
    g = G*m/(r*r)                     # SystemBody::CalcSurfaceGravity
    L = g/cp                          # lapseRate_L, deg/m
    p0 = PA2ATM*(rho0/M)*R*T0         # SetAtmFromParameters, atm
    k  = cp*M/R                       # exponent
    h_top = (1.0-(p_cut/p0)**(1.0/k))*T0/L     # m_atmosRadius
    H = R*T0/(M*g)                    # isothermal scale height (shader model only)
    print(f"{name}: g={g:.3f} L={L*1000:.3f}K/km p0={p0:.4f}atm "
          f"top={h_top/1000:.2f}km T@top={T0-L*h_top:.1f}K "
          f"H_shader={H:.0f}m top_shader={10*H/1000:.1f}km")
    for h in (0,1000,5000,10000,15000,20000,25000):
        if h*L >= T0: continue        # beyond the adiabatic limit
        p = p0*(1-L*h/T0)**k
        rho = (p/PA2ATM)*M/(R*(T0-L*h))
        print(f"  h={h/1000:5.1f}km p={p:8.5f}atm T={T0-L*h:7.2f}K rho={rho:9.5f}kg/m3")

profile("Earth",   1.0,   1.0, 288, 1.0,    0.02897,      1000.5)
profile("Jupiter", 11.0, 317.8, 165, 1.63,  0.0023139903, 12950.0)
profile("Venus",   0.95,  0.815, 735, 53.0, 0.02897,      1000.5)
profile("Mars",    0.533, 0.107, 278, 0.489,0.02897,      1000.5)
profile("Io",      0.286, 0.015, 130, 100/1225, 0.02897,  1000.5)
```
