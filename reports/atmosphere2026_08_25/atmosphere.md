# Atmospheric effects on bodies — design & future-enhancement notes

Date: 2026-08-25
Status: design (v1 implemented: Fresnel limb-glow shell, data-driven)

## TL;DR

Add a **simple, data-driven atmospheric rim** to bodies that have one. The
effect is a single smooth sphere shell slightly larger than the body, shaded
with a **Fresnel (limb) term** so it is transparent at the disk centre and
bright at the limb. One pass gives both the classic **atmospheric ring**
around the planet *and* the **horizon haze** over the surface, and it looks
correct from far away or in low orbit.

It is deliberately *not* a physically-scattering atmosphere (no Rayleigh/Mie,
no scale-height density falloff). It is the cheapest thing that reads as
"this world has air", and it slots cleanly into the existing renderer.
Everything needed to go *beyond* it (the physically-based path) is documented
in §8 so it can be picked up later without re-deriving the constraints.

The one non-obvious constraint that shapes the whole design: **the scene uses
a logarithmic depth buffer and the skybox is drawn *after* the opaque
planets**, so the atmosphere must (a) replicate the log-depth formula or it
will fight the depth buffer, and (b) be drawn *after* the skybox or its rim
ring gets overpainted. Both are handled; both are load-bearing.

---

## 1. What exists today

- **No atmosphere concept anywhere.** `grep -i atmos` over `src/` and `res/`
  matches nothing (only vendored `CLI11`/bullet strings). No shader, no data
  field, no mesh. `has_sea` is the only per-body "has something" flag, and in
  the KSP system the two `has_sea: true` bodies (Kerbin, Laythe) are exactly
  the two that have air.
- **A body is a `TerrainBody`** (`src/main.cpp:~207`): a cube of six `GeoPatch`
  quads displaced by simplex noise, LOD-subdivided. It is *not* a UV sphere —
  so the atmosphere cannot reuse the terrain mesh; it needs its own smooth
  sphere.
- **Planets are 100% procedural, vertex-coloured.** No planet textures exist.
  The surface colour is baked per-vertex from a `surface.palette` elevation
  ramp; the fragment shader does only a Lambert term
  (`res/terrainShader.fs`).
- **Logarithmic depth buffer.** Every scene vertex shader applies the Outerra
  trick (`logz = log(gl_Position.w*C+1)*FC; gl_Position.z=(2*logz-1)*w`) with
  `C=11, far=1e13` (`res/terrainShader.vs:26-34`, same in `sunShader`/
  `partsShader`), and the fragment shader writes `gl_FragDepth = logz`.
  `zNear=1 / zFar=1e13` (`src/main.cpp:~3419`). **Any fragment that
  depth-tests against terrain must write the same `gl_FragDepth`**, or the
  comparison is garbage.
- **Render state** (`src/display.cpp:75-83`): `GL_DEPTH_TEST` on (`GL_LESS`),
  `GL_CULL_FACE` on (back face, CCW front), no global blend. Blending is
  toggled per feature (engine plume `GL_ONE, GL_ONE` at `main.cpp:4426-4431`;
  HUD `SRC_ALPHA/ONE_MINUS_SRC_ALPHA` at `main.cpp:4437-4463`).
- **The render frame is the active ship's frame.** `camera->GetPos()`, the
  per-body `transform`, and the `Normal` uniform (which is actually the Model
  matrix → **world/render-frame normals**) all live in that frame
  (`GeoPatch::Draw`, `src/main.cpp:~745`).
- **Draw order** (`src/main.cpp` render loop, `~4270-4470`):
  clear → space ports → ships → **planets** (`planet->Update`; `planet->Draw`
  for each body) → **skybox** (starfield, `main.cpp:4402`) → engine plumes →
  HUD billboards → orbit lines → ImGui.
- **The skybox draws at the far plane** (`res/skyboxShader` uses
  `gl_Position = pos.xyww`, i.e. NDC z=1.0) with `GL_LEQUAL`
  (`src/skybox.cpp:125-148`). It is issued *after* the planets and fills only
  pixels whose depth is still the cleared 1.0. **A transparent object drawn
  before it, with depth-write off, in a pixel that has no opaque geometry,
  gets overpainted by the skybox.** This is why the atmosphere pass goes
  *after* the skybox (see §5).
- **The `Shader` class caps at 8 uniforms** (`MAX_NUM_UNIFORMS`,
  `src/shader.h:~41`). The atmosphere shader needs six — it fits.
- **Scene renders into an offscreen FBO** (`src/postfx.cpp`) when CRT is on;
  a normal RGBA8 target, so blending is unaffected. `Renderer::
  SaveScreenshot()` (`src/display.cpp:127`) gives a PNG for visual checks.
- **Reference implementation is vendored** if we later want the physical
  model: `references/pioneer/src/galaxy/AtmosphereParameters.h` (scale
  height, Rayleigh/Mie coefficients, `atmosRadius`, `atmosDensity`) and
  `SystemBody.h` (`HasAtmosphere()`, `GetAtmosphereFlavor(...)`,
  `ComputeDensity(...)`).

## 2. Goals / non-goals

**Goal (v1):** a per-body, data-driven atmospheric rim that reads as "this
world has air", correct at all camera distances, cheap (one small sphere +
one draw call per body), and non-invasive (no changes to terrain, physics, or
the existing draw order of opaque things).

**Non-goals (v1):** physical scattering, per-gas colour, clouds, dynamic
day/night terminator on the rim, atmospheric absorption of the surface behind
it, refraction. These are the §8 roadmap.

## 3. The effect, and why the cheap version is enough

The atmosphere is drawn as a sphere of radius `R_atm > R_body + max_relief`.
Only the **near hemisphere** is rendered (back face culled, as usual). The
fragment alpha is a Fresnel term:

```
N   = normalize(worldNormal)
V   = normalize(worldPos - cameraPos)     // camera -> point
rim = clamp(1.0 + dot(N, V), 0.0, 1.0)    // 0 at disk centre, 1 at limb
a   = pow(rim, power) * intensity
gl_FragColor = vec4(color, a)
gl_FragDepth = logz                        // MUST match the terrain VS
```

Sign check (near hemisphere): at the disk centre `N ≈ -V` so `dot(N,V) ≈ -1`
→ `rim ≈ 0` → transparent. At the limb `N ⊥ V` so `dot ≈ 0` → `rim ≈ 1` →
bright. So `rim = clamp(1 + dot(N,V), 0, 1)` is 0→1 from centre→limb, which
is exactly the falloff we want.

**Why one near-hemisphere pass gives both features:**
- The near hemisphere projects to the *full* atmosphere disc (angular radius
  `asin(R_atm/d)`), which is larger than the planet disc (`asin(R_body/d)`).
- The **annulus** between the two discs is the atmospheric ring — alpha rises
  to 1 at the outer edge. It blends over the starfield.
- The region **inside** the planet disc is the horizon haze — alpha is lower
  toward the centre and peaks at the planet's limb, so the surface reads as
  hazed near the horizon and clear at the nadir. It blends over the terrain.

The near hemisphere is *strictly in front of* the terrain (it is closer to
the camera by `R_atm - R_body`), so with depth-test on and depth-write off it
never z-fights the surface — it simply passes and blends on top.

## 4. Load-bearing constraints (get these wrong and it breaks)

1. **Log-depth replication.** The atmosphere vertex shader must compute
   `logz` with the *identical* `C=11, far=1e13` constants and the fragment
   shader must write `gl_FragDepth = logz`. Otherwise the depth test against
   the terrain (and the far-plane skybox) is meaningless. Copy the block
   from `res/terrainShader.vs:26-34` verbatim.
2. **Draw order: after the skybox.** The skybox (far plane, `GL_LEQUAL`) is
   issued after the opaque planets and would overpaint a depth-write-off rim
   ring. So the atmosphere pass is a *separate* pass placed after
   `skybox.Draw(...)` (`main.cpp:4402`), iterating bodies that have one.
   (Alternative — moving the skybox to *before* the planets — also works and
   is the more standard order, but it touches existing code; the additive
   separate pass is less invasive.)
3. **Shell radius > `radius + amplitude`** (and above `sea_level`), else
   high-relief terrain pokes through the shell. Compute `R_atm = radius +
   surface.max_height + thickness`.
4. **Render-state hygiene.** Enable blend, `glDepthMask(false)`, keep
   depth-test and cull as-is, draw, then restore blend off + `glDepthMask
   (true)`. The stencil state is already off at that point (the terrain
   passes disable it), but do not assume it — save/restore what we touch.
5. **GLSL 120.** `attribute`/`varying`, not `in`/`out`. `pow/clamp/
   normalize/dot` are all fine.
6. **Coordinate space.** Pass `cameraPos` in the ship/render frame
   (`camera->GetPos()`), and use the Model matrix (the `Normal` uniform) for
   world position + normal — the same convention the terrain uses.
7. **≤8 uniforms.** `MVP, Normal, cameraPos, color, intensity, power` = 6.

## 5. Where the code goes

| Piece | Location | Change |
|---|---|---|
| `Surface` struct | `src/main.cpp:~168` | Add `Atmosphere` sub-struct: `enabled, color(vec3), thickness(m), power, intensity`. |
| Data parse | `src/main.cpp` `load_system` surface block (`~460-505`) | Parse optional `surface.atmosphere { color, thickness, power, intensity }`. |
| Body member | `src/main.cpp` `TerrainBody` (`~207`) | `Model *atmosphere = nullptr;` + `BuildAtmosphere()` (lazily builds the sphere `Mesh` + `Shader` only when enabled). |
| Sphere mesh | `src/main.cpp` (near `create_grid_mesh`) | `create_sphere_mesh(lat, lon, radius)` → `PosNorIndInterface` UV sphere, `mesh->InitMesh(...)`. |
| Shader setup | `src/main.cpp` shader init (`~2976-3000`) | `atmosphereshader = new Shader; registerAttribs({position,normal}); registerUniforms({MVP,Normal,cameraPos,color,intensity,power}); FromFile("./res/atmosphereShader");` |
| Draw | `src/main.cpp` `TerrainBody::DrawAtmosphere(camera, renderFrame)` (new) + a loop **after** `skybox.Draw` (`main.cpp:4402`) | Per body: set `MVP=Projection*(View*transform)`, `Normal=transform`, `cameraPos`, `color/intensity/power`; enable blend, depth-mask off; `mesh->Draw()`; restore. |
| Shaders | `res/atmosphereShader.vs` / `.fs` (new) | Per §3; VS copies the log-depth block. |
| Data | `gen_systems.py` + `ksp_system.json` + `system.json` | Add `surface.atmosphere` to Kerbin, Laythe (blue) and Eerbon (blue). |

## 6. Data model

Optional `surface.atmosphere` block; absent ⇒ no atmosphere (so `has_sea`
bodies without an explicit block stay atmosphere-less, and a body can have an
atmosphere without a sea, e.g. a future Duna CO2 rim):

```json
"surface": {
  "...": "...",
  "atmosphere": {
    "color":     [0.30, 0.50, 1.00],   // rim tint (N2/O2 blue)
    "thickness": 20000,                 // [m] shell radius above radius+max_height
    "power":     3.0,                   // Fresnel falloff (higher = tighter rim)
    "intensity": 1.0                    // overall alpha scale
  }
}
```

Defaults (used when a field is omitted but the block is present): a blue
`[0.3, 0.5, 1.0]`, `thickness` ~3% of radius, `power` 3.0, `intensity` 1.0.

Bodies to enable first: **Kerbin** and **Laythe** (`ksp_system.json`),
**Eerbon** (`system.json`).

## 7. Verification

- Build (`make`), run the game, orbit to Kerbin/Laythe and confirm: a blue
  rim ring around the disc, haze intensifying toward the horizon, nothing
  z-fighting the terrain, the starfield still visible outside the rim.
- Screenshot path: `Renderer::SaveScreenshot` (`src/display.cpp:127`) — a
  PNG for a side-by-side without a live display.
- Render-state regression: after the atmosphere pass, ships/plumes/HUD must
  render unchanged (blend off, depth-mask back on, stencil off).
- e2e battery (`make e2e`) should stay green — it exercises the render loop
  we are inserting a pass into.

## 8. Future enhancements (the roadmap this report is also notes for)

Ordered roughly by value-per-effort. Each is a local extension of the v1
shell, not a rewrite — the data block, the shell mesh, the draw pass, and the
log-depth convention all carry over.

1. **Sun-facing limb tint (dusk/dawn).** Modulate `intensity` by how much the
   limb faces the sun: `face = clamp(dot(N, -lightDirection), 0, 1)`, and
   lerp the rim colour toward a warm `[1.0, 0.5, 0.3]` as `face`→0 on the
   sun side. Needs the `lightDirection` the terrain already receives
   (`TerrainBody::SunlightDir`, `src/main.cpp:~249`). Turns the flat blue rim
   into a lit-day / dark-night terminator. ~10 lines of shader + 1 uniform.
2. **Physically-based scattering (Rayleigh/Mie).** Replace the flat `color`
   with a wavelength-dependent scatter: short path (nadir, centre) → blue;
   long path (limb, tangent) → redder. This is the real "blue sky, red
   sunset" effect. The vendored Pioneer model is a proven starting point:
   `references/pioneer/src/galaxy/AtmosphereParameters.h`
   (`atmosInvScaleHeight`, Rayleigh/Mie coefficient tables) and
   `SystemBody.h` (`ComputeDensity(radius, height, h, scaleHeight)`,
   `GetAtmosphereFlavor`). Adds a `scaleHeight` + `density` + per-gas
   coefficient set to the data block. This is the big one; do it after the
   rim looks good.
3. **Per-gas atmospheres.** Key the colour/flavour on a `gas` field
   (`N2_O2` blue, `CO2` tan/red, `CH4` orange) rather than a raw colour, so
   new bodies (Eve, Duna) get plausible air for free. Trivial data change;
   the colour just becomes a lookup.
4. **Surface absorption / air-mass tint.** When looking *through* the
   atmosphere at the surface, tint by slant path length (more haze at the
   limb). This is the "back" of the same Fresnel term applied to the terrain
   fragment rather than the shell — a small term in `terrainShader.fs`, or a
   second shell pass with different blending.
5. **Sky dome when on the surface.** From the ground the shell is a blue
   dome overhead (currently it would cover the starfield skybox — physically
   correct, but a design call). If we want a proper sky: a gradient from
   zenith (deep blue) to horizon (pale), a sun disc + glow, and the §2
   terminator. Interacts with the skybox, so sequence after §2.
6. **Clouds.** A separate, slowly-rotating textured shell just inside the
   atmosphere (the `pos+uv` mesh layouts in `src/mesh.h` already exist).
   Needs a planet cloud texture (the first planet texture in the project) or
   a procedural noise pass.
7. **Transparent sorting.** v1 draws all atmospheres in one pass in body
   order; two overlapping atmosphere rims on screen blend in the wrong order.
   Rare (bodies are far apart) but fixable with a back-to-front sort of the
   atmosphere pass by camera distance.
8. **Depth-bias the shell** if any limb z-fighting ever shows up at extreme
   ranges (a small `logz` nudge or a slightly larger `thickness`). Should not
   be needed given §4.3.
9. **`MAX_NUM_UNIFORMS` headroom.** If §2/§5 add many parameters (Rayleigh/
   Mie vectors, sun colour, sky gradient stops), the 8-uniform cap in
   `src/shader.h:~41` will be hit. Either bump the cap (it is a fixed array,
   cheap) or pack coefficients into `vec4` uniforms. Decide when it bites.

## 9. What we are *not* doing (and why)

- **Not** a volume/ray-marched atmosphere in v1. The renderer already has a
  ray-march precedent (`ComputeTerrainShadow`, `src/main.cpp:~2233`), so it is
  *possible*, but a per-fragment march over the enormous `zFar=1e13` range is
  the expensive path and buys little over a Fresnel shell until we do the
  physical model (§8.2). Defer.
- **Not** coupling atmosphere to `has_sea`. They are unrelated (air ≠ ocean);
  an explicit `atmosphere` block is cleaner and lets a body have one without
  the other.
- **Not** reusing the terrain mesh. It is a noise-displaced cube of quads;
  the atmosphere wants a smooth sphere. A small dedicated UV sphere is simpler
  and LOD-consistent (one sphere at any range).
- **Not** changing the existing opaque draw order. The atmosphere is an
  additive pass after the skybox; the planets/ships/skybox keep their current
  relative order.

## 10. Key code references

| Concern | Location |
|---|---|
| Body type / 6 patches | `src/main.cpp:~207` `TerrainBody`, `:240` `Create()` |
| Surface params struct | `src/main.cpp:~168` `Surface` |
| Body draw (2 stencil passes) | `src/main.cpp:~265` `TerrainBody::Draw` |
| Patch leaf draw / MVP+Normal | `src/main.cpp:~723` `GeoPatch::Draw` |
| Grid mesh + vertex colour | `src/main.cpp:~2329` `create_grid_mesh` |
| Terrain height / noise | `src/main.cpp:~2183` `noise3d`, `GetTerrainHeight` |
| Sun direction | `src/main.cpp:~249` `SunlightDir` |
| Terrain shadow ray-march (precedent) | `src/main.cpp:~2233` `ComputeTerrainShadow` |
| Data load + surface parse | `src/main.cpp:405` `load_system`, `~460-505` |
| Shader init block | `src/main.cpp:~2976-3000` |
| Render loop / draw order | `src/main.cpp:~4270-4470` |
| Skybox (far plane, drawn after planets) | `src/main.cpp:4402`, `src/skybox.cpp:125-148` |
| Log-depth formula | `res/terrainShader.vs:26-34`, `res/terrainShader.fs` |
| GL base state (depth/cull) | `src/display.cpp:75-83` |
| Blend usage (plume / HUD) | `src/main.cpp:4426-4431`, `4437-4463` |
| 8-uniform cap | `src/shader.h:~41` |
| FBO / post-fx (blend-safe) | `src/postfx.cpp` |
| Screenshot (visual check) | `src/display.cpp:127` `SaveScreenshot` |
| Physical atmosphere reference | `references/pioneer/src/galaxy/AtmosphereParameters.h`, `SystemBody.h` |
| Data files | `ksp_system.json` (default), `system.json` (Eerbon), `gen_systems.py` |
