# Switching to SDL_gpu — evaluation

Date: 2026-09-13. Status: research/decision report (no code changed).

## TL;DR

**Recommendation: not yet, for this game, at this time.** SDL_gpu is a solid,
official, actively-developed cross-vendor GPU API, and the *plumbing* to adopt it is
cheap (the official ImGui backend is already vendored; the e2e box even has a software
Vulkan driver). But it is **not a drop-in replacement for what this game actually does
with OpenGL**, and the one thing the game most needs — writing a per-fragment depth
value (`gl_FragDepth`, the log-z hack) — is **not exposed by SDL_gpu's shader model**.
That is a hard functional blocker for a space sim whose entire depth strategy depends
on it. Secondary regressions (wireframe, line width, free MSAA) are smaller but real.
The performance upside is weak, because the game is CPU/physics-bound, not
GPU-bound.

The right trigger to revisit this is **when a second platform (macOS, or a real
Vulkan/Metal/D3D12 target) actually matters**, because that is SDL_gpu's genuine
superpower. Until then, raw GL 4.5 is the lower-risk, lower-effort choice and we are
already deep into it. If we do move, it is a **whole-render-layer rewrite**, not a
port — budget it as such, and do it with the log-z replacement designed first.

---

## 1. What we actually have today

It is worth being precise, because the framing "we just switched to SDL3, now consider
SDL_gpu" hides the real situation: **we do not render with SDL at all.** SDL3 is used
only for windowing, events, keyboard/mouse, and surfaces. The 3D image is drawn with
**raw OpenGL 4.5 core via GLEW**, and the HUD with **Dear ImGui's OpenGL3 backend**.

Verified from the tree:

- `Makefile`: links `libGLEW.a`, `-lGL`, `libSDL3.a`. ImGui builds
  `imgui_impl_opengl3.o` (not the SDL renderer, not an SDL_gpu backend).
- `bootstrap.sh`: SDL3 is built static with **`-DSDL_GPU=OFF -DSDL_VULKAN=OFF`**
  and `-DSDL_WAYLAND=OFF -DSDL_X11=ON`. The comment says it plainly:
  "the game runs on X11 … and uses GL 4.5 via GLEW."
- `src/display.cpp`: requests a **core-profile** GL context with configurable
  major/minor, `SDL_GL_DEPTH_SIZE 24`, `SDL_GL_STENCIL_SIZE 8`, and MSAA via context
  attributes (`SDL_GL_MULTISAMPLESAMPLES`). So **MSAA is free** — it is a property of
  the default framebuffer we ask the driver for.
- Shaders are **GLSL 450** files in `res/` (~10 programs: parts, terrain, skybox, sun,
  atmosphere, clouds, billboard, line, line2, postfx quads).
- Feature inventory of the GL surface (from a count of every `gl*` call in `src/`):
  VAOs, interleaved VBOs, 2D + cube textures, mipmaps, blending, depth, **stencil**,
  **wireframe (`glPolygonMode`)**, **line width (`glLineWidth`)**, FBOs + renderbuffers
  (the post-fx chain), `glReadPixels` (screenshots), `glDrawElementsBaseVertex`
  (baseVertex is **always 0** — clean), `GL_LINE_LOOP` (skyline).

So "switching to SDL_gpu" really means **replacing the entire OpenGL render layer
(GLEW + GLSL + VAO + FBO + the depth hack) with the SDL_gpu command-buffer/pipeline
model**, and swapping the ImGui backend. Physics (Bullet3), input, and the math core
are untouched.

## 2. What SDL_gpu is

From the vendored header (`middleware/sdl3/include/SDL3/SDL_gpu.h`, SDL 3.4.16) and
the SDL docs:

- A **full explicit GPU API in the Vulkan/D3D12/Metal mould** — not a thin wrapper and
  not a state machine. You create a device, claim a window (which makes a swapchain),
  create pipelines/buffers/textures/samplers, record into a command buffer, and
  submit with fences. It maps roughly 1:1 onto the three native backends.
- **Backends:** Vulkan, Direct3D 12, Metal.
- **Stable:** it is part of the official SDL 3 release (3.2.0 — our vendored
  `WhatsNew.txt` marks 3.2.0 as "the SDL 3.0 release"), and the released entries in our
  vendored `WhatsNew.txt` are all additive (new features + bugfixes). Actively
  developed: our vendored copy is **3.4.16, tagged 2026-09-02**, with GPU fixes in
  recent 3.4.x releases (e.g. 3.4.0 added GPU device-creation properties).
- **Compute:** yes (`SDL_DispatchGPUCompute`, storage buffers, std430 layout). This is
  the big new capability we do not have today.
- **No geometry/mesh/tessellation shaders** — the only shader stages are vertex and
  fragment. (Fine for this game; we use none.)
- **Shaders are precompiled bytecode only** — SPIR-V (Vulkan), DXBC/DXIL (D3D12),
  MSL/METALLIB (Metal). **No runtime GLSL/HLSL source.** We would compile our GLSL to
  SPIR-V with `glslc` (none of `glslc`/`dxc`/`spirv-cross` is installed on this box;
  it is a new build dependency).
- **Fixed descriptor-set layout** — this is the #1 reported shader-writing gotcha. The
  `SDL_CreateGPUShader()` doc in `SDL_gpu.h` ("For SPIR-V shaders, use the following
  resource sets") fixes four sets: **set 0** = vertex sampled + storage textures and
  storage buffers; **set 1** = vertex uniform buffers; **set 2** = fragment sampled +
  storage textures and storage buffers; **set 3** = fragment uniform buffers. Our
  shaders use unqualified `uniform`/`sampler2D` with no explicit sets, so **every
  shader needs `layout(set=…, binding=…)` added** to match.
- **Depth formats:** D16, D24, D32_FLOAT (+stencil variants). Good — 32-bit float depth
  is available.

## 3. Feature mapping — what transfers, what's hard, what's blocked

| Game feature (GL today) | SDL_gpu | Verdict |
|---|---|---|
| Mesh draw, VBO/VAO, interleaved attrs | `SDL_GPUBuffer` + vertex input state; no VAO concept | ✅ clean (simpler — no VAO state) |
| Textures (2D, mipmaps, linear) | `SDL_GPUTexture` + `SDL_GPUSampler` + `SDL_GenerateMipmapsForGPUTexture` | ✅ clean |
| Cube-map skybox (`GL_TEXTURE_CUBE_MAP`) | `SDL_GPU_TEXTURETYPE_CUBE` | ✅ clean |
| Blending, cull, depth test | `SDL_GPUGraphicsPipelineCreateInfo` (baked into pipeline) | ✅ clean |
| **Terrain stencil trick** (stamp=1, draw skirts where 0) — `src/terrain.h:449` | Full `SDL_GPUStencilOpState` in pipeline | ✅ clean (maps directly) |
| `glDrawElementsBaseVertex` (baseVertex always 0) | `SDL_DrawGPUIndexedPrimitives` (has `first_index`) | ✅ clean |
| Post-fx FBO chain + depth renderbuffer (`src/postfx.cpp`) | Render passes + textures + a D24/D32 depth attachment | ✅ clean (more explicit code) |
| Screenshots `glReadPixels` (`src/display.cpp:354`) | `SDL_DownloadFromGPUTexture` | ✅ clean (arguably easier) |
| **MSAA** — free via context attr (`src/display.cpp`) | **Must create an MSAA color+depth attachment and resolve** in the pass | ⚠️ more code, small perf cost (a resolve blit) |
| **Wireframe** F11 debug (`glPolygonMode`, `src/events.cpp:357`, `src/main.cpp:759`) | **No rasterizer wireframe mode** | ⚠️ regression — must fake with a barycentric-coord fragment trick or emit explicit line geometry |
| **Line width** `glLineWidth(4)` skyline (`src/render.cpp:456`) | **No line width >1px in any backend** | ⚠️ regression — the 4px skyline line becomes 1px, or redrawn as quads |
| **Log-z depth hack** `gl_FragDepth` (parts, terrain, sun, cloud, atmosphere shaders) | **No fragment depth-write / `SV_Depth` / `gl_FragDepth` primitive** | 🔴 **BLOCKER** — see §4 |
| ImGui + ImPlot HUD | **Official `imgui_impl_sdlgpu3` already in our vendored ImGui 1.92.9b** | ✅ near drop-in (swap the backend, feed it the swapchain format) |
| Physics (Bullet3, double precision) | Untouched (CPU side) | ✅ n/a |

The pattern: **almost everything is a clean 1:1**, and the "hard" items (wireframe,
line width, manual MSAA) have workarounds. The **one blocker is the depth hack**, and
it goes to the heart of how a space sim renders.

## 4. The blocker: the log-z depth hack

**Five** shaders do the
[logarithmic depth buffer trick](http://outerra.blogspot.com/2009/08/logarithmic-z-buffer.html)
(`partsShader.fs`, `terrainShader.fs`, `sunShader.fs`, `cloudShader.fs`,
`atmosphereShader.fs`): the vertex shader computes `logz` (with `far = 1e13`), and the
fragment shader writes `gl_FragDepth = logz;`. This is what lets a single depth buffer
resolve both "on the planet's surface" and "a far body in space" without z-fighting —
the whole reason a KSP-style game can show the world at every zoom level.

**SDL_gpu does not expose this.** I read the header directly. The only depth-related
pipeline state is:

- `SDL_GPURasterizerState.enable_depth_write` — a boolean on/off, not a per-fragment
  value, and
- `depth_bias_*` — a constant/slope/clamp offset, not a nonlinear per-fragment remap.

There is no `gl_FragDepth`, no `SV_Depth`, no "write depth from the fragment shader"
primitive anywhere in `SDL_gpu.h`. You *can* pick a `D32_FLOAT` depth attachment
(which is what Vulkan/D3D12/Metal require for shader depth writes), but SDL_gpu's shader
model does not hand you the depth output.

Why this is a **hard** blocker and not a "work around it":

- Standard perspective depth over a 1e13 range is hopeless — that is *exactly* why the
  log-z hack exists. Removing it and relying on `D32_FLOAT` + a normal near/far would
  reintroduce z-fighting between terrain/skirts and between near/far bodies, which is
  precisely what the hack and the stencil-skirt trick currently prevent together.
- It *could* be done on the **Vulkan backend only**, by writing a hand-crafted SPIR-V
  fragment shader with a depth output (Vulkan allows it when the attachment is
  D32_FLOAT). But that is (a) outside SDL_gpu's supported abstraction, (b) **will not
  port to Metal or D3D12**, and (c) defeats the purpose of adopting a cross-vendor
  API. So it is not a real option for the stated goal.

Mitigations, if we ever move: (1) lean harder on **floating origin** (we already have
that — `reports/floating-origin`) to shrink the *relative* depth range, combined with
`D32_FLOAT` + reverse-Z; (2) split the scene into depth-partitioned passes. Both are
**rendering-strategy changes with their own risk**, and neither is a "switch and done."
This is the single reason to treat SDL_gpu as a project, not a port.

## 5. The good news: the ecosystem is ready

This is where SDL_gpu is genuinely pleasant:

- **Dear ImGui ships an official SDL_gpu backend**, and we already vendor ImGui
  **1.92.9b** which **includes `backends/imgui_impl_sdlgpu3.{h,cpp}`** plus a
  `sdlgpu3/` folder with the shader sources + build instructions. The HUD/panels (the
  largest UI surface, `src/gameui.cpp` at ~2150 lines, plus ImPlot) would move by
  swapping `imgui_impl_opengl3` → `imgui_impl_sdlgpu3` and feeding it the swapchain
  format. ImPlot is backend-agnostic, so it rides along. This is the one big piece
  that is *not* a rewrite.
- **SDL_shadercross** (official, `libsdl-org`) cross-compiles SPIR-V/HLSL →
  DXBC/DXIL/SPIR-V/MSL, so the shader toolchain has a first-party path.
- There are solid community examples (`TheSpydog/SDL_gpu_examples`) and a full
  learning guide, and at least a few small games/engines run on it.

## 6. Build / toolchain impact

- **New build dependency:** `glslc` (glslang) to compile GLSL → SPIR-V, and/or
  `SDL_shadercross` for the other targets. Neither `glslc`, `dxc`, nor `spirv-cross`
  is installed here. This is a bootstrap.sh change.
- **SDL3 rebuild:** flip `bootstrap.sh` from `-DSDL_GPU=OFF -DSDL_VULKAN=OFF` to
  `-DSDL_GPU=ON -DSDL_VULKAN=ON` (and drop `-DSDL_OPENGL=ON`/GLEW once we stop using
  GL). Binary size grows: the SDL3 port deliberately compiled out the GPU + Vulkan
  glue (listed as "unused" alongside the 2D renderer, the ~1 MB it trimmed for size).
- **Shader edits:** add `layout(set=…, binding=…)` to every shader to satisfy SDL's
  fixed descriptor layout; re-express the log-z (see §4) or accept the regression.
- **Drops:** GLEW and the system `libGL` go away. Net: one dependency added (glslc),
  two removed (GLEW, libGL).

## 7. Performance — the weak motivator

**There are no published benchmarks of SDL_gpu vs. raw GL/Vulkan**, and I found none.
What the design and community say:

- It is an **explicit, 1:1 mapping** onto the native backends and spawns no threads of
  its own — the application owns the submission thread. The overhead is CPU-side
  command-buffer recording and explicit synchronization, plus a **resolve blit for
  MSAA** we don't pay today.
- Practical guidance: keep render passes **long** (pass start is the main CPU cost),
  use **push constants** (`SDL_PushGPU*UniformData`) for small per-draw data, and
  lean on SDL's internal **resource cycling** for per-frame buffer updates.
- **For this game specifically, the GPU is not the bottleneck.** The heavy work is
  Bullet3 double-precision dynamics + the job-runner physics, which run on the CPU and
  are identical under either API. The render loop here draws a modest scene (a handful
  of meshes, terrain patches, a skybox, a post-fx chain) — comfortably within what a
  software rasterizer can do. So the **performance case for switching is weak**; we
  would be trading a known-quantity (GL 4.5 on llvmpipe/Mesa) for an unknown, for
  little measurable gain on the box we actually run on.

If performance ever *does* become the driver, the honest options are (a) raw Vulkan
(maximum control, including the fragment-depth write we need) or (b) GPU compute for
terrain/particles — and note SDL_gpu *does* give us compute, which raw-GL-4.5 also
gives us via `GL_ARB_compute_shader`/GL 4.3+. So even the compute motivation is not
unique to SDL_gpu.

## 8. e2e / CI impact — a real, quiet risk

Our e2e battery (`e2e/run.py`) launches `./osp` under **Xvfb** (software GL via
Mesa/llvmpipe) and asserts on stdout. That path is green and well-trodden.

Under SDL_gpu, the game would need a **Vulkan device** instead. On this box that
means **lavapipe** (Mesa's software Vulkan, `lvp_icd.json` is present,
`mesa-vulkan-drivers` installed) — *if* we target Vulkan. Concerns:

- **Lavapipe is generally reported to be slower and less complete than llvmpipe.**
  Feature gaps and driver bugs in software Vulkan are more common. Several e2e cases (13 frame-cap, 14/15
  post-fx, 31-34 gamma/color/post-fx, 33 antialiasing) are exactly the
  rendering-sensitive ones most likely to misbehave under a different rasterizer.
- **The 120s default case limit** is tight for a software Vulkan path; the battery may
  start timing out for no gameplay reason.
- **The log-z blocker (§4) would surface here first** — depth regressions show up as
  z-fighting, which is visual and easy for a stdout-assertion battery to miss. That is
  a testing gap: our e2e checks numbers, not pixels. A GPU swap needs *visual*
  regression coverage we do not currently have.

Net: switching the GPU stack **weakens our most reliable CI signal** until we rebuild
confidence in the lavapipe path and add screenshot/pixel checks.

## 9. Effort estimate (if we proceed)

This is a **render-layer rewrite**, not a port. Order matters:

1. **Design the depth strategy first** (log-z replacement or floating-origin +
   reverse-Z). Everything else is mechanical; this is the risk. *Gate the whole
   project on this.*
2. **Toolchain:** add glslc/SDL_shadercross to bootstrap; flip SDL3 to
   `SDL_GPU=ON/VULKAN=ON`; get a triangle + a textured quad through a render pass on
   lavapipe under Xvfb. *Prove the e2e box can even run it.*
3. **Port the core draw path:** mesh, texture, parts, terrain (incl. the stencil
   skirt trick), skybox, sun, atmosphere, clouds, billboard, lines. Replace VAOs/FBOs
   with buffers/passes; add `layout(set/binding)` to shaders.
4. **Post-fx chain:** FBOs → offscreen textures + passes; wire the MSAA resolve.
5. **HUD:** swap ImGui to `imgui_impl_sdlgpu3`; re-verify all panels + ImPlot.
6. **Fix the regressions:** wireframe (F11), line width (skyline), screenshots.
7. **CI:** get the full e2e battery green under lavapipe; add a screenshot/pixel
   comparison for the depth-sensitive scenes.

Files touched (14 files carry direct `gl*` calls today): `src/render.cpp`,
`mesh.cpp`, `texture.cpp`, `shader.cpp`, `postfx.cpp`, `skybox.cpp`,
`billboard.cpp`, `terrain.{h,cpp}`, `body.h` (per-part texture bind + uniforms in
`DrawAt`), `physics.cpp` (debug lines), `display.cpp`, `events.cpp` (wireframe),
`main.cpp`, `gldebug.{h,cpp}` (GL debug callback — retired), plus `Makefile`,
`bootstrap.sh`, all 18 `res/*.vs`/`*.fs`, and the ImGui build target. Note the GL is
**stateful immediate-mode** (per-draw `glEnable`/`glBlendFunc`/`glDepthMask`/
`glCullFace` toggles, the Mesa default-VAO-0 quirk) — that state must become baked
pipelines, not just a 1:1 call swap. **Not touched:** physics, input, math core, data
model, UI *logic* (only its renderer).

Realistic shape: a multi-week, high-careful effort with a real chance of a depth
regression that the current CI will not catch. That is not a "quick switch."

## 10. Alternatives

- **Stay on GL 4.5 (status quo).** Lowest risk, lowest effort. Fine on Linux/X11
  (where we are). The cost: **macOS caps at OpenGL 4.1** (no 4.5 core), and Windows
  core-GL is a dead end long-term. So this is only "fine" while we are single-platform.
- **Raw Vulkan.** Maximum control — *including the fragment-depth write we need* — but
  the most code, and we'd own all the driver/MSAA/sync complexity. Only worth it if
  we need the features SDL_gpu withholds and we have Vulkan expertise on hand.
- **A small engine (tinygl / ngl).** A higher-level Vulkan wrapper; less code than raw
  Vulkan, but a third-party dependency and its own abstraction limits — and it may
  also not expose fragment-depth writes.
- **SDL_gpu.** The middle path: official, stable, compute included, official ImGui
  backend. Blocked for *this* game by the depth hack, ideal for a *portable* game.

## 11. Different lenses (the "diverse perspectives" pass)

- **The "what are we optimizing for" lens:** if the goal is *this game on this box*,
  SDL_gpu buys almost nothing and risks the depth model. If the goal is *a game that
  ships on Mac + Windows + Linux from one codebase*, SDL_gpu (or raw Vulkan) is the
  obvious move and GL 4.5 is the thing that will break on macOS. **The answer depends
  entirely on whether a second platform is real.** It is not, today.
- **The "sunk cost / momentum" lens:** we are deep into GL (VAOs, FBOs, a tuned
  depth hack, a working CI). Rewriting the render layer is a large, visible,
  regression-prone change. That inertia is a *real* cost, not a bug — but it is an
  argument for a deliberate decision, not a drift.
- **The "option value" lens:** the cheapest thing we can do now is **keep the render
  code behind a thin boundary** (it mostly already is: `render.cpp`/`mesh.cpp`/
  `texture.cpp`/`shader.cpp` are the GL surface). Maintaining that boundary keeps the
  SDL_gpu/Vulkan door open for a few weeks of work instead of a rewrite, *if* we ever
  need it. Worth doing either way.
- **The "compute is the real prize" lens:** the one capability SDL_gpu genuinely adds
  is GPU compute (terrain culling, particle sims, maybe offloading some dynamics).
  If *that* is the attraction, note GL 4.5 already gives us compute shaders, so we
  could capture most of the prize **without** the depth-hack regression — by adopting
  GL compute first and deferring the backend switch.
- **The "testing honesty" lens:** our CI asserts on *numbers*, not pixels. Any GPU
  swap that changes depth/MSAA/wireframe is exactly the kind of change this CI will
  let through broken. A precondition of *any* GPU change (not just SDL_gpu) is adding
  a visual/pixel regression check. That is worth building regardless.

## 12. Decision criteria (what would flip this)

Revisit "switch to SDL_gpu" when **any** of these become true:

1. **A second platform is a real target** (macOS, or Windows/Mac from one tree) — this
   is the killer feature and GL 4.5 is the thing that breaks.
2. We have **designed and validated a log-z replacement** (floating-origin +
   reverse-Z on D32_FLOAT, or depth-partitioned passes) and proven it holds on
   lavapipe.
3. We have **added pixel/screenshot regression coverage** to e2e and are comfortable
   running the battery under lavapipe within its time limits.
4. We want **GPU compute** for terrain/particles *and* are willing to pay the
   rewrite cost to get it portably.

None of these are true today, so the recommendation stands: **keep GL 4.5, keep the
render layer behind its existing boundary, and treat SDL_gpu as the planned path the
moment a second platform or a compute feature makes it necessary.**

## 13. References

- Vendored API: `middleware/sdl3/include/SDL3/SDL_gpu.h` (SDL 3.4.16) — depth state
  (§`SDL_GPURasterizerState`), formats (D32_FLOAT), primitive types, shader formats,
  push constants, transfer buffers.
- Build: `bootstrap.sh` (`-DSDL_GPU=OFF -DSDL_VULKAN=OFF`), `Makefile` (GLEW/`-lGL`,
  `imgui_impl_opengl3`).
- Depth hack: `gl_FragDepth = logz` in `res/{parts,terrain,sun,cloud,atmosphere}Shader.fs`
  (with `far = 1e13` in the matching `.vs`); stencil skirt: `src/terrain.h:449-456`.
- Regressions: wireframe `src/events.cpp:357`, `src/main.cpp:759`; line width
  `src/render.cpp:456`; free MSAA `src/display.cpp:54-56`.
- Ecosystem: `middleware/imgui/backends/imgui_impl_sdlgpu3.{h,cpp}` (ImGui 1.92.9b),
  `backends/sdlgpu3/build_instructions.txt`; `libsdl-org/SDL_shadercross`;
  `TheSpydog/SDL_gpu_examples`.
- e2e: `e2e/run.py` (Xvfb, 120s limit); lavapipe `lvp_icd.json`, `mesa-vulkan-drivers`.
- SDL docs: `wiki.libsdl.org/SDL3/CategoryGPU`, `SDL_CreateGPUDevice`,
  `SDL_DispatchGPUCompute`, `SDL_ClaimWindowForGPUDevice`, `SDL_WaitAndAcquireGPUSwapchainTexture`.
- Context: `reports/floating-origin2026_08_25` (existing depth-range strategy).
