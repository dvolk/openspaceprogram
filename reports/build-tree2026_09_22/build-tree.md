# Build tree — plan

Date: 2026-09-22
Status: planning (no code written)

## TL;DR

Move every byte of build output out of the repo root and the vendored
submodules, into one tree named by its real build dimensions:

    build/<os>-<march>-<mtune>/<config>/

With the middleware built once per (os, march, mtune) and **shared** across
the configs that link it. Today the binaries and `obj*` dirs sit loose in the
repo root and each middleware builds inside its own submodule (`middleware/<n>/build`),
which clutters the working tree and hides the fact that we already build
several *different* binaries (release/asan/tsan) and one ISA baseline.

The naming also does double duty as the reservation for cross-platform: a
future Windows or macOS build lands as `build/windows-...` / `build/macos-...`
without re-laying the tree. It does **not** make those build — that is a
separate, larger effort (Phase 1). This report scopes Phase 0 (this box,
Linux, fully testable now) and stubs Phase 1/2.

Defaults locked this session: `-march=x86-64-v2 -mtune=znver3` (runs on ~2010+
CPUs, tuned for a modern core), a new `debug` config, `native` kept as opt-in.

## Current state (verified 2026-09-22)

Build system
- Makefile + g++ (C++20), `-O2 -flto=$(nproc)`, `-march=$(MARCH)` where
  `MARCH ?= native`. **No `-mtune` anywhere.** Binary is ISA-locked to
  whatever `-march` was used (older CPU → SIGILL); the Makefile already notes
  a `make clean` is required after changing `MARCH`.
- Configs that exist today:
  - default → `osp` (release: `-O2 -flto`)
  - `make asan` → `osp_asan` (`-g3 -fsanitize=address,leak,undefined`), objects in `obj_asan/`
  - `make tsan` → `osp_tsan` (`-g3 -fsanitize=thread,undefined`), objects in `obj_tsan/`
  - `PGO=gen`/`PGO=use` — a 2-phase optimization mode; the final artifact *is* a release binary; profile data in `tmp/pgo`.
  - **No `make debug`** — there is no plain `-O0 -g3` build target.
- Object dirs at root: `obj/` (game, LTO bytecode), `obj_test/` (test objects),
  `obj_asan/`, `obj_tsan/`. Binaries at root: `osp`, `osp_asan`, `osp_tsan`,
  ~34 `test_*` binaries, `test_gl_vao`.

Middleware (built by `bootstrap.sh`, then linked by the Makefile)
- bullet3, SDL3, SDL_image, SDL3_mixer, assimp → `middleware/<name>/build/`
  (inside each submodule's working tree). GLEW (a fetched tarball, gitignored,
  not a submodule) → source in `middleware/glew/`, output in `middleware/glew/build-cmake/`
  (its CMake project is the *source* dir `middleware/glew/build/cmake`).
- All middleware is built **once, uninstrumented, and shared** by release/asan/tsan
  (instrumenting bullet3/SDL3 is slow/noisy; the Makefile says so).
- `bootstrap.sh` hardcodes `-march=native` **independently** of the Makefile's
  `MARCH` — so today a `make MARCH=x86-64-v3` game links native-tuned
  middleware. (See the constraint below.)

Platform
- Single platform, Linux/X11/PulseAudio. **No** `_WIN32`/`__APPLE__`/`_MSC_VER`/
  `__MINGW` in `src/` (verified), no CI config (no `.github/`, no `*.yml`),
  e2e runs under `xvfb-run`, the Makefile links a fixed Linux closure
  (`-lX11 … -lpulse -lasound -lGL`).

Tooling that hardcodes the binary location
- `e2e/run.py:400` — `os.path.join(REPO_ROOT, "osp")` (+ its error message/docstring).
- `README.md` + `bootstrap.sh` final echo — `./osp`.
- `.gitignore` — scattered build entries: `obj/`, `obj_test/`, `obj_asan/`,
  `obj_tsan/`, `osp`, `osp_asan`, `osp_tsan`, `/test_*`, `middleware/glew/`.

## Design (locked this session)

### Scheme

    build/<os>-<march>-<mtune>/<config>/

Tokens
- `<os>`: `linux`, `windows`, `macos` (names a platform + its toolchain).
- `<march>`: `base` (x86-64), `v2` (x86-64-v2), `v3` (x86-64-v3), `native`
  — the **compatibility contract**: which ISA the binary may use.
- `<mtune>`: any g++ `-mtune=` value — `generic`, `skylake`, `icelake`,
  `znver2`, `znver3`, `native`, … — the **perf knob**; does not change compatibility.
- `<config>`: `release`, `debug`, `asan`, `tsan`.

### Layout (nested, middleware shared)

    build/
      linux-v2-znver3/
        middleware/            # built ONCE per (os,march,mtune), shared by all configs
        release/ → osp, obj/
        debug/   → osp, obj/
        asan/    → osp_asan, obj/
        tsan/    → osp_tsan, obj/
      linux-native-generic/    # opt-in, only if someone builds it

The middleware sits at the (os, march, mtune) level, **not** per-config,
because it is built once and shared (the existing design). This is why the
layout is nested rather than a flat self-contained `build/<os>-<march>-<mtune>-<config>/`
leaf — a flat leaf would rebuild bullet3/SDL3 once per config.

### Defaults

- `MARCH=x86-64-v2` (portable baseline: SSE4.2/POPCNT, ~2010+ CPUs), `MTUNE=znver3`
  (a modern core; `generic` if a family-neutral tune is preferred — one env var).
- `native` stays available (`MARCH=native MTUNE=native`) for "runs only on my box".

### Configs

- `release`: `-O2 -flto` (the current default). PGO remains a *mode* of
  release — its final artifact is a release binary, so no separate tree; the
  `gen` phase's instrumented build is throwaway and profile data stays in `tmp/pgo`.
- `debug`: `-O0 -g3` (new).
- `asan`: `-g3 -fsanitize=address,leak,undefined` (unchanged flags, new location).
- `tsan`: `-g3 -fsanitize=thread,undefined` (unchanged flags, new location).

### The load-bearing constraint

`-march` decides which instructions are emitted; running the binary on an
older CPU is a SIGILL. That contract must hold for the **whole** binary, so
the **middleware must be built at the same (or higher-compatibility) baseline
as the game**. Phase 0 therefore wires `MARCH` + `MTUNE` into `bootstrap.sh`,
not just the Makefile, so game + middleware share one baseline. (This is the
one real correctness hazard of the cross-arch story.)

Side benefit: each `(march, mtune)` gets its own tree, so the old
"must `make clean` after changing `MARCH`" hazard goes away — a different ISA
is a different dir and can't relink stale LTO bytecode into the wrong binary.

## Target state

`rm -rf build` = a clean tree; one gitignore entry (`build/`); the vendored
`middleware/` submodules stay pristine (no untracked `build/` in their working
trees). A fresh clone is: `./bootstrap.sh && make`, then `build/linux-v2-znver3/release/osp`.

## Phases

### Phase 0 — Linux: adopt the layout + march/mtune knobs (this box, testable now)

Each step is a commit-sized unit with its own acceptance gate. Order matters:
0.1 is the foundation (the path tokens), 0.2/0.3 reshape the tree, 0.4 is tooling.

**0.1 — Add the `MTUNE` knob + new defaults (Makefile + bootstrap.sh).**
Introduce `MTUNE ?= znver3`, change the default `MARCH` to `x86-64-v2`, keep
`native` opt-in. Build still emits to the root (no layout change yet) so this
is isolated and independently verifiable.
  - Acceptance: `make` + `make asan` + `make tsan` build and run; `make test`
    + `make e2e` green; `objdump -d osp | grep` shows v2-class instructions
    (no AVX2/FMA in the default build, i.e. the v2 baseline is actually in
    effect), and that the middleware `.a`s were reconfigured to the same baseline.

**0.2 — Game-side layout: `BINDIR`/`OBJDIR` → `build/<os>-<march>-<mtune>/<config>/`.**
Generalize the default target and the `asan`/`tsan` recursive makes (they
already pass `TARGET=`/`OBJDIR=`) to compute the dir from the four tokens;
add a `debug` target (`-O0 -g3`). `obj_test` + the ~34 test binaries move under
the matching config dir.
  - Acceptance: `make` / `make debug` / `make asan` / `make tsan` each land in
    their own `build/linux-v2-znver3/<config>/` tree; `make test` runs from the
    new locations; no two configs share a dir.

**0.3 — Middleware layout: → `build/<os>-<march>-<mtune>/middleware/<name>`.**
Point every `cmake -B`/`--build` at the shared `build/.../middleware/...`;
repoint the two `SDL3_DIR` cross-refs (sdl3-image, sdl-mixer); handle GLEW's
odd layout (source stays in `middleware/glew/`, only the output moves); update
the Makefile's `ASSIMP_A`/`SDL3_A`/`SDLIMG_A`/`SDLMIXER_A`/`GLEW_A`/`PNG_A`/
`ZLIB_A`/`BULLET3_OBJS` paths. The `bullet -> src` symlink and the
`git checkout -- zconf.h` (keeps the zlib submodule clean) stay put — they are
source-tree, not build-tree.
  - Acceptance: `rm -rf build && ./bootstrap.sh && make` reproduces the tree
    from scratch; `make test` + `make e2e` green; `git -C middleware/<sub> status`
    is clean (no untracked `build/`) for every submodule.

**0.4 — Tooling + docs + gitignore.**
`e2e/run.py` (game path + error msg + docstring) → `build/linux-v2-znver3/release/osp`;
`README.md` + `bootstrap.sh` final echo → `./build/linux-v2-znver3/release/osp`;
`.gitignore` collapses the ~8 build entries to `build/` (keep `middleware/glew/`);
`clean`/`remove` handle the new tree.
  - Acceptance: `make e2e` green end-to-end; a fresh `git status` on a built
    tree shows only `build/` (and `tmp/`) as the untracked build footprint.

**0.5 — Cleanup (after 0.4 green, confirm with the user first).**
Remove the now-stale root artifacts (`obj/`, `obj_test/`, `obj_asan/`,
`obj_tsan/`, `osp`, `osp_asan`, `osp_tsan`, `test_*`, `test_gl_vao`) and the old
`middleware/*/build*` dirs. The user is nosy about deletions, so this is a
separate, confirmed step — not folded into 0.1–0.4.
  - Acceptance: repo root has no build artifacts; `build/` is the only tree.

### Phase 1 — First real second platform (stub, needs its own budget)

Not part of Phase 0. Pick one (recommend Windows first). Needs, at minimum:
a toolchain (MSVC or MinGW-g++), per-OS SDL3 driver flags (no X11; WASAPI/CoreAudio
audio; GL via GLEW or ANGLE/D3D), a per-OS link-lib closure, a non-`xvfb` e2e
path, and `bootstrap.sh` per-OS. Lands as `build/windows-v2-znver3/...`. Large
enough to have its own plan; Phase 0's layout is what lets it slot in cleanly.

### Phase 2 — CI matrix (stub)

Once ≥2 OSes build: a CI matrix over (os × config ∈ {release, asan}) running
`make test` + `make e2e`, plus tag → artifacts. Extends the older
`release-infra2026_09_03` plan (which targeted a Windows CI build) — this
report supplies the build-tree layout that plan's Windows phase assumes.

## Risks / gotchas

- **Middleware baseline consistency (SIGILL)** — the big one. `MARCH`/`MTUNE`
  must reach `bootstrap.sh`, and the middleware baseline must be ≤ the game's.
  Enforce in 0.1 + verify in 0.3.
- **CMake build dirs are path-locked** (`CMakeCache.txt` holds absolute paths):
  relocating means a one-time `bootstrap.sh` re-run (full middleware rebuild, a
  few minutes). After that it's incremental as usual.
- **LTO bytecode is compiler-version-locked** (pre-existing): a compiler
  upgrade still needs a `make clean`-equivalent (fresh dir). Not worsened here.
- **GLEW's odd layout** (tarball source; CMake project in a `build/cmake`
  source subdir; output in `build-cmake`) — special-case in 0.3.
- **`znver3` default = AMD-tuned.** Fine as a portable default; swap to
  `generic` or an Intel core (`MTUNE=…`) if the audience skews Intel.
- **asan/tsan link uninstrumented middleware** (pre-existing, accepted): TSan
  coverage is partial by design (races entirely inside middleware are invisible).
  Preserved, not changed, by this work.
- **`native` is host-relative** — fine, because `build/` is local + gitignored;
  but it means the `native` tree is not shareable/reproducible across boxes.
- **Don't silently delete** stale dirs — 0.5 is a confirmed step.

## Out of scope

- Actual cross-platform porting (Phase 1+) — no Windows/macOS build this phase.
- Any gameplay / `src/` source changes.
- Changing the asan/tsan coverage model (still "uninstrumented middleware").

## How to reproduce / verify (Phase 0, end state)

    rm -rf build
    ./bootstrap.sh                 # one-time: builds middleware into build/linux-v2-znver3/middleware/
    make                           # → build/linux-v2-znver3/release/osp
    make test && make e2e          # gates
    make debug && make asan && make tsan   # each in its own config dir
    build/linux-v2-znver3/release/osp --help
