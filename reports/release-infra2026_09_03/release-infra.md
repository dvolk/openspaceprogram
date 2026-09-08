# Release infrastructure — plan

Date: 2026-09-03
Status: planning (no code written)

## TL;DR

Set up GitHub Actions so that tagging `v0.1.0` on master publishes a
**self-contained Linux binary** and (phase 2) a **Windows build** to
GitHub Releases. Linux is the priority; the game links SDL2/GLEW/assimp
shared libs today, so the first piece of work is an opt-in **static**
build. Everything needed is already in place: public repo, `make test` +
`make e2e` batteries to gate CI, tag-based versioning (`make version`),
and a small `res/` (1.4 MB) to ship alongside the binary. Steam is not in
scope — only the one constraint it imposes (no system-lib assumptions),
which the static build already satisfies.

## Current state (verified 2026-09-03)

- Repo: `github.com/dvolk/openspaceprogram`, **public**, no tags yet, no
  `.github/` (no CI).
- Build: Makefile + g++ (C++11). Deps: SDL2, SDL2_image, GLEW, GL,
  assimp (system shared libs), plus source-built bullet3 (cmake, double
  precision), imgui, implot (pinned git submodules), glm (header-only).
- `ldd osp`: SDL2, SDL2_image, GLEW, assimp + X11/wayland/pulse closure
  via SDL2.
- Resources: loaded relative to CWD (`./res/...`); `res/` = 1.4 MB.
  A release = binary + `res/` + license + run notes.
- Versioning: `make version` writes `src/version.h` from
  `git describe --tags --always --dirty --match "[0-9A-Z]*.[0-9A-Z]*"`
  — tag `v0.1.0` already matches the pattern.
- Test gates: `make test` (~25 unit test binaries, several link the real
  Bullet/physics sources), `make e2e` (30+ cases under Xvfb, `e2e/run.py`).
- Portability probes (for the Windows cross): no `SDL_Audio*`/`SDL_Joystick*`
  use anywhere in `src/`; no POSIX-only code beyond one `#include <sys/stat.h>`
  (main.cpp); jobs use `std::thread`/`std::mutex`; assimp usage is the
  stable 5.x API (`Assimp::Importer`, `aiProcess_CalcTangentSpace |
  aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
  aiProcess_SortByPType` in mesh.cpp).
- Submodules (all public, CI-fetchable): glm, bullet3, imgui, implot.

## Target state

Tag `v0.1.0` on master → GitHub Actions builds and publishes:

- `osp-0.1.0-linux-x64.tar.xz` — self-contained `osp` (runtime deps are
  glibc only), `res/`, license, run notes.
- `osp-0.1.0-win64.zip` — `osp.exe`, `res/`, bundled DLLs, license.
- `SHA256SUMS` alongside both.

Plus CI on every push/PR: build + `make test` + `make e2e JOBS=2`.

## Phase 1 — Linux static build + CI + releases

### 1a. Opt-in static build in the Makefile (local dev flow unchanged)

- New `static` target (or `STATIC=1` override):
  `libSDL2.a`, `libSDL2_image.a`, `libassimp.a`, `libGLEW.a` +
  `-static-libstdc++ -static-libgcc`.
- SDL2 and SDL2_image **from source** (pinned release tags, configure +
  make, ~1 min) with `--disable-audio --disable-video-wayland` — safe
  because the game never touches SDL audio/joystick (verified above), and
  it drops pulse/alsa/wayland from the closure. Remaining closure
  (X11, Xext, Xcursor, Xi, Xfixes, Xrandr, Xss, drm) all have `.a` in
  distro dev packages.
- Fallback if the distro `.a` closure is fiddly: build assimp and GLEW
  from source into one `staging/` prefix too. More build time, zero
  distro guessing.
- Acceptance: `ldd osp-static` shows only `libc.so.6` / `libm.so.6`;
  `make test` and `make e2e` pass on the static binary; it runs from a
  clean directory on a machine without the dev packages.

### 1b. CI workflow — `.github/workflows/ci.yml`

- Trigger: push + pull_request to master.
- `ubuntu-24.04`; `actions/checkout` with `submodules: true`,
  `fetch-depth: 0` (so `git describe` → `src/version.h` works).
- apt: g++, cmake, python3, xvfb, libgl1-mesa-dri, plus the dev libs and
  the X11 closure headers from 1a.
- Steps: build bullet3 (the README steps) → `make` → `make test` →
  `make e2e JOBS=2`.
- Value: free regression gate on every push, and the same job shape the
  release reuses.
- Optional: `actions/cache` for `middleware/bullet3/build` (saves
  ~1–2 min).

### 1c. Release workflow — `.github/workflows/release.yml`

- Trigger: `tags: ['v*']`.
- `permissions: contents: write`.
- Steps: static build (1a) → stage `dist/osp-<ver>-linux-x64/`
  (`osp`, `res/`, GPL-3 + CC-BY-SA 3.0 licenses, `README.txt` with run
  instructions) → `tar.xz` → `SHA256SUMS` →
  `softprops/action-gh-release@v2`.
- Trial: tag `v0.0.1` from a branch first, verify the artifact on a clean
  machine, then delete the trial release.

## Phase 2 — Windows cross (same Linux runner)

- apt adds: `g++-mingw-w64-x86-64`,
  `mingw-w64-x86-64-{sdl2,sdl2-image,glew,assimp}-dev`.
- New `make windows` target (or a cross-recipe block):
  `CXX=x86_64-w64-mingw32-g++`, mingw include/lib paths,
  `-lopengl32 -lgdi32 -lshell32 -lversion -lwinpthread -lmingw32`,
  objects in `obj_win/` (bullet3/imgui/implot are portable C++ and build
  fine under mingw — rebuild their objects with the cross compiler).
- Link static `.a` where the mingw packages ship them (SDL2, SDL2_image,
  GLEW typically do); bundle the rest (assimp.dll, libgcc/winpthread if
  not static) next to `osp.exe` in the zip.
- `make test` / `make e2e` stay Linux-only. Optional smoke check:
  `wine64 osp.exe --help`.
- Verify first: mingw-w64 package versions on the 24.04 runner
  (assimp API compat — our usage is 5.x-stable), and GLEW's mingw static
  lib linking cleanly against system `opengl32`.

## Phase 3 — Steam (constraints only, no work now)

- Steam requires **no system-lib assumptions** and a self-contained
  depot. The static Linux + bundled-DLL Windows artifacts are already
  exactly that shape — no redesign needed.
- Future work (when it comes): link `steam_api.so` / `steam_api.dll`
  (init before SDL), `steam_appid.txt`, app icon + Windows manifest
  (DPI awareness), store metadata. CI artifact layout maps 1:1 to a
  depot build artifact.

## Order of operations

1. 1a — static Makefile target (the only piece that touches code);
   validate locally: `ldd`, `make test`, `make e2e`, clean-dir run.
2. 1b + 1c — workflows; trial tag; verify artifact on a clean machine.
3. Phase 2 — separate branch; mingw packages; `make windows`; verify on
   a real Windows box (or wine).

## Known unknowns (verify when implementing)

- Exact static `.a` closure for SDL2 on 24.04 (audio-off should make it
  small — but check which objects the distro `.a` pulls in).
- mingw-w64 `assimp`/`glew` package versions and static-lib availability
  on the 24.04 runner.
- e2e timing under llvmpipe on the runner (local env is Mesa 26; one
  case could be timing-sensitive — `JOBS=2` default keeps it mild).
- `SDL2_image` static archive on the runner (Debian/Ubuntu ship
  `libSDL2_image.a`; if not, build from source alongside SDL2 — same
  recipe).

## Parking lot (later polish)

- Resolve `res/` relative to the executable instead of CWD (small code
  change; makes the extracted dir runnable from anywhere).
- Windows: app icon embedded in `osp.exe`, DPI-awareness manifest.
- Consider a CMake wrapper only if the two Makefile code paths (native +
  cross) start to diverge — not needed for this plan.
