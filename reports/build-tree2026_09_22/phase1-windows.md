# Phase 1 — Windows build — plan

Date: 2026-09-22
Status: planning (no code written)
Parent: build-tree.md (Phase 0 is complete on this box)

## TL;DR

Build a Windows `osp.exe` for the game **cross-compiled from this Linux
box** with mingw-w64 g++, reusing the existing GNU-make build system
unchanged in principle. The layout already reserved for it
(`build/windows-v2-znver3/...`) slots in directly. Verification on this
box is via Wine (unit tests stay native; e2e runs the .exe under
`xvfb-run` + Wine).

The code is already portable — see Findings. Phase 1 is a
**build-system-only** effort.

## Findings (verified 2026-09-22)

**The source is platform-clean.** `grep` over `src/` finds:
- zero `_WIN32` / `__APPLE__` / `__linux__` / `_MSC_VER` conditionals;
- no POSIX headers (`unistd.h`, `dirent.h`, `sys/*`, `pthread.h`);
- no POSIX syscalls (the two `mkdir(2)` mentions are comments);
- file access goes through `std::filesystem` (save/datadir/gameui/
  shipdef/main) — portable by definition;
- all OS contact is through SDL3 / SDL_image / SDL_mixer APIs.

So there is **no porting work in the game itself**. The entire platform
surface is:

| Touchpoint | Location | Linux-specific bit |
|---|---|---|
| Link closure | Makefile `SDL3_SYS` | `-lX11 … -lpulse -lasound -ldl -lm -lpthread` |
| OS token | Makefile `OSTOK=linux` | hardcoded |
| Symlink | Makefile `all` → `ln -sfn … osp` | POSIX convenience |
| C++ driver | Makefile `CXX` | g++ (mingw-g++ is a drop-in for flags) |
| SDL3 drivers | bootstrap.sh | X11 on, ALSA/Pulse on, Wayland off |
| e2e wrapper | e2e/run.py | `xvfb-run -a <game>` |
| tsan-run | Makefile | `DISPLAY`/`xvfb-run` (tsan itself: see Configs) |

**Toolchain availability.** Not installed here, both in apt:
`g++-mingw-w64-x86-64` (g++ 13.2) and `wine64` (10.0). Note the mingw
compiler is a *different version* from the host g++ 15 — irrelevant,
because the trees are separate (the LTO compiler-version lock only
applies within one tree).

## Decisions

**D1 — Platform: Windows.** Per the parent report's recommendation.
macOS is a later, separate phase (its middleware set and link closure
differ more: CoreAudio, Metal/ANGLE vs GLEW, and no cross path from
Linux without Xcode bits).

**D2 — Toolchain: mingw-w64 cross-compile from this box (recommended).**
- Reuses the *entire* existing build system: GNU make, the g++ driver
  (same `-O2 -flto -march -mtune -Wl,--gc-sections` flag set), the cmake
  bootstrap pattern. `OS=windows` swaps the compiler triple, the SDL3
  driver flags, and the link closure.
- The `<march>`/`<mtune>` tokens stay *honest* on Windows too: mingw
  g++ speaks the same `-march=x86-64-v2 -mtune=znver3`, so
  `build/windows-v2-znver3/` means the same thing as the Linux tree.
  (MSVC has no march/mtune — its `/arch:` + `/O` model would break the
  token scheme and the "same binary contract" story.)
- Verifiable **on this box** (Wine), and Phase 2's CI matrix can run it
  on a Linux runner (no Windows runner minutes).
- Cost: an extra cross-toolchain install; the exe carries MinGW
  runtime characteristics (mitigated by fully-static linking, below).

  Alternative (rejected for now): native MSVC build. Needs a second build
  system (CMake for the game), a Windows box in the loop, and breaks the
  flag/token parity. Revisit only if MinGW output proves slow or broken
  on real hardware.

**D3 — Configs on Windows.** `release`, `debug`, `asan` (minus `leak` —
no LSan on Windows). **`tsan` is Linux-only**: MinGW has no ThreadSanitizer
(the parent report's "uninstrumented middleware" coverage note applies
even more). The `tsan` target errors out with a clear message when
`OS=windows`.

**D4 — Verification path.**
- `make test` — stays native Linux (the tests compile `src/` with host
  g++; nothing to cross there, and the test suite has no Windows-only
  code paths).
- `osp.exe` smoke — `wine osp.exe --help` (no window needed), then a
  `--timeout` run under `xvfb-run` + Wine (Wine renders to X, so the
  existing Xvfb story keeps working).
- e2e — `e2e/run.py` gains a `--wine` mode: same cases, same in-process
  input injection (`--sim-press`/`--sim-mouse`), game command wrapped as
  `wine <exe>`. The Makefile `e2e` target passes it when `OS=windows`.
- Pin `WINEPREFIX` at `tmp/wine` (one-time prefix init cost; keeps the
  repo clean).

**D5 — Self-contained exe.** Link fully static (MinGW's libgcc/libstdc++
into the exe) so the artifact is a single portable `osp.exe` — no
runtime DLL sidecar to ship or mis-version.

## Phases

Each step is a commit-sized unit with its own acceptance gate.

**1.1 — Toolchain bring-up.** Install `g++-mingw-w64-x86-64` + `wine64`
(apt). Cross-compile a hello-world; run it under Wine.
  - Acceptance: `wine hello.exe` prints; `x86_64-w64-mingw32-g++
    -flto -march=x86-64-v2 -mtune=znver3` compiles + links an LTO exe
    that Wine runs (de-risks the exact flag set we ship).

**1.2 — Windows middleware (bootstrap.sh).** One `OS`-aware pass over the
six cmake builds: `CMAKE_SYSTEM_NAME=Windows` + the mingw C/CXX
compilers, same `-march/-mtune/-flto/sections` flags, output into
`build/windows-v2-znver3/middleware/<name>`. SDL3 driver set for
Windows: OpenGL (wgl) on, GLES off (no ANGLE), WASAPI audio (the
Windows default) + dummy driver kept, no X11/ALSA/Pulse. SDL_image /
sdl-mixer SDL3_DIR cross-refs follow the new dir. GLEW/bullet3/assimp:
no driver flags, straight cross.
  - Acceptance: all six archives under `build/windows-.../middleware/`;
    `file` shows PE objects; every submodule `git status` clean; a Linux
    `rm -rf build && ./bootstrap.sh` still works (the OS pass is
    additive, not a fork of the script).

**1.3 — Game cross-build (Makefile).** `OS ?= linux` (drives `OSTOK`);
`OS=windows` selects `CXX=x86_64-w64-mingw32-g++`, `TARGET=osp.exe`,
the Windows link closure (SDL3-static closure: `opengl32 winmm version
user32 gdi32 advapi32 shell32 ole32` + the usual `-lGL` for GLEW),
fully-static runtime (D5). The `all` symlink step is Linux-only
(a `windows` build's entry point *is* `build/windows-.../release/osp.exe`;
the Linux `./osp` symlink must not be clobbered by a windows build).
  - Acceptance: `make OS=windows` links; `file` → PE32+ executable;
    `wine build/windows-v2-znver3/release/osp.exe --help` prints usage;
    the Linux tree and its `./osp` symlink are untouched.

**1.4 — e2e under Wine.** `run.py --wine` (game command wrapped in
`wine`, Xvfb unchanged, `WINEPREFIX=tmp/wine`); Makefile `e2e` target
wires it when `OS=windows`.
  - Acceptance: `smoke` + one `vab` case pass under Wine; on GL failure
    (see Risks) the gate degrades honestly to `--help` + a documented
    "needs real Windows hardware" note — never a faked pass.

**1.5 — Configs + guards.** `make OS=windows debug` / `asan` build and
run under Wine; `tsan` target refuses with a clear Linux-only message;
`make clean`/`remove` handle the windows tree.
  - Acceptance: all three windows configs land under
    `build/windows-v2-znver3/<config>/`; `make tsan` with `OS=windows`
    fails with the explanatory message; `make test` (native) still green.

**1.6 — Docs.** README: Windows section (what works, how to run the
exe, what's Linux-only); bootstrap echo unchanged (Linux entry point);
this report's Phase 1 stub cross-referenced from build-tree.md's stub
(append-only note in the new file, not an edit to the old one).
  - Acceptance: a reader can go from fresh clone to a running `osp.exe`
    following the README alone.

## Risks / unknowns

- **OpenGL under Wine (biggest).** The game needs a GL 4.5 context via
  GLEW; Wine passes GL through to the X server (Mesa/llvmpipe under
  Xvfb). If llvmpipe's 4.5 is incomplete, e2e cases that draw will fail
  even though the build is fine. Mitigation: D4's honest-degradation
  gate; ultimate fallback is a real Windows box for e2e only.
- **Audio under Wine.** WASAPI is only partially implemented; SDL3's
  dummy driver is the fallback. Acceptance: smoke cases must not depend
  on audible output (they don't — they assert on game state).
- **LTO on mingw 13.** Supported, but the least-proven flag in the set;
  1.1 de-risks it before any real build. Fallback: drop `-flto` for the
  windows tree only (CFGFLAGS is already per-config; add a per-OS carve-out).
- **ASan under Wine** can be flaky; 1.5's gate is "builds + `--help`
  runs", full asan validation deferred to a real box.
- **Wine prefix first-run cost** (minutes) — one-time, pinned under `tmp/`.
- **Static-CRT edge cases** (thread-local init order etc.) — unknown
  until 1.3; a fully-static exe is the *goal*, but if it misbehaves the
  fallback is shipping the 2 MinGW DLLs next to the exe.

## Out of scope

- macOS (its own phase, different closure).
- Any `src/` gameplay changes (none expected — Findings says the source
  is portable).
- Native MSVC build path (revisit only on evidence).
- Windows e2e in CI (that's Phase 2's matrix decision).

## Open questions (for the user, before 1.1)

1. Approve the mingw-cross toolchain (D2)? It's the load-bearing choice.
2. `tsan` Linux-only (D3)?
3. Is there a real Windows machine reachable later for the "ultimate"
   validation (GL under Wine is the risk we'd most want to escape)?
