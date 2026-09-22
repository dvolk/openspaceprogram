# Phase 1 — Windows build — results

Date: 2026-09-22
Status: complete (1.1–1.5 landed; 1.6 docs same day)
Parent: build-tree.md; plan: phase1-windows.md

## TL;DR

A self-contained `osp.exe` now builds from this Linux box with
`make OS=windows` and runs under Wine — including the full e2e
launch battery (smoke + 3 vab cases, 4/4). No source porting was
needed (as predicted): the one source-level fix was a hardening of
the GL error drain (it would have spun on any wedged driver), not a
port.

| Step | Result | Commit |
|---|---|---|
| 1.1 toolchain bring-up | mingw 13.2 + wine 10.0 in apt; `--help` runs under wine | ad9e960 |
| 1.2 bootstrap cross-builds | SDL3/SDL_image/SDL_mixer/GLEW/assimp build for windows, shared across configs | 274d6d9 |
| 1.3 game links | `make OS=windows` produces a runnable `osp.exe` | 05cddeb |
| 1.4 e2e under wine | smoke + 3 vab cases PASS; no orphaned processes | 436a4c1 |
| 1.5 configs | debug builds + runs under wine; asan/tsan refuse with clear messages | 8af9704 |
| 1.6 docs | README Windows section + this note | (this commit) |

## Findings worth keeping

**Wine's pre-context glGetError is a trap.** With no context current,
Wine's stack returns a fresh `GL_INVALID_OPERATION` on *every*
`glGetError()` call. The old `_check_gl_error` drained that in an
unbounded loop: 100% CPU, multi-GB log, and the e2e timeout then
killed only the wine wrapper, orphaning the PE at 95% CPU. Fixed
1.4: the drain is capped at 16 (a healthy core app never queues
more) and the pre-context checks in display.cpp are gone. Any future
"the wine build hangs at startup, writing a huge log" is this class
of bug first.

**`__declspec(dllimport)` + static libs on mingw (GLEW_STATIC).**
GLEW's `GLEWAPI` is `__declspec(dllimport)` unless `GLEW_STATIC` is
defined. We link the GLEW *static* archive, so the compiler must not
emit `__imp_` import-thunk references — the non-LTO debug config
died at link with `undefined reference to '__imp___glewX'` until
`-DGLEW_STATIC` went into CXXFLAGS. LTO happened to resolve the same
symbols to plain data refs, which is how release built before the
fix — a coincidence, not a guarantee. SDL3's `SDL_DECLSPEC` is empty
for static consumers, so it never bit us there.

**ASan does not exist for this mingw target.** The Ubuntu
`g++-mingw-w64-x86-64` package compiles `-fsanitize=address` but has
no ASan runtime for the Windows target (no `libsanitizer.spec`, no
libasan under `/usr/lib/gcc/x86_64-w64-mingw32/`), so link fails
after a full compile. The `asan` target probes with a one-line link
in ~1 s and refuses with the probe's error; on a toolchain that
ships the runtime the probe passes and the build proceeds. This is
the "full asan validation deferred to a real Windows box" from the
plan's Risks, now enforced instead of merely documented.

**Two concurrent software-GL games are flaky.** Parallel wine +
Xvfb + llvmpipe instances died mid-boot intermittently (exit 1);
serial is stable. e2e/run.py therefore auto-serializes `.exe` games
(explicit `--jobs` still wins).

## Deviations from plan

- Plan 1.5 gate was "all three windows configs land under
  `build/windows-v2-znver3/<config>/`". The asan config cannot land
  here — the toolchain has no runtime for it (above). That is an
  honest toolchain limit, not a skipped step: the gate now fails
  fast and says why, and the plan already deferred asan *validation*
  to a real Windows box (the user's weekly Win11 VM is the
  candidate for that).
- Everything else as planned. D2 (mingw cross), D3 (tsan
  Linux-only), D5 (self-contained exe) all held.

## Verification (2026-09-22, this box)

- `make OS=windows` from clean: release + debug both link; `nm` on
  the release exe shows zero `__imp___glew*` undefined refs.
- `make OS=windows asan`: fails in ~2 s with the toolchain message.
- `make OS=windows tsan`: refuses with the linux-only message.
- Wine: debug exe boots to the main loop and exits cleanly on
  `--timeout` (17 s startup at -O0 — slow, not hung); release exe
  passes the e2e battery smoke + vab-launch + vab-launch-orbit +
  vab-launch-body (4/4), serial.
- Linux: build + `make test` green with the new CXXFLAGS
  (`-DGLEW_STATIC` is a no-op on Linux); `./osp` symlink untouched
  by windows builds/removes.
- No stale wine/Xvfb processes after any run.

## Open

- ASan/UBSan validation on a real Windows box (user's Win11 VM).
- Phase 2 (macOS) remains as scoped in the parent report — separate
  effort (different middleware set, no cross path from Linux).
