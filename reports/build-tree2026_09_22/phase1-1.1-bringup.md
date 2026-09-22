# Phase 1.1 — toolchain bring-up — record

Date: 2026-09-22
Status: done (acceptance met)

## What was installed

- `g++-mingw-w64-x86-64` → `x86_64-w64-mingw32-g++ (GCC) 13-win32`
- `wine64` → `wine-10.0 (Ubuntu 10.0~repack-12ubuntu1)`

Both via apt (candidates pre-verified). Host g++ stays 15; the two
compilers live in separate build trees, so the LTO compiler-version lock
never crosses between them.

## Acceptance test (per phase1-windows.md 1.1)

Two-TU C++20 program (`tmp/hello.cpp` + `tmp/hello2.cpp`, cross-TU call)
compiled with the exact shipping flag set, then run under Wine:

    x86_64-w64-mingw32-g++ -std=c++20 -O2 -flto \
        -march=x86-64-v2 -mtune=znver3 -static -Wall \
        -o tmp/hello.exe tmp/hello.cpp tmp/hello2.cpp

- `file`: PE32+ executable for MS Windows, x86-64 ✓
- LTO across TUs linked ✓ (the `factor` call is cross-TU)
- `WINEPREFIX=$PWD/tmp/wine wine64 tmp/hello.exe` →
  `hello from mingw cross; acc=522956312 sum=64`, exit 0 ✓
  (the `nodrv_CreateWindow`/explorer noise is first-run WINEPREFIX
  init without a display; the console app itself ran clean)
- ISA contract: `objdump -d` shows **0** AVX/FMA instructions ✓
  (v2 baseline is a ceiling, not a floor — the absence of v2-class
  instructions in a 20-line scalar program is expected)

Artifacts kept in `tmp/`: `hello.cpp`, `hello2.cpp`, `hello.exe`,
`hello.dis`, the `wine/` prefix. Install log: `tmp/p1-install.log`.

## Consequence

The riskiest unknown in the phase 1 plan (does the exact flag set —
LTO + v2 baseline + znver3 tune + fully-static — work on the cross
toolchain at all) is closed. 1.2 (Windows middleware) can proceed with
the same flag assumptions.
