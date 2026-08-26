# E2E tests — scope and plan

Date: 2026-08-25
Status: investigation (no code written)

## TL;DR

**The game already has ~80% of an e2e harness built into it**: synthetic
keyboard/mouse input (`--sim-press`, `--sim-mouse`), timed auto-exit
(`--timeout`), 9 start scenarios, machine-readable telemetry
(`--orbit-log`, `--dbg-log`), built-in self-tests (`--selftest-stage`,
`--selftest-rails`), and a working screenshot path (F12 →
`./tmp/osp_*.png`). Headless execution under Xvfb + llvmpipe was
**verified working** during this investigation:

```
$ xvfb-run -a ./osp --timeout 6 --time-accel 1 --sim-press 1500,100,SPACE
...
Stage: nothing left to separate          <- synthetic SPACE fired (racer is 1 stage: correct)
Timeout reached (6.0 s); exiting main loop.
$ echo $?
0
```

What's missing, in priority order:

1. **A pass/fail contract.** The game always exits 0 (crashes aside);
   `--selftest-*` prints "OK"/"DRIFT!" but still exits 0. There is no
   assertion mechanism, so nothing can *fail*.
2. **A runner.** No script, no Makefile target, no case definitions. E2E
   today is ad-hoc `xvfb-run ./osp ...` by hand (the pattern documented in
   QWEN.md).
3. **Determinism of timing.** `--sim-press`/`--sim-mouse`/`--timeout` are
   **wall-clock** based, and the main loop has **no frame limiter** (the
   `FPS = 60` constant at `main.cpp:66` is declared but never used — there is
   no `SDL_Delay` anywhere). Under Xvfb there is no vsync, so the loop runs
   at full CPU speed (measured: a 4.3 s wall-clock run consumed **16.7 s of
   user CPU**). The tick at which a "1500 ms" keypress lands therefore
   varies with machine speed and load — timing-sensitive tests will be
   flaky.
4. **Visual checks** (optional, later): screenshots are F12-only with
   timestamped names; no CLI screenshot, no image comparison.

**Recommendation:** a three-layer build, each layer independently useful:

- **L1 — pass/fail battery (do first, zero or near-zero C++):** a Python
  runner + data-driven case files that launch `xvfb-run ./osp …`, check the
  exit code, and assert EXPECT/FORBID substrings against stdout (the game's
  existing log lines — `Stage: dropped …`, `[orbitlog] …`, `GL_…` — are
  already well-suited anchors). `make e2e` runs the battery.
- **L2 — determinism (small C++):** a tick counter in the loop plus
  tick-based input scheduling (`--press-ticks`, `--mouse-ticks`) and a
  fixed-length run (`--ticks N`), so tests are specified entirely in
  sim-time and are repeatable. Optional `--fps N` limiter, which also fixes
  the CPU spin under llvmpipe.
- **L3 — visual (optional):** `--screenshot-tick N PATH` reusing the
  existing `display.SaveScreenshot`, plus a coarse "not black / not
  constant" image check (PIL + numpy are installed). Goldens only if
  wanted later.

Estimated total: ~2–3 focused sessions, low risk, no architectural change.
Unit tests (`make test`) already cover the pure-math core; the e2e suite
covers the integrated loop only: **input → command → physics → staging →
HUD → GL**.

---

## 1. What exists today (verified, 2026-08-25)

### 1.1 CLI surface that already serves e2e

All in `src/main.cpp` (CLI11, parsed in `main`):

| Flag | What it does for e2e | Notes |
|---|---|---|
| `--sim-press START_MS,DURATION_MS,KEY` (repeatable) | Synthetic keypress: down/up relative to **wall clock** after loop start. One-shot actions fire from the pushed `SDL_KEYDOWN`; held commands (WASDQE, I, X, B, N, R, F) are OR'd into the `isDown()` check because `SDL_PushEvent` does not update `SDL_GetKeyboardState` (verified in-code comment, `main.cpp:2556`) | key names: `SPACE`, `TAB`, `A..Z`, `F1-F12`, or decimal keycodes |
| `--sim-mouse TIME_MS,DURATION_MS,X,Y,BTN` (repeatable) | Synthetic mouse: move / LMB-MMB-RMB click / RMB-drag (camera orbit, 200 px ≈ 1 rad) | X,Y in **1920×1080 window pixels** — the window size is hardcoded (`DISPLAY_WIDTH/HEIGHT`, `main.cpp:64`) |
| `--timeout S` | Auto-exit the main loop after S **wall-clock** seconds | prints `Timeout reached (S s); exiting main loop.` |
| `-t,--time-accel N` | Initial sim speed (0 = paused). `.`/`,` keys scale ×10/÷10; ≥10000 = rails warp | staging via SPACE requires `time_accel > 0` and orbit cam mode |
| `--scenario` | 9 starts: `pad`, `pad-polar`, `rot-orbit`, `inertial-orbit`, `high-orbit`, `high-polar`, `ellipse-peri`, `ellipse-apo`, `ellipse-mid` | the `ellipse-*` set is a 10×1000 km ASL orbit — cheap orbital setups |
| `--ship` (repeatable) / `--fleet` / `--body` / `--system` / `--parts` | Fleet/ship/system selection | ships: `res/ships/{racer,stager,transporter,tanker,laythe_explorer}.json`; `stager.json` is 2-stage |
| `--orbit-log` / `--orbit-interval S` | `[orbitlog] t=… frame="…" r=… v=… sma=… ecc=… peri=… apo=… inc=… T=… ttAp=… ttPe=… |h|=… E=…` (`main.cpp:4194`) | stable, parseable — the anchor for orbital assertions |
| `--dbg-log` | `[dbg] t=… pos=[…] alt=… vel=[…] |v|=…` (`main.cpp:4215`) | anchor for altitude/velocity assertions |
| `--spin-log`, `--radial-test` | Spin diagnostics / prebuilt spin-test ships (`stacks`, `parstacks`, …) | regression cases for the earlier spin bugs |
| `--selftest-stage` | Stages the first multi-stage ship, verifies last-stage refusal, runs 30 physics ticks, exits | prints `selftest-stage: 30 ticks after separation, no crash; OK` — **but exits 0 even if something is wrong** |
| `--selftest-rails` | Coasts a railed ship 30 ticks, checks conic conservation (prints `(conserved OK)` or `(DRIFT!)`), handoff continuity check, exits | same exit-code caveat |
| F12 | Screenshot → `./tmp/osp_<timestamp>.png` | `display.SaveScreenshot` = `glReadPixels` + `IMG_SavePNG` (`display.cpp:127`), already implemented and exercised |
| `--free-cam-pos/fwd/up` | Deterministic camera placement | useful for visual cases |

Other useful anchors already printed to stdout:

- `Stage: dropped %d part(s); now on stage %d of %d` / `Stage: nothing left to separate` (SPACE handler)
- `Rails warp refused: '…' is neither in free fall nor grounded` (warp entry)
- `@@@ %s switching frame from %s to …` (SOI crossings, `main.cpp:1615/1631`)
- `Active ship %d of %d: %s` (TAB ship-switch, `main.cpp:3572`; **not** printed at startup — `activeIdx` starts at 0 silently)
- `GL_%s - %s:%d` (from `check_gl_error`, `gldebug.cpp:5`) — a ready-made FORBID target
- `error: …` for CLI/data setup failures (which *do* exit 1 today)

### 1.2 Environment

- **Xvfb is available** (`/usr/bin/xvfb-run`, `/usr/bin/Xvfb`); no `DISPLAY`
  set in the default shell.
- The game **requires a real GL 4.3 core context** (`SDL_WINDOW_OPENGL`,
  `display.cpp:16`, GLEW 4.3 check), so `SDL_VIDEODRIVER=dummy` is not an
  option — Xvfb + llvmpipe (or a GPU) is the headless path. This works here.
- The Makefile already documents the pattern: `test-gl` runs under
  `DISPLAY=:99` (Xvfb) and notes Mesa-26-specific GL quirks.
- Python 3 with **PIL 12.1.1 + numpy** is installed (image-diff capable);
  the project already ships Python tooling (`check_mesh.py`,
  `gen_adapter.py`, `rescale_obj.py`), so a Python runner fits the project.
- Cost per run: startup ~0.5–1 s (shaders, 18 KSP-system bodies, terrain
  collision), then the loop. A 10-case battery is ~1–2 min.

### 1.3 Determinism facts (measured / read from code)

- **Single-threaded**: no `std::thread`/`pthread`/`SDL_CreateThread`
  anywhere in `src/`.
- **Fixed timestep**: `dt = 1.0/50.0` (`main.cpp:3471`); substep count
  `n = max(3, round(dt·accel / 0.1))` (`main.cpp:4136`) — a pure function of
  the time-accel.
- **Bullet double precision, serial** solver.
- ⇒ Given the *same tick sequence*, the sim state is deterministic on a
  given machine + driver.
- **The one real non-determinism**: everything time-related in the test
  path (input scheduling, `--timeout`) is wall-clock, and loop speed is
  unbounded (no frame limiter). Ticks-per-wall-second = rendering FPS, which
  under Xvfb/llvmpipe is "as fast as the CPU allows" (4.3 s wall = 16.7 s
  user CPU in the smoke run). A "1500 ms" keypress can therefore land after
  150 ticks on a 60 FPS machine or 15000+ ticks here. **The sim state at
  press time is machine-dependent.**

---

## 2. Gaps

| # | Gap | Impact |
|---|---|---|
| G1 | No pass/fail contract: always exit 0 (crashes aside); `--selftest-*` verdicts are stdout-only | Nothing can *fail*; a harness can only catch crashes |
| G2 | Wall-clock ↔ tick coupling + no frame limiter (G2) | Timing-sensitive cases are flaky across machines and under load |
| G3 | No runner / case definitions / Makefile target | E2E is manual; not a gate |
| G4 | Telemetry lines are mixed with noise (`added terrain collision` ×N, ImGui chatter) | Workable via substring assertions; a quieter channel would be nicer but not required |
| G5 | No CLI screenshot, no image comparison | Rendering regressions (black screen, NaN uniforms, broken shader) are invisible to stdout-based tests |
| G6 | Every run loads the full 18-body KSP system | ~1–2 s overhead per case; a cheap test system would trim it |
| G7 | 1920×1080 window hardcoded | `--sim-mouse` pixel coordinates are pinned to that geometry (fine — just note it) |

---

## 3. Recommended design (layered; each layer stands alone)

### L1 — pass/fail battery  *(the core; zero C++ required)*

**Runner**: `e2e/run.py` (Python, no new deps — `subprocess`, `re`, and
`PIL` only if L3 is added). For each case file in `e2e/cases/*.txt`:

```
# e2e/cases/staging-basic.txt
NAME staging-basic
ARGS --scenario pad --ship res/ships/stager.json --time-accel 1 --timeout 10
     --sim-press 2000,100,SPACE
     --sim-press 6000,100,SPACE
EXPECT Stage: dropped
EXPECT nothing left to separate
FORBID GL_
FORBID error:
```

**Case file format** (deliberately trivial):

- `NAME <label>` — one line, used in the summary.
- `ARGS …` — one or more lines, concatenated and passed to `./osp` (spaces
  in values are fine; the runner splits on whitespace like the shell).
- `EXPECT <substring>` — must occur ≥1 time in the captured output (repeat).
- `FORBID <substring>` — must occur 0 times (repeat).
- `#` comments.

**Runner contract**:

1. Build check: `./osp` must exist (or `make` first — the Makefile target
   handles that).
2. Launch `xvfb-run -a ./osp <ARGS>` from the repo root, capture
   stdout+stderr, wait (hard cap = `--timeout` if present, else a generous
   default, e.g. 120 s; kill on overrun = FAIL).
3. PASS iff: exit code 0 ∧ all EXPECT found ∧ no FORBID found.
4. On failure: print the case's full output (or the tail + the missing
   EXPECTs) so a failure is diagnosable without re-running.
5. Summary table at the end; process exit code = 1 if any case failed.

**Why the harness asserts rather than adding `--require/--forbid` to the
binary**: zero C++ change, assertions live next to the cases, and the binary
stays clean (QWEN.md: simple C++, look for opportunities to simplify). The
in-process variant can be added later if a standalone self-checking `./osp`
run is wanted; nothing in L1 blocks it.

**Makefile target**:

```make
e2e: $(TARGET)
	python3 e2e/run.py
```

**Initial battery** (assertions are substrings + range checks on parsed
`[orbitlog]`/`[dbg]` values — per QWEN.md, *not* exact-value tests; strings
marked `verify:` should be confirmed against a real run when the case is
written):

| Case | Setup | What it proves | Anchors |
|---|---|---|---|
| `smoke` | `--timeout 5` defaults | boots, loads system, renders, clean exit | EXPECT `Main loop starting`; FORBID `GL_`, `error:` |
| `staging-basic` | `pad` + `stager.json` + SPACE ×2 | input → command → stage separation → last-stage refusal | EXPECT `Stage: dropped`, `nothing left to separate` |
| `thrust-ascent` | `pad` + hold `I` 3 s @1×, `--dbg-log` | thrust command → physics response | parse `[dbg] … alt=` series: strictly increasing after burn starts |
| `orbit-insert` | `pad`, `R R` throttle, `B` prograde, hold `I`, `SPACE` stage, coast; `--orbit-log` | full flight loop incl. staging mid-flight | last `[orbitlog]` line: `ecc < 0.95`, `apo > <threshold>` (verify: pick threshold from an actual run with margin) |
| `rails-warp` | `rot-orbit`, `time-accel 1000`, press `.` ×2 → 10000, coast, `,` ×2 | warp entry/exit key path, no crash | FORBID `GL_`; EXPECT `Rails warp` line only if refused (verify: success path prints nothing — consider a one-line `printf` on successful warp entry as a testability aid) |
| `ship-switch` | `--fleet res/fleet.json` (verify: fleet has ≥2 ships), TAB | select_ship handoff path | EXPECT `Active ship 2 of 2` (verify: exact count from `res/fleet.json`) |
| `spin-regression` | `--radial-test stacks --timeout 20 --spin-log` | earlier spin/impulse bugs stay fixed | FORBID `nan`, `GL_` |
| `ui-click` | `--sim-mouse 1500,0,<x>,<y>,1` on a main-menu checkbox (e.g. "Orbit Info") | ImGui path under llvmpipe, no crash | FORBID `GL_`; EXPECT `Main loop starting` |
| `soi-crossing` (verify: reachable) | `ellipse-apo` + long prograde burn across SOI | frame-switch bookkeeping in the loop | EXPECT `@@@ ` (frame switch line) |

That's 9 cases; all but `soi-crossing` and `orbit-insert` are
mechanical. `orbit-insert` is the flagship (it exercises the whole loop in
one run) and is the one where margins matter — L2 removes its flakiness.

**G6 mitigation (optional, cheap):** a small `res/e2e_system.json`
(1 star + 1 planet + 1 moon, small radii, low `surface.amplitude`/`octaves`
— the `surface` JSON block already supports this) and point most cases at
it via `--system`. Only worth it if the battery gets slow; the KSP system
boots in ~1 s here.

### L2 — determinism  *(small C++, removes the flakiness)*

Root cause (G2): input times and the timeout are wall-clock; loop speed is
unbounded. Fix by expressing tests in **ticks**, where a tick is one main-loop
iteration (already the unit `--selftest-stage`/`--selftest-rails` count in):

1. **Tick counter**: one `int tick` incremented once per loop iteration
   (next to the existing `selftest_ticks` logic).
2. **`--ticks N`**: exit the loop after N ticks (deterministic duration;
   replaces `--timeout` in test ARGS).
3. **`--press-ticks TICK,DURATION_TICKS,KEY`** and
   **`--mouse-ticks TICK,DURATION_TICKS,X,Y,BTN`**: identical semantics to
   the ms variants but scheduled against the tick counter instead of
   `SDL_GetTicks() - loop_start_ms`. Reuses the existing
   `SimKeyPress`/`SimMouseAction` structs and emission code
   (`main.cpp:3727`/`3755`) — add an `in_ticks` bool and a second comparison
   source. ~40–60 lines in `main.cpp`.
4. **Optional `--fps N` frame limiter** (sleep against target frame time at
   the bottom of the loop): makes the *wall-clock* variants deterministic
   too, and fixes the CPU spin (16.7 s user per 4.3 s wall under llvmpipe —
   worth doing independently of e2e; it also makes interactive runs on
   slow-GPU machines behave).

After L2 a case like `orbit-insert` is fully specified in sim units:
same binary + same args + same `res/` + same machine/driver ⇒ same sim state
every run. Keep assertions **range-based** anyway (cross-machine libm/driver
differences) — that is also the QWEN.md "don't make tests too specific" rule.

### L3 — visual checks  *(optional, later)*

1. **`--screenshot-tick TICK PATH`** (pairs with L2; or
   `--screenshot-ms MS PATH` to use the existing wall-clock clock): call
   the already-implemented `display.SaveScreenshot(PATH)` when the tick is
   reached (`main.cpp:4864` shows the F12 path — ~10 lines to add a CLI
   trigger + explicit path).
2. **Coarse image check in the runner** (no goldens to start): with
   PIL/numpy, assert the image is not constant / not black (std-dev and
   min/max over RGB), and optionally that a chosen ROI is non-uniform.
   Catches: black screen, shader not compiling (fallback clear color), NaN
   uniforms, forgotten `glViewport`.
3. **Goldens** (`e2e/goldens/*.png` + mean-abs-diff with a tolerance) only
   if the heuristic proves too coarse. Brittle across Mesa versions — pin
   to the CI image's driver and re-bless deliberately. Given
   "very early development", the heuristic is the right first stop.

Value: this is the only layer that sees rendering-only regressions, which
stdout cannot.

---

## 4. What was considered and rejected

- **Splitting `main.cpp` to make it testable**: the 4910-line `main` mixes
  CLI, world-building, input, physics stepping and 60+ ImGui windows. It
  *would* be a better long-term shape, but it is a large refactor with zero
  e2e-specific payoff — the CLI-driven harness gets the e2e value without
  it. Revisit separately when the game grows (e.g. when a second planet
  lands).
- **gtest/ctest for e2e**: the existing unit tests are plain-`main()`
  C++ without a framework; e2e is better as a shell/Python harness around
  the binary anyway (it has to manage Xvfb, timeouts, and output capture).
- **`SDL_VIDEODRIVER=dummy` headless mode**: not possible without ripping
  the GL requirement out of `display.cpp`; Xvfb+llvmpipe is already
  installed and verified.
- **Pixel-perfect goldens up front**: Mesa-version brittleness; start with
  the not-black heuristic.
- **CI right now**: QWEN.md says don't push to GitHub; the battery is local
  (`make e2e`). The shape (runner + case files + `make e2e`) is CI-portable
  later — an Actions job would just need `xvfb` + the README's apt deps.

---

## 5. Risks and open questions

1. **Flaky timing until L2 lands.** Mitigation: for L1 cases, prefer
   wide-margin assertions (thresholds with 2–5× headroom) and set up state
   before the timed input where possible (e.g. start on a scenario instead
   of flying to it). L2 then tightens things properly.
2. **`FORBID GL_` may be environment-sensitive.** The Makefile's `test-gl`
   comment documents Mesa-26 quirks (VAO-0 draws fail). If llvmpipe emits
   benign `GL_` lines here, the forbid target narrows to
   `GL_INVALID_OPERATION`/`GL_OUT_OF_MEMORY` or the rule is dropped in favor
   of "no crash". Verify with the first `smoke` run.
3. **Some anchor strings need a confirming run** (marked `verify:` in §3):
   `Active ship 2 of 2` depends on `res/fleet.json` contents; the warp-entry
   success path currently prints nothing (a one-line `printf` there is a
   reasonable testability aid, in the spirit of the existing
   `Rails warp refused` line); `orbit-insert` thresholds come from an actual
   run.
4. **`--sim-mouse` coordinates assume 1920×1080** (hardcoded window). If the
   window size ever becomes a CLI option (the `// should be cli args` TODO
   at `main.cpp:64`), mouse cases must pass or derive coordinates from it.
5. **Startup cost × case count** is ~1–2 min for 9–10 cases here. If it
   ever hurts, the cheap test system (§3, L1) or a shared-Xvfb reuse in the
   runner are the levers — don't do either preemptively.

---

## 6. Sequencing and effort

| Step | Content | Size |
|---|---|---|
| 1 | `e2e/run.py` + case-file format + `make e2e` + `smoke`, `staging-basic`, `spin-regression`, `ui-click` | small; first end-to-end green run |
| 2 | `thrust-ascent`, `orbit-insert`, `rails-warp`, `ship-switch`, `soi-crossing` (with `verify:` pass) | small; pick thresholds from real runs |
| 3 | L2: tick counter + `--ticks` + `--press-ticks`/`--mouse-ticks` (+ `--fps` if the CPU spin bothers) | ~40–60 lines in `main.cpp`, C++11, local |
| 4 | Migrate timing-sensitive cases to tick units; tighten margins | small |
| 5 | (optional) L3: `--screenshot-tick` + not-black check + `--free-cam-*`-based visual case | small |

Nothing here blocks on each other except 4→(needs 3) and 5→(prefers 3).
Steps 1–2 already convert the current "run it by hand and eyeball it"
workflow (the QWEN.md `--sim-press` advice) into a repeatable gate.
