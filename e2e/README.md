# E2E cases

`e2e/INDEX.md` is the map: one row per case with its claim and kind. Read that
before the case files, and regenerate it after editing a case
(`python3 e2e/index.py`; `--check` fails if it is stale).

## Layout

Cases live under a subject directory: `flight/`, `crew/`, `science/`, `vab/`,
`save/`, `ui/`, `render/`, `harness/`. Selectors match the case name, the
filename and the relative path:

```
python3 e2e/run.py 172              # one case by number
python3 e2e/run.py save/172         # one case by path
python3 e2e/run.py 172 177 262      # up to five at once
```

There is no full-battery mode. At most five cases run per invocation; a wider
selection is refused with the list of what it matched. A run also starts a
10-minute cooldown: until it expires, every invocation refuses, including
`make e2e`, and there is no flag that skips it.
`--kind` is a filter on top of that narrow selection (e.g.
`172 177 --kind invariant`), not a way to select the whole suite. Prefer the
unit tests for anything whose claim is pure logic.

## Getting somewhere through the UI, not a flag

Where a player route exists, a case takes it. Two routes cover the save and the
editor:

- **A saved game.** Game Debug (F1) ticks the e2e fixture root; the Load sheet
  picks the "fixtures" game; the slot filter narrows ~47 slots to one row; row +
  Load call the same `load_game()` that `--load` did. A load starts PAUSED, so
  the preamble presses PERIOD (WarpUp) once per decade of the warp the case used
  to pass as `--time-accel`. `e2e/convert_load_to_ui.py` writes this preamble.
- **The VAB.** Title Menu -> New Game -> Start, then the `4` key; Game Debug
  turns on dev vehicles, which is what puts a def in the Load picker (the
  `testship` marks and `e2e/vehicles`); the filter narrows the roster, and row +
  Load call the same `vabLoad()`. `e2e/convert_vab_to_ui.py` writes that one.

Both shift the case's own scripted events later -- the setup lands at 2-6 s, not
at boot -- and grow `--timeout` to match. Both add `FORBID FAILED`: a route is
4-10 clicks, and a dead click should name itself instead of surfacing as a
missing assertion three steps later.

Fixtures live in `e2e/fixtures/` (saves, rosters, start-ship lists) and
`e2e/vehicles/` (ship defs the picker lists). Game Debug's two switches are the
only way those trees reach the game, and both grey out when the tree is absent,
which is what a packaged build has.

Boot flags that have no player route yet are still here (`--startship`,
`--kerbals`, `--system`, `--dev-scenarios`); see the open issues for which ones
are next.

## What a case asserts

Every case declares both, and `run.py` refuses to load one without them:

```
NAME    dock
CLAIM   Two ports in range mate into one vessel and the absorbed hull leaves the world once.
KIND    invariant
```

`KIND` is the weight of the evidence, not a topic:

- `invariant` -- a behaviour or player route the game must keep
- `golden` -- tuned numbers or a content table; update deliberately when tuning
- `harness` -- CLI/test-engine plumbing, not game spec
- `canary` -- only proves the thing boots

Do not read an `EXPECT` string as a requirement on its own. The claim says what
the case is for; a `golden` case's numbers are the current tuning, not a law.

A path a case names is checked before the case runs: every `e2e/...` or
`res/...` string in ARGS, SEED, or the text of an EXPECT/FORBID/CHECK has to
exist in the repo. A fixture that moved then fails with its own path named
instead of as a missing assertion three lines later. Case 89 pins a refusal of
a system file that must NOT exist, so it opts out with a `# missing-ok: <path>`
line -- the only case that needs it. `python3 e2e/index.py --check` parses every
case file, and `make test` runs it, so a stale path fails on the commit that
introduces it rather than days later when someone spends a run on it.

GL errors and crash strings (`Segmentation`, `Assertion`, `terminate called`,
shader compile/link failures) are forbidden in every case, so do not repeat
them in `FORBID`. `error:` is not default: a clean boot refusal prints it on
purpose. `NOGLCHECK` turns the GL drains off for a case whose subject is frame
timing.

## Driving the UI in a case

The case format (NAME/ARGS/EXPECT/CHECK/...) is documented in `run.py`'s
docstring. This section is about the part that is easy to get wrong: clicking the
game's own widgets instead of using a headless hook.

Discover the address first. `--ui-list MS` prints every clickable item as
`window="..." label="..." rect=... [disabled]`, and `--ui-click MS,"Window/Label"`
matches that pair exactly, imgui's `##` disambiguator included. A path that
guesses at a label fails with `not found`; a path that matches two items fails
as `ambiguous`. `--ui-list` is the source of truth, not the source code.

A click presses in one frame and releases in the next, so a click can never be
same-frame with a hook phase. That is the only reason a hook still exists in a
UI-driven case: 244 needs a detach and a launch in one frame, 243 needs a scan
between two passes, 237 needs a plan the UI refuses to build.

For a 3D pick, `--ui-focus MS,PART_ID` parks the cursor on a pixel whose real
raycast lands on that part, and `--sim-mouse MS,0,CUR,CUR,BTN` clicks there. A
fixed pixel on a multi-part build picks whatever is under it, which is how 233
used to fail. The aim prints `[uifocus] ... -> x,y` or `FAILED <why>`, so every
such case carries `FORBID FAILED`. In the VAB, RMB there opens the part window,
whose title is the catalog label with the build id as its suffix --
`Capsule (3m)##capsule_r1.5h3_1/Seat 1##s0`, not a name you can invent.

A greyed-out row is an assertion, not a mistake. `--ui-click` refuses a disabled
widget, and the refusal line is what the case EXPECTs (236, 237). Pin WHICH
refusal: there is one at aim time and one at release (`FAILED disabled at
release`, uiinput.cpp:185), and the release wording contains the aim-time
substring, so forbid the other one.

An open combo owns the mouse. A refused row does not close the dropdown, so the
next click hits the popup's scrim and reports `MISMATCH nothing was hovered at
release`. Picking a row is the only way out -- click `##Combo_00/Empty` (236,
237). Nothing else closes it: the game binds Escape to a scene action and imgui's
keyboard nav is never enabled.
