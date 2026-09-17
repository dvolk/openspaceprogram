# Addendum — C1 reproduced under ASan; C2 was misdiagnosed

Written 2026-09-17 after rebuilding `osp_asan` at HEAD and applying the fixes.
This closes out §C of `verification.md`. Neither earlier file is edited; where
this disagrees with them, this wins.

## C1 confirmed — heap-use-after-free, reproduced

`osp_asan` rebuilt from HEAD (`make TARGET=osp_asan OBJDIR=obj_asan
SANITIZE='-g3 -fsanitize=address -fsanitize=leak -fsanitize=undefined'`; the
shared `obj/imgui` and `obj/implot` objects were backed up first and verified
byte-identical afterwards, so the normal `osp` build is unaffected).

`--selftest-spawn` step 3 (remove the active ship) aborts on the unfixed tree:

```
==520437==ERROR: AddressSanitizer: heap-use-after-free on address 0x6db00478e580
READ of size 8 at 0x6db00478e580 thread T0
    #0 ... in Game::remove_ship(Vehicle*) src/game.cpp:1057
    #1 ... in main src/main.cpp:690
0x6db00478e580 is located 0 bytes inside of 936-byte region
freed by thread T0 here:
    #1 ... in Game::remove_ship(Vehicle*) src/game.cpp:1044
SUMMARY: AddressSanitizer: heap-use-after-free src/game.cpp:1057 in Game::remove_ship(Vehicle*)
```

(Line numbers are the pre-fix file: `1044` is `delete v`, `1057` is
`x->isCrewAboard()`.) The **READ of size 8 at offset 0** of the freed object is
the vptr — i.e. exactly the virtual-dispatch read predicted, not an incidental
field access. `isCrewAboard()` is virtual (`vehicle.h:462`), the identity check
`x == v` sat one statement *below* it, and `all` is snapshotted before the
delete so it still holds the freed pointer.

Reproduction log: `tmp/asan_baseline_2026_09_17.log`.

## C2 was wrong — the dangling-`ship` path is unreachable

`verification.md` §C2 claimed the `next == nullptr` branch is "reachable from the
Ship List with a two-ship fleet whose non-active ship is crewed". It is not. The
claim rests on reading `isCrewAboard()` as *"this ship has crew aboard"*; it
actually means *"this vehicle is a crew character currently aboard a capsule"*:

- `Vehicle::isCrewAboard()` returns `false` unconditionally (`vehicle.cpp:887`) —
  a capsule carrying crew is **selectable**.
- `Kerbal::isCrewAboard()` overrides it to `isAboard()` (`eva.h:52,61`).

That is also why the removal guard at `game.cpp:1020` needs *both* clauses
(`v->isCrewAboard() || !shipCrew(v).empty()`) — the first alone would not catch a
crewed ship.

So after removing `v` (which the guard ensures carries no crew), at least one
selectable entry always remains in `all`:

- any surviving **ship** → `isCrewAboard()` false → selectable;
- a **free** kerbal (on EVA, in its body's ship list) → `isAboard()` false →
  selectable;
- an **aboard** kerbal → not selectable, but `collectVehicles` (`ships.cpp:27-36`)
  pushes each ship's crew *immediately after that ship*, so its carrier is in
  `all` too, is a ship, and is not `v` → selectable.

Hence the forward loop, or its reverse-order fallback (`game.cpp:1065-1070`,
reachable whenever `v` is the last entry), always finds a successor.

**The branch was still worth closing**, but as a latent hazard rather than a live
bug: the pre-existing `if(next != nullptr)` guard shows the author already
contemplated a null successor, and the missing `else` left `ship` pointing at
freed memory in that case. It is now routed into the existing no-ship state
(`syncShipFocus()`), which is also where stage 4 of the refactor will put
`replaceScene(Title)`.

Consequence for the refactor: §C2's advice that the stage-4 invariant must be
"ship is non-null **and live**" is over-cautious — `enterFlight` checking
non-null is sufficient, because a non-null `ship` is always live.

## What changed

- `src/game.cpp` `remove_ship`: the `x == v` pointer compare moved above the
  virtual `isCrewAboard()` call (C1); an `else` arm added for a null successor
  that enters the no-ship state instead of leaving `ship` dangling (C2,
  defensive).
- `src/main.cpp` `--selftest-spawn` step 3: asserts the handoff actually happened
  (`ship != sp2 && ship != nullptr`) **before** the `printf` that dereferences
  `ship->name` — previously a failed handoff would have crashed the selftest
  instead of reporting a failure.

## Verification after the fix

- `osp_asan --selftest-spawn`: no AddressSanitizer report; "all checks passed" +
  "30 ticks after spawn/remove, no crash; OK", exit 0
  (`tmp/asan_fixed_2026_09_17.log`).
- `make test`: exit 0, 18 pass markers, no failures
  (`tmp/maketest_2026_09_17.log`).
- e2e subset (smoke, ship-switch, all five eva cases, dock, undock,
  dock-approach, save, save-load): 12/12 passed
  (`tmp/e2e_removeship_2026_09_17.log`).
- No e2e case exercises `remove_ship` — its only callers are the Ship List's "x"
  button (`gameui.cpp:1310`) and `--selftest-spawn` (`main.cpp:677,690`). The
  selftest **is** the coverage for this path, which is why it was strengthened.

## Not addressed here

C3 (`vabLaunch` leaking the camera park) and C4 (`load_game` clearing the fleet
before it can fail) are unchanged. C3 is fixed structurally by stack rule 4 in
stage 2; C4 is a small standalone fix in `save.cpp` and is a prerequisite for the
three-arm load handling in §A3.
