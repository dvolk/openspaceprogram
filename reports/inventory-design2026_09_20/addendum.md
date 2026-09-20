# Addendum: phases 0, 1.1, 1.2 and 1.3 as built

Date: 2026-09-20. Companion to `inventory-design.md` (rev 2) in this directory.
Records what implementing the first four steps actually turned up — including
one finding materially worse than the review predicted.

## 1. The duplicate-id bug was a crash, not a wrong controller

`inventory-design.md` §1.7(a) predicted two consequences of resolving part
references by string id: a wrong controller (Repro A) and a permanently
un-undockable ship (Repro B). Both were reasoned from the code and both are
real. Neither is what actually happens first.

Docking a `--dock-test near` pair, saving, and loading that save **segfaults**
on the pre-uid code:

```
$ ./osp --dock-test near --time-accel 1 --timeout 3 --save tmp/e2e_docked_old
[dock] t=0.020 a="dockprobe" b="dockstation" d=0.750 m v=0.028 m/s
Saved 1 ship(s) to tmp/e2e_docked_old
$ ./osp --load tmp/e2e_docked_old --time-accel 1 --timeout 3
Segmentation fault (core dumped)          # exit 139

Thread 1 "osp" received signal SIGSEGV
#0  Vehicle::partWorldPose(Part const*, glm::dvec3&, glm::dmat3&) const
#1  main ()
```

The game could not load a docked ship it had saved itself.

### Why: `docktest.cpp` never sets `Part::id`

The merged ship's save record, pre-uid:

```
nparts     14
ids        ['', '', '', '', '', '', '', '', '', '', '', '', '', '']
parents    ['<absent>'] * 14
controller <absent>
docks      [{'name': 'dockstation', 'port': '', 'root': ''}]
```

`--dock-test` builds both halves straight from the parts catalog
(`src/docktest.h`) and leaves `Part::id` empty, so the collision is not "two
ships from the same def share generated ids" — it is *all fourteen parts share
the empty string*. Three separate consequences follow from the old format's
empty-means-absent conventions:

- `savePartToJson` writes `parent` only `if(!p.parent.empty())`, and
  `p->parent->id` is `""` for every part — so **`parent` was omitted for all
  14 parts**, not just the root. On load, part *i* resolved
  `idToIndex[""]`, which the previous iteration had just set to *i-1*. The part
  tree therefore came back as a **flat 14-part chain**, with the station's root
  attached to the last probe part instead of to the probe's docking port.
- `controller` was omitted for the same reason, so `finalize()`'s default
  picked it (Repro A's mechanism, arriving by a different route).
- The seam's `port` and `root` were both `""`, so both resolved to the
  last-inserted part — index 13, a leaf.

`partWorldPose` then ran over that structure and faulted.

This is a strictly worse instance than §1.7(a) described, and it is reachable
from a single CLI flag rather than needing two same-def ships to be docked.

### After the fix

The same save, loaded by the uid-keyed code, is refused explicitly rather than
crashing:

```
Load failed: load: saved ship 'dockprobe' part '' has no uid (save predates part identity)
exit 1
```

and a save written by the new code round-trips correctly:

```
Loaded game from tmp/e2e_docked (active: dockprobe)
[fuel] t=3.100s ship="dockprobe" g0=H2:626.8/626.8[626.8] ... g1=H2:626.8/626.8[626.8] ...
[undock] t=3.660 a="dockprobe" b="dockstation"
exit 0
```

with the seam and controller keyed by uid in the file:

```
uids    : [8, 9, 10, 11, 12, 13, 14, 1, 2, 3, 4, 5, 6, 7]   (probe, then station)
ctrl    : 12
docks   : [{'name': 'dockstation', 'port': 8, 'root': 1}]
```

## 2. The committed fixtures were stale beyond the format

Regenerating `save_racer` from a live capture with the same command that
produced the original (`--scenario rot-orbit --ship res/ships/racer.json
--time-accel 1 --timeout 3`, which reproduces the original's `time` of
3.0800000000000023 exactly) showed two state differences unrelated to uids:

| | old fixture | regenerated |
|---|---|---|
| `throttle` | 0.0 | 1.0 — `Vehicle::thruster_util`'s default, so a freshly spawned racer saves at full throttle |
| crew entry | no `pose`, no `onRails` | both present — `saveShipFromVehicle`'s crew branch writes them |
| `save.json` | carries a `"scene": "flight"` key | absent — `SaveMeta` has no such field, so the old key was vestigial and the permissive reader ignored it |

The fixtures had drifted from the code that writes them. They are regenerated
rather than hand-patched; `regen_fixtures.py` in this directory is the script,
and it backs the outgoing fixtures up to `tmp/fixtures_old/` before replacing
them. It takes the racer capture directory as an argument and derives the repo
root by walking up to the `Makefile`, so it runs from either `tmp/` or here.

## 3. New coverage

**`e2e/cases/79-dock-save-load.txt`** + fixture `e2e/fixtures/save_docked`
(the 14-part merged vessel). Two assertions, chosen to be structural rather
than incidental:

- **Two fuel groups.** The docking ports are `fuel_barrier`, so a correctly
  rebuilt tree puts the probe's tanks in one group and the station's in
  another. A flattened chain places both barriers in series and yields three
  conducting segments — three groups. This directly discriminates the failure
  mode found in §1.
- **Undock works and names `dockstation`.** A seam resolving to the wrong part
  either extracts the wrong subtree or finds nothing to extract, reports
  "Cannot undock", and returns before `seams.pop_back()` — the wedge described
  in Repro B.

The group check was verified to be a live discriminator, not a vacuous pass:
flipping it to `== 3` produces `CHECK failed`, and it was restored.

**`tests/test_save.cpp`** gains two cases at the pure-JSON layer: a reference
keyed by *string* where a uid is expected reads back as 0 (exactly what an
old-format save looks like, and why 0 is a hard error rather than a miss), and
a two-part ship whose parts share an `id` but not a `uid` round-trips with both
distinguishable — controller, fuel link, parent and seam all resolving to the
second part rather than the first.

**`tests/test_dock.cpp`** gains `test_uid` (committed with phase 1.1): uids
distinct within a ship, across two independently built ships, and across a
merge that collides every `id` — asserted, not assumed — plus that a split
neither remints nor reuses.

## 4. Loader strictness, deliberately increased

Five paths that used to degrade silently now refuse the load. Each is a case
where guessing produces a wrong ship rather than an obviously broken one:

| condition | before | after |
|---|---|---|
| part with `uid == 0` | n/a | throw "save predates part identity" |
| two parts in one ship sharing a uid | last writer wins | throw |
| two ship files claiming one uid | n/a | throw |
| `controller` names an absent part | silent fallback to `finalize()`'s default | throw |
| dock seam names an absent part | silent `continue` (the seam vanished) | throw |

One path stays lenient, on purpose: a **dock intent** (`dock_target_port`,
`dock_arm_port`) that names a part which is gone simply is not restored. A
stale target is an ordinary runtime state — `Game::updateDocking` already
validates and drops one whose ship or port went away — whereas a dropped seam
strands a real joint. The asymmetry is commented at the call site so it does
not read as an oversight.

`dock_target_port` also gained a membership check it did not have before: the
uid map spans every ship file, so a uid alone does not prove the part belongs
to the ship named beside it, and `updateDocking` assumes it does.
`findSavedPart` resolves the uid *and* confirms the part is in that ship's list.

## 5. Known gaps and follow-ups

- **Repro B (two seams) is not covered end-to-end.** No hook builds two
  sequential docks, so the "dock B then C, both must stay undockable" case
  cannot be driven from a CLI fixture. It moves to step 1.4, where making
  `absorbShip` copy `B->seams` needs a `test_dock` case for two sequential
  absorbs anyway — that is the natural place to pin it.
- **`docktest.cpp` and `radialtest.cpp` leave `Part::id` empty.** Harmless now
  that uid is the key, but it is why §1's crash was total rather than partial,
  and `id` is still what the diagnostics and the VAB show. Worth setting when
  those two files move out of `src/` (§7 of the main report).
- **`Part::id` is now write-only in the save path.** It is persisted and
  restored, but nothing resolves by it any more. It stays as the
  authoring-facing name; if it later turns out nothing reads it, drop it from
  the format.

## 6. Verification

- `make test`: green, 0 failures across all suites (`test_dock` 35 checks,
  `test_inertia` 959, `test_crew` 333, `test_save` OK). The six
  `-Wunused-result` warnings on `system()` in `test_save.cpp` are pre-existing;
  the line numbers moved because of the inserted cases.
- e2e 15/15: `01-smoke`, `02-staging-basic`, `06-ship-switch`, `26-eva-walk`,
  `29-eva-spawn`, `30-eva-rcs-fuel`, `45-dock`, `46-undock`,
  `47-dock-approach`, `53-save`, `54-save-load`, `59-vab-load`, `64-reload`,
  `65-reload-refused`, `79-dock-save-load`.
- Before/after established by `git stash push src/save.h src/save.cpp`,
  rebuilding, and running the identical commands on the pre-uid binary, then
  `git stash pop`. `src/part.h` (phase 1.1, already committed) was left in
  place for the "before" run; its `uid` field is unread by the old save code,
  so the comparison is clean.
