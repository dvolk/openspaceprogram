#!/usr/bin/env python3
"""Convert an e2e case's boot `--load e2e/fixtures/<x>` into a Save/Load sheet
load driven through the UI.

Dry-run by default; --apply writes. A case is SKIPPED (never half-converted)
when the load is not a plain fixture path or the case asserts on something only
a boot load produces. See the skip reasons it prints.

The load lands at LOAD_MS (the sheet dance runs first), so every scripted event
in the case shifts later by SHIFT_MS and --timeout grows to match. SHIFT_MS
also leaves room for the clock resume: commitSaveWorld zeroes time_accel, and
only a BOOT load re-applies --time-accel (main.cpp:833). A sheet load therefore
starts PAUSED, so any case that flies, docks or burns after it would be pressing
keys against a frozen sim -- the Undock/Stage branches literally gate on
time_accel > 0. The preamble drops --time-accel and presses WarpUp once per
decade instead (paused -> 1x -> 10x -> 100x ...), which is the ladder a player
would climb.
"""
import argparse
import glob
import re
import sys

LOAD_MS = 4000          # when the sheet's Load button fires
SHIFT_MS = 5000         # how much later the case's own scripted events run
WARP0_MS = 4300         # first WarpUp press; +100ms per extra decade
SHIFT_FLAGS = ("sim-press", "sim-mouse", "ui-click", "ui-type",
               "ui-scroll", "ui-focus", "ui-list")

def warp_decades(accel):
    """WarpUp presses that rebuild `accel` from paused: 1 -> 1, 10 -> 2, ..."""
    if accel == 0:
        return 0
    decades, v = 1, accel
    while v > 1:
        if v % 10:
            return None          # not on the warp ladder
        v //= 10
        decades += 1
    return decades

def preamble(fix, decades, shifted):
    tail = ""
    if shifted:
        tail += (' -- and every scripted event here is shifted'
                 '\n# +%dms because the load now lands at %ds, not at boot.'
                 % (SHIFT_MS, LOAD_MS // 1000))
    if decades:
        tail += ('.' if not shifted else '\n#')
        tail += (' The PERIOD presses climb'
                 '\n# the warp ladder where --time-accel used to: a sheet'
                 '\n# load starts paused.')
    if not tail:
        tail = "."
    note = (
        '# Reached by the Save/Load sheet, not --load: Game Debug ticks the e2e'
        '\n# fixture root, the sheet picks the "fixtures" game, the slot filter'
        '\n# narrows the fixture list to this one row, and row + Load button do'
        '\n# the rest. Same load_game() either way, so the assertions below are'
        '\n# unchanged' + tail
    ).split("\n")
    lines = note + [
        'ARGS --sim-press 500,0,F1',
        'ARGS --ui-click 1000,"Game Debug/Load e2e fixture saves"',
        'ARGS --sim-press 1500,0,F1',
        'ARGS --ui-click 2000,"Title Menu/Save/Load"',
        'ARGS --ui-click 2500,"fixtures"',
        'ARGS --ui-type 3000,"Save/Load/##filter",%s' % fix,
        'ARGS --ui-click 3800,"%s"' % fix,
        'ARGS --ui-click %d,"Save/Load/Load##saveload"' % LOAD_MS,
    ]
    for k in range(decades):
        lines.append('ARGS --sim-press %d,0,PERIOD' % (WARP0_MS + 100 * k))
    return lines

def convert(path, apply):
    lines = open(path).read().split("\n")
    body = [(i, l) for i, l in enumerate(lines) if not l.strip().startswith("#")]

    # --- the load, and what it loads -------------------------------------
    load_line = load_fix = None
    for i, l in body:
        m = re.search(r'--load\s+(e2e/fixtures/([A-Za-z0-9_.-]+))', l)
        if m:
            load_line, load_fix = i, (m.group(1), m.group(2))
            break
    if not load_line:
        return "skip", "no `--load e2e/fixtures/<x>` ARGS line"
    full, fix = load_fix

    # --- what makes a boot load the thing under test ----------------------
    joined = "\n".join(l for _, l in body)
    for _, l in body:
        # An EXPECT that asserts the refusal is the thing under test. A FORBID
        # of the same words says the opposite -- a positive case pins it.
        if l.startswith("EXPECT ") and ("refused" in l or "Load failed" in l):
            return "skip", "asserts a refused load (the sheet cannot reach it)"
    if re.search(r'^EXIT\s+[1-9]', joined, re.M):
        return "skip", "demands a non-zero exit (boot-only failure mode)"
    if re.search(r'--system\s', joined):
        return "skip", "cross-system: --system sets the STARTING system"
    if re.search(r'EXPECT .*\[boot\]', joined):
        return "skip", "asserts a [boot] line"
    if re.search(r'^(NAME|CLAIM) .*boot', joined, re.M | re.I):
        return "skip", "the case is about boot"
    # A scripted value holds one group unless its comma count says otherwise:
    # --sim-press is TIME,DUR,KEY (2 commas), --sim-mouse TIME,DUR,X,Y,BTN (4).
    group_commas = {"sim-press": 2, "sim-mouse": 4}
    for _, l in body:
        for f, n in group_commas.items():
            m = re.search(r'--%s\s+(\S+)' % f, l)
            if m and m.group(1).count(",") > n:
                return "skip", "multi-group --%s value" % f

    # --- the clock the boot load was handed -------------------------------
    m = re.search(r'--time-accel\s+(\d+)', joined)
    accel = int(m.group(1)) if m else 0
    decades = warp_decades(accel)
    if decades is None:
        return "skip", "--time-accel %d is off the warp ladder" % accel
    if decades > 5:
        return "skip", "--time-accel %d needs %d warp presses" % (accel, decades)

    # Does the case have scripted events of its own to shift? If the load was
    # the only one, saying so in the note would be noise.
    shifted = False
    for i, l in body:
        if i == load_line or not l.startswith("ARGS "):
            continue
        if any("--%s" % f in l for f in SHIFT_FLAGS):
            shifted = True
            break

    # --- rewrite ----------------------------------------------------------
    # The preamble lands after the LAST ARGS line of the first ARGS block,
    # whether or not that line survives the --load removal.
    insert_after = -1
    for i, l in enumerate(lines):
        if l.startswith("ARGS "):
            insert_after = i
        elif insert_after >= 0 and not l.strip().startswith("#") and l.strip():
            break

    def bump_timeout(m):
        v = float(m.group(2)) + SHIFT_MS / 1000.0
        return m.group(1) + (str(int(v)) if v == int(v) else str(v))

    def shift_value(m):
        return m.group(1) + str(int(m.group(2)) + SHIFT_MS)

    out, timeout_seen = [], False
    for i, l in enumerate(lines):
        if i == load_line:
            l = re.sub(r'\s*--load\s+' + re.escape(full), '', l)
            if l.strip() in ("ARGS", ""):
                continue
        # A comment is prose: 188's header carries a fixture-MINT recipe whose
        # timings belong to a different command (--startship, no --load).
        if l.strip().startswith("#"):
            out.append(l)
            continue
        # --time-accel only ever meant the BOOT world; the preamble's warp
        # presses are what the sheet route can actually do.
        lean = re.sub(r'--time-accel\s+\d+\s*', '', l)
        if lean != l and lean.strip() in ("ARGS", ""):
            continue
        l = lean
        m = re.search(r'--timeout\s+(\d+(?:\.\d+)?)', l)
        if m:
            l = re.sub(r'(--timeout\s+)(\d+(?:\.\d+)?)', bump_timeout, l)
            timeout_seen = True
        for f in SHIFT_FLAGS:
            l = re.sub(r'(--%s\s+)(\d+)' % f, shift_value, l)
        out.append(l)
        if i == insert_after:
            out.extend(preamble(fix, decades, shifted))
    if insert_after < 0:
        return "skip", "no ARGS block to anchor the preamble on"
    if not timeout_seen:
        return "skip", "no --timeout to grow"

    # The sheet route drives 6-9 clicks; a renamed widget, a clipped row or an
    # ambiguous label otherwise surfaces as "EXPECT not found: Loaded game from"
    # three steps from the cause. run.py's DEFAULT_FORBID does not cover the
    # test engine's own lines, so pin it here.
    if not any(l == "FORBID FAILED" for l in out):
        # Anchor on the last assertion: with no FORBID of its own to join,
        # "after the last FORBID" would land the line above the case's prose.
        last = max((j for j, l in enumerate(out)
                    if re.match(r'(EXPECT|FORBID|CHECK|EXIT|FILE|NOFILE)\b', l.strip())),
                   default=len(out) - 1)
        out.insert(last + 1, "FORBID FAILED")

    # Pin the sheet's own toast: it is the new route's evidence.
    for j, l in enumerate(out):
        if l.startswith("EXPECT ") and ("Loaded game from " + full) in l:
            out.insert(j + 1, 'EXPECT [toast] Loaded fixtures/%s' % fix)
            break

    if apply:
        open(path, "w").write("\n".join(out))
    return "convert", "%s -> sheet load (+%dms, %dx)" % (full, SHIFT_MS, accel)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("paths", nargs="*")
    a = ap.parse_args()
    paths = a.paths or sorted(glob.glob("e2e/cases/*/*.txt"))
    done, skipped = [], []
    for p in paths:
        if "--load" not in open(p).read():
            continue
        verdict, why = convert(p, a.apply)
        (done if verdict == "convert" else skipped).append((p, why))
    print("%s %d case(s):" % ("converted" if a.apply else "would convert", len(done)))
    for p, why in done:
        print("  %-52s %s" % (p, why))
    print("skipped %d:" % len(skipped))
    for p, why in skipped:
        print("  %-52s %s" % (p, why))

main()
