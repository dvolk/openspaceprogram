#!/usr/bin/env python3
"""Convert an e2e case's boot `--vab <def>` / `--vab-empty` into the player's
route into the editor: New Game (unless the boot already runs a game), the '4'
scene key, Game Debug's dev-vehicles checkbox, then the Load picker -- filter,
combo, row, Load.

Dry-run by default; --apply. A case is SKIPPED (never half-converted) when the
flag is the thing under test or the route cannot reach it. See the reasons it
prints.

The editor opens at EDITOR_MS and the def lands at LOAD_MS, so every scripted
event in the case shifts later and --timeout grows to match. Times shift by the
FIRST number of each scripted value, which is also each group's leading number
when a case repeats the flag; a value carrying two groups in one quoted string
is skipped rather than half-shifted.
"""
import argparse
import glob
import os
import re
import sys

EDITOR_MS = 2600        # '4' opens the editor (New Game ran first)
SHIFT_NO_DEF = 3500     # how much later the case's own events run
SHIFT_WITH_DEF = 7000
SHIFT_FLAGS = ("sim-press", "sim-mouse", "ui-click", "ui-type",
               "ui-scroll", "ui-focus", "ui-list")

# A boot that already runs a game needs no New Game, so the whole dance starts
# earlier and the case's events shift less.
GAME_FLAGS = ("--startship", "--startships", "--dock-test", "--radial-test",
              "--load")

def preamble(def_name, has_game):
    """The route, in ms. Returns (lines, editor_ms, load_ms)."""
    t = 1000
    lines = []
    if not has_game:
        lines = ['ARGS --ui-click %d,"Title Menu/New Game"' % t,
                 'ARGS --ui-click %d,"New Game/Start"' % (t + 600)]
        t += 1600
    editor = t
    lines.append('ARGS --sim-press %d,0,4' % t)
    t += 500
    lines.append('ARGS --sim-press %d,0,F1' % t)
    t += 500
    lines.append('ARGS --ui-click %d,"Game Debug/Dev vehicles in the VAB picker"' % t)
    t += 500
    lines.append('ARGS --sim-press %d,0,F1' % t)
    if not def_name:
        return lines, editor, None
    t += 500
    lines.append('ARGS --ui-type %d,"VAB TopBar/##vehfilter",%s' % (t, def_name))
    t += 500
    lines.append('ARGS --ui-click %d,"VAB TopBar/##vabload"' % t)
    t += 600
    lines.append('ARGS --ui-click %d,"%s"' % (t, def_name))
    t += 600
    lines.append('ARGS --ui-click %d,"VAB TopBar/Load"' % t)
    return lines, editor, t

def note(def_name, has_game, shift):
    head = ('# Reached the way a player reaches the editor: %s, then\n'
            '# the 4 key.'
            % ("the boot's start ship is already a game" if has_game
               else 'Title Menu -> New Game -> Start'))
    tail = ('# Game Debug turns on dev vehicles, which is what puts this def in the\n'
            '# Load picker; the filter narrows the roster to one row, and row + Load\n'
            '# do the rest. Same vabLoad() either way, so the assertions below are\n'
            '# unchanged -- and every scripted event here is shifted +%dms because\n'
            '# the editor opens at %.1fs, not at boot.' % (shift, EDITOR_MS / 1000.0))
    if not def_name:
        tail = ('# Game Debug turns on dev vehicles, which is what the palette and\n'
                '# the dev roster need. Every scripted event here is shifted +%dms\n'
                '# because the editor opens at %.1fs, not at boot.'
                % (shift, EDITOR_MS / 1000.0))
    return (head + "\n" + tail).split("\n")

def def_of(line):
    """The --vab <path> stem, or None for --vab-empty."""
    m = re.search(r'--vab\s+(\S+)', line)
    if not m:
        return None
    p = m.group(1)
    return os.path.basename(p)[:-5] if p.endswith(".json") else os.path.basename(p)

def convert(path, apply):
    lines = open(path).read().split("\n")
    body = [(i, l) for i, l in enumerate(lines) if not l.strip().startswith("#")]

    vab_line = None
    for i, l in body:
        if re.search(r'--vab(-empty\b|\s+\S+|\s*$)', l):
            vab_line = i
            break
    if vab_line is None:
        return "skip", "no `--vab`/`--vab-empty` ARGS line"
    has_empty = "--vab-empty" in lines[vab_line]
    def_name = def_of(lines[vab_line])
    joined = "\n".join(l for _, l in body)

    # --- what makes the flag the thing under test -------------------------
    if re.search(r'EXPECT .*error: --vab', joined):
        return "skip", "asserts the --vab loader's own error (the flag is the subject)"
    if re.search(r'EXPECT .*title -> vab', joined):
        return "skip", "asserts an editor opened from the Title, which no route reaches"
    if def_name and not (os.path.exists("res/ships/%s.json" % def_name)
                         or os.path.exists("e2e/vehicles/%s.json" % def_name)):
        return "skip", "def %s is in no tree the picker lists" % def_name

    # A boot that fails before the editor matters (a bad --startship site) needs
    # the flag gone, not a route: its clicks could never land, and a failed click
    # would trip the FORBID FAILED every routed case carries.
    if re.search(r'^EXIT\s+[1-9]', joined, re.M):
        out = [l for l in lines]
        l = re.sub(r'\s*--vab-empty\b', '', out[vab_line])
        m = re.search(r'--vab\s+\S+', l)
        if m:
            l = l[:m.start()].rstrip() + l[m.end():]
        out[vab_line] = l
        if apply:
            open(path, "w").write("\n".join(out))
        return "strip", "boot fails before the editor: flag dropped, no route"

    # A quoted value holding two groups would need its SECOND group's time
    # shifted too; leave the case whole for hand work.
    group_commas = {"sim-press": 2, "sim-mouse": 4}
    for _, l in body:
        for f, n in group_commas.items():
            m = re.search(r'--%s\s+(\S+)' % f, l)
            if m and m.group(1).count(",") > n:
                return "skip", "multi-group --%s value" % f

    has_game = any(f in joined for f in GAME_FLAGS)
    if not has_game and re.search(r'ui-click \d+,"Title Menu/New Game"', joined):
        has_game = True   # the case drives New Game itself

    # --- rewrite ----------------------------------------------------------
    insert_after = -1
    for i, l in enumerate(lines):
        if l.startswith("ARGS "):
            insert_after = i
        elif insert_after >= 0 and not l.strip().startswith("#") and l.strip():
            break

    shift = SHIFT_WITH_DEF if def_name else SHIFT_NO_DEF
    if has_game:
        shift = max(2000, shift - 1600)

    def bump_timeout(m):
        v = float(m.group(2)) + shift / 1000.0
        return m.group(1) + (str(int(v)) if v == int(v) else str(v))

    def shift_value(m):
        return m.group(1) + str(int(m.group(2)) + shift)

    out, timeout_seen = [], False
    for i, l in enumerate(lines):
        if i == vab_line:
            l = re.sub(r'\s*--vab-empty\b', '', l)
            l = re.sub(r'\s*--dev-vehicles\b', '', l)
            m = re.search(r'--vab\s+\S+', l)
            if m:
                l = l[:m.start()].rstrip() + l[m.end():]
            if l.strip() in ("ARGS", ""):
                continue
        if "--dev-vehicles" in l and i != vab_line:
            l = re.sub(r'\s*--dev-vehicles\b', '', l)
        if l.strip().startswith("#"):
            out.append(l)
            continue
        m = re.search(r'--timeout\s+(\d+(?:\.\d+)?)', l)
        if m:
            l = re.sub(r'(--timeout\s+)(\d+(?:\.\d+)?)', bump_timeout, l)
            timeout_seen = True
        for f in SHIFT_FLAGS:
            l = re.sub(r'(--%s\s+)(\d+)' % f, shift_value, l)
        out.append(l)
        if i == insert_after:
            pre, editor_ms, load_ms = preamble(def_name, has_game)
            out.extend(note(def_name, has_game, shift))
            out.extend(pre)
    if insert_after < 0:
        return "skip", "no ARGS block to anchor the preamble on"
    if not timeout_seen:
        return "skip", "no --timeout to grow"

    # The route is 4-10 clicks; a dead one must name itself rather than show up
    # as a missing game assertion three steps later. Unless the case's SUBJECT is
    # a refusal -- a case that EXPECTs a FAILED line (280, 282) cannot also
    # forbid the word -- or the case already names which FAILED lines it
    # tolerates: 247 forbids "re-aim FAILED" but not the palette toggle's
    # spurious release MISMATCH, which is what "Add fuel link" produces when it
    # renames itself to "Cancel fuel link" under the cursor (#320).
    wants_failed = any(l.startswith("EXPECT ") and "FAILED" in l for l in out)
    scoped_failed = any(re.match(r'FORBID\b.*\bFAILED\b', l.strip()) for l in out)
    if not wants_failed and not scoped_failed:
        # With no FORBID of its own to join, "after the last FORBID" lands the
        # line on line 1, above the case's prose (343). Anchor on the last
        # assertion instead.
        last = max((j for j, l in enumerate(out)
                    if re.match(r'(EXPECT|FORBID|CHECK|EXIT|FILE|NOFILE)\b', l.strip())),
                   default=len(out) - 1)
        out.insert(last + 1, "FORBID FAILED")

    if apply:
        open(path, "w").write("\n".join(out))
    return "convert", "%s -> editor by UI (+%dms%s)" % (
        def_name or "empty build", shift, ", game already up" if has_game else "")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--apply", action="store_true")
    ap.add_argument("paths", nargs="*")
    a = ap.parse_args()
    paths = a.paths or sorted(glob.glob("e2e/cases/*/*.txt"))
    done, stripped, skipped = [], [], []
    for p in paths:
        body = "\n".join(l for l in open(p).read().split("\n")
                         if not l.strip().startswith("#"))
        if not re.search(r'--vab(-empty\b|\s+\S+|\s*$)', body):
            continue
        verdict, why = convert(p, a.apply)
        (done if verdict == "convert" else
         stripped if verdict == "strip" else skipped).append((p, why))
    print("%s %d case(s):" % ("converted" if a.apply else "would convert", len(done)))
    for p, why in done:
        print("  %-52s %s" % (p, why))
    print("%s %d case(s):" % ("stripped" if a.apply else "would strip", len(stripped)))
    for p, why in stripped:
        print("  %-52s %s" % (p, why))
    print("skipped %d:" % len(skipped))
    for p, why in skipped:
        print("  %-52s %s" % (p, why))

main()
