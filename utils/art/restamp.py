#!/usr/bin/env python3
"""Re-stamp the geometry fingerprints the e2e save fixtures record.

Every part in an e2e save fixture carries a `geom` hex: a fingerprint of its mesh, written by
geomFingerprintHex (src/shipdef.h). The loader compares it against the catalog when the save is
read, so a mesh edit -- any edit, including a repaint that changes `vn` lines -- makes the
fixture's save refuse to load. That is the mechanism, not a test artifact: see #456 and the
Stage-1 art commit that broke case 867 this way.

tests/test_shipload.cpp sweeps the fixtures and prints the fix for each stale stamp; the sweep
is advisory (WARN) since 2026-11-25, so this is the command that applies it. It runs that test,
reads its own report rather than re-deriving hashes here -- a Python re-implementation of the
mixer would be a second truth to keep in step -- and rewrites the recorded hex in place,
touching nothing else in the file.

    python3 utils/art/restamp.py [--dry-run] [--test-binary PATH]

Fixtures whose drift is deliberate (save_geom_stale, save_geom_stale_root's named part) never
appear in that report, so they are never rewritten.
"""
import argparse
import glob
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STALE = re.compile(r"^stale geom: (\S+) id=(\S+) part=(\S+) mesh=(\S+) recorded ([0-9a-f]+), "
                   r"catalog now ([0-9a-f]+)")
# The hexes are whatever this code computes. Re-stamping from a binary older than it bakes a
# dead fingerprint into a fixture, which is the trap the Makefile notes for the test mixins.
HASHED_BY = ("src/shipdef.h", "src/shipdef.cpp", "tests/test_shipload.cpp")


def find_binary(explicit):
    if explicit:
        if not os.path.exists(explicit):
            sys.exit("%s: no such test binary" % explicit)
        binary = explicit
    else:
        # One build directory per arch/tune triple; take the most recent so a fresh `make` is
        # what gets read without the caller having to know their own ARCHDIR tokens.
        hits = sorted(glob.glob(os.path.join(REPO, "build", "*", "tests", "test_shipload")),
                      key=os.path.getmtime)
        if not hits:
            sys.exit("no test_shipload under build/*/tests -- run `make tests` first (or pass "
                     "--test-binary)")
        binary = hits[-1]
    if not os.access(binary, os.X_OK):
        sys.exit("%s is not executable" % binary)
    newer = [p for p in HASHED_BY if os.path.exists(os.path.join(REPO, p))
             and os.path.getmtime(os.path.join(REPO, p)) > os.path.getmtime(binary)]
    if newer:
        # Hexes out of a binary older than the mixer that defines them are fiction, and fiction
        # written into a fixture is a stale save that claims to be current.
        sys.exit("%s is older than %s -- run `bear -- make -j8 tests` first"
                 % (binary, " and ".join(newer)))
    return binary


def report(binary):
    r = subprocess.run([binary], capture_output=True, text=True, cwd=REPO)
    if r.returncode != 0:
        # The hexes come from this run. If the sweep is red for any reason, what it prints
        # about fingerprints is not trustworthy either, and re-stamping would bury the failure
        # under a fixture that matches the broken output.
        sys.exit("%s exited %d -- fix the sweep before re-stamping" % (binary, r.returncode))
    out, found = [], {}
    for ln in (r.stdout + r.stderr).splitlines():
        m = STALE.match(ln)
        if not m:
            continue
        path, _pid, _slug, _mesh, have, want = m.groups()
        if not os.path.exists(os.path.join(REPO, path)):
            print("note: %s names %s, which is not in the tree today" % (ln, path))
            continue
        prev = found.setdefault(path, {}).setdefault(have, set())
        prev.add(want)
        out.append((path, have, want))
    return out, found


def rewrite(text, table):
    """Swap recorded hexes for their current ones. Returns (text, swaps).

    One pass over the file, not one replace per stamp: applied in sequence, a chain (part a
    recorded with the hex part b used to hold, b recorded with the hex c used to hold) gives a
    the wrong value and counts b's field twice. Quoted matches only, so a hex inside an id or a
    note cannot be caught: these values only ever appear as a JSON string value.
    """
    pat = re.compile("|".join('"%s"' % h for h in sorted(table)))
    counted = [0]

    def fixup(m):
        counted[0] += 1
        return '"%s"' % table[m.group(0)[1:-1]]

    return pat.sub(fixup, text), counted[0]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dry-run", action="store_true",
                    help="print what would change, write nothing")
    ap.add_argument("--test-binary",
                    help="tests/test_shipload to read the report from (default: the newest one "
                         "under build/*/tests)")
    args = ap.parse_args()
    binpath = find_binary(args.test_binary)
    hits, per_file = report(binpath)
    if not hits:
        print("restamp: every geom stamp under e2e/fixtures matches the catalog (%s)" % binpath)
        return 0

    # Two parts in one file recording the same hex but wanting different ones means the hex
    # is not a unique key there, and a blind replace would guess. Stop and say so.
    for path, by_hex in per_file.items():
        for have, wants in by_hex.items():
            if len(wants) > 1:
                sys.exit("%s: %s is recorded by parts that want %s -- edit that file by hand"
                         % (path, have, " or ".join(sorted(wants))))

    changed = {}
    for path in sorted(per_file):
        full = os.path.join(REPO, path)
        text = open(full, encoding="utf-8").read()
        # One pass, not one replace per stamp: with sequential replaces a stamp that is a
        # recorded value in an earlier row picks up that row's fix, so a chain (a wants b's
        # old hex, b wants c's) silently writes the wrong value for a and counts it twice.
        table = {have: next(iter(wants)) for have, wants in per_file[path].items()}
        text, n = rewrite(text, table)
        expected = sum(1 for h in hits if h[0] == path)
        if n != expected:
            # The test reported stamps this pass did not match, so the file is not the shape the
            # report assumed (a hand edit in between, or a hex written bare).
            print("note: %s reported %d stale stamp(s) but %d matched here -- check the file"
                  % (path, expected, n))
        if dry:
            print("would rewrite %d stamp(s) in %s" % (n, path))
        else:
            # Write-then-rename: a Ctrl-C mid-write must not leave a truncated fixture, which
            # the loader reads as a corrupt save rather than a stale one.
            tmp = full + ".tmp"
            with open(tmp, "w", encoding="utf-8") as fh:
                fh.write(text)
            os.replace(tmp, full)
            print("re-stamped %d in %s" % (n, path))
        changed[path] = n

    if dry:
        return 0
    # Prove it: re-run the sweep. Reporting a fix that did not take is worse than reporting
    # nothing.
    again, _ = report(binpath)
    if again:
        print("restamp: %d still stale after writing -- %s" % (len(again), again[0]))
        return 1
    print("restamp: %d stamp(s) across %d fixture file(s); sweep is clean now"
          % (sum(changed.values()), len(changed)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
