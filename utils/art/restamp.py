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
import glob
import os
import re
import subprocess
import sys

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
STALE = re.compile(r"^stale geom: (\S+) id=(\S+) part=(\S+) mesh=(\S+) recorded ([0-9a-f]+), "
                   r"catalog now ([0-9a-f]+)")


def find_binary(explicit):
    if explicit:
        if not os.path.exists(explicit):
            sys.exit("%s: no such test binary" % explicit)
        return explicit
    # One build directory per arch/tune triple; take the most recent so a fresh `make` is
    # what gets read without the caller having to know their own ARCHDIR tokens.
    hits = sorted(glob.glob(os.path.join(REPO, "build", "*", "tests", "test_shipload")),
                  key=os.path.getmtime)
    if not hits:
        sys.exit("no test_shipload under build/*/tests -- run `make tests` first (or pass "
                 "--test-binary)")
    return hits[-1]


def report(binary):
    if not os.access(binary, os.X_OK):
        sys.exit("%s is not executable" % binary)
    r = subprocess.run([binary], capture_output=True, text=True, cwd=REPO)
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


def main():
    argv = sys.argv[1:]
    dry = "--dry-run" in argv
    binary = None
    if "--test-binary" in argv:
        i = argv.index("--test-binary")
        if i + 1 >= len(argv):
            sys.exit("--test-binary wants a path")
        binary = argv[i + 1]
    binpath = find_binary(binary)
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
        n = 0
        for have, wants in per_file[path].items():
            want = next(iter(wants))
            # Quoted so a hex inside an id or a comment cannot be caught: these values only
            # ever appear as a JSON string value.
            quoted = '"%s"' % have
            n += text.count(quoted)
            text = text.replace(quoted, '"%s"' % want)
        if dry:
            print("would rewrite %d stamp(s) in %s" % (n, path))
        else:
            open(full, "w", encoding="utf-8").write(text)
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
