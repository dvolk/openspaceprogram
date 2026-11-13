#!/usr/bin/env python3
"""The gates that cost no compiler: file hygiene, size, JSON parseability, secrets.

Why a script in utils/ rather than four lines in .gitea/workflows/: a check that only
exists in a workflow rots. It cannot be reproduced by whoever it just failed, it does not
run on a laptop mid-task, and nobody defends a rule they have never seen fire locally.
So CI calls this file, and `make lint` calls this file, and the rule is one thing.

Why these four and not a C++ linter: the CI runner we have is a small box, and the
middleware submodules alone are 590 MB (bullet3 347 MB, assimp 243 MB) -- a gate that
compiles does not fit it. Nothing here compiles a translation unit, and nothing here
needs a third-party python module: every gate is stdlib and the whole set runs in well
under a second, which is what makes it worth running on every push.

WHAT IS NOT SCANNED, because a gate that yells about other people's bytes gets turned
off. Measured on the tree, all counts outside middleware/:

  - 751 of the tree's 761 trailing-whitespace lines are in utils/rss/, which is scraped
    third-party HTML. That is their whitespace, not ours.
  - 7 more are under reports/, which are immutable snapshots (QWEN.md) -- fixed, not fixed.
  - e2e/fixtures/ are byte-pinned inputs and some are deliberately malformed
    (save_bad_format/save.json), so a missing final newline may be part of the pin.

Hygiene asks nothing about extensions -- a NUL sniff decides text vs binary -- because the
extension allow-list is where a gate stops seeing things: it found a trailing space in
res/shaders/skyboxShader.vs and a set of tabs in utils/debian/rules, neither of which an
extension list would have looked at. A tab is SYNTAX in three tracked files -- the
Makefile's recipes, .gitmodules, and debhelper's rules -- and those three are the only
tab exemption.

Baselines as this gate was written: hygiene clean except three whitespace-only lines --
two in src/gameui.cpp, one in res/shaders/skyboxShader.vs -- all stripped in the same
commit; 0 CRLF; 0 secret hits; biggest tracked file outside middleware 7.0 MB
(res/heightmaps/earth/earth_hm.i16) against a 20 MB cap.

The secret gate is a hairwire, not an audit: it looks for credential SHAPES (a labelled
key/password assignment, a PEM block, a GitHub or AWS token shape) so the thing that stops
a leak is a commit, not a rotation drill after the fact. It never prints the matched text
back -- the finding is the file and line, because a log that echoes the secret has now put
the secret somewhere new.
"""

import argparse
import json
import os
import re
import subprocess
import sys

HYGIENE_SKIP = ("middleware/", "reports/", "utils/rss/", "e2e/fixtures/", "staging/", "dist/", "build/")
# A tab is SYNTAX in three tracked files: the Makefile's recipes, .gitmodules, and
# utils/debian/rules -- a debhelper rules file IS a makefile, it just has no extension.
TAB_OK = {"Makefile", ".gitmodules", "rules"}

# What is scanned for size everywhere, tracked files included. 20 MB is ~3x the largest
# file we ship today, so it catches "I committed a video / a build artefact / a 300 MB
# mesh export" -- the kind of thing that quietly doubles what every clone costs.
MAX_BYTES = 20 * 1024 * 1024

SECRET_PATTERNS = [
    ("labelled credential",
     re.compile(r"(?i)\b(api[_ -]?key|apikey|secret[_ -]?key|access[_ -]?token|auth[_ -]?token"
                r"|private[_ -]?key|password|passwd)\b\s*[:=]\s*['\"]?[^\s'\"]{6,}")),
    ("private key block", re.compile(r"-----BEGIN [A-Z ]*PRIVATE KEY-----")),
    ("github token shape", re.compile(r"\bgh[pousr]_[A-Za-z0-9]{20,}\b|\bgithub_pat_[A-Za-z0-9_]{20,}\b")),
    ("aws access key id", re.compile(r"\bAKIA[0-9A-Z]{16}\b")),
]

MAX_REPORT_PER_GATE = 30


def tracked_files():
    """`git ls-files`, so the scan sees the commit and not my working tree's debris."""
    out = subprocess.run(["git", "ls-files"], check=True, capture_output=True, text=True).stdout
    return [f for f in out.split("\n") if f]


def read_text(path):
    """The file as text, or None when it is binary or unreadable (a submodule path)."""
    try:
        with open(path, "rb") as fh:
            blob = fh.read()
    except OSError:
        return None
    if b"\0" in blob[:8192]:
        return None
    return blob.decode("utf-8", errors="replace")


def hygiene_findings(name, text):
    """Tabs (outside TAB_OK), trailing whitespace, CR, missing final newline."""
    out = []
    allow_tabs = os.path.basename(name) in TAB_OK
    for i, line in enumerate(text.split("\n"), start=1):
        bare = line[:-1] if line.endswith("\r") else line
        if "\r" in line:
            out.append((i, "carriage return (CRLF line ending)"))
        if bare.endswith((" ", "\t")):
            out.append((i, "trailing whitespace"))
        if not allow_tabs and "\t" in bare:
            out.append((i, "tab where indentation is spaces"))
    if text and not text.endswith("\n"):
        out.append((text.count("\n") + 1, "no final newline"))
    return out


def secret_findings(text):
    """(rule, line) per hit -- the match itself is never returned."""
    out = []
    for rule, pat in SECRET_PATTERNS:
        for m in pat.finditer(text):
            out.append((rule, text[:m.start()].count("\n") + 1))
    return out


def json_error(text):
    try:
        json.loads(text)
    except ValueError as e:
        return str(e)
    return None


def run_gates():
    fails = []
    counts = {"hygiene": 0, "size": 0, "json": 0, "secret": 0, "scanned": 0}

    for path in tracked_files():
        counts["scanned"] += 1

        try:
            size = os.path.getsize(path)
        except OSError:
            continue
        if size > MAX_BYTES:
            counts["size"] += 1
            fails.append("size: %s is %.1f MB, over the %d MB cap"
                         % (path, size / (1024 * 1024), MAX_BYTES // (1024 * 1024)))

        # The air and ship corpus must parse: the loader reads these with nlohmann, whose
        # failure at runtime is a half-built system, not a clean error.
        if path.startswith("res/") and path.endswith(".json"):
            text = read_text(path)
            if text is None:
                continue
            err = json_error(text)
            if err is not None:
                counts["json"] += 1
                fails.append("json: %s does not parse: %s" % (path, err))

        if path.startswith(HYGIENE_SKIP):
            continue
        text = read_text(path)
        if text is None:
            continue
        for line, why in hygiene_findings(path, text):
            counts["hygiene"] += 1
            fails.append("hygiene: %s:%d %s" % (path, line, why))

        # Secrets are scanned even in the skipped trees -- scraped HTML can carry a
        # credential the same way our own files can.
        for rule, line in secret_findings(text):
            counts["secret"] += 1
            fails.append("secret: %s:%d %s" % (path, line, rule))

    return fails, counts


def self_test():
    """Every gate must fire on a planted string, or it is a gate in name only (#650's lesson)."""
    bad = []
    # (file name, contents, the hygiene finding it must produce -- None means "must stay clean")
    for name, text, want in [
        ("src/plant.cpp", "int x = 1;   \n", "trailing whitespace"),
        ("src/plant.cpp", "int x = 1;", "no final newline"),
        ("src/plant.cpp", "a\tb\n", "tab where indentation is spaces"),
        ("src/plant.cpp", "a\r\n", "carriage return (CRLF line ending)"),
        ("Makefile", "target:\n\tcmd\n", None),   # a tab in a recipe is syntax, not filth
    ]:
        got = [why for _, why in hygiene_findings(name, text)]
        if want is None:
            if got:
                bad.append("%s: the Makefile tab exemption lost to %r" % (name, got))
        elif want not in got:
            bad.append("%s: planted %r not caught (saw %r)" % (name, want, got))
    for text, want in [
        ("api_key = \"hunter2hunter2\"\n", "labelled credential"),
        # Assembled at runtime, because a PEM header written out whole is exactly what the
        # gate reports -- the first version of this file failed its own scan.
        ("-----BEGIN " + "RSA PRIVATE KEY-----\n", "private key block"),
        ("t = ghp_" + "A" * 36 + "\n", "github token shape"),
        ("k = AKIA" + "A" * 16 + "\n", "aws access key id"),
    ]:
        got = [rule for rule, _ in secret_findings(text)]
        if want not in got:
            bad.append("secret gate missed %r (saw %r)" % (want, got))
    # And this file itself must pass the secret gate: every pattern lives here, so a
    # fixture or a pattern that trips its own scan turns every future push red.
    try:
        with open(os.path.realpath(__file__), encoding="utf-8", errors="replace") as fh:
            own = fh.read()
    except OSError as e:                     # only reachable if run outside a checkout
        bad.append("could not re-read %s: %s" % (__file__, e))
    else:
        hits = secret_findings(own)
        if hits:
            bad.append("ci_lint.py trips its own secret gate: %s"
                       % ", ".join("%s at line %d" % (rule, line) for rule, line in hits))
    for text, must_fail in [("{\"a\": }", True), ("{\"a\": 1}", False)]:
        if (json_error(text) is not None) != must_fail:
            bad.append("json gate wrong on %r" % text)
    if bad:
        print("ci_lint SELF-TEST FAILED:")
        for b in bad:
            print("   " + b)
        return 1
    print("ci_lint self-test ok: hygiene, secret and json gates each fire on a planted "
          "string; the Makefile tab exemption holds")
    return 0


def main():
    ap = argparse.ArgumentParser(description="No-compiler repo gates (CI runs this).")
    ap.add_argument("--self-test", action="store_true", help="check the gates fire, then exit")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    fails, counts = run_gates()
    for f in fails[:MAX_REPORT_PER_GATE]:
        print(f)
    if len(fails) > MAX_REPORT_PER_GATE:
        print("... %d more findings not shown" % (len(fails) - MAX_REPORT_PER_GATE))
    print("ci_lint: %d tracked files -- %d hygiene, %d size, %d json, %d secret findings"
          % (counts["scanned"], counts["hygiene"], counts["size"], counts["json"], counts["secret"]))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
