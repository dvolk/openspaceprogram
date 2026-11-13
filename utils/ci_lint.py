#!/usr/bin/env python3
"""The gates that cost no compiler: file hygiene, size, JSON parseability, secrets.

Why a script in utils/ rather than four lines in .gitea/workflows/: a check that only
exists in a workflow rots. It cannot be reproduced by whoever it just failed, it does not
run on a laptop mid-task, and nobody defends a rule they have never seen fire locally.
So CI calls this file, and `make lint` calls this file, and the rule is one thing.

Why these four and not a C++ linter: the CI runner we have is a small box, and the
middleware submodules alone are 590 MB (bullet3 347 MB, assimp 243 MB) -- a gate that
compiles does not fit it. Nothing here compiles a translation unit, and nothing here
needs a third-party python module: every gate is stdlib and the whole set runs in about a
second, which is what makes it worth running on every push.

WHAT HYGIENE IS NOT ASKED OF, because a gate that yells about other people's bytes gets
turned off. After the baseline was cleaned, 758 trailing-whitespace lines remain outside
middleware/: 751 in utils/rss/ (scraped third-party HTML -- their whitespace, not ours)
and 7 under reports/, which are immutable snapshots (QWEN.md). e2e/fixtures/ are
byte-pinned inputs, some deliberately malformed (save_bad_format/save.json), where a
missing final newline may be part of the pin.

That skip list is HYGIENE only. The secret scan runs on every tracked text file, the
scraped and pinned trees included -- those are exactly where a credential gets pasted: a
worklog under reports/, a fixture under e2e/fixtures/. An earlier draft `continue`d past
the skip list and lost the secret scan with it, while its comment promised the opposite;
--self-test now pins the split, because a gate that reports "0 secret findings" for a tree
it never opened is indistinguishable from a clean tree.

Hygiene asks nothing about extensions -- a NUL sniff decides text vs binary -- because an
extension allow-list is where a gate stops seeing things: it found a trailing space in
res/shaders/skyboxShader.vs and tabs in utils/debian/rules, neither of which a list would
have looked at. A tab is SYNTAX in exactly three tracked files, named by full path.

Baselines as this gate was written: hygiene clean; 0 CRLF; 0 secret hits; biggest tracked
file outside middleware 7.0 MB (res/heightmaps/earth/earth_hm.i16) against a 20 MB cap.

The secret gate is a hairwire, not an audit: credential SHAPES (a labelled assignment, a
key block, a provider token shape, an env assignment whose name says secret) so the thing
that stops a leak is a commit rather than a rotation drill afterwards. It never prints the
matched text -- the finding is file and line, because a log that echoes the secret has put
the secret somewhere new. Tuning it against this repo, not against imagination: the first
version called `passwd = getpwuid(getuid())` a credential; the second called
`token = token.replace(...)` one, because a bare `{12,}` backtracks to exactly twelve
characters. Both cases are pinned in --self-test as must-stay-quiet, alongside must-fire
fixtures that are assembled at runtime -- this file is scanned too, and a gate whose own
source trips it turns every future push red. A gate that cries wolf on ordinary code is a
gate that gets muted.
"""

import argparse
import json
import os
import re
import subprocess
import sys

# Hygiene only -- see the docstring: the secret scan ignores this list.
HYGIENE_SKIP = ("middleware/", "reports/", "utils/rss/", "e2e/fixtures/", "staging/", "dist/", "build/")
# Full paths, not basenames: a basename match would exempt every file named `rules` that
# anyone ever adds.
TAB_OK = {"Makefile", ".gitmodules", "utils/debian/rules"}

# 20 MB is ~3x the largest file we ship today, so it catches "I committed a video / a
# build artefact / a 300 MB mesh export" -- the kind of thing that quietly doubles what
# every clone costs. An over-cap file is reported and NOT read: slurping the thing we just
# called too big is how a 2 GB mistake turns into an OOM instead of one clean line.
MAX_BYTES = 20 * 1024 * 1024

# Three shapes, each checked against the real lines in this repo, because every rule that
# cries wolf on ordinary code gets muted:
#  - a labelled assignment: a credential WORD then `:`/`=`, then a quoted literal or a
#    12+ char bare run. The run ends at a boundary the class knows (`(?![.../(])`), NOT at
#    `(?!\()` -- with a bare `{12,}` the engine backtracks to exactly 12 characters and
#    `token = token.replace(...)` / `os.environ.get(...)` both "match". `\\?["']` so an
#    escaped quote (JSON in code) still counts.
#  - a key block: PEM or PGP.
#  - an unambiguous provider shape, or an env-style assignment whose NAME says secret.
# That is why the self-test's fixtures are assembled at runtime: this file is scanned too.
SECRET_PATTERNS = [
    ("labelled credential",
     re.compile(r"(?i)\b(api[_ -]?key|apikey|secret[_ -]?key|access[_ -]?token|auth[_ -]?token"
                r"|private[_ -]?key|password|passwd|secret|token)\b"
                r"\s*[:=]\s*(?:\\?['\"][^'\"]{6,}\\?['\"]|[A-Za-z0-9\-_+=/.]{12,}(?![A-Za-z0-9\-_+=/(]))")),
    ("private key block", re.compile(r"-----BEGIN [A-Z ]*PRIVATE KEY( BLOCK)?-----")),
    ("github token shape", re.compile(r"\bgh[pousr]_[A-Za-z0-9]{20,}\b|\bgithub_pat_[A-Za-z0-9_]{20,}\b")),
    ("aws access key id", re.compile(r"\bAKIA[0-9A-Z]{16}\b")),
    ("slack/gitlab token shape", re.compile(r"\bxox[baprs]-[A-Za-z0-9-]{10,}\b|\bglpat-[A-Za-z0-9_\-]{20,}\b")),
    ("env-style secret assignment",
     re.compile(r"(?im)^[ \t]*(?:export[ \t]+)?[A-Z0-9_]*"
                r"(?:SECRET|API_KEY|ACCESS_KEY|AUTH_TOKEN|PASSWORD|CREDENTIAL)[A-Z0-9_]*"
                r"[ \t]*=[ \t]*[A-Za-z0-9/+_\-]{16,}")),
]

MAX_HYGIENE_REPORT = 30
MAX_OTHER_REPORT = 100


def tracked_files():
    """(files, submodules) from `git ls-files -z -s`.

    -z for two reasons: paths arrive unquoted (a C-quoted name like '"caf\\xc3\\xa9.txt"'
    cannot be stat'd, and an un-stat'able file used to skip every gate and still count as
    scanned) and it is correct for names containing newlines. -s so submodule gitlinks are
    recognised by mode 160000 instead of by the accident of open() failing.
    """
    raw = subprocess.run(["git", "ls-files", "-z", "-s"], check=True, capture_output=True).stdout
    files, submodules = [], []
    for entry in raw.split(b"\0"):
        if not entry:
            continue
        meta, _, name = entry.partition(b"\t")
        path = os.fsdecode(name)
        if meta.startswith(b"160000 "):
            submodules.append(path)
        else:
            files.append(path)
    return files, submodules


def read_text(path):
    """The file as text, or None when it is binary (NUL in the first 8 KiB)."""
    with open(path, "rb") as fh:
        blob = fh.read()
    if b"\0" in blob[:8192]:
        return None
    return blob.decode("utf-8", errors="replace")


def hygiene_applies(path):
    return not path.startswith(HYGIENE_SKIP)


def size_finding(name, nbytes):
    if nbytes > MAX_BYTES:
        return ("size: %s is %.1f MB, over the %d MB cap"
                % (name, nbytes / (1024 * 1024), MAX_BYTES // (1024 * 1024)))
    return None


def hygiene_findings(name, text):
    """Tabs (outside TAB_OK), trailing whitespace, CR, missing final newline."""
    out = []
    allow_tabs = name in TAB_OK
    for i, line in enumerate(text.split("\n"), start=1):
        # Strip a trailing CR before the trailing-whitespace test so a CRLF file reports
        # the carriage return once, not CR plus "trailing whitespace" on every line.
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
    """finds maps gate -> list of printable findings."""
    files, submodules = tracked_files()
    finds = {gate: [] for gate in ("hygiene", "size", "json", "secret", "unread")}

    for path in files:
        try:
            size = os.path.getsize(path)
        except OSError as e:
            # Never silent: an unreadable tracked path is a hole in every gate below.
            finds["unread"].append("unread: %s (%s)" % (path, e.strerror or e))
            continue
        big = size_finding(path, size)
        if big is not None:
            finds["size"].append(big)
            continue

        try:
            text = read_text(path)
        except OSError as e:
            finds["unread"].append("unread: %s (%s)" % (path, e.strerror or e))
            continue

        if text is None:
            continue

        # The air and ship corpus must parse: the loader reads these with nlohmann, whose
        # failure at runtime is a half-built system, not a clean error. A res JSON we
        # cannot read as text is a finding, not a pass -- an embedded NUL or a UTF-16 file
        # from a Windows editor is exactly what would otherwise sneak through.
        if path.startswith("res/") and path.endswith(".json"):
            err = json_error(text)
            if err is not None:
                finds["json"].append("json: %s does not parse: %s" % (path, err))

        if hygiene_applies(path):
            for line, why in hygiene_findings(path, text):
                finds["hygiene"].append("hygiene: %s:%d %s" % (path, line, why))

        # Unconditional: skip lists are about whitespace, never about leaks.
        for rule, line in secret_findings(text):
            finds["secret"].append("secret: %s:%d %s" % (path, line, rule))

    return finds, len(files), len(submodules)


def self_test():
    """Every gate must fire on a planted string, or it is a gate in name only (#650's lesson)."""
    bad = []
    # (path, contents, the one hygiene finding it must produce -- None means stay clean)
    for name, text, want in [
        ("src/plant.cpp", "int x = 1;   \n", "trailing whitespace"),
        ("src/plant.cpp", "int x = 1;", "no final newline"),
        ("src/plant.cpp", "a\tb\n", "tab where indentation is spaces"),
        ("src/plant.cpp", "a\r\n", "carriage return (CRLF line ending)"),
        ("Makefile", "target:\n\tcmd\n", None),        # a tab in a recipe is syntax
        ("utils/debian/rules", "build:\n\techo hi\n", None),
    ]:
        got = [why for _, why in hygiene_findings(name, text)]
        if want is None:
            if got:
                bad.append("%s: tab exemption lost, saw %r" % (name, got))
        elif got != [want]:
            bad.append("%s: planted %r -> %r (want exactly that one finding)" % (name, want, got))

    # Secrets: assembled where writing the shape out whole would trip the scan (and the
    # scan runs on this file). Labels need a credential-shaped value half.
    for text, want in [
        ("api" + '_key = "hunter2hunter2"\n', "labelled credential"),
        ("token: \\" + '"supersecretvalue\\"' + "\n", "labelled credential"),   # escaped quotes
        ("-----BEGIN " + "RSA PRIVATE KEY-----\n", "private key block"),
        ("-----BEGIN " + "PGP PRIVATE KEY BLOCK-----\n", "private key block"),
        ("t = ghp_" + "A" * 36 + "\n", "github token shape"),
        ("k = AKIA" + "A" * 16 + "\n", "aws access key id"),
        ("s = xoxb-" + "A" * 24 + "\n", "slack/gitlab token shape"),
        ("GITLA" + "B_TOKEN = glpat-" + "B" * 24 + "\n", "slack/gitlab token shape"),
        ("MY_APP_SECRET " + "= abcdef0123456789abcd\n", "env-style secret assignment"),
        ("export " + "SERVICE_PASSWORD=hunter2hunter2hunter\n", "env-style secret assignment"),
    ]:
        got = [rule for rule, _ in secret_findings(text)]
        if want not in got:
            bad.append("secret gate missed %r (saw %r)" % (want, got))
    # ... and must NOT fire on ordinary code. The last three are lines that exist in this
    # repo, which is how the missed/wolf-ish balance got set: a bare `{12,}` followed by
    # `(?!\()` lets the engine backtrack to exactly 12 characters and calls
    # `token = token.replace(...)` a credential.
    for text in ["passwd = getpwuid(getuid());\n", "cfg.api_key = load_key();\n",
                 "const char *password = prompt();\n", "token: optional field\n",
                 'token = token.replace("\\\\", "")\n',
                 'token = os.environ.get("TEAWRAP_' + 'TOKEN")\n',
                 "SECRET = os.environ.get('WHATEVER')\n",
                 "AWS_KEY_NOTE = not a value, just prose\n"]:
        got = secret_findings(text)
        if got:
            bad.append("secret gate false-positive on %r -> %r" % (text, got))

    # The skip list is hygiene-only. Pinned, because the first draft lost the secret scan
    # for the skipped trees and still printed a green verdict.
    for path, want in [("reports/x/y.md", False), ("utils/rss/z.html", False),
                       ("e2e/fixtures/save_bad_format/save.json", False),
                       ("src/gameui.cpp", True), ("utils/thing.py", True)]:
        if hygiene_applies(path) != want:
            bad.append("hygiene scope wrong for %s" % path)

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

    over, under = MAX_BYTES + 1, MAX_BYTES - 1
    if size_finding("res/plant.bin", over) is None or size_finding("res/plant.bin", under) is not None:
        bad.append("size gate wrong at the cap")

    if bad:
        print("ci_lint SELF-TEST FAILED:")
        for b in bad:
            print("   " + b)
        return 1
    print("ci_lint self-test ok: hygiene, size, json and secret gates each fire on a "
          "planted string; the tab exemptions hold; the secret gate stays quiet on "
          "ordinary code; hygiene scope is hygiene-only")
    return 0


def print_gate(gate, items, cap):
    for f in items[:cap]:
        print(f)
    if len(items) > cap:
        print("%s: ... %d more not shown" % (gate, len(items) - cap))


def main():
    ap = argparse.ArgumentParser(description="No-compiler repo gates (CI runs this).")
    ap.add_argument("--self-test", action="store_true", help="check the gates fire, then exit")
    args = ap.parse_args()
    if args.self_test:
        return self_test()

    # git ls-files is CWD-relative while every gate matches root-relative prefixes, so a
    # run from a subdirectory would scan part of the tree, drop the res/ gates and turn the
    # skip list into noise. Anchor on the toplevel.
    top = subprocess.run(["git", "rev-parse", "--show-toplevel"],
                         check=True, capture_output=True, text=True).stdout.strip()
    if top:
        os.chdir(top)

    finds, nfiles, nsub = run_gates()
    # Hygiene is capped because it is the one gate that can produce hundreds of lines of
    # near-duplicates. Size, json, secret and unread are capped far above what we ever see
    # so the reason a build broke is never cut off -- a global cap once buried the one
    # secret finding behind 30 hygiene lines.
    print_gate("hygiene", finds["hygiene"], MAX_HYGIENE_REPORT)
    for gate in ("size", "json", "secret", "unread"):
        print_gate(gate, finds[gate], MAX_OTHER_REPORT)
    total = sum(len(v) for v in finds.values())
    print("ci_lint: %d tracked files (%d submodules skipped) -- %d hygiene, %d size, "
          "%d json, %d secret, %d unread findings"
          % (nfiles, nsub, len(finds["hygiene"]), len(finds["size"]), len(finds["json"]),
             len(finds["secret"]), len(finds["unread"])))
    return 1 if total else 0


if __name__ == "__main__":
    sys.exit(main())
