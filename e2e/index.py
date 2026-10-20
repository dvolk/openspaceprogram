#!/usr/bin/env python3
"""Generate e2e/INDEX.md from the cases' NAME/CLAIM/KIND headers.

The index is the one page to read before the 286 case files: it says what each
case pins and what kind of evidence it is. `--check` fails when the committed
index is stale (run it after adding or editing a case).
"""
import argparse
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run  # noqa: E402

INDEX = os.path.join(HERE, "INDEX.md")
SUBJECTS = ["flight", "crew", "science", "vab", "save", "ui", "render", "harness"]
KIND_ORDER = ["invariant", "golden", "harness", "canary"]
KIND_TEXT = {
    "invariant": "a behaviour or a player route the game must keep",
    "golden": "tuned numbers or a content table -- update deliberately when tuning",
    "harness": "CLI/test-engine plumbing, not game spec",
    "canary": "only proves the thing boots",
}


def collect():
    files = sorted(run.glob.glob(os.path.join(run.CASES_DIR, "**", "*.txt"),
                                 recursive=True))
    rows = []
    for path in files:
        case = run.parse_cases(path)
        rel = os.path.splitext(os.path.relpath(path, run.CASES_DIR))[0]
        subject, _, stem = rel.partition(os.sep)
        rows.append({
            "subject": subject,
            "path": rel + ".txt",
            "stem": stem,
            "number": int(stem.split("-", 1)[0]),
            "name": case["name"],
            "kind": case["kind"],
            "claim": case["claim"],
        })
    rows.sort(key=lambda r: (SUBJECTS.index(r["subject"])
                             if r["subject"] in SUBJECTS else 99,
                             r["number"], r["stem"]))
    return rows


def render(rows):
    from collections import Counter
    subjects = Counter(r["subject"] for r in rows)
    kinds = Counter(r["kind"] for r in rows)
    out = [
        "# E2E index",
        "",
        "Generated from each case's `CLAIM` and `KIND` by `python3 e2e/index.py`.",
        "Read this before the case files: a case pins the thing in its claim, and",
        "kind says how much weight the assertions carry.",
        "",
        "| kind | meaning | cases |",
        "|---|---|---|",
    ]
    for k in KIND_ORDER:
        out.append("| `%s` | %s | %d |" % (k, KIND_TEXT[k], kinds.get(k, 0)))
    out += [
        "| | **total** | **%d** |" % len(rows),
        "",
        "There is no full battery: the runner takes at most 5 cases per",
        "invocation and starts a 10-minute cooldown. Select by number, name or",
        "path (a whole subject is usually too wide), e.g.",
        "`python3 e2e/run.py save/172 262`.",
        "",
    ]
    for subject in SUBJECTS:
        group = [r for r in rows if r["subject"] == subject]
        if not group:
            continue
        out += [
            "## %s (%d)" % (subject, len(group)),
            "",
            "| case | kind | pins |",
            "|---|---|---|",
        ]
        for r in group:
            out.append("| `%s` | %s | %s |"
                       % (r["path"], r["kind"], r["claim"].replace("|", "\\|")))
        out.append("")
    return "\n".join(out).rstrip() + "\n"


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true",
                    help="fail if INDEX.md is not what the cases say")
    ap.add_argument("--stdout", action="store_true",
                    help="print instead of writing")
    args = ap.parse_args()
    text = render(collect())
    if args.stdout:
        sys.stdout.write(text)
        return 0
    if args.check:
        try:
            old = open(INDEX).read()
        except FileNotFoundError:
            old = ""
        if old != text:
            print("e2e/INDEX.md is stale; run: python3 e2e/index.py")
            return 1
        print("e2e/INDEX.md is current")
        return 0
    with open(INDEX, "w") as f:
        f.write(text)
    print("wrote %s (%d cases)" % (os.path.relpath(INDEX, run.REPO_ROOT),
                                   len(collect())))
    return 0


if __name__ == "__main__":
    sys.exit(main())
