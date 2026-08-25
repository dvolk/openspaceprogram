#!/usr/bin/env python3
"""E2E battery: launch ./osp under Xvfb and check the result.

Usage:
  python3 e2e/run.py            run all cases
  python3 e2e/run.py orbit      run only cases matching "orbit"
  python3 e2e/run.py smoke 02   run cases matching "smoke" or "02"

Each test is a case file in e2e/cases/*.txt with these keys (one per line,
`#` starts a comment):

  NAME <label>                 shown in the summary
  ARGS <game args>             may span lines; split on whitespace, passed to ./osp
  EXPECT <substring>           must occur in the output (repeat for more)
  FORBID <substring>           must NOT occur in the output (repeat)
  CHECK <python expression>    must be truthy (repeat); see the namespace below
  LIMIT <seconds>              runner hard timeout for this case (default 120)

A case PASSES iff: the process exits 0, every EXPECT is found, no FORBID is
found, and every CHECK is truthy.

CHECK namespace (parsed from the game's stdout):
  out     the full captured output (str)
  orbit   list of dicts, one per [orbitlog] line:
          t, frame, r, v, sma, ecc, peri, apo, inc, T, ttAp, ttPe, h, E
          (apo == -1 on a hyperbolic/escape trajectory)
  dbg     list of dicts, one per [dbg] line: t, pos (3-tuple), alt,
          vel (3-tuple), v
  first / last                 first() / last() of a list
Example:  CHECK last(orbit)["E"] > first(orbit)["E"]

Stdlib only. Run from anywhere; the repo root is derived from this file.
"""

import glob
import os
import re
import shutil
import subprocess
import sys

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CASES_DIR = os.path.join(REPO_ROOT, "e2e", "cases")
DEFAULT_LIMIT = 120.0

ORBIT_RE = re.compile(
    r"\[orbitlog\]\s+t=([\d.]+)s\s+frame=\"([^\"]*)\"\s+r=([-\d.e+]+) m\s+"
    r"v=([-\d.e+]+) m/s\s+sma=([-\d.e+]+) m\s+ecc=([-\d.e+]+)\s+"
    r"peri=([-\d.e+]+) m\s+apo=([-\d.e+]+) m\s+inc=([-\d.e+]+) deg\s+"
    r"T=([-\d.e+]+) s\s+ttAp=([-\d.e+]+) s\s+ttPe=([-\d.e+]+) s\s+"
    r"\|h\|=([-\d.e+]+) m2/s\s+E=([-\d.e+]+) J/kg"
)
DBG_RE = re.compile(
    r"\[dbg\]\s+t=([\d.]+)s\s+pos=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"alt=([-\d.]+) m\s+vel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+\|v\|=([-\d.]+) m/s"
)


def parse_cases(path):
    name = os.path.basename(path)
    args = []
    expect = []
    forbid = []
    check = []
    limit = DEFAULT_LIMIT
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            key, _, rest = line.partition(" ")
            key = key.upper()
            rest = rest.strip()
            if key == "NAME":
                name = rest
            elif key == "ARGS":
                args.extend(rest.split())
            elif key == "EXPECT":
                expect.append(rest)
            elif key == "FORBID":
                forbid.append(rest)
            elif key == "CHECK":
                check.append(rest)
            elif key == "LIMIT":
                limit = float(rest)
            else:
                raise ValueError("%s: unknown key %r" % (os.path.basename(path), key))
    if not args:
        raise ValueError("%s: no ARGS" % os.path.basename(path))
    return {
        "name": name,
        "args": args,
        "expect": expect,
        "forbid": forbid,
        "check": check,
        "limit": limit,
    }


def parse_orbit(out):
    rows = []
    for m in ORBIT_RE.finditer(out):
        (t, frame, r, v, sma, ecc, peri, apo, inc, T, ttAp, ttPe, h, E) = m.groups()
        rows.append({
            "t": float(t), "frame": frame, "r": float(r), "v": float(v),
            "sma": float(sma), "ecc": float(ecc), "peri": float(peri),
            "apo": float(apo), "inc": float(inc), "T": float(T),
            "ttAp": float(ttAp), "ttPe": float(ttPe), "h": float(h),
            "E": float(E),
        })
    return rows


def parse_dbg(out):
    rows = []
    for m in DBG_RE.finditer(out):
        (t, px, py, pz, alt, vx, vy, vz, v) = m.groups()
        rows.append({
            "t": float(t), "pos": (float(px), float(py), float(pz)),
            "alt": float(alt), "vel": (float(vx), float(vy), float(vz)),
            "v": float(v),
        })
    return rows


def first(seq):
    return seq[0]


def last(seq):
    return seq[-1]


def build_cmd(game, args):
    if os.environ.get("DISPLAY") and shutil.which("xvfb-run") is None:
        # A real display is available and no Xvfb to fake one.
        return [game] + args
    xvfb = shutil.which("xvfb-run")
    if xvfb:
        return [xvfb, "-a", game] + args
    # No Xvfb and no display: run bare; it will fail to open a window, which
    # the case will report as a failure. (Headless envs should install Xvfb.)
    return [game] + args


def run_case(case):
    """Return (passed, diagnostics-lines)."""
    game = os.path.join(REPO_ROOT, "osp")
    if not os.path.exists(game):
        return False, ["./osp not found; run `make` (or `make e2e`) first."]
    # Start each case from a clean ImGui layout (window positions persist in
    # imgui.ini otherwise, which would make UI clicks non-deterministic).
    try:
        os.remove(os.path.join(REPO_ROOT, "imgui.ini"))
    except FileNotFoundError:
        pass

    cmd = build_cmd(game, case["args"])
    diag = []
    timed_out = False
    exit_code = None
    out = ""
    try:
        proc = subprocess.run(
            cmd, cwd=REPO_ROOT, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, timeout=case["limit"],
        )
        out = proc.stdout.decode("utf-8", "replace")
        exit_code = proc.returncode
    except subprocess.TimeoutExpired as e:
        timed_out = True
        out = (e.output or b"").decode("utf-8", "replace")

    # 1) exit code
    if timed_out:
        diag.append("runner timeout after %.0fs (LIMIT)" % case["limit"])
    elif exit_code != 0:
        diag.append("exit code %d (expected 0)" % exit_code)

    # 2) EXPECT / FORBID
    missing = [s for s in case["expect"] if s not in out]
    for s in missing:
        diag.append("EXPECT not found: %r" % s)
    hit = [s for s in case["forbid"] if s in out]
    for s in hit:
        diag.append("FORBID found: %r" % s)

    # 3) CHECK
    orbit = parse_orbit(out)
    dbg = parse_dbg(out)
    ns = {
        "out": out, "orbit": orbit, "dbg": dbg,
        "first": first, "last": last,
        "abs": abs, "len": len, "any": any, "all": all,
        "max": max, "min": min, "float": float, "int": int,
    }
    for expr in case["check"]:
        try:
            ok = bool(eval(expr, {"__builtins__": {}}, ns))  # noqa: S307 - trusted case files
        except Exception as e:
            diag.append("CHECK raised: %r (%s)" % (expr, e))
            continue
        if not ok:
            diag.append("CHECK failed: %s" % expr)

    passed = (not diag)
    return passed, diag


def select_cases(case_files, selectors):
    """Filter case_files down to those matching any selector.

    A selector matches a case if it is a (case-insensitive) substring of the
    case NAME or of the filename without its .txt extension -- so `smoke`,
    `01-smoke` and `orbit` (-> orbit-burn) all work. No selectors = all cases.
    """
    if not selectors:
        return case_files
    sel = [s.lower() for s in selectors]
    out = []
    for path in case_files:
        base = os.path.splitext(os.path.basename(path))[0].lower()
        try:
            name = parse_cases(path)["name"].lower()
        except ValueError:
            name = ""
        if any(s in name or s in base for s in sel):
            out.append(path)
    return out


def available_names(case_files):
    names = []
    for path in case_files:
        try:
            names.append(parse_cases(path)["name"])
        except ValueError:
            names.append(os.path.basename(path))
    return names


def main():
    argv = sys.argv[1:]
    if "--help" in argv or "-h" in argv:
        print(__doc__)
        print("usage: run.py [CASE ...]   (run all cases when none given)")
        return 0
    selectors = argv

    all_files = sorted(glob.glob(os.path.join(CASES_DIR, "*.txt")))
    if not all_files:
        print("no case files found in %s" % CASES_DIR)
        return 1

    case_files = select_cases(all_files, selectors)
    if not case_files:
        print("no case matches %r" % (selectors,))
        print("available: %s" % ", ".join(available_names(all_files)))
        return 1

    results = []
    for path in case_files:
        try:
            case = parse_cases(path)
        except ValueError as e:
            results.append((os.path.basename(path), False, ["bad case file: %s" % e]))
            continue
        passed, diag = run_case(case)
        results.append((case["name"], passed, diag))

    width = max(len(n) for n, _, _ in results)
    fails = 0
    for name, passed, diag in results:
        status = "PASS" if passed else "FAIL"
        if not passed:
            fails += 1
        print("%-*s  %s" % (width, name, status))
        for line in diag:
            print("        %s" % line)

    total = len(results)
    print("-" * (width + 8))
    print("%d/%d passed" % (total - fails, total))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main())
