#!/usr/bin/env python3
"""E2E battery: launch ./osp under Xvfb and check the result.

Usage:
  python3 e2e/run.py            run all cases
  python3 e2e/run.py orbit      run only cases matching "orbit"
  python3 e2e/run.py smoke 02   run cases matching "smoke" or "02"
  python3 e2e/run.py --jobs 4   run up to 4 cases in parallel
                                (default: 2; --jobs 1 = serial)

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
  att     list of dicts, one per [attlog] line: t, nose (3-tuple),
          w (3-tuple), wnorm (|w|), wroll (the nose-axis component of w),
          awroll (abs of wroll)
  eva     list of dicts, one per [evalog] line: t, mode ("ground"/"space"),
          grounded (0/1), pos (3-tuple), vel (3-tuple), alt (m above the
          analytic terrain), mass (kg; None if the binary predates the field)
  first / last                 first() / last() of a list
  re      the stdlib `re` module (regex checks against `out`)
Example:  CHECK last(orbit)["E"] > first(orbit)["E"]

Stdlib only. Run from anywhere; the repo root is derived from this file.
"""

import argparse
import glob
import os
import re
import shutil
import subprocess
import sys
from concurrent.futures import ThreadPoolExecutor

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CASES_DIR = os.path.join(REPO_ROOT, "e2e", "cases")
DEFAULT_LIMIT = 120.0
DEFAULT_JOBS = 2

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
XFER_RE = re.compile(
    r"\[xferlog\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+"
    r"dv_dep=([-\d.e+]+) m/s\s+dv_cap=([-\d.e+]+) m/s\s+total=([-\d.e+]+) m/s\s+"
    r"tof=([-\d.e+]+) s\s+v_inf=([-\d.e+]+) m/s\s+r_cap=([-\d.e+]+) m\s+"
    r"burn=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]"
)
PORKCHOP_RE = re.compile(
    r"\[porkchop\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+"
    r"(\d+)x(\d+)\s+dv_min=([-\d.e+]+) m/s\s+dv_hi=([-\d.e+]+) m/s\s+"
    r"t_dep_min=([-\d.e+]+) s\s+tof_min=([-\d.e+]+) s"
)
SURFMAP_RE = re.compile(
    r"\[surfmap\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+"
    r"(\d+)x(\d+)\s+albedo=\[([\d.]+) ([\d.]+) ([\d.]+)\]\s+"
    r"shaded=\[([\d.]+) ([\d.]+) ([\d.]+)\]\s+shade=(on|off)"
)
ATT_RE = re.compile(
    r"\[attlog\]\s+t=([\d.]+)s\s+"
    r"nose=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"w=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"\|w\|=([-\d.]+) rad/s"
)
EVA_RE = re.compile(
    r"\[evalog\]\s+t=([\d.]+)s\s+mode=(\w+)\s+grounded=(\d)\s+"
    r"pos=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"vel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+alt=([-\d.]+) m"
    r"(?:\s+mass=([-\d.]+)kg)?"
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


def parse_xfer(out):
    rows = []
    for m in XFER_RE.finditer(out):
        (t, target, dv_dep, dv_cap, total, tof, v_inf, r_cap,
         bx, by, bz) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            "dv_dep": float(dv_dep), "dv_cap": float(dv_cap),
            "total": float(total), "tof": float(tof),
            "v_inf": float(v_inf), "r_cap": float(r_cap),
            "burn": (float(bx), float(by), float(bz)),
        })
    return rows


def parse_porkchop(out):
    rows = []
    for m in PORKCHOP_RE.finditer(out):
        (t, target, n_dep, n_tof, dv_min, dv_hi, t_dep_min, tof_min) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            "n_dep": int(n_dep), "n_tof": int(n_tof),
            "dv_min": float(dv_min), "dv_hi": float(dv_hi),
            "t_dep_min": float(t_dep_min), "tof_min": float(tof_min),
        })
    return rows


def parse_surfmap(out):
    rows = []
    for m in SURFMAP_RE.finditer(out):
        (t, body, w, h, ar, ag, ab, sr, sg, sb, shade) = m.groups()
        rows.append({
            "t": float(t), "body": body,
            "w": int(w), "h": int(h),
            "albedo": (float(ar), float(ag), float(ab)),
            "shaded": (float(sr), float(sg), float(sb)),
            "shade": shade,
        })
    return rows


def parse_att(out):
    rows = []
    for m in ATT_RE.finditer(out):
        (t, nx, ny, nz, wx, wy, wz, wnorm) = m.groups()
        nose = (float(nx), float(ny), float(nz))
        w = (float(wx), float(wy), float(wz))
        wroll = w[0]*nose[0] + w[1]*nose[1] + w[2]*nose[2]
        rows.append({
            "t": float(t), "nose": nose, "w": w,
            "wnorm": float(wnorm),
            # The nose-axis (roll) component of the angular velocity: ~0
            # during a pure pitch (spin is perpendicular to the nose), grows
            # once a roll is added -- the coupling the attitude-physics case
            # asserts on. awroll is pre-computed (abs) so a CHECK can use it
            # inside a generator without a free var in the body (an eval
            # gotcha: free vars in a generator body resolve against globals,
            # and the CHECK namespace is passed as locals).
            "wroll": wroll,
            "awroll": abs(wroll),
        })
    return rows


def parse_eva(out):
    rows = []
    for m in EVA_RE.finditer(out):
        (t, mode, grounded, px, py, pz, vx, vy, vz, alt, mass) = m.groups()
        rows.append({
            "t": float(t), "mode": mode, "grounded": int(grounded),
            "pos": (float(px), float(py), float(pz)),
            "vel": (float(vx), float(vy), float(vz)),
            "alt": float(alt),
            "mass": float(mass) if mass is not None else None,
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
    xfer = parse_xfer(out)
    porkchop = parse_porkchop(out)
    surfmap = parse_surfmap(out)
    att = parse_att(out)
    eva = parse_eva(out)
    ns = {
        "out": out, "orbit": orbit, "dbg": dbg, "xfer": xfer,
        "porkchop": porkchop, "surfmap": surfmap, "att": att, "eva": eva,
        "first": first, "last": last,
        "abs": abs, "len": len, "any": any, "all": all,
        "max": max, "min": min, "float": float, "int": int, "zip": zip,
        "re": re,
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


def run_one(path):
    """Parse and run one case; return (name, passed, diag). Never raises."""
    try:
        case = parse_cases(path)
    except ValueError as e:
        return os.path.basename(path), False, ["bad case file: %s" % e]
    try:
        passed, diag = run_case(case)
    except Exception as e:
        return case["name"], False, ["runner error: %r" % e]
    return case["name"], passed, diag


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("selectors", nargs="*",
                        help="case name/filename substring(s) to run; "
                             "default: all cases")
    parser.add_argument("--jobs", type=int, default=DEFAULT_JOBS,
                        help="max cases to run in parallel (1 = serial; "
                             "default: 2)")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be >= 1")
    selectors = args.selectors

    all_files = sorted(glob.glob(os.path.join(CASES_DIR, "*.txt")))
    if not all_files:
        print("no case files found in %s" % CASES_DIR)
        return 1

    case_files = select_cases(all_files, selectors)
    if not case_files:
        print("no case matches %r" % (selectors,))
        print("available: %s" % ", ".join(available_names(all_files)))
        return 1

    # Cases are independent: each gets its own Xvfb display (xvfb-run -a
    # retries on a taken display) and captures its own stdout, so they can
    # run concurrently. map() preserves input order, so the summary prints
    # in case-file order regardless of which case finishes first.
    if args.jobs == 1:
        results = [run_one(p) for p in case_files]
    else:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            results = list(pool.map(run_one, case_files))

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
