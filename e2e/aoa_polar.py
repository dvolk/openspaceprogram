#!/usr/bin/env python3
"""Held-angle-of-attack aero polar: sweep elevator TRIM, read the AoA the rows print.

#1211 asked for this before anyone quotes a shaping win ("Cd wallows across AoA 0.1..2.8 deg
and any CdA delta under ~15% is noise"). It exists because two obvious ways of holding an
angle both fail, quietly:

  * Asking `aoa=` on --startship (#1220) sets the SPAWN attitude only. Measured on xr2 at
    250 m/s: asks of -6, 0 and +6 all settle at AoA +3.1 deg, and only the t=1 row differs
    (+2.0 / +2.7 / +3.3). The hull weathervanes to its zero-moment angle inside 2 s, which
    is the correct physics -- but it means the ask is not an input to a polar.
  * Latching kill-rot (X) does NOT freeze the angle either. It damps the rate, and the
    airframe's aero torque at an off-trim alpha is 20-40 kN m on a 4 t hull (0.5 rad/s^2),
    so the attitude still walks to the zero-moment angle: with X latched at 0 ms or 50 ms
    and aoa=-6 asked, t=1 reads +2.0 and t=3 onward reads +3.1.

What does work: the elevator `trim` is what SETS the zero-moment angle, so a hull per trim
value gives a held alpha, and kill-rot then flattens the phugoid's pitch so the reading is
steady. Measured on xr2, trim -20..+20, spawn 250 m/s: alpha -3.5..+3.7 deg, drifting only
0.0-0.1 deg between t=6 and t=12, with |tau| down to 19..460 N m (a tenth of the off-trim
torque) -- i.e. each row really is moment-balanced, not a hull pinned against its own air.
Trimming ailerons and rudders as well is a trap: it adds their deflection drag and moved
this hull's Cd by +0.055 (0.433 -> 0.488) at one point in the sweep, so --trim-kinds defaults
to elevator alone.

With the engine off the hull decelerates, so one run also walks the Mach number while alpha
holds; `--lever` fires the jet (T held, F for (1-lever)/0.6 s) to spend the speed back. The
mean Mach of each row is printed because it is a confound, not a free variable.

Read the bills as the [drag] line means them (tick.cpp, #455): the airframe's form drag is
`--drag-cd x Cd x A + Profile`, with `Induced=` charged on top once. Cd and A are the RAW
hull product and do NOT carry the multiplier; Profile and Induced already do.

Scratch hulls (the shipped def plus one trim value on its pitch surfaces) go in tmp/aoa_polar/.

  python3 e2e/aoa_polar.py --ship res/ships/xr2.json
  python3 e2e/aoa_polar.py --ship res/ships/spaceplane.json --trims=0,6,12,18 --speeds=120
  python3 e2e/aoa_polar.py --ship res/ships/skiff_trim.json --lever 0.4 --csv tmp/polar.csv

(Negative trim values need the --trims=-20,... form: argparse reads a bare token starting
with '-' as an option, not as this flag's value.)
"""

import argparse
import json
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run as e2e  # noqa: E402

REPO = e2e.REPO_ROOT
SCRATCH = os.path.join(REPO, "tmp", "aoa_polar")
DRAG_CD = 1.2  # cli.h's --drag-cd default, which Cd x A does not carry but Profile does.
THROTTLE_RAMP = 0.6  # vehicle.cpp adjustThrottle: 0.6 of the lever per second of R/F.


def scratch_hull(src, trim, kinds):
    """The shipped def with one trim value on every part whose name starts with a
    pitch-surface kind. Returns (path, name, parts_touched)."""
    with open(src) as f:
        d = json.load(f)
    touched = 0
    for p in d["parts"]:
        if any(p["part"].startswith(k) for k in kinds):
            p["trim"] = trim
            touched += 1
    base = d.get("name") or os.path.basename(src)[:-5]
    d["name"] = "%s_t%+d" % (base, trim)
    os.makedirs(SCRATCH, exist_ok=True)
    path = os.path.join(SCRATCH, "%s.json" % d["name"])
    with open(path, "w") as f:
        json.dump(d, f)
    return path, d["name"], touched


def flight(game, path, name, bed, speed, steps, lever, parts=None):
    """One headless run: kill-rot latched at 50 ms, and if a lever is asked for, F walks
    the throttle down to it and T is held PAST the end of the run (Thrust is a held
    command -- a press window that closes mid-flight turns every later row into a glide).
    `parts` swaps the catalogue (--parts), which is how a cd anchor gets A/B'd."""
    args = ["--time-accel", "1", "--sim-steps", str(steps), "--drag-log", "--no-paint"]
    if parts:
        args += ["--parts", parts]
    start = "%s,%s,Kerbin,%s" % (name, path, bed)
    if speed is not None:
        start += ",speed=%g" % speed
    args += ["--startship", start]
    presses = ["50,0,X"]
    if lever is not None:
        ms = int(round((1.0 - lever) / THROTTLE_RAMP * 1000.0))
        presses += ["100,%d,F" % ms, "150,%d,T" % (steps * 100)]
    cmd = e2e.build_cmd(game, args)
    for p in presses:
        cmd += ["--sim-press", p]
    proc = subprocess.run(cmd, cwd=REPO, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=600,
                          env=e2e.launch_env(game))
    rows = e2e.parse_drag(proc.stdout.decode("utf-8", "replace"))
    if not rows:
        sys.stderr.write("aoa_polar: %s exited %d without one [drag] row\n"
                         % (name, proc.returncode))
    return rows


def summarize(rows, t0, t1):
    """Window means, plus the evidence that the angle is actually being held."""
    win = [d for d in rows if t0 <= d["t"] <= t1 and d.get("aoa") is not None]
    if len(win) < 3:
        return None

    def mean(key):
        vals = [d[key] for d in win if d.get(key) is not None]
        return sum(vals) / len(vals) if vals else None

    aoas = [d["aoa"] for d in win]
    out = {"n": len(win), "aoa": mean("aoa"),
           "hold": max(aoas) - min(aoas),
           "mach": mean("mach"), "v": mean("v"), "cd": mean("cd"),
           "a": mean("a"), "profile": mean("profile"),
           "induced": mean("induced"), "L": mean("L"), "ld": mean("ld"),
           "sm": mean("sm"), "tau": mean("tau")}
    if out["cd"] is not None and out["a"] is not None:
        body = DRAG_CD * out["cd"] * out["a"]
        out["body"] = body
        out["total"] = body + (out["profile"] or 0.0) + (out["induced"] or 0.0)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ship", default="res/ships/xr2.json")
    ap.add_argument("--trims", default="-20,-12,-6,0,6,12,12,18",
                    help="elevator trim values to sweep (default -20..18)")
    ap.add_argument("--speeds", default="250",
                    help="spawn airspeeds on the bed, comma separated (default 250)")
    ap.add_argument("--bed", default="flying-cruise")
    ap.add_argument("--steps", type=int, default=900, help="sim steps per run (60/s)")
    ap.add_argument("--window", default="6,12", help="t0,t1 of the averaging window")
    ap.add_argument("--lever", type=float, default=None,
                    help="fly with the jet at this throttle instead of gliding")
    ap.add_argument("--trim-kinds", default="elevator",
                    help="part-name prefixes that get the trim (default: elevator only; "
                         "trimming ailerons or rudders adds form drag and skews the polar)")
    ap.add_argument("--parts", default=None,
                    help="parts catalog for the run (--parts), to A/B a cd anchor")
    ap.add_argument("--label", default="", help="printed under the header, for A/B runs")
    ap.add_argument("--csv", default=None, help="also dump every row to this CSV")
    args = ap.parse_args()

    game = os.path.join(REPO, "osp")
    src = args.ship if os.path.isabs(args.ship) else os.path.join(REPO, args.ship)
    trims = [float(x) for x in args.trims.split(",")]
    speeds = [float(x) for x in args.speeds.split(",")]
    kinds = tuple(k.strip() for k in args.trim_kinds.split(",") if k.strip())
    t0, t1 = (float(x) for x in args.window.split(","))

    print("hull %s   bed %s   window t=%.0f..%.0f s   kill-rot latched at 50 ms   "
          "bill = %.1f x Cd x A + Profile + Induced"
          % (os.path.relpath(src, REPO), args.bed, t0, t1, DRAG_CD))
    print("%6s %7s %7s %5s %6s %7s %7s %6s %8s %8s %8s %8s %7s %7s %8s"
          % ("trim", "AoA", "hold", "n", "Mach", "|v|", "Cd", "A", "body",
             "Profile", "Induced", "TOTAL", "L/D", "SM", "|tau|"))
    if args.csv:
        with open(args.csv, "w") as f:
            f.write("spawn_speed,trim,t,alt_asl,rho,v,mach,aoa,cd,a,body_cdA,"
                    "profile,induced,total_cdA,L,ld,sm,tau\n")
    for sp in speeds:
        for tr in trims:
            path, name, touched = scratch_hull(src, tr, kinds)
            if touched == 0:
                print("%6g no pitch surfaces matched (%s)" % (tr, ",".join(kinds)))
                continue
            rows = flight(game, path, name, args.bed, sp, args.steps, args.lever,
                          args.parts)
            if args.csv:
                with open(args.csv, "a") as f:
                    for d in rows:
                        if d.get("aoa") is None:
                            continue
                        body = DRAG_CD * d["cd"] * d.get("a", 0.0)
                        tot = body + d.get("profile", 0.0) + d.get("induced", 0.0)
                        f.write(",".join("%g" % x if x is not None else "" for x in
                                         (sp, tr, d["t"], d["alt_asl"], d["rho"], d["v"],
                                          d.get("mach"), d["aoa"], d["cd"], d.get("a"),
                                          body, d.get("profile"), d.get("induced"), tot,
                                          d.get("L"), d.get("ld"), d.get("sm"),
                                          d.get("tau"))) + "\n")
            s = summarize(rows, t0, t1)
            if s is None:
                print("%6g no usable rows (%d logged)" % (tr, len(rows)))
                continue
            print("%6g %7.2f %7.2f %5d %6.3f %7.1f %7.3f %6.2f %8.3f %8.3f %8.3f %8.3f"
                  " %7.2f %7.2f %8.0f"
                  % (tr, s["aoa"], s["hold"], s["n"], s["mach"], s["v"], s["cd"], s["a"],
                     s["body"], s["profile"] or 0.0, s["induced"] or 0.0, s["total"],
                     s["ld"] or 0.0, s["sm"] or 0.0, s["tau"]))
        print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
