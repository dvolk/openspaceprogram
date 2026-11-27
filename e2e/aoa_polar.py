#!/usr/bin/env python3
"""Held-angle-of-attack aero polar: sweep elevator TRIM and read the angle back.

#1211 asked for this before anyone quotes a shaping win ("Cd wallows across AoA 0.1..2.8 deg
and any CdA delta under ~15% is noise").

WHAT HOLDS AN ANGLE: the elevator `trim`, because trim is what SETS the hull's zero-moment
angle -- and a hull at its zero-moment angle stays there on its own. No key input is needed
and none is sent. Measured on xr2 at 250 m/s, window t=6..12 s: no press at all and a 0 ms
`--sim-press 50,0,X` print the SAME rows (AoA 3.10, hold 0.00, Cd 0.444, |tau| 394), and the
trim sweep -20..+18 gives held AoA -3.5..+3.7 deg drifting 0.0-0.1 deg across the window with
|tau| of 19-460 N m against the 20-40 kN m an off-trim angle produces. Each row is a moment-
balanced flight condition, not a hull pinned against its own air.

THE TRAPS THIS TOOL WAS BUILT AROUND, all of them silent:

  * `--sim-press START,0,X` does nothing. A zero duration is one keypress, and Slot::KillRot
    is a HOLD override that is re-armed every tick and cleared by tick.cpp's clearRotCmd(),
    so one frame of it changes nothing. (Writing that press is what made me report "kill-rot
    damps the rate but the torque still wins", which was never measured. `--kill-rot` here
    presses X for the whole run.)
  * Holding X for real pins the ATTITUDE, and then AoA wanders: at 250 m/s with the elevator
    at trim 12 and kill-rot held all run, AoA reads 1.30..2.10 across one window -- the
    attitude is fixed while the flight path curves under it. That is worse for a polar, which
    is why --kill-rot defaults off and the hold comes from trim.
  * Asking `aoa=` on --startship sets the SPAWN attitude only: asks of -6, 0 and +6 all
    settle at the hull's own trimmed angle inside 2 s (res/ships/README.md recipe item 3,
    #569). An asked angle is not a polar input.
  * Trimming ailerons and rudders as well as elevators adds their deflection drag and moved
    this hull's Cd by +0.055 (0.433 -> 0.488), so --trim-kinds defaults to elevator alone.
  * `--trims=-20,...` needs the equals form: argparse reads a bare token starting with '-'
    as an option, not as this flag's value.

With the engine off the hull decelerates, so one run also walks Mach while the angle holds --
the window's mean Mach is printed because it is a confound, not a free variable. `--lever`
fires the jet (F for (1-lever)/0.6 s, then T held PAST the end of the run: Thrust is a held
command and a press window that closes mid-flight turns every later row into a glide).

READ THE BILLS as the [drag] line means them (tick.cpp, #455/#947): the airframe's along-flow
form drag is `--drag-cd x Cd x A + Profile`, with `Induced=` charged on top once. Cd and A are
the RAW hull product and do not carry the multiplier; Profile and Induced already do (checked
against vehicle.cpp's lastSurfCdA and lastInducedCdA). `SideCdA=` is printed as its own column
and NOT folded into TOTAL: it is #947's crossflow NORMAL force, a flank force, not part of the
along-flow bill -- on this hull it runs 1.33 m2 at the trim angle, bigger than Profile, which
is exactly the term #1237 argues overlaps the frontal one.

Scratch hulls (the shipped def plus one trim value on its pitch surfaces) and a scratch
--data-dir (so a user's settings.json keybinds and SAS policy cannot change what a row
measures, the way e2e/run.py isolates each case) live under tmp/aoa_polar/.

  python3 e2e/aoa_polar.py --ship res/ships/xr2.json
  python3 e2e/aoa_polar.py --ship res/ships/spaceplane.json --trims=0,6,12,18 --speeds=140
  python3 e2e/aoa_polar.py --ship res/ships/skiff_trim.json --trims=12 --speeds=85 \
          --steps 24000 --window 60,400 --csv tmp/skiff.csv     # a settled glide's L/D
  python3 e2e/aoa_polar.py --ship res/ships/xr2.json --trims=12 --lever 0.4 --steps 4800
"""

import argparse
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run as e2e  # noqa: E402

REPO = e2e.REPO_ROOT
SCRATCH = os.path.join(REPO, "tmp", "aoa_polar")
DATA = os.path.join(SCRATCH, "data")
DRAG_CD = 1.2  # cli.h's --drag-cd default: Cd x A does not carry it, Profile and Induced do.
THROTTLE_RAMP = 0.6  # vehicle.cpp adjustThrottle: 0.6 of the lever per second of R/F.


def scratch_hull(src, trim, kinds, base):
    """The shipped def with one trim value on every part whose name starts with a
    pitch-surface kind. Returns (path, name, parts_touched)."""
    with open(src) as f:
        d = json.load(f)
    touched = 0
    for p in d["parts"]:
        if any(p["part"].startswith(k) for k in kinds):
            p["trim"] = trim
            touched += 1
    d["name"] = "%s_t%g" % (base, trim)  # %g, not %d: 0.4 and 0.6 must not share a file
    path = os.path.join(SCRATCH, "%s.json" % d["name"])
    with open(path, "w") as f:
        json.dump(d, f)
    return path, d["name"], touched


def fresh_data_dir():
    """A clean --data-dir, the way e2e/run.py gives every case one (run.py:2665-2678). The
    real per-user dir carries keybinds and the SAS policy, either of which silently changes
    what a polar row measures -- and a polar runs the game dozens of times in a row."""
    os.makedirs(DATA, exist_ok=True)
    for junk in ("settings.json", "saves", "ships", "flights"):
        p = os.path.join(DATA, junk)
        if os.path.isdir(p):
            shutil.rmtree(p, ignore_errors=True)
        elif os.path.exists(p):
            os.remove(p)
    return DATA


def flight(game, path, name, bed, speed, steps, lever, parts=None, kill_rot=False):
    """One headless run of one scratch hull at one trim."""
    args = ["--time-accel", "1", "--sim-steps", str(steps), "--drag-log", "--no-paint"]
    if parts:
        args += ["--parts", parts]
    start = "%s,%s,Kerbin,%s" % (name, path, bed)
    if speed is not None:
        start += ",speed=%g" % speed
    args += ["--startship", start]
    presses = []
    if kill_rot:
        presses.append("50,%d,X" % (steps * 100))  # a real hold; 0 duration does nothing
    if lever is not None:
        ms = int(round((1.0 - lever) / THROTTLE_RAMP * 1000.0))
        presses += ["100,%d,F" % ms, "150,%d,T" % (steps * 100)]
    cmd = e2e.build_cmd(game, args + ["--data-dir", DATA])
    for p in presses:
        cmd += ["--sim-press", p]
    proc = subprocess.run(cmd, cwd=REPO, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=900,
                          env=e2e.launch_env(game))
    rows = e2e.parse_drag(proc.stdout.decode("utf-8", "replace"))
    if not rows:
        sys.stderr.write("aoa_polar: %s exited %d without one [drag] row -- the bed refused "
                         "the spawn, or the hull left the air before the first 1 s gate\n"
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
    out = {"n": len(win), "aoa": mean("aoa"), "hold": max(aoas) - min(aoas),
           "mach": mean("mach"), "v": mean("v"), "cd": mean("cd"), "a": mean("a"),
           "profile": mean("profile"), "induced": mean("induced"),
           "side": mean("sidecda"), "L": mean("L"), "ld": mean("ld"),
           "sm": mean("sm"), "tau": mean("tau")}
    if out["cd"] is not None and out["a"] is not None:
        body = DRAG_CD * out["cd"] * out["a"]
        out["body"] = body
        # Along-flow only: SideCdA (#947) is a flank normal force, not part of this sum.
        out["total"] = body + (out["profile"] or 0.0) + (out["induced"] or 0.0)
    return out


def part_families(ship_path):
    """Distinct part-name prefixes, for the 'nothing matched' message: a hull steered by
    rudders or by body lift (liftbody.json) has no elevator to trim, and saying so without
    naming what IS there just sends the reader back to the JSON. The prefix split assumes
    the `_r<radius>` naming convention; it is a hint in an error message, not a key."""
    with open(ship_path) as f:
        d = json.load(f)
    return ", ".join(sorted({p["part"].split("_r")[0] for p in d["parts"]}))


def cell(v, spec):
    return spec % v if v is not None else "-"


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ship", default="res/ships/xr2.json")
    ap.add_argument("--trims", default="-20,-12,-6,0,6,12,18",
                    help="trim values to sweep (default -20..18)")
    ap.add_argument("--speeds", default="250",
                    help="spawn airspeeds on the bed, comma separated (default 250)")
    ap.add_argument("--bed", default="flying-cruise")
    ap.add_argument("--steps", type=int, default=900, help="sim steps per run (60 per s)")
    ap.add_argument("--window", default="6,12", help="t0,t1 of the averaging window")
    ap.add_argument("--lever", type=float, default=None,
                    help="fly with the jet at this throttle instead of gliding")
    ap.add_argument("--kill-rot", action="store_true",
                    help="hold X all run (pins the ATTITUDE; AoA then drifts with the path)")
    ap.add_argument("--trim-kinds", default="elevator",
                    help="part-name prefixes that get the trim (default: elevator only; "
                         "trimming ailerons or rudders adds form drag and skews the polar)")
    ap.add_argument("--parts", default=None,
                    help="parts catalog for the run (--parts), to A/B a cd anchor")
    ap.add_argument("--csv", default=None, help="also dump every row to this CSV")
    args = ap.parse_args()

    game = os.path.join(REPO, "osp")
    if not os.path.exists(game):
        sys.exit("aoa_polar: no %s -- build it (bear -- make) first" % os.path.relpath(game, REPO))
    src = args.ship if os.path.isabs(args.ship) else os.path.join(REPO, args.ship)
    if not os.path.exists(src):
        sys.exit("aoa_polar: no hull at %s" % src)
    if args.lever is not None and not 0.0 <= args.lever < 1.0:
        sys.exit("aoa_polar: --lever takes 0..1, got %g" % args.lever)
    trims = [float(x) for x in args.trims.split(",") if x.strip() != ""]
    if len(set(trims)) != len(trims):
        sys.exit("aoa_polar: --trims repeats a value, so two runs would share one scratch "
                 "hull: %s" % args.trims)
    speeds = [float(x) for x in args.speeds.split(",") if x.strip() != ""]
    kinds = tuple(k.strip() for k in args.trim_kinds.split(",") if k.strip())
    t0, t1 = (float(x) for x in args.window.split(","))
    with open(src) as f:
        hull = json.load(f)
    if not any(any(p["part"].startswith(k) for k in kinds) for p in hull["parts"]):
        sys.exit("aoa_polar: no part in %s matches --trim-kinds (%s); this hull is built "
                 "from: %s" % (os.path.relpath(src, REPO), ",".join(kinds),
                               part_families(src)))
    base = hull.get("name") or os.path.basename(src)[:-5]
    os.makedirs(SCRATCH, exist_ok=True)
    DATA = fresh_data_dir()

    print("hull %s   bed %s   window t=%.0f..%.0f s   %s   trim on %s   "
          "along-flow bill = %.1f x Cd x A + Profile + Induced"
          % (os.path.relpath(src, REPO), args.bed, t0, t1,
             "kill-rot held" if args.kill_rot else "no key input (trim holds the angle)",
             ",".join(kinds), DRAG_CD))
    hdr = ("%6s %7s %7s %5s %6s %7s %7s %6s %8s %8s %8s %8s %8s %7s %7s %8s"
           % ("trim", "AoA", "hold", "n", "Mach", "|v|", "Cd", "A", "body", "Profile",
              "SideCdA", "Induced", "TOTAL", "L/D", "SM", "|tau|"))
    print(hdr)
    if args.csv:
        with open(args.csv, "w") as f:
            f.write("spawn_speed,trim,t,alt_asl,rho,v,mach,aoa,cd,a,body_cdA,profile,"
                    "sidecda,induced,total_cdA,L,ld,sm,tau\n")
    measured = 0
    for sp in speeds:
        for tr in trims:
            path, name, _ = scratch_hull(src, tr, kinds, base)
            rows = flight(game, path, name, args.bed, sp, args.steps, args.lever,
                          args.parts, args.kill_rot)
            if args.csv:
                with open(args.csv, "a") as f:
                    for d in rows:
                        if d.get("aoa") is None or d.get("a") is None:
                            continue
                        body = DRAG_CD * d["cd"] * d["a"]
                        tot = body + d.get("profile", 0.0) + d.get("induced", 0.0)
                        f.write(",".join("%g" % x if x is not None else "" for x in
                                         (sp, tr, d["t"], d["alt_asl"], d["rho"], d["v"],
                                          d.get("mach"), d["aoa"], d["cd"], d["a"], body,
                                          d.get("profile"), d.get("sidecda"),
                                          d.get("induced"), tot, d.get("L"), d.get("ld"),
                                          d.get("sm"), d.get("tau"))) + "\n")
            s = summarize(rows, t0, t1)
            if s is None:
                print("%6g no usable rows (%d logged)" % (tr, len(rows)))
                continue
            measured += 1
            print("%6g %7s %7s %5d %6s %7s %7s %6s %8s %8s %8s %8s %8s %7s %7s %8s"
                  % (tr, cell(s["aoa"], "%.2f"), cell(s["hold"], "%.2f"), s["n"],
                     cell(s["mach"], "%.3f"), cell(s["v"], "%.1f"), cell(s["cd"], "%.3f"),
                     cell(s["a"], "%.2f"), cell(s.get("body"), "%.3f"),
                     cell(s["profile"], "%.3f"), cell(s["side"], "%.3f"),
                     cell(s["induced"], "%.3f"), cell(s.get("total"), "%.3f"),
                     cell(s["ld"], "%.2f"), cell(s["sm"], "%.2f"), cell(s["tau"], "%.0f")))
        print()
    if measured == 0:
        sys.stderr.write("aoa_polar: %d runs, none usable -- nothing was measured\n"
                         % (len(trims) * len(speeds)))
        sys.exit(1)
