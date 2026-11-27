#!/usr/bin/env python3
"""Photograph a ship def from the six fitted orthographic views, headless.

The framing lives in the game (vabSetView, src/vab.cpp: the hull's box projected
on the view's own axes, orthographic, UI-independent). This script only drives
keys and names the results: it boots the VAB, hides the HUD (Tab), presses the
view keys (5,6,7,8,9,0), and pairs each "[vab] view NAME" line with the
"Screenshot saved:" line that follows it.

    python3 utils/shipshots.py spaceplane
    python3 utils/shipshots.py racer --views dorsal,right
    python3 utils/shipshots.py racer --portrait      # plan views in a tall window

Files land in tmp/shipshots/<ship>-<view>.png. Intended for iterating on a hull:
read one back and look at what was actually built.
"""
import argparse
import json
import os
import re
import shutil
import subprocess
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# The editor's view keys, in VabView order (src/keys.h).
VIEWS = [("nose", "5"), ("tail", "6"), ("right", "7"),
         ("left", "8"), ("dorsal", "9"), ("belly", "0")]

# The plan views stand the hull's long axis up, so a tall window spends its
# pixels on it: 13 m of airframe fits ~30% bigger portrait than landscape.
LANDSCAPE = ("nose", "tail", "right", "left")

VIEW_RE = re.compile(r"^\[vab\] view (\w+): hull ([0-9. x-]+m), frame half-height ([0-9.]+) m")
SHOT_RE = re.compile(r"^Screenshot saved: (\S+) \((\d+)x(\d+)\)")


def find_def(name):
    """A ship stem must live where the editor's own lookup finds it, because the
    load is driven through the VAB's filter box by NAME, not by path."""
    if name.endswith(".json"):
        name = os.path.basename(name)[:-5]
    for d in (os.path.join(ROOT, "res", "ships"),
              os.path.expanduser("~/.local/share/openspaceprogram/ships")):
        p = os.path.join(d, name + ".json")
        if os.path.isfile(p):
            return name, p
    sys.exit("no ships/%s.json in res/ships or the data dir; put the def there" % name)


def shoot(osp, name, path, views, size, out):
    """One headless run; returns {view: (png, hull, halfH)} for the views asked for."""
    # Entering the VAB from the CLI is the same recipe as e2e/cases/vab: go to the
    # editor (key 4), filter the vehicle list, load, then the views.
    args = [osp, "--startship", "%s,%s,Kerbin,flying-cruise" % (name, os.path.relpath(path, ROOT)),
            "--width", str(size[0]), "--height", str(size[1]),
            "--timeout", str(4.0 + 0.7 * len(views) + 1.0),
            "--sim-press", "800,0,4",
            "--ui-type", "1500,VAB TopBar/##vehfilter,%s" % name,
            "--ui-click", "2000,VAB TopBar/##vabload",
            "--ui-click", "2600,%s" % name,
            "--ui-click", "3200,VAB TopBar/Load",
            "--sim-press", "3800,0,Tab"]
    t = 4000
    keys = dict(VIEWS)
    press = []
    for v in views:
        press.append("%d,0,%s" % (t, keys[v]))
        press.append("%d,0,F12" % (t + 400))
        t += 700
    args += ["--sim-press"] + press

    env = dict(os.environ, SDL_VIDEODRIVER="offscreen", SDL_AUDIODRIVER="dummy")
    run = subprocess.run(args, cwd=ROOT, env=env, capture_output=True, text=True)
    shots, pending = {}, None
    for line in run.stdout.splitlines():
        m = VIEW_RE.match(line)
        if m:
            pending = (m.group(1), m.group(2), float(m.group(3)))
            continue
        m = SHOT_RE.match(line)
        if m and pending:
            shots[pending[0]] = (m.group(1), pending[1], pending[2], (int(m.group(2)), int(m.group(3))))
            pending = None
    os.makedirs(out, exist_ok=True)
    with open(os.path.join(out, "%s-run.log" % name), "w") as f:
        f.write(" ".join(args) + "\n\n" + run.stdout + "\n" + run.stderr)
    for v in views:
        if v not in shots:
            sys.exit("view %s produced no screenshot (exit %d); see %s-run.log"
                     % (v, run.returncode, name))
    for v, (png, hull, halfH, px) in shots.items():
        if v in views:
            shutil.copyfile(png, os.path.join(out, "%s-%s.png" % (name, v)))
            shots[v] = (os.path.join(out, "%s-%s.png" % (name, v)), hull, halfH, px)
    return shots


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("ship", help="ship def name or res/ships path")
    ap.add_argument("--views", default=",".join(v for v, _ in VIEWS),
                    help="comma list of %s" % ",".join(v for v, _ in VIEWS))
    ap.add_argument("--out", default=os.path.join(ROOT, "tmp", "shipshots"))
    ap.add_argument("--size", default="1920x1080", help="window WxH")
    ap.add_argument("--portrait", action="store_true",
                    help="shoot dorsal/belly in a tall window instead of the given one")
    args = ap.parse_args()

    osp = os.path.join(ROOT, "osp")
    if not os.path.isfile(osp):
        sys.exit("build the game first (bear -- make)")
    name, path = find_def(args.ship)
    try:
        json.load(open(path))
    except Exception as e:
        sys.exit("%s is not readable json: %s" % (path, e))

    want = [v.strip() for v in args.views.split(",") if v.strip()]
    for v in want:
        if v not in dict(VIEWS):
            sys.exit("unknown view %r (want %s)" % (v, ",".join(v for v, _ in VIEWS)))
    w, h = (int(x) for x in args.size.lower().split("x"))

    flat = want if not args.portrait else [v for v in want if v in LANDSCAPE]
    tall = [v for v in want if v not in LANDSCAPE]
    shots = {}
    if flat:
        shots.update(shoot(osp, name, path, flat, (w, h), args.out))
    if tall:
        shots.update(shoot(osp, name, path, tall, (h, w), args.out))
    for v, _ in VIEWS:
        if v in shots:
            png, hull, halfH, px = shots[v]
            print("%-7s %-24s frame half-height %6.2f m  %dx%d  %s"
                  % (v, "hull " + hull + ":", halfH, px[0], px[1], png))


if __name__ == "__main__":
    main()
