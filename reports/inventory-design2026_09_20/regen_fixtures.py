#!/usr/bin/env python3
"""Regenerate the e2e save fixtures after a save-format change.

Usage:  python3 regen_fixtures.py [fresh_racer_save_dir]
        (default: tmp/fixture_regen)

Produce the inputs first, from the repo root:

    xvfb-run -a ./osp --scenario rot-orbit --ship res/ships/racer.json \
        --time-accel 1 --timeout 3 --save tmp/fixture_regen
    xvfb-run -a ./osp --dock-test near --time-accel 1 --timeout 3 \
        --save tmp/fixture_docked

save_racer   <- the racer capture, verbatim (cases 54-save-load, 64-reload)
save_corrupt <- save_racer + a third vessel whose first part names something
                the catalog does not have, which is what makes
                buildShipFromSaveParts throw (case 65-reload-refused)
save_docked  <- the --dock-test capture: ONE vessel of 14 parts, the merged
                probe + station, carrying a dock seam (case 79-dock-save-load)

The corrupt vessel is a copy of v0 with DISTINCT uids (+100), so the only thing
wrong with it is the unknown part name -- a uid collision with v0 would trip
the loader's cross-file duplicate check instead, and the case would be testing
the wrong failure.

The outgoing fixtures are backed up to tmp/fixtures_old/ first.
"""
import json
import os
import shutil
import sys


def repo_root(start):
    """Walk up to the checkout, so this works from reports/ or tmp/ alike."""
    d = os.path.abspath(start)
    while d != "/":
        if os.path.exists(os.path.join(d, "Makefile")) \
           and os.path.isdir(os.path.join(d, "e2e", "cases")):
            return d
        d = os.path.dirname(d)
    raise SystemExit("could not find the repo root from " + os.path.abspath(start))


REPO = repo_root(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(REPO, sys.argv[1] if len(sys.argv) > 1 else "tmp/fixture_regen")
DOCKED = os.path.join(REPO, "tmp", "fixture_docked")
RACER = os.path.join(REPO, "e2e", "fixtures", "save_racer")
CORRUPT = os.path.join(REPO, "e2e", "fixtures", "save_corrupt")
DOCKED_OUT = os.path.join(REPO, "e2e", "fixtures", "save_docked")


def load(path):
    with open(path) as f:
        return json.load(f)


def dump(path, obj):
    with open(path, "w") as f:
        json.dump(obj, f, indent=1, sort_keys=True)
        f.write("\n")


def replace(dst, src):
    if os.path.isdir(dst):
        shutil.rmtree(dst)
    shutil.copytree(src, dst)


if not os.path.isdir(SRC):
    raise SystemExit("no racer capture at " + SRC + " -- see the usage above")

# --- back up the outgoing fixtures so they can be diffed afterwards ---------
BACKUP = os.path.join(REPO, "tmp", "fixtures_old")
if os.path.isdir(BACKUP):
    shutil.rmtree(BACKUP)
os.makedirs(BACKUP)
for name, path in (("save_racer", RACER), ("save_corrupt", CORRUPT),
                   ("save_docked", DOCKED_OUT)):
    if os.path.isdir(path):
        shutil.copytree(path, os.path.join(BACKUP, name))
print("backed up the old fixtures to tmp/fixtures_old/")

# --- save_racer: the fresh capture, verbatim --------------------------------
replace(RACER, SRC)

# --- save_docked: the merged --dock-test capture, verbatim ------------------
if os.path.isdir(DOCKED):
    replace(DOCKED_OUT, DOCKED)
else:
    print("NOTE: no " + os.path.relpath(DOCKED, REPO)
          + ", leaving save_docked as it is")

# --- save_corrupt: save_racer + a broken third vessel -----------------------
replace(CORRUPT, RACER)

v0 = load(os.path.join(CORRUPT, "ships", "v0.json"))
v2 = json.loads(json.dumps(v0))          # deep copy
v2["name"] = "broken"
v2["slot"] = 0
for p in v2["parts"]:
    p["uid"] += 100
    if p.get("parent", 0):
        p["parent"] += 100
if v2.get("controller", 0):
    v2["controller"] += 100
v2["parts"][0]["part"] = "no_such_part_in_the_catalog"
dump(os.path.join(CORRUPT, "ships", "v2.json"), v2)

meta = load(os.path.join(CORRUPT, "save.json"))
meta["ships"] = meta["ships"] + ["v2"]
dump(os.path.join(CORRUPT, "save.json"), meta)

for name in ("save_racer", "save_corrupt", "save_docked"):
    d = os.path.join(REPO, "e2e", "fixtures", name, "ships")
    if os.path.isdir(d):
        print(name + ":", sorted(os.listdir(d)))
print("corrupt v2 uids:", [p["uid"] for p in v2["parts"]],
      "parents:", [p.get("parent", 0) for p in v2["parts"]])
print("corrupt v2 parts[0]:", v2["parts"][0]["part"])
