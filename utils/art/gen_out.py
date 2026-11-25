#!/usr/bin/env python3
"""The write guard for generators that name their own output inside the repo (#1051).

gen_gear.py, gen_kerbal.py, gen_platform.py and gen_station.py used to write straight into
res/ with no way to point them elsewhere, so an exploratory run -- `python3 utils/art/gen_gear.py
--help`, which has no argparse and simply runs -- rewrote a shipped mesh, and no gate in the
suite would have noticed. They now all take --out ROOT (write under that tree rather than
this repo's) and refuse to replace a file that already exists unless --force says they meant
to. The nine generators that take an explicit destination path are unchanged: naming a
shipped path is already the intent, so a guard there would only teach people to pass --force.

utils/art/check_mesh_pins.py is what --out exists for: it regenerates each pinned asset into a
temp tree and weighs the bytes against the committed one.
"""
import os

# One spelling of the repo root for the generators in this directory: utils/art sits two
# levels below it. Seven copies of that line is seven to move the next time the tree
# changes, so they read it from here instead.
REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


def add_args(ap, repo_root=REPO_ROOT):
    """Give a generator --out/--force. `repo_root` defaults to the repo this file
    lives in; pass one to write under a different tree."""
    ap.add_argument("--out", metavar="ROOT", default=repo_root,
                    help="write under this tree's res/ instead of the repo's")
    ap.add_argument("--force", action="store_true",
                    help="replace the file if it already exists")


def ship(out_root, *parts, force=False):
    """The path <out_root>/res/<parts>, made room for, refusing to clobber without --force."""
    path = os.path.join(out_root, "res", *parts)
    if os.path.exists(path) and not force:
        raise SystemExit(
            "refusing to overwrite %s\n"
            "  pass --force to replace the shipped asset, or --out <temp tree> to generate "
            "a copy (which is what utils/art/check_mesh_pins.py does)" % path)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    return path
