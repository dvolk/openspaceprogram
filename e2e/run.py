#!/usr/bin/env python3
"""E2E runner: run selected e2e/cases/<subject>/*.txt against the game binary.

Usage: python3 e2e/run.py <selectors> [--jobs N] [--game PATH] [--limit S]
[--show-cmd] [--kind K]. No selectors runs the whole battery -- that is what
`make release` does. Nothing caps or paces a run: how much e2e is worth running
is the caller's judgement, not the runner's policy. --limit overrides
every case's LIMIT (a slow box, or a case you want to drive by hand);
--show-cmd prints the exact command + env so a failure re-runs under a
debugger; --kind runs one kind of case (repeatable). Case keys: NAME CLAIM
KIND NOGLCHECK ARGS EXPECT FORBID
CHECK LIMIT WRITE SEED FILE NOFILE EXIT. CLAIM is one sentence: what the case
pins. KIND says what sort of evidence it is -- invariant (behaviour or a
player route), golden (tuned numbers or content tables: update deliberately),
harness (CLI/test plumbing, not game spec) or canary (proves it boots). Both
are required; `python3 e2e/index.py` turns them into e2e/INDEX.md. Every
`e2e/...` or `res/...` path a case names -- in ARGS, SEED, or the text of an
EXPECT/FORBID/CHECK -- must exist in the repo, so a fixture that moved fails
with its own path named rather than as a missing assertion; a case that
deliberately pins a refusal of an absent path opts out with a
`# missing-ok: <path>` comment line. GL errors
and crash strings are forbidden by default, so a case names only its own
refusals; NOGLCHECK opts out of the GL drains for timing-sensitive cases.
CHECK sees out and
the parsed logs (orbit/dbg/att/eva/fuel/drainlog/power/dv/mass/drag/shake/cam/
terrain/surf/xfer/porkchop/surfmap/mapb/plane/slew/toast/scene) plus
first/last/re. The row lists parse stdout from the LAST "Loaded game from"
line: a successful load
starts a new world, and rows the previous world printed are not evidence about
the one under test (toast/scene stay whole-run -- they describe the run).
The load-through-the-UI shape most save/load cases now take: Game Debug ticks
the e2e fixture root, the sheet picks the "fixtures" game, the slot filter
narrows the list to one row, and the row + Load button do the rest. The load
lands at 4s, so a case's own scripted events shift +5000ms and the clock
resumes on the warp ladder (PERIOD presses) -- a load starts paused, where
--time-accel used to hand a boot load its warp. e2e/convert_load_to_ui.py
generates that preamble. ARGS tokenizes like a
shell, so a value may be quoted to carry spaces. SEED <fixture> <path-under-
data-dir> [AGE_DAYS] stages an existing save into the case's scratch --data-dir.
FILE <path-under-data-dir> [substring...] and NOFILE <path-under-data-dir>
assert on what the GAME wrote there (#271); both take globs, since a game dir is
stamped YYYYMMDD_HHMMSS-<name>. EXIT
<code> is the exit code the case demands (default 0) -- without it a game that
correctly refuses a bad boot flag looks identical to one that crashed.
Artifacts land in tmp/e2e/runs/<stamp>/ + history.csv.
"""

import argparse
import csv
import datetime
import glob
import json
import os
import re
import shlex
import shutil
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CASES_DIR = os.path.join(REPO_ROOT, "e2e", "cases")
# --game wins, else the legacy ./osp
GAME = None
DEFAULT_LIMIT = 120.0
DEFAULT_JOBS = 2
# --limit: run every case under this wall-clock ceiling instead of its own
# LIMIT (raise it to drive one case by hand on a slow box; the case's own
# --timeout still decides when the game exits). --show-cmd: print the exact
# command line + env before each case, so a failure can be re-run under
# gdb/valgrind without reconstructing the invocation by hand.
LIMIT = None
SHOW_CMD = False
# Every case forbids these. A GL error, an abort and a shader failure are never
# an expected outcome, and repeating the same six lines in every case file
# buried the assertions a case actually makes. "error:" is NOT here: a clean
# boot refusal prints it on purpose (177, 190, 191).
DEFAULT_FORBID = [
    "GL_",
    "Segmentation",
    "Assertion",
    "terminate called",
    "Error compiling shader",
    "Error linking shader program",
]
KINDS = ("invariant", "golden", "harness", "canary")

ORBIT_RE = re.compile(
    r"\[orbitlog\]\s+t=([\d.]+)s\s+frame=\"([^\"]*)\"\s+r=([-\d.e+]+) m\s+"
    r"v=([-\d.e+]+) m/s\s+sma=([-\d.e+]+) m\s+ecc=([-\d.e+]+)\s+"
    # Units live in the key names (#404): *_r are radii from the body CENTRE,
    # *_asl metres above its mean sea level. The apo_* pair is ABSENT on an
    # open conic -- it has no apoapsis, and a -1 there reads like a plausible
    # altitude (that is how a suborbital ship got logged as "in orbit").
    r"peri_r=([-\d.e+]+) m\s+peri_asl=([-\d.e+]+) m\s+"
    r"(?:apo_r=([-\d.e+]+) m\s+apo_asl=([-\d.e+]+) m\s+)?"
    r"inc=([-\d.e+]+) deg\s+"
    r"T=([-\d.e+]+) s\s+ttAp=([-\d.e+]+) s\s+ttPe=([-\d.e+]+) s\s+"
    r"\|h\|=([-\d.e+]+) m2/s\s+E=([-\d.e+]+) J/kg"
    # Plane angles (#171) trail the line; optional so nothing else breaks.
    # lan/lpe are either a number followed by " deg" or a bare "-".
    r"(?:\s+plane=(\w+)\s+lan=(\S+)(?: deg)?\s+lpe=(\S+)(?: deg)?)?"
)
MAP_RE = re.compile(
    r"\[mapdump\]\s+t=([\d.]+)s\s+slot=(\w+)\s+"
    r"n=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+"
    r"e1=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+"
    r"e2=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+sweep=([-+]\d+)"
    # The AN/DN crossings that slot draws (meters, focus at the origin), or "-".
    # Optional so a dump without them still parses.
    r"(?:\s+an=(\S+)\s+dn=(\S+))?"
)
DBG_RE = re.compile(
    r"\[dbg\]\s+t=([\d.]+)s\s+pos=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"alt=([-\d.]+) m\s+vel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+\|v\|=([-\d.]+) m/s"
)
# --plane-log (render.cpp drawOrbitPlane): the two plane normals the orbital-plane
# overlay drew, in the RENDER frame, plus the dihedral between them and the
# prograde / normal flight markers as placed, so a marker that leaves the plane
# it belongs on fails a CHECK instead of only looking wrong.
PLANE_RE = re.compile(
    r"\[planelog\]\s+t=([\d.]+)s\s+frame=(\w+)\s+plane=(\w+)\s+inc=([-\d.]+)\s+deg\s+"
    r"n_orb=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+"
    r"n_ref=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+"
    r"node=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+ticks=(\S+)\s+"
    r"prg=\(([-\d.]+),([-\d.]+),([-\d.]+)\)\s+"
    r"nrm=\(([-\d.]+),([-\d.]+),([-\d.]+)\)"
)
XFER_RE = re.compile(
    r"\[xferlog\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+"
    r"dv_dep=([-\d.e+]+) m/s\s+dv_cap=([-\d.e+]+) m/s\s+total=([-\d.e+]+) m/s\s+"
    r"tof=([-\d.e+]+) s\s+v_inf=([-\d.e+]+) m/s\s+"
    # r_cap is the capture periapsis RADIUS around the target, r_cap_asl the
    # same point as a HEIGHT above its sea level (#405). Both absent = this
    # solution has no capture burn.
    r"(?:r_cap=([-\d.e+]+) m\s+r_cap_asl=([-\d.e+]+) m\s+)?"
    r"burn=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s*(?:retrograde=(\d))?\s*(?:rev=(\d))?"
)
PORKCHOP_RE = re.compile(
    r"\[porkchop\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+"
    r"(\d+)x(\d+)\s+dep=\[([-\d.e+]+),([-\d.e+]+)\]s\s+tof=\[([-\d.e+]+),([-\d.e+]+)\]s\s+"
    r"dv_min=([-\d.e+]+) m/s\s+dv_hi=([-\d.e+]+) m/s\s+"
    r"t_dep_min=([-\d.e+]+) s\s+tof_min=([-\d.e+]+) s"
)
SURFMAP_RE = re.compile(
    r"\[surfmap\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+"
    r"(\d+)x(\d+)\s+albedo=\[([\d.]+) ([\d.]+) ([\d.]+)\]\s+"
    r"shaded=\[([\d.]+) ([\d.]+) ([\d.]+)\]\s+shade=(on|off)\s+"
    r"ocean=(on|off)\s+sites=(\d+)"
)
# --slew-log (vehicle.cpp slew_log): the attitude target the autopilot steers
# toward, in the ship's own frame. Pairs with --plane-log's n_orb: a prograde
# hold that aims at the ground's motion instead of the orbit's shows up as a
# target sitting off the plane.
SLEW_RE = re.compile(
    r"\[slew\]\s+t=([\d.]+)\s+mode=(\S+)\s+E=([-\d.]+)\s+rad\s+.*?"
    r"nose=\[([-\d.+]+) ([-\d.+]+) ([-\d.+]+)\]\s+"
    r"target=\[([-\d.+]+) ([-\d.+]+) ([-\d.+]+)\]"
)
# Game::toast mirrors every on-screen message to stdout ("the on-screen toast is
# transient, the log is not"), so a hotkey that only changes a flag still leaves
# an observable line. Parsed as plain strings: what a case wants is which
# messages appeared and in what order.
TOAST_RE = re.compile(r"\[toast\] (.*)")
ATT_RE = re.compile(
    r"\[attlog\]\s+t=([\d.]+)s\s+"
    r"nose=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"w=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"\|w\|=([-\d.]+) rad/s"
)
EVA_RE = re.compile(
    r"\[evalog\]\s+t=([\d.]+)s\s+mode=(\w+)\s+grounded=(\d)\s+"
    r"pos=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"vel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"face=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+alt=([-\d.]+) m"
    r"(?:\s+viewyaw=([-\d.]+))?(?:\s+rcs=(\d+))?(?:\s+mass=([-\d.]+)kg)?"
)
DRAINLOG_RE = re.compile(
    r"\[drainlog\]\s+t=([\d.]+)s\s+dt=([\d.]+)s\s+ship=\"([^\"]*)\"\s+"
    r"thrust=([-\d.]+)N\s+(.*)"
)
# --power-log (vehicle.cpp power_log): the whole electrical model in one row.
# gen/const_draw are watts (the RTGs and life support); charge/capacity are
# watt-hours pooled across the ship's battery parts; powered= is the gate the
# reaction wheels live behind -- they are electric, so no power means no
# attitude control. charge is the only one of the four that MOVES, and it
# should move by (gen - const_draw - wheels-while-commanding) x dt: a ship
# whose charge sits still is a ship whose battery is decorative (#379).
POWER_RE = re.compile(
    r"\[powerlog\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+powered=(\d+)\s+"
    r"gen=([-\d.]+)W\s+const_draw=([-\d.]+)W\s+charge=([-\d.]+)Wh\s+"
    r"capacity=([-\d.]+)Wh"
)
# --dv-log (vehicle.cpp dv_log): the running delta-v budget, cumulative since
# the ship existed. dv_del integrates the force the integrator was handed,
# dv_rocket the Tsiolkovsky cost of the propellant actually drained, dv_along
# the delivered impulse projected onto the velocity. mass is the ANALYTIC ship
# (getMass, what the fuel bookkeeping and the HUD use); hullmass is the compound
# Bullet actually accelerates, which rebuildCompound only re-makes once the two
# drift past kMassRebuildFrac apart -- so they differ by design, and a case pins
# the band rather than hiding it in a tolerance. A case takes the DIFFERENCE
# between two rows: that is what one burn spent.
DV_RE = re.compile(
    r"\[dv\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+mass=([-\d.]+)\s*kg\s+"
    r"hullmass=([-\d.]+)\s*kg\s+thrust=([-\d.]+)N\s+dv_del=([-\d.]+)\s+"
    r"dv_along=([-\d.]+)\s+dv_rocket=([-\d.]+)\s+prop=([-\d.]+)\s*kg\s+"
    r"ve=([-\d.]+) m/s\s+dv_left=([-\d.]+)"
)
# --mass-log (vehicle.cpp mass_log): one line per VEHICLE per sample, so the
# fleet's mass adds up. A decoupler hands its subtree to a new Vehicle, so the
# active ship's rows alone cannot show whether a split kept the mass it moved
# (#367) -- the survivor's rows are the other half of the sum. hullmass is
# absent for a hull out of the world: that number is the last one it had in
# physics rather than anything being accelerated now, and railed= is printed on
# every row so its absence stays a stated fact instead of something a parser
# infers (a nan hullmass then reads as a hole, not as a railed ship).
MASS_RE = re.compile(
    r"\[mass\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+parts=(\d+)\s+"
    r"mass=([-\d.]+)\s*kg(?:\s+hullmass=([-\d.]+)\s*kg)?\s+railed=(\d)"
)
DRAG_RE = re.compile(
    r"\[drag\]\s+t=([\d.]+)s\s+alt=([-\d.]+) m\s+rho=([-\d.e+]+) kg/m3\s+"
    r"\|v\|=([-\d.]+) m/s\s+\|F\|=([-\d.]+) N"
    r"(?:\s+\|L\|=([-\d.]+) N)?(?:\s+L_up=([-\d.]+) N)?(?:\s+Up=([-\d.]+))?"
    r"(?:\s+\|tau\|=([-\d.]+) Nm)?"
    r"\s+Cd=([-\d.]+)"
    r"(?:\s+A=([-\d.]+) m2)?(?:\s+(?:AoA=([-\d.e+]+) deg|AoA=-))?"
    r"(?:\s+Chute=([-\d.]+) m2)?(?:\s+Induced=([-\d.]+) m2)?"
    r"(?:\s+(?:Beta=([-\d.e+]+) deg|Beta=-))?"
)
# --drag-log's control surfaces (src/tick.cpp): one [ctrl] line under each [drag]
# line, one "name[index](axis)=±deg" term per surface the aero pass scored. It
# carries no t= of its own -- it prints inside the same log gate -- so parse_ctrl
# borrows the timestamp of the [drag] row above it. Every surface prints even at
# zero travel, so a missing row means the aero pass did not run or the part is
# not a surface, NOT that a surface held at zero. Until now nothing parsed this:
# no case could assert a surface moved, which is the whole observable of #453
# (per-part incidence) and #454 (static margin).
CTRL_ITEM_RE = re.compile(r"([^\s\[]+)\[(\d+)\]\((\w+)\)=([-+]\d+\.\d+)deg")
SHAKE_RE = re.compile(
    r"\[shakelog\]\s+t=([\d.]+)s\s+a=([-\d.]+) m/s2\s+"
    r"amp=([-\d.]+) m\s+off=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]"
)
# --cam-log: the camera the render pass left this frame. yaw/pitch are what a
# --sim-mouse drag is supposed to change; rmb/cap say whether the gesture
# reached the camera at all (a part window under the cursor eats the drag).
CAM_RE = re.compile(
    r"\[camlog\]\s+t=([\d.]+)s\s+mode=(\w+)\s+focus=\"([^\"]*)\"\s+"
    r"yaw=([-+\d.]+) rad\s+pitch=([-+\d.]+) rad\s+dist=([-+\d.]+) m\s+"
    r"fwd=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+rmb=(\d)\s+cap=(\d)"
)
TERRAIN_RE = re.compile(
    r"\[terrain\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+patches=(\d+)\s+"
    r"deepest=(\d+)\s+max_depth=(\d+)\s+collision=(\d+)\s+"
    r"deep_off=([-\d.e+]+) m\s+cam_r=([-\d.e+]+) m"
)
# aoa prints "-" (no value) outside an atmosphere or at rest in air, so it gets
# its own alternative rather than a wider character class that would eat the "-".
SURF_RE = re.compile(
    r"\[surfinfo\]\s+t=([\d.]+)s\s+alt_agl=([-\d.e+]+) m\s+"
    r"alt_asl=([-\d.e+]+) m\s+vs=([-\d.e+]+) m/s\s+hs=([-\d.e+]+) m/s\s+"
    r"lat=([-\d.e+]+) deg\s+lon=([-\d.e+]+) deg\s+pitch=([-\d.e+]+) deg\s+"
    r"roll=([-\d.e+]+) deg\s+hdg=([-\d.e+]+) deg\s+"
    r"(?:aoa=([-\d.e+]+) deg|aoa=-)\s+"
    r"acc=([-\d.e+]+) m/s2\s+"
    r"body=\"([^\"]*)\"\s+bme=\"([^\"]*)\"\s+sit=\"([^\"]*)\""
)
# A scene CHANGE only. The two refusal lines ("already in X -- push refused",
# "X is the floor -- pop refused") name no from/to pair, so they are not rows:
# a refused transition must not anchor anything.
SCENE_RE = re.compile(r"\[scene\]\s+(\w+) -> (\w+)\s+\((\w+)\)\s+t=([\d.]+)s")
DRAINLOG_RATE_RE = re.compile(r"g(\d+)=([-\d.]+)")
FUEL_RE = re.compile(
    r"\[fuel\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+(.*)"
)
# Unit repeat cannot cross into the next group: it requires `:` after the
# name while a group id is followed by `=`.
FUEL_GROUP_RE = re.compile(
    r"g(\d+)=((?:\s*[A-Za-z0-9]+:[-\d.]+/[-\d.]+\[[^\]]*\])*)"
)
FUEL_RES_RE = re.compile(r"([A-Za-z0-9]+):([-\d.]+)/([-\d.]+)\[([^\]]*)\]")
FUEL_LINK_RE = re.compile(r"links=(\S+)")
FUEL_LINK_PAIR_RE = re.compile(r"g?(\d+)->g?(\d+)")

# A repo-relative fixture path a case names, resolved against the repo root (the
# game runs with cwd=REPO_ROOT). One shape for every place a path can appear:
# ARGS values are comma-joined (--startship name,def,body,pad) and a flag can
# carry its path as --foo=res/..., so matching whole tokens only would miss
# both. A path inside an assertion's free text is the quieter half of #360:
# case 227's ARGS were fine, its EXPECT was not, because the game echoes the
# path back from inside a save. A backslash stays part of the token so a CHECK
# that escapes punctuation for its OWN regex ("res/ships/racer\.json") reads as
# one path instead of a truncated one; fixture_path() undoes the escapes. The
# lookbehind is not decoration: "e2e/fixtures/save_x" contains "res/save_x",
# which is no path the repo has -- case 186 proved that by failing the gate.
FIXTURE_PATH_RE = re.compile(r"(?<![A-Za-z0-9_/-])(?:e2e|res)/[A-Za-z0-9_.\\/-]+")
# A case that pins a REFUSAL of a path deliberately has no such file. It opts out
# of the existence check with a `# missing-ok: <path>` comment line.
MISSING_OK_RE = re.compile(r"^#\s*missing-ok:\s*(\S+)")


def fixture_path(token):
    """The path as the repo would spell it. A CHECK may escape punctuation for
    its own regex, and resdir::path() accepts a leading "./", so both spellings
    name one file -- and a `# missing-ok:` line has to match either spelling."""
    token = token.replace("\\", "")
    return token[2:] if token.startswith("./") else token


def under_data_dir(pattern, case_path, key):
    """A FILE/NOFILE pattern is relative to the case's scratch --data-dir. An
    absolute one would silently escape it -- os.path.join drops the first
    argument -- so it is a case bug, caught at parse time."""
    if os.path.isabs(pattern):
        raise ValueError("%s: %s path must be under the scratch data dir, not "
                         "absolute: %r" % (os.path.basename(case_path), key,
                                           pattern))
    return pattern


def check_fixture_paths(pairs, case_path, allowed):
    """A fixture that moved leaves its case pointing at the old path. The game
    then reports it cannot open the def, so the case fails on a missing EXPECT
    -- which reads like a behaviour change and hides the real bug (#360: the
    vehicle defs moved under e2e/vehicles/ and two cases kept the old path).
    Check every path a case names, at parse time, so the diagnosis names the
    dead path instead. `pairs` is (where-it-appeared, token); `allowed` is the
    case's own missing-ok list."""
    allowed = {fixture_path(p) for p in allowed}
    for key, token in pairs:
        token = fixture_path(token)
        if token in allowed:
            continue
        if not os.path.exists(os.path.join(REPO_ROOT, token)):
            raise ValueError("%s: %s names a path the repo does not have: %r "
                             "(a deliberate missing path needs a "
                             "'# missing-ok: %s' line)"
                             % (os.path.basename(case_path), key, token, token))


def parse_cases(path):
    name = os.path.basename(path)
    claim = ""
    kind = ""
    noglcheck = False
    args = []
    expect = []
    forbid = []
    check = []
    writes = []
    seeds = []
    files = []
    nofiles = []
    limit = DEFAULT_LIMIT
    # Paths a case deliberately names without the file existing (see
    # MISSING_OK_RE), collected from `# missing-ok: <path>` comment lines.
    missing_ok = []
    # EXIT: the exit code the case demands. Default 0; a boot-time refusal
    # (a bad --vab def, an unknown --startship body) is a PASS only if the
    # process exits nonzero, which no case could ask for before this key.
    want_exit = 0
    with open(path) as f:
        for raw in f:
            line = raw.strip()
            if line.startswith("#"):
                m = MISSING_OK_RE.match(line)
                if m:
                    missing_ok.append(m.group(1))
                continue
            if not line:
                continue
            key, _, rest = line.partition(" ")
            key = key.upper()
            rest = rest.strip()
            if key == "NAME":
                name = rest
            elif key == "CLAIM":
                claim = rest
            elif key == "KIND":
                kind = rest.lower()
            elif key == "NOGLCHECK":
                noglcheck = True
            elif key == "ARGS":
                # shlex, not split(): a value may be quoted to carry spaces
                # (--ui-click 2000,"Game Menu/Tracking Station"). No shipped
                # case quotes anything, so plain tokens tokenize identically.
                args.extend(shlex.split(rest))
            elif key == "EXPECT":
                expect.append(rest)
            elif key == "FORBID":
                forbid.append(rest)
            elif key == "CHECK":
                check.append(rest)
            elif key == "LIMIT":
                limit = float(rest)
            elif key == "EXIT":
                want_exit = int(rest)
            elif key == "WRITE":
                wpath, _, wcontent = rest.partition(" ")
                writes.append((wpath.strip(), wcontent.strip()))
            elif key == "SEED":
                # SEED <fixture dir> <dest under the scratch data dir>
                # [AGE_DAYS]: stages an existing save tree, which is how a
                # case reaches the state "the player already has saves"
                # without running the game twice. AGE_DAYS re-stamps the
                # seeded save.json that many days old, because a checkout
                # gives every fixture the same mtime and "which save is
                # last?" is decided by mtime.
                src, _, rest2 = rest.partition(" ")
                dst, _, age = rest2.partition(" ")
                src, dst, age = src.strip(), dst.strip(), age.strip()
                if not src or not dst:
                    raise ValueError("%s: SEED needs <fixture> <dest>"
                                     % os.path.basename(path))
                seeds.append((src, dst, int(age) if age else None))
            elif key == "FILE":
                # FILE <path under the scratch data dir> [substring ...]: the
                # game's own output, which no stdout assertion can reach
                # (#271). The path may glob -- a game dir is stamped
                # YYYYMMDD_HHMMSS-<name>, so no case can name it outright. An
                # absolute path would silently escape the scratch dir, so it is
                # a case bug. Substrings are shlex-split: quote one that
                # carries a space.
                fpath, _, fbody = rest.partition(" ")
                files.append((under_data_dir(fpath.strip(), path, key),
                              shlex.split(fbody)))
            elif key == "NOFILE":
                # NOFILE <path under the scratch data dir>: must NOT match
                # anything (an orphaned ship file, a temp the save left).
                # Vacuous if the pattern can never match -- a typo passes the
                # case without testing it, so pair it with a FILE on the same
                # directory.
                nofiles.append(under_data_dir(rest, path, key))
            else:
                raise ValueError("%s: unknown key %r" % (os.path.basename(path), key))
    if not args:
        raise ValueError("%s: no ARGS" % os.path.basename(path))
    # Every repo-relative path the case names, wherever it names it: the inputs
    # it hands the game, and the assertion text it expects the game to echo back.
    # The (?:e2e|res)/ prefix in FIXTURE_PATH_RE is the shape filter -- most of
    # ARGS' tokens are flags, timings and key names.
    fixture_paths = []
    for tok in args:
        fixture_paths.extend(("ARGS", p) for p in FIXTURE_PATH_RE.findall(tok))
    fixture_paths.extend(("SEED", src) for src, _, _ in seeds)
    for key, texts in (("EXPECT", expect), ("FORBID", forbid), ("CHECK", check)):
        for text in texts:
            fixture_paths.extend((key, p)
                                 for p in FIXTURE_PATH_RE.findall(text))
    check_fixture_paths(fixture_paths, path, missing_ok)
    if not claim:
        raise ValueError("%s: no CLAIM (one sentence: what does this pin?)"
                         % os.path.basename(path))
    if kind not in KINDS:
        raise ValueError("%s: KIND must be one of %s, not %r"
                         % (os.path.basename(path), ", ".join(KINDS), kind))
    return {
        "name": name,
        "claim": claim,
        "kind": kind,
        "noglcheck": noglcheck,
        "args": args,
        "expect": expect,
        "forbid": forbid,
        "check": check,
        "writes": writes,
        "seeds": seeds,
        "files": files,
        "nofiles": nofiles,
        "limit": limit,
        "exit": want_exit,
    }


def parse_orbit(out):
    rows = []
    for m in ORBIT_RE.finditer(out):
        (t, frame, r, v, sma, ecc, peri_r, peri_asl, apo_r, apo_asl, inc, T,
         ttAp, ttPe, h, E, plane, lan, lpe) = m.groups()
        rows.append({
            "t": float(t), "frame": frame, "r": float(r), "v": float(v),
            "sma": float(sma), "ecc": float(ecc),
            # *_r: radius from the body centre. *_asl: metres above its mean
            # sea level. Neither is the other (#404). apo_* is None when the
            # conic is open, so an apoapsis CHECK on an escaping ship fails
            # loudly instead of passing off a sentinel.
            "peri_r": float(peri_r), "peri_asl": float(peri_asl),
            "apo_r": float(apo_r) if apo_r is not None else None,
            "apo_asl": float(apo_asl) if apo_asl is not None else None,
            "inc": float(inc), "T": float(T),
            "ttAp": float(ttAp), "ttPe": float(ttPe), "h": float(h),
            "E": float(E),
            # None on a line without the #171 tail.
            "plane": plane, "lan": lan, "lpe": lpe,
        })
    return rows


def parse_mapb(out):
    rows = []
    for m in MAP_RE.finditer(out):
        (t, slot, nx, ny, nz, e1x, e1y, e1z, e2x, e2y, e2z, sweep,
         an, dn) = m.groups()
        def node(s):
            # "-" (or absent) = this slot has no crossing to mark.
            if not s or s == "-":
                return None
            return tuple(float(v) for v in s.strip("()").split(","))
        rows.append({
            "t": float(t), "slot": slot,
            "n": (float(nx), float(ny), float(nz)),
            "e1": (float(e1x), float(e1y), float(e1z)),
            "e2": (float(e2x), float(e2y), float(e2z)),
            "sweep": int(sweep),
            "an": node(an), "dn": node(dn),
        })
    return rows


def parse_plane(out):
    rows = []
    for m in PLANE_RE.finditer(out):
        (t, frame, plane_name, inc, ox, oy, oz, rx, ry, rz,
         nx, ny, nz, ticks, px, py, pz, sx, sy, sz) = m.groups()
        rows.append({
            "t": float(t), "frame": frame, "plane": plane_name,
            "inc": float(inc),
            "n_orb": (float(ox), float(oy), float(oz)),
            "n_ref": (float(rx), float(ry), float(rz)),
            # n_ref x n_orb: the ascending node (orbit.h's convention).
            "node": (float(nx), float(ny), float(nz)),
            # "-" when the planes are coplanar: no node to point at.
            "ticks": ticks,
            # The prograde marker and the normal axis (the + side, like n_orb),
            # unit, as paintWorld places them.
            "prg": (float(px), float(py), float(pz)),
            "nrm": (float(sx), float(sy), float(sz)),
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


def parse_slew(out):
    rows = []
    for m in SLEW_RE.finditer(out):
        (t, mode, e, nx, ny, nz, tx, ty, tz) = m.groups()
        rows.append({
            "t": float(t), "mode": mode, "E": float(e),
            "nose": (float(nx), float(ny), float(nz)),
            "target": (float(tx), float(ty), float(tz)),
        })
    return rows


def parse_toast(out):
    return [m.group(1).strip() for m in TOAST_RE.finditer(out)]


def parse_xfer(out):
    rows = []
    for m in XFER_RE.finditer(out):
        (t, target, dv_dep, dv_cap, total, tof, v_inf, r_cap, r_cap_asl,
         bx, by, bz, retro, rev) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            "dv_dep": float(dv_dep), "dv_cap": float(dv_cap),
            "total": float(total), "tof": float(tof),
            "v_inf": float(v_inf),
            # None = no capture burn. r_cap is a radius from the target's
            # centre; r_cap_asl is the height over its sea level (#405).
            "r_cap": float(r_cap) if r_cap is not None else None,
            "r_cap_asl": float(r_cap_asl) if r_cap_asl is not None else None,
            "burn": (float(bx), float(by), float(bz)),
            # -1 = a log line from before the field existed, not "prograde".
            "retrograde": int(retro) if retro is not None else -1,
            "rev": int(rev) if rev is not None else -1,
        })
    return rows


def parse_porkchop(out):
    rows = []
    for m in PORKCHOP_RE.finditer(out):
        (t, target, n_dep, n_tof, dep_lo, dep_hi, tof_lo, tof_hi,
         dv_min, dv_hi, t_dep_min, tof_min) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            "n_dep": int(n_dep), "n_tof": int(n_tof),
            # The window the sweep searched, not just what it found in it.
            "dep_lo": float(dep_lo), "dep_hi": float(dep_hi),
            "tof_lo": float(tof_lo), "tof_hi": float(tof_hi),
            "dv_min": float(dv_min), "dv_hi": float(dv_hi),
            "t_dep_min": float(t_dep_min), "tof_min": float(tof_min),
        })
    return rows


def parse_surfmap(out):
    rows = []
    for m in SURFMAP_RE.finditer(out):
        (t, body, w, h, ar, ag, ab, sr, sg, sb, shade, ocean, sites) = m.groups()
        rows.append({
            "t": float(t), "body": body,
            "w": int(w), "h": int(h),
            "albedo": (float(ar), float(ag), float(ab)),
            "shaded": (float(sr), float(sg), float(sb)),
            "shade": shade, "ocean": ocean, "sites": int(sites),
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
            # awroll pre-computed: free vars in a CHECK generator resolve
            # against globals, not the locals namespace.
            "wroll": wroll,
            "awroll": abs(wroll),
        })
    return rows


def parse_eva(out):
    rows = []
    for m in EVA_RE.finditer(out):
        (t, mode, grounded, px, py, pz, vx, vy, vz,
         fx, fy, fz, alt, viewyaw, rcs, mass) = m.groups()
        rows.append({
            "t": float(t), "mode": mode, "grounded": int(grounded),
            "pos": (float(px), float(py), float(pz)),
            "vel": (float(vx), float(vy), float(vz)),
            "face": (float(fx), float(fy), float(fz)),
            "alt": float(alt),
            # viewyaw, not yaw: cam["yaw"] is the camera azimuth (and the other
            # input to the same attitude law, #241).
            "viewyaw": float(viewyaw) if viewyaw is not None else None,
            # what the render pass gates the RCS puff on (#242).
            "rcs": int(rcs) if rcs is not None else None,
            "mass": float(mass) if mass is not None else None,
        })
    return rows


def parse_fuel(out):
    rows = []
    for m in FUEL_RE.finditer(out):
        t, ship, rest = m.groups()
        groups = {}
        for gm in FUEL_GROUP_RE.finditer(rest):
            res = {}
            for rm in FUEL_RES_RE.finditer(gm.group(2)):
                tanks = [float(x) for x in rm.group(4).split(",") if x]
                res[rm.group(1)] = (float(rm.group(2)), float(rm.group(3)), tanks)
            groups[int(gm.group(1))] = res
        links = []
        lm = FUEL_LINK_RE.search(rest)
        if lm:
            for pm in FUEL_LINK_PAIR_RE.finditer(lm.group(1)):
                links.append((int(pm.group(1)), int(pm.group(2))))
        rows.append({"t": float(t), "ship": ship,
                     "groups": groups, "links": links})
    return rows


def parse_drainlog(out):
    rows = []
    for m in DRAINLOG_RE.finditer(out):
        t, dt, ship, thrust, rest = m.groups()
        rates = {}
        for gm in DRAINLOG_RATE_RE.finditer(rest):
            rates[int(gm.group(1))] = float(gm.group(2))
        rows.append({
            "t": float(t), "dt": float(dt), "ship": ship,
            "thrust": float(thrust),
            "rates": rates,
        })
    return rows


def parse_power(out):
    rows = []
    for m in POWER_RE.finditer(out):
        (t, ship, powered, gen, const_draw, charge, capacity) = m.groups()
        rows.append({
            "t": float(t), "ship": ship, "powered": int(powered),
            "gen": float(gen), "const_draw": float(const_draw),
            "charge": float(charge), "capacity": float(capacity),
        })
    return rows


def parse_dv(out):
    rows = []
    for m in DV_RE.finditer(out):
        (t, ship, mass, hullmass, thrust, dv_del, dv_along, dv_rocket,
         prop, ve, dv_left) = m.groups()
        rows.append({
            "t": float(t), "ship": ship, "mass": float(mass),
            "hullmass": float(hullmass),
            "thrust": float(thrust), "dv_del": float(dv_del),
            "dv_along": float(dv_along), "dv_rocket": float(dv_rocket),
            "prop": float(prop), "ve": float(ve),
            "dv_left": float(dv_left),
        })
    return rows


def parse_mass(out):
    rows = []
    for m in MASS_RE.finditer(out):
        (t, ship, parts, mass, hullmass, railed) = m.groups()
        rows.append({
            "t": float(t), "ship": ship, "parts": int(parts),
            "mass": float(mass),
            "hullmass": float(hullmass) if hullmass is not None else None,
            "railed": int(railed),
        })
    return rows


def parse_shake(out):
    rows = []
    for m in SHAKE_RE.finditer(out):
        (t, a, amp, ox, oy, oz) = m.groups()
        rows.append({
            "t": float(t), "a": float(a), "amp": float(amp),
            "off": (float(ox), float(oy), float(oz)),
        })
    return rows


def parse_cam(out):
    rows = []
    for m in CAM_RE.finditer(out):
        (t, mode, focus, yaw, pitch, dist,
         fx, fy, fz, rmb, cap) = m.groups()
        rows.append({
            "t": float(t), "mode": mode, "focus": focus,
            "yaw": float(yaw), "pitch": float(pitch), "dist": float(dist),
            "fwd": (float(fx), float(fy), float(fz)),
            "rmb": int(rmb), "cap": int(cap),
        })
    return rows


def parse_terrain(out):
    rows = []
    for m in TERRAIN_RE.finditer(out):
        (t, body, patches, deepest, max_depth, collision,
         deep_off, cam_r) = m.groups()
        rows.append({
            "t": float(t), "body": body, "patches": int(patches),
            "deepest": int(deepest), "max_depth": int(max_depth),
            "collision": int(collision), "deep_off": float(deep_off),
            "cam_r": float(cam_r),
        })
    return rows


def parse_surf(out):
    rows = []
    for m in SURF_RE.finditer(out):
        (t, agl, asl, vs, hs, lat, lon, pitch, roll, hdg, aoa, acc,
         body, bme, sit) = m.groups()
        rows.append({
            "t": float(t), "alt_agl": float(agl), "alt_asl": float(asl),
            "vs": float(vs), "hs": float(hs), "lat": float(lat),
            "lon": float(lon), "pitch": float(pitch), "roll": float(roll),
            "hdg": float(hdg),
            # None when the ship had no air-relative flow to have an angle to.
            "aoa": float(aoa) if aoa is not None else None,
            "acc": float(acc),
            "body": body, "bme": bme, "sit": sit,
        })
    return rows


def parse_ctrl(out):
    """One row per surface per --drag-log sample (see CTRL_ITEM_RE). The [ctrl]
    line has no timestamp of its own, so each row carries the t= of the [drag]
    line it printed under; a row whose t is None would mean a [ctrl] with no
    [drag] above it, which tick.cpp cannot produce."""
    rows = []
    t = None
    for line in out.splitlines():
        if line.startswith("[ctrl]"):
            for name, idx, axis, deg in CTRL_ITEM_RE.findall(line):
                rows.append({"t": t, "name": name, "index": int(idx),
                             "axis": axis, "deg": float(deg)})
        elif line.startswith("[drag]"):
            m = DRAG_RE.match(line)
            if m:
                t = float(m.group(1))
    return rows


def parse_scene(out):
    rows = []
    for m in SCENE_RE.finditer(out):
        frm, to, verb, t = m.groups()
        rows.append({"from": frm, "to": to, "verb": verb, "t": float(t)})
    return rows


def parse_drag(out):
    rows = []
    for m in DRAG_RE.finditer(out):
        (t, alt, rho, v, F, L, lup, up, tau, cd, a, aoa, chute, ind,
         beta) = m.groups()
        row = {
            "t": float(t), "alt": float(alt), "rho": float(rho),
            "v": float(v), "F": float(F), "cd": float(cd),
        }
        if L is not None:
            row["L"] = float(L)
        if lup is not None:
            row["L_up"] = float(lup)
        if up is not None:
            row["up"] = float(up)
        if tau is not None:
            row["tau"] = float(tau)
        if a is not None:
            row["a"] = float(a)
        if aoa is not None:
            row["aoa"] = float(aoa)
        if chute is not None:
            row["chute"] = float(chute)
        if ind is not None:
            row["induced"] = float(ind)
        if beta is not None:
            row["beta"] = float(beta)
        rows.append(row)
    return rows


def first(seq):
    return seq[0]


def last(seq):
    return seq[-1]


def wine_for(game):
    """Wine binary for .exe artifacts, or None."""
    if not game.endswith(".exe"):
        return None
    return shutil.which("wine64") or shutil.which("wine")


def have_render_node():
    """True if a DRM render node is openable (SDL offscreen/EGL path)."""
    for path in sorted(glob.glob("/dev/dri/renderD*")):
        try:
            fd = os.open(path, os.O_RDWR)
        except OSError:
            continue
        os.close(fd)
        return True
    return False


def build_cmd(game, args):
    wine = wine_for(game)
    if wine:
        xvfb = shutil.which("xvfb-run")
        if xvfb:
            return [xvfb, "-a", wine] + [game] + args
        return [wine] + [game] + args

    # SDL offscreen (EGL) when a render node is usable; else Xvfb/GLX.
    if have_render_node():
        return [game] + args

    if os.environ.get("DISPLAY") and shutil.which("xvfb-run") is None:
        return [game] + args
    xvfb = shutil.which("xvfb-run")
    if xvfb:
        return [xvfb, "-a"] + [game] + args
    # Bare (will fail to open a window -- the case reports it).
    return [game] + args


def launch_env(game):
    """Environment overrides for the game process, or None to inherit."""
    wine = wine_for(game)
    if wine:
        # WINEDEBUG=-all: Wine's own "GL_" chatter would trip FORBID GL_.
        prefix = os.path.join(REPO_ROOT, "tmp", "wine")
        os.makedirs(prefix, exist_ok=True)
        env = dict(os.environ)
        env["WINEPREFIX"] = prefix
        env["WINEDEBUG"] = "-all"
        return env
    if have_render_node():
        env = dict(os.environ)
        # offscreen driver + no DISPLAY so SDL cannot fall through to X11.
        env["SDL_VIDEODRIVER"] = "offscreen"
        env.pop("DISPLAY", None)
        return env
    return None


def run_case(case):
    """Run one case. Returns (passed, diagnostics-lines, out, duration_s)."""
    game = GAME or os.path.join(REPO_ROOT, "osp")
    if not os.path.exists(game):
        return False, ["%s not found; run `make` first"
                       % os.path.relpath(game, REPO_ROOT)], "", 0.0
    # Nothing to clean for ImGui layout: main.cpp sets io.IniFilename =
    # nullptr, so the game neither reads nor writes imgui.ini (#45 turned out
    # to be a race on a file that can never exist). If the ini is ever
    # re-enabled it lands in the process cwd -- the repo root, shared by every
    # parallel case -- and then it needs a per-case path, not this delete.

    # Scratch --data-dir per case so settings/saves cannot leak between runs.
    # saves/ is wiped, not just settings.json: a case that saves leaves the
    # slot on disk, and a later run of any case that asks "does the player
    # have a save?" would answer for the previous run instead.
    #
    # ships/ for the same reason and with the same consequence: the VAB writes
    # the player's ships there, so a FILE ships/... assertion would be satisfied
    # by the previous run of the very case making it.
    data_dir = os.path.join(REPO_ROOT, "tmp", "e2e", "data", case["name"])
    os.makedirs(data_dir, exist_ok=True)
    try:
        os.remove(os.path.join(data_dir, "settings.json"))
    except FileNotFoundError:
        pass
    shutil.rmtree(os.path.join(data_dir, "saves"), ignore_errors=True)
    shutil.rmtree(os.path.join(data_dir, "ships"), ignore_errors=True)

    # WRITE staging (after cleanup so a fixture is live for exactly this
    # case). settings.json goes in the scratch data dir (datadir.h).
    for wpath, wcontent in case.get("writes", []):
        target = (os.path.join(data_dir, "settings.json")
                  if wpath == "settings.json"
                  else os.path.join(REPO_ROOT, wpath))
        with open(target, "w") as wf:
            wf.write(wcontent)

    # SEED staging: an existing save tree under the scratch data dir, so a
    # case can boot straight into "there is a save to resume".
    for src, dst, age_days in case.get("seeds", []):
        shutil.copytree(os.path.join(REPO_ROOT, src),
                        os.path.join(data_dir, dst))
        if age_days is not None:
            # copytree preserves the fixture's mtime, and every fixture shares
            # one (the checkout time), so an age has to be stamped explicitly.
            meta = os.path.join(data_dir, dst, "save.json")
            t = time.time() - age_days * 86400.0
            os.utime(meta, (t, t))

    # GL_ is the suite's GL-error tripwire: the game prints "GL_<ENUM> -
    # file:line" from its glGetError drains, and those drains are off by
    # default (the call can flush the command buffer, ~100 sites). Every case
    # gets the drains unless NOGLCHECK says its timing is the subject;
    # --gl-debug would install the debug callback too, whose driver chatter
    # (Mesa performance warnings) trips the same FORBID.
    args = list(case["args"])
    if not case.get("noglcheck"):
        args.append("--gl-error-checks")

    cmd = build_cmd(game, args + ["--data-dir", data_dir])
    env = launch_env(game)
    if SHOW_CMD:
        # Quoted so the line pastes straight into a shell (the --ui-click
        # paths carry spaces).
        print("cmd  %s: %s" % (case["name"],
                               " ".join(shlex.quote(a) for a in cmd)),
              flush=True)
        if env:
            extra = {k: v for k, v in env.items() if os.environ.get(k) != v}
            if extra:
                print("env  %s: %s" % (case["name"],
                                       " ".join("%s=%s" % kv
                                                for kv in sorted(extra.items()))),
                      flush=True)
    diag = []
    timed_out = False
    exit_code = None
    out = ""
    # Own process group so a timeout can kill wine + the game PE + Xvfb.
    t0 = time.monotonic()
    limit = LIMIT if LIMIT is not None else case["limit"]
    proc = subprocess.Popen(
        cmd, cwd=REPO_ROOT, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )
    try:
        out_bytes, _ = proc.communicate(timeout=limit)
        out = out_bytes.decode("utf-8", "replace")
        exit_code = proc.returncode
    except subprocess.TimeoutExpired as e:
        timed_out = True
        out = (e.output or b"").decode("utf-8", "replace")
    finally:
        if proc.poll() is None:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except (ProcessLookupError, PermissionError):
                pass
            try:
                proc.wait(timeout=10)
            except subprocess.TimeoutExpired:
                pass
    duration = time.monotonic() - t0

    # 1) exit code
    if timed_out:
        diag.append("runner timeout after %.0fs (%s)"
                    % (limit, "--limit" if LIMIT is not None else "LIMIT"))
    elif exit_code != case["exit"]:
        diag.append("exit code %s (expected %s)" % (exit_code, case["exit"]))

    # 2) EXPECT / FORBID
    missing = [s for s in case["expect"] if s not in out]
    for s in missing:
        diag.append("EXPECT not found: %r" % s)
    hit = [s for s in case["forbid"] + DEFAULT_FORBID if s in out]
    for s in dict.fromkeys(hit):
        diag.append("FORBID found: %r" % s)

    # 3) CHECK
    # Telemetry belongs to the world that printed it. A successful load starts a
    # new one, so rows before the LAST load line describe a world the case is no
    # longer testing -- a sheet load from the Title would otherwise leave the
    # Title's rows at the head of every list, and first()/all()/last() would
    # read them. A boot load cuts nothing: its line precedes every row.
    # `scene` and `toast` deliberately stay whole-run -- they describe the run,
    # not a world, so a CHECK that mixes one with a world-scoped row list is
    # comparing across that line and should say so.
    cut = out.rfind("Loaded game from ")
    world = out[cut:] if cut >= 0 else out
    orbit = parse_orbit(world)
    dbg = parse_dbg(world)
    xfer = parse_xfer(world)
    porkchop = parse_porkchop(world)
    surfmap = parse_surfmap(world)
    att = parse_att(world)
    eva = parse_eva(world)
    fuel = parse_fuel(world)
    drainlog = parse_drainlog(world)
    power = parse_power(world)
    dv = parse_dv(world)
    mass = parse_mass(world)
    drag = parse_drag(world)
    ctrl = parse_ctrl(world)
    shake = parse_shake(world)
    cam = parse_cam(world)
    terrain = parse_terrain(world)
    surf = parse_surf(world)
    scene = parse_scene(out)
    mapb = parse_mapb(world)
    plane = parse_plane(world)
    slew = parse_slew(world)
    toast = parse_toast(out)
    ns = {
        "out": out, "orbit": orbit, "dbg": dbg, "xfer": xfer,
        "porkchop": porkchop, "surfmap": surfmap, "att": att, "eva": eva,
        "fuel": fuel, "drainlog": drainlog, "power": power, "dv": dv, "mass": mass,
        "drag": drag, "shake": shake,
        "ctrl": ctrl,
        "cam": cam,
        "terrain": terrain,
        "surf": surf, "mapb": mapb, "plane": plane, "slew": slew,
        "scene": scene,
        "toast": toast,
        "first": first, "last": last,
        "abs": abs, "len": len, "any": any, "all": all,
        "max": max, "min": min, "float": float, "int": int, "zip": zip,
        "set": set, "sum": sum,
        "re": re,
    }
    # `ns` must be globals: free vars in a CHECK generator/comprehension
    # resolve against globals, not locals. Empty __builtins__ blocks open/exec.
    check_globals = dict(ns)
    check_globals["__builtins__"] = {}
    for expr in case["check"]:
        try:
            ok = bool(eval(expr, check_globals))  # noqa: S307 - trusted case files
        except Exception as e:
            diag.append("CHECK raised: %r (%s)" % (expr, e))
            continue
        if not ok:
            diag.append("CHECK failed: %s" % expr)

    # 4) FILE / NOFILE: what the game wrote under its scratch --data-dir.
    # stdout cannot show a torn file, an orphan the sweep missed, or a saved
    # field nobody prints (#271, #268). A path may glob; FILE needs at least
    # one match, and each substring is checked against the matched contents
    # JOINED -- so "saves/*/slot*/ships/*.json" asks "is it in some ship
    # file", which is what a slug-keyed save makes a name lookup become.
    for pattern, subs in case.get("files", []):
        hits = sorted(glob.glob(os.path.join(data_dir, pattern)))
        if not hits:
            diag.append("FILE missing: %s" % pattern)
            continue
        blob = ""
        for hp in hits:
            try:
                with open(hp, errors="replace") as fh:
                    # Joined WITH a separator: without one, a substring could
                    # straddle the end of one file and the start of the next
                    # and pass on text neither file contains.
                    blob += fh.read() + "\n"
            except OSError as e:
                diag.append("FILE unreadable: %s (%s)" % (hp, e))
        for s in subs:
            if s not in blob:
                diag.append("FILE %s lacks %r" % (pattern, s))
    for pattern in case.get("nofiles", []):
        hits = sorted(glob.glob(os.path.join(data_dir, pattern)))
        if hits:
            diag.append("NOFILE found: %s (%s)"
                        % (pattern, ", ".join(os.path.relpath(h, data_dir)
                                              for h in hits[:4])))

    passed = (not diag)
    return passed, diag, out, duration


def select_cases(case_files, selectors):
    """Filter to cases whose NAME, filename or subject path contains any
    selector (case-insensitive). A path selector like `save/` or
    `save/270` scopes a run to one group."""
    if not selectors:
        return case_files
    sel = [s.lower() for s in selectors]
    out = []
    for path in case_files:
        base = os.path.splitext(os.path.basename(path))[0].lower()
        rel = os.path.splitext(os.path.relpath(path, CASES_DIR))[0].lower()
        try:
            name = parse_cases(path)["name"].lower()
        except ValueError:
            name = ""
        if any(s in name or s in base or s in rel for s in sel):
            out.append(path)
    return out


def case_kind(path):
    """The case's KIND, or "" if the file does not parse. A broken case must not
    take down the whole invocation: run_one() turns the same ValueError into one
    per-case FAIL, so filtering on a raw parse_cases() here would raise out of
    main() and every other selected case would never run. Name the file anyway:
    a silent "" drops it from a --kind selection as if it never existed."""
    try:
        return parse_cases(path)["kind"]
    except ValueError as e:
        warn("warning: %s" % e)
        return ""


def available_names(case_files):
    names = []
    for path in case_files:
        try:
            names.append(parse_cases(path)["name"])
        except ValueError:
            names.append(os.path.basename(path))
    return names


def run_one(path):
    """Parse and run one case. Never raises.
    Returns (filebase, name, passed, diag, out, duration_s)."""
    filebase = os.path.splitext(os.path.basename(path))[0]
    try:
        case = parse_cases(path)
    except ValueError as e:
        return filebase, filebase, False, \
            ["bad case file: %s" % e], "", 0.0
    try:
        passed, diag, out, duration = run_case(case)
    except Exception as e:
        return filebase, case["name"], False, ["runner error: %r" % e], "", 0.0
    return filebase, case["name"], passed, diag, out, duration


def git_state():
    """(short commit, dirty flag) best-effort; dirty is None if git is unavailable."""
    def git(*args):
        p = subprocess.run(["git", *args], cwd=REPO_ROOT, capture_output=True,
                           text=True, timeout=10)
        return p.stdout if p.returncode == 0 else ""
    try:
        commit = git("rev-parse", "--short", "HEAD").strip() or "none"
        dirty = bool(git("status", "--porcelain").strip())
        return commit, dirty
    except (OSError, subprocess.SubprocessError):
        return "none", None


def new_run_dir():
    """tmp/e2e/runs/<UTCstamp>/ (suffixed on collision). mkdir is atomic."""
    stamp = datetime.datetime.now(datetime.timezone.utc).strftime("%Y%m%dT%H%M%SZ")
    base = os.path.join(REPO_ROOT, "tmp", "e2e", "runs")
    os.makedirs(base, exist_ok=True)
    n = 1
    while True:
        path = os.path.join(base, stamp if n == 1 else "%s-%d" % (stamp, n))
        try:
            os.mkdir(path)
            return path
        except FileExistsError:
            n += 1


def warn(msg):
    """Diagnostic line. stderr, so it cannot land inside a parsed run summary."""
    print(msg, file=sys.stderr, flush=True)


HISTORY_FIELDS = ["datetime_utc", "commit", "dirty", "renderer", "game",
                  "jobs", "cases", "n", "passed", "failed", "run_duration_s",
                  "run_dir"]


def print_progress(idx, total, res):
    """One line per case as it finishes. Flush: piped output is block-buffered."""
    _, name, passed, diag, _, dur = res
    print("[%d/%d] %s %s (%.1fs)" % (idx, total, name,
                                     "PASS" if passed else "FAIL", dur),
          flush=True)
    for line in diag:
        print("        %s" % line, flush=True)


def format_summary(results):
    """Human pass/fail table (console + summary.txt)."""
    if not results:
        return "0/0 passed"
    width = max(len(r[1]) for r in results)
    lines = []
    for filebase, name, passed, diag, out, dur in results:
        lines.append("%-*s  %6.1fs  %s" % (width, name, dur,
                                          "PASS" if passed else "FAIL"))
        for line in diag:
            lines.append("        %s" % line)
    total = len(results)
    fails = sum(1 for r in results if not r[2])
    lines.append("-" * (width + 14))
    lines.append("%d/%d passed" % (total - fails, total))
    return "\n".join(lines)


def write_run_artifacts(results, meta):
    """Persist per-case .log + summary.json/txt in a fresh run dir, and append
    one history.csv row. Returns (run_dir, history_path)."""
    run_dir = new_run_dir()
    total = len(results)
    fails = sum(1 for r in results if not r[2])
    cases = []
    for filebase, name, passed, diag, out, dur in results:
        with open(os.path.join(run_dir, filebase + ".log"), "w",
                  encoding="utf-8") as f:
            f.write(out)
        cases.append({
            "file": filebase + ".txt",
            "name": name,
            "passed": passed,
            "duration_s": round(dur, 3),
            "diag": diag,
        })

    summary = {
        "started": meta["started"].strftime("%Y-%m-%dT%H:%M:%SZ"),
        "finished": meta["finished"].strftime("%Y-%m-%dT%H:%M:%SZ"),
        "duration_s": round((meta["finished"] - meta["started"]).total_seconds(), 3),
        "commit": meta["commit"],
        "dirty": meta["dirty"],
        "renderer": meta["renderer"],
        "game": meta["game"],
        "jobs": meta["jobs"],
        "selectors": meta["selectors"],
        "passed": total - fails,
        "failed": fails,
        "cases": cases,
    }
    with open(os.path.join(run_dir, "summary.json"), "w",
              encoding="utf-8") as f:
        json.dump(summary, f, indent=2)
        f.write("\n")

    header = "\n".join([
        "e2e run   %s" % summary["started"],
        "commit    %s%s" % (summary["commit"], " (dirty)" if summary["dirty"] else ""),
        "renderer  %s" % summary["renderer"],
        "game      %s" % summary["game"],
        "jobs      %d" % summary["jobs"],
        "cases     %s" % ("all" if not summary["selectors"]
                         else ";".join(summary["selectors"])),
        "",
    ])
    with open(os.path.join(run_dir, "summary.txt"), "w",
              encoding="utf-8") as f:
        f.write(header + format_summary(results) + "\n")

    history_path = os.path.join(REPO_ROOT, "tmp", "e2e", "history.csv")
    # O_EXCL: exactly one concurrent runner creates the header.
    fresh = False
    try:
        fd = os.open(history_path, os.O_CREAT | os.O_EXCL | os.O_WRONLY)
        os.close(fd)
        fresh = True
    except FileExistsError:
        if os.path.getsize(history_path) == 0:
            fresh = True
    with open(history_path, "w" if fresh else "a",
              newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        if fresh:
            w.writerow(HISTORY_FIELDS)
        w.writerow([
            summary["started"],
            summary["commit"],
            "" if summary["dirty"] is None else ("yes" if summary["dirty"] else "no"),
            summary["renderer"],
            summary["game"],
            summary["jobs"],
            "all" if not summary["selectors"] else ";".join(summary["selectors"]),
            total,
            total - fails,
            fails,
            "%.1f" % summary["duration_s"],
            os.path.relpath(run_dir, REPO_ROOT),
        ])
    return run_dir, history_path


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("selectors", nargs="*",
                        help="case name/filename/path substring(s) to run; "
                             "omit to run the whole battery")
    parser.add_argument("--jobs", type=int, default=DEFAULT_JOBS,
                        help="max cases to run in parallel (1 = serial; "
                             "default: 2)")
    parser.add_argument("--game", default=None,
                        help="game binary to launch (default: ./osp)")
    parser.add_argument("--limit", type=float, default=None,
                        help="wall-clock ceiling per case, in seconds, "
                             "overriding every case's LIMIT (the game still "
                             "exits on its own --timeout; this only decides "
                             "when the runner kills the process tree)")
    parser.add_argument("--show-cmd", action="store_true",
                        help="print each case's exact command line (and the "
                             "env it differs in) before running it, so a "
                             "failure can be re-run under gdb/valgrind "
                             "without rebuilding the invocation by hand")
    parser.add_argument("--kind", action="append", choices=KINDS,
                        help="run only cases of this KIND (repeatable): "
                             "invariant, golden, harness or canary")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be >= 1")
    if args.limit is not None and args.limit <= 0:
        parser.error("--limit must be > 0")
    global GAME, LIMIT, SHOW_CMD
    GAME = args.game
    LIMIT = args.limit
    SHOW_CMD = args.show_cmd
    if wine_for(GAME or ""):
        renderer = "wine+xvfb"
    elif have_render_node():
        renderer = "egl-offscreen"
        print("GL: SDL offscreen/EGL (DRM render node present)")
    else:
        renderer = "xvfb-glx"
        print("GL: Xvfb/GLX software (no /dev/dri/renderD*)")
    commit, dirty = git_state()
    # Default to serial under Wine: concurrent wine+Xvfb+llvmpipe instances
    # overload a dev box and go flaky. An explicit --jobs wins.
    if args.jobs == DEFAULT_JOBS and GAME and wine_for(GAME):
        args.jobs = 1
    selectors = args.selectors

    # No selectors means the whole battery. That is what `make release` runs;
    # the wide-selection warning below is what tells a day-to-day runner it has
    # asked for too much.
    all_files = sorted(glob.glob(os.path.join(CASES_DIR, "**", "*.txt"),
                                 recursive=True))
    if not all_files:
        print("no case files found in %s" % CASES_DIR)
        return 1

    case_files = select_cases(all_files, selectors)
    if args.kind:
        case_files = [p for p in case_files if case_kind(p) in args.kind]
    if not case_files:
        # Name every filter that contributed: with no selectors a --kind that
        # matches nothing otherwise reads as `no case matches ()`.
        asked = ["%r" % s for s in selectors]
        asked += ["--kind %s" % k for k in (args.kind or [])]
        print("no case matches %s" % " ".join(asked))
        names = available_names(all_files)
        print("available: %s" % ", ".join(names[:40]))
        if len(names) > 40:
            print("... %d more; e2e/INDEX.md lists every case with its claim."
                  % (len(names) - 40))
        return 1

    # Cases are independent (own Xvfb display + stdout capture), so they can
    # run concurrently. Progress prints in completion order; the final table
    # stays in case-file order.
    started = datetime.datetime.now(datetime.timezone.utc)
    total = len(case_files)
    results = [None] * total
    if args.jobs == 1:
        for i, p in enumerate(case_files):
            results[i] = run_one(p)
            print_progress(i + 1, total, results[i])
    else:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            fut_idx = {pool.submit(run_one, p): i for i, p in enumerate(case_files)}
            for done, fut in enumerate(as_completed(fut_idx), 1):
                i = fut_idx[fut]
                results[i] = fut.result()
                print_progress(done, total, results[i])
    finished = datetime.datetime.now(datetime.timezone.utc)

    # Persist before printing so the console can point at the artifacts.
    # A persist failure must not swallow the summary or the exit code.
    meta = {
        "started": started,
        "finished": finished,
        "commit": commit,
        "dirty": dirty,
        "renderer": renderer,
        "game": os.path.relpath(GAME or os.path.join(REPO_ROOT, "osp"), REPO_ROOT),
        "jobs": args.jobs,
        "selectors": list(args.selectors),
    }
    try:
        run_dir, history = write_run_artifacts(results, meta)
    except OSError as e:
        print("warning: run artifacts not saved: %s" % e, file=sys.stderr)
        run_dir = history = None

    print(format_summary(results))
    if run_dir is not None:
        print("logs:    %s" % run_dir)
        print("history: %s" % history)
    return 1 if any(not r[2] for r in results) else 0


if __name__ == "__main__":
    sys.exit(main())
