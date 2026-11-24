#!/usr/bin/env python3
"""E2E runner: run selected e2e/cases/<subject>/*.txt against the game binary.

Usage: python3 e2e/run.py <selectors> [--jobs N] [--game PATH] [--limit S]
[--show-cmd] [--kind K] [--self-test]. No selectors runs the whole battery --
that is what `make release` does. A selector that matches no case refuses the
run with its name even when its neighbours matched (#1061): the count a batch
prints is the count that was ASKED for, not the count that happened to exist.
Nothing caps or paces a run: how much e2e is
worth running is the caller's judgement, not the runner's policy. --limit
overrides
every case's LIMIT (a slow box, or a case you want to drive by hand);
--show-cmd prints the exact command + env so a failure re-runs under a
debugger; --kind runs one kind of case (repeatable). Case keys: NAME CLAIM
KIND NOGLCHECK ARGS EXPECT FORBID
CHECK LIMIT WRITE SEED FILE NOFILE EXIT. CLAIM is one sentence: what the case
pins. A CHECK is ONE expression: a trailing comma makes it a tuple and a bare
list/set/dict literal is truthy whatever it holds, so both are refused at parse
time -- as is a CHECK that does not parse -- rather than passing quietly, which
is how an assertion goes silently dead (#650). An `all()`/`any()` whose input was
EMPTY prints a `#note` line beside the case -- `all([])` is true, so a filtered
aggregate that matched no rows asserts nothing -- and does not redden it, because
an empty filter can be exactly what the case means (#653). The other way a CHECK
goes silently dead is a row that never reached the list: a number that blew up
prints "inf"/"-nan", this file's number fields stop at those letters, and the
whole line is dropped. A case that names a row list now fails with the offending
line quoted when the game printed a line of that tag whose number reads inf or
nan, which the parser could not read (#408). KIND says what sort of
evidence it is -- invariant (behaviour or a
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
terrain/surf/xfer/porkchop/surfmap/surfmark/solar/mapb/plane/slew/toast/scene) plus
first/last, `re`, and the pure builtins `abs len any all max min float int zip
set sum sorted tuple range` (#910). A CHECK evaluates against an empty
`__builtins__` -- that is what keeps a case out of the filesystem and the
interpreter -- so a helper a case author needs gets added to that list, not
worked around inside the case. Every row of a world-scoped log carries `ship`: the vessel that
was active when the game printed the line, read from the game's own "Active ship
N of M: NAME" marker, or None while the run has never switched; `shipswitch`
lists those handoffs themselves, so a CHECK can name its ship instead of mixing
two vessels' rows (#447). `rec` is the exception to "parse stdout": the flight recorder's
trace is a FILE, so the harness reads it from the case's scratch --data-dir
(flights/*.log, #662). The row lists parse stdout from the LAST "Loaded game from"
line: a successful load
starts a new world, and rows the previous world printed are not evidence about
the one under test (toast/scene stay whole-run -- they describe the run).
The load-through-the-UI shape most save/load cases now take: Game Debug ticks
the e2e fixture root, the sheet picks the "fixtures" game, the slot filter
narrows the list to one row, and the row + Load button do the rest. The load
lands at 4s, so a case's own scripted events shift +5000ms and the clock
resumes on the warp ladder (PERIOD presses) -- a load starts paused, where
--time-accel used to hand a boot load its warp. e2e/convert_load_to_ui.py
generates that preamble. The game reports every scripted --ui-click/--ui-type/
--ui-scroll on stdout, and the runner holds a case to that report (#860): an
action the game never mentioned fails the case, and so does a FAILED line the
case did not pin -- an EXPECT that is a substring of the refusal is how a case
says the refusal is the point. ARGS tokenizes like a
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
import ast
import csv
import datetime
import glob
import json
import math
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
# What a CHECK may call besides the row lists and first/last. A CHECK evaluates
# against an EMPTY __builtins__ (see check_globals), which is what keeps a case
# out of the filesystem and the interpreter; this dict is the whole allowlist, so
# a helper a case author needs gets added HERE rather than worked around inside
# the case (#910). `any`/`all` deliberately stay out: the runner swaps in
# rec_any/rec_all, which count a walk that saw no rows (#653). --self-test pins
# the trio below and that the allowlist is still closed.
CHECK_BUILTINS = {
    "abs": abs, "len": len,
    "max": max, "min": min, "float": float, "int": int, "zip": zip,
    "set": set, "sum": sum,
    # The order-pinning trio: a case whose claim IS a sequence -- staging order,
    # legend draw order, which row came first -- wants `tuple(sorted(...))`, and
    # walking two lists by index wants `range`.
    "sorted": sorted, "tuple": tuple, "range": range,
    "re": re,
}
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
# --ascent-log (src/tick.cpp): the ascent law's own state, one row per log tick.
# The phase is the law's side of the contract with the player -- "done" means the
# orbit is the one that was asked for, "failed" means it says so out loud -- so a
# case should read it rather than grep for it. peri_asl/apo_asl/inc_eq/node_r are
# dashed when the law has no such number for that tick: a railed hull solves no
# conic, an open trajectory has no apoapsis, a coast before the node has no node.
# The dash is the absence every other row-based log already prints (#894 migrated
# this row off "nan"), and like those fields a dashed one carries NO unit: the row
# reads `node_r=-`, not `node_r=- m`. That is what makes the two-arm form below
# unambiguous -- `[-\d.e+]+ m` would happily eat a lone '-' and leave float() with
# something it cannot read, which is exactly how this migration was caught. The
# convention cuts one way, too: the field AFTER a dash must not start with the unit's
# letter, since `peri_asl=- mass=` would match the numeric arm with the group '-'.
# Today's followers are apo_asl / inc_eq / node_r / accel / why, all safe, and the
# mistake raises in parse_ascent rather than reporting a wrong number. A CHECK
# that wants a real number now fails against None instead of agreeing with a
# placeholder that used to read like a measurement (-1 m apoapsis, -1.00 deg
# inclination) -- and instead of comparing a float("nan"), where every comparison
# is quietly false. A nan that slips back into these fields matches NEITHER arm, so
# the row is dropped and named by #408 rather than parsed; --self-test pins that.
ASCENT_RE = re.compile(
    r"\[ascent\]\s+t=([\d.]+)s\s+phase=(\w+)\s+lever=([-\d.e+]+)\s+pitch=([-\d.e+]+) deg\s+"
    r"(?:peri_asl=([-\d.e+]+) m|peri_asl=-)\s+"
    r"(?:apo_asl=([-\d.e+]+) m|apo_asl=-)\s+"
    r"(?:inc_eq=([-\d.e+]+) deg|inc_eq=-)\s+"
    r"(?:node_r=([-\d.e+]+) m|node_r=-)"
    # The stage's full-throttle acceleration and the body's pull at the hull: the
    # pair the law's lift-off gate compares, so "this stage cannot lift off" can
    # be pinned from the row instead of taken on trust. Optional: a row without
    # them still parses.
    r"(?:\s+accel=([-\d.e+]+) m/s2\s+g=([-\d.e+]+) m/s2)?"
    r"\s+why=\"([^\"]*)\""
    # The ascent profile the autopilot flies by (#675): how high the hull has been,
    # the conic's speed, the whole-hull dynamic pressure, the flight-path angle, and
    # what the burns have cost. APPENDED after why= rather than inserted mid-line:
    # every CHECK that reads these rows does so through the group ORDER above, and
    # the comment at the top of this regex exists to say that a field the parser
    # cannot read must raise or fail, never silently answer a wrong number -- a
    # mid-line insert is exactly the mistake that rule guards against. Optional
    # whole, so a row written before #675 still parses and a CHECK that wants a
    # reading fails loudly on the old format instead of comparing against None by
    # accident... which is what the group is for: None means the row has no number
    # (gamma dashes when the hull has no velocity to have an angle THROUGH).
    r"(?:\s+alt_asl=([-\d.e+]+) m\s+v_orb=([-\d.e+]+) m/s\s+q=([-\d.e+]+) Pa\s+"
    r"(?:gamma=([-\d.e+]+) deg|gamma=-)"
    r"\s+dv=([-\d.e+]+) m/s\s+grav=([-\d.e+]+) m/s\s+drag=([-\d.e+]+) m/s"
    r"\s+steer=([-\d.e+]+) m/s\s+burns=(\d+))?"
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
# --dbg-log's orbital buildings (src/tick.cpp): each driven pose in the body's
# inertial frame, so a CHECK can pin that a station actually propagates (r stays
# on its conic, pos advances) rather than sitting still.
BLDG_RE = re.compile(
    r"\[bldg\]\s+t=([\d.]+)s\s+id=(\S+)\s+pos=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"r=([-\d.]+) m\s+live=(\d)"
)
# --map-dump's station marks (src/gameui.cpp, #872): the driven buildings the orbital
# and Tracking maps would mark around the body they are centred on. The maps draw pure
# imgui geometry, so this row is the only way a case can pin a marker at all -- and
# pts=0 for an air platform is the feature, not a missing sample: a wind-riding
# platform has no conic to draw. pos is the mark and anchor the building's placement
# pose, both in the map's frame: on a circular orbit their radii are equal, so only the
# distance between the two shows a marker frozen at placement. peri_r is where the
# sampled ring starts (its periapsis) and gate_r the radius its size gate uses; drawn
# is how many marks a map WINDOW drew that frame -- without it the row proves the maths
# but not that a map draws it.
MAPBLDG_RE = re.compile(
    r"\[mapbldg\]\s+t=([\d.]+)s\s+focus=(\S+)\s+id=(\S+)\s+kind=(\w+)\s+"
    r"r=([-\d.]+)\s+pts=(\d+)\s+peri_r=([-\d.]+)\s+gate_r=([-\d.]+)\s+"
    r"pos=([-\d.]+),([-\d.]+),([-\d.]+)\s+anchor=(none|[-\d.]+,[-\d.]+,[-\d.]+)\s+"
    r"drawn=(-?\d+)"
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
    # prg=/nrm= print "-" when the state has no such direction (#870). A row that
    # carries nan instead matches NEITHER branch, so it stays a dropped row that
    # #408's note names.
    r"(?:prg=\(([-\d.]+),([-\d.]+),([-\d.]+)\)|prg=-)\s+"
    r"(?:nrm=\(([-\d.]+),([-\d.]+),([-\d.]+)\)|nrm=-)\s+"
    # rdl is the +/- radial pair (#892). No dash arm: the line from the focus body to
    # the hull always exists, so a row without it is a printer that regressed.
    r"rdl=\(([-\d.]+),([-\d.]+),([-\d.]+)\)"
)
XFER_RE = re.compile(
    r"\[xferlog\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+"
    # The hull the plan was solved FROM (#809). The line used to name only the
    # target, so a plan belonging to a hull the player had left behind was
    # indistinguishable in the log from one they were actually looking at.
    r"ship=\"([^\"]*)\"\s+"
    r"dv_dep=([-\d.e+]+) m/s\s+dv_cap=([-\d.e+]+) m/s\s+total=([-\d.e+]+) m/s\s+"
    r"tof=([-\d.e+]+) s\s+v_inf=([-\d.e+]+) m/s\s+"
    # r_cap is the capture periapsis RADIUS around the target, r_cap_asl the
    # same point as a HEIGHT above its sea level (#405). Both absent = this
    # solution has no capture burn.
    r"(?:r_cap=([-\d.e+]+) m\s+r_cap_asl=([-\d.e+]+) m\s+)?"
    r"burn=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s*(?:retrograde=(\d))?\s*(?:rev=(\d))?"
)
# --relvel-log (render.cpp): what the HUD's two target relative-velocity markers
# were placed FROM (#873). They are billboards in a 3D pass, so this row is the only
# witness that a target of a given KIND reached that code at all -- a station target
# used to draw nothing, silently. tgt_v is the target's speed in the shared inertial
# frame, which is what lets a CHECK test the NUMBER against the target's own conic
# instead of just its presence.
RELVEL_RE = re.compile(
    r"\[relvel\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+kind=(\w+)\s+"
    r"rel_spd=([-\d.e+]+) m/s\s+tgt_v=([-\d.e+]+) m/s\s+"
    r"rel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]"
)
# --tgtmark-log (render.cpp): what the HUD's transfer-target mark was HANDED
# (#1145). A billboard in the 3D pass leaves no other face, but no stdout row can
# prove a GL draw ran, so this one stops proving anything else either: com is the
# target's point in this pass's render frame, r_f that point's distance from the
# FOCUS body's centre (the authored number a case can hold it to -- the Kerbin
# station fixture sits 400 km over a 600 km Kerbin, so 1.0e6 m), cam is the eye,
# dist is eye to target (the marker's screen size is indifferent to it by design),
# dir is the normalized vector the billboard itself received -- read back off the
# marker, as [planelog]'s rdl= is -- and drawn says whether the guard above the
# draw let it happen at all. dir DASHES when drawn=0, so an absence parses as an
# absence rather than as the stale vector the marker held last frame.
TGTMARK_RE = re.compile(
    r"\[tgtmark\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+kind=(\w+)\s+"
    r"com=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\] m\s+r_f=([-\d.e+]+) m\s+"
    r"cam=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\] m\s+dist=([-\d.e+]+) m\s+"
    r"dir=(?:\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]|-)\s+drawn=([01])"
)
PORKCHOP_RE = re.compile(
    r"\[porkchop\]\s+t=([\d.]+)s\s+target=\"([^\"]*)\"\s+"
    r"(\d+)x(\d+)\s+dep=\[([-\d.e+]+),([-\d.e+]+)\]s\s+tof=\[([-\d.e+]+),([-\d.e+]+)\]s\s+"
    r"dv_min=([-\d.e+]+) m/s\s+dv_hi=([-\d.e+]+) m/s\s+"
    r"t_dep_min=([-\d.e+]+) s\s+tof_min=([-\d.e+]+) s"
)
# #745: the line gained a map type and level, and the temperature layer's own columns.
# A temperature map reports no albedos at all (its base pixels are RAMP COLOURS), so those
# columns dash and read as None; a world the model cannot price dashes the temperature
# columns and carries a note naming which absence that is (#726/#727).
SURFMAP_RE = re.compile(
    r"\[surfmap\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+"
    r"(\d+)x(\d+)\s+map=(\S+)\s+level=(\S+)\s+"
    r"albedo=(?:\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]|-)\s+"
    r"shaded=(?:\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]|-)\s+shade=(on|off)\s+"
    r"ocean=(on|off)\s+sites=(\d+)"
    r"(?:\s+temp=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+temp_ends=\[([-\d.]+) ([-\d.]+)\]\s+"
    r"temp_ramp=\[([-\d.]+) ([-\d.]+)\]\s+"
    # #794: the authored phase lag, printed DASHED when no file states one (an absence, never a
    # zero), and where the picture's hottest pixel actually sits. peak_deg is degrees EAST of
    # this bake's subsolar meridian; sun_lon is that meridian in the grid's own degrees. Together
    # they hold the ENGINE to what the law alone cannot show: with no lag the hot spot is under
    # the star, with a lag it is in the afternoon, and as the sim advances the pattern rides with
    # the sun instead of being pasted to a longitude. A flipped hour angle MIRRORS the picture
    # rather than shifting it, and every other number in the row would still look plausible.
    # peak_deg is printed with %+.2f, so its sign is ALWAYS there: a class of [-\d.]+ would eat
    # a leading '-' and choke on a '+', which is the trap this file's other parsers already
    # document above -- and it only bites on rows where the peak happens to sit west of noon.
    r"lag_tau=([-\d.]+)\s+peak_deg=([-+\d.]+)\s+sun_lon=([-\d.]+)\s+"
    # #793: the ground's height, present only where the body's own air profile lapses. The two
    # floors/ceilings are the elevations this grid actually coloured (m above the sea-level datum)
    # and the two kelvin figures are what the profile is worth there -- so a case can tie the
    # picture's cold end to a named mountain instead of trusting temp_ramp's arithmetic.
    # #828: relief_terms= is the coldest and warmest term ANYWHERE between those floors, which is
    # what the palette is priced from. It equals the two figures left of it exactly when the
    # profile never warms with height; on an inversion (a tropopause under a stratosphere, with
    # terrain reaching past it) the coldest ground is the tropopause, not the summit, and the pair
    # comes apart. Printing both is what lets a case see that difference instead of assuming it.
    # In the SAME group as relief=, for the reason spelled out below at wind_: these tails are
    # positional, and a row that printed one without the other is a broken row, not an old build.
    r"(?:relief=\[([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"relief_terms=\[([-\d.]+) ([-\d.]+)\]\s+)?"
    r"temp_mean4=([-\d.]+)\s+datum_mean4=([-\d.]+)\s+anchor=([-\d.]+))?"
    r"(?:\s+temp=-\s+note=\"([^\"]*)\")?"
    # #769: the wind map's tail. wind= is what the sweep painted over this body's GROUND,
    # wind_ends= what its file states at its rows, wind_ramp= the span the picture's colours
    # and arrow lengths actually run over (priced from the ground's floor and ceiling, NOT the
    # file's rows -- otherwise Venus paints its whole world in the bottom 3% of the palette),
    # wind_rows= how many rows that is, and wind_cells= how many arrow cells the vector half
    # drew -- the only observable the arrows have. Same dash law as temp=: a body with no wind
    # prints wind=- plus a note saying which absence ("no air" and "air the file states no wind
    # for" are different news, and a render rim is not air).
    # #772/#777: WHICH rows drew those arrows. A table's bands cover the globe (the loader
    # insists) without landing neatly on the sweep's grid, so surfmapArrowRows() draws one row
    # per band when the bands are many enough and spaced enough and falls back to the uniform
    # grid otherwise. wind_cells= counts arrows either way, so without these three columns no
    # case can tell the two paths apart -- and a body like Uranus, whose belts come from the
    # #775 generator, has 13 of them. They are in the SAME group as wind=, not their own:
    # src/surfmap.cpp prints all eight columns in one row, so a tail that stops halfway is a
    # broken row, not a wind row from an older build. Splitting them let a half-parsed row
    # read as a plan-less wind row, which is how a case would silently lose its pins.
    r"(?:\s+wind=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+wind_ends=\[([-\d.]+) ([-\d.]+)\]\s+"
    r"wind_ramp=\[([-\d.]+) ([-\d.]+)\]\s+"
    r"wind_rows=(\d+)\s+wind_cells=(\d+)"
    r"\s+wind_bands=(\d+)\s+wind_arrow_rows=(\d+)\s+wind_rows_from=(grid|bands))?"
    r"(?:\s+wind=-\s+note=\"([^\"]*)\")?"
    # #839: did THIS bake sample the ground's heights, or reuse the previous bake's grid for the
    # same body at the same size? The column prints after every tail (just before sweep_ms=),
    # because the tails above are positional and a column in the middle hides them -- the same
    # reason sweep_ms= sits last. `-` is an absent QUESTION, not a miss: the picture reads no
    # height at all, which since #841 means a banded body (band table, not noise), a refused
    # field, or a temperature map on air whose profile does not lapse.
    # This group is deliberately NOT optional: every row carries the column, so a row that lost
    # it stops matching here (the case goes red for a missing bake) instead of reading as a row
    # that never had one. anchor_src= is #687's provenance, which no case reads; it is spelled
    # out only so the match can reach past it -- the temp tail ends at anchor= and the provenance
    # sits between that and this column.
    r"(?:\s+anchor_src=\"[^\"]*\")?"
    r"\s+elev_cache=(hit|miss|-)"
)
# --surfmap-log's SECOND tag: the sites the Surface Map window actually marks (src/gameui.cpp),
# published on a 1 s gate while the window is open. [surfmap]'s sites= counts the VAB_LAUNCH_PAD
# filter during the BAKE (src/surfmap.cpp) and says nothing about this loop -- #1133's symptom
# was a pad that had stopped appearing on the map, and a count from the other file cannot see a
# draw loop that draws nothing. Every row carries the count, and a body with no marks prints one
# marks=0 row, so a set that vanishes mid-run cannot hide behind the rows an earlier frame
# printed. lon/lat are the degrees the mark's pixel comes from (equirectLonLat of the pad's dir),
# not the pixel: px belongs to the player's pan and zoom. lon unwraps to [0, 360), so an authored
# west-negative Earth site prints one turn later (alcantara -44.4175 -> 315.5825), the same
# spelling tests/test_buildings.cpp documents.
SURFMARK_RE = re.compile(
    r"\[surfmark\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+marks=(\d+)"
    r"(?:\s+id=\"([^\"]*)\"\s+lon=([-\d.]+) deg\s+lat=([-\d.]+) deg)?"
)
# --solar-log (src/cli.cpp's flag, printed by the kGateSolar block in tick.cpp): the
# star as the world states it, and the ground under the hull as the world states it.
# A dashed albedo or bb_T is NOT a zero -- it means that body authors no bond albedo,
# which is #722's whole complaint about the fictional half, and #726's argument that
# "absent" needs its own spelling. This is also the only end-to-end read of
# src/solar.h, so a CHECK can re-derive #687's black-body balance in python and fail
# if the C++ ever spells sigma or the divide-by-four differently (the #707 worry).
SOLAR_RE = re.compile(
    r"\[solar\]\s+t=([\d.]+)s\s+star=\"([^\"]*)\"\s+dist=([-\d.e+]+|-) m\s+"
    r"flux=([-\d.e+]+|-) W/m2\s+elev=([-\d.]+|-) deg\s+flat=([-\d.e+]+|-) W/m2\s+"
    r"body=\"([^\"]*)\"\s+albedo=([-\d.]+|-)\s+ground_T=([-\d.]+|-) K\s+bb_T=([-\d.]+|-) K"
    # #724 rung C: the ground UNDER THIS HULL right now, and the knob that prices the swing.
    # Absent is a dash rather than a number: an unpriced knob has no default (groundtemp.h),
    # so a body whose source states no night/day pair prints '-' and a CHECK must ask for
    # None, exactly like the albedo column above.
    r"\s+underfoot_T=([-\d.]+|-) K"
    # #801: WHICH rung of the ground law priced that number -- "stated ground temperature",
    # "the air's datum", "DERIVED: sun + albedo", or a file's own provenance string. It sits
    # next to the kelvin it describes and dashes with it, and it is what tells a reader that
    # on most of a generated system underfoot_T is our arithmetic and not a source's reading
    # (science/801 counts how many anchors that is in the shipped file).
    r"\s+underfoot_src=(\"[^\"]*\"|-)"
    r"\s+swing=([-\d.]+|-)\s+lag=([-\d.]+|-)"
    r"\s+air_T=([-\d.]+|-) K"
    # #837: lag is the third knob of the same law, and the row's underfoot_T is a LAGGED
    # answer whenever it is a number. Dash = the file states no lag (NOT a stated zero, which
    # prints 0.0000 and means "this ground peaks at noon"); same rule as the dossier's
    # `ground lag` row (#830) and the surfmap log's lag_tau=.
    # #724 rung D: the profile's own temperature at this altitude and the weight the column
    # supplies, so the row closes as arithmetic (air_T = prof_T + follow * w * (underfoot_T -
    # prof_T)) instead of being admired. Both dash with the air they describe.
    r"\s+prof_T=([-\d.]+|-) K\s+w=([-\d.]+|-)\s+follow=([-\d.]+|-)"
)
# --slew-log (vehicle.cpp slew_log): the attitude target the autopilot steers
# toward, in the ship's own frame. Pairs with --plane-log's n_orb: a prograde
# hold that aims at the ground's motion instead of the orbit's shows up as a
# target sitting off the plane.
# alpha/alpha_full are the two pointing authorities of #608: what the actuators can
# lever THIS tick (TVC rides the armed thrust, so a coasting TVC-only hull reads 0),
# and what the ignited stage could lever at full throttle. Both are needed to tell
# "this hull cannot turn" from "this hull is not burning".
# tau_ask/tau_spent are #599's pair on the TORQUE side: the N m the stick and the latched
# laws summed for the last actuated substep, and what the wheels, the gimballed nozzles
# and the control surfaces actually delivered -- measured off the forces those passes
# bill, not off the allocator's routing. tau_ask far above tau_spent is the hardware
# refusing; tau_spent above tau_ask is a gimbal holding its aim from the last substep.
# ask == 0 with E > 0 is a law that has been handed no authority at all. Since #595 the
# thruster ring is one of the channels behind tau_spent, so a coasting pod-only hull
# spends here rather than reading 0 forever.
# w_slew/w_roll/w_third are the hull's rate split on the frame the law drives (#421): the
# axis that closes the error, the nose axis (an uncommanded roll), and the third that
# completes it. Parsed because a case that wants "the actuators MOVED the hull" cannot ask
# it of E alone -- E also rises when nothing acts and the target merely drifts.
# stick= is on every numeric row since #1137: the manual keys' own roll/pitch/yaw demand,
# -1..1, printed whether or not a law is latched. Under a latched law the hand is still an
# asker (SasStickOwnsAxis yields whatever axis is held), so tau_ask on a mode=prograde row
# can be mostly the pilot's -- which 421's "who wins the axis" could not otherwise tell
# apart from a key the input layer ignored.
SLEW_RE = re.compile(
    r"\[slew\]\s+t=([\d.]+)\s+mode=(\S+)\s+stick=\[([-+\d. ]+)\]\s+E=([-\d.]+)\s+rad\s+\([-\d.]+ deg\)\s+"
    r"w_slew=([-\d+.]+)\s+w_roll=([-\d+.]+)\s+w_third=([-\d+.]+)\s+\|w\|=([-\d+.]+)\s+"
    r"alpha=([-\d.]+)\s+alpha_full=([-\d.]+)\s+Ieff=([-\d.]+)\s+"
    r"w_des=([-\d+.]+)\s+w_cap=(\S+)\s+.*?"
    r"tau_ask=([-\d.]+)\s+tau_spent=([-\d.]+)\s+.*?"
    r"nose=\[([-\d.+]+) ([-\d.+]+) ([-\d.+]+)\]\s+"
    r"target=\[([-\d.+]+) ([-\d.+]+) ([-\d.+]+)\]"
)
# The SAME --slew-log prints a different row while kill-rot is the latched mode
# (vehicle.cpp:3948): the law has no target, so there is no error E, no alpha and no
# nose/target pair to quote -- only the rate it is fighting and the torque pair. It is
# parsed into the same list (mode="killrot") because the question a kill-rot case asks
# is the one #595's case asks: did a latched law ASK, did the hardware SPEND, and did
# the hull RESPOND. #677 was invisible to the harness until now -- kill-rot rows were
# not parsed at all, so a case could not tell "asked for nothing forever" from "no
# rows". Keys the row does not print are None: a case that mixes modes has to name the
# mode it means.
KILLROT_RE = re.compile(
    r"\[slew\]\s+t=([\d.]+)\s+mode=killrot\s+stick=\[([-+\d. ]+)\]\s+\|w\|=([-\d.]+)\s+rad/s\s+"
    r"w=\[([-+\d. ]+)\]\s+tau_ask=([-\d.]+)\s+tau_spent=([-\d.]+)"
)
# ... and a third while ONLY the manual stick is held (#1130): no latched law, so again
# no E, no alpha and no nose/target pair -- but the stick's own ask, the rate, and the
# same torque pair. Before this row the part of a turn the player causes by hand was the
# part the trace could not see, and #1130's dial question had to be inferred from the
# spin still standing three seconds later. stick= is roll / pitch / yaw, -1..1.
STICK_RE = re.compile(
    r"\[slew\]\s+t=([\d.]+)\s+mode=stick\s+stick=\[([-+\d. ]+)\]\s+"
    r"\|w\|=([-\d.]+)\s+rad/s\s+"
    r"w=\[([-+\d. ]+)\]\s+tau_ask=([-\d.]+)\s+tau_spent=([-\d.]+)"
)
# --perf's frame-cost record (#676): one row per window, every number a per-frame
# average. Parsed so a build can be DIFFED against a build instead of read as
# prose at exit. `win=` is in the row because the window is gated on wall seconds
# and cannot close mid-frame, so windows are not equal length when the frame rate
# moves. What a case may assert is the SHAPE -- the six phases sum to `work`, and
# render's five halves sum to `render`, both true by construction on any machine.
# Absolute costs are NOT assertable: these are wall-clock readings off a shared
# box, and a timing CHECK would be flaky and would teach people to ignore failing
# cases (the ticket's own warning).
PERF_KEYS = ("t", "win", "frames", "fps", "work", "events", "logic", "jobs",
             "refresh", "render", "present", "paint", "composite", "ui",
             "imgui_gl", "tail", "ticks", "ms_per_tick", "idled",
             "draws", "worst", "over")
PERF_INT_KEYS = ("frames", "ticks", "idled", "over")
PERF_RE = re.compile(
    r"\[perf\]\s+t=([\d.]+)s\s+win=([\d.]+)s\s+frames=(\d+)\s+fps=([-\d.]+)\s+"
    r"work=([-\d.]+)ms\s+events=([-\d.]+)ms\s+logic=([-\d.]+)ms\s+"
    r"jobs=([-\d.]+)ms\s+refresh=([-\d.]+)ms\s+render=([-\d.]+)ms\s+"
    r"present=([-\d.]+)ms\s+paint=([-\d.]+)ms\s+composite=([-\d.]+)ms\s+"
    r"ui=([-\d.]+)ms\s+imgui-gl=([-\d.]+)ms\s+tail=([-\d.]+)ms\s+"
    r"ticks=(-?\d+)\s+ms/tick=([0-9.]+|-)\s+idled=(\d+)\s+"
    r"draws=([-\d.]+)\s+worst=([-\d.]+)ms\s+over=(\d+)"
)
# Game::toast mirrors every on-screen message to stdout ("the on-screen toast is
# transient, the log is not"), so a hotkey that only changes a flag still leaves
# an observable line. Parsed as plain strings: what a case wants is which
# messages appeared and in what order.
TOAST_RE = re.compile(r"\[toast\] (.*)")
ATT_RE = re.compile(
    r"\[attlog\]\s+t=([\d.]+)s\s+"
    r"nose=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"right=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"up=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"w=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"\|w\|=([-\d.]+) rad/s"
    # #1056: WHICH model wrote the row -- 0 Bullet integrated it, 1 it coasted on its
    # conic, 2 it is a frozen park (pose pinned to the node, rate left over from the
    # world it left). Optional, so the pre-marker grammar still parses and a case that
    # does not care reads None.
    r"(?:\s+railed=(\d))?"
)
EVA_RE = re.compile(
    r"\[evalog\]\s+t=([\d.]+)s\s+mode=(\w+)\s+grounded=(\d)\s+"
    r"pos=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"vel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"face=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+alt=([-\d.]+) m"
    r"(?:\s+viewyaw=([-\d.]+))?(?:\s+rcs=(\d+))?(?:\s+mass=([-\d.]+)kg)?"
    # #1073: the grabbed ladder part's uid (0 = not on a ladder). Optional, so
    # the pre-ladder grammar still parses.
    r"(?:\s+ladder=(\d+))?"
)
DRAINLOG_RE = re.compile(
    r"\[drainlog\]\s+t=([\d.]+)s\s+dt=([\d.]+)s\s+ship=\"([^\"]*)\"\s+"
    r"thrust=([-\d.]+)N\s+(.*)"
)
# --power-log (vehicle.cpp power_log): the whole electrical model in one row.
# gen/const_draw are watts (the RTGs and life support); charge/capacity are
# watt-hours pooled across the ship's battery parts; powered= is the gate the
# reaction wheels live behind -- they are electric, so no power means no
# attitude control. charge is the only one of the four that MOVES, and it moves
# by (gen - const_draw - active) x dt, so active= is that third term printed
# rather than inferred (#643): it is wheelDrawWatts(), so it is what the hull is
# BILLING this substep (0 while coasting), not the drums' rating. A ship whose
# charge sits still is a ship whose battery is decorative (#379).
POWER_RE = re.compile(
    r"\[powerlog\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+powered=(\d+)\s+"
    r"gen=([-\d.]+)W\s+const_draw=([-\d.]+)W\s+charge=([-\d.]+)Wh\s+"
    r"capacity=([-\d.]+)Wh\s+active=([-\d.]+)W\s+net=([-\d.]+)W"
    # #659: the net powerTick balanced on, so a readout can quote a printed figure
    # instead of re-adding the terms. NOT optional: the printer and this parser ship
    # in the same tree, so a row missing the field is a printer that LOST it, and the
    # strict arm makes that a row #408 names rather than a None nobody asked for. Same
    # shape as gen= and active= beside it; a dashed net= would still parse (the bare
    # '-' is what num() reads as no value, #901), which is how the convention stays
    # available if this figure ever needs one.
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
# dv_left is getDeltaV(), the coarse one-burn figure (#41); dv_staged is the
# ship's own staged plan, the number the Vessel Info window shows (#948). They
# differ on a mixed-Isp ship, and only the staged one is monotone under a stage
# press -- which is what case 948 pins.
DV_RE = re.compile(
    r"\[dv\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+mass=([-\d.]+)\s*kg\s+"
    r"hullmass=([-\d.]+)\s*kg\s+thrust=([-\d.]+)N\s+dv_del=([-\d.]+)\s+"
    r"dv_along=([-\d.]+)\s+dv_rocket=([-\d.]+)\s+prop=([-\d.]+)\s*kg\s+"
    r"ve=([-\d.]+) m/s\s+dv_left=([-\d.]+)\s+m/s\s+dv_staged=([-\d.]+)"
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
    r"\[drag\]\s+t=([\d.]+)s\s+alt_asl=([-\d.]+) m\s+rho=([-\d.e+]+) kg/m3\s+"
    r"\|v\|=([-\d.]+) m/s\s+\|F\|=([-\d.]+) N"
    r"(?:\s+\|L\|=([-\d.]+) N)?(?:\s+L_up=([-\d.]+) N)?(?:\s+Up=([-\d.]+))?"
    r"(?:\s+\|tau\|=([-\d.]+) Nm)?"
    r"\s+Cd=([-\d.]+)"
    r"(?:\s+A=([-\d.]+) m2)?(?:\s+(?:AoA=([-\d.e+]+) deg|AoA=-))?"
    r"(?:\s+Chute=([-\d.]+) m2)?"
    # #560: Induced= dashes with the flow fields now, and the case that proves why is
    # flight/560-parked-hull-bills-nothing (a parked skiff billed 0.000..1.442 m2 of
    # induced drag, row to row, against a 0.00 N force -- applyInducedDrag's coefficient
    # is q-normalised, so a jitter flow cancels its own q out of it). Like the other
    # dashed-with-units fields, the dash carries no unit, so it needs its own branch:
    # [-\d.]+ would eat the '-' and float() would throw.
    r"(?:\s+(?:Induced=([-\d.]+) m2|Induced=-))?"
    r"(?:\s+(?:Beta=([-\d.e+]+) deg|Beta=-))?"
    r"(?:\s+(?:CpL=([-\d.e+]+) m|CpL=-))?"
    # The lift pair (#454) prints as a unit -- both numbered or both dashed
    # (src/tick.cpp) -- so it is one optional group, not two.
    r"(?:\s+(?:CLa=([-\d.e+]+) m2/rad|CLa=-)\s+(?:SM=([-\d.e+]+) m|SM=-))?"
    # #455: the lifting/steering surfaces' OWN form-drag CdA, printed last so every
    # older parser keeps working. Before it, that bill was inside A=/Cd= -- mixed
    # into the hull's union at the hull's cd mean -- so a case could not tell "the
    # body is blunt" from "the wings are being charged as plates". #559 dashes it
    # with no usable flow direction, like CpL/SM/FormCp, so the dash is a parseable
    # reading and not a dropped tail.
    r"(?:\s+(?:Profile=([-\d.]+) m2|Profile=-))?"
    # #455: the lever of the COMBINED form drag (hull + surfaces), signed, printed
    # numbered or dashed -- one group, not two, because it prints as one reading.
    r"(?:\s+(?:FormCp=([-\d.e+]+) m|FormCp=-))?"
    # #552 A3: the AIR at the altitude this line names -- what the body's profile says
    # there (src/drag.h through vehicle.h:dragAtm): one constant read for a scale
    # height, a different one per level for a layer table (#552 A4) -- plus the
    # Mach number of the air-relative flow. One group, not four: the game prints all
    # four together or dashes all four (no air at all). Plain units, one scale per
    # field -- K, Pa, m/s, dimensionless -- because the window's "188.0 K (-85.2 C)"
    # gloss is for a human and a parsed field is not.
    # These four need a DIGIT-anchored pattern, unlike the [-\d.]+ used above: those
    # fields carry a trailing unit (" deg", " m"), so a dashed "AoA=-" cannot match the
    # numeric branch by accident. These have no unit, and a bare "-" does match
    # [-\d.]+ -- so the dashed row parsed as T=- and float() threw.
    r"(?:\s+(?:T=(-?\d+(?:\.\d+)?) p=(-?\d+(?:\.\d+)?) a=(-?\d+(?:\.\d+)?)"
    r" Mach=(-?\d+(?:\.\d+)?)"
    r"|T=- p=- a=- Mach=-))?"
    # #694: Ld= -- the air's force split about the air-relative flow, L over D, which is
    # the ratio a glide ratio equals (vehicle.h:lastFlowLift / lastFlowDrag: 7.19 in a
    # settled glide, where the hull's own track says 7.28). Since #699 it is also what
    # |L| over the CHARGED bills gives -- q x (1.2 CdA + Induced + Profile), median 7.17
    # both on flight/492's hold. Over the RAW A=/Cd= product the same recipe reads 7.66:
    # the 6 % is --drag-cd, which the line prints un-multiplied (see the [drag] legend).
    # res/ships/README.md's 8.6 reads the raw product AND predates #699, so it is stale
    # twice over. The field is unitless, so a bare "-" would match the wider character
    # class used above and throw in float(); the numeric branch is digit-anchored, and the
    # dashed alternative is spelled out so a reader sees the row that carries no value.
    r"(?:\s+(?:Ld=(-?\d+(?:\.\d+)?)|Ld=-))?"
    # #947: SideCdA=/SideCp= -- the crossflow normal bill on the parts the along-flow
    # union hides, and its signed lever. One group, not two: tick.cpp prints the pair
    # together (both numbered or both dashed) off the same lastDragAlphaValid gate as
    # AoA, because the reading is meaningless without a flow to be perpendicular to.
    # Printed LAST so every older parser keeps matching -- the same courtesy the
    # Profile=/FormCp=/Ld= tails were extended by. Both fields carry a trailing
    # unit, so the wide [-\d.]+ class is safe here (a bare "-" needs no unit to
    # match and float() to throw -- see the digit-anchored T=/p=/a=/Mach= note).
    r"(?:\s+(?:SideCdA=([-\d.]+) m2 SideCp=([-\d.e+]+) m|SideCdA=- SideCp=-))?"
    # #1097: BodyCdA=/BodyCp= -- the hull's OWN normal force (body lift: authored
    # body_lift x the EXPOSED silhouette x cos a, SIGNED: negative is net downforce)
    # and its CdA-weighted lever. One group like the Side pair (printed together off
    # the same lastDragAlphaValid gate), appended last, both unit-anchored.
    r"(?:\s+(?:BodyCdA=([-\d.e+]+) m2 BodyCp=([-\d.e+]+) m|BodyCdA=- BodyCp=-))?"
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
# #485's [surface] row: one group per lifting or steering part --
# "<label> a=<deg> q=<Pa>[ mach=<n>][ watts=<W>]". The angle is signed into SHIP axes (up
# for a wing or tailplane, right for a fin), NOT each mount's own convention, so a mirrored
# twin pair reads alike and a pair that actually differs is the signal, not the noise.
SURFACE_ITEM_RE = re.compile(
    # #732: the angle carries its own dash when the part met the aero pass but the
    # law has no lift frame to measure it in (a plate broadside to the flow). It has
    # to be a PARSED absence, not a dropped item: every [surface] reader walks these
    # rows by position -- flight/485 and flight/730 both assume the ten items under a
    # [drag] row are the ten parts -- so an unparseable token silently shortens the
    # list and misaligns every later pairing rather than failing.
    r"(\S+) a=(-|[-+]\d+(?:\.\d+)?) q=(\d+(?:\.\d+)?)"
    # #552 A3: this surface's OWN Mach number (its flow speed over the sound speed
    # its air implies). Optional so a run captured before the field still parses.
    r"(?: mach=(\d+(?:\.\d+)?))?"
    # #730: the watts the air is taking off this surface (its own q x its own CdA x its
    # own flow speed, form drag plus the induced price of its lift). Optional so a
    # pre-#730 capture still parses; the #730 case fails loudly when it is None rather
    # than reading a 0 that was never printed.
    r"(?: watts=(\d+(?:\.\d+)?))?"
)
SHAKE_RE = re.compile(
    r"\[shakelog\]\s+t=([\d.]+)s\s+a=([-\d.]+) m/s2\s+"
    r"amp=([-\d.]+) m\s+off=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]"
)
# --cam-log: the camera the render pass left this frame. yaw/pitch are what a
# --sim-mouse drag is supposed to change; rmb/cap say whether the gesture
# reached the camera at all (a part window under the cursor eats the drag).
# #876 added the orientation half: up/lv/roll/pole/gf/camh. Those answer "does the
# picture read", not "where is the turntable": pole is the angle between the
# turntable's yaw axis and the local vertical (90 deg = a left/right drag rolls
# the image instead of orbiting it), roll is how tilted that vertical looks on
# screen, gf is the ground-frame weight that blends the turntable between the
# hull's attitude and the north-up surface frame. lv/roll/pole/ax1/camh dash when
# there is no local vertical to measure against (no ship, or a body focus);
# pole/gf/pin additionally dash outside orbit mode, where ref and orbitPitch are
# whatever the last orbit frame left. roll goes NOISY rather than dashing when
# the view line runs along lv, so a case pins it at a side view, never at the
# poles. hi= never dashes -- it is the turntable's live +side stop, the number
# pin= compares against, so a case can tell "held at the stop" from "past a
# stop that moved" (pitch > hi never happens: ComputeView() pulls it back).
# pan=/aim= are the #919 aim offset: panning moves no part of the position, so a
# case pins a drag as aim moving while dist/yaw/pitch/camh do not. mmb= is the
# MMB-drag gate in events.cpp, the pan twin of rmb=.
CAM_RE = re.compile(
    r"\[camlog\]\s+t=([\d.]+)s\s+mode=(\w+)\s+focus=\"([^\"]*)\"\s+"
    r"yaw=([-+\d.]+) rad\s+pitch=([-+\d.]+) rad\s+dist=([-+\d.]+) m\s+"
    r"fwd=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"up=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]\s+"
    r"(?:lv=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]|lv=\[-\])\s+"
    r"roll=(?:([-+\d.]+) deg|-)\s+"
    r"pole=(?:([-+\d.]+) deg|-)\s+"
    r"ax1=(?:([-+\d.]+) deg|-)\s+"
    r"gf=(?:([-+]\d+\.\d+)|-)\s+"
    r"camh=(?:([-+\d.]+) m|-)\s+"
    r"hi=([-+\d.]+) rad\s+"
    r"pan=(?:\[([-+\d.]+) ([-+\d.]+)\]|-)\s+"
    r"aim=(?:\[([-+\d.]+) ([-+\d.]+)\]|-)\s+"
    r"pin=(hi|lo|-)\s+rmb=(\d)\s+mmb=(\d)\s+cap=(\d)"
    # #1069: fov (deg) -- the frustum angle, so the flight scroll (events.cpp)
    # is pinnable. Optional: a log written before the field existed still parses.
    r"(?:\s+fov=([-\d.]+))?"
)
TERRAIN_RE = re.compile(
    r"\[terrain\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+patches=(\d+)\s+"
    r"deepest=(\d+)\s+max_depth=(\d+)\s+collision=(\d+)\s+"
    r"deep_off=([-\d.e+]+) m\s+cam_r=([-\d.e+]+) m"
)
# aoa prints "-" (no value) outside an atmosphere or at rest in air, so it gets
# its own alternative rather than a wider character class that would eat the "-".
# beta (#450) sits next to aoa in the line because they are one AeroFrame reading, but
# since #787 it dashes on a STRICTER test: the flow must also project onto the hull's
# nose-right plane (constants.h:kBetaIllCondDeg), so a row can carry aoa and dash beta --
# a flow straight up a hull's mast. Nothing may select rows on beta being present and
# then be satisfied by selecting none; a case that does so says so explicitly.
# aprg (#498) is the Game Debug window's Prg (air): nose to AIR-relative prograde. It
# dashes on the same flow floor as aoa (no air, or a flow with no direction), and the
# pair aprg/[orbinfo] prg is the gap between flying through the air and flying the
# orbit.
# cla/sm (#499) are the SURFACE window's stability pair: cla dashes with aoa (no
# air), sm dashes alone when the lifting surfaces stop agreeing, so each needs
# the same two-arm treatment. wind_up (#767) is the third component of the body's
# wind table -- the air along the local vertical -- and dashes on exactly the same
# test as wind_east/wind_north, a body that authors no wind at all. It answers 0
# for every body that does, because no published sheet states a large-scale updraft.
SURF_RE = re.compile(
    r"\[surfinfo\]\s+t=([\d.]+)s\s+alt_agl=([-\d.e+]+) m\s+"
    r"alt_asl=([-\d.e+]+) m\s+vs=([-\d.e+]+) m/s\s+hs=([-\d.e+]+) m/s\s+"
    r"lat=([-\d.e+]+) deg\s+lon=([-\d.e+]+) deg\s+pitch=([-\d.e+]+) deg\s+"
    r"roll=([-\d.e+]+) deg\s+hdg=([-\d.e+]+) deg\s+"
    r"(?:aprg=([-\d.e+]+) deg|aprg=-)\s+"
    r"(?:aoa=([-\d.e+]+) deg|aoa=-)\s+"
    r"(?:beta=([-\d.e+]+) deg|beta=-)\s+"
    r"(?:cla=([-\d.e+]+) m2/rad|cla=-)\s+"
    r"(?:sm=([-\d.e+]+) m|sm=-)\s+"
    r"(?:tas=([-\d.e+]+) m/s|tas=-)\s+"
    r"(?:wind_east=([-\d.e+]+) m/s|wind_east=-)\s+"
    r"(?:wind_north=([-\d.e+]+) m/s|wind_north=-)\s+"
    r"(?:wind_up=([-\d.e+]+) m/s|wind_up=-)\s+"
    r"acc=([-\d.e+]+) m/s2\s+"
    r"body=\"([^\"]*)\"\s+bme=\"([^\"]*)\"\s+sit=\"([^\"]*)\""
)
# [dockwin] is the Docking window's publish (gameui.cpp), one row per kDockReadSec tick
# while the window is open, in whichever of the two forms the gate left the pair in: the
# measurable one carries the dial's figures, every other state -- no arm, no target, a
# hull on the rails, a pair in different frames -- prints its why token alone because
# there is no port-to-port line to plot (#1026-b). The optional group IS that split, so a
# case reading twist= sees None on a refused row rather than losing the row, and
# first()/last() still walk the whole run in line order.
DOCKWIN_RE = re.compile(
    r"\[dockwin\] a=\"([^\"]*)\" b=\"([^\"]*)\" why=([a-z]+)"
    r"(?:\s+align=([-\d.e+]+),([-\d.e+]+)\s+turn=([-\d.e+]+),([-\d.e+]+)"
    r"\s+push=([-\d.e+]+),([-\d.e+]+)\s+drift=([-\d.e+]+),([-\d.e+]+)"
    r"\s+twist=([-\d.e+]+)\s+rng=([-\d.e+]+)\s+gap=([-\d.e+]+)\s+cap=([-\d.e+]+)"
    r"\s+speed=([-\d.e+]+)"
    r"\s+along=([-\d.e+]+)\s+closing=([-\d.e+]+))?"
    r"\s+at=([-\d.]+),([-\d.]+)\s+size=([-\d.]+),([-\d.]+)"
)
# [orbinfo] is the ORBITAL window's own numbers. Nothing parsed it until #498
# needed it: prg/rtg are now the ONLY place the CONIC's prograde angle prints -- the
# window dropped its own Prg/Rtg rows on 2026_11_06, leaving the HUD marker and these
# keys. #498 is precisely that the conic points where the ORBIT is going rather than
# where a plane in an atmosphere is flying. apo_* print "-" for an open conic,
# lan/lpe for a missing node line and prg/rtg for a conic with no prograde to
# measure to (#870), so those get the two-arm treatment; the peri_* trio and
# period print whatever the conic says, so they go through num() -- which reads an
# open conic's inf or nan as None. Left as a float it would slip into a CHECK and
# make every comparison with it quietly false (#901).
ORBINFO_RE = re.compile(
    r"\[orbinfo\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+vel=([-\d.e+]+) m/s\s+"
    r"r=([-\d.e+]+) m\s+"
    r"(?:apo_r=([-\d.e+]+) m|apo_r=-)\s+"
    r"(?:apo_asl=([-\d.e+]+) m|apo_asl=-)\s+"
    r"(?:apo_t=([-\d.e+]+) s|apo_t=-)\s+"
    r"peri_r=([-\d.e+]+) m\s+peri_asl=(\S+) m\s+peri_t=(\S+) s\s+"
    r"period=(\S+) s\s+inc=([-\d.e+]+) deg\s+ecc=([-\d.e+]+)\s+"
    r"sma=([-\d.e+]+) m\s+plane=(\S+)\s+"
    r"(?:lan=([-\d.e+]+) deg|lan=-)\s+"
    r"(?:lpe=([-\d.e+]+) deg|lpe=-)\s+"
    r"(?:prg=([-\d.e+]+) deg|prg=-)\s+"
    r"(?:rtg=([-\d.e+]+) deg|rtg=-)\s+"
    r"energy=([-\d.e+]+) J/kg"
)
# What a --timeout run says about its own sim clock (#402): "the sim advanced X s
# in Y s of wall clock", and the WARNING that follows it when the 10-tick catch-up
# cap threw some of the remaining wall time away. Before that line existed a
# wall-clock run reported only its wall cutoff, so `--timeout 180 --time-accel 4`
# read as 720 s of flight and could quietly have been 437 s of it -- a factor that
# depends on the machine and on how many other runs share it.
SIM_ADVANCED_RE = re.compile(r"the sim advanced ([\d.]+) s in ([\d.]+) s of wall")
SIM_DROPPED_RE = re.compile(r"catch-up cap \([^\)]*\) discarded another ([\d.]+) s")
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

# Every parser that turns ONE stdout tag into a list of rows, as the guard at the CHECKs
# needs it: (namespace name, the regex that reads it, what that regex was fed). These
# patterns are POSITIONAL, so a column inserted mid-line by a printer -- which is a normal,
# reasonable thing for the game to do -- matches nothing, the list comes back empty, and
# `all(...)` over an empty list is TRUE. That has now bitten twice (SURFMAP_RE, SOLAR_RE);
# both times the printer and the regex were fixed in the same commit, and what caught it was
# a case that COUNTS its rows. The near miss to guard against is the case that only walks
# `all(...)`, which would have gone green reading nothing.
ROW_PARSERS = [
    ("orbit", ORBIT_RE, "world"), ("ascent", ASCENT_RE, "world"),
    ("mapb", MAP_RE, "world"), ("mapbldg", MAPBLDG_RE, "world"),
    ("dbg", DBG_RE, "world"), ("bldg", BLDG_RE, "world"),
    ("plane", PLANE_RE, "world"), ("xfer", XFER_RE, "world"),
    ("relvel", RELVEL_RE, "world"), ("tgtmark", TGTMARK_RE, "world"),
    ("porkchop", PORKCHOP_RE, "world"), ("surfmap", SURFMAP_RE, "world"),
    ("surfmark", SURFMARK_RE, "world"),
    ("solar", SOLAR_RE, "world"), ("slew", SLEW_RE, "world"), ("perf", PERF_RE, "out"),
    ("toast", TOAST_RE, "out"), ("att", ATT_RE, "world"), ("eva", EVA_RE, "world"),
    ("drainlog", DRAINLOG_RE, "world"), ("power", POWER_RE, "world"), ("dv", DV_RE, "world"),
    ("mass", MASS_RE, "world"), ("drag", DRAG_RE, "world"), ("shake", SHAKE_RE, "world"),
    ("cam", CAM_RE, "world"), ("terrain", TERRAIN_RE, "world"), ("surf", SURF_RE, "world"),
    ("orbinfo", ORBINFO_RE, "world"), ("dockwin", DOCKWIN_RE, "world"),
    ("scene", SCENE_RE, "out"), ("fuel", FUEL_RE, "world"),
]


def line_tag(rx):
    """The '[tag]' a parser's regex starts with, or None when it does not start with one
    (an item-level regex reads inside a line, not a line of its own -- none of those live in
    ROW_PARSERS, so today every entry has a tag; the None path is for one that does not)."""
    m = re.match(r'\[([a-z]+)\]', rx.pattern.replace("\\[", "[").replace("\\]", "]"))
    return None if m is None else "[%s]" % m.group(1)


def dropped_nonfinite_rows(rx, printed):
    """Of the lines the game printed for one parser, the ones it could not read because
    a number in them is spelled inf or nan (#408). Every row parser is anchored on its
    tag (line_tag), so match() asks the same question finditer does at that offset."""
    return [ln for ln in printed if rx.match(ln) is None and NONFINITE_RE.search(ln)]


def num(s):
    """ONE spelling, for every row parser, of "this field carries no number" (#901).

    An absent group (a field the row format does not have) is None, a dashed field is
    None, and a token that is not a FINITE number is None too. That last case is the one
    the strict numeric arms never produce -- their character class holds no letters -- so
    it only arrives through a lenient any-token arm, where float() happily answers inf or
    nan. A CHECK against such a float is quietly false in every direction, which is the
    hole #408 was filed for; reading it as None makes the same CHECK fail or raise
    instead. The row still parses, so #408's dropped-row naming does NOT fire here -- the
    loud answer is the CHECK itself, and a case that wants the row named should be using
    a strict arm."""
    if s is None:
        return None
    try:
        f = float(s)
    except ValueError:
        return None
    return f if math.isfinite(f) else None


# Every log row describes the vessel that was ACTIVE when it printed, and the game says
# who that is -- four spellings, all on their own line: the handoff (game.cpp:1671), the
# handoff after a removal (:2500), the removal that left the choice alone (:2516, which
# prints "(none)" when the game holds vessels and none of them is active), and the
# removal that left nothing to control (:2509). A marker carries no timestamp, so a
# row's own t= cannot say which side of a switch it fell on; line order can, and losing
# it is what turned a ship-switch bug into an experiment (#447: a row at t=2.0s next to
# a press at 2000 ms and a 0.25 s sampling interval).
SHIP_SWITCH_RE = re.compile(
    r"(?:Active ship \d+ of \d+: |Removed '[^']*'; active ship \d+ of \d+: )"
    r"([^\n]*)"
    r"|Removed '[^']*' \(active unchanged: (.*)\)"
    r"|Removed '[^']*'; nothing left to control -- no active vessel")
NO_VESSEL = ""


def marker_name(m):
    """The vessel a marker names -- whichever of its two shapes carried one. ''
    (NO_VESSEL) for the shipless game and for 'active unchanged: (none)', which is
    the same state in the game's own words. The third alternative's closing paren is
    part of the pattern, not the name: a display name may hold a ')' itself."""
    name = m.group(1) if m.group(1) is not None else m.group(2)
    return NO_VESSEL if name is None or name.strip() == "(none)" else name.strip()


def ship_segments(text):
    """[(ship, chunk)] split at the active-ship markers, in line order. The marker
    belongs to the vessel it names, so each chunk starts at its marker. `ship` is None
    before the first marker: the run never switched, so those rows describe the ship it
    started with, which is the ship the case chose."""
    marks = list(SHIP_SWITCH_RE.finditer(text))
    if not marks:
        return [(None, text)]
    parts = []
    if marks[0].start():
        parts.append((None, text[:marks[0].start()]))
    for i, m in enumerate(marks):
        end = marks[i + 1].start() if i + 1 < len(marks) else len(text)
        parts.append((marker_name(m), text[m.start():end]))
    return parts


def parse_scoped(text, parser):
    """Run a row parser over the log one vessel at a time so every dict row knows which
    ship was active when the game printed it: row["ship"] (#447). Rows keep their line
    order across the segments. A parser whose own line already names the ship -- [fuel],
    [mass], [drainlog] all print ship= -- keeps that name, because the line is the more
    precise source: it says which vessel the ROW is about, where the marker only says
    which one was active. No row regex spans a line, so splitting at a marker boundary
    cannot cut a row in half."""
    rows = []
    for ship, chunk in ship_segments(text):
        for row in parser(chunk):
            if isinstance(row, dict) and "ship" not in row:
                row["ship"] = ship
            rows.append(row)
    return rows


def parse_shipswitch(text):
    """The markers themselves as rows, so a case can say how many handoffs happened and
    to whom instead of regexing stdout. This is the only row list that is not a [tag]
    table, so it is deliberately not in ROW_PARSERS: the printed-vs-read guards there
    count lines that start with a tag."""
    rows = []
    for m in SHIP_SWITCH_RE.finditer(text):
        line = m.group(0)
        rows.append({
            "name": marker_name(m),
            "verb": ("switch" if line.startswith("Active ship") else
                     "none" if "nothing left to control" in line else
                     "unchanged" if "active unchanged" in line else "removed"),
        })
    return rows


# A tag is not a row. The same tags also carry NOTES, and no parser above can read them:
# "[solar] WARNING: ..." (system.cpp:2459,2481), "[mapdump] no active ship, nothing to dump"
# (gameui.cpp:7969), "[porkchop] ... no-solution", "[xferlog] ... no-solution" and
# "[porkchop] sweep FAILED" (transferplanner.cpp:111,364,552), "[scene] ... push refused" and
# "... pop refused" (scene.cpp:221,250). The empty-list guard counts printed rows, so it must
# not count these: a case whose list is legitimately empty would be told its regex and the
# printer disagree, and a wrong diagnosis costs more than no guard at all.
NOTE_WORDS = re.compile(r"\b(WARNING|FAILED|refused|no-solution|no active ship)\b")
# A number that blew up prints "inf" or "-nan", and every number field in the parsers
# above is [-\d.e+]+ -- it takes the sign and stops at the letters, so the field never
# closes and the whole row fails to match and disappears: len(<list>) comes up short,
# last() is an older row, and the CHECKs assert about the rows that survived (#408).
# Only a line the parser COULD NOT read is the finding here, and no row is exempt any
# more: [ascent] used to be the exception -- it printed "nan" for "no node yet" and its
# regex read that, so the row was neither dropped nor named. #894 moved those fields onto
# the dash, so a nan in one of them now matches neither arm and IS named. The lookaround
# is not decoration -- "--info-log", "information" and "inflight" merely start with these
# letters.
NONFINITE_RE = re.compile(r"(?<![A-Za-z0-9_.])[-+]?(?:inf|nan)(?![A-Za-z0-9_])",
                          re.IGNORECASE)
# A CHECK's own string literals, masked before a parser name is looked for in the expression.
# "orbit" inside `last(cam)["mode"] == "orbit"`, `[mass]` inside a case's own regex and
# "Drag power" on a wing card all name a parser's word without that case walking the list --
# and a drift in the parser would land its diagnosis on the wrong case.
LIT_STR_RE = re.compile(r"\"[^\"]*\"|'[^']*'")

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

# The --ui-* flags that schedule an action, and the log tag the game reports that
# action under (uiinput.cpp: the press at :481, the refusals at :182/:188, the
# typed echo at :360). A case that schedules an action the game never reported did
# not run the story its CHECKs weigh (#860), so the runner pairs them up.
UI_ACTION_FLAGS = {"--ui-click": "uiclick", "--ui-scroll": "uiscroll",
                   "--ui-type": "uitype"}


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


def check_check_shapes(exprs, case_path):
    """A CHECK is ONE expression. `CHECK cond, "and why"` parses to a tuple,
    and a non-empty tuple is always true, so the assertion passes whatever the
    game did (#650). Refuse it where the case is parsed -- every commit reaches
    this through e2e/index.py --check -- rather than at eval time, after the
    game has been booted for it. Its sibling shapes go with it, because they
    are the same dead assertion one keystroke away: a bare list/set/dict
    literal (true whenever it is not empty), a tuple parked inside an
    `and`/`or`, and an UNCALLED lambda -- `CHECK (lambda e: cond)` evaluates to
    a function, which is truthy, so the case passes whatever the game printed.
    (#735 shipped one: the CHECK was written for one argument and its argument
    list was edited away, leaving the call off. It was green over a correct run
    and, when tested, green over a wrong one.) Calling it -- `CHECK (lambda e:
    cond)(re.search(...).group(0))` -- is the shipped idiom and stays legal.
    Comprehensions stay legal -- `CHECK {r["t"] for r in rows if
    r["x"]}` really does mean "some rows matched". Deliberately narrow
    otherwise: it does NOT demand a bool, because 58 shipped CHECKs are
    `re.search(...)`, which hands back a Match and means the right answer
    anyway. A tuple that only appears at run time (`CHECK first(pairs)`) is
    caught by check_verdict below."""
    def dead_container(node):
        if isinstance(node, ast.Tuple):
            return "a tuple (a trailing comma makes one)"
        if isinstance(node, (ast.List, ast.Set, ast.Dict)):
            return "a container literal"
        if isinstance(node, ast.Lambda):
            return "a lambda with nothing passed to it"
        return None

    name = os.path.basename(case_path)
    for expr in exprs:
        try:
            tree = ast.parse(expr, mode="eval")
        except SyntaxError as e:
            raise ValueError("%s: CHECK does not parse: %s (%s)"
                             % (name, e.msg, expr[:60]))
        why = dead_container(tree.body)
        if why is None and isinstance(tree.body, ast.BoolOp):
            for val in tree.body.values:      # (a, b) and "a note"
                why = dead_container(val)
                if why is not None:
                    break
        if why is not None:
            raise ValueError("%s: CHECK is %s, not one assertion -- it is "
                             "ALWAYS true: %s" % (name, why, expr[:60]))


# EXPECT and FORBID are substring tests (`s in out` above), never patterns. That is
# deliberate -- the game's stdout is the spec, and a substring is a fact -- but it
# leaves a trap: a line written as a regex is not wrong-shaped, it is simply DEAD.
# #830's review found five shipped FORBIDs of that kind (`ground lag\s+0\.000 rad`,
# `lag_tau=[0-9]`), each of them green over the very thing it meant to forbid. A
# backslash in stdout text is essentially never real output, and a bracket group
# spelling a character class (not a literal like "[v0]" or "temp_ramp=[0.00 0.00]")
# is likewise a pattern wearing literal clothes.
REGEX_SHAPED_RE = re.compile(
    r"\\[sSwWdDb.+*?|{}]|[()]\*|\[[\da-zA-Z]*-[\da-zA-Z]*\]|\[\\[dsw]\]"
)


def check_literal_shapes(expect, forbid, case_path):
    name = os.path.basename(case_path)
    for key, texts in (("EXPECT", expect), ("FORBID", forbid)):
        for text in texts:
            if REGEX_SHAPED_RE.search(text):
                raise ValueError(
                    "%s: %s is a literal substring, not a regex -- this line can "
                    "never match. Write the exact text, or move the pattern into a "
                    "CHECK (re.search / re.findall). Offending line: %s"
                    % (name, key, text[:80]))


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
    check_check_shapes(check, path)
    check_literal_shapes(expect, forbid, path)
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
            "apo_r": num(apo_r), "apo_asl": num(apo_asl),
            "inc": float(inc), "T": float(T),
            "ttAp": float(ttAp), "ttPe": float(ttPe), "h": float(h),
            "E": float(E),
            # None on a line without the #171 tail.
            "plane": plane, "lan": lan, "lpe": lpe,
        })
    return rows


def parse_ascent(out):
    rows = []
    for m in ASCENT_RE.finditer(out):
        (t, phase, lever, pitch, peri, apo, inc_eq, node, accel, gpull,
         why, alt_asl, v_orb, q, gamma, dv_exp, grav, drag, steer, burns) = m.groups()
        # A dashed field captures as an absent group, so it reads as None: no number
        # for this tick, which is neither 0 nor nan (#894).
        row = {
            "t": float(t), "phase": phase, "lever": float(lever),
            "pitch": float(pitch), "peri_asl": num(peri), "apo_asl": num(apo),
            "inc_eq": num(inc_eq), "node_r": num(node), "why": why,
        }
        # None when the row predates the field, so a CHECK can ask for it and
        # fail loudly on an old-format row rather than compare against None.
        row["accel"] = num(accel)
        row["g"] = num(gpull)
        # #675's profile. Same rule: None on a pre-#675 row. gamma is None on a row
        # that HAS the fields and no angle to report -- a hull below orbit.h's conic
        # speed floor -- which is not the same claim as None above, so a CHECK that
        # means "the row is old" should test a field that never dashes (alt_asl).
        for key, val in (("alt_asl", alt_asl), ("v_orb", v_orb), ("q", q),
                         ("gamma", gamma), ("dv_exp", dv_exp), ("grav", grav),
                         ("drag", drag), ("steer", steer)):
            row[key] = num(val)
        row["burns"] = None if burns is None else int(burns)
        rows.append(row)
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
         nx, ny, nz, ticks, px, py, pz, sx, sy, sz, dx, dy, dz) = m.groups()
        rows.append({
            "t": float(t), "frame": frame, "plane": plane_name,
            "inc": float(inc),
            "n_orb": (float(ox), float(oy), float(oz)),
            "n_ref": (float(rx), float(ry), float(rz)),
            # n_ref x n_orb: the ascending node (orbit.h's convention).
            "node": (float(nx), float(ny), float(nz)),
            # "-" when the planes are coplanar: no node to point at.
            "ticks": ticks,
            # The prograde marker and the normal axis (the + side, like n_orb), unit,
            # as the billboards were placed. None when the state has no such direction
            # to place them from (#870): a hull resting on a world that does not turn
            # has a conic speed of solver jitter, and the markers are absent rather
            # than pointing at wherever that jitter happens to fall.
            "prg": None if px is None else (float(px), float(py), float(pz)),
            "nrm": None if sx is None else (float(sx), float(sy), float(sz)),
            # The radial pair (#892): the line from the focus body to the hull, unit,
            # lifted into the frame the row says it is drawing in. In a bubble frame this
            # is NOT the hull's position in that frame -- the bubble's origin is the
            # building, so what that position points at depends on where the building
            # parked the hull, not on where the planet is.
            "rdl": (float(dx), float(dy), float(dz)),
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


def parse_bldg(out):
    rows = []
    for m in BLDG_RE.finditer(out):
        (t, bid, px, py, pz, r, live) = m.groups()
        rows.append({
            "t": float(t), "id": bid,
            "pos": (float(px), float(py), float(pz)), "r": float(r),
            "live": int(live),
        })
    return rows


def parse_mapbldg(out):
    rows = []
    for m in MAPBLDG_RE.finditer(out):
        (t, focus, bid, kind, r, pts, peri_r, gate_r, px, py, pz, anc, drawn) = m.groups()
        rows.append({
            "t": float(t), "focus": focus, "id": bid, "kind": kind,
            "r": float(r), "pts": int(pts), "peri_r": float(peri_r),
            "gate_r": float(gate_r),
            "pos": (float(px), float(py), float(pz)),
            # "none" for an air platform: its pose is the wind law, so there is no
            # placement anchor to compare the mark against.
            "anchor": None if anc == "none" else tuple(float(v) for v in anc.split(",")),
            "drawn": int(drawn),
        })
    return rows


# Every shape --slew-log prints is parsed into ONE list, so a case that mixes modes has to
# name the mode it means. Keys a row does not print are None rather than absent: a missing
# key makes a case's arithmetic raise on one mode and pass on another with no clue which.
# One builder, because three near-identical literals are how a new column gets added to
# two shapes and silently missed on the third.
SLEW_ROW_KEYS = ("E", "w_slew", "w_roll", "w_third", "wnorm", "alpha", "alpha_full",
                 "Ieff", "w_des", "w_cap", "tau_ask", "tau_spent", "nose", "target",
                 "wxyz", "stick")


def _slew_row(t, mode, **kw):
    row = {"t": float(t), "mode": mode}
    row.update(dict.fromkeys(SLEW_ROW_KEYS, None))
    row.update(kw)
    return row


def parse_slew(out):
    rows = []
    for m in SLEW_RE.finditer(out):
        (t, mode, ask, e, w_slew, w_roll, w_third, wmag, alpha, alpha_full, ieff,
         w_des, w_cap, tau_ask, tau_spent, nx, ny, nz, tx, ty, tz) = m.groups()
        rows.append(_slew_row(
            t, mode,
            E=float(e),
            w_slew=float(w_slew), w_roll=float(w_roll), w_third=float(w_third),
            wnorm=float(wmag),
            alpha=float(alpha), alpha_full=float(alpha_full), Ieff=float(ieff),
            # What the law wants and the ceiling on it (#678). w_cap prints "-" when
            # unlimited: a 0 there would read as a law forbidden to move, so it parses
            # to None rather than to 0.0.
            w_des=float(w_des),
            w_cap=None if w_cap == "-" else float(w_cap),
            tau_ask=float(tau_ask), tau_spent=float(tau_spent),
            nose=(float(nx), float(ny), float(nz)),
            target=(float(tx), float(ty), float(tz)),
            # The hand on the axis alongside the law (#1137) -- the row cannot say which
            # of tau_ask is whose, but it can say whether anyone was holding a key.
            stick=tuple(float(v) for v in ask.split()),
        ))
    # Kill-rot rows, same list, keys the row does not print set to None.
    for m in KILLROT_RE.finditer(out):
        t, ask, wmag, wxyz, tau_ask, tau_spent = m.groups()
        rows.append(_slew_row(
            t, "killrot",
            wnorm=float(wmag),
            tau_ask=float(tau_ask), tau_spent=float(tau_spent),
            wxyz=tuple(float(v) for v in wxyz.split()),
            stick=tuple(float(v) for v in ask.split()),
        ))
    # Stick-only rows (#1130), same list and same reasoning: the row carries no law state,
    # so every law key is None and a case that mixes modes has to name the mode it means.
    for m in STICK_RE.finditer(out):
        t, ask, wmag, wxyz, tau_ask, tau_spent = m.groups()
        rows.append(_slew_row(
            t, "stick",
            wnorm=float(wmag),
            tau_ask=float(tau_ask), tau_spent=float(tau_spent),
            wxyz=tuple(float(v) for v in wxyz.split()),
            stick=tuple(float(v) for v in ask.split()),
        ))
    rows.sort(key=lambda r: r["t"])
    return rows


def parse_perf(out):
    rows = []
    for m in PERF_RE.finditer(out):
        r = dict(zip(PERF_KEYS, m.groups()))
        for k in PERF_KEYS:
            if k == "ms_per_tick":
                continue
            r[k] = int(r[k]) if k in PERF_INT_KEYS else float(r[k])
        # ms/tick prints "-" when the window ran no ticks (a paused run): a rate
        # over zero ticks is not a number, so it parses to None and not to 0.0 --
        # the same spelling and the same reason as w_cap in parse_slew (#678).
        r["ms_per_tick"] = None if r["ms_per_tick"] == "-" else float(r["ms_per_tick"])
        rows.append(r)
    rows.sort(key=lambda r: r["t"])
    return rows


def parse_toast(out):
    return [m.group(1).strip() for m in TOAST_RE.finditer(out)]


def parse_xfer(out):
    rows = []
    for m in XFER_RE.finditer(out):
        (t, target, ship, dv_dep, dv_cap, total, tof, v_inf, r_cap, r_cap_asl,
         bx, by, bz, retro, rev) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            # The subject: whose journey this is (#809).
            "ship": ship,
            "dv_dep": float(dv_dep), "dv_cap": float(dv_cap),
            "total": float(total), "tof": float(tof),
            "v_inf": float(v_inf),
            # None = no capture burn. r_cap is a radius from the target's
            # centre; r_cap_asl is the height over its sea level (#405).
            "r_cap": num(r_cap), "r_cap_asl": num(r_cap_asl),
            "burn": (float(bx), float(by), float(bz)),
            # -1 = a log line from before the field existed, not "prograde".
            "retrograde": int(retro) if retro is not None else -1,
            "rev": int(rev) if rev is not None else -1,
        })
    return rows


def parse_relvel(out):
    rows = []
    for m in RELVEL_RE.finditer(out):
        (t, target, kind, rel_spd, tgt_v, rx, ry, rz) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            # station / ship / dock: which target the markers followed, and
            # whether they came from the transfer target at all (#873).
            # A closing speed, not a burn: [xferlog]'s dv_ fields are burns.
            "kind": kind, "rel_spd": float(rel_spd),
            # |target velocity| in the focus body's inertial frame.
            "tgt_v": float(tgt_v),
            # target - you, in the ship's own render frame: the retro marker's
            # direction, and -rel for the prograde one.
            "rel": (float(rx), float(ry), float(rz)),
        })
    return rows


def parse_tgtmark(out):
    rows = []
    for m in TGTMARK_RE.finditer(out):
        (t, target, kind, cx, cy, cz, r_f, ex, ey, ez, dist,
         dx, dy, dz, drawn) = m.groups()
        rows.append({
            "t": float(t), "target": target,
            # body / ship / station: which kind of target the mark followed.
            "kind": kind,
            # The target's own point in this pass's render frame -- the same
            # targetState() the solver and [relvel] lift (#1145).
            "com": (float(cx), float(cy), float(cz)),
            # |com| about the focus body's centre: an authored distance, so a
            # CHECK can hold the mark to the target's real orbit.
            "r_f": float(r_f),
            # The eye, in the same frame as com -- what makes dir checkable.
            "cam": (float(ex), float(ey), float(ez)),
            # Eye to target, which the marker's screen size ignores by design.
            "dist": float(dist),
            # The unit vector the billboard itself was handed; None when the row
            # dashes it, which is the guard above the draw saying no.
            "dir": (float(dx), float(dy), float(dz)) if dx is not None else None,
            "drawn": int(drawn),
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
        (t, body, w, h, mtype, level, ar, ag, ab, sr, sg, sb, shade, ocean,
         sites, tmin, tmean, tmax, elo, ehi, trlo, trhi,
         lag_tau, peak_deg, sun_lon,
         rel_lo, rel_hi, rel_off_lo, rel_off_hi, rel_term_lo, rel_term_hi,
         mean4, datum4, anchor, note,
         wmin, wmean, wmax, welo, wehi, wrlo, wrhi, wrows, wcells,
         wbands, warrowrows, wrowsfrom, wnote, elev_cache) = m.groups()
        # A temperature row that carries neither its numbers nor a reason is a MALFORMED row,
        # not a ground row: the whole temp tail is optional in the regex (a ground row has no
        # such tail at all), so a regression that mangles only those columns would otherwise
        # turn every temperature row into a ground row and cases that do not assert
        # `temp is not None` would read it as normal. Fail loudly instead (#727's law needs a
        # harness that can tell an absence from a mangled reading) -- and fail HERE, before the
        # dict below converts those groups: converting first turned a mangled row into a
        # TypeError from float(None), which says nothing about which column went missing.
        if mtype == "temperature" and tmin is None and note is None:
            raise ValueError(
                "surfmap row claims map=temperature but carries neither temp= nor note=: "
                + m.group(0))
        # Same law for the wind map (#769): "wind" is a picture the world either can or cannot
        # draw, and a row that says it drew one while printing neither its numbers nor a reason
        # is a mangled row, not a refusal.
        if mtype == "wind" and wmin is None and wnote is None:
            raise ValueError(
                "surfmap row claims map=wind but carries neither wind= nor note=: "
                + m.group(0))
        trip = lambda a, b, c: None if a is None else (float(a), float(b), float(c))
        # A column that printed '-' is an ABSENCE, the same as one the row never printed: both
        # read as None, and neither is a 0.0 that a CHECK could mistake for a measurement (#727).
        dash_s = lambda s: None if s is None or s == "-" else float(s)
        rows.append({
            "t": float(t), "body": body,
            "w": int(w), "h": int(h),
            # #745: which picture this row describes, and the level it was drawn at.
            "map": mtype, "level": level,
            # A dashed column is None, not a zero: under a temperature map the base
            # pixels are ramp colours and there is no albedo to average.
            "albedo": trip(ar, ag, ab),
            "shaded": trip(sr, sg, sb),
            "shade": shade, "ocean": ocean, "sites": int(sites),
            # The sweep's own temperatures: what it painted, the ends the LAW states at the
            # datum, the ends the PICTURE runs over, the area-weighted mean taken in T^4 both
            # with and without the height term, and the anchor the relief-free mean comes back
            # to exactly (#793: with relief on, temp_mean4 sits BELOW the anchor by the mean of
            # the lapse over real terrain -- a measured ~1 K on Duna -- which is why the identity
            # pin belongs on datum_mean4 and the gap stays a number a reader can see).
            "temp": trip(tmin, tmean, tmax),
            "temp_ends": (float(elo), float(ehi)) if elo is not None else None,
            "temp_ramp": (float(trlo), float(trhi)) if trlo is not None else None,
            # #794: the authored phase lag in radians -- None when the file states none, which is
            # an ABSENCE and not a lag of zero (both answer the unlagged law, but only one of them
            # was written down). peak_deg is where the sweep's hottest pixel sits, east of this
            # bake's subsolar meridian; sun_lon is that meridian itself. The pair is the engine's
            # side of the hour-angle convention: the sign of peak_deg is what no unit test of the
            # law can see, because cos h is even in h.
            # All three dash, and for different reasons: no lag authored, no span to have a hottest
            # pixel in (an even share of 1 paints one flat field, and naming its maximum would be
            # naming the first pixel the loop reached), and no star to have a meridian from. A
            # dash is read as None here rather than handed to float(), which would answer with a
            # TypeError that says nothing about which column it choked on.
            "lag_tau": dash_s(lag_tau),
            "peak_deg": dash_s(peak_deg),
            "sun_lon": dash_s(sun_lon),
            # [floor m, ceiling m, K at the floor, K at the ceiling] -- only on a body whose air
            # profile lapses; None where the ground has no height term at all.
            "relief": (float(rel_lo), float(rel_hi), float(rel_off_lo), float(rel_off_hi))
                      if rel_lo is not None else None,
            # #828: [coldest K, warmest K] over the whole floor..ceiling span, which is what the
            # palette is actually priced from. Equal to relief[2:] reversed when the lapse never
            # warms with height; a tropopause inside that span moves the cold end off the summit.
            "relief_terms": (float(rel_term_lo), float(rel_term_hi))
                            if rel_term_lo is not None else None,
            "temp_mean4": None if mean4 is None else float(mean4),
            "datum_mean4": None if datum4 is None else float(datum4),
            "anchor": None if anchor is None else float(anchor),
            "note": note,
            # #839: 'hit' = this bake painted from the heights an earlier bake sampled for this
            # body at this size; 'miss' = it sampled them itself; '-' = the picture reads no
            # height at all (a banded body's colour comes from its band table, a refused field
            # paints flat grey, a temperature map on air whose profile does not lapse has no
            # relief term to price). Ground and ground+ocean print hit/miss since #841. A string,
            # not None-or-bool: the three states are different news and a bare False would merge
            # "no heights needed" into "heights re-sampled".
            "elev_cache": elev_cache,
            # #769: the wind map. "wind" is what the sweep painted over the ground (min, the
            # cos-lat weighted mean, max); "wind_ends" is what the body's file states at its
            # rows; "wind_ramp" is the span the PICTURE runs over, priced from the ground's own
            # floor and ceiling. The gap between ends and ramp is the information: Earth's deck
            # row sits at 11 km and Venus' 100 m/s row at 65 km, and no ground stands in either.
            "wind": trip(wmin, wmean, wmax),
            "wind_ends": (float(welo), float(wehi)) if welo is not None else None,
            "wind_ramp": (float(wrlo), float(wrhi)) if wrlo is not None else None,
            "wind_rows": None if wrows is None else int(wrows),
            "wind_cells": None if wcells is None else int(wcells),
            # How many (lat,lon) bands the table holds, how many rows the plan actually drew,
            # and which plan drew them: 'bands' == one arrow row per band, 'grid' == the uniform
            # fallback. None on a refusal, which prints no numbers at all.
            "wind_bands": None if wbands is None else int(wbands),
            "wind_arrow_rows": None if warrowrows is None else int(warrowrows),
            "wind_rows_from": wrowsfrom,
            "wind_note": wnote,
        })
    return rows


def parse_surfmark(out):
    # The marks the Surface Map window draws, published on a gate while it is open. See
    # SURFMARK_RE: this is the draw-side witness, where [surfmap]'s sites= is the bake-side
    # count. The marks= column is on every row; the id/lon/lat tail is absent on the marks=0
    # row, which is a body with nothing to mark -- an absence the case can see.
    rows = []
    for m in SURFMARK_RE.finditer(out):
        (t, body, marks, mid, lon, lat) = m.groups()
        rows.append({
            "t": float(t), "body": body, "marks": int(marks),
            "id": mid,
            "lon": None if lon is None else float(lon),
            "lat": None if lat is None else float(lat),
        })
    return rows


def parse_solar(out):
    # A dashed column reads as None, same as every other optional column here: the
    # row prints '-' when the WORLD states nothing (no authored albedo, no live star
    # in range), and a CHECK that wants "absent" must ask for None, not for 0.0.
    rows = []
    for m in SOLAR_RE.finditer(out):
        (t, star, dist, flux, elev, flat, body, albedo, ground_t, bb_t,
         underfoot_t, underfoot_src, swing, lag, air_t, prof_t, wgt, follow) = m.groups()
        num = lambda s: None if s == "-" else float(s)
        rows.append({
            "t": float(t), "star": star, "body": body,
            "dist": num(dist), "flux": num(flux), "elev": num(elev),
            "flat": num(flat), "albedo": num(albedo),
            "ground_T": num(ground_t), "bb_T": num(bb_t),
            "underfoot_T": num(underfoot_t), "swing": num(swing),
            # #801: the rung in the printer's own words, quotes stripped; None when the row
            # dashes, which is the same "the world states nothing" as the dash above.
            "underfoot_src": None if underfoot_src == "-" else underfoot_src.strip('"'),
            # #837: radians of phase, the authored knob behind a lagged underfoot_T. None is
            # the file stating nothing; 0.0 is a stated noon peak.
            "lag": num(lag),
            # MEASURED at the hull (vehicle.cpp's lastAirT), not recomputed for the log.
            "air_T": num(air_t), "prof_T": num(prof_t), "w": num(wgt),
            "follow": num(follow),
        })
    return rows


def parse_att(out):
    rows = []
    for m in ATT_RE.finditer(out):
        (t, nx, ny, nz, rx, ry, rz, ux, uy, uz, wx, wy, wz, wnorm,
         railed) = m.groups()
        nose = (float(nx), float(ny), float(nz))
        right = (float(rx), float(ry), float(rz))
        up = (float(ux), float(uy), float(uz))
        w = (float(wx), float(wy), float(wz))
        # The rate resolved on the ship's OWN frame -- the one the attitude path
        # speaks (#571) -- so a case can tell pitch from yaw.
        # The sign follows the AXIS, not the key: the stick drives pitch about -right
        # and yaw about -up (applyRotationForce in src/vehicle.cpp), so holding W or A
        # reads NEGATIVE here. Pin magnitudes (the aw* forms), or pin a sign knowing it
        # is the axis's sign, not the direction the ship turns.
        wroll = w[0]*nose[0] + w[1]*nose[1] + w[2]*nose[2]
        wpitch = w[0]*right[0] + w[1]*right[1] + w[2]*right[2]
        wyaw = w[0]*up[0] + w[1]*up[1] + w[2]*up[2]
        rows.append({
            "t": float(t), "nose": nose, "right": right, "up": up, "w": w,
            "wnorm": float(wnorm),
            # Signed, and the abs_ forms pre-computed: free vars in a CHECK
            # generator resolve against globals, not the locals namespace.
            "wroll": wroll, "wpitch": wpitch, "wyaw": wyaw,
            "awroll": abs(wroll), "awpitch": abs(wpitch), "awyaw": abs(wyaw),
            # 0 under physics, 1 coasting on the conic, 2 a frozen park (#1056). The
            # rails' own answer is trustworthy on 1 -- the conic moves the hull and
            # advanceRailOrient turns it -- while a 2 row prints a rate nothing is
            # spending, so a heading pin that wants a moving hull asks for 1.
            "railed": None if railed is None else int(railed),
        })
    return rows


def parse_eva(out):
    rows = []
    for m in EVA_RE.finditer(out):
        (t, mode, grounded, px, py, pz, vx, vy, vz,
         fx, fy, fz, alt, viewyaw, rcs, mass, ladder) = m.groups()
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
            # the grabbed ladder part's uid, 0 = none (#1073).
            "ladder": int(ladder) if ladder is not None else None,
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
        (t, ship, powered, gen, const_draw, charge, capacity, active,
         net) = m.groups()
        rows.append({
            "t": float(t), "ship": ship, "powered": int(powered),
            "gen": float(gen), "const_draw": float(const_draw),
            "charge": float(charge), "capacity": float(capacity),
            "active": float(active),
            # What powerTick balanced on (#659); None on a pre-#659 row.
            "net": num(net),
        })
    return rows


def parse_dv(out):
    rows = []
    for m in DV_RE.finditer(out):
        (t, ship, mass, hullmass, thrust, dv_del, dv_along, dv_rocket,
         prop, ve, dv_left, dv_staged) = m.groups()
        rows.append({
            "t": float(t), "ship": ship, "mass": float(mass),
            "hullmass": float(hullmass),
            "thrust": float(thrust), "dv_del": float(dv_del),
            "dv_along": float(dv_along), "dv_rocket": float(dv_rocket),
            "prop": float(prop), "ve": float(ve),
            "dv_left": float(dv_left), "dv_staged": float(dv_staged),
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
         fx, fy, fz, ux, uy, uz, lvx, lvy, lvz, roll, pole, ax1, gf, camh, hi,
         pany, panp, aimx, aimy, pin, rmb, mmb, cap, fov) = m.groups()
        lv = None if lvx is None else (num(lvx), num(lvy), num(lvz))
        pan = None if pany is None else (float(pany), float(panp))
        aim = None if aimx is None else (float(aimx), float(aimy))
        rows.append({
            "t": float(t), "mode": mode, "focus": focus,
            "yaw": float(yaw), "pitch": float(pitch), "dist": float(dist),
            "fwd": (float(fx), float(fy), float(fz)),
            "up": (float(ux), float(uy), float(uz)),
            # Dashed fields go through num() rather than a plausible 0, so a
            # case that selects on them cannot be satisfied by a view that
            # printed nothing (#876).
            "lv": lv,
            "roll": num(roll), "pole": num(pole), "ax1": num(ax1),
            "gf": num(gf),
            "camh": num(camh),
            # The live +side stop: pitch never exceeds it, so pin == "hi" means
            # pitch == hi and a case can say so numerically.
            "hi": float(hi),
            # #919: pan is the aim offset in rad, aim where that leaves the
            # focus on screen in deg (+x right, +y up). Dashed outside orbit.
            "pan": pan,
            "aim": aim,
            "pin": pin,
            "rmb": int(rmb), "mmb": int(mmb), "cap": int(cap),
            # #1069: the frustum angle the flight wheel now nudges.
            "fov": num(fov),
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
        (t, agl, asl, vs, hs, lat, lon, pitch, roll, hdg, aprg, aoa, beta, cla, sm,
         tas, we, wn, wu, acc, body, bme, sit) = m.groups()
        rows.append({
            "t": float(t), "alt_agl": float(agl), "alt_asl": float(asl),
            "vs": float(vs), "hs": float(hs), "lat": float(lat),
            "lon": float(lon), "pitch": float(pitch), "roll": float(roll),
            "hdg": float(hdg),
            # #498: aprg is nose-to-AIR-relative-prograde, None when the ship had
            # no air (or no flow with a direction) to be pointed along.
            "aprg": float(aprg) if aprg is not None else None,
            # None when the ship had no air-relative flow to have an angle to.
            "aoa": float(aoa) if aoa is not None else None,
            # #450: beta is the yaw-plane half of the same AeroFrame pair, and since #787
            # it goes None on a STRICTER test than aoa -- the flow needs a yaw, not just a
            # direction -- so a row with aoa can still dash beta.
            "beta": float(beta) if beta is not None else None,
            # #717: the frame split, as numbers. hs above is speed over the GROUND;
            # tas is |airRelativeVel|, the speed the aero law bills. They differ by the
            # wind projected onto the hull's heading, which is how a case pins that a
            # wind moved the air rather than the hull. None with no air or no flow
            # above the aero law's own floor, exactly like aoa. wind_east/north are the
            # body's authored table interpolated at this hull's alt_asl, and None on a
            # body that authors no wind at all -- a 0 there would claim a calm air the
            # file never claimed.
            "tas": float(tas) if tas is not None else None,
            "wind_east": float(we) if we is not None else None,
            "wind_north": float(wn) if wn is not None else None,
            # #767: the rising component, along the local vertical. Every authored body
            # answers 0 -- the axis is the capability, not data -- so a case can tell
            # "no such axis" (None) from "an axis that answers 0".
            "wind_up": float(wu) if wu is not None else None,
            # #499: cla is None with no air; sm is None with no air OR with the
            # lifting surfaces disagreeing, which is when the margin is dashed.
            "cla": float(cla) if cla is not None else None,
            "sm": float(sm) if sm is not None else None,
            "acc": float(acc),
            "body": body, "bme": bme, "sit": sit,
        })
    return rows


def parse_dockwin(out):
    rows = []
    for m in DOCKWIN_RE.finditer(out):
        (a, b, why, alA, alB, tnY, tnP, pR, pU, dR, dU, tw, rng, gap, cap, spd, alng, cls,
         atx, aty, szx, szy) = m.groups()
        rows.append({
            "a": a, "b": b, "why": why,
            # Every figure below is None on a row the gate could not measure: the panel
            # printed its sentence because there is no line to plot, not because the
            # line is zero. A case that wants the measured half asks `is not None`.
            "alignA": num(alA), "alignB": num(alB),
            "turnY": num(tnY), "turnP": num(tnP),
            "pushRight": num(pR), "pushUp": num(pU),
            "driftRight": num(dR), "driftUp": num(dU),
            # The RELATIVE spin the dial points at (gameui.cpp): |w_a - w_b| in deg/s,
            # which is #1041's figure -- not the flight panel's "Angular rate", which is
            # spin against the hull's own frame and moves when a hull changes frames.
            # The pair's own capture distance (Vehicle::dockCaptureRange, #1152): 0.4 of
            # the shallower port's depth, floored at the collision standoff. This is the
            # radius the window draws its capture ring at, so a case can pin the gate the
            # pilot is being asked to fly into.
            "twist": num(tw), "rng": num(rng), "gap": num(gap), "cap": num(cap),
            "speed": num(spd), "along": num(alng), "closing": num(cls),
            "at": (float(atx), float(aty)), "size": (float(szx), float(szy)),
        })
    return rows


def parse_orbinfo(out):
    """One row per --info-log ORBITAL-window dump (#498). apo_* / lan / lpe / prg /
    rtg print "-" and become None; the peri_* pair and period print whatever the
    conic says, so those fields take ANY token -- the strict class would refuse an
    open conic's inf/nan and drop the whole row, taking the other 18 figures with
    it. num() then reads that non-finite token as None rather than as a float that
    makes every comparison quietly false (#901). Note the asymmetry with #408: a
    lenient arm means the row PARSES, so the dropped-row naming cannot fire and the
    loud answer is the failing CHECK itself."""
    rows = []
    for m in ORBINFO_RE.finditer(out):
        (t, body, vel, r, apo_r, apo_asl, apo_t, peri_r, peri_asl, peri_t,
         period, inc, ecc, sma, plane, lan, lpe, prg, rtg,
         energy) = m.groups()
        rows.append({
            "t": float(t), "body": body, "vel": float(vel), "r": float(r),
            "apo_r": num(apo_r), "apo_asl": num(apo_asl), "apo_t": num(apo_t),
            "peri_r": float(peri_r), "peri_asl": num(peri_asl),
            "peri_t": num(peri_t), "period": num(period),
            "inc": float(inc), "ecc": float(ecc), "sma": float(sma),
            "plane": plane, "lan": num(lan), "lpe": num(lpe),
            # nose to the CONIC's prograde / retrograde (marker_vel) -- the pair the
            # ORBITAL window used to print as rows of its own (2026_11_06). The Game
            # Debug window's Prg (air) is the air-relative pair to these (#498). None
            # when the conic has no prograde to measure to at all (#870).
            "prg": num(prg), "rtg": num(rtg),
            "energy": float(energy),
        })
    return rows


def parse_sim_report(out):
    """The run-level accounting line a --timeout run prints (#402), or None when
    there is none -- a scripted run exits on its step budget and prints a different
    line, and a run the harness killed never printed one. `dropped` is 0.0 when the
    catch-up cap never bit, which is the normal case on a box with a render node;
    what it costs on one without (or with eight runs sharing it) is the point."""
    adv = SIM_ADVANCED_RE.search(out)
    if not adv:
        return None
    drop = SIM_DROPPED_RE.search(out)
    return {"advanced": float(adv.group(1)), "wall": float(adv.group(2)),
            "dropped": float(drop.group(1)) if drop else 0.0}


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


def parse_surface(out):
    """#485: one row per lifting or steering part per --drag-log sample. The
    [surface] line has no timestamp of its own either, so rows inherit the t= of
    the [drag] line above them. It only prints when a ship's surface readout is
    armed -- the flight model's SURFACE window, or --drag-log on the ship you are
    piloting -- so a case that reads it must pass --drag-log."""
    rows = []
    t = None
    for line in out.splitlines():
        if line.startswith("[surface]"):
            for name, a, q, mach, watts in SURFACE_ITEM_RE.findall(line):
                # findall hands back "" (not None) for a group that did not
                # participate, so the optional mach= and watts= test truthy.
                rows.append({"t": t, "name": name,
                             "a": None if a == "-" else float(a), "q": float(q),
                             "mach": float(mach) if mach else None,
                             "watts": float(watts) if watts else None})
        elif line.startswith("[drag]"):
            m = DRAG_RE.match(line)
            if m:
                t = float(m.group(1))
    return rows


UILIST_HEAD_RE = re.compile(r"^\[uilist\] loop=(\d+\.\d+)s items=(\d+)")
UILIST_ITEM_RE = re.compile(r'^\[uilist\] window="((?:[^"\\]|\\.)*)" '
                            r'label="((?:[^"\\]|\\.)*)"'
                            r' rect=(-?\d+),(-?\d+)-(-?\d+),(-?\d+)(.*)$')


def parse_uilist(out):
    """Every imgui item a --ui-list dump saw, tagged with its dump's time in seconds.
    Text rows are included -- the trailing ` text` marks them as not clickable, but
    they are what the player READS, and #601 made the dump carry their text so a case
    can pin a readout. #734's 2 Hz hold is only assertable because the part window's
    flow/watts rows are drawn through uiText."""
    rows = []
    t = None
    declared, seen = None, 0

    def close_dump():
        # The dump header states how many items the registry held, and the printer draws a
        # subset of those (unlabelled entries are skipped), so the counts are not equal and
        # cannot be compared. They CAN say "the registry had items and I parsed none", which is
        # what a rect-format change (a decimal sneaking in) would do -- every uilist CHECK in
        # the suite would then be scanning an empty list and still pass. Loud beats empty.
        if declared and declared > 0 and seen == 0:
            raise ValueError(
                f"uilist dump at t={t}s declared {declared} items and the parser read none"
                " -- the dump format changed and every row is being dropped")

    for line in out.splitlines():
        m = UILIST_HEAD_RE.match(line)
        if m:
            close_dump()
            t = float(m.group(1))
            declared, seen = int(m.group(2)), 0
            continue
        m = UILIST_ITEM_RE.match(line)
        if m and t is not None:
            seen += 1
            # The rect, as numbers: where a row sits is itself a fact worth pinning --
            # #745's chooser asks for two dropdowns ON THE SAME LINE as the buttons, and
            # "same line" means "same y", which is checkable and not adjectives.
            rows.append({"t": t, "window": m.group(1), "label": m.group(2),
                         "rect": (int(m.group(3)), int(m.group(4)),
                                  int(m.group(5)), int(m.group(6))),
                         "text": " text" in m.group(7)})
    close_dump()
    return rows


# The game reports every scripted UI action on stdout: one line when it lands
# ("... window=\"W\" label=\"L\" at=x,y pressed"), one when it refuses ("... FAILED
# <why>"), plus the typed echo. A --ui-type prints a [uiclick] press too, because
# focusing a field is a click on it.
UIROW_RE = re.compile(r'^\[(uiclick|uitype|uiscroll)\] loop=(\d+\.\d+)s '
                      r'(?:path|target)="((?:[^"\\]|\\.)*)"(?: (.*))?$',
                      re.MULTILINE)
UIROW_LANDED_RE = re.compile(r'^(?:window="((?:[^"\\]|\\.)*)" '
                             r'label="((?:[^"\\]|\\.)*)" )?at=(-?\d+),(-?\d+) '
                             r'(pressed|aimed)$')


def parse_ui_actions(out):
    """(clicks, types, scrolls): every scripted UI action the game admitted to.
    `ok` is False on a refusal, whose text sits in `why` ("not found (...)",
    "disabled (...)", "MISMATCH ...", "clipped ...", "blocked ...", "off screen
    ..."). These are whole-run lists like toast/scene: a save/load case presses
    its way through the Title, and those rows are above the "Loaded game from"
    cut the world-scoped lists read from."""
    clicks, types, scrolls = [], [], []
    for m in UIROW_RE.finditer(out):
        tag, t, path, tail = m.group(1), float(m.group(2)), m.group(3), m.group(4)
        row = {"t": t, "path": path, "ok": True, "why": None,
               # The line as printed, so a guard can weigh it the way EXPECT does.
               "line": m.group(0), "tag": tag}
        if tail:
            if tail.startswith("FAILED "):
                row["ok"] = False
                row["why"] = tail[len("FAILED "):]
            else:
                lm = UIROW_LANDED_RE.match(tail)
                if lm:
                    if lm.group(1) is not None:
                        row["window"] = lm.group(1)
                        row["label"] = lm.group(2)
                    row["at"] = (int(lm.group(3)), int(lm.group(4)))
        {"uiclick": clicks, "uitype": types, "uiscroll": scrolls}[tag].append(row)
    return clicks, types, scrolls


def ui_actions(args):
    """The UI actions a case schedules, as [(flag, tag, path)] in schedule order.
    Read off the tokenized ARGS, which is what the game receives: the value is
    MS,"PATH"[,extra], so the path is the second comma field -- and for a click,
    the last (a click takes no third field, and no shipped label holds a comma)."""
    out = []
    for i, tok in enumerate(args):
        tag = UI_ACTION_FLAGS.get(tok)
        if tag is None or i + 1 >= len(args):
            continue
        _ms, _, rest = args[i + 1].partition(",")
        path = rest.split(",")[0] if tag == "uiclick" else rest.partition(",")[0]
        out.append((tok, tag, path))
    return out


def ui_action_gaps(specs, rows, expect):
    """(#860) Two ways a scripted UI action goes silently unreal.

    missed -- the case scheduled the action and the game never reported it. The
    schedule runs on wall time and the UI advances on sim time, so on a loaded
    box the timeout can arrive before the menu is drawn; every CHECK about what
    the click was supposed to do then weighs a run where it never happened.

    refused -- the game printed FAILED and the case never said that refusal is
    the point. 148 of the 188 click-using cases held `FORBID FAILED`, and the
    rest were green over missed clicks: the red depended on a substring the case
    happened to carry. The opt-in is the case's own vocabulary -- an EXPECT that
    is a substring of the line, which is exactly how EXPECT already tests stdout.
    A case that pins its refusal is unaffected; one that relied on not having one
    now says so out loud.
    """
    got = {}
    for r in rows:
        key = (r["tag"], r["path"])
        got[key] = got.get(key, 0) + 1
    missed = []
    for flag, tag, path in specs:
        if got.get((tag, path), 0) <= 0:
            missed.append((flag, tag, path))
        else:
            got[(tag, path)] -= 1     # two clicks on one widget: two lines
    refused = [r for r in rows
               if not r["ok"] and not any(e in r["line"] for e in expect)]
    return missed, refused


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
         beta, cpl, cla, sm, profile, formcp, airT, airP, airA, mach, ld,
         sidecda, sidecp, bodycda, bodycp) = m.groups()
        row = {
            # alt_asl: metres above the body's sea-level DATUM, which is NOT the
            # ground and goes negative exactly where rho peaks (#245). Renamed from
            # bare "alt" in #552 A3 to join the names the other logs already use for
            # that datum -- [surfinfo]'s alt_agl/alt_asl pair, [orbitlog]'s peri_asl,
            # [ascent]'s peri_asl/apo_asl. ([dbg] and [evalog] still print a bare
            # alt= that means ABOVE TERRAIN, which is the confusion #626 is about.)
            "t": float(t), "alt_asl": float(alt), "rho": float(rho),
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
        # #560: Induced= now dashes with the flow fields, because its CdA is
        # q-NORMALISED (vehicle.cpp:applyInducedDrag bills CL_sys = sum(L_i/q_i) /
        # sum(S_i) and then inducedCd(CL_sys, ...)): a parked hull's jitter flow
        # cancels its own q out of the coefficient and charged 0.000..1.442 m2, row to
        # row, against a 0.00 N force. So None means "there was no flow to bill", which
        # is NOT the same reading as 0.000 = "the wings are unloaded".
        if ind is not None:
            row["induced"] = float(ind)
        if beta is not None:
            row["beta"] = float(beta)
        if cpl is not None:
            row["cpl"] = float(cpl)
        # #454: CLa is the whole airframe's lift slope (CL·A per radian, so times q
        # for the force) -- it collapses as the surfaces droop past the stall. SM is
        # the static margin: metres the lift CHANGE sits behind the COM, so SM > 0 is
        # the restoring one. SM is None when the ship had no lifting slope that pass
        # (dashed on the line); CLa prints 0.00 there rather than dashing.
        if cla is not None:
            row["cla"] = float(cla)
        if sm is not None:
            row["sm"] = float(sm)
        # #455: the surfaces' own form-drag CdA -- 0.00 for a hull with no lifting
        # or steering parts, and DASHED (so absent from the row) when AeroFrame had
        # no usable flow direction, which #559 added after a parked glider printed
        # a shape bill off integrator jitter. The regex group is optional so a
        # captured line from before #455 still parses.
        if profile is not None:
            row["profile"] = float(profile)
        # The combined form-drag lever (#455): metres along the nose from the COM,
        # + = ahead (diverging). None when the line dashed it for want of a flow.
        if formcp is not None:
            row["formcp"] = float(formcp)
        # #552 A3: the air this row stood in. airT [K] / airP [Pa] / airA [m/s] are
        # what the body's profile says at alt_asl -- one constant read for a scale
        # height, per-level for a layer table (#552 A4) -- the same seam
        # the atlas dossier reads -- and mach is the air-relative speed over airA,
        # NOT row["v"] over airA: on a spinning body the air moves (#97). All four
        # are absent when the line dashed them, which is how a case tells "no air"
        # from "air this thin".
        if airT is not None:
            row["airT"] = float(airT)
            row["airP"] = float(airP)
            row["airA"] = float(airA)
            row["mach"] = float(mach)
        # #694: the lift over the drag the air applies ALONG the flow -- the ratio a
        # glide ratio is meant to match, and the one res/ships/README.md's L/D table
        # should be quoting. Absent (None) when the line dashed it: no usable flow, or
        # the air pushing the hull instead of holding it back. A CHECK that reads this
        # must skip None rows, since a filter over absent keys passes vacuously (#653).
        if ld is not None:
            row["ld"] = float(ld)
        # #947: the crossflow normal bill (sidecda, m2) and its lever (sidecp, m along
        # the nose from the COM, same sign convention as cpl/formcp). The force is a
        # flank normal -- along the flow's component PERPENDICULAR TO THE PART'S AXIS,
        # which is why it is its own pair of columns instead of a share of Cd/A or of
        # FormCp. q x sidecda is that force's size: the sin of the angle to the axis
        # already lives inside the strip. Absent when the row dashed the pair: the
        # same no-usable-flow gate as aoa.
        if sidecda is not None:
            row["sidecda"] = float(sidecda)
            row["sidecp"] = float(sidecp)
        # #1097: the body-lift bill (bodycda, signed m2) and its lever (bodycp, m
        # along the nose, same sign convention as cpl/formcp). q x bodycda is the
        # summed body-lift force's SIZE (the column folds in the sin of the angle
        # between the part's axis and its own local flow), and the force is
        # perpendicular to that FLOW. 0.00 for every hull whose parts leave
        # body_lift unauthored -- every flight-pinned hull, though the catalog's
        # liftbody_4m is opt-in from the VAB.
        if bodycda is not None:
            row["bodycda"] = float(bodycda)
            row["bodycp"] = float(bodycp)
        rows.append(row)
    return rows


def parse_rec_file(path):
    """--record (#662): the flight recorder's trace, read from the FILE it writes
    under the case's scratch data dir rather than from stdout. Rows are `key=value`
    pairs named by the file's own `#rec v<n> channels=<list>` header, so this parser
    is deliberately generic: a new channel needs no parser change, which is the whole
    reason the trace carries its channel list. A dashed column ('-') reads as None,
    exactly as an absent optional --drag-log field does."""
    rows = []
    with open(path, "r", errors="replace") as f:
        for line in f:
            if not line.startswith("[rec] "):
                continue          # the #rec / #ship header lines, and anything else
            row = {}
            for tok in line[len("[rec] "):].split():
                key, sep, val = tok.partition("=")
                if not sep:
                    continue
                row[key] = None if val == "-" else float(val)
            if "t" in row:
                rows.append(row)
    return rows


def rec_header(path):
    """The trace's header line: the channel list it promises, or '' when the file
    has none (a recorder without a header is unreadable after it is gone)."""
    try:
        with open(path, "r", errors="replace") as f:
            for line in f:
                if line.startswith("#rec "):
                    return line.split("channels=", 1)[1].strip() if "channels=" in line else ""
    except OSError:
        pass
    return ""


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


# A CHECK whose all()/any() walked an EMPTY input asserts nothing: `all(r["charge"]
# > 0 for r in power if r["kind"] == "warp")` stays true when the filter keeps no
# rows at all, so a renamed log column or a typo in a row key silently deletes the
# assertion and the case goes green doing it (#653). The harness cannot know
# whether an empty filter WAS the point, so this records the fact instead of
# reddening the case: the line prints beside the case and lands in summary.json's
# diag with NOTE_PREFIX, and `passed` asks "is every diag line a note?".
NOTE_PREFIX = "#note "
_builtin_all, _builtin_any = all, any


def recording_aggregates(vacuous):
    """all/any for the CHECK namespace, counting how often they saw no rows.

    Only the FIRST element is pulled out and tested by hand, so the builtin's
    short-circuit survives: forcing the iterable to a list would evaluate rows a
    passing CHECK never needed to reach, and a row that raises on element 7 would
    turn a pass into "CHECK raised"."""
    def rec_all(ip):
        it = iter(ip)
        try:
            head = next(it)
        except StopIteration:
            vacuous.append("all()")
            return True
        if not head:
            return False
        return _builtin_all(it)

    def rec_any(ip):
        it = iter(ip)
        try:
            head = next(it)
        except StopIteration:
            vacuous.append("any()")
            return False
        if head:
            return True
        return _builtin_any(it)
    return rec_all, rec_any


def vacuous_note(what, expr):
    """The note line for one vacuous aggregate (#653)."""
    verdict = "true" if what == "all()" else "false"
    return ("%sCHECK's %s walked an empty input, so it is vacuously %s and pins "
            "nothing (#653): %s" % (NOTE_PREFIX, what, verdict, expr))


def notes_only(diag):
    """Is every diagnostic line a note? One expression, so nothing that counts
    diagnostics gets to re-decide what a failure is."""
    return all(line.startswith(NOTE_PREFIX) for line in diag)


def check_verdict(expr, check_globals):
    """One CHECK: None when it passes, else the failure line to report. The
    tuple test is the #650 backstop for a tuple the case file does not spell
    out -- a helper that hands back a row, `CHECK first(pairs)`: a non-empty
    tuple is always true, and a dead assertion is worse than a failing one. An
    EMPTY tuple is left to truthiness: it is always false, and "CHECK failed"
    is the honest line for it. Everything else keeps today's truthiness,
    because `re.search(...)` -- a Match, not a bool -- is the shipped idiom in
    58 CHECK lines. The truthiness test stays inside the try: a value whose
    __bool__ raises was reported as `CHECK raised` before, and still is."""
    try:
        val = eval(expr, check_globals)  # noqa: S307 - trusted case files
        if isinstance(val, tuple) and val:
            return ("CHECK evaluated to a tuple, which is always true -- a "
                    "trailing comma, or a helper that returns one (#650): %s"
                    % expr)
        if callable(val):
            return ("CHECK evaluated to a callable, which is always true -- "
                    "the lambda was written but never passed anything: %s" % expr)
        if not val:
            return "CHECK failed: %s" % expr
    except Exception as e:
        return "CHECK raised: %r (%s)" % (expr, e)
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
    # The flight recorder writes traces here (#662), so the same rule as the save
    # slots: an old case's trace would answer this case's assertions.
    shutil.rmtree(os.path.join(data_dir, "flights"), ignore_errors=True)

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
    # The lists fed from `world` describe the vessel that was ACTIVE when the line
    # printed, so parse_scoped stamps row["ship"] from the game's markers (#447);
    # perf/toast/scene read the whole run and stay unattributed, as they already are.
    orbit = parse_scoped(world, parse_orbit)
    dbg = parse_scoped(world, parse_dbg)
    bldg = parse_scoped(world, parse_bldg)
    xfer = parse_scoped(world, parse_xfer)
    porkchop = parse_scoped(world, parse_porkchop)
    surfmap = parse_scoped(world, parse_surfmap)
    surfmark = parse_scoped(world, parse_surfmark)
    solar = parse_scoped(world, parse_solar)
    att = parse_scoped(world, parse_att)
    ascent = parse_scoped(world, parse_ascent)
    eva = parse_scoped(world, parse_eva)
    fuel = parse_scoped(world, parse_fuel)
    drainlog = parse_scoped(world, parse_drainlog)
    power = parse_scoped(world, parse_power)
    dv = parse_scoped(world, parse_dv)
    mass = parse_scoped(world, parse_mass)
    drag = parse_scoped(world, parse_drag)
    ctrl = parse_ctrl(world)
    surface = parse_surface(world)
    uilist = parse_uilist(world)
    shake = parse_scoped(world, parse_shake)
    cam = parse_scoped(world, parse_cam)
    terrain = parse_scoped(world, parse_terrain)
    surf = parse_scoped(world, parse_surf)
    orbinfo = parse_scoped(world, parse_orbinfo)
    dockwin = parse_scoped(world, parse_dockwin)
    scene = parse_scene(out)
    mapb = parse_scoped(world, parse_mapb)
    mapbldg = parse_scoped(world, parse_mapbldg)
    relvel = parse_scoped(world, parse_relvel)
    tgtmark = parse_scoped(world, parse_tgtmark)
    plane = parse_scoped(world, parse_plane)
    slew = parse_scoped(world, parse_slew)
    shipswitch = parse_shipswitch(world)
    uiclick, uitype, uiscroll = parse_ui_actions(out)
    perf = parse_perf(out)
    toast = parse_toast(out)
    simreport = parse_sim_report(out)
    # The recorder's trace is a FILE, not stdout (#662), and a CHECK cannot open
    # files (empty __builtins__, below) -- so the harness reads the traces this run
    # wrote into its scratch data dir. The dir is swept before each case and one
    # run writes one trace, so the glob is enough to find it.
    rec = []
    for trace in sorted(glob.glob(os.path.join(data_dir, "flights", "*.log"))):
        rec.extend(parse_rec_file(trace))
    # #653: the CHECK namespace's all/any count the times they saw no rows.
    vacuous = []
    rec_all, rec_any = recording_aggregates(vacuous)
    ns = {
        "out": out, "orbit": orbit, "dbg": dbg, "bldg": bldg, "xfer": xfer,
        "porkchop": porkchop, "surfmap": surfmap, "surfmark": surfmark, "solar": solar, "att": att, "ascent": ascent, "eva": eva,
        "fuel": fuel, "drainlog": drainlog, "power": power, "dv": dv, "mass": mass,
        "drag": drag, "shake": shake,
        "rec": rec,
        "ctrl": ctrl,
        "surface": surface,
        "uilist": uilist,
        "cam": cam,
        "terrain": terrain,
        "surf": surf, "orbinfo": orbinfo, "dockwin": dockwin,
        "mapb": mapb, "mapbldg": mapbldg,
        "plane": plane, "slew": slew, "relvel": relvel, "tgtmark": tgtmark,
        # The handoffs themselves: [{"verb": switch/removed/unchanged/none, "name": ...}].
        "shipswitch": shipswitch,
        # Every scripted UI action the game admitted to, whole-run like toast/scene.
        "uiclick": uiclick, "uitype": uitype, "uiscroll": uiscroll,
        "perf": perf,
        "scene": scene,
        "toast": toast,
        # None unless the run printed its --timeout accounting (#402).
        "simreport": simreport,
        "first": first, "last": last, "any": rec_any, "all": rec_all,
        **CHECK_BUILTINS,
    }
    # 3b) A row list that came back EMPTY while the game clearly printed its tag is a
    # disagreement between this file's regex and the game's printer, not a world with no rows.
    # Every CHECK below walks such a list, so `all(...)` on it passes; the failure mode is a
    # green case that asserts nothing (#653's vacuity, in its loudest guise: it also throws
    # away rows a case did ask for). Only a case that names the list is held to it -- a run
    # prints tags it was never asked about, and an unrelated `[toast]` must not become this
    # case's failure. The loop's second half is the same disagreement when only SOME rows
    # vanish, which the empty-list test cannot see; see #408 below.
    for name, rx, fed in ROW_PARSERS:
        tag = line_tag(rx)
        if tag is None:
            continue
        # The name has to appear as an identifier, not inside a quoted string: a case that
        # pins "[mass]" inside its own regex, or compares a camera mode to "orbit", never
        # walks those lists (see LIT_STR_RE).
        if not any(re.search(r"\b%s\b" % name, LIT_STR_RE.sub('""', expr))
                   for expr in case["check"]):
            continue
        printed = [ln for ln in (out if fed == "out" else world).splitlines()
                   if ln.startswith(tag) and not NOTE_WORDS.search(ln)]
        if not ns[name]:
            if printed:
                diag.append("%s: %s printed %d line(s) and the %s parser read 0 -- this "
                            "file's regex and the game's printer disagree, so every CHECK "
                            "on %s is walking an empty list"
                            % (name, tag, len(printed), name, name))
            continue
        # The list came back non-empty, so the guard above is silent -- but rows can still
        # be missing, and the shape that hides best is a number that blew up: the game
        # prints "inf"/"-nan" where its own math ran out, this file's number fields stop at
        # those letters, and the row is dropped rather than reported (#408). A CHECK over
        # the surviving rows is then green over a world that produced a blow-up, and
        # last(<list>) is an older row than the case thinks it is.
        blown = dropped_nonfinite_rows(rx, printed)
        for ln in blown[:2]:
            diag.append("%s: the %s parser read %d of the %d %s lines the game printed -- "
                        "the one below carries inf/nan, which this file's number fields do "
                        "not match, so the row is gone and no CHECK ever sees the blow-up: "
                        "%s" % (name, name, len(ns[name]), len(printed), tag, ln))
        if len(blown) > 2:
            diag.append("%s: ... %d more %s line(s) dropped for inf/nan (%d of %d printed)"
                        % (name, len(blown) - 2, tag, len(blown), len(printed)))
    # 3d) #860: the UI actions the case scheduled versus the ones the game admitted to.
    # A click that never happened was invisible -- only the 148 cases carrying
    # `FORBID FAILED` turned a missed click red, and then for the wrong reason (the
    # substring, not the story). Both halves are named here.
    missed, refused = ui_action_gaps(ui_actions(case["args"]),
                                     uiclick + uitype + uiscroll, case["expect"])
    for flag, tag, path in missed:
        diag.append("%s: the case asked for %s ...,%r and the game never reported it -- "
                    "no [%s] line for that path, so whatever the CHECKs weigh, this run "
                    "never clicked it (a schedule runs on wall time while the UI advances "
                    "on sim time) (#860)" % (tag, flag, path, tag))
    for r in refused[:3]:
        diag.append("%s: the game refused a scripted UI action the case did not ask for "
                    "-- pin it with an EXPECT if the refusal is the point: %s"
                    % (r["tag"], r["line"]))
    if len(refused) > 3:
        diag.append("%s: ... %d more refused UI actions (%d in total)"
                    % ("ui", len(refused) - 3, len(refused)))
    # `ns` must be globals: free vars in a CHECK generator/comprehension
    # resolve against globals, not locals. Empty __builtins__ blocks open/exec.
    check_globals = dict(ns)
    check_globals["__builtins__"] = {}
    for expr in case["check"]:
        del vacuous[:]
        line = check_verdict(expr, check_globals)
        if line is not None:
            diag.append(line)
        # A vacuous aggregate is not a failure -- an empty filter may be exactly
        # what the case means -- but a green case that asserts nothing is the
        # thing worth seeing, so it prints beside the case and lands in
        # summary.json (#653).
        for what in vacuous:
            diag.append(vacuous_note(what, expr))

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

    passed = notes_only(diag)
    return passed, diag, out, duration


def select_cases(case_files, selectors):
    """Filter to cases whose NAME, filename or subject path contains any
    selector (case-insensitive). A path selector like `save/` or
    `save/270` scopes a run to one group.

    Also returns the selectors that matched NO case (#1061). A selector is an
    intent: `run.py A B C` asks for three runs, and an OR filter that simply
    ignores the dead one reports the number that RAN -- `4/4 passed`, exit 0 --
    rather than the number that was ASKED. main() refuses the run on a non-empty
    list here, the same way it already refuses when nothing at all matches."""
    if not selectors:
        return case_files, []
    sel = [s.lower() for s in selectors]
    hit = [False] * len(sel)
    out = []
    for path in case_files:
        base = os.path.splitext(os.path.basename(path))[0].lower()
        rel = os.path.splitext(os.path.relpath(path, CASES_DIR))[0].lower()
        try:
            name = parse_cases(path)["name"].lower()
        except ValueError as e:
            # Like case_kind(): a case that does not parse must not vanish from
            # a name selection as if it never existed -- and this one is worse,
            # since a missed NAME match prints "no case matches".
            warn("warning: %s" % e)
            name = ""
        matched = [s in name or s in base or s in rel for s in sel]
        hit = [h or m for h, m in zip(hit, matched)]
        if any(matched):
            out.append(path)
    return out, [selectors[i] for i in range(len(sel)) if not hit[i]]


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


def print_no_match(asked, all_files):
    """One shape for "you asked for cases that are not here", whichever filter
    went unmatched."""
    print("no case matches %s" % " ".join(asked))
    names = available_names(all_files)
    print("available: %s" % ", ".join(names[:40]))
    if len(names) > 40:
        print("... %d more; e2e/INDEX.md lists every case with its claim."
              % (len(names) - 40))


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


def self_test():
    """The runner's own pins, run by `make test` (test-gen). No e2e case can
    pin these: a case file asserts about the GAME, not about run.py. Uses the
    real parser on a temp file rather than a copy of its rules."""
    import tempfile

    base = ["NAME selftest", "CLAIM pins the runner's own assertion rules",
            "KIND harness", "ARGS --timeout 1"]

    def parse_with(check_lines):
        # The real parser, so a CHECK text here that LOOKS like a fixture path
        # (an `e2e/...` or `res/...` token) is fixture-checked like any case's.
        with tempfile.TemporaryDirectory() as d:
            p = os.path.join(d, "1-selftest.txt")
            with open(p, "w", encoding="utf-8") as f:
                f.write("\n".join(base + check_lines) + "\n")
            return parse_cases(p)

    fails = []
    # The #650 trap and its siblings: a trailing comma makes a tuple, and a
    # bare container is truthy whatever it holds, so the case passes without
    # testing anything.
    traps = ['CHECK 1 == 2, "and the note says why"',
             'CHECK (1 == 2), "parenthesised the same way"',
             'CHECK [1 == 2, 3 == 4]',
             'CHECK {1 == 2}',
             'CHECK (1 == 2, 3 == 4) and "a note"',
             'CHECK (lambda e: e == 2)',
             'CHECK (lambda e: e == 2) and "a note"']
    for bad in traps:
        try:
            parse_with([bad])
            fails.append("accepted a dead-shape CHECK: %s" % bad)
        except ValueError as e:
            if "ALWAYS true" not in str(e):
                fails.append("dead-shape CHECK refused wrongly: %s" % e)
    try:
        parse_with(["CHECK 1 == ="])
        fails.append("accepted a CHECK that does not parse")
    except ValueError as e:
        if "does not parse" not in str(e):
            fails.append("bad CHECK refused for the wrong reason: %s" % e)
    # The shipped idioms must stay parseable. re.search() answers with a Match,
    # not a bool, so "a CHECK must evaluate to a bool" -- this ticket's own fix
    # idea -- would have failed 58 existing cases instead of fixing one.
    # Comprehensions stay legal too: a bare `{...}`/`[...]` comprehension means
    # "some rows matched", which is truthiness doing the right thing, unlike a
    # literal of the same shape.
    try:
        parse_with(['CHECK re.search("x", out)',
                    "CHECK (band := [1, 2]) and len(band) >= 2",
                    'CHECK all(r["t"] > 0 for r in power)',
                    'CHECK {r["t"] for r in power if r["t"] > 0}'])
    except ValueError as e:
        fails.append("refused a legitimate CHECK: %s" % e)

    ns = {"re": re, "len": len, "all": all, "__builtins__": {}}
    for expr, want in (("1 == 1", None),
                       ("1 == 2", "CHECK failed"),
                       ("1/0", "CHECK raised"),
                       ('re.search("x", "bxb")', None),
                       ('re.search("q", "bab")', "CHECK failed"),
                       ("(1, 2)", "tuple"),
                       ("(1,)", "tuple"),
                       ("()", "CHECK failed"),
                       ("[1, 2]", None),
                       ("[]", "CHECK failed"),
                       ("lambda e: e == 2", "callable")):
        got = check_verdict(expr, dict(ns))
        if want is None and got is not None:
            fails.append("CHECK %r reported %r" % (expr, got))
        if want is not None and (got is None or want not in got):
            fails.append("CHECK %r reported %r, wanted %r" % (expr, got, want))

    # #653: an aggregate over a filter that kept nothing. `all([])` is True, so a
    # CHECK like all(r["charge"] > 0 for r in power if r["kind"] == "warp") passes
    # without looking at a row the moment the filter stops matching -- a renamed log
    # column silently deletes the assertion. The harness records the emptiness as a
    # NOTE (an empty filter can be the point) rather than reddening the case, so both
    # halves need pinning: the recording, and that a note is not a failure.
    vac = []
    rec_all, rec_any = recording_aggregates(vac)
    ns_rec = {"all": rec_all, "any": rec_any, "rows": [], "__builtins__": {}}
    if check_verdict('all(r > 0 for r in rows if r > 5)', dict(ns_rec)) is not None:
        fails.append("a vacuous all() CHECK reported a failure line")
    if vac != ["all()"]:
        fails.append("a vacuous all() recorded %r, wanted ['all()']" % (vac,))
    del vac[:]
    if check_verdict('any(r > 0 for r in rows)', dict(ns_rec)) is None:
        fails.append("a vacuous any() CHECK passed")
    if vac != ["any()"]:
        fails.append("a vacuous any() recorded %r, wanted ['any()']" % (vac,))
    note = vacuous_note("all()", 'all(r > 0 for r in rows)')
    if not note.startswith(NOTE_PREFIX):
        fails.append("the vacuous note is not marked, so it would fail the case")
    if not notes_only([note]):
        fails.append("notes_only() reddened a note-only diag")
    if notes_only([note, "CHECK failed: 1 == 2"]):
        fails.append("notes_only() let a real failure through")
    if notes_only([]) is not True:
        fails.append("notes_only([]) must pass -- a case with nothing to say is green")
    # Builtin semantics survive: a falsy head answers False and the rest of the
    # generator is never pulled. Forcing the iterable to a list -- the obvious way
    # to count rows -- would evaluate rows a passing CHECK never needed, and turn a
    # pass into "CHECK raised".
    def _boom_after_false():
        yield False
        raise RuntimeError("pulled past the short-circuit")
    def _boom_after_true():
        yield True
        raise RuntimeError("pulled past the short-circuit")
    if rec_all(_boom_after_false()) is not False:
        fails.append("recording all() lost the short-circuit")
    if rec_any(_boom_after_true()) is not True:
        fails.append("recording any() lost the short-circuit")
    del vac[:]
    if check_verdict('all(r > 0 for r in [1, 2])', dict(ns_rec)) is not None or vac:
        fails.append("a non-empty aggregate was recorded as vacuous: %r" % (vac,))

    # #408: a number that blew up prints "inf" or "-nan". This file reads numbers as
    # [-\d.e+]+, which takes the sign and stops at the letters, so the field never
    # closes and the WHOLE row is dropped -- the list comes back one short, last() is
    # an older row, and every CHECK walks a world with no blow-up in it. Both halves
    # are pinned with the real parsers on real printed lines: the row that vanishes
    # gets named, and the field that spells nan ON PURPOSE does not.
    good_orb = ('[orbinfo] t=3.9s body="Kerbin" vel=255.271 m/s r=608430 m '
                'apo_r=608496 m apo_asl=8495.52 m apo_t=562.014 s peri_r=3368.87 m '
                'peri_asl=-596631 m peri_t=279.145 s period=565.737 s inc=1.64213 deg '
                'ecc=0.988988 sma=305932 m plane=equ lan=192.273 deg lpe=12.2813 deg '
                'prg=8.48935 deg rtg=171.511 deg energy=-5.77233e+06 J/kg')
    bad_orb = good_orb.replace("prg=8.48935", "prg=-nan").replace("rtg=171.511", "rtg=nan")
    if ORBINFO_RE.match(good_orb) is None:
        fails.append("the #408 pin's clean [orbinfo] line stopped matching ORBINFO_RE")
    if dropped_nonfinite_rows(ORBINFO_RE, [good_orb, bad_orb]) != [bad_orb]:
        fails.append("an [orbinfo] row carrying prg=-nan was not named as dropped")
    # #894 moved [ascent]'s missing numbers onto the dash, which flips this row's half
    # of the #408 question: a nan that slips into these fields is now a row nobody can
    # read, so it must be NAMED rather than parsed. Both rows pin all four arms, because
    # a typo in one alternative is invisible if that field is only ever numeric (or only
    # ever dashed) in the sample.
    asc_num = ('[ascent] t=32.3s phase=pad lever=1.00 pitch=87.5 deg peri_asl=-250000 m '
               'apo_asl=129.753 m inc_eq=0.286474 deg node_r=1.2e+05 m accel=18.08 m/s2 '
               'g=2.69 m/s2 why=""')
    asc_dash = ('[ascent] t=32.3s phase=pad lever=1.00 pitch=87.5 deg peri_asl=- '
                'apo_asl=- inc_eq=- node_r=- accel=18.08 m/s2 g=2.69 m/s2 why=""')
    for tag, ln, want in (("all-numeric", asc_num,
                           (-250000.0, 129.753, 0.286474, 120000.0)),
                          ("all-dashed", asc_dash, (None, None, None, None))):
        got = [r[k] for r in parse_ascent(ln)
               for k in ("peri_asl", "apo_asl", "inc_eq", "node_r")]
        if got != list(want):
            fails.append("the %s [ascent] line parsed as %r, expected %r"
                         % (tag, got, want))
    # #675 appended the ascent profile after why=. Pinned here rather than in a case,
    # because the tail is what every future flight CHECK walks: a full row, a dashed
    # gamma (a hull with no velocity above orbit.h's conic floor has no angle
    # THROUGH), and -- the one that catches a regression in the OTHER direction --
    # that the two rows above, written before the tail existed, report NO profile
    # instead of a profile of zeros. A CHECK like `r["grav"] < 500` must fail loudly
    # on an old row, not agree with a 0 that nothing measured.
    asc_prof = (asc_num + ' alt_asl=129.8 m v_orb=35.50 m/s q=0.0 Pa'
                ' gamma=87.50 deg dv=1128.5 m/s grav=188.6 m/s drag=0.0 m/s'
                ' steer=12.4 m/s burns=1')
    got = parse_ascent(asc_prof)
    want = (129.8, 35.5, 0.0, 87.5, 1128.5, 188.6, 0.0, 12.4, 1)
    if len(got) != 1 or tuple(got[0][k] for k in ("alt_asl", "v_orb", "q", "gamma",
                                                  "dv_exp", "grav", "drag", "steer",
                                                  "burns")) != want:
        fails.append("the #675 [ascent] tail parsed as %r, expected %r" % (got, want))
    got = parse_ascent(asc_prof.replace("gamma=87.50 deg", "gamma=-"))
    if len(got) != 1 or got[0]["gamma"] is not None or got[0]["alt_asl"] != 129.8:
        fails.append("a [ascent] row carrying gamma=- parsed as %r" % (got,))
    for tag, ln in (("all-numeric", asc_num), ("all-dashed", asc_dash)):
        got = parse_ascent(ln)
        if len(got) != 1 or any(got[0][k] is not None for k in
                                ("alt_asl", "v_orb", "q", "gamma", "dv_exp", "grav",
                                 "drag", "steer", "burns")):
            fails.append("the %s [ascent] line reported a #675 profile it does not "
                         "carry: %r" % (tag, got))
    for was, now in (("peri_asl=-250000 m", "peri_asl=nan m"),
                     ("apo_asl=129.753 m", "apo_asl=nan m"),
                     ("inc_eq=0.286474 deg", "inc_eq=nan deg"),
                     ("node_r=1.2e+05 m", "node_r=nan m")):
        bad = asc_num.replace(was, now)
        if ASCENT_RE.match(bad) is not None:
            fails.append("ASCENT_RE still parses a [ascent] line carrying %s" % now)
        if dropped_nonfinite_rows(ASCENT_RE, [bad]) != [bad]:
            fails.append("an [ascent] row carrying %s was not named as dropped" % now)
    # A blow-up inside the tail is NOT #408-named, and that is a choice, not an
    # oversight: the tail is optional whole, so an unreadable value in it degrades the
    # row to "no profile" and the CHECK fails against None. That is what accel/g have
    # done since #610 added them. What must not happen is the reverse -- a broken tail
    # that reads as a profile of measurements -- so the pin is on the None, not on the
    # row being named.
    for was, now in (("q=0.0 Pa", "q=nan Pa"), ("grav=188.6 m/s", "grav=-inf m/s"),
                     ("burns=1", "burns=nan")):
        got = parse_ascent(asc_prof.replace(was, now))
        if len(got) != 1 or any(got[0][k] is not None for k in
                                ("alt_asl", "v_orb", "q", "gamma", "dv_exp", "grav",
                                 "drag", "steer", "burns")):
            fails.append("a [ascent] row carrying %s parsed as %r; the tail must "
                         "either read clean or not at all" % (now, got))
    # A word that merely starts with these letters is not a blow-up, and the two
    # spellings glibc prints have to be caught however they sit in the line.
    for word in ("--info-log", "information", "inflight", "nano.png", "format inf_"):
        if NONFINITE_RE.search(word):
            fails.append("NONFINITE_RE matched the word %r" % word)
    for word in ("inf", "-nan", "INF", "prg=+nan deg", "eta=inf s", "hdg=nan,"):
        if not NONFINITE_RE.search(word):
            fails.append("NONFINITE_RE missed %r" % word)
    # #732 put a dash in the [surface] ANGLE ("a=-") for a part the aero pass billed but
    # whose law has no lift frame to measure it in (a plate broadside to the flow). It has
    # to parse as an absence rather than vanish: flight/485 and flight/730 walk these items
    # BY POSITION, assuming the ten items under a [drag] row are the ten parts, so one
    # unparseable token silently shortens the list and misaligns every pairing after it --
    # a wrong answer that still prints PASS. The row's other figures must survive: a
    # refused angle says nothing about the q, Mach and watts the same part paid.
    surf_num = ('[surface] wing_l a=+11.8 q=1234 mach=0.200 watts=50'
                ' wing_r a=-3.4 q=1200 mach=0.190 watts=40')
    surf_dash = ('[surface] wing_l a=- q=1234 mach=0.200 watts=50'
                 ' wing_r a=+3.4 q=1200 mach=0.190 watts=40')
    if [r["a"] for r in parse_surface(surf_num)] != [11.8, -3.4]:
        fails.append("a numeric [surface] pair parsed as %r" % (parse_surface(surf_num),))
    dashed = parse_surface(surf_dash)
    if len(dashed) != 2 or dashed[0]["a"] is not None or dashed[1]["a"] != 3.4:
        fails.append("a [surface] row carrying a=- parsed as %r; the dashed angle must "
                     "read as an absence and keep its neighbour" % (dashed,))
    if any(dashed[0][k] is None for k in ("q", "mach", "watts")):
        fails.append("the dashed-angle [surface] item lost its own figures: %r" % (dashed[0],))

    # #1056: [attlog] grew a railed= column saying WHICH model produced the row -- the
    # rail conic or Bullet. Both spellings have to parse: a case wants to read the
    # marker, and the two dozen cases predating it print none. A row that stops parsing
    # is a silent hole in every heading pin that follows it.
    att_row = ('[attlog] t=85.000s nose=[+1.0000 -0.0000 +0.0000] '
               'right=[+0.0000 +1.0000 -0.0000] up=[-0.0000 +0.0000 +1.0000] '
               'w=[-0.0001 +0.0003 +0.0001] |w|=0.0003 rad/s railed=1')
    marked = parse_att(att_row)
    if len(marked) != 1 or marked[0]["railed"] != 1:
        fails.append("an [attlog] row carrying railed=1 parsed as %r" % (marked,))
    # 2 is a frozen park, whose row prints a rate nothing is spending; a heading pin
    # that wants a turning hull has to be able to tell the two rail states apart.
    frozen = parse_att(att_row.replace("railed=1", "railed=2"))
    if len(frozen) != 1 or frozen[0]["railed"] != 2:
        fails.append("an [attlog] row carrying railed=2 parsed as %r" % (frozen,))
    unmarked = parse_att(att_row.replace(" railed=1", ""))
    if len(unmarked) != 1 or unmarked[0]["railed"] is not None \
            or abs(unmarked[0]["wnorm"] - 0.0003) > 1e-12:
        fails.append("an [attlog] row WITHOUT railed= parsed as %r; the column is "
                     "optional, not part of the row" % (unmarked,))

    # #870: a hull resting on a world that does not turn has no conic prograde and no
    # plane normal, so both files print "-" where a direction used to be. The parse has
    # to answer None -- so a case can pin `p["prg"] is None` -- rather than drop the row
    # or raise inside float(). Both printers are pinned, in both spellings.
    good_plane_row = ('[planelog] t=3.5s frame=rot plane=equ inc=51.6000 deg '
                      'n_orb=(0.000000,1.000000,0.000000) '
                      'n_ref=(0.000000,1.000000,0.000000) '
                      'node=(0.000000,0.000000,1.000000) ticks=drawn '
                      'prg=(-0.500000,0.866025,0.000000) nrm=(0.000000,1.000000,0.000000) '
                      'rdl=(0.866025,0.000000,0.500000)')
    dashed_plane_row = good_plane_row.replace(
        'prg=(-0.500000,0.866025,0.000000) nrm=(0.000000,1.000000,0.000000)', 'prg=- nrm=-')
    if PLANE_RE.match(good_plane_row) is None:
        fails.append("the #870 pin's clean [planelog] line stopped matching PLANE_RE")
    prow = parse_plane(good_plane_row)
    if len(prow) != 1 or prow[0]["prg"] != (-0.5, 0.866025, 0.0) \
            or prow[0]["nrm"] != (0.0, 1.0, 0.0):
        fails.append("parse_plane lost a numeric prg=/nrm= (%r)" % (prow,))
    # #892: the radial pair's direction rides at the end of the row, so a line printed
    # without it (or with a non-number in it) must not parse into a row with a silent
    # hole where the marker direction belongs.
    if prow[0]["rdl"] != (0.866025, 0.0, 0.5):
        fails.append("parse_plane lost the rdl= triplet (%r)" % (prow,))
    if PLANE_RE.match(good_plane_row.replace(" rdl=(0.866025,0.000000,0.500000)", "")) is not None:
        fails.append("PLANE_RE matched a [planelog] row with no rdl= field")
    if PLANE_RE.match(good_plane_row.replace("rdl=(0.866025", "rdl=(-nan")) is not None:
        fails.append("PLANE_RE matched a [planelog] row carrying rdl=-nan")
    drow = parse_plane(dashed_plane_row)
    if len(drow) != 1 or drow[0]["prg"] is not None or drow[0]["nrm"] is not None:
        fails.append("parse_plane did not read a dashed prg=/nrm= as None (%r)" % (drow,))
    # The dash arm must not become an excuse to swallow a blow-up: a row that PRINTS
    # nan is still a broken instrument, and #408's note is what names it.
    if PLANE_RE.match(good_plane_row.replace("prg=(-0.500000", "prg=(-nan")) is not None:
        fails.append("PLANE_RE matched a [planelog] row carrying prg=-nan")
    dash_orb = good_orb.replace("prg=8.48935 deg rtg=171.511 deg", "prg=- rtg=-")
    orow = parse_orbinfo(dash_orb)
    if len(orow) != 1 or orow[0]["prg"] is not None or orow[0]["rtg"] is not None:
        fails.append("parse_orbinfo did not read a dashed prg=/rtg= as None (%r)" % (orow,))
    if parse_orbinfo(good_orb)[0]["prg"] != 8.48935:
        fails.append("parse_orbinfo lost a numeric prg= while adding the dash arm")
    # #901: peri_* and period take any token, so an inf/nan row PARSES and #408's
    # dropped-row naming cannot fire. num() is what makes that state loud rather than a
    # float every comparison with which is quietly false, so both directions are pinned.
    nan_orb = (good_orb.replace("peri_asl=-596631 m", "peri_asl=nan m")
               .replace("period=565.737 s", "period=inf s"))
    nrow = parse_orbinfo(nan_orb)
    if len(nrow) != 1:
        fails.append("an [orbinfo] line carrying peri_asl=nan stopped parsing")
    elif nrow[0]["peri_asl"] is not None or nrow[0]["period"] is not None:
        fails.append("an [orbinfo] inf/nan token parsed as a float instead of None")
    if parse_orbinfo(good_orb)[0]["peri_asl"] != -596631.0:
        fails.append("num() lost a finite [orbinfo] peri_asl")
    # The shared helper has to answer the same way for the strict parsers, where an
    # absent group is the ordinary absence (#894, #405).
    if num(None) is not None or num("-") is not None or num("1.5e+05") != 150000.0:
        fails.append("num() mishandled an absent, dashed or finite token")
    # #659's net= column, written from what vehicle.cpp PRINTS rather than from what
    # the format string was meant to say -- that is how #894's dash bug survived its
    # own self-test. The arm is STRICT, unlike [ascent]'s dashed pair, because this
    # figure has no "no value" state to spell: powerTick always pays a number. A row
    # that lost the field, or ever grew a dash, drops out and #408 names it rather
    # than handing every reader of power["net"] a None nobody asked for.
    good_power = ('[powerlog] t=5.0s ship="power_test" powered=1 gen=298.4W '
                  'const_draw=600.0W charge=11996.3Wh capacity=12000.0Wh '
                  'active=2475.0W net=-2776.6W')
    if parse_power(good_power)[0]["net"] != -2776.6:
        fails.append("parse_power lost the #659 net= column")
    for label, mangled in (("missing", good_power.replace(" net=-2776.6W", "")),
                           ("dashed", good_power.replace("net=-2776.6W", "net=-")),
                           ("mangled", good_power.replace("net=-2776.6W", "net=near-2776.6W"))):
        if POWER_RE.match(mangled) is not None:
            fails.append("POWER_RE matched a [powerlog] row whose net= field is %s" % label)

    # #872: the [mapbldg] row is the only observable face the map's station marks have,
    # and the two cases read it field by field. A field renamed or reordered in the
    # printf is caught here, in milliseconds, rather than as a "0 rows" mystery in a
    # case that has to boot the game to notice.
    good_mb = ('[mapbldg] t=2.5s focus=Kerbin id=kerbin_station kind=orbit r=1000000 '
               'pts=64 peri_r=1000000 gate_r=1000000 pos=1000000,0,0 '
               'anchor=1000000,0,0 drawn=2')
    if MAPBLDG_RE.match(good_mb) is None:
        fails.append("the #872 pin's clean [mapbldg] line stopped matching MAPBLDG_RE")
    for lost in (" pos=1000000,0,0 anchor=1000000,0,0", " drawn=2", " peri_r=1000000",
                 " gate_r=1000000", " pts=64"):
        if MAPBLDG_RE.match(good_mb.replace(lost, "")) is not None:
            fails.append("MAPBLDG_RE matched a [mapbldg] row missing%s" % lost)
    # The row prints pos=..., with commas: a space-separated triplet would parse as one
    # field and silently drop the rest of the line.
    if MAPBLDG_RE.match(good_mb.replace("pos=1000000,0,0", "pos=1000000 0 0")) is not None:
        fails.append("MAPBLDG_RE took a space-separated pos=, so the #872 pins read noise")
    # The air-platform spelling: no anchor and no ring start (its pose is the wind law,
    # so its gate is its own distance) and drawn=-1 (no map window marked anything this
    # frame) all have to survive the parse.
    air_mb = (good_mb.replace("kind=orbit", "kind=air")
              .replace("pts=64 peri_r=1000000 gate_r=1000000", "pts=0 peri_r=0 gate_r=6107800")
              .replace("anchor=1000000,0,0", "anchor=none").replace("drawn=2", "drawn=-1"))
    if MAPBLDG_RE.match(air_mb) is None:
        fails.append("the #872 air [mapbldg] spelling stopped matching MAPBLDG_RE")
    air_row = parse_mapbldg(air_mb)[0]
    if air_row["anchor"] is not None or air_row["drawn"] != -1 or air_row["pts"] != 0:
        fails.append("parse_mapbldg misread anchor=none, drawn=-1 or pts=0")
    if air_row["gate_r"] != 6107800.0 or air_row["peri_r"] != 0.0:
        fails.append("parse_mapbldg misread the gate_r/peri_r pair")

    # #873: the [relvel] row is the only face the HUD's two relative-velocity markers
    # have, and a case that walks it field by field is only as good as this row shape.
    good_rv = ('[relvel] t=2.5s target="Kerbin station" kind=station rel_spd=1762.8765 m/s '
               'tgt_v=1879.3299 m/s rel=[1120.3823 1127.0262 -763.0785]')
    if RELVEL_RE.match(good_rv) is None:
        fails.append("the #873 pin's clean [relvel] line stopped matching RELVEL_RE")
    for lost in (" kind=station", " tgt_v=1879.3299 m/s", " rel_spd=1762.8765 m/s",
                 ' target="Kerbin station"'):
        if RELVEL_RE.match(good_rv.replace(lost, "")) is not None:
            fails.append("RELVEL_RE matched a [relvel] row missing%s" % lost)
    # rel= prints a space-separated bracket (like burn= in [xferlog]); a parenthesised
    # or comma-separated triplet would parse as one field and drop the rest of the line.
    if RELVEL_RE.match(good_rv.replace(
            "rel=[1120.3823 1127.0262 -763.0785]", "rel=(1120.3823,1127.0262,-763.0785)")) is not None:
        fails.append("RELVEL_RE took a parenthesised rel=, so the #873 pins read noise")
    rv = parse_relvel(good_rv)[0]
    if (rv["kind"] != "station" or rv["rel_spd"] != 1762.8765 or rv["tgt_v"] != 1879.3299
            or rv["rel"] != (1120.3823, 1127.0262, -763.0785)):
        fails.append("parse_relvel misread kind, rel_spd, tgt_v or rel")
    # A negative dv is not a speed the printer can produce, but a parse that swallowed
    # "dv=-1" as "dv=" plus junk would hide a printf bug.
    if parse_relvel(good_rv.replace("rel_spd=1762.8765", "rel_spd=-1"))[0]["rel_spd"] != -1.0:
        fails.append("parse_relvel did not read a signed rel_spd=")

    # #1145: [tgtmark] is the only face the target mark has, and every CHECK about it
    # walks these columns, so the row shape gets the same guard [relvel] has.
    good_tm = ('[tgtmark] t=2.9s target="Kerbin station" kind=station '
               'com=[683893.562 -344590.855 643075.998] m r_f=1000000.0 m '
               'cam=[3011.252 3001.246 600248.962] m dist=765673.0 m '
               'dir=[0.889260 -0.453969 0.055934] drawn=1')
    if TGTMARK_RE.match(good_tm) is None:
        fails.append("the #1145 pin's clean [tgtmark] line stopped matching TGTMARK_RE")
    for lost in (" kind=station", ' target="Kerbin station"', " r_f=1000000.0 m",
                 " dist=765673.0 m", "com=[683893.562 -344590.855 643075.998] m",
                 " cam=[3011.252 3001.246 600248.962] m",
                 " dir=[0.889260 -0.453969 0.055934]", " drawn=1"):
        if TGTMARK_RE.match(good_tm.replace(lost, "")) is not None:
            fails.append("TGTMARK_RE matched a [tgtmark] row missing%s" % lost)
    if TGTMARK_RE.match(good_tm.replace(
            "com=[683893.562 -344590.855 643075.998]",
            "com=(683893.562,-344590.855,643075.998)")) is not None:
        fails.append("TGTMARK_RE took a parenthesised com=, so the #1145 pins read noise")
    tm = parse_tgtmark(good_tm)[0]
    if (tm["kind"] != "station" or tm["r_f"] != 1000000.0 or tm["dist"] != 765673.0
            or tm["com"] != (683893.562, -344590.855, 643075.998)
            or tm["cam"] != (3011.252, 3001.246, 600248.962)
            or tm["dir"] != (0.889260, -0.453969, 0.055934) or tm["drawn"] != 1):
        fails.append("parse_tgtmark misread kind, r_f, cam, dist, dir or drawn")
    # The skipped-draw row dashes dir rather than repeating the vector the marker
    # held last frame, so a case can tell "not placed" from "placed there".
    tm0 = parse_tgtmark(good_tm.replace(
        "dir=[0.889260 -0.453969 0.055934] drawn=1", "dir=- drawn=0"))[0]
    if tm0["dir"] is not None or tm0["drawn"] != 0:
        fails.append("the #1145 row's dashed dir= did not read as an absence")

    # #447: a row belongs to the ship that was ACTIVE when it printed, and the
    # marker that says so carries no timestamp -- line order is the only evidence,
    # and the old flat parse threw it away. Pinned on the four marker spellings,
    # on the ordering across segments, and on [mass] keeping the name its own
    # line prints.
    log447 = "\n".join([
        good_orb.replace("t=3.9s", "t=1.0s"),
        "Active ship 2 of 8: tanker",
        good_orb.replace("t=3.9s", "t=2.0s"),
        "Removed 'racer'; active ship 1 of 7: hauler",
        good_orb.replace("t=3.9s", "t=3.0s"),
        "Removed 'hauler'; nothing left to control -- no active vessel",
        good_orb.replace("t=3.9s", "t=4.0s"),
    ])
    rows447 = parse_scoped(log447, parse_orbinfo)
    if [r.get("ship") for r in rows447] != [None, "tanker", "hauler", NO_VESSEL]:
        fails.append("the ship each [orbinfo] row belongs to is wrong: %r"
                     % [r.get("ship") for r in rows447])
    if [r["t"] for r in rows447] != [1.0, 2.0, 3.0, 4.0]:
        fails.append("parse_scoped lost the line order across ship segments")
    if [r.get("ship") for r in parse_scoped(good_orb, parse_orbinfo)] != [None]:
        fails.append("a run that never switched ships invented a handoff")
    mass_line = '[mass] t=0.333s ship="stager" parts=7 mass=4235.86 kg ' \
                 'hullmass=4235.86 kg railed=0'
    if [r.get("ship") for r in parse_scoped("Active ship 3 of 3: tanker\n" + mass_line,
                                            parse_mass)] != ["stager"]:
        fails.append("the marker overwrote the ship [mass] printed for its own row")
    if ([(s["verb"], s["name"]) for s in parse_shipswitch(log447)]
            != [("switch", "tanker"), ("removed", "hauler"),
                ("none", NO_VESSEL)]):
        fails.append("the shipswitch rows do not describe the four markers")
    if parse_shipswitch("Removed 'x' (active unchanged: (none))") != [
            {"verb": "unchanged", "name": NO_VESSEL}]:
        fails.append("'active unchanged: (none)' is the shipless game too")
    if [s["name"] for s in parse_shipswitch(
            "Removed 'x' (active unchanged: pod (2))")] != ["pod (2)"]:
        fails.append("the marker's closing paren ate a display name's own paren")
    if parse_shipswitch("[toast] Active ship: racer, warp 10x"):
        fails.append("a toast that merely names the active ship read as a handoff")

    # #860: the game reports every scripted UI action, and the runner used to
    # ignore the report. Two ways that hid a run that never did the thing the
    # CHECKs weigh: an action the game never reported at all, and a FAILED line
    # the case did not ask for. The opt-in is an EXPECT of the line, which is
    # already how a case says "this stdout is the point".
    ui_log = "\n".join([
        '[uiclick] loop=1.01s path="Game Debug/Load e2e fixture saves" '
        'window="Game Debug" label="Load e2e fixture saves" at=870,508 pressed',
        '[uiclick] loop=3.03s path="Save/Load/##filter" window="Save/Load" '
        'label="##filter" at=960,463 pressed',
        '[uitype] loop=3.04s path="Save/Load/##filter" typed="save_eva_ground"',
        '[uiclick] loop=6.52s path="Settings/Apply DPI" FAILED disabled (the '
        'widget is greyed out)',
    ])
    ui_clicks, ui_types, ui_scrolls = parse_ui_actions(ui_log)
    if [(r["path"], r["ok"]) for r in ui_clicks] != [
            ("Game Debug/Load e2e fixture saves", True),
            ("Save/Load/##filter", True), ("Settings/Apply DPI", False)]:
        fails.append("the [uiclick] rows are not the three the log printed")
    if len(ui_types) != 1 or ui_types[0]["ok"] is not True:
        fails.append("the [uitype] echo is not a row")
    if ui_scrolls:
        fails.append("the log has no [uiscroll] line")
    if ui_clicks[0].get("window") != "Game Debug" or ui_clicks[0].get("at") != (870, 508):
        fails.append("a landed click lost the window/label/rect it reported")
    if not (ui_clicks[2]["why"] or "").startswith("disabled"):
        fails.append("a refusal lost its reason: %r" % (ui_clicks[2]["why"],))
    ui_args = shlex.split(
        '--ui-click 1000,"Game Debug/Load e2e fixture saves" '
        '--ui-click 2500,"fixtures" '
        '--ui-type 3000,"Save/Load/##filter",save_eva_ground '
        '--ui-click 6500,"Settings/Apply DPI"')
    ui_specs = ui_actions(ui_args)
    if [p for _f, _t, p in ui_specs] != ["Game Debug/Load e2e fixture saves",
                                         "fixtures", "Save/Load/##filter",
                                         "Settings/Apply DPI"]:
        fails.append("the scheduled UI actions were not read off ARGS: %r"
                     % (ui_specs,))
    all_rows = ui_clicks + ui_types + ui_scrolls
    pinned = ['path="Settings/Apply DPI" FAILED disabled']
    if ui_action_gaps(ui_specs, all_rows, pinned) != (
            [("--ui-click", "uiclick", "fixtures")], []):
        fails.append("a click the game never reported is not named: %r"
                     % (ui_action_gaps(ui_specs, all_rows, pinned)[0],))
    missed, refused = ui_action_gaps(ui_specs, all_rows, [])
    if len(refused) != 1 or refused[0]["path"] != "Settings/Apply DPI":
        fails.append("a FAILED line with no EXPECT behind it is not named: %r"
                     % ([r["path"] for r in refused],))
    if ui_action_gaps(ui_specs, all_rows, pinned)[1]:
        fails.append("an EXPECTed refusal still reddened the case")

    # EXPECT/FORBID are substring tests, so a line written as a regex is a DEAD pin.
    # Refusing it at parse time is only worth anything if the refusal itself is pinned:
    # the shapes that must be refused, and the literals that must NOT be (a save-format
    # marker like "[v0]" and a printed pair like "temp_ramp=[0.00 0.00]" are real stdout,
    # and a guard that reddens those gets switched off rather than obeyed).
    for want_refused, expect_txt, forbid_txt in (
            (True, "", r"ground lag\s+0\.000 rad"),
            (True, "", "lag_tau=[0-9]"),
            (True, r"Loaded game from (\w+)", ""),
            (False, "", "lag=0.0000"),
            (False, "Loaded game from e2e/fixtures/save_name_clash (active: Ash) [v0]",
             "temp_ramp=[0.00 0.00]")):
        try:
            check_literal_shapes([expect_txt] if expect_txt else [],
                                 [forbid_txt] if forbid_txt else [],
                                 "self_test_case.txt")
            if want_refused:
                fails.append("regex-shaped FORBID %r was accepted" % (forbid_txt,))
        except ValueError:
            if not want_refused:
                fails.append("literal %r was refused as regex-shaped"
                             % (forbid_txt or expect_txt,))
    # The CHECK namespace (#910), in both directions. The ordering trio a case
    # author reaches for has to be there -- and the empty __builtins__ that makes
    # CHECK_BUILTINS an ALLOWLIST has to stay empty, because a "helpful" widening
    # turns every case file into a script that can read the repo.
    for expr, want in (('tuple(sorted(r["n"] for r in rows))', (1, 2, 3)),
                       ('[i for i in range(len(rows))]', [0, 1, 2]),
                       ('sum(sum(t) for t in zip(range(3), range(3)))', 6)):
        try:
            got = eval(expr, dict(CHECK_BUILTINS, __builtins__={},
                                  rows=[{"n": 3}, {"n": 1}, {"n": 2}]))
            if got != want:
                fails.append("CHECK namespace %r gave %r, want %r"
                             % (expr, got, want))
        except Exception as e:
            fails.append("CHECK namespace cannot evaluate %r: %s" % (expr, e))
    for barred in ("open", "exec", "eval", "compile", "__import__", "globals",
                   "vars", "dir", "getattr", "input"):
        try:
            eval(barred, dict(CHECK_BUILTINS, __builtins__={}))
            fails.append("CHECK namespace lets a case reach %r" % barred)
        except NameError:
            pass
    # A selector is an intent (#1061). Temp files rather than the real battery,
    # so this pin cannot be satisfied or broken by what e2e/cases happens to
    # hold on the day it runs.
    with tempfile.TemporaryDirectory() as d:
        real = os.path.join(d, "flight", "05-real-thing.txt")
        os.makedirs(os.path.dirname(real))
        with open(real, "w", encoding="utf-8") as f:
            f.write("\n".join(base) + "\n")
        picked, missed = select_cases([real], ["selftest", "05-real",
                                               "flight/nope", "nope-name"])
        if len(picked) != 1:
            fails.append("select_cases dropped a case its selectors matched")
        if missed != ["flight/nope", "nope-name"]:
            fails.append("select_cases missed-selector report is %r, want "
                         "['flight/nope', 'nope-name'] -- `selftest` matches "
                         "only the NAME line and must count as matched"
                         % (missed,))
        _, missed_all = select_cases([real], [])
        if missed_all:
            fails.append("no selectors produced a missed list %r" % (missed_all,))
    if fails:
        for f in fails:
            print("SELF-TEST FAIL: %s" % f)
        return 1
    print("e2e runner self-test ok: dead-shape CHECKs (a trailing comma, a container, "
          "an uncalled lambda) refused at parse and eval; a CHECK whose all()/any() "
          "walked an empty input recorded as a note that cannot redden the case; a "
          "printed row dropped because its number reads inf/nan named, and "
          "[ascent]'s dashed fields parsed as None while its nan row is named too; "
          "[surface]'s dashed angle parsed as an absence that keeps its row and the "
          "other figures on it; [attlog]'s railed= column read as the model that "
          "produced the row, and its absence read as absence; "
          "every world-scoped row stamped with the vessel that was active when it "
          "printed, in line order, and [mass]'s own ship= kept; a scripted UI action "
          "the game never reported, and a FAILED line with no EXPECT behind it, both "
          "named; regex-shaped EXPECT/FORBID refused at parse while literal text "
          "containing brackets stays legal; re.search, walrus, comprehension and "
          "immediately-called lambda CHECKs accepted; CHECK failed/raised wording "
          "intact; a batch selector that matches no case named and the run refused "
          "rather than a count quietly lowered; the CHECK allowlist carries "
          "sorted/tuple/range and still refuses "
          "open/exec/__import__")
    return 0


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
    parser.add_argument("--self-test", action="store_true",
                        help="run the runner's own pins and exit: the rules "
                             "about CHECK lines themselves, which no e2e case "
                             "can assert about (make test runs this)")
    args = parser.parse_args()
    if args.self_test:
        # Run no cases, so say so rather than exiting 0 on a selection the user
        # clearly meant to run.
        if args.selectors or args.kind:
            parser.error("--self-test runs no cases; drop the selectors")
        return self_test()
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

    case_files, missed = select_cases(all_files, selectors)
    if args.kind:
        case_files = [p for p in case_files if case_kind(p) in args.kind]
    if missed:
        # A dead selector inside a batch used to just lower the count: five
        # selectors asked, `4/4 passed`, exit 0, and the case that should have
        # covered the change never started (#1061). Refuse the whole run, naming
        # only the selectors that matched nothing, so a typo cannot turn a check
        # into no check.
        print_no_match(["%r" % s for s in missed], all_files)
        return 1
    if not case_files:
        # Name every filter that contributed: with no selectors a --kind that
        # matches nothing otherwise reads as `no case matches ()`.
        asked = ["%r" % s for s in selectors]
        asked += ["--kind %s" % k for k in (args.kind or [])]
        print_no_match(asked, all_files)
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
