#!/usr/bin/env python3
"""E2E runner: run selected e2e/cases/<subject>/*.txt against the game binary.

Usage: python3 e2e/run.py <selectors> [--jobs N] [--game PATH] [--limit S]
[--show-cmd] [--kind K] [--self-test]. No selectors runs the whole battery --
that is what `make release` does. Nothing caps or paces a run: how much e2e is
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
is how an assertion goes silently dead (#650). KIND says what sort of
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
terrain/surf/xfer/porkchop/surfmap/solar/mapb/plane/slew/toast/scene) plus
first/last/re. `rec` is the exception to "parse stdout": the flight recorder's
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
import ast
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
# --ascent-log (src/tick.cpp): the ascent law's own state, one row per log tick.
# The phase is the law's side of the contract with the player -- "done" means the
# orbit is the one that was asked for, "failed" means it says so out loud -- so a
# case should read it rather than grep for it. peri_asl/apo_asl/inc_eq/node_r are
# "nan" when the law has no such number for that tick: a railed hull solves no
# conic, an open trajectory has no apoapsis, a coast before the node has no node.
# float("nan") parses and every comparison with it is false, so a CHECK that
# wanted a real number fails loudly rather than agreeing with a placeholder that
# used to read like a measurement (-1 m apoapsis, -1.00 deg inclination).
ASCENT_RE = re.compile(
    r"\[ascent\]\s+t=([\d.]+)s\s+phase=(\w+)\s+lever=([-\d.e+]+)\s+pitch=([-\d.e+]+) deg\s+"
    r"peri_asl=([-\d.e+]+|nan) m\s+apo_asl=([-\d.e+]+|nan) m\s+inc_eq=([-\d.e+]+|nan) deg\s+"
    r"node_r=([-\d.e+]+|nan) m"
    # The stage's full-throttle acceleration and the body's pull at the hull: the
    # pair the law's lift-off gate compares, so "this stage cannot lift off" can
    # be pinned from the row instead of taken on trust. Optional: a row without
    # them still parses.
    r"(?:\s+accel=([-\d.e+]+) m/s2\s+g=([-\d.e+]+) m/s2)?"
    r"\s+why=\"([^\"]*)\""
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
    r"(?:relief=\[([-\d.]+) ([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+)?"
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
    r"\s+underfoot_T=([-\d.]+|-) K\s+swing=([-\d.]+|-)\s+lag=([-\d.]+|-)"
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
SLEW_RE = re.compile(
    r"\[slew\]\s+t=([\d.]+)\s+mode=(\S+)\s+E=([-\d.]+)\s+rad\s+\([-\d.]+ deg\)\s+"
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
    r"\[slew\]\s+t=([\d.]+)\s+mode=killrot\s+\|w\|=([-\d.]+)\s+rad/s\s+"
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
# attitude control. charge is the only one of the four that MOVES, and it moves
# by (gen - const_draw - active) x dt, so active= is that third term printed
# rather than inferred (#643): it is wheelDrawWatts(), so it is what the hull is
# BILLING this substep (0 while coasting), not the drums' rating. A ship whose
# charge sits still is a ship whose battery is decorative (#379).
POWER_RE = re.compile(
    r"\[powerlog\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+powered=(\d+)\s+"
    r"gen=([-\d.]+)W\s+const_draw=([-\d.]+)W\s+charge=([-\d.]+)Wh\s+"
    r"capacity=([-\d.]+)Wh\s+active=([-\d.]+)W"
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
    r"(\S+) a=([-+]\d+(?:\.\d+)?) q=(\d+(?:\.\d+)?)"
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
# beta (#450) dashes on exactly the same test -- the same AeroFrame pair, the same
# flow floor -- and sits next to aoa in the line because they are one reading.
# aprg (#498) is the Game Debug window's Prg (air): nose to AIR-relative prograde. It
# dashes on the same flow floor again (no air, or a flow with no direction), and the
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
# [orbinfo] is the ORBITAL window's own numbers. Nothing parsed it until #498
# needed it: prg/rtg are now the ONLY place the CONIC's prograde angle prints -- the
# window dropped its own Prg/Rtg rows on 2026_11_06, leaving the HUD marker and these
# keys. #498 is precisely that the conic points where the ORBIT is going rather than
# where a plane in an atmosphere is flying. apo_* print "-" for an open conic and
# lan/lpe for a missing node line, so those get the two-arm treatment; the peri_*
# trio and period print whatever the conic says, so they go through num() -- an inf or
# nan from an open conic becomes None instead of raising inside a CHECK.
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
    r"prg=([-\d.e+]+) deg\s+rtg=([-\d.e+]+) deg\s+"
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
            "apo_r": float(apo_r) if apo_r is not None else None,
            "apo_asl": float(apo_asl) if apo_asl is not None else None,
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
         why) = m.groups()
        row = {
            "t": float(t), "phase": phase, "lever": float(lever),
            "pitch": float(pitch), "peri_asl": float(peri), "apo_asl": float(apo),
            "inc_eq": float(inc_eq), "node_r": float(node), "why": why,
        }
        # None when the row predates the field, so a CHECK can ask for it and
        # fail loudly on an old-format row rather than compare against None.
        row["accel"] = None if accel is None else float(accel)
        row["g"] = None if gpull is None else float(gpull)
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
        (t, mode, e, w_slew, w_roll, w_third, wmag, alpha, alpha_full, ieff,
         w_des, w_cap, tau_ask, tau_spent, nx, ny, nz, tx, ty, tz) = m.groups()
        rows.append({
            "t": float(t), "mode": mode, "E": float(e),
            "w_slew": float(w_slew), "w_roll": float(w_roll),
            "w_third": float(w_third), "wnorm": float(wmag),
            "alpha": float(alpha), "alpha_full": float(alpha_full),
            "Ieff": float(ieff),
            # What the law wants and the ceiling on it (#678). w_cap prints "-" when
            # unlimited: a 0 there would read as a law forbidden to move, so it parses
            # to None rather than to 0.0.
            "w_des": float(w_des),
            "w_cap": None if w_cap == "-" else float(w_cap),
            "tau_ask": float(tau_ask), "tau_spent": float(tau_spent),
            "nose": (float(nx), float(ny), float(nz)),
            "target": (float(tx), float(ty), float(tz)),
        })
    # Kill-rot rows, same list, keys the row does not print set to None.
    for m in KILLROT_RE.finditer(out):
        t, wmag, wxyz, tau_ask, tau_spent = m.groups()
        rows.append({
            "t": float(t), "mode": "killrot", "E": None,
            "w_slew": None, "w_roll": None, "w_third": None,
            "wnorm": float(wmag), "alpha": None, "alpha_full": None,
            "Ieff": None, "w_des": None, "w_cap": None,
            "tau_ask": float(tau_ask),
            "tau_spent": float(tau_spent),
            "nose": None, "target": None,
            "wxyz": tuple(float(v) for v in wxyz.split()),
        })
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
        (t, body, w, h, mtype, level, ar, ag, ab, sr, sg, sb, shade, ocean,
         sites, tmin, tmean, tmax, elo, ehi, trlo, trhi,
         lag_tau, peak_deg, sun_lon,
         rel_lo, rel_hi, rel_off_lo, rel_off_hi, mean4, datum4, anchor, note,
         wmin, wmean, wmax, welo, wehi, wrlo, wrhi, wrows, wcells,
         wbands, warrowrows, wrowsfrom, wnote) = m.groups()
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
            "temp_mean4": None if mean4 is None else float(mean4),
            "datum_mean4": None if datum4 is None else float(datum4),
            "anchor": None if anchor is None else float(anchor),
            "note": note,
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


def parse_solar(out):
    # A dashed column reads as None, same as every other optional column here: the
    # row prints '-' when the WORLD states nothing (no authored albedo, no live star
    # in range), and a CHECK that wants "absent" must ask for None, not for 0.0.
    rows = []
    for m in SOLAR_RE.finditer(out):
        (t, star, dist, flux, elev, flat, body, albedo, ground_t, bb_t,
         underfoot_t, swing, lag, air_t, prof_t, wgt, follow) = m.groups()
        num = lambda s: None if s == "-" else float(s)
        rows.append({
            "t": float(t), "star": star, "body": body,
            "dist": num(dist), "flux": num(flux), "elev": num(elev),
            "flat": num(flat), "albedo": num(albedo),
            "ground_T": num(ground_t), "bb_T": num(bb_t),
            "underfoot_T": num(underfoot_t), "swing": num(swing),
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
        (t, nx, ny, nz, rx, ry, rz, ux, uy, uz, wx, wy, wz, wnorm) = m.groups()
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
        (t, ship, powered, gen, const_draw, charge, capacity, active) = m.groups()
        rows.append({
            "t": float(t), "ship": ship, "powered": int(powered),
            "gen": float(gen), "const_draw": float(const_draw),
            "charge": float(charge), "capacity": float(capacity),
            "active": float(active),
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
            # #450: beta is the yaw-plane half of the same AeroFrame pair, so it
            # goes None on exactly the same test as aoa.
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


def parse_orbinfo(out):
    """One row per --info-log ORBITAL-window dump (#498). apo_* / lan / lpe print
    "-" and become None; the peri_* pair and period print whatever the conic says,
    and an open conic's inf/nan would make the strict number class fail to match
    the whole line -- which drops the row silently -- so those take any token and
    become None here instead."""
    def num(s):
        try:
            return float(s)
        except (TypeError, ValueError):
            return None
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
            # Debug window's Prg (air) is the air-relative pair to these (#498).
            "prg": float(prg), "rtg": float(rtg),
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
                rows.append({"t": t, "name": name, "a": float(a), "q": float(q),
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
         beta, cpl, cla, sm, profile, formcp, airT, airP, airA, mach, ld) = m.groups()
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
    orbit = parse_orbit(world)
    dbg = parse_dbg(world)
    xfer = parse_xfer(world)
    porkchop = parse_porkchop(world)
    surfmap = parse_surfmap(world)
    solar = parse_solar(world)
    att = parse_att(world)
    ascent = parse_ascent(world)
    eva = parse_eva(world)
    fuel = parse_fuel(world)
    drainlog = parse_drainlog(world)
    power = parse_power(world)
    dv = parse_dv(world)
    mass = parse_mass(world)
    drag = parse_drag(world)
    ctrl = parse_ctrl(world)
    surface = parse_surface(world)
    uilist = parse_uilist(world)
    shake = parse_shake(world)
    cam = parse_cam(world)
    terrain = parse_terrain(world)
    surf = parse_surf(world)
    orbinfo = parse_orbinfo(world)
    scene = parse_scene(out)
    mapb = parse_mapb(world)
    plane = parse_plane(world)
    slew = parse_slew(world)
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
    ns = {
        "out": out, "orbit": orbit, "dbg": dbg, "xfer": xfer,
        "porkchop": porkchop, "surfmap": surfmap, "solar": solar, "att": att, "ascent": ascent, "eva": eva,
        "fuel": fuel, "drainlog": drainlog, "power": power, "dv": dv, "mass": mass,
        "drag": drag, "shake": shake,
        "rec": rec,
        "ctrl": ctrl,
        "surface": surface,
        "uilist": uilist,
        "cam": cam,
        "terrain": terrain,
        "surf": surf, "orbinfo": orbinfo, "mapb": mapb, "plane": plane, "slew": slew,
        "perf": perf,
        "scene": scene,
        "toast": toast,
        # None unless the run printed its --timeout accounting (#402).
        "simreport": simreport,
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
        line = check_verdict(expr, check_globals)
        if line is not None:
            diag.append(line)

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
        except ValueError as e:
            # Like case_kind(): a case that does not parse must not vanish from
            # a name selection as if it never existed -- and this one is worse,
            # since a missed NAME match prints "no case matches".
            warn("warning: %s" % e)
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
    if fails:
        for f in fails:
            print("SELF-TEST FAIL: %s" % f)
        return 1
    print("e2e runner self-test ok: dead-shape CHECKs (tuple, container, "
          "uncalled lambda) refused at parse and eval; regex-shaped EXPECT/FORBID "
          "refused at parse while literal text containing brackets stays legal; "
          "re.search, walrus, comprehension and immediately-called lambda CHECKs "
          "accepted; CHECK failed/raised wording intact")
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
