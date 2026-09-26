#!/usr/bin/env python3
"""E2E battery: launch the game and check the result.

The binary defaults to ./osp; point at another build with --game
(e.g. the new tree: --game build/linux-v2-znver3/release/osp).
A windows artifact (--game build/windows-.../release/osp.exe) runs
under Wine; that path still needs Xvfb (Wine renders to X).

Linux GL: if a DRM render node is usable (/dev/dri/renderD*), the game
runs with SDL_VIDEODRIVER=offscreen (EGL on the GPU -- radeonsi when the
host passes the device through, llvmpipe if not). Otherwise it runs under
Xvfb + GLX (software GL), which is what cloud CI does.

Usage:
  python3 e2e/run.py orbit      run only cases matching "orbit"
  python3 e2e/run.py smoke 02   run cases matching "smoke" or "02"
  python3 e2e/run.py --force    run all cases (needs --force; see below)
  python3 e2e/run.py --jobs 4   run up to 4 cases in parallel
                                (default: 2; --jobs 1 = serial)

A full battery (no selectors) or --jobs > 2 is refused without --force:
a full battery takes a LONG time, and software-GL runs still use ~500%
CPU per case. A targeted selector and the default 2 jobs usually cover
what a change needs; use --force to run it anyway.

Every run is persisted (always, pass or fail) under tmp/e2e/:
  runs/<UTCstamp>/<case>.log   the case's captured output (partial on timeout)
  runs/<UTCstamp>/summary.json machine-readable run + per-case summary
  runs/<UTCstamp>/summary.txt  the human summary printed to the console
  history.csv                  one appended row per run (datetime, commit,
                               renderer, cases, pass/fail counts, duration)

Each test is a case file in e2e/cases/*.txt with these keys (one per line,
`#` starts a comment):

  NAME <label>                 shown in the summary
  ARGS <game args>             may span lines; split on whitespace, passed to ./osp
  EXPECT <substring>           must occur in the output (repeat for more)
  FORBID <substring>           must NOT occur in the output (repeat)
  CHECK <python expression>    must be truthy (repeat); see the namespace below
  LIMIT <seconds>              runner hard timeout for this case (default 120)
  WRITE <path> <content>       write <content> to <path> (REPO_ROOT-relative)
                               before the game launches -- stage a file for
                               just this case. "settings.json" is special:
                               it lands in the case's scratch data directory
                               (each case runs with --data-dir tmp/e2e/data/
                               <name>), where the game reads it (datadir.h)

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
          grounded (0/1), pos (3-tuple), vel (3-tuple), face (3-tuple,
          the kerbal's face axis), alt (m above the analytic terrain),
          mass (kg; None if the binary predates the field)
  fuel    list of dicts, one per [fuel] line: t, ship, groups
          (group id -> {resource: (current, capacity, per-tank currents)}),
          links (a list of (from_group, to_group) fuel-link pairs)
  drainlog list of dicts, one per [drainlog] line: t, dt (the sample
          interval, s), ship, thrust (N, the thrust delivered in the
          sample's tick), rates (group id -> drain rate in kg/s,
          H2+LOX combined; a group only appears while it carries fuel)
  drag    list of dicts, one per [drag] line: t, alt (m above the surface),
          rho (kg/m^3, the air density), v (m/s, air-relative speed),
          F (N, the drag force magnitude), cd (the drag coefficient)
  shake   list of dicts, one per [shakelog] line: t, a (m/s^2, the felt
          acceleration), amp (m, the shake's target amplitude),
          off (3-tuple, the live smoothed offset)
  terrain list of dicts, one per [terrain] line (--terrain-log): t, body
          (the LOCAL body's name), patches (alive patch count), deepest
          (the deepest leaf's depth), max_depth (the body's subdivision
          stop), collision (leaves carrying a Bullet body), deep_off (m,
          the camera -> nearest deepest leaf distance: the detail belongs
          under the camera), cam_r (m, the camera -> body centre distance)
  surf    list of dicts, one per [surfinfo] line (--info-log): t, alt_agl,
          alt_asl, vs, hs, lat, lon, pitch, roll, hdg (degrees), acc
          (m/s^2). Printed while paused too, so a case can compare the
          paused readout against the one after the first unpaused tick.
  first / last                 first() / last() of a list
  re      the stdlib `re` module (regex checks against `out`)
Example:  CHECK last(orbit)["E"] > first(orbit)["E"]

Stdlib only. Run from anywhere; the repo root is derived from this file.
"""

import argparse
import csv
import datetime
import glob
import json
import os
import re
import shutil
import signal
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CASES_DIR = os.path.join(REPO_ROOT, "e2e", "cases")
# The game binary to launch: --game wins, else the legacy ./osp. (The new
# build tree puts it under build/<os>-<march>-<mtune>/<config>/, so that
# flow passes --game explicitly.)
GAME = None
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
    r"vel=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+"
    r"face=\[([-\d.]+) ([-\d.]+) ([-\d.]+)\]\s+alt=([-\d.]+) m"
    r"(?:\s+mass=([-\d.]+)kg)?"
)
DRAINLOG_RE = re.compile(
    r"\[drainlog\]\s+t=([\d.]+)s\s+dt=([\d.]+)s\s+ship=\"([^\"]*)\"\s+"
    r"thrust=([-\d.]+)N\s+(.*)"
)
DRAG_RE = re.compile(
    r"\[drag\]\s+t=([\d.]+)s\s+alt=([-\d.]+) m\s+rho=([-\d.e+]+) kg/m3\s+"
    r"\|v\|=([-\d.]+) m/s\s+\|F\|=([-\d.]+) N"
    r"(?:\s+\|L\|=([-\d.]+) N)?"
    r"(?:\s+\|tau\|=([-\d.]+) Nm)?"
    r"\s+Cd=([-\d.]+)"
    r"(?:\s+A=([-\d.]+) m2)?(?:\s+AoA=([-\d.e+]+) deg)?"
)
SHAKE_RE = re.compile(
    r"\[shakelog\]\s+t=([\d.]+)s\s+a=([-\d.]+) m/s2\s+"
    r"amp=([-\d.]+) m\s+off=\[([-+\d.]+) ([-+\d.]+) ([-+\d.]+)\]"
)
TERRAIN_RE = re.compile(
    r"\[terrain\]\s+t=([\d.]+)s\s+body=\"([^\"]*)\"\s+patches=(\d+)\s+"
    r"deepest=(\d+)\s+max_depth=(\d+)\s+collision=(\d+)\s+"
    r"deep_off=([-\d.e+]+) m\s+cam_r=([-\d.e+]+) m"
)
SURF_RE = re.compile(
    r"\[surfinfo\]\s+t=([\d.]+)s\s+alt_agl=([-\d.e+]+) m\s+"
    r"alt_asl=([-\d.e+]+) m\s+vs=([-\d.e+]+) m/s\s+hs=([-\d.e+]+) m/s\s+"
    r"lat=([-\d.e+]+) deg\s+lon=([-\d.e+]+) deg\s+pitch=([-\d.e+]+) deg\s+"
    r"roll=([-\d.e+]+) deg\s+hdg=([-\d.e+]+) deg\s+acc=([-\d.e+]+) m/s2"
)
DRAINLOG_RATE_RE = re.compile(r"g(\d+)=([-\d.]+)")
FUEL_RE = re.compile(
    r"\[fuel\]\s+t=([\d.]+)s\s+ship=\"([^\"]*)\"\s+(.*)"
)
# One `gN=RES:cur/cap[tanks] [RES:cur/cap[tanks] ...]` segment per group.
# The unit repeat stops at the next group id because the unit requires a
# `:` after the name, and a group id is followed by `=` (same for
# `links=`); so it cannot run into the next group.
FUEL_GROUP_RE = re.compile(
    r"g(\d+)=((?:\s*[A-Za-z0-9]+:[-\d.]+/[-\d.]+\[[^\]]*\])*)"
)
FUEL_RES_RE = re.compile(r"([A-Za-z0-9]+):([-\d.]+)/([-\d.]+)\[([^\]]*)\]")
FUEL_LINK_RE = re.compile(r"links=(\S+)")
FUEL_LINK_PAIR_RE = re.compile(r"g?(\d+)->g?(\d+)")


def parse_cases(path):
    name = os.path.basename(path)
    args = []
    expect = []
    forbid = []
    check = []
    writes = []
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
            elif key == "WRITE":
                # "path content": the content is the rest after the path token.
                wpath, _, wcontent = rest.partition(" ")
                writes.append((wpath.strip(), wcontent.strip()))
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
        "writes": writes,
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
        (t, mode, grounded, px, py, pz, vx, vy, vz,
         fx, fy, fz, alt, mass) = m.groups()
        rows.append({
            "t": float(t), "mode": mode, "grounded": int(grounded),
            "pos": (float(px), float(py), float(pz)),
            "vel": (float(vx), float(vy), float(vz)),
            "face": (float(fx), float(fy), float(fz)),
            "alt": float(alt),
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
            # group id -> drain rate (kg/s, H2+LOX combined); a group only
            # appears while it still carries fuel.
            "rates": rates,
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
        (t, agl, asl, vs, hs, lat, lon, pitch, roll, hdg, acc) = m.groups()
        rows.append({
            "t": float(t), "alt_agl": float(agl), "alt_asl": float(asl),
            "vs": float(vs), "hs": float(hs), "lat": float(lat),
            "lon": float(lon), "pitch": float(pitch), "roll": float(roll),
            "hdg": float(hdg), "acc": float(acc),
        })
    return rows


def parse_drag(out):
    rows = []
    for m in DRAG_RE.finditer(out):
        (t, alt, rho, v, F, L, tau, cd, a, aoa) = m.groups()
        row = {
            "t": float(t), "alt": float(alt), "rho": float(rho),
            "v": float(v), "F": float(F), "cd": float(cd),
        }
        if L is not None:
            row["L"] = float(L)
        if tau is not None:
            row["tau"] = float(tau)
        if a is not None:
            row["a"] = float(a)
        if aoa is not None:
            row["aoa"] = float(aoa)
        rows.append(row)
    return rows


def first(seq):
    return seq[0]


def last(seq):
    return seq[-1]


def wine_for(game):
    """windows artifacts (.exe) run under Wine (phase 1.4,
    reports/build-tree2026_09_22/phase1-windows.md). Wine renders to X, so
    the Xvfb path below is unchanged. Returns the wine binary, or None."""
    if not game.endswith(".exe"):
        return None
    return shutil.which("wine64") or shutil.which("wine")


def have_render_node():
    """True if a DRM render node is openable, so SDL offscreen/EGL can talk
    to a real GPU (LXD gputype=physical, bare metal, ...). Cloud CI has no
    /dev/dri and gets the Xvfb + llvmpipe path instead."""
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
        # Wine always needs an X server.
        xvfb = shutil.which("xvfb-run")
        if xvfb:
            return [xvfb, "-a", wine] + [game] + args
        return [wine] + [game] + args

    # Prefer SDL offscreen (EGL) when a render node is usable: Mesa loads
    # radeonsi/llvmpipe on EGL_PLATFORM_DEVICE without Xvfb. The offscreen
    # driver is only selected via SDL_VIDEODRIVER (see launch_env).
    if have_render_node():
        return [game] + args

    if os.environ.get("DISPLAY") and shutil.which("xvfb-run") is None:
        # A real display is available and no Xvfb to fake one.
        return [game] + args
    xvfb = shutil.which("xvfb-run")
    if xvfb:
        return [xvfb, "-a"] + [game] + args
    # No Xvfb and no display: run bare; it will fail to open a window, which
    # the case will report as a failure. (Headless envs should install Xvfb.)
    return [game] + args


def launch_env(game):
    """Environment overrides for the game process, or None to inherit."""
    wine = wine_for(game)
    if wine:
        # Pin Wine's prefix in the tree (tmp/wine) so first-run init and
        # per-run state stay out of the home dir. WINEDEBUG=-all silences
        # wine's own fixme/err chatter on stderr -- one of those lines
        # ("using GL_RENDERER ...") contains "GL_" and would trip the
        # cases' FORBID GL_ checks, which are meant to catch the GAME's
        # GL errors only.
        prefix = os.path.join(REPO_ROOT, "tmp", "wine")
        os.makedirs(prefix, exist_ok=True)
        env = dict(os.environ)
        env["WINEPREFIX"] = prefix
        env["WINEDEBUG"] = "-all"
        return env
    if have_render_node():
        env = dict(os.environ)
        # Force SDL's offscreen driver: EGL on a DRM render node, no X.
        # Drop DISPLAY so SDL cannot fall through to X11/GLX (llvmpipe).
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
    # Start each case from a clean ImGui layout (window positions persist in
    # imgui.ini otherwise, which would make UI clicks non-deterministic).
    try:
        os.remove(os.path.join(REPO_ROOT, "imgui.ini"))
    except FileNotFoundError:
        pass

    # The game keeps settings.json + saves/ in its data directory
    # (datadir.h), not the repo root. Give the case a scratch one via
    # --data-dir, so a locally saved settings.json (display mode, postfx,
    # the UI knobs) or a fixture from a prior run can't leak in.
    data_dir = os.path.join(REPO_ROOT, "tmp", "e2e", "data", case["name"])
    os.makedirs(data_dir, exist_ok=True)
    try:
        os.remove(os.path.join(data_dir, "settings.json"))
    except FileNotFoundError:
        pass

    # Stage any files the case declares (WRITE): written after the cleanup
    # above, so a fixture (e.g. a rebind settings.json) is live for exactly
    # this case and the next case's cleanup removes it again. settings.json
    # goes into the scratch data directory -- that is where the game reads
    # it (datadir.h); any other path is REPO_ROOT-relative as before.
    for wpath, wcontent in case.get("writes", []):
        target = (os.path.join(data_dir, "settings.json")
                  if wpath == "settings.json"
                  else os.path.join(REPO_ROOT, wpath))
        with open(target, "w") as wf:
            wf.write(wcontent)

    cmd = build_cmd(game, case["args"] + ["--data-dir", data_dir])
    env = launch_env(game)
    diag = []
    timed_out = False
    exit_code = None
    out = ""
    # The case runs in its own process group (start_new_session) so a
    # timeout can kill the whole tree: the wine wrapper is only the
    # direct child -- killing it leaves the game PE (a grandchild)
    # orphaned, spinning at 100% CPU, and Xvfb behind it.
    t0 = time.monotonic()
    proc = subprocess.Popen(
        cmd, cwd=REPO_ROOT, stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT, env=env, start_new_session=True,
    )
    try:
        out_bytes, _ = proc.communicate(timeout=case["limit"])
        out = out_bytes.decode("utf-8", "replace")
        exit_code = proc.returncode
    except subprocess.TimeoutExpired as e:
        timed_out = True
        out = (e.output or b"").decode("utf-8", "replace")
    finally:
        if proc.poll() is None:
            # Timed out (or wedged): kill the whole group, then reap.
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
    fuel = parse_fuel(out)
    drainlog = parse_drainlog(out)
    drag = parse_drag(out)
    shake = parse_shake(out)
    terrain = parse_terrain(out)
    surf = parse_surf(out)
    ns = {
        "out": out, "orbit": orbit, "dbg": dbg, "xfer": xfer,
        "porkchop": porkchop, "surfmap": surfmap, "att": att, "eva": eva,
        "fuel": fuel, "drainlog": drainlog, "drag": drag, "shake": shake,
        "terrain": terrain,
        "surf": surf,
        "first": first, "last": last,
        "abs": abs, "len": len, "any": any, "all": all,
        "max": max, "min": min, "float": float, "int": int, "zip": zip,
        "set": set,
        "re": re,
    }
    # `ns` goes in the GLOBALS, not just locals: free variables in a
    # generator/comprehension body resolve against the globals (the locals
    # dict is invisible to them), so a check like `max(... for g in ...)`
    # would raise NameError with the names only in locals. Keeping
    # __builtins__ empty still blocks a case file from reaching open/exec.
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

    passed = (not diag)
    return passed, diag, out, duration


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
    """Parse and run one case. Never raises.

    Returns (filebase, name, passed, diag, out, duration_s) -- filebase is
    the case filename without .txt (the unique key for the .log file), and
    out is the captured game output ("" when the case never launched)."""
    filebase = os.path.splitext(os.path.basename(path))[0]
    try:
        case = parse_cases(path)
    except ValueError as e:
        # No NAME line could be read, so the filename is the label.
        return filebase, filebase, False, \
            ["bad case file: %s" % e], "", 0.0
    try:
        passed, diag, out, duration = run_case(case)
    except Exception as e:
        return filebase, case["name"], False, ["runner error: %r" % e], "", 0.0
    return filebase, case["name"], passed, diag, out, duration


def git_state():
    """(short commit or 'none', dirty or None) from the repo, best-effort.
    None = not a git repo / git unavailable -- the CSV row leaves it blank
    rather than guessing."""
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
    """tmp/e2e/runs/<UTCstamp>/, with a -2, -3, ... suffix if the name is
    taken (two runs in the same second must not share a directory).
    mkdir is atomic, so concurrent runners each get their own dir."""
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


HISTORY_FIELDS = ["datetime_utc", "commit", "dirty", "renderer", "game",
                  "jobs", "cases", "n", "passed", "failed", "run_duration_s",
                  "run_dir"]


def format_summary(results):
    """The human pass/fail table (shared by the console and summary.txt)."""
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
    """Persist a run: per-case .log files + summary.json/summary.txt in a
    fresh tmp/e2e/runs/<stamp>/ dir, and one appended row in
    tmp/e2e/history.csv (the across-runs record). Returns
    (run_dir, history_path), both absolute.

    results: run_one's (filebase, name, passed, diag, out, duration) tuples.
    meta: started/finished (datetime), commit, dirty, renderer, game
    (REPO_ROOT-relative), jobs, selectors (list).
    """
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
    # Claim header ownership atomically: O_EXCL means exactly one of several
    # concurrent runners creates the file and writes the header. A
    # pre-existing 0-byte file is also initialized (the only overwrite).
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
                        help="case name/filename substring(s) to run; "
                             "default: all cases")
    parser.add_argument("--jobs", type=int, default=DEFAULT_JOBS,
                        help="max cases to run in parallel (1 = serial; "
                             "default: 2)")
    parser.add_argument("--force", action="store_true",
                        help="run even when heavy (full battery, or "
                             "--jobs > 2), despite the warning above")
    parser.add_argument("--game", default=None,
                        help="game binary to launch (default: ./osp)")
    args = parser.parse_args()
    if args.jobs < 1:
        parser.error("--jobs must be >= 1")
    global GAME
    GAME = args.game
    if wine_for(GAME or ""):
        renderer = "wine+xvfb"
    elif have_render_node():
        renderer = "egl-offscreen"
        print("GL: SDL offscreen/EGL (DRM render node present)")
    else:
        renderer = "xvfb-glx"
        print("GL: Xvfb/GLX software (no /dev/dri/renderD*)")
    commit, dirty = git_state()
    # Wine cases default to serial: two concurrent wine + Xvfb + llvmpipe
    # instances (each software-rendering the whole game) overload a dev
    # box and make the cases flaky (measured: parallel runs intermittently
    # die mid-boot with exit 1; serial is stable). An explicit --jobs wins.
    if args.jobs == DEFAULT_JOBS and GAME and wine_for(GAME):
        args.jobs = 1
    selectors = args.selectors

    # A full battery and --jobs > 2 are both expensive (each case spins a
    # full game loop; software GL is ~500% CPU), so neither runs without an
    # explicit --force -- the message is the nudge to pick a selector /
    # fewer jobs.
    reasons = []
    if not selectors:
        reasons.append("a full battery takes a LONG time")
    if args.jobs > 2:
        reasons.append("each case already runs a full game loop -- more "
                       "parallelism is usually overkill")
    if reasons and not args.force:
        print("Refusing to run: " + "; ".join(reasons) + ".")
        print("Consider whether you really need it -- a targeted selector "
              "(e.g. `python3 e2e/run.py orbit`) and the default 2 jobs "
              "usually cover what a change needs.")
        print("Run it anyway with --force.")
        return 1

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
    started = datetime.datetime.now(datetime.timezone.utc)
    if args.jobs == 1:
        results = [run_one(p) for p in case_files]
    else:
        with ThreadPoolExecutor(max_workers=args.jobs) as pool:
            results = list(pool.map(run_one, case_files))
    finished = datetime.datetime.now(datetime.timezone.utc)

    # Persist before printing, so the console can point at the artifacts.
    # A failure here (full disk, read-only tmp/) must not swallow the
    # summary or the pass/fail exit code that `make e2e` relies on.
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
