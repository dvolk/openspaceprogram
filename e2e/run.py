#!/usr/bin/env python3
"""E2E battery: run e2e/cases/*.txt against the game binary.

Usage: python3 e2e/run.py [selectors] [--jobs N] [--force] [--game PATH]
Case keys: NAME ARGS EXPECT FORBID CHECK LIMIT WRITE. CHECK sees out and
the parsed logs (orbit/dbg/att/eva/fuel/drainlog/drag/shake/terrain/surf/
xfer/porkchop/surfmap) plus first/last/re. ARGS tokenizes like a shell, so
a value may be quoted to carry spaces. Full battery and --jobs>2 need
--force. Artifacts land in tmp/e2e/runs/<stamp>/ + history.csv.
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

ORBIT_RE = re.compile(
    r"\[orbitlog\]\s+t=([\d.]+)s\s+frame=\"([^\"]*)\"\s+r=([-\d.e+]+) m\s+"
    r"v=([-\d.e+]+) m/s\s+sma=([-\d.e+]+) m\s+ecc=([-\d.e+]+)\s+"
    r"peri=([-\d.e+]+) m\s+apo=([-\d.e+]+) m\s+inc=([-\d.e+]+) deg\s+"
    r"T=([-\d.e+]+) s\s+ttAp=([-\d.e+]+) s\s+ttPe=([-\d.e+]+) s\s+"
    r"\|h\|=([-\d.e+]+) m2/s\s+E=([-\d.e+]+) J/kg"
    # Plane angles (#171) trail the line; optional so nothing else breaks.
    # lan/lpe are either a number followed by " deg" or a bare "-".
    r"(?:\s+plane=(\w+)\s+lan=(\S+)(?: deg)?\s+lpe=(\S+)(?: deg)?)?"
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
    r"roll=([-\d.e+]+) deg\s+hdg=([-\d.e+]+) deg\s+acc=([-\d.e+]+) m/s2\s+"
    r"body=\"([^\"]*)\"\s+bme=\"([^\"]*)\"\s+sit=\"([^\"]*)\""
)
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
            elif key == "WRITE":
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
        (t, frame, r, v, sma, ecc, peri, apo, inc, T, ttAp, ttPe, h, E,
         plane, lan, lpe) = m.groups()
        rows.append({
            "t": float(t), "frame": frame, "r": float(r), "v": float(v),
            "sma": float(sma), "ecc": float(ecc), "peri": float(peri),
            "apo": float(apo), "inc": float(inc), "T": float(T),
            "ttAp": float(ttAp), "ttPe": float(ttPe), "h": float(h),
            "E": float(E),
            # None on a line without the #171 tail.
            "plane": plane, "lan": lan, "lpe": lpe,
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
        (t, agl, asl, vs, hs, lat, lon, pitch, roll, hdg, acc,
         body, bme, sit) = m.groups()
        rows.append({
            "t": float(t), "alt_agl": float(agl), "alt_asl": float(asl),
            "vs": float(vs), "hs": float(hs), "lat": float(lat),
            "lon": float(lon), "pitch": float(pitch), "roll": float(roll),
            "hdg": float(hdg), "acc": float(acc),
            "body": body, "bme": bme, "sit": sit,
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
    # Clean ImGui layout: persisted imgui.ini would make UI clicks flaky.
    try:
        os.remove(os.path.join(REPO_ROOT, "imgui.ini"))
    except FileNotFoundError:
        pass

    # Scratch --data-dir per case so settings/saves cannot leak between runs.
    data_dir = os.path.join(REPO_ROOT, "tmp", "e2e", "data", case["name"])
    os.makedirs(data_dir, exist_ok=True)
    try:
        os.remove(os.path.join(data_dir, "settings.json"))
    except FileNotFoundError:
        pass

    # WRITE staging (after cleanup so a fixture is live for exactly this
    # case). settings.json goes in the scratch data dir (datadir.h).
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
    # Own process group so a timeout can kill wine + the game PE + Xvfb.
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

    passed = (not diag)
    return passed, diag, out, duration


def select_cases(case_files, selectors):
    """Filter to cases whose NAME or filename contains any selector (case-insensitive)."""
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
    # Default to serial under Wine: concurrent wine+Xvfb+llvmpipe instances
    # overload a dev box and go flaky. An explicit --jobs wins.
    if args.jobs == DEFAULT_JOBS and GAME and wine_for(GAME):
        args.jobs = 1
    selectors = args.selectors

    # Full battery and --jobs>2 are expensive; require --force.
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
