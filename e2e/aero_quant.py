#!/usr/bin/env python3
"""Quantitative aero measurements: prograde/broadside area, max speed,
terminal velocity, tumble flag, and a straight-up boost budget."""

import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run as e2e  # noqa: E402

REPO = e2e.REPO_ROOT
KRB_G = 9.81  # Kerbin surface gravity (m/s^2)


def ship_mass_and_radius(ship_path, parts_path):
    """(dry_mass, fueled_mass, max_radius, ve, flow) from the parts catalog.
    Part `mass` is dry structure; propellant rides `capacity`. EC is charge,
    not mass. max_radius is the widest part (prograde cross-section)."""
    with open(ship_path) as f:
        ship = json.load(f)
    with open(parts_path) as f:
        parts = {p["name"]: p for p in json.load(f)["parts"]}
    dry = fuel = 0.0
    maxr = 0.0
    ve = flow = 0.0  # first rocket engine's exhaust velocity + prop flow (kg/s)
    for ps in ship["parts"]:
        d = parts.get(ps["part"])
        if d is None:
            continue
        dry += d.get("mass", 0.0)
        maxr = max(maxr, d.get("radius", 0.0))
        cap = d.get("capacity", {})
        for res in ("hydrogen", "lox", "jetfuel"):
            fuel += cap.get(res, 0.0)
        for res in ("hydrazine", "oxygen", "water", "food"):
            dry += cap.get(res, 0.0)
        # Rated thrust = total propellant flow * ve (H2+LOX chemical, H2 nuclear).
        prop = d.get("propellant", {})
        if sum(prop.values()) > 0 and d.get("exhaust_velocity", 0.0) > 0 \
                and not d.get("jet"):
            ve = d["exhaust_velocity"]
            flow = sum(prop.values())
    fueled = dry + fuel
    return dry, fueled, maxr, ve, flow


def flight(presses, timeout, body, ship, cd, autopilot=None):
    """One headless flight; returns the parsed --drag-log rows.
    autopilot engages the slew hold at startup (needed for a straight-up boost)."""
    game = os.path.join(REPO, "osp")
    name = os.path.basename(ship)
    if name.endswith(".json"):
        name = name[:-5]
    args = [
        "--startship", "%s,%s,%s,pad" % (name, ship, body),
        "--time-accel", "1", "--timeout", str(timeout),
        "--drag-log", "--drag-cd", str(cd),
    ]
    if autopilot:
        args += ["--autopilot", autopilot]
    cmd = e2e.build_cmd(game, args)
    for p in presses:
        cmd += ["--sim-press", p]
    proc = subprocess.run(cmd, cwd=REPO, stdout=subprocess.PIPE,
                          stderr=subprocess.STDOUT, timeout=timeout + 30)
    return e2e.parse_drag(proc.stdout.decode("utf-8", "replace"))


def terminal_velocity(rows):
    """Free-fall terminal speed = max airspeed after the apex."""
    if not rows:
        return 0.0
    apex_t = max(rows, key=lambda d: d["alt"])["t"]
    descent = [d for d in rows if d["t"] > apex_t]
    return max((d["v"] for d in descent), default=0.0)


def boost_budget(args, dry, fueled, maxr, ve, flow):
    """Full vertical boost (radial-out hold): apex + where the delta-v went.
    gravity loss = g * t_burn; drag loss = integral(F/m)dt over the climb."""
    # Radial-out autopilot + full-throttle burn (R ramps throttle, T fires).
    rows = flight(["400,3000,R", "700,120000,T"], 170, args.body, args.ship,
                  args.cd, autopilot="radial-out")
    if not rows:
        return None
    apex = max(rows, key=lambda d: d["alt"])
    apex_t = apex["t"]
    t_burn = (fueled - dry) / flow if flow > 0 else 0.0
    g_loss = KRB_G * t_burn
    # Drag loss (delta-v units) over the CLIMB only: sum(F dt)/m, average mass.
    m_avg = 0.5 * (fueled + dry)
    drag_dv = 0.0
    climb = [d for d in rows if d["t"] <= apex_t]
    for i in range(1, len(climb)):
        dt = climb[i]["t"] - climb[i - 1]["t"]
        if dt <= 0:
            continue
        drag_dv += climb[i]["F"] * dt / m_avg
    dv_total = ve * math.log(fueled / dry) if dry > 0 else 0.0
    return {
        "apex_alt": apex["alt"], "apex_speed": apex["v"],
        "dv_total": dv_total, "t_burn": t_burn,
        "g_loss": g_loss, "drag_dv": drag_dv,
    }


def report(args, dry, fueled, maxr, ve, flow):
    cd = args.cd
    # Flight 1: prograde thrust (area + max speed). R = throttle, T = thrust.
    prograde = flight(["500,4000,R", "800,8000,T"], 12, args.body, args.ship, cd)
    near = [d for d in prograde
            if d.get("aoa") is not None and abs(d["aoa"]) < 15 and d.get("a")]
    # Logged Cd is the parts' area-weighted mean (cd_ship); use cd*cd_ship
    # for the theoretical terminal velocity, not the bare --cd master scale.
    pro_row = min(near, key=lambda d: d["a"]) if near else None
    pro_area = pro_row["a"] if pro_row else None
    cd_pro = pro_row["cd"] if pro_row else cd
    max_speed = max((d["v"] for d in prograde), default=0.0)
    # Flight 2: thrust up, cut, free fall (terminal velocity + tumble).
    fall = flight(["500,6000,R", "800,7000,T"], 26, args.body, args.ship, cd)
    vt = terminal_velocity(fall)
    cd_broad = cd
    if fall:
        apex_t = max(fall, key=lambda d: d["alt"])["t"]
        desc = [d for d in fall if d["t"] > apex_t]
        broad_row = max((d for d in desc if d.get("a")), key=lambda d: d["a"],
                        default=None)
        broad_area = broad_row["a"] if broad_row else (pro_area or 0.0)
        if broad_row:
            cd_broad = broad_row["cd"]
        max_aoa = max((abs(d["aoa"]) for d in desc if d.get("aoa") is not None),
                      default=0.0)
        tumbling = (max_aoa > 90.0) or (
            (pro_area or 0.0) > 0 and broad_area > 2.0 * pro_area)
    else:
        broad_area, max_aoa, tumbling = 0.0, 0.0, False

    swing = (broad_area / pro_area) if pro_area else 0.0
    # Terminal velocity theory: 0.5*rho*Cd*A*v^2 = m*g => v = sqrt(2*m*g/(rho*Cd*A)).
    rho0 = 1.225  # Kerbin sea-level density
    if fall:
        rho0 = min(d["rho"] for d in fall if d.get("rho")) or rho0
    # Real coefficient is --cd (master) x cd_ship (parts' area-weighted mean).
    vt_pro = math.sqrt(2.0 * fueled * KRB_G / (rho0 * cd * cd_pro * pro_area)) if pro_area else 0.0
    vt_broad = math.sqrt(2.0 * fueled * KRB_G / (rho0 * cd * cd_broad * broad_area)) if broad_area else 0.0
    expected_pro = math.pi * maxr * maxr

    def line(label, value, unit=""):
        return "  %-20s %s" % (label + ":", value + (" " + unit if unit else ""))

    print("=" * 62)
    print("quantitative aero:  %s on %s   (cd=%s)" % (os.path.basename(args.ship), args.body, cd))
    print("=" * 62)
    print(line("mass (fueled / dry)", "%.0f / %.0f kg" % (fueled, dry)))
    print(line("widest part", "%.2f m (r)" % maxr))
    print("-" * 62)
    print(line("prograde area", "%.2f m2" % pro_area if pro_area else "  n/a"))
    print(line("  expected (pi*r^2)", "%.2f m2" % expected_pro))
    print(line("broadside area", "%.2f m2" % broad_area))
    print(line("area swing", "%.2fx" % swing))
    print("-" * 62)
    print(line("max thrust speed", "%.0f m/s" % max_speed))
    print(line("terminal velocity", "%.0f m/s" % vt))
    print(line("  theory, prograde", "%.0f m/s" % vt_pro))
    print(line("  theory, broadside", "%.0f m/s" % vt_broad))
    print("-" * 62)
    if tumbling:
        print("  TUMBLE in free fall: yes -- the nose swings to ~%.0f deg off the" % max_aoa)
        print("  flow and the area balloons to ~%.0f m2 (broadside). The ship" % broad_area)
        print("  is pitch-neutral (center of pressure ~ center of mass), so it")
        print("  doesn't weathervane nose-first; it tumbles, presenting its SIDE")
        print("  area. That's why terminal velocity sits near the broadside")
        print("  theory, not the prograde theory -- 'drag feels too high' is")
        print("  really 'it's broadside, not edge-on'.")
    else:
        print("  TUMBLE in free fall: no -- the nose holds to the flow")
        print("  (max AoA %.0f deg); it falls edge-on at the prograde area." % max_aoa)
    # Flight 3: full vertical boost (apex + delta-v budget).
    bb = boost_budget(args, dry, fueled, maxr, ve, flow)
    if bb:
        print("-" * 62)
        print(line("apex (straight up)", "%.0f m @ %.0f m/s" % (bb["apex_alt"], bb["apex_speed"])))
        print(line("delta-v (engine)", "%.0f m/s" % bb["dv_total"]))
        print(line("  burn time", "%.0f s" % bb["t_burn"]))
        print(line("  gravity loss", "-%.0f m/s" % bb["g_loss"],
                  "(g x burn time; becomes altitude)"))
        print(line("  drag loss", "-%.0f m/s" % bb["drag_dv"],
                  "(bled off by the air, on the way up)"))
        print("=" * 62)
        return {
            "pro_area": pro_area, "expected_pro": expected_pro,
            "broad_area": broad_area, "swing": swing,
            "max_speed": max_speed, "vt": vt,
            "vt_pro": vt_pro, "vt_broad": vt_broad, "tumbling": tumbling,
            "apex_alt": bb["apex_alt"], "apex_speed": bb["apex_speed"],
        }
    print("=" * 62)
    return {
        "pro_area": pro_area, "expected_pro": expected_pro,
        "broad_area": broad_area, "swing": swing,
        "max_speed": max_speed, "vt": vt,
        "vt_pro": vt_pro, "vt_broad": vt_broad, "tumbling": tumbling,
    }


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ship", default="res/ships/racer.json",
                    help="ship def JSON (default: res/ships/racer.json)")
    ap.add_argument("--body", default="Kerbin",
                    help="body the ship starts on (default: Kerbin)")
    ap.add_argument("--cd", type=float, default=1.2,
                    help="drag coefficient (default 1.2, the game default)")
    ap.add_argument("--boost-only", action="store_true",
                    help="only run the full-boost delta-v budget (skip the "
                         "prograde/terminal-velocity flights)")
    args = ap.parse_args()

    ship_path = os.path.join(REPO, args.ship)
    parts_path = os.path.join(REPO, "res/data/parts.json")
    dry, fueled, maxr, ve, flow = ship_mass_and_radius(ship_path, parts_path)
    if args.boost_only:
        bb = boost_budget(args, dry, fueled, maxr, ve, flow)
        if not bb:
            print("no flight data")
            return 1
        print("straight-up boost:  %s on %s" % (os.path.basename(args.ship), args.body))
        print("  apex (straight up) %.0f m @ %.0f m/s" % (bb["apex_alt"], bb["apex_speed"]))
        print("  delta-v (engine)   %.0f m/s   (burn %.0f s)" % (bb["dv_total"], bb["t_burn"]))
        print("  gravity loss       -%.0f m/s  (g x burn time; becomes altitude)" % bb["g_loss"])
        print("  drag loss          -%.0f m/s  (bled off by the air, going up)" % bb["drag_dv"])
        return 0
    report(args, dry, fueled, maxr, ve, flow)
    return 0


if __name__ == "__main__":
    sys.exit(main())
