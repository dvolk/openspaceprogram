#!/usr/bin/env python3
"""aero_quant.py -- quantitative aero measurements + a plain-English report.

The "is the aero soupy?" instrument. Runs the ship headless (xvfb, like the
e2e battery) in two short flights and reduces the --drag-log time series to
the numbers that define how the flight model feels:

  PROGRADE AREA     silhouette area with the nose into the flow. The whole
                    point of the hull-silhouette drag model: a prograde
                    rocket shows ONE end face (pi*r^2), not its side.
  AREA SWING        broadside / prograde area ratio (how much the drag grows
                    as the ship goes from edge-on to broadside).
  MAX THRUST SPEED  the speed it reaches at full thrust (thrust vs
                    weight+drag; the "max speed feels low?" number).
  TERMINAL VELOCITY the free-fall speed plateau (drag = weight; the
                    "~60 m/s feels low?" number).
  TUMBLE            does the ship hold its nose to the flow in free fall, or
                    tumble broadside (AoA swinging past 90, area ballooning)?
                    A tumbling ship presents its SIDE area, so it falls far
                    slower than the prograde ideal -- this is usually the
                    real cause of "drag feels too high."

It also prints the theoretical free-fall terminal velocity for both the
prograde and the broadside area (from the ship's mass + the body's air), so
you can see exactly where the measured value sits between the two.

Stdlib only (reuses e2e/run.py's parser + xvfb wrapper). Not a pass/fail
battery -- it reports numbers and flags the tumble.
"""

import argparse
import json
import math
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import run as e2e  # noqa: E402  (REPO_ROOT, build_cmd, parse_drag)

REPO = e2e.REPO_ROOT
KRB_G = 9.81  # Kerbin surface gravity (m/s^2); see res/ksp_system.json


def ship_mass_and_radius(ship_path, parts_path):
    """(dry_mass, fueled_mass, max_radius, ve, flow) from the parts catalog.

    A part's `mass` is its DRY structure only (the propellant rides
    `capacity`, like the game's effectiveMass). So the ship's FUELED mass is
    the sum of part masses plus every tank's capacity, and the DRY (empty)
    mass is that minus the burnable propellant (H2/LOX/jetfuel). EC is
    charge, not mass, so it never counts. max_radius is the widest part (its
    end face is the prograde cross-section)."""
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
        # Inert resources (mono, life support) still weigh in at the end.
        for res in ("hydrazine", "oxygen", "water", "food"):
            dry += cap.get(res, 0.0)
        # The rated thrust is 2*flow*ve (H2 + LOX both end up in the plume),
        # so the propellant flow is 2*flow and ve is the exhaust velocity.
        if d.get("fuel_rate", 0.0) > 0 and d.get("exhaust_velocity", 0.0) > 0 \
                and not d.get("jet"):
            ve = d["exhaust_velocity"]
            flow = 2.0 * d["fuel_rate"]
    fueled = dry + fuel  # structure + inert + full propellant
    return dry, fueled, maxr, ve, flow


def flight(presses, timeout, body, ship, cd, autopilot=None):
    """Run one headless flight; return the parsed --drag-log rows.

    autopilot (e.g. "radial-out") engages the slew hold at startup so the
    ship stays pointed where you want it -- needed for a straight-up boost
    (the radial-out hold) that a hand-held stick can't maintain headless."""
    game = os.path.join(REPO, "osp")
    args = [
        "--body", body, "--scenario", "pad", "--ship", ship,
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
    """Free-fall terminal speed = the max airspeed reached AFTER the apex.

    Rows are time-ordered; the apex is the highest altitude. The descent is
    every sample strictly after it; the plateau speed is the max v there."""
    if not rows:
        return 0.0
    apex_t = max(rows, key=lambda d: d["alt"])["t"]
    descent = [d for d in rows if d["t"] > apex_t]
    return max((d["v"] for d in descent), default=0.0)


def boost_budget(args, dry, fueled, maxr, ve, flow):
    """Full vertical boost (radial-out autopilot, straight up): the APEX.

    The radial-out hold keeps the nose straight up, so the delta-v becomes
    ALTITUDE (a clean vertical boost) instead of an arc. The apex is the
    headline number. The losses (gravity + drag) are reported for context:
      gravity loss = g x t_burn -- every second the engine fires, ~9.8 m/s of
                   delta-v is spent holding the ship up (it becomes altitude,
                   not speed). A long, low-thrust burn loses a lot here.
      drag loss    = integral(F/m)dt over the CLIMB -- the velocity the air
                   bleeds off. (Only the climb counts; the fall's drag is the
                   ship coming back down, not a loss of the delta-v.)
    """
    # Radial-out autopilot (the "straight up" hold) + full-throttle burn.
    # R ramps the throttle, T fires; hold T long enough for a full burn.
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
    # --- Flight 1: prograde thrust (area + max speed) ---
    # R = throttle up (hold to ramp), T = thrust (hold to fire).
    prograde = flight(["500,4000,R", "800,8000,T"], 12, args.body, args.ship, cd)
    near = [d for d in prograde
            if d.get("aoa") is not None and abs(d["aoa"]) < 15 and d.get("a")]
    # The row that shows the prograde area also carries the model's REAL
    # coefficient for that orientation (the logged Cd = the parts'
    # area-weighted mean, cd_ship). Grab it so the theoretical terminal
    # velocity below uses cd x cd_ship, not the bare --cd master scale.
    pro_row = min(near, key=lambda d: d["a"]) if near else None
    pro_area = pro_row["a"] if pro_row else None
    cd_pro = pro_row["cd"] if pro_row else cd
    max_speed = max((d["v"] for d in prograde), default=0.0)
    # --- Flight 2: thrust up, cut, free fall (terminal velocity + tumble) ---
    fall = flight(["500,6000,R", "800,7000,T"], 26, args.body, args.ship, cd)
    vt = terminal_velocity(fall)
    # The tumble: how far the nose swings from the flow during the fall, and
    # how far the area balloons (broadside). The descent is after the apex.
    cd_broad = cd  # the model's coefficient at the broadside (fall) orientation
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
    # Theoretical free-fall terminal velocity: drag = weight ->
    #   0.5*rho*Cd*A*v^2 = m*g   =>   v = sqrt(2*m*g / (rho*Cd*A))
    # Use the sea-level density the body reports (the fall starts low).
    rho0 = 1.225  # Kerbin sea-level density; the fall samples carry it too
    if fall:
        rho0 = min(d["rho"] for d in fall if d.get("rho")) or rho0
    # The model's real coefficient is --cd (master) x cd_ship (the parts'
    # area-weighted mean), so the theory uses that product, not the bare --cd.
    vt_pro = math.sqrt(2.0 * fueled * KRB_G / (rho0 * cd * cd_pro * pro_area)) if pro_area else 0.0
    vt_broad = math.sqrt(2.0 * fueled * KRB_G / (rho0 * cd * cd_broad * broad_area)) if broad_area else 0.0
    expected_pro = math.pi * maxr * maxr  # one end face of the widest part

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
    # --- Flight 3: full vertical boost -- the apex (the "how high can it
    #    go straight up?" number) and where the delta-v went ---
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
    parts_path = os.path.join(REPO, "res/parts.json")
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
