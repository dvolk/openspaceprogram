#!/usr/bin/env python3
"""Stage / mass / authority table for a ship def (offline paper model).

Mirrors the game's staging rule (vehicle.cpp:1342, game.cpp:1995):
activeStage_ starts at the highest part stage; an engine is ignited once
part.stage >= activeStage_ and stays lit; a decoupler at the CURRENT counter
value drops its subtree when the counter steps off it.
"""
import json
import math
import os
import sys

REPO = "/home/ubuntu/openspaceprogram"
PARTS = {p["name"]: p for p in json.load(open(REPO + "/res/data/parts.json"))["parts"]}
PROPS = ("hydrogen", "lox", "methane", "jetfuel")


def load(path):
    d = json.load(open(path))
    byid, kids, order = {}, {}, []
    for p in d["parts"]:
        pid = p.get("id", p["part"])
        byid[pid] = p
        order.append(pid)
        if p.get("parent"):
            kids.setdefault(p["parent"], []).append(pid)
    return byid, kids, order


def subtree(kids, pid):
    out = [pid]
    for c in kids.get(pid, []):
        out += subtree(kids, c)
    return out


def mass(byid, kids, pid, fueled=True):
    d = PARTS[byid[pid]["part"]]
    m = d.get("mass", 0.0)
    for r, v in d.get("capacity", {}).items():
        if r == "ec" or (not fueled and r in PROPS):
            continue
        m += v
    for c in kids.get(pid, []):
        m += mass(byid, kids, c, fueled)
    return m


def engines_in(byid, kids, ids, active):
    """(thrust N, flow kg/s, best ve) for ignited rocket engines in `ids`."""
    thr = flow = 0.0
    ve = 0.0
    for pid in ids:
        d = PARTS[byid[pid]["part"]]
        if d.get("jet"):
            continue
        f = sum(v for k, v in d.get("propellant", {}).items() if k in PROPS)
        if f <= 0 or d.get("exhaust_velocity", 0) <= 0:
            continue
        if byid[pid].get("stage", 1) < active:
            continue
        thr += f * d["exhaust_velocity"]
        flow += f
        ve = max(ve, d["exhaust_velocity"])
    return thr, flow, ve


def propellant(byid, kids, ids):
    tot = 0.0
    for pid in ids:
        for r, v in PARTS[byid[pid]["part"]].get("capacity", {}).items():
            if r in PROPS:
                tot += v
    return tot


def table(path, g=9.81, mu=None, R=None, label=""):
    byid, kids, order = load(path)
    stage_of = {p: byid[p].get("stage", 1) for p in order}
    top = max(stage_of.values())
    roots = [p for p in order if not byid[p].get("parent")]

    def live_mass(live, fueled=True):
        # Sum each surviving part's OWN mass: a decoupled subtree is gone, and
        # recursing from the roots would pull it back in.
        tot = 0.0
        for pid in live:
            d = PARTS[byid[pid]["part"]]
            tot += d.get("mass", 0.0)
            for r, v in d.get("capacity", {}).items():
                if r == "ec" or (not fueled and r in PROPS):
                    continue
                tot += v
        return tot

    live = set(order)
    fueled = live_mass(live)
    dry = live_mass(live, False)
    wheels = sum(PARTS[byid[p]["part"]].get("torque", 0.0) for p in live)
    print("== %s%s" % (os.path.basename(path), " (" + label + ")" if label else ""))
    print("   liftoff %.0f kg (dry %.0f kg, prop %.0f kg); wheel authority %.0f N m"
          % (fueled, dry, propellant(byid, kids, live), wheels))
    for active in range(top, 0, -1):
        ids = [p for p in order if p in live]
        thr, flow, ve = engines_in(byid, kids, ids, active)
        m = live_mass(live)
        decs = [p for p in ids if PARTS[byid[p]["part"]].get("decoupler")
                and stage_of[p] == active]
        pm = propellant(byid, kids, ids)
        dv = ve * math.log(m / (m - pm)) if ve and m > pm else 0.0
        print("   counter %d: engines %d  thrust %7.0f kN  TWR %5.2f  flow %6.1f kg/s"
              "  burn %4.0f s  mass %8.0f kg  dv %5.0f m/s  decouplers %d"
              % (active, len([1 for p in ids
                              if PARTS[byid[p]["part"]].get("propellant")
                              and stage_of[p] >= active]),
                 thr / 1e3, thr / (m * g) if m else 0, flow,
                 pm / flow if flow else 0, m, dv, len(decs)))
        for dc in decs:
            live -= set(subtree(kids, dc))
    if mu and R:
        tot = 0.0
        for alt in (70000, 100000, 150000, 200000):
            r = R + alt
            vc = math.sqrt(mu / r)
            # ideal: vis-viva from a parked pad + gravity/drag allowance not modelled
            print("   circular @ alt %6.0f km: v_circ %.0f m/s,"
                  " ideal-impulsive dv from rest at R ~ %.0f m/s"
                  % (alt / 1e3, vc, vc + (mu / R - mu / r) / vc))


if __name__ == "__main__":
    mu = 6.674e-11 * 5.292e22   # Kerbin
    args = sys.argv[1:] or [REPO + "/res/ships/mars_explorer_mk1.json"]
    for a in args:
        table(a, 9.81, mu, 600000.0)
        print()
