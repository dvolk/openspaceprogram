#!/usr/bin/env python3
"""Markdown run tables straight from the archived logs - no hand-typed numbers.
usage: table.py logs/*.log > logs/table.md"""
import math
import re
import sys

P = re.compile(
    r"\[pilot\] t=(?P<t>[\d.]+) ph=(?P<ph>\w+) alt=(?P<alt>-?[\d.]+) h_gnd=(?P<h>-?[\d.]+)"
    r" apo=(?P<apo>-?[\d.]+) peri=(?P<peri>-?[\d.]+) thr=(?P<thr>[\d.]+)"
    r" v_air=(?P<vair>[\d.]+) v_orb=(?P<vorb>-?[\d.]+) q=(?P<q>[\d.]+) AoA=(?P<aoa>-?[\d.]+)"
    r" tau_aero=(?P<ta>[\d.]+) tau_wheel=(?P<tw>[\d.]+) cpLev=(?P<cp>[-+][\d.]+)"
    r" stage=(?P<st>\d+)/(?P<stt>\d+) mass=(?P<m>[\d.]+)"
    r"(?: dv=(?P<dv>[-+][\d.]+) vneed=(?P<vn>[\d.]+) amax=(?P<am>[\d.]+) gap=(?P<gap>-?[\d.]+)"
    r" ptErr=(?P<pe>[\d.]+) ptRate=(?P<pr>[\d.]+)(?: gated=(?P<gd>\d+))?)?")
O = re.compile(
    r"\[orbitlog\] t=(?P<t>[\d.]+)s frame=\"(?P<fr>[^\"]*)\" r=(?P<r>[\d.eE+-]+) m"
    r" v=(?P<v>[\d.]+) m/s sma=(?P<sma>-?[\d.eE+-]+) m ecc=(?P<ecc>[\d.eE+-]+)"
    r" peri=(?P<peri>-?[\d.eE+-]+) m apo=(?P<apo>-?[\d.eE+-]+) m inc=(?P<inc>[\d.]+) deg")

VE = 4400.0        # exhaust velocity of the mex upper stage, m/s

# [orbitlog] prints periapsis/apoapsis as radii from the focus (see orbit.h),
# while the pilot's own apo/peri are metres above the surface. Convert the
# former before comparing them or every verdict is off by a body radius.
# From res/systems/ksp_system.json.
RADII = {"Kerbin": 600000.0, "Moho": 250000.0, "Mun": 200000.0, "Kerbol": 261600000.0}


def num(s):
    try:
        return float(s)
    except (TypeError, ValueError):
        return None


def rows(rx, text):
    return [{k: (v if k in ("fr", "ph") else num(v)) for k, v in m.groupdict().items()}
            for m in rx.finditer(text)]


def build_of(text, first):
    """Which prototype variant produced this log, read off its own field set.
    Only variant D can be rebuilt from appendix-patch.diff."""
    if " gated=" in text:
        b = "D"
    elif " ptErr=" in text:
        b = "C"
    elif " vneed=" in text:
        b = "B"
    else:
        b = "A"
    if first["tw"] != 5200.0:          # parts.json edited for the authority test
        b += " w%d" % (first["tw"] / 5200.0)
    return b


def summarize(path):
    text = open(path, errors="replace").read()
    pr = rows(P, text)
    if not pr:
        return None
    first, last = pr[0], pr[-1]

    # Mass steps: a stage drop is a multi-tonne step in one tick; anything
    # smaller is burnt propellant.
    burned = jett = 0.0
    prev = first["m"]
    for r in pr:
        d = prev - r["m"]
        if d > 5000.0:
            jett += d
        elif d > 0:
            burned += d
        prev = r["m"]

    wheel = first["tw"]
    over = [r for r in pr if r["ta"] > wheel]
    aoa = max(pr, key=lambda r: abs(r["aoa"]))
    # The torque law is symmetric about retrograde (sin AoA), so the angle that
    # costs authority is the off-flow angle min(AoA, 180 - AoA).
    qa = [r for r in pr if r["q"] > 1.0]      # only rows where there is air
    off = max((min(abs(r["aoa"]), 180.0 - abs(r["aoa"])) for r in qa), default=0.0)
    tau = max(pr, key=lambda r: r["ta"])
    pe = max((r["pe"] for r in pr if r.get("pe") is not None), default=None)

    # Orbital elements are only meaningful while airborne: a landed ship has
    # v ~ 0 and its elements go degenerate (apo = current radius, ecc ~ 1).
    air = [r for r in pr if r["h"] > 200.0]
    orb = [o for o in rows(O, text) if (o["v"] or 0.0) > 50.0]
    R = RADII.get(orb[-1]["fr"].split(" (")[0]) if orb else None
    if R is not None:                      # -> metres above the surface
        for o in orb:
            o["peri"] -= R
            o["apo"] -= R
    # The pilot declares "done" on touchdown; later rows are post-landing drift.
    dn = [r for r in pr if r["ph"] == "done"]
    touch = dn[0] if dn else None
    grounded = touch is not None and touch["h"] < 50.0 and abs(touch["vorb"]) < 20.0
    la = air[-1] if air else last

    apomax = max((r["apo"] for r in air), default=0.0) / 1e3
    if grounded:
        if first["h"] < 50.0:      # started on the ground: a hop, not a landing
            verdict = "down again at t=%.0f s (apogee %.1f km)" % (touch["t"], apomax)
        else:
            verdict = "landed %.0f m/s at t=%.0f s" % (abs(touch["vorb"]), touch["t"])
    elif orb and orb[-1]["ecc"] >= 1.0:
        verdict = "escape (ecc %.2f)" % orb[-1]["ecc"]
    elif orb and orb[-1]["peri"] > 0:
        verdict = "orbit %dx%d km" % (orb[-1]["peri"] / 1e3, orb[-1]["apo"] / 1e3)
    elif la["peri"] > 0:
        verdict = "orbit %dx%d km" % (la["peri"] / 1e3, la["apo"] / 1e3)
    else:
        verdict = "suborbital"

    o = orb[-1] if orb else None
    return dict(
        case=path.split("/")[-1].replace(".log", ""), verdict=verdict,
        build=build_of(text, first), t=last["t"],
        dv=VE * math.log(first["m"] / max(1.0, first["m"] - burned)),
        burned=burned / 1e3, jett=jett / 1e3, aoa=abs(aoa["aoa"]), off=off,
        tau=tau["ta"], over=len(over), n=len(pr), pe=pe,
        gated=int(last.get("gd") or 0),
        apo=(o["apo"] if o else la["apo"]) / 1e3,
        peri=(o["peri"] if o else la["peri"]) / 1e3,
        apomax=apomax,
        perimax=max((r["peri"] for r in air), default=0.0) / 1e3,
        ecc=o["ecc"] if o else None, inc=o["inc"] if o else None,
        incmax=max((x["inc"] for x in orb if x["inc"] is not None), default=None))


def main(paths):
    S = [s for s in (summarize(p) for p in paths) if s]
    S.sort(key=lambda s: s["case"])
    print("| case | build | outcome | sim s | dv spent | burned t | jett t | AoA max |"
          " off-flow max | tau max | over-auth | ptErr max | gated |")
    print("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for s in S:
        print("| %s | %s | %s | %.0f | %.0f | %.1f | %.1f | %.1f | %.1f | %.0f | %d/%d | %s | %d |"
              % (s["case"], s["build"], s["verdict"], s["t"], s["dv"], s["burned"],
                 s["jett"], s["aoa"], s["off"], s["tau"], s["over"], s["n"],
                 ("%.0f" % s["pe"]) if s["pe"] is not None else "-", s["gated"]))
    print()
    print("| case | max apo while airborne (km) | max peri while airborne (km) |"
          " apo at last airborne sample (km) | peri at last airborne sample (km) | ecc | inc | inc max |")
    print("|---|---|---|---|---|---|---|---|")
    for s in S:
        print("| %s | %.1f | %.1f | %.1f | %.1f | %s | %s | %s |"
              % (s["case"], s["apomax"], s["perimax"], s["apo"], s["peri"],
                 ("%.4f" % s["ecc"]) if s["ecc"] is not None else "-",
                 ("%.2f" % s["inc"]) if s["inc"] is not None else "-",
                 ("%.2f" % s["incmax"]) if s["incmax"] is not None else "-"))


if __name__ == "__main__":
    main(sys.argv[1:])
