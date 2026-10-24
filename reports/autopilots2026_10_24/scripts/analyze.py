#!/usr/bin/env python3
"""Per-run summary from the [pilot]/[orbitlog] logs. (Superseded by table.py,
which is the source of the tables in the report; kept for the rho/dv accounting.)
outcome, propellant accounting, aero-authority margins, pointing state."""
import math
import re
import sys

PILOT = re.compile(
    r"\[pilot\] t=(?P<t>[\d.]+) ph=(?P<ph>\w+) alt=(?P<alt>-?[\d.]+) h_gnd=(?P<h>-?[\d.]+)"
    r" apo=(?P<apo>-?[\d.]+) peri=(?P<peri>-?[\d.]+) thr=(?P<thr>[\d.]+)"
    r" v_air=(?P<vair>[\d.]+) v_orb=(?P<vorb>[\d.]+) q=(?P<q>[\d.]+) AoA=(?P<aoa>-?[\d.]+)"
    r" tau_aero=(?P<ta>[\d.]+) tau_wheel=(?P<tw>[\d.]+) cpLev=(?P<cp>[-+][\d.]+)"
    r" stage=(?P<st>\d+)/(?P<stt>\d+) mass=(?P<m>[\d.]+)"
    r"(?: dv=(?P<dv>[-+][\d.]+) vneed=(?P<vn>[\d.]+) amax=(?P<am>[\d.]+) gap=(?P<gap>-?[\d.]+)"
    r" ptErr=(?P<pe>[\d.]+) ptRate=(?P<pr>[\d.]+)(?: gated=(?P<gd>\d+))?)?")
ORBIT = re.compile(
    r"\[orbitlog\] t=(?P<t>[\d.]+)s frame=\"(?P<fr>[^\"]*)\" r=(?P<r>[\d.eE+-]+) m"
    r" v=(?P<v>[\d.]+) m/s sma=(?P<sma>-?[\d.eE+-]+) m ecc=(?P<ecc>[\d.eE+-]+)"
    r" peri=(?P<peri>-?[\d.eE+-]+) m apo=(?P<apo>-?[\d.eE+-]+) m inc=(?P<inc>[\d.]+) deg")
STAGE = re.compile(r"Stage: dropped (?P<ships>\d+) ship\(s\) / (?P<parts>\d+) part\(s\);"
                   r" now on stage (?P<st>\d+) of (?P<stt>\d+)")


def rows(rx, text):
    out = []
    for m in rx.finditer(text):
        d = {k: v for k, v in m.groupdict().items()}
        for k, v in list(d.items()):
            if v is None:
                continue
            try:
                d[k] = float(v)
            except ValueError:
                pass
        out.append(d)
    return out


def ve_for(ship):
    """Exhaust velocity of the stage-4 engines, for the dv-spent accounting."""
    return 4400.0


def summarize(path, ve=4400.0):
    text = open(path, errors="replace").read()
    P = rows(PILOT, text)
    O = rows(ORBIT, text)
    if not P:
        return None
    first, last = P[0], P[-1]
    m0 = first["m"]
    # Mass steps: staging jettisons mass in one tick; anything else is burnt.
    burned = 0.0
    jettisoned = 0.0
    prev = m0
    for r in P:
        d = prev - r["m"]
        if d > 0:
            # A stage drop is a step of several tonnes in one tick (>20 t here);
            # a tick of full-throttle burn is at most ~0.3 t.
            if d > 5000.0:
                jettisoned += d
            else:
                burned += d
        prev = r["m"]
    def g(r, k, d=0.0):
        v = r.get(k)
        return d if v is None else v

    aoa = max(P, key=lambda r: abs(r["aoa"]))
    tau = max(P, key=lambda r: r["ta"])
    wheel = first["tw"]
    over = [r for r in P if r["ta"] > wheel]
    firstover = over[0] if over else None
    pt = max(P, key=lambda r: g(r, "pe"))
    pr = max(P, key=lambda r: g(r, "pr"))
    grounded = last["h"] < 200.0 and abs(last["vorb"]) < 10.0
    if last["ph"] == "done" and grounded:
        verdict = "LANDED %.0f m/s" % abs(last["vorb"])
    elif last["peri"] > 0 and abs(last["apo"] - last["peri"]) < 0.05 * max(last["apo"], 1):
        verdict = "CIRCULAR %dx%d km" % (last["peri"] / 1e3, last["apo"] / 1e3)
    elif last["peri"] > 0:
        verdict = "ORBIT %dx%d km" % (last["peri"] / 1e3, last["apo"] / 1e3)
    else:
        verdict = "SUBORBITAL (peri %.0f km)" % (last["peri"] / 1e3)
    # Air density recovered from q = 0.5 rho v^2, at the highest-q sample.
    qs = [r for r in P if r["vair"] > 30.0 and r["q"] > 1.0]
    rho_at = None
    if qs:
        r = max(qs, key=lambda r: r["q"])
        rho_at = (2.0 * r["q"] / (r["vair"] ** 2), r["alt"], r["q"], r["vair"])
    # Authority ceiling for 1 deg AoA, from the measured q -> tau relation.
    slope = None
    pairs = [(r["q"], r["ta"]) for r in P if r["q"] > 50.0 and 0.2 < abs(r["aoa"]) < 5.0]
    if pairs:
        slope = sum(t / q for q, t in pairs) / len(pairs)   # N m per Pa at that AoA
    return dict(
        path=path, verdict=verdict, t_end=last["t"], m0=m0, m_end=last["m"],
        burned=burned, jettisoned=jettisoned, dv_spent=ve * math.log(m0 / max(1.0, m0 - burned)),
        aoa_max=abs(aoa["aoa"]), aoa_t=aoa["t"], tau_max=tau["ta"], tau_t=tau["t"],
        wheel=wheel, over=len(over), nrows=len(P),
        firstover=(firstover["alt"], firstover["q"], firstover["vair"]) if firstover else None,
        pt_max=g(pt, "pe"), pt_rate=g(pr, "pr"), gated=int(g(last, "gd")),
        apo=last["apo"], peri=last["peri"],
        inc=O[-1]["inc"] if O else None, ecc=O[-1]["ecc"] if O else None,
        rho_at=rho_at, slope=slope,
    )


def main(paths):
    hdr = ("case | verdict | t(s) | dv spent(m/s) | burned(t) | jett(t) | "
           "|AoA|max | tau_aero|max | over-auth rows | q@first-over | ptErr|max | gated")
    print(hdr)
    print("|".join(["---"] * len(hdr.split(" | "))))
    for p in paths:
        s = summarize(p)
        if s is None:
            print("%s | no rows" % p)
            continue
        name = p.split("/")[-1].replace(".log", "")
        qo = ("%.0f Pa @ %.0f m, %.0f m/s" % (s["firstover"][1], s["firstover"][0],
                                             s["firstover"][2])) if s["firstover"] else "-"
        print("| %s | %s | %.0f | %.0f | %.1f | %.1f | %.1f | %.0f (wheel %.0f) | %d/%d | %s | %.0f | %d |"
              % (name, s["verdict"], s["t_end"], s["dv_spent"], s["burned"] / 1e3,
                 s["jettisoned"] / 1e3, s["aoa_max"], s["tau_max"], s["wheel"],
                 s["over"], s["nrows"], qo, s["pt_max"], s["gated"]))
    print()
    print("case | final apo (km ASL) | final peri (km ASL) | ecc | inc")
    print("|---|---|---|---|---|")
    for p in paths:
        s = summarize(p)
        if s is None:
            continue
        print("| %s | %.1f | %.1f | %s | %s |"
              % (p.split("/")[-1].replace(".log", ""), s["apo"] / 1e3, s["peri"] / 1e3,
                 ("%.4f" % s["ecc"]) if s["ecc"] is not None else "-",
                 ("%.2f" % s["inc"]) if s["inc"] is not None else "-"))
    print()
    print("case | rho@maxq (kg/m3) | alt | q | v_air | tau/q slope (N m/Pa, AoA<5deg)")
    print("|---|---|---|---|---|---|")
    for p in paths:
        s = summarize(p)
        if s is None or s["rho_at"] is None:
            continue
        rho, alt, q, v = s["rho_at"]
        print("| %s | %.3f | %.0f | %.0f | %.0f | %s |"
              % (p.split("/")[-1].replace(".log", ""), rho, alt, q, v,
                 ("%.3f" % s["slope"]) if s["slope"] else "-"))


if __name__ == "__main__":
    main(sys.argv[1:])
