#!/usr/bin/env python3
"""Summarize a prototype [pilot] run: phase timeline, orbit at key moments,
staging, and the authority margins."""
import re
import sys

PILOT = re.compile(
    r"\[pilot\] t=(?P<t>[\d.]+) ph=(?P<ph>\w+) alt=(?P<alt>-?[\d.]+) h_gnd=(?P<h>-?[\d.]+)"
    r" apo=(?P<apo>-?[\d.]+) peri=(?P<peri>-?[\d.]+) thr=(?P<thr>[\d.]+)"
    r" v_air=(?P<vair>[\d.]+) v_orb=(?P<vorb>[\d.]+) q=(?P<q>[\d.]+) AoA=(?P<aoa>-?[\d.]+)"
    r" tau_aero=(?P<ta>[\d.]+) tau_wheel=(?P<tw>[\d.]+) cpLev=(?P<cp>[-+][\d.]+)"
    r" stage=(?P<st>\d+)/(?P<stt>\d+) mass=(?P<m>[\d.]+)")
ORBIT = re.compile(
    r"\[orbitlog\] t=(?P<t>[\d.]+)s frame=\"(?P<fr>[^\"]*)\" r=(?P<r>[\d.e+-]+)"
    r" v=(?P<v>[\d.]+) m/s sma=(?P<sma>-?[\d.e+-]+) m ecc=(?P<ecc>[\d.]+)"
    r" peri=(?P<peri>-?[\d.e+-]+) m apo=(?P<apo>-?[\d.e+-]+) m inc=(?P<inc>[\d.]+)"
    r" deg")
DV = re.compile(r"\[dvlog\] (?P<rest>.*)")


def rows(line_re, text):
    out = []
    for m in line_re.finditer(text):
        d = m.groupdict()
        for k, v in d.items():
            try:
                d[k] = float(v)
            except (TypeError, ValueError):
                pass
        out.append(d)
    return out


def main(paths):
    for path in paths:
        text = open(path, errors="replace").read()
        P = rows(PILOT, text)
        O = rows(ORBIT, text)
        stages = [l for l in text.splitlines() if l.startswith("Stage:")]
        print("=" * 100)
        print(path)
        if not P:
            print("  no [pilot] rows")
            continue
        # phase transitions
        print("  phase timeline:")
        last = None
        for r in P:
            if r["ph"] != last:
                print("    t=%7.1f s  -> %-11s alt=%9.0f h_gnd=%9.0f apo=%9.0f"
                      " peri=%9.0f v_orb=%7.1f q=%8.0f AoA=%+6.2f"
                      " tau_aero=%8.0f/%.0f" %
                      (r["t"], r["ph"], r["alt"], r["h"], r["apo"], r["peri"],
                       r["vorb"], r["q"], r["aoa"], r["ta"], r["tw"]))
                last = r["ph"]
        for s in stages:
            print("    %s" % s)
        last = P[-1]
        print("  last row: t=%.1f ph=%s alt=%.0f apo=%.0f peri=%.0f ecc~%.4f"
              " mass=%.0f stages=%d/%d" %
              (last["t"], last["ph"], last["alt"], last["apo"], last["peri"],
               ((last["apo"] - last["peri"]) / (last["apo"] + last["peri"])
                if (last["apo"] + last["peri"]) > 0 else float("nan")),
               last["m"], last["st"], last["stt"]))
        # worst authority / AoA seen
        worst_aoa = max(P, key=lambda r: abs(r["aoa"]))
        worst_tau = max(P, key=lambda r: r["ta"])
        overs = [r for r in P if r["ta"] > r["tw"]]
        print("  |AoA| max %.1f deg at t=%.1f (alt %.0f, v_air %.0f)"
              % (abs(worst_aoa["aoa"]), worst_aoa["t"], worst_aoa["alt"],
                 worst_aoa["vair"]))
        print("  |tau_aero| max %.0f N m at t=%.1f vs wheel %.0f N m"
              "  (over authority in %d/%d rows)" %
              (worst_tau["ta"], worst_tau["t"], worst_tau["tw"],
               len(overs), len(P)))
        cpl = [r["cp"] for r in P if r["cp"] != 0.0]
        if cpl:
            print("  cp lever (signed, m along nose): min %+.2f max %+.2f"
                  % (min(cpl), max(cpl)))
        if O:
            o = O[-1]
            print("  last orbitlog: t=%.1f frame=%s ecc=%.5f peri=%.0f apo=%.0f"
                  " inc=%.2f v=%.1f" %
                  (o["t"], o["fr"], o["ecc"], o["peri"], o["apo"], o["inc"], o["v"]))
        for l in text.splitlines():
            if l.startswith("[dvlog]") or l.startswith("Pilot:"):
                print("  %s" % l)


if __name__ == "__main__":
    main(sys.argv[1:])
