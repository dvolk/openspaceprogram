#!/usr/bin/env python3
"""Fit the measured aero torque law tau = k * q * sin(AoA) and turn it into the
attitude-authority ceiling for each ship: q*|sin(AoA)| <= wheel_torque / k.
k = Cd * A_projected * lever, measured (not assumed)."""
import math
import re
import sys

PILOT = re.compile(
    r"\[pilot\] t=(?P<t>[\d.]+) ph=(?P<ph>\w+) alt=(?P<alt>-?[\d.]+)"
    r" h_gnd=-?[\d.]+ apo=-?[\d.]+ peri=-?[\d.]+ thr=[\d.]+"
    r" v_air=(?P<vair>[\d.]+) v_orb=-?[\d.]+ q=(?P<q>[\d.]+) AoA=(?P<aoa>-?[\d.]+)"
    r" tau_aero=(?P<ta>[\d.]+) tau_wheel=(?P<tw>[\d.]+)")


def main(paths):
    ks = []
    for p in paths:
        for m in PILOT.finditer(open(p, errors="replace").read()):
            q = float(m["q"]); aoa = math.radians(abs(float(m["aoa"])))
            tau = float(m["ta"]); s = math.sin(aoa)
            if q > 200.0 and 0.005 < s < 0.35 and tau > 50.0:
                ks.append(tau / (q * s))
    if not ks:
        print("no usable samples")
        return
    ks.sort()
    k = ks[len(ks) // 2]
    print("samples %d   k = Cd*A*lever = %.1f N m per Pa (median, IQR %.1f-%.1f)"
          % (len(ks), k, ks[len(ks) // 4], ks[3 * len(ks) // 4]))
    print()
    print("Authority ceiling: q*sin(AoA) <= wheel/k.   Max q (Pa) the wheels can hold:")
    print("| AoA | wheel 5200 N m | wheel 2000 N m |")
    print("|---|---|---|")
    for deg in (0.2, 0.5, 1.0, 2.0, 5.0, 10.0):
        s = math.sin(math.radians(deg))
        print("| %.1f deg | %.0f | %.0f |" % (deg, 5200 / k / s, 2000 / k / s))
    print()
    print("Equivalent airspeed (m/s) at which that ceiling is reached, rho from the logs:")
    print("| rho (kg/m3) | AoA 1 deg | AoA 2 deg | AoA 5 deg |")
    print("|---|---|---|---|")
    for rho in (1.15, 0.5, 0.26, 0.1, 0.02):
        row = []
        for deg in (1.0, 2.0, 5.0):
            qmax = 5200 / k / math.sin(math.radians(deg))
            row.append("%.0f" % math.sqrt(2 * qmax / rho))
        print("| %.2f | %s |" % (rho, " | ".join(row)))


if __name__ == "__main__":
    main(sys.argv[1:])
