#!/usr/bin/env python3
"""Report figures. Run from the report directory: python3 scripts/figures.py ."""
import math
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

PILOT = re.compile(
    r"\[pilot\] t=(?P<t>[\d.]+) ph=(?P<ph>\w+) alt=(?P<alt>-?[\d.]+)"
    r" h_gnd=-?[\d.]+ apo=(?P<apo>-?[\d.]+) peri=(?P<peri>-?[\d.]+) thr=(?P<thr>[\d.]+)"
    r" v_air=(?P<vair>[\d.]+) v_orb=(?P<vorb>-?[\d.]+) q=(?P<q>[\d.]+) AoA=(?P<aoa>-?[\d.]+)"
    r" tau_aero=(?P<ta>[\d.]+) tau_wheel=(?P<tw>[\d.]+) cpLev=[-+][\d.]+ stage=\d+/\d+"
    r" mass=(?P<m>[\d.]+)"
    r"(?: .*? ptErr=(?P<pe>[\d.]+) ptRate=(?P<pr>[\d.]+))?")
K = 38.9          # measured Cd*A*lever, N m per Pa (see scripts/authority.py)
WHEEL = 5200.0    # N m, mars_explorer_mk1 wheel authority as shipped


def rows(path):
    out = []
    for m in PILOT.finditer(open(path, errors="replace").read()):
        d = dict(m.groupdict())
        for k, v in list(d.items()):
            if v is None:
                continue
            try:
                d[k] = float(v)
            except ValueError:
                del d[k]
        if "aoa" in d:
            # Aero torque goes as sin(AoA), so the angle that costs authority is
            # the off-flow angle: 0 = nose into the flow, 180 = tail-first.
            d["off"] = min(abs(d["aoa"]), 180.0 - abs(d["aoa"]))
        out.append(d)
    return out


def fig_qaoa(paths_labels, out):
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    aoas = [0.2, 0.5, 1.0, 2.0, 5.0, 15.0, 45.0, 90.0]
    for wheel, style, label in ((WHEEL, "k--", "wheel authority, 5.2 kN m (as shipped)"),
                                (4 * WHEEL, "k:", "4x wheel authority, 20.8 kN m")):
        qs = [min(wheel / (K * math.sin(math.radians(a))), 3e5) for a in aoas]
        ax.plot(qs, aoas, style, lw=1.4, label=label)
    for p, label in paths_labels:
        R = [r for r in rows(p) if r.get("q", 0) > 1.0 and r["off"] > 0.05]
        if not R:
            continue
        ax.plot([r["q"] for r in R], [r["off"] for r in R], "-", lw=1.2, alpha=0.85,
                label=label)
    ax.set_xscale("log")
    ax.set_xlabel("dynamic pressure of the air, q (Pa)")
    ax.set_ylabel("off-flow angle (deg): 0 = nose into flow, 180 = tail-first")
    ax.set_title("Above the dashed line the reaction wheels cannot hold the nose\n"
                 "(mars_explorer_mk1, 106 t, measured Cd*A*lever = 38.9 N m/Pa)")
    ax.grid(alpha=0.3, which="both")
    ax.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "q-vs-aoa.png"), dpi=130)
    plt.close(fig)


def fig_ascent(path, out):
    R = rows(path)
    t = [r["t"] for r in R]
    fig, (a1, a2) = plt.subplots(2, 1, figsize=(7.2, 6.0), sharex=True)
    a1.plot(t, [r["alt"] / 1e3 for r in R], label="altitude ASL (km)")
    a1.plot(t, [max(r["apo"], 0) / 1e3 for r in R], label="apoapsis (km)")
    a1.plot(t, [max(r["peri"], -600) / 1e3 for r in R], label="periapsis (km)")
    a1.axhline(70, color="0.5", ls=":", lw=1, label="atmosphere top 70 km")
    a1.axhline(50, color="C3", ls=":", lw=1, label="target 50 km")
    a1.set_ylabel("km")
    a1.legend(fontsize=8, ncol=2)
    a1.grid(alpha=0.3)
    a2.plot(t, [r["m"] / 1e3 for r in R], "C0", label="mass (t)")
    a2.plot(t, [r["vair"] for r in R], "C2", label="air speed (m/s)")
    a2.plot(t, [r["vorb"] for r in R], "C1", label="orbital speed (m/s)")
    a2.set_ylabel("t / m/s")
    a2.set_xlabel("sim time (s)")
    a2.legend(fontsize=8)
    a2.grid(alpha=0.3)
    a1.set_title("Radial-out-until-out-of-air, the profile the user flies by hand:\n"
                 "89 of 106 t gone, still suborbital when the 317 s wall cut it off\n"
                 "(apoapsis 282 km, periapsis 561 km under the surface)")
    fig.tight_layout()
    fig.savefig(os.path.join(out, "ascent-kerbin-vertical.png"), dpi=130)
    plt.close(fig)


def fig_pointing(paths_labels, out):
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for p, label in paths_labels:
        R = [r for r in rows(p) if "pe" in r]
        if not R:
            continue
        ax.plot([r["t"] for r in R], [r["pe"] for r in R], label=label, lw=1.2)
    ax.axhline(8.0, color="k", ls="--", lw=1, label="pointing gate (8 deg)")
    ax.set_xlabel("sim time (s)")
    ax.set_ylabel("nose-to-command error (deg)")
    ax.set_title("Deorbit from an 85 km Moho orbit: at the shipped wheel authority the\n"
                 "nose needs 79-87 s to settle inside 8 deg of a 180 deg reversal (40.5 s\n"
                 "with 4x the wheels). Ungated, the burn thrustes prograde through the\n"
                 "whole window and the ship goes hyperbolic instead of landing.")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(os.path.join(out, "pointing-lag.png"), dpi=130)
    plt.close(fig)


def main(d):
    L = os.path.join(d, "logs")
    fig_qaoa([(os.path.join(L, "P_kerbin_low.log"), "turn starts at 5 km"),
              (os.path.join(L, "Q_kerbin_mid.log"), "turn starts at 20 km"),
              (os.path.join(L, "O_kerbin_top.log"), "no turn until out of air"),
              (os.path.join(L, "P_t4.log"), "turn at 5 km, wheels x4"),
              (os.path.join(L, "V_kerbin_land2.log"), "Kerbin descent (retrograde)")], d)
    fig_ascent(os.path.join(L, "O_kerbin_top.log"), d)
    fig_pointing([(os.path.join(L, "R_moho_land.log"), "no pointing gate"),
                  (os.path.join(L, "AA_moho_land3.log"), "with pointing gate"),
                  (os.path.join(L, "U_t4.log"), "gate + wheels x4")], d)
    print("wrote figures to", d)


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else ".")
