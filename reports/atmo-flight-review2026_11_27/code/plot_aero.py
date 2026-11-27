#!/usr/bin/env python3
"""Figures from the REAL drag.h laws (curves.cpp output)."""
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

OUT = os.path.dirname(os.path.abspath(__file__))

def load(name):
    return np.genfromtxt(os.path.join(OUT, name), delimiter=',', names=True)

# Fig: Mach regime join -- lift slope and wave drag
ls = load('lift_slope.csv')
wc = load('wave_cd.csv')
fig, axs = plt.subplots(1, 2, figsize=(11, 4.5))
ax = axs[0]
ax.plot(ls['mach'], ls['slope'], 'k-', lw=2, label='game law (liftSlopeMach)')
m = ls['mach']
ax.plot(m[m < 0.999], ls['pg'][m < 0.999], '--', color='#1f77b4',
        label='Prandtl–Glauert $c_l/\\sqrt{1-M^2}$')
ax.plot(m[m > 1.001], ls['ackeret'][m > 1.001], '--', color='#d62728',
        label='thin-airfoil $4/\\sqrt{M^2-1}$')
ax.axvspan(0.75, 1.25, alpha=0.12, color='orange', label='linear join band')
ax.set_xlabel('Mach'); ax.set_ylabel('lift slope [1/rad]')
ax.set_title('Lift slope vs Mach ($c_l$ = 6.0, every shipped wing)')
ax.set_ylim(0, 14); ax.grid(alpha=0.3); ax.legend(fontsize=8)
ax = axs[1]
ax.plot(wc['mach'], wc['tc044'], label='t/c = 0.044')
ax.plot(wc['mach'], wc['tc060'], label='t/c = 0.060')
ax.plot(wc['mach'], wc['tc086'], label='t/c = 0.086')
ax.axvspan(0.75, 1.25, alpha=0.12, color='orange')
ax.axhline(0.01, color='gray', ls=':', label='wing profile cd (0.01)')
ax.set_xlabel('Mach'); ax.set_ylabel('$c_{d,wave}$')
ax.set_title('Wave drag vs Mach (Ackeret $4(t/c)^2/\\sqrt{M^2-1}$)')
ax.grid(alpha=0.3); ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_mach.png'), dpi=130)

# Fig: soft stall
lc = load('lift_curve.csv')
fig, ax = plt.subplots(figsize=(7, 4.5))
ax.plot(lc['alpha_deg'], lc['cl_at_m0'], label='M ≈ 0')
ax.plot(lc['alpha_deg'], lc['cl_at_m075'], label='M = 0.75')
ax.plot(lc['alpha_deg'], lc['cl_at_m15'], label='M = 1.5')
ax.axvline(20.05, color='gray', ls=':', label='stall angle 0.35 rad')
ax.axvline(40.1, color='gray', ls='--', label='2× stall (CL → 0)')
ax.set_xlabel('angle of attack [deg]'); ax.set_ylabel('$C_L$')
ax.set_title('Soft stall: linear to 20°, cosine droop to zero by 40°\n'
             '(stall angle fixed at every Mach — #633)')
ax.grid(alpha=0.3); ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_stall.png'), dpi=130)

# Fig: jet thrust vs speed
jt = load('jet_thrust.csv')
fig, axs = plt.subplots(1, 2, figsize=(11, 4.5), sharey=True)
for ax, key, title in [(axs[0], 'stock_N', 'Shipped 2 m jet: fan 90 kN, 1.2 kg/s, $v_e$ 550'),
                       (axs[1], 'demo_N', '#1206 demo: fan 3 kN, 3 kg/s, $v_e$ 3200')]:
    for rho, c, lab in [(1.225, '#1f77b4', 'sea level (1.225)'),
                        (0.4165, '#2ca02c', '10 km (0.417)'),
                        (0.0948, '#d62728', '20 km (0.095)')]:
        d = jt[jt['rho'] == rho]
        ax.plot(d['v'], d[key] / 1000, color=c, label=lab)
    ax.set_xlabel('airspeed [m/s]'); ax.set_title(title)
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
axs[0].set_ylabel('thrust [kN]')
fig.suptitle('Air-breathing thrust vs speed on Kerbin (drag.h::jetThrust, dry air)')
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_jet.png'), dpi=130)

# Fig: the composition gate
jg = load('jet_gate.csv')
fig, ax = plt.subplots(figsize=(7, 4.5))
ax.plot(jg['rho'], jg['dry'], label='dry air (Earth/Kerbin)')
ax.plot(jg['rho'], jg['eve_trace'], label='Eve CO₂ + 0.096% O₂ trace')
ax.plot(jg['rho'], jg['jool'], label='Jool H₂/He (no oxidizer)')
ax.set_xlabel('intake density [kg/m³]'); ax.set_ylabel('usable fraction of rating')
ax.set_title('jetAirGate: min(air, oxidizer) — a dense world can still be a dead engine')
ax.grid(alpha=0.3); ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_jet_gate.png'), dpi=130)
print('aero figures written')
