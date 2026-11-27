#!/usr/bin/env python3
"""Review evidence: plot the shipped atmosphere profiles.

Uses utils/atmo_profile.py's Profile -- the quadrature re-implementation that
tests/test_scenarios pins against the C++ loader -- so the figures describe the
air the game actually bills, not a re-derivation.
"""
import json, math, os, sys
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

ROOT = '/home/ubuntu/openspaceprogram'
sys.path.insert(0, os.path.join(ROOT, 'utils'))
from atmo_profile import Profile

OUT = os.path.dirname(os.path.abspath(__file__))

# Every shipped air body (all 15), so the datum table and the top-q figure are
# the whole catalogue, not a subset.
WANT = None

def sound(pr, z):
    return math.sqrt(pr.gamma * pr.Rs * pr.T(z))

profiles = {}
for fn in ['ksp_system.json', 'solar_system.json']:
    doc = json.load(open(os.path.join(ROOT, 'res/systems', fn)))
    for b in doc['bodies']:
        at = (b.get('surface') or {}).get('atmosphere') or {}
        if 'sea_level_pressure' in at:
            profiles[b['name']] = Profile(b)

# CSV dump
with open(os.path.join(OUT, 'profiles.csv'), 'w') as f:
    f.write('body,alt_m,T_K,p_Pa,rho_kgm3,a_ms\n')
    for name, pr in profiles.items():
        for z in np.linspace(0, pr.top, 161):
            if z >= pr.top:
                continue
            p = pr.p(z)
            f.write('%s,%.1f,%.4f,%.6e,%.6e,%.4f\n'
                    % (name, z, pr.T(z), p, p / (pr.Rs * pr.T(z)), sound(pr, z)))

# Figure 1: the four profile facts, six representative bodies
sel = ['Kerbin', 'Earth', 'Venus', 'Mars', 'Duna', 'Jool']
colors = {'Kerbin': '#1f77b4', 'Earth': '#2ca02c', 'Venus': '#d62728',
          'Mars': '#ff7f0e', 'Duna': '#9467bd', 'Jool': '#8c564b'}
fig, axs = plt.subplots(2, 2, figsize=(11, 8))
for name in sel:
    pr = profiles[name]
    zs = np.linspace(0, pr.top * 0.999, 140)
    Ts = [pr.T(z) for z in zs]
    ps = [pr.p(z) for z in zs]
    rs = [p / (pr.Rs * pr.T(z)) for p, z in zip(ps, zs)]
    as_ = [sound(pr, z) for z in zs]
    c = colors[name]
    axs[0][0].plot(zs / 1000, Ts, color=c, label=name)
    axs[0][1].plot(zs / 1000, ps, color=c, label=name)
    axs[1][0].plot(zs / 1000, rs, color=c, label=name)
    axs[1][1].plot(zs / 1000, as_, color=c, label=name)
for ax, title, ylab, log in [
        (axs[0][0], 'Temperature', 'T [K]', False),
        (axs[0][1], 'Pressure (hydrostatic, 1/r² field)', 'p [Pa]', True),
        (axs[1][0], 'Density', 'ρ [kg/m³]', True),
        (axs[1][1], 'Sound speed', 'a [m/s]', False)]:
    ax.set_title(title)
    ax.set_xlabel('altitude above sea-level datum [km]')
    ax.set_ylabel(ylab)
    if log:
        ax.set_yscale('log')
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
fig.suptitle('Shipped atmosphere profiles (utils/atmo_profile.py quadrature, pinned vs C++ loader)')
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_profiles.png'), dpi=130)

# Figure 2: Kerbin vs Earth density -- the converted US-standard table
fig, ax = plt.subplots(figsize=(7, 5))
for name, c in [('Kerbin', '#1f77b4'), ('Earth', '#2ca02c')]:
    pr = profiles[name]
    zs = np.linspace(0, pr.top * 0.999, 140)
    rs = [pr.p(z) / (pr.Rs * pr.T(z)) for z in zs]
    ax.plot(zs / 1000, rs, color=c, label='%s (R=%.0f km)' % (name, pr.radius / 1000))
ax.set_yscale('log')
ax.set_xlabel('altitude [km]')
ax.set_ylabel('ρ [kg/m³]')
ax.set_title('Kerbin wears a converted copy of Earth\'s 1976 standard table\n'
             '(geopotential→geometric conversion, #686/#690)')
ax.grid(alpha=0.3, which='both')
ax.legend()
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_kerbin_earth.png'), dpi=130)

# Figure 3: q of a circular orbit at the authored air top (#682's band)
names = list(profiles.keys())
qt, tops = [], []
for name in names:
    pr = profiles[name]
    # the limit from BELOW the top, the same convention utils/atmo_profile.py
    # prints (the game bills exactly 0 at top; this is the last air it feels)
    z = pr.top
    qt.append(0.5 * pr.rho(z) * pr.vcirc(z) ** 2)
    tops.append(pr.top / 1000)
fig, ax = plt.subplots(figsize=(9, 5))
bars = ax.bar(names, qt, color=['#1f77b4' if 10 <= q <= 1000 else '#d62728' for q in qt])
ax.axhspan(10, 1000, alpha=0.12, color='green', label='#682 band: 10..1000 Pa')
ax.set_yscale('log')
ax.set_ylabel('q = ½ ρ v_circ² at the air top [Pa]')
ax.set_title('Where the air stops: dynamic pressure a circular orbit meets at the ceiling')
for b, q, t in zip(bars, qt, tops):
    ax.text(b.get_x() + b.get_width() / 2, q * 1.15, '%.0f km\n%.1f Pa' % (t, q),
            ha='center', fontsize=8)
ax.legend()
ax.grid(alpha=0.3, axis='y')
fig.tight_layout()
fig.savefig(os.path.join(OUT, 'fig_top_q.png'), dpi=130)

# datum table for the report
with open(os.path.join(OUT, 'datum_table.md'), 'w') as f:
    f.write('| Body | R [km] | g [m/s²] | p₀ [Pa] | T₀ [K] | ρ₀ [kg/m³] | a₀ [m/s] | top [km] | profile | M [g/mol] | γ |\n')
    f.write('|---|---|---|---|---|---|---|---|---|---|---|\n')
    for name, pr in profiles.items():
        shape = '%d layers' % len(pr.levels) if pr.levels else 'H=%.1f km (isothermal)' % (pr.H / 1000)
        f.write('| %s | %.0f | %.3f | %.4g | %.2f | %.4g | %.1f | %.0f | %s | %.3f | %.4f |\n'
                % (name, pr.radius / 1000, pr.g, pr.p0, pr.T(0), pr.rho0,
                   sound(pr, 0), pr.top / 1000, shape, pr.M * 1000, pr.gamma))
print('profiles written:', ', '.join(profiles))
